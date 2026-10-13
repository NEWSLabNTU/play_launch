//! Host checks (`docs/design/big-system-observability.md` §L): the facts
//! about the machine and the middleware setup that decide whether a ROS
//! system can work at all, and that nobody looks at until it does not.
//!
//! - **Which RMW, and which DDS config file it reads.** `RMW_IMPLEMENTATION`
//!   unset means the distro default (Fast DDS on Humble and Jazzy), and a
//!   `CYCLONEDDS_URI` beside it is read by nothing. Found on this repo's own
//!   Autoware fixture, which sets a `cyclonedds.xml` and ran Fast DDS.
//! - **Cyclone's socket buffer against the kernel's.** A Cyclone config with
//!   `SocketReceiveBufferSize min=` above `net.core.rmem_max` makes every
//!   participant fail to start.
//! - **`/dev/shm`**: how full, and Fast DDS ports and segments whose owner is
//!   gone. Fast DDS holds a `flock` on each one's `_el`/`_sl` file while it is
//!   open; `fastdds shm clean` takes that lock to test it. Taking it here,
//!   while nodes start, could race their own locking, so this reads
//!   `/proc/locks` and matches inodes instead: the same answer, read-only.
//!   Nothing is deleted.
//! - **Disk under the log directory**, and **swapping** (pages in and out
//!   since the last check), which turns every latency on the host into noise.
//!
//! Read every [`HOST_PERIOD`] on a blocking thread; a few thousand `stat`s
//! on a busy `/dev/shm`, milliseconds.

use std::{
    collections::HashSet,
    os::unix::fs::MetadataExt,
    path::{Path, PathBuf},
    time::Duration,
};

use serde::Serialize;

use super::{Condition, Link, Severity, Subject};

pub const HOST_PERIOD: Duration = Duration::from_secs(60);
/// Free space under the log directory below this is a warning…
const DISK_WARN_BYTES: u64 = 2 << 30;
/// …and below this an error: logs stop, and with them most of this page.
const DISK_ERROR_BYTES: u64 = 256 << 20;
/// `/dev/shm` fuller than this is a warning: Fast DDS's next segment fails.
const SHM_WARN_PERCENT: f64 = 80.0;
/// Pages swapped in or out per second, averaged over the period.
const SWAP_WARN_PAGES_PER_S: f64 = 100.0;

#[derive(Debug, Clone, Default, Serialize, PartialEq)]
pub struct HostFacts {
    /// `RMW_IMPLEMENTATION` as set, or `None`.
    pub rmw_env: Option<String>,
    /// The RMW in effect: the variable, else the distro's default.
    pub rmw: String,
    pub rmw_is_default: bool,
    pub ros_distro: Option<String>,
    /// DDS configuration variables set, `NAME=value`.
    pub dds_config: Vec<String>,
    /// Cyclone's `SocketReceiveBufferSize min`, bytes, when its config says.
    pub cyclone_rcvbuf_min: Option<u64>,
    pub rmem_max: Option<u64>,
    pub wmem_max: Option<u64>,
    pub shm_total: Option<u64>,
    pub shm_used: Option<u64>,
    /// Fast DDS ports and segments in `/dev/shm`, by whether a process
    /// still holds them.
    pub fastdds_live: usize,
    pub fastdds_zombie: usize,
    /// Bytes the zombies hold.
    pub fastdds_zombie_bytes: u64,
    pub log_dir: PathBuf,
    pub disk_free: Option<u64>,
    pub disk_total: Option<u64>,
    pub swap_total: Option<u64>,
    pub swap_used: Option<u64>,
    /// Pages swapped in plus out per second since the previous read.
    pub swap_pages_per_s: Option<f64>,
}

/// What `read` needs to carry from one call to the next.
#[derive(Debug, Default)]
pub struct HostState {
    last_swap: Option<(std::time::Instant, u64)>,
}

fn read_u64(path: &str) -> Option<u64> {
    std::fs::read_to_string(path).ok()?.trim().parse().ok()
}

fn statvfs(path: &Path) -> Option<(u64, u64)> {
    use std::os::unix::ffi::OsStrExt;
    let c = std::ffi::CString::new(path.as_os_str().as_bytes()).ok()?;
    let mut s: libc::statvfs = unsafe { std::mem::zeroed() };
    // SAFETY: a valid C string and an out-pointer to a zeroed struct.
    if unsafe { libc::statvfs(c.as_ptr(), &mut s) } != 0 {
        return None;
    }
    let frag = s.f_frsize as u64;
    Some((s.f_blocks as u64 * frag, s.f_bavail as u64 * frag))
}

/// Cyclone's size syntax: a number and an optional unit, `kB`/`KiB` and so on
/// all 1024-based, as Cyclone reads them.
fn parse_cyclone_size(s: &str) -> Option<u64> {
    let s = s.trim();
    let split = s
        .find(|c: char| !c.is_ascii_digit() && c != '.')
        .unwrap_or(s.len());
    let (num, unit) = s.split_at(split);
    let n: f64 = num.parse().ok()?;
    let mult = match unit.trim().to_ascii_lowercase().as_str() {
        "" | "b" => 1u64,
        "kb" | "kib" => 1 << 10,
        "mb" | "mib" => 1 << 20,
        "gb" | "gib" => 1 << 30,
        _ => return None,
    };
    Some((n * mult as f64) as u64)
}

/// `SocketReceiveBufferSize min="…"` from a `CYCLONEDDS_URI`: a `file://`
/// path, a bare path, or inline XML; several comma-separated are allowed.
fn cyclone_rcvbuf_min(uri: &str) -> Option<u64> {
    let mut texts = Vec::new();
    for part in uri.split(',') {
        let part = part.trim();
        if part.starts_with('<') {
            texts.push(part.to_string());
        } else if let Ok(t) = std::fs::read_to_string(part.strip_prefix("file://").unwrap_or(part))
        {
            texts.push(t);
        }
    }
    texts.iter().find_map(|t| {
        let i = t.find("SocketReceiveBufferSize")?;
        let rest = &t[i..t[i..].find('>').map_or(t.len(), |e| i + e)];
        let m = rest.find("min=")? + 4;
        let q = rest[m..].chars().next()?;
        let body = &rest[m + 1..];
        parse_cyclone_size(&body[..body.find(q)?])
    })
}

/// Inodes (device major, minor, inode) holding a lock, from `/proc/locks`.
fn locked_inodes() -> HashSet<(u32, u32, u64)> {
    let Ok(text) = std::fs::read_to_string("/proc/locks") else {
        return HashSet::new();
    };
    text.lines()
        .filter_map(|l| {
            // `1: FLOCK  ADVISORY  WRITE 1234 00:1a:5678 0 EOF`; a blocked
            // waiter's line has a `->` before the type.
            let f: Vec<&str> = l.split_whitespace().filter(|w| *w != "->").collect();
            let mut id = f.get(5)?.split(':');
            let maj = u32::from_str_radix(id.next()?, 16).ok()?;
            let min = u32::from_str_radix(id.next()?, 16).ok()?;
            let ino = id.next()?.parse().ok()?;
            Some((maj, min, ino))
        })
        .collect()
}

/// Fast DDS ports and segments in `dir`: (live, zombie, zombie bytes).
fn fastdds_shm(dir: &Path) -> (usize, usize, u64) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return (0, 0, 0);
    };
    let locked = locked_inodes();
    let (mut live, mut zombie, mut bytes) = (0, 0, 0u64);
    for e in entries.flatten() {
        let name = e.file_name();
        let Some(name) = name.to_str() else { continue };
        if !name.starts_with("fastrtps_") {
            continue;
        }
        let Some(base) = name
            .strip_suffix("_el")
            .or_else(|| name.strip_suffix("_sl"))
        else {
            continue;
        };
        let Ok(meta) = e.metadata() else { continue };
        let dev = meta.dev();
        let key = (libc::major(dev), libc::minor(dev), meta.ino());
        if locked.contains(&key) {
            live += 1;
        } else {
            zombie += 1;
            bytes += std::fs::metadata(dir.join(base)).map_or(0, |m| m.len());
        }
    }
    (live, zombie, bytes)
}

/// Read the host. Blocking.
pub fn read(log_dir: &Path, state: &mut HostState) -> HostFacts {
    let env = |k: &str| std::env::var(k).ok().filter(|v| !v.is_empty());
    let rmw_env = env("RMW_IMPLEMENTATION");
    let rmw = rmw_env
        .clone()
        .unwrap_or_else(|| "rmw_fastrtps_cpp".to_string());
    let dds_config: Vec<String> = [
        "CYCLONEDDS_URI",
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        "FASTDDS_DEFAULT_PROFILES_FILE",
        "RMW_FASTRTPS_USE_QOS_FROM_XML",
        "ROS_LOCALHOST_ONLY",
        "ROS_AUTOMATIC_DISCOVERY_RANGE",
    ]
    .iter()
    .filter_map(|k| env(k).map(|v| format!("{k}={v}")))
    .collect();
    let (shm_total, shm_free) = statvfs(Path::new("/dev/shm")).unzip();
    let (fastdds_live, fastdds_zombie, fastdds_zombie_bytes) = fastdds_shm(Path::new("/dev/shm"));
    let (disk_total, disk_free) = statvfs(log_dir).unzip();

    let meminfo = std::fs::read_to_string("/proc/meminfo").unwrap_or_default();
    let kb = |key: &str| {
        meminfo
            .lines()
            .find(|l| l.starts_with(key))
            .and_then(|l| l.split_whitespace().nth(1)?.parse::<u64>().ok())
            .map(|k| k * 1024)
    };
    let swap_total = kb("SwapTotal:");
    let swap_used = swap_total
        .zip(kb("SwapFree:"))
        .map(|(t, f)| t.saturating_sub(f));
    let vmstat = std::fs::read_to_string("/proc/vmstat").unwrap_or_default();
    let pages = vmstat
        .lines()
        .filter(|l| l.starts_with("pswpin ") || l.starts_with("pswpout "))
        .filter_map(|l| l.split_whitespace().nth(1)?.parse::<u64>().ok())
        .sum::<u64>();
    let now = std::time::Instant::now();
    let swap_pages_per_s = state.last_swap.map(|(t, p)| {
        pages.saturating_sub(p) as f64 / now.duration_since(t).as_secs_f64().max(1.0)
    });
    state.last_swap = Some((now, pages));

    HostFacts {
        rmw_is_default: rmw_env.is_none(),
        rmw_env,
        rmw,
        ros_distro: env("ROS_DISTRO"),
        cyclone_rcvbuf_min: env("CYCLONEDDS_URI").and_then(|u| cyclone_rcvbuf_min(&u)),
        dds_config,
        rmem_max: read_u64("/proc/sys/net/core/rmem_max"),
        wmem_max: read_u64("/proc/sys/net/core/wmem_max"),
        shm_used: shm_total.zip(shm_free).map(|(t, f)| t.saturating_sub(f)),
        shm_total,
        fastdds_live,
        fastdds_zombie,
        fastdds_zombie_bytes,
        log_dir: std::fs::canonicalize(log_dir).unwrap_or_else(|_| log_dir.to_path_buf()),
        disk_free,
        disk_total,
        swap_total,
        swap_used,
        swap_pages_per_s,
    }
}

fn mib(b: u64) -> String {
    if b >= 1 << 30 {
        format!("{:.1} GiB", b as f64 / (1u64 << 30) as f64)
    } else {
        format!("{:.0} MiB", b as f64 / (1u64 << 20) as f64)
    }
}

fn host_condition(
    key: &str,
    kind: &'static str,
    severity: Severity,
    title: String,
    detail: String,
) -> Condition {
    Condition {
        key: key.to_string(),
        kind,
        severity,
        subject: Subject {
            kind: "host",
            name: "host".to_string(),
        },
        title,
        detail,
        link: Some(Link {
            view: "health",
            target: Some("host".to_string()),
        }),
    }
}

pub fn host_conditions(h: &HostFacts) -> Vec<Condition> {
    let mut out = Vec::new();
    let cyclone = h.rmw.contains("cyclonedds");
    let fastdds = h.rmw.contains("fastrtps") || h.rmw.contains("fastdds");
    let has = |k: &str| h.dds_config.iter().any(|c| c.starts_with(&format!("{k}=")));
    let default_note = if h.rmw_is_default {
        " (RMW_IMPLEMENTATION is unset, so the distro default)"
    } else {
        ""
    };
    if has("CYCLONEDDS_URI") && !cyclone {
        out.push(host_condition(
            "rmw_config_ignored:CYCLONEDDS_URI",
            "rmw_config_ignored",
            Severity::Warning,
            "CYCLONEDDS_URI is set, but the RMW is not Cyclone DDS".to_string(),
            format!(
                "the middleware is {}{default_note}; it never reads that file. Set \
                 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp to use it, or unset it",
                h.rmw
            ),
        ));
    }
    for k in [
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        "FASTDDS_DEFAULT_PROFILES_FILE",
    ] {
        if has(k) && !fastdds {
            out.push(host_condition(
                &format!("rmw_config_ignored:{k}"),
                "rmw_config_ignored",
                Severity::Warning,
                format!("{k} is set, but the RMW is not Fast DDS"),
                format!("the middleware is {}; it never reads that file", h.rmw),
            ));
        }
    }
    if cyclone
        && let (Some(min), Some(rmem)) = (h.cyclone_rcvbuf_min, h.rmem_max)
        && min > rmem
    {
        out.push(host_condition(
            "socket_buffer",
            "socket_buffer",
            Severity::Error,
            "Cyclone DDS asks for a larger receive buffer than the kernel allows".to_string(),
            format!(
                "SocketReceiveBufferSize min is {} but net.core.rmem_max is {}: every \
                 participant fails to start. sudo sysctl -w net.core.rmem_max={min}",
                mib(min),
                mib(rmem)
            ),
        ));
    }
    if let (Some(t), Some(u)) = (h.shm_total, h.shm_used)
        && t > 0
        && u as f64 * 100.0 / t as f64 > SHM_WARN_PERCENT
    {
        out.push(host_condition(
            "shm_full",
            "shm_full",
            Severity::Warning,
            format!("/dev/shm is {:.0}% full", u as f64 * 100.0 / t as f64),
            format!(
                "{} of {}; shared-memory transports fail to create segments when it fills",
                mib(u),
                mib(t)
            ),
        ));
    }
    if h.fastdds_zombie > 0 {
        out.push(host_condition(
            "shm_zombies",
            "shm_zombies",
            Severity::Info,
            format!(
                "{} Fast DDS shared-memory files in /dev/shm belong to no process",
                h.fastdds_zombie
            ),
            format!(
                "left by processes that died without cleaning up, {} in all ({} still in \
                 use). `fastdds shm clean` removes them; play_launch deletes nothing",
                mib(h.fastdds_zombie_bytes),
                h.fastdds_live
            ),
        ));
    }
    if let Some(free) = h.disk_free
        && free < DISK_WARN_BYTES
    {
        out.push(host_condition(
            "disk_low",
            "disk_low",
            if free < DISK_ERROR_BYTES {
                Severity::Error
            } else {
                Severity::Warning
            },
            format!("{} free under the log directory", mib(free)),
            format!(
                "{} holds every node's output; when the disk fills, logs stop",
                h.log_dir.display()
            ),
        ));
    }
    if let Some(rate) = h.swap_pages_per_s
        && rate > SWAP_WARN_PAGES_PER_S
    {
        out.push(host_condition(
            "swapping",
            "swapping",
            Severity::Warning,
            format!("the host is swapping, {rate:.0} pages/s"),
            format!(
                "{} of swap in use; a node whose pages are swapped out stalls for \
                 milliseconds per fault, which reads as latency everywhere",
                h.swap_used.map_or("?".to_string(), mib)
            ),
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn facts() -> HostFacts {
        HostFacts {
            rmw: "rmw_fastrtps_cpp".into(),
            rmw_is_default: true,
            rmem_max: Some(212_992),
            shm_total: Some(1 << 30),
            shm_used: Some(1 << 20),
            disk_free: Some(100 << 30),
            ..Default::default()
        }
    }

    fn kinds(h: &HostFacts) -> Vec<&'static str> {
        host_conditions(h).iter().map(|c| c.kind).collect()
    }

    #[test]
    fn a_healthy_host_reports_nothing() {
        assert!(kinds(&facts()).is_empty());
    }

    #[test]
    fn a_cyclone_config_under_fast_dds_is_ignored_and_said_so() {
        let mut h = facts();
        h.dds_config = vec!["CYCLONEDDS_URI=file:///x/cyclonedds.xml".into()];
        let c = host_conditions(&h);
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].kind, "rmw_config_ignored");
        assert!(c[0].detail.contains("unset"), "{}", c[0].detail);
        h.rmw = "rmw_cyclonedds_cpp".into();
        assert!(kinds(&h).is_empty(), "Cyclone reads it");
    }

    #[test]
    fn a_receive_buffer_above_rmem_max_is_an_error_under_cyclone_only() {
        let mut h = facts();
        h.cyclone_rcvbuf_min = Some(10 << 20);
        assert!(kinds(&h).is_empty(), "Fast DDS does not read it");
        h.rmw = "rmw_cyclonedds_cpp".into();
        assert_eq!(kinds(&h), ["socket_buffer"]);
        h.rmem_max = Some(2_147_483_647);
        assert!(kinds(&h).is_empty());
    }

    #[test]
    fn disk_shm_zombies_and_swapping() {
        let mut h = facts();
        h.disk_free = Some(100 << 20);
        h.shm_used = Some(900 << 20);
        h.fastdds_zombie = 3;
        h.swap_pages_per_s = Some(500.0);
        let c = host_conditions(&h);
        let k: Vec<_> = c.iter().map(|c| (c.kind, c.severity)).collect();
        assert!(k.contains(&("disk_low", Severity::Error)), "{k:?}");
        assert!(k.contains(&("shm_full", Severity::Warning)), "{k:?}");
        assert!(k.contains(&("shm_zombies", Severity::Info)), "{k:?}");
        assert!(k.contains(&("swapping", Severity::Warning)), "{k:?}");
    }

    #[test]
    fn cyclone_sizes_and_configs_parse() {
        assert_eq!(parse_cyclone_size("10MB"), Some(10 << 20));
        assert_eq!(parse_cyclone_size("64 KiB"), Some(64 << 10));
        assert_eq!(parse_cyclone_size("1000"), Some(1000));
        assert_eq!(parse_cyclone_size("ten"), None);
        let inline = r#"<CycloneDDS><Domain><Internal><SocketReceiveBufferSize min="10MB"/></Internal></Domain></CycloneDDS>"#;
        assert_eq!(cyclone_rcvbuf_min(inline), Some(10 << 20));
        assert_eq!(cyclone_rcvbuf_min("<CycloneDDS/>"), None);
    }

    #[test]
    fn a_held_lock_is_live_and_a_released_one_a_zombie() {
        let dir = tempfile::tempdir().unwrap();
        let held = dir.path().join("fastrtps_aaaaaaaaaaaaaaaa_el");
        let dead = dir.path().join("fastrtps_bbbbbbbbbbbbbbbb_el");
        std::fs::write(
            dir.path().join("fastrtps_bbbbbbbbbbbbbbbb"),
            vec![0u8; 4096],
        )
        .unwrap();
        let f = std::fs::File::create(&held).unwrap();
        std::fs::File::create(&dead).unwrap();
        use std::os::fd::AsRawFd;
        // SAFETY: flock on a descriptor this test owns.
        assert_eq!(unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_SH) }, 0);
        assert_eq!(fastdds_shm(dir.path()), (1, 1, 4096));
        drop(f);
        assert_eq!(fastdds_shm(dir.path()).0, 0, "released: no longer live");
    }
}
