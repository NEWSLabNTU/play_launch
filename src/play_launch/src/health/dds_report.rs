//! The DDS report (phase 89 D4, `docs/design/dds-assist.md` item 4): one
//! block of plain text holding every fact a DDS question needs, to paste
//! into an issue or a forum post. Most such requests stall on exactly these:
//! which RMW and version, which config file and whether it is read at all,
//! the kernel's buffers, `/dev/shm`, how many participants against what
//! limit, drops, and what the Health page already found.
//!
//! Shown on the Health page (`GET /api/health/dds_report`) and written to
//! the bundle as `dds_report.txt` each time the host is read and at the end
//! of the run. It reports; it changes nothing.

use std::fmt::Write;

use super::{Issue, discovery, host};
use ros_launch_resolve::ros::dds_env::DdsEnvSplit;

/// Issue kinds about DDS, listed in the report when open.
const DDS_KINDS: [&str; 14] = [
    "rmw_config_ignored",
    "socket_buffer",
    "shm_full",
    "shm_zombies",
    "participants_over_limit",
    "dds_udp_drops",
    "discovery_pressure",
    "dds_env_split",
    "large_udp_message",
    "qos_incompatible",
    "missing_node",
    "foreign_node",
    "duplicate_node",
    "never_ready",
];

/// Issues of one kind beyond this are summarised in one line.
const GROUP_AT: usize = 3;

/// A package's `<version>` from the first `share/<pkg>/package.xml` on
/// `AMENT_PREFIX_PATH`.
fn package_version(pkg: &str) -> Option<String> {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").ok()?;
    prefixes.split(':').find_map(|p| {
        let xml = std::fs::read_to_string(
            std::path::Path::new(p)
                .join("share")
                .join(pkg)
                .join("package.xml"),
        )
        .ok()?;
        let s = xml.find("<version>")? + "<version>".len();
        let e = xml[s..].find('<')? + s;
        Some(xml[s..e].trim().to_string())
    })
}

/// A CMake package's version from its `*-config-version.cmake` or
/// `*ConfigVersion.cmake` (`set(PACKAGE_VERSION "2.6.12")`), under
/// `share/<dir>/cmake` or `lib/cmake/<dir>` of a prefix on
/// `AMENT_PREFIX_PATH`. DDS libraries are CMake packages, not ament ones:
/// Humble's Fast DDS has no `package.xml`.
fn cmake_version(dirs: &[&str]) -> Option<String> {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").ok()?;
    for p in prefixes.split(':') {
        let p = std::path::Path::new(p);
        for d in dirs {
            for dir in [
                p.join("share").join(d).join("cmake"),
                p.join("lib/cmake").join(d),
            ] {
                let Ok(entries) = std::fs::read_dir(&dir) else {
                    continue;
                };
                for e in entries.flatten() {
                    let name = e.file_name().to_string_lossy().to_lowercase();
                    if !name.ends_with("version.cmake") {
                        continue;
                    }
                    let Ok(text) = std::fs::read_to_string(e.path()) else {
                        continue;
                    };
                    let Some(i) = text.find("set(PACKAGE_VERSION \"") else {
                        continue;
                    };
                    let rest = &text[i + "set(PACKAGE_VERSION \"".len()..];
                    if let Some(end) = rest.find('"') {
                        return Some(rest[..end].to_string());
                    }
                }
            }
        }
    }
    None
}

/// The DDS library an RMW wraps, and its version.
fn dds_library(rmw: &str) -> Option<(&'static str, Option<String>)> {
    if rmw.contains("cyclonedds") {
        Some(("Cyclone DDS", cmake_version(&["CycloneDDS", "cyclonedds"])))
    } else if rmw.contains("fastrtps") || rmw.contains("fastdds") {
        Some(("Fast DDS", cmake_version(&["fastrtps", "fastdds"])))
    } else if rmw.contains("connextdds") {
        Some(("RTI Connext DDS", None))
    } else {
        None
    }
}

fn bytes(b: Option<u64>) -> String {
    match b {
        None => "unknown".to_string(),
        Some(b) if b >= 1 << 20 => format!("{} ({:.1} MiB)", b, b as f64 / (1u64 << 20) as f64),
        Some(b) => format!("{b} ({:.0} KiB)", b as f64 / 1024.0),
    }
}

fn opt<T: std::fmt::Display>(v: Option<T>) -> String {
    v.map_or("unknown".to_string(), |v| v.to_string())
}

/// What the report is built from.
pub struct ReportInputs<'a> {
    pub host: Option<&'a host::HostFacts>,
    pub discovery: Option<&'a discovery::DiscoveryView>,
    pub dds_env: &'a [DdsEnvSplit],
    pub startup_drops: &'a discovery::StartupDrops,
    pub open: &'a [Issue],
    /// Wall time, ms since the epoch.
    pub now_ms: u64,
}

pub fn render(r: &ReportInputs) -> String {
    let mut o = String::new();
    let _ = writeln!(
        o,
        "play_launch {} DDS report, {} (unix ms {})",
        env!("CARGO_PKG_VERSION"),
        chrono::DateTime::from_timestamp_millis(r.now_ms as i64)
            .map(|t| t.to_rfc3339())
            .unwrap_or_default(),
        r.now_ms
    );
    let _ = writeln!(o);

    let _ = writeln!(o, "## Middleware");
    match r.host {
        None => {
            let _ = writeln!(o, "(the host has not been read yet)");
        }
        Some(h) => {
            let _ = writeln!(
                o,
                "RMW: {} {}{}",
                h.rmw,
                opt(package_version(&h.rmw)),
                if h.rmw_is_default {
                    " (RMW_IMPLEMENTATION unset: the distro default)"
                } else {
                    ""
                }
            );
            if let Some((name, version)) = dds_library(&h.rmw) {
                let _ = writeln!(o, "DDS: {name} {}", opt(version));
            }
            let _ = writeln!(
                o,
                "ROS_DISTRO: {}; ROS_DOMAIN_ID: {}",
                opt(h.ros_distro.as_deref()),
                h.domain_id
            );
            if h.dds_config.is_empty() {
                let _ = writeln!(o, "DDS variables: none set");
            }
            let cyclone = h.rmw.contains("cyclonedds");
            let fastdds = h.rmw.contains("fastrtps") || h.rmw.contains("fastdds");
            for c in &h.dds_config {
                let ignored = (c.starts_with("CYCLONEDDS_URI=") && !cyclone)
                    || ((c.starts_with("FASTRTPS_DEFAULT_PROFILES_FILE=")
                        || c.starts_with("FASTDDS_DEFAULT_PROFILES_FILE="))
                        && !fastdds);
                let _ = writeln!(
                    o,
                    "DDS variable: {c}{}",
                    if ignored {
                        "  <- NOT READ: the RMW in effect is not that middleware"
                    } else {
                        ""
                    }
                );
            }
            if let Some(idx) = &h.cyclone_participant_index {
                let _ = writeln!(
                    o,
                    "Cyclone ParticipantIndex: {idx}; MaxAutoParticipantIndex: {}",
                    opt(h.cyclone_max_auto_index)
                );
            }
            if let Some(m) = h.cyclone_rcvbuf_min {
                let _ = writeln!(o, "Cyclone SocketReceiveBufferSize min: {}", bytes(Some(m)));
            }
            let _ = writeln!(o);
            let _ = writeln!(o, "## Kernel");
            let _ = writeln!(o, "net.core.rmem_max: {}", bytes(h.rmem_max));
            let _ = writeln!(o, "net.core.wmem_max: {}", bytes(h.wmem_max));
            let _ = writeln!(o, "net.ipv4.ipfrag_time: {} s", opt(h.ipfrag_time_s));
            let _ = writeln!(
                o,
                "net.ipv4.ipfrag_high_thresh: {}",
                bytes(h.ipfrag_high_thresh)
            );
            let _ = writeln!(
                o,
                "UDP receive-buffer drops, host-wide: {} a minute",
                h.udp_rcvbuf_errors_per_min
                    .map_or("unknown".to_string(), |r| format!("{r:.0}"))
            );
            let _ = writeln!(
                o,
                "/dev/shm: {} used of {}; Fast DDS files: {} in use, {} orphaned ({})",
                bytes(h.shm_used),
                bytes(h.shm_total),
                h.fastdds_live,
                h.fastdds_zombie,
                bytes(Some(h.fastdds_zombie_bytes))
            );
            let _ = writeln!(
                o,
                "Swap: {} used of {}",
                bytes(h.swap_used),
                bytes(h.swap_total)
            );
        }
    }
    let _ = writeln!(o);

    let _ = writeln!(o, "## Participants and discovery");
    match r.discovery {
        None => {
            let _ = writeln!(o, "(no members yet)");
        }
        Some(d) => {
            let _ = writeln!(
                o,
                "Participants held by this run: {} (plus up to {} plain executables)",
                d.participants, d.executables
            );
            let _ = writeln!(
                o,
                "Under --container-mode observable: {}",
                d.participants_observable
            );
            let _ = writeln!(
                o,
                "Limit: {}{}",
                opt(d.limit),
                d.limit_reason
                    .as_deref()
                    .map(|r| format!(" ({r})"))
                    .unwrap_or_default()
            );
            let _ = writeln!(
                o,
                "Nodes on the domain at the last graph read: {}",
                opt(d.graph_nodes)
            );
            let _ = writeln!(
                o,
                "Sockets on the domain's ports: {}; their drops now: {} a minute",
                d.domain_udp_sockets,
                d.domain_udp_drops_per_min
                    .map_or("unknown".to_string(), |r| format!("{r:.0}"))
            );
        }
    }
    let _ = writeln!(
        o,
        "Dropped on the domain's sockets while starting: {} (peak {:.0} a minute)",
        r.startup_drops.total, r.startup_drops.peak_per_min
    );
    let _ = writeln!(o);

    let _ = writeln!(o, "## Per-node DDS environment (from the launch file)");
    if r.dds_env.is_empty() {
        let _ = writeln!(o, "No node sets a DDS variable different from the rest.");
    }
    for s in r.dds_env {
        let _ = writeln!(o, "- {}", s.message());
    }
    let _ = writeln!(o);

    let _ = writeln!(o, "## Open DDS issues");
    let mut any = false;
    for kind in DDS_KINDS {
        let of: Vec<&Issue> = r.open.iter().filter(|i| i.kind == kind).collect();
        if of.is_empty() {
            continue;
        }
        any = true;
        // Several of one kind are one fact: say how many, name a few, and
        // give the first one's explanation once.
        if of.len() > GROUP_AT {
            let names: Vec<&str> = of.iter().take(5).map(|i| i.subject.name.as_str()).collect();
            let _ = writeln!(
                o,
                "- [{kind}] {} open: {}{}. First: {}: {}",
                of.len(),
                names.join(", "),
                if of.len() > names.len() { ", …" } else { "" },
                of[0].title,
                of[0].detail
            );
            continue;
        }
        for i in of {
            let _ = writeln!(
                o,
                "- [{}] {} ({}): {}",
                i.kind, i.title, i.subject.name, i.detail
            );
        }
    }
    if !any {
        let _ = writeln!(o, "None.");
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_report_holds_the_config_that_is_not_read_and_the_counts() {
        let h = host::HostFacts {
            rmw: "rmw_fastrtps_cpp".into(),
            rmw_is_default: true,
            dds_config: vec!["CYCLONEDDS_URI=file:///a.xml".into()],
            ipfrag_time_s: Some(30),
            ipfrag_high_thresh: Some(4 << 20),
            fastdds_zombie: 773,
            ..Default::default()
        };
        let d = discovery::DiscoveryView {
            participants: 119,
            participants_observable: 49,
            limit: Some(120),
            ..Default::default()
        };
        let text = render(&ReportInputs {
            host: Some(&h),
            discovery: Some(&d),
            dds_env: &[],
            startup_drops: &Default::default(),
            open: &[],
            now_ms: 0,
        });
        for want in [
            "CYCLONEDDS_URI=file:///a.xml  <- NOT READ",
            "(RMW_IMPLEMENTATION unset: the distro default)",
            "net.ipv4.ipfrag_time: 30 s",
            "net.ipv4.ipfrag_high_thresh: 4194304 (4.0 MiB)",
            "773 orphaned",
            "Participants held by this run: 119",
            "Under --container-mode observable: 49",
            "Limit: 120",
        ] {
            assert!(text.contains(want), "missing {want:?} in\n{text}");
        }
    }
}
