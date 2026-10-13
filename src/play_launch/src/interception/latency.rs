//! Delivery latency per subscription: how long a message waited between its
//! publish and the take that handed it to a node — transport plus the
//! subscriber's executor queue (`docs/design/health-page.md` §D).
//!
//! The interceptor reports a publish and every take of a message with the
//! same `header.stamp`, so the stamp is a KEY: per topic, the last
//! [`STAMPS_PER_TOPIC`] stamps map to their publish time, and a take with a
//! stamp looks its own up. Equality, never comparison, of stamps, so it works
//! under `use_sim_time` and with any clock the publisher stamps from. Both
//! times are `CLOCK_MONOTONIC`, which is one clock across processes.
//!
//! Each child has its own ring and the consumer drains them in turn, so a
//! take can be read before the publish it took. Such a take waits in
//! `pending` and is retried after each drain pass; one still unmatched after
//! [`PENDING_TTL_NS`] had no publish play_launch saw (a publisher outside the
//! launch, a stamp republished from elsewhere, a ring overflow) and counts
//! only against coverage.
//!
//! Not measured: unstamped types (no key), and rclcpp's intra-process
//! transport, which bypasses rcl.

use std::collections::{HashMap, VecDeque};

/// Stamps kept per topic. At 1 kHz this is half a second of history, longer
/// than any take a healthy subscriber makes.
const STAMPS_PER_TOPIC: usize = 512;
/// Takes waiting for their publish, across all topics.
const PENDING_CAP: usize = 4096;
/// A pending take still unmatched this long after it happened is dropped.
const PENDING_TTL_NS: u64 = 1_000_000_000;
/// Length of one histogram epoch. The live view reads the current epoch and
/// the one before it, so it covers the last 10 to 20 seconds.
pub(crate) const EPOCH_NS: u64 = 10_000_000_000;

/// Log-scale buckets from [`LO_NS`] upward, each about 1.25 times the last;
/// 64 of them reach past 10 s, the last one open-ended.
const BUCKETS: usize = 64;
const LO_NS: f64 = 10_000.0;
/// `ln((1e10 / 1e4)^(1/62))`: 10 µs to 10 s in 62 steps (a ratio of about
/// 1.25), plus one bucket under and one over.
const LN_RATIO: f64 = 0.222_830_815_451_036_7;

fn bucket_of(ns: u64) -> usize {
    if (ns as f64) < LO_NS {
        return 0;
    }
    let i = ((ns as f64 / LO_NS).ln() / LN_RATIO).floor() as usize + 1;
    i.min(BUCKETS - 1)
}

/// The upper edge of a bucket in ns: what a percentile falling in it reports,
/// so the figure errs high by at most one bucket (25%).
fn upper_ns(bucket: usize) -> f64 {
    LO_NS * (LN_RATIO * bucket as f64).exp()
}

#[derive(Debug, Clone)]
pub(crate) struct Histogram {
    counts: [u32; BUCKETS],
    total: u64,
    max_ns: u64,
}

impl Default for Histogram {
    fn default() -> Self {
        Self {
            counts: [0; BUCKETS],
            total: 0,
            max_ns: 0,
        }
    }
}

impl Histogram {
    fn add(&mut self, ns: u64) {
        let b = &mut self.counts[bucket_of(ns)];
        *b = b.saturating_add(1);
        self.total += 1;
        self.max_ns = self.max_ns.max(ns);
    }

    fn merge(&mut self, other: &Histogram) {
        for (a, b) in self.counts.iter_mut().zip(other.counts.iter()) {
            *a = a.saturating_add(*b);
        }
        self.total += other.total;
        self.max_ns = self.max_ns.max(other.max_ns);
    }

    pub(crate) fn samples(&self) -> u64 {
        self.total
    }

    /// The `q` quantile in milliseconds, never above the largest sample.
    pub(crate) fn quantile_ms(&self, q: f64) -> Option<f64> {
        if self.total == 0 {
            return None;
        }
        let rank = ((q * self.total as f64).ceil() as u64).max(1);
        let mut seen = 0u64;
        for (i, &c) in self.counts.iter().enumerate() {
            seen += c as u64;
            if seen >= rank {
                // The last bucket is open above: its "upper edge" (12.5 s)
                // would cap every larger value, and a stamp held for 40 s
                // read 12.5 s. Report the largest sample instead, which errs
                // high as every other bucket does.
                if i == BUCKETS - 1 {
                    return Some(self.max_ns as f64 / 1e6);
                }
                return Some(upper_ns(i).min(self.max_ns as f64) / 1e6);
            }
        }
        Some(self.max_ns as f64 / 1e6)
    }

    pub(crate) fn max_ms(&self) -> Option<f64> {
        (self.total > 0).then(|| self.max_ns as f64 / 1e6)
    }
}

/// One subscription's latencies: the run, and two epochs for the live view.
#[derive(Debug, Clone, Default)]
pub(crate) struct Latency {
    pub(crate) run: Histogram,
    epoch: u64,
    current: Histogram,
    previous: Histogram,
    /// Takes that carried a stamp, matched or not.
    pub(crate) stamped_takes: u64,
    pub(crate) matched: u64,
}

impl Latency {
    pub(crate) fn add(&mut self, at_ns: u64, latency_ns: u64) {
        self.rotate(at_ns);
        self.current.add(latency_ns);
        self.run.add(latency_ns);
        self.matched += 1;
    }

    fn rotate(&mut self, at_ns: u64) {
        let e = at_ns / EPOCH_NS;
        if e == self.epoch {
            return;
        }
        self.previous = if e == self.epoch + 1 {
            std::mem::take(&mut self.current)
        } else {
            Histogram::default()
        };
        self.current = Histogram::default();
        self.epoch = e;
    }

    /// The last 10 to 20 seconds as of `now_ns`.
    pub(crate) fn recent(&self, now_ns: u64) -> Histogram {
        let e = now_ns / EPOCH_NS;
        let mut out = Histogram::default();
        if self.epoch == e {
            out.merge(&self.current);
            out.merge(&self.previous);
        } else if self.epoch + 1 == e {
            out.merge(&self.current);
        }
        out
    }

    /// Matched takes over stamped ones; `None` with no stamped take.
    pub(crate) fn coverage(&self) -> Option<f64> {
        (self.stamped_takes > 0).then(|| self.matched as f64 / self.stamped_takes as f64)
    }
}

/// When a stamp was published: the first and the latest publish of it, for a
/// topic where a relay republishes a stamp an upstream node already sent.
#[derive(Debug, Clone, Copy)]
struct Published {
    first_ns: u64,
    latest_ns: u64,
}

#[derive(Debug, Default)]
struct StampIndex {
    at: HashMap<u64, Published>,
    order: VecDeque<u64>,
}

impl StampIndex {
    fn insert(&mut self, stamp: u64, at_ns: u64) {
        if let Some(p) = self.at.get_mut(&stamp) {
            p.latest_ns = p.latest_ns.max(at_ns);
            return;
        }
        if self.order.len() >= STAMPS_PER_TOPIC
            && let Some(old) = self.order.pop_front()
        {
            self.at.remove(&old);
        }
        self.at.insert(
            stamp,
            Published {
                first_ns: at_ns,
                latest_ns: at_ns,
            },
        );
        self.order.push_back(stamp);
    }

    /// The latest publish of `stamp` not after `take_ns`.
    fn publish_before(&self, stamp: u64, take_ns: u64) -> Option<u64> {
        let p = self.at.get(&stamp)?;
        if p.latest_ns <= take_ns {
            Some(p.latest_ns)
        } else if p.first_ns <= take_ns {
            Some(p.first_ns)
        } else {
            None
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct PendingTake<K> {
    key: K,
    topic: u64,
    stamp: u64,
    take_ns: u64,
}

/// Matches takes to publishes. `K` identifies a subscription.
#[derive(Debug)]
pub(crate) struct Matcher<K> {
    topics: HashMap<u64, StampIndex>,
    pending: VecDeque<PendingTake<K>>,
}

impl<K> Default for Matcher<K> {
    fn default() -> Self {
        Self {
            topics: HashMap::new(),
            pending: VecDeque::new(),
        }
    }
}

/// A stamp as one key; `None` for the zero stamp an unstamped message carries.
pub(crate) fn stamp_key(sec: i32, nanosec: u32) -> Option<u64> {
    let k = ((sec as u32 as u64) << 32) | nanosec as u64;
    (k != 0).then_some(k)
}

impl<K: Copy + Eq + std::hash::Hash> Matcher<K> {
    pub(crate) fn publish(&mut self, topic: u64, stamp: u64, at_ns: u64) {
        self.topics.entry(topic).or_default().insert(stamp, at_ns);
    }

    /// Record a stamped take on `lat`; a take whose publish is not yet seen
    /// waits for [`Matcher::settle`].
    pub(crate) fn take(&mut self, key: K, lat: &mut Latency, topic: u64, stamp: u64, take_ns: u64) {
        lat.stamped_takes += 1;
        match self
            .topics
            .get(&topic)
            .and_then(|t| t.publish_before(stamp, take_ns))
        {
            Some(pub_ns) => lat.add(take_ns, take_ns - pub_ns),
            None => {
                if self.pending.len() >= PENDING_CAP {
                    self.pending.pop_front();
                }
                self.pending.push_back(PendingTake {
                    key,
                    topic,
                    stamp,
                    take_ns,
                });
            }
        }
    }

    /// Retry waiting takes and return the ones now matched, as
    /// `(subscription, take time, latency)`; drop those older than
    /// [`PENDING_TTL_NS`] at `now_ns`.
    pub(crate) fn settle(&mut self, now_ns: u64) -> Vec<(K, u64, u64)> {
        let mut matched = Vec::new();
        let topics = &self.topics;
        self.pending.retain(|p| {
            match topics
                .get(&p.topic)
                .and_then(|t| t.publish_before(p.stamp, p.take_ns))
            {
                Some(pub_ns) => {
                    matched.push((p.key, p.take_ns, p.take_ns - pub_ns));
                    false
                }
                None => now_ns.saturating_sub(p.take_ns) < PENDING_TTL_NS,
            }
        });
        matched
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    #[test]
    fn a_percentile_past_ten_seconds_is_not_capped_at_the_last_edge() {
        let mut h = Histogram::default();
        for _ in 0..10 {
            h.add(40_000 * MS);
        }
        assert_eq!(h.quantile_ms(0.5), Some(40_000.0));
    }

    #[test]
    fn buckets_cover_ten_microseconds_to_ten_seconds() {
        assert_eq!(bucket_of(0), 0);
        assert_eq!(bucket_of(9_999), 0);
        assert_eq!(bucket_of(10_000), 1);
        assert!(upper_ns(BUCKETS - 2) >= 1e10, "{}", upper_ns(BUCKETS - 2));
        // Every sample sits at or under its bucket's upper edge.
        for ns in [10_000u64, 123_456, 50 * MS, 3_000 * MS] {
            let b = bucket_of(ns);
            assert!(
                upper_ns(b) >= ns as f64 && upper_ns(b - 1) <= ns as f64,
                "{ns}"
            );
        }
    }

    #[test]
    fn quantiles_err_high_by_at_most_a_bucket() {
        let mut h = Histogram::default();
        for i in 1..=100u64 {
            h.add(i * MS);
        }
        let p50 = h.quantile_ms(0.5).unwrap();
        let p99 = h.quantile_ms(0.99).unwrap();
        assert!((50.0..=50.0 * LN_RATIO.exp()).contains(&p50), "{p50}");
        assert!((99.0..=100.0).contains(&p99), "{p99}");
        assert_eq!(h.max_ms(), Some(100.0));
    }

    #[test]
    fn a_take_is_matched_to_its_publish_by_stamp() {
        let mut m: Matcher<u8> = Matcher::default();
        let mut lat = Latency::default();
        m.publish(1, 7, 100 * MS);
        m.publish(1, 8, 150 * MS);
        m.take(0, &mut lat, 1, 7, 130 * MS);
        m.take(0, &mut lat, 1, 8, 152 * MS);
        assert_eq!(lat.matched, 2);
        assert_eq!(lat.run.max_ms(), Some(30.0));
    }

    #[test]
    fn a_take_read_before_its_publish_is_matched_on_settle() {
        let mut m: Matcher<u8> = Matcher::default();
        let mut lat = Latency::default();
        // The subscriber's ring drained first.
        m.take(0, &mut lat, 1, 7, 140 * MS);
        assert_eq!(lat.matched, 0);
        m.publish(1, 7, 100 * MS);
        for (_, at, l) in m.settle(150 * MS) {
            lat.add(at, l);
        }
        assert_eq!((lat.stamped_takes, lat.matched), (1, 1));
        assert_eq!(lat.run.max_ms(), Some(40.0));
    }

    #[test]
    fn an_unmatched_take_counts_against_coverage_and_expires() {
        let mut m: Matcher<u8> = Matcher::default();
        let mut lat = Latency::default();
        m.take(0, &mut lat, 1, 7, 100 * MS);
        assert!(m.settle(2_000 * MS).is_empty());
        assert!(m.pending.is_empty());
        assert_eq!(lat.coverage(), Some(0.0));
    }

    #[test]
    fn a_republished_stamp_matches_the_latest_publish_before_the_take() {
        let mut m: Matcher<u8> = Matcher::default();
        let mut lat = Latency::default();
        m.publish(1, 7, 100 * MS);
        m.publish(1, 7, 120 * MS);
        m.take(0, &mut lat, 1, 7, 125 * MS);
        assert_eq!(lat.run.max_ms(), Some(5.0));
        // A take between the two publishes took the first.
        m.take(0, &mut lat, 1, 7, 110 * MS);
        assert_eq!(lat.run.samples(), 2);
        assert!(lat.run.quantile_ms(0.0).unwrap() <= 5.0 * LN_RATIO.exp());
    }

    #[test]
    fn the_index_forgets_the_oldest_stamps() {
        let mut m: Matcher<u8> = Matcher::default();
        let mut lat = Latency::default();
        for s in 1..=(STAMPS_PER_TOPIC as u64 + 1) {
            m.publish(1, s, s * MS);
        }
        m.take(0, &mut lat, 1, 1, 10_000 * MS);
        assert_eq!(lat.matched, 0, "stamp 1 was evicted");
        m.take(0, &mut lat, 1, 2, 10_000 * MS);
        assert_eq!(lat.matched, 1);
    }

    #[test]
    fn the_live_view_covers_the_last_two_epochs_only() {
        let mut lat = Latency::default();
        lat.add(5 * EPOCH_NS, 200 * MS);
        lat.add(6 * EPOCH_NS + MS, MS);
        assert_eq!(lat.recent(6 * EPOCH_NS + 2 * MS).samples(), 2);
        assert_eq!(lat.recent(7 * EPOCH_NS).samples(), 1);
        assert_eq!(lat.recent(8 * EPOCH_NS).samples(), 0);
        assert_eq!(lat.run.samples(), 2);
    }

    #[test]
    fn the_zero_stamp_is_no_key() {
        assert_eq!(stamp_key(0, 0), None);
        assert!(stamp_key(0, 1).is_some());
        assert_ne!(stamp_key(1, 0), stamp_key(0, 1));
    }
}
