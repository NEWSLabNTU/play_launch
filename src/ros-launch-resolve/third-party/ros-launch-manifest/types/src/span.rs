//! Source span tracking for diagnostic output.
//!
//! Provides a [`SpanIndex`] that scans YAML source text to map key paths
//! (like `"topics.chatter"`) to byte offset ranges. This avoids modifying
//! the YAML AST while still enabling source-level diagnostics.

use std::{collections::HashMap, ops::Range};

/// A value paired with its source location.
#[derive(Debug, Clone)]
pub struct Spanned<T> {
    pub value: T,
    pub span: Range<usize>,
}

impl<T> Spanned<T> {
    pub fn new(value: T, span: Range<usize>) -> Self {
        Self { value, span }
    }

    /// Create a Spanned with no location info (span 0..0).
    pub fn no_span(value: T) -> Self {
        Self { value, span: 0..0 }
    }
}

impl<T: PartialEq> PartialEq for Spanned<T> {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

/// Build a line-starts table from source text.
/// Returns byte offsets where each line begins.
pub fn line_starts(source: &str) -> Vec<usize> {
    let mut starts = vec![0];
    for (i, b) in source.bytes().enumerate() {
        if b == b'\n' {
            starts.push(i + 1);
        }
    }
    starts
}

/// Convert a (line, col) pair to a byte offset.
/// Lines and columns are 0-based.
pub fn line_col_to_offset(starts: &[usize], line: usize, col: usize) -> usize {
    if line < starts.len() {
        starts[line] + col
    } else {
        starts.last().copied().unwrap_or(0)
    }
}

/// Index mapping YAML key paths to byte ranges in the source text.
///
/// Built by scanning the source with yaml-rust2's event parser, which
/// provides `Marker { index, line, col }` for every token. Paths are
/// dot-separated (e.g., `"topics.chatter"`, `"nodes.ndt.pub.sensor_points"`).
#[derive(Debug, Clone, Default)]
pub struct SpanIndex {
    /// Map from dot-path → byte range of the key token.
    pub spans: HashMap<String, Range<usize>>,
    /// The original source text.
    pub source: String,
}

impl SpanIndex {
    /// Build a span index by replaying the YAML parser events.
    pub fn build(source: &str) -> Self {
        let mut index = SpanIndex {
            spans: HashMap::new(),
            source: source.to_string(),
        };

        let mut receiver = SpanReceiver::new();
        let mut parser = yaml_rust2::parser::Parser::new_from_str(source);
        let _ = parser.load(&mut receiver, true);

        index.spans = receiver.spans;
        index
    }

    /// Look up the byte range for a dot-separated key path.
    pub fn get(&self, path: &str) -> Option<Range<usize>> {
        self.spans.get(path).cloned()
    }
}

/// Event receiver that builds the span index.
struct SpanReceiver {
    /// Stack of (path_segment, is_mapping) for tracking nesting.
    stack: Vec<StackEntry>,
    /// The current mapping key (set by Scalar events when we're in key position).
    pending_key: Option<(String, usize)>,
    /// Whether the next scalar is a key or a value.
    expect_key: bool,
    /// Collected spans.
    spans: HashMap<String, Range<usize>>,
}

struct StackEntry {
    segment: String,
    is_mapping: bool,
    seq_index: usize,
}

impl SpanReceiver {
    fn new() -> Self {
        Self {
            stack: Vec::new(),
            pending_key: None,
            expect_key: false,
            spans: HashMap::new(),
        }
    }

    fn current_path(&self) -> String {
        self.stack
            .iter()
            .filter(|e| !e.segment.is_empty())
            .map(|e| e.segment.as_str())
            .collect::<Vec<_>>()
            .join(".")
    }
}

impl yaml_rust2::parser::MarkedEventReceiver for SpanReceiver {
    fn on_event(&mut self, ev: yaml_rust2::parser::Event, mark: yaml_rust2::scanner::Marker) {
        use yaml_rust2::parser::Event;

        match ev {
            Event::MappingStart(_, _) => {
                // If we had a pending key, push it as the current segment
                let segment = if let Some((key, _)) = self.pending_key.take() {
                    key
                } else {
                    String::new()
                };
                self.stack.push(StackEntry {
                    segment,
                    is_mapping: true,
                    seq_index: 0,
                });
                self.expect_key = true;
            }
            Event::MappingEnd => {
                self.stack.pop();
                // After closing a mapping value, next scalar in parent mapping is a key
                if let Some(parent) = self.stack.last()
                    && parent.is_mapping
                {
                    self.expect_key = true;
                }
            }
            Event::SequenceStart(_, _) => {
                let segment = if let Some((key, _)) = self.pending_key.take() {
                    key
                } else {
                    String::new()
                };
                self.stack.push(StackEntry {
                    segment,
                    is_mapping: false,
                    seq_index: 0,
                });
            }
            Event::SequenceEnd => {
                self.stack.pop();
                if let Some(parent) = self.stack.last()
                    && parent.is_mapping
                {
                    self.expect_key = true;
                }
            }
            Event::Scalar(ref value, _, _, _) => {
                if self.expect_key {
                    // This scalar is a mapping key
                    let byte_offset = mark.index();
                    self.pending_key = Some((value.clone(), byte_offset));
                    self.expect_key = false;

                    // Record span for this key path
                    let parent_path = self.current_path();
                    let full_path = if parent_path.is_empty() {
                        value.clone()
                    } else {
                        format!("{parent_path}.{value}")
                    };
                    let key_end = byte_offset + value.len();
                    self.spans.insert(full_path, byte_offset..key_end);
                } else {
                    // This scalar is a value — after consuming a value in a mapping,
                    // the next scalar is a key again
                    self.pending_key = None;
                    if let Some(parent) = self.stack.last_mut() {
                        if parent.is_mapping {
                            self.expect_key = true;
                        } else {
                            parent.seq_index += 1;
                        }
                    }
                }
            }
            Event::DocumentStart
            | Event::DocumentEnd
            | Event::StreamStart
            | Event::StreamEnd
            | Event::Nothing
            | Event::Alias(_) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line_starts() {
        let src = "abc\ndef\nghi";
        let starts = line_starts(src);
        assert_eq!(starts, vec![0, 4, 8]);
    }

    #[test]
    fn test_line_col_to_offset() {
        let starts = vec![0, 4, 8];
        assert_eq!(line_col_to_offset(&starts, 0, 0), 0);
        assert_eq!(line_col_to_offset(&starts, 0, 2), 2);
        assert_eq!(line_col_to_offset(&starts, 1, 0), 4);
        assert_eq!(line_col_to_offset(&starts, 2, 1), 9);
    }

    #[test]
    fn test_span_index_simple() {
        let yaml = "version: 1\nnodes:\n  talker:\n    pub: [chatter]\n";
        let idx = SpanIndex::build(yaml);

        assert!(
            idx.get("version").is_some(),
            "spans: {:?}",
            idx.spans.keys().collect::<Vec<_>>()
        );
        assert!(idx.get("nodes").is_some());
        assert!(idx.get("nodes.talker").is_some());
        assert!(idx.get("nodes.talker.pub").is_some());
    }

    #[test]
    fn test_span_index_topics() {
        let yaml = "topics:\n  chatter:\n    type: std_msgs/msg/String\n    rate_hz: 10\n";
        let idx = SpanIndex::build(yaml);

        assert!(idx.get("topics").is_some());
        assert!(idx.get("topics.chatter").is_some());
        assert!(idx.get("topics.chatter.type").is_some());
        assert!(idx.get("topics.chatter.rate_hz").is_some());
    }

    #[test]
    fn test_span_index_byte_offset() {
        let yaml = "topics:\n  chatter:\n    type: std_msgs/msg/String\n";
        let idx = SpanIndex::build(yaml);

        let span = idx.get("topics.chatter").unwrap();
        assert_eq!(&yaml[span.clone()], "chatter");

        let span = idx.get("topics.chatter.type").unwrap();
        assert_eq!(&yaml[span.clone()], "type");
    }

    #[test]
    fn test_span_index_qos_nested() {
        let yaml = "topics:\n  pc:\n    qos:\n      reliability: best_effort\n      durability: volatile\n";
        let idx = SpanIndex::build(yaml);

        assert!(idx.get("topics.pc.qos").is_some());
        assert!(idx.get("topics.pc.qos.reliability").is_some());
        assert!(idx.get("topics.pc.qos.durability").is_some());

        let span = idx.get("topics.pc.qos.reliability").unwrap();
        assert_eq!(&yaml[span.clone()], "reliability");
    }

    #[test]
    fn test_span_index_endpoint_qos_nested() {
        // Per-endpoint qos blocks under nodes.<n>.pub.<ep>.qos.<field>
        // and nodes.<n>.sub.<ep>.qos.<field>, plus per-sub
        // max_transport_ms, must be addressable for diagnostic spans.
        let yaml = r#"
nodes:
  perception:
    sub:
      pointcloud:
        qos:
          reliability: reliable
        max_transport_ms: 0
"#;
        let idx = SpanIndex::build(yaml);

        assert!(
            idx.get("nodes.perception.sub.pointcloud.qos").is_some(),
            "missing endpoint qos span"
        );
        assert!(
            idx.get("nodes.perception.sub.pointcloud.qos.reliability")
                .is_some(),
            "missing endpoint qos.reliability span"
        );
        assert!(
            idx.get("nodes.perception.sub.pointcloud.max_transport_ms")
                .is_some(),
            "missing per-sub max_transport_ms span"
        );

        let span = idx
            .get("nodes.perception.sub.pointcloud.qos.reliability")
            .unwrap();
        assert_eq!(&yaml[span.clone()], "reliability");
    }

    #[test]
    fn test_span_index_paths() {
        let yaml = "nodes:\n  a:\n    paths:\n      main:\n        max_latency_ms: 50\n";
        let idx = SpanIndex::build(yaml);

        assert!(idx.get("nodes.a.paths.main.max_latency_ms").is_some());
    }

    #[test]
    fn test_span_index_empty() {
        let idx = SpanIndex::build("");
        assert!(idx.spans.is_empty());
    }
}
