use std::fmt;
use std::path::{Path, PathBuf};

use crate::fixtures;

/// A node that exited unexpectedly during the smoke test.
pub struct NodeExit {
    pub name: String,
    pub detail: String,
    pub log_line: String,
}

/// A composable node that failed to load into its container.
pub struct LoadNodeFailure {
    pub container: String,
    pub component: String,
    pub error: String,
    pub log_line: String,
}

/// Aggregated health report from a play_launch run.
pub struct HealthReport {
    pub processes_expected: usize,
    pub processes_actual: usize,
    pub node_exits: Vec<NodeExit>,
    pub load_node_failures: Vec<LoadNodeFailure>,
    pub play_log: PathBuf,
}

impl HealthReport {
    /// Analyze play_launch output and play_log directory to build a health
    /// report.
    ///
    /// - `play_log`: path to `play_log/latest`
    /// - `output_path`: path to the captured stdout file (play_launch writes
    ///   tracing output to stdout)
    /// - `expected_procs`: expected number of processes (nodes + containers)
    pub fn analyze(
        play_log: &Path,
        output_path: &Path,
        expected_procs: usize,
    ) -> Self {
        let processes_actual = fixtures::count_cmdline_files(play_log);

        let raw_content = std::fs::read_to_string(output_path)
            .unwrap_or_default();
        let content = strip_ansi_escapes(&raw_content);

        let mut node_exits = Vec::new();
        let mut seen_exits = std::collections::HashSet::new();
        let mut load_node_failures = Vec::new();
        let mut seen_failures = std::collections::HashSet::new();

        for line in content.lines() {
            // Match: ERROR ... [node_name] Exited without code
            // Match: ERROR ... [node_name] Exited with code N
            if line.contains("ERROR") && line.contains("Exited") {
                if let Some(exit) = parse_node_exit(line) {
                    if seen_exits.insert(exit.name.clone()) {
                        node_exits.push(exit);
                    }
                }
            }

            // Match: WARN ... container_name: LoadNode FAILED for component_name: error='...'
            if line.contains("LoadNode FAILED") {
                if let Some(failure) = parse_load_node_failure(line) {
                    let key = format!("{}/{}", failure.container, failure.component);
                    if seen_failures.insert(key) {
                        load_node_failures.push(failure);
                    }
                }
            }
        }

        HealthReport {
            processes_expected: expected_procs,
            processes_actual,
            node_exits,
            load_node_failures,
            play_log: play_log.to_path_buf(),
        }
    }

    /// Returns true if no errors were detected (ignoring known environment issues).
    ///
    /// `ignored_exits` is a list of node names to skip (e.g., nodes that require
    /// hardware like TensorRT or a display server).
    pub fn is_healthy(&self, ignored_exits: &[&str]) -> bool {
        let unexpected_exits = self
            .node_exits
            .iter()
            .filter(|e| !ignored_exits.contains(&e.name.as_str()))
            .count();
        unexpected_exits == 0
            && self.load_node_failures.is_empty()
            && self.processes_actual == self.processes_expected
    }
}

/// Strip ANSI escape sequences from a string.
fn strip_ansi_escapes(s: &str) -> String {
    let mut result = String::with_capacity(s.len());
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c == '\x1b' {
            // Skip ESC + '[' + parameters + final byte
            if let Some('[') = chars.next() {
                // Consume until we hit a letter (the final byte of the sequence)
                for c2 in chars.by_ref() {
                    if c2.is_ascii_alphabetic() {
                        break;
                    }
                }
            }
        } else {
            result.push(c);
        }
    }
    result
}

/// Parse a node exit line like:
///   `2026-02-08T01:45:56 ERROR play_launch::... [shape_estimation] Exited without code`
///   `2026-02-08T01:45:56 ERROR play_launch::... [shape_estimation] Exited with code 1`
fn parse_node_exit(line: &str) -> Option<NodeExit> {
    // Find the node name in brackets: [node_name]
    let bracket_start = line.find('[')?;
    let bracket_end = line[bracket_start..].find(']')? + bracket_start;
    let name = line[bracket_start + 1..bracket_end].to_string();

    // Extract the detail (everything after the closing bracket + space)
    let detail_start = bracket_end + 2; // skip "] "
    let detail = if detail_start < line.len() {
        line[detail_start..].trim().to_string()
    } else {
        "Exited".to_string()
    };

    Some(NodeExit {
        name,
        detail,
        log_line: line.to_string(),
    })
}

/// Parse a LoadNode failure line like:
///   `2026-02-08T01:46:03 WARN play_launch::... pointcloud_container: LoadNode FAILED for occupancy_grid_map_node: error='...'`
fn parse_load_node_failure(line: &str) -> Option<LoadNodeFailure> {
    // Find "LoadNode FAILED for "
    let marker = "LoadNode FAILED for ";
    let marker_pos = line.find(marker)?;

    // Container name: word before ": LoadNode FAILED"
    // Walk backwards from marker_pos to find ": " preceding the container name
    let before_marker = &line[..marker_pos];
    let colon_pos = before_marker.rfind(": ")?;
    // The container name is the word before the colon
    let before_colon = before_marker[..colon_pos].trim();
    let container = before_colon
        .rsplit_once(|c: char| c.is_whitespace())
        .map(|(_, name)| name)
        .unwrap_or(before_colon)
        .to_string();

    // Component name: between "FAILED for " and ": error="
    let after_marker = &line[marker_pos + marker.len()..];
    let (component, error) = if let Some(error_pos) = after_marker.find(": error='") {
        let component = after_marker[..error_pos].to_string();
        // Extract error message between error=' and trailing '
        let error_start = error_pos + ": error='".len();
        let error_msg = after_marker[error_start..]
            .trim_end_matches('\'')
            .to_string();
        (component, error_msg)
    } else if let Some(error_pos) = after_marker.find(": error=") {
        let component = after_marker[..error_pos].to_string();
        let error_start = error_pos + ": error=".len();
        let error_msg = after_marker[error_start..].to_string();
        (component, error_msg)
    } else {
        // No error= found, take everything as component
        (after_marker.trim().to_string(), String::new())
    };

    Some(LoadNodeFailure {
        container,
        component,
        error,
        log_line: line.to_string(),
    })
}

/// Read the last N lines of a file, returning them as a single string.
fn tail_lines(path: &Path, n: usize) -> String {
    match std::fs::read_to_string(path) {
        Ok(content) => {
            let lines: Vec<&str> = content.lines().collect();
            let start = lines.len().saturating_sub(n);
            lines[start..].join("\n")
        }
        Err(_) => "(file not found)".to_string(),
    }
}

impl fmt::Display for HealthReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "=== Autoware Smoke Test Report ===")?;
        writeln!(f)?;
        writeln!(
            f,
            "Processes: {}/{}",
            self.processes_actual, self.processes_expected
        )?;

        if !self.node_exits.is_empty() {
            writeln!(f)?;
            writeln!(f, "--- Node Exits ({}) ---", self.node_exits.len())?;
            for exit in &self.node_exits {
                writeln!(f)?;
                writeln!(f, "  {}: {}", exit.name, exit.detail)?;
                writeln!(f, "    Log: {}", exit.log_line)?;

                // Show last 10 lines of the node's stderr
                let err_path = self.play_log.join(format!("node/{}/err", exit.name));
                let stderr_tail = tail_lines(&err_path, 10);
                if stderr_tail != "(file not found)" && !stderr_tail.is_empty() {
                    writeln!(
                        f,
                        "    Stderr ({}, last 10 lines):",
                        err_path.display()
                    )?;
                    for line in stderr_tail.lines() {
                        writeln!(f, "      {}", line)?;
                    }
                }
            }
        }

        if !self.load_node_failures.is_empty() {
            writeln!(f)?;
            writeln!(
                f,
                "--- LoadNode Failures ({}) ---",
                self.load_node_failures.len()
            )?;
            for failure in &self.load_node_failures {
                writeln!(f)?;
                writeln!(
                    f,
                    "  {} / {}:",
                    failure.container, failure.component
                )?;
                if !failure.error.is_empty() {
                    writeln!(f, "    {}", failure.error)?;
                }
                writeln!(f, "    Log: {}", failure.log_line)?;
            }
        }

        writeln!(f)?;
        if self.is_healthy(&[]) {
            writeln!(f, "RESULT: PASS")?;
        } else {
            let mut parts = Vec::new();
            if !self.node_exits.is_empty() {
                parts.push(format!("{} node exits", self.node_exits.len()));
            }
            if !self.load_node_failures.is_empty() {
                parts.push(format!(
                    "{} load_node failures",
                    self.load_node_failures.len()
                ));
            }
            if self.processes_actual != self.processes_expected {
                parts.push(format!(
                    "process count {}/{}",
                    self.processes_actual, self.processes_expected
                ));
            }
            writeln!(f, "RESULT: FAIL ({})", parts.join(", "))?;
        }

        Ok(())
    }
}
