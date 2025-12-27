//! Timestamp formatting utilities

/// Format the current time as a timestamp string
///
/// Format: YYYY-MM-DD_HH-MM-SS
pub fn format_timestamp() -> String {
    chrono::Local::now().format("%Y-%m-%d_%H-%M-%S").to_string()
}
