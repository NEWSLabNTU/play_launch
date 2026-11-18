use super::diagnostic_data::DiagnosticStatus;
use chrono::{DateTime, Utc};
use csv::Writer;
use eyre::{Result, WrapErr};
use std::{fs::File, path::Path, sync::Mutex};

/// CSV writer for diagnostic data
pub struct DiagnosticCsvWriter {
    writer: Mutex<Writer<File>>,
    /// Write a row per key/value (`diagnostics.record: all`); otherwise one
    /// row per status with `key` and `value` empty.
    values: bool,
}

impl DiagnosticCsvWriter {
    /// Create a new CSV writer for diagnostics
    pub fn new(log_dir: &Path, values: bool) -> Result<Self> {
        let csv_path = log_dir.join("diagnostics.csv");

        let file = File::create(&csv_path)
            .with_context(|| format!("Failed to create diagnostics CSV at {:?}", csv_path))?;

        let mut writer = Writer::from_writer(file);

        // Write header
        writer
            .write_record([
                "timestamp",
                "hardware_id",
                "diagnostic_name",
                "level",
                "level_name",
                "message",
                "key",
                "value",
            ])
            .wrap_err("Failed to write CSV header")?;

        writer.flush().wrap_err("Failed to flush CSV header")?;

        Ok(Self {
            writer: Mutex::new(writer),
            values,
        })
    }

    /// Write a diagnostic status to CSV
    /// Each key-value pair gets its own row for normalized format.
    ///
    /// Buffered: rows reach the file on [`Self::flush`], which the monitor
    /// calls once a second and at shutdown.
    pub fn write_diagnostic(&self, status: &DiagnosticStatus) -> Result<()> {
        let mut writer = self
            .writer
            .lock()
            .map_err(|_| eyre::eyre!("diagnostics CSV writer lock poisoned"))?;

        let timestamp: DateTime<Utc> = status.timestamp.into();
        let timestamp_str = timestamp.to_rfc3339();

        if status.values.is_empty() || !self.values {
            // Write single row with no key-value data
            writer
                .write_record([
                    &timestamp_str,
                    &status.hardware_id,
                    &status.name,
                    &(status.level as u8).to_string(),
                    status.level.as_str(),
                    &status.message,
                    "",
                    "",
                ])
                .wrap_err("Failed to write diagnostic CSV record")?;
        } else {
            // Write one row per key-value pair
            for (key, value) in &status.values {
                writer
                    .write_record([
                        &timestamp_str,
                        &status.hardware_id,
                        &status.name,
                        &(status.level as u8).to_string(),
                        status.level.as_str(),
                        &status.message,
                        key,
                        value,
                    ])
                    .wrap_err("Failed to write diagnostic CSV record")?;
            }
        }

        Ok(())
    }

    /// Push buffered rows to the file.
    pub fn flush(&self) -> Result<()> {
        let mut writer = self
            .writer
            .lock()
            .map_err(|_| eyre::eyre!("diagnostics CSV writer lock poisoned"))?;
        writer.flush().wrap_err("Failed to flush CSV writer")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::diagnostics::diagnostic_data::DiagnosticLevel;

    fn rows(values: bool) -> usize {
        let dir = tempfile::tempdir().unwrap();
        let w = DiagnosticCsvWriter::new(dir.path(), values).unwrap();
        w.write_diagnostic(&DiagnosticStatus {
            hardware_id: "h".into(),
            name: "n".into(),
            level: DiagnosticLevel::Warning,
            message: "m".into(),
            values: [("a", "1"), ("b", "2"), ("c", "3")]
                .into_iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
            timestamp: std::time::SystemTime::now(),
        })
        .unwrap();
        w.flush().unwrap();
        let body = std::fs::read_to_string(dir.path().join("diagnostics.csv")).unwrap();
        body.lines().count() - 1
    }

    /// #0098: `diagnostics.record: all` writes a row per key/value; the
    /// default (`changes`) writes one row per status.
    #[test]
    fn values_are_rows_only_when_asked_for() {
        assert_eq!(rows(true), 3);
        assert_eq!(rows(false), 1);
    }
}
