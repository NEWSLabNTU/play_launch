use super::diagnostic_data::DiagnosticStatus;
use chrono::{DateTime, Utc};
use csv::Writer;
use eyre::{Result, WrapErr};
use std::{fs::File, path::PathBuf};
use tokio::sync::Mutex;

/// CSV writer for diagnostic data
pub struct DiagnosticCsvWriter {
    writer: Mutex<Writer<File>>,
}

impl DiagnosticCsvWriter {
    /// Create a new CSV writer for diagnostics
    pub fn new(log_dir: &PathBuf) -> Result<Self> {
        let csv_path = log_dir.join("diagnostics.csv");

        let file = File::create(&csv_path)
            .with_context(|| format!("Failed to create diagnostics CSV at {:?}", csv_path))?;

        let mut writer = Writer::from_writer(file);

        // Write header
        writer
            .write_record(&[
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
        })
    }

    /// Write a diagnostic status to CSV
    /// Each key-value pair gets its own row for normalized format
    pub async fn write_diagnostic(&self, status: &DiagnosticStatus) -> Result<()> {
        let mut writer = self.writer.lock().await;

        let timestamp: DateTime<Utc> = status.timestamp.into();
        let timestamp_str = timestamp.to_rfc3339();

        if status.values.is_empty() {
            // Write single row with no key-value data
            writer
                .write_record(&[
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
                    .write_record(&[
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

        writer.flush().wrap_err("Failed to flush CSV writer")?;

        Ok(())
    }
}
