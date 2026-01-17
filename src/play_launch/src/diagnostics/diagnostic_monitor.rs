use super::{
    diagnostic_data::{DiagnosticLevel, DiagnosticRegistry, DiagnosticStatus},
    storage::DiagnosticCsvWriter,
};
use crate::cli::config::DiagnosticsSettings;
use eyre::{Result, WrapErr};
use std::{
    collections::HashMap,
    path::PathBuf,
    sync::Arc,
    time::{Duration, SystemTime},
};
use tokio::sync::watch;
use tracing::{debug, error, info};

/// Main diagnostic monitoring task
pub async fn run_diagnostic_task(
    config: DiagnosticsSettings,
    shared_ros_node: Arc<rclrs::Node>,
    log_dir: PathBuf,
    registry: Arc<DiagnosticRegistry>,
    mut shutdown_signal: watch::Receiver<bool>,
) -> Result<()> {
    info!("Starting diagnostic monitoring task");
    debug!(
        "Diagnostic settings: topics={:?}, debounce_ms={}",
        config.topics, config.debounce_ms
    );

    // Create CSV writer
    let csv_writer =
        DiagnosticCsvWriter::new(&log_dir).wrap_err("Failed to create diagnostic CSV writer")?;

    // Create channel for receiving diagnostic messages from ROS callback
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();

    // Create subscriptions for each topic
    let mut subscriptions = Vec::new();

    for topic in &config.topics {
        debug!("Creating subscription for topic: {}", topic);

        // Clone tx for this subscription
        let tx_clone = tx.clone();
        let topic_name = topic.clone();

        // Create subscription
        // Note: We use a closure that sends messages to the async task via channel
        let _subscription = shared_ros_node
            .create_subscription(topic, move |msg: diagnostic_msgs::msg::DiagnosticArray| {
                debug!(
                    "Received DiagnosticArray on topic '{}' with {} statuses",
                    topic_name,
                    msg.status.len()
                );

                // Send to async processing task
                if let Err(e) = tx_clone.send(msg) {
                    error!(
                        "Failed to send diagnostic message to processing task: {:#}",
                        e
                    );
                }
            })
            .wrap_err_with(|| format!("Failed to create subscription for topic '{}'", topic))?;

        // Keep subscription alive
        subscriptions.push(_subscription);
    }

    info!(
        "Diagnostic monitoring active on {} topic(s)",
        config.topics.len()
    );

    // Debounce mechanism: track last write time per diagnostic key
    let mut last_write_times: HashMap<String, SystemTime> = HashMap::new();
    let debounce_duration = Duration::from_millis(config.debounce_ms);

    // Process messages until shutdown
    loop {
        tokio::select! {
            Some(msg) = rx.recv() => {
                // Process each diagnostic status in the array
                for status_msg in msg.status {
                    // Convert ROS message to our DiagnosticStatus
                    let diagnostic_status = DiagnosticStatus {
                        hardware_id: status_msg.hardware_id.clone(),
                        name: status_msg.name.clone(),
                        level: DiagnosticLevel::from_u8(status_msg.level),
                        message: status_msg.message.clone(),
                        values: status_msg
                            .values
                            .iter()
                            .map(|kv| (kv.key.clone(), kv.value.clone()))
                            .collect(),
                        timestamp: SystemTime::now(),
                    };

                    let key = diagnostic_status.key();

                    // Check debounce
                    let should_write = if let Some(&last_time) = last_write_times.get(&key) {
                        diagnostic_status.timestamp.duration_since(last_time).unwrap_or(Duration::ZERO) >= debounce_duration
                    } else {
                        true // First time seeing this diagnostic
                    };

                    if should_write {
                        // Apply filter if configured
                        if !config.filter_hardware_ids.is_empty()
                            && !config.filter_hardware_ids.contains(&diagnostic_status.hardware_id)
                        {
                            continue;
                        }

                        // Update registry
                        registry.update(diagnostic_status.clone()).await;

                        // Write to CSV
                        if let Err(e) = csv_writer.write_diagnostic(&diagnostic_status).await {
                            error!("Failed to write diagnostic to CSV: {:#}", e);
                        }

                        // Update last write time
                        last_write_times.insert(key, diagnostic_status.timestamp);

                        debug!(
                            "Logged diagnostic: {}/{} [{}]",
                            diagnostic_status.hardware_id,
                            diagnostic_status.name,
                            diagnostic_status.level.as_str()
                        );
                    }
                }
            }
            _ = shutdown_signal.changed() => {
                if *shutdown_signal.borrow() {
                    info!("Diagnostic monitoring task received shutdown signal");
                    break;
                }
            }
        }
    }

    info!("Diagnostic monitoring task stopped");
    Ok(())
}
