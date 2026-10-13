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
use tracing::{debug, error, info, trace};

/// How often buffered `diagnostics.csv` rows reach the file. A killed run
/// loses at most this much.
const CSV_FLUSH_INTERVAL: Duration = Duration::from_secs(1);

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
                // trace, not debug: one line per message, and the run log
                // records debug. On Autoware (~250 msg/s) the two per-message
                // lines here were 44k lines in 100 s, 270 MB an hour, each
                // formatted and written on the shared rclrs executor thread.
                trace!(
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

    // Rate limiting: track last write time per diagnostic key, and the level
    // last recorded for it. Level transitions bypass the debounce entirely —
    // see the comment at the check below.
    let mut last_write_times: HashMap<String, SystemTime> = HashMap::new();
    let mut last_levels: HashMap<String, DiagnosticLevel> = HashMap::new();
    let debounce_duration = Duration::from_millis(config.debounce_ms);

    // The CSV is flushed once a second rather than once per status: at
    // Autoware's ~250 msg/s that was a write syscall per status.
    let mut flush_tick = tokio::time::interval(CSV_FLUSH_INTERVAL);

    // Process messages until shutdown
    loop {
        tokio::select! {
            Some(msg) = rx.recv() => {
                // Self-cost. Nothing below awaits.
                let _cost = crate::util::self_cost::section(
                    crate::util::self_cost::Subsystem::Diagnostics,
                );
                // Process each diagnostic status in the array. The message is
                // owned, so its strings move into the status rather than being
                // cloned.
                for status_msg in msg.status {
                    // Apply filter if configured. Before anything else, so a
                    // filtered-out diagnostic costs nothing and cannot leave
                    // bookkeeping behind for a key that is never recorded.
                    if !config.filter_hardware_ids.is_empty()
                        && !config.filter_hardware_ids.contains(&status_msg.hardware_id)
                    {
                        continue;
                    }

                    let diagnostic_status = DiagnosticStatus {
                        hardware_id: status_msg.hardware_id,
                        name: status_msg.name,
                        level: DiagnosticLevel::from_u8(status_msg.level),
                        message: status_msg.message,
                        values: status_msg
                            .values
                            .into_iter()
                            .map(|kv| (kv.key, kv.value))
                            .collect(),
                        timestamp: SystemTime::now(),
                    };

                    let key = diagnostic_status.key();

                    // A level transition is never rate limited.
                    //
                    // The debounce exists to stop a chatty publisher filling
                    // the CSV with identical rows, and for that it is right.
                    // Applied to transitions it destroys the only rows that
                    // matter: Autoware publishes diagnostics at 10 Hz, which is
                    // exactly the 100 ms default, so a leaf going OK -> ERROR
                    // -> OK inside one window vanished from both the registry
                    // and the CSV. A fault that clears before the next window
                    // is precisely the one worth catching.
                    let level_changed = last_levels.get(&key) != Some(&diagnostic_status.level);
                    let debounce_elapsed = match last_write_times.get(&key) {
                        Some(&last_time) => {
                            diagnostic_status
                                .timestamp
                                .duration_since(last_time)
                                .unwrap_or(Duration::ZERO)
                                >= debounce_duration
                        }
                        // First time seeing this diagnostic
                        None => true,
                    };

                    if level_changed || debounce_elapsed {
                        // Update registry
                        registry.update(diagnostic_status.clone());

                        // Write to CSV (buffered; flushed on the tick below)
                        if let Err(e) = csv_writer.write_diagnostic(&diagnostic_status) {
                            error!("Failed to write diagnostic to CSV: {:#}", e);
                        }

                        // Update last write time
                        last_write_times.insert(key.clone(), diagnostic_status.timestamp);
                        last_levels.insert(key, diagnostic_status.level);

                        trace!(
                            "Logged diagnostic: {}/{} [{}]",
                            diagnostic_status.hardware_id,
                            diagnostic_status.name,
                            diagnostic_status.level.as_str()
                        );
                    }
                }
            }
            _ = flush_tick.tick() => {
                if let Err(e) = csv_writer.flush() {
                    error!("Failed to flush diagnostics CSV: {:#}", e);
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

    if let Err(e) = csv_writer.flush() {
        error!("Failed to flush diagnostics CSV: {:#}", e);
    }
    info!("Diagnostic monitoring task stopped");
    Ok(())
}
