//! Server-Sent Events (SSE) handlers for log streaming and state updates.

use super::WebState;
use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::{
        sse::{Event, Sse},
        IntoResponse, Response,
    },
};
use futures::stream::Stream;
use std::{
    convert::Infallible,
    io::{BufRead, BufReader, Seek, SeekFrom},
    path::PathBuf,
    sync::Arc,
    time::Duration,
};
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tracing::{debug, error, warn};

/// Number of lines to send on initial connection
const INITIAL_LINES: usize = 100;

/// Poll interval for file changes (milliseconds)
const POLL_INTERVAL_MS: u64 = 500;

/// SSE keepalive/heartbeat interval
const SSE_KEEPALIVE_INTERVAL: Duration = Duration::from_secs(3);

/// Stream stdout logs for a node
pub async fn stream_stdout(
    State(state): State<Arc<WebState>>,
    Path(name): Path<String>,
) -> Response {
    stream_log_file(state, &name, "out").await
}

/// Stream stderr logs for a node
pub async fn stream_stderr(
    State(state): State<Arc<WebState>>,
    Path(name): Path<String>,
) -> Response {
    stream_log_file(state, &name, "err").await
}

/// Stream a log file as SSE
async fn stream_log_file(state: Arc<WebState>, node_name: &str, file_name: &str) -> Response {
    // Get the log file path from the member handle
    let log_path = {
        let member_handle = &state.member_handle;
        match member_handle.get_member_state(node_name).await {
            Some(member) => {
                let base_path = member.output_dir;
                if file_name == "out" {
                    base_path.join("out")
                } else {
                    base_path.join("err")
                }
            }
            None => {
                return (
                    StatusCode::NOT_FOUND,
                    format!("Node '{}' not found", node_name),
                )
                    .into_response();
            }
        }
    };

    // Create the SSE stream
    let stream = create_file_stream(log_path);

    Sse::new(stream)
        .keep_alive(
            axum::response::sse::KeepAlive::new()
                .interval(SSE_KEEPALIVE_INTERVAL)
                .text("keep-alive"),
        )
        .into_response()
}

/// Create a stream that tails a file
fn create_file_stream(
    path: PathBuf,
) -> impl Stream<Item = Result<Event, Infallible>> + Send + 'static {
    let (tx, rx) = mpsc::channel::<Result<Event, Infallible>>(100);

    // Spawn a task to read the file and send events
    tokio::spawn(async move {
        if let Err(e) = stream_file_to_channel(&path, tx.clone()).await {
            error!("Error streaming file {:?}: {}", path, e);
            let _ = tx
                .send(Ok(Event::default().data(format!("Error: {}", e))))
                .await;
        }
    });

    ReceiverStream::new(rx)
}

/// Stream file contents to a channel
async fn stream_file_to_channel(
    path: &PathBuf,
    tx: mpsc::Sender<Result<Event, Infallible>>,
) -> eyre::Result<()> {
    // Send immediate connection confirmation to prevent "Connecting..." delay
    let _ = tx.send(Ok(Event::default().comment("connected"))).await;

    // Wait for file to exist
    let mut attempts = 0;
    while !path.exists() && attempts < 10 {
        tokio::time::sleep(Duration::from_millis(500)).await;
        attempts += 1;
    }

    if !path.exists() {
        let _ = tx
            .send(Ok(Event::default().data("Log file not yet created")))
            .await;

        // Keep waiting for file to appear
        loop {
            tokio::time::sleep(Duration::from_secs(1)).await;
            if path.exists() {
                break;
            }
            if tx.is_closed() {
                return Ok(());
            }
        }
    }

    // Open the file
    let file = std::fs::File::open(path)?;
    let mut reader = BufReader::new(file);

    // Send initial lines (last N lines)
    let initial_lines = read_last_n_lines(path, INITIAL_LINES)?;
    for line in initial_lines {
        if tx.is_closed() {
            return Ok(());
        }
        let _ = tx.send(Ok(Event::default().data(line))).await;
    }

    // Seek to end of file
    reader.seek(SeekFrom::End(0))?;

    // Poll for new content
    let mut line_buf = String::new();
    loop {
        if tx.is_closed() {
            debug!("SSE channel closed, stopping file stream");
            return Ok(());
        }

        line_buf.clear();
        match reader.read_line(&mut line_buf) {
            Ok(0) => {
                // No new data, wait and try again
                tokio::time::sleep(Duration::from_millis(POLL_INTERVAL_MS)).await;
            }
            Ok(_) => {
                let line = line_buf.trim_end().to_string();
                if !line.is_empty() {
                    let _ = tx.send(Ok(Event::default().data(line))).await;
                }
            }
            Err(e) => {
                warn!("Error reading log file: {}", e);
                tokio::time::sleep(Duration::from_millis(POLL_INTERVAL_MS)).await;
            }
        }
    }
}

/// Read the last N lines from a file
fn read_last_n_lines(path: &PathBuf, n: usize) -> eyre::Result<Vec<String>> {
    let file = std::fs::File::open(path)?;
    let reader = BufReader::new(file);

    let lines: Vec<String> = reader.lines().map_while(Result::ok).collect();

    let start = if lines.len() > n { lines.len() - n } else { 0 };
    Ok(lines[start..].to_vec())
}

/// Stream state updates for all nodes (SSE)
///
/// This endpoint provides real-time state updates for all nodes via Server-Sent Events.
/// Each SSE message contains a JSON-serialized StateEvent.
pub async fn stream_state_updates(State(state): State<Arc<WebState>>) -> Response {
    debug!("New SSE client connected for state updates");

    // Subscribe to state events
    let mut rx = state.state_broadcaster.subscribe().await;

    // Create the event stream with periodic heartbeat data events.
    // We send heartbeats as `data: keep-alive` (not SSE comments) because
    // JavaScript's EventSource.onmessage only fires for data events.
    let stream = async_stream::stream! {
        let mut heartbeat = tokio::time::interval(SSE_KEEPALIVE_INTERVAL);
        // First tick fires immediately — skip it since the client just connected.
        heartbeat.tick().await;

        loop {
            tokio::select! {
                event = rx.recv() => {
                    match event {
                        Some(event) => {
                            match serde_json::to_string(&event) {
                                Ok(json) => {
                                    yield Ok::<_, Infallible>(Event::default().data(json));
                                }
                                Err(e) => {
                                    error!("Failed to serialize StateEvent: {}", e);
                                }
                            }
                        }
                        None => break, // channel closed
                    }
                }
                _ = heartbeat.tick() => {
                    yield Ok::<_, Infallible>(Event::default().data("keep-alive"));
                }
            }
        }
        debug!("SSE state updates stream ended");
    };

    Sse::new(stream).into_response()
}

/// Stream system-wide metrics (CPU, memory, network, disk) via SSE.
///
/// Subscribes to the `SystemMetricsBroadcaster` and forwards each snapshot
/// as a JSON SSE event. Returns 503 if monitoring is disabled.
pub async fn stream_system_metrics(State(state): State<Arc<WebState>>) -> Response {
    let broadcaster = match &state.metrics_broadcaster {
        Some(b) => b.clone(),
        None => {
            return (StatusCode::SERVICE_UNAVAILABLE, "Monitoring is disabled").into_response();
        }
    };

    debug!("New SSE client connected for system metrics");
    let mut rx = broadcaster.subscribe().await;

    let stream = async_stream::stream! {
        let mut heartbeat = tokio::time::interval(SSE_KEEPALIVE_INTERVAL);
        heartbeat.tick().await;

        loop {
            tokio::select! {
                snapshot = rx.recv() => {
                    match snapshot {
                        Some(snap) => {
                            match serde_json::to_string(&snap) {
                                Ok(json) => {
                                    yield Ok::<_, Infallible>(Event::default().data(json));
                                }
                                Err(e) => {
                                    error!("Failed to serialize SystemStatsSnapshot: {}", e);
                                }
                            }
                        }
                        None => break,
                    }
                }
                _ = heartbeat.tick() => {
                    yield Ok::<_, Infallible>(Event::default().data("keep-alive"));
                }
            }
        }
        debug!("SSE system metrics stream ended");
    };

    Sse::new(stream).into_response()
}

/// Number of initial CSV lines to send for node metrics (2min of history at 2s interval)
const INITIAL_METRICS_LINES: usize = 60;

/// Stream per-node metrics CSV via SSE.
///
/// Tails the node's `metrics.csv` file, sending the last 60 lines on connect
/// then streaming new lines as they appear. Returns 404 if the node is unknown.
pub async fn stream_node_metrics(
    State(state): State<Arc<WebState>>,
    Path(name): Path<String>,
) -> Response {
    let metrics_path = {
        match state.member_handle.get_member_state(&name).await {
            Some(member) => member.output_dir.join("metrics.csv"),
            None => {
                return (StatusCode::NOT_FOUND, format!("Node '{}' not found", name))
                    .into_response();
            }
        }
    };

    let stream = create_file_stream_with_initial(metrics_path, INITIAL_METRICS_LINES);

    Sse::new(stream)
        .keep_alive(
            axum::response::sse::KeepAlive::new()
                .interval(SSE_KEEPALIVE_INTERVAL)
                .text("keep-alive"),
        )
        .into_response()
}

/// Create a file-tailing stream with a configurable number of initial lines.
fn create_file_stream_with_initial(
    path: PathBuf,
    initial_lines: usize,
) -> impl Stream<Item = Result<Event, Infallible>> + Send + 'static {
    let (tx, rx) = mpsc::channel::<Result<Event, Infallible>>(100);

    tokio::spawn(async move {
        if let Err(e) = stream_file_to_channel_with_initial(&path, tx.clone(), initial_lines).await
        {
            error!("Error streaming file {:?}: {}", path, e);
            let _ = tx
                .send(Ok(Event::default().data(format!("Error: {}", e))))
                .await;
        }
    });

    ReceiverStream::new(rx)
}

/// Stream file contents to a channel with a configurable initial line count.
async fn stream_file_to_channel_with_initial(
    path: &PathBuf,
    tx: mpsc::Sender<Result<Event, Infallible>>,
    initial_lines: usize,
) -> eyre::Result<()> {
    let _ = tx.send(Ok(Event::default().comment("connected"))).await;

    // Wait for file to exist
    let mut attempts = 0;
    while !path.exists() && attempts < 10 {
        tokio::time::sleep(Duration::from_millis(500)).await;
        attempts += 1;
    }

    if !path.exists() {
        let _ = tx
            .send(Ok(Event::default().data("Metrics file not yet created")))
            .await;

        loop {
            tokio::time::sleep(Duration::from_secs(1)).await;
            if path.exists() {
                break;
            }
            if tx.is_closed() {
                return Ok(());
            }
        }
    }

    let file = std::fs::File::open(path)?;
    let mut reader = BufReader::new(file);

    // Send initial lines (last N lines)
    let last_lines = read_last_n_lines(path, initial_lines)?;
    for line in last_lines {
        if tx.is_closed() {
            return Ok(());
        }
        let _ = tx.send(Ok(Event::default().data(line))).await;
    }

    // Seek to end and poll for new content
    reader.seek(SeekFrom::End(0))?;
    let mut line_buf = String::new();

    loop {
        if tx.is_closed() {
            debug!("SSE channel closed, stopping metrics file stream");
            return Ok(());
        }

        line_buf.clear();
        match reader.read_line(&mut line_buf) {
            Ok(0) => {
                tokio::time::sleep(Duration::from_millis(POLL_INTERVAL_MS)).await;
            }
            Ok(_) => {
                let line = line_buf.trim_end().to_string();
                if !line.is_empty() {
                    let _ = tx.send(Ok(Event::default().data(line))).await;
                }
            }
            Err(e) => {
                warn!("Error reading metrics file: {}", e);
                tokio::time::sleep(Duration::from_millis(POLL_INTERVAL_MS)).await;
            }
        }
    }
}
