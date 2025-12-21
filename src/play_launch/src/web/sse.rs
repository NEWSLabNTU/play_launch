//! Server-Sent Events (SSE) handlers for log streaming.

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
    // Get the log file path from the registry
    let log_path = {
        let registry = state.registry.lock().await;
        match registry.get(node_name) {
            Some(handle) => {
                if file_name == "out" {
                    handle.log_paths.stdout.clone()
                } else {
                    handle.log_paths.stderr.clone()
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
                .interval(Duration::from_secs(15))
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
    let _ = tx
        .send(Ok(Event::default().comment("connected")))
        .await;

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
