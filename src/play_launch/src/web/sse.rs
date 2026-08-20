//! Server-Sent Events (SSE) handlers for log streaming and state updates.

use super::WebState;
use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::{
        IntoResponse, Response,
        sse::{Event, Sse},
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

/// Maximum bytes to scan from the tail of a file for initial lines.
const MAX_TAIL_SCAN_BYTES: u64 = 2 * 1024 * 1024;

/// Maximum line length before truncation (8 KB).
const MAX_LINE_LEN: usize = 8 * 1024;

/// Maximum concurrent SSE subscribers per broadcast channel.
const MAX_SSE_SUBSCRIBERS: usize = 50;

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

    tail_lines_to_channel(&mut reader, path, &tx, "log").await
}

/// Tail an already-opened file, forwarding each new line as an SSE event.
///
/// Shared by the log and metrics streams, which had byte-identical copies of
/// this loop. That duplication was a hazard, not just noise: a fix applied to
/// one and not the other looks complete and is not.
///
/// TRUNCATION. Both copies seeked to the end and then only ever read forward,
/// so if the file shrank underneath them the reader sat past EOF and the
/// stream went silent for good — no error, no reconnect, just a UI that
/// stopped updating. That is what froze the Metrics tab when a respawn
/// re-created `metrics.csv`. The writer no longer truncates, but a reader
/// that cannot survive it is one `File::create` away from the same silence,
/// so detect it here too: on EOF, compare the file's length against our
/// offset, and if the file is now shorter, start again from the top.
///
/// Re-reading from the top re-sends the CSV header, which is correct rather
/// than merely tolerable — the client rebuilds its column map from it, so a
/// file replaced with a different schema is handled.
async fn tail_lines_to_channel(
    reader: &mut BufReader<std::fs::File>,
    path: &std::path::Path,
    tx: &mpsc::Sender<Result<Event, Infallible>>,
    what: &str,
) -> eyre::Result<()> {
    let mut pos = reader.seek(SeekFrom::End(0))?;
    let mut line_buf = String::new();

    loop {
        if tx.is_closed() {
            debug!("SSE channel closed, stopping {what} stream");
            return Ok(());
        }

        line_buf.clear();
        match reader.read_line(&mut line_buf) {
            Ok(0) => {
                // EOF. Before sleeping, check whether the file shrank — the
                // only signal available that someone truncated it, since a
                // truncating rewrite keeps the same inode.
                if let Ok(meta) = std::fs::metadata(path)
                    && meta.len() < pos
                {
                    warn!(
                        "{what} file {} shrank ({} -> {} bytes); re-reading from the start",
                        path.display(),
                        pos,
                        meta.len()
                    );
                    pos = reader.seek(SeekFrom::Start(0))?;
                    continue;
                }
                tokio::time::sleep(Duration::from_millis(POLL_INTERVAL_MS)).await;
            }
            Ok(n) => {
                pos += n as u64;
                let line = line_buf.trim_end();
                if !line.is_empty() {
                    let _ = tx
                        .send(Ok(Event::default().data(truncate_line(line))))
                        .await;
                }
                // Shrink buffer if an oversized line inflated it
                if line_buf.capacity() > MAX_LINE_LEN * 2 {
                    line_buf = String::new();
                }
            }
            Err(e) => {
                warn!("Error reading {what} file: {e}");
                tokio::time::sleep(Duration::from_millis(POLL_INTERVAL_MS)).await;
            }
        }
    }
}

/// Truncate a line to `MAX_LINE_LEN`, appending a size marker if truncated.
fn truncate_line(line: &str) -> String {
    if line.len() <= MAX_LINE_LEN {
        line.to_string()
    } else {
        let total = line.len();
        // Truncate at a char boundary
        let truncated = &line[..line.floor_char_boundary(MAX_LINE_LEN)];
        format!("{}... (truncated, {} bytes total)", truncated, total)
    }
}

/// Read the last N lines from the tail of a file.
///
/// Scans at most `MAX_TAIL_SCAN_BYTES` from the end. Lines exceeding
/// `MAX_LINE_LEN` are truncated with a byte-count marker.
fn read_last_n_lines(path: &PathBuf, n: usize) -> eyre::Result<Vec<String>> {
    let file = std::fs::File::open(path)?;
    let file_len = file.metadata()?.len();

    let start = file_len.saturating_sub(MAX_TAIL_SCAN_BYTES);
    let mut reader = BufReader::new(file);
    reader.seek(SeekFrom::Start(start))?;

    // If we seeked into the middle, skip the first partial line
    if start > 0 {
        let mut discard = String::new();
        reader.read_line(&mut discard)?;
    }

    let mut lines = Vec::new();
    let mut buf = String::new();
    loop {
        buf.clear();
        if reader.read_line(&mut buf)? == 0 {
            break;
        }
        let line = buf.trim_end();
        if !line.is_empty() {
            lines.push(truncate_line(line));
        }
    }

    let start = lines.len().saturating_sub(n);
    Ok(lines[start..].to_vec())
}

/// Stream state updates for all nodes (SSE)
///
/// This endpoint provides real-time state updates for all nodes via Server-Sent Events.
/// Each SSE message contains a JSON-serialized StateEvent.
///
/// If the client falls behind by more than the broadcast buffer capacity,
/// a `refresh` event is sent so the client can re-sync via `/api/nodes`.
pub async fn stream_state_updates(State(state): State<Arc<WebState>>) -> Response {
    if state.state_broadcaster.subscriber_count() >= MAX_SSE_SUBSCRIBERS {
        warn!(
            "SSE state subscriber limit reached ({})",
            MAX_SSE_SUBSCRIBERS
        );
        return (StatusCode::SERVICE_UNAVAILABLE, "Too many SSE connections").into_response();
    }

    debug!("New SSE client connected for state updates");

    let mut rx = state.state_broadcaster.subscribe();

    // Create the event stream with periodic heartbeat data events.
    // We send heartbeats as `data: keep-alive` (not SSE comments) because
    // JavaScript's EventSource.onmessage only fires for data events.
    let stream = async_stream::stream! {
        let mut heartbeat = tokio::time::interval(SSE_KEEPALIVE_INTERVAL);
        // First tick fires immediately — skip it since the client just connected.
        heartbeat.tick().await;

        loop {
            tokio::select! {
                result = rx.recv() => {
                    match result {
                        Ok(event) => {
                            match serde_json::to_string(&event) {
                                Ok(json) => {
                                    yield Ok::<_, Infallible>(Event::default().data(json));
                                }
                                Err(e) => {
                                    error!("Failed to serialize StateEvent: {}", e);
                                }
                            }
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                            warn!("SSE state client lagged by {} events, sending refresh", n);
                            yield Ok::<_, Infallible>(
                                Event::default()
                                    .event("refresh")
                                    .data(format!("{{\"lagged\":{n}}}"))
                            );
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
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

    if broadcaster.subscriber_count() >= MAX_SSE_SUBSCRIBERS {
        warn!(
            "SSE metrics subscriber limit reached ({})",
            MAX_SSE_SUBSCRIBERS
        );
        return (StatusCode::SERVICE_UNAVAILABLE, "Too many SSE connections").into_response();
    }

    debug!("New SSE client connected for system metrics");
    let mut rx = broadcaster.subscribe();

    let stream = async_stream::stream! {
        let mut heartbeat = tokio::time::interval(SSE_KEEPALIVE_INTERVAL);
        heartbeat.tick().await;

        loop {
            tokio::select! {
                result = rx.recv() => {
                    match result {
                        Ok(snap) => {
                            match serde_json::to_string(&snap) {
                                Ok(json) => {
                                    yield Ok::<_, Infallible>(Event::default().data(json));
                                }
                                Err(e) => {
                                    error!("Failed to serialize SystemStatsSnapshot: {}", e);
                                }
                            }
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                            warn!("SSE metrics client lagged by {} events", n);
                            // Metrics are stateless snapshots — just continue,
                            // the next recv() delivers the latest snapshot.
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
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

/// How often the diagnostics stream re-sends a snapshot in the absence of any
/// level change. See [`stream_diagnostics`].
const DIAGNOSTICS_TICK: Duration = Duration::from_secs(1);

/// Stream the full diagnostics table via SSE.
///
/// Push rather than poll: the view used to `setInterval(fetch, 5000)`, so an
/// ERROR could sit five seconds behind the system that raised it.
///
/// Each event is the whole table plus its counts, not a delta. The table is
/// tens of entries and the client re-renders all of it anyway, so a delta
/// protocol would buy nothing and add a resync path to get wrong.
///
/// Two things wake the stream, and both are needed:
///
/// - the registry's change notification, which fires the instant a diagnostic
///   changes level. This is the latency that matters.
/// - a 1 Hz tick, because staleness is evaluated when the table is read. A
///   publisher going silent produces no notification by construction — silence
///   is the signal — so without the tick a stale diagnostic would keep reading
///   OK until something unrelated changed.
pub async fn stream_diagnostics(State(state): State<Arc<WebState>>) -> Response {
    debug!("New SSE client connected for diagnostics");

    let registry = state.diagnostic_registry.clone();
    let mut changes = registry.subscribe();

    let stream = async_stream::stream! {
        let mut tick = tokio::time::interval(DIAGNOSTICS_TICK);

        loop {
            let payload = serde_json::json!({
                "diagnostics": registry.list_all(),
                "counts": registry.get_counts(),
            });

            match serde_json::to_string(&payload) {
                Ok(json) => yield Ok::<_, Infallible>(Event::default().data(json)),
                Err(e) => error!("Failed to serialize diagnostics snapshot: {}", e),
            }

            tokio::select! {
                // Lagged means several changes landed while we were busy. The
                // next snapshot already reflects all of them, so it is not an
                // error condition — only Closed ends the stream.
                result = changes.recv() => {
                    if matches!(result, Err(tokio::sync::broadcast::error::RecvError::Closed)) {
                        break;
                    }
                }
                _ = tick.tick() => {}
            }
        }
        debug!("SSE diagnostics stream ended");
    };

    Sse::new(stream)
        .keep_alive(
            axum::response::sse::KeepAlive::new()
                .interval(SSE_KEEPALIVE_INTERVAL)
                .text("keep-alive"),
        )
        .into_response()
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

    tail_lines_to_channel(&mut reader, path, &tx, "metrics").await
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    /// A tail must survive the file being truncated underneath it.
    ///
    /// Both tail loops seeked to the end and only read forward, so a shrinking
    /// file left the reader past EOF and the stream silent forever — the
    /// mechanism that froze the web UI's Metrics tab when a respawn re-created
    /// `metrics.csv`. Without the shrink check in `tail_lines_to_channel` this
    /// test times out with zero post-truncation lines.
    #[tokio::test]
    async fn a_truncated_file_is_re_read_from_the_start() {
        let tmp = tempfile::TempDir::new().unwrap();
        let path = tmp.path().join("metrics.csv");
        std::fs::write(&path, "header\nold-1\nold-2\n").unwrap();

        let (tx, mut rx) = mpsc::channel::<Result<Event, Infallible>>(100);
        let file = std::fs::File::open(&path).unwrap();
        let mut reader = BufReader::new(file);
        let p = path.clone();
        tokio::spawn(async move {
            let _ = tail_lines_to_channel(&mut reader, &p, &tx, "test").await;
        });

        // Appends arrive normally.
        tokio::time::sleep(Duration::from_millis(50)).await;
        {
            let mut f = std::fs::OpenOptions::new()
                .append(true)
                .open(&path)
                .unwrap();
            writeln!(f, "old-3").unwrap();
        }
        let before = collect_for(&mut rx, Duration::from_millis(1200)).await;
        assert!(
            before.iter().any(|l| l.contains("old-3")),
            "append before truncation should stream: {before:?}"
        );

        // Truncate and write fresh, SHORTER content — a respawn rewriting the file.
        std::fs::write(&path, "header\nnew-1\n").unwrap();

        let after = collect_for(&mut rx, Duration::from_millis(3000)).await;
        assert!(
            after.iter().any(|l| l.contains("new-1")),
            "post-truncation content must reach the client, got: {after:?}"
        );
    }

    /// Drain whatever arrives within `window`, returning the event payloads.
    async fn collect_for(
        rx: &mut mpsc::Receiver<Result<Event, Infallible>>,
        window: Duration,
    ) -> Vec<String> {
        let deadline = tokio::time::Instant::now() + window;
        let mut out = Vec::new();
        while let Ok(Some(Ok(ev))) = tokio::time::timeout_at(deadline, rx.recv()).await {
            out.push(format!("{ev:?}"));
        }
        out
    }
}
