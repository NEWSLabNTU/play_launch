//! Privileged RT scheduling helper daemon for play_launch
//!
//! This daemon runs with CAP_SYS_NICE capability so it can apply Linux
//! real-time scheduling policy/priority/affinity to processes on behalf of
//! the unprivileged main `play_launch` process, which spawns nodes without
//! elevated capabilities.
//!
//! Communication with play_launch happens over length-prefixed bincode
//! messages on a pair of pipe file descriptors — same framing as
//! `play_launch_io_helper`, but a wire-independent protocol
//! (`ipc::sched_protocol`), since these are two independently-capped
//! binaries with nothing to do with each other.
//!
//! Deliberately minimal: only `play_launch::{ipc, sched}` plus
//! tokio/libc/eyre/tracing. No clap, no ROS, nothing from the main binary's
//! `execution`/`ros` module trees.

use eyre::{Context, Result};
use play_launch::ipc::{SchedRequest, SchedResponse, decode_message, encode_message};
use play_launch::sched::apply_tier;
use std::os::unix::io::{FromRawFd, RawFd};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tracing::{debug, error, info};

#[derive(Debug)]
struct HelperConfig {
    request_fd: RawFd,
    response_fd: RawFd,
}

impl HelperConfig {
    fn from_args() -> Result<Self> {
        let args: Vec<String> = std::env::args().collect();

        if args.len() < 5 || args[1] != "--request-fd" || args[3] != "--response-fd" {
            eyre::bail!(
                "Usage: {} --request-fd <fd> --response-fd <fd>",
                args.first()
                    .map(|s| s.as_str())
                    .unwrap_or("play_launch_rt_helper")
            );
        }

        let request_fd = args[2]
            .parse::<RawFd>()
            .wrap_err("Failed to parse request-fd")?;

        let response_fd = args[4]
            .parse::<RawFd>()
            .wrap_err("Failed to parse response-fd")?;

        Ok(Self {
            request_fd,
            response_fd,
        })
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // CRITICAL: Set up parent death signal to prevent orphans
    // If play_launch (parent) crashes, helper receives SIGTERM and exits
    #[cfg(target_os = "linux")]
    unsafe {
        use libc::{PR_SET_PDEATHSIG, SIGTERM, prctl};
        if prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 {
            eprintln!("Warning: Failed to set parent death signal");
        }
    }

    // Initialize tracing subscriber for logging
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    info!("play_launch_rt_helper starting (parent death signal: SIGTERM)");

    let config = HelperConfig::from_args().wrap_err("Failed to parse arguments")?;

    info!(
        "Using FDs: request_fd={}, response_fd={}",
        config.request_fd, config.response_fd
    );

    // Convert raw FDs to tokio File handles
    // SAFETY: The parent process has passed us these FDs via command line.
    // We trust the parent to give us valid pipe FDs.
    let request_file = unsafe { std::fs::File::from_raw_fd(config.request_fd) };
    let response_file = unsafe { std::fs::File::from_raw_fd(config.response_fd) };

    // Convert std::fs::File to tokio::fs::File for async I/O
    let request_stream = tokio::fs::File::from_std(request_file);
    let response_stream = tokio::fs::File::from_std(response_file);

    info!("Pipe streams ready, starting request handler");

    // Handle requests from pipes
    if let Err(e) = handle_client(request_stream, response_stream).await {
        error!("Error handling client: {}", e);
    }

    info!("Shutting down");

    Ok(())
}

/// Handle requests from pipes (separate read and write streams)
async fn handle_client(
    mut request_stream: tokio::fs::File,
    mut response_stream: tokio::fs::File,
) -> Result<()> {
    let mut shutdown_requested = false;

    while !shutdown_requested {
        // Read length prefix (4 bytes, little-endian u32)
        let mut len_buf = [0u8; 4];
        match request_stream.read_exact(&mut len_buf).await {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                debug!("Client disconnected (pipe closed)");
                break;
            }
            Err(e) => {
                return Err(e).wrap_err("Failed to read message length");
            }
        }

        let msg_len = u32::from_le_bytes(len_buf) as usize;

        // Sanity check: limit message size to 1MB
        if msg_len > 1024 * 1024 {
            error!("Message too large: {} bytes", msg_len);
            break;
        }

        // Read message payload
        let mut msg_buf = vec![0u8; msg_len];
        request_stream
            .read_exact(&mut msg_buf)
            .await
            .wrap_err("Failed to read message payload")?;

        // Decode request
        let request: SchedRequest =
            decode_message(&msg_buf).wrap_err("Failed to decode request")?;

        debug!("Received request: {:?}", request);

        // Process request
        let response = match request {
            SchedRequest::ApplySched { pid, tier } => {
                SchedResponse::Applied(apply_tier(pid, &tier))
            }
            SchedRequest::Ping => SchedResponse::Pong,
            SchedRequest::Shutdown => {
                shutdown_requested = true;
                SchedResponse::ShutdownAck
            }
        };

        debug!("Sending response: {:?}", response);

        // Encode and send response
        let response_buf = encode_message(&response).wrap_err("Failed to encode response")?;

        response_stream
            .write_all(&response_buf)
            .await
            .wrap_err("Failed to send response")?;

        response_stream
            .flush()
            .await
            .wrap_err("Failed to flush stream")?;
    }

    Ok(())
}
