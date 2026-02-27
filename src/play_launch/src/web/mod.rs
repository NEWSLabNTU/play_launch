//! Web server module for the play_launch web UI.
//!
//! Provides a web interface for monitoring and controlling ROS nodes.

use crate::{diagnostics::DiagnosticRegistry, member_actor::MemberHandle};
use axum::{
    http::{header, StatusCode},
    response::{IntoResponse, Response},
    routing::{get, post},
    Router,
};
use rust_embed::Embed;
use std::{collections::HashSet, net::SocketAddr, path::PathBuf, sync::Arc};
use tokio::sync::Mutex as TokioMutex;
use tower_http::cors::{Any, CorsLayer};
use tracing::{info, warn};

mod broadcaster;
mod handlers;
mod sse;
pub mod web_types;

pub use broadcaster::StateEventBroadcaster;

/// Embedded static assets for the web UI
#[derive(Embed)]
#[folder = "src/web/assets/"]
struct Assets;

/// Shared state for the web server
pub struct WebState {
    /// Handle for actor control and state queries (Arc for sharing)
    pub member_handle: Arc<MemberHandle>,
    /// Base log directory (used for log file access)
    #[allow(dead_code)]
    pub log_dir: PathBuf,
    /// Track nodes currently being operated on (to prevent racing conditions)
    pub operations_in_progress: TokioMutex<HashSet<String>>,
    /// Broadcaster for state events to SSE clients
    pub state_broadcaster: Arc<StateEventBroadcaster>,
    /// Diagnostic registry for storing diagnostic data
    pub diagnostic_registry: Arc<DiagnosticRegistry>,
}

impl WebState {
    /// Create a new WebState
    pub fn new(
        member_handle: Arc<MemberHandle>,
        log_dir: PathBuf,
        state_broadcaster: Arc<StateEventBroadcaster>,
        diagnostic_registry: Arc<DiagnosticRegistry>,
    ) -> Self {
        Self {
            member_handle,
            log_dir,
            operations_in_progress: TokioMutex::new(HashSet::new()),
            state_broadcaster,
            diagnostic_registry,
        }
    }
}

/// Serve embedded static files
async fn serve_static(path: &str) -> Response {
    let path = if path.is_empty() || path == "/" {
        "index.html"
    } else {
        path.trim_start_matches('/')
    };

    match Assets::get(path) {
        Some(content) => {
            let mime = mime_guess::from_path(path).first_or_octet_stream();
            (
                StatusCode::OK,
                [(header::CONTENT_TYPE, mime.as_ref())],
                content.data.into_owned(),
            )
                .into_response()
        }
        None => (StatusCode::NOT_FOUND, "Not Found").into_response(),
    }
}

/// Handler for the root path - serves index.html
async fn index_handler() -> Response {
    serve_static("index.html").await
}

/// Handler for static assets
async fn static_handler(axum::extract::Path(path): axum::extract::Path<String>) -> Response {
    serve_static(&path).await
}

/// Create the web server router
pub fn create_router(state: Arc<WebState>) -> Router {
    // CORS layer for development
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    Router::new()
        // Static files
        .route("/", get(index_handler))
        .route("/static/*path", get(static_handler))
        .route("/assets/*path", get(static_handler))
        // API endpoints
        .route("/api/nodes", get(handlers::list_nodes))
        .route("/api/nodes/:name", get(handlers::get_node))
        .route("/api/nodes/:name/start", post(handlers::start_node))
        .route("/api/nodes/:name/stop", post(handlers::stop_node))
        .route("/api/nodes/:name/restart", post(handlers::restart_node))
        .route("/api/nodes/:name/load", post(handlers::load_node))
        .route("/api/nodes/:name/unload", post(handlers::unload_node))
        .route("/api/nodes/:name/load-all", post(handlers::load_all_nodes))
        .route(
            "/api/nodes/:name/unload-all",
            post(handlers::unload_all_nodes),
        )
        .route(
            "/api/nodes/:name/respawn/:enabled",
            post(handlers::toggle_respawn),
        )
        .route(
            "/api/nodes/:name/auto-load/:enabled",
            post(handlers::toggle_auto_load),
        )
        .route(
            "/api/nodes/:name/parameters",
            get(handlers::get_node_parameters).post(handlers::set_node_parameter),
        )
        .route("/api/health", get(handlers::health_summary))
        // Bulk operations
        .route("/api/nodes/start-all", post(handlers::start_all))
        .route("/api/nodes/stop-all", post(handlers::stop_all))
        .route("/api/nodes/restart-all", post(handlers::restart_all))
        // Diagnostics endpoints
        .route("/api/diagnostics/list", get(handlers::list_diagnostics))
        .route(
            "/api/diagnostics/counts",
            get(handlers::get_diagnostic_counts),
        )
        // SSE endpoints for log streaming
        .route("/api/nodes/:name/logs/stdout", get(sse::stream_stdout))
        .route("/api/nodes/:name/logs/stderr", get(sse::stream_stderr))
        // SSE endpoint for state updates
        .route("/api/state/updates", get(sse::stream_state_updates))
        .layer(cors)
        .with_state(state)
}

/// Run the web server with graceful shutdown support
pub async fn run_server(
    state: Arc<WebState>,
    bind_addr: &str,
    port: u16,
    shutdown: tokio::sync::watch::Receiver<bool>,
) -> eyre::Result<()> {
    let app = create_router(state);

    // Parse bind address
    let ip: std::net::IpAddr = bind_addr
        .parse()
        .map_err(|e| eyre::eyre!("Invalid bind address '{}': {}", bind_addr, e))?;
    let addr = SocketAddr::new(ip, port);

    if bind_addr == "0.0.0.0" {
        warn!("Web UI is exposed to network (0.0.0.0) - ensure this is intentional!");
    }

    let listener = tokio::net::TcpListener::bind(addr).await?;

    // Wait for shutdown signal
    axum::serve(listener, app)
        .with_graceful_shutdown(async move {
            let mut shutdown = shutdown;
            loop {
                if *shutdown.borrow() {
                    break;
                }
                if shutdown.changed().await.is_err() {
                    break;
                }
            }
            info!("Web server shutting down...");
        })
        .await
        .map_err(|e| eyre::eyre!("Web server error: {}", e))?;

    Ok(())
}
