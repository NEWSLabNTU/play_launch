//! Web server module for the play_launch web UI.
//!
//! Provides a web interface for monitoring and controlling ROS nodes.

use crate::event_driven::{events::EventBus, member_registry::MemberRegistry};
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

mod handlers;
mod sse;
pub mod web_types;

/// Embedded static assets for the web UI
#[derive(Embed)]
#[folder = "src/web/assets/"]
struct Assets;

/// Shared state for the web server (event-driven architecture)
pub struct WebState {
    /// Member registry for querying node state
    pub registry: Arc<TokioMutex<MemberRegistry>>,
    /// Event bus for publishing control commands
    pub event_bus: EventBus,
    /// Base log directory (used for log file access)
    #[allow(dead_code)]
    pub log_dir: PathBuf,
    /// Track nodes currently being operated on (to prevent racing conditions)
    pub operations_in_progress: TokioMutex<HashSet<String>>,
}

impl WebState {
    /// Create a new WebState with event-driven architecture
    pub fn new(
        registry: Arc<TokioMutex<MemberRegistry>>,
        event_bus: EventBus,
        log_dir: PathBuf,
    ) -> Self {
        Self {
            registry,
            event_bus,
            log_dir,
            operations_in_progress: TokioMutex::new(HashSet::new()),
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
        // API endpoints
        .route("/api/nodes", get(handlers::list_nodes))
        .route("/api/nodes/:name", get(handlers::get_node))
        .route("/api/nodes/:name/start", post(handlers::start_node))
        .route("/api/nodes/:name/stop", post(handlers::stop_node))
        .route("/api/nodes/:name/restart", post(handlers::restart_node))
        .route(
            "/api/nodes/:name/respawn/:enabled",
            post(handlers::toggle_respawn),
        )
        .route("/api/health", get(handlers::health_summary))
        // SSE endpoints for log streaming
        .route("/api/nodes/:name/logs/stdout", get(sse::stream_stdout))
        .route("/api/nodes/:name/logs/stderr", get(sse::stream_stderr))
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

    info!("Web UI available at http://{}:{}", bind_addr, port);
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
