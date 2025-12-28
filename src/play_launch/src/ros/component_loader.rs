// Component loader module - loads composable nodes via ROS service calls
//
// This module provides a background thread that handles loading composable nodes
// into containers using the composition_interfaces/srv/LoadNode service.
//
// The approach uses direct ROS service calls via rclrs instead of spawning
// `ros2 component load` subprocesses, which eliminates Python startup overhead
// and enables better parallelization.

use eyre::{Context as _, Result};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::Duration,
};
use tokio::sync::{mpsc, oneshot};
use tracing::{debug, error};

/// Response from loading a composable node
#[derive(Clone, Debug, PartialEq)]
pub struct LoadNodeResponse {
    pub success: bool,
    pub error_message: String,
    pub full_node_name: String,
    pub unique_id: u64,
}

/// Handle to the component loader background thread
#[derive(Clone)]
pub struct ComponentLoaderHandle {
    request_tx: mpsc::UnboundedSender<LoadRequest>,
}

/// Internal request structure for loading a node
struct LoadRequest {
    container_name: String,
    package_name: String,
    plugin_name: String,
    node_name: String,
    node_namespace: String,
    remap_rules: Vec<String>,
    parameters: Vec<(String, String)>,
    extra_args: Vec<(String, String)>,
    response_tx: oneshot::Sender<Result<LoadNodeResponse>>,
    timeout: Duration,
}

impl ComponentLoaderHandle {
    /// Request to load a composable node into a container
    ///
    /// Returns a future that resolves when the service call completes
    #[allow(clippy::too_many_arguments)]
    pub async fn load_node(
        &self,
        container_name: &str,
        package_name: &str,
        plugin_name: &str,
        node_name: &str,
        node_namespace: &str,
        remap_rules: Vec<String>,
        parameters: Vec<(String, String)>,
        extra_args: Vec<(String, String)>,
        timeout: Duration,
    ) -> Result<LoadNodeResponse> {
        let (response_tx, response_rx) = oneshot::channel();

        let request = LoadRequest {
            container_name: container_name.to_string(),
            package_name: package_name.to_string(),
            plugin_name: plugin_name.to_string(),
            node_name: node_name.to_string(),
            node_namespace: node_namespace.to_string(),
            remap_rules,
            parameters,
            extra_args,
            response_tx,
            timeout,
        };

        debug!("Sending load request to component loader thread...");
        self.request_tx.send(request).wrap_err_with(|| {
            format!(
                "Component loader thread has stopped (while loading {}/{} into {})",
                package_name, plugin_name, container_name
            )
        })?;

        debug!("Waiting for response from component loader thread...");
        let result = response_rx.await.wrap_err_with(|| {
            format!(
                "Component loader dropped request for {}/{} into {}",
                package_name, plugin_name, container_name
            )
        })?;

        debug!("Received response from component loader thread");
        result
    }
}

/// Start the component loader background thread
///
/// This creates a ROS node and executor that processes load requests
pub fn start_component_loader_thread() -> Result<ComponentLoaderHandle> {
    let (request_tx, request_rx) = mpsc::unbounded_channel();

    std::thread::Builder::new()
        .name("component_loader".to_string())
        .spawn(move || {
            if let Err(e) = run_component_loader_loop(request_rx) {
                error!("Component loader thread failed: {:#}", e);
            }
        })
        .context("Failed to spawn component loader thread")?;

    debug!("Started component loader background thread");

    Ok(ComponentLoaderHandle { request_tx })
}

/// Main loop for the component loader thread
///
/// This runs in a dedicated thread and handles all service calls
fn run_component_loader_loop(mut request_rx: mpsc::UnboundedReceiver<LoadRequest>) -> Result<()> {
    use rclrs::CreateBasicExecutor;

    // Initialize ROS context and node (use empty args to avoid deprecation warnings)
    let context = rclrs::Context::new(
        vec!["play_launch".to_string()],
        rclrs::InitOptions::default(),
    )
    .context("Failed to create ROS context")?;

    let mut executor = context.create_basic_executor();
    let node = executor
        .create_node("play_launch_component_loader")
        .context("Failed to create component loader node")?;

    debug!("Component loader node created");

    // Cache service clients for each container
    let clients: Arc<Mutex<HashMap<String, rclrs::Client<composition_interfaces::srv::LoadNode>>>> =
        Arc::new(Mutex::new(HashMap::new()));

    // Start a background thread to spin the executor
    let clients_for_spin = Arc::clone(&clients);
    std::thread::spawn(move || loop {
        executor.spin(rclrs::SpinOptions::spin_once());
        std::thread::sleep(std::time::Duration::from_millis(10));
    });

    // Main request processing loop
    while let Some(request) = request_rx.blocking_recv() {
        debug!(
            "Loading node {}/{} into container {}",
            request.package_name, request.plugin_name, request.container_name
        );

        // Get or create service client for this container
        let service_name = format!("{}/_container/load_node", request.container_name);
        let client = {
            let mut clients_lock = clients_for_spin.lock().unwrap();
            clients_lock
                .entry(service_name.clone())
                .or_insert_with(|| {
                    debug!("Creating service client for {}", service_name);
                    node.create_client::<composition_interfaces::srv::LoadNode>(&service_name)
                        .expect("Failed to create service client")
                })
                .clone()
        };

        // Process the request
        let result = tokio::runtime::Runtime::new()
            .context("Failed to create tokio runtime")?
            .block_on(async {
                call_component_load_service(
                    client,
                    &request.package_name,
                    &request.plugin_name,
                    &request.node_name,
                    &request.node_namespace,
                    &request.remap_rules,
                    &request.parameters,
                    &request.extra_args,
                    request.timeout,
                )
                .await
            });

        let _ = request.response_tx.send(result);
    }

    debug!("Component loader shutting down");
    Ok(())
}

/// Call LoadNode service using rclrs
///
/// This replaces the subprocess-based approach with direct service calls
#[allow(clippy::too_many_arguments)]
async fn call_component_load_service(
    client: rclrs::Client<composition_interfaces::srv::LoadNode>,
    package: &str,
    plugin: &str,
    node_name: &str,
    namespace: &str,
    remap_rules: &[String],
    parameters: &[(String, String)],
    extra_args: &[(String, String)],
    timeout: Duration,
) -> Result<LoadNodeResponse> {
    debug!(
        "Calling LoadNode service for {}/{} (node_name: {}, namespace: {})",
        package, plugin, node_name, namespace
    );

    // Build the service request
    let request = composition_interfaces::srv::LoadNode_Request {
        package_name: package.to_string(),
        plugin_name: plugin.to_string(),
        node_name: node_name.to_string(),
        node_namespace: namespace.to_string(),
        log_level: 0, // Default log level
        remap_rules: remap_rules.to_vec(),
        parameters: convert_parameters_to_ros(parameters)?,
        extra_arguments: convert_parameters_to_ros(extra_args)?,
    };

    // Call the service
    debug!("Sending service request...");
    let response_future: rclrs::Promise<composition_interfaces::srv::LoadNode_Response> = client
        .call(&request)
        .wrap_err_with(|| format!("Failed to call LoadNode service for {}/{}", package, plugin))?;

    // Wait for response with timeout
    match tokio::time::timeout(timeout, response_future).await {
        Ok(Ok(response)) => {
            debug!(
                "Received response: success={}, full_node_name={}, unique_id={}",
                response.success, response.full_node_name, response.unique_id
            );

            Ok(LoadNodeResponse {
                success: response.success,
                error_message: response.error_message.clone(),
                full_node_name: response.full_node_name.clone(),
                unique_id: response.unique_id,
            })
        }
        Ok(Err(e)) => {
            debug!("Service call was canceled: {:?}", e);
            Ok(LoadNodeResponse {
                success: false,
                error_message: format!("Service call was canceled: {:?}", e),
                full_node_name: String::new(),
                unique_id: 0,
            })
        }
        Err(_) => {
            debug!("Service call timed out after {:?}", timeout);
            Ok(LoadNodeResponse {
                success: false,
                error_message: format!("Timeout after {:?}", timeout),
                full_node_name: String::new(),
                unique_id: 0,
            })
        }
    }
}

/// Convert string parameters to ROS Parameter messages
fn convert_parameters_to_ros(
    params: &[(String, String)],
) -> Result<Vec<rcl_interfaces::msg::Parameter>> {
    params
        .iter()
        .map(|(name, value)| {
            Ok(rcl_interfaces::msg::Parameter {
                name: name.clone(),
                value: parse_parameter_value(value)?,
            })
        })
        .collect()
}

/// Parse a parameter value string into a ParameterValue
///
/// This function mimics the behavior of ROS2 CLI's get_parameter_value() which uses
/// yaml.safe_load() to determine parameter types. This allows proper handling of
/// array parameters like [1, 2, 3] (integer_array) or [0.1, 0.2] (double_array).
fn parse_parameter_value(value: &str) -> Result<rcl_interfaces::msg::ParameterValue> {
    // Parse value as YAML to determine type
    let yaml_value: serde_yaml::Value = match serde_yaml::from_str(value) {
        Ok(v) => v,
        Err(_) => {
            // If YAML parsing fails, treat as string
            return Ok(create_string_parameter(value));
        }
    };

    match yaml_value {
        // Boolean
        serde_yaml::Value::Bool(b) => Ok(create_bool_parameter(b)),

        // Integer
        serde_yaml::Value::Number(n) if n.is_i64() => {
            Ok(create_integer_parameter(n.as_i64().unwrap()))
        }

        // Double (includes f64 and integers that need to be floats)
        serde_yaml::Value::Number(n) if n.is_f64() => {
            Ok(create_double_parameter(n.as_f64().unwrap()))
        }

        // Arrays (Sequence)
        serde_yaml::Value::Sequence(seq) => {
            if seq.is_empty() {
                // Empty array defaults to string array
                return Ok(create_string_array_parameter(Vec::new()));
            }

            // Check if all elements are the same type
            if seq.iter().all(|v| matches!(v, serde_yaml::Value::Bool(_))) {
                let values: Vec<bool> = seq.iter().filter_map(|v| v.as_bool()).collect();
                Ok(create_bool_array_parameter(values))
            } else if seq
                .iter()
                .all(|v| matches!(v, serde_yaml::Value::Number(n) if n.is_i64()))
            {
                let values: Vec<i64> = seq.iter().filter_map(|v| v.as_i64()).collect();
                Ok(create_integer_array_parameter(values))
            } else if seq
                .iter()
                .all(|v| matches!(v, serde_yaml::Value::Number(_)))
            {
                // All numbers, treat as doubles (handles mixed int/float)
                let values: Vec<f64> = seq.iter().filter_map(|v| v.as_f64()).collect();
                Ok(create_double_array_parameter(values))
            } else if seq
                .iter()
                .all(|v| matches!(v, serde_yaml::Value::String(_)))
            {
                let values: Vec<String> = seq
                    .iter()
                    .filter_map(|v| v.as_str().map(String::from))
                    .collect();
                Ok(create_string_array_parameter(values))
            } else {
                // Mixed types or unsupported, fall back to string
                Ok(create_string_parameter(value))
            }
        }

        // String or other types
        serde_yaml::Value::String(s) => Ok(create_string_parameter(&s)),
        _ => Ok(create_string_parameter(value)),
    }
}

// Helper functions to create ParameterValue structs
// These eliminate repetitive code and match the pattern used by ROS2 CLI

fn create_bool_parameter(value: bool) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
        bool_value: value,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_integer_parameter(value: i64) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
        bool_value: false,
        integer_value: value,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_double_parameter(value: f64) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
        bool_value: false,
        integer_value: 0,
        double_value: value,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_string_parameter(value: &str) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: value.to_string(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_bool_array_parameter(values: Vec<bool>) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: values,
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_integer_array_parameter(values: Vec<i64>) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: values,
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_double_array_parameter(values: Vec<f64>) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: values,
        string_array_value: Vec::new(),
    }
}

fn create_string_array_parameter(values: Vec<String>) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: values,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_bool_true() {
        let result = parse_parameter_value("true").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_BOOL
        );
        assert!(result.bool_value);
    }

    #[test]
    fn test_parse_bool_false() {
        let result = parse_parameter_value("false").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_BOOL
        );
        assert!(!result.bool_value);
    }

    #[test]
    fn test_parse_integer() {
        let result = parse_parameter_value("42").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER
        );
        assert_eq!(result.integer_value, 42);
    }

    #[test]
    fn test_parse_negative_integer() {
        let result = parse_parameter_value("-123").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER
        );
        assert_eq!(result.integer_value, -123);
    }

    #[test]
    fn test_parse_double() {
        let result = parse_parameter_value("3.14").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE
        );
        assert!((result.double_value - std::f64::consts::PI).abs() < 0.0001);
    }

    #[test]
    fn test_parse_string() {
        let result = parse_parameter_value("hello world").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING
        );
        assert_eq!(result.string_value, "hello world");
    }

    #[test]
    fn test_parse_integer_array() {
        let result = parse_parameter_value("[1, 2, 3, 4]").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY
        );
        assert_eq!(result.integer_array_value, vec![1, 2, 3, 4]);
    }

    #[test]
    fn test_parse_double_array() {
        let result = parse_parameter_value("[0.1, 0.3, 20.0, 30.0]").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY
        );
        assert_eq!(result.double_array_value.len(), 4);
        assert!((result.double_array_value[0] - 0.1).abs() < 0.0001);
        assert!((result.double_array_value[1] - 0.3).abs() < 0.0001);
        assert!((result.double_array_value[2] - 20.0).abs() < 0.0001);
        assert!((result.double_array_value[3] - 30.0).abs() < 0.0001);
    }

    #[test]
    fn test_parse_string_array() {
        let result = parse_parameter_value("[curbstone]").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY
        );
        assert_eq!(result.string_array_value, vec!["curbstone"]);
    }

    #[test]
    fn test_parse_string_array_multiple() {
        let result = parse_parameter_value("[foo, bar, baz]").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY
        );
        assert_eq!(result.string_array_value, vec!["foo", "bar", "baz"]);
    }

    #[test]
    fn test_parse_bool_array() {
        let result = parse_parameter_value("[true, false, true]").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY
        );
        assert_eq!(result.bool_array_value, vec![true, false, true]);
    }

    #[test]
    fn test_parse_empty_array() {
        let result = parse_parameter_value("[]").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY
        );
        assert!(result.string_array_value.is_empty());
    }

    #[test]
    fn test_parse_yaml_string_array_with_quotes() {
        // This is how StateMonitor's launch.map appears in record.json
        let result =
            parse_parameter_value("['topic_state_monitor_vector_map: map_topic_status']").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY
        );
        assert_eq!(result.string_array_value.len(), 1);
        assert_eq!(
            result.string_array_value[0],
            "topic_state_monitor_vector_map: map_topic_status"
        );
    }

    #[test]
    fn test_parse_mixed_number_array_as_double() {
        // Arrays with mixed int/float should become double_array
        let result = parse_parameter_value("[1, 2.5, 3]").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY
        );
        assert_eq!(result.double_array_value.len(), 3);
        assert!((result.double_array_value[0] - 1.0).abs() < 0.0001);
        assert!((result.double_array_value[1] - 2.5).abs() < 0.0001);
        assert!((result.double_array_value[2] - 3.0).abs() < 0.0001);
    }

    #[test]
    fn test_parse_scientific_notation() {
        let result = parse_parameter_value("1.5e-7").unwrap();
        assert_eq!(
            result.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE
        );
        assert!((result.double_value - 1.5e-7).abs() < 1e-10);
    }
}
