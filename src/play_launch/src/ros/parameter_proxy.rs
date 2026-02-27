//! ParameterProxy — service client wrapper for querying/setting ROS 2 node parameters.
//!
//! Wraps the four parameter service clients needed for the web UI parameter tab.
//! Constructed on-demand per request (service clients are cheap to create in rclrs).

use super::parameter_types::{
    type_name_from_ros, FloatRange, IntegerRange, ParamEntry, ParamValue, SetParamResult,
};
use eyre::{Context as _, Result};
use std::{sync::Arc, time::Duration};
use tracing::debug;

/// Timeout for parameter service calls.
const PARAM_SERVICE_TIMEOUT: Duration = Duration::from_secs(5);

/// Wraps the parameter service clients for one target node.
pub struct ParameterProxy {
    list_client: rclrs::Client<rcl_interfaces::srv::ListParameters>,
    get_client: rclrs::Client<rcl_interfaces::srv::GetParameters>,
    set_client: rclrs::Client<rcl_interfaces::srv::SetParameters>,
    describe_client: rclrs::Client<rcl_interfaces::srv::DescribeParameters>,
}

impl ParameterProxy {
    /// Create a new ParameterProxy for the given target node FQN.
    ///
    /// Builds service names from the target FQN and creates four clients
    /// on the shared ROS node (reusing its executor thread).
    pub fn new(ros_node: &Arc<rclrs::Node>, target_fqn: &str) -> Result<Self> {
        let list_client = ros_node
            .create_client(&format!("{}/list_parameters", target_fqn))
            .context("Failed to create list_parameters client")?;
        let get_client = ros_node
            .create_client(&format!("{}/get_parameters", target_fqn))
            .context("Failed to create get_parameters client")?;
        let set_client = ros_node
            .create_client(&format!("{}/set_parameters", target_fqn))
            .context("Failed to create set_parameters client")?;
        let describe_client = ros_node
            .create_client(&format!("{}/describe_parameters", target_fqn))
            .context("Failed to create describe_parameters client")?;

        Ok(Self {
            list_client,
            get_client,
            set_client,
            describe_client,
        })
    }

    /// List all parameters, describe them, and get their values in one call.
    ///
    /// Returns a full snapshot of all parameters with values, types, descriptions,
    /// and range constraints.
    pub async fn list_all(&self) -> Result<Vec<ParamEntry>> {
        // Step 1: List all parameter names (recursive, no prefix filter)
        let list_request = rcl_interfaces::srv::ListParameters_Request {
            prefixes: vec![],
            depth: 0, // 0 = recursive (all depths)
        };

        let list_promise: rclrs::Promise<rcl_interfaces::srv::ListParameters_Response> = self
            .list_client
            .call(&list_request)
            .context("Failed to call list_parameters")?;
        let list_response = tokio::time::timeout(PARAM_SERVICE_TIMEOUT, list_promise)
            .await
            .context("list_parameters timed out")?
            .context("list_parameters call failed")?;

        let names = list_response.result.names;
        if names.is_empty() {
            return Ok(Vec::new());
        }

        debug!("Listed {} parameters", names.len());

        // Step 2: Describe all parameters (types, constraints, read_only)
        let describe_request = rcl_interfaces::srv::DescribeParameters_Request {
            names: names.clone(),
        };

        let describe_promise: rclrs::Promise<rcl_interfaces::srv::DescribeParameters_Response> =
            self.describe_client
                .call(&describe_request)
                .context("Failed to call describe_parameters")?;
        let describe_response = tokio::time::timeout(PARAM_SERVICE_TIMEOUT, describe_promise)
            .await
            .context("describe_parameters timed out")?
            .context("describe_parameters call failed")?;

        // Step 3: Get all parameter values
        let get_request = rcl_interfaces::srv::GetParameters_Request {
            names: names.clone(),
        };

        let get_promise: rclrs::Promise<rcl_interfaces::srv::GetParameters_Response> = self
            .get_client
            .call(&get_request)
            .context("Failed to call get_parameters")?;
        let get_response = tokio::time::timeout(PARAM_SERVICE_TIMEOUT, get_promise)
            .await
            .context("get_parameters timed out")?
            .context("get_parameters call failed")?;

        // Combine into ParamEntry results
        let mut entries = Vec::with_capacity(names.len());
        for (i, name) in names.into_iter().enumerate() {
            let descriptor = describe_response
                .descriptors
                .get(i)
                .cloned()
                .unwrap_or_default();
            let value = get_response.values.get(i).cloned().unwrap_or_default();

            let integer_range = descriptor.integer_range.first().map(|r| IntegerRange {
                from: r.from_value,
                to: r.to_value,
                step: r.step,
            });

            let floating_point_range =
                descriptor.floating_point_range.first().map(|r| FloatRange {
                    from: r.from_value,
                    to: r.to_value,
                    step: r.step,
                });

            entries.push(ParamEntry {
                name,
                value: ParamValue::from_ros(&value),
                type_name: type_name_from_ros(descriptor.type_).to_string(),
                description: descriptor.description,
                read_only: descriptor.read_only,
                integer_range,
                floating_point_range,
            });
        }

        Ok(entries)
    }

    /// Set a single parameter on the target node.
    pub async fn set(&self, name: &str, value: ParamValue) -> Result<SetParamResult> {
        let parameter = rcl_interfaces::msg::Parameter {
            name: name.to_string(),
            value: value.to_ros(),
        };

        let request = rcl_interfaces::srv::SetParameters_Request {
            parameters: vec![parameter],
        };

        let set_promise: rclrs::Promise<rcl_interfaces::srv::SetParameters_Response> = self
            .set_client
            .call(&request)
            .context("Failed to call set_parameters")?;
        let response = tokio::time::timeout(PARAM_SERVICE_TIMEOUT, set_promise)
            .await
            .context("set_parameters timed out")?
            .context("set_parameters call failed")?;

        let result = response.results.into_iter().next().unwrap_or_default();

        Ok(SetParamResult {
            successful: result.successful,
            reason: result.reason,
        })
    }
}
