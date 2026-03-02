use super::super::LaunchTraverser;
use crate::{
    error::{ParseError, Result},
    file_cache::read_file_cached,
    substitution::ArgumentMetadata,
};
use std::path::Path;

impl LaunchTraverser {
    /// Process a YAML launch file to extract argument declarations
    pub(crate) fn process_yaml_launch_file(&mut self, path: &Path) -> Result<()> {
        use serde_yaml::Value;

        let content = read_file_cached(path)?;
        let yaml: Value = serde_yaml::from_str(&content)
            .map_err(|e| ParseError::InvalidSubstitution(format!("Invalid YAML: {}", e)))?;

        // Extract 'launch' key
        if let Some(launch_list) = yaml.get("launch").and_then(|v| v.as_sequence()) {
            for item in launch_list {
                // Check if this is an arg declaration
                if let Some(arg_map) = item.get("arg").and_then(|v| v.as_mapping())
                    && let (Some(name), default_value) = (
                        arg_map
                            .get(Value::String("name".to_string()))
                            .and_then(|v| v.as_str()),
                        arg_map
                            .get(Value::String("default".to_string()))
                            .and_then(|v| v.as_str()),
                    )
                {
                    // Declare the argument in the context
                    log::debug!("[RUST] YAML declares arg: {} = {:?}", name, default_value);
                    self.context.declare_argument(ArgumentMetadata {
                        name: name.to_string(),
                        default: default_value.map(|s| s.to_string()),
                        description: None,
                        choices: None,
                    });

                    // Set the configuration value if a default is provided and not already set
                    if let Some(default) = default_value
                        && self.context.get_configuration(name).is_none()
                    {
                        log::debug!(
                            "[RUST] YAML setting default value for {}: {}",
                            name,
                            default
                        );
                        self.context
                            .set_configuration(name.to_string(), default.to_string());
                    }
                }
            }
        }

        Ok(())
    }
}
