use super::{super::LaunchTraverser, include::IncludeArgs};
use crate::{
    captures::DeclaredArgumentCapture,
    error::{ParseError, Result},
    exchange::{ContextState, ExecResult, IncludeHost, IncludeRequest, ListSync, Produced},
};
use std::path::{Path, PathBuf};

/// The two launch-file-location substitutions, as they cross the Python
/// boundary. `ThisLaunchFileDir()` and `ThisLaunchFile()` are captured as these
/// literal strings by the `pyexec` mock and resolved by the host.
///
/// `ThisLaunchFile()` emits `$(filename)` and not a token of its own, because
/// in ROS 2 they ARE the same substitution class: `this_launch_file.py` is
/// `@expose_substitution('filename')`. The mock used to emit
/// `$(this-launch-file)`, which no grammar on either side knew, so the literal
/// reached the record, the model and the command line (issue 0041).
const DIRNAME_TOKEN: &str = "$(dirname)";
const FILENAME_TOKEN: &str = "$(filename)";

/// The resolved values of `$(dirname)` and `$(filename)` for one launch file.
struct FileSubstitutions {
    dirname: Option<String>,
    /// The ABSOLUTE PATH, matching `Substitution::Filename` and ROS 2's
    /// `ThisLaunchFile` — not the basename.
    filename: Option<String>,
}

impl FileSubstitutions {
    fn of(path: &Path) -> Self {
        let abs = crate::record::absolute_path(path);
        Self {
            dirname: abs.parent().and_then(|p| p.to_str().map(String::from)),
            filename: abs.to_str().map(String::from),
        }
    }

    /// Rewrite the two file-location tokens, and ONLY those. `$(dirname)` is
    /// a property of the file that DECLARED the member, fixed at parse time.
    fn rewrite(&self, value: &mut String) {
        if let Some(dir) = &self.dirname
            && value.contains(DIRNAME_TOKEN)
        {
            *value = value.replace(DIRNAME_TOKEN, dir);
        }
        if let Some(file) = &self.filename
            && value.contains(FILENAME_TOKEN)
        {
            *value = value.replace(FILENAME_TOKEN, file);
        }
    }

    fn rewrite_all(&self, values: &mut [String]) {
        for value in values {
            self.rewrite(value);
        }
    }

    fn rewrite_pairs(&self, pairs: &mut [(String, String)]) {
        for (key, value) in pairs {
            self.rewrite(key);
            self.rewrite(value);
        }
    }

    fn rewrite_produced(&self, produced: &mut Produced) {
        for capture in &mut produced.nodes {
            self.rewrite_pairs(&mut capture.parameters);
            self.rewrite_all(&mut capture.params_files);
            self.rewrite_pairs(&mut capture.remappings);
            self.rewrite_all(&mut capture.arguments);
            self.rewrite_all(&mut capture.ros_arguments);
            self.rewrite_pairs(&mut capture.env_vars);
            for source in &mut capture.param_sources {
                match source {
                    crate::record::types::ParamSource::Inline { value, .. } => self.rewrite(value),
                    // File sources hold YAML CONTENT, not a path.
                    crate::record::types::ParamSource::File { .. } => {}
                }
            }
        }
        for capture in &mut produced.containers {
            self.rewrite_all(&mut capture.cmd);
            self.rewrite_all(&mut capture.ros_arguments);
            self.rewrite_pairs(&mut capture.parameters);
            self.rewrite_all(&mut capture.params_files);
            self.rewrite_pairs(&mut capture.remappings);
            self.rewrite_all(&mut capture.arguments);
            self.rewrite_pairs(&mut capture.env_vars);
        }
        for capture in &mut produced.load_nodes {
            self.rewrite_pairs(&mut capture.parameters);
            self.rewrite_pairs(&mut capture.remappings);
            for value in capture.extra_args.values_mut() {
                self.rewrite(value);
            }
        }
    }
}

/// The traverser, as the Python half sees it while one `.launch.py` runs: it
/// takes in what the file produced and runs each include the file reaches,
/// in the state the file had reached, and hands its state back.
struct PythonIncludeHost<'a> {
    traverser: &'a mut LaunchTraverser,
    /// The file being run, which owns every capture it produces.
    path: PathBuf,
    /// This side's reference point for the shared global lists.
    sync: ListSync,
    /// Every declaration the file executed, across all hand-overs.
    declared: Vec<DeclaredArgumentCapture>,
    /// The typed error an include failed with, so it survives the trip
    /// through the Python half as more than a message.
    error: Option<ParseError>,
}

impl PythonIncludeHost<'_> {
    /// Take in a batch of what the file produced, in order.
    fn take_in(&mut self, mut produced: Produced) {
        FileSubstitutions::of(&self.path).rewrite_produced(&mut produced);
        let ctx = &mut self.traverser.context;
        ctx.captured_nodes_mut().extend(produced.nodes);
        ctx.captured_containers_mut().extend(produced.containers);
        ctx.captured_load_nodes_mut().extend(produced.load_nodes);
        self.declared.extend(produced.declared_arguments);
        // Anything the Python frontend could not model. Stamped with THIS
        // file, which is the one thing the mock action could not know.
        for (action, detail) in produced.unsupported {
            log::warn!(
                "Unsupported action type: {action} (in {})",
                self.path.display()
            );
            self.traverser.note_dropped(crate::record::DroppedAction {
                action,
                file: Some(self.path.display().to_string()),
                detail,
            });
        }
    }

    /// Adopt the state the file reached. An include scopes nothing, so this
    /// IS the includer's state from here on.
    fn adopt(&mut self, state: &ContextState) {
        self.sync = self.traverser.context.import_state(state, &self.sync);
    }
}

impl IncludeHost for PythonIncludeHost<'_> {
    fn include(&mut self, request: IncludeRequest) -> std::result::Result<ContextState, String> {
        let IncludeRequest {
            produced,
            state,
            file_path,
            args,
            delay_secs,
        } = request;
        self.take_in(produced);
        self.adopt(&state);

        log::debug!(
            "Python include from {}: {} ({} args)",
            self.path.display(),
            file_path,
            args.len()
        );
        let mark = self.traverser.delay_mark();
        let result = self
            .traverser
            .process_include_target(&file_path, IncludeArgs::Resolved(&args));
        // A `TimerAction` around the include delays everything it starts,
        // as `<timer>` around an `<include>` does.
        if let Some(secs) = delay_secs {
            self.traverser.apply_start_delay(mark, secs);
        }
        // The included file ran with this file as the current file's
        // includer; it is the current file again.
        self.traverser.context.set_current_file(self.path.clone());
        if let Err(e) = result {
            let message = e.to_string();
            self.error = Some(e);
            return Err(message);
        }

        let out = self.traverser.context.export_state(&self.sync);
        self.sync = self.traverser.context.list_sync();
        Ok(out)
    }
}

impl LaunchTraverser {
    /// Run a `.launch.py` and hand back what it declared (issue 0030), so an
    /// include of it can be held to launch's required-argument rule. A
    /// declaration with no default whose name was unset when it executed is
    /// refused the way `DeclareLaunchArgument.execute` refuses it.
    ///
    /// The file runs in this traverser's context and leaves its state there:
    /// an include scopes nothing, so what the file declares, sets, pushes or
    /// appends is the includer's afterwards. Each `IncludeLaunchDescription`
    /// it reaches is run here, at that point, in the state the file had built
    /// up — see [`crate::exchange`].
    pub(crate) fn execute_python_file(
        &mut self,
        path: &Path,
    ) -> Result<Vec<DeclaredArgumentCapture>> {
        // The file being executed is the current file for as long as it runs.
        // Restored afterwards: `$(dirname)` in the includer's later entities
        // is the includer's directory again.
        let previous_file = self.context.current_file().cloned();
        self.context.set_current_file(path.to_path_buf());
        let result = self.execute_python_file_inner(path);
        match previous_file {
            Some(prev) => self.context.set_current_file(prev),
            None => self.context.clear_current_file(),
        }
        result
    }

    fn execute_python_file_inner(&mut self, path: &Path) -> Result<Vec<DeclaredArgumentCapture>> {
        // The backend, resolved BEFORE anything runs: if there is no Python
        // half in this build, say so while we can still name the file.
        let backend = crate::python_backend::require(
            crate::python_backend::PythonNeed::LaunchFile,
            &path.display().to_string(),
        )
        .map_err(|e| ParseError::PythonError(e.to_string()))?;

        log::debug!("Executing Python file: {}", path.display());
        let path_str = path.to_str().ok_or_else(|| {
            ParseError::PythonError(format!("Invalid UTF-8 in path: {}", path.display()))
        })?;

        // The far side starts with nothing, so no list is "the same" yet.
        let state = self.context.export_state(&ListSync::default());
        let sync = self.context.list_sync();
        let mut host = PythonIncludeHost {
            traverser: self,
            path: path.to_path_buf(),
            sync,
            declared: Vec::new(),
            error: None,
        };
        let outcome = backend.exec_file(path_str, state, &mut host);
        let ExecResult { produced, state } = match outcome {
            Ok(r) => r,
            // An include's own error, typed, beats its rendering as a Python
            // exception.
            Err(message) => {
                return Err(host
                    .error
                    .take()
                    .unwrap_or(ParseError::PythonError(message)));
            }
        };
        host.take_in(produced);
        host.adopt(&state);
        let declared = std::mem::take(&mut host.declared);

        for d in &declared {
            if !d.has_default && d.unset_at_execute {
                return Err(ParseError::RequiredArgumentNotProvided {
                    name: d.name.clone(),
                    description: d
                        .description
                        .clone()
                        .unwrap_or_else(|| "no description given".to_string()),
                    file: path.display().to_string(),
                });
            }
        }

        log::debug!(
            "Python file '{}' completed: {} nodes, {} containers, {} load_nodes captured so far",
            path.display(),
            self.context.captured_nodes().len(),
            self.context.captured_containers().len(),
            self.context.captured_load_nodes().len()
        );
        Ok(declared)
    }
}
