//! The Launch tree's switches (phase 106 W4): `GET /api/selection`,
//! `POST /api/selection/{check,apply,reload}`.
//!
//! The page speaks in member ids; the selection code speaks in model keys.
//! [`SelectionState`] holds the map between them, the model the run spawns
//! from (Reload replaces it), and where the step record goes
//! (`selection.jsonl`). Every switch evaluates through
//! `ros_launch_resolve::selection`, the code behind `--only` and
//! `--exclude`, so the page and the command line cannot disagree about a
//! closure or a cut input. Design: `docs/design/launch-selection.md` 6, 7.

// The handlers return an axum `Response` as their early-exit error, which is
// the size of a response; boxing it would buy nothing on an API this small.
#![allow(clippy::result_large_err)]

use std::{
    collections::{BTreeSet, HashMap, HashSet},
    io::Write,
    path::PathBuf,
    sync::Arc,
    time::Duration,
};

use axum::{
    Json,
    extract::State,
    http::StatusCode,
    response::{IntoResponse, Response},
};
use ros_launch_manifest_model::SystemModel;
use ros_launch_resolve::selection::{self, Selection, shell_quote};
use serde::Deserialize;
use serde_json::json;
use tokio::sync::{Mutex, RwLock};

use super::WebState;
use crate::member_actor::{
    MemberHandle, MemberState, MemberSummary, MemberType, events::ControlEvent,
};

/// The launch file and arguments a `launch` run came from: what Reload
/// re-resolves and Export starts from. `up` has none.
pub struct LaunchSource {
    pub package_or_path: String,
    pub launch_file: Option<String>,
    pub launch_arguments: Vec<String>,
    pub parser: ros_launch_resolve::verbs::ParserBackend,
    pub contracts: Option<PathBuf>,
    pub no_provider_contracts: bool,
    pub sched: Option<PathBuf>,
    pub target: String,
    pub isolated: bool,
}

impl LaunchSource {
    /// Resolve the launch files as they are now.
    pub async fn resolve(&self) -> eyre::Result<SystemModel> {
        let (dump, launch_path) = ros_launch_resolve::verbs::parse_to_launch_dump(
            &self.package_or_path,
            self.launch_file.as_deref(),
            &self.launch_arguments,
            self.parser,
        )
        .await?;
        ros_launch_resolve::model::build_checked_model(
            ros_launch_resolve::model::ModelBuildInputs {
                dump: &dump,
                launch_path: Some(&launch_path),
                bringup_root: None,
                arg_binding: ros_launch_resolve::verbs::parse_launch_arguments(
                    &self.launch_arguments,
                )
                .into_iter()
                .collect(),
                contracts: self.contracts.as_deref(),
                no_provider_contracts: self.no_provider_contracts,
                sched: self.sched.as_deref(),
                system: None,
                target: self.target.as_str(),
                explain: false,
                process_per_composable: self.isolated,
            },
        )
    }
}

/// Builds the control events that hand changed members their new spawn
/// context, from the reloaded model: `(member id, event)` pairs.
pub type ReplaceFn = Arc<
    dyn Fn(&SystemModel, &HashMap<String, String>, &BTreeSet<String>) -> Vec<(String, ControlEvent)>
        + Send
        + Sync,
>;

pub struct SelectionState {
    pub model: RwLock<Arc<SystemModel>>,
    /// Member id -> model key.
    pub key_of: HashMap<String, String>,
    /// Model key -> member id.
    pub id_of: HashMap<String, String>,
    /// The command the run was started with, selection flags removed.
    pub base_command: Vec<String>,
    pub source: Option<Arc<LaunchSource>>,
    /// The run's log directory (Export writes under it).
    pub log_dir: PathBuf,
    pub replace: Option<ReplaceFn>,
    /// Model keys whose resolution changed at the last Reload and that have
    /// not been restarted since.
    pub changed: Mutex<BTreeSet<String>>,
    /// `play_log/<ts>/selection.jsonl`.
    pub step_log: PathBuf,
    /// One Apply or Reload at a time.
    pub busy: Mutex<()>,
    /// Topics every publisher of which is off, shared with the rule engine.
    pub absent_topics: crate::runtime_enforcement::AbsentTopics,
}

impl SelectionState {
    pub fn new(
        model: Arc<SystemModel>,
        members: &[(String, String)],
        log_dir: &std::path::Path,
        source: Option<Arc<LaunchSource>>,
        replace: Option<ReplaceFn>,
        absent_topics: crate::runtime_enforcement::AbsentTopics,
    ) -> Self {
        let key_of: HashMap<String, String> = members.iter().cloned().collect();
        let id_of = members
            .iter()
            .map(|(i, k)| (k.clone(), i.clone()))
            .collect();
        SelectionState {
            model: RwLock::new(model),
            key_of,
            id_of,
            base_command: base_command(std::env::args().collect()),
            source,
            log_dir: log_dir.to_path_buf(),
            replace,
            changed: Mutex::new(BTreeSet::new()),
            step_log: log_dir.join("selection.jsonl"),
            busy: Mutex::new(()),
            absent_topics,
        }
    }

    fn append_step(&self, line: serde_json::Value) {
        let write = || -> std::io::Result<()> {
            let mut f = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&self.step_log)?;
            writeln!(f, "{line}")
        };
        if let Err(e) = write() {
            tracing::warn!("could not append to {}: {e}", self.step_log.display());
        }
    }
}

/// The run's command line without its selection flags.
fn base_command(argv: Vec<String>) -> Vec<String> {
    let mut out = Vec::new();
    let mut skip = false;
    for a in argv {
        if skip {
            skip = false;
            continue;
        }
        match a.as_str() {
            "--only" | "--exclude" | "--start" => skip = true,
            "--no-deps" => {}
            s if s.starts_with("--only=")
                || s.starts_with("--exclude=")
                || s.starts_with("--start=") => {}
            _ => out.push(a),
        }
    }
    out
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_millis() as u64)
}

fn error(status: StatusCode, msg: impl std::fmt::Display) -> Response {
    (status, Json(json!({ "error": msg.to_string() }))).into_response()
}

fn state_of(web: &WebState) -> Result<&SelectionState, Response> {
    web.selection.as_deref().ok_or_else(|| {
        error(
            StatusCode::NOT_FOUND,
            "this run has no launch tree to switch (started by `run`)",
        )
    })
}

/// Model keys of the members that are on (not Excluded).
fn current_on(sel: &SelectionState, members: &[MemberSummary]) -> HashSet<String> {
    members
        .iter()
        .filter(|m| !m.excluded)
        .filter_map(|m| sel.key_of.get(&m.id).cloned())
        .collect()
}

fn command_for(
    sel: &SelectionState,
    model: &SystemModel,
    on: &HashSet<String>,
) -> (Vec<String>, String) {
    let flags = selection::describe(model, on).to_flags();
    let words: Vec<String> = sel.base_command.iter().map(|w| shell_quote(w)).collect();
    let mut cmd = words.join(" ");
    for f in &flags {
        cmd.push(' ');
        cmd.push_str(f);
    }
    (flags, cmd)
}

pub async fn get_selection(State(web): State<Arc<WebState>>) -> Response {
    let sel = match state_of(&web) {
        Ok(s) => s,
        Err(r) => return r,
    };
    let model = sel.model.read().await.clone();
    let members = web.member_handle.list_members().await;
    let on = current_on(sel, &members);
    let (flags, command) = command_for(sel, &model, &on);
    let changed: Vec<String> = sel
        .changed
        .lock()
        .await
        .iter()
        .filter_map(|k| sel.id_of.get(k).cloned())
        .collect();
    Json(json!({
        "reload": sel.source.is_some(),
        "export": sel.source.is_some(),
        "excluded": web.member_handle.excluded_members(),
        "changed": changed,
        "flags": flags,
        "command": command,
    }))
    .into_response()
}

#[derive(Deserialize, Default)]
pub struct SwitchBody {
    #[serde(default)]
    on: Vec<String>,
    #[serde(default)]
    off: Vec<String>,
    #[serde(default)]
    mode: Option<String>,
}

/// What a pending set would do.
struct Plan {
    sel: Selection,
    start: Vec<String>,
    stop: Vec<String>,
    restart: Vec<String>,
}

async fn plan(web: &WebState, sel: &SelectionState, body: &SwitchBody) -> Result<Plan, Response> {
    let model = sel.model.read().await.clone();
    let members = web.member_handle.list_members().await;
    let current = current_on(sel, &members);
    let key = |id: &String| {
        sel.key_of
            .get(id)
            .cloned()
            .ok_or_else(|| error(StatusCode::BAD_REQUEST, format!("unknown member `{id}`")))
    };
    let on: Vec<String> = body.on.iter().map(key).collect::<Result<_, _>>()?;
    let off: BTreeSet<String> = body.off.iter().map(key).collect::<Result<_, _>>()?;
    let mut target = current.clone();
    target.extend(on.iter().cloned());
    for k in &off {
        target.remove(k);
    }
    let evaluated = selection::evaluate_set(&model, target, &off, false)
        .map_err(|e| error(StatusCode::UNPROCESSABLE_ENTITY, e))?;
    let changed = sel.changed.lock().await.clone();
    let to_id = |k: &String| sel.id_of.get(k).cloned();
    let start = evaluated
        .kept
        .iter()
        .filter(|k| !current.contains(*k))
        .filter_map(to_id)
        .collect();
    let stop = current
        .iter()
        .filter(|k| !evaluated.is_kept(k))
        .filter_map(to_id)
        .collect();
    let restart = on
        .iter()
        .filter(|k| current.contains(*k) && changed.contains(*k))
        .filter_map(to_id)
        .collect();
    Ok(Plan {
        sel: evaluated,
        start,
        stop,
        restart,
    })
}

fn report(sel: &SelectionState, plan: &Plan) -> serde_json::Value {
    let id = |k: &String| sel.id_of.get(k).cloned().unwrap_or_else(|| k.clone());
    json!({
        "start": plan.start,
        "stop": plan.stop,
        "restart": plan.restart,
        "added": plan.sel.added.iter().map(|a| json!({
            "container": id(&a.container),
            "needed_by": a.needed_by.len(),
        })).collect::<Vec<_>>(),
        "cut_inputs": plan.sel.cut_inputs.iter().map(|c| json!({
            "topic": c.topic,
            "subscribers": c.subscribers.len(),
            "publishers": c.publishers,
        })).collect::<Vec<_>>(),
        "warnings": plan.sel.warnings,
    })
}

pub async fn check(State(web): State<Arc<WebState>>, Json(body): Json<SwitchBody>) -> Response {
    let sel = match state_of(&web) {
        Ok(s) => s,
        Err(r) => return r,
    };
    match plan(&web, sel, &body).await {
        Ok(p) => Json(report(sel, &p)).into_response(),
        Err(r) => r,
    }
}

fn by_id<'a>(members: &'a [MemberSummary], id: &str) -> Option<&'a MemberSummary> {
    members.iter().find(|m| m.id == id)
}

/// The member id of a composable's container.
fn container_id(sel: &SelectionState, m: &MemberSummary) -> Option<String> {
    m.target_container
        .as_ref()
        .and_then(|c| sel.id_of.get(c).cloned())
}

fn running(m: Option<&MemberSummary>) -> bool {
    matches!(
        m.map(|m| &m.state),
        Some(MemberState::Running { .. } | MemberState::Respawning { .. })
    )
}

async fn send(h: &MemberHandle, id: &str, ev: ControlEvent) {
    if let Err(e) = h.send_control(id, ev).await {
        tracing::warn!("selection: {id}: {e:#}");
    }
}

/// Switch members off: composables first (unloaded, not auto-loaded again),
/// then plain nodes, then containers. Each is marked Excluded BEFORE its
/// stop, so its exit is neither a crash nor a required node ending the
/// launch.
async fn switch_off(h: &MemberHandle, members: &[MemberSummary], ids: &[String], exclude: bool) {
    let kind = |id: &String| by_id(members, id).map(|m| m.member_type);
    for pass in [
        MemberType::ComposableNode,
        MemberType::Node,
        MemberType::Container,
    ] {
        for id in ids.iter().filter(|i| kind(i) == Some(pass)) {
            if exclude {
                h.set_excluded(id, true);
            }
            if pass == MemberType::ComposableNode {
                send(h, id, ControlEvent::ToggleAutoLoad(false)).await;
            }
            send(h, id, ControlEvent::Stop).await;
        }
    }
}

/// Switch members on: containers first, with the composables to start in
/// them set to auto-load so they load as the container comes up; a
/// composable whose container already runs is loaded into it; plain nodes
/// last.
async fn switch_on(
    h: &MemberHandle,
    sel: &SelectionState,
    members: &[MemberSummary],
    ids: &[String],
    containers_down: &HashSet<String>,
) {
    let kind = |id: &String| by_id(members, id).map(|m| m.member_type);
    for id in ids
        .iter()
        .filter(|i| kind(i) == Some(MemberType::ComposableNode))
    {
        h.set_excluded(id, false);
        send(h, id, ControlEvent::ToggleAutoLoad(true)).await;
    }
    for id in ids
        .iter()
        .filter(|i| kind(i) == Some(MemberType::Container))
    {
        send(h, id, ControlEvent::Start).await;
    }
    for id in ids
        .iter()
        .filter(|i| kind(i) == Some(MemberType::ComposableNode))
    {
        let container = by_id(members, id).and_then(|m| container_id(sel, m));
        let up = container.as_ref().is_some_and(|c| {
            !containers_down.contains(c) && !ids.contains(c) && running(by_id(members, c))
        });
        if up {
            send(h, id, ControlEvent::Start).await;
        }
    }
    for id in ids.iter().filter(|i| kind(i) == Some(MemberType::Node)) {
        send(h, id, ControlEvent::Start).await;
    }
}

/// Wait until none of `ids` is a running process (fresh Apply), at most
/// `limit`.
async fn wait_stopped(h: &MemberHandle, ids: &HashSet<String>, limit: Duration) {
    let deadline = tokio::time::Instant::now() + limit;
    loop {
        let members = h.list_members().await;
        let busy = members
            .iter()
            .filter(|m| ids.contains(&m.id) && m.member_type != MemberType::ComposableNode)
            .any(|m| running(Some(m)));
        if !busy || tokio::time::Instant::now() >= deadline {
            if busy {
                tracing::warn!(
                    "selection: some members had not stopped after {limit:?}; starting anyway"
                );
            }
            return;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

pub async fn apply(State(web): State<Arc<WebState>>, Json(body): Json<SwitchBody>) -> Response {
    let sel = match state_of(&web) {
        Ok(s) => s,
        Err(r) => return r,
    };
    let fresh = match body.mode.as_deref() {
        None | Some("live") => false,
        Some("fresh") => true,
        Some(m) => {
            return error(
                StatusCode::BAD_REQUEST,
                format!("mode `{m}`: use `live` or `fresh`"),
            );
        }
    };
    let _busy = sel.busy.lock().await;
    let plan = match plan(&web, sel, &body).await {
        Ok(p) => p,
        Err(r) => return r,
    };
    let h = &web.member_handle;
    let members = h.list_members().await;
    let model = sel.model.read().await.clone();

    // Changed members get their new context before they start again.
    let restart_keys: BTreeSet<String> = plan
        .restart
        .iter()
        .chain(plan.start.iter())
        .filter_map(|id| sel.key_of.get(id).cloned())
        .collect();
    let mut changed = sel.changed.lock().await;
    let replacing: BTreeSet<String> = restart_keys.intersection(&changed).cloned().collect();
    if let Some(replace) = &sel.replace
        && !replacing.is_empty()
    {
        for (id, ev) in replace(&model, &sel.id_of, &replacing) {
            send(h, &id, ev).await;
        }
    }
    for k in &replacing {
        changed.remove(k);
    }
    drop(changed);

    if fresh {
        // Everything that runs stops; the new set starts from nothing.
        let on_now: Vec<String> = members
            .iter()
            .filter(|m| !m.excluded)
            .map(|m| m.id.clone())
            .collect();
        let leaving: HashSet<&String> = plan.stop.iter().collect();
        let staying: Vec<String> = on_now
            .iter()
            .filter(|i| !leaving.contains(i))
            .cloned()
            .collect();
        switch_off(h, &members, &plan.stop, true).await;
        switch_off(h, &members, &staying, false).await;
        let all: HashSet<String> = on_now.iter().cloned().collect();
        wait_stopped(h, &all, Duration::from_secs(15)).await;
        let members = h.list_members().await;
        let mut target: Vec<String> = staying;
        target.extend(plan.start.iter().cloned());
        let down: HashSet<String> = members
            .iter()
            .filter(|m| m.member_type == MemberType::Container)
            .map(|m| m.id.clone())
            .collect();
        switch_on(h, sel, &members, &target, &down).await;
    } else {
        switch_off(h, &members, &plan.stop, true).await;
        let down: HashSet<String> = plan.stop.iter().cloned().collect();
        switch_on(h, sel, &members, &plan.start, &down).await;
        let restart: Vec<&String> = plan
            .restart
            .iter()
            .filter(|i| {
                by_id(&members, i).is_some_and(|m| m.member_type != MemberType::ComposableNode)
            })
            .collect();
        for id in restart {
            send(h, id, ControlEvent::Restart).await;
        }
        for id in plan.restart.iter().filter(|i| {
            by_id(&members, i).is_some_and(|m| m.member_type == MemberType::ComposableNode)
        }) {
            send(h, id, ControlEvent::Restart).await;
        }
    }

    if let Ok(mut t) = sel.absent_topics.write() {
        *t = plan.sel.absent_topics.iter().cloned().collect();
    }
    let open_issues: Vec<String> = web
        .health
        .as_ref()
        .map(|rx| rx.borrow().open.iter().map(|i| i.key.clone()).collect())
        .unwrap_or_default();
    let on_after: HashSet<String> = plan.sel.kept.iter().cloned().collect();
    sel.append_step(json!({
        "t_ms": now_ms(),
        "step": if fresh { "apply_fresh" } else { "apply_live" },
        "selection": selection::describe(&model, &on_after).to_flags(),
        "started": plan.start,
        "stopped": plan.stop,
        "restarted": plan.restart,
        "cut_inputs": plan.sel.cut_inputs.iter().map(|c| &c.topic).collect::<Vec<_>>(),
        "issues_open_before": open_issues,
    }));
    tracing::info!(
        "selection: applied ({}): started {}, stopped {}, restarted {}",
        if fresh { "fresh" } else { "live" },
        plan.start.len(),
        plan.stop.len(),
        plan.restart.len()
    );
    Json(report(sel, &plan)).into_response()
}

pub async fn reload(State(web): State<Arc<WebState>>) -> Response {
    let sel = match state_of(&web) {
        Ok(s) => s,
        Err(r) => return r,
    };
    let Some(source) = &sel.source else {
        return error(
            StatusCode::CONFLICT,
            "Reload re-reads the launch files, which only `play_launch launch` has; a run of a \
             saved model (`up`) has none",
        );
    };
    let _busy = sel.busy.lock().await;
    let new = match source.resolve().await {
        Ok(m) => Arc::new(m),
        Err(e) => return error(StatusCode::UNPROCESSABLE_ENTITY, format!("{e:#}")),
    };
    let mut model = sel.model.write().await;
    let old = model.clone();
    let mut changed_keys = Vec::new();
    for (k, n) in &new.structure.nodes {
        if let Some(o) = old.structure.nodes.get(k)
            && o != n
        {
            changed_keys.push(k.clone());
        }
    }
    let added: Vec<String> = new
        .structure
        .nodes
        .keys()
        .filter(|k| !old.structure.nodes.contains_key(*k))
        .cloned()
        .collect();
    let removed: Vec<String> = old
        .structure
        .nodes
        .keys()
        .filter(|k| !new.structure.nodes.contains_key(*k))
        .filter_map(|k| sel.id_of.get(k).cloned())
        .collect();
    *model = new;
    drop(model);
    let mut changed = sel.changed.lock().await;
    changed.extend(changed_keys.iter().cloned());
    let changed_ids: Vec<String> = changed
        .iter()
        .filter_map(|k| sel.id_of.get(k).cloned())
        .collect();
    drop(changed);
    sel.append_step(json!({
        "t_ms": now_ms(),
        "step": "reload",
        "changed": changed_keys,
        "added": added,
        "removed": removed,
    }));
    tracing::info!(
        "selection: reloaded the launch files: {} changed, {} new, {} gone",
        changed_keys.len(),
        added.len(),
        removed.len()
    );
    Json(json!({ "changed": changed_ids, "added": added, "removed": removed })).into_response()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base_command_drops_only_the_selection_flags() {
        let argv = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(
            base_command(argv(&[
                "play_launch",
                "launch",
                "p",
                "f.launch.xml",
                "a:=1",
                "--only",
                "x.launch.xml",
                "--exclude=/y",
                "--no-deps",
                "--start",
                "none",
                "--run-for",
                "5s",
            ])),
            argv(&[
                "play_launch",
                "launch",
                "p",
                "f.launch.xml",
                "a:=1",
                "--run-for",
                "5s"
            ])
        );
    }
}

// ---------------------------------------------------------------------------
// Export (W5): the switches as a launch file, written into the run's log
// directory, never to a path the browser names.
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
pub struct ExportBody {
    form: String,
    #[serde(default)]
    name: Option<String>,
}

fn safe_name(name: &str) -> Option<String> {
    let n = name.trim().trim_end_matches(".zip");
    (!n.is_empty()
        && n.len() <= 64
        && n.chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-'))
    .then(|| n.to_string())
}

pub async fn export(State(web): State<Arc<WebState>>, Json(body): Json<ExportBody>) -> Response {
    use ros_launch_resolve::verbs::export::{ExportInputs, Form};
    let sel = match state_of(&web) {
        Ok(s) => s,
        Err(r) => return r,
    };
    let Some(source) = &sel.source else {
        return error(
            StatusCode::CONFLICT,
            "Export starts from the launch files, which only `play_launch launch` has; for a \
             run of a saved model, use `play_launch export --model <system_model.yaml>`",
        );
    };
    let form = match body.form.as_str() {
        "include" => Form::Include,
        "self-contained" => Form::SelfContained,
        "command" => Form::Command,
        f => {
            return error(
                StatusCode::BAD_REQUEST,
                format!("form `{f}`: use include, self-contained or command"),
            );
        }
    };
    let name = match body.name.as_deref().map(safe_name) {
        None => "selection".to_string(),
        Some(Some(n)) => n,
        Some(None) => {
            return error(
                StatusCode::BAD_REQUEST,
                "a name is letters, digits, `_` and `-`, at most 64",
            );
        }
    };
    let _busy = sel.busy.lock().await;
    let model = sel.model.read().await.clone();
    let members = web.member_handle.list_members().await;
    let on = current_on(sel, &members);
    let spec = selection::describe(&model, &on);
    if spec.start_none {
        return error(
            StatusCode::UNPROCESSABLE_ENTITY,
            "nothing is switched on; there is nothing to export",
        );
    }
    let out_dir = sel.log_dir.join("export").join(&name);
    let out_abs = std::path::absolute(&out_dir).unwrap_or(out_dir.clone());
    let mut command = vec!["play_launch".to_string(), "export".to_string()];
    let p = std::path::Path::new(&source.package_or_path);
    command.push(match std::fs::canonicalize(p) {
        Ok(abs) if p.exists() => abs.display().to_string(),
        _ => source.package_or_path.clone(),
    });
    command.extend(source.launch_file.iter().cloned());
    command.extend(source.launch_arguments.iter().cloned());
    for s in &spec.only {
        command.extend(["--only".to_string(), s.to_string()]);
    }
    for s in &spec.exclude {
        command.extend(["--exclude".to_string(), s.to_string()]);
    }
    if spec.no_deps {
        command.push("--no-deps".into());
    }
    command.extend([
        "--form".to_string(),
        form.as_str().to_string(),
        "-o".to_string(),
        out_abs.display().to_string(),
        "--name".to_string(),
        name.clone(),
    ]);
    let inputs = ExportInputs {
        package_or_path: Some(source.package_or_path.clone()),
        launch_file: source.launch_file.clone(),
        launch_arguments: source.launch_arguments.clone(),
        parser: source.parser,
        contracts: source.contracts.clone(),
        no_provider_contracts: source.no_provider_contracts,
        sched: source.sched.clone(),
        target: source.target.clone(),
        process_per_composable: source.isolated,
        model: None,
        selection: spec,
        form,
        out_dir: out_abs,
        name: Some(name.clone()),
        source_command: command,
    };
    match ros_launch_resolve::verbs::export::export(inputs).await {
        Ok(r) => {
            tracing::info!(
                "selection: exported {} ({})",
                r.file.display(),
                form.as_str()
            );
            Json(json!({
                "file": r.file.display().to_string(),
                "run": r.run,
                "lost": r.lost,
                "opened": r.opened,
                "differences": r.differences,
                "zip": format!("/api/selection/export/{name}.zip"),
            }))
            .into_response()
        }
        Err(e) => error(StatusCode::UNPROCESSABLE_ENTITY, format!("{e:#}")),
    }
}

/// `GET /api/selection/export/<name>.zip`: an export directory as a zip.
pub async fn export_zip(
    State(web): State<Arc<WebState>>,
    axum::extract::Path(file): axum::extract::Path<String>,
) -> Response {
    let sel = match state_of(&web) {
        Ok(s) => s,
        Err(r) => return r,
    };
    let Some(name) = safe_name(&file) else {
        return error(StatusCode::BAD_REQUEST, "no such export");
    };
    let dir = sel.log_dir.join("export").join(&name);
    let mut files = Vec::new();
    let mut stack = vec![dir.clone()];
    while let Some(d) = stack.pop() {
        let Ok(rd) = std::fs::read_dir(&d) else {
            continue;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                stack.push(p);
            } else if let (Ok(rel), Ok(data)) = (p.strip_prefix(&dir), std::fs::read(&p)) {
                files.push((format!("{name}/{}", rel.display()), data));
            }
        }
    }
    if files.is_empty() {
        return error(StatusCode::NOT_FOUND, format!("no export named `{name}`"));
    }
    files.sort();
    (
        [
            (
                axum::http::header::CONTENT_TYPE,
                "application/zip".to_string(),
            ),
            (
                axum::http::header::CONTENT_DISPOSITION,
                format!("attachment; filename=\"{name}.zip\""),
            ),
        ],
        stored_zip(&files),
    )
        .into_response()
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// A zip of uncompressed ("stored") entries: a launch file and its
/// parameter files are small, and this needs no compression crate.
fn stored_zip(files: &[(String, Vec<u8>)]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut central = Vec::new();
    for (name, data) in files {
        let crc = crc32(data);
        let offset = out.len() as u32;
        let n = name.as_bytes();
        let len = data.len() as u32;
        let header = |sig: u32, central: bool, buf: &mut Vec<u8>| {
            buf.extend_from_slice(&sig.to_le_bytes());
            if central {
                buf.extend_from_slice(&20u16.to_le_bytes()); // made by
            }
            buf.extend_from_slice(&20u16.to_le_bytes()); // version needed
            buf.extend_from_slice(&0u16.to_le_bytes()); // flags
            buf.extend_from_slice(&0u16.to_le_bytes()); // stored
            buf.extend_from_slice(&0u32.to_le_bytes()); // time, date
            buf.extend_from_slice(&crc.to_le_bytes());
            buf.extend_from_slice(&len.to_le_bytes());
            buf.extend_from_slice(&len.to_le_bytes());
            buf.extend_from_slice(&(n.len() as u16).to_le_bytes());
            buf.extend_from_slice(&0u16.to_le_bytes()); // extra
            if central {
                buf.extend_from_slice(&0u16.to_le_bytes()); // comment
                buf.extend_from_slice(&0u16.to_le_bytes()); // disk
                buf.extend_from_slice(&0u16.to_le_bytes()); // internal attrs
                buf.extend_from_slice(&0u32.to_le_bytes()); // external attrs
                buf.extend_from_slice(&offset.to_le_bytes());
            }
            buf.extend_from_slice(n);
        };
        header(0x0403_4b50, false, &mut out);
        out.extend_from_slice(data);
        header(0x0201_4b50, true, &mut central);
    }
    let cd_offset = out.len() as u32;
    let cd_len = central.len() as u32;
    out.extend_from_slice(&central);
    out.extend_from_slice(&0x0605_4b50u32.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes());
    out.extend_from_slice(&(files.len() as u16).to_le_bytes());
    out.extend_from_slice(&(files.len() as u16).to_le_bytes());
    out.extend_from_slice(&cd_len.to_le_bytes());
    out.extend_from_slice(&cd_offset.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes());
    out
}

#[cfg(test)]
mod zip_tests {
    use super::*;

    #[test]
    fn crc32_matches_the_reference_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn a_stored_zip_is_read_back_by_unzip() {
        let dir = tempfile::TempDir::new().unwrap();
        let zip = dir.path().join("x.zip");
        std::fs::write(
            &zip,
            stored_zip(&[
                ("x/a.launch.xml".into(), b"<launch/>".to_vec()),
                ("x/params/p.yaml".into(), b"a: 1\n".to_vec()),
            ]),
        )
        .unwrap();
        let Ok(out) = std::process::Command::new("unzip")
            .arg("-t")
            .arg(&zip)
            .output()
        else {
            return; // no unzip on this host
        };
        assert!(
            out.status.success(),
            "{}",
            String::from_utf8_lossy(&out.stdout)
        );
    }
}
