//! WasmCompiler: walks IR tree, emits WASM module sections + instructions.

use crate::string_pool::StringPool;
use play_launch_parser::ir::{
    Action, ActionKind, ComposableNodeDecl, Condition, LaunchProgram,
};
use play_launch_wasm_common::{imports, memory, ABI_VERSION, HOST_MODULE};
use wasm_encoder::{
    CodeSection, ConstExpr, DataSection, EntityType, ExportKind, ExportSection, Function,
    FunctionSection, GlobalSection, GlobalType, ImportSection, Instruction, MemorySection,
    MemoryType, Module, TypeSection, ValType,
};

/// Tracks a host import: name, type index, and function index.
struct ImportEntry {
    name: &'static str,
    type_idx: u32,
    func_idx: u32,
}

/// WASM compiler: walks IR tree, emits WASM module.
#[derive(Default)]
pub struct WasmCompiler {
    pub(crate) string_pool: StringPool,
    /// Registered imports: name -> (type_idx, func_idx)
    imports: Vec<ImportEntry>,
    /// Deduplicated function type signatures: (params, results)
    type_signatures: Vec<(Vec<ValType>, Vec<ValType>)>,
    /// Next function index (imports consume indices first)
    next_func_idx: u32,
}

impl WasmCompiler {
    pub fn new() -> Self {
        Self {
            string_pool: StringPool::new(),
            imports: Vec::new(),
            type_signatures: Vec::new(),
            next_func_idx: 0,
        }
    }

    /// Get or create a type index for the given signature.
    fn intern_type(&mut self, params: Vec<ValType>, results: Vec<ValType>) -> u32 {
        for (idx, sig) in self.type_signatures.iter().enumerate() {
            if sig.0 == params && sig.1 == results {
                return idx as u32;
            }
        }
        let idx = self.type_signatures.len() as u32;
        self.type_signatures.push((params, results));
        idx
    }

    /// Register an import function, returning its function index.
    fn register_import(
        &mut self,
        name: &'static str,
        params: Vec<ValType>,
        results: Vec<ValType>,
    ) -> u32 {
        // Check if already registered
        for entry in &self.imports {
            if entry.name == name {
                return entry.func_idx;
            }
        }
        let type_idx = self.intern_type(params, results);
        let func_idx = self.next_func_idx;
        self.next_func_idx += 1;
        self.imports.push(ImportEntry {
            name,
            type_idx,
            func_idx,
        });
        func_idx
    }

    /// Get the function index for an import by name, registering it if needed.
    pub(crate) fn import_func_index(&mut self, name: &'static str) -> u32 {
        // Check if already registered
        for entry in &self.imports {
            if entry.name == name {
                return entry.func_idx;
            }
        }
        // Register with appropriate signature
        let (params, results) = import_signature(name);
        self.register_import(name, params, results)
    }

    /// Compile the entire IR program into a WASM module binary.
    pub fn compile(&mut self, program: &LaunchProgram) -> anyhow::Result<Vec<u8>> {
        // Phase 1: Walk IR to collect all strings
        self.collect_strings(program);

        // Phase 2: Compile actions into instructions (this also registers needed imports)
        let mut instrs = Vec::new();
        for action in &program.body {
            self.compile_action(action, &mut instrs);
        }
        instrs.push(Instruction::End);

        // Phase 3: Build WASM module sections
        let mut module = Module::new();

        // Type section: all signatures used by imports + the plan function
        let plan_type_idx = self.intern_type(vec![], vec![]);
        let mut types = TypeSection::new();
        for (params, results) in &self.type_signatures {
            types.ty().function(params.iter().copied(), results.iter().copied());
        }
        module.section(&types);

        // Import section
        let mut import_section = ImportSection::new();
        for entry in &self.imports {
            import_section.import(
                HOST_MODULE,
                entry.name,
                EntityType::Function(entry.type_idx),
            );
        }
        // Import memory from host? No — we export our own memory.
        module.section(&import_section);

        // Function section: declare the plan function
        let plan_func_idx = self.next_func_idx;
        let mut functions = FunctionSection::new();
        functions.function(plan_type_idx);
        module.section(&functions);

        // Memory section: 1 page minimum, exported
        let mut memories = MemorySection::new();
        // Calculate pages needed: data segment + bump area
        let data_size = self.string_pool.len();
        let min_pages = std::cmp::max(1, (data_size + memory::BUMP_BASE).div_ceil(0x10000) as u64);
        memories.memory(MemoryType {
            minimum: min_pages,
            maximum: None,
            memory64: false,
            shared: false,
            page_size_log2: None,
        });
        module.section(&memories);

        // Global section: $abi_version (immutable i32)
        let mut globals = GlobalSection::new();
        globals.global(
            GlobalType {
                val_type: ValType::I32,
                mutable: false,
                shared: false,
            },
            &ConstExpr::i32_const(ABI_VERSION as i32),
        );
        module.section(&globals);

        // Export section: "memory", "plan", "abi_version"
        let mut exports = ExportSection::new();
        exports.export("memory", ExportKind::Memory, 0);
        exports.export("plan", ExportKind::Func, plan_func_idx);
        exports.export("abi_version", ExportKind::Global, 0);
        module.section(&exports);

        // Code section: plan function body
        let mut code = CodeSection::new();
        let mut func = Function::new(vec![]);
        for instr in &instrs {
            func.instruction(instr);
        }
        code.function(&func);
        module.section(&code);

        // Data section: string pool at offset 0
        let mut data = DataSection::new();
        if !self.string_pool.is_empty() {
            data.active(0, &ConstExpr::i32_const(0), self.string_pool.data().to_vec());
        }
        module.section(&data);

        Ok(module.finish())
    }

    /// Walk the IR tree to collect all string literals into the string pool.
    fn collect_strings(&mut self, program: &LaunchProgram) {
        for action in &program.body {
            self.collect_strings_action(action);
        }
    }

    fn collect_strings_action(&mut self, action: &Action) {
        if let Some(ref cond) = action.condition {
            self.collect_strings_condition(cond);
        }
        match &action.kind {
            ActionKind::DeclareArgument { name, default, .. } => {
                self.string_pool.intern(name);
                if let Some(expr) = default {
                    self.collect_strings_expr(expr);
                }
            }
            ActionKind::SetVariable { name, value } => {
                self.string_pool.intern(name);
                self.collect_strings_expr(value);
            }
            ActionKind::SetEnv { name, value } => {
                self.string_pool.intern(name);
                self.collect_strings_expr(value);
            }
            ActionKind::UnsetEnv { name } => {
                self.string_pool.intern(name);
            }
            ActionKind::PushNamespace { namespace } => {
                self.collect_strings_expr(namespace);
            }
            ActionKind::PopNamespace => {}
            ActionKind::SetParameter { name, value } => {
                self.string_pool.intern(name);
                self.collect_strings_expr(value);
            }
            ActionKind::SetRemap { from, to } => {
                self.collect_strings_expr(from);
                self.collect_strings_expr(to);
            }
            ActionKind::Group { namespace, body } => {
                if let Some(ns) = namespace {
                    self.collect_strings_expr(ns);
                }
                for a in body {
                    self.collect_strings_action(a);
                }
            }
            ActionKind::Include { body, args, .. } => {
                for arg in args {
                    self.string_pool.intern(&arg.name);
                    self.collect_strings_expr(&arg.value);
                }
                if let Some(body) = body {
                    for a in &body.body {
                        self.collect_strings_action(a);
                    }
                }
            }
            ActionKind::SpawnNode {
                package,
                executable,
                name,
                namespace,
                params,
                param_files,
                remaps,
                env,
                args,
                respawn,
                respawn_delay,
            } => {
                self.collect_strings_expr(package);
                self.collect_strings_expr(executable);
                if let Some(n) = name { self.collect_strings_expr(n); }
                if let Some(ns) = namespace { self.collect_strings_expr(ns); }
                for p in params {
                    self.string_pool.intern(&p.name);
                    self.collect_strings_expr(&p.value);
                }
                for pf in param_files { self.collect_strings_expr(pf); }
                for r in remaps {
                    self.collect_strings_expr(&r.from);
                    self.collect_strings_expr(&r.to);
                }
                for e in env {
                    self.collect_strings_expr(&e.name);
                    self.collect_strings_expr(&e.value);
                }
                if let Some(a) = args { self.collect_strings_expr(a); }
                if let Some(r) = respawn { self.collect_strings_expr(r); }
                if let Some(rd) = respawn_delay { self.collect_strings_expr(rd); }
            }
            ActionKind::SpawnExecutable { cmd, name, args, env } => {
                self.collect_strings_expr(cmd);
                if let Some(n) = name { self.collect_strings_expr(n); }
                for a in args { self.collect_strings_expr(a); }
                for e in env {
                    self.collect_strings_expr(&e.name);
                    self.collect_strings_expr(&e.value);
                }
            }
            ActionKind::SpawnContainer { package, executable, name, namespace, args, nodes } => {
                self.collect_strings_expr(package);
                self.collect_strings_expr(executable);
                self.collect_strings_expr(name);
                if let Some(ns) = namespace { self.collect_strings_expr(ns); }
                if let Some(a) = args { self.collect_strings_expr(a); }
                for node in nodes {
                    self.collect_strings_composable_node(node);
                }
            }
            ActionKind::LoadComposableNode { target, nodes } => {
                self.collect_strings_expr(target);
                for node in nodes {
                    self.collect_strings_composable_node(node);
                }
            }
            ActionKind::OpaqueFunction { .. } => {}
        }
    }

    fn collect_strings_composable_node(&mut self, node: &ComposableNodeDecl) {
        self.collect_strings_expr(&node.package);
        self.collect_strings_expr(&node.plugin);
        self.collect_strings_expr(&node.name);
        if let Some(ns) = &node.namespace {
            self.collect_strings_expr(ns);
        }
        for p in &node.params {
            self.string_pool.intern(&p.name);
            self.collect_strings_expr(&p.value);
        }
        for r in &node.remaps {
            self.collect_strings_expr(&r.from);
            self.collect_strings_expr(&r.to);
        }
        for (k, v) in &node.extra_args {
            self.string_pool.intern(k);
            self.string_pool.intern(v);
        }
        if let Some(cond) = &node.condition {
            self.collect_strings_condition(cond);
        }
    }

    fn collect_strings_condition(&mut self, cond: &Condition) {
        match cond {
            Condition::If(expr) | Condition::Unless(expr) => {
                self.collect_strings_expr(expr);
            }
        }
    }

    fn collect_strings_expr(&mut self, expr: &play_launch_parser::ir::Expr) {
        for sub in &expr.0 {
            self.collect_strings_substitution(sub);
        }
    }

    fn collect_strings_substitution(&mut self, sub: &play_launch_parser::substitution::Substitution) {
        match sub {
            Substitution::Text(s) => { self.string_pool.intern(s); }
            Substitution::LaunchConfiguration(parts) => {
                for p in parts { self.collect_strings_substitution(p); }
            }
            Substitution::EnvironmentVariable { name, default } |
            Substitution::OptionalEnvironmentVariable { name, default } => {
                for p in name { self.collect_strings_substitution(p); }
                if let Some(d) = default {
                    for p in d { self.collect_strings_substitution(p); }
                }
            }
            Substitution::Command { cmd, .. } => {
                for p in cmd { self.collect_strings_substitution(p); }
            }
            Substitution::FindPackageShare(parts) => {
                for p in parts { self.collect_strings_substitution(p); }
            }
            Substitution::Dirname => { self.string_pool.intern("__dirname"); }
            Substitution::Filename => { self.string_pool.intern("__filename"); }
            Substitution::Anon(parts) => {
                self.string_pool.intern("__anon_");
                for p in parts { self.collect_strings_substitution(p); }
            }
            Substitution::Eval(parts) => {
                for p in parts { self.collect_strings_substitution(p); }
            }
        }
    }

    // --- Action compilation ---

    fn compile_action(&mut self, action: &Action, instrs: &mut Vec<Instruction<'static>>) {
        // Handle condition wrapping
        if let Some(ref condition) = action.condition {
            self.compile_condition_start(condition, instrs);
            self.compile_action_kind(&action.kind, instrs);
            instrs.push(Instruction::End);
        } else {
            self.compile_action_kind(&action.kind, instrs);
        }
    }

    fn compile_condition_start(
        &mut self,
        condition: &Condition,
        instrs: &mut Vec<Instruction<'static>>,
    ) {
        match condition {
            Condition::If(expr) => {
                self.compile_expr(expr, instrs);
                let func_idx = self.import_func_index(imports::IS_TRUTHY);
                instrs.push(Instruction::Call(func_idx));
                instrs.push(Instruction::If(wasm_encoder::BlockType::Empty));
            }
            Condition::Unless(expr) => {
                self.compile_expr(expr, instrs);
                let func_idx = self.import_func_index(imports::IS_TRUTHY);
                instrs.push(Instruction::Call(func_idx));
                instrs.push(Instruction::I32Eqz);
                instrs.push(Instruction::If(wasm_encoder::BlockType::Empty));
            }
        }
    }

    fn compile_action_kind(&mut self, kind: &ActionKind, instrs: &mut Vec<Instruction<'static>>) {
        match kind {
            ActionKind::DeclareArgument { name, default, .. } => {
                let (name_off, name_len) = self.string_pool.intern(name);
                instrs.push(Instruction::I32Const(name_off as i32));
                instrs.push(Instruction::I32Const(name_len as i32));
                // Default value: if present compile expr, else push sentinel (0, -1)
                if let Some(default_expr) = default {
                    self.compile_expr(default_expr, instrs);
                } else {
                    instrs.push(Instruction::I32Const(0));
                    instrs.push(Instruction::I32Const(-1));
                }
                let func_idx = self.import_func_index(imports::DECLARE_ARG);
                instrs.push(Instruction::Call(func_idx));
            }

            ActionKind::SetVariable { name, value } => {
                let (name_off, name_len) = self.string_pool.intern(name);
                instrs.push(Instruction::I32Const(name_off as i32));
                instrs.push(Instruction::I32Const(name_len as i32));
                self.compile_expr(value, instrs);
                let func_idx = self.import_func_index(imports::SET_VAR);
                instrs.push(Instruction::Call(func_idx));
            }

            ActionKind::SetEnv { name, value } => {
                let (name_off, name_len) = self.string_pool.intern(name);
                instrs.push(Instruction::I32Const(name_off as i32));
                instrs.push(Instruction::I32Const(name_len as i32));
                self.compile_expr(value, instrs);
                let func_idx = self.import_func_index(imports::SET_ENV);
                instrs.push(Instruction::Call(func_idx));
            }

            ActionKind::UnsetEnv { name } => {
                let (name_off, name_len) = self.string_pool.intern(name);
                instrs.push(Instruction::I32Const(name_off as i32));
                instrs.push(Instruction::I32Const(name_len as i32));
                let func_idx = self.import_func_index(imports::UNSET_ENV);
                instrs.push(Instruction::Call(func_idx));
            }

            ActionKind::PushNamespace { namespace } => {
                self.compile_expr(namespace, instrs);
                let func_idx = self.import_func_index(imports::PUSH_NAMESPACE);
                instrs.push(Instruction::Call(func_idx));
            }

            ActionKind::PopNamespace => {
                let func_idx = self.import_func_index(imports::POP_NAMESPACE);
                instrs.push(Instruction::Call(func_idx));
            }

            ActionKind::SetParameter { name, value } => {
                let (name_off, name_len) = self.string_pool.intern(name);
                instrs.push(Instruction::I32Const(name_off as i32));
                instrs.push(Instruction::I32Const(name_len as i32));
                self.compile_expr(value, instrs);
                let func_idx = self.import_func_index(imports::SET_GLOBAL_PARAM);
                instrs.push(Instruction::Call(func_idx));
            }

            ActionKind::SetRemap { from, to } => {
                self.compile_expr(from, instrs);
                self.compile_expr(to, instrs);
                let func_idx = self.import_func_index(imports::SET_REMAP);
                instrs.push(Instruction::Call(func_idx));
            }

            ActionKind::Group { namespace, body } => {
                let save_idx = self.import_func_index(imports::SAVE_SCOPE);
                instrs.push(Instruction::Call(save_idx));

                if let Some(ns_expr) = namespace {
                    self.compile_expr(ns_expr, instrs);
                    let push_idx = self.import_func_index(imports::PUSH_NAMESPACE);
                    instrs.push(Instruction::Call(push_idx));
                }

                for action in body {
                    self.compile_action(action, instrs);
                }

                let restore_idx = self.import_func_index(imports::RESTORE_SCOPE);
                instrs.push(Instruction::Call(restore_idx));
            }

            ActionKind::Include { args, body, .. } => {
                if let Some(body) = body {
                    let save_idx = self.import_func_index(imports::SAVE_SCOPE);
                    instrs.push(Instruction::Call(save_idx));

                    // Set include args
                    for arg in args {
                        let (name_off, name_len) = self.string_pool.intern(&arg.name);
                        instrs.push(Instruction::I32Const(name_off as i32));
                        instrs.push(Instruction::I32Const(name_len as i32));
                        self.compile_expr(&arg.value, instrs);
                        let set_var_idx = self.import_func_index(imports::SET_VAR);
                        instrs.push(Instruction::Call(set_var_idx));
                    }

                    // Compile included body
                    for action in &body.body {
                        self.compile_action(action, instrs);
                    }

                    let restore_idx = self.import_func_index(imports::RESTORE_SCOPE);
                    instrs.push(Instruction::Call(restore_idx));
                }
                // If body is None (Python include), skip — same as OpaqueFunction
            }

            ActionKind::SpawnNode {
                package,
                executable,
                name,
                namespace,
                params,
                param_files,
                remaps,
                env,
                args,
                respawn,
                respawn_delay,
            } => {
                let begin_idx = self.import_func_index(imports::BEGIN_NODE);
                instrs.push(Instruction::Call(begin_idx));

                // Package
                self.compile_expr(package, instrs);
                let set_pkg_idx = self.import_func_index(imports::SET_NODE_PKG);
                instrs.push(Instruction::Call(set_pkg_idx));

                // Executable
                self.compile_expr(executable, instrs);
                let set_exec_idx = self.import_func_index(imports::SET_NODE_EXEC);
                instrs.push(Instruction::Call(set_exec_idx));

                // Name (optional)
                if let Some(name_expr) = name {
                    self.compile_expr(name_expr, instrs);
                    let set_name_idx = self.import_func_index(imports::SET_NODE_NAME);
                    instrs.push(Instruction::Call(set_name_idx));
                }

                // Namespace (optional)
                if let Some(ns_expr) = namespace {
                    self.compile_expr(ns_expr, instrs);
                    let set_ns_idx = self.import_func_index(imports::SET_NODE_NAMESPACE);
                    instrs.push(Instruction::Call(set_ns_idx));
                }

                // Parameters
                for param in params {
                    let (pname_off, pname_len) = self.string_pool.intern(&param.name);
                    instrs.push(Instruction::I32Const(pname_off as i32));
                    instrs.push(Instruction::I32Const(pname_len as i32));
                    self.compile_expr(&param.value, instrs);
                    let add_param_idx = self.import_func_index(imports::ADD_NODE_PARAM);
                    instrs.push(Instruction::Call(add_param_idx));
                }

                // Param files
                for pf in param_files {
                    self.compile_expr(pf, instrs);
                    let add_pf_idx = self.import_func_index(imports::ADD_NODE_PARAM_FILE);
                    instrs.push(Instruction::Call(add_pf_idx));
                }

                // Remaps
                for remap in remaps {
                    self.compile_expr(&remap.from, instrs);
                    self.compile_expr(&remap.to, instrs);
                    let add_remap_idx = self.import_func_index(imports::ADD_NODE_REMAP);
                    instrs.push(Instruction::Call(add_remap_idx));
                }

                // Environment
                for env_decl in env {
                    self.compile_expr(&env_decl.name, instrs);
                    self.compile_expr(&env_decl.value, instrs);
                    let add_env_idx = self.import_func_index(imports::ADD_NODE_ENV);
                    instrs.push(Instruction::Call(add_env_idx));
                }

                // Args
                if let Some(args_expr) = args {
                    self.compile_expr(args_expr, instrs);
                    let set_args_idx = self.import_func_index(imports::SET_NODE_ARGS);
                    instrs.push(Instruction::Call(set_args_idx));
                }

                // Respawn
                if let Some(respawn_expr) = respawn {
                    self.compile_expr(respawn_expr, instrs);
                    let set_respawn_idx = self.import_func_index(imports::SET_NODE_RESPAWN);
                    instrs.push(Instruction::Call(set_respawn_idx));
                }

                // Respawn delay
                if let Some(delay_expr) = respawn_delay {
                    self.compile_expr(delay_expr, instrs);
                    let set_delay_idx = self.import_func_index(imports::SET_NODE_RESPAWN_DELAY);
                    instrs.push(Instruction::Call(set_delay_idx));
                }

                let end_idx = self.import_func_index(imports::END_NODE);
                instrs.push(Instruction::Call(end_idx));
            }

            ActionKind::SpawnExecutable { cmd, name, args, env } => {
                let begin_idx = self.import_func_index(imports::BEGIN_EXECUTABLE);
                instrs.push(Instruction::Call(begin_idx));

                // Cmd
                self.compile_expr(cmd, instrs);
                let set_cmd_idx = self.import_func_index(imports::SET_EXEC_CMD);
                instrs.push(Instruction::Call(set_cmd_idx));

                // Name (optional)
                if let Some(name_expr) = name {
                    self.compile_expr(name_expr, instrs);
                    let set_name_idx = self.import_func_index(imports::SET_EXEC_NAME);
                    instrs.push(Instruction::Call(set_name_idx));
                }

                // Args
                for arg in args {
                    self.compile_expr(arg, instrs);
                    let add_arg_idx = self.import_func_index(imports::ADD_EXEC_ARG);
                    instrs.push(Instruction::Call(add_arg_idx));
                }

                // Environment
                for env_decl in env {
                    self.compile_expr(&env_decl.name, instrs);
                    self.compile_expr(&env_decl.value, instrs);
                    let add_env_idx = self.import_func_index(imports::ADD_EXEC_ENV);
                    instrs.push(Instruction::Call(add_env_idx));
                }

                let end_idx = self.import_func_index(imports::END_EXECUTABLE);
                instrs.push(Instruction::Call(end_idx));
            }

            ActionKind::SpawnContainer {
                package,
                executable,
                name,
                namespace,
                args,
                nodes,
            } => {
                let begin_idx = self.import_func_index(imports::BEGIN_CONTAINER);
                instrs.push(Instruction::Call(begin_idx));

                self.compile_expr(package, instrs);
                let set_pkg_idx = self.import_func_index(imports::SET_CONTAINER_PKG);
                instrs.push(Instruction::Call(set_pkg_idx));

                self.compile_expr(executable, instrs);
                let set_exec_idx = self.import_func_index(imports::SET_CONTAINER_EXEC);
                instrs.push(Instruction::Call(set_exec_idx));

                self.compile_expr(name, instrs);
                let set_name_idx = self.import_func_index(imports::SET_CONTAINER_NAME);
                instrs.push(Instruction::Call(set_name_idx));

                if let Some(ns_expr) = namespace {
                    self.compile_expr(ns_expr, instrs);
                    let set_ns_idx = self.import_func_index(imports::SET_CONTAINER_NAMESPACE);
                    instrs.push(Instruction::Call(set_ns_idx));
                }

                if let Some(args_expr) = args {
                    self.compile_expr(args_expr, instrs);
                    let set_args_idx = self.import_func_index(imports::SET_CONTAINER_ARGS);
                    instrs.push(Instruction::Call(set_args_idx));
                }

                for node in nodes {
                    self.compile_composable_node(node, instrs);
                }

                let end_idx = self.import_func_index(imports::END_CONTAINER);
                instrs.push(Instruction::Call(end_idx));
            }

            ActionKind::LoadComposableNode { target, nodes } => {
                self.compile_expr(target, instrs);
                let begin_idx = self.import_func_index(imports::BEGIN_LOAD_NODE);
                instrs.push(Instruction::Call(begin_idx));

                for node in nodes {
                    self.compile_composable_node(node, instrs);
                }

                let end_idx = self.import_func_index(imports::END_LOAD_NODE);
                instrs.push(Instruction::Call(end_idx));
            }

            ActionKind::OpaqueFunction { .. } => {
                // Skip — nothing to compile
            }
        }
    }

    fn compile_composable_node(
        &mut self,
        node: &ComposableNodeDecl,
        instrs: &mut Vec<Instruction<'static>>,
    ) {
        // Handle condition on composable node
        if let Some(ref condition) = node.condition {
            self.compile_condition_start(condition, instrs);
            self.compile_composable_node_body(node, instrs);
            instrs.push(Instruction::End);
        } else {
            self.compile_composable_node_body(node, instrs);
        }
    }

    fn compile_composable_node_body(
        &mut self,
        node: &ComposableNodeDecl,
        instrs: &mut Vec<Instruction<'static>>,
    ) {
        let begin_idx = self.import_func_index(imports::BEGIN_COMPOSABLE_NODE);
        instrs.push(Instruction::Call(begin_idx));

        self.compile_expr(&node.package, instrs);
        let set_pkg_idx = self.import_func_index(imports::SET_COMP_PKG);
        instrs.push(Instruction::Call(set_pkg_idx));

        self.compile_expr(&node.plugin, instrs);
        let set_plugin_idx = self.import_func_index(imports::SET_COMP_PLUGIN);
        instrs.push(Instruction::Call(set_plugin_idx));

        self.compile_expr(&node.name, instrs);
        let set_name_idx = self.import_func_index(imports::SET_COMP_NAME);
        instrs.push(Instruction::Call(set_name_idx));

        if let Some(ns_expr) = &node.namespace {
            self.compile_expr(ns_expr, instrs);
            let set_ns_idx = self.import_func_index(imports::SET_COMP_NAMESPACE);
            instrs.push(Instruction::Call(set_ns_idx));
        }

        for param in &node.params {
            let (pname_off, pname_len) = self.string_pool.intern(&param.name);
            instrs.push(Instruction::I32Const(pname_off as i32));
            instrs.push(Instruction::I32Const(pname_len as i32));
            self.compile_expr(&param.value, instrs);
            let add_param_idx = self.import_func_index(imports::ADD_COMP_PARAM);
            instrs.push(Instruction::Call(add_param_idx));
        }

        for remap in &node.remaps {
            self.compile_expr(&remap.from, instrs);
            self.compile_expr(&remap.to, instrs);
            let add_remap_idx = self.import_func_index(imports::ADD_COMP_REMAP);
            instrs.push(Instruction::Call(add_remap_idx));
        }

        for (key, value) in &node.extra_args {
            let (k_off, k_len) = self.string_pool.intern(key);
            let (v_off, v_len) = self.string_pool.intern(value);
            instrs.push(Instruction::I32Const(k_off as i32));
            instrs.push(Instruction::I32Const(k_len as i32));
            instrs.push(Instruction::I32Const(v_off as i32));
            instrs.push(Instruction::I32Const(v_len as i32));
            let add_extra_idx = self.import_func_index(imports::ADD_COMP_EXTRA_ARG);
            instrs.push(Instruction::Call(add_extra_idx));
        }

        let end_idx = self.import_func_index(imports::END_COMPOSABLE_NODE);
        instrs.push(Instruction::Call(end_idx));
    }
}

use play_launch_parser::substitution::Substitution;

/// Return the (params, results) signature for a given host import name.
fn import_signature(name: &str) -> (Vec<ValType>, Vec<ValType>) {
    match name {
        // Context operations — no return
        imports::DECLARE_ARG => (vec![ValType::I32; 4], vec![]),       // name_ptr, name_len, default_ptr, default_len
        imports::SET_VAR => (vec![ValType::I32; 4], vec![]),           // name_ptr, name_len, val_ptr, val_len
        imports::SET_ENV => (vec![ValType::I32; 4], vec![]),
        imports::UNSET_ENV => (vec![ValType::I32; 2], vec![]),         // name_ptr, name_len
        imports::PUSH_NAMESPACE => (vec![ValType::I32; 2], vec![]),
        imports::POP_NAMESPACE => (vec![], vec![]),
        imports::SET_GLOBAL_PARAM => (vec![ValType::I32; 4], vec![]),
        imports::SET_REMAP => (vec![ValType::I32; 4], vec![]),
        imports::SAVE_SCOPE => (vec![], vec![]),
        imports::RESTORE_SCOPE => (vec![], vec![]),

        // String-returning functions → (ptr, len) result
        imports::RESOLVE_VAR => (vec![ValType::I32; 2], vec![ValType::I32; 2]),
        imports::FIND_PACKAGE_SHARE => (vec![ValType::I32; 2], vec![ValType::I32; 2]),
        imports::RESOLVE_EXEC_PATH => (vec![ValType::I32; 4], vec![ValType::I32; 2]),
        imports::EVAL_ENV_VAR => (vec![ValType::I32; 4], vec![ValType::I32; 2]),
        imports::EVAL_COMMAND => (vec![ValType::I32; 2], vec![ValType::I32; 2]),
        imports::EVAL_PYTHON_EXPR => (vec![ValType::I32; 2], vec![ValType::I32; 2]),
        imports::CONCAT => (vec![ValType::I32; 4], vec![ValType::I32; 2]),
        imports::STR_EQUALS => (vec![ValType::I32; 4], vec![ValType::I32]),

        // Boolean return
        imports::IS_TRUTHY => (vec![ValType::I32; 2], vec![ValType::I32]),

        // Node builder — no return
        imports::BEGIN_NODE => (vec![], vec![]),
        imports::SET_NODE_PKG => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_EXEC => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_NAME => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_NAMESPACE => (vec![ValType::I32; 2], vec![]),
        imports::ADD_NODE_PARAM => (vec![ValType::I32; 4], vec![]),
        imports::ADD_NODE_PARAM_FILE => (vec![ValType::I32; 2], vec![]),
        imports::ADD_NODE_REMAP => (vec![ValType::I32; 4], vec![]),
        imports::ADD_NODE_ENV => (vec![ValType::I32; 4], vec![]),
        imports::SET_NODE_ARGS => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_RESPAWN => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_RESPAWN_DELAY => (vec![ValType::I32; 2], vec![]),
        imports::END_NODE => (vec![], vec![]),

        // Executable builder
        imports::BEGIN_EXECUTABLE => (vec![], vec![]),
        imports::SET_EXEC_CMD => (vec![ValType::I32; 2], vec![]),
        imports::SET_EXEC_NAME => (vec![ValType::I32; 2], vec![]),
        imports::ADD_EXEC_ARG => (vec![ValType::I32; 2], vec![]),
        imports::ADD_EXEC_ENV => (vec![ValType::I32; 4], vec![]),
        imports::END_EXECUTABLE => (vec![], vec![]),

        // Container builder
        imports::BEGIN_CONTAINER => (vec![], vec![]),
        imports::SET_CONTAINER_PKG => (vec![ValType::I32; 2], vec![]),
        imports::SET_CONTAINER_EXEC => (vec![ValType::I32; 2], vec![]),
        imports::SET_CONTAINER_NAME => (vec![ValType::I32; 2], vec![]),
        imports::SET_CONTAINER_NAMESPACE => (vec![ValType::I32; 2], vec![]),
        imports::SET_CONTAINER_ARGS => (vec![ValType::I32; 2], vec![]),
        imports::BEGIN_COMPOSABLE_NODE => (vec![], vec![]),
        imports::SET_COMP_PKG => (vec![ValType::I32; 2], vec![]),
        imports::SET_COMP_PLUGIN => (vec![ValType::I32; 2], vec![]),
        imports::SET_COMP_NAME => (vec![ValType::I32; 2], vec![]),
        imports::SET_COMP_NAMESPACE => (vec![ValType::I32; 2], vec![]),
        imports::ADD_COMP_PARAM => (vec![ValType::I32; 4], vec![]),
        imports::ADD_COMP_REMAP => (vec![ValType::I32; 4], vec![]),
        imports::ADD_COMP_EXTRA_ARG => (vec![ValType::I32; 4], vec![]),
        imports::END_COMPOSABLE_NODE => (vec![], vec![]),
        imports::END_CONTAINER => (vec![], vec![]),

        // Load composable node
        imports::BEGIN_LOAD_NODE => (vec![ValType::I32; 2], vec![]),
        imports::END_LOAD_NODE => (vec![], vec![]),

        _ => panic!("Unknown import: {name}"),
    }
}
