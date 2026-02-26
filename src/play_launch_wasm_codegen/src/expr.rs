//! Expression compilation: Expr -> WASM instruction sequence leaving (ptr, len) on stack.

use crate::compiler::WasmCompiler;
use play_launch_parser::ir::Expr;
use play_launch_parser::substitution::Substitution;
use play_launch_wasm_common::{imports, NO_VALUE_SENTINEL};
use wasm_encoder::Instruction;

impl WasmCompiler {
    /// Compile an Expr into instructions that leave (ptr: i32, len: i32) on the stack.
    pub(crate) fn compile_expr(&mut self, expr: &Expr, instrs: &mut Vec<Instruction<'static>>) {
        let parts = &expr.parts;
        if parts.is_empty() {
            // Empty expression: push empty string (ptr=0, len=0)
            instrs.push(Instruction::I32Const(0));
            instrs.push(Instruction::I32Const(0));
            return;
        }

        // Compile first part
        self.compile_substitution(&parts[0], instrs);

        // For each subsequent part, compile and concat
        for part in &parts[1..] {
            // Stack has (ptr1, len1) from previous
            self.compile_substitution(part, instrs);
            // Stack: (ptr1, len1, ptr2, len2) -> call $concat -> (ptr, len)
            let func_idx = self.import_func_index(imports::CONCAT);
            instrs.push(Instruction::Call(func_idx));
        }
    }

    /// Compile a single Substitution variant into instructions leaving (ptr, len) on stack.
    fn compile_substitution(
        &mut self,
        sub: &Substitution,
        instrs: &mut Vec<Instruction<'static>>,
    ) {
        match sub {
            Substitution::Text(s) => {
                let (offset, len) = self.string_pool.intern(s);
                instrs.push(Instruction::I32Const(offset as i32));
                instrs.push(Instruction::I32Const(len as i32));
            }

            Substitution::LaunchConfiguration(name_parts) => {
                // Compile inner name expression -> (ptr, len) for name
                self.compile_substitution_parts(name_parts, instrs);
                let func_idx = self.import_func_index(imports::RESOLVE_VAR);
                instrs.push(Instruction::Call(func_idx));
            }

            Substitution::FindPackageShare(pkg_parts) => {
                self.compile_substitution_parts(pkg_parts, instrs);
                let func_idx = self.import_func_index(imports::FIND_PACKAGE_SHARE);
                instrs.push(Instruction::Call(func_idx));
            }

            Substitution::EnvironmentVariable { name, default } => {
                self.compile_substitution_parts(name, instrs);
                // Default: if present compile it, otherwise push (0, -1) as sentinel for "no default"
                if let Some(default_parts) = default {
                    self.compile_substitution_parts(default_parts, instrs);
                } else {
                    instrs.push(Instruction::I32Const(0));
                    instrs.push(Instruction::I32Const(NO_VALUE_SENTINEL));
                }
                let func_idx = self.import_func_index(imports::EVAL_ENV_VAR);
                instrs.push(Instruction::Call(func_idx));
            }

            Substitution::OptionalEnvironmentVariable { name, default } => {
                // Same as EnvironmentVariable but default to empty string if not set
                self.compile_substitution_parts(name, instrs);
                if let Some(default_parts) = default {
                    self.compile_substitution_parts(default_parts, instrs);
                } else {
                    // Default to empty string for optional env var
                    let (offset, len) = self.string_pool.intern("");
                    instrs.push(Instruction::I32Const(offset as i32));
                    instrs.push(Instruction::I32Const(len as i32));
                }
                let func_idx = self.import_func_index(imports::EVAL_ENV_VAR);
                instrs.push(Instruction::Call(func_idx));
            }

            Substitution::Command { cmd, .. } => {
                self.compile_substitution_parts(cmd, instrs);
                let func_idx = self.import_func_index(imports::EVAL_COMMAND);
                instrs.push(Instruction::Call(func_idx));
            }

            Substitution::Eval(expr_parts) => {
                self.compile_substitution_parts(expr_parts, instrs);
                let func_idx = self.import_func_index(imports::EVAL_PYTHON_EXPR);
                instrs.push(Instruction::Call(func_idx));
            }

            Substitution::Dirname => {
                // Resolve special "__dirname" variable
                let (offset, len) = self.string_pool.intern("__dirname");
                instrs.push(Instruction::I32Const(offset as i32));
                instrs.push(Instruction::I32Const(len as i32));
                let func_idx = self.import_func_index(imports::RESOLVE_VAR);
                instrs.push(Instruction::Call(func_idx));
            }

            Substitution::Filename => {
                let (offset, len) = self.string_pool.intern("__filename");
                instrs.push(Instruction::I32Const(offset as i32));
                instrs.push(Instruction::I32Const(len as i32));
                let func_idx = self.import_func_index(imports::RESOLVE_VAR);
                instrs.push(Instruction::Call(func_idx));
            }

            Substitution::Anon(name_parts) => {
                // Anon names: compile name, then resolve with "__anon_" prefix
                // The host will handle anonymous name generation
                let (prefix_off, prefix_len) = self.string_pool.intern("__anon_");
                instrs.push(Instruction::I32Const(prefix_off as i32));
                instrs.push(Instruction::I32Const(prefix_len as i32));
                self.compile_substitution_parts(name_parts, instrs);
                let concat_idx = self.import_func_index(imports::CONCAT);
                instrs.push(Instruction::Call(concat_idx));
                let resolve_idx = self.import_func_index(imports::RESOLVE_VAR);
                instrs.push(Instruction::Call(resolve_idx));
            }
        }
    }

    /// Compile a Vec<Substitution> (inner parts of compound substitutions) into
    /// instructions that leave (ptr, len) on stack.
    fn compile_substitution_parts(
        &mut self,
        parts: &[Substitution],
        instrs: &mut Vec<Instruction<'static>>,
    ) {
        if parts.is_empty() {
            instrs.push(Instruction::I32Const(0));
            instrs.push(Instruction::I32Const(0));
            return;
        }

        self.compile_substitution(&parts[0], instrs);

        for part in &parts[1..] {
            self.compile_substitution(part, instrs);
            let func_idx = self.import_func_index(imports::CONCAT);
            instrs.push(Instruction::Call(func_idx));
        }
    }
}
