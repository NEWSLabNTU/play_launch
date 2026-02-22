//! Compiles play_launch IR (LaunchProgram) into WASM binaries.

mod compiler;
mod expr;
mod string_pool;

pub use compiler::WasmCompiler;

use play_launch_parser::ir::LaunchProgram;

/// Compile a LaunchProgram IR into a WASM binary.
pub fn compile_to_wasm(program: &LaunchProgram) -> anyhow::Result<Vec<u8>> {
    let mut compiler = WasmCompiler::new();
    compiler.compile(program)
}
