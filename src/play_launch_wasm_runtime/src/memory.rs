//! Guest memory helpers: read/write strings from/to WASM linear memory.

use crate::host::LaunchHost;
use anyhow::{bail, Result};
use play_launch_wasm_common::NO_VALUE_SENTINEL;
use wasmtime::{AsContext, AsContextMut, Caller, Memory};

/// Get the guest memory from a Caller.
pub fn get_memory(caller: &mut Caller<'_, LaunchHost>) -> Result<Memory> {
    caller
        .get_export("memory")
        .and_then(|e| e.into_memory())
        .ok_or_else(|| anyhow::anyhow!("Missing 'memory' export"))
}

/// Read a UTF-8 string from guest memory at (ptr, len).
pub fn read_guest_string(
    memory: &Memory,
    caller: &Caller<'_, LaunchHost>,
    ptr: i32,
    len: i32,
) -> Result<String> {
    if len < 0 {
        bail!("Negative string length: {len}");
    }
    if len == 0 {
        return Ok(String::new());
    }
    let ptr = ptr as usize;
    let len = len as usize;
    let data = memory.data(caller.as_context());
    if ptr + len > data.len() {
        bail!(
            "String read out of bounds: ptr={ptr}, len={len}, memory_size={}",
            data.len()
        );
    }
    let bytes = &data[ptr..ptr + len];
    Ok(std::str::from_utf8(bytes)?.to_string())
}

/// Read a string if len >= 0, otherwise return None (`NO_VALUE_SENTINEL` means absent).
pub fn read_optional_string(
    memory: &Memory,
    caller: &Caller<'_, LaunchHost>,
    ptr: i32,
    len: i32,
) -> Result<Option<String>> {
    if len == NO_VALUE_SENTINEL {
        return Ok(None);
    }
    Ok(Some(read_guest_string(memory, caller, ptr, len)?))
}

/// Write a string to guest memory at the current bump offset, returning (ptr, len).
/// Must be called with a mutable reference to Caller.
pub fn write_guest_string(
    memory: &Memory,
    caller: &mut Caller<'_, LaunchHost>,
    s: &str,
) -> Result<(i32, i32)> {
    let bytes = s.as_bytes();
    let byte_len = bytes.len() as u32;
    let offset = caller.data().bump_offset;
    let needed = (offset + byte_len) as usize;

    // Grow memory if needed
    let current_size = memory.data_size(caller.as_context());
    if needed > current_size {
        let pages_needed = (needed - current_size).div_ceil(0x10000) as u64;
        memory.grow(caller.as_context_mut(), pages_needed)?;
    }

    let data = memory.data_mut(caller.as_context_mut());
    data[offset as usize..offset as usize + bytes.len()].copy_from_slice(bytes);
    caller.data_mut().bump_offset = offset + byte_len;

    Ok((offset as i32, byte_len as i32))
}
