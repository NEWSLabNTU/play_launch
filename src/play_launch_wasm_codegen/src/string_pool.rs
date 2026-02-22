//! String pool: collects and deduplicates string literals into a data segment.

use std::collections::HashMap;

/// Packed string data with deduplication for WASM data segments.
#[derive(Default)]
pub struct StringPool {
    data: Vec<u8>,
    offsets: HashMap<String, (u32, u32)>,
}

impl StringPool {
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
            offsets: HashMap::new(),
        }
    }

    /// Intern a string, returning (offset, len). Deduplicates identical strings.
    pub fn intern(&mut self, s: &str) -> (u32, u32) {
        if let Some(&entry) = self.offsets.get(s) {
            return entry;
        }
        let offset = self.data.len() as u32;
        let len = s.len() as u32;
        self.data.extend_from_slice(s.as_bytes());
        self.offsets.insert(s.to_string(), (offset, len));
        (offset, len)
    }

    /// Raw bytes for the data segment.
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    /// Total data segment size.
    pub fn len(&self) -> u32 {
        self.data.len() as u32
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_intern_deduplicates() {
        let mut pool = StringPool::new();
        let (off1, len1) = pool.intern("hello");
        let (off2, len2) = pool.intern("hello");
        assert_eq!((off1, len1), (off2, len2));
        assert_eq!(pool.len(), 5);
    }

    #[test]
    fn test_intern_different_strings() {
        let mut pool = StringPool::new();
        let (off1, _) = pool.intern("abc");
        let (off2, _) = pool.intern("def");
        assert_eq!(off1, 0);
        assert_eq!(off2, 3);
        assert_eq!(pool.len(), 6);
    }

    #[test]
    fn test_empty_string() {
        let mut pool = StringPool::new();
        let (off, len) = pool.intern("");
        assert_eq!(off, 0);
        assert_eq!(len, 0);
    }
}
