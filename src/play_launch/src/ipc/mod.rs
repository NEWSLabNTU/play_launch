//! IPC protocol for play_launch <-> play_launch_io_helper communication
//!
//! This crate defines the message protocol used for communication between
//! the main play_launch process and the privileged I/O helper daemon.
//!
//! The protocol uses bincode for efficient binary serialization over
//! Unix domain sockets.

pub mod error;
pub mod protocol;
pub mod sched_protocol;

pub use error::{HelperError, ProcIoError};
pub use protocol::{ProcIoResult, ProcIoStats, Request, Response};
pub use sched_protocol::{SchedRequest, SchedResponse};

/// Current protocol version for compatibility checking
pub const PROTOCOL_VERSION: u32 = 1;

/// Encode a message to bincode bytes with length prefix
pub fn encode_message<T: serde::Serialize>(msg: &T) -> Result<Vec<u8>, bincode::Error> {
    let data = bincode::serialize(msg)?;
    let len = data.len() as u32;

    // Length-prefixed format: [u32 length][payload]
    let mut buf = Vec::with_capacity(4 + data.len());
    buf.extend_from_slice(&len.to_le_bytes());
    buf.extend_from_slice(&data);

    Ok(buf)
}

/// Decode a length-prefixed bincode message
pub fn decode_message<T: serde::de::DeserializeOwned>(buf: &[u8]) -> Result<T, bincode::Error> {
    bincode::deserialize(buf)
}

#[cfg(test)]
mod tests {
    use super::*;
    use protocol::*;

    #[test]
    fn test_encode_decode_request() {
        let req = Request::ReadProcIo { pid: 1234 };
        let encoded = encode_message(&req).unwrap();

        // Skip length prefix (first 4 bytes) for decoding
        let decoded: Request = decode_message(&encoded[4..]).unwrap();

        match decoded {
            Request::ReadProcIo { pid } => assert_eq!(pid, 1234),
            _ => panic!("Wrong request type"),
        }
    }

    #[test]
    fn test_encode_decode_response() {
        let stats = ProcIoStats {
            rchar: 1000,
            wchar: 2000,
            syscr: 10,
            syscw: 20,
            read_bytes: 512,
            write_bytes: 1024,
            cancelled_write_bytes: 0,
        };

        let resp = Response::ProcIo(ProcIoResult {
            pid: 5678,
            result: Ok(stats.clone()),
        });

        let encoded = encode_message(&resp).unwrap();
        let decoded: Response = decode_message(&encoded[4..]).unwrap();

        match decoded {
            Response::ProcIo(result) => {
                assert_eq!(result.pid, 5678);
                let s = result.result.unwrap();
                assert_eq!(s.rchar, 1000);
                assert_eq!(s.wchar, 2000);
            }
            _ => panic!("Wrong response type"),
        }
    }

    #[test]
    fn test_batch_request() {
        let req = Request::ReadProcIoBatch {
            pids: vec![100, 200, 300],
        };

        let encoded = encode_message(&req).unwrap();
        let decoded: Request = decode_message(&encoded[4..]).unwrap();

        match decoded {
            Request::ReadProcIoBatch { pids } => {
                assert_eq!(pids, vec![100, 200, 300]);
            }
            _ => panic!("Wrong request type"),
        }
    }
}
