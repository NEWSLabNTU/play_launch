//! IPC protocol for play_launch <-> play_launch_rt_helper communication
//!
//! This is a deliberately SEPARATE request/response protocol from
//! `ipc::protocol` (used by `play_launch_io_helper`). The two helper
//! binaries are capped with different, independent Linux capabilities
//! (`CAP_SYS_PTRACE` for the I/O helper, `CAP_SYS_NICE` for this one), so
//! their wire protocols must not share an enum — that would entangle two
//! binaries that otherwise have nothing to do with each other.
//!
//! Framing is shared: see `ipc::{encode_message, decode_message}`.

use crate::sched::{AppliedTier, SchedApplyError};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum SchedRequest {
    /// Apply the tier to EVERY thread of `pid` (see `sched::apply_tier`).
    ApplySched { pid: u32, tier: AppliedTier },
    Ping,
    Shutdown,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum SchedResponse {
    Applied(Result<(), SchedApplyError>),
    Pong,
    ShutdownAck,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ipc::{decode_message, encode_message};
    use crate::sched::SchedPolicy;

    fn roundtrip_request(req: &SchedRequest) {
        let encoded = encode_message(req).unwrap();
        // Strip the 4-byte LE length prefix before decoding.
        let decoded: SchedRequest = decode_message(&encoded[4..]).unwrap();
        assert_eq!(&decoded, req);
    }

    fn roundtrip_response(resp: &SchedResponse) {
        let encoded = encode_message(resp).unwrap();
        let decoded: SchedResponse = decode_message(&encoded[4..]).unwrap();
        assert_eq!(&decoded, resp);
    }

    #[test]
    fn apply_sched_request_roundtrips() {
        roundtrip_request(&SchedRequest::ApplySched {
            pid: 1234,
            tier: AppliedTier {
                policy: SchedPolicy::Fifo,
                priority: 42,
                core: Some(3),
                tier_name: "rt-critical".to_string(),
            },
        });
    }

    #[test]
    fn ping_request_roundtrips() {
        roundtrip_request(&SchedRequest::Ping);
    }

    #[test]
    fn shutdown_request_roundtrips() {
        roundtrip_request(&SchedRequest::Shutdown);
    }

    #[test]
    fn applied_ok_response_roundtrips() {
        roundtrip_response(&SchedResponse::Applied(Ok(())));
    }

    #[test]
    fn applied_err_response_roundtrips() {
        // Proves the error type itself crosses the wire intact, not just
        // the Ok(()) happy path.
        roundtrip_response(&SchedResponse::Applied(Err(
            SchedApplyError::PermissionDenied { pid: 5678 },
        )));
    }

    #[test]
    fn pong_response_roundtrips() {
        roundtrip_response(&SchedResponse::Pong);
    }

    #[test]
    fn shutdown_ack_response_roundtrips() {
        roundtrip_response(&SchedResponse::ShutdownAck);
    }
}
