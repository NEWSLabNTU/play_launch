//! Resolves the byte offset of `header.stamp` within a ROS message using
//! rosidl introspection. Called once per publisher/subscriber (cold path).

use std::ffi::{c_char, CStr};

use rcl_interception_sys::*;

/// Fully-qualified type name of `builtin_interfaces/msg/Time`, used to guard
/// the bare top-level `stamp` fallback (see `find_bare_stamp_offset`) against
/// false positives — a field merely *named* `stamp` that isn't actually a
/// `Time` must not be mistaken for a timestamp.
const TIME_TYPE_IDENTITY: &str = "builtin_interfaces/msg/Time";

/// Walk a `MessageMembers` array looking for a field named "header" with
/// `type_id_ == ROS_TYPE_MESSAGE`. Returns its byte offset within the message
/// struct, or `None` if no such field exists.
///
/// # Safety
///
/// `members` must point to a valid `MessageMembers` whose `members_` array
/// has `member_count_` valid entries.
unsafe fn find_header_offset(members: *const MessageMembers) -> Option<usize> {
    let members = unsafe { &*members };
    if members.members_.is_null() || members.member_count_ == 0 {
        return None;
    }
    let slice =
        unsafe { std::slice::from_raw_parts(members.members_, members.member_count_ as usize) };
    for member in slice {
        if member.name_.is_null() {
            continue;
        }
        let name = unsafe { CStr::from_ptr(member.name_) };
        if name.to_bytes() == b"header" && member.type_id_ == ROS_TYPE_MESSAGE {
            return Some(member.offset_ as usize);
        }
    }
    None
}

/// Walk a `MessageMembers` array looking for a *top-level* field named
/// "stamp" whose type is exactly `builtin_interfaces/msg/Time`. This covers
/// messages that carry a bare `stamp` field as their first member instead of
/// a nested `header.stamp` (e.g. `autoware_control_msgs/Control` and other
/// vehicle command/report types) — `find_header_offset` alone misses these,
/// so the frontier plugin never observes a stamp for them.
///
/// Returns the byte offset of the `Time` struct within the message (i.e. the
/// offset to read `sec`/`nanosec` from, same convention as the header path:
/// `Time` is `{sec: i32, nanosec: u32}` at offset 0 of itself).
///
/// The type check (not just the name) guards against false positives: a
/// field merely named `stamp` that isn't a `builtin_interfaces/msg/Time`
/// (e.g. a `string stamp` or an unrelated message type) is skipped.
///
/// # Safety
///
/// Same preconditions as `find_header_offset`.
unsafe fn find_bare_stamp_offset(members: *const MessageMembers) -> Option<usize> {
    let members = unsafe { &*members };
    if members.members_.is_null() || members.member_count_ == 0 {
        return None;
    }
    let slice =
        unsafe { std::slice::from_raw_parts(members.members_, members.member_count_ as usize) };
    for member in slice {
        if member.name_.is_null() {
            continue;
        }
        let name = unsafe { CStr::from_ptr(member.name_) };
        if name.to_bytes() != b"stamp" || member.type_id_ != ROS_TYPE_MESSAGE {
            continue;
        }
        if member.members_.is_null() {
            continue;
        }
        let Some(identity) = (unsafe { find_type_identity(member.members_) }) else {
            continue;
        };
        if identity == TIME_TYPE_IDENTITY {
            return Some(member.offset_ as usize);
        }
    }
    None
}

/// Extract the message type identity as `"pkg/msg/Name"` from
/// `type_support`. Used by the consistency-runtime rule (Phase 36) to
/// compare the actual runtime type against the manifest's declared
/// `type:`. Returns `None` if introspection isn't available.
///
/// # Safety
///
/// Same preconditions as [`find_stamp_offset`].
pub unsafe fn find_type_identity(
    type_support: *const rosidl_message_type_support_t,
) -> Option<String> {
    unsafe {
        try_type_identity(type_support, TYPESUPPORT_INTROSPECTION_C_IDENTIFIER).or_else(|| {
            try_type_identity(type_support, TYPESUPPORT_INTROSPECTION_CPP_IDENTIFIER)
        })
    }
}

/// Read `message_namespace_` and `message_name_` from the introspection
/// MessageMembers struct via the given identifier.
///
/// # Safety
///
/// `type_support` must be a valid, non-null pointer.
unsafe fn try_type_identity(
    type_support: *const rosidl_message_type_support_t,
    identifier: &[u8],
) -> Option<String> {
    let ts = unsafe { &*type_support };
    let func = ts.func?;
    let handle = unsafe { func(type_support, identifier.as_ptr() as *const c_char) };
    if handle.is_null() {
        return None;
    }
    let data = unsafe { (*handle).data };
    if data.is_null() {
        return None;
    }
    let members = unsafe { &*(data as *const MessageMembers) };
    if members.message_namespace_.is_null() || members.message_name_.is_null() {
        return None;
    }
    let ns_bytes = unsafe { CStr::from_ptr(members.message_namespace_) }.to_bytes();
    let name_bytes = unsafe { CStr::from_ptr(members.message_name_) }.to_bytes();

    // The namespace from introspection uses "__" (C identifier) or "::"
    // (C++ identifier) instead of "/" — e.g. "std_msgs__msg" or
    // "std_msgs::msg" both → "std_msgs/msg". Normalise back to the
    // canonical "pkg/msg/Name" form used in manifests.
    let ns = String::from_utf8_lossy(ns_bytes)
        .replace("__", "/")
        .replace("::", "/");
    let name = String::from_utf8_lossy(name_bytes);
    Some(format!("{ns}/{name}"))
}

/// Resolve the byte offset of the message's timestamp (either nested
/// `header.stamp` or a bare top-level `stamp` field) within messages
/// published or received through `type_support`.
///
/// 1. Calls `type_support->func(type_support, "rosidl_typesupport_introspection_c")`
///    to obtain the introspection handle.
/// 2. Casts `handle->data` to `MessageMembers*`.
/// 3. Finds the "header" member with `type_id_ == ROS_TYPE_MESSAGE` and
///    returns `header.offset_` — the stamp lives at offset 0 within Header.
/// 4. If no "header" field exists, falls back to a top-level field named
///    "stamp" whose type is exactly `builtin_interfaces/msg/Time` (e.g.
///    `autoware_control_msgs/Control` and other vehicle command/report
///    types carry a bare `stamp` as their first field instead of a
///    `std_msgs/Header`). Returns that field's offset.
///
/// Returns `None` for messages without a `header` field or a `Time`-typed
/// `stamp` field (e.g. `std_msgs/String`).
///
/// # Safety
///
/// `type_support` must be a valid, non-null pointer to a `rosidl_message_type_support_t`.
pub unsafe fn find_stamp_offset(
    type_support: *const rosidl_message_type_support_t,
) -> Option<usize> {
    // Try both C and C++ introspection identifiers. C++ nodes typically
    // register with the C++ typesupport, so the C identifier alone won't
    // resolve. The MessageMembers/MessageMember structs have identical
    // memory layout across both.
    unsafe {
        try_introspection(type_support, TYPESUPPORT_INTROSPECTION_C_IDENTIFIER)
            .or_else(|| {
                try_introspection(type_support, TYPESUPPORT_INTROSPECTION_CPP_IDENTIFIER)
            })
    }
}

/// Attempt to obtain `MessageMembers` via a specific introspection identifier,
/// then find the header offset.
///
/// # Safety
///
/// `type_support` must be a valid, non-null pointer.
unsafe fn try_introspection(
    type_support: *const rosidl_message_type_support_t,
    identifier: &[u8],
) -> Option<usize> {
    let ts = unsafe { &*type_support };
    let func = ts.func?;

    let handle = unsafe { func(type_support, identifier.as_ptr() as *const c_char) };
    if handle.is_null() {
        return None;
    }

    let data = unsafe { (*handle).data };
    if data.is_null() {
        return None;
    }

    let members = data as *const MessageMembers;
    unsafe { find_header_offset(members) }.or_else(|| unsafe { find_bare_stamp_offset(members) })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_header_offset() {
        let header_name = c"header";
        let data_name = c"data";

        unsafe {
            let mut members: [MessageMember; 2] = std::mem::zeroed();

            // First member: "data" (string type, not a message)
            members[0].name_ = data_name.as_ptr();
            members[0].type_id_ = 16; // ROS_TYPE_STRING
            members[0].offset_ = 0;

            // Second member: "header" (message type)
            members[1].name_ = header_name.as_ptr();
            members[1].type_id_ = ROS_TYPE_MESSAGE;
            members[1].offset_ = 24;

            let mut msg_members: MessageMembers = std::mem::zeroed();
            msg_members.member_count_ = 2;
            msg_members.members_ = members.as_ptr();

            assert_eq!(
                find_header_offset(&msg_members as *const MessageMembers),
                Some(24)
            );
        }
    }

    #[test]
    fn returns_none_without_header() {
        let data_name = c"data";

        unsafe {
            let mut members: [MessageMember; 1] = std::mem::zeroed();
            members[0].name_ = data_name.as_ptr();
            members[0].type_id_ = 16; // ROS_TYPE_STRING
            members[0].offset_ = 0;

            let mut msg_members: MessageMembers = std::mem::zeroed();
            msg_members.member_count_ = 1;
            msg_members.members_ = members.as_ptr();

            assert_eq!(
                find_header_offset(&msg_members as *const MessageMembers),
                None
            );
        }
    }

    #[test]
    fn returns_none_for_empty_members() {
        unsafe {
            let mut msg_members: MessageMembers = std::mem::zeroed();
            msg_members.member_count_ = 0;
            msg_members.members_ = std::ptr::null();

            assert_eq!(
                find_header_offset(&msg_members as *const MessageMembers),
                None
            );
        }
    }

    // -----------------------------------------------------------------------
    // Bare top-level `stamp` fallback (autoware_control_msgs/Control etc.)
    // -----------------------------------------------------------------------

    /// A `func` that always hands back the same `rosidl_message_type_support_t`
    /// it was given, regardless of the requested identifier — enough to
    /// stand in for the real introspection dispatch in tests, since we only
    /// ever need `(*handle).data` to resolve.
    unsafe extern "C" fn return_self(
        ts: *const rosidl_message_type_support_t,
        _identifier: *const c_char,
    ) -> *const rosidl_message_type_support_t {
        ts
    }

    /// Build a `rosidl_message_type_support_t` whose introspection resolves
    /// to a message named `namespace_/name_` with no fields — enough for
    /// `find_type_identity` to read off a `"pkg/msg/Name"` string.
    fn make_type_support(
        members: &MessageMembers,
        ts_storage: &mut rosidl_message_type_support_t,
    ) -> *const rosidl_message_type_support_t {
        ts_storage.typesupport_identifier = std::ptr::null();
        ts_storage.data = members as *const MessageMembers as *const std::ffi::c_void;
        ts_storage.func = Some(return_self);
        ts_storage as *const rosidl_message_type_support_t
    }

    #[test]
    fn finds_bare_stamp_offset_when_typed_time() {
        let ns = c"builtin_interfaces__msg";
        let name = c"Time";
        let stamp_name = c"stamp";
        let frame_id_name = c"frame_id";

        unsafe {
            // The `Time` message's own MessageMembers (namespace/name only —
            // find_type_identity doesn't look at member fields).
            let mut time_members: MessageMembers = std::mem::zeroed();
            time_members.message_namespace_ = ns.as_ptr();
            time_members.message_name_ = name.as_ptr();

            let mut time_ts: rosidl_message_type_support_t = std::mem::zeroed();
            let time_ts_ptr = make_type_support(&time_members, &mut time_ts);

            // Top-level message with a bare `stamp: builtin_interfaces/Time`
            // field followed by an unrelated `frame_id` field.
            let mut fields: [MessageMember; 2] = std::mem::zeroed();
            fields[0].name_ = stamp_name.as_ptr();
            fields[0].type_id_ = ROS_TYPE_MESSAGE;
            fields[0].offset_ = 0;
            fields[0].members_ = time_ts_ptr;

            fields[1].name_ = frame_id_name.as_ptr();
            fields[1].type_id_ = 16; // ROS_TYPE_STRING
            fields[1].offset_ = 8;

            let mut msg_members: MessageMembers = std::mem::zeroed();
            msg_members.member_count_ = 2;
            msg_members.members_ = fields.as_ptr();

            assert_eq!(
                find_bare_stamp_offset(&msg_members as *const MessageMembers),
                Some(0)
            );
        }
    }

    #[test]
    fn ignores_stamp_field_with_wrong_type() {
        let stamp_name = c"stamp";

        unsafe {
            // A field named "stamp" that is a plain string, not a Time —
            // must not be mistaken for a timestamp.
            let mut fields: [MessageMember; 1] = std::mem::zeroed();
            fields[0].name_ = stamp_name.as_ptr();
            fields[0].type_id_ = 16; // ROS_TYPE_STRING
            fields[0].offset_ = 0;

            let mut msg_members: MessageMembers = std::mem::zeroed();
            msg_members.member_count_ = 1;
            msg_members.members_ = fields.as_ptr();

            assert_eq!(
                find_bare_stamp_offset(&msg_members as *const MessageMembers),
                None
            );
        }
    }

    #[test]
    fn ignores_stamp_message_field_of_non_time_type() {
        let ns = c"my_pkg__msg";
        let name = c"NotTime";
        let stamp_name = c"stamp";

        unsafe {
            let mut other_members: MessageMembers = std::mem::zeroed();
            other_members.message_namespace_ = ns.as_ptr();
            other_members.message_name_ = name.as_ptr();

            let mut other_ts: rosidl_message_type_support_t = std::mem::zeroed();
            let other_ts_ptr = make_type_support(&other_members, &mut other_ts);

            // A field named "stamp" whose message type is NOT
            // builtin_interfaces/Time — must be rejected.
            let mut fields: [MessageMember; 1] = std::mem::zeroed();
            fields[0].name_ = stamp_name.as_ptr();
            fields[0].type_id_ = ROS_TYPE_MESSAGE;
            fields[0].offset_ = 0;
            fields[0].members_ = other_ts_ptr;

            let mut msg_members: MessageMembers = std::mem::zeroed();
            msg_members.member_count_ = 1;
            msg_members.members_ = fields.as_ptr();

            assert_eq!(
                find_bare_stamp_offset(&msg_members as *const MessageMembers),
                None
            );
        }
    }

    #[test]
    fn header_offset_takes_priority_over_bare_stamp() {
        // A message that (hypothetically) has both a "header" field and a
        // top-level "stamp" field should resolve via the header path first
        // — `try_introspection`'s `.or_else` only falls back when the
        // header lookup returns `None`. This test exercises
        // `find_header_offset` directly to document that priority; the
        // actual short-circuit is in `try_introspection`.
        let header_name = c"header";

        unsafe {
            let mut fields: [MessageMember; 1] = std::mem::zeroed();
            fields[0].name_ = header_name.as_ptr();
            fields[0].type_id_ = ROS_TYPE_MESSAGE;
            fields[0].offset_ = 4;

            let mut msg_members: MessageMembers = std::mem::zeroed();
            msg_members.member_count_ = 1;
            msg_members.members_ = fields.as_ptr();

            assert_eq!(
                find_header_offset(&msg_members as *const MessageMembers),
                Some(4)
            );
        }
    }
}
