//! Resolves the byte offset of `header.stamp` within a ROS message using
//! rosidl introspection. Called once per publisher/subscriber (cold path).

use std::ffi::{c_char, CStr};

use rcl_interception_sys::*;

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

/// Resolve the byte offset of `header.stamp` within messages published or
/// received through `type_support`.
///
/// 1. Calls `type_support->func(type_support, "rosidl_typesupport_introspection_c")`
///    to obtain the introspection handle.
/// 2. Casts `handle->data` to `MessageMembers*`.
/// 3. Finds the "header" member with `type_id_ == ROS_TYPE_MESSAGE`.
/// 4. Returns `header.offset_` — the stamp lives at offset 0 within Header.
///
/// Returns `None` for messages without a `header` field (e.g. `std_msgs/String`).
///
/// # Safety
///
/// `type_support` must be a valid, non-null pointer to a `rosidl_message_type_support_t`.
pub unsafe fn find_stamp_offset(
    type_support: *const rosidl_message_type_support_t,
) -> Option<usize> {
    let ts = unsafe { &*type_support };
    let func = ts.func?;

    let introspection_handle = unsafe {
        func(
            type_support,
            TYPESUPPORT_INTROSPECTION_C_IDENTIFIER.as_ptr() as *const c_char,
        )
    };
    if introspection_handle.is_null() {
        return None;
    }

    let data = unsafe { (*introspection_handle).data };
    if data.is_null() {
        return None;
    }

    unsafe { find_header_offset(data as *const MessageMembers) }
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
}
