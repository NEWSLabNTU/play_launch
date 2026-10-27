//! FFI for the serialized and loaned message paths, and for measuring a
//! message's size.
//!
//! Kept here rather than in `rcl_interception_sys` because nothing outside
//! this crate uses it, and every signature is plain C taken from the Humble
//! headers (unchanged in Jazzy): `rcl/publisher.h`, `rcl/subscription.h`,
//! `rmw/rmw.h` and `rcutils/types/uint8_array.h`.

#![allow(non_camel_case_types)]

use std::ffi::c_void;

use rcl_interception_sys::{
    fn_types::rcl_ret_t,
    opaque::{
        rcl_publisher_t, rcl_subscription_t, rcutils_allocator_t, rmw_message_info_t,
        rmw_publisher_allocation_t, rmw_subscription_allocation_t,
    },
    rosidl_message_type_support_t,
};

/// `rcutils_uint8_array_t`, which is also `rmw_serialized_message_t` and
/// `rcl_serialized_message_t`. Layout-aware: the hooks read `buffer_length`,
/// and the size sampler owns one on its stack.
#[repr(C)]
pub struct rcutils_uint8_array_t {
    pub buffer: *mut u8,
    pub buffer_length: usize,
    pub buffer_capacity: usize,
    pub allocator: rcutils_allocator_t,
}

/// `rcl_publish_serialized_message(publisher, serialized_message, allocation)`
pub type FnRclPublishSerialized = unsafe extern "C" fn(
    *const rcl_publisher_t,
    *const rcutils_uint8_array_t,
    *mut rmw_publisher_allocation_t,
) -> rcl_ret_t;

/// `rcl_publish_loaned_message(publisher, ros_message, allocation)`
pub type FnRclPublishLoaned = unsafe extern "C" fn(
    *const rcl_publisher_t,
    *mut c_void,
    *mut rmw_publisher_allocation_t,
) -> rcl_ret_t;

/// `rcl_take_serialized_message(subscription, serialized_message, message_info, allocation)`
pub type FnRclTakeSerialized = unsafe extern "C" fn(
    *const rcl_subscription_t,
    *mut rcutils_uint8_array_t,
    *mut rmw_message_info_t,
    *mut rmw_subscription_allocation_t,
) -> rcl_ret_t;

/// `rcl_take_loaned_message(subscription, loaned_message, message_info, allocation)`
pub type FnRclTakeLoaned = unsafe extern "C" fn(
    *const rcl_subscription_t,
    *mut *mut c_void,
    *mut rmw_message_info_t,
    *mut rmw_subscription_allocation_t,
) -> rcl_ret_t;

/// `rmw_serialize(ros_message, type_support, serialized_message) -> rmw_ret_t`
pub type FnRmwSerialize = unsafe extern "C" fn(
    *const c_void,
    *const rosidl_message_type_support_t,
    *mut rcutils_uint8_array_t,
) -> i32;

/// `rcl_publisher_get_actual_qos(publisher) -> const rmw_qos_profile_t *`: the
/// profile the middleware actually applied, `SYSTEM_DEFAULT`s resolved.
pub type FnRclPublisherGetActualQos =
    unsafe extern "C" fn(*const rcl_publisher_t) -> *const rcl_interception_sys::qos::RmwQosProfile;

/// `rcl_subscription_get_actual_qos(subscription) -> const rmw_qos_profile_t *`
pub type FnRclSubscriptionGetActualQos =
    unsafe extern "C" fn(
        *const rcl_subscription_t,
    ) -> *const rcl_interception_sys::qos::RmwQosProfile;

/// `rcutils_uint8_array_init(array, capacity, allocator) -> rcutils_ret_t`
pub type FnRcutilsUint8ArrayInit =
    unsafe extern "C" fn(*mut rcutils_uint8_array_t, usize, *const rcutils_allocator_t) -> i32;

/// `rcutils_uint8_array_fini(array) -> rcutils_ret_t`
pub type FnRcutilsUint8ArrayFini = unsafe extern "C" fn(*mut rcutils_uint8_array_t) -> i32;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uint8_array_layout_matches_rcutils() {
        // buffer, buffer_length, buffer_capacity, then the 40-byte allocator.
        assert_eq!(
            std::mem::offset_of!(rcutils_uint8_array_t, buffer_length),
            8
        );
        assert_eq!(
            std::mem::offset_of!(rcutils_uint8_array_t, buffer_capacity),
            16
        );
        assert_eq!(std::mem::offset_of!(rcutils_uint8_array_t, allocator), 24);
        assert_eq!(std::mem::size_of::<rcutils_uint8_array_t>(), 64);
    }
}

// ---------------------------------------------------------------------------
// Services (phase 95): clients, servers, requests and responses
// ---------------------------------------------------------------------------
//
// From the Humble headers `rcl/client.h`, `rcl/service.h` and `rmw/types.h`
// (unchanged in Jazzy). The handle and option types are only ever passed
// through, so they stay opaque.

/// `rmw_request_id_t`: the request's writer GUID and sequence number.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct rmw_request_id_t {
    pub writer_guid: [i8; 16],
    pub sequence_number: i64,
}

/// `rmw_service_info_t`: what a service-related take carries. Whether the
/// middleware fills `source_timestamp` is one of the questions phase 95
/// answers by observation (`srv.<e>.stale_after` relies on it).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct rmw_service_info_t {
    pub source_timestamp: i64,
    pub received_timestamp: i64,
    pub request_id: rmw_request_id_t,
}

/// `rcl_client_init(client, node, type_support, service_name, options)`, and
/// `rcl_service_init` with the same shape.
pub type FnRclCallEndpointInit = unsafe extern "C" fn(
    *mut c_void,
    *const rcl_interception_sys::opaque::rcl_node_t,
    *const c_void,
    *const std::ffi::c_char,
    *const c_void,
) -> rcl_ret_t;

/// `rcl_client_get_service_name(client)` / `rcl_service_get_service_name`:
/// the fully-qualified, remapped name rcl itself resolved.
pub type FnRclGetServiceName = unsafe extern "C" fn(*const c_void) -> *const std::ffi::c_char;

/// `rcl_send_request(client, ros_request, sequence_number)`
pub type FnRclSendRequest =
    unsafe extern "C" fn(*const c_void, *const c_void, *mut i64) -> rcl_ret_t;

/// `rcl_take_response_with_info(client, request_header, ros_response)` and
/// `rcl_take_request_with_info(service, request_header, ros_request)`.
pub type FnRclTakeWithInfo =
    unsafe extern "C" fn(*const c_void, *mut rmw_service_info_t, *mut c_void) -> rcl_ret_t;

/// `rcl_take_response(client, request_header, ros_response)` and
/// `rcl_take_request(service, request_header, ros_request)`: the forms
/// rclcpp calls, which in rcl call the `_with_info` ones.
pub type FnRclTakeIdOnly =
    unsafe extern "C" fn(*const c_void, *mut rmw_request_id_t, *mut c_void) -> rcl_ret_t;

/// `rcl_send_response(service, request_header, ros_response)`
pub type FnRclSendResponse =
    unsafe extern "C" fn(*const c_void, *mut rmw_request_id_t, *mut c_void) -> rcl_ret_t;
