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
