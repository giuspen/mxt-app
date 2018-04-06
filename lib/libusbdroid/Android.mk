LOCAL_PATH := $(call my-dir)/code/src/LibUSBDroid/jni
include $(CLEAR_VARS)

LOCAL_SRC_FILES := core.c descriptor.c hotplug.c io.c libusb_jni.c sync.c os/linux_netlink.c os/linux_usbfs.c os/poll_posix.c os/threads_posix.c
LOCAL_CFLAGS := -fPIC -DPIC
LOCAL_MODULE := libusbdroid

include $(BUILD_STATIC_LIBRARY)
