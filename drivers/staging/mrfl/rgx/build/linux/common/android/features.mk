########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Dual MIT/GPLv2
# 
# The contents of this file are subject to the MIT license as set out below.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# Alternatively, the contents of this file may be used under the terms of
# the GNU General Public License Version 2 ("GPL") in which case the provisions
# of GPL are applicable instead of those above.
# 
# If you wish to allow use of your version of this file only under the terms of
# GPL, and not to allow others to use your version of this file under the terms
# of the MIT license, indicate your decision by deleting the provisions above
# and replace them with the notice and other provisions required by GPL as set
# out in the file called "GPL-COPYING" included in this distribution. If you do
# not delete the provisions above, a recipient may use your version of this file
# under the terms of either the MIT license or GPL.
# 
# This License is also included in this distribution in the file called
# "MIT-COPYING".
# 
# EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
# PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
### ###########################################################################

include ../common/android/platform_version.mk

# Basic support option tuning for Android
#
SUPPORT_ANDROID_PLATFORM := 1
SUPPORT_OPENGLES1_V1_ONLY := 1

# Composition requires texture dependencies
#
SUPPORT_EGL_IMAGE_SYNC_DEPENDENCY := 1

# Always print debugging after 5 seconds of no activity
#
CLIENT_DRIVER_DEFAULT_WAIT_RETRIES := 50

# Android WSEGL is always the same
#
OPK_DEFAULT := libpvrANDROID_WSEGL.so

# srvkm is always built
#
KERNEL_COMPONENTS := srvkm

# Kernel modules are always installed here under Android
#
PVRSRV_MODULE_BASEDIR := /system/modules/

# Enable secure FD export in Services
#
SUPPORT_SECURE_EXPORT := 1

##############################################################################
# Handle various platform includes for unittests
#
ifeq ($(is_at_least_jellybean),1)
SYS_CFLAGS += -I$(ANDROID_ROOT)/frameworks/native/opengl/include
ANDROID_MOVED_INCLUDES := \
 $(ANDROID_ROOT)/libnativehelper/include/nativehelper
else
SYS_CFLAGS += -I$(ANDROID_ROOT)/frameworks/base/opengl/include
ANDROID_MOVED_INCLUDES := \
 $(ANDROID_ROOT)/dalvik/libnativehelper/include/nativehelper
endif

##############################################################################
# Proprietary libraries go to a vendor directory
#
SHLIB_DESTDIR := /system/vendor/lib
DEMO_DESTDIR := /system/vendor/bin

# EGL libraries go in a special place
#
EGL_DESTDIR := $(SHLIB_DESTDIR)/egl

# Must give our EGL/GLES libraries a globally unique name
#
EGL_BASENAME_SUFFIX := _POWERVR_ROGUE

# Use stlport instead of libstdc++ for STL compatibility
#
SYS_CXXFLAGS := \
 -fuse-cxa-atexit \
 $(SYS_CFLAGS) \
 -isystem $(ANDROID_ROOT)/bionic \
 -isystem $(ANDROID_ROOT)/external/stlport/stlport

##############################################################################
# ICS requires that at least one driver EGLConfig advertises the
# EGL_RECORDABLE_ANDROID attribute. The platform requires that surfaces
# rendered with this config can be consumed by an OMX video encoder.
#
EGL_EXTENSION_ANDROID_RECORDABLE := 1

##############################################################################
# ICS added the EGL_ANDROID_blob_cache extension. Enable support for this
# extension in EGL/GLESv2.
#
EGL_EXTENSION_ANDROID_BLOB_CACHE ?= 1

##############################################################################
# JB added a new corkscrew API for userland backtracing.
#
ifeq ($(is_at_least_jellybean),1)
PVR_ANDROID_HAS_CORKSCREW_API := 1
endif

# Placeholder for future version handling
#
ifeq ($(is_future_version),1)
-include ../common/android/future_version.mk
endif
