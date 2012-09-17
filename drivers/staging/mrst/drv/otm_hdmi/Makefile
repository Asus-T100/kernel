########################################################################
#
# This file is provided under a dual BSD/GPLv2 license.  When using or
# redistributing this file, you may do so under either license.
#
# GPL LICENSE SUMMARY
#
# Copyright(c) 2011 Intel Corporation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of version 2 of the GNU General Public License as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
# The full GNU General Public License is included in this distribution
# in the file called LICENSE.GPL.
#
# Contact Information:
# Intel Corporation
# 2200 Mission College Blvd.
# Santa Clara, CA  97052
#
# BSD LICENSE
#
# Copyright(c) 2011 Intel Corporation. All rights reserved.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.
#   * Neither the name of Intel Corporation nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

TARGET = otm_hdmi

OTM_HDMI_TOP = .
OTM_HDMI_INCDIR = $(srctree)/$(src)/$(OTM_HDMI_TOP)

INCDIR=$(OTM_HDMI_INCDIR)/../..
MEDIA_INCDIR=drivers/staging/intel_media

include_dirs := \
	-I$(INCDIR)/pvr/include4 \
        -I$(INCDIR)/pvr/services4/include \
        -I$(INCDIR)/pvr/services4/include/env/linux \
        -I$(INCDIR)/pvr/services4/srvkm/env/linux \
        -I$(INCDIR)/pvr/services4/srvkm/include \
        -I$(INCDIR)/pvr/services4/srvkm/bridged \
	-I$(INCDIR)/pvr/services4/system/include \
        -I$(INCDIR)/pvr/services4/srvkm/hwdefs \
        -I$(INCDIR)/pvr/services4/srvkm/bridged/sgx \
        -I$(INCDIR)/pvr/services4/srvkm/devices/sgx \
        -I$(INCDIR)/pvr/services4/srvkm/common \
        -I$(INCDIR)/pvr/services4/3rdparty/linux_framebuffer_drm \
        -I$(INCDIR)/ \
        -I$(INCDIR)/drv \
        -I$(MEDIA_INCDIR)/bc_video \
        -I$(MEDIA_INCDIR)/common \
        -I$(MEDIA_INCDIR)/video/common \
        -I$(INCDIR)/imgv \
	-Iinclude/linux \
	-Iinclude/drm

ccflags-y += $(include_dirs)
ccflags-y += $(ANDROID_TOOLCHAIN_FLAGS) -I$(INCDIR)/pvr/services4/system/intel_drm -DANDROID -D_linux_ -D__KERNEL__

ifeq ($(pvrdbg),1)
ccflags-y += -DPVRDEBUG
endif
ifeq ($(CONFIG_DRM_MDFLD),y)
ccflags-y += -DMEDFIELD
IS_SGX540=y
endif
ifeq ($(CONFIG_DRM_CTP),y)
ccflags-y += -DCLOVERTRAIL_PHONE
ifeq ($(CONFIG_DRM_CTP_PR1),y)
IS_SGX544=y
else
IS_SGX545=y
endif
endif

ccflags-$(IS_SGX540) += -DSGX540 -DSUPPORT_SGX540 -DSGX_CORE_REV=121
ccflags-$(IS_SGX544) += -DSGX544 -DSGX_CORE_REV=115 \
	-DSGX_FEATURE_MP=1 -DSGX_FEATURE_MP_CORE_COUNT=2 -DSGX_FEATURE_SYSTEM_CACHE=1
ccflags-$(IS_SGX545) += -DSGX545 -DSUPPORT_SGX545 -DSGX_CORE_REV=1014

ccflags-y += \
	-DLINUX \
    -DPVRSRV_MODNAME="\"pvrsrvkm\"" \
	-DPVR_BUILD_DIR="\"pc_i686_medfield_linux\"" \
	-DSERVICES4 \
	-D_XOPEN_SOURCE=600 \
	-DPVR2D_VALIDATE_INPUT_PARAMS \
	-DDISPLAY_CONTROLLER=mrstlfb \
	-UDEBUG_LOG_PATH_TRUNCATE \
	-DSUPPORT_LIBDRM_LITE \
	-DOPK_FALLBACK="" \
	-DSUPPORT_ANDROID_PLATFORM \
	-DPROFILE_COMM \
	-DPVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE\
    -DSUPPORT_ANDROID_PLATFORM \
	-DSUPPORT_GRAPHICS_HAL \
	-DPVR_SECURE_FD_EXPORT \
	-DSUPPORT_SRVINIT \
	-DSUPPORT_SGX \
	-DSUPPORT_PERCONTEXT_PB \
	-DSUPPORT_LINUX_X86_WRITECOMBINE \
	-DTRANSFER_QUEUE \
	-DSUPPORT_DRI_DRM \
	-DSUPPORT_DRI_DRM_EXT \
    -DSUPPORT_DRI_DRM_NO_DROPMASTER \
	-DSYS_USING_INTERRUPTS \
	-DSUPPORT_HW_RECOVERY \
	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	-DPVR_SECURE_HANDLES \
	-DLDM_PCI \
	-DUSE_PTHREADS \
	-DSUPPORT_SGX_EVENT_OBJECT \
	-DSUPPORT_SGX_HWPERF \
	-DSUPPORT_LINUX_X86_PAT \
	-DPVR_PROC_USE_SEQ_FILE \
	-DSUPPORT_CPU_CACHED_BUFFERS \
    -DDRM_PVR_RESERVED_INTEL_ORDER \
    -DDRM_PVR_USE_INTEL_FB \
	-DSUPPORT_MEMINFO_IDS \
    -DSUPPORT_SGX_NEW_STATUS_VALS

#    -DSUPPORT_PDUMP_MULTI_PROCESS \

ifneq ($(IS_SGX545),y)
ccflags-y += -DSUPPORT_SGX_LOW_LATENCY_SCHEDULING
endif

ccflags-$(CONFIG_DRM_MID_RELEASE) += -DBUILD="\"release\"" -DPVR_BUILD_TYPE="\"release\"" -DRELEASE
ccflags-$(CONFIG_DRM_MID_DEBUG) += -DBUILD="\"debug\"" -DPVR_BUILD_TYPE="\"debug\"" -DDEBUG -DDEBUG_LINUX_MEM_AREAS -DDEBUG_LINUX_MEMORY_ALLOCATIONS -DDEBUG_LINUX_MMAP_AREAS -DDEBUG_BRIDGE_KM
ccflags-$(CONFIG_PCI_MSI) += -DCONFIG_PCI_MSI
ccflags-$(CONFIG_MDFD_GL3) += -DSUPPORT_EXTERNAL_SYSTEM_CACHE -DCONFIG_MDFD_GL3

ccflags-y += \
	-I$(OTM_HDMI_INCDIR)/os/android/include \
	-I$(OTM_HDMI_INCDIR)/pil/include \
	-I$(OTM_HDMI_INCDIR)/pil/common \
	-I$(OTM_HDMI_INCDIR)/pil/specific/include \
	-I$(OTM_HDMI_INCDIR)/ipil/include \
	-I$(OTM_HDMI_INCDIR)/ipil/common \
	-I$(OTM_HDMI_INCDIR)/ipil/specific/include \

# This flag should always be kept ON
ccflags-y += -Werror

#uncomment for debug prints
#ccflags-y += -DDEBUG

#only for internal testing
ccflags-y += -DOTM_HDMI_UNIT_TEST

#enable HDCP
ccflags-y += -DOTM_HDMI_HDCP_ENABLE
#ccflags-y += -DOTM_HDMI_HDCP_ALWAYS_ENC

ifeq ($(CONFIG_MDFD_HDMI_REDRIDGE),y)
ccflags-y += \
	-DMFLD_HDMI_DV1 \
	-I$(OTM_HDMI_INCDIR)/pil/specific/mfld \
	-I$(OTM_HDMI_INCDIR)/ipil/specific/mfld
else ifeq ($(CONFIG_MDFD_HDMI_PR2),y)
ccflags-y += \
	-I$(OTM_HDMI_INCDIR)/pil/specific/mfld \
	-I$(OTM_HDMI_INCDIR)/ipil/specific/mfld
else ifeq ($(CONFIG_CTP_HDMI),y)
ccflags-y += \
	-I$(OTM_HDMI_INCDIR)/pil/specific/ctp \
	-I$(OTM_HDMI_INCDIR)/ipil/specific/mfld
endif

# Platform independent library
$(TARGET)-y += \
	$(OTM_HDMI_TOP)/pil/common/otm_hdmi.o \
	$(OTM_HDMI_TOP)/pil/common/mode_info.o \
	$(OTM_HDMI_TOP)/pil/common/hdcp.o \
	$(OTM_HDMI_TOP)/pil/common/edid.o \
	$(OTM_HDMI_TOP)/pil/common/edid_print.o \
	$(OTM_HDMI_TOP)/pil/common/infoframes.o

# IP independent library
$(TARGET)-y += \
	$(OTM_HDMI_TOP)/ipil/common/otm_ipil_main.o \
	$(OTM_HDMI_TOP)/ipil/common/ipil_hdcp.o

# OS specific library
$(TARGET)-y += \
	$(OTM_HDMI_TOP)/os/android/android_hdmi.o

ifeq ($(CONFIG_MDFD_HDMI_REDRIDGE),y)
$(TARGET)-y += \
	$(OTM_HDMI_TOP)/ipil/specific/mfld/ips_hdmi.o \
	$(OTM_HDMI_TOP)/ipil/specific/mfld/ips_hdcp.o \
	$(OTM_HDMI_TOP)/pil/specific/mfld/ps_hdmi.o
else ifeq ($(CONFIG_MDFD_HDMI_PR2),y)
$(TARGET)-y += \
	$(OTM_HDMI_TOP)/ipil/specific/mfld/ips_hdmi.o \
	$(OTM_HDMI_TOP)/ipil/specific/mfld/ips_hdcp.o \
	$(OTM_HDMI_TOP)/pil/specific/mfld/ps_hdmi.o
else ifeq ($(CONFIG_CTP_HDMI),y)
$(TARGET)-y += \
	$(OTM_HDMI_TOP)/ipil/specific/mfld/ips_hdmi.o \
	$(OTM_HDMI_TOP)/ipil/specific/mfld/ips_hdcp.o \
	$(OTM_HDMI_TOP)/pil/specific/ctp/ps_hdmi.o
endif

obj-$(CONFIG_MDFD_HDMI) += $(TARGET).o
