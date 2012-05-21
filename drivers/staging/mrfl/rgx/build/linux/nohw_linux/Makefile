########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

include ../kernel_version.mk

PVR_SYSTEM := rgx_nohw
KERNEL_COMPONENTS := srvkm
DRM_DC_MODULE ?= 0


ifeq ($(DRM_DC_MODULE),1)
DISPLAY_CONTROLLER := dc_drmexample
DISPLAY_CONTROLLER_DIR := dc_drmexample

SUPPORT_DRI_DRM := 1
SUPPORT_DRM_DC_MODULE := 1
else
DISPLAY_CONTROLLER := dc_example
DISPLAY_CONTROLLER_DIR := dc_example

DC_EXAMPLE_WIDTH ?= 640
DC_EXAMPLE_HEIGHT ?= 480
DC_EXAMPLE_BIT_DEPTH ?= 32
endif


OPK_DEFAULT := libpvrNULL_WSEGL.so

include ../common/xorg_test.mk
ifeq ($(want_xorg),1)
ifneq ($(SUPPORT_DRI_DRM),1)
SUPPORT_DRI_DRM := 1
endif
OPK_FALLBACK := libpvrXDRI_WSEGL.so
XORG_TOOLCHAIN ?= tarballs-ubuntu-next
else
OPK_FALLBACK := libpvrNULL_WSEGL.so
endif


ifeq ($(SUPPORT_DRI_DRM),1)
LDM_PLATFORM := 1
PVR_DRI_DRM_PLATFORM_DEV := 1
PVR_DRI_DRM_STATIC_BUS_ID := 1
ifeq ($(call kernel-version-at-least,3,1),true)
PVR_DRI_DRM_DEV_BUS_ID := "\"platform:rgxnohw:-1\""
endif
ifeq ($(SUPPORT_DRM_DC_MODULE),1)
KERNEL_COMPONENTS += $(DISPLAY_CONTROLLER)
else
EXTRA_PVRSRVKM_COMPONENTS += $(DISPLAY_CONTROLLER_DIR)
endif
else
KERNEL_COMPONENTS += $(DISPLAY_CONTROLLER)
endif


PDUMP ?= 1
BUILD ?= debug
NO_HARDWARE := 1

GLES2_EXTENSION_EGL_IMAGE_EXTERNAL := 1

CACHEFLUSH_TYPE ?= CACHEFLUSH_X86

# Should be last
include ../config/core.mk
include ../common/xorg.mk
include ../common/dridrm.mk
include ../common/opencl.mk
include ../common/pvrgdb.mk

$(eval $(call TunableKernelConfigC,DC_EXAMPLE_WIDTH,$(DC_EXAMPLE_WIDTH)))
$(eval $(call TunableKernelConfigC,DC_EXAMPLE_HEIGHT,$(DC_EXAMPLE_HEIGHT)))
$(eval $(call TunableKernelConfigC,DC_EXAMPLE_BIT_DEPTH,$(DC_EXAMPLE_BIT_DEPTH)))
$(eval $(call TunableUserConfigC,SGLTRACE,))
