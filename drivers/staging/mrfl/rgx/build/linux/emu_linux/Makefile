########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

PVR_SYSTEM := emu
KERNEL_COMPONENTS := srvkm

DISPLAY_CONTROLLER := dc_example
DISPLAY_CONTROLLER_DIR := dc_example

OPK_DEFAULT := libpvrNULL_WSEGL.so

include ../common/xorg_test.mk
ifeq ($(want_xorg),1)
SUPPORT_DRI_DRM := 1
OPK_FALLBACK := libpvrXDRI_WSEGL.so
XORG_TOOLCHAIN ?= tarballs-ubuntu-next
else
OPK_FALLBACK := libpvrNULL_WSEGL.so
endif

ifeq ($(SUPPORT_DRI_DRM),1)
LDM_PCI := 1
EXTRA_PVRSRVKM_COMPONENTS += $(DISPLAY_CONTROLLER_DIR)
else
KERNEL_COMPONENTS += $(DISPLAY_CONTROLLER)
endif

PDUMP ?= 1
BUILD ?= debug
NO_HARDWARE := 0
LMA ?= 0
CACHEFLUSH_TYPE ?= CACHEFLUSH_X86

DC_EXAMPLE_WIDTH ?= 640
DC_EXAMPLE_HEIGHT ?= 480
DC_EXAMPLE_BIT_DEPTH ?= 32

# Should be last
include ../config/core.mk
include ../common/xorg.mk
include ../common/dridrm.mk
include ../common/opencl.mk
include ../common/pvrgdb.mk
$(eval $(call BothConfigC,EMULATOR,1))
$(eval $(call TunableBothConfigC,LMA,))
$(eval $(call KernelConfigC,DC_EXAMPLE_WIDTH,$(DC_EXAMPLE_WIDTH)))
$(eval $(call KernelConfigC,DC_EXAMPLE_HEIGHT,$(DC_EXAMPLE_HEIGHT)))
$(eval $(call KernelConfigC,DC_EXAMPLE_BIT_DEPTH,$(DC_EXAMPLE_BIT_DEPTH)))
$(eval $(call TunableUserConfigC,VNC_IN_DRIVER,))
$(eval $(call TunableUserConfigMake,VNC_IN_DRIVER,))
