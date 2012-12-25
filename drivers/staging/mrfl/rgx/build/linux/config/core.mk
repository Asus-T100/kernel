########################################################################### ###
#@File
#@Title         Root build configuration.
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

# Configuration wrapper for new build system. This file deals with
# configuration of the build. Add to this file anything that deals
# with switching driver options on/off and altering the defines or
# objects the build uses.
#
# At the end of this file is an exhaustive list of all variables
# that are passed between the platform/config stage and the generic
# build. PLEASE refrain from adding more variables than necessary
# to this stage -- almost all options can go through config.h.
#

# Sanity check: Make sure preconfig has been included
ifeq ($(TOP),)
$(error TOP not defined: Was preconfig.mk included in root makefile?)
endif

################################# MACROS ####################################

ALL_TUNABLE_OPTIONS :=

# Write out a kernel GNU make option.
#
define KernelConfigMake
$$(shell echo "override $(1) := $(2)" >>$(CONFIG_KERNEL_MK).new)
$(if $(filter config,$(D)),$(info KernelConfigMake $(1) := $(2) 	# $(if $($(1)),$(origin $(1)),default)))
endef

# Write out a GNU make option for both user & kernel
#
define BothConfigMake
$$(eval $$(call KernelConfigMake,$(1),$(2)))
$$(eval $$(call UserConfigMake,$(1),$(2)))
endef

# Conditionally write out a kernel GNU make option
#
define _TunableKernelConfigMake
ifneq ($$($(1)),)
ifneq ($$($(1)),0)
$$(eval $$(call KernelConfigMake,$(1),$$($(1))))
endif
else
ifneq ($(2),)
$$(eval $$(call KernelConfigMake,$(1),$(2)))
endif
endif
endef

define TunableKernelConfigMake
$$(eval $$(call _TunableKernelConfigMake,$(1),$(2)))
ALL_TUNABLE_OPTIONS += $(1)
ifeq ($(INTERNAL_DESCRIPTION_FOR_$(1)),)
INTERNAL_DESCRIPTION_FOR_$(1) := $(3)
endif
INTERNAL_CONFIG_DEFAULT_FOR_$(1) := $(2)
endef

# Conditionally write out a GNU make option for both user & kernel
#
define TunableBothConfigMake
$$(eval $$(call _TunableKernelConfigMake,$(1),$(2)))
$$(eval $$(call _TunableUserConfigMake,$(1),$(2)))
ALL_TUNABLE_OPTIONS += $(1)
ifeq ($(INTERNAL_DESCRIPTION_FOR_$(1)),)
INTERNAL_DESCRIPTION_FOR_$(1) := $(3)
endif
INTERNAL_CONFIG_DEFAULT_FOR_$(1) := $(2)
endef

# Write out a kernel-only option
#
define KernelConfigC
$$(shell echo "#define $(1) $(2)" >>$(CONFIG_KERNEL_H).new)
$(if $(filter config,$(D)),$(info KernelConfigC    #define $(1) $(2) 	/* $(if $($(1)),$(origin $(1)),default) */),)
endef

# Write out an option for both user & kernel
#
define BothConfigC
$$(eval $$(call KernelConfigC,$(1),$(2)))
$$(eval $$(call UserConfigC,$(1),$(2)))
endef

# Conditionally write out a kernel-only option
#
define _TunableKernelConfigC
ifneq ($$($(1)),)
ifneq ($$($(1)),0)
ifeq ($$($(1)),1)
$$(eval $$(call KernelConfigC,$(1),))
else
$$(eval $$(call KernelConfigC,$(1),$$($(1))))
endif
endif
else
ifneq ($(2),)
ifeq ($(2),1)
$$(eval $$(call KernelConfigC,$(1),))
else
$$(eval $$(call KernelConfigC,$(1),$(2)))
endif
endif
endif
endef

define TunableKernelConfigC
$$(eval $$(call _TunableKernelConfigC,$(1),$(2)))
ALL_TUNABLE_OPTIONS += $(1)
ifeq ($(INTERNAL_DESCRIPTION_FOR_$(1)),)
INTERNAL_DESCRIPTION_FOR_$(1) := $(3)
endif
INTERNAL_CONFIG_DEFAULT_FOR_$(1) := $(2)
endef

# Conditionally write out an option for both user & kernel
#
define TunableBothConfigC
$$(eval $$(call _TunableKernelConfigC,$(1),$(2)))
$$(eval $$(call _TunableUserConfigC,$(1),$(2)))
ALL_TUNABLE_OPTIONS += $(1)
ifeq ($(INTERNAL_DESCRIPTION_FOR_$(1)),)
INTERNAL_DESCRIPTION_FOR_$(1) := $(3)
endif
INTERNAL_CONFIG_DEFAULT_FOR_$(1) := $(2)
endef

############################### END MACROS ##################################

# Check we have a new enough version of GNU make.
#
need := 3.81
ifeq ($(filter $(need),$(firstword $(sort $(MAKE_VERSION) $(need)))),)
$(error A version of GNU make >= $(need) is required - this is version $(MAKE_VERSION))
endif

include ../defs.mk

# Infer PVR_BUILD_DIR from the directory configuration is launched from.
# Check anyway that such a directory exists.
#
PVR_BUILD_DIR := $(notdir $(abspath .))
$(call directory-must-exist,$(TOP)/build/linux/$(PVR_BUILD_DIR))

# Output directory for configuration, object code,
# final programs/libraries, and install/rc scripts.
#
BUILD        ?= release
OUT          ?= $(TOP)/binary_$(PVR_BUILD_DIR)_$(BUILD)
override OUT := $(if $(filter /%,$(OUT)),$(OUT),$(TOP)/$(OUT))

CONFIG_MK			:= $(OUT)/config.mk
CONFIG_H			:= $(OUT)/config.h
CONFIG_KERNEL_MK	:= $(OUT)/config_kernel.mk
CONFIG_KERNEL_H		:= $(OUT)/config_kernel.h

# Convert commas to spaces in $(D). This is so you can say "make
# D=config-changes,freeze-config" and have $(filter config-changes,$(D))
# still work.
override D := $(subst $(comma),$(space),$(D))

# Create the OUT directory 
#
$(shell mkdir -p $(OUT))

# Some targets don't need information about any modules. If we only specify
# these targets on the make command line, set INTERNAL_CLOBBER_ONLY to
# indicate that toplevel.mk shouldn't read any makefiles
CLOBBER_ONLY_TARGETS := clean clobber help install
INTERNAL_CLOBBER_ONLY :=
ifneq ($(strip $(MAKECMDGOALS)),)
INTERNAL_CLOBBER_ONLY := \
$(if \
 $(strip $(foreach _cmdgoal,$(MAKECMDGOALS),\
          $(if $(filter $(_cmdgoal),$(CLOBBER_ONLY_TARGETS)),,x))),,true)
endif

# For a clobber-only build, we shouldn't regenerate any config files
ifneq ($(INTERNAL_CLOBBER_ONLY),true)

-include ../config/user-defs.mk

#
# Core handling 


# delete any previous intermediary files
$(shell \
	for file in $(CONFIG_KERNEL_H).new $(CONFIG_KERNEL_MK).new ; do \
		rm -f $$file; \
	done)

# Check BVNC version
List_Of_KM_BVNCs_files=$(shell ls $(TOP)/hwdefs/km/cores/rgxcore_km_*h)
List_Of_KM_BVNCs_filenames=$(notdir $(basename $(List_Of_KM_BVNCs_files)))
List_Of_KM_BVNCs=$(subst rgxcore_km_,,$(List_Of_KM_BVNCs_filenames))
List_Of_KM_BVNCs2=$(subst $(space),$(comma)$(space),$(List_Of_KM_BVNCs))
ifeq ($(findstring $(RGX_BVNC),$(List_Of_KM_BVNCs)),)
$(error Error: Invalid Kernel RGX_BVNC=${RGX_BVNC}. Valid Kernel BVNCs: $(List_Of_KM_BVNCs2))
endif


# Check if BVNC file exist
RGX_BVNC_CORE_KM := $(TOP)/hwdefs/km/cores/rgxcore_km_$(RGX_BVNC).h
RGX_BVNC_CORE_KM_HEADER := \"cores/rgxcore_km_$(RGX_BVNC).h\" 
#"rgxcore_km_$(RGX_BVNC).h"
ifeq ($(wildcard $(RGX_BVNC_CORE_KM)),)
$(error The file $(RGX_BVNC_CORE_KM) does not exist. Valid BVNCs: $(List_Of_KM_BVNCs2))
endif

# Enforced dependencies. Move this to an include.
#
SUPPORT_LINUX_USING_WORKQUEUES ?= 1
ifeq ($(SUPPORT_LINUX_USING_WORKQUEUES),1)
override PVR_LINUX_USING_WORKQUEUES := 1
override PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE := 1
override PVR_LINUX_TIMERS_USING_WORKQUEUES := 1
override SYS_CUSTOM_POWERLOCK_WRAP := 1
else ifeq ($(SUPPORT_LINUX_USING_SHARED_WORKQUEUES),1)
override PVR_LINUX_USING_WORKQUEUES := 1
override PVR_LINUX_MISR_USING_WORKQUEUE := 1
override PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE := 1
override SYS_CUSTOM_POWERLOCK_WRAP := 1
endif

ifneq ($(PDUMP),1)
override SUPPORT_PDUMP_MULTI_PROCESS := 0
endif

ifeq ($(NO_HARDWARE),1)
override SYS_USING_INTERRUPTS := 0
override SUPPORT_HW_RECOVERY := 0
endif

# Rather than requiring the user to have to define two variables (one quoted,
# one not), make PVRSRV_MODNAME a non-tunable and give it an overridable
# default here.
#
PVRSRV_MODNAME ?= pvrsrvkm

# Build's selected list of components.
# - components.mk is a per-build file that specifies the components that are
#   to be built
-include components.mk

ifeq ($(SUPPORT_ION_OMAP),1)
override SUPPORT_ION := 1
endif

# PDUMP needs extra components
#
ifeq ($(PDUMP),1)
ifneq ($(COMPONENTS),)
COMPONENTS += pdump
endif
ifeq ($(SUPPORT_DRM),1)
EXTRA_PVRSRVKM_COMPONENTS += dbgdrv
else
KERNEL_COMPONENTS += dbgdrv
endif
endif

# PVRGDB needs extra components
#
ifeq ($(PVRGDB),1)
ifneq ($(COMPONENTS),)
COMPONENTS += pvrgdb pvrdebugipc
endif
endif

$(if $(filter config,$(D)),$(info Build configuration:))

################################# CONFIG ####################################

# If KERNELDIR is set, write it out to the config.mk, with
# KERNEL_COMPONENTS and KERNEL_ID
#
ifneq ($(strip $(KERNELDIR)),)
PVRSRV_MODULE_BASEDIR ?= /lib/modules/$(KERNEL_ID)/extra/
$(eval $(call KernelConfigMake,KERNELDIR,$(KERNELDIR)))
# Needed only by install script
$(eval $(call UserConfigMake,KERNEL_ID,$(KERNEL_ID)))
$(eval $(call KernelConfigMake,KERNEL_COMPONENTS,$(KERNEL_COMPONENTS)))
$(eval $(call TunableKernelConfigMake,EXTRA_PVRSRVKM_COMPONENTS,))
$(eval $(call TunableKernelConfigMake,EXTRA_KBUILD_SOURCE,))

# If KERNEL_CROSS_COMPILE is set to "undef", this is magically
# equivalent to being unset. If it is unset, we use CROSS_COMPILE
# (which might also be unset). If it is set, use it directly.
ifneq ($(KERNEL_CROSS_COMPILE),undef)
KERNEL_CROSS_COMPILE ?= $(CROSS_COMPILE)
$(eval $(call TunableBothConfigMake,KERNEL_CROSS_COMPILE,))
endif

# Check the KERNELDIR has a kernel built and also check that it is
# not 64-bit, which we do not support.
KERNEL_AUTOCONF := \
 $(strip $(wildcard $(KERNELDIR)/include/linux/autoconf.h) \
         $(wildcard $(KERNELDIR)/include/generated/autoconf.h))
ifneq ($(KERNEL_AUTOCONF),)
LINUX_IS_64BIT :=  $(shell egrep -q "CONFIG_64BIT\s+1" $(KERNEL_AUTOCONF) || echo false)
ifneq ($(LINUX_IS_64BIT),false)
$(warning $$(KERNELDIR) is 64-bit, which is not supported. Kbuild will probably fail.)
endif
else
$(warning autoconf.h not found in $$(KERNELDIR)/include/linux \
or $$(KERNELDIR)/include/generated. Check your $$(KERNELDIR) variable \
and kernel configuration.)
endif
endif

# Ubuntu ARM compilers need an explicit -marm option to build native ARM code,
# rather than ARM thumb code.
EXPLICIT_NOT_THUMB :=
ifeq ($(CROSS_COMPILE),arm-linux-gnueabi-)
EXPLICIT_NOT_THUMB = 1
endif
ifeq ($(CROSS_COMPILE),arm-linux-gnueabihf-)
EXPLICIT_NOT_THUMB = 1
endif

ifeq ($(EXPLICIT_NOT_THUMB),1)
SYS_CFLAGS += -marm
endif

$(eval $(call UserConfigC,PVRSRV_MODULE_BASEDIR,\"$(PVRSRV_MODULE_BASEDIR)\"))

# Ideally configured by platform Makefiles, as necessary
#

# Invariant options for Linux
#
$(eval $(call BothConfigC,LINUX,))

$(eval $(call BothConfigC,PVR_BUILD_DIR,"\"$(PVR_BUILD_DIR)\""))
$(eval $(call BothConfigC,PVR_BUILD_TYPE,"\"$(BUILD)\""))
$(eval $(call BothConfigC,PVRSRV_MODNAME,"\"$(PVRSRV_MODNAME)\""))
$(eval $(call UserConfigMake,PVRSRV_MODNAME,$(PVRSRV_MODNAME)))
$(eval $(call UserConfigMake,PVR_BUILD_DIR,$(PVR_BUILD_DIR)))
$(eval $(call UserConfigMake,PVR_BUILD_TYPE,$(BUILD)))

$(eval $(call BothConfigC,SUPPORT_RGX,1))
$(eval $(call UserConfigMake,SUPPORT_RGX,1))

$(eval $(call BothConfigC,PVR_SECURE_HANDLES,))
$(eval $(call UserConfigC,USE_PTHREADS,))

ifneq ($(DISPLAY_CONTROLLER),)
$(eval $(call BothConfigC,DISPLAY_CONTROLLER,$(DISPLAY_CONTROLLER)))
$(eval $(call UserConfigMake,DISPLAY_CONTROLLER,$(DISPLAY_CONTROLLER)))
endif

$(eval $(call UserConfigC,OPK_DEFAULT,"\"$(OPK_DEFAULT)\""))
$(eval $(call UserConfigC,OPK_FALLBACK,"\"$(OPK_FALLBACK)\""))

$(eval $(call BothConfigMake,PVR_SYSTEM,$(PVR_SYSTEM)))

# Build-type dependent options
#
$(eval $(call BothConfigMake,BUILD,$(BUILD)))

ifeq ($(BUILD),debug)
$(eval $(call BothConfigC,DEBUG,))
$(eval $(call KernelConfigC,DEBUG_LINUX_MEMORY_ALLOCATIONS,))
$(eval $(call KernelConfigC,DEBUG_LINUX_MEM_AREAS,))
$(eval $(call KernelConfigC,DEBUG_LINUX_MMAP_AREAS,))
$(eval $(call KernelConfigC,DEBUG_BRIDGE_KM,))
$(eval $(call UserConfigC,DLL_METRIC,1))
$(eval $(call TunableBothConfigC,RGXFW_ALIGNCHECKS,1))
else ifeq ($(BUILD),release)
$(eval $(call BothConfigC,RELEASE,))
$(eval $(call TunableBothConfigMake,DEBUGLINK,1))
# FIXME: Move the default value into a separate variable so we only have to
# write the documentation for these options once.
$(eval $(call TunableBothConfigC,RGXFW_ALIGNCHECKS,))
else ifeq ($(BUILD),timing)
$(eval $(call BothConfigC,TIMING,))
$(eval $(call UserConfigC,DLL_METRIC,1))
$(eval $(call TunableBothConfigMake,DEBUGLINK,1))
else
$(error BUILD= must be either debug, release or timing)
endif

# User-configurable options
#

$(eval $(call TunableBothConfigC,RGX_BVNC_CORE_KM_HEADER,))

$(eval $(call TunableBothConfigC,SUPPORT_HW_RECOVERY,1))
$(eval $(call TunableBothConfigC,SUPPORT_RENDER_TARGET_ARRAYS,1))
$(eval $(call TunableBothConfigC,SUPPORT_PDUMP_MULTI_PROCESS,))
$(eval $(call TunableBothConfigC,SUPPORT_DBGDRV_EVENT_OBJECTS,1))
$(eval $(call TunableBothConfigC,PVR_DBG_BREAK_ASSERT_FAIL,,\
Enable this to treat PVR_DBG_BREAK as PVR_ASSERT(0)._\
Otherwise it is ignored._\
))
$(eval $(call TunableBothConfigC,PDUMP,,\
Enable parameter dumping in the driver._\
This adds code to record the parameters being sent to the hardware for_\
later analysis._\
))
$(eval $(call TunableBothConfigC,NO_HARDWARE,))
$(eval $(call TunableBothConfigC,PDUMP_DEBUG_OUTFILES,))
$(eval $(call TunableBothConfigC,SYS_USING_INTERRUPTS,1))
$(eval $(call TunableBothConfigC,PVRSRV_NEED_PVR_DPF,,\
Enable this to turn on PVR_DPF in release builds._\
))
$(eval $(call TunableBothConfigC,PVRSRV_NEED_PVR_ASSERT,,\
Enable this to turn on PVR_ASSERT in release builds._\
))
$(eval $(call TunableBothConfigC,PVRSRV_NEED_PVR_TRACE,,\
Enable this to turn on PVR_TRACE in release builds._\
))
$(eval $(call TunableBothConfigC,REFCOUNT_DEBUG,))
$(eval $(call TunableBothConfigC,DC_DEBUG,))
$(eval $(call TunableBothConfigC,SCP_DEBUG,))
$(eval $(call TunableBothConfigC,PHSHEAP_DEBUG,))
$(eval $(call TunableBothConfigC,CACHEFLUSH_TYPE,CACHEFLUSH_GENERIC))
$(eval $(call TunableBothConfigC,SUPPORT_INSECURE_EXPORT,1))
$(eval $(call TunableBothConfigC,SUPPORT_SECURE_EXPORT,1,\
Enable support for secure device memory and sync export._\
This replaces export handles with file descriptors$(comma) which can be passed_\
between processes to share memory._\
))
$(eval $(call TunableBothConfigC,SUPPORT_PMMIF,))
$(eval $(call TunableBothConfigC,SUPPORT_ION,))
$(eval $(call TunableBothConfigC,RGXFW_HWR,))
$(eval $(call TunableBothConfigC,SUPPORT_TRUSTED_ZONE,))

$(eval $(call TunableKernelConfigC,SUPPORT_LINUX_X86_WRITECOMBINE,1))
$(eval $(call TunableKernelConfigC,SUPPORT_LINUX_X86_PAT,1))
$(eval $(call TunableKernelConfigC,SYS_CUSTOM_POWERLOCK_WRAP,))
$(eval $(call TunableKernelConfigC,PVRSRV_RESET_ON_HWTIMEOUT,))
$(eval $(call TunableKernelConfigC,PVR_LINUX_USING_WORKQUEUES,))
$(eval $(call TunableKernelConfigC,PVR_LINUX_MISR_USING_WORKQUEUE,))
$(eval $(call TunableKernelConfigC,PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE,))
$(eval $(call TunableKernelConfigC,PVR_LINUX_TIMERS_USING_WORKQUEUES,))
$(eval $(call TunableKernelConfigC,PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE,))
$(eval $(call TunableKernelConfigC,PVR_LDM_PLATFORM_PRE_REGISTERED,))
$(eval $(call TunableKernelConfigC,PVR_LDM_PLATFORM_PRE_REGISTERED_DEV,))
$(eval $(call TunableKernelConfigC,PVR_LDM_DRIVER_REGISTRATION_NAME,"\"$(PVRSRV_MODNAME)\""))
$(eval $(call TunableKernelConfigC,LDM_DEVICE_CLASS,))
$(eval $(call TunableKernelConfigC,LDM_PLATFORM,))
$(eval $(call TunableKernelConfigC,LDM_PCI,))
$(eval $(call TunableKernelConfigC,SYNC_DEBUG,))
$(eval $(call TunableKernelConfigC,PVR_LINUX_DONT_USE_RANGE_BASED_INVALIDATE,))


$(eval $(call TunableBothConfigMake,CACHEFLUSH_TYPE,CACHEFLUSH_GENERIC))
$(eval $(call TunableBothConfigMake,PDUMP,))
$(eval $(call TunableBothConfigMake,SUPPORT_INSECURE_EXPORT,1))
$(eval $(call TunableBothConfigMake,SUPPORT_SECURE_EXPORT,1))
$(eval $(call TunableBothConfigMake,SUPPORT_PMMIF,))

$(eval $(call TunableBothConfigMake,SUPPORT_ION,))
$(eval $(call TunableKernelConfigMake,SUPPORT_ION_OMAP,))

$(eval $(call TunableBothConfigMake,OPTIM,))

$(eval $(call TunableBothConfigC,SUPPORT_MMU_FREELIST,))
$(eval $(call TunableBothConfigC,SUPPORT_VFP,))

$(eval $(call TunableBothConfigC,SUPPORT_META_SLAVE_BOOT,))

$(eval $(call UserConfigC,EGL_BASENAME_SUFFIX,\"$(EGL_BASENAME_SUFFIX)\"))




$(eval $(call TunableBothConfigC,PVR_TRANSPORTLAYER_TESTING,,\
Enable this to build in support for testing the PVR Transport Layer API._\
))


endif # INTERNAL_CLOBBER_ONLY

export INTERNAL_CLOBBER_ONLY
export TOP
export OUT

MAKE_ETC := -Rr --no-print-directory -C $(TOP) TOP=$(TOP) OUT=$(OUT) \
	        -f build/linux/toplevel.mk

# This must match the default value of MAKECMDGOALS below, and the default
# goal in toplevel.mk
.DEFAULT_GOAL := build

ifeq ($(MAKECMDGOALS),)
MAKECMDGOALS := build
else
# We can't pass autogen to toplevel.mk
MAKECMDGOALS := $(filter-out autogen,$(MAKECMDGOALS))
endif

.PHONY: autogen
autogen:
ifeq ($(INTERNAL_CLOBBER_ONLY),)
	@$(MAKE) -s --no-print-directory -C $(TOP) \
		-f build/linux/prepare_tree.mk \
		LDM_PCI=$(LDM_PCI) \
		LDM_PLATFORM=$(LDM_PLATFORM) \
		LDM_DEVICE_CLASS=$(LDM_DEVICE_CLASS)
else
	@:
endif

include ../config/help.mk

# This deletes built-in suffix rules. Otherwise the submake isn't run when
# saying e.g. "make thingy.a"
.SUFFIXES:

# Because we have a match-anything rule below, we'll run the main build when
# we're actually trying to remake various makefiles after they're read in.
# These rules try to prevent that
%.mk: ;
Makefile%: ;
Makefile: ;

.PHONY: build kbuild install
build kbuild install: autogen
	@$(if $(MAKECMDGOALS),$(MAKE) $(MAKE_ETC) $(MAKECMDGOALS) $(eval MAKECMDGOALS :=),:)

%: autogen
	@$(if $(MAKECMDGOALS),$(MAKE) $(MAKE_ETC) $(MAKECMDGOALS) $(eval MAKECMDGOALS :=),:)
