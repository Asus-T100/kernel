########################################################################### ###
#@File
#@Title         Root build configuration.
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
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

################################# MACROS ####################################

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
endef

# Conditionally write out a kernel GNU make option
#
define TunableKernelConfigMake
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

# Conditionally write out a GNU make option for both user & kernel
#
define TunableBothConfigMake
$$(eval $$(call TunableKernelConfigMake,$(1),$(2)))
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
endef

# Conditionally write out a kernel-only option
#
define TunableKernelConfigC
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

# Conditionally write out an option for both user & kernel
#
define TunableBothConfigC
$$(eval $$(call TunableKernelConfigC,$(1),$(2)))
endef

############################### END MACROS ##################################

# Check we have a new enough version of GNU make.
#
need := 3.81
ifeq ($(filter $(need),$(firstword $(sort $(MAKE_VERSION) $(need)))),)
$(error A version of GNU make >= $(need) is required - this is version $(MAKE_VERSION))
endif

# Try to guess PVRSW_ROOT if it wasn't set. Check this location.
#
_GUESSED_PVRSW_ROOT := $(abspath ../../..)
ifneq ($(strip $(PVRSW_ROOT)),)
# We don't want to warn about PVRSW_ROOT if it's empty: this might mean that
# it's not set at all anywhere, but it could also mean that it's set like
# "export PVRSW_ROOT=" or "make PVRSW_ROOT= sometarget". If it is set but
# empty, we'll act as if it's unset and not warn.
ifneq ($(strip $(PVRSW_ROOT)),$(_GUESSED_PVRSW_ROOT))
$(warning PVRSW_ROOT is set (via: $(origin PVRSW_ROOT)), but its value does not)
$(warning match the root of this source tree, so it is being ignored)
$(warning PVRSW_ROOT is set to: $(PVRSW_ROOT))
$(warning The detected root is: $(_GUESSED_PVRSW_ROOT))
$(warning To suppress this message, unset PVRSW_ROOT or set it empty)
endif
# else, PVRSW_ROOT matched the actual root of the source tree: don't warn
endif
override PVRSW_ROOT := $(_GUESSED_PVRSW_ROOT)
TOP := $(PVRSW_ROOT)

ifneq ($(words $(TOP)),1)
$(warning This source tree is located in a path which contains whitespace,)
$(warning which is not supported.)
$(warning $(space)The root is: $(TOP))
$(error Whitespace found in $$(TOP))
endif

$(call directory-must-exist,$(TOP))

include ../defs.mk

# Infer PVR_BUILD_DIR from the directory configuration is launched from.
# Check anyway that such a directory exists.
#
PVR_BUILD_DIR := $(notdir $(abspath .))
$(call directory-must-exist,$(TOP)/build/linux/$(PVR_BUILD_DIR))

# Output directory for configuration, object code,
# final programs/libraries, and install/rc scripts.
#
BUILD	?= release
OUT		?= $(TOP)/binary_$(PVR_BUILD_DIR)_$(BUILD)
override OUT := $(if $(filter /%,$(OUT)),$(OUT),$(TOP)/$(OUT))

CONFIG_MK			:= $(OUT)/config.mk
CONFIG_H			:= $(OUT)/config.h
CONFIG_KERNEL_MK	:= $(OUT)/config_kernel.mk
CONFIG_KERNEL_H		:= $(OUT)/config_kernel.h
RGX_BVNC_H			:= $(OUT)/config_bvnc.h

# Convert commas to spaces in $(D). This is so you can say "make
# D=config-changes,freeze-config" and have $(filter config-changes,$(D))
# still work.
comma := ,
empty :=
space := $(empty) $(empty)
override D := $(subst $(comma),$(space),$(D))

# Create the OUT directory and delete any previous intermediary files
#
$(shell mkdir -p $(OUT))
$(shell \
	for file in $(CONFIG_MK).new $(CONFIG_H).new $(RGX_BVNC_H).new \
				$(CONFIG_KERNEL_MK).new $(CONFIG_KERNEL_H).new; do \
		rm -f $$file; \
	done)

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

# Check BVNC
ifeq ($(RGX_BVNC),)
$(error Need to define RGX_BVNC variable, e.g. RGX_BVNC=1.2.3.4)
endif

# Check rgxcore_BVNC.h file exist
RGX_BVNC_CORE_H := $(TOP)/hwdefs/km/rgxcore_$(RGX_BVNC).h
ifeq ($(wildcard $(RGX_BVNC_CORE_H)),)
$(error Invalid BVNC $(RGX_BVNC). The file $(RGX_BVNC_CORE_H) does not exist)
endif

$(shell cp $(RGX_BVNC_CORE_H) $(RGX_BVNC_H).new)


# Enforced dependencies. Move this to an include.
#
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

# The user didn't set CROSS_COMPILE. There's probably nothing wrong
# with that, but we'll let them know anyway.
#
ifeq ($(CROSS_COMPILE),)
$(warning CROSS_COMPILE is not set. Target components will be built with the host compiler)
endif

# There shouldn't have been two options with different names to
# set here (SUPPORT_ options implying components to build are
# architecturally deprecated), but maintain some compatibility
# here for a while.
#
ifeq ($(SUPPORT_PVR_REMOTE),1)
$(warning SUPPORT_PVR_REMOTE=1 is deprecated and will go away. Use PVR_REMOTE=1 instead.)
PVR_REMOTE := 1
endif

# The user is trying to set one of the old SUPPORT_ options on the
# command line or in the environment. This isn't supported any more
# and will often break the build. The user is generally only trying
# to remove a component from the list of targets to build, so we'll
# point them at the new way of doing this.
define sanity-check-support-option-origin
ifeq ($$(filter undefined file,$$(origin $(1))),)
$$(warning *** Setting $(1) via $$(origin $(1)) is deprecated)
$$(error If you are trying to disable a component, use e.g. EXCLUDED_APIS="opengles1 opengl")
endif
endef
$(foreach _o,SYS_CFLAGS SYS_CXXFLAGS SYS_EXE_LDFLAGS SYS_LIB_LDFLAGS,$(eval $(call sanity-check-support-option-origin,$(_o))))

# Check for words in EXCLUDED_APIS that aren't understood by the
# common/apis/*.mk files. This should be kept in sync with all the tests on
# EXCLUDED_APIS in those files
_excludable_apis := opencl opengl opengles1 opengles3 ews unittests scripts xorg xorg_unittests

_unrecognised := $(strip $(filter-out $(_excludable_apis),$(EXCLUDED_APIS)))
ifneq ($(_unrecognised),)
$(warning *** Unrecognised entries in EXCLUDED_APIS: $(_unrecognised))
$(warning *** EXCLUDED_APIS was set via: $(origin EXCLUDED_APIS))
$(error Excludable APIs are: $(_excludable_apis))
endif

# Build's selected list of components.
# - components.mk is a per-build file that specifies the components that are
#   to be built
-include components.mk

# PDUMP needs extra components
#
ifeq ($(PDUMP),1)
ifneq ($(COMPONENTS),)
COMPONENTS += pdump
endif
ifeq ($(SUPPORT_DRI_DRM),1)
EXTRA_PVRSRVKM_COMPONENTS += dbgdrv
else
KERNEL_COMPONENTS += dbgdrv
endif
endif

# PVRGDB needs extra components
#
ifeq ($(PVRGDB),1)
ifneq ($(COMPONENTS),)
COMPONENTS += pvrgdb
endif
endif

$(if $(filter config,$(D)),$(info Build configuration:))

################################# CONFIG ####################################

# If KERNELDIR is set, write it out to the config.mk, with
# KERNEL_COMPONENTS and KERNEL_ID
#
ifneq ($(strip $(KERNELDIR)),)
include ../kernel_version.mk
PVRSRV_MODULE_BASEDIR ?= /lib/modules/$(KERNEL_ID)/extra/
$(eval $(call KernelConfigMake,KERNELDIR,$(KERNELDIR)))
# Needed only by install script
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


# Ideally configured by platform Makefiles, as necessary
#

# Invariant options for Linux
#
$(eval $(call BothConfigC,LINUX,))

$(eval $(call BothConfigC,PVR_BUILD_DIR,"\"$(PVR_BUILD_DIR)\""))
$(eval $(call BothConfigC,PVR_BUILD_TYPE,"\"$(BUILD)\""))
$(eval $(call BothConfigC,PVRSRV_MODNAME,"\"pvrsrvkm\""))

$(eval $(call TunableBothConfigC,SUPPORT_RGX,1))

$(eval $(call BothConfigC,PVR_SECURE_HANDLES,))

$(eval $(call BothConfigC,RGXCONFIG,$(RGXCONFIG)))

ifneq ($(DISPLAY_CONTROLLER),)
$(eval $(call BothConfigC,DISPLAY_CONTROLLER,$(DISPLAY_CONTROLLER)))
endif


# Forming config options from make variables instead of using
# TunableUserConfig* is a confusing thing to do, but it's necessary in this
# case. FIXME: remove this when it's no longer needed
GLES_MILESTONE ?= 4

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
$(eval $(call TunableBothConfigC,RGXFW_ALIGNCHECKS,1))
$(eval $(call TunableBothConfigC,SUPPORT_RGXFW_LOG,1))
else ifeq ($(BUILD),release)
$(eval $(call BothConfigC,RELEASE,))
$(eval $(call TunableBothConfigMake,DEBUGLINK,1))
$(eval $(call TunableBothConfigC,RGXFW_ALIGNCHECKS,))
$(eval $(call TunableBothConfigC,SUPPORT_RGXFW_LOG,))
else ifeq ($(BUILD),timing)
$(eval $(call BothConfigC,TIMING,))
$(eval $(call TunableBothConfigMake,DEBUGLINK,1))
else
$(error BUILD= must be either debug, release or timing)
endif

# User-configurable options
#
$(eval $(call TunableBothConfigC,SUPPORT_PERCONTEXT_PB,1))
$(eval $(call TunableBothConfigC,SUPPORT_HW_RECOVERY,1))
$(eval $(call TunableBothConfigC,SUPPORT_PDUMP_MULTI_PROCESS,))
$(eval $(call TunableBothConfigC,SUPPORT_DBGDRV_EVENT_OBJECTS,1))
$(eval $(call TunableBothConfigC,PVR_DBG_BREAK_ASSERT_FAIL,))
$(eval $(call TunableBothConfigC,PDUMP,))
$(eval $(call TunableBothConfigC,NO_HARDWARE,))
$(eval $(call TunableBothConfigC,PDUMP_DEBUG_OUTFILES,))
$(eval $(call TunableBothConfigC,PVRSRV_RESET_ON_HWTIMEOUT,))
$(eval $(call TunableBothConfigC,SYS_USING_INTERRUPTS,1))
$(eval $(call TunableBothConfigC,PVRSRV_NEW_PVR_DPF,))
$(eval $(call TunableBothConfigC,PVRSRV_NEED_PVR_DPF,))
$(eval $(call TunableBothConfigC,PVRSRV_NEED_PVR_ASSERT,))
$(eval $(call TunableBothConfigC,PVRSRV_NEED_PVR_TRACE,))
$(eval $(call TunableBothConfigC,REFCOUNT_DEBUG,))
$(eval $(call TunableBothConfigC,DC_DEBUG,))
$(eval $(call TunableBothConfigC,SCP_DEBUG,))
$(eval $(call TunableBothConfigC,PHSHEAP_DEBUG,))
$(eval $(call TunableBothConfigC,CACHEFLUSH_TYPE,CACHEFLUSH_GENERIC))
$(eval $(call TunableBothConfigC,SUPPORT_SECURE_EXPORT,))
$(eval $(call TunableBothConfigC,SUPPORT_PMMIF,))

$(eval $(call TunableKernelConfigC,SUPPORT_LINUX_X86_WRITECOMBINE,1))
$(eval $(call TunableKernelConfigC,SUPPORT_LINUX_X86_PAT,1))
$(eval $(call TunableKernelConfigC,SYS_CUSTOM_POWERLOCK_WRAP,))
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


$(eval $(call TunableBothConfigMake,CACHEFLUSH_TYPE,CACHEFLUSH_GENERIC))
$(eval $(call TunableBothConfigMake,PDUMP,))
$(eval $(call TunableBothConfigMake,SUPPORT_SECURE_EXPORT,))
$(eval $(call TunableBothConfigMake,SUPPORT_PMMIF,))

$(eval $(call TunableBothConfigMake,OPTIM,))


$(eval $(call TunableBothConfigC,SUPPORT_RGXFW_UNITTESTS,))

$(eval $(call TunableBothConfigC,SUPPORT_META_SLAVE_BOOT,))



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
	@$(MAKE) -s --no-print-directory -C $(PVRSW_ROOT) \
		-f build/linux/prepare_tree.mk \
		LDM_PCI=$(LDM_PCI) \
		LDM_PLATFORM=$(LDM_PLATFORM) \
		LDM_DEVICE_CLASS=$(LDM_DEVICE_CLASS)
else
	@:
endif

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
