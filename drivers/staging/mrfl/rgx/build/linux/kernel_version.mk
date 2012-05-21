########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

$(if $(KERNELDIR),,$(error KERNELDIR must be set to obtain a version))

override KERNEL_VERSION := \
 $(shell grep "^VERSION = " $(KERNELDIR)/Makefile | cut -f3 -d' ')
override KERNEL_PATCHLEVEL := \
 $(shell grep "^PATCHLEVEL = " $(KERNELDIR)/Makefile | cut -f3 -d' ')
override KERNEL_SUBLEVEL := \
 $(shell grep "^SUBLEVEL = " $(KERNELDIR)/Makefile | cut -f3 -d' ')
override KERNEL_EXTRAVERSION := \
 $(shell grep "^EXTRAVERSION = " $(KERNELDIR)/Makefile | cut -f3 -d' ')

# Break the kernel version up into a space separated list
kernel_version_as_list := $(KERNEL_VERSION) \
				$(KERNEL_PATCHLEVEL) \
				$(KERNEL_SUBLEVEL) \
				$(patsubst .%,%,$(KERNEL_EXTRAVERSION))

# The base ID doesn't have to be accurate; we only use it for
# feature checks which will not care about extraversion bits
#
override KERNEL_BASE_ID := \
 $(KERNEL_VERSION).$(KERNEL_PATCHLEVEL).$(KERNEL_SUBLEVEL)

# Try to get the kernel ID from the kernel.release file.
# 
KERNEL_ID ?= \
 $(shell cat $(KERNELDIR)/include/config/kernel.release 2>/dev/null)

# If the kernel ID isn't set yet, try to set it from the UTS_RELEASE
# macro.
#
ifeq ($(strip $(KERNEL_ID)),)
KERNEL_ID := \
 $(shell grep -h '\#define UTS_RELEASE' \
	$(KERNELDIR)/include/linux/* | cut -f3 -d' ' | sed s/\"//g)
endif

ifeq ($(strip $(KERNEL_ID)),)
KERNEL_ID := \
 $(KERNEL_VERSION).$(KERNEL_PATCHLEVEL).$(KERNEL_SUBLEVEL)$(KERNEL_EXTRAVERSION)
endif

# Return 1 if the kernel version is at least the value passed to the
# function, else return nothing.
# Examples
# 	$(call kernel-version-at-least,2,6,35)
# 	$(call kernel-version-at-least,2,6,35,7)
#
define kernel-version-at-least
$(shell set -- $(kernel_version_as_list); \
	Y=true; \
	for D in $1 $2 $3 $4; \
	do \
		[ $$1 ] || break; \
		[ $$1 -eq $$D ] && { shift; continue; };\
		[ $$1 -lt $$D ] && Y=; \
		break; \
	done; \
	echo $$Y)
endef
