########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

define must-be-defined
$(if $(filter undefined,$(origin $(1))),$(error In makefile $(THIS_MAKEFILE): $$($(1)) must be defined),)
endef

define must-be-nonempty
$(if $(strip $($(1))),,$(error In makefile $(THIS_MAKEFILE): $$($(1)) must contain a value))
endef

define directory-must-exist
$(if $(wildcard $(abspath $(1)/)),,$(error Directory $(1) must exist))
endef

define one-word-only
$(if $(filter-out $(firstword $($(1))),$($(1))),$(error In makefile $(THIS_MAKEFILE): $$($(1)) must contain only one word),)
endef

define target-intermediates-of
$(addprefix $(TARGET_OUT)/intermediates/$(1)/,$(2))
endef

define host-intermediates-of
$(addprefix $(HOST_OUT)/intermediates/$(1)/,$(2))
endef

define module-library
$(patsubst lib%.so,%,$(if $($(1)_target),$($(1)_target),$(1).so))
endef

# This is done to allow module type makefiles to use $(THIS_MAKEFILE)
define register-module
INTERNAL_MAKEFILE_FOR_MODULE_$(1) := $(THIS_MAKEFILE)
endef

define process-module
THIS_MODULE := $(1)
THIS_MAKEFILE := $(INTERNAL_MAKEFILE_FOR_MODULE_$(1))
include $$(MAKE_TOP)/this_makefile.mk
$$(call must-be-nonempty,THIS_MAKEFILE)
$$(call must-be-nonempty,$(1)_type)
MODULE_HOST_BUILD := $$(if $(filter host_%,$($(1)_type)),true,)
include $$(MAKE_TOP)/moduledefs.mk
include $$(MAKE_TOP)/$$(patsubst host_%,%,$($(1)_type)).mk
INTERNAL_TARGETS_FOR_$(THIS_MODULE) := $(MODULE_TARGETS)
endef

# This can be used by module_type.mk files to indicate that they can't be
# built as host_module_type
define target-build-only
$(if $(filter true,$(MODULE_HOST_BUILD)),$(error In makefile $(THIS_MAKEFILE): Module $(THIS_MODULE) attempted to build a host $(1), which is not supported))
endef

define relative-to-top
$(patsubst $(TOP)/%,%,$(1))
endef

define cc-check
$(shell \
	CC_CHECK=$(patsubst @%,%,$(CC_CHECK)) && \
	$(patsubst @%,%,$(CHMOD)) +x $$CC_CHECK && \
	$$CC_CHECK --cc "$(1)" --out "$(2)" $(3))
endef

define cc-is-64bit
$(call cc-check,$(1),$(OUT),--64)
endef

define cc-option
$(call cc-check,$(patsubst @%,%,$(CC)),$(OUT),$(1))
endef

define cxx-option
$(call cc-check,$(patsubst @%,%,$(CXX)),$(OUT),$(1))
endef

define host-cc-option
$(call cc-check,$(patsubst @%,%,$(HOST_CC)),$(OUT),$(1))
endef

define kernel-cc-option
$(call cc-check,$(KERNEL_CROSS_COMPILE)gcc,$(OUT),$(1))
endef

# Turn a particular warning on, or explicitly turn it off, depending on
# the value of W. The "-W" or "-Wno-" part of the warning need not be
# specified.
define cc-optional-warning
$(call cc-option,-W$(if $(W),,no-)$(patsubst -W%,%,$(patsubst -Wno-%,%,$(1))))
endef

define host-cc-optional-warning
$(call host-cc-option,-W$(if $(W),,no-)$(patsubst -W%,%,$(patsubst -Wno-%,%,$(1))))
endef

define kernel-cc-optional-warning
$(call kernel-cc-option,-W$(if $(W),,no-)$(patsubst -W%,%,$(patsubst -Wno-%,%,$(1))))
endef

define module-info-line
$(if $(filter modules,$(D)),$(info $(1)),)
endef

# $(call if-exists,A,B) => A if A is a file which exists, otherwise B
define if-exists
$(if $(wildcard $(1)),$(1),$(2))
endef

#
# Joins a given list of strings together with the given separator.
#
# (1): the list of strings to join
# (2): the separator to use for joining
#
NOOP=
SPACE=$(NOOP) $(NOOP)
define list-join
$(subst $(SPACE),$(2),$(strip $(1)))
endef

# 
# Check if a given path is absolute
#
# $(1): path to check
# $(2): return when true
# $(3): return when false
#
define if-abs-path
$(if $(filter /%,$(1)),$(2),$(3))
endef

# 
# Add a prefix to every path in a list, when the path isn't absolute.
#
# $(1): prefix to add
# $(2): list of paths
#
define addprefix-ifnot-abs
$(foreach _path,$(2),$(call if-abs-path,$(_path),$(_path),$(1)$(_path)))
endef
