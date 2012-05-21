########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

# Rules for running the bridge generator (create_bridge.pl) to create bridge
# headers and source files

$(call one-word-only,MODULE_SOURCES)
MODULE_TEMPLATES_DIR := \
 $(if $($(THIS_MODULE)_templates),$($(THIS_MODULE)_templates),services/bridge/env/linux)
$(call one-word-only,MODULE_TEMPLATES_DIR)
_PERL := $(patsubst @%,%,$(PERL))
# MODULE_BRIDGE_DEPS contains the generated makefile fragment that
# create_bridge.pl -depmod generates
MODULE_BRIDGE_DEPS := \
 $(strip \
 $(shell $(_PERL) services/bridge/create_bridge.pl \
	-depmod -templatepath $(MODULE_TEMPLATES_DIR) \
	$(MODULE_SOURCES) -odir $(MODULE_INTERMEDIATES_DIR)))
$(if $(MODULE_BRIDGE_DEPS),,\
 $(error In makefile $(THIS_MAKEFILE): Module $(THIS_MODULE) produced \
	empty bridge dependencies, possibly due to an error in \
	$(strip $(MODULE_SOURCES)) (see above)))
MODULE_TARGETS := $(strip $(shell echo $(MODULE_BRIDGE_DEPS) | cut -d: -f1))
$(call must-be-nonempty,MODULE_TARGETS)

$(call module-info-line,[$(THIS_MODULE)] $(Host_or_target) generated bridge: $(MODULE_TARGETS))
$(call target-build-only,bridge)

# We should only run create_bridge.pl once for each bridge, but it might
# generate lots of files. _RULE_TARGET is the output file we will create a
# rule for, which runs the bridge generator. _OTHER_TARGETS are the other
# output files, which will depend on _RULE_TARGET.
_RULE_TARGET := $(firstword $(MODULE_TARGETS))
_OTHER_TARGETS := $(strip $(wordlist 2,99999,$(MODULE_TARGETS)))

.PHONY: $(THIS_MODULE)
$(THIS_MODULE): $(MODULE_TARGETS)

$(eval $(MODULE_BRIDGE_DEPS))

# This rule runs the bridge generator
$(_RULE_TARGET): MODULE_INTERMEDIATES_DIR := $(MODULE_INTERMEDIATES_DIR)
$(_RULE_TARGET): MODULE_TEMPLATES_DIR := $(MODULE_TEMPLATES_DIR)
$(_RULE_TARGET): | $(MODULE_INTERMEDIATES_DIR)
$(_RULE_TARGET): $(MODULE_SOURCES)
	$(if $(V),,@echo "  BRIDGE  " $(call relative-to-top,$<))
	$(PERL) services/bridge/create_bridge.pl \
		-templatepath $(MODULE_TEMPLATES_DIR) \
		$< \
		-odir $(MODULE_INTERMEDIATES_DIR)

ifneq ($(_OTHER_TARGETS),)
# These rules indicate that the other files are generated as a side effect of
# running the bridge generator
$(_OTHER_TARGETS): $(_RULE_TARGET) | $(MODULE_INTERMEDIATES_DIR)
	$(TOUCH) -c $@
endif
