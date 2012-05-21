########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

$(TARGET_OUT)/kbuild/Makefile: $(MAKE_TOP)/kbuild/Makefile.template
	@[ ! -e $(dir $@) ] && mkdir -p $(dir $@) || true
	$(CP) -f $< $@

# We need to make INTERNAL_KBUILD_MAKEFILES absolute because the files will be
# read while chdir'd into $(KERNELDIR)
INTERNAL_KBUILD_MAKEFILES := $(abspath $(foreach _m,$(KERNEL_COMPONENTS) $(EXTRA_PVRSRVKM_COMPONENTS),$(if $(INTERNAL_KBUILD_MAKEFILE_FOR_$(_m)),$(INTERNAL_KBUILD_MAKEFILE_FOR_$(_m)),$(error Unknown kbuild module "$(_m)"))))
INTERNAL_KBUILD_OBJECTS := $(foreach _m,$(KERNEL_COMPONENTS),$(if $(INTERNAL_KBUILD_OBJECTS_FOR_$(_m)),$(INTERNAL_KBUILD_OBJECTS_FOR_$(_m)),$(error BUG: Unknown kbuild module "$(_m)" should have been caught earlier)))
INTERNAL_EXTRA_KBUILD_OBJECTS := $(foreach _m,$(EXTRA_PVRSRVKM_COMPONENTS),$(if $(INTERNAL_KBUILD_OBJECTS_FOR_$(_m)),$(INTERNAL_KBUILD_OBJECTS_FOR_$(_m)),$(error BUG: Unknown kbuild module "$(_m)" should have been caught earlier)))
.PHONY: kbuild kbuild_clean kbuild_check

kbuild_check:
	@: $(if $(strip $(KERNELDIR)),,$(error KERNELDIR must be set))
	@: $(call directory-must-exist,$(KERNELDIR))
	@: $(foreach _m,$(ALL_KBUILD_MODULES),$(if $(wildcard $(abspath $(INTERNAL_KBUILD_MAKEFILE_FOR_$(_m)))),,$(error In makefile $(INTERNAL_MAKEFILE_FOR_MODULE_$(_m)): Module $(_m) requires kbuild makefile $(INTERNAL_KBUILD_MAKEFILE_FOR_$(_m)), which is missing)))

# Services server headers are generated as part of running the bridge
# generator, which might be included in KM code. So as well as depending on
# the kbuild Makefile, we need to make kbuild also depend on each bridge
# module (including direct bridges), so that 'make kbuild' in a clean tree
# works.
kbuild: kbuild_check $(TARGET_OUT)/kbuild/Makefile $(BRIDGES) $(DIRECT_BRIDGES)
	@$(MAKE) -Rr --no-print-directory -C $(KERNELDIR) M=$(abspath $(TARGET_OUT)/kbuild) \
		INTERNAL_KBUILD_MAKEFILES="$(INTERNAL_KBUILD_MAKEFILES)" \
		INTERNAL_KBUILD_OBJECTS="$(INTERNAL_KBUILD_OBJECTS)" \
		INTERNAL_EXTRA_KBUILD_OBJECTS="$(INTERNAL_EXTRA_KBUILD_OBJECTS)" \
		EXTRA_KBUILD_SOURCE="$(EXTRA_KBUILD_SOURCE)" \
		CROSS_COMPILE="$(CCACHE) $(KERNEL_CROSS_COMPILE)" \
		EXTRA_CFLAGS="$(ALL_KBUILD_CFLAGS)" \
		V=$(V) W=$(W) \
		TOP=$(TOP)
ifeq ($(DEBUGLINK),1)
	@for kernel_module in $(addprefix $(TARGET_OUT)/kbuild/,$(INTERNAL_KBUILD_OBJECTS:.o=.ko)); do \
		$(patsubst @%,%,$(STRIP)) --strip-unneeded $$kernel_module; \
	done
endif
	@for kernel_module in $(addprefix $(TARGET_OUT)/kbuild/,$(INTERNAL_KBUILD_OBJECTS:.o=.ko)); do \
		cp $$kernel_module $(TARGET_OUT); \
	done

kbuild_clean: kbuild_check $(TARGET_OUT)/kbuild/Makefile
	@$(MAKE) -Rr --no-print-directory -C $(KERNELDIR) M=$(abspath $(TARGET_OUT)/kbuild) \
		INTERNAL_KBUILD_MAKEFILES="$(INTERNAL_KBUILD_MAKEFILES)" \
		INTERNAL_KBUILD_OBJECTS="$(INTERNAL_KBUILD_OBJECTS)" \
		INTERNAL_EXTRA_KBUILD_OBJECTS="$(INTERNAL_EXTRA_KBUILD_OBJECTS)" \
		EXTRA_KBUILD_SOURCE="$(EXTRA_KBUILD_SOURCE)" \
		CROSS_COMPILE="$(CCACHE) $(KERNEL_CROSS_COMPILE)" \
		EXTRA_CFLAGS="$(ALL_KBUILD_CFLAGS)" \
		V=$(V) W=$(W) \
		TOP=$(TOP) clean

kbuild_install: kbuild_check $(TARGET_OUT)/kbuild/Makefile
	@: $(if $(strip $(DISCIMAGE)),,$(error $$(DISCIMAGE) was empty or unset while trying to use it to set INSTALL_MOD_PATH for modules_install))
	@$(MAKE) -Rr --no-print-directory -C $(KERNELDIR) M=$(abspath $(TARGET_OUT)/kbuild) \
		INTERNAL_KBUILD_MAKEFILES="$(INTERNAL_KBUILD_MAKEFILES)" \
		INTERNAL_KBUILD_OBJECTS="$(INTERNAL_KBUILD_OBJECTS)" \
		INTERNAL_EXTRA_KBUILD_OBJECTS="$(INTERNAL_EXTRA_KBUILD_OBJECTS)" \
		EXTRA_KBUILD_SOURCE="$(EXTRA_KBUILD_SOURCE)" \
		CROSS_COMPILE="$(CCACHE) $(KERNEL_CROSS_COMPILE)" \
		EXTRA_CFLAGS="$(ALL_KBUILD_CFLAGS)" \
		INSTALL_MOD_PATH="$(DISCIMAGE)" \
		V=$(V) W=$(W) \
		TOP=$(TOP) modules_install
