# This makefile is included from vendor/intel/*/AndroidBoard.mk.
# If the kernel source is present, AndroidBoard.mk will perform a kernel build
# otherise, AndroidBoard.mk will find the kernel binaries in a tarball.

# This can be overridden on the make command line.
TARGET_KERNEL_SOURCE_IS_PRESENT ?= true
# Force using bash as a shell, otherwise, on Ubuntu, dash will break some
# dependency due to its bad handling of echo \1
MAKE += SHELL=/bin/bash

ifeq ($(TARGET_KERNEL_SOURCE_IS_PRESENT),true)

.PHONY: menuconfig xconfig gconfig get_kernel_from_source
.PHONY: build_bzImage copy_modules_to_root
# This rule is useful for creating a kernel that will be
# shared with a tree that does not have kernel source.
make_kernel_tarball: get_kernel_from_source bootimage
	@echo Building kernel tarball: $(TARGET_KERNEL_TARBALL)
	@rm -rf $(PRODUCT_OUT)/kerneltarball
	@mkdir -p $(PRODUCT_OUT)/kerneltarball/root/lib/modules
	@cp $(PRODUCT_OUT)/root/lib/modules/* $(PRODUCT_OUT)/kerneltarball/lib/modules
	@cp $(PRODUCT_OUT)/bzImage $(PRODUCT_OUT)/kerneltarball/
	tar cvzf $(TARGET_KERNEL_TARBALL) -C $(PRODUCT_OUT)/kerneltarball bzImage root/lib/modules

ALL_KERNEL_MODULES :=

KERNEL_SOC_medfield := mfld
KERNEL_SOC_clovertrail := ctp
KERNEL_SOC_merrifield := mrfl
KERNEL_SOC_baytrail := byt
KERNEL_SOC_moorefield := moor
KERNEL_SOC_carboncanyon := crc

KERNEL_SOC := $(KERNEL_SOC_$(TARGET_BOARD_PLATFORM))

ifeq ($(KERNEL_SOC),)
$(error cannot build kernel, TARGET_BOARD_PLATFORM is not defined)
endif

ifeq ($(BOARD_USE_64BIT_KERNEL),true)
KERNEL_ARCH := x86_64
KERNEL_EXTRA_FLAGS := -B ANDROID_TOOLCHAIN_FLAGS=
else
KERNEL_ARCH := i386
KERNEL_EXTRA_FLAGS := ANDROID_TOOLCHAIN_FLAGS=-mno-android
endif

KERNEL_CROSS_COMP := $(notdir $(TARGET_TOOLS_PREFIX))
KERNEL_CCACHE :=$(firstword $(TARGET_CC))
KERNEL_PATH := $(ANDROID_BUILD_TOP)/vendor/intel/support
ifeq ($(notdir $(KERNEL_CCACHE)),ccache)
KERNEL_CROSS_COMP := "ccache $(KERNEL_CROSS_COMP)"
KERNEL_PATH := $(KERNEL_PATH):$(ANDROID_BUILD_TOP)/$(dir $(KERNEL_CCACHE))
endif

KERNEL_OUT_DIR := $(PRODUCT_OUT)/linux/kernel
KERNEL_MODULES_ROOT := $(PRODUCT_OUT)/root/lib/modules
KERNEL_CONFIG := $(KERNEL_OUT_DIR)/.config
KERNEL_BLD_FLAGS := \
    ARCH=$(KERNEL_ARCH) \
    O=../../$(KERNEL_OUT_DIR) \
    $(KERNEL_EXTRA_FLAGS)

KERNEL_BLD_ENV := CROSS_COMPILE=$(KERNEL_CROSS_COMP) \
    PATH=$(KERNEL_PATH):$(PATH)
KERNEL_FAKE_DEPMOD := $(KERNEL_OUT_DIR)/fakedepmod/lib/modules

KERNEL_DEFCONFIG := $(KERNEL_SRC_DIR)/arch/x86/configs/$(KERNEL_ARCH)_$(KERNEL_SOC)_defconfig
KERNEL_DIFFCONFIG_DIR ?= $(TARGET_DEVICE_DIR)
KERNEL_DIFFCONFIG := $(KERNEL_DIFFCONFIG_DIR)/$(TARGET_DEVICE)_diffconfig
KERNEL_VERSION_FILE := $(KERNEL_OUT_DIR)/include/config/kernel.release

$(KERNEL_CONFIG): $(KERNEL_DEFCONFIG) $(wildcard $(KERNEL_DIFFCONFIG))
	@echo Regenerating kernel config $(KERNEL_OUT_DIR)
	@mkdir -p $(KERNEL_OUT_DIR)
	@cat $^ > $@
	@$(KERNEL_BLD_ENV) $(MAKE) -C $(KERNEL_SRC_DIR) $(KERNEL_BLD_FLAGS) defoldconfig

build_bzImage: $(KERNEL_CONFIG) openssl $(MINIGZIP)
	@$(KERNEL_BLD_ENV) $(MAKE) -C $(KERNEL_SRC_DIR) $(KERNEL_BLD_FLAGS)
	@cp -f $(KERNEL_OUT_DIR)/arch/x86/boot/bzImage $(PRODUCT_OUT)/kernel

clean_kernel:
	@$(KERNEL_BLD_ENV) $(MAKE) -C $(KERNEL_SRC_DIR) $(KERNEL_BLD_FLAGS) clean

#need to do this to have a modules.dep correctly set.
#it is not optimized (copying all modules for each rebuild) but better than kernel-build.sh
#fake depmod with a symbolic link to have /lib/modules/$(version_tag)/xxxx.ko
copy_modules_to_root: build_bzImage
	@$(RM) -rf $(KERNEL_MODULES_ROOT)
	@mkdir -p $(KERNEL_MODULES_ROOT)
	@find $(KERNEL_OUT_DIR) -name "*.ko" -exec cp -f {} $(KERNEL_MODULES_ROOT)/ \;
	@find $(ALL_KERNEL_MODULES) -name "*.ko" -exec cp -f {} $(KERNEL_MODULES_ROOT)/ \;
	@mkdir -p $(KERNEL_FAKE_DEPMOD)
	@ln -fns $(ANDROID_BUILD_TOP)/$(KERNEL_MODULES_ROOT) $(KERNEL_FAKE_DEPMOD)/`cat $(KERNEL_VERSION_FILE)`
	@/sbin/depmod -b $(KERNEL_OUT_DIR)/fakedepmod `cat $(KERNEL_VERSION_FILE)`

get_kernel_from_source: copy_modules_to_root

#ramdisk depends on kernel modules
$(PRODUCT_OUT)/ramdisk.img: copy_modules_to_root

menuconfig xconfig gconfig: $(KERNEL_CONFIG)
	@$(KERNEL_BLD_ENV) $(MAKE) -C $(KERNEL_SRC_DIR) $(KERNEL_BLD_FLAGS) $@
	@./$(KERNEL_SRC_DIR)/scripts/diffconfig -m $(KERNEL_DEFCONFIG) $(KERNEL_OUT_DIR)/.config > $(KERNEL_DIFFCONFIG)
	@echo ===========
	@echo $(KERNEL_DIFFCONFIG) has been modified !
	@echo ===========

#used to build out-of-tree kernel modules
#$(1) is source path relative Android top, $(2) is module name
#$(3) is extra flags
define build_kernel_module
.PHONY: $(2)

$(2): build_bzImage
	@echo Building kernel module $(2) in $(1)
	@mkdir -p $(KERNEL_OUT_DIR)/../../$(1)
	@+$(KERNEL_BLD_ENV) $(MAKE) -C $(KERNEL_SRC_DIR) $(KERNEL_BLD_FLAGS) M=../../$(1) $(3)

$(2)_clean:
	@echo Cleaning kernel module $(2) in $(1)
	@$(KERNEL_BLD_ENV) $(MAKE) -C $(KERNEL_SRC_DIR) $(KERNEL_BLD_FLAGS) M=../../$(1) clean

copy_modules_to_root: $(2)

clean_kernel: $(2)_clean

ALL_KERNEL_MODULES += $(PRODUCT_OUT)/$(1)
endef
endif #TARGET_KERNEL_SOURCE_IS_PRESENT
