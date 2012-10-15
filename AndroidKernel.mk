# This makefile is included from vendor/intel/*/AndroidBoard.mk.
# If the kernel source is present, AndroidBoard.mk will perform a kernel build
# otherise, AndroidBoard.mk will find the kernel binaries in a tarball.

# This can be overridden on the make command line.
TARGET_KERNEL_SOURCE_IS_PRESENT ?= true

.PHONY: get_kernel_from_source menuconfig

ifeq ($(BOARD_USE_64BIT_KERNEL),true)
KERNEL_BUILD_FLAGS := -B
endif

menuconfig get_kernel_from_source: $(MINIGZIP)
	+TARGET_TOOLS_PREFIX="$(ANDROID_BUILD_TOP)/$(TARGET_TOOLS_PREFIX)" DIFFCONFIGS="$(DIFFCONFIGS)" vendor/intel/support/kernel-build.sh $(KERNEL_BUILD_FLAGS) -c $(TARGET_DEVICE) -o $@

# This rule is useful for creating a kernel that will be
# shared with a tree that does not have kernel source.
make_kernel_tarball: get_kernel_from_source bootimage
	@echo Building kernel tarball: $(TARGET_KERNEL_TARBALL)
	@rm -rf $(PRODUCT_OUT)/kerneltarball
	@mkdir -p $(PRODUCT_OUT)/kerneltarball/root/lib/modules
	@cp $(PRODUCT_OUT)/root/lib/modules/* $(PRODUCT_OUT)/kerneltarball/lib/modules
	@cp $(PRODUCT_OUT)/bzImage $(PRODUCT_OUT)/kerneltarball/
	tar cvzf $(TARGET_KERNEL_TARBALL) -C $(PRODUCT_OUT)/kerneltarball bzImage root/lib/modules

$(PRODUCT_OUT)/bzImage: get_kernel_from_source
