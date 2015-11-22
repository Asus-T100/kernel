
# ASUS lid
obj-$(CONFIG_INPUT_LID) += platform_asus_lid.o

# +ASUSTEK BOARD INFO
obj-y += platform_asustek_boardinfo.o
obj-y += asustek_boardinfo.o
KBUILD_CFLAGS += -I$(srctree)/arch/x86/platform/asustek

# DDS: use default.
#ifeq ($(wildcard $(srctree)/arch/x86/platform/asustek/$(TARGET_PRODUCT)), )
	obj-y += default/
	KBUILD_CFLAGS += -I$(srctree)/arch/x86/platform/asustek/default
#else
#	obj-y += $(TARGET_PRODUCT)/
#	KBUILD_CFLAGS += -I$(srctree)/arch/x86/platform/asustek/$(TARGET_PRODUCT)
#endif
# -ASUSTEK BOARD INFO
