
obj-$(CONFIG_CHARGER_SNB2808)	+= snb2808_charger.o

ifneq ($(wildcard $(srctree)/drivers/power/SNB2808/$(TARGET_PRODUCT)), )
EXTRA_CFLAGS += -I$(srctree)/drivers/power/SNB2808/$(TARGET_PRODUCT)
else
EXTRA_CFLAGS += -I$(srctree)/drivers/power/SNB2808/default
endif
