#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
# sensor drivers
obj-$(CONFIG_SENSORS_KXTJ2)     += kxtj2/
obj-$(CONFIG_SENSORS_AK09911)   += akm09911/
obj-$(CONFIG_SENSORS_CAP1106)   += cap1106/
obj-$(CONFIG_INPUT_LID)  += lid.o

ifneq ($(wildcard $(srctree)/include/linux/input/sensors/$(TARGET_PRODUCT)), )
EXTRA_CFLAGS += -I$(srctree)/include/linux/input/sensors/$(TARGET_PRODUCT)
else
EXTRA_CFLAGS += -I$(srctree)/include/linux/input/sensors/default
endif
