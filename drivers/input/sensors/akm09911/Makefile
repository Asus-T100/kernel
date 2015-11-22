obj-$(CONFIG_SENSORS_AK09911)	+= akm09911.o

ifneq ($(wildcard $(srctree)/include/linux/input/sensors/$(TARGET_PRODUCT)), )
EXTRA_CFLAGS += -I$(srctree)/include/linux/input/sensors/$(TARGET_PRODUCT)
else
EXTRA_CFLAGS += -I$(srctree)/include/linux/input/sensors/default
endif
