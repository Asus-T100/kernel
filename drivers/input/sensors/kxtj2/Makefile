obj-$(CONFIG_SENSORS_KXTJ2)  += kxtj2.o

ifneq ($(wildcard $(srctree)/include/linux/input/sensors/$(TARGET_PRODUCT)), )
EXTRA_CFLAGS += -I$(srctree)/include/linux/input/sensors/$(TARGET_PRODUCT)
else
EXTRA_CFLAGS += -I$(srctree)/include/linux/input/sensors/default
endif
