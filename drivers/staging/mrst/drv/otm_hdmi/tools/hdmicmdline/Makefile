obj-m = hdmicmd.o
# TODO: Enable the below logic to config KDIR once the OTM MAKEFILE is updated.
#ifeq ($(CONFIG_MDFD_HDMI_REDRIDGE),y)
KDIR := ../../../../../../../../../../out/target/product/mfld_dv10/obj/kernel
#else ifeq ($(CONFIG_MDFD_HDMI_PR2),y)
#KDIR := ../../../../../../../../../../out/target/product/mfld_pr2/kernel_build
#endif
PWD := $(shell pwd)
all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean
