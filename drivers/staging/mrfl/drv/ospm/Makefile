 # Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/mrfl
HDMI_DIR=drivers/staging/intel_media/otm_hdmi

ccflags-y += \
	-I$(INCDIR)/ \
	-I$(INCDIR)/interface \
	-I$(INCDIR)/rgx/include \
	-I$(INCDIR)/drv \
	-I$(INCDIR)/drv/ospm \
	-I$(INCDIR)/imgv \
	-I$(INCDIR)/vsp \
	-I$(HDMI_DIR)/os/android/include \
	-Iinclude/linux \
	-Iinclude/drm \
	-I$(INCDIR)/../intel_media/video/common \
	-I$(INCDIR)/../intel_media/video/vsp \
	-I$(INCDIR)/../intel_media/video/encode \
	-I$(INCDIR)/../intel_media/video/decode

ccflags-y += -DANDROID -DLINUX -D_linux_ -D__KERNEL__

ccflags-$(CONFIG_DRM_MDFLD) += -DMEDFIELD
ccflags-$(CONFIG_DRM_MRFLD) += -DMERRIFIELD
ccflags-$(CONFIG_PCI_MSI) += -DCONFIG_PCI_MSI

obj-y += pwr_mgmt.o \
	gfx_ospm.o \
	gfx_rtpm.o \
	dc_ospm.o \
	video_ospm.o \
	early_suspend.o
