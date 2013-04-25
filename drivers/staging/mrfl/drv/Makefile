# Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/mrfl
HDMI_DIR=drivers/staging/intel_media/otm_hdmi

ccflags-y += \
	-I$(INCDIR)/ \
	-I$(INCDIR)/interface \
	-I$(INCDIR)/rgx/include \
	-I$(INCDIR)/drv \
	-I$(HDMI_DIR)/os/android/include \
	-I$(HDMI_DIR)/pil/include \
	-I$(INCDIR)/drv/ospm \
	-Iinclude/linux \
	-Iinclude/drm \
	-I$(INCDIR)/../intel_media/video/common \
	-I$(INCDIR)/../intel_media/video/decode \
	-I$(INCDIR)/../intel_media/video/encode \
	-I$(INCDIR)/../intel_media/video/vsp

ccflags-y += -DANDROID -DLINUX -D_linux_ -D__KERNEL__

ccflags-$(CONFIG_DRM_MDFLD) += -DMEDFIELD
ccflags-$(CONFIG_DRM_MRFLD) += -DMERRIFIELD
ccflags-$(CONFIG_PCI_MSI) += -DCONFIG_PCI_MSI
ccflags-$(CONFIG_SUPPORT_VSP) += -DSUPPORT_VSP

obj-y += psb_bl.o \
	psb_dpst.o \
	psb_dpst_func.o \
	psb_drv.o \
	psb_fb.o \
	psb_gtt.o \
	psb_hotplug.o \
	psb_intel_display.o \
	mdfld_hdmi_audio.o \
	mdfld_msic.o \
	mdfld_csc.o \
	mdfld_intel_hdcp.o \
	psb_intel_hdmi.o \
	psb_socket.o \
	psb_umevents.o \
	mdfld_dsi_dbi.o \
	mdfld_dsi_dpi.o \
	mdfld_dsi_output.o \
	mdfld_output.o \
	mdfld_dsi_esd.o \
	mdfld_dsi_dbi_dsr.o \
	dispmgrnl.o \
	mdfld_dsi_pkg_sender.o \
	mrfld_clock.o \
	pmu_tng.o \
	psb_irq.o \
	tng_wa.o \
	jdi_vid.o \
	jdi_cmd.o \
	cmi_vid.o \
	cmi_cmd.o \
	dc_callbacks.o

obj-$(CONFIG_MID_DSI_DPU) += mdfld_dsi_dbi_dpu.o

