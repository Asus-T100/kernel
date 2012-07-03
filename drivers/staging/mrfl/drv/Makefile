 # Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/mrfl

ccflags-y += \
	-I$(INCDIR)/ \
	-I$(INCDIR)/interface \
	-I$(INCDIR)/drv \
	-I$(INCDIR)/imgv \
	-I$(INCDIR)/vsp \
	-Iinclude/linux \
	-Iinclude/drm

ccflags-y += -DANDROID -D_linux_ -D__KERNEL__

ccflags-$(CONFIG_DRM_MDFLD) += -DMEDFIELD
ccflags-$(CONFIG_DRM_MRFLD) += -DMERRIFIELD
ccflags-$(CONFIG_PCI_MSI) += -DCONFIG_PCI_MSI

obj-y += psb_bl.o \
	psb_dpst.o \
	psb_dpst_func.o \
	psb_drv.o \
	psb_fb.o \
	psb_gtt.o \
	psb_hotplug.o \
	psb_intel_opregion.o \
	psb_intel_display.o \
	psb_intel_i2c.o \
	psb_intel_lvds.o \
	psb_intel_modes.o \
	mdfld_hdmi_audio.o \
	mdfld_msic.o \
	mdfld_csc.o \
	mdfld_intel_hdcp.o \
	psb_intel_hdmi.o \
	psb_intel_hdmi_i2c.o \
	psb_reset.o \
	psb_schedule.o \
	psb_sgx.o \
	psb_socket.o \
	psb_umevents.o \
	psb_intel_dsi.o \
	psb_intel_dsi_aava.o \
	mdfld_dsi_dbi.o \
	mdfld_dsi_dpi.o \
	mdfld_dsi_output.o \
	mdfld_output.o \
	tpo_cmd.o \
	tpo_vid.o \
	tmd_vid.o \
	pyr_cmd.o \
	dispmgrnl.o \
	mdfld_dsi_pkg_sender.o \
	mrfld_clock.o \
	psb_powermgmt.o \
	psb_irq.o \
	tmd_vid.o \
	tmd_6x10_vid.o \
	dc_callbacks.o \
	mrfld_s3d_ovl.o

obj-$(CONFIG_MID_DSI_DPU) += mdfld_dsi_dbi_dpu.o

