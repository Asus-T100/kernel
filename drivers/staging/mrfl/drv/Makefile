 # Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/mrfl

ccflags-y += \
	-I$(INCDIR)/ \
	-I$(INCDIR)/interface \
	-I$(INCDIR)/rgx/include \
	-I$(INCDIR)/drv \
	-I$(INCDIR)/imgv \
	-I$(INCDIR)/vsp \
	-I$(INCDIR)/drv/otm_hdmi/os/android/include \
	-I$(INCDIR)/drv/otm_hdmi/pil/include \
	-Iinclude/linux \
	-Iinclude/drm

ccflags-y += -DANDROID -DLINUX -D_linux_ -D__KERNEL__

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
	psb_intel_display.o \
	mdfld_hdmi_audio.o \
	mdfld_msic.o \
	mdfld_csc.o \
	mdfld_intel_hdcp.o \
	psb_intel_hdmi.o \
	psb_schedule.o \
	psb_sgx.o \
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
	psb_powermgmt.o \
	psb_irq.o \
	tng_wa.o \
	jdi_vid.o \
	dc_callbacks.o \
	mrfl_display_po.o

obj-$(CONFIG_MID_DSI_DPU) += mdfld_dsi_dbi_dpu.o

