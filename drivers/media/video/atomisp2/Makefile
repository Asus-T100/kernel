ifeq ($(CONFIG_X86_MRFLD), y)
	css_folder = css2400
	css_platform_folder = hive_isp_css_2400_system
else
	css_folder = css
	css_platform_folder = hive_isp_css_system
endif

atomisp-objs := \
		atomisp_ioctl.o \
		atomisp_cmd.o \
		atomisp_fops.o \
		atomisp_subdev.o \
		atomisp_csi2.o \
		atomisp_tpg.o \
		atomisp_file.o \
		atomisp_v4l2.o \
		atomisp_acc.o \
		mmu/isp_mmu.o \
		mmu/sh_mmu_mrfld.o \
		mmu/sh_mmu_mfld.o \
		hmm/hmm.o \
		hmm/hmm_bo.o \
		hmm/hmm_bo_dev.o \
		hmm/hmm_vm.o \
		hrt/hive_isp_css_mm_hrt.o

atomisp-objs += \
		$(css_folder)/sh_css.o \
		$(css_folder)/sh_css_binary.o \
		$(css_folder)/sh_css_debug.o  \
		$(css_folder)/sh_css_hrt.o  \
		$(css_folder)/sh_css_params.o \
		$(css_folder)/sh_css_sp.o \
		$(css_folder)/sh_css_rx.o \
		$(css_folder)/sh_css_sp_start.o \
		$(css_folder)/sh_css_metrics.o \
		$(css_folder)/sh_css_firmware.o \
		$(css_folder)/sh_css_accelerate.o \
		$(css_folder)/sh_css_refcount.o \
		$(css_folder)/sh_css_pipeline.o \
		$(css_folder)/sh_css_param_shading.o \
		$(css_folder)/$(css_platform_folder)/host/gdc.o \
		$(css_folder)/$(css_platform_folder)/host/irq.o \
		$(css_folder)/$(css_platform_folder)/host/mmu.o \
		$(css_folder)/$(css_platform_folder)/host/dma.o \
		$(css_folder)/$(css_platform_folder)/host/sp.o \
		$(css_folder)/$(css_platform_folder)/host/isp.o \
		$(css_folder)/hive_isp_css_shared/host/queue.o \
		$(css_folder)/hive_isp_css_shared/host/ia_css_i_rmgr_shared.o \
		$(css_folder)/hive_isp_css_shared/host/ia_css_i_rmgr_vbuf_shared.o \
		$(css_folder)/hive_isp_css_shared/host/tag.o \
		$(css_folder)/hive_isp_css_shared/host/sw_event.o \
		$(css_folder)/$(css_platform_folder)/host/fifo_monitor.o \
		$(css_folder)/$(css_platform_folder)/host/gp_device.o \
		$(css_folder)/$(css_platform_folder)/host/input_formatter.o \
		$(css_folder)/$(css_platform_folder)/device_access/device_access.o \
		$(css_folder)/$(css_platform_folder)/memory_access/memory_access.o

obj-$(CONFIG_VIDEO_ATOMISP_CSS15) = atomisp.o

INCLUDES := -I$(srctree)/$(src)/include \
	    -I$(srctree)/$(src)/include/atomisp \
	    -I$(src) \
	    -I$(src)/hrt \
	    -I$(srctree)/drivers/staging/intel_media/bc_video

INCLUDES += -I$(src)/$(css_folder) \
	    -I$(srctree)/$(src)/$(css_folder)/hrt \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include/stdint \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include/stdbool \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include/host \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include/device_access \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include/memory_access \
	    -I$(srctree)/$(src)/$(css_folder)/$(css_platform_folder) \
	    -I$(srctree)/$(src)/$(css_folder)/$(css_platform_folder)/host \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_shared \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_shared/host

DEFINES := -DHRT_HW -DHRT_ISP_CSS_CUSTOM_HOST -DHRT_USE_VIR_ADDRS -DHRT_KERNEL -D__HOST__
DEFINES += -DUSE_DYNAMIC_BIN
DEFINES += -DISP_POWER_GATING
DEFINES += -DUSE_INTERRUPTS
#DEFINES += -DUSE_SSSE3
DEFINES += -DPUNIT_CAMERA_BUSY
DEFINES += -DUSE_KMEM_CACHE

EXTRA_CFLAGS := $(INCLUDES) $(DEFINES) -fno-common -Werror
