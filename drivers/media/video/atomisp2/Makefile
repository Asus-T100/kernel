atomisp-objs := \
		atomisp_ioctl.o \
		atomisp_cmd.o \
		atomisp_fops.o \
		atomisp_subdev.o \
		atomisp_csi2.o \
		atomisp_tpg.o \
		atomisp_file.o \
		atomisp_v4l2.o \
		mmu/isp_mmu.o \
		mmu/sh_mmu_mrfld.o \
		mmu/sh_mmu_mfld.o \
		hmm/hmm.o \
		hmm/hmm_bo.o \
		hmm/hmm_bo_dev.o \
		hmm/hmm_vm.o \
		hrt/hive_isp_css_mm_hrt.o

atomisp-objs += \
		css/sh_css.o \
		css/sh_css_binary.o \
		css/sh_css_debug.o  \
		css/sh_css_hrt.o  \
		css/sh_css_params.o \
		css/sh_css_sp.o \
		css/sh_css_rx.o \
		css/sh_css_sp_start.o \
		css/sh_css_metrics.o \
		css/sh_css_firmware.o \
		css/sh_css_accelerate.o \
		css/sh_css_refcount.o \
		css/sh_css_pipeline.o \
		css/sh_css_param_shading.o \
		css/hive_isp_css_system/host/gdc.o \
		css/hive_isp_css_system/host/irq.o \
		css/hive_isp_css_system/host/mmu.o \
		css/hive_isp_css_system/host/dma.o \
		css/hive_isp_css_system/host/sp.o \
		css/hive_isp_css_system/host/isp.o \
		css/hive_isp_css_shared/host/queue.o \
		css/hive_isp_css_shared/host/tag.o \
                css/hive_isp_css_shared/host/sw_event.o \
		css/hive_isp_css_system/host/fifo_monitor.o \
		css/hive_isp_css_system/host/gp_device.o \
		css/hive_isp_css_system/host/input_formatter.o \
		css/hive_isp_css_system/device_access/device_access.o \
		css/hive_isp_css_system/memory_access/memory_access.o

obj-$(CONFIG_VIDEO_ATOMISP_CSS15) = atomisp.o

INCLUDES := -I$(srctree)/$(src)/include \
	    -I$(srctree)/$(src)/include/atomisp \
	    -I$(src) \
	    -I$(src)/hrt \
	    -I$(srctree)/drivers/staging/intel_media/bc_video

INCLUDES += -I$(src)/css \
	    -I$(srctree)/$(src)/css/hrt \
	    -I$(srctree)/$(src)/css/hive_isp_css_include \
	    -I$(srctree)/$(src)/css/hive_isp_css_include/stdint \
	    -I$(srctree)/$(src)/css/hive_isp_css_include/stdbool \
	    -I$(srctree)/$(src)/css/hive_isp_css_include/host \
	    -I$(srctree)/$(src)/css/hive_isp_css_include/device_access \
	    -I$(srctree)/$(src)/css/hive_isp_css_include/memory_access \
	    -I$(srctree)/$(src)/css/hive_isp_css_system \
	    -I$(srctree)/$(src)/css/hive_isp_css_system/host \
	    -I$(srctree)/$(src)/css/hive_isp_css_shared \
	    -I$(srctree)/$(src)/css/hive_isp_css_shared/host

DEFINES := -DHRT_HW -DHRT_ISP_CSS_CUSTOM_HOST -DHRT_USE_VIR_ADDRS -DHRT_KERNEL -D__HOST__
DEFINES += -DUSE_DYNAMIC_BIN
DEFINES += -DISP_POWER_GATING
DEFINES += -DUSE_INTERRUPTS
#DEFINES += -DUSE_SSSE3
DEFINES += -DPUNIT_CAMERA_BUSY
DEFINES += -DUSE_KMEM_CACHE

EXTRA_CFLAGS := $(INCLUDES) $(DEFINES) -fno-common -Werror
