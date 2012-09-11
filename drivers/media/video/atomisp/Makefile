atomisp-objs := \
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
		mmu/isp_mmu.o \
		mmu/sh_mmu_mrfld.o \
		mmu/sh_mmu_mfld.o \
		hmm/hmm.o \
		hmm/hmm_bo.o \
		hmm/hmm_bo_dev.o \
		hmm/hmm_vm.o \
		hrt/hive_isp_css_mm_hrt.o \
		atomisp_ioctl.o \
		atomisp_cmd.o \
		atomisp_fops.o \
		atomisp_subdev.o \
		atomisp_csi2.o \
		atomisp_tpg.o \
		atomisp_file.o \
		atomisp_v4l2.o \
		atomisp_mem_ops.o

obj-$(CONFIG_VIDEO_ATOMISP_CSS10) = atomisp.o

INCLUDES := -I$(srctree)/$(src)/include \
	    -I$(srctree)/$(src)/include/atomisp \
	    -I$(src) \
	    -I$(src)/hrt \
	    -I$(src)/css \
	    -I$(srctree)/$(src)/css/hrt \
	    -I$(srctree)/$(src)/css/hrt_2400 \
	    -I$(srctree)/drivers/staging/intel_media/bc_video

DEFINES := -DHRT_HW -DHRT_ISP_CSS_CUSTOM_HOST -DHRT_USE_VIR_ADDRS -DHRT_KERNEL
DEFINES += -DUSE_DYNAMIC_BIN
DEFINES += -DISP_POWER_GATING
DEFINES += -DUSE_INTERRUPTS
#DEFINES += -DUSE_SSSE3
DEFINES += -DPUNIT_CAMERA_BUSY
DEFINES += -DUSE_KMEM_CACHE

EXTRA_CFLAGS := $(INCLUDES) $(DEFINES) -fno-common -Werror
