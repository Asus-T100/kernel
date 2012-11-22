# Makefile for Intel MID Audio drivers

snd-intel-sst-objs := intel_sst.o intel_sst_ipc.o intel_sst_stream.o intel_sst_drv_interface.o  intel_sst_dsp.o intel_sst_pvt.o intel_sst_app_interface.o

ifdef CONFIG_DEBUG_FS
	snd-intel-sst-objs += intel_sst_debug.o
endif

obj-$(CONFIG_SND_INTEL_SST) += snd-intel-sst.o


ifdef CONFIG_SND_MRFLD_MACHINE
ccflags-y += -DMRFLD_WORD_WA
endif
