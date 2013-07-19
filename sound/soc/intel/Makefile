#EXTRA CFLAGS
ccflags-y += -Werror

# SST Platform Driver
PLATFORM_LIBS = platform-libs/controls_v1.o platform-libs/controls_v2.o \
		platform-libs/ipc_lib_v2.o

snd-soc-sst-platform-objs := pcm.o compress.o $(PLATFORM_LIBS)
obj-$(CONFIG_SND_SST_PLATFORM) += snd-soc-sst-platform.o

# Relevant Machine driver
obj-$(CONFIG_SND_SST_MACHINE) += board/

# DSP driver
obj-$(CONFIG_SND_INTEL_SST) += sst/

# Audio Comms
obj-$(CONFIG_SND_SOC_COMMS_SSP) += ssp/
