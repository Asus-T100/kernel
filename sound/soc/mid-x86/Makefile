# SST Platform Driver
snd-soc-sst-platform-objs := sst_platform.o sst_platform_dsp.o sst_platform_controls_ctp.o
obj-$(CONFIG_SND_SST_PLATFORM) += snd-soc-sst-platform.o

# Medfield board
snd-soc-mfld-machine-objs := mfld_common.o mfld_machine.o
obj-$(CONFIG_SND_MFLD_MACHINE) += snd-soc-mfld-machine.o

# Gilligan Island/Lexington board
snd-soc-mfld-machine-gi-objs := mfld_common.o mfld_machine_gi.o
obj-$(CONFIG_SND_MFLD_MACHINE_GI) += snd-soc-mfld-machine-gi.o

# Yukkabeach board
snd-soc-yukkabeach-machine-objs := mfld_common.o yukkabeach_machine.o
obj-$(CONFIG_SND_YUKKABEACH_MACHINE) += snd-soc-yukkabeach-machine.o

# Cloverview/Clovertrail+ board
snd-soc-ctp-machine-objs := ctp_common.o ctp_rhb_cs42l73.o ctp_vb_cs42l73.o
obj-$(CONFIG_SND_CTP_MACHINE) += snd-soc-ctp-machine.o

# Merrifield board
snd-mrfld-machine-lm49453-objs := mrfld_machine_lm49453.o
obj-$(CONFIG_SND_MRFLD_MACHINE) += snd-mrfld-machine-lm49453.o

# DSP driver
obj-$(CONFIG_SND_INTEL_SST) += sst/

# Audio Comms
obj-$(CONFIG_SND_SOC_COMMS_SSP) += mid_ssp.o
