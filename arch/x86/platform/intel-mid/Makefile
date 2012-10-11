CFLAGS_intel_soc_pm_debug.o := -Werror
CFLAGS_intel_soc_pmu.o := -Werror
CFLAGS_intel_soc_mdfld.o := -Werror
CFLAGS_intel_soc_clv.o := -Werror
CFLAGS_intel_soc_mdfld_clv_common.o := -Werror
CFLAGS_intel_soc_mrfld.o := -Werror

# platform configuration for board devices
obj-y += device_libs/

# INTEL-MID specific files
obj-$(CONFIG_X86_INTEL_MID) += intel_mid_vrtc.o
obj-$(CONFIG_EARLY_PRINTK_INTEL_MID) += early_printk_intel_mid.o
obj-$(CONFIG_X86_INTEL_MID) += intel-mid.o
obj-$(CONFIG_X86_INTEL_MID) += intel_mid_scu.o
obj-$(CONFIG_INTEL_ATOM_SOC_DUMP)	+= intel_soc_dump.o
# MFLD specific files
obj-$(CONFIG_X86_MDFLD) += mfld.o
# PMU driver
obj-$(CONFIG_ATOM_SOC_POWER) += intel_soc_pm_debug.o intel_soc_pmu.o
obj-$(CONFIG_INTEL_ATOM_MDFLD_POWER) += intel_soc_mdfld.o intel_soc_mdfld_clv_common.o
obj-$(CONFIG_INTEL_ATOM_CLV_POWER) += intel_soc_clv.o intel_soc_mdfld_clv_common.o
obj-$(CONFIG_INTEL_ATOM_MRFLD_POWER)	+= intel_soc_mrfld.o
# BOARD files
obj-$(CONFIG_X86_INTEL_MID) += board.o
