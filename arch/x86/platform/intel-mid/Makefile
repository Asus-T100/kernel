CFLAGS_intel_soc_pm_debug.o := -Werror
CFLAGS_intel_soc_pmu.o := -Werror
CFLAGS_intel_soc_mdfld.o := -Werror
CFLAGS_intel_soc_clv.o := -Werror

# platform configuration for board devices
obj-y += device_libs/

# INTEL-MID specific files
obj-$(CONFIG_X86_INTEL_MID) += intel_mid_vrtc.o
obj-$(CONFIG_EARLY_PRINTK_INTEL_MID) += early_printk_intel_mid.o
obj-$(CONFIG_X86_INTEL_MID) += intel-mid.o
obj-$(CONFIG_X86_INTEL_MID) += intel_mid_scu.o
# MFLD specific files
obj-$(CONFIG_X86_MDFLD) += mfld.o
# PMU driver
obj-$(CONFIG_ATOM_SOC_POWER) += intel_soc_pm_debug.o intel_soc_pmu.o
obj-$(CONFIG_INTEL_ATOM_MDFLD_POWER) += intel_soc_mdfld.o
obj-$(CONFIG_INTEL_ATOM_CLV_POWER) += intel_soc_clv.o
# BOARD files
obj-$(CONFIG_X86_INTEL_MID) += board.o
