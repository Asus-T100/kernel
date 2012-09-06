CFLAGS_intel_soc_pm_debug.o		:= -Werror
CFLAGS_intel_soc_pmu.o			:= -Werror
CFLAGS_intel_soc_mdfld.o		:= -Werror
CFLAGS_intel_soc_clv.o			:= -Werror

# platform configuration for board devices
obj-y					+= device_libs/

# INTEL-MID specific files
obj-$(CONFIG_X86_INTEL_MID)		+= intel_mid_vrtc.o
obj-$(CONFIG_EARLY_PRINTK_INTEL_MID)	+= early_printk_intel_mid.o
obj-$(CONFIG_X86_INTEL_MID)		+= intel-mid.o
obj-$(CONFIG_INTEL_ATOM_SOC_DUMP)	+= intel_soc_dump.o
# MRST specific files
obj-$(CONFIG_X86_MRST)			+= mrst.o
obj-$(CONFIG_X86_MRST)			+= mrst-pmu.o
# MFLD specific files
obj-$(CONFIG_SERIAL_MFD_HSU)		+= mfld-hsu.o

obj-$(CONFIG_X86_MDFLD)			+= mfld.o
obj-$(CONFIG_INTEL_MID_RAM_CONSOLE)     += intel_mid_ram_console.o
# MRFL specific file
obj-$(CONFIG_X86_MRFLD)			+= mrfl.o

obj-$(CONFIG_ATOM_SOC_POWER)		+= intel_soc_pm_debug.o intel_soc_pmu.o
obj-$(CONFIG_INTEL_ATOM_MDFLD_POWER)	+= intel_soc_mdfld.o
obj-$(CONFIG_INTEL_ATOM_CLV_POWER)	+= intel_soc_clv.o

# BOARD files
obj-$(CONFIG_BOARD_MFLD_BLACKBAY)	+= board-blackbay.o
obj-$(CONFIG_BOARD_REDRIDGE)		+= board-redridge.o
obj-$(CONFIG_BOARD_CTP)			+= board-ctp.o
obj-$(CONFIG_BOARD_MRFLD_VP)		+= board-vp.o
obj-$(CONFIG_BOARD_MRFLD_HVP)		+= board-hvp.o
