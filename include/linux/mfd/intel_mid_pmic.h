#ifndef __CRYSTAL_COVE_H__
#define __CRYSTAL_COVE_H__

#ifdef CONFIG_CRYSTAL_COVE
int intel_mid_pmic_readb(int reg);
int intel_mid_pmic_writeb(int reg, u8 val);
int intel_mid_pmic_setb(int reg, u8 mask);
int intel_mid_pmic_clearb(int reg, u8 mask);
int intel_mid_pmic_set_pdata(const char *name, void *data, int len);
#else
static inline int intel_mid_pmic_readb(int reg) { return 0; }
static inline int intel_mid_pmic_writeb(int reg, u8 val) { return 0; }
static inline int intel_mid_pmic_setb(int reg, u8 mask) { return 0; }
static inline int intel_mid_pmic_clearb(int reg, u8 mask) { return 0; }
static inline int intel_mid_pmic_set_pdata(const char *name,
				void *data, int len) {
	return 0;
}
#endif
#endif

