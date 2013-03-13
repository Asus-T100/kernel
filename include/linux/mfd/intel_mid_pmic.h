#ifndef __CRYSTAL_COVE_H__
#define __CRYSTAL_COVE_H__

int intel_mid_pmic_readb(int reg);
int intel_mid_pmic_writeb(int reg, u8 val);
int intel_mid_pmic_setb(int reg, u8 mask);
int intel_mid_pmic_clearb(int reg, u8 mask);

#endif

