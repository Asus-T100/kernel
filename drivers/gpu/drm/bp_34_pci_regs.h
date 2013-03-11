/*  This file contains definitions from kernel 3.6 include/linux/pci_regs.h */

#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
#error Please remove this file and references to it; backport code no longer needed
#endif


#define PCI_CAP_EXP_ENDPOINT_SIZEOF_V2	44	/* v2 endpoints end here */
#define PCI_EXP_LNKCAP2		44	/* Link Capability 2 */
#define  PCI_EXP_LNKCAP2_SLS_2_5GB 0x01	/* Current Link Speed 2.5GT/s */
#define  PCI_EXP_LNKCAP2_SLS_5_0GB 0x02	/* Current Link Speed 5.0GT/s */
#define  PCI_EXP_LNKCAP2_SLS_8_0GB 0x04	/* Current Link Speed 8.0GT/s */
#define  PCI_EXP_LNKCAP2_CROSSLINK 0x100 /* Crosslink supported */
