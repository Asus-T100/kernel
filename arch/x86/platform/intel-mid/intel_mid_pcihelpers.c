#include <linux/export.h>
#include <linux/pci.h>
#include <linux/intel_mid_pm.h>

#include <asm/intel_mid_pcihelpers.h>

/* Unified message bus read/write operation */
static DEFINE_SPINLOCK(msgbus_lock);

static struct pci_dev *pci_root;

static int intel_mid_msgbus_init(void)
{
	pci_root = pci_get_bus_and_slot(0, PCI_DEVFN(0, 0));
	if (!pci_root) {
		pr_err("%s: Error: msgbus PCI handle NULL\n", __func__);
		return -ENODEV;
	}
	return 0;
}
fs_initcall(intel_mid_msgbus_init);

u32 intel_mid_msgbus_read32_raw(u32 cmd)
{
	unsigned long irq_flags;
	u32 data;

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	pci_read_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, &data);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);

	return data;
}
EXPORT_SYMBOL(intel_mid_msgbus_read32_raw);

/*
 * GU: this function is only used by the VISA and 'VXD' drivers.
 */
u32 intel_mid_msgbus_read32_raw_ext(u32 cmd, u32 cmd_ext)
{
	unsigned long irq_flags;
	u32 data;

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_EXT_REG, cmd_ext);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	pci_read_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, &data);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);

	return data;
}
EXPORT_SYMBOL(intel_mid_msgbus_read32_raw_ext);

void intel_mid_msgbus_write32_raw(u32 cmd, u32 data)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, data);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);
}
EXPORT_SYMBOL(intel_mid_msgbus_write32_raw);

/*
 * GU: this function is only used by the VISA and 'VXD' drivers.
 */
void intel_mid_msgbus_write32_raw_ext(u32 cmd, u32 cmd_ext, u32 data)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, data);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_EXT_REG, cmd_ext);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);
}
EXPORT_SYMBOL(intel_mid_msgbus_write32_raw_ext);

u32 intel_mid_msgbus_read32(u8 port, u32 addr)
{
	unsigned long irq_flags;
	u32 data;
	u32 cmd;
	u32 cmdext;

	cmd = (PCI_ROOT_MSGBUS_READ << 24) | (port << 16) |
		((addr & 0xff) << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;
	cmdext = addr & 0xffffff00;

	spin_lock_irqsave(&msgbus_lock, irq_flags);

	if (cmdext) {
		/* This resets to 0 automatically, no need to write 0 */
		pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_EXT_REG,
					cmdext);
	}

	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	pci_read_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, &data);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);

	return data;
}

EXPORT_SYMBOL(intel_mid_msgbus_read32);
void intel_mid_msgbus_write32(u8 port, u32 addr, u32 data)
{
	unsigned long irq_flags;
	u32 cmd;
	u32 cmdext;

	cmd = (PCI_ROOT_MSGBUS_WRITE << 24) | (port << 16) |
		((addr & 0xFF) << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;
	cmdext = addr & 0xffffff00;

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, data);

	if (cmdext) {
		/* This resets to 0 automatically, no need to write 0 */
		pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_EXT_REG,
					cmdext);
	}

	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);
}
EXPORT_SYMBOL(intel_mid_msgbus_write32);

/* called only from where is later then fs_initcall */
u32 intel_mid_soc_stepping(void)
{
	return pci_root->revision;
}
EXPORT_SYMBOL(intel_mid_soc_stepping);

static bool is_south_complex_device(struct pci_dev *dev)
{
	unsigned base_class = dev->class >> 16;
	unsigned sub_class  = (dev->class & SUB_CLASS_MASK) >> 8;

	/* other than camera, pci bridges and display,
	 * everything else are south complex devices.
	 */
	if (((base_class == PCI_BASE_CLASS_MULTIMEDIA) &&
	     (sub_class == ISP_SUB_CLASS)) ||
	    (base_class == PCI_BASE_CLASS_BRIDGE) ||
	    ((base_class == PCI_BASE_CLASS_DISPLAY) && !sub_class))
		return false;
	else
		return true;
}

/* In BYT platform, d3_delay for internal south complex devices,
 * they are not subject to 10 ms d3 to d0 delay required by pci spec.
 */
static void pci_d3_delay_fixup(struct pci_dev *dev)
{
	if (platform_is(INTEL_ATOM_BYT)) {
		/* All internal devices are in bus 0. */
		if (dev->bus->number == 0 && is_south_complex_device(dev)) {
			dev->d3_delay = INTERNAL_PCI_PM_D3_WAIT;
			dev->d3cold_delay = INTERNAL_PCI_PM_D3_WAIT;
		}
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, PCI_ANY_ID, pci_d3_delay_fixup);
