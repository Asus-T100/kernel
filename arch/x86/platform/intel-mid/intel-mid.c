/*
 * intel-mid.c: Intel MID platform setup code
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#define	SFI_SIG_OEM0	"OEM0"

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/intel_pmic_gpio.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/intel_mid_pm.h>
#include <linux/hsi/hsi.h>
#include <linux/spinlock.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/blkdev.h>
#include <linux/acpi.h>
#include <linux/intel_mid_acpi.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/mrst-vrtc.h>
#include <asm/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/apb_timer.h>
#include <asm/reboot.h>
#include "intel_mid_weak_decls.h"

#include "intel_mid_scu.h"
#include "intel_mid_sfi.h"

/*
 * the clockevent devices on Moorestown/Medfield can be APBT or LAPIC clock,
 * cmdline option x86_intel_mid_timer can be used to override the configuration
 * to prefer one or the other.
 * at runtime, there are basically three timer configurations:
 * 1. per cpu apbt clock only
 * 2. per cpu always-on lapic clocks only, this is Penwell/Medfield only
 * 3. per cpu lapic clock (C3STOP) and one apbt clock, with broadcast.
 *
 * by default (without cmdline option), platform code first detects cpu type
 * to see if we are on lincroft or penwell, then set up both lapic or apbt
 * clocks accordingly.
 * i.e. by default, medfield uses configuration #2, moorestown uses #1.
 * config #3 is supported but not recommended on medfield.
 *
 * rating and feature summary:
 * lapic (with C3STOP) --------- 100
 * apbt (always-on) ------------ 110
 * lapic (always-on,ARAT) ------ 150
 */

__cpuinitdata enum intel_mid_timer_options intel_mid_timer_options;

struct kobject *spid_kobj;
struct soft_platform_id spid;
#ifdef CONFIG_ACPI
struct kobject *pidv_kobj;
struct platform_id pidv;
#endif
char intel_mid_ssn[INTEL_MID_SSN_SIZE + 1];
/* intel_mid_ops to store sub arch ops */
struct intel_mid_ops *intel_mid_ops;
/* getter function for sub arch ops*/
static void *(*get_intel_mid_ops[])(void) = INTEL_MID_OPS_INIT;
static u32 sfi_mtimer_usage[SFI_MTMR_MAX_NUM];
static struct sfi_timer_table_entry sfi_mtimer_array[SFI_MTMR_MAX_NUM];
enum intel_mid_cpu_type __intel_mid_cpu_chip;
EXPORT_SYMBOL_GPL(__intel_mid_cpu_chip);

int sfi_mtimer_num;

struct sfi_rtc_table_entry sfi_mrtc_array[SFI_MRTC_MAX];
EXPORT_SYMBOL_GPL(sfi_mrtc_array);
int sfi_mrtc_num;
u32 nbr_hsi_clients = 2;

void intel_mid_power_off(void)
{
	pmu_power_off();
};

/* Unified message bus read/write operation */
static DEFINE_SPINLOCK(msgbus_lock);

static struct pci_dev *pci_root;

static int intel_mid_msgbus_init(void)
{
	pci_root = pci_get_bus_and_slot(0, PCI_DEVFN(0, 0));
	if (!pci_root) {
		printk(KERN_ALERT "%s: Error: msgbus PCI handle NULL",
			__func__);
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

void intel_mid_msgbus_write32_raw(u32 cmd, u32 data)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, data);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);
}
EXPORT_SYMBOL(intel_mid_msgbus_write32_raw);

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

/* parse all the mtimer info to a static mtimer array */
static int __init sfi_parse_mtmr(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_timer_table_entry *pentry;
	struct mpc_intsrc mp_irq;
	int totallen;

	sb = (struct sfi_table_simple *)table;
	if (!sfi_mtimer_num) {
		sfi_mtimer_num = SFI_GET_NUM_ENTRIES(sb,
					struct sfi_timer_table_entry);
		pentry = (struct sfi_timer_table_entry *) sb->pentry;
		totallen = sfi_mtimer_num * sizeof(*pentry);
		memcpy(sfi_mtimer_array, pentry, totallen);
	}

	pr_debug("SFI MTIMER info (num = %d):\n", sfi_mtimer_num);
	pentry = sfi_mtimer_array;
	for (totallen = 0; totallen < sfi_mtimer_num; totallen++, pentry++) {
		pr_debug("timer[%d]: paddr = 0x%08x, freq = %dHz, irq = %d\n",
			totallen, (u32)pentry->phys_addr,
			pentry->freq_hz, pentry->irq);
			if (!pentry->irq)
				continue;
			mp_irq.type = MP_INTSRC;
			mp_irq.irqtype = mp_INT;
/* triggering mode edge bit 2-3, active high polarity bit 0-1 */
			mp_irq.irqflag = 5;
			mp_irq.srcbus = MP_BUS_ISA;
			mp_irq.srcbusirq = pentry->irq;	/* IRQ */
			mp_irq.dstapic = MP_APIC_ALL;
			mp_irq.dstirq = pentry->irq;
			mp_save_irq(&mp_irq);
	}

	return 0;
}

struct sfi_timer_table_entry *sfi_get_mtmr(int hint)
{
	int i;
	if (hint < sfi_mtimer_num) {
		if (!sfi_mtimer_usage[hint]) {
			pr_debug("hint taken for timer %d irq %d\n",\
				hint, sfi_mtimer_array[hint].irq);
			sfi_mtimer_usage[hint] = 1;
			return &sfi_mtimer_array[hint];
		}
	}
	/* take the first timer available */
	for (i = 0; i < sfi_mtimer_num;) {
		if (!sfi_mtimer_usage[i]) {
			sfi_mtimer_usage[i] = 1;
			return &sfi_mtimer_array[i];
		}
		i++;
	}
	return NULL;
}

void sfi_free_mtmr(struct sfi_timer_table_entry *mtmr)
{
	int i;
	for (i = 0; i < sfi_mtimer_num;) {
		if (mtmr->irq == sfi_mtimer_array[i].irq) {
			sfi_mtimer_usage[i] = 0;
			return;
		}
		i++;
	}
}

/* parse all the mrtc info to a global mrtc array */
int __init sfi_parse_mrtc(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_rtc_table_entry *pentry;
	struct mpc_intsrc mp_irq;

	int totallen;

	sb = (struct sfi_table_simple *)table;
	if (!sfi_mrtc_num) {
		sfi_mrtc_num = SFI_GET_NUM_ENTRIES(sb,
						struct sfi_rtc_table_entry);
		pentry = (struct sfi_rtc_table_entry *)sb->pentry;
		totallen = sfi_mrtc_num * sizeof(*pentry);
		memcpy(sfi_mrtc_array, pentry, totallen);
	}

	pr_debug("SFI RTC info (num = %d):\n", sfi_mrtc_num);
	pentry = sfi_mrtc_array;
	for (totallen = 0; totallen < sfi_mrtc_num; totallen++, pentry++) {
		pr_debug("RTC[%d]: paddr = 0x%08x, irq = %d\n",
			totallen, (u32)pentry->phys_addr, pentry->irq);
		mp_irq.type = MP_INTSRC;
		mp_irq.irqtype = mp_INT;
		mp_irq.irqflag = 0xf;	/* level trigger and active low */
		mp_irq.srcbus = MP_BUS_ISA;
		mp_irq.srcbusirq = pentry->irq;	/* IRQ */
		mp_irq.dstapic = MP_APIC_ALL;
		mp_irq.dstirq = pentry->irq;
		mp_save_irq(&mp_irq);
	}
	return 0;
}

unsigned long __init intel_mid_calibrate_tsc(void)
{
	return 0;
}

static void __init intel_mid_time_init(void)
{
	sfi_table_parse(SFI_SIG_MTMR, NULL, NULL, sfi_parse_mtmr);
	switch (intel_mid_timer_options) {
	case INTEL_MID_TIMER_APBT_ONLY:
		break;
	case INTEL_MID_TIMER_LAPIC_APBT:
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		break;
	default:
		if (!boot_cpu_has(X86_FEATURE_ARAT))
			break;
		x86_init.timers.setup_percpu_clockev = setup_boot_APIC_clock;
		x86_cpuinit.setup_percpu_clockev = setup_secondary_APIC_clock;
		return;
	}
	/* we need at least one APB timer */
	pre_init_apic_IRQ0();
	apbt_time_init();
}

static void __cpuinit intel_mid_arch_setup(void)
{
	if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x27)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_PENWELL;
	else if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x26)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_LINCROFT;
	else if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x35)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_CLOVERVIEW;
	else if (boot_cpu_data.x86 == 6 && (boot_cpu_data.x86_model == 0x3C ||
					    boot_cpu_data.x86_model == 0x4A))
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_TANGIER;
	else if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x37)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_VALLEYVIEW2;
	else if (boot_cpu_data.x86 == 6 && boot_cpu_data.x86_model == 0x5A)
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_ANNIEDALE;
	else {
		pr_err("Unknown Moorestown CPU (%d:%d), default to Lincroft\n",
			boot_cpu_data.x86, boot_cpu_data.x86_model);
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_LINCROFT;
	}

	if (__intel_mid_cpu_chip < MAX_CPU_OPS(get_intel_mid_ops))
		intel_mid_ops = get_intel_mid_ops[__intel_mid_cpu_chip]();
	else {
		intel_mid_ops = get_intel_mid_ops[INTEL_MID_CPU_CHIP_PENWELL]();
		pr_info("ARCH: Uknown SoC, assuming PENWELL!\n");
	}

	if (intel_mid_ops->arch_setup)
		intel_mid_ops->arch_setup();
}

/* MID systems don't have i8042 controller */
int intel_mid_i8042_detect(void)
{
	return 0;
}

static int force_cold_boot;
module_param(force_cold_boot, int, 0644);
MODULE_PARM_DESC(force_cold_boot,
		 "Set to Y to force a COLD BOOT instead of a COLD RESET "
		 "on the next reboot system call.");

static void intel_mid_reboot(char *cmd)
{
	if (intel_scu_ipc_fw_update()) {
		pr_debug("intel_scu_fw_update: IFWI upgrade failed...\n");
		BUG();
	}
	if (force_cold_boot) {
		pr_info("Immediate COLD BOOT\n");
		rpmsg_send_generic_simple_command(IPCMSG_COLD_BOOT, 0);
	} else {
		pr_info("Immediate COLD RESET\n");
		rpmsg_send_generic_simple_command(IPCMSG_COLD_RESET, 0);
	}
}

static void intel_mid_emergency_reboot(void)
{
	/* Change system state to poll IPC status until IPC not busy*/
	system_state = SYSTEM_RESTART;

	while (intel_scu_ipc_check_status())
		udelay(10);

	if (force_cold_boot)
		rpmsg_send_generic_raw_command(IPCMSG_COLD_BOOT,
			0, NULL, 0, NULL, 0, 0, 0);
	else
		rpmsg_send_generic_raw_command(IPCMSG_COLD_RESET,
			0, NULL, 0, NULL, 0, 0, 0);
}

/*
 * Moorestown specific x86_init function overrides and early setup
 * calls.
 */
void __init x86_intel_mid_early_setup(void)
{
	x86_init.resources.probe_roms = x86_init_noop;
	x86_init.resources.reserve_resources = x86_init_noop;

	x86_init.timers.timer_init = intel_mid_time_init;
	x86_init.timers.setup_percpu_clockev = x86_init_noop;

	x86_init.irqs.pre_vector_init = x86_init_noop;

	x86_init.oem.arch_setup = intel_mid_arch_setup;

	x86_cpuinit.setup_percpu_clockev = apbt_setup_secondary_clock;

	x86_platform.calibrate_tsc = intel_mid_calibrate_tsc;
	x86_platform.i8042_detect = intel_mid_i8042_detect;
	x86_init.timers.wallclock_init = intel_mid_rtc_init;
	x86_init.pci.init = intel_mid_pci_init;
	x86_init.pci.fixup_irqs = x86_init_noop;

	legacy_pic = &null_legacy_pic;

	pm_power_off = intel_mid_power_off;
#ifndef CONFIG_INTEL_MID_OSNIB_ILB
	machine_ops.restart = intel_mid_reboot;
	machine_ops.emergency_restart  = intel_mid_emergency_reboot;
#endif
	/* Avoid searching for BIOS MP tables */
	x86_init.mpparse.find_smp_config = x86_init_noop;
	x86_init.mpparse.get_smp_config = x86_init_uint_noop;
	set_bit(MP_BUS_ISA, mp_bus_not_pci);
}

/*
 * if user does not want to use per CPU apb timer, just give it a lower rating
 * than local apic timer and skip the late per cpu timer init.
 */
static inline int __init setup_x86_intel_mid_timer(char *arg)
{
	if (!arg)
		return -EINVAL;

	if (strcmp("apbt_only", arg) == 0)
		intel_mid_timer_options = INTEL_MID_TIMER_APBT_ONLY;
	else if (strcmp("lapic_and_apbt", arg) == 0)
		intel_mid_timer_options = INTEL_MID_TIMER_LAPIC_APBT;
	else {
		pr_warning("X86 INTEL_MID timer option %s not recognised"
			   " use x86_intel_mid_timer=apbt_only or lapic_and_apbt\n",
			   arg);
		return -EINVAL;
	}
	return 0;
}
__setup("x86_intel_mid_timer=", setup_x86_intel_mid_timer);

/*
 * Parsing OEM0 table.
 */
static struct sfi_table_header *oem0_table;

static int __init sfi_parse_oem0(struct sfi_table_header *table)
{
	oem0_table = table;
	return 0;
}

void *get_oem0_table(void)
{
	return oem0_table;
}

/*
 * Parsing GPIO table first, since the DEVS table will need this table
 * to map the pin name to the actual pin.
 */
static struct sfi_gpio_table_entry *gpio_table;
static int gpio_num_entry;

static int __init sfi_parse_gpio(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_gpio_table_entry *pentry;
	int num, i;

	if (gpio_table)
		return 0;
	sb = (struct sfi_table_simple *)table;
	num = SFI_GET_NUM_ENTRIES(sb, struct sfi_gpio_table_entry);
	pentry = (struct sfi_gpio_table_entry *)sb->pentry;

	gpio_table = (struct sfi_gpio_table_entry *)
				kmalloc(num * sizeof(*pentry), GFP_KERNEL);
	if (!gpio_table)
		return -1;
	memcpy(gpio_table, pentry, num * sizeof(*pentry));
	gpio_num_entry = num;

	pr_info("GPIO pin info:\n");
	for (i = 0; i < num; i++, pentry++)
		pr_info("info[%2d]: controller = %16.16s, pin_name = %16.16s,"
		" pin = %d\n", i,
			pentry->controller_name,
			pentry->pin_name,
			pentry->pin_no);
	return 0;
}

int get_gpio_by_name(const char *name)
{
	struct sfi_gpio_table_entry *pentry = gpio_table;
	int i;

	if (!pentry)
		return -1;
	for (i = 0; i < gpio_num_entry; i++, pentry++) {
		if (!strncmp(name, pentry->pin_name, SFI_NAME_LEN))
			return pentry->pin_no;
	}
	return -1;
}

#define MAX_IPCDEVS	24
static struct platform_device *ipc_devs[MAX_IPCDEVS];
static int ipc_next_dev;

#define MAX_SCU_SPI	24
static struct spi_board_info *spi_devs[MAX_SCU_SPI];
static int spi_next_dev;

#define MAX_SCU_I2C	24
static struct i2c_board_info *i2c_devs[MAX_SCU_I2C];
static int i2c_bus[MAX_SCU_I2C];
static int i2c_next_dev;

void __init intel_scu_device_register(struct platform_device *pdev)
{
	if (ipc_next_dev == MAX_IPCDEVS)
		pr_err("too many SCU IPC devices");
	else
		ipc_devs[ipc_next_dev++] = pdev;
}

static void __init intel_scu_spi_device_register(struct spi_board_info *sdev)
{
	struct spi_board_info *new_dev;

	if (spi_next_dev == MAX_SCU_SPI) {
		pr_err("too many SCU SPI devices");
		return;
	}

	new_dev = kzalloc(sizeof(*sdev), GFP_KERNEL);
	if (!new_dev) {
		pr_err("failed to alloc mem for delayed spi dev %s\n",
			sdev->modalias);
		return;
	}
	memcpy(new_dev, sdev, sizeof(*sdev));

	spi_devs[spi_next_dev++] = new_dev;
}

static void __init intel_scu_i2c_device_register(int bus,
						struct i2c_board_info *idev)
{
	struct i2c_board_info *new_dev;

	if (i2c_next_dev == MAX_SCU_I2C) {
		pr_err("too many SCU I2C devices");
		return;
	}

	new_dev = kzalloc(sizeof(*idev), GFP_KERNEL);
	if (!new_dev) {
		pr_err("failed to alloc mem for delayed i2c dev %s\n",
			idev->type);
		return;
	}
	memcpy(new_dev, idev, sizeof(*idev));

	i2c_bus[i2c_next_dev] = bus;
	i2c_devs[i2c_next_dev++] = new_dev;
}

struct blocking_notifier_head intel_scu_notifier =
			BLOCKING_NOTIFIER_INIT(intel_scu_notifier);
EXPORT_SYMBOL_GPL(intel_scu_notifier);

/* Called by IPC driver */
void intel_scu_devices_create(void)
{
	int i;

	for (i = 0; i < ipc_next_dev; i++)
		platform_device_add(ipc_devs[i]);

	for (i = 0; i < spi_next_dev; i++)
		spi_register_board_info(spi_devs[i], 1);

	for (i = 0; i < i2c_next_dev; i++) {
		struct i2c_adapter *adapter;
		struct i2c_client *client;

		adapter = i2c_get_adapter(i2c_bus[i]);
		if (adapter) {
			client = i2c_new_device(adapter, i2c_devs[i]);
			if (!client)
				pr_err("can't create i2c device %s\n",
					i2c_devs[i]->type);
		} else
			i2c_register_board_info(i2c_bus[i], i2c_devs[i], 1);
	}
	intel_scu_notifier_post(SCU_AVAILABLE, 0L);
}
EXPORT_SYMBOL_GPL(intel_scu_devices_create);

/* Called by IPC driver */
void intel_scu_devices_destroy(void)
{
	int i;

	intel_scu_notifier_post(SCU_DOWN, 0L);

	for (i = 0; i < ipc_next_dev; i++)
		platform_device_del(ipc_devs[i]);
}
EXPORT_SYMBOL_GPL(intel_scu_devices_destroy);


static struct platform_device *psh_ipc;
void intel_psh_devices_create(void)
{
	psh_ipc = platform_device_alloc("intel_psh_ipc", 0);
	if (psh_ipc == NULL) {
		pr_err("out of memory for platform device psh_ipc.\n");
		return;
	}

	platform_device_add(psh_ipc);
}
EXPORT_SYMBOL_GPL(intel_psh_devices_create);

void intel_psh_devices_destroy(void)
{
	if (psh_ipc)
		platform_device_del(psh_ipc);
}
EXPORT_SYMBOL_GPL(intel_psh_devices_destroy);

void __init install_irq_resource(struct platform_device *pdev, int irq)
{
	/* Single threaded */
	static struct resource __initdata res = {
		.name = "IRQ",
		.flags = IORESOURCE_IRQ,
	};
	res.start = irq;
	platform_device_add_resources(pdev, &res, 1);
}

static void __init sfi_handle_ipc_dev(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct platform_device *pdev;
	void *pdata = NULL;
	pr_info("IPC bus, name = %16.16s, irq = 0x%2x\n",
		pentry->name, pentry->irq);
	pdata = dev->get_platform_data(pentry);
	pdev = platform_device_alloc(pentry->name, 0);
	if (pdev == NULL) {
		pr_err("out of memory for SFI platform device '%s'.\n",
			pentry->name);
		return;
	}
	install_irq_resource(pdev, pentry->irq);

	pdev->dev.platform_data = pdata;
	intel_scu_device_register(pdev);
}

#ifdef CONFIG_INTEL_PSH_IPC
static int __init intel_psh_ipc_subdev_init(void)
{
	struct platform_device *psh;
	psh = platform_device_alloc("psh", 0);
	if (psh == NULL) {
		pr_err("out of memory for platform device psh.\n");
		return -1;
	}

	platform_device_add(psh);

	return 0;
}
device_initcall(intel_psh_ipc_subdev_init);
#endif

static void __init sfi_handle_spi_dev(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct spi_board_info spi_info;
	void *pdata = NULL;

	memset(&spi_info, 0, sizeof(spi_info));
	strncpy(spi_info.modalias, pentry->name, SFI_NAME_LEN);
	spi_info.irq = ((pentry->irq == (u8)0xff) ? 0 : pentry->irq);
	spi_info.bus_num = pentry->host_num;
	spi_info.chip_select = pentry->addr;
	spi_info.max_speed_hz = pentry->max_freq;
	pr_info("SPI bus=%d, name=%16.16s, irq=0x%2x, max_freq=%d, cs=%d\n",
		spi_info.bus_num,
		spi_info.modalias,
		spi_info.irq,
		spi_info.max_speed_hz,
		spi_info.chip_select);

	pdata = dev->get_platform_data(&spi_info);

	spi_info.platform_data = pdata;
	if (dev->delay)
		intel_scu_spi_device_register(&spi_info);
	else
		spi_register_board_info(&spi_info, 1);
}

static void __init sfi_handle_i2c_dev(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct i2c_board_info i2c_info;
	void *pdata = NULL;

	memset(&i2c_info, 0, sizeof(i2c_info));
	strncpy(i2c_info.type, pentry->name, SFI_NAME_LEN);
	i2c_info.irq = ((pentry->irq == (u8)0xff) ? 0 : pentry->irq);
	i2c_info.addr = pentry->addr;
	pr_info("I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
		pentry->host_num,
		i2c_info.type,
		i2c_info.irq,
		i2c_info.addr);
	pdata = dev->get_platform_data(&i2c_info);
	i2c_info.platform_data = pdata;

	if (dev->delay)
		intel_scu_i2c_device_register(pentry->host_num, &i2c_info);
	else
		i2c_register_board_info(pentry->host_num, &i2c_info, 1);
}

static void sfi_handle_hsi_dev(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct hsi_board_info hsi_info;
	void *pdata = NULL;

	pr_info("HSI bus = %d, name = %16.16s, port = %d\n",
		pentry->host_num,
		pentry->name,
		pentry->addr);

	memset(&hsi_info, 0, sizeof(hsi_info));
	hsi_info.name = pentry->name;
	hsi_info.hsi_id = pentry->host_num;
	hsi_info.port = pentry->addr;

	pdata = dev->get_platform_data(&hsi_info);
	if (pdata) {
		pr_info("SFI register platform data for HSI device %s with %d number of clients\n",
					dev->name,
					nbr_hsi_clients);
		hsi_register_board_info(pdata, nbr_hsi_clients);
	}
}

static void __init sfi_handle_sd_dev(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct sd_board_info sd_info;
	void *pdata = NULL;

	memset(&sd_info, 0, sizeof(sd_info));
	strncpy(sd_info.name, pentry->name, 16);
	sd_info.bus_num = pentry->host_num;
	sd_info.board_ref_clock = pentry->max_freq;
	sd_info.addr = pentry->addr;
	pr_info("SDIO bus = %d, name = %16.16s, "
			"ref_clock = %d, addr =0x%x\n",
			sd_info.bus_num,
			sd_info.name,
			sd_info.board_ref_clock,
			sd_info.addr);
	pdata = dev->get_platform_data(&sd_info);
	sd_info.platform_data = pdata;
}

struct devs_id __init *get_device_id(u8 type, char *name)
{
	struct devs_id *dev = (get_device_ptr ? get_device_ptr() : NULL);

	if (dev == NULL)
		return NULL;

	while (dev->name[0]) {
		if (dev->type == type &&
			!strncmp(dev->name, name, SFI_NAME_LEN)) {
			return dev;
		}
		dev++;
	}

	return NULL;
}

static struct sfi_device_table_entry imx135_entry = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x10,
	.irq = 0xFF,
	.max_freq = 400000,
	.name = "imx135",
};

static struct sfi_device_table_entry s5k8aay_entry = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x3c,
	.irq = 0xFF,
	.max_freq = 400000,
	.name = "s5k8aay",
};

static struct sfi_device_table_entry dw9719_entry = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x0c,
	.irq = 0xFF,
	.max_freq = 400000,
	.name = "dw9719",
};

static struct sfi_device_table_entry lm3559_entry = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x53,
	.irq = 0xFF,
	.max_freq = 400000,
	.name = "lm3559",
};

static int __init sfi_parse_devs(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct sfi_device_table_entry *pentry;
	struct devs_id *dev = NULL;
	int num, i;
	int ioapic;
	struct io_apic_irq_attr irq_attr;

	sb = (struct sfi_table_simple *)table;
	num = SFI_GET_NUM_ENTRIES(sb, struct sfi_device_table_entry);
	pentry = (struct sfi_device_table_entry *)sb->pentry;

	for (i = 0; i < num; i++, pentry++) {
		int irq = pentry->irq;

		if (irq != (u8)0xff) { /* native RTE case */
			/* these SPI2 devices are not exposed to system as PCI
			 * devices, but they have separate RTE entry in IOAPIC
			 * so we have to enable them one by one here
			 */
			ioapic = mp_find_ioapic(irq);
			if (ioapic >= 0) {
				irq_attr.ioapic = ioapic;
				irq_attr.ioapic_pin = irq;
				irq_attr.trigger = 1;
				if (intel_mid_identify_cpu() ==
						INTEL_MID_CPU_CHIP_TANGIER) {
					if (!strncmp(pentry->name,
						"r69001-ts-i2c", 13))
						/* active low */
						irq_attr.polarity = 1;
					else if (!strncmp(pentry->name,
						"synaptics_3202", 14))
						/* active low */
						irq_attr.polarity = 1;
					else if (irq == 41)
						/* fast_int_1 */
						irq_attr.polarity = 1;
					else
						/* active high */
						irq_attr.polarity = 0;
				} else {
					/* PNW and CLV go with active low */
					irq_attr.polarity = 1;
				}
				io_apic_set_pci_routing(NULL, irq, &irq_attr);
			} else
				printk(KERN_INFO "APIC entry not found for: name=%s, irq=%d, ioapic=%d",
					pentry->name, irq, ioapic);
		}

		dev = get_device_id(pentry->type, pentry->name);
		if ((dev == NULL) || (dev->get_platform_data == NULL))
			continue;

		if (dev->device_handler) {
			dev->device_handler(pentry, dev);
		} else {
			switch (pentry->type) {
			case SFI_DEV_TYPE_IPC:
				sfi_handle_ipc_dev(pentry, dev);
				break;
			case SFI_DEV_TYPE_SPI:
				sfi_handle_spi_dev(pentry, dev);
				break;
			case SFI_DEV_TYPE_I2C:
				sfi_handle_i2c_dev(pentry, dev);
				break;
			case SFI_DEV_TYPE_SD:
				sfi_handle_sd_dev(pentry, dev);
				break;
			case SFI_DEV_TYPE_HSI:
				sfi_handle_hsi_dev(pentry, dev);
				break;
			default:
				break;
			}
		}
	}

	if ((INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, PRO) ||
	     INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, ENG) ||
	     INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO) ||
	     INTEL_MID_BOARD(2, PHONE, CLVTP, VB, ENG)) &&
	     (SPID_HARDWARE_ID(CLVTP, PHONE, VB, PR1A) ||
	      SPID_HARDWARE_ID(CLVTP, PHONE, VB, PR1B))) {
		pr_info("Simulating VB SFI table\n");
		dev = get_device_id(SFI_DEV_TYPE_I2C, "imx135");
		if (dev && dev->device_handler)
			dev->device_handler(&imx135_entry, dev);
		dev = get_device_id(SFI_DEV_TYPE_I2C, "s5k8aay");
		if (dev && dev->device_handler)
			dev->device_handler(&s5k8aay_entry, dev);
		dev = get_device_id(SFI_DEV_TYPE_I2C, "dw9719");
		if (dev && dev->device_handler)
			dev->device_handler(&dw9719_entry, dev);
		dev = get_device_id(SFI_DEV_TYPE_I2C, "lm3559");
		if (dev && dev->device_handler)
			dev->device_handler(&lm3559_entry, dev);
	}

	return 0;
}

static int __init sfi_parse_oemb(struct sfi_table_header *table)
{
	struct sfi_table_oemb *oemb;
	u32 board_id;
	u8 sig[SFI_SIGNATURE_SIZE + 1] = {'\0'};
	u8 oem_id[SFI_OEM_ID_SIZE + 1] = {'\0'};
	u8 oem_table_id[SFI_OEM_TABLE_ID_SIZE + 1] = {'\0'};

	oemb = (struct sfi_table_oemb *) table;
	if (!oemb) {
		pr_err("%s: fail to read MFD Validation SFI OEMB Layout\n",
			__func__);
		return -ENODEV;
	}

	board_id = oemb->board_id | (oemb->board_fab << 4);

	memcpy(&spid, &oemb->spid, sizeof(struct soft_platform_id));

	if (oemb->header.len <
			(char *)oemb->ssn + INTEL_MID_SSN_SIZE - (char *)oemb) {
		pr_err("SFI OEMB does not contains SSN\n");
		intel_mid_ssn[0] = '\0';
	} else {
		memcpy(intel_mid_ssn, oemb->ssn, INTEL_MID_SSN_SIZE);
		intel_mid_ssn[INTEL_MID_SSN_SIZE] = '\0';
	}

	snprintf(sig, (SFI_SIGNATURE_SIZE + 1), "%s",
		oemb->header.sig);
	snprintf(oem_id, (SFI_OEM_ID_SIZE + 1), "%s",
		oemb->header.oem_id);
	snprintf(oem_table_id, (SFI_OEM_TABLE_ID_SIZE + 1), "%s",
		oemb->header.oem_table_id);
	pr_info("MFLD Validation SFI OEMB Layout\n");
	pr_info("\tOEMB signature               : %s\n"
		"\tOEMB length                  : %d\n"
		"\tOEMB revision                : %d\n"
		"\tOEMB checksum                : 0x%X\n"
		"\tOEMB oem_id                  : %s\n"
		"\tOEMB oem_table_id            : %s\n"
		"\tOEMB board_id                : 0x%02X\n"
		"\tOEMB iafw version            : %03d.%03d\n"
		"\tOEMB val_hooks version       : %03d.%03d\n"
		"\tOEMB ia suppfw version       : %03d.%03d\n"
		"\tOEMB scu runtime version     : %03d.%03d\n"
		"\tOEMB ifwi version            : %03d.%03d\n"
		"\tOEMB spid customer id        : %04x\n"
		"\tOEMB spid vendor id          : %04x\n"
		"\tOEMB spid manufacturer id    : %04x\n"
		"\tOEMB spid platform family id : %04x\n"
		"\tOEMB spid product line id    : %04x\n"
		"\tOEMB spid hardware id        : %04x\n"
		"\tOEMB spid fru[4..0]          : %02x %02x %02x %02x %02x\n"
		"\tOEMB spid fru[9..5]          : %02x %02x %02x %02x %02x\n"
		"\tOEMB ssn                     : %s\n",
		sig,
		oemb->header.len,
		oemb->header.rev,
		oemb->header.csum,
		oem_id,
		oem_table_id,
		board_id,
		oemb->iafw_major_version,
		oemb->iafw_main_version,
		oemb->val_hooks_major_version,
		oemb->val_hooks_minor_version,
		oemb->ia_suppfw_major_version,
		oemb->ia_suppfw_minor_version,
		oemb->scu_runtime_major_version,
		oemb->scu_runtime_minor_version,
		oemb->ifwi_major_version,
		oemb->ifwi_minor_version,
		spid.customer_id,
		spid.vendor_id,
		spid.manufacturer_id,
		spid.platform_family_id,
		spid.product_line_id,
		spid.hardware_id,
		spid.fru[4], spid.fru[3], spid.fru[2], spid.fru[1],
		spid.fru[0], spid.fru[9], spid.fru[8], spid.fru[7],
		spid.fru[6], spid.fru[5],
		intel_mid_ssn);
	return 0;
}

static ssize_t customer_id_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%04x\n", spid.customer_id);
}
spid_attr(customer_id);

static ssize_t vendor_id_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%04x\n", spid.vendor_id);
}
spid_attr(vendor_id);

static ssize_t manufacturer_id_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%04x\n", spid.manufacturer_id);
}
spid_attr(manufacturer_id);

static ssize_t platform_family_id_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%04x\n", spid.platform_family_id);
}
spid_attr(platform_family_id);

static ssize_t product_line_id_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%04x\n", spid.product_line_id);
}
spid_attr(product_line_id);

static ssize_t hardware_id_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%04x\n", spid.hardware_id);
}
spid_attr(hardware_id);

static ssize_t fru_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%02x\n%02x\n%02x\n%02x\n%02x\n"
			    "%02x\n%02x\n%02x\n%02x\n%02x\n",
		       spid.fru[0], spid.fru[1], spid.fru[2], spid.fru[3],
		       spid.fru[4], spid.fru[5], spid.fru[6], spid.fru[7],
		       spid.fru[8], spid.fru[9]);
}
spid_attr(fru);

static struct attribute *spid_attrs[] = {
	&customer_id_attr.attr,
	&vendor_id_attr.attr,
	&manufacturer_id_attr.attr,
	&platform_family_id_attr.attr,
	&product_line_id_attr.attr,
	&hardware_id_attr.attr,
	&fru_attr.attr,
	NULL,
};

static struct attribute_group spid_attr_group = {
	.attrs = spid_attrs,
};

/* size of SPID cmdline : androidboot.spid=vend:cust:manu:plat:prod:hard */
#define SPID_CMDLINE_SIZE 46
#define SPID_PARAM_NAME "androidboot.spid="
#define SPID_DEFAULT_VALUE "xxxx:xxxx:xxxx:xxxx:xxxx:xxxx"

void populate_spid_cmdline(void)
{
	char *spid_param, *spid_default_value;
	char spid_cmdline[SPID_CMDLINE_SIZE+1];

	/* parameter format : cust:vend:manu:plat:prod:hard */
	snprintf(spid_cmdline, sizeof(spid_cmdline),
			"%04x:%04x:%04x:%04x:%04x:%04x",
			spid.vendor_id,
			spid.customer_id,
			spid.manufacturer_id,
			spid.platform_family_id,
			spid.product_line_id,
			spid.hardware_id
			);

	/* is there a spid param ? */
	spid_param = strstr(saved_command_line, SPID_PARAM_NAME);
	if (spid_param) {
		/* is the param set to default value ? */
		spid_default_value = strstr(saved_command_line,
						SPID_DEFAULT_VALUE);
		if (spid_default_value) {
			spid_param += strlen(SPID_PARAM_NAME);
			if (strlen(spid_param) > strlen(spid_cmdline))
				memcpy(spid_param, spid_cmdline,
							strlen(spid_cmdline));
			else
				pr_err("Not enough free space for SPID in command line.\n");
		} else
			pr_warning("SPID already populated. Do not overwrite.\n");
	} else
		pr_err("SPID not found in kernel command line.\n");
}

#ifdef CONFIG_ACPI
static int __init acpi_parse_pidv(struct acpi_table_header *table)
{
	struct acpi_table_pidv *pidv_tbl;

	pidv_tbl = (struct acpi_table_pidv *)table;
	if (!pidv_tbl) {
		printk(KERN_WARNING "Unable to map PIDV\n");
		return -ENODEV;
	}

	memcpy(&pidv, &(pidv_tbl->pidv), sizeof(struct platform_id));
	/*
	 * FIXME: add spid accessor, instead of memcpy
	 */
	memcpy(&spid, &(pidv_tbl->pidv.ext_id_1),
			sizeof(struct soft_platform_id));
	/*
	 * FIXME: add ssn accessor, instead of memcpy
	 */
	memcpy(&intel_mid_ssn, &(pidv_tbl->pidv.part_number),
			INTEL_MID_SSN_SIZE);
	intel_mid_ssn[INTEL_MID_SSN_SIZE] = '\0';

	return 0;
}

static ssize_t iafw_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%02X.%02X\n", pidv.iafw_major, pidv.iafw_minor);
}
pidv_attr(iafw_version);

static ssize_t secfw_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%02X.%02X\n", pidv.secfw_major, pidv.secfw_minor);
}
pidv_attr(secfw_version);

static struct attribute *pidv_attrs[] = {
	&iafw_version_attr.attr,
	&secfw_version_attr.attr,
	NULL,
};

static struct attribute_group pidv_attr_group = {
	.attrs = pidv_attrs,
};

#else
#define acpi_parse_pidv NULL
#endif

static int __init intel_mid_platform_init(void)
{
	int ret = 0;

	/* create sysfs entries for soft platform id */
	spid_kobj = kobject_create_and_add("spid", NULL);
	if (!spid_kobj) {
		pr_err("SPID: ENOMEM for spid_kobj\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(spid_kobj, &spid_attr_group);
	if (ret) {
		pr_err("SPID: failed to create /sys/spid\n");
		return ret;
	}

	/*
	 * FIXME: add SFI compile flag check
	 * FIXME: add SCU compile flag check
	 */
	switch (intel_mid_identify_cpu()) {
	case INTEL_MID_CPU_CHIP_VALLEYVIEW2:
#ifdef CONFIG_ACPI
		acpi_table_parse(ACPI_SIG_PIDV, acpi_parse_pidv);
		pidv_kobj = kobject_create_and_add("pidv", firmware_kobj);
		if (!pidv_kobj) {
			pr_err("pidv: ENOMEM for pidv_kobj\n");
			return -ENOMEM;
		}

		ret = sysfs_create_group(pidv_kobj, &pidv_attr_group);
		if (ret) {
			pr_err("SPID: failed to create /sys/spid\n");
			return ret;
		}
#endif
		break;
	default:
		/* Get MFD Validation SFI OEMB Layout */
		handle_sfi_table(SFI_SIG_OEMB, NULL, NULL, sfi_parse_oemb);
		handle_sfi_table(SFI_SIG_OEM0, NULL, NULL, sfi_parse_oem0);
		handle_sfi_table(SFI_SIG_GPIO, NULL, NULL, sfi_parse_gpio);
		handle_sfi_table(SFI_SIG_DEVS, NULL, NULL, sfi_parse_devs);

		intel_mid_rproc_init();
		break;
	}

	/* Populate command line with SPID values */
	populate_spid_cmdline();

	return ret;
}
arch_initcall(intel_mid_platform_init);
