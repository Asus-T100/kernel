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
#include <linux/ipc_device.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/intel_mid_pm.h>
#include <linux/hsi/hsi.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <linux/atomisp_platform.h>
#include <linux/spinlock.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/blkdev.h>

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
#include <asm/apb_timer.h>
#include <asm/reboot.h>
#include "intel_mid_weak_decls.h"

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
struct sfi_soft_platform_id spid;
u32 board_id;
static u32 sfi_mtimer_usage[SFI_MTMR_MAX_NUM];
static struct sfi_timer_table_entry sfi_mtimer_array[SFI_MTMR_MAX_NUM];
enum intel_mid_cpu_type __intel_mid_cpu_chip;
EXPORT_SYMBOL_GPL(__intel_mid_cpu_chip);

#ifdef CONFIG_X86_MRFLD
enum intel_mrfl_sim_type __intel_mrfl_sim_platform;
EXPORT_SYMBOL_GPL(__intel_mrfl_sim_platform);
#endif /* X86_CONFIG_MRFLD */

int sfi_mtimer_num;

struct sfi_rtc_table_entry sfi_mrtc_array[SFI_MRTC_MAX];
EXPORT_SYMBOL_GPL(sfi_mrtc_array);
int sfi_mrtc_num;
#ifdef CONFIG_X86_MDFLD
void (*saved_shutdown)(void);
#include <intel_soc_pmu.h>
/* This function is here just to have a hook to execute code before
 * generic x86 shutdown is executed. saved_shutdown contains pointer
 * to original generic x86 shutdown function */
void mfld_shutdown(void)
{
	down(&mid_pmu_cxt->scu_ready_sem);

	if (saved_shutdown)
		saved_shutdown();
}
#endif

/* Unified message bus read/write operation */
DEFINE_SPINLOCK(msgbus_lock);

u32 intel_mid_msgbus_read32_raw(u32 cmd)
{
	struct pci_dev *pci_root;
	unsigned long irq_flags;
	u32 data;

	pci_root = pci_get_bus_and_slot(0, PCI_DEVFN(0, 0));

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	pci_read_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, &data);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);

	pci_dev_put(pci_root);

	return data;
}
EXPORT_SYMBOL(intel_mid_msgbus_read32_raw);

void intel_mid_msgbus_write32_raw(u32 cmd, u32 data)
{
	struct pci_dev *pci_root;
	unsigned long irq_flags;

	pci_root = pci_get_bus_and_slot(0, PCI_DEVFN(0, 0));

	spin_lock_irqsave(&msgbus_lock, irq_flags);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_DATA_REG, data);
	pci_write_config_dword(pci_root, PCI_ROOT_MSGBUS_CTRL_REG, cmd);
	spin_unlock_irqrestore(&msgbus_lock, irq_flags);

	pci_dev_put(pci_root);
}
EXPORT_SYMBOL(intel_mid_msgbus_write32_raw);

u32 intel_mid_msgbus_read32(u8 port, u8 addr)
{
	u32 cmd = (PCI_ROOT_MSGBUS_READ << 24) | (port << 16) |
		(addr << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;

	return intel_mid_msgbus_read32_raw(cmd);
}
EXPORT_SYMBOL(intel_mid_msgbus_read32);

void intel_mid_msgbus_write32(u8 port, u8 addr, u32 data)
{
	u32 cmd = (PCI_ROOT_MSGBUS_WRITE << 24) | (port << 16) |
		(addr << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;

	intel_mid_msgbus_write32_raw(cmd, data);
}
EXPORT_SYMBOL(intel_mid_msgbus_write32);

/* when ITP is needed we must avoid touching the configurations of these pins.*/
/* see gpio part of this file */
static int itp_connected;
static int __init parse_itp(char *arg)
{
	itp_connected = 1;
	return 0;
}
early_param("itp", parse_itp);

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
		pr_debug("timer[%d]: paddr = 0x%08x, freq = %dHz,"
			" irq = %d\n", totallen, (u32)pentry->phys_addr,
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

/* [REVERT ME] ARAT capability not set in VP. Force setting */
#ifdef CONFIG_X86_MRFLD
	if (intel_mrfl_identify_sim() == INTEL_MRFL_CPU_SIMULATION_VP)
		set_cpu_cap(&boot_cpu_data, X86_FEATURE_ARAT);
#endif /* CONFIG_X86_MRFLD */

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
	else {
		pr_err("Unknown Moorestown CPU (%d:%d), default to Lincroft\n",
			boot_cpu_data.x86, boot_cpu_data.x86_model);
		__intel_mid_cpu_chip = INTEL_MID_CPU_CHIP_LINCROFT;
	}
}

/* MID systems don't have i8042 controller */
int intel_mid_i8042_detect(void)
{
	return 0;
}

static int force_cold_boot;
module_param(force_cold_boot, bool, 0644);
MODULE_PARM_DESC(force_cold_boot,
		 "Set to Y to force a COLD BOOT instead of a COLD RESET "
		 "on the next reboot system call.");

static void intel_mid_reboot(void)
{
	if (intel_scu_ipc_fw_update()) {
		pr_debug("intel_scu_fw_update: IFWI upgrade failed...\n");
		BUG();
	}
	if (force_cold_boot) {
		pr_info("Immediate COLD BOOT\n");
		intel_scu_ipc_simple_command(IPCMSG_COLD_BOOT, 0);
	} else {
		pr_info("Immediate COLD RESET\n");
		intel_scu_ipc_simple_command(IPCMSG_COLD_RESET, 0);
	}
}

static void intel_mid_emergency_reboot(char *cmd)
{
	/* Change system state to poll IPC status until IPC not busy*/
	system_state = SYSTEM_RESTART;

	while (intel_scu_ipc_check_status())
		udelay(10);

	if (force_cold_boot)
		intel_scu_ipc_raw_cmd(IPCMSG_COLD_BOOT,
			0, NULL, 0, NULL, 0, 0, 0);
	else
		intel_scu_ipc_raw_cmd(IPCMSG_COLD_RESET,
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

	/* Moorestown specific power_off/restart method */
	pm_power_off = intel_mid_power_off;
#ifdef CONFIG_X86_MDFLD
	if (mfld_shutdown) {
		saved_shutdown = machine_ops.shutdown;
		machine_ops.shutdown = mfld_shutdown;
	}
#endif
	machine_ops.restart = intel_mid_reboot;
	machine_ops.emergency_restart  = intel_mid_emergency_reboot;

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

#define MAX_SCU_SPI	24
static struct spi_board_info *spi_devs[MAX_SCU_SPI];
static int spi_next_dev;

#define MAX_SCU_I2C	24
static struct i2c_board_info *i2c_devs[MAX_SCU_I2C];
static int i2c_bus[MAX_SCU_I2C];
static int i2c_next_dev;

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

#define MAX_DELAYEDDEVS 60
static void *delayed_devs[MAX_DELAYEDDEVS];
typedef void (*delayed_callback_t)(void *dev_desc);
static delayed_callback_t delayed_callbacks[MAX_DELAYEDDEVS];
static int delayed_next_dev;

void intel_delayed_device_register(void *dev,
				void (*delayed_callback)(void *dev_desc))
{
	delayed_devs[delayed_next_dev] = dev;
	delayed_callbacks[delayed_next_dev++] = delayed_callback;
	BUG_ON(delayed_next_dev == MAX_DELAYEDDEVS);
}

/* Called by IPC driver, and we have only one IPC bus */
void intel_scu_devices_create(int bus_id)
{
	int i;

	if (bus_id == IPC_SCU) {

		ipc_register_devices(bus_id);

		for (i = 0; i < delayed_next_dev; i++)
			delayed_callbacks[i](delayed_devs[i]);

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
				i2c_register_board_info(i2c_bus[i],
						i2c_devs[i], 1);
		}
	}

}
EXPORT_SYMBOL_GPL(intel_scu_devices_create);

/* Called by IPC driver */
void intel_scu_devices_destroy(int bus_id)
{
	if (bus_id == IPC_SCU)
		ipc_remove_devices(bus_id);
}
EXPORT_SYMBOL_GPL(intel_scu_devices_destroy);

void __init install_ipc_irq_resource(struct ipc_device *ipcdev, int irq)
{
	/* Single threaded */
	static struct resource __initdata res = {
		.name = "IRQ",
		.flags = IORESOURCE_IRQ,
	};
	res.start = irq;
	ipc_device_add_resources(ipcdev, &res, 1);
}

static void __init sfi_handle_ipc_dev(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct ipc_device *ipcdev;
	void *pdata = NULL;

	pr_info("IPC bus = %d, name = %16.16s, "
		"irq = 0x%2x\n", pentry->host_num, pentry->name, pentry->irq);

	pdata = dev->get_platform_data(pentry);

	ipcdev = ipc_device_alloc(pentry->name, 0);
	if (ipcdev == NULL) {
		pr_err("out of memory for SFI ipc device '%s'.\n",
			pentry->name);
		return;
	}

	install_ipc_irq_resource(ipcdev, pentry->irq);

	ipcdev->dev.platform_data = pdata;
	ipc_device_add_to_list(ipcdev);
}

#ifdef CONFIG_INTEL_SCU_IPC
static int __init intel_scu_ipc_subdev_init(void)
{
	int i, ret;
	static struct ipc_board_info info[] __initdata = {
		[0] = {
			.name = "intel_scu_pmic",
			.bus_id = IPC_SCU,
		},

		[1] = {
			.name = "intel_scu_mip",
			.bus_id = IPC_SCU,
		},

		[2] = {
			.name = "intel_fw_update",
			.bus_id = IPC_SCU,
		},
	};

	for (i = 0; i < ARRAY_SIZE(info); i++) {
		ret = ipc_new_device(&info[i]);
		if (ret) {
			pr_err("Fail to create ipc device: %s\n", info[i].name);
			return -EINVAL;
		}
	}

	return 0;
}
postcore_initcall(intel_scu_ipc_subdev_init);
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
	pr_info("SPI bus = %d, name = %16.16s, "
		"irq = 0x%2x, max_freq = %d, cs = %d\n",
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
	pr_info("I2C bus = %d, name = %16.16s, "
		"irq = 0x%2x, addr = 0x%x\n",
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


	memset(&hsi_info, 0, sizeof(hsi_info));
	hsi_info.name = pentry->name;
	hsi_info.hsi_id = pentry->host_num;
	hsi_info.port = pentry->addr;
	pr_info("HSI bus = %d, name = %16.16s, "
		"port = %d\n",
		hsi_info.hsi_id,
		hsi_info.name,
		hsi_info.port);
	pdata = dev->get_platform_data(&hsi_info);

	if (pdata) {
		pr_info("SFI register platform data for HSI device %s\n",
					dev->name);
		hsi_register_board_info(pdata, 2);
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

struct devs_id *get_device_id(u8 type, char *name)
{
	const struct devs_id *dev = device_ids;

	if (device_ids == NULL)
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
#ifdef CONFIG_X86_MRFLD
				irq_attr.polarity = 0; /* Active high */
#else
				irq_attr.polarity = 1; /* Active low */
#endif
				io_apic_set_pci_routing(NULL, irq, &irq_attr);
			} else
				printk(KERN_INFO, "APIC entry not found for: name=%s, irq=%d, ioapic=%d",
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
#ifndef CONFIG_HSI_NO_MODEM
				sfi_handle_hsi_dev(pentry, dev);
#endif
				break;
			case SFI_DEV_TYPE_UART:
			default:
				;
			}
		}
	}

	return 0;
}

static int __init sfi_parse_oemb(struct sfi_table_header *table)
{
	struct sfi_table_oemb *oemb;
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

	memcpy(&spid, &oemb->spid, sizeof(struct sfi_soft_platform_id));

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
		"\tOEMB spid fru[9..5]          : %02x %02x %02x %02x %02x\n",
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
		spid.fru[6], spid.fru[5]);
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

static struct kobject *board_properties;

static int __init intel_mid_board_properties(void)
{
	int ret = 0;

	board_properties = kobject_create_and_add("board_properties", NULL);
	if (!board_properties) {
		pr_err("failed to create /sys/board_properties\n");
		ret = -EINVAL;
	}

	return ret;
}

int intel_mid_create_property(const struct attribute *attr)
{
	if (!board_properties)
		return -EINVAL;

	return sysfs_create_file(board_properties, attr);
}

static int __init intel_mid_platform_init(void)
{
	intel_mid_board_properties();

	/* create sysfs entries for soft platform id */
	spid_kobj = kobject_create_and_add("spid", NULL);
	if (!spid_kobj)
		pr_err("SPID: ENOMEM for spid_kobj\n");
	if (sysfs_create_group(spid_kobj, &spid_attr_group))
		pr_err("SPID: failed to create /sys/spid\n");

	/* Get MFD Validation SFI OEMB Layout */
	sfi_table_parse(SFI_SIG_OEMB, NULL, NULL, sfi_parse_oemb);
	sfi_table_parse(SFI_SIG_OEM0, NULL, NULL, sfi_parse_oem0);
	sfi_table_parse(SFI_SIG_GPIO, NULL, NULL, sfi_parse_gpio);
	sfi_table_parse(SFI_SIG_DEVS, NULL, NULL, sfi_parse_devs);
	return 0;
}
arch_initcall(intel_mid_platform_init);
