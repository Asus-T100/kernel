#include <linux/device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <asm/intel_vlv2.h>
#include <asm/io_apic.h>
#include <asm/acpi.h>
#include <asm/hw_irq.h>

/* FIXME: should be put into ACPI table */
static int __init register_psh_i2c_dev(void)
{
	static int psh_gpios[2] = { 59, 95 };
	struct i2c_board_info info = {
		I2C_BOARD_INFO("psh_byt_i2c", 0x19),
		.irq = 68,
		.platform_data = (void *)psh_gpios,
	};
	struct io_apic_irq_attr irq_attr;

	irq_attr.ioapic = mp_find_ioapic(68);
	irq_attr.ioapic_pin = 68;
	irq_attr.trigger = 1;
	irq_attr.polarity = 0;
	io_apic_set_pci_routing(NULL, 68, &irq_attr);
	i2c_register_board_info(5, &info, 1);

	return 0;
}

fs_initcall(register_psh_i2c_dev);
