/*
 * iio_crystalcove_gpio.c - Intel Merrifield Crystal Cove GPIO Driver
 *
 * Copyright (C) 2012 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Bin Yang <bin.yang@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/gpio.h>
#include <asm/intel_vlv2.h>

#define NUM_GPIO		16

#define UPDATE_TYPE		(1 << 0)
#define UPDATE_MASK		(1 << 1)

#define GPIO0IRQ	0x0b
#define GPIO1IRQ	0x0c
#define MGPIO0IRQS0	0x19
#define MGPIO1IRQS0	0x1a
#define MGPIO0IRQSX	0x1b
#define MGPIO1IRQSX	0x1c
#define GPIO0P0CTLO	0x2b
#define GPIO0P0CTLI	0x33
#define GPIO1P0CTLO	0x3b
#define GPIO1P0CTLI	0x43

#define CTLI_INTCNT_NE	(1 << 1)
#define CTLI_INTCNT_PE	(2 << 1)
#define CTLI_INTCNT_BE	(3 << 1)

#define CTLO_DIR_OUT	(1 << 5)
#define CTLO_DRV_CMOS	(1 << 4)
#define CTLO_DRV_REN	(1 << 3)
#define CTLO_RVAL_2KDW	(0)
#define CTLO_RVAL_2KUP	(1 << 1)
#define CTLO_RVAL_50KDW	(2 << 1)
#define CTLO_RVAL_50KUP	(3 << 1)

#define CTLO_INPUT_DEF	(CTLO_DRV_CMOS | CTLO_DRV_REN | CTLO_RVAL_2KUP)
#define CTLO_OUTPUT_DEF	(CTLO_DIR_OUT | CTLO_INPUT_DEF)

struct crystalcove_gpio {
	struct mutex		buslock;
	struct gpio_chip	chip;
	int			irq;
	int			irq_base;
	int			update;
	int			trigger_type;
	int			irq_mask;
};
static struct crystalcove_gpio gpio_info;

static void __crystalcove_irq_mask(int gpio, int mask)
{
	u8 mirqs0 = gpio < 8 ? MGPIO0IRQS0 : MGPIO1IRQS0;
	int offset = gpio < 8 ? gpio : gpio - 8;

	if (mask)
		intel_mid_pmic_setb(mirqs0, 1 << offset);
	else
		intel_mid_pmic_clearb(mirqs0, 1 << offset);
}

static void __crystalcove_irq_type(int gpio, int type)
{
	int offset = gpio < 8 ? gpio : gpio - 8;
	u8 ctli = gpio < 8 ? GPIO0P0CTLI + gpio : GPIO1P0CTLI + (gpio - 8);

	type &= IRQ_TYPE_EDGE_BOTH;
	intel_mid_pmic_clearb(ctli, CTLI_INTCNT_BE);
	if (type == IRQ_TYPE_EDGE_BOTH)
		intel_mid_pmic_setb(ctli, CTLI_INTCNT_BE);
	else if (type == IRQ_TYPE_EDGE_RISING)
		intel_mid_pmic_setb(ctli, CTLI_INTCNT_PE);
	else if (type & IRQ_TYPE_EDGE_FALLING)
		intel_mid_pmic_setb(ctli, CTLI_INTCNT_NE);
}

static int crystalcove_gpio_direction_input(struct gpio_chip *chip,
		unsigned gpio)
{
	u8 ctlo = gpio < 8 ? GPIO0P0CTLO + gpio : GPIO1P0CTLO + (gpio - 8);

	intel_mid_pmic_writeb(ctlo, CTLO_INPUT_DEF);
	return 0;
}

static int crystalcove_gpio_direction_output(struct gpio_chip *chip,
		unsigned gpio, int value)
{
	u8 ctlo = gpio < 8 ? GPIO0P0CTLO + gpio : GPIO1P0CTLO + (gpio - 8);

	intel_mid_pmic_writeb(ctlo, CTLO_OUTPUT_DEF | value);
	return 0;
}

static int crystalcove_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	u8 ctli = gpio < 8 ? GPIO0P0CTLI + gpio : GPIO1P0CTLI + (gpio - 8);

	return intel_mid_pmic_readb(ctli) & 0x1;
}

static void crystalcove_gpio_set(struct gpio_chip *chip,
		unsigned gpio, int value)
{
	u8 ctlo = gpio < 8 ? GPIO0P0CTLO + gpio : GPIO1P0CTLO + (gpio - 8);

	if (value)
		intel_mid_pmic_setb(ctlo, 1);
	else
		intel_mid_pmic_clearb(ctlo, 1);
}

static int crystalcove_irq_type(struct irq_data *data, unsigned type)
{
	struct crystalcove_gpio *cg = irq_data_get_irq_chip_data(data);

	cg->trigger_type = type;
	cg->update |= UPDATE_TYPE;
	return 0;
}

static int crystalcove_gpio_to_irq(struct gpio_chip *chip, unsigned gpio)
{
	struct crystalcove_gpio *cg =
		container_of(chip, struct crystalcove_gpio, chip);

	return cg->irq_base + gpio;
}

static void crystalcove_bus_lock(struct irq_data *data)
{
	struct crystalcove_gpio *cg = irq_data_get_irq_chip_data(data);

	mutex_lock(&cg->buslock);
}

static void crystalcove_bus_sync_unlock(struct irq_data *data)
{
	struct crystalcove_gpio *cg = irq_data_get_irq_chip_data(data);
	int gpio = data->irq - cg->irq_base;

	if (cg->update & UPDATE_TYPE)
		__crystalcove_irq_type(gpio, cg->trigger_type);
	if (cg->update & UPDATE_MASK)
		__crystalcove_irq_mask(gpio, cg->irq_mask);
	cg->update = 0;
	mutex_unlock(&cg->buslock);
}

static void crystalcove_irq_unmask(struct irq_data *data)
{
	struct crystalcove_gpio *cg = irq_data_get_irq_chip_data(data);

	cg->irq_mask = 0;
	cg->update |= UPDATE_MASK;
}

static void crystalcove_irq_mask(struct irq_data *data)
{
	struct crystalcove_gpio *cg = irq_data_get_irq_chip_data(data);

	cg->irq_mask = 1;
	cg->update |= UPDATE_MASK;
}

static struct irq_chip crystalcove_irqchip = {
	.name			= "PMIC-GPIO",
	.irq_mask		= crystalcove_irq_mask,
	.irq_unmask		= crystalcove_irq_unmask,
	.irq_set_type		= crystalcove_irq_type,
	.irq_bus_lock		= crystalcove_bus_lock,
	.irq_bus_sync_unlock	= crystalcove_bus_sync_unlock,
};

static irqreturn_t crystalcove_gpio_irq_handler(int irq, void *data)
{
	struct crystalcove_gpio *cg = data;
	int pending;
	int gpio;

	pending = intel_mid_pmic_readb(GPIO0IRQ) & 0xff;
	pending |= (intel_mid_pmic_readb(GPIO1IRQ) & 0xff) << 8;
	intel_mid_pmic_writeb(GPIO0IRQ, pending & 0xff);
	intel_mid_pmic_writeb(GPIO1IRQ, (pending >> 8) & 0xff);
	for (gpio = 0; gpio < cg->chip.ngpio; gpio++) {
		if (pending & (1 << gpio)) {
			pr_err("crystalcove pin %d triggered\n", gpio);
			generic_handle_irq(cg->irq_base + gpio);
		}
	}
	return IRQ_HANDLED;
}

static void crystalcove_gpio_dbg_show(struct seq_file *s,
				struct gpio_chip *chip)
{
	struct crystalcove_gpio *cg =
		container_of(chip, struct crystalcove_gpio, chip);
	int gpio, offset;
	u8 ctlo, ctli, mirqs0, mirqsx, irq;

	for (gpio = 0; gpio < cg->chip.ngpio; gpio++) {
		offset = gpio < 8 ? gpio : gpio - 8;
		ctlo = intel_mid_pmic_readb(
			(gpio < 8 ? GPIO0P0CTLO : GPIO1P0CTLO) + offset);
		ctli = intel_mid_pmic_readb(
			(gpio < 8 ? GPIO0P0CTLI : GPIO1P0CTLI) + offset);
		mirqs0 = intel_mid_pmic_readb(
			gpio < 8 ? MGPIO0IRQS0 : MGPIO0IRQS0);
		mirqsx = intel_mid_pmic_readb(
			gpio < 8 ? MGPIO0IRQSX : MGPIO0IRQSX);
		irq = intel_mid_pmic_readb(
			gpio < 8 ? GPIO0IRQ : GPIO1IRQ);
		seq_printf(s,
			" gpio-%-2d %s %s %s %s "
			"ctlo=%2x,%s %s %s\n",
			gpio,
			ctlo & CTLO_DIR_OUT ? "out" : "in ",
			ctli & 0x1 ? "hi" : "lo",
			ctli & CTLI_INTCNT_NE ? "fall" : "    ",
			ctli & CTLI_INTCNT_PE ? "rise" : "    ",
			ctlo,
			mirqs0 & (1 << offset) ? "s0 mask  " : "s0 unmask",
			mirqsx & (1 << offset) ? "sx mask  " : "sx unmask",
			irq & (1 << offset) ? "pending" : "       ");
	}
}

static int __devinit crystalcove_gpio_probe(struct platform_device *pdev)
{
	int irq = platform_get_irq(pdev, 0);
	struct crystalcove_gpio *cg = &gpio_info;
	int retval;
	int i;
	int gpio_base, irq_base;
	struct device *dev = *(struct device **)pdev->dev.platform_data;

	mutex_init(&cg->buslock);
	cg->irq_base = VV_PMIC_GPIO_IRQBASE;
	cg->chip.label = "intel_crystalcove";
	cg->chip.direction_input = crystalcove_gpio_direction_input;
	cg->chip.direction_output = crystalcove_gpio_direction_output;
	cg->chip.get = crystalcove_gpio_get;
	cg->chip.set = crystalcove_gpio_set;
	cg->chip.to_irq = crystalcove_gpio_to_irq;
	cg->chip.base = VV_PMIC_GPIO_BASE;
	cg->chip.ngpio = NUM_GPIO;
	cg->chip.can_sleep = 1;
	cg->chip.dev = dev;
	cg->chip.dbg_show = crystalcove_gpio_dbg_show;
	retval = gpiochip_add(&cg->chip);
	if (retval) {
		pr_warn("crystalcove: add gpio chip error: %d\n", retval);
		return retval;
	}

	retval = request_threaded_irq(irq, NULL, crystalcove_gpio_irq_handler,
			IRQF_ONESHOT, "crystalcove_gpio", cg);
	if (retval) {
		pr_warn("Interrupt request failed\n");
		return retval;
	}

	irq_base = irq_alloc_descs(cg->irq_base, 0, NUM_GPIO, 0);
	if (cg->irq_base != irq_base)
		panic("gpio base irq fail, needs %d, return %d\n",
				cg->irq_base, irq_base);
	for (i = 0; i < NUM_GPIO; i++) {
		pr_err("gpio %x: set handler: %d\n", cg, i + cg->irq_base);
		irq_set_chip_data(i + cg->irq_base, cg);
		irq_set_chip_and_handler_name(i + cg->irq_base,
					      &crystalcove_irqchip,
					      handle_simple_irq,
					      "demux");
	}

	return 0;
}

static struct platform_driver crystalcove_gpio_driver = {
	.probe = crystalcove_gpio_probe,
	.driver = {
		.name = "crystal_cove_gpio",
	},
};

module_platform_driver(crystalcove_gpio_driver);

MODULE_AUTHOR("Yang Bin<bin.yang@intel.com>");
MODULE_DESCRIPTION("Intel Merrifield Crystal Cove GPIO Driver");
MODULE_LICENSE("GPL");
