/*
 * GPIO driver for Intel Valleyview 2 PCH >
 * Copyright (c) 2012, Intel Corporation.
 *
 * Author: Mathias Nyman <mathias.nyman@linux.intel.com>
 * Author: Yang Bin <bin.yang@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/irqdomain.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/io.h>

/* memory mapped register offsets */
#define VV_CONF0_REG		0x000
#define VV_CONF1_REG		0x004
#define VV_VAL_REG		0x008
#define VV_DFT_REG		0x00c
#define VV_INT_STAT_REG		0x800

/* VV_CONF0_REG register bits */
#define VV_TRIG_NEG		BIT(26)
#define VV_TRIG_POS		BIT(25)
#define VV_TRIG_LVL		BIT(24)
#define VV_PIN_MUX		0x07

/* VV_VAL_REG register bits */
#define VV_INPUT_EN		BIT(2)  /* 0: input enabled (active low)*/
#define VV_OUTPUT_EN		BIT(1)  /* 0: output enabled (active low)*/
#define VV_LEVEL		BIT(0)

#define VV_DIR_MASK		(BIT(1) | BIT(2))
#define VV_TRIG_MASK		(BIT(26) | BIT(25) | BIT(24))

#define VV_NGPIO_SCORE		102
#define VV_NGPIO_NCORE		28
#define VV_NGPIO_SUS		43

/*
 * Valleyview gpio controller consist of three separate sub-controllers called
 * SCORE, NCORE and SUS. The sub-controllers are identified by their acpi UID.
 *
 * GPIO numbering is _not_ ordered meaning that gpio # 0 in ACPI namespace does
 * _not_ correspond to the first gpio register at controller's gpio base.
 * There is no logic or pattern in mapping gpio numbers to registers (pads) so
 * each sub-controller needs to have its own mapping table
 */

static unsigned score_gpio_to_pad[VV_NGPIO_SCORE] = {
	85, 89, 93, 96, 99, 102, 98, 101, 34, 37,
	36, 38, 39, 35, 40, 84, 62, 61, 64, 59,
	54, 56, 60, 55, 63, 57, 51, 50, 53, 47,
	52, 49, 48, 43, 46, 41, 45, 42, 58, 44,
	95, 105, 70, 68, 67, 66, 69, 71, 65, 72,
	86, 90, 88, 92, 103, 77, 79, 83, 78, 81,
	80, 82, 13, 12, 15, 14, 17, 18, 19, 16,
	2, 1, 0, 4, 6, 7, 9, 8, 33, 32,
	31, 30, 29, 27, 25, 28, 26, 23, 21, 20,
	24, 22, 5, 3, 10, 11, 106, 87, 91, 104,
	97, 100,
};

static unsigned ncore_gpio_to_pad[VV_NGPIO_NCORE] = {
	19, 18, 17, 20, 21, 22, 24, 25, 23, 16,
	14, 15, 12, 26, 27, 1, 4, 8, 11, 0,
	3, 6, 10, 13, 2, 5, 9, 7,
};

static unsigned sus_gpio_to_pad[VV_NGPIO_SUS] = {
	29, 33, 30, 31, 32, 34, 36, 35, 38, 37,
	18, 11, 20, 17, 1, 8, 10, 19, 12, 0,
	2, 23, 39, 28, 27, 22, 21, 24, 25, 26,
	51, 56, 54, 49, 55, 48, 57, 50, 58, 52,
	53, 59, 40,
};

struct gpio_bank {
	char		*uid; /* acpi _UID */
	int		ngpio;
	unsigned	*to_pad;
};

static struct gpio_bank vlv_banks[] = {
	{
		.uid = "1",
		.ngpio = VV_NGPIO_SCORE,
		.to_pad = score_gpio_to_pad,
	},
	{
		.uid = "2",
		.ngpio = VV_NGPIO_NCORE,
		.to_pad = ncore_gpio_to_pad,
	},
	{
		.uid = "3",
		.ngpio = VV_NGPIO_SUS,
		.to_pad = sus_gpio_to_pad,
	},
	{
	},
};

struct vlv_gpio {
	struct gpio_chip	chip;
	struct irq_domain	*domain;
	struct platform_device	*pdev;
	spinlock_t		lock;
	void __iomem		*reg_base;
	unsigned		*gpio_to_pad;
};

static void __iomem *vlv_gpio_reg(struct gpio_chip *chip, unsigned offset,
				 int reg)
{
	struct vlv_gpio *vg = container_of(chip, struct vlv_gpio, chip);
	u32 reg_offset;
	void __iomem *ptr;

	if (reg == VV_INT_STAT_REG)
		reg_offset = (offset / 32) * 4;
	else
		reg_offset = vg->gpio_to_pad[offset] * 16;

	ptr = (void __iomem *) (vg->reg_base + reg_offset + reg);
	return ptr;
}

static int vlv_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *reg = vlv_gpio_reg(chip, offset, VV_CONF0_REG);
	u32 value;

	value =	readl(reg) & VV_PIN_MUX;

/* Policy about what should be done when requesting a gpio is unclear.
 * In most cases PIN MUX 000 means gpio function, with the exception of SUS
 * core pins 11-21 where gpio is mux 001.
 *
 * Some pins are set by bios to a non-gpio mux, but still marked as gpio
 * resource in acpi tables, and they work just as they should when not touching
 * the pin muxing. (For example mmc card detect switch)
 *
 * option 1, check pin mux is "gpio", else fail (FIXME gpio SUS pins 11-21):
 *	if (value)
 *		return -EINVAL;
 *
 * option 2, force pin mux to gpio mode (FIXME gpio SUS pins 11-21):
 *	writel(value & ~VV_PIN_MUX, reg);
 *
 * option 3: don't touch the pinmuxing at all, let BIOS handle it
 */
	return 0;
}

static void vlv_gpio_free(struct gpio_chip *chip, unsigned offset)
{
}

static int vlv_irq_type(struct irq_data *d, unsigned type)
{
	struct vlv_gpio *vg = irq_data_get_irq_chip_data(d);
	u32 offset = irqd_to_hwirq(d);
	u32 value;
	unsigned long flags;
	void __iomem *reg = vlv_gpio_reg(&vg->chip, offset, VV_CONF0_REG);

	if (offset >= vg->chip.ngpio)
		return -EINVAL;

	spin_lock_irqsave(&vg->lock, flags);
	value = readl(reg);

	if (type & IRQ_TYPE_EDGE_RISING)
		value |= VV_TRIG_POS;
	else
		value &= ~VV_TRIG_POS;

	if (type & IRQ_TYPE_EDGE_FALLING)
		value |= VV_TRIG_NEG;
	else
		value &= ~VV_TRIG_NEG;

	writel(value, reg);

	spin_unlock_irqrestore(&vg->lock, flags);

	return 0;
}

static int vlv_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *reg = vlv_gpio_reg(chip, offset, VV_VAL_REG);
	return readl(reg) & VV_LEVEL;
}

static void vlv_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct vlv_gpio *vg = container_of(chip, struct vlv_gpio, chip);
	void __iomem *reg = vlv_gpio_reg(chip, offset, VV_VAL_REG);
	unsigned long flags;
	u32 old_val;

	spin_lock_irqsave(&vg->lock, flags);

	old_val = readl(reg);

	if (value)
		writel(old_val | VV_LEVEL, reg);
	else
		writel(old_val & ~VV_LEVEL, reg);

	spin_unlock_irqrestore(&vg->lock, flags);
}

static int vlv_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct vlv_gpio *vg = container_of(chip, struct vlv_gpio, chip);
	void __iomem *reg = vlv_gpio_reg(chip, offset, VV_VAL_REG);
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&vg->lock, flags);

	value = readl(reg) | VV_DIR_MASK;
	value = value & (~VV_INPUT_EN); /* active low */
	writel(value, reg);

	spin_unlock_irqrestore(&vg->lock, flags);

	return 0;
}

static int vlv_gpio_direction_output(struct gpio_chip *chip,
				     unsigned gpio, int value)
{
	struct vlv_gpio *vg = container_of(chip, struct vlv_gpio, chip);
	void __iomem *reg = vlv_gpio_reg(chip, gpio, VV_VAL_REG);
	unsigned long flags;
	u32 reg_val;

	spin_lock_irqsave(&vg->lock, flags);

	reg_val = readl(reg) | (VV_DIR_MASK | !!value);
	reg_val &= ~(VV_OUTPUT_EN | !value);
	writel(reg_val, reg);

	spin_unlock_irqrestore(&vg->lock, flags);

	return 0;
}

static void vlv_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct vlv_gpio *vg = container_of(chip, struct vlv_gpio, chip);
	int i;
	unsigned long flags;
	u32 conf0,  val, offs;

	spin_lock_irqsave(&vg->lock, flags);
	for (i = 0; i < vg->chip.ngpio; i++) {

		offs = vg->gpio_to_pad[i] * 16;
		conf0 = readl(vg->reg_base + offs + VV_CONF0_REG);
		val = readl(vg->reg_base + offs + VV_VAL_REG);

		seq_printf(s, " gpio-%-3d %s %s %s pad-%-3d offset:0x%03x "
				"mux:%d %s %s %s\n",
			   i,
			   val & VV_INPUT_EN ? "  " : "in",
			   val & VV_OUTPUT_EN ? "   " : "out",
			   val & VV_LEVEL ? "hi" : "lo",
			   vg->gpio_to_pad[i], offs,
			   conf0 & 0x7,
			   conf0 & VV_TRIG_NEG ? "fall" : "",
			   conf0 & VV_TRIG_POS ? "rise" : "",
			   conf0 & VV_TRIG_LVL ? "lvl " : "");

	}
	spin_unlock_irqrestore(&vg->lock, flags);
}

static int vlv_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct vlv_gpio *vg = container_of(chip, struct vlv_gpio, chip);
	return irq_create_mapping(vg->domain, offset);
}

static void vlv_gpio_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct irq_data *data = irq_desc_get_irq_data(desc);
	struct vlv_gpio *vg = irq_data_get_irq_handler_data(data);
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	u32 base, pin, mask;
	void __iomem *reg;
	u32 pending;
	unsigned virq;

	/* check from GPIO controller which pin triggered the interrupt */
	for (base = 0; base < vg->chip.ngpio; base += 32) {

		reg = vlv_gpio_reg(&vg->chip, base, VV_INT_STAT_REG);

		while ((pending = readl(reg))) {
			pin = __ffs(pending);
			mask = BIT(pin);
			/* Clear before handling so we can't lose an edge */
			writel(mask, reg);

			virq = irq_find_mapping(vg->domain, base + pin);
			generic_handle_irq(virq);
		}
	}
	chip->irq_eoi(data);
}

static void vlv_irq_unmask(struct irq_data *d)
{
}

static void vlv_irq_mask(struct irq_data *d)
{
}

static struct irq_chip vlv_irqchip = {
	.name = "VLV-GPIO",
	.irq_mask = vlv_irq_mask,
	.irq_unmask = vlv_irq_unmask,
	.irq_set_type = vlv_irq_type,
};

static void vlv_gpio_irq_init_hw(struct vlv_gpio *vg)
{
	void __iomem *reg;
	u32 base;

	/* clear interrupt status trigger registers */
	for (base = 0; base < vg->chip.ngpio; base += 32) {
		reg = vlv_gpio_reg(&vg->chip, base, VV_INT_STAT_REG);
		writel(0xffffffff, reg);
	}
}

static int vlv_gpio_irq_map(struct irq_domain *d, unsigned int virq,
			    irq_hw_number_t hw)
{
	struct vlv_gpio *vg = d->host_data;

	irq_set_chip_and_handler_name(virq, &vlv_irqchip, handle_simple_irq,
				      "demux");
	irq_set_chip_data(virq, vg);
	irq_set_irq_type(virq, IRQ_TYPE_NONE);

	return 0;
}

static const struct irq_domain_ops vlv_gpio_irq_ops = {
	.map = vlv_gpio_irq_map,
};

static int vlv_gpio_probe(struct platform_device *pdev)
{
	struct vlv_gpio *vg;
	struct gpio_chip *gc;
	struct resource *mem_rc, *irq_rc;
	struct device *dev = &pdev->dev;
	struct acpi_device *acpi_dev;
	struct gpio_bank *bank;
	acpi_handle handle = ACPI_HANDLE(dev);
	unsigned hwirq;
	int ret;

	if (acpi_bus_get_device(handle, &acpi_dev))
		return -ENODEV;

	vg = devm_kzalloc(dev, sizeof(struct vlv_gpio), GFP_KERNEL);
	if (!vg) {
		dev_err(&pdev->dev, "can't allocate vlv_gpio chip data\n");
		ret = -ENOMEM;
	}

	for (bank = vlv_banks; bank->uid; bank++) {
		if (!strcmp(acpi_dev->pnp.unique_id, bank->uid)) {
			vg->chip.ngpio = bank->ngpio;
			vg->gpio_to_pad = bank->to_pad;
			break;
		}
	}

	if (!vg->chip.ngpio || !vg->gpio_to_pad)
		return -ENODEV;

	vg->pdev = pdev;
	platform_set_drvdata(pdev, vg);

	mem_rc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq_rc = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (!mem_rc) {
		dev_err(&pdev->dev, "missing MEM resource\n");
		return -EINVAL;
	}

	vg->reg_base = devm_request_and_ioremap(dev, mem_rc);

	if (vg->reg_base == NULL) {
		dev_err(&pdev->dev, "error mapping resource\n");
		return -EFAULT;
	}

	spin_lock_init(&vg->lock);

	gc = &vg->chip;
	gc->label = dev_name(&pdev->dev);
	gc->owner = THIS_MODULE;
	gc->request = vlv_gpio_request;
	gc->free = vlv_gpio_free;
	gc->direction_input = vlv_gpio_direction_input;
	gc->direction_output = vlv_gpio_direction_output;
	gc->get = vlv_gpio_get;
	gc->set = vlv_gpio_set;
	gc->dbg_show = vlv_gpio_dbg_show;
	gc->base = -1;
	gc->can_sleep = 0;
	gc->dev = dev;

	ret = gpiochip_add(gc);
	if (ret) {
		dev_err(&pdev->dev, "failed adding vlv-gpio chip\n");
		return ret;
	}

	/* set up interrupts  */
	if (irq_rc && irq_rc->start) {
		hwirq = irq_rc->start;
		gc->to_irq = vlv_gpio_to_irq;

		vg->domain = irq_domain_add_linear(NULL, gc->ngpio,
						   &vlv_gpio_irq_ops, vg);
		if (!vg->domain)
			return -ENXIO;

		vlv_gpio_irq_init_hw(vg);

		irq_set_handler_data(hwirq, vg);
		irq_set_chained_handler(hwirq, vlv_gpio_irq_handler);
	}

	return 0;
}

static const struct acpi_device_id vlv_gpio_acpi_match[] = {
	{ "INT33B2", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, vlv_gpio_acpi_match);

static int vlv_gpio_remove(struct platform_device *pdev)
{
	struct vlv_gpio *vg = platform_get_drvdata(pdev);
	int err;
	err = gpiochip_remove(&vg->chip);
	if (err)
		dev_warn(&pdev->dev, "failed to remove gpio_chip.\n");

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver vlv_gpio_driver = {
	.probe          = vlv_gpio_probe,
	.remove         = vlv_gpio_remove,
	.driver         = {
		.name   = "vlv_gpio",
		.owner  = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(vlv_gpio_acpi_match),
	},
};

static int __init vlv_gpio_init(void)
{
	return platform_driver_register(&vlv_gpio_driver);
}

subsys_initcall(vlv_gpio_init);
