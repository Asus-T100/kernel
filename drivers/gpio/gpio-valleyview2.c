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
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <asm/intel_vlv2.h>
#include <linux/pnp.h>

#define GPIO_PATH_MAX	64

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

struct gpio_bank_pnp {
	char		*name;
	int		gpio_base;
	int		irq_base;
	int		ngpio;
	unsigned	*to_pad;
};

static struct gpio_bank_pnp vlv_banks_pnp[] = {
	{
		.name = "GPO0",
		.gpio_base = VV_GPIO_BASE,
		.irq_base = VV_GPIO_IRQBASE,
		.ngpio = VV_NGPIO_SCORE,
		.to_pad = score_gpio_to_pad,
	},
	{
		.name = "GPO1",
		.gpio_base = VV_GPIO_BASE + VV_NGPIO_SCORE,
		.irq_base = VV_GPIO_IRQBASE + VV_NGPIO_SCORE,
		.ngpio = VV_NGPIO_NCORE,
		.to_pad = ncore_gpio_to_pad,
	},
	{
		.name = "GPO2",
		.gpio_base = VV_GPIO_BASE + VV_NGPIO_SCORE + VV_NGPIO_NCORE,
		.irq_base = VV_GPIO_IRQBASE + VV_NGPIO_SCORE + VV_NGPIO_NCORE,
		.ngpio = VV_NGPIO_SUS,
		.to_pad = sus_gpio_to_pad,
	},
	{
	},
};

struct vlv_gpio {
	struct gpio_chip	chip;
	struct pnp_dev		*pdev;
	spinlock_t		lock;
	void __iomem		*reg_base;
	unsigned		*gpio_to_pad;
	int			irq_base;
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
	return 0;
}

static void vlv_gpio_free(struct gpio_chip *chip, unsigned offset)
{
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
	void __iomem *reg;
	u32 base;
	u32 pending;

	spin_lock_irqsave(&vg->lock, flags);
	for (base = 0; base < vg->chip.ngpio; base += 32) {
		reg = vlv_gpio_reg(&vg->chip, base, VV_INT_STAT_REG);
		pending = readl(reg);
		seq_printf(s, "VV_INT_STAT_REG[%d-%d]: 0x%x\n",
				base, base+32, pending);
	}
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

static int
vlv_gpio_pnp_probe(struct pnp_dev *pdev, const struct pnp_device_id *id)
{
	struct vlv_gpio *vg;
	struct gpio_chip *gc;
	struct resource *mem_rc;
	struct device *dev = &pdev->dev;
	struct gpio_bank_pnp *bank;
	int ret = 0;
	int gpio_base;
	char path[GPIO_PATH_MAX];

	vg = devm_kzalloc(dev, sizeof(struct vlv_gpio), GFP_KERNEL);
	if (!vg) {
		dev_err(&pdev->dev, "can't allocate vlv_gpio chip data\n");
		return -ENOMEM;
	}
	vg->pdev = pdev;

	for (bank = vlv_banks_pnp; bank; bank++) {
		if (!strcmp(pdev->name, bank->name)) {
			vg->chip.ngpio = bank->ngpio;
			vg->gpio_to_pad = bank->to_pad;
			break;
		}
	}
	if (!bank) {
		dev_err(&pdev->dev, "can't find %s\n", pdev->name);
		ret = -ENODEV;
		goto err;
	}

	snprintf(path, sizeof(path), "\\_SB.%s", bank->name);
	gpio_base = acpi_get_gpio(path, 0);
	if (gpio_base < 0) {
		dev_err(&pdev->dev, "Cannot find ACPI GPIO chip %s", path);
		gpio_base = bank->gpio_base;
	}
	dev_info(&pdev->dev, "%s: gpio base %d\n", path, gpio_base);

	mem_rc = pnp_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_rc) {
		dev_err(&pdev->dev, "missing MEM resource\n");
		ret = -EINVAL;
		goto err;
	}

	vg->reg_base = devm_request_and_ioremap(dev, mem_rc);
	if (vg->reg_base == NULL) {
		dev_err(&pdev->dev, "error mapping resource\n");
		ret = -EINVAL;
		goto err;
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
	gc->base = gpio_base;
	gc->can_sleep = 0;
	gc->dev = dev;

	ret = gpiochip_add(gc);
	if (ret) {
		dev_err(&pdev->dev, "failed adding vlv-gpio chip\n");
		goto err;
	}

	return 0;
err:
	kfree(vg);
	return ret;
}

static const struct pnp_device_id vlv_gpio_pnp_match[] = {
	{ "INT33B2", 0 },
	{ }
};
MODULE_DEVICE_TABLE(pnp, vlv_gpio_pnp_match);

static struct pnp_driver vlv_gpio_pnp_driver = {
	.name		= "vlv_gpio",
	.id_table	= vlv_gpio_pnp_match,
	.probe          = vlv_gpio_pnp_probe,
};

static int __init vlv_gpio_init(void)
{
	return pnp_register_driver(&vlv_gpio_pnp_driver);
}

module_init(vlv_gpio_init);
