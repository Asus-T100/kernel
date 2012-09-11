#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <asm/setup.h>
#include <asm/intel_mid_hsu.h>

/* a 3 bits bit-map, from 0 to 7, default 0 */
unsigned char hsu_dma_enable;
EXPORT_SYMBOL_GPL(hsu_dma_enable);

int __init setup_hsu_dma_enable_flag(char *p)
{
	if (!p)
		return -EINVAL;

	hsu_dma_enable = (unsigned char)memparse(p, &p);
	if (hsu_dma_enable & (~0x7))
		return -EINVAL;

	return 0;
}
early_param("hsu_dma", setup_hsu_dma_enable_flag);

int hsu_rx_wa;
EXPORT_SYMBOL_GPL(hsu_rx_wa);

int __init setup_hsu_rx_workarround_flag(char *p)
{
	hsu_rx_wa = 1;
	return 0;
}
early_param("hsu_rx_wa", setup_hsu_rx_workarround_flag);

static struct mfld_hsu_info default_hsu_info[] = {
	[0] = {
		.id = 0,
		.name = "hsu0",
		.wake_gpio = 13,
		.cts_gpio = 96+28,
		.cts_alt = 1,
		.rts_gpio = 96+29,
		.rts_alt = 1,
	},
	[1] = {
		.id = 1,
		.name = "hsu1",
		.wake_gpio = 64,
		.rx_gpio = 64,
		.rx_alt = 1,
		.tx_gpio = 65,
		.tx_alt = 1,
		.cts_gpio = 68,
		.cts_alt = 1,
		.rts_gpio = 66,
		.rts_alt = 2,
	},
	[2] = {
		.id = 2,
		.name = "hsu2",
	},
	[3] = {
		.id = 1,
		.name = "hsu3",
		.wake_gpio = 96+30,
		.rx_gpio = 96+30,
		.rx_alt = 1,
		.tx_gpio = 96+31,
		.tx_alt = 1,
	},

};

struct mfld_hsu_info *platform_hsu_info = default_hsu_info;

static irqreturn_t hsu_wakeup_isr(int irq, void *dev)
{
	dev_dbg(dev, "HSU wake up\n");
	pm_runtime_get(dev);
	pm_runtime_put(dev);
	return IRQ_HANDLED;
}

static void hsu_port_enable(int port)
{
	struct mfld_hsu_info *info = platform_hsu_info + port;

	if (info->rx_gpio) {
		lnw_gpio_set_alt(info->rx_gpio, info->rx_alt);
		gpio_direction_input(info->rx_gpio);
	}
	if (info->tx_gpio) {
		gpio_direction_output(info->tx_gpio, 0);
		lnw_gpio_set_alt(info->tx_gpio, info->tx_alt);
	}
	if (info->cts_gpio) {
		lnw_gpio_set_alt(info->cts_gpio, info->cts_alt);
		gpio_direction_input(info->cts_gpio);
	}
	if (info->rts_gpio) {
		gpio_direction_output(info->rts_gpio, 0);
		lnw_gpio_set_alt(info->rts_gpio, info->rts_alt);
	}
}

static void hsu_port_disable(int port)
{
	struct mfld_hsu_info *info = platform_hsu_info + port;

	if (info->rx_gpio) {
		lnw_gpio_set_alt(info->rx_gpio, LNW_GPIO);
		gpio_direction_input(info->rx_gpio);
	}
	if (info->tx_gpio) {
		lnw_gpio_set_alt(info->tx_gpio, LNW_GPIO);
		gpio_direction_input(info->tx_gpio);
	}
	if (info->cts_gpio) {
		lnw_gpio_set_alt(info->cts_gpio, LNW_GPIO);
		gpio_direction_input(info->cts_gpio);
	}
	if (info->rts_gpio) {
		lnw_gpio_set_alt(info->rts_gpio, LNW_GPIO);
		gpio_direction_input(info->rts_gpio);
	}
}

void intel_mid_hsu_set_rts(int port, int set)
{
	struct mfld_hsu_info *info = platform_hsu_info + port;

	if (set) {
		if (info->rts_gpio) {
			gpio_direction_output(info->rts_gpio, 1);
			lnw_gpio_set_alt(info->rts_gpio, LNW_GPIO);
		}
	} else {
		if (info->rts_gpio)
			lnw_gpio_set_alt(info->rts_gpio, info->rts_alt);
	}
}

void intel_mid_hsu_suspend(int port)
{
	int ret;
	struct mfld_hsu_info *info = platform_hsu_info + port;

	if (info->rts_gpio) {
		gpio_direction_output(info->rts_gpio, 1);
		lnw_gpio_set_alt(info->rts_gpio, LNW_GPIO);
	}
	if (info->wake_gpio) {
		lnw_gpio_set_alt(info->wake_gpio, LNW_GPIO);
		gpio_direction_input(info->wake_gpio);
		udelay(100);
		ret = request_irq(gpio_to_irq(info->wake_gpio), info->wake_isr,
				IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING,
				info->name, info->dev);
		if (ret)
			dev_err(info->dev, "failed to register wakeup irq\n");
	}
}

void intel_mid_hsu_resume(int port)
{
	struct mfld_hsu_info *info = platform_hsu_info + port;

	if (info->wake_gpio)
		free_irq(gpio_to_irq(info->wake_gpio), info->dev);
	hsu_port_enable(port);
}

void intel_mid_hsu_switch(int port)
{
	int i;
	struct mfld_hsu_info *tmp;
	struct mfld_hsu_info *info = platform_hsu_info + port;

	for (i = 0; i < MFLD_HSU_NUM; i++) {
		tmp = platform_hsu_info + i;
		if (tmp != info && tmp->id == info->id)
			hsu_port_disable(i);
	}
	hsu_port_enable(port);
}


/**
 * intel_mid_hsu_port_map- get logic function port and share logic
 * function port index
 *
 * @logic_idx: basic logic port index for one physical UART port
 * @share_idx: share logic port index for same physical UART port that
 *             logci_idx map
 *
 * physical 3 UART port will map to 4 logic UART port, so there will
 * be 1 physical  UART port with 2 logci functions.
 * e.g. penwell physical UART port 1 map to logic port 1 for modem
 * function and logic port 3 for console function.
 */

void intel_mid_hsu_port_map(int *logic_idx, int *share_idx)
{
	int i, j;

	for (i = 0; i < ARRAY_SIZE(default_hsu_info); i++) {
		for (j = i + 1; j < ARRAY_SIZE(default_hsu_info); j++) {
			if (platform_hsu_info[i].id ==
			platform_hsu_info[j].id) {
				*logic_idx = i;
				*share_idx = j;
				return;
			}
		}
	}
}

int intel_mid_hsu_init(int port, struct device *dev, irq_handler_t wake_isr)
{
	struct mfld_hsu_info *info;

	if (platform_hsu_info == NULL)
		return -ENODEV;
	if (port >= MFLD_HSU_NUM)
		return -ENODEV;
	info = platform_hsu_info + port;
	info->dev = dev;
	info->wake_isr = wake_isr;
	if (info->wake_gpio)
		gpio_request(info->wake_gpio, "hsu");
	if (info->rx_gpio)
		gpio_request(info->rx_gpio, "hsu");
	if (info->tx_gpio)
		gpio_request(info->tx_gpio, "hsu");
	if (info->cts_gpio)
		gpio_request(info->cts_gpio, "hsu");
	if (info->rts_gpio)
		gpio_request(info->rts_gpio, "hsu");
}

