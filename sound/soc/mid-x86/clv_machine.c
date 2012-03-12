/*
 *  clv_machine.c - ASoc Machine driver for Intel Cloverview MID platform
 *
 *  Copyright (C) 2011-12 Intel Corp
 *  Author: KP Jeeja<jeeja.kp@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/ipc_device.h>
#include <asm/intel_scu_ipc.h>
#include <sound/intel_sst.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/cs42l73.h"
#include <linux/gpio.h>

/* CDB42L73 Y1 (6.144 MHz) )oscillator =  MCLK1 */
#define C42L73_DEFAULT_MCLK	19200000
#define CS42L73_HPSENSE_GPIO 	34
#define CS42L73_BUTTON_GPIO	32

static unsigned int vsp_mode;

static vsp_mode_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = vsp_mode;
	return 0;
}

static int vsp_mode_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	vsp_mode  = ucontrol->value.integer.value[0];
	return 0;
}

static const char *vsp_mode_text[] = {"Master", "Slave"};

static const struct soc_enum vsp_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(vsp_mode_text), vsp_mode_text);

static const struct snd_kcontrol_new clv_controls[] = {
	SOC_ENUM_EXT("VSP Mode", vsp_mode_enum, vsp_mode_get, vsp_mode_set),
};

static int clv_asp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	/* CS42L73  Slave Mode`*/
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);

	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}
	ret = snd_soc_dai_set_sysclk(codec_dai, CS42L73_CLKID_MCLK1,
			C42L73_DEFAULT_MCLK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec clock %d\n", ret);
		return ret;
	}
	return 0;
}

static int clv_vsp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret , clk_source;

	if (!vsp_mode) {
		pr_debug("Master Mode selected\n");
		/* CS42L73  Master Mode`*/
		fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBM_CFM;
		clk_source = SND_SOC_CLOCK_OUT;

	} else {
		pr_debug("Slave Mode selected\n");
		/* CS42L73  Slave Mode`*/
		fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBS_CFS;
		clk_source = SND_SOC_CLOCK_IN;
	}

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, CS42L73_CLKID_MCLK1,
			C42L73_DEFAULT_MCLK, clk_source);
	if (ret < 0) {
		pr_err("can't set codec clock %d\n", ret);
		return ret;
	}
	return 0;
}

struct clv_mc_private {
	struct ipc_device *socdev;
	void __iomem *int_base;
	struct snd_soc_codec *codec;
};

/* Headset jack */
struct snd_soc_jack clv_jack;

/* Headset jack detection gpios */
static struct snd_soc_jack_gpio hs_jack_gpio = {

	.gpio = CS42L73_HPSENSE_GPIO,
	.name = "hsdet-gpio",
	.report = SND_JACK_HEADSET,
	.debounce_time = 400,
};

static struct snd_soc_jack_gpio hs_button_gpio = {

	.gpio = CS42L73_BUTTON_GPIO,
	.name = "hsbutton-gpio",
	.report = SND_JACK_BTN_0,
	.debounce_time = 100,
};
static void clv_soc_jack_gpio_detect(struct snd_soc_jack_gpio *gpio)
{
	int enable;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	cs42l73_hp_detection(codec, jack, enable);
}

/* irq handler for gpio pin */
static irqreturn_t clv_gpio_handler(int irq, void *data)
{
	struct snd_soc_jack_gpio *gpio = data;

	schedule_delayed_work(&gpio->work,
				msecs_to_jiffies(gpio->debounce_time));

	return IRQ_HANDLED;
}

/* gpio work */
static void clv_gpio_work(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio;

	gpio = container_of(work, struct snd_soc_jack_gpio, work.work);
	clv_soc_jack_gpio_detect(gpio);
}

/*
	Add the INTn_CS42L73 -> GPIOxxx
	INTn_CS42L73 is the L73's interrupt and requires
	a different type of handling.
*/
int clv_soc_jack_add_gpio(struct snd_soc_jack *jack,
			    struct snd_soc_jack_gpio *gpio)
{
	int ret;
	struct snd_soc_codec *codec = jack->codec;

	if (!gpio_is_valid(gpio->gpio)) {
		pr_err("Invalid gpio %d\n", gpio->gpio);
		ret = -EINVAL;
		goto undo;
	}

	if (!gpio->name) {
		pr_err("No name for gpio %d\n", gpio->gpio);
		ret = -EINVAL;
		goto undo;
	}

	ret = gpio_request(gpio->gpio, gpio->name);
	if (ret)
		goto undo;

	ret = gpio_direction_input(gpio->gpio);
	if (ret)
		goto err;

	INIT_DELAYED_WORK(&gpio->work, clv_gpio_work);
	gpio->jack = jack;

	ret = request_irq(gpio_to_irq(gpio->gpio), clv_gpio_handler,
				 IRQF_TRIGGER_FALLING, codec->name, gpio);
	if (ret)
		goto err;

#ifdef CONFIG_GPIO_SYSFS
	/* Expose GPIO value over sysfs for diagnostic purposes */
	gpio_export(gpio->gpio, false);
#endif
	/* Update initial jack status */
	if (gpio->gpio == CS42L73_HPSENSE_GPIO)
		clv_soc_jack_gpio_detect(gpio);
	return 0;

err:
	gpio_free(gpio->gpio);
undo:
	snd_soc_jack_free_gpios(jack, 1, gpio);

return ret;
}

/* CDB42L73 widgets */
static const struct snd_soc_dapm_widget clv_dapm_widgets[] = {

	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

/* CDB42L73 Audio Map */
static const struct snd_soc_dapm_route clv_audio_map[] = {
	{"MIC1", NULL, "Headset Mic"},
	{"MIC2", NULL, "Headset Mic"},
	/* Headphone (L+R)->  HPOUTA, HPOUTB */
	{"Headphone", NULL, "HPOUTA"},
	{"Headphone", NULL, "HPOUTB"},
};


/* Board specific codec bias level control */
static int clv_set_bias_level(struct snd_soc_card *card,
				enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec = card->rtd->codec;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF)
			intel_sst_set_pll(true, SST_PLL_MSIC);
		break;
	case SND_SOC_BIAS_OFF:
		intel_sst_set_pll(false, SST_PLL_MSIC);
	}
	codec->dapm.bias_level = level;
	pr_debug("codec->bias_level %u\n", card->dapm.bias_level);

	return 0;
}


static int clv_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;
	struct snd_soc_card *card = runtime->card;

	/*Enable  IHFAMP_SD_N  GPIO */
	ret = intel_scu_ipc_iowrite8(0x70, 0x3d);
	if (ret)
		pr_err("write of  failed, err %d\n", ret);

	/* Set codec bias level */
	clv_set_bias_level(card, SND_SOC_BIAS_OFF);


	/* Add Jack specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, clv_dapm_widgets,
					ARRAY_SIZE(clv_dapm_widgets));
	if (ret)
		return ret;

	/* Set up Jack specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, clv_audio_map,
				ARRAY_SIZE(clv_audio_map));

	/*In VV board MIC1 is connected  and MIC2 is PR boards */
	if (ctp_board_id() == CTP_BID_VV)
		snd_soc_dapm_disable_pin(dapm, "MIC1");
	else
		snd_soc_dapm_disable_pin(dapm, "MIC2");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);
	/* Headset and button jack detection */
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0 |
			SND_JACK_BTN_1, &clv_jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}

	ret = clv_soc_jack_add_gpio(&clv_jack, &hs_jack_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}

	ret = clv_soc_jack_add_gpio(&clv_jack, &hs_button_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}

	return ret;
}

static struct snd_soc_ops clv_asp_ops = {
	.hw_params = clv_asp_hw_params,
};
static struct snd_soc_ops clv_vsp_ops = {
	.hw_params = clv_vsp_hw_params,
};
struct snd_soc_dai_link clv_msic_dailink[] = {
	{
		.name = "Cloverview ASP",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = clv_init,
		.ignore_suspend = 1,
		.ops = &clv_asp_ops,
	},
	{
		.name = "Cloverview VSP",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "cs42l73-vsp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &clv_vsp_ops,
	},
};

#ifdef CONFIG_PM

static int snd_clv_suspend(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}
static int snd_clv_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return 0;
}

static int snd_clv_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}

#else
#define snd_clv_suspend NULL
#define snd_clv_resume NULL
#define snd_clv_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_clv = {
	.name = "cloverview_audio",
	.dai_link = clv_msic_dailink,
	.num_links = ARRAY_SIZE(clv_msic_dailink),
	.set_bias_level = clv_set_bias_level,
	.controls = clv_controls,
	.num_controls = ARRAY_SIZE(clv_controls),
};

static int snd_clv_mc_probe(struct ipc_device *ipcdev)
{
	int ret_val = 0;
	struct clv_mc_private *mc_drv_ctx;

	pr_debug("In %s\n", __func__);
	mc_drv_ctx = kzalloc(sizeof(*mc_drv_ctx), GFP_ATOMIC);
	if (!mc_drv_ctx) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	/* register the soc card */
	snd_soc_card_clv.dev = &ipcdev->dev;
	snd_soc_initialize_card_lists(&snd_soc_card_clv);
	ret_val = snd_soc_register_card(&snd_soc_card_clv);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	vsp_mode = 1;
	ipc_set_drvdata(ipcdev, &snd_soc_card_clv);
	snd_soc_card_set_drvdata(&snd_soc_card_clv, mc_drv_ctx);
	pr_debug("successfully exited probe\n");
	return ret_val;

unalloc:
	kfree(mc_drv_ctx);
	return ret_val;
}

static int __devexit snd_clv_mc_remove(struct ipc_device *ipcdev)
{
	struct snd_soc_card *soc_card = ipc_get_drvdata(ipcdev);
	struct mfld_mc_private *mc_drv_ctx = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	kfree(mc_drv_ctx);
	snd_soc_jack_free_gpios(&clv_jack, 1, &hs_jack_gpio);
	snd_soc_jack_free_gpios(&clv_jack, 1, &hs_button_gpio);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	ipc_set_drvdata(ipcdev, NULL);
	return 0;
}
const struct dev_pm_ops snd_clv_mc_pm_ops = {
	.suspend = snd_clv_suspend,
	.resume = snd_clv_resume,
	.poweroff = snd_clv_poweroff,
};

static struct ipc_driver snd_clv_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "clvcs_audio",
		.pm   = &snd_clv_mc_pm_ops,
	},
	.probe = snd_clv_mc_probe,
	.remove = __devexit_p(snd_clv_mc_remove),
};

static int __init snd_clv_driver_init(void)
{
	pr_debug("In %s\n", __func__);
	return ipc_driver_register(&snd_clv_mc_driver);
}

module_init(snd_clv_driver_init);

static void __exit snd_clv_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	ipc_driver_unregister(&snd_clv_mc_driver);
}

module_exit(snd_clv_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Cloverview MID Machine driver");
MODULE_AUTHOR("Jeeja KP<jeeja.kp@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ipc:clvcs42l73-audio");
