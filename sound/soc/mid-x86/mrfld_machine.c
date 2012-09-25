/*
 *  mrfld_machine.c - ASoc Machine driver for Intel Merrfield MID platform
 *
 *  Copyright (C) 2011-12 Intel Corp
 *  Author: Dharageswari R <dharageswari.r@intel.com>
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
#include <linux/gpio.h>
#include <linux/ipc_device.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_scu_ipcutil.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/cs42l73.h"

/* CDB42L73 Y1 (6.144 MHz) )oscillator =  MCLK1 */
#define C42L73_DEFAULT_MCLK	19200000
#define CS42L73_HPSENSE_GPIO	34
#define CS42L73_BUTTON_GPIO	32
#define GPIO_AMP_ON 0x3d
#define GPIO_AMP_OFF 0x0
#define GPIOHVCTL 0x70

struct mrfld_mc_private {
	struct ipc_device *socdev;
	void __iomem *int_base;
	struct snd_soc_codec *codec;
};

static int mrfld_asp_hw_params(struct snd_pcm_substream *substream,
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

/* Headset jack */
struct snd_soc_jack mrfld_jack;

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

static void mrfld_soc_jack_gpio_detect_bp(struct snd_soc_jack_gpio *gpio)
{
	int enable;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;

	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s: gpio->%d=0x%x\n", __func__, gpio->gpio, enable);
	cs42l73_bp_detection(codec, jack, enable);
}

/* irq handler for button_press gpio pin */
static irqreturn_t mrfld_gpio_handler_bp(int irq, void *data)
{
	struct snd_soc_jack_gpio *gpio = data;

	schedule_delayed_work(&gpio->work,
				msecs_to_jiffies(gpio->debounce_time));

	return IRQ_HANDLED;
}

/* gpio work */
static void mrfld_gpio_work_bp(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio;

	gpio = container_of(work, struct snd_soc_jack_gpio, work.work);
	mrfld_soc_jack_gpio_detect_bp(gpio);
}

static void mrfld_soc_jack_gpio_detect(struct snd_soc_jack_gpio *gpio)
{
	int enable;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);
	cs42l73_hp_detection(codec, jack, enable);
}

/* irq handler for gpio pin */
static irqreturn_t mrfld_gpio_handler(int irq, void *data)
{
	struct snd_soc_jack_gpio *gpio = data;

	schedule_delayed_work(&gpio->work,
				msecs_to_jiffies(gpio->debounce_time));

	return IRQ_HANDLED;
}

/* gpio work */
static void mrfld_gpio_work(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio;

	gpio = container_of(work, struct snd_soc_jack_gpio, work.work);
	mrfld_soc_jack_gpio_detect(gpio);
}

/*
	Add the INTn_CS42L73 -> GPIOxxx
	INTn_CS42L73 is the L73's interrupt and requires
	a different type of handling.
*/
int mrfld_soc_jack_add_gpio(struct snd_soc_jack *jack,
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

	if (gpio->gpio == CS42L73_HPSENSE_GPIO) {

		INIT_DELAYED_WORK(&gpio->work, mrfld_gpio_work);
		gpio->jack = jack;

		ret = request_irq(gpio_to_irq(gpio->gpio), mrfld_gpio_handler,
				IRQF_TRIGGER_FALLING, codec->name, gpio);
	} else {
		INIT_DELAYED_WORK(&gpio->work, mrfld_gpio_work_bp);
		gpio->jack = jack;

		ret = request_irq(gpio_to_irq(gpio->gpio),
				mrfld_gpio_handler_bp, IRQF_TRIGGER_FALLING,
							codec->name, gpio);

	}
	if (ret)
		goto err;

#ifdef CONFIG_GPIO_SYSFS
	/* Expose GPIO value over sysfs for diagnostic purposes */
	gpio_export(gpio->gpio, false);
#endif
	/* Update initial jack status */
	if (gpio->gpio == CS42L73_HPSENSE_GPIO)
		mrfld_soc_jack_gpio_detect(gpio);

	return 0;

err:
	gpio_free(gpio->gpio);
undo:
	snd_soc_jack_free_gpios(jack, 1, gpio);

return ret;
}


static int mrfld_amp_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	int ret;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		/*Enable  IHFAMP_SD_N  GPIO */
		ret = intel_scu_ipc_iowrite8(GPIOHVCTL, GPIO_AMP_ON);
		if (ret)
			pr_err("write of  failed, err %d\n", ret);
	} else {
		/*Disable  IHFAMP_SD_N  GPIO */
		ret = intel_scu_ipc_iowrite8(GPIOHVCTL, GPIO_AMP_OFF);
		if (ret)
			pr_err("write of  failed, err %d\n", ret);
	}
	return 0;
}


/* CDB42L73 widgets */
static const struct snd_soc_dapm_widget mrfld_dapm_widgets[] = {

	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", mrfld_amp_event),
};

/* CDB42L73 Audio Map */
static const struct snd_soc_dapm_route mrfld_audio_map[] = {
	{"MIC1", NULL, "Headset Mic"},
	{"MIC2", NULL, "Headset Mic"},
	/* Headphone (LR)->  HPOUTA, HPOUTB */
	{"Headphone", NULL, "HPOUTA"},
	{"Headphone", NULL, "HPOUTB"},
	{"Ext Spk", NULL, "SPKLINEOUT"},
	{"Ext Spk", NULL, "SPKOUT"},
};


/* Board specific codec bias level control */
static int mrfld_set_bias_level(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		if (card->dapm.bias_level == SND_SOC_BIAS_OFF)
			intel_scu_ipc_set_osc_clk0(true, CLK0_MSIC);
		card->dapm.bias_level = level;
		break;
	case SND_SOC_BIAS_OFF:
		/* OSC clk will be turned OFF after processing
		 * codec->dapm.bias_level = SND_SOC_BIAS_OFF.
		 */
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);

	return 0;
}

static int mrfld_set_bias_level_post(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;
	/* we have only one codec in this machine */
	codec = list_entry(card->codec_dev_list.next, struct snd_soc_codec,
			card_list);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		/* Processing already done during set_bias_level()
		 * callback. No action required here.
		 */
		break;
	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level != SND_SOC_BIAS_OFF)
			break;
		intel_scu_ipc_set_osc_clk0(false, CLK0_MSIC);
		card->dapm.bias_level = level;
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("%s:card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);

	return 0;
}

static int mrfld_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;
	struct snd_soc_card *card = runtime->card;


	/* Set codec bias level */
	mrfld_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;
	/* Add Jack specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, mrfld_dapm_widgets,
					ARRAY_SIZE(mrfld_dapm_widgets));
	if (ret)
		return ret;

	/* Set up Jack specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, mrfld_audio_map,
					ARRAY_SIZE(mrfld_audio_map));
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);

	/* Headset and button jack detection */
#if 0
	/* No jack on CR4 */
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0,
			&mrfld_jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}

	ret = mrfld_soc_jack_add_gpio(&mrfld_jack, &hs_jack_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}

	ret = mrfld_soc_jack_add_gpio(&mrfld_jack, &hs_button_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}
#endif
	return ret;
}

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count	= ARRAY_SIZE(rates_48000),
	.list	= rates_48000,
};

static int mrfld_startup(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   &constraints_48000);
}

static struct snd_soc_ops mrfld_asp_ops = {
	.startup = mrfld_startup,
	.hw_params = mrfld_asp_hw_params,
};

struct snd_soc_dai_link mrfld_msic_dailink[] = {
	{
		.name = "Merrifield ASP",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = mrfld_init,
		.ignore_suspend = 1,
		.ops = &mrfld_asp_ops,
	},
};

#ifdef CONFIG_PM

static int snd_mrfld_suspend(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}
static int snd_mrfld_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return 0;
}

static int snd_mrfld_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}

#else
#define snd_mrfld_suspend NULL
#define snd_mrfld_resume NULL
#define snd_mrfld_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_mrfld = {
	.name = "merrifield_audio",
	.dai_link = mrfld_msic_dailink,
	.num_links = ARRAY_SIZE(mrfld_msic_dailink),
	.set_bias_level = mrfld_set_bias_level,
	.set_bias_level_post = mrfld_set_bias_level_post,
};

static int snd_mrfld_mc_probe(struct ipc_device *ipcdev)
{
	int ret_val = 0;
	struct mrfld_mc_private *drv;

	pr_debug("In %s\n", __func__);
	drv = kzalloc(sizeof(*drv), GFP_KERNEL);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	/* register the soc card */
	snd_soc_card_mrfld.dev = &ipcdev->dev;
	ret_val = snd_soc_register_card(&snd_soc_card_mrfld);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	ipc_set_drvdata(ipcdev, &snd_soc_card_mrfld);
	snd_soc_card_set_drvdata(&snd_soc_card_mrfld, drv);
	pr_debug("successfully exited probe\n");
	return ret_val;

unalloc:
	kfree(drv);
	return ret_val;
}

static int __devexit snd_mrfld_mc_remove(struct ipc_device *ipcdev)
{
	struct snd_soc_card *soc_card = ipc_get_drvdata(ipcdev);
	struct mfld_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	kfree(drv);
	snd_soc_jack_free_gpios(&mrfld_jack, 1, &hs_jack_gpio);
	snd_soc_jack_free_gpios(&mrfld_jack, 1, &hs_button_gpio);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	ipc_set_drvdata(ipcdev, NULL);
	return 0;
}
const struct dev_pm_ops snd_mrfld_mc_pm_ops = {
	.suspend = snd_mrfld_suspend,
	.resume = snd_mrfld_resume,
	.poweroff = snd_mrfld_poweroff,
};

static struct ipc_driver snd_mrfld_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mrfld_cs42l73",
		.pm   = &snd_mrfld_mc_pm_ops,
	},
	.probe = snd_mrfld_mc_probe,
	.remove = __devexit_p(snd_mrfld_mc_remove),
};

static int __init snd_mrfld_driver_init(void)
{
	pr_info("In %s\n", __func__);
	return ipc_driver_register(&snd_mrfld_mc_driver);
}
late_initcall(snd_mrfld_driver_init);

static void __exit snd_mrfld_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	ipc_driver_unregister(&snd_mrfld_mc_driver);
}

module_exit(snd_mrfld_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Merrifield MID Machine driver");
MODULE_AUTHOR("Dharageswari R<dharageswari.r@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ipc:mrfldcs42l73-audio");
