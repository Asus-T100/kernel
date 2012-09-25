/*
 *  clv_machine.c - ASoc Machine driver for Intel Cloverview MID platform
 *
 *  Copyright (C) 2011-12 Intel Corp
 *  Author: KP Jeeja<jeeja.kp@intel.com>
 *  Author: Vaibhav Agarwal <vaibhav.agarwal@intel.com>
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
#include <linux/wakelock.h>
#include <linux/gpio.h>
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

	pr_debug("Slave Mode selected\n");
	/* CS42L73  Master Mode`*/
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	clk_source = SND_SOC_CLOCK_IN;

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
	/* Jack related */
	struct delayed_work jack_work;
	struct snd_soc_jack clv_jack;
	atomic_t bpirq_flag;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock *jack_wake_lock;
#endif
};


/* Headset jack detection gpios func(s) */
static int clv_soc_jack_gpio_detect(void);
static int clv_soc_jack_gpio_detect_bp(void);

#define HPDETECT_POLL_INTERVAL	msecs_to_jiffies(1000)	/* 1sec */


static struct snd_soc_jack_gpio hs_gpio[] = {
	{
		.gpio = CS42L73_HPSENSE_GPIO,
		.name = "cs-hsdet-gpio",
		.report = SND_JACK_HEADSET,
		.debounce_time = 100,
		.jack_status_check = clv_soc_jack_gpio_detect,
		.irq_flags = IRQF_TRIGGER_FALLING,
	}, {
		.gpio = CS42L73_BUTTON_GPIO,
		.name = "cs-hsbutton-gpio",
		.report = SND_JACK_HEADSET | SND_JACK_BTN_0,
		.debounce_time = 100,
		.jack_status_check = clv_soc_jack_gpio_detect_bp,
		.irq_flags = IRQF_TRIGGER_FALLING,
	},
};

static int set_mic_bias(struct snd_soc_jack *jack, int state)
{
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	mutex_lock(&codec->mutex);
	switch (state) {
	case MIC_BIAS_DISABLE:
		if (ctp_board_id() == CTP_BID_VV)
			snd_soc_dapm_disable_pin(dapm, "MIC1 Bias");
		else
			snd_soc_dapm_disable_pin(dapm, "MIC2 Bias");
		break;
	case MIC_BIAS_ENABLE:
		if (ctp_board_id() == CTP_BID_VV)
			snd_soc_dapm_force_enable_pin(dapm, "MIC1 Bias");
		else
			snd_soc_dapm_force_enable_pin(dapm, "MIC2 Bias");
		break;
	}
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);

	return 0;
}

int clv_soc_jack_gpio_detect(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[0];
	int enable, status;
	int irq;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct clv_mc_private *ctx =
		container_of(jack, struct clv_mc_private, clv_jack);

	/* Get Jack status */
	gpio = &hs_gpio[0];
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);
	pr_debug("Current jack status = 0x%x\n", jack->status);
	set_mic_bias(jack, MIC_BIAS_ENABLE);
	status = cs42l73_hp_detection(codec, jack, enable);
	if (!status) {
		set_mic_bias(jack, MIC_BIAS_DISABLE);
		/* Jack removed, Disable BP interrupts if not done already */
		if (!atomic_dec_return(&ctx->bpirq_flag)) {
			gpio = &hs_gpio[1];
			irq = gpio_to_irq(gpio->gpio);
			if (irq < 0) {
				pr_err("%d:Failed to map gpio_to_irq\n", irq);
				return status;
			}
			/* Disable Button_press interrupt if no Headset */
			pr_debug("Disable %d interrupt line\n", irq);
			disable_irq_nosync(irq);
		} else {
			atomic_inc(&ctx->bpirq_flag);
		}
	} else { /* If jack inserted, schedule delayed_wq */
		schedule_delayed_work(&ctx->jack_work, HPDETECT_POLL_INTERVAL);
#ifdef CONFIG_HAS_WAKELOCK
		/*
		 * Take wakelock for one second to give time for the detection
		 * to finish. Jack detection is happening rarely so this doesn't
		 * have big impact to power consumption.
		 */
		wake_lock_timeout(ctx->jack_wake_lock,
				HPDETECT_POLL_INTERVAL + msecs_to_jiffies(50));
#endif
	}
	return status;
}

/* Func to verify Jack status after HPDETECT_POLL_INTERVAL */
static void headset_status_verify(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[0];
	int enable, status, irq;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	unsigned int mask = SND_JACK_HEADSET;
	struct clv_mc_private *ctx =
		container_of(jack, struct clv_mc_private, clv_jack);

	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);
	pr_debug("Current jack status = 0x%x\n", jack->status);

	status = cs42l73_hp_detection(codec, jack, enable);
	gpio = &hs_gpio[1];
	irq = gpio_to_irq(gpio->gpio);
	if (irq < 0) {
		pr_err("%d:Failed to map gpio_to_irq\n", irq);
		return;
	}

	/* Enable Button_press interrupt if HS is inserted
	 * and interrupts are not already enabled
	 */
	if (status == SND_JACK_HEADSET) {
		if (atomic_inc_return(&ctx->bpirq_flag) == 1) {
			/* If BP intr not enabled */
			pr_debug("Enable %d interrupt line\n", irq);
			enable_irq(irq);
		} else {
			atomic_dec(&ctx->bpirq_flag);
		}
		/* else do nothing as interrupts are already enabled
		 * This case occurs during slow insertion when
		 * multiple plug-in events are reported
		 */
	} else {
		set_mic_bias(jack, MIC_BIAS_DISABLE);
		if (!atomic_dec_return(&ctx->bpirq_flag)) {
			/* Disable Button_press interrupt if no Headset */
			pr_debug("Disable %d interrupt line\n", irq);
			disable_irq_nosync(irq);
		} else {
			atomic_inc(&ctx->bpirq_flag);
		}

	}

	if (jack->status != status)
		snd_soc_jack_report(jack, status, mask);

	pr_debug("%s: status 0x%x\n", __func__, status);
}

int clv_soc_jack_gpio_detect_bp(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[1];
	int enable, hs_status, status, irq;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct clv_mc_private *ctx =
		container_of(jack, struct clv_mc_private, clv_jack);

	status = 0;
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s: gpio->%d=0x%x\n", __func__, gpio->gpio, enable);

	/* Check for headset status before processing interrupt */
	gpio = &hs_gpio[0];
	hs_status = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		hs_status = !hs_status;
	pr_debug("%s: gpio->%d=0x%x\n", __func__, gpio->gpio, hs_status);
	if (!hs_status) {/* HS present, process the interrupt */
		if (!enable)
			status = cs42l73_bp_detection(codec, jack, enable);
		else {
			status = jack->status;
			pr_debug("%s:Invalid BP interrupt\n", __func__);
		}
	} else {
		pr_debug("%s:Spurious BP interrupt : HS_status 0x%x\n",
				__func__, hs_status);
		/* Disbale BP interrupts, in case enabled */
		if (!atomic_dec_return(&ctx->bpirq_flag)) {
			set_mic_bias(jack, MIC_BIAS_DISABLE);
			gpio = &hs_gpio[1];
			irq = gpio_to_irq(gpio->gpio);
			if (irq < 0) {
				pr_err("%d:Failed to map gpio_to_irq\n", irq);
				return status;
			}

			/* Disable Button_press interrupt if no Headset */
			pr_debug("Disable %d interrupt line\n", irq);
			disable_irq_nosync(irq);
		} else {
			atomic_inc(&ctx->bpirq_flag);
		}

	}

	pr_debug("%s: status 0x%x\n", __func__, status);

	return status;
}

static int clv_amp_event(struct snd_soc_dapm_widget *w,
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
static const struct snd_soc_dapm_widget clv_dapm_widgets[] = {

	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", clv_amp_event),
};

/* CDB42L73 Audio Map */
static const struct snd_soc_dapm_route clv_audio_map[] = {
	{"MIC1", NULL, "Headset Mic"},
	{"MIC2", NULL, "Headset Mic"},
	/* Headphone (L+R)->  HPOUTA, HPOUTB */
	{"Headphone", NULL, "HPOUTA"},
	{"Headphone", NULL, "HPOUTB"},
	{"Ext Spk", NULL, "SPKLINEOUT"},
	{"Ext Spk", NULL, "SPKOUT"},
};

/* Board specific codec bias level control */
static int clv_set_bias_level(struct snd_soc_card *card,
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

static int clv_set_bias_level_post(struct snd_soc_card *card,
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


static int clv_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret, irq;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_jack_gpio *gpio = &hs_gpio[1];
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);

	/* Set codec bias level */
	clv_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	/* Add Jack specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, clv_dapm_widgets,
					ARRAY_SIZE(clv_dapm_widgets));
	if (ret)
		return ret;

	/* Set up Jack specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, clv_audio_map,
					ARRAY_SIZE(clv_audio_map));

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "EAROUT");
	snd_soc_dapm_ignore_suspend(dapm, "HPOUTA");
	snd_soc_dapm_ignore_suspend(dapm, "HPOUTB");
	snd_soc_dapm_ignore_suspend(dapm, "VSPOUT");

	/*In VV board SPKOUT is connected and SPKLINEOUT on PR board*/
	/*In VV board MIC1 is connected  and MIC2 is PR boards */
	if (ctp_board_id() == CTP_BID_VV) {
		snd_soc_dapm_disable_pin(dapm, "MIC1");
		snd_soc_dapm_disable_pin(dapm, "SPKOUT");
		snd_soc_dapm_ignore_suspend(dapm, "SPKLINEOUT");
	} else {
		snd_soc_dapm_disable_pin(dapm, "MIC2");
		snd_soc_dapm_disable_pin(dapm, "SPKLINEOUT");
		snd_soc_dapm_ignore_suspend(dapm, "SPKOUT");
	}
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);

	/* Setup the HPDET timer */
	INIT_DELAYED_WORK(&ctx->jack_work, headset_status_verify);

	/* Headset and button jack detection */
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0, &ctx->clv_jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}

	ret = snd_soc_jack_add_gpios(&ctx->clv_jack, 2, hs_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}
	irq = gpio_to_irq(gpio->gpio);
	if (irq < 0) {
		pr_err("%d:Failed to map gpio_to_irq\n", irq);
		return irq;
	}

	/* Disable Button_press interrupt if no Headset */
	pr_err("Disable %d interrupt line\n", irq);
	disable_irq_nosync(irq);
	atomic_set(&ctx->bpirq_flag, 0);

	return ret;
}

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count	= ARRAY_SIZE(rates_48000),
	.list	= rates_48000,
};

static int clv_startup(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   &constraints_48000);
	return 0;
}

static struct snd_soc_ops clv_asp_ops = {
	.startup = clv_startup,
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

	{
		.name = "Cloverview Comp ASP",
		.stream_name = "Compress-Audio",
		.cpu_dai_name = "Virtual-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &clv_asp_ops,
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
	.set_bias_level_post = clv_set_bias_level_post,
};

static int snd_clv_mc_probe(struct ipc_device *ipcdev)
{
	int ret_val = 0;
	struct clv_mc_private *ctx;

	pr_debug("In %s\n", __func__);
	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

#ifdef CONFIG_HAS_WAKELOCK
	ctx->jack_wake_lock =
		kzalloc(sizeof(*(ctx->jack_wake_lock)), GFP_ATOMIC);
	if (!ctx->jack_wake_lock) {
		pr_err("allocation failed for wake_lock\n");
		kfree(ctx);
		return -ENOMEM;
	}
	wake_lock_init(ctx->jack_wake_lock, WAKE_LOCK_SUSPEND,
			"jack_detect");
#endif

	/* register the soc card */
	snd_soc_card_clv.dev = &ipcdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_clv, ctx);
	ret_val = snd_soc_register_card(&snd_soc_card_clv);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	ipc_set_drvdata(ipcdev, &snd_soc_card_clv);

	pr_debug("successfully exited probe\n");
	return ret_val;

unalloc:
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(ctx->jack_wake_lock))
		wake_unlock(ctx->jack_wake_lock);
	wake_lock_destroy(ctx->jack_wake_lock);
	kfree(ctx->jack_wake_lock);
#endif
	kfree(ctx);
	return ret_val;
}

static int __devexit snd_clv_mc_remove(struct ipc_device *ipcdev)
{
	struct snd_soc_card *soc_card = ipc_get_drvdata(ipcdev);
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	cancel_delayed_work_sync(&ctx->jack_work);
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(ctx->jack_wake_lock))
		wake_unlock(ctx->jack_wake_lock);
	wake_lock_destroy(ctx->jack_wake_lock);
	kfree(ctx->jack_wake_lock);
#endif
	snd_soc_jack_free_gpios(&ctx->clv_jack, 2, hs_gpio);
	kfree(ctx);
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
	pr_info("In %s\n", __func__);
	return ipc_driver_register(&snd_clv_mc_driver);
}
late_initcall(snd_clv_driver_init);

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
