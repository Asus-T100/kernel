/*
 *  ctp_common.c - ASoc Machine driver for Intel Clovertrail MID platform
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author: KP Jeeja<jeeja.kp@intel.com>
 *  Author: Vaibhav Agarwal <vaibhav.agarwal@intel.com>
 *  Author: Dharageswari.R<dharageswari.r@intel.com>
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

#include <linux/delay.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_scu_ipc.h>
#include <linux/wakelock.h>
#include <asm/intel_scu_ipcutil.h>
#include <linux/ipc_device.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include "ctp_common.h"
#include <asm/platform_ctp_audio.h>

#define HPDETECT_POLL_INTERVAL	msecs_to_jiffies(1000)	/* 1sec */

struct snd_soc_card snd_soc_card_ctp = {
	.name = "cloverview_audio",
	.set_bias_level = ctp_set_bias_level,
	.set_bias_level_post = ctp_set_bias_level_post,
};

unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list = rates_8000_16000,
};

unsigned int rates_48000[] = {
	48000,
};

struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count	= ARRAY_SIZE(rates_48000),
	.list	= rates_48000,
};

unsigned int rates_16000[] = {
	16000,
};

struct snd_pcm_hw_constraint_list constraints_16000 = {
	.count	= ARRAY_SIZE(rates_16000),
	.list	= rates_16000,
};

static struct snd_soc_jack_gpio hs_gpio[] = {
	[CTP_HSDET_GPIO] = {
		.name = "cs-hsdet-gpio",
		.report = SND_JACK_HEADSET,
		.debounce_time = 100,
		.jack_status_check = ctp_soc_jack_gpio_detect,
		.irq_flags = IRQF_TRIGGER_FALLING,
	},
	[CTP_BTN_GPIO] = {
		.name = "cs-hsbutton-gpio",
		.report = SND_JACK_HEADSET | SND_JACK_BTN_0,
		.debounce_time = 100,
		.jack_status_check = ctp_soc_jack_gpio_detect_bp,
		.irq_flags = IRQF_TRIGGER_FALLING,
	},
};


int ctp_startup_asp(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
					SNDRV_PCM_HW_PARAM_RATE,
					&constraints_48000);
	return 0;
}
int ctp_startup_vsp(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_16000);
	return 0;
}
int ctp_startup_bt_xsp(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_8000_16000);
	return 0;
}
int get_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_bt_sco_master_mode;
	return 0;
}
EXPORT_SYMBOL_GPL(get_ssp_bt_sco_master_mode);

int set_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_bt_sco_master_mode)
		return 0;

	ctl->ssp_bt_sco_master_mode = ucontrol->value.integer.value[0];

	return 0;
}
EXPORT_SYMBOL_GPL(set_ssp_bt_sco_master_mode);

int get_ssp_voip_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_voip_master_mode;
	return 0;
}
EXPORT_SYMBOL_GPL(get_ssp_voip_master_mode);

int set_ssp_voip_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_voip_master_mode)
		return 0;

	ctl->ssp_voip_master_mode = ucontrol->value.integer.value[0];

	return 0;
}
EXPORT_SYMBOL_GPL(set_ssp_voip_master_mode);

int get_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_modem_master_mode;
	return 0;
}
EXPORT_SYMBOL_GPL(get_ssp_modem_master_mode);

int set_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_modem_master_mode)
		return 0;

	ctl->ssp_modem_master_mode = ucontrol->value.integer.value[0];

	return 0;
}
EXPORT_SYMBOL_GPL(set_ssp_modem_master_mode);

static int mc_driver_ops(struct ctp_mc_private *ctx,
			struct ctp_audio_platform_data *pdata)
{
	switch (pdata->spid->product_line_id) {
	case INTEL_CLVTP_PHONE_RHB_ENG:
		if (pdata->spid->hardware_id == CLVTP_PHONE_RHB_VBDV1) {
			ctx->ops = ctp_get_vb_ops();
			return 0;
		} else {
			ctx->ops = ctp_get_rhb_ops();
			return 0;
		}
	default:
		pr_err("No data for prod line id: %x",
				pdata->spid->product_line_id);
		return -EINVAL;

	};
}
/* Board specific codec bias level control */
int ctp_set_bias_level(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;

	/* Clock management is done only if there is an associated codec
	 * to dapm context and if this not the dummy codec
	 */
	if (dapm->codec) {
		codec = dapm->codec;
		if (!strcmp(codec->name, "snd-soc-dummy"))
			return 0;
	} else {
		pr_info("In %s dapm context has no associated codec or it is dummy codec.", __func__);
		return 0;
	}

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

int ctp_set_bias_level_post(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;

	/* Clock management is done only if there is an associated codec
	 * to dapm context and if this not the dummy codec
	 */
	if (dapm->codec) {
		codec = dapm->codec;
		if (!strcmp(codec->name, "snd-soc-dummy"))
			return 0;
	} else {
		pr_info("In %s dapm context has no associated codec or it is dummy codec.", __func__);
		return 0;
	}

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

static int set_mic_bias(struct snd_soc_jack *jack, int state)
{
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	switch (state) {
	case MIC_BIAS_DISABLE:
		snd_soc_dapm_disable_pin(dapm, "MIC2 Bias");
		break;
	case MIC_BIAS_ENABLE:
		snd_soc_dapm_force_enable_pin(dapm, "MIC2 Bias");
		break;
	}
	snd_soc_dapm_sync(&codec->dapm);
	return 0;
}

int ctp_amp_event(struct snd_soc_dapm_widget *w,
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


int ctp_soc_jack_gpio_detect(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_HSDET_GPIO];
	int enable, status;
	int irq;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct ctp_mc_private *ctx =
		container_of(jack, struct ctp_mc_private, ctp_jack);

	/* Get Jack status */
	gpio = &hs_gpio[CTP_HSDET_GPIO];
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);
	pr_debug("Current jack status = 0x%x\n", jack->status);
	set_mic_bias(jack, MIC_BIAS_ENABLE);
	status = ctx->ops->hp_detection(codec, jack, enable);
	if (!status) {
		set_mic_bias(jack, MIC_BIAS_DISABLE);
		/* Jack removed, Disable BP interrupts if not done already */
		if (!atomic_dec_return(&ctx->bpirq_flag)) {
			gpio = &hs_gpio[CTP_BTN_GPIO];
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
void headset_status_verify(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_HSDET_GPIO];
	int enable, status, irq;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	unsigned int mask = SND_JACK_HEADSET;
	struct ctp_mc_private *ctx =
		container_of(jack, struct ctp_mc_private, ctp_jack);

	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);
	pr_debug("Current jack status = 0x%x\n", jack->status);

	status = ctx->ops->hp_detection(codec, jack, enable);
	gpio = &hs_gpio[CTP_BTN_GPIO];
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

int ctp_soc_jack_gpio_detect_bp(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_BTN_GPIO];
	int enable, hs_status, status, irq;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct ctp_mc_private *ctx =
		container_of(jack, struct ctp_mc_private, ctp_jack);

	status = 0;
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s: gpio->%d=0x%x\n", __func__, gpio->gpio, enable);

	/* Check for headset status before processing interrupt */
	gpio = &hs_gpio[CTP_HSDET_GPIO];
	hs_status = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		hs_status = !hs_status;
	pr_debug("%s: gpio->%d=0x%x\n", __func__, gpio->gpio, hs_status);
	if (!hs_status) {/* HS present, process the interrupt */
		if (!enable)
			status = ctx->ops->bp_detection(codec, jack, enable);
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
			gpio = &hs_gpio[CTP_BTN_GPIO];
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


#ifdef CONFIG_PM

static int snd_ctp_suspend(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	return snd_soc_suspend(dev);
}
static int snd_ctp_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_resume(dev);
}
static int snd_ctp_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}

#else
#define snd_ctp_suspend NULL
#define snd_ctp_resume NULL
#define snd_ctp_poweroff NULL
#endif


static int __devexit snd_ctp_mc_remove(struct ipc_device *ipcdev)
{
	struct snd_soc_card *soc_card = ipc_get_drvdata(ipcdev);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	cancel_delayed_work_sync(&ctx->jack_work);
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(ctx->jack_wake_lock))
		wake_unlock(ctx->jack_wake_lock);
	wake_lock_destroy(ctx->jack_wake_lock);
#endif
	snd_soc_jack_free_gpios(&ctx->ctp_jack, 2, ctx->hs_gpio_ops);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	ipc_set_drvdata(ipcdev, NULL);
	return 0;
}


int snd_ctp_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret, irq;
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_BTN_GPIO];
	struct snd_soc_codec *codec = runtime->codec;

	ret = ctx->ops->ctp_init(runtime);
	if (ret) {
		pr_err("CTP init returned failure\n");
		return ret;
	}
	/* Setup the HPDET timer */
	INIT_DELAYED_WORK(&ctx->jack_work, headset_status_verify);

	/* Headset and button jack detection */
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0, &ctx->ctp_jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}
	ret = snd_soc_jack_add_gpios(&ctx->ctp_jack, 2, ctx->hs_gpio_ops);
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
/* SoC card */
static int snd_ctp_mc_probe(struct ipc_device *ipcdev)
{
	int ret_val = 0;
	struct ctp_mc_private *ctx;
	struct ctp_audio_platform_data *pdata = ipcdev->dev.platform_data;

	pr_debug("In %s\n", __func__);
	ctx = devm_kzalloc(&ipcdev->dev, sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
#ifdef CONFIG_HAS_WAKELOCK
	ctx->jack_wake_lock =
		devm_kzalloc(&ipcdev->dev, sizeof(*(ctx->jack_wake_lock)), GFP_ATOMIC);
	if (!ctx->jack_wake_lock) {
		pr_err("allocation failed for wake_lock\n");
		return -ENOMEM;
	}
	wake_lock_init(ctx->jack_wake_lock, WAKE_LOCK_SUSPEND,
			"jack_detect");
#endif
	/* register the soc card */
	snd_soc_card_ctp.dev = &ipcdev->dev;

	if (0 != mc_driver_ops(ctx, pdata)) {
		ret_val = -EINVAL;
		goto unalloc;
	}
	ctx->ops->dai_link(&snd_soc_card_ctp);
	if (pdata->codec_gpio_hsdet >= 0 && pdata->codec_gpio_button >= 0) {
		hs_gpio[CTP_HSDET_GPIO].gpio = pdata->codec_gpio_hsdet;
		hs_gpio[CTP_BTN_GPIO].gpio = pdata->codec_gpio_button;
	}
	ctx->hs_gpio_ops = hs_gpio;
	snd_soc_card_set_drvdata(&snd_soc_card_ctp, ctx);
	ret_val = snd_soc_register_card(&snd_soc_card_ctp);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}

	ipc_set_drvdata(ipcdev, &snd_soc_card_ctp);
	pr_debug("successfully exited probe\n");
	return ret_val;

unalloc:
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(ctx->jack_wake_lock))
		wake_unlock(ctx->jack_wake_lock);
	wake_lock_destroy(ctx->jack_wake_lock);
#endif
	return ret_val;
}
const struct dev_pm_ops snd_ctp_mc_pm_ops = {
	.suspend = snd_ctp_suspend,
	.resume = snd_ctp_resume,
	.poweroff = snd_ctp_poweroff,
};

static struct ipc_driver snd_ctp_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ctp_audio",
		.pm   = &snd_ctp_mc_pm_ops,
	},
	.probe = snd_ctp_mc_probe,
	.remove = __devexit_p(snd_ctp_mc_remove),
};
static int __init snd_ctp_driver_init(void)
{
	pr_info("In %s\n", __func__);
	return ipc_driver_register(&snd_ctp_mc_driver);
}
module_init(snd_ctp_driver_init);

static void __exit snd_ctp_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	ipc_driver_unregister(&snd_ctp_mc_driver);
}
module_exit(snd_ctp_driver_exit);
