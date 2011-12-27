/*
 *  mfld_machine.c - ASoc Machine driver for Intel Medfield MID platform
 *
 *  Copyright (C) 2010 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Author: Harsha Priya <priya.harsha@intel.com>
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
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <asm/mrst.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/sn95031.h"

#define MID_MONO 1
#define MID_STEREO 2
#define MID_MAX_CAP 5
#define MFLD_JACK_INSERT 0x04
#define HEADSET_DET_PIN 77

enum soc_mic_bias_zones {
	MFLD_MV_START = 0,
	/* mic bias volutage range for Headphones*/
	MFLD_MV_HP = 400,
	/* mic bias volutage range for American Headset*/
	MFLD_MV_AM_HS = 650,
	/* mic bias volutage range for Headset*/
	MFLD_MV_HS = 2000,
	MFLD_MV_UNDEFINED,
};

static unsigned int	hs_switch;
static unsigned int	lo_dac;

struct mfld_mc_private {
	struct platform_device *socdev;
	void __iomem *int_base;
	struct snd_soc_codec *codec;
	u8 jack_interrupt_status;
	u8 oc_interrupt_status;
	spinlock_t lock; /* lock for interrupt status and jack debounce */
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock wake_lock;
#endif
};

static struct snd_soc_jack mfld_jack;

/* jack detection voltage zones */
static struct snd_soc_jack_zone mfld_zones[] = {
	{MFLD_MV_START, MFLD_MV_AM_HS, SND_JACK_HEADPHONE},
	{MFLD_MV_AM_HS, MFLD_MV_HS, SND_JACK_HEADSET},
};

/* sound card controls */
static const char *headset_switch_text[] = {"Earpiece", "Headset"};

static const char *lo_text[] = {"Vibra", "Headset", "IHF", "None"};

static const struct soc_enum headset_enum =
	SOC_ENUM_SINGLE_EXT(2, headset_switch_text);

static const struct soc_enum lo_enum =
	SOC_ENUM_SINGLE_EXT(4, lo_text);

static const char *sn95031_pcm1_mode_text[] = {"Slave", "Master"};

static const struct soc_enum SN95031_pcm1_mode_config_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sn95031_pcm1_mode_text),
							sn95031_pcm1_mode_text);

static int headset_get_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hs_switch;
	return 0;
}

static int headset_set_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (ucontrol->value.integer.value[0] == hs_switch)
		return 0;

	if (ucontrol->value.integer.value[0]) {
		pr_debug("hs_set HS path\n");
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
	} else {
		pr_debug("hs_set EP path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_enable_pin(&codec->dapm, "EPOUT");
	}
	snd_soc_dapm_sync(&codec->dapm);
	hs_switch = ucontrol->value.integer.value[0];

	return 0;
}

static void lo_enable_out_pins(struct snd_soc_codec *codec)
{
	snd_soc_dapm_enable_pin(&codec->dapm, "IHFOUTL");
	snd_soc_dapm_enable_pin(&codec->dapm, "IHFOUTR");
	snd_soc_dapm_enable_pin(&codec->dapm, "LINEOUTL");
	snd_soc_dapm_enable_pin(&codec->dapm, "LINEOUTR");
	snd_soc_dapm_enable_pin(&codec->dapm, "VIB1OUT");
	snd_soc_dapm_enable_pin(&codec->dapm, "VIB2OUT");
	if (hs_switch) {
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
	} else {
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_enable_pin(&codec->dapm, "EPOUT");
	}
}

static int lo_get_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = lo_dac;
	return 0;
}

static int lo_set_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (ucontrol->value.integer.value[0] == lo_dac)
		return 0;

	/* we dont want to work with last state of lineout so just enable all
	 * pins and then disable pins not required
	 */
	lo_enable_out_pins(codec);
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		pr_debug("set vibra path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "VIB1OUT");
		snd_soc_dapm_disable_pin(&codec->dapm, "VIB2OUT");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0);
		break;

	case 1:
		pr_debug("set hs  path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0x22);
		break;

	case 2:
		pr_debug("set spkr path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "IHFOUTL");
		snd_soc_dapm_disable_pin(&codec->dapm, "IHFOUTR");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0x44);
		break;

	case 3:
		pr_debug("set null path\n");
		snd_soc_dapm_disable_pin(&codec->dapm, "LINEOUTL");
		snd_soc_dapm_disable_pin(&codec->dapm, "LINEOUTR");
		snd_soc_update_bits(codec, SN95031_LOCTL, 0x66, 0x66);
		break;
	}
	snd_soc_dapm_sync(&codec->dapm);
	lo_dac = ucontrol->value.integer.value[0];
	return 0;
}
static int sn95031_get_pcm1_mode(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int mode;
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	mode = snd_soc_read(codec, SN95031_PCM1C3) >> 7;
	pr_debug("mode: %d\n", mode);
	ucontrol->value.integer.value[0] = mode;
	return 0;
}
static int sn95031_set_pcm1_mode(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 mode = ucontrol->value.integer.value[0];
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (mode) {
		pr_debug("can we set the master mode settings\n");
		snd_soc_update_bits(codec, SN95031_PCM1C3,
				BIT(1)|BIT(2)|BIT(3)|BIT(7), BIT(7)|BIT(1));
		snd_soc_update_bits(codec, SN95031_PCM1C1, BIT(0)|BIT(1),
								BIT(0)|BIT(1));
		snd_soc_update_bits(codec, SN95031_PCM1C2,
						BIT(0)|BIT(1)|BIT(2), 0);
	} else {
		pr_debug("setting the slave mode settings\n");
		snd_soc_update_bits(codec, SN95031_PCM1C3, BIT(7), 0);
		snd_soc_update_bits(codec, SN95031_PCM1C1, BIT(0)|BIT(1), 0);
		snd_soc_update_bits(codec, SN95031_PCM1C2, BIT(2), BIT(2));

	}
	return 0;
}

static const struct snd_kcontrol_new mfld_snd_controls[] = {
	SOC_ENUM_EXT("Playback Switch", headset_enum,
			headset_get_switch, headset_set_switch),
	SOC_ENUM_EXT("Lineout Mux", lo_enum,
			lo_get_switch, lo_set_switch),
	SOC_ENUM_EXT("PCM1 Mode", SN95031_pcm1_mode_config_enum,
			sn95031_get_pcm1_mode, sn95031_set_pcm1_mode),
};

static const struct snd_soc_dapm_widget mfld_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_soc_dapm_route mfld_map[] = {
	{"Headphones", NULL, "HPOUTR"},
	{"Headphones", NULL, "HPOUTL"},
	{"AMIC1", NULL, "Mic"},
};

static void mfld_jack_check(unsigned int intr_status)
{
	struct mfld_jack_data jack_data;

	jack_data.mfld_jack = &mfld_jack;
	jack_data.intr_id = intr_status;

	sn95031_jack_detection(&jack_data);
	/* TODO: add american headset detection post gpiolib support */
}

static unsigned int async_param;
static LIST_HEAD(mfld_jack_async_list);
static void mfld_jack_check_async(void *ptr, async_cookie_t cookie)
{
	mfld_jack_check(*(unsigned int *)ptr);
	return;
}

static int mfld_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret_val;

	/* Add jack sense widgets */
	snd_soc_dapm_new_controls(dapm, mfld_widgets, ARRAY_SIZE(mfld_widgets));

	/* Set up the map */
	snd_soc_dapm_add_routes(dapm, mfld_map, ARRAY_SIZE(mfld_map));

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "Headphones");
	snd_soc_dapm_enable_pin(dapm, "Mic");
	snd_soc_dapm_sync(dapm);

	ret_val = snd_soc_add_controls(codec, mfld_snd_controls,
				ARRAY_SIZE(mfld_snd_controls));
	if (ret_val) {
		pr_err("soc_add_controls failed %d", ret_val);
		return ret_val;
	}
	/* default is earpiece pin, userspace sets it explcitly */
	snd_soc_dapm_disable_pin(dapm, "Headphones");
	/* default is lineout NC, userspace sets it explcitly */
	snd_soc_dapm_disable_pin(dapm, "LINEOUTL");
	snd_soc_dapm_disable_pin(dapm, "LINEOUTR");
	lo_dac = 3;
	hs_switch = 0;
	/* we dont use linein in this so set to NC */
	snd_soc_dapm_disable_pin(dapm, "LINEINL");
	snd_soc_dapm_disable_pin(dapm, "LINEINR");
	snd_soc_dapm_disable_pin(dapm, "DMIC2");
	snd_soc_dapm_disable_pin(dapm, "DMIC3");
	snd_soc_dapm_disable_pin(dapm, "DMIC4");
	snd_soc_dapm_disable_pin(dapm, "DMIC6");
	snd_soc_dapm_disable_pin(dapm, "AMIC2");

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "PCM1_IN");
	snd_soc_dapm_ignore_suspend(dapm, "PCM1_Out");
	snd_soc_dapm_ignore_suspend(dapm, "EPOUT");
	snd_soc_dapm_ignore_suspend(dapm, "IHFOUTL");
	snd_soc_dapm_ignore_suspend(dapm, "IHFOUTR");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC5");
	snd_soc_dapm_ignore_suspend(dapm, "Headphones");
	snd_soc_dapm_ignore_suspend(dapm, "Mic");
	snd_soc_dapm_sync(dapm);
	/* Headset and button jack detection */
	ret_val = snd_soc_jack_new(codec, "Intel(R) MID Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0 |
			SND_JACK_BTN_1, &mfld_jack);
	if (ret_val) {
		pr_err("jack creation failed\n");
		return ret_val;
	}

	ret_val = snd_soc_jack_add_zones(&mfld_jack,
			ARRAY_SIZE(mfld_zones), mfld_zones);
	if (ret_val) {
		pr_err("adding jack zones failed\n");
		return ret_val;
	}

	/* we want to check if anything is inserted at boot,
	 * so send a fake event to codec and it will read adc
	 * to find if anything is there or not */
	async_param = MFLD_JACK_INSERT;
	async_schedule_domain(mfld_jack_check_async,
			&async_param, &mfld_jack_async_list);
	return ret_val;
}

static struct snd_soc_dai_link mfld_msic_dailink[] = {
	{
		.name = "Medfield Headset",
		.stream_name = "Headset",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "SN95031 Headset",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = mfld_init,
		.ignore_suspend = 1,
	},
	{
		.name = "Medfield Speaker",
		.stream_name = "Speaker",
		.cpu_dai_name = "Speaker-cpu-dai",
		.codec_dai_name = "SN95031 Speaker",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
	},
/*
 *	This configurtaion doesnt need Vibra as PCM device
 *	so comment this out.
 *	codec should have SPI controls
 */
/*	{
		.name = "Medfield Vibra",
		.stream_name = "Vibra1",
		.cpu_dai_name = "Vibra1-cpu-dai",
		.codec_dai_name = "SN95031 Vibra1",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
	},
	{
		.name = "Medfield Haptics",
		.stream_name = "Vibra2",
		.cpu_dai_name = "Vibra2-cpu-dai",
		.codec_dai_name = "SN95031 Vibra2",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
	},
*/	{
		.name = "Medfield Voice",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "SN95031 Voice",
		.codec_name = "sn95031",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
	},
};

#ifdef CONFIG_PM

static int snd_mfld_mc_suspend(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}
static int snd_mfld_mc_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return 0;
}

static int snd_mfld_mc_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}

#else
#define snd_mfld_mc_suspend NULL
#define snd_mfld_mc_resume NULL
#define snd_mfld_mc_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_mfld = {
	.name = "medfield_audio",
	.dai_link = mfld_msic_dailink,
	.num_links = ARRAY_SIZE(mfld_msic_dailink),
};

static irqreturn_t snd_mfld_jack_intr_handler(int irq, void *dev)
{
	struct mfld_mc_private *mc_private = (struct mfld_mc_private *) dev;
	u16 intr_status;

	memcpy_fromio(&intr_status, ((void *)(mc_private->int_base)),
			sizeof(u16));
	/* not overwrite status here */
	spin_lock(&mc_private->lock);
	/*To retrieve the jack_interrupt_status value (MSB)*/
	mc_private->jack_interrupt_status |= 0x0F & (intr_status >> 8);
	/*To retrieve the oc_interrupt_status value (LSB)*/
	mc_private->oc_interrupt_status |= 0x1F & intr_status;
	spin_unlock(&mc_private->lock);
#ifdef CONFIG_HAS_WAKELOCK
	/*
	 * We don't have any call back from the jack detection completed.
	 * Take wakelock for two seconds to give time for the detection
	 * to finish. Jack detection is happening rarely so this doesn't
	 * have big impact to power consumption.
	 */
	wake_lock_timeout(&mc_private->wake_lock, 2*HZ);
#endif
	return IRQ_WAKE_THREAD;
}

static irqreturn_t snd_mfld_codec_intr_detection(int irq, void *data)
{
	struct mfld_mc_private *mc_drv_ctx = (struct mfld_mc_private *) data;
	unsigned long flags;
	u8 jack_int_value = 0;

	if (mfld_jack.codec == NULL) {
		pr_debug("codec NULL returning..");
		spin_lock_irqsave(&mc_drv_ctx->lock, flags);
		mc_drv_ctx->jack_interrupt_status = 0;
		mc_drv_ctx->oc_interrupt_status = 0;
		spin_unlock_irqrestore(&mc_drv_ctx->lock, flags);
		goto ret;
	}
	spin_lock_irqsave(&mc_drv_ctx->lock, flags);
	if (!((mc_drv_ctx->jack_interrupt_status) ||
			(mc_drv_ctx->oc_interrupt_status))) {
		spin_unlock_irqrestore(&mc_drv_ctx->lock, flags);
		pr_err("OC and Jack Intr with status 0, return....\n");
		goto ret;
	}
	if (mc_drv_ctx->oc_interrupt_status) {
		pr_info("OC int value: %d\n", mc_drv_ctx->oc_interrupt_status);
		mc_drv_ctx->oc_interrupt_status = 0;
	}
	if (mc_drv_ctx->jack_interrupt_status) {
		jack_int_value = mc_drv_ctx->jack_interrupt_status;
		mc_drv_ctx->jack_interrupt_status = 0;
	}
	spin_unlock_irqrestore(&mc_drv_ctx->lock, flags);

	if (jack_int_value)
		mfld_jack_check(jack_int_value);

ret:
	return IRQ_HANDLED;
}

static int __devinit snd_mfld_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0, irq;
	struct mfld_mc_private *mc_drv_ctx;
	struct resource *irq_mem;

	pr_debug("snd_mfld_mc_probe called\n");

	/* retrive the irq number */
	irq = platform_get_irq(pdev, 0);

	/* audio interrupt base of SRAM location where
	 * interrupts are stored by System FW */
	mc_drv_ctx = kzalloc(sizeof(*mc_drv_ctx), GFP_ATOMIC);
	if (!mc_drv_ctx) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
	spin_lock_init(&mc_drv_ctx->lock);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&mc_drv_ctx->wake_lock,
		       WAKE_LOCK_SUSPEND, "jack_detect");
#endif

	irq_mem = platform_get_resource_byname(
				pdev, IORESOURCE_MEM, "IRQ_BASE");
	if (!irq_mem) {
		pr_err("no mem resource given\n");
		ret_val = -ENODEV;
		goto unalloc;
	}

	if (mfld_board_id() == MFLD_BID_PR3) {
		ret_val = gpio_request(HEADSET_DET_PIN, "headset_detect_pin");
		if (ret_val) {
			pr_err("HEADSET GPIO allocation failed: %d\n", ret_val);
			kfree(mc_drv_ctx);
			return ret_val;
		}
		ret_val = gpio_direction_input(HEADSET_DET_PIN);
		if (ret_val) {
			pr_err("HEADSET GPIO direction wrong: %d\n", ret_val);
			kfree(mc_drv_ctx);
			return ret_val;
		}
	}

	mc_drv_ctx->int_base = ioremap_nocache(irq_mem->start,
					resource_size(irq_mem));
	if (!mc_drv_ctx->int_base) {
		pr_err("Mapping of cache failed\n");
		ret_val = -ENOMEM;
		goto unalloc;
	}
	/* register for interrupt */
	ret_val = request_threaded_irq(irq, snd_mfld_jack_intr_handler,
			snd_mfld_codec_intr_detection,
			IRQF_SHARED | IRQF_NO_SUSPEND,
			pdev->dev.driver->name, mc_drv_ctx);
	if (ret_val) {
		pr_err("cannot register IRQ\n");
		goto unalloc;
	}
	/* register the soc card */
	snd_soc_card_mfld.dev = &pdev->dev;
	snd_soc_initialize_card_lists(&snd_soc_card_mfld);
	ret_val = snd_soc_register_card(&snd_soc_card_mfld);
	if (ret_val) {
		pr_debug("snd_soc_register_card failed %d\n", ret_val);
		goto freeirq;
	}
	platform_set_drvdata(pdev, &snd_soc_card_mfld);
	snd_soc_card_set_drvdata(&snd_soc_card_mfld, mc_drv_ctx);
	pr_debug("successfully exited probe\n");
	return ret_val;

freeirq:
	free_irq(irq, mc_drv_ctx);
unalloc:
	kfree(mc_drv_ctx);
	return ret_val;
}

static int __devexit snd_mfld_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mfld_mc_private *mc_drv_ctx = snd_soc_card_get_drvdata(soc_card);
	pr_debug("snd_mfld_mc_remove called\n");
	free_irq(platform_get_irq(pdev, 0), mc_drv_ctx);
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(&mc_drv_ctx->wake_lock))
		wake_unlock(&mc_drv_ctx->wake_lock);
	wake_lock_destroy(&mc_drv_ctx->wake_lock);
#endif
	kfree(mc_drv_ctx);
	if (mfld_board_id() == MFLD_BID_PR3)
		gpio_free(HEADSET_DET_PIN);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}
static const struct dev_pm_ops snd_mfld_mc_pm_ops = {
	.suspend = snd_mfld_mc_suspend,
	.resume = snd_mfld_mc_resume,
	.poweroff = snd_mfld_mc_poweroff,
};

static struct platform_driver snd_mfld_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "msic_audio",
		.pm   = &snd_mfld_mc_pm_ops,
	},
	.probe = snd_mfld_mc_probe,
	.remove = __devexit_p(snd_mfld_mc_remove),
};

static int __init snd_mfld_driver_init(void)
{
	pr_debug("snd_mfld_driver_init called\n");
	return platform_driver_register(&snd_mfld_mc_driver);
}
module_init_async(snd_mfld_driver_init);

static void __exit snd_mfld_driver_exit(void)
{
	pr_debug("snd_mfld_driver_exit called\n");
	async_synchronize_full_domain(&mfld_jack_async_list);
	platform_driver_unregister(&snd_mfld_mc_driver);
}
module_exit(snd_mfld_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) MID Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msic-audio");
