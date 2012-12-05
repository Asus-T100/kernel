/*
 *  ctp_common.c - ASoc Machine driver for Intel Clovertrail MID platform
 *
 *  Copyright (C) 2011-12 Intel Corp
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
#include <linux/platform_device.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_scu_pmic.h>
#include <linux/wakelock.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include "ctp_common.h"

#define HPDETECT_POLL_INTERVAL	msecs_to_jiffies(1000)	/* 1sec */

bool ctp_vv_board(void)
{
	return  INTEL_MID_BOARD(1, PHONE, CLVTP) &&
		(SPID_HARDWARE_ID(CLVTP, PHONE, RHB, CCVV0) ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, CCVV1) ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, CCVV2) ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, CCVV3) ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, CCVV1P));
}

int set_mic_bias(struct snd_soc_jack *jack, int state)
{
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	mutex_lock(&codec->mutex);
	switch (state) {
	case MIC_BIAS_DISABLE:
		if (ctp_vv_board())
			snd_soc_dapm_disable_pin(dapm, "MIC1 Bias");
		else
			snd_soc_dapm_disable_pin(dapm, "MIC2 Bias");
		break;
	case MIC_BIAS_ENABLE:
		if (ctp_vv_board())
			snd_soc_dapm_force_enable_pin(dapm, "MIC1 Bias");
		else
			snd_soc_dapm_force_enable_pin(dapm, "MIC2 Bias");
		break;
	}
	snd_soc_dapm_sync(&codec->dapm);
	mutex_unlock(&codec->mutex);

	return 0;
}

int clv_amp_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
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


#ifdef CONFIG_PM

int snd_clv_suspend(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}
int snd_clv_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return 0;
}

int snd_clv_poweroff(struct device *dev)
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


int __devexit snd_clv_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct clv_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	cancel_delayed_work_sync(&ctx->jack_work);
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(ctx->jack_wake_lock))
		wake_unlock(ctx->jack_wake_lock);
	wake_lock_destroy(ctx->jack_wake_lock);
	kfree(ctx->jack_wake_lock);
#endif
	snd_soc_jack_free_gpios(&ctx->clv_jack, 2, ctx->hs_gpio_ops);
	kfree(ctx);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}
