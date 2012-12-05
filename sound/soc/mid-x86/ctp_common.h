/*
 *  ctp_common.h - Common routines for the Clovertrail platform
 *  based on Intel Medfield MID platform
 *
 *  Copyright (C) 2012 Intel Corp
 *  Author: Dharageswari.R <dharageswari.r@intel.com>
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
#ifndef _CTP_COMMON_H
#define _CTP_COMMON_H

#include <linux/gpio.h>
#include <asm/intel-mid.h>

/* CDB42L73 Y1 (6.144 MHz) )oscillator =  MCLK1 */
#define DEFAULT_MCLK	19200000
#define HPSENSE_GPIO	34
#define BUTTON_GPIO	32
#define MIC_BIAS_DISABLE     1
#define MIC_BIAS_ENABLE	0
#define GPIO_AMP_ON 0x3d
#define GPIO_AMP_OFF 0x0
#define GPIOHVCTL 0x70

struct clv_mc_private {
	struct ipc_device *socdev;
	void __iomem *int_base;
	struct snd_soc_codec *codec;
	/* Jack related */
	struct delayed_work jack_work;
	struct snd_soc_jack clv_jack;
	struct snd_soc_jack_gpio hs_gpio_ops[2];
	atomic_t bpirq_flag;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock *jack_wake_lock;
#endif
};

int set_mic_bias(struct snd_soc_jack *jack, int state);
int __devexit snd_clv_mc_remove(struct platform_device *pdev);
int snd_clv_suspend(struct device *dev);
int snd_clv_resume(struct device *dev);
int snd_clv_poweroff(struct device *dev);
int clv_amp_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event);

extern bool ctp_vv_board(void);
#endif
