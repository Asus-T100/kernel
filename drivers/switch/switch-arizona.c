/*
 * extcon-arizona.c - Extcon driver Wolfson Arizona devices
 *
 *  Copyright (C) 2012 Wolfson Microelectronics plc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/switch.h>

#include <sound/soc.h>

#include <linux/mfd/arizona/core.h>
#include <linux/mfd/arizona/pdata.h>
#include <linux/mfd/arizona/registers.h>

#define ARIZONA_DEFAULT_HP 32

#define ARIZONA_MAX_MICD_RANGE 8

#define ARIZONA_ACCDET_MODE_MIC 0
#define ARIZONA_ACCDET_MODE_HPL 1
#define ARIZONA_ACCDET_MODE_HPR 2

#define ARIZONA_HPDET_MAX 10000

#define HPDET_DEBOUNCE 500
#define DEFAULT_MICD_TIMEOUT 2000

struct arizona_extcon_info {
	struct device *dev;
	struct arizona *arizona;
	struct mutex lock;
	struct regulator *micvdd;
	struct input_dev *input;

	u16 last_jackdet;

	int micd_mode;
	const struct arizona_micd_config *micd_modes;
	int micd_num_modes;

	const struct arizona_micd_range *micd_ranges;
	int num_micd_ranges;

	int micd_timeout;

	bool micd_reva;
	bool micd_clamp;
	unsigned int spk_clamp;

	struct delayed_work probe_work;
	struct delayed_work hpdet_work;
	struct delayed_work micd_detect_work;
	struct delayed_work micd_timeout_work;

	bool hpdet_active;
	bool hpdet_done;
	bool hpdet_retried;

	int num_hpdet_res;
	unsigned int hpdet_res[3];

	bool mic;
	bool detecting;
	int jack_flips;
	bool cable;

	int hpdet_ip;

	struct switch_dev sdev;
};

static const struct arizona_micd_config micd_default_modes[] = {
	{ ARIZONA_ACCDET_SRC, 1 << ARIZONA_MICD_BIAS_SRC_SHIFT, 0 },
	{ 0,                  2 << ARIZONA_MICD_BIAS_SRC_SHIFT, 1 },
};

static const struct arizona_micd_range micd_default_ranges[] = {
	{ .max =  11, .key = BTN_0 },
	{ .max =  28, .key = BTN_1 },
	{ .max =  54, .key = BTN_2 },
	{ .max = 100, .key = BTN_3 },
	{ .max = 186, .key = BTN_4 },
	{ .max = 430, .key = BTN_5 },
};

static const int arizona_micd_levels[] = {
	3, 6, 8, 11, 13, 16, 18, 21, 23, 26, 28, 31, 34, 36, 39, 41, 44, 46,
	49, 52, 54, 57, 60, 62, 65, 67, 70, 73, 75, 78, 81, 83, 89, 94, 100,
	105, 111, 116, 122, 127, 139, 150, 161, 173, 186, 196, 209, 220, 245,
	270, 295, 321, 348, 375, 402, 430, 489, 550, 614, 681, 752, 903, 1071,
	1257,
};

/* These values are copied from Android WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static void arizona_probe_work(struct work_struct *work);
static void arizona_start_hpdet_acc_id(struct arizona_extcon_info *info);

static void arizona_extcon_do_magic(struct arizona_extcon_info *info,
				    unsigned int magic)
{
	struct arizona *arizona = info->arizona;
	int ret;

	mutex_lock(&arizona->dapm->card->mutex);

	/* Keep the HP output stages disabled while doing the magic */
	if (magic) {
		ret = regmap_update_bits(arizona->regmap,
					 ARIZONA_OUTPUT_ENABLES_1,
					 ARIZONA_OUT1L_ENA |
					 ARIZONA_OUT1R_ENA, 0);
		if (ret != 0)
			dev_warn(arizona->dev,
				"Failed to disable headphone outputs: %d\n",
				 ret);
	}

	ret = regmap_update_bits(arizona->regmap, 0x225, 0x4000,
				 magic);
	if (ret != 0)
		dev_warn(arizona->dev, "Failed to do magic: %d\n",
				 ret);

	ret = regmap_update_bits(arizona->regmap, 0x226, 0x4000,
				 magic);
	if (ret != 0)
		dev_warn(arizona->dev, "Failed to do magic: %d\n",
			 ret);

	/* Restore the desired state while not doing the magic */
	if (!magic) {
		ret = regmap_update_bits(arizona->regmap,
					 ARIZONA_OUTPUT_ENABLES_1,
					 ARIZONA_OUT1L_ENA |
					 ARIZONA_OUT1R_ENA, arizona->hp_ena);
		if (ret != 0)
			dev_warn(arizona->dev,
				 "Failed to restore headphone outputs: %d\n",
				 ret);
	}

	mutex_unlock(&arizona->dapm->card->mutex);
}

static void arizona_extcon_set_mode(struct arizona_extcon_info *info, int mode)
{
	struct arizona *arizona = info->arizona;

	mode %= info->micd_num_modes;

	if (arizona->pdata.micd_pol_gpio > 0)
		gpio_set_value_cansleep(arizona->pdata.micd_pol_gpio,
					info->micd_modes[mode].gpio);
	regmap_update_bits(arizona->regmap, ARIZONA_MIC_DETECT_1,
			   ARIZONA_MICD_BIAS_SRC_MASK,
			   info->micd_modes[mode].bias);
	regmap_update_bits(arizona->regmap, ARIZONA_ACCESSORY_DETECT_MODE_1,
			   ARIZONA_ACCDET_SRC, info->micd_modes[mode].src);

	info->micd_mode = mode;

	dev_dbg(arizona->dev, "Set jack polarity to %d\n", mode);
}

static const char *arizona_extcon_get_micbias(struct arizona_extcon_info *info)
{
	switch (info->micd_modes[0].bias >> ARIZONA_MICD_BIAS_SRC_SHIFT) {
	case 1:
		return "MICBIAS1";
	case 2:
		return "MICBIAS2";
	case 3:
		return "MICBIAS3";
	default:
		return "MICVDD";
	}
}

static void arizona_extcon_pulse_micbias(struct arizona_extcon_info *info)
{
	struct arizona *arizona = info->arizona;
	const char *widget = arizona_extcon_get_micbias(info);
	struct snd_soc_dapm_context *dapm = arizona->dapm;
	int ret;

	mutex_lock(&dapm->card->mutex);

	ret = snd_soc_dapm_force_enable_pin(dapm, widget);
	if (ret != 0)
		dev_warn(arizona->dev, "Failed to enable %s: %d\n",
			 widget, ret);

	mutex_unlock(&dapm->card->mutex);

	snd_soc_dapm_sync(dapm);

	if (!arizona->pdata.micd_force_micbias) {
		mutex_lock(&dapm->card->mutex);

		ret = snd_soc_dapm_disable_pin(arizona->dapm, widget);
		if (ret != 0)
			dev_warn(arizona->dev, "Failed to disable %s: %d\n",
				 widget, ret);

		mutex_unlock(&dapm->card->mutex);

		snd_soc_dapm_sync(dapm);
	}
}

static void arizona_start_mic(struct arizona_extcon_info *info)
{
	struct arizona *arizona = info->arizona;
	bool change;
	int ret;

	/* Microphone detection can't use idle mode */
	pm_runtime_get(info->dev);

	ret = regulator_enable(info->micvdd);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to enable MICVDD: %d\n",
			ret);
	}

	if (info->micd_reva) {
		regmap_write(arizona->regmap, 0x80, 0x3);
		regmap_write(arizona->regmap, 0x294, 0);
		regmap_write(arizona->regmap, 0x80, 0x0);
	}

	regmap_update_bits(arizona->regmap,
			   ARIZONA_ACCESSORY_DETECT_MODE_1,
			   ARIZONA_ACCDET_MODE_MASK, ARIZONA_ACCDET_MODE_MIC);

	arizona_extcon_pulse_micbias(info);

	regmap_update_bits_check(arizona->regmap, ARIZONA_MIC_DETECT_1,
				 ARIZONA_MICD_ENA, ARIZONA_MICD_ENA,
				 &change);
	if (!change) {
		regulator_disable(info->micvdd);
		pm_runtime_put(info->dev);
	}
}

static void arizona_stop_mic(struct arizona_extcon_info *info)
{
	struct arizona *arizona = info->arizona;
	const char *widget = arizona_extcon_get_micbias(info);
	struct snd_soc_dapm_context *dapm = arizona->dapm;
	bool change;
	int ret;

	regmap_update_bits_check(arizona->regmap, ARIZONA_MIC_DETECT_1,
				 ARIZONA_MICD_ENA, 0,
				 &change);

	mutex_lock(&dapm->card->mutex);

	ret = snd_soc_dapm_disable_pin(dapm, widget);
	if (ret != 0)
		dev_warn(arizona->dev,
			 "Failed to disable %s: %d\n",
			 widget, ret);

	mutex_unlock(&dapm->card->mutex);

	snd_soc_dapm_sync(dapm);

	if (info->micd_reva) {
		regmap_write(arizona->regmap, 0x80, 0x3);
		regmap_write(arizona->regmap, 0x294, 2);
		regmap_write(arizona->regmap, 0x80, 0x0);
	}

	if (change) {
		regulator_disable(info->micvdd);
		pm_runtime_put(info->dev);
	}
}

static struct {
	unsigned int factor_a;
	unsigned int factor_b;
} arizona_hpdet_b_ranges[] = {
	{  5528,   362464 },
	{ 11084,  6186851 },
	{ 11065, 65460395 },
};

static struct {
	int min;
	int max;
} arizona_hpdet_c_ranges[] = {
	{ 0,       30 },
	{ 8,      100 },
	{ 100,   1000 },
	{ 1000, 10000 },
};

static int arizona_hpdet_read(struct arizona_extcon_info *info)
{
	struct arizona *arizona = info->arizona;
	unsigned int val, range;
	int ret;

	ret = regmap_read(arizona->regmap, ARIZONA_HEADPHONE_DETECT_2, &val);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to read HPDET status: %d\n",
			ret);
		return ret;
	}

	switch (info->hpdet_ip) {
	case 0:
		if (!(val & ARIZONA_HP_DONE)) {
			dev_err(arizona->dev, "HPDET did not complete: %x\n",
				val);
			return -EAGAIN;
		}

		val &= ARIZONA_HP_LVL_MASK;
		break;

	case 1:
		if (!(val & ARIZONA_HP_DONE_B)) {
			dev_err(arizona->dev, "HPDET did not complete: %x\n",
				val);
			return -EAGAIN;
		}

		ret = regmap_read(arizona->regmap, ARIZONA_HP_DACVAL, &val);
		if (ret != 0) {
			dev_err(arizona->dev, "Failed to read HP value: %d\n",
				ret);
			return -EAGAIN;
		}

		regmap_read(arizona->regmap, ARIZONA_HEADPHONE_DETECT_1,
			    &range);
		range = (range & ARIZONA_HP_IMPEDANCE_RANGE_MASK)
			   >> ARIZONA_HP_IMPEDANCE_RANGE_SHIFT;

		if (range < ARRAY_SIZE(arizona_hpdet_b_ranges) - 1 &&
		    (val < 100 || val > 0x3fb)) {
			range++;
			dev_dbg(arizona->dev,
				"Moving to HPDET range %d (%x)\n",
				range, val);
			regmap_update_bits(arizona->regmap,
					   ARIZONA_HEADPHONE_DETECT_1,
					   ARIZONA_HP_IMPEDANCE_RANGE_MASK,
					   range <<
					   ARIZONA_HP_IMPEDANCE_RANGE_SHIFT);
			return -EAGAIN;
		}

		/* If we go out of range report top of range */
		if (val < 100 || val > 0x3fb) {
			dev_dbg(arizona->dev, "Measurement out of range: %x\n",
				val);
			return ARIZONA_HPDET_MAX;
		}

		dev_dbg(arizona->dev, "HPDET read %d in range %d\n",
			val, range);

		val = arizona_hpdet_b_ranges[range].factor_b
			/ ((val * 100) -
			   arizona_hpdet_b_ranges[range].factor_a);
		break;

	default:
		dev_warn(arizona->dev, "Unknown HPDET IP revision %d\n",
			 info->hpdet_ip);
	case 2:
		if (!(val & ARIZONA_HP_DONE_B)) {
			dev_err(arizona->dev, "HPDET did not complete: %x\n",
				val);
			return -EAGAIN;
		}

		val &= ARIZONA_HP_LVL_B_MASK;

		regmap_read(arizona->regmap, ARIZONA_HEADPHONE_DETECT_1,
			    &range);
		range = (range & ARIZONA_HP_IMPEDANCE_RANGE_MASK)
			   >> ARIZONA_HP_IMPEDANCE_RANGE_SHIFT;

		/* Skip up or down a range? */
		if (range && (val < arizona_hpdet_c_ranges[range].min)) {
			range--;
			dev_dbg(arizona->dev, "Moving to HPDET range %d-%d\n",
				arizona_hpdet_c_ranges[range].min,
				arizona_hpdet_c_ranges[range].max);
			regmap_update_bits(arizona->regmap,
					   ARIZONA_HEADPHONE_DETECT_1,
					   ARIZONA_HP_IMPEDANCE_RANGE_MASK,
					   range <<
					   ARIZONA_HP_IMPEDANCE_RANGE_SHIFT);
			return -EAGAIN;
		}

		if (range < ARRAY_SIZE(arizona_hpdet_c_ranges) - 1 &&
		    (val >= arizona_hpdet_c_ranges[range].max)) {
			range++;
			dev_dbg(arizona->dev, "Moving to HPDET range %d-%d\n",
				arizona_hpdet_c_ranges[range].min,
				arizona_hpdet_c_ranges[range].max);
			regmap_update_bits(arizona->regmap,
					   ARIZONA_HEADPHONE_DETECT_1,
					   ARIZONA_HP_IMPEDANCE_RANGE_MASK,
					   range <<
					   ARIZONA_HP_IMPEDANCE_RANGE_SHIFT);
			return -EAGAIN;
		}
	}

	dev_dbg(arizona->dev, "HP impedance %d ohms\n", val);
	return val;
}

static int arizona_hpdet_do_id(struct arizona_extcon_info *info, int *reading,
			       bool *mic)
{
	struct arizona *arizona = info->arizona;
	int id_gpio = arizona->pdata.hpdet_id_gpio;

	/*
	 * If we're using HPDET for accessory identification we need
	 * to take multiple measurements, step through them in sequence.
	 */
	if (arizona->pdata.hpdet_acc_id) {
		info->hpdet_res[info->num_hpdet_res++] = *reading;

		/* Only check the mic directly if we didn't already ID it */
		if (id_gpio && info->num_hpdet_res == 1) {
			dev_dbg(arizona->dev, "Measuring mic\n");

			regmap_update_bits(arizona->regmap,
					   ARIZONA_ACCESSORY_DETECT_MODE_1,
					   ARIZONA_ACCDET_MODE_MASK |
					   ARIZONA_ACCDET_SRC,
					   ARIZONA_ACCDET_MODE_HPR |
					   info->micd_modes[0].src);

			gpio_set_value_cansleep(id_gpio, 1);

			regmap_update_bits(arizona->regmap,
					   ARIZONA_HEADPHONE_DETECT_1,
					   ARIZONA_HP_POLL, ARIZONA_HP_POLL);
			return -EAGAIN;
		}

		/* OK, got both.  Now, compare... */
		dev_dbg(arizona->dev, "HPDET measured %d %d\n",
			info->hpdet_res[0], info->hpdet_res[1]);

		/* Take the headphone impedance for the main report */
		*reading = info->hpdet_res[0];

		/* Sometimes we get false readings due to slow insert */
		if (*reading >= ARIZONA_HPDET_MAX && !info->hpdet_retried) {
			dev_dbg(arizona->dev, "Retrying high impedance\n");
			info->num_hpdet_res = 0;
			info->hpdet_retried = true;
			arizona_start_hpdet_acc_id(info);
			pm_runtime_put(info->dev);
			return -EAGAIN;
		}

		/*
		 * If we measure the mic as 
		 */
		if (!id_gpio || info->hpdet_res[1] > 50) {
			dev_dbg(arizona->dev, "Detected mic\n");
			*mic = true;
			info->detecting = true;
		} else {
			dev_dbg(arizona->dev, "Detected headphone\n");
		}

		/* Make sure everything is reset back to the real polarity */
		regmap_update_bits(arizona->regmap,
				   ARIZONA_ACCESSORY_DETECT_MODE_1,
				   ARIZONA_ACCDET_SRC,
				   info->micd_modes[0].src);
	}

	return 0;
}

static irqreturn_t arizona_hpdet_irq(int irq, void *data)
{
	struct arizona_extcon_info *info = data;
	struct arizona *arizona = info->arizona;
	int id_gpio = arizona->pdata.hpdet_id_gpio;
	int ret, reading;
	bool mic = false;

	mutex_lock(&info->lock);

	/* If we got a spurious IRQ for some reason then ignore it */
	if (!info->hpdet_active) {
		dev_warn(arizona->dev, "Spurious HPDET IRQ\n");
		mutex_unlock(&info->lock);
		return IRQ_NONE;
	}

	if (!info->cable) {
		dev_dbg(arizona->dev, "Ignoring HPDET for removed cable\n");

		/* Reset back to starting range */
		regmap_update_bits(arizona->regmap,
				   ARIZONA_HEADPHONE_DETECT_1,
				   ARIZONA_HP_IMPEDANCE_RANGE_MASK |
				   ARIZONA_HP_POLL, 0);

		goto done;
	}

	ret = arizona_hpdet_read(info);
	if (ret == -EAGAIN) {
		goto out;
	} else if (ret < 0) {
		goto done;
	}
	reading = ret;

	/* Reset back to starting range */
	regmap_update_bits(arizona->regmap,
			   ARIZONA_HEADPHONE_DETECT_1,
			   ARIZONA_HP_IMPEDANCE_RANGE_MASK | ARIZONA_HP_POLL,
			   0);

	ret = arizona_hpdet_do_id(info, &reading, &mic);
	if (ret == -EAGAIN) {
		goto out;
	} else if (ret < 0) {
		goto done;
	}

	if (arizona->pdata.hpdet_cb)
		arizona->pdata.hpdet_cb(reading);

done:
	arizona_extcon_do_magic(info, 0);

	if (id_gpio)
		gpio_set_value_cansleep(id_gpio, 0);

	/* Revert back to MICDET mode */
	regmap_update_bits(arizona->regmap,
			   ARIZONA_ACCESSORY_DETECT_MODE_1,
			   ARIZONA_ACCDET_MODE_MASK, ARIZONA_ACCDET_MODE_MIC);

	/* If we have a mic then reenable MICDET */
	if (mic || info->mic)
		arizona_start_mic(info);
	else
		switch_set_state(&info->sdev, BIT_HEADSET_NO_MIC);

	if (info->hpdet_active) {
		pm_runtime_put(info->dev);
		info->hpdet_active = false;
	}

	info->hpdet_done = true;

out:
	mutex_unlock(&info->lock);

	return IRQ_HANDLED;
}

static void arizona_identify_headphone(struct arizona_extcon_info *info)
{
	struct arizona *arizona = info->arizona;
	int ret;

	if (info->hpdet_done)
		goto err;

	dev_dbg(arizona->dev, "Starting HPDET\n");

	/* Make sure we keep the device enabled during the measurement */
	pm_runtime_get(info->dev);

	info->hpdet_active = true;

	if (info->mic)
		arizona_stop_mic(info);

	arizona_extcon_do_magic(info, 0x4000);

	ret = regmap_update_bits(arizona->regmap,
				 ARIZONA_ACCESSORY_DETECT_MODE_1,
				 ARIZONA_ACCDET_MODE_MASK,
				 ARIZONA_ACCDET_MODE_HPL);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to set HPDETL mode: %d\n", ret);
		goto err;
	}

	ret = regmap_update_bits(arizona->regmap, ARIZONA_HEADPHONE_DETECT_1,
				 ARIZONA_HP_POLL, ARIZONA_HP_POLL);
	if (ret != 0) {
		dev_err(arizona->dev, "Can't start HPDETL measurement: %d\n",
			ret);
		goto err;
	}

	return;

err:
	regmap_update_bits(arizona->regmap, ARIZONA_ACCESSORY_DETECT_MODE_1,
			   ARIZONA_ACCDET_MODE_MASK, ARIZONA_ACCDET_MODE_MIC);

	/* Just report headphone */
	if (info->mic) {
		switch_set_state(&info->sdev, BIT_HEADSET);
	} else {
		switch_set_state(&info->sdev, BIT_HEADSET_NO_MIC);
	}
}

static void arizona_start_hpdet_acc_id(struct arizona_extcon_info *info)
{
	struct arizona *arizona = info->arizona;
	int hp_reading = 32;
	bool mic;
	int ret;

	dev_dbg(arizona->dev, "Starting identification via HPDET\n");

	/* Make sure we keep the device enabled during the measurement */
	pm_runtime_get_sync(info->dev);

	info->hpdet_active = true;

	arizona_extcon_do_magic(info, 0x4000);

	ret = regmap_update_bits(arizona->regmap,
				 ARIZONA_ACCESSORY_DETECT_MODE_1,
				 ARIZONA_ACCDET_SRC | ARIZONA_ACCDET_MODE_MASK,
				 info->micd_modes[0].src |
				 ARIZONA_ACCDET_MODE_HPL);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to set HPDETL mode: %d\n", ret);
		goto err;
	}

	if (arizona->pdata.hpdet_acc_id_line) {
		ret = regmap_update_bits(arizona->regmap,
					 ARIZONA_HEADPHONE_DETECT_1,
					 ARIZONA_HP_POLL, ARIZONA_HP_POLL);
		if (ret != 0) {
			dev_err(arizona->dev,
				"Can't start HPDETL measurement: %d\n",
				ret);
			goto err;
		}
	} else {
		arizona_hpdet_do_id(info, &hp_reading, &mic);
	}

	return;

err:
	regmap_update_bits(arizona->regmap, ARIZONA_ACCESSORY_DETECT_MODE_1,
			   ARIZONA_ACCDET_MODE_MASK, ARIZONA_ACCDET_MODE_MIC);

	/* Just report headphone */
	if (info->mic)
		switch_set_state(&info->sdev, BIT_HEADSET);
	else
		switch_set_state(&info->sdev, BIT_HEADSET_NO_MIC);

	info->hpdet_active = false;
}

static void arizona_micd_timeout_work(struct work_struct *work)
{
	struct arizona_extcon_info *info = container_of(work,
							struct arizona_extcon_info,
							micd_timeout_work.work);

	mutex_lock(&info->lock);

	dev_dbg(info->arizona->dev, "MICD timed out, reporting HP\n");
	arizona_identify_headphone(info);

	info->detecting = false;

	arizona_stop_mic(info);

	mutex_unlock(&info->lock);
}

static void arizona_micd_detect(struct work_struct *work)
{
	struct arizona_extcon_info *info = container_of(work,
							struct arizona_extcon_info,
							micd_detect_work.work);
	struct arizona *arizona = info->arizona;
	unsigned int val = 0, lvl;
	int ret, i, key;

	cancel_delayed_work_sync(&info->micd_timeout_work);

	mutex_lock(&info->lock);

	for (i = 0; i < 10 && !(val & 0x7fc); i++) {
		ret = regmap_read(arizona->regmap, ARIZONA_MIC_DETECT_3, &val);
		if (ret != 0) {
			dev_err(arizona->dev, "Failed to read MICDET: %d\n", ret);
			mutex_unlock(&info->lock);
			return;
		}

		dev_dbg(arizona->dev, "MICDET: %x\n", val);

		if (!(val & ARIZONA_MICD_VALID)) {
			dev_warn(arizona->dev, "Microphone detection state invalid\n");
			mutex_unlock(&info->lock);
			return;
		}
	}

	if (i == 10 && !(val & 0x7fc)) {
		dev_err(arizona->dev, "Failed to get valid MICDET value\n");
		mutex_unlock(&info->lock);
		return;
	}

	/* Due to jack detect this should never happen */
	if (!(val & ARIZONA_MICD_STS)) {
		dev_warn(arizona->dev, "Detected open circuit\n");
		info->detecting = false;
		goto handled;
	}

	/* If we got a high impedence we should have a headset, report it. */
	if (info->detecting && (val & 0x400)) {
		info->mic = true;

		arizona_identify_headphone(info);
		switch_set_state(&info->sdev, BIT_HEADSET);

		info->detecting = false;

		goto handled;
	}

	/* If we detected a lower impedence during initial startup
	 * then we probably have the wrong polarity, flip it.  Don't
	 * do this for the lowest impedences to speed up detection of
	 * plain headphones.  If both polarities report a low
	 * impedence then give up and report headphones.
	 */
	if (info->detecting && (val & 0x3f8)) {
		if (info->jack_flips >= info->micd_num_modes * 10) {
			dev_dbg(arizona->dev, "Detected HP/line\n");
			arizona_identify_headphone(info);

			info->detecting = false;

			arizona_stop_mic(info);
		} else {
			info->micd_mode++;
			if (info->micd_mode == info->micd_num_modes)
				info->micd_mode = 0;
			arizona_extcon_set_mode(info, info->micd_mode);

			info->jack_flips++;
		}

		goto handled;
	}

	/*
	 * If we're still detecting and we detect a short then we've
	 * got a headphone.  Otherwise it's a button press.
	 */
	if (val & 0x3fc) {
		if (info->mic) {
			dev_dbg(arizona->dev, "Mic button detected\n");

			lvl = val & ARIZONA_MICD_LVL_MASK;
			lvl >>= ARIZONA_MICD_LVL_SHIFT;

			for (i = 0; i < info->num_micd_ranges; i++)
				input_report_key(info->input,
						 info->micd_ranges[i].key, 0);

			WARN_ON(!lvl);
			WARN_ON(ffs(lvl) - 1 >= info->num_micd_ranges);
			if (lvl && ffs(lvl) - 1 < info->num_micd_ranges) {
				key = info->micd_ranges[ffs(lvl) - 1].key;
				input_report_key(info->input, key, 1);
				input_sync(info->input);
			}

		} else if (info->detecting) {
			dev_dbg(arizona->dev, "Headphone detected\n");
			info->detecting = false;
			arizona_stop_mic(info);

			arizona_identify_headphone(info);
		} else {
			dev_warn(arizona->dev, "Button with no mic: %x\n",
				 val);
		}
	} else {
		dev_dbg(arizona->dev, "Mic button released\n");
		for (i = 0; i < info->num_micd_ranges; i++)
			input_report_key(info->input,
					 info->micd_ranges[i].key, 0);
		input_sync(info->input);
		arizona_extcon_pulse_micbias(info);
	}

handled:
	if (info->detecting)
		schedule_delayed_work(&info->micd_timeout_work,
				      msecs_to_jiffies(info->micd_timeout));

	pm_runtime_mark_last_busy(info->dev);
	mutex_unlock(&info->lock);
}

static irqreturn_t arizona_micdet(int irq, void *data)
{
	struct arizona_extcon_info *info = data;
	struct arizona *arizona = info->arizona;
	int debounce = arizona->pdata.micd_detect_debounce;

	cancel_delayed_work_sync(&info->micd_detect_work);
	cancel_delayed_work_sync(&info->micd_timeout_work);

	mutex_lock(&info->lock);
	if (!info->detecting)
		debounce = 0;
	mutex_unlock(&info->lock);

	if (debounce)
		schedule_delayed_work(&info->micd_detect_work,
				      msecs_to_jiffies(debounce));
	else
		arizona_micd_detect(&info->micd_detect_work.work);

	return IRQ_HANDLED;
}

static void arizona_hpdet_work(struct work_struct *work)
{
	struct arizona_extcon_info *info = container_of(work,
							struct arizona_extcon_info,
							hpdet_work.work);

	mutex_lock(&info->lock);
	arizona_start_hpdet_acc_id(info);
	mutex_unlock(&info->lock);
}

static irqreturn_t arizona_jackdet(int irq, void *data)
{
	struct arizona_extcon_info *info = data;
	struct arizona *arizona = info->arizona;
	unsigned int val, present, mask;
	bool cancelled_hp, cancelled_mic;
	int ret, i;

	cancelled_hp = cancel_delayed_work_sync(&info->hpdet_work);
	cancelled_mic = cancel_delayed_work_sync(&info->micd_timeout_work);

	pm_runtime_get_sync(info->dev);

	mutex_lock(&info->lock);

	if (arizona->pdata.jd_gpio5) {
		mask = ARIZONA_MICD_CLAMP_STS;
		present = 0;
	} else {
		mask = ARIZONA_JD1_STS;
		present = ARIZONA_JD1_STS;
	}

	ret = regmap_read(arizona->regmap, ARIZONA_AOD_IRQ_RAW_STATUS, &val);
	if (ret != 0) {
		dev_err(arizona->dev, "Failed to read jackdet status: %d\n",
			ret);
		mutex_unlock(&info->lock);
		pm_runtime_put(info->dev);
		return IRQ_NONE;
	}

	val &= mask;
	if (val == info->last_jackdet) {
		dev_dbg(arizona->dev, "Suppressing duplicate JACKDET\n");
		if (cancelled_hp)
			schedule_delayed_work(&info->hpdet_work,
					      msecs_to_jiffies(HPDET_DEBOUNCE));

		if (cancelled_mic)
			schedule_delayed_work(&info->micd_timeout_work,
					      msecs_to_jiffies(info->micd_timeout));

		goto out;
	}
	info->last_jackdet = val;

	if (info->last_jackdet == present) {
		dev_dbg(arizona->dev, "Detected jack\n");
		info->cable = true;

		if (info->spk_clamp)
			regmap_update_bits(arizona->regmap,
					   ARIZONA_OUTPUT_ENABLES_1,
					   info->spk_clamp, 0);

		if (!arizona->pdata.hpdet_acc_id) {
			info->detecting = true;
			info->mic = false;
			info->jack_flips = 0;

			arizona_start_mic(info);
		} else {
			schedule_delayed_work(&info->hpdet_work,
					      msecs_to_jiffies(HPDET_DEBOUNCE));
		}

		regmap_update_bits(arizona->regmap,
				   ARIZONA_JACK_DETECT_DEBOUNCE,
				   ARIZONA_MICD_CLAMP_DB | ARIZONA_JD1_DB, 0);
	} else {
		dev_dbg(arizona->dev, "Detected jack removal\n");

		info->cable = false;
		arizona_stop_mic(info);

		if (info->spk_clamp)
			regmap_update_bits(arizona->regmap,
					   ARIZONA_OUTPUT_ENABLES_1,
					   info->spk_clamp, info->spk_clamp);

		info->num_hpdet_res = 0;
		for (i = 0; i < ARRAY_SIZE(info->hpdet_res); i++)
			info->hpdet_res[i] = 0;
		info->mic = false;
		info->hpdet_done = false;
		info->hpdet_retried = false;

		for (i = 0; i < info->num_micd_ranges; i++)
			input_report_key(info->input,
					 info->micd_ranges[i].key, 0);
		input_sync(info->input);

		if (switch_get_state(&info->sdev) != BIT_NO_HEADSET) {
			switch_set_state(&info->sdev, BIT_NO_HEADSET);
		}

		regmap_update_bits(arizona->regmap,
				   ARIZONA_JACK_DETECT_DEBOUNCE,
				   ARIZONA_MICD_CLAMP_DB | ARIZONA_JD1_DB,
				   ARIZONA_MICD_CLAMP_DB | ARIZONA_JD1_DB);
	}

	/* Clear trig_sts to make sure DCVDD is not forced up */
	regmap_write(arizona->regmap, ARIZONA_AOD_WKUP_AND_TRIG,
		     ARIZONA_MICD_CLAMP_FALL_TRIG_STS |
		     ARIZONA_MICD_CLAMP_RISE_TRIG_STS |
		     ARIZONA_JD1_FALL_TRIG_STS |
		     ARIZONA_JD1_RISE_TRIG_STS);

out:
	mutex_unlock(&info->lock);

	pm_runtime_put(info->dev);

	return IRQ_HANDLED;
}

/* Map a level onto a slot in the register bank */
static void arizona_micd_set_level(struct arizona *arizona, int index,
				   unsigned int level)
{
	int reg;
	unsigned int mask;

	reg = ARIZONA_MIC_DETECT_LEVEL_4 - (index / 2);

	if (!(index % 2)) {
		mask = 0x3f00;
		level <<= 8;
	} else {
		mask = 0x3f;
	}

	/* Program the level itself */
	regmap_update_bits(arizona->regmap, reg, mask, level);
}

static int arizona_extcon_probe(struct platform_device *pdev)
{
	struct arizona *arizona = dev_get_drvdata(pdev->dev.parent);
	struct arizona_pdata *pdata;
	struct arizona_extcon_info *info;

	pdata = dev_get_platdata(arizona->dev);

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	info->micvdd = devm_regulator_get(arizona->dev, "MICVDD");
	if (IS_ERR(info->micvdd)) {
		dev_err(arizona->dev, "Failed to get MICVDD: %ld\n",
			PTR_ERR(info->micvdd));
		return PTR_ERR(info->micvdd);
	}

	mutex_init(&info->lock);
	info->arizona = arizona;
	info->dev = &pdev->dev;
	info->last_jackdet = ~(ARIZONA_MICD_CLAMP_STS | ARIZONA_JD1_STS);
	INIT_DELAYED_WORK(&info->probe_work, arizona_probe_work);
	INIT_DELAYED_WORK(&info->hpdet_work, arizona_hpdet_work);
	INIT_DELAYED_WORK(&info->micd_detect_work, arizona_micd_detect);
	INIT_DELAYED_WORK(&info->micd_timeout_work, arizona_micd_timeout_work);
	platform_set_drvdata(pdev, info);

	switch (arizona->type) {
	case WM5102:
		switch (arizona->rev) {
		case 0:
			info->micd_reva = true;
			break;
		default:
			info->micd_clamp = true;
			info->hpdet_ip = 1;
			break;
		}
		break;
	default:
		break;
	}

	schedule_delayed_work(&info->probe_work, 0);

	return 0;
}

static void arizona_probe_work(struct work_struct *work)
{
	struct arizona_extcon_info *info = container_of(work,
							struct arizona_extcon_info,
							probe_work.work);
	struct arizona *arizona = info->arizona;
	struct arizona_pdata *pdata = dev_get_platdata(arizona->dev);
	unsigned int val;
	int jack_irq_fall, jack_irq_rise;
	int ret, mode, i, j;

	/* Defer until the audio has come up */
	if (!info->arizona->dapm) {
		dev_info(info->dev, "Waiting for DAPM to initialise...\n");
		schedule_delayed_work(&info->probe_work,
				      msecs_to_jiffies(500));
		return;
	}

	info->sdev.name = "h2w";
	ret = switch_dev_register(&info->sdev);
	if (ret < 0) {
		dev_err(arizona->dev, "extcon_dev_register() failed: %d\n",
			ret);
		return;
	}

	if (pdata->num_micd_configs) {
		info->micd_modes = pdata->micd_configs;
		info->micd_num_modes = pdata->num_micd_configs;
	} else {
		info->micd_modes = micd_default_modes;
		info->micd_num_modes = ARRAY_SIZE(micd_default_modes);
	}

	if (arizona->pdata.micd_pol_gpio > 0) {
		if (info->micd_modes[0].gpio)
			mode = GPIOF_OUT_INIT_HIGH;
		else
			mode = GPIOF_OUT_INIT_LOW;

		ret = gpio_request_one(arizona->pdata.micd_pol_gpio,
				       mode, "MICD polarity");
		if (ret != 0) {
			dev_err(arizona->dev, "Failed to request GPIO%d: %d\n",
				arizona->pdata.micd_pol_gpio, ret);
			goto err_register;
		}
	}

	if (arizona->pdata.hpdet_id_gpio > 0) {
		ret = gpio_request_one(arizona->pdata.hpdet_id_gpio,
				       GPIOF_OUT_INIT_LOW, "HPDET");
		if (ret != 0) {
			dev_err(arizona->dev, "Failed to request GPIO%d: %d\n",
				arizona->pdata.hpdet_id_gpio, ret);
			goto err_register;
		}
	}

	info->input = input_allocate_device();
	if (!info->input) {
		dev_err(arizona->dev, "Can't allocate input dev\n");
		ret = -ENOMEM;
		goto err_register;
	}

	info->input->name = "Headset";
	info->input->phys = "arizona/extcon";
	info->input->dev.parent = info->dev;

	if (arizona->pdata.micd_bias_start_time)
		regmap_update_bits(arizona->regmap, ARIZONA_MIC_DETECT_1,
				   ARIZONA_MICD_BIAS_STARTTIME_MASK,
				   arizona->pdata.micd_bias_start_time
				   << ARIZONA_MICD_BIAS_STARTTIME_SHIFT);

	if (arizona->pdata.micd_rate)
		regmap_update_bits(arizona->regmap, ARIZONA_MIC_DETECT_1,
				   ARIZONA_MICD_RATE_MASK,
				   arizona->pdata.micd_rate
				   << ARIZONA_MICD_RATE_SHIFT);

	if (arizona->pdata.micd_dbtime)
		regmap_update_bits(arizona->regmap, ARIZONA_MIC_DETECT_1,
				   ARIZONA_MICD_DBTIME_MASK,
				   arizona->pdata.micd_dbtime
				   << ARIZONA_MICD_DBTIME_SHIFT);

	BUILD_BUG_ON(ARRAY_SIZE(arizona_micd_levels) != 0x40);

	if (arizona->pdata.num_micd_ranges) {
		info->micd_ranges = pdata->micd_ranges;
		info->num_micd_ranges = pdata->num_micd_ranges;
	} else {
		info->micd_ranges = micd_default_ranges;
		info->num_micd_ranges = ARRAY_SIZE(micd_default_ranges);
	}

	if (arizona->pdata.num_micd_ranges > ARIZONA_MAX_MICD_RANGE) {
		dev_err(arizona->dev, "Too many MICD ranges: %d\n",
			arizona->pdata.num_micd_ranges);
	}

	if (info->num_micd_ranges > 1) {
		for (i = 1; i < info->num_micd_ranges; i++) {
			if (info->micd_ranges[i - 1].max >
			    info->micd_ranges[i].max) {
				dev_err(arizona->dev,
					"MICD ranges must be sorted\n");
				ret = -EINVAL;
				goto err_input;
			}
		}
	}

	/* Disable all buttons by default */
	regmap_update_bits(arizona->regmap, ARIZONA_MIC_DETECT_2,
			   ARIZONA_MICD_LVL_SEL_MASK, 0x81);

	/* Set up all the buttons the user specified */
	for (i = 0; i < info->num_micd_ranges; i++) {
		for (j = 0; j < ARRAY_SIZE(arizona_micd_levels); j++)
			if (arizona_micd_levels[j] >= info->micd_ranges[i].max)
				break;

		if (j == ARRAY_SIZE(arizona_micd_levels)) {
			dev_err(arizona->dev, "Unsupported MICD level %d\n",
				info->micd_ranges[i].max);
			ret = -EINVAL;
			goto err_input;
		}

		dev_dbg(arizona->dev, "%d ohms for MICD threshold %d\n",
			arizona_micd_levels[j], i);

		arizona_micd_set_level(arizona, i, j);
		input_set_capability(info->input, EV_KEY,
				     info->micd_ranges[i].key);

		/* Enable reporting of that range */
		regmap_update_bits(arizona->regmap, ARIZONA_MIC_DETECT_2,
				   1 << i, 1 << i);
	}

	/* Set all the remaining keys to a maximum */
	for (; i < ARIZONA_MAX_MICD_RANGE; i++)
		arizona_micd_set_level(arizona, i, 0x3f);

	/*
	 * If we have a clamp use it, activating in conjunction with
	 * GPIO5 if that is connected for jack detect operation.
	 */
	if (info->micd_clamp) {
		if (arizona->pdata.jd_gpio5) {
			/* Put the GPIO into input mode with optional pull */
			val = 0xc101;
			if (arizona->pdata.jd_gpio5_nopull)
				val &= ~ARIZONA_GPN_PU;

			regmap_write(arizona->regmap, ARIZONA_GPIO5_CTRL,
				     val);

			regmap_update_bits(arizona->regmap,
					   ARIZONA_MICD_CLAMP_CONTROL,
					   ARIZONA_MICD_CLAMP_MODE_MASK, 9);
		} else {
			regmap_update_bits(arizona->regmap,
					   ARIZONA_MICD_CLAMP_CONTROL,
					   ARIZONA_MICD_CLAMP_MODE_MASK, 4);
		}

		regmap_update_bits(arizona->regmap,
				   ARIZONA_JACK_DETECT_DEBOUNCE,
				   ARIZONA_MICD_CLAMP_DB,
				   ARIZONA_MICD_CLAMP_DB);
	}

	if (arizona->pdata.micd_timeout)
		info->micd_timeout = arizona->pdata.micd_timeout;
	else
		info->micd_timeout = DEFAULT_MICD_TIMEOUT;

	switch (arizona->pdata.mic_spk_clamp) {
	case ARIZONA_MIC_CLAMP_SPKLN:
	case ARIZONA_MIC_CLAMP_SPKLP:
		info->spk_clamp = ARIZONA_OUT4L_ENA;
		break;
	case ARIZONA_MIC_CLAMP_SPKRN:
	case ARIZONA_MIC_CLAMP_SPKRP:
		info->spk_clamp = ARIZONA_OUT4R_ENA;
		break;
	default:
		break;
	}

	arizona_extcon_set_mode(info, 0);

	pm_runtime_enable(info->dev);
	pm_runtime_idle(info->dev);
	pm_runtime_get_sync(info->dev);

	if (arizona->pdata.jd_gpio5) {
		jack_irq_rise = ARIZONA_IRQ_MICD_CLAMP_RISE;
		jack_irq_fall = ARIZONA_IRQ_MICD_CLAMP_FALL;
	} else {
		jack_irq_rise = ARIZONA_IRQ_JD_RISE;
		jack_irq_fall = ARIZONA_IRQ_JD_FALL;
	}

	ret = arizona_request_irq(arizona, jack_irq_rise,
				  "JACKDET rise", arizona_jackdet, info);
	if (ret != 0) {
		dev_err(info->dev, "Failed to get JACKDET rise IRQ: %d\n",
			ret);
		goto err_input;
	}

	ret = arizona_set_irq_wake(arizona, jack_irq_rise, 1);
	if (ret != 0) {
		dev_err(info->dev, "Failed to set JD rise IRQ wake: %d\n",
			ret);
		goto err_rise;
	}

	ret = arizona_request_irq(arizona, jack_irq_fall,
				  "JACKDET fall", arizona_jackdet, info);
	if (ret != 0) {
		dev_err(info->dev, "Failed to get JD fall IRQ: %d\n", ret);
		goto err_rise_wake;
	}

	ret = arizona_set_irq_wake(arizona, jack_irq_fall, 1);
	if (ret != 0) {
		dev_err(info->dev, "Failed to set JD fall IRQ wake: %d\n",
			ret);
		goto err_fall;
	}

	ret = arizona_request_irq(arizona, ARIZONA_IRQ_MICDET,
				  "MICDET", arizona_micdet, info);
	if (ret != 0) {
		dev_err(info->dev, "Failed to get MICDET IRQ: %d\n", ret);
		goto err_fall_wake;
	}

	ret = arizona_request_irq(arizona, ARIZONA_IRQ_HPDET,
				  "HPDET", arizona_hpdet_irq, info);
	if (ret != 0) {
		dev_err(info->dev, "Failed to get HPDET IRQ: %d\n", ret);
		goto err_micdet;
	}

	arizona_clk32k_enable(arizona);
	regmap_update_bits(arizona->regmap, ARIZONA_JACK_DETECT_DEBOUNCE,
			   ARIZONA_JD1_DB, ARIZONA_JD1_DB);
	regmap_update_bits(arizona->regmap, ARIZONA_JACK_DETECT_ANALOGUE,
			   ARIZONA_JD1_ENA, ARIZONA_JD1_ENA);

	pm_runtime_put(info->dev);

	ret = input_register_device(info->input);
	if (ret) {
		dev_err(info->dev, "Can't register input device: %d\n", ret);
		goto err_hpdet;
	}

	return;

err_hpdet:
	arizona_free_irq(arizona, ARIZONA_IRQ_HPDET, info);
err_micdet:
	arizona_free_irq(arizona, ARIZONA_IRQ_MICDET, info);
err_fall_wake:
	arizona_set_irq_wake(arizona, jack_irq_fall, 0);
err_fall:
	arizona_free_irq(arizona, jack_irq_fall, info);
err_rise_wake:
	arizona_set_irq_wake(arizona, jack_irq_rise, 0);
err_rise:
	arizona_free_irq(arizona, jack_irq_rise, info);
err_input:
err_register:
	pm_runtime_disable(info->dev);
	switch_dev_unregister(&info->sdev);
}

static int arizona_extcon_remove(struct platform_device *pdev)
{
	struct arizona_extcon_info *info = platform_get_drvdata(pdev);
	struct arizona *arizona = info->arizona;
	int jack_irq_rise, jack_irq_fall;

	pm_runtime_disable(&pdev->dev);

	regmap_update_bits(arizona->regmap,
			   ARIZONA_MICD_CLAMP_CONTROL,
			   ARIZONA_MICD_CLAMP_MODE_MASK, 0);

	if (arizona->pdata.jd_gpio5) {
		jack_irq_rise = ARIZONA_IRQ_MICD_CLAMP_RISE;
		jack_irq_fall = ARIZONA_IRQ_MICD_CLAMP_FALL;
	} else {
		jack_irq_rise = ARIZONA_IRQ_JD_RISE;
		jack_irq_fall = ARIZONA_IRQ_JD_FALL;
	}

	arizona_set_irq_wake(arizona, jack_irq_rise, 0);
	arizona_set_irq_wake(arizona, jack_irq_fall, 0);
	arizona_free_irq(arizona, ARIZONA_IRQ_HPDET, info);
	arizona_free_irq(arizona, ARIZONA_IRQ_MICDET, info);
	arizona_free_irq(arizona, jack_irq_rise, info);
	arizona_free_irq(arizona, jack_irq_fall, info);
	cancel_delayed_work_sync(&info->hpdet_work);
	regmap_update_bits(arizona->regmap, ARIZONA_JACK_DETECT_ANALOGUE,
			   ARIZONA_JD1_ENA, 0);
	arizona_clk32k_disable(arizona);
	switch_dev_unregister(&info->sdev);

	return 0;
}

static struct platform_driver arizona_extcon_driver = {
	.driver		= {
		.name	= "arizona-extcon",
		.owner	= THIS_MODULE,
	},
	.probe		= arizona_extcon_probe,
	.remove		= arizona_extcon_remove,
};

static int __init arizona_extcon_gpio_init(void)
{
	return platform_driver_register(&arizona_extcon_driver);
}
late_initcall(arizona_extcon_gpio_init);

static void __exit arizona_extcon_gpio_exit(void)
{
	platform_driver_unregister(&arizona_extcon_driver);
}
module_exit(arizona_extcon_gpio_exit);

MODULE_DESCRIPTION("Arizona Extcon driver");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:extcon-arizona");
