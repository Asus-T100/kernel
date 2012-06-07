/*
 * intel_mdf_msic_gpadc.c - Intel Medfield MSIC GPADC Driver
 *
 * Copyright (C) 2010 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Jenny TC <jenny.tc@intel.com>
 * Author: Bin Yang <bin.yang@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/pm_qos_params.h>
#include <linux/intel_mid_pm.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_gpadc.h>

#define VAUDACNT		0x0DB
#define MCCINT			0x013
#define IRQLVL1			0x002
#define IRQLVL1MSK		0x021
#define ADC1INT			0x003
#define ADC1ADDR0		0x1C5
#define ADC1SNS0H		0x1D4
#define ADC1OFFSETH		0x1C3
#define ADC1OFFSETL		0x1C4
#define ADC1CNTL1		0x1C0
#define ADC1CNTL2		0x1C1
#define ADC1CNTL3		0x1C2
#define	ADC1BV0H		0x1F2
#define ADC1BI0H		0x1FA

#define EEPROMCAL1		0x317
#define EEPROMCAL2		0x318

#define MCCINT_MCCCAL		(1 << 1)
#define MCCINT_MOVERFLOW	(1 << 0)

#define IRQLVL1MSK_ADCM		(1 << 1)

#define ADC1CNTL1_AD1OFFSETEN	(1 << 6)
#define ADC1CNTL1_AD1CALEN	(1 << 5)
#define ADC1CNTL1_ADEN		(1 << 4)
#define ADC1CNTL1_ADSTRT	(1 << 3)
#define ADC1CNTL1_ADSLP		7
#define ADC1CNTL1_ADSLP_DEF	1

#define ADC1INT_ADC1CAL		(1 << 2)
#define ADC1INT_GSM		(1 << 1)
#define ADC1INT_RND		(1 << 0)

#define ADC1CNTL3_ADCTHERM	(1 << 2)
#define ADC1CNTL3_GSMDATARD	(1 << 1)
#define ADC1CNTL3_RRDATARD	(1 << 0)

#define ADC1CNTL2_DEF		0x7
#define ADC1CNTL2_ADCGSMEN	(1 << 7)

#define MSIC_STOPCH		(1 << 4)

#define GPADC_CH_MAX		15

#define GPADC_POWERON_DELAY	1

#define SAMPLE_CH_MAX		2

static void *adc_handle[GPADC_CH_MAX] = {};
static int sample_result[GPADC_CH_MAX][SAMPLE_CH_MAX];
static struct completion gsmadc_complete;
static int vol_val;
static int cur_val;

struct gpadc_info {
	int initialized;
	int depth;

	struct workqueue_struct *workq;
	wait_queue_head_t trimming_wait;
	struct work_struct trimming_work;
	struct work_struct gsmpulse_work;
	int trimming_start;

	/* This mutex protects gpadc sample/config from concurrent conflict.
	   Any function, which does the sample or config, needs to
	   hold this lock.
	   If it is locked, it also means the gpadc is in active mode.
	   GSM mode sample does not need to hold this lock. It can be used with
	   normal sample concurrent without poweron.
	*/
	struct mutex lock;
	struct device *dev;
	int irq;
	void __iomem *intr;
	int irq_status;

	int vzse;
	int vge;
	int izse;
	int ige;
	int addr_mask;

	wait_queue_head_t wait;
	int rnd_done;
	int conv_done;
	int gsmpulse_done;

	struct pm_qos_request_list pm_qos_request;
	void (*gsmadc_notify)(int vol, int cur);
};

struct gpadc_request {
	int count;
	int vref;
	int ch[GPADC_CH_MAX];
	int addr[GPADC_CH_MAX];
};

static struct gpadc_info gpadc_info;

static inline int gpadc_clear_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0, mask);
}

static inline int gpadc_set_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0xff, mask);
}

static inline int gpadc_write(u16 addr, u8 data)
{
	return intel_scu_ipc_iowrite8(addr, data);
}

static inline int gpadc_read(u16 addr, u8 *data)
{
	return intel_scu_ipc_ioread8(addr, data);
}

static void gpadc_dump(struct gpadc_info *mgi)
{
	u8 data;
	int i;

	gpadc_read(VAUDACNT, &data);
	dev_err(mgi->dev, "VAUDACNT: 0x%x\n", data);
	gpadc_read(IRQLVL1MSK, &data);
	dev_err(mgi->dev, "IRQLVL1MSK: 0x%x\n", data);
	gpadc_read(IRQLVL1, &data);
	dev_err(mgi->dev, "IRQLVL1: 0x%x\n", data);
	gpadc_read(ADC1INT, &data);
	dev_err(mgi->dev, "ADC1INT: 0x%x\n", data);
	gpadc_read(ADC1CNTL1, &data);
	dev_err(mgi->dev, "ADC1CNTL1: 0x%x\n", data);
	gpadc_read(ADC1CNTL2, &data);
	dev_err(mgi->dev, "ADC1CNTL2: 0x%x\n", data);
	gpadc_read(ADC1CNTL3, &data);
	dev_err(mgi->dev, "ADC1CNTL3: 0x%x\n", data);
	for (i = 0; i < GPADC_CH_MAX; i++) {
		gpadc_read(ADC1ADDR0+i, &data);
		dev_err(mgi->dev, "ADC1ADDR[%d]: 0x%x\n", i, data);
	}
}

static int gpadc_poweron(struct gpadc_info *mgi, int vref)
{
	if (!mgi->depth++) {
		if (gpadc_set_bits(ADC1CNTL1, ADC1CNTL1_ADEN) != 0)
			return -EIO;
		msleep(GPADC_POWERON_DELAY);
	}
	if (vref) {
		if (gpadc_set_bits(ADC1CNTL3, ADC1CNTL3_ADCTHERM) != 0)
			return -EIO;
		msleep(GPADC_POWERON_DELAY);
	}
	return 0;
}

static int gpadc_poweroff(struct gpadc_info *mgi)
{
	if (!--mgi->depth) {
		if (gpadc_clear_bits(ADC1CNTL1, ADC1CNTL1_ADEN) != 0)
			return -EIO;
		if (gpadc_clear_bits(ADC1CNTL3, ADC1CNTL3_ADCTHERM) != 0)
			return -EIO;
	}
	return 0;
}

static void gpadc_trimming(struct work_struct *work)
{
	u8 data;
	int fse, zse, fse_sign, zse_sign;
	struct gpadc_info *mgi =
		container_of(work, struct gpadc_info, trimming_work);

	mutex_lock(&mgi->lock);
	mgi->trimming_start = 1;
	wake_up(&mgi->trimming_wait);
	if (gpadc_poweron(mgi, 1)) {
		dev_err(mgi->dev, "power on failed\n");
		goto failed;
	}
	/* calibration */
	gpadc_read(ADC1CNTL1, &data);
	data &= ~ADC1CNTL1_AD1OFFSETEN;
	data |= ADC1CNTL1_AD1CALEN;
	gpadc_write(ADC1CNTL1, data);
	gpadc_read(ADC1INT, &data);

	/*workarround: no calib int */
	msleep(300);
	gpadc_set_bits(ADC1INT, ADC1INT_ADC1CAL);
	gpadc_clear_bits(ADC1CNTL1, ADC1CNTL1_AD1CALEN);

	/* voltage trim */
	gpadc_read(EEPROMCAL1, &data);
	zse = (data & 0xf)/2;
	fse = ((data >> 4) & 0xf)/2;
	gpadc_read(EEPROMCAL2, &data);
	zse_sign = (data & (1 << 6)) ? 1 : 0;
	fse_sign = (data & (1 << 7)) ? 1 : 0;
	zse *= zse_sign;
	fse *= fse_sign;
	mgi->vzse = zse;
	mgi->vge = fse - zse;

	/* current trim */
	fse = (data & 0xf)/2;
	fse_sign = (data & (1 << 5)) ? 1 : 0;
	fse = ~(fse_sign * fse) + 1;
	gpadc_read(ADC1OFFSETH, &data);
	zse = data << 2;
	gpadc_read(ADC1OFFSETL, &data);
	zse += data & 0x3;
	mgi->izse = zse;
	mgi->ige = fse + zse;
	if (gpadc_poweroff(mgi)) {
		dev_err(mgi->dev, "power off failed\n");
		goto failed;
	}

failed:
	mutex_unlock(&mgi->lock);
}

static irqreturn_t msic_gpadc_isr(int irq, void *data)
{
	struct gpadc_info *mgi = data;

	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW)
		mgi->irq_status = ADC1INT_RND;
	else
		mgi->irq_status = readl(mgi->intr) >> 8 & 0xff;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t msic_gpadc_irq(int irq, void *data)
{
	struct gpadc_info *mgi = data;

	if (mgi->irq_status & ADC1INT_GSM) {
		mgi->gsmpulse_done = 1;
		queue_work(mgi->workq, &mgi->gsmpulse_work);
	} else if (mgi->irq_status & ADC1INT_RND) {
		mgi->rnd_done = 1;
		wake_up(&mgi->wait);
	} else if (mgi->irq_status & ADC1INT_ADC1CAL) {
		mgi->conv_done = 1;
		wake_up(&mgi->wait);
	} else {
		/* coulomb counter should be handled by firmware. Ignore it */
		dev_dbg(mgi->dev, "coulomb counter is not support\n");
	}
	return IRQ_HANDLED;
}

static int alloc_channel_addr(struct gpadc_info *mgi, int ch)
{
	int i;
	int addr = -EBUSY;
	int last = 0;

	for (i = 0; i < GPADC_CH_MAX; i++)
		if (mgi->addr_mask & (1 << i))
			last = i;

	for (i = 0; i < GPADC_CH_MAX; i++) {
		if (!(mgi->addr_mask & (1 << i))) {
			addr = i;
			mgi->addr_mask |= 1 << i;
			if (addr > last) {
				gpadc_clear_bits(ADC1ADDR0+last, MSIC_STOPCH);
				gpadc_write(ADC1ADDR0+addr, ch|MSIC_STOPCH);
			} else {
				gpadc_write(ADC1ADDR0+addr, ch);
			}
			break;
		}
	}
	return addr;
}

static void free_channel_addr(struct gpadc_info *mgi, int addr)
{
	int last = 0;
	int i;

	mgi->addr_mask &= ~(1 << addr);
	for (i = 0; i < GPADC_CH_MAX; i++)
		if (mgi->addr_mask & (1 << i))
			last = i;
	if (addr > last)
		gpadc_set_bits(ADC1ADDR0+last, MSIC_STOPCH);
}

static void gpadc_gsmpulse_work(struct work_struct *work)
{
	int i;
	u8 data;
	int tmp;
	int vol, cur;
	struct gpadc_info *mgi =
		container_of(work, struct gpadc_info, gsmpulse_work);

	mutex_lock(&mgi->lock);
	gpadc_set_bits(ADC1CNTL3, ADC1CNTL3_GSMDATARD);

	vol = 0;
	cur = 0;
	for (i = 0; i < 4; i++) {
		gpadc_read(ADC1BV0H + i * 2, &data);
		tmp = data << 2;
		gpadc_read(ADC1BV0H + i * 2 + 1, &data);
		tmp += data & 0x3;
		if (tmp > vol)
			vol = tmp;

		gpadc_read(ADC1BI0H + i * 2, &data);
		tmp = data << 2;
		gpadc_read(ADC1BI0H + i * 2 + 1, &data);
		tmp += data & 0x3;
		if (tmp > cur)
			cur = tmp;
	}

	/**
	 * Using the calibration data, we have the voltage and current
	 * after calibration correction as below:
	 * V_CAL_CODE = V_RAW_CODE - (VZSE + (VGE)* VRAW_CODE/1023)
	 * I_CAL_CODE = I_RAW_CODE - (IZSE + (IGE)* IRAW_CODE/1023)
	*/
	vol -= mgi->vzse + mgi->vge * (vol) / 1023;
	cur -= mgi->izse + mgi->ige * (cur) / 1023;

	gpadc_set_bits(ADC1INT, ADC1INT_GSM);
	gpadc_clear_bits(ADC1CNTL3, ADC1CNTL3_GSMDATARD);
	if (mgi->gsmadc_notify)
		mgi->gsmadc_notify(vol, cur);
	mutex_unlock(&mgi->lock);
}

/**
 * intel_mid_gpadc_gsmpulse_register - power on gsm adc and register a callback
 * @fn: callback function after gsm adc conversion is completed
 *
 * Returns 0 on success or an error code.
 *
 * This function may sleep.
 */
int intel_mid_gpadc_gsmpulse_register(void(*fn)(int vol, int cur))
{
	int ret = 0;
	struct gpadc_info *mgi = &gpadc_info;

	if (!mgi->initialized)
		return -ENODEV;
	mutex_lock(&mgi->lock);
	if (!mgi->gsmadc_notify) {
		gpadc_write(ADC1CNTL2, ADC1CNTL2_DEF);
		gpadc_set_bits(ADC1CNTL2, ADC1CNTL2_ADCGSMEN);
		mgi->gsmadc_notify = fn;
	} else {
		ret = -EBUSY;
	}
	mutex_unlock(&mgi->lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_gpadc_gsmpulse_register);

/**
 * intel_mid_gpadc_gsmpulse_unregister - power off gsm adc and unregister
 *					the callback
 * @fn: callback function after gsm adc conversion is completed
 *
 * Returns 0 on success or an error code.
 *
 * This function may sleep.
 */
int intel_mid_gpadc_gsmpulse_unregister(void(*fn)(int vol, int cur))
{
	int ret = 0;
	struct gpadc_info *mgi = &gpadc_info;

	if (!mgi->initialized)
		return -ENODEV;
	mutex_lock(&mgi->lock);
	if (mgi->gsmadc_notify == fn) {
		mgi->gsmadc_notify = NULL;
		gpadc_clear_bits(ADC1CNTL2, ADC1CNTL2_ADCGSMEN);
	}
	mutex_unlock(&mgi->lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_gpadc_gsmpulse_unregister);

/**
 * intel_mid_gpadc_sample - do gpadc sample.
 * @handle: the gpadc handle
 * @sample_count: do sample serveral times and get the average value.
 * @...: sampling resulting arguments of all channels. refer to sscanf.
 *       caller should not access it before return.
 *
 * Returns 0 on success or an error code.
 *
 * This function may sleep.
 */
int intel_mid_gpadc_sample(void *handle, int sample_count, ...)
{

	struct gpadc_request *rq = handle;
	struct gpadc_info *mgi = &gpadc_info;
	int i;
	u8 data;
	int ret = 0;
	int count;
	int tmp;
	int *val[GPADC_CH_MAX];
	va_list args;

	if (!mgi->initialized)
		return -ENODEV;

	mutex_lock(&mgi->lock);
	va_start(args, sample_count);
	for (i = 0; i < rq->count; i++) {
		val[i] = va_arg(args, int*);
		*val[i] = 0;
	}
	va_end(args);
	pm_qos_add_request(&mgi->pm_qos_request,
			PM_QOS_CPU_DMA_LATENCY,	CSTATE_EXIT_LATENCY_S0i1-1);
	gpadc_poweron(mgi, rq->vref);
	gpadc_clear_bits(ADC1CNTL1, ADC1CNTL1_AD1OFFSETEN);
	gpadc_read(ADC1CNTL1, &data);
	data = (data & ~ADC1CNTL1_ADSLP) + ADC1CNTL1_ADSLP_DEF;
	gpadc_write(ADC1CNTL1, data);
	mgi->rnd_done = 0;
	gpadc_set_bits(ADC1CNTL1, ADC1CNTL1_ADSTRT);
	for (count = 0; count < sample_count; count++) {
		if (wait_event_timeout(mgi->wait, mgi->rnd_done, HZ) == 0) {
			gpadc_dump(mgi);
			dev_err(mgi->dev, "sample timeout\n");
			ret = -ETIMEDOUT;
			goto fail;
		}
		gpadc_set_bits(ADC1CNTL3, ADC1CNTL3_RRDATARD);
		for (i = 0; i < rq->count; ++i) {
			tmp = 0;
			gpadc_read(ADC1SNS0H + 2 * rq->addr[i], &data);
			tmp += data << 2;
			gpadc_read(ADC1SNS0H + 2 * rq->addr[i] + 1, &data);
			tmp += data & 0x3;

			/**
			 * Using the calibration data, we have the voltage and
			 * current after calibration correction as below:
			 * V_CAL_CODE = V_RAW_CODE - (VZSE+(VGE)*VRAW_CODE/1023)
			 * I_CAL_CODE = I_RAW_CODE - (IZSE+(IGE)*IRAW_CODE/1023)
			 */
			if (rq->ch[i] & CH_NEED_VCALIB)
				tmp -= mgi->vzse + mgi->vge * tmp / 1023;
			if (rq->ch[i] & CH_NEED_ICALIB)
				tmp -= mgi->izse + mgi->ige * tmp / 1023;
			*val[i] += tmp;
		}
		gpadc_clear_bits(ADC1CNTL3, ADC1CNTL3_RRDATARD);
		mgi->rnd_done = 0;
	}

	for (i = 0; i < rq->count; ++i)
		*val[i] /= sample_count;

fail:
	gpadc_clear_bits(ADC1CNTL1, ADC1CNTL1_ADSTRT);
	gpadc_poweroff(mgi);
	pm_qos_remove_request(&mgi->pm_qos_request);
	mutex_unlock(&mgi->lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_gpadc_sample);

/**
 * intel_mid_gpadc_free - free gpadc
 * @handle: the gpadc handle
 *
 * This function may sleep.
 */
void intel_mid_gpadc_free(void *handle)
{
	struct gpadc_request *rq = handle;
	struct gpadc_info *mgi = &gpadc_info;
	int i;

	mutex_lock(&mgi->lock);
	for (i = 0; i < rq->count; i++)
		free_channel_addr(mgi, rq->addr[i]);
	mutex_unlock(&mgi->lock);
	kfree(rq);
}
EXPORT_SYMBOL(intel_mid_gpadc_free);

/**
 * intel_mid_gpadc_alloc - allocate gpadc for channels
 * @count: the count of channels
 * @...: the channel parameters. (channel idx | flags)
 *       flags:
 *             CH_NEED_VCALIB   it needs voltage calibration
 *             CH_NEED_ICALIB   it needs current calibration
 *
 * Returns gpadc handle on success or NULL on fail.
 *
 * This function may sleep.
 */
void *intel_mid_gpadc_alloc(int count, ...)
{
	struct gpadc_request *rq;
	struct gpadc_info *mgi = &gpadc_info;
	va_list args;
	int ch;
	int i;

	if (!mgi->initialized)
		return NULL;

	rq = kzalloc(sizeof(struct gpadc_request), GFP_KERNEL);
	if (rq == NULL)
		return NULL;

	va_start(args, count);
	mutex_lock(&mgi->lock);
	rq->count = count;
	for (i = 0; i < count; i++) {
		ch = va_arg(args, int);
		rq->ch[i] = ch;
		if (ch & CH_NEED_VREF)
			rq->vref = 1;
		ch &= 0xf;
		rq->addr[i] = alloc_channel_addr(mgi, ch);
		if (rq->addr[i] < 0) {
			dev_err(mgi->dev, "alloc addr failed\n");
			while (i-- > 0)
				free_channel_addr(mgi, rq->addr[i]);
			kfree(rq);
			rq = NULL;
			break;
		}
	}
	mutex_unlock(&mgi->lock);
	va_end(args);

	return rq;
}
EXPORT_SYMBOL(intel_mid_gpadc_alloc);

static ssize_t intel_mid_gpadc_store_alloc_channel(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int val, hdn;
	int ch[SAMPLE_CH_MAX];

	val = sscanf(buf, "%d %x %x", &hdn, &ch[0], &ch[1]);

	if (val < 2 || val > 3) {
		dev_err(dev, "invalid number of arguments");
		return -EINVAL;
	}

	if (hdn < 1 || hdn > GPADC_CH_MAX) {
		dev_err(dev, "invalid handle value");
		return -EINVAL;
	}

	if (adc_handle[hdn - 1]) {
		dev_err(dev, "adc handle %d has been occupied", hdn);
		return -EBUSY;
	}
	if (val == 2)
		adc_handle[hdn - 1] = intel_mid_gpadc_alloc(1, ch[0]);
	else
		adc_handle[hdn - 1] = intel_mid_gpadc_alloc(2, ch[0], ch[1]);

	if (!adc_handle[hdn - 1]) {
		dev_err(dev, "allocating adc handle %d failed", hdn);
		return -ENOMEM;
	}

	return size;
}

static ssize_t intel_mid_gpadc_store_free_channel(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int hdn;

	if (sscanf(buf, "%d", &hdn) != 1) {
		dev_err(dev, "invalid number of argument");
		return -EINVAL;
	}

	if (hdn < 1 || hdn > GPADC_CH_MAX) {
		dev_err(dev, "invalid handle value");
		return -EINVAL;
	}

	if (adc_handle[hdn - 1]) {
		intel_mid_gpadc_free(adc_handle[hdn - 1]);
		adc_handle[hdn - 1] = NULL;
	}

	return size;
}

static ssize_t intel_mid_gpadc_store_sample(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int hdn, spc;
	int ret;
	struct gpadc_request *rq;

	if (sscanf(buf, "%d %d", &hdn, &spc) != 2) {
		dev_err(dev, "invalid number of arguments");
		return -EINVAL;
	}

	if (hdn < 1 || hdn > GPADC_CH_MAX) {
		dev_err(dev, "invalid handle value");
		return -EINVAL;
	}

	rq = adc_handle[hdn - 1];
	if (!rq) {
		dev_err(dev, "null handle");
		return -EINVAL;
	}

	if (rq->count == 1)
		ret = intel_mid_gpadc_sample(adc_handle[hdn-1],
			spc, &sample_result[hdn - 1][0]);
	else
		ret = intel_mid_gpadc_sample(adc_handle[hdn - 1],
			spc, &sample_result[hdn - 1][0],
			&sample_result[hdn - 1][1]);

	if (ret) {
		dev_err(dev, "sampling failed. adc handle: %d", hdn);
		return -EINVAL;
	}

	return size;
}

static ssize_t intel_mid_gpadc_show_sample(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int hdc;
	int used = 0;
	struct gpadc_request *rq;

	for (hdc = 0; hdc < GPADC_CH_MAX; hdc++) {
		if (adc_handle[hdc]) {
			rq = adc_handle[hdc];
			if (rq->count == 1)
				used += sprintf(buf + used, "%d ",
					sample_result[hdc][0]);
			else
				used += sprintf(buf + used, "%d %d ",
					sample_result[hdc][0],
					sample_result[hdc][1]);
		}
	}

	return used;
}


static void gsmpulse_sysfs_callback(int vol, int cur)
{
	vol_val = vol;
	cur_val = cur;
	complete(&gsmadc_complete);
}

static ssize_t intel_mid_gpadc_show_gsmpulse_sample(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	INIT_COMPLETION(gsmadc_complete);
	intel_mid_gpadc_gsmpulse_register(gsmpulse_sysfs_callback);
	ret = wait_for_completion_interruptible(&gsmadc_complete);
	intel_mid_gpadc_gsmpulse_unregister(gsmpulse_sysfs_callback);
	if (ret)
		return 0;
	else
		return sprintf(buf, "%d %d", vol_val, cur_val);
}

static DEVICE_ATTR(alloc_channel, S_IWUSR, NULL,
		intel_mid_gpadc_store_alloc_channel);
static DEVICE_ATTR(free_channel, S_IWUSR, NULL,
		intel_mid_gpadc_store_free_channel);
static DEVICE_ATTR(sample, S_IRUGO | S_IWUSR,
		intel_mid_gpadc_show_sample, intel_mid_gpadc_store_sample);
static DEVICE_ATTR(gsmpulse_sample, S_IRUGO,
		intel_mid_gpadc_show_gsmpulse_sample, NULL);

static struct attribute *intel_mid_gpadc_attrs[] = {
	&dev_attr_alloc_channel.attr,
	&dev_attr_free_channel.attr,
	&dev_attr_sample.attr,
	&dev_attr_gsmpulse_sample.attr,
	NULL,
};

static struct attribute_group intel_mid_gpadc_attr_group = {
	.name = "mid_gpadc",
	.attrs = intel_mid_gpadc_attrs,
};

static int __devinit msic_gpadc_probe(struct platform_device *pdev)
{
	struct gpadc_info *mgi = &gpadc_info;
	struct intel_mid_gpadc_platform_data *pdata = pdev->dev.platform_data;
	int err = 0;

	mutex_init(&mgi->lock);
	init_waitqueue_head(&mgi->wait);
	init_waitqueue_head(&mgi->trimming_wait);
	mgi->workq = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (mgi->workq == NULL)
		return -ENOMEM;

	mgi->dev = &pdev->dev;
	mgi->intr = ioremap_nocache(pdata->intr, 4);
	mgi->irq = platform_get_irq(pdev, 0);

	gpadc_clear_bits(IRQLVL1MSK, IRQLVL1MSK_ADCM);
	if (request_threaded_irq(mgi->irq, msic_gpadc_isr, msic_gpadc_irq,
					IRQF_ONESHOT, "msic_adc", mgi)) {
		dev_err(&pdev->dev, "unable to register irq %d\n", mgi->irq);
		err = -ENODEV;
		goto err_exit;
	}

	gpadc_write(ADC1ADDR0, MSIC_STOPCH);
	INIT_WORK(&mgi->trimming_work, gpadc_trimming);
	INIT_WORK(&mgi->gsmpulse_work, gpadc_gsmpulse_work);
	queue_work(mgi->workq, &mgi->trimming_work);
	wait_event(mgi->trimming_wait, mgi->trimming_start);
	mgi->initialized = 1;

	init_completion(&gsmadc_complete);

	err = sysfs_create_group(&pdev->dev.kobj, &intel_mid_gpadc_attr_group);
	if (err) {
		dev_err(&pdev->dev, "Unable to export sysfs interface, error: %d\n",
			err);
		goto err_release_irq;
	}

	return 0;

err_release_irq:
	free_irq(mgi->irq, mgi);
err_exit:
	if (mgi->intr)
		iounmap(mgi->intr);
	return err;
}

static int __devexit msic_gpadc_remove(struct platform_device *pdev)
{
	struct gpadc_info *mgi = &gpadc_info;

	sysfs_remove_group(&pdev->dev.kobj, &intel_mid_gpadc_attr_group);
	free_irq(mgi->irq, mgi);
	iounmap(mgi->intr);
	flush_workqueue(mgi->workq);
	destroy_workqueue(mgi->workq);
	return 0;
}

#ifdef CONFIG_PM
static int msic_gpadc_suspend_noirq(struct device *dev)
{
	struct gpadc_info *mgi = &gpadc_info;

	/* If the gpadc is locked, it means gpadc is still in active mode. */
	if (mutex_trylock(&mgi->lock))
		return 0;
	else
		return -EBUSY;
}

static int msic_gpadc_resume_noirq(struct device *dev)
{
	struct gpadc_info *mgi = &gpadc_info;

	mutex_unlock(&mgi->lock);
	return 0;
}
#else
#define msic_gpadc_suspend_noirq    NULL
#define msic_gpadc_resume_noirq     NULL
#endif

static const struct dev_pm_ops msic_gpadc_driver_pm_ops = {
	.suspend_noirq	= msic_gpadc_suspend_noirq,
	.resume_noirq	= msic_gpadc_resume_noirq,
};

static struct platform_driver msic_gpadc_driver = {
	.driver = {
		   .name = "msic_adc",
		   .owner = THIS_MODULE,
		   .pm = &msic_gpadc_driver_pm_ops,
		   },
	.probe = msic_gpadc_probe,
	.remove = __devexit_p(msic_gpadc_remove),
};

static int __init msic_gpadc_module_init(void)
{
	return platform_driver_register(&msic_gpadc_driver);
}

static void __exit msic_gpadc_module_exit(void)
{
	platform_driver_unregister(&msic_gpadc_driver);
}

module_init(msic_gpadc_module_init);
module_exit(msic_gpadc_module_exit);

MODULE_AUTHOR("Jenny TC <jenny.tc@intel.com>");
MODULE_DESCRIPTION("Intel Medfield MSIC GPADC Driver");
MODULE_LICENSE("GPL");
