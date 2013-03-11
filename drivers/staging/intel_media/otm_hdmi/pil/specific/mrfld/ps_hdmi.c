/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2012 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  Contact Information:

  Intel Corporation
  2200 Mission College Blvd.
  Santa Clara, CA  95054

  BSD LICENSE

  Copyright(c) 2011 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "otm_hdmi_types.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/string.h>
#include "otm_hdmi.h"
#include "ipil_hdmi.h"
#include "ps_hdmi.h"
#include <asm/intel_scu_pmic.h>
#include <asm/intel-mid.h>


/* Implementation of the Merrifield specific PCI driver for receiving
 * Hotplug and other device status signals.
 * In Merrifield platform, the HPD and OCP signals are delivered to the
 * display sub-system using the TI TPD Companion chip.
 */

/* Constants */
#define PS_HDMI_HPD_PCI_DRIVER_NAME "Merrifield HDMI HPD Driver"

/* Globals */
static hdmi_context_t *g_context;

#define PS_HDMI_MMIO_RESOURCE 0
#define PS_VDC_OFFSET 0x00000000
#define PS_VDC_SIZE 0x000080000
#define PS_MSIC_PCI_DEVICE_ID 0x11A6
#define PS_MSIC_HPD_GPIO_PIN 16
/* HDMI_LS_EN GPIO pin is connected to Levelshifter's LS_OE pin */
#define PS_MSIC_LS_EN_GPIO_PIN 177


/* For Merrifield, it is required that SW pull up or pull down the
 * LS_OE GPIO pin based on cable status. This is needed before
 * performing any EDID read operation on Merrifield.
 */
static void __ps_gpio_configure_edid_read(void)
{
	static int old_pin_value  = -1;
	int new_pin_value = gpio_get_value(PS_MSIC_HPD_GPIO_PIN);

	if (new_pin_value == old_pin_value)
		return;

	old_pin_value = new_pin_value;

	if (new_pin_value == 0)
		gpio_set_value(PS_MSIC_LS_EN_GPIO_PIN, 0);
	else
		gpio_set_value(PS_MSIC_LS_EN_GPIO_PIN, 1);

	pr_debug("%s: CTP_HDMI_LS_OE pin = %d (%d)\n", __func__,
		 gpio_get_value(PS_MSIC_LS_EN_GPIO_PIN), new_pin_value);
}

otm_hdmi_ret_t ps_hdmi_pci_dev_init(void *context, struct pci_dev *pdev)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	int result = 0;
	unsigned int vdc_start;
	uint32_t pci_address = 0;
	uint8_t pci_dev_revision = 0;
	hdmi_context_t *ctx = NULL;

	if (pdev == NULL || context == NULL) {
		rc = OTM_HDMI_ERR_INTERNAL;
		goto exit;
	}
	ctx = (hdmi_context_t *)context;

	pr_debug("get resource start\n");
	result = pci_read_config_dword(pdev, 16, &vdc_start);
	if (result != 0) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}
	pci_address = vdc_start + PS_VDC_OFFSET;

	pr_debug("map IO region\n");
	/* Map IO region and save its length */
	ctx->io_length = PS_VDC_SIZE;
	ctx->io_address = ioremap_cache(pci_address, ctx->io_length);
	if (!ctx->io_address) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}

	pr_debug("get PCI dev revision\n");
	result = pci_read_config_byte(pdev, 8, &pci_dev_revision);
	if (result != 0) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}
	ctx->dev.id = pci_dev_revision;
	/* Store this context for use by MSIC PCI driver */
	g_context = ctx;

	/* Handle Merrifield specific GPIO configuration
	 * to enable EDID reads
	 */
	if (gpio_request(PS_MSIC_LS_EN_GPIO_PIN, "HDMI_LS_EN")) {
		pr_err("%s: Unable to request gpio %d\n", __func__,
		       PS_MSIC_LS_EN_GPIO_PIN);
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}

	if (!gpio_is_valid(PS_MSIC_LS_EN_GPIO_PIN)) {
		pr_err("%s: Unable to validate gpio %d\n", __func__,
		       PS_MSIC_LS_EN_GPIO_PIN);
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}

	/* Set the GPIO based on cable status */
	__ps_gpio_configure_edid_read();

exit:
	return rc;
}

otm_hdmi_ret_t ps_hdmi_pci_dev_deinit(void *context)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	hdmi_context_t *ctx = NULL;

	if (context == NULL) {
		rc = OTM_HDMI_ERR_INTERNAL;
		goto exit;
	}
	ctx = (hdmi_context_t *)context;

	/* unmap IO region */
	iounmap(ctx->io_address);

	/* Free GPIO resources */
	gpio_free(PS_MSIC_LS_EN_GPIO_PIN);
exit:
	return rc;
}

otm_hdmi_ret_t ps_hdmi_i2c_edid_read(void *ctx, unsigned int sp,
				  unsigned int offset, void *buffer,
				  unsigned int size)
{
	hdmi_context_t *context = (hdmi_context_t *)ctx;

	char *src = context->edid_raw + sp * SEGMENT_SIZE + offset;
	memcpy(buffer, src, size);

	return OTM_HDMI_SUCCESS;
}

static void ps_hdmi_power_on_pipe(u32 msg_port, u32 msg_reg,
							u32 val_comp, u32 val_write)
{
	u32 ret;
	int retry = 0;

	pr_debug("Entered %s\n", __func__);

	ret = intel_mid_msgbus_read32(msg_port, msg_reg);

	if ((ret & val_comp) == 0) {
		pr_err("%s: pipe is already powered on\n", __func__);
		return;
	} else {
		intel_mid_msgbus_write32(msg_port, msg_reg, ret & val_write);
		ret = intel_mid_msgbus_read32(msg_port, msg_reg);
		while ((retry < 1000) && ((ret & val_comp) != 0)) {
			usleep_range(500, 1000);
			ret = intel_mid_msgbus_read32(msg_port, msg_reg);
			retry++;
		}
		if ((ret & val_comp) != 0)
			pr_err("%s: powering on pipe failed\n", __func__);
		if (msg_port == 0x4 && msg_reg == 0x3b)
			pr_err("%s: skip powering up MIO AFE\n", __func__);
	}
	pr_debug("Leaving %s\n", __func__);
}

bool ps_hdmi_power_rails_on(void)
{
	pr_debug("Entered %s\n", __func__);
	ps_hdmi_power_on_pipe(0x4, 0x36, 0xc000000, 0xfffffff3);
		/* pipe B */
	ps_hdmi_power_on_pipe(0x4, 0x3c, 0x3000000, 0xfffffffc);
		/* HDMI */

	intel_scu_ipc_iowrite8(0x7F, 0x31);
	pr_debug("Leaving %s\n", __func__);
	return true;
}

bool ps_hdmi_power_rails_off(void)
{
	pr_debug("Entered %s\n", __func__);

	return 0;

}


/*
 * ps_hdmi_get_cable_status - Get HDMI cable connection status
 * @context: hdmi device context
 *
 * Returns - boolean state.
 * true - HDMI cable connected
 * false - HDMI cable disconnected
 */
bool ps_hdmi_get_cable_status(void *context)
{
	hdmi_context_t *ctx = (hdmi_context_t *)context;
	if (ctx == NULL)
		return false;

	/* Read HDMI cable status from GPIO */
	/* For Merrifield, it is required that SW pull up or pull down the
	 * LS_OE GPIO pin based on cable status. This is needed before
	 * performing any EDID read operation on Merrifield.
	 */
	__ps_gpio_configure_edid_read();

	if (gpio_get_value(PS_MSIC_HPD_GPIO_PIN) == 0)
		return false;
	else
		return true;
}

/**
 * hdmi interrupt handler (top half).
 * @irq:	irq number
 * @data:	data for the interrupt handler
 *
 * Returns:	IRQ_HANDLED on NULL input arguments, and if the
 *			interrupt is not HDMI HPD interrupts.
 *		IRQ_WAKE_THREAD if this is a HDMI HPD interrupt.
 * hdmi interrupt handler (upper half). handles the interrupts
 * by reading hdmi status register and waking up bottom half if needed.
 */
irqreturn_t ps_hdmi_irq_handler(int irq, void *data)
{
	if (g_context == NULL)
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

/* Power management functions */
static int ps_hdmi_hpd_suspend(struct device *dev)
{
	pr_debug("Entered %s\n", __func__);
	ps_hdmi_power_rails_off();
	return 0;
}

static int ps_hdmi_hpd_resume(struct device *dev)
{
	pr_debug("Entered %s\n", __func__);
	ps_hdmi_power_rails_on();
	return 0;
}

/* PCI probe function */
static int __devinit ps_hdmi_hpd_probe(struct pci_dev *pdev,
				       const struct pci_device_id *id)
{
	int result = 0;
	hdmi_context_t *ctx = g_context;

	if (pdev == NULL || ctx == NULL) {
		pr_err("%s: called with NULL device or context\n", __func__);
		result = -EINVAL;
		return result;
	}

	/* Verify probe is called for the intended device */
	if (pdev->device != PS_MSIC_PCI_DEVICE_ID) {
		pr_err("%s: called for wrong device id = 0x%x\n", __func__,
		       pdev->device);
		result = -EINVAL;
		goto exit;
	}

	pr_debug("pci_enable_device for 0x%x\n",
					PS_MSIC_PCI_DEVICE_ID);
	result = pci_enable_device(pdev);
	if (result) {
		pr_err("%s: Failed to enable MSIC PCI device = 0x%x\n",
		       __func__, PS_MSIC_PCI_DEVICE_ID);
		goto exit;
	}

	/* Perform the GPIO configuration */
	result = gpio_request(PS_MSIC_HPD_GPIO_PIN, "hdmi_hpd");
	if (result) {
		pr_debug("%s: Failed to request GPIO %d for kbd IRQ\n",
			 __func__, PS_MSIC_HPD_GPIO_PIN);
		goto exit2;
	}

	result = gpio_direction_input(PS_MSIC_HPD_GPIO_PIN);
	if (result) {
		pr_debug("%s: Failed to set GPIO %d as input\n",
			 __func__, PS_MSIC_HPD_GPIO_PIN);
		goto exit3;
	}

	ctx->irq_number = gpio_to_irq(PS_MSIC_HPD_GPIO_PIN);
	pr_debug("%s: IRQ number assigned = %d\n", __func__, ctx->irq_number);

	result = irq_set_irq_type(ctx->irq_number, IRQ_TYPE_EDGE_BOTH);
	if (result) {
		pr_debug("%s: Failed to set HDMI HPD IRQ type for IRQ %d\n",
			 __func__, ctx->irq_number);
		goto exit3;
	}

	/* This is unused on Merrifield platform, since we use GPIO */
	ctx->dev.irq_io_address = 0;

	result = request_threaded_irq(ctx->irq_number, ps_hdmi_irq_handler,
				      ctx->hpd_callback, IRQF_SHARED,
				      PS_HDMI_HPD_PCI_DRIVER_NAME,
				      ctx->hpd_data);
	if (result) {
		pr_debug("%s: Register irq interrupt %d failed\n",
			 __func__, ctx->irq_number);
		goto exit3;
	}
	return result;

exit3:
	gpio_free(PS_MSIC_HPD_GPIO_PIN);
exit2:
	pci_disable_device(pdev);
exit:
	pci_dev_put(pdev);
	return result;
}

/* PCI driver related structures */
static DEFINE_PCI_DEVICE_TABLE(ps_hdmi_hpd_pci_id) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PS_MSIC_PCI_DEVICE_ID) },
	{ 0 }
};

static const struct dev_pm_ops ps_hdmi_hpd_pm_ops = {
	.suspend = ps_hdmi_hpd_suspend,
	.resume = ps_hdmi_hpd_resume,
};

static struct pci_driver ps_hdmi_hpd_driver = {
	.name = PS_HDMI_HPD_PCI_DRIVER_NAME,
	.id_table = ps_hdmi_hpd_pci_id,
	.probe = ps_hdmi_hpd_probe,
	.driver.pm = &ps_hdmi_hpd_pm_ops,
};

/* PCI Driver registration function */
int ps_hdmi_hpd_register_driver(void)
{
	pr_debug("%s: Registering PCI driver for HDMI HPD\n", __func__);
	return pci_register_driver(&ps_hdmi_hpd_driver);
}

/* PCI Driver Cleanup function */
int ps_hdmi_hpd_unregister_driver(void)
{
	pci_unregister_driver(&ps_hdmi_hpd_driver);
	return 0;
}


