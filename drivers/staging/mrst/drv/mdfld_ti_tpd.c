/*
 * Copyright (c) 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Austin Hu <austin.hu@intel.com>
 */

/*
 * TI HDMI COMPANION CHIP (TPD12S015) used for Intel CloverView SoC.
 */

#include "mdfld_ti_tpd.h"
#include "psb_drv.h"
#include "psb_intel_hdmi.h"

static struct mid_intel_hdmi_priv *hdmi_priv;

void mdfld_ti_tpd_init(struct mid_intel_hdmi_priv *p_hdmi_priv)
{
	hdmi_priv = p_hdmi_priv;
}

/**
 *  ti_tpd_hpd_handler
 */
irqreturn_t hdmi_hpd_handler(int irq, void *dev_id)
{
	struct mid_intel_hdmi_priv *hdmi_priv = dev_id;
	struct drm_device *dev = hdmi_priv ? hdmi_priv->dev : NULL;
	struct drm_psb_private *dev_priv = NULL;

	PSB_DEBUG_ENTRY("\n");

	if (dev) {
		dev_priv = psb_priv(dev);

		if (dev_priv && dev_priv->um_start)
			hpd_notify_um(dev);
		else
			return IRQ_NONE;
	} else
		return IRQ_NONE;

	return IRQ_HANDLED;
}

/**
 *  ti_tpd_probe
 */
static int __devinit ti_tpd_probe(struct pci_dev *pdev,
		const struct pci_device_id *ent)
{
	struct drm_device *dev = hdmi_priv ? hdmi_priv->dev : 0;
	struct drm_psb_private *dev_priv = dev ? psb_priv(dev) : 0;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	if (pdev->device != TI_TPD_PCI_DEVICE_ID) {
		DRM_ERROR("%s: pciid = 0x%x is not ti_tpd pciid.\n",
			  __func__, pdev->device);
		ret = -EINVAL;
		goto err0;
	}

	/* enable HDMI COMPANION CHIP device */
	ret = pci_enable_device(pdev);
	if (ret) {
		DRM_ERROR("%s:Fail to enable pci devices\n",
				__func__);
		goto err0;
	}

	ret = gpio_request(CLV_TI_HPD_GPIO_PIN, "ti_tpd_hdmi_hpd");
	if (ret) {
		DRM_ERROR("%s: Failed to request GPIO %d for kbd IRQ\n",
				__func__, CLV_TI_HPD_GPIO_PIN);
		goto err1;
	}

	ret = gpio_direction_input(CLV_TI_HPD_GPIO_PIN);
	if (ret) {
		DRM_ERROR("%s: Failed to set GPIO %d as input\n",
				__func__, CLV_TI_HPD_GPIO_PIN);
		goto err2;
	}

	ret = irq_set_irq_type(gpio_to_irq(CLV_TI_HPD_GPIO_PIN),
			IRQ_TYPE_EDGE_BOTH);
	if (ret) {
		DRM_ERROR("%s: Failed to set HDMI HPD IRQ type\n",
				__func__);
		goto err2;
	}

	if (dev_priv == NULL) {
		DRM_ERROR("%s: Invalid parameter", __func__);
		ret = -EINVAL;
		goto err2;
	}
	dev_priv->hpd_detect = create_freezable_workqueue("hpd_detect");
	if (dev_priv->hpd_detect == NULL) {
		DRM_ERROR("%s: Creating workqueue failed", __func__);
		ret = -ENOMEM;
		goto err2;
	}

	ret = request_irq(gpio_to_irq(CLV_TI_HPD_GPIO_PIN),
			hdmi_hpd_handler, IRQF_SHARED,
			"hdmi_hpd_handler", (void *)hdmi_priv);
	if (ret) {
		DRM_ERROR("%s: Can not register GPIO %d IRQ!\n",
				__func__, CLV_TI_HPD_GPIO_PIN);
		goto err3;
	}

	PSB_DEBUG_ENTRY("%s: Requested HDMI HPD IRQ"
			"sussessfully.\n", __func__);
	return ret;

err3:
	destroy_workqueue(dev_priv->hpd_detect);
err2:
	gpio_free(CLV_TI_HPD_GPIO_PIN);
err1:
	pci_disable_device(pdev);
err0:
	pci_dev_put(pdev);
	DRM_ERROR("%s: request_irq failed. ret = %d.\n", __func__, ret);
	return ret;
}

static DEFINE_PCI_DEVICE_TABLE(ti_tpd_pci_id_list) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, TI_TPD_PCI_DEVICE_ID) },
	{ 0 }
};

/* field for registering driver to PCI device */
static struct pci_driver ti_tpd_pci_driver = {
	.name = "ti_tpd_hdmi_driver",
	.id_table = ti_tpd_pci_id_list,
	.probe = ti_tpd_probe
};

/**
 *  ti_tpd_regsiter_driver - register the TI HDMI COMPANION CHIP to PCI system.
 */
int ti_tpd_regsiter_driver(void)
{
	return pci_register_driver(&ti_tpd_pci_driver);
}

/**
 *  ti_tpd_unregsiter_driver - unregister the HDMI COMPANION CHIP from PCI system.
 */
int ti_tpd_unregister_driver(void)
{
	pci_unregister_driver(&ti_tpd_pci_driver);
	return 0;
}
