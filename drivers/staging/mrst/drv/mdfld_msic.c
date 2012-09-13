/*
 * Copyright © 2010 Intel Corporation
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
 *	Jim Liu <jim.liu@intel.com>
 */

#include "mdfld_msic.h"
#include "psb_drv.h"
#include "psb_intel_hdmi.h"
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34))
#include <asm/intel_scu_pmic.h>
#endif

#define SRAM_MSIC_VRINT_ADDR 0xFFFF7FCB
static u8 *sram_vreg_addr = 0;
unsigned char vrint_dat;
static bool hpd_suspended;
static u8 hdmi_saved_status;

/*
 *
 */
static struct mid_intel_hdmi_priv *hdmi_priv;

void mdfld_msic_init(struct mid_intel_hdmi_priv *p_hdmi_priv)
{
	hdmi_priv = p_hdmi_priv;
}

/**
 *  hpd_notify_um
 */
void hpd_notify_um(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = NULL;
	struct pci_dev *pdev = NULL;
	struct device *ddev = NULL;
	struct kobject *kobj = NULL;

	if (dev)
	{
		PSB_DEBUG_ENTRY("\n");
		dev_priv = psb_priv(dev);
		dev_priv->hdmi_done_reading_edid = false;
	}

	/*find handle to drm kboject*/
	if(dev == NULL){
		DRM_ERROR("%s: dev == NULL.\n", __func__);
		return;
	}
	pdev = dev->pdev;

	if(pdev == NULL){
		DRM_ERROR("%s: pdev == NULL.\n", __func__);
		return;
	}
	ddev = &pdev->dev;

	if(ddev == NULL){
		DRM_ERROR("%s: ddev == NULL.\n", __func__);
		return;
	}
	kobj = &ddev->kobj;

	if(kobj == NULL){
		DRM_ERROR("%s: kobj == NULL.\n", __func__);
		return;
	}

	if (dev_priv->psb_hotplug_state) {
		PSB_DEBUG_ENTRY("%s: HPD interrupt.\n", __func__);
		psb_hotplug_notify_change_um("hpd_hdmi", dev_priv->psb_hotplug_state);
	} else {
		DRM_INFO("%s: Hotplug comm layer isn't initialized!\n",
				__func__);
	}

	/* send drm uevent message */
	queue_work(dev_priv->hpd_detect, &dev_priv->hdmi_hotplug_wq);

	return;
}

/**
 *  msic_vreg_handler
 */
irqreturn_t msic_vreg_handler(int irq, void *dev_id)
{
	struct drm_device *dev = hdmi_priv ? hdmi_priv->dev : NULL;
	struct drm_psb_private *dev_priv = NULL;
	vrint_dat = 0;

	if (hpd_suspended)
		return IRQ_HANDLED;

	/* Need to add lock later.*/

	/* Read VREG interrupt status register */
	if (sram_vreg_addr)
		vrint_dat = readb(sram_vreg_addr);
	else
		DRM_ERROR("%s: sram_vreg_addr = 0x%x.\n",
				__func__, (u32) sram_vreg_addr);

	if (dev)
	{
		PSB_DEBUG_ENTRY("vrint data = 0x%x.\n", vrint_dat);
		dev_priv = psb_priv(dev);

		/* handle HDMI HPD interrupts. */
		if (vrint_dat & (HDMI_HPD_STATUS | HDMI_OCP_STATUS))
		{
			DRM_INFO("%s: HPD interrupt.vrint data = 0x%x.\n", __func__, vrint_dat);

			if (dev_priv->um_start)
				hpd_notify_um(dev);
		}
	}
	/* handle other msic vreq interrupts when necessary. */

	return IRQ_HANDLED;
}

/*
 * Tell irq handler that possible hpd interrupts should be handled
 * again (hpd_suspended).
 *
 * Power on HDMI rails again. In case we have cable plugged in -> HPD
 * interrupt is generated and detection is started.
 *
 * Check possible cable plug out by comparing current hdmi status
 * against the saved one (hdmi_saved_status).
 *
 * Please note that workqueue which does the detection itself is frozen at
 * this point. Possible hpd interrupt after hpd_suspended is set as
 * false will queue detection, but the detection is started later
 * when workqueue is woken up again.
 */
static int msic_resume(struct device *dev)
{
	int ret = 0;
	u8 hdmi_status;
	struct drm_device *drm_dev = hdmi_priv ? hdmi_priv->dev : NULL;

	hpd_suspended = false;

	ret = intel_scu_ipc_update_register(MSIC_IRQLVL1_MASK, 0x0, VREG_MASK);
	if (ret) {
		DRM_ERROR("%s: Failed to unmask VREG IRQ.\n",
			__func__);
		goto err;
	}

	ret = intel_scu_ipc_iowrite8(MSIC_VCC330CNT, VCC330_ON);
	if (ret) {
		DRM_ERROR("%s: Failed to read MSIC_HDMI_STATUS.\n",
			  __func__);
		goto err;
	}

	/* MSIC documentation requires that there be a 500us delay
	   after enabling VCC330 before you can enable VHDMI */
	usleep_range(500, 1000);

	ret = intel_scu_ipc_iowrite8(MSIC_VHDMICNT, VHDMI_ON | VHDMI_DB_30MS);
	if (ret) {
		DRM_ERROR("%s: Failed to power on VHDMI.\n",
			  __func__);
		goto err;
	}

	ret = intel_scu_ipc_ioread8(MSIC_HDMI_STATUS, &hdmi_status);
	if (ret)
		DRM_ERROR("%s: Failed to read MSIC_HDMI_STATUS.\n",
			  __func__);

	if (!ret && (hdmi_saved_status & HPD_SIGNAL_STATUS) &&
	    !(hdmi_status & HPD_SIGNAL_STATUS))
		hpd_notify_um(drm_dev);

	return ret;

err:
	hpd_suspended = true;
	return ret;
}

/*
 * On suspend we want to power off VHDMI and VCC330. Hotplug
 * detection doesn't work without these rails. This is acceptable as
 * long as changes are detected when waking up from suspend.
 *
 * HDMI cable plug out detection during suspend is done in msic_resume by saving
 * hdmi status (hdmi_saved_status) and using it in in resume to detect
 * cable unplug. Cable plug in is detected as hpd interrupt is
 * generated always when powering on HDMI rails while HDMI cable is
 * plugged in.
 *
 * Hpd_suspended is used to tell irq handler that the detection is now
 * suspended and possible HPD interrupts can be ignored. Otherwise
 * powering down HDMI rails would cause interrupt and detection when
 * suspending with HDMI cable plugged in.
 *
 * Please note that workqueue which does the detection itself is frozen at
 * this point. Possible hpd interrupt before hpd_suspended is set true
 * will queue detection, but it's started later when workqueue is woken up
 * again on resume path.
 */
static int msic_suspend(struct device *dev)
{
	int ret = 0, err = 0;
	u8 hdmi_status;
	struct drm_device *drm_dev = hdmi_priv ? hdmi_priv->dev : NULL;

	ret = intel_scu_ipc_ioread8(MSIC_HDMI_STATUS, &hdmi_saved_status);
	if (ret) {
		DRM_ERROR("%s: Failed to read MSIC_HDMI_STATUS.\n",
			  __func__);
		goto err1;
	}

	ret = intel_scu_ipc_update_register(MSIC_IRQLVL1_MASK, 0xff, VREG_MASK);
	if (ret) {
		DRM_ERROR("%s: Failed to mask VREG IRQ.\n",
			  __func__);
		goto err1;
	}

	hpd_suspended = true;

	ret = intel_scu_ipc_iowrite8(MSIC_VHDMICNT, VHDMI_OFF);
	if (ret) {
		DRM_ERROR("%s: Failed to power off VHDMI.\n",
			  __func__);
		goto err2;
	}

	ret = intel_scu_ipc_iowrite8(MSIC_VCC330CNT, VCC330_OFF);
	if (ret) {
		DRM_ERROR("%s: Failed to power off VCC330.\n",
			  __func__);
		goto err3;
	}

	return ret;

err3:
	err |= intel_scu_ipc_iowrite8(MSIC_VHDMICNT, VHDMI_ON | VHDMI_DB_30MS);
err2:
	hpd_suspended = false;
	err |= intel_scu_ipc_update_register(MSIC_IRQLVL1_MASK, 0x0, VREG_MASK);
	err |= intel_scu_ipc_ioread8(MSIC_HDMI_STATUS, &hdmi_status);
	if (!err && (hdmi_saved_status & HPD_SIGNAL_STATUS) !=
	    (hdmi_status & HPD_SIGNAL_STATUS))
		hpd_notify_um(drm_dev);
err1:
	return ret;
}

/**
 *  msic_probe
 */
static int __devinit msic_probe(struct pci_dev *pdev,
			const struct pci_device_id *ent)
{
	struct drm_device *dev = hdmi_priv ? hdmi_priv->dev : 0;
	struct drm_psb_private *dev_priv = dev ? psb_priv(dev) : 0;
	int ret = 0;

	if (pdev->device != MSIC_PCI_DEVICE_ID) {
		DRM_ERROR("%s: pciid = 0x%x is not msic_hdmi pciid.\n",
			  __func__, pdev->device);
		goto err1;
	}

	/* enable msic hdmi device */
	ret = pci_enable_device(pdev);
	if (ret) {
		DRM_ERROR("%s: Enable pci device failed. ret = 0x%x.\n",
			  __func__, ret);
		goto err1;
	}

	sram_vreg_addr = ioremap_nocache(SRAM_MSIC_VRINT_ADDR, 0x2);
	if (sram_vreg_addr == NULL) {
		DRM_ERROR("%s: Memory map failed", __func__);
		ret = -ENOMEM;
		goto err2;
	}

	if (dev_priv == NULL) {
		DRM_ERROR("%s: Invalid parameter", __func__);
		ret = -EINVAL;
		goto err3;
	}

	dev_priv->hpd_detect = create_freezable_workqueue("hpd_detect");
	if (dev_priv->hpd_detect == NULL) {
		DRM_ERROR("%s: Creating workqueue failed", __func__);
		ret = -ENOMEM;
		goto err3;
	}

	ret = request_irq(pdev->irq, msic_vreg_handler, IRQF_SHARED,
			  "msic_hdmi_driver", (void *)&hdmi_priv);
	if (ret) {
		DRM_ERROR("%s: request_irq failed. ret = 0x%x.\n",
			  __func__, ret);
		goto err4;
	}

	return ret;

err4:
	destroy_workqueue(dev_priv->hpd_detect);
err3:
	iounmap(sram_vreg_addr);
err2:
	pci_disable_device(pdev);
err1:
	pci_dev_put(pdev);

	return ret;
}

static const struct dev_pm_ops msic_pm_ops = {
	.suspend = msic_suspend,
	.resume = msic_resume,
};

static struct pci_device_id msic_pci_id_list[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, MSIC_PCI_DEVICE_ID) },
	{ 0 }
};

/*MODULE_DEVICE_TABLE(pci, msic_pci_id_list);*/


/* field for registering driver to PCI device */
static struct pci_driver msic_pci_driver = {
	.name = "msic_hdmi_driver",
	.id_table = msic_pci_id_list,
	.probe = msic_probe,
	.driver.pm = &msic_pm_ops,
};

/**
 *  msic_regsiter_driver - register the msic hdmi device to PCI system.
 */
int msic_regsiter_driver(void)
{
	return pci_register_driver(&msic_pci_driver);
}

/**
 *  msic_unregsiter_driver - unregister the msic hdmi device from PCI system.
 */
int msic_unregister_driver(void)
{
	if (!sram_vreg_addr) {
		iounmap(sram_vreg_addr);
		sram_vreg_addr = 0;
	}
	pci_unregister_driver(&msic_pci_driver);
	return 0;
}
