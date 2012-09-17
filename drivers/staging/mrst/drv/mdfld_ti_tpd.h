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
#ifndef __MDFLD_TI_TPD_H__
#define __MDFLD_TI_TPD_H__

#include <drm/drmP.h>

#define TI_TPD_PCI_DEVICE_ID 0x901
#define CLV_TI_HPD_GPIO_PIN 43
#define CLV_HDMI_LS_OE_GPIO_PIN 91

int ti_tpd_regsiter_driver(void);
int ti_tpd_unregister_driver(void);
extern void hpd_notify_um(struct drm_device *dev);

#endif /* __MDFLD_TI_TPD_H__ */
