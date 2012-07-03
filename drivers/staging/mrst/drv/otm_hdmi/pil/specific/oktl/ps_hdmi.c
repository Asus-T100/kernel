/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2011 Intel Corporation. All rights reserved.

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

#include <asm/io.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include "otm_hdmi.h"
#include "ipil_hdmi.h"

#define PS_MSIC_PCI_DEVICE_ID 0x080d

otm_hdmi_ret_t ps_hdmi_pci_dev_init(void *context, struct pci_dev *pvdev)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	int result = 0;
	struct pci_dev *pdev = NULL;
	uint32_t pci_address = 0;
	uint8_t pci_dev_revision = 0;
	hdmi_context_t *ctx = NULL;

	if (context == NULL) {
		rc = OTM_HDMI_ERR_INTERNAL;
		goto exit;
	}

	pr_debug("pci_get_device for 0x%x\n", PS_MSIC_PCI_DEVICE_ID);
	pdev = pci_get_device(PCI_VENDOR_INTEL, PS_MSIC_PCI_DEVICE_ID,
					pdev);
	if (pdev == NULL) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}
	pr_debug("pci_enable_device for 0x%x\n",
						PS_MSIC_PCI_DEVICE_ID);
	result = pci_enable_device(pdev);
	if (result) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}
	pr_debug("IRQ number assigned = %d\n", pdev->irq);
	ctx->irq_number = pdev->irq;

	result =
		pci_read_config_dword(pdev, (4 * PCI_RESOURCE_REGISTERS) + 16,
								&pci_address);
	if (result != 0) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}

	/* Map IO region and save its length */
	ctx->io_length = PCI_LENGTH_HDMI;
	ctx->io_address = ioremap_cache(pci_address, ctx->io_length);
	if (!ctx->io_address) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}

	/* Same IO reqion mapping for irq and other registers */
	ctx->dev.irq_io_address = ctx->io_address;

	result = pci_read_config_byte(pdev, 8, &pci_dev_revision);
	if (result != 0) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}
	ctx->dev.id = pci_dev_revision;

exit:
	LOG_EXIT(PD_LOG_LEVEL_HIGH, rc);
	return rc;
}

otm_hdmi_ret_t ps_hdmi_pci_dev_deinit(void *context)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	int result = 0;
	struct pci_dev *msic_pdev = NULL;
	hdmi_context_t *ctx = NULL;

	if (context == NULL) {
		rc = OTM_HDMI_ERR_INTERNAL;
		goto exit;
	}
	ctx = (hdmi_context_t *)context;

	/* unmap IO region */
	iounmap(ctx->io_address) ;

	msic_pdev = pci_get_device(PCI_VENDOR_INTEL,
				PS_MSIC_PCI_DEVICE_ID, msic_pdev);
	if (msic_pdev == NULL) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}
	result = pci_disable_device(msic_pdev);
	if (result) {
		rc = OTM_HDMI_ERR_FAILED;
		goto exit;
	}

exit:
	return rc;
}

static bool __hdmi_verify_block_checksum(uint8_t *data, uint16_t length)
{
	uint8_t checksum = 0;
	uint16_t i = 0;

	if (!data || length == 0)
		return false;

	for (i = 0; i < length; i++)
		checksum += data[i];

	if (checksum)
		return false;
	else
		return true;
}


otm_hdmi_ret_t ps_hdmi_i2c_edid_read(void *ctx, unsigned int sp,
				  unsigned int offset, void *buffer,
				  unsigned int size)
{
	return OTM_HDMI_SUCCESS;
}

/**
 *	ps_hdmi_read_edid	-	read edid information
 *
 *	Read edid informantion
 *
 *	Returns - Null or buffer contains edid informnation
 */
unsigned char *ps_hdmi_read_edid(void)
{
	unsigned char *buffer = NULL;
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	int exts = 0;
	uint8_t retry_count = 0;
	uint8_t i = 0;

	if (g_context == NULL)
		goto exit;

retry:
	buffer = (unsigned char *)kmalloc(SEGMENT_SIZE, GFP_KERNEL);
	if (buffer == NULL)
		goto exit;

	/* Read 1st Block */
	rc = ps_hdmi_i2c_edid_read(g_context, 0, 0, buffer, SEGMENT_SIZE);
	if (rc != OTM_HDMI_SUCCESS)
		goto exit;

	exts = buffer[0x7e];

	/* Verify: what if more than 4 blocks - return NULL? */
	if (exts > 3)
		goto exit;

	if (exts != 0 && exts < 4) {
		/* support max 4 blocks */
		unsigned char *new_buffer;
		new_buffer = krealloc(buffer, (1+exts) * SEGMENT_SIZE,
					GFP_KERNEL);
		/* realloc failed - free buffer */
		if (new_buffer == NULL)
			goto exit;

		/* Update buffer pointer */
		buffer = new_buffer;

		/* read second block */
		rc = ps_hdmi_i2c_edid_read(g_context, 0, SEGMENT_SIZE,
					buffer + SEGMENT_SIZE, SEGMENT_SIZE);
		if (rc != OTM_HDMI_SUCCESS)
			goto exit;

		/* Update segment pointer for next extension blocks */
		if (exts > 1) {
			rc = ps_hdmi_i2c_edid_read(g_context, 1, 0,
						buffer + 2*SEGMENT_SIZE,
						(exts-1) * SEGMENT_SIZE);
			if (rc != OTM_HDMI_SUCCESS)
				goto exit;
		}
	}

	 for (i = 0; i < (exts+1); i++) {
		bool value =
			__hdmi_verify_block_checksum(buffer + i * SEGMENT_SIZE,
							SEGMENT_SIZE);
		if (value == false) {
			pr_debug("Edid Block %d Checksum Failed\n", i);
			if (retry_count < 3) {
				kfree(buffer);
				retry_count++;
				goto retry;
			}
		}
	}

	return buffer;

exit:
	if (buffer)
		kfree(buffer);
	return NULL;

}
