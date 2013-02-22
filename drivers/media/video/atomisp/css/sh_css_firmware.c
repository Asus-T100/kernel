/*
* Support for Medfield PNW Camera Imaging ISP subsystem.
*
* Copyright (c) 2010 Intel Corporation. All Rights Reserved.
*
* Copyright (c) 2010 Silicon Hive www.siliconhive.com.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License version
* 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
* 02110-1301, USA.
*
*/

#include "sh_css_hrt.h"
#include "sh_css_defs.h"
#include "sh_css_firmware.h"
#include "sh_css_internal.h"

/* One extra for the sp binary */
#define NUM_BINARIES (SH_CSS_BINARY_NUM_IDS+1)

struct sh_css_sp_fw sh_css_sp_fw;
struct sh_css_blob_info sh_css_blob_info[SH_CSS_BINARY_NUM_IDS];

/*
 * Split the loaded firmware into blobs
 */

/* Setup sp binary */
static void
setup_sp(struct sh_css_fw_bi_h *fw, const char *fw_data)
{
	const char *blob_data;
	unsigned int blob_size;

	blob_data = fw_data + fw->offset;
	blob_size = fw->size;

	sh_css_sp_fw.text        = blob_data + fw->text_source;
	sh_css_sp_fw.text_size   = fw->text_size;

	sh_css_sp_fw.data        = blob_data + fw->data_source;
	sh_css_sp_fw.data_target = fw->data_target;
	sh_css_sp_fw.data_size   = fw->data_size;

	sh_css_sp_fw.bss_target  = fw->bss_target;
	sh_css_sp_fw.bss_size    = fw->bss_size;
}

enum sh_css_err
sh_css_load_firmware(const char *fw_data,
		     unsigned int fw_size)
{
	int i, num_binaries;
	struct sh_css_fw_bi_h *binaries;
	struct sh_css_fw_bi_file_h *file_header;

	file_header = (struct sh_css_fw_bi_file_h *)fw_data;
	binaries = (struct sh_css_fw_bi_h *)(&file_header[1]);

	/* some sanity checks */
	if (!fw_data || fw_size < sizeof(struct sh_css_fw_bi_file_h))
		return sh_css_err_internal_error;

	if (file_header->h_size != sizeof(struct sh_css_fw_bi_file_h))
		return sh_css_err_internal_error;

	num_binaries = file_header->binary_nr;

	for (i = 0; i < num_binaries; i++) {
		struct sh_css_fw_bi_h *bi = &binaries[i];

		/* sanity check */
		if (bi->size != bi->text_size + bi->data_size)
			return sh_css_err_internal_error;

		if (bi->offset + bi->size > fw_size)
			return sh_css_err_internal_error;

		if (bi->id == 0) {
			setup_sp(bi, fw_data);
		} else if (bi->id > 0) {
			const unsigned char *blob =
				(const unsigned char *)fw_data + bi->offset;
			sh_css_blob_info[bi->id-1].blob = blob;
			sh_css_blob_info[bi->id-1].size = bi->size;
		}
	}
	return sh_css_success;
}

void *
sh_css_load_blob(const unsigned char *blob, unsigned size)
{
	void *target_addr = hrt_isp_css_mm_alloc(size);
	/* this will allocate memory aligned to a DDR word boundary which
	   is required for the CSS DMA to read the instructions. */
	hrt_isp_css_mm_store(target_addr, blob, size);
	if (SH_CSS_PREVENT_UNINIT_READS) {
		unsigned i;
		unsigned padded_size = CEIL_MUL(size, HIVE_ISP_DDR_WORD_BYTES);
		for (i = 0; i < padded_size - size; i++)
			hrt_isp_css_mm_store_char(target_addr + size + i, 0);
	}
	return target_addr;
}
