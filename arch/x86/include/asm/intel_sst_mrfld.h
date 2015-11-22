/* intel_sst_mrlfd.h - Common enum of the Merrifield platform
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Samreen Nilofer <samreen.nilofer@intel.com>
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
#ifndef _INTEL_SST_MRFLD_H
#define _INTEL_SST_MRFLD_H

enum {
	MERR_SALTBAY_AUDIO = 0,
	MERR_SALTBAY_COMPR,
	MERR_SALTBAY_VOIP,
	MERR_SALTBAY_PROBE,
	MERR_SALTBAY_AWARE,
	MERR_SALTBAY_VAD,
	MERR_SALTBAY_POWER,
};

enum {
	MERR_DPCM_AUDIO = 0,
	MERR_DPCM_DB,
	MERR_DPCM_LL,
	MERR_DPCM_COMPR,
	MERR_DPCM_VOIP,
	MERR_DPCM_PROBE,
	MERR_DPCM_CAPTURE,
};

#endif
