/*
 * drivers/misc/intel_fabricid_def.h
 *
 * Copyright (C) 2011 Intel Corp
 * Author: winson.w.yung@intel.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef __INTEL_FABRICID_DEF_H
#define __INTEL_FABRICID_DEF_H

#define FAB_ID_FULLCHIP					0
#define FAB_ID_AUDIO					1
#define FAB_ID_SECONDARY				2
#define FAB_ID_GP					3
#define FAB_ID_SC					4
#define FAB_ID_UNKNOWN					5

char *fabric_error_lookup(u32 fab_id, u32 error_index, int use_hidword);

#endif /* __INTEL_FABRICID_DEF_H */
