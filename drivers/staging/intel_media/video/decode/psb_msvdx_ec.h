/**************************************************************************
 *
 * Copyright (c) 2012 Intel Corporation, Hillsboro, OR, USA
 * Copyright (c) Imagination Technologies Limited, UK
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Authors:
 *    Li Zeng <li.zeng@intel.com>
 *
 **************************************************************************/

#ifdef CONFIG_DRM_MRFLD
#define _PSB_MSVDX_EC_H_

#define MSVDX_CMDS_BASE 0x1000

#define MSVDX_CMDS_END_SLICE_PICTURE_OFFSET (0x0404)

#define MSVDX_CORE_CR_MSVDX_COMMAND_SPACE_OFFSET (0x0028)

#define MSVDX_CORE_BASE	(0x600)

void psb_msvdx_update_frame_info(struct msvdx_private *msvdx_priv,
					struct ttm_object_file *tfile,
					void *cmd);
void psb_msvdx_backup_cmd(struct msvdx_private *msvdx_priv,
				struct ttm_object_file *tfile,
				void *cmd,
				uint32_t cmd_size,
				uint32_t deblock_cmd_offset);

void psb_msvdx_mtx_message_dump(struct drm_device *dev);
void psb_msvdx_do_concealment(struct work_struct *work);
#endif
