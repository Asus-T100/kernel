/**************************************************************************
 * Copyright (c) 2007, Intel Corporation.
 * All Rights Reserved.
 * Copyright (c) 2008, Tungsten Graphics Inc.  Cedar Park, TX., USA.
 * All Rights Reserved.
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
 **************************************************************************/

#ifndef _DRM_SHARED_H_
#define _DRM_SHARED_H_

/*
 * Menlow/MRST graphics driver package version
 * a.b.c.xxxx
 * a - Product Family: 5 - Linux
 * b - Major Release Version: 0 - non-Gallium (Unbuntu)
 *                            1 - Gallium (Moblin2)
 *                            2 - IMG     (Moblin2)
 *                            3 - IMG     (Meego)
 *                            4 - IMG     (Android)
 * c - Hotfix Release
 * xxxx - Graphics internal build #
 */
#define PSB_PACKAGE_VERSION "5.4.0.1020"

#define DRM_PSB_SAREA_MAJOR 0
#define DRM_PSB_SAREA_MINOR 2

/* Controlling the kernel modesetting buffers */

#define DRM_PSB_KMS_OFF		0x00
#define DRM_PSB_KMS_ON		0x01
#define DRM_PSB_VT_LEAVE        0x02
#define DRM_PSB_VT_ENTER        0x03
#define DRM_PSB_EXTENSION       0x06
#define DRM_PSB_SIZES           0x07
#define DRM_PSB_FUSE_REG	0x08
#define DRM_PSB_VBT		0x09
#define DRM_PSB_DC_STATE	0x0A
#define DRM_PSB_ADB		0x0B
#define DRM_PSB_MODE_OPERATION	0x0C
#define DRM_PSB_STOLEN_MEMORY	0x0D
#define DRM_PSB_REGISTER_RW	0x0E
#define DRM_PSB_GTT_MAP         0x0F
#define DRM_PSB_GTT_UNMAP       0x10
#define DRM_PSB_GETPAGEADDRS	0x11
/**
 * NOTE: Add new commands here, but increment
 * the values below and increment their
 * corresponding defines where they're
 * defined elsewhere.
 */
#define DRM_PVR_RESERVED1	0x12
#define DRM_PVR_RESERVED2	0x13
#define DRM_PVR_RESERVED3	0x14
#define DRM_PVR_RESERVED4	0x15
#define DRM_PVR_RESERVED5	0x16

#define DRM_PSB_HIST_ENABLE	0x17
#define DRM_PSB_HIST_STATUS	0x18
#define DRM_PSB_UPDATE_GUARD	0x19
#define DRM_PSB_INIT_COMM	0x1A
#define DRM_PSB_DPST		0x1B
#define DRM_PSB_GAMMA		0x1C
#define DRM_PSB_DPST_BL		0x1D

#define DRM_PVR_RESERVED6	0x1E

#define DRM_PSB_GET_PIPE_FROM_CRTC_ID 0x1F
#define DRM_PSB_DPU_QUERY 0x20
#define DRM_PSB_DPU_DSR_ON 0x21
#define DRM_PSB_DPU_DSR_OFF 0x22
#define DRM_PSB_HDMI_FB_CMD 0x23

/* HDCP IOCTLs */
#define DRM_PSB_QUERY_HDCP		0x24
#define DRM_PSB_VALIDATE_HDCP_KSV	0x25
#define DRM_PSB_GET_HDCP_STATUS		0x26
#define DRM_PSB_ENABLE_HDCP		0x27
#define DRM_PSB_DISABLE_HDCP		0x28
#define DRM_PSB_GET_HDCP_LINK_STATUS	0x29

/* S3D IOCTLs */
#define DRM_PSB_S3D_QUERY               0x2A
#define DRM_PSB_S3D_PREMODESET          0x2B
#define DRM_PSB_S3D_ENABLE              0x2C

/* CSC IOCTLS */
#define DRM_PSB_SET_CSC                 0x2D

    /*****************************
     *  BEGIN S3D OVERLAY
     ****************************/

    /*****************************
     *  S3D OVERLAY IOCTLs
     *
     *      DRM_OVL_S3D_ACQUIRE
     *          Gain access to the driver allowing use of the overlay
     *          s3d features  for video display.  Returns a  non-zero
     *          identifier in the value pointed to by 'data' and zero
     *          on success or a negative error code.
     *
     *      DRM_OVL_S3D_RELEASE
     *          When passed a valid non-zero identifier, releases the
     *          exclusive access to the overlay s3d features. Returns
     *          zero on success or a negative error.
     *
     *      DRM_OVL_S3D_UPDATE
     *          Perform an update in accordance with a populated ovl_
     *          s3d_update_t  record.  Returns  zero on success  or a
     *          negative error.
     *
     *      DRM_OVL_S3D_CONFIG
     *          Accepts a configuration record describing the desired
     *          s3d overlay setup,  and  performs the setup.  Returns
     *          zero on success or a negative error code.
     */
#define DRM_OVL_S3D_ACQUIRE             0x2E
#define DRM_OVL_S3D_RELEASE             0x2F
#define DRM_OVL_S3D_UPDATE              0x30
#define DRM_OVL_S3D_CONFIG              0x31

    /*****************************
     *  This is for debugging and troubleshooting on the overlay reg-
     *      isters; it may go away someday.  Individual registers may
     *      be read and/or written.
     */
#define DRM_OVL_S3D_OVLREG              0x32

#define DRM_PSB_DSR_ENABLE	0xfffffffe
#define DRM_PSB_DSR_DISABLE	0xffffffff

#endif
