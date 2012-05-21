#ifndef MRFLD_S3D_OVL_H
#define MRFLD_S3D_OVL_H
/*********************************
 *  Copyright (c) 2011, Intel Corporation.
 *
 *  This  program  is free software;  you can redistribute it  and/or
 *  modify  it  under the  terms  and conditions of the  GNU  General
 *  Public License,  version  2,  as published  by the  Free Software
 *  Foundation.
 *
 *  This program  is distributed in the  hope it will be useful,  but
 *  WITHOUT  ANY  WARRANTY;  without  even the  implied  warranty  of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along  with  this program;  if not,  write  to the  Free Software
 *  Foundation,  Inc.,  51  Franklin St  -  Fifth Floor,  Boston,  MA
 *  02110-1301 USA.
 *
 *  Authors:
 *      William Schmidt <williamx.f.schmidt@intel.com>
 */
#include <drm/drmP.h>
#include "psb_drm.h"
#include "psb_drv.h"
#include "psb_intel_reg.h"

/*********************************
 *           T Y P E S  A N D  D E F I N I T I O N S
 ********************************/

    /*****************************
     *  Ubiquitous listing of the overlays...
     */
typedef enum tagS3DOVERLAY {
	OVA,			/*  overlay A, which is left    */
	OVC,			/*  overlay C, which is right   */
	OVCOUNT			/*  overlay count               */
} ovl_s3d_t;

    /*****************************
     *  Defines the left and right s3d destinations.
     */
#define OVLEFT      OVA		/*  pseudonym                   */
#define OVRIGHT     0VC		/*  pseudonym                   */

    /*****************************
     *  These fill in the gaps  from 'psb_intel_reg.h' for offsets to
     *      other overlay registers.
     */
#define OV_BUF0Y    0x30100	/*  buffer 0 Y linear offset    */
#define OV_BUF1Y    0x30104	/*  buffer 1 Y linear offset    */
#define OV_BUF0U    0x30108	/*  buffer 0 U linear offset    */
#define OV_BUF0V    0x3010C	/*  buffer 0 V linear offset    */
#define OV_BUF1U    0x30110	/*  buffer 1 U linear offset    */
#define OV_BUF1V    0x30114	/*  buffer 1 V linear offset    */
#define OV_STRIDE   0x30118	/*  video data stride           */
#define OV_YRGBVPH  0x3011C	/*  Y/RGB vertical phase        */
#define OV_UVVPH    0x30120	/*  UV vertical phase           */
#define OV_HORIZPH  0x30124	/*  horizontal phase            */
#define OV_INITPSH  0x30128	/*  initial phase shift         */
#define OV_DWINPOS  0x3012C	/*  destination window position */
#define OV_DWINSZ   0x30130	/*  destination window size     */
#define OV_SWIDTH   0x30134	/*  source width                */
#define OV_SWIDTHSW 0x30138	/*  source SWORD width          */
#define OV_SHEIGHT  0x3013C	/*  source height               */
#define OV_YRGBSCL  0x30140	/*  Y/RGB scale factor          */
#define OV_UVSCL    0x30144	/*  UV scale factor             */
#define OV_CLRC0    0x30148	/*  color correction 0          */
#define OV_CLRC1    0x3014C	/*  color correction 1          */
#define OV_DCLRKV   0x30150	/*  destination color key       */
#define OV_DCLRKM   0x30154	/*  destination color mask      */
#define OV_SCHRKVH  0x30158	/*  source chroma key high      */
#define OV_SCHRKVL  0x3015C	/*  source chroma key low       */
#define OV_SCHRKEN  0x30160	/*  source chroma key enable    */
#define OV_CONFIG   0x30164	/*  configuration               */
#define OV_CMD      0x30168	/*  command                     */
#define OV_START0Y  0x30170	/*  buffer 0 Y base address     */
#define OV_START1Y  0x30174	/*  buffer 1 Y base address     */
#define OV_START0U  0x30178	/*  buffer 0 U base address     */
#define OV_START0V  0x3017C	/*  buffer 0 V base address     */
#define OV_START1U  0x30180	/*  buffer 1 U base address     */
#define OV_START1V  0x30184	/*  buffer 1 V base address     */
#define OV_TILE0Y   0x30188	/*  buffer 0 Y tile offset      */
#define OV_TILE1Y   0x3018C	/*  buffer 1 Y tile offset      */
#define OV_TILE0U   0x30190	/*  buffer 0 U tile offset      */
#define OV_TILE0V   0x30194	/*  buffer 0 V tile offset      */
#define OV_TILE1U   0x30198	/*  buffer 1 U tile offset      */
#define OV_TILE1V   0x3019C	/*  buffer 1 V tile offset      */
#define OV_FASTHSC  0x301A0	/*  fast horizontal downscale   */
#define OV_UVSCALE  0x301A4	/*  UV vertical downscale       */
#define OV_UVHCOEF  0x30700	/*  UV horizontal filter coef.  */

    /*****************************
     *  These read and/or write values to one or the other overlay as
     *      determined by the 'ov' parameter.
     */
#define OVLWR(ov, rg, val)    \
    PSB_WVDC32((val), ((ov) == OVA ? (rg) : (rg) + OV_C_OFFSET))
#define OVLRD(ov, rg)        \
    PSB_RVDC32((ov) == OVA ? (rg) : (rg) + OV_C_OFFSET)

    /*****************************
     *  These symbolically relate  a register to an array position of
     *      all  writable overlay registers.  In the  private overlay
     *      driver record, these arrays are in the 'save' field.
     */
typedef enum tagS3DOVERLAYREGINDEX {
	IDX_OVADD,		/*  overlay address register    */
	IDX_OGAMC5,		/*  gamma correction 5          */
	IDX_OGAMC4,		/*  gamma correction 4          */
	IDX_OGAMC3,		/*  gamma correction 3          */
	IDX_OGAMC2,		/*  gamma correction 2          */
	IDX_OGAMC1,		/*  gamma correction 1          */
	IDX_OGAMC0,		/*  gamma correction 0          */
	IDX_BUF0Y,		/*  buffer 0 Y linear offset    */
	IDX_BUF1Y,		/*  buffer 1 Y linear offset    */
	IDX_BUF0U,		/*  buffer 0 U linear offset    */
	IDX_BUF0V,		/*  buffer 0 V linear offset    */
	IDX_BUF1U,		/*  buffer 1 U linear offset    */
	IDX_BUF1V,		/*  buffer 1 V linear offset    */
	IDX_STRIDE,		/*  video data stride           */
	IDX_YRGBVPH,		/*  Y/RGB vertical phase        */
	IDX_UVVPH,		/*  UV vertical phase           */
	IDX_HORIZPH,		/*  horizontal phase            */
	IDX_INITPSH,		/*  initial phase shift         */
	IDX_DWINPOS,		/*  destination window position */
	IDX_DWINSZ,		/*  destination window size     */
	IDX_SWIDTH,		/*  source width                */
	IDX_SWIDTHSW,		/*  source SWORD width          */
	IDX_SHEIGHT,		/*  source height               */
	IDX_YRGBSCL,		/*  Y/RGB scale factor          */
	IDX_UVSCL,		/*  UV scale factor             */
	IDX_CLRC0,		/*  color correction 0          */
	IDX_CLRC1,		/*  color correction 1          */
	IDX_DCLRKV,		/*  destination color key       */
	IDX_DCLRKM,		/*  destination color mask      */
	IDX_SCHRKVH,		/*  source chroma key high      */
	IDX_SCHRKVL,		/*  source chroma key low       */
	IDX_SCHRKEN,		/*  source chroma key enable    */
	IDX_CONFIG,		/*  configuration               */
	IDX_CMD,		/*  command                     */
	IDX_START0Y,		/*  buffer 0 Y base address     */
	IDX_START1Y,		/*  buffer 1 Y base address     */
	IDX_START0U,		/*  buffer 0 U base address     */
	IDX_START0V,		/*  buffer 0 V base address     */
	IDX_START1U,		/*  buffer 1 U base address     */
	IDX_START1V,		/*  buffer 1 V base address     */
	IDX_TILE0Y,		/*  buffer 0 Y tile offset      */
	IDX_TILE1Y,		/*  buffer 1 Y tile offset      */
	IDX_TILE0U,		/*  buffer 0 U tile offset      */
	IDX_TILE0V,		/*  buffer 0 V tile offset      */
	IDX_TILE1U,		/*  buffer 1 U tile offset      */
	IDX_TILE1V,		/*  buffer 1 V tile offset      */
	IDX_FASTHSC,		/*  fast horizontal downscale   */
	IDX_UVSCALE,		/*  UV vertical downscale       */
	IDX_UVHCOEF,		/*  UV horizontal filter coef.  */
	NUM_IDX			/*  writable register count     */
} ovl_s3d_index_t;

#define NUM_WR      (NUM_IDX + 63)

    /*****************************
     *  ovl_s3d_config_t  -  private (driver only) version of the s3d
     *      overlay configuration.  The first portion of this  is the
     *      same as the fields in the 'ovl_s3d_config_t' record found
     *      in 'psb_drm.h'. The remainder is for driver housekeeping.
     */
typedef struct tagPRIVATEOVLS3DCONFIGURATION {
	unsigned int valid;	/*  valid fields bitmask        */
	int id,			/*  acquisition identifier      */
	 sw, sh;		/*  source geometry             */
	enum ovlmode_t mode;		/*  s3d overlay mode to set     */
	enum ovlpixel_t pixel;	/*  surface pixel format        */
	enum ovlpipe_t pipe;		/*  destination pipe            */
	unsigned int mmlen,	/*  buffer size in bytes        */
	 stride,		/*  horizontal stride in bytes  */
	 uvstride,		/*  UV stride in planar modes   */
	 yoffset,		/*  Y planar location           */
	 uoffset,		/*  U planar location           */
	 voffset;		/*  V planar location           */
	int sx, sy,		/*  pan origin                  */
	 x, y,			/*  requested position          */
	 w, h;			/*  requested geometry          */
	uint32_t lbuf,		/*  left buffer                 */
	 rbuf;			/*  right buffer                */
	int enabled,		/*  current state flag          */
	 lx, ly,		/*  left position               */
	 rx, ry,		/*  right position              */
	 vw, vh,		/*  video geometry              */
	 dw, dh,		/*  display width, height       */
	 iscale,		/*  integer portion of scale    */
	 fscale;		/*  fractional portion of scale */
	uint32_t save[2][NUM_WR],	/*  saved register states       */
	 work[2][NUM_WR];	/*  working copy                */
} ovl_s3d_priv_t, *ovl_s3d_priv_p;

    /*****************************
     *  These macros  are compliments  to the  configuration routines
     *      below.  In order to use them correctly, a call to 'mrfld_
     *      ovl_s3d_save' must have been completed prior to doing the
     *      configuration!
     */
#define mrfld_s3d_ovl_from_line_interleave(d, p)         \
    mrfld_s3d_ovl_restore((d), (p))

#define mrfld_s3d_ovl_from_line_interleave_half(d, p)    \
    mrfld_s3d_ovl_restore((d), (p))

#define mrfld_s3d_ovl_from_pixel_interleave(d, p)        \
    mrfld_s3d_ovl_restore((d), (p))

#define mrfld_s3d_ovl_from_pixel_interleave_half(d, p)   \
    mrfld_s3d_ovl_restore((d), (p))

#define mrfld_s3d_ovl_from_sidebyside(d, p)              \
    mrfld_s3d_ovl_restore((d), (p))

#define mrfld_s3d_ovl_from_sidebyside_half(d, p)         \
    mrfld_s3d_ovl_restore((d), (p))

#define mrfld_s3d_ovl_from_topbottom(d, p)               \
    mrfld_s3d_ovl_restore((d), (p))

#define mrfld_s3d_ovl_from_topbottom_half(d, p)          \
    mrfld_s3d_ovl_restore((d), (p))

/*********************************
 *                  M O D U L E  P U B L I C S
 ********************************/

/*********************************
 *  mrfld_s3d_ovl_line_interleave(  struct drm_device  *dev,
 *                                  ovl_s3d_priv_p      config)
 *
 *  Purpose :   Place both overlays  in line interleave mode  for S3D
 *              video.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   Pointer (config) to the S3D configuration.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_line_interleave(struct drm_device *, ovl_s3d_priv_p);

/*********************************
 *  mrfld_s3d_ovl_line_interleave_half( struct drm_device  *dev,
 *                                      ovl_s3d_priv_p      config)
 *
 *  Purpose :   Place both overlays in half line interleave mode  for
 *              S3D video.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   Pointer (config) to the S3D configuration.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_line_interleave_half(struct drm_device *,
					      ovl_s3d_priv_p);

/*********************************
 *  mrfld_s3d_ovl_pixel_interleave( struct drm_device  *dev,
 *                                  ovl_s3d_priv_p      config)
 *
 *  Purpose :   Place both overlays in pixel interleave mode  for S3D
 *              video.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   Pointer (config) to the S3D configuration.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_pixel_interleave(struct drm_device *, ovl_s3d_priv_p);

/*********************************
 *  mrfld_s3d_ovl_pixel_interleave_half(struct drm_device  *dev,
 *                                      ovl_s3d_priv_p      config)
 *
 *  Purpose :   Place both overlays in half pixel interleave mode for
 *              S3D video.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   Pointer (config) to the S3D configuration.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_pixel_interleave_half(struct drm_device *,
					       ovl_s3d_priv_p);

/*********************************
 *  mrfld_s3d_ovl_sidebyside(   struct drm_device  *dev,
 *                              ovl_s3d_priv_p      config)
 *
 *  Purpose :   Place  both overlays in  side-by-side  mode  for  S3D
 *              video.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   Pointer (config) to the S3D configuration.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_sidebyside(struct drm_device *, ovl_s3d_priv_p);

/*********************************
 *  mrfld_s3d_ovl_sidebyside_half(  struct drm_device  *dev,
 *                                  ovl_s3d_priv_p      config)
 *
 *  Purpose :   Place both overlays in half side-by-sied mode for S3D
 *              video.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   Pointer (config) to the S3D configuration.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_sidebyside_half(struct drm_device *, ovl_s3d_priv_p);

/*********************************
 *  mrfld_s3d_ovl_topbottom(struct drm_device  *dev,
 *                          ovl_s3d_priv_p      config)
 *
 *  Purpose :   Place both overlays in top-bottom mode for S3D video.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   Pointer (config) to the S3D configuration.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_topbottom(struct drm_device *, ovl_s3d_priv_p);

/*********************************
 *  mrfld_s3d_ovl_topbottom_half(   struct drm_device  *dev,
 *                                  ovl_s3d_priv_p      config)
 *
 *  Purpose :   Place both overlays  in half top-bottom mode  for S3D
 *              video.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   Pointer (config) to the S3D configuration.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_topbottom_half(struct drm_device *, ovl_s3d_priv_p);

/*********************************
 *  mrfld_s3d_ovl_update(struct drm_device *dev,int en)
 *
 *  Purpose :   Update the video image and/or position for an overlay
 *              pair.
 *  Input   :   Pointer (dev) to the  device record, pointer (config)
 *              to the private configuration record,  and the pointer
 *              (update) to the update parameters.
 *  Output  :   None.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_update(struct drm_device *,
				ovl_s3d_priv_p, ovl_s3d_update_p);

/*********************************
 *  mrfld_s3d_ovl_save( struct drm_device  *dev,
 *                      ovl_s3d_priv_p      config)
 *
 *  Purpose :   Save whatever configuration is present at the time of
 *              this call.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   None.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_save(struct drm_device *, ovl_s3d_priv_p);

/*********************************
 *  mrfld_s3d_ovl_restore(  struct drm_device  *dev,
 *                          ovl_s3d_priv_p      config)
 *
 *  Purpose :   Restore whatever configuration was present before the
 *              S3D video mode was entered.
 *  Input   :   Pointer  (dev) to the  device record  and the pointer
 *              (config) to the properly populated S3D configuration.
 *  Output  :   None.
 *  Return  :   Zero on success or a negative error code.
 */
extern int mrfld_s3d_ovl_restore(struct drm_device *, ovl_s3d_priv_p);

extern int mrfld_s3d_ovl_regrw(struct drm_device *, ovl_s3d_reg_p);

#endif				/* MRFLD_S3D_OVL_H */
