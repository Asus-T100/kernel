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
#include "mrfld_s3d_ovl.h"
#include "psb_drv.h"
#include "psb_powermgmt.h"

/*********************************
 *           T Y P E S  A N D  D E F I N I T I O N S
 ********************************/

#define REG_WRTRACE
#define REG_NOSRTRACE
/* #define REG_RDTRACE */

    /*****************************
     *  Error/debug reporting
     */
#ifdef OVL_DEBUG
#define OVL_MSG     KERN_ALERT "[s3dovl] "
#else
#define OVL_MSG     KERN_ERR "[s3dovl] "
#endif

    /*****************************
     *  FIXME
     *
     *  This is a bogus number; we need to find out what the gap in
     *      top-to-bottom s3d really is.
     */
#define VGAP(conf)      20

    /*****************************
     *  FIXME
     *
     *  This should convert the handle passed in 'buf'  to an overlay
     *      64K aligned graphics memory address. Here  we assume that
     *      the handle is this address.
     */
#define GADDR(buf)      ((buf) & 0xFFFF0000)

    /*****************************
     *  interleave_t - lists the types of interleaving available with
     *      the s3d overlays.
     */
typedef enum tagINTERLEAVETYPE {
	OI_NONE,		/*  no interleaving             */
	OI_PIXEL,		/*  pixel interleaving          */
	OI_LINE,		/*  line interleaving           */
	OI_COUNT		/*  count of interleave types   */
} interleave_t;

    /*****************************
     *  scale_t  - lists the types of scaling that can be used in the
     *      s3d overlays.
     */
typedef enum tagSCALETYPE {
	OS_FULL,		/*  full scale both             */
	OS_VHALF,		/*  half vertical               */
	OS_HHALF,		/*  half horizontal             */
	OS_COUNT		/*  count of scale types        */
} scale_t;

    /*****************************
     *  These flags reside in bit positions which represent registers
     *      in the overlay which need to be accessed.  Note that some
     *      indicate  a set of registers, while others  indicate just
     *      one.
     */
#define FLG_OVADD           0x00000001
#define FLG_GAMMA           0x00000002
#define FLG_BUF             0x00000004
#define FLG_STRIDE          0x00000008
#define FLG_YRGBVPH         0x00000010
#define FLG_UVVPH           0x00000020
#define FLG_HORIZPH         0x00000040
#define FLG_INITPSH         0x00000080
#define FLG_DWINPOS         0x00000100
#define FLG_DWINSZ          0x00000200
#define FLG_SWIDTH          0x00000400
#define FLG_SWIDTHSW        0x00000800
#define FLG_SHEIGHT         0x00001000
#define FLG_YRGBSCL         0x00002000
#define FLG_UVSCL           0x00004000
#define FLG_CLRC            0x00008000
#define FLG_DCLRKV          0x00010000
#define FLG_DCLRKM          0x00020000
#define FLG_SCHRKV          0x00040000
#define FLG_SCHRKEN         0x00080000
#define FLG_CONFIG          0x00100000
#define FLG_CMD             0x00200000
#define FLG_START           0x00400000
#define FLG_TILE            0x00800000
#define FLG_FASTHSC         0x01000000
#define FLG_UVSCALE         0x02000000
#define FLG_UVHCOEF         0x04000000
#define ALL_FLAGS           0x07FFFFFF
#define NUM_FLAGS           27

    /*****************************
     *  Pseudo flag to indicate that a flip of the surfaces should be
     *      performed.
     */
#define FLG_FLIP            0x80000000

    /*****************************
     *  DISPLAY_GEO(priv,r)
     *
     *  Purpose :   Determine the screen geometry of the display.
     *  Input   :   Pointer (priv) to the overlay record.
     *  Output  :   The geometry (w, h) and the (r)esult.
     *  Return  :   Contents of  'r'  will be zero on success or con-
     *              tain a negative error code.  'R' will also be the
     *              value of the macro.
     *
     *  FIXME
     *  Note    :   This macro  is incomplete!  It  fakes  an  actual
     *              query to find the real display resolution
     */
#define DISPLAY_GEO(priv, r)                                     \
do {                                                            \
	switch (priv->pipe) {                                   \
	case OP_PIPEA:                                          \
	case OP_PIPEC:                                          \
		priv->dw = 480;                                 \
		priv->dh = 854;                                 \
		r        = 0;                                   \
		break;                                          \
	case OP_PIPEB:                                          \
		priv->dw = 1920;                                \
		priv->dh = 1080;                                \
		r        = 0;                                   \
		break;                                          \
	default:                                                \
		r = -EINVAL;                                    \
		break;                                          \
	}                                                       \
} while (0);				/* DISPLAY_GEO */

    /*****************************
     *  OVERLAY_PIPE(priv,pipe,f)
     *
     *  Purpose :   Direct each overlay to the designated pipe.
     *  Input   :   Pointer  (priv)  to the private config record and
     *              the (pipe) to use.
     *  Output  :   The  flag (f) word, which has the  necessary bits
     *              set reflecting the pipe change.
     *  Return  :   None.
     */
#define OVERLAY_PIPE(priv, pipe, f)                             \
do {                                                            \
	priv->work[OVA][IDX_OVADD] &= ~0x000000C0;              \
	priv->work[OVC][IDX_OVADD] &= ~0x000000C0;              \
	switch (pipe) {                                         \
	case OP_PIPEA:                                          \
		break;                                          \
	case OP_PIPEB:                                          \
		priv->work[OVA][IDX_OVADD] |= 0x80;             \
		priv->work[OVC][IDX_OVADD] |= 0x80;             \
		break;                                          \
	default:                                                \
		priv->work[OVA][IDX_OVADD] |= 0x40;             \
		priv->work[OVC][IDX_OVADD] |= 0x40;             \
		break;                                          \
	}                                                       \
	f |= FLG_OVADD;                                         \
} while (0);				/* OVERLAY_PIPE */

    /*****************************
     *  OVERLAY_INTERLEAVE(priv,type,f)
     *
     *  Purpose :   Setup S3D interleaving.
     *  Input   :   Pointer  (priv)  to the private config record and
     *              the (type) of interleaving to use.
     *  Output  :   The  flag (f) word, which has the  necessary bits
     *              set reflecting the pipe change.
     *  Return  :   None.
     */
#define OVERLAY_INTERLEAVE(priv, type, f)                       \
do {                                                            \
	priv->work[OVA][IDX_CMD] &= ~0x00700000;                \
	switch (type) {                                         \
	case OI_PIXEL:                                          \
		priv->work[OVA][IDX_CMD] |= 0x600000;           \
		break;                                          \
	case OI_LINE:                                           \
		priv->work[OVA][IDX_CMD] |= 0x500000;           \
		break;                                          \
	default:                                                \
		break;                                          \
	}                                                       \
	f |= FLG_CMD;                                           \
} while (0);				/* OVERLAY_INTERLEAVE */

    /*****************************
     *  VWAIT(priv)
     *
     *  Purpose :   Wait for a vertical retrace interval.
     *  Input   :   Pointer (priv) to the private s3d overlay record.
     *  Output  :   None.
     *  Return  :   No return value.
     */
#define VWAIT(priv)                                             \
do {                                                            \
	uint32_t    pstat = PIPEASTAT;                          \
	int         retry = 3000;                               \
	switch (priv->pipe) {                                   \
	case OP_PIPEA:                                          \
	default:                                                \
		break;                                          \
	case OP_PIPEB:                                          \
		pstat = PIPEBSTAT;                              \
		break;                                          \
	case OP_PIPEC:                                          \
		pstat = PIPECSTAT;                              \
		break;                                          \
	}                                                       \
	while (--retry) {                                       \
		if ((PSB_RVDC32(pstat) & PIPE_VBLANK_STATUS))   \
			break;                                  \
		udelay(10);                                     \
	}                                                       \
	if (!retry)                                             \
		printk(OVL_MSG "VWAIT timeout\n");              \
} while (0);			/* VWAIT */

    /*****************************
     *  FLIPWAIT(priv)
     *
     *  Purpose :   Wait for the s3d suface flip to complete.
     *  Input   :   Pointer (priv) to the private s3d overlay record.
     *  Output  :   None.
     *  Return  :   No return value.
     */
#define FLIPWAIT(priv)                                          \
do {                                                            \
	int retry = 3000;                                       \
	while (--retry) {                                       \
		if (OVLRD(OVA, OV_DOVASTA) & 0x80000000)        \
			break;                                  \
		udelay(10);                                     \
	}                                                       \
	if (!retry)                                             \
		printk(OVL_MSG "FLIPWAIT timeout\n");           \
} while (0);			/* FLIPWAIT */

/*********************************
 *                  M O D U L E  S T A T I C S
 ********************************/

static void rdOvl(struct drm_device *, uint32_t *, ovl_s3d_t, uint32_t);
static void wrOvl(struct drm_device *, uint32_t *, ovl_s3d_t, uint32_t);
static int scaleOvl(ovl_s3d_priv_p, ovl_s3d_t, scale_t, unsigned int *);
static int placeOvl(ovl_s3d_priv_p, ovl_s3d_t, int, int,
		    int, int, unsigned int *);
static int mapOvl(ovl_s3d_priv_p, unsigned int *);
static int surfaceOvl(ovl_s3d_priv_p, ovl_s3d_update_p, unsigned int *);
static int swidth(unsigned int, unsigned int);

/*********************************
 *                     M O D U L E  D A T A
 ********************************/

    /*****************************
     *  This table  is intended  to be accessed  by bit position when
     *      examining a  32-bit flag  field  built from  ORed  FLG_xx
     *      value combinations.  It gives the starting offset [0] and
     *      the number of registers [1] that correspond to the bit.
     */
static uint32_t m_rwtab[NUM_FLAGS][2] = {
	{OV_OVADD, 1},
	{OV_OGAMC5, 6},
	{OV_BUF0Y, 6},
	{OV_STRIDE, 1},
	{OV_YRGBVPH, 1},
	{OV_UVVPH, 1},
	{OV_HORIZPH, 1},
	{OV_INITPSH, 1},
	{OV_DWINPOS, 1},
	{OV_DWINSZ, 1},
	{OV_SWIDTH, 1},
	{OV_SWIDTHSW, 1},
	{OV_SHEIGHT, 1},
	{OV_YRGBSCL, 1},
	{OV_UVSCL, 1},
	{OV_CLRC0, 2},
	{OV_DCLRKV, 1},
	{OV_DCLRKM, 1},
	{OV_SCHRKVH, 2},
	{OV_SCHRKEN, 1},
	{OV_CONFIG, 1},
	{OV_CMD, 1},
	{OV_START0Y, 6},
	{OV_TILE0Y, 6},
	{OV_FASTHSC, 1},
	{OV_UVSCALE, 1},
	{OV_UVHCOEF, 64}
};

#if defined(REG_WRTRACE) || defined(REG_RDTRACE)

#ifdef REG_NOSRTRACE
static int saverestore;
#endif

static char *rname[] = {
	"OVADD   ", "OGAMC5  ", "OGAMC4  ", "OGAMC3  ", "OGAMC2  ",
	"OGAMC1  ", "OGAMC0  ", "BUF0Y   ", "BUF1Y   ", "BUF0U   ",
	"BUF0V   ", "BUF1U   ", "BUF1V   ", "STRIDE  ", "YRGBVPH ",
	"UVVPH   ", "HORIZPH ", "INITPSH ", "DWINPOS ", "DWINSZ  ",
	"SWIDTH  ", "SWIDTHSW", "SHEIGHT ", "YRGBSCL ", "UVSCL   ",
	"CLRC0   ", "CLRC1   ", "DCLRKV  ", "DCLRKM  ", "SCHRKVH ",
	"SCHRKVL ", "SCHRKEN ", "CONFIG  ", "CMD     ", "START0Y ",
	"START1Y ", "START0U ", "START0V ", "START1U ", "START1V ",
	"TILE0Y  ", "TILE1Y  ", "TILE0U  ", "TILE0V  ", "TILE1U  ",
	"TILE1V  ", "FASTHSC ", "UVSCALE ", "UVHCOEF "
};
#endif

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
int mrfld_s3d_ovl_line_interleave(struct drm_device *dev, ovl_s3d_priv_p config)
{
	int res;
	uint32_t flg = 0;

	/*
	 *  Collect the  current display geometry  into the  private con-
	 *  figuration record.
	 */
	DISPLAY_GEO(config, res);

	if (res != 0) {

		printk(OVL_MSG "Invalid display geometry\n");

		return res;
	}

	/*
	 *  Direct the overlays  to the requested pipe and set the inter-
	 *  leaving to line.
	 */
	OVERLAY_PIPE(config, config->pipe, flg);
	OVERLAY_INTERLEAVE(config, OI_LINE, flg);

	/*
	 *  Scale the image to fit  the destination rectangle.  Also, set
	 *  the registers that are dependant upon  the source memory map-
	 *  ping info.
	 */
	res = scaleOvl(config, OVA, OS_FULL, &flg);
	if (res != 0)
		return res;
	else {
		res = scaleOvl(config, OVC, OS_FULL, &flg);
		if (res != 0)
			return res;
	}

	res = mapOvl(config, &flg);
	if (res != 0)
		return res;

	/*
	 *  In line interleave, both overlays are placed at the requested
	 *  screen location.
	 */
	res = placeOvl(config, OVA, config->x, config->y,
			config->w, config->h, &flg);
	if (res != 0)
		return res;
	else {
		res = placeOvl(config, OVC, config->x,
				config->y, config->w, config->h, &flg);
		if (res != 0)
			return res;
	}

	/*
	 *  Make sure  that the command register's enable bit  is cleared
	 *  before writing the overlays. The enabled flag is reset.
	 */
	config->enabled = 0;
	config->lx = config->x;
	config->ly = config->y;
	config->rx = config->x;
	config->ry = config->y;
	config->dw = config->w;
	config->dh = config->h;
	config->work[OVA][IDX_CMD] &= ~0x00000001;
	config->work[OVC][IDX_CMD] &= ~0x00000001;
	flg |= FLG_CMD;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Could not access display island\n");

		return -EBUSY;
	}

	wrOvl(dev, config->work[OVA], OVA, flg);
	wrOvl(dev, config->work[OVC], OVC, flg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_line_interleave */

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
int mrfld_s3d_ovl_line_interleave_half(struct drm_device *dev,
				       ovl_s3d_priv_p config)
{
	int res;
	uint32_t flg = 0;

	/*
	 *  The selected pipe must be one of the MIPI types; HDMI won't
	 *  support line interleave half.
	 */
	if ((config->pipe != OP_PIPEA) && (config->pipe != OP_PIPEC)) {

		printk(OVL_MSG "HDMI doesn't support line interleave half\n");

		return -EINVAL;
	}

	/*
	 *  Collect the  current display geometry  into the  private con-
	 *  figuration record.
	 */
	DISPLAY_GEO(config, res);

	if (res != 0) {

		printk(OVL_MSG "Invalid display geometry\n");

		return res;
	}

	/*
	 *  Direct the overlays  to the requested pipe and set the inter-
	 *  leaving to line.
	 */
	OVERLAY_PIPE(config, config->pipe, flg);
	OVERLAY_INTERLEAVE(config, OI_LINE, flg);

	/*
	 *  Scale the image to fit  the destination rectangle.  Also, set
	 *  the registers that are dependant upon  the source memory map-
	 *  ping info.
	 */
	res = scaleOvl(config, OVA, OS_VHALF, &flg);
	if (res != 0)
		return res;
	else {
		res = scaleOvl(config, OVC, OS_VHALF, &flg);
		if (res != 0)
			return res;
	}

	res = mapOvl(config, &flg);
	if (res != 0)
		return res;

	/*
	 *  In  half line interleave, both overlays are placed at the re-
	 *  quested screen location.
	 */
	res = placeOvl(config, OVA, config->x, config->y,
			config->w, config->h >> 1, &flg);
	if (res != 0)
		return res;
	else {
		res = placeOvl(config, OVC, config->x, config->y,
				config->w, config->h >> 1, &flg);
		if (res != 0)
			return res;
	}

	/*
	 *  Make sure  that the command register's enable bit  is cleared
	 *  before writing the overlays. The enabled flag is reset.
	 */
	config->enabled = 0;
	config->lx = config->x;
	config->ly = config->y;
	config->rx = config->x;
	config->ry = config->y;
	config->dw = config->w;
	config->dh = config->h >> 1;
	config->work[OVA][IDX_CMD] &= ~0x00000001;
	config->work[OVC][IDX_CMD] &= ~0x00000001;
	flg |= FLG_CMD;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Could not access display island\n");

		return -EBUSY;
	}

	wrOvl(dev, config->work[OVA], OVA, flg);
	wrOvl(dev, config->work[OVC], OVC, flg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_line_interleave_half */

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
int mrfld_s3d_ovl_pixel_interleave(struct drm_device *dev,
				   ovl_s3d_priv_p config)
{
	int res;
	uint32_t flg = 0;

	/*
	 *  We can only use pixel interleave with the HDMI pipe B...
	 */
	if (config->pipe != OP_PIPEB) {

		printk(OVL_MSG "pixel interleave requires HDMI\n");

		return -EINVAL;
	}

	/*
	 *  Collect the  current display geometry  into the  private con-
	 *  figuration record.
	 */
	DISPLAY_GEO(config, res);

	if (res != 0) {

		printk(OVL_MSG "Invalid display geometry\n");

		return res;
	}

	/*
	 *  Direct the overlays  to the requested pipe and set the inter-
	 *  leaving to pixel.
	 */
	OVERLAY_PIPE(config, config->pipe, flg);
	OVERLAY_INTERLEAVE(config, OI_PIXEL, flg);

	/*
	 *  Scale the image to fit  the destination rectangle.  Also, set
	 *  the registers that are dependant upon  the source memory map-
	 *  ping info.
	 */
	res = scaleOvl(config, OVA, OS_FULL, &flg);
	if (res != 0)
		return res;
	else {
		res = scaleOvl(config, OVC, OS_FULL, &flg);
		if (res != 0)
			return res;
	}

	res = mapOvl(config, &flg);
	if (res != 0)
		return res;

	/*
	 *  In pixel interleave both overlays are placed at the requested
	 *  screen location.
	 */
	res = placeOvl(config, OVA, config->x, config->y,
			config->w, config->h, &flg);
	if (res != 0)
		return res;
	else {
		res = placeOvl(config, OVC, config->x,
				config->y, config->w, config->h, &flg);
		if (res != 0)
			return res;
	}

	/*
	 *  Make sure  that the command register's enable bit  is cleared
	 *  before writing the overlays. The enabled flag is reset.
	 */
	config->enabled = 0;
	config->lx = config->x;
	config->ly = config->y;
	config->rx = config->x;
	config->ry = config->y;
	config->dw = config->w;
	config->dh = config->h;
	config->work[OVA][IDX_CMD] &= ~0x00000001;
	config->work[OVC][IDX_CMD] &= ~0x00000001;
	flg |= FLG_CMD;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Could not access display island\n");

		return -EBUSY;
	}

	wrOvl(dev, config->work[OVA], OVA, flg);
	wrOvl(dev, config->work[OVC], OVC, flg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_pixel_interleave */

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
int mrfld_s3d_ovl_pixel_interleave_half(struct drm_device *dev,
					ovl_s3d_priv_p config)
{
	int res;
	uint32_t flg = 0;

	/*
	 *  We can only use pixel interleave with the HDMI pipe B...
	 */
	if (config->pipe != OP_PIPEB) {

		printk(OVL_MSG "pixel interleave half requires HDMI\n");

		return -EINVAL;
	}

	/*
	 *  Collect the  current display geometry  into the  private con-
	 *  figuration record.
	 */
	DISPLAY_GEO(config, res);

	if (res != 0) {

		printk(OVL_MSG "Invalid display geometry\n");

		return res;
	}

	/*
	 *  Direct the overlays  to the requested pipe and set the inter-
	 *  leaving to pixel.
	 */
	OVERLAY_PIPE(config, config->pipe, flg);
	OVERLAY_INTERLEAVE(config, OI_PIXEL, flg);

	/*
	 *  Scale the image to fit  the destination rectangle.  Also, set
	 *  the registers that are dependant upon  the source memory map-
	 *  ping info.
	 */
	res = scaleOvl(config, OVA, OS_VHALF, &flg);
	if (res != 0)
		return res;
	else {
		res = scaleOvl(config, OVC, OS_VHALF, &flg);
		if (res != 0)
			return res;
	}

	res = mapOvl(config, &flg);
	if (res != 0)
		return res;

	/*
	 *  In half pixel interleave, both overlays are placed at the re-
	 *  quested screen location.
	 */
	res = placeOvl(config, OVA, config->x, config->y,
			config->w, config->h >> 1, &flg);
	if (res != 0)
		return res;
	else {
		res = placeOvl(config, OVC, config->x, config->y,
			config->w, config->h >> 1, &flg);

		if (res != 0)
			return res;
	}

	/*
	 *  Make sure  that the command register's enable bit  is cleared
	 *  before writing the overlays. The enabled flag is reset.
	 */
	config->enabled = 0;
	config->lx = config->x;
	config->ly = config->y;
	config->rx = config->x;
	config->ry = config->y;
	config->dw = config->w;
	config->dh = config->h >> 1, config->work[OVA][IDX_CMD] &= ~0x00000001;
	config->work[OVC][IDX_CMD] &= ~0x00000001;
	flg |= FLG_CMD;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Could not access display island\n");

		return -EBUSY;
	}

	wrOvl(dev, config->work[OVA], OVA, flg);
	wrOvl(dev, config->work[OVC], OVC, flg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_pixel_interleave_half */

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
int mrfld_s3d_ovl_sidebyside(struct drm_device *dev, ovl_s3d_priv_p config)
{
	int res;
	uint32_t flg = 0;

	/*
	 *  Side by side is only available on HDMI...
	 */
	if (config->pipe != OP_PIPEB) {

		printk(OVL_MSG "side by side must be used with HDMI\n");

		return -EINVAL;
	}

	/*
	 *  Collect the  current display geometry  into the  private con-
	 *  figuration record.
	 */
	DISPLAY_GEO(config, res);

	if (res != 0) {

		printk(OVL_MSG "Invalid display geometry\n");

		return res;
	}

	/*
	 *  Direct the overlays  to the requested pipe and set the inter-
	 *  leaving to none.
	 */
	OVERLAY_PIPE(config, config->pipe, flg);
	OVERLAY_INTERLEAVE(config, OI_NONE, flg);

	/*
	 *  Scale the image to fit  the destination rectangle.  Also, set
	 *  the registers that are dependant upon  the source memory map-
	 *  ping info.
	 */
	res = scaleOvl(config, OVA, OS_FULL, &flg);
	if (res != 0)
		return res;
	else {
		res = scaleOvl(config, OVC, OS_FULL, &flg);
		if (res != 0)
			return res;
	}

	res = mapOvl(config, &flg);
	if (res != 0)
		return res;

	/*
	 *  In side by side mode, the left image is placed at the target
	 *  location, and the right image at the right edge.
	 */
	res = placeOvl(config, OVA, config->x, config->y,
			config->w, config->h, &flg);
	if (res != 0)
		return res;
	else {
		res = placeOvl(config, OVC, config->x + config->w,
			config->y, config->w, config->h, &flg);
		if (res != 0)
			return res;
	}

	/*
	 *  Make sure  that the command register's enable bit  is cleared
	 *  before writing the overlays. The enabled flag is reset.
	 */
	config->enabled = 0;
	config->lx = config->x;
	config->ly = config->y;
	config->rx = config->x + config->w;
	config->ry = config->y;
	config->dw = config->w;
	config->dh = config->h, config->work[OVA][IDX_CMD] &= ~0x00000001;
	config->work[OVC][IDX_CMD] &= ~0x00000001;
	flg |= FLG_CMD;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Could not access display island\n");

		return -EBUSY;
	}

	wrOvl(dev, config->work[OVA], OVA, flg);
	wrOvl(dev, config->work[OVC], OVC, flg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_sidebyside */

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
int mrfld_s3d_ovl_sidebyside_half(struct drm_device *dev, ovl_s3d_priv_p config)
{
	int res;
	uint32_t flg = 0;

	/*
	 *  Side by side is only available on HDMI...
	 */
	if (config->pipe != OP_PIPEB) {

		printk(OVL_MSG "side by side half must be used with HDMI\n");

		return -EINVAL;
	}

	/*
	 *  Collect the  current display geometry  into the  private con-
	 *  figuration record.
	 */
	DISPLAY_GEO(config, res);

	if (res != 0) {

		printk(OVL_MSG "Invalid display geometry\n");

		return res;
	}

	/*
	 *  Direct the overlays  to the requested pipe and set the inter-
	 *  leaving to none.
	 */
	OVERLAY_PIPE(config, config->pipe, flg);
	OVERLAY_INTERLEAVE(config, OI_NONE, flg);

	/*
	 *  Scale the image to fit  the destination rectangle.  Also, set
	 *  the registers that are dependant upon  the source memory map-
	 *  ping info.
	 */
	res = scaleOvl(config, OVA, OS_HHALF, &flg);
	if (res != 0)
		return res;
	else {
		res = scaleOvl(config, OVC, OS_HHALF, &flg);
		if (res != 0)
			return res;
	}

	res = mapOvl(config, &flg);
	if (res != 0)
		return res;

	/*
	 *  In side by side half mode,  the left image is  placed  at the
	 *  target location, and the right image at the right edge, which
	 *  is half width.
	 */
	res = placeOvl(config, OVA, config->x, config->y,
		config->w >> 1, config->h, &flg);
	if (res != 0)
		return res;
	else {
		res = placeOvl(config, OVC, config->x + (config->w >> 1),
				config->y, config->w >> 1, config->h, &flg);
		if (res != 0)
			return res;
	}

	/*
	 *  Make sure  that the command register's enable bit  is cleared
	 *  before writing the overlays. The enabled flag is reset.
	 */
	config->enabled = 0;
	config->lx = config->x;
	config->ly = config->y;
	config->rx = config->x + config->w;
	config->ry = config->y;
	config->dw = config->w >> 1,
	    config->dh = config->h, config->work[OVA][IDX_CMD] &= ~0x00000001;
	config->work[OVC][IDX_CMD] &= ~0x00000001;
	flg |= FLG_CMD;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Could not access display island\n");

		return -EBUSY;
	}

	wrOvl(dev, config->work[OVA], OVA, flg);
	wrOvl(dev, config->work[OVC], OVC, flg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_sidebyside_half */

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
int mrfld_s3d_ovl_topbottom(struct drm_device *dev, ovl_s3d_priv_p config)
{
	int res;
	uint32_t flg = 0;

	/*
	 *  Top bottom is only available on HDMI...
	 */
	if (config->pipe != OP_PIPEB) {
		printk(OVL_MSG "top bottom must be used with HDMI\n");

		return -EINVAL;
	}

	/*
	 *  Collect the  current display geometry  into the  private con-
	 *  figuration record.
	 */
	DISPLAY_GEO(config, res);

	if (res != 0) {
		printk(OVL_MSG "Invalid display geometry\n");

		return res;
	}

	/*
	 *  Direct the overlays  to the requested pipe and set the inter-
	 *  leaving to none.
	 */
	OVERLAY_PIPE(config, config->pipe, flg);
	OVERLAY_INTERLEAVE(config, OI_NONE, flg);

	/*
	 *  Scale the image to fit  the destination rectangle.  Also, set
	 *  the registers that are dependant upon  the source memory map-
	 *  ping info.
	 */
	res = scaleOvl(config, OVA, OS_FULL, &flg);
	if (res != 0)
		return res;
	else {
		res = scaleOvl(config, OVC, OS_FULL, &flg);
		if (res != 0)
			return res;
	}

	res = mapOvl(config, &flg);
	if (res != 0)
		return res;

	/*
	 *  In top bottom mode,  the left image is  placed  at the target
	 *  location, and the right image at the bottom edge.
	 */
	res = placeOvl(config, OVA, config->x, config->y,
			config->w, config->h, &flg);
	if (res != 0)
		return res;
	else {
		res = placeOvl(config, OVC, config->x,
				config->y + config->h + VGAP(config),
				config->w, config->h, &flg);
		if (res != 0)
			return res;
	}

	/*
	 *  Make sure  that the command register's enable bit  is cleared
	 *  before writing the overlays. The enabled flag is reset.
	 */
	config->enabled = 0;
	config->lx = config->x;
	config->ly = config->y;
	config->rx = config->x;
	config->ry = config->y + config->h + VGAP(config);
	config->dw = config->w,
	    config->dh = config->h, config->work[OVA][IDX_CMD] &= ~0x00000001;
	config->work[OVC][IDX_CMD] &= ~0x00000001;
	flg |= FLG_CMD;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Could not access display island\n");

		return -EBUSY;
	}

	wrOvl(dev, config->work[OVA], OVA, flg);
	wrOvl(dev, config->work[OVC], OVC, flg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_topbottom */

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
extern int mrfld_s3d_ovl_topbottom_half(struct drm_device *dev,
					ovl_s3d_priv_p config)
{
	int res;
	uint32_t flg = 0;

	/*
	 *  Top bottom half is only available on HDMI...
	 */
	if (config->pipe != OP_PIPEB) {

		printk(OVL_MSG "top bottom half must be used with HDMI\n");

		return -EINVAL;
	}

	/*
	 *  Collect the  current display geometry  into the  private con-
	 *  figuration record.
	 */
	DISPLAY_GEO(config, res);

	if (res != 0) {
		printk(OVL_MSG "Invalid display geometry\n");

		return res;
	}

	/*
	 *  Direct the overlays  to the requested pipe and set the inter-
	 *  leaving to none.
	 */
	OVERLAY_PIPE(config, config->pipe, flg);
	OVERLAY_INTERLEAVE(config, OI_NONE, flg);

	/*
	 *  Scale the image to fit  the destination rectangle.  Also, set
	 *  the registers that are dependant upon  the source memory map-
	 *  ping info.
	 */
	res = scaleOvl(config, OVA, OS_VHALF, &flg);
	if (res != 0)
		return res;
	else {
		res = scaleOvl(config, OVC, OS_VHALF, &flg);
		if (res != 0)
			return res;
	}

	res = mapOvl(config, &flg);
	if (res != 0)
		return res;

	/*
	 *  In top bottom mode,  the left image is  placed  at the target
	 *  location,  and the right image  at the bottom edge,  which is
	 *  half height.
	 */
	res = placeOvl(config, OVA, config->x, config->y, config->w, config->h >> 1, &flg);
	if (res != 0)
		return res;
	else {
		res = placeOvl(config, OVC, config->x,
				config->y + (config->h >> 1) + VGAP(config),
				config->w, config->h >> 1, &flg);
		if (res != 0)
			return res;
	}

	/*
	 *  Make sure  that the command register's enable bit  is cleared
	 *  before writing the overlays. The enabled flag is reset.
	 */
	flg |= FLG_CMD;
	config->enabled = 0;
	config->lx = config->x;
	config->ly = config->y;
	config->rx = config->x;
	config->ry = config->y + (config->h >> 1) + VGAP(config);
	config->dw = config->w;
	config->dh = config->h >> 1;
	config->work[OVA][IDX_CMD] &= ~0x00000001;
	config->work[OVC][IDX_CMD] &= ~0x00000001;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Could not access display island\n");

		return -EBUSY;
	}

	wrOvl(dev, config->work[OVA], OVA, flg);
	wrOvl(dev, config->work[OVC], OVC, flg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_topbottom_half */

/*********************************
 *  mrfld_s3d_ovl_update(struct drm_device *dev,
 *                       ovl_s3d_priv_p     config,
 *                       ovl_s3d_update_p   update)
 *
 *  Purpose :   Update the video image and/or position for an overlay
 *              pair.
 *  Input   :   Pointer (dev) to the  device record, pointer (config)
 *              to the private configuration record,  and the pointer
 *              (update) to the update parameters.
 *  Output  :   None.
 *  Return  :   Zero on success or a negative error code.
 */
int mrfld_s3d_ovl_update(struct drm_device *dev,
			 ovl_s3d_priv_p config, ovl_s3d_update_p update)
{
	int res = 0;
	uint32_t flg = 0;
	struct drm_psb_private *dev_priv = psb_priv(dev);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Could not access display island\n");

		return -EBUSY;
	}

	if (update->vwait) {
		VWAIT(config);
	}

	/*
	 *  If we're supposed to disable the overlays, then do that right
	 *  away.
	 */
	if (((update->valid & OU_ENABLE) != 0) && !update->enable) {
		config->work[OVA][IDX_CMD] &= ~0x00000001;
		config->work[OVC][IDX_CMD] &= ~0x00000001;
		update->valid &= ~OU_ENABLE;
		config->enabled = 0;

		wrOvl(dev, config->work[OVA], OVA, FLG_CMD);
		wrOvl(dev, config->work[OVC], OVC, FLG_CMD);
	}

	if ((update->valid & OU_GEOMETRY) != 0) {
		int xa, ya, wa, ha, xc, yc, wc, hc;
		scale_t scl = OS_FULL;

		xa = xc = config->x = update->x;
		ya = yc = config->y = update->y;
		wa = wc = config->w = update->w;
		ha = hc = config->h = update->h;

		switch (config->mode) {
		case OM_HALFLINEINTER:
			scl = OS_VHALF;
			ha >>= 1;
			hc >>= 1;
			break;

		case OM_PIXINTER:
			break;

		case OM_HALFPIXINTER:
			scl = OS_VHALF;
			ha >>= 1;
			hc >>= 1;
			break;

		case OM_SIDEBYSIDE:
			xc += ha;
			break;

		case OM_HALFSIDEBYSIDE:
			scl = OS_HHALF;
			wa >>= 1;
			wc >>= 1;
			xc += wc;
			break;

		case OM_TOPBOTTOM:
		case OM_HALFTOPBOTTOM:
		case OM_LINEINTER:
		default:
			break;
		}

		res = scaleOvl(config, OVA, scl, &flg);
		if (res != 0)
			goto bail;
		else {
			res = scaleOvl(config, OVC, scl, &flg) != 0;
			if (res != 0)
				goto bail;
		}

		res = placeOvl(config, OVA, xa, ya, wa, ha, &flg);
		if (res != 0) {
			goto bail;
		}

		res = placeOvl(config, OVC, xc, yc, wc, hc, &flg);
		if (res != 0) {
			goto bail;
		}

		config->lx = xa;
		config->ly = ya;
		config->rx = xc;
		config->ry = yc;
		config->dw = wa;
		config->dh = ha;
	}

	if ((update->valid & OU_PAN) != 0) {
		/* FIXME */
		config->sx = 0;
		config->sy = 0;
	}

	/*
	 *  Recalculate the surface registers if these are chaging.
	 */
	if ((update->valid & OU_SURFACE) != 0) {
		res = surfaceOvl(config, update, &flg);
		if (res != 0) {
			goto bail;
		}

		config->lbuf = update->lbuf;
		config->rbuf = update->rbuf;
	}

	/*
	 *  Finally, the only way the update flag bit can be set is if we
	 *  are to enable the overlays.
	 */
	if ((update->valid & OU_ENABLE) != 0) {
		config->work[OVA][IDX_CMD] |= 0x00000001;
		config->work[OVC][IDX_CMD] |= 0x00000001;
		config->enabled = 1;

		wrOvl(dev, config->work[OVA], OVA, FLG_CMD);
		wrOvl(dev, config->work[OVC], OVC, FLG_CMD);
	}

	if (((update->valid & OU_SURFACE) != 0) && config->enabled) {
		FLIPWAIT(config);
	}

 bail:

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return res;
}				/* mrfld_s3d_ovl_update */

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
int mrfld_s3d_ovl_save(struct drm_device *dev, ovl_s3d_priv_p config)
{
	/*
	 *  Access the display island to ensure  that the overlays can be
	 *  read. Report an error is this fails.
	 */
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Can't access display island\n");

		return -EAGAIN;
	}
#ifdef REG_NOSRTRACE
	saverestore = 1;
#endif

	/*
	 *  Saving  involves a read  of all overlay registers  into their
	 *  respective slots in the private configuration register array.
	 *  Then, an initial working copy is made from what is found.
	 */
	rdOvl(dev, config->save[OVA], OVA, ALL_FLAGS & ~FLG_UVHCOEF);
	rdOvl(dev, config->save[OVC], OVC, ALL_FLAGS & ~FLG_UVHCOEF);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	memcpy(config->work, config->save, sizeof config->work);

	printk(OVL_MSG "mrfl_s3d_ovl_save : completed\n");

#ifdef REG_NOSRTRACE
	saverestore = 0;
#endif

	return 0;
}				/* mrfld_s3d_ovl_save */

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
int mrfld_s3d_ovl_restore(struct drm_device *dev, ovl_s3d_priv_p config)
{
	uint32_t ocmda, ocmdc;
	struct drm_psb_private *dev_priv = dev->dev_private;

	/*
	 *  Access the display island to ensure  that the overlays can be
	 *  written. Report an error is this fails.
	 */
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Can't access display island\n");

		return -EAGAIN;
	}
#ifdef REG_NOSRTRACE
	saverestore = 1;
#endif

	/*
	 *  Disable both overlays before attempting a restore. We use the
	 *  current contents of the command registers. The LSB (bit 0) is
	 *  cleared to disable.
	 */
	ocmda = OVLRD(OVA, OV_CMD);
	ocmdc = OVLRD(OVC, OV_CMD + OV_C_OFFSET);

	OVLWR(OVA, OV_CMD, ocmda & ~1);
	OVLWR(OVC, OV_CMD + OV_C_OFFSET, ocmdc & ~1);

	/*
	 *  Recover  all registers, except the command register, for each
	 *  overlay. Then restore the original command register.
	 */
	wrOvl(dev, config->save[OVA], OVA, ALL_FLAGS & ~FLG_UVHCOEF & ~FLG_CMD);
	wrOvl(dev, config->save[OVC], OVC, ALL_FLAGS & ~FLG_UVHCOEF & ~FLG_CMD);
	OVLWR(OVA, OV_CMD, config->save[OVA][IDX_CMD]);
	OVLWR(OVC, OV_CMD + OV_C_OFFSET, config->save[OVC][IDX_CMD]);

#ifdef REG_NOSRTRACE
	saverestore = 0;
#endif

	printk(OVL_MSG "mrfl_s3d_ovl_restore : completed\n");

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_restore */

int mrfld_s3d_ovl_regrw(struct drm_device *dev, ovl_s3d_reg_p reg)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int n = (reg->ovc ? OVC : OVA);

	/*
	 *  Access the display island to ensure  that the overlays can be
	 *  written. Report an error is this fails.
	 */
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON)) {

		printk(OVL_MSG "Can't access display island\n");

		return -EAGAIN;
	}

	/*
	 *  Perform a write request first,  then the read (if any).  Note
	 *  that if the caller supplies nothing, nothing is done.
	 */
	if (reg->mode & OM_WRITE) {
		OVLWR(n, reg->reg, reg->data);
	}

	if (reg->mode & OM_READ) {
		reg->data = OVLRD(n, reg->reg);
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return 0;
}				/* mrfld_s3d_ovl_regrw */

/*********************************
 *                  M O D U L E  S T A T I C S
 ********************************/

/*********************************
 *  Reads the registers (and/or sets) indicated by 'flags' from 'ovl'
 *      into the register buffer 'buf'.
 */
static void rdOvl(struct drm_device *dev,
		  uint32_t *buf, ovl_s3d_t ovl, uint32_t flags)
{
	unsigned int ofs, msk;
	uint32_t reg;
	int i, j, k;
	struct drm_psb_private *dev_priv = dev->dev_private;

	ofs = (ovl ? OV_C_OFFSET : 0);
	msk = 0x1;
	i = 0;
	k = 0;

	while (flags && msk && (i < NUM_FLAGS)) {
		if (flags & msk) {
			reg = m_rwtab[i][0];

			for (j = 0; j < m_rwtab[i][1]; j++) {

				buf[k + j] = OVLRD(ovl, reg + ofs);

#ifdef REG_RDTRACE
#ifdef REG_NOSRTRACE
				if (!saverestore) {
					printk(OVL_MSG
					       "read register 0x%08x -> 0x%08x\n",
					       (unsigned int)(reg + ofs),
					       buf[k + j]);
				}
#else
				printk(OVL_MSG
				       "read register 0x%08x -> 0x%08x\n",
				       (unsigned int)(reg + ofs), buf[k + j]);
#endif
#endif

				reg += 4;
			}

			flags &= ~msk;
		}

		msk <<= 1;
		k += m_rwtab[i][1];

		++i;
	}
}				/* rdOvl */

/*********************************
 *  Writes the registers (and/or sets) indicated by  'flags' to 'ovl'
 *      from the register buffer 'buf'.
 */
static void wrOvl(struct drm_device *dev,
		  uint32_t *buf, ovl_s3d_t ovl, uint32_t flags)
{
#if defined(REG_WRTRACE)
	static char *oname[2] = { "OVA_", "OVC_" };
#endif
	unsigned int ofs, msk;
	uint32_t reg;
	int i, j, k;
	struct drm_psb_private *dev_priv = dev->dev_private;

	ofs = (ovl ? OV_C_OFFSET : 0);
	msk = 0x1;
	i = 0;
	k = 0;

	while (flags && msk && (i < NUM_FLAGS)) {
		if (flags & msk) {
			reg = m_rwtab[i][0];

			for (j = 0; j < m_rwtab[i][1]; j++) {

				OVLWR(ovl, reg + ofs, buf[k + j]);

#ifdef REG_WRTRACE
#ifdef REG_NOSRTRACE
				if (!saverestore) {
					printk(OVL_MSG "%s%s <- 0x%08x\n",
					       oname[ovl], rname[k + j],
					       buf[k + j]);
				}
#else
				printk(OVL_MSG "%s%s <- 0x%08x\n",
				       oname[ovl], rname[k + j], buf[k + j]);
#endif
#endif

				reg += 4;
			}

			flags &= ~msk;
		}

		msk <<= 1;
		k += m_rwtab[i][1];

		++i;
	}
}				/* wrOvl */

/*********************************
 *  Determines the  scaling that should be applied  to a source image
 *      so that it fits on the display rectangle. Populates 'work' in
 *      'priv' and adjusts the scaling according to  'type'.  Updates
 *      the  adjustment bits in 'flg'.  Returns zero on success  or a
 *      negative error code.
 *
 *  NOTE : The code in this routine is based largely on that found in
 *      psb_video/src/psb_overlay.c, a user space video driver.
 */
static int scaleOvl(ovl_s3d_priv_p priv,
		    ovl_s3d_t ovl, scale_t type, unsigned int *flg)
{
#define SHSCL   15		/*  maybe should be 16?             */

	uint32_t newval;
	int xscaleInt, xscaleFract, yscaleInt, yscaleFract,
	    xscaleIntUV, xscaleFractUV, yscaleIntUV, yscaleFractUV, changed = 0;

	/*
	 *  If the  source and destination  rectangles  are the same size
	 *  then there is no scaling.  Otherwise  we need to find  both a
	 *  vertical and horizontal scale factor.  Factors  are multiples
	 *  of 4096.
	 */
	if ((priv->sw == priv->w) && (priv->sh == priv->h)) {
		xscaleFract = 1 << 12;
		yscaleFract = 1 << 12;
	} else {
		xscaleFract = ((priv->sw - 1) << 12) / priv->w;
		yscaleFract = ((priv->sh - 1) << 12) / priv->h;
	}

	/*
	 *  These may need  to be qualified by the 'type' of scaling that
	 *  the mode implies.
	 */
	switch (type) {
	case OS_VHALF:
		yscaleFract >>= 1;
		break;

	case OS_HHALF:
		xscaleFract >>= 1;
		break;

	default:
		break;
	}

	/*
	 *  When a planar format is applied, the UV scaling factor is 1/2
	 *  that of the luminance Y. To keep the relative Y and UV ratios
	 *  exact, round the Y scales to a multiple of the Y/UV ratio.
	 */
	xscaleFractUV = xscaleFract >> 1;
	yscaleFractUV = yscaleFract >> 1;
	xscaleFract = xscaleFractUV << 1;
	yscaleFract = yscaleFractUV << 1;

	/*
	 *  Find the integer (un-multiplied) portion  as these are stored
	 *  separately in the scaling registers.
	 */
	xscaleInt = xscaleFract >> 12;
	yscaleInt = yscaleFract >> 12;
	xscaleIntUV = xscaleFractUV >> 12;
	yscaleIntUV = yscaleFractUV >> 12;

	/*
	 *  This shouldn't happen...
	 */
	if ((xscaleInt > 7) || (xscaleIntUV > 7)) {

		printk(OVL_MSG "scaling failure - factor too large\n");

		return -EINVAL;
	}

	/*
	 *  Compose the register values  that reflect the proper  x and y
	 *  scaling. Should either be different from what is currently in
	 *  the register, mark the change.
	 */
	newval = (xscaleInt << SHSCL) |
	    ((xscaleFract & 0xFFF) << 3) | ((yscaleFract & 0xFFF) << 20);

	if (newval != priv->work[ovl][IDX_YRGBSCL]) {
		changed = 1;
		priv->work[ovl][IDX_YRGBSCL] = newval;
		*flg |= FLG_YRGBSCL;
	}

	newval = (xscaleIntUV << SHSCL) |
	    ((xscaleFractUV & 0xFFF) << 3) | ((yscaleFractUV & 0xFFF) << 20);

	if (newval != priv->work[ovl][IDX_UVSCL]) {
		changed = 1;
		priv->work[ovl][IDX_UVSCL] = newval;
		*flg |= FLG_UVSCL;
	}

	return 0;
}				/* scaleOvl */

/*********************************
 *  Position 'ovl' on the screen according to  'x', 'y', 'w' and 'h'.
 *      Populates 'work' in 'priv' and updates the adjustment bits in
 *      'flg'. Return zero on success or a negative error code.
 */
static int placeOvl(ovl_s3d_priv_p priv,
		    ovl_s3d_t ovl,
		    int x, int y, int w, int h, unsigned int *flg)
{
	int ox, oy;

	x &= 0xfff;
	y &= 0xfff;
	w &= 0xfff;
	h &= 0xfff;

	if (ovl == OVLEFT) {
		ox = priv->lx;
		oy = priv->ly;
	} else {
		ox = priv->rx;
		oy = priv->ry;
	}

	if ((x != ox) || (y != oy)) {
		priv->work[ovl][IDX_DWINPOS] = (y << 16) | x;
		*flg |= FLG_DWINPOS;
	}

	if ((w != priv->dw) || (h != priv->dh)) {
		priv->work[ovl][IDX_DWINSZ] = (h << 16) | w;
		*flg |= FLG_DWINSZ;
	}

	return 0;
}				/* placeOvl */

/*********************************
 *  Sets the register fields in  'work' from 'priv'  according to the
 *      mapping information (e.g.,  yoffset, stride,  etc.).  Adjusts
 *      bits in 'flg' accordingly. Returns zero on success  or a neg-
 *      ative error code.
 */
static int mapOvl(ovl_s3d_priv_p priv, unsigned int *flg)
{
	uint32_t stride, swidthy, swidthuv, w, h;

	stride = priv->stride & 0x0000ffff;
	w = priv->sw;
	h = priv->sh;

	/*
	 *  The values that will appear  in the linear buffers depends on
	 *  the current video pixel mode.
	 */
	priv->work[OVA][IDX_BUF0Y] = priv->yoffset;
	priv->work[OVA][IDX_BUF1Y] = priv->yoffset;
	priv->work[OVC][IDX_BUF0Y] = priv->yoffset;
	priv->work[OVC][IDX_BUF1Y] = priv->yoffset;
	*flg |= FLG_BUF;

	switch (priv->pixel) {
	case OX_NV12:
		/*
		 *  NV12 has an interleaved  UV plane,  thus there is no need
		 *  to set the V buffers.
		 */
		priv->work[OVA][IDX_BUF0U] = priv->uoffset;
		priv->work[OVA][IDX_BUF1U] = priv->uoffset;
		priv->work[OVA][IDX_BUF0V] = priv->uoffset;
		priv->work[OVA][IDX_BUF1V] = priv->uoffset;

		/*
		 *  The stride for both planes in this mode  are identical as
		 *  both are 8 BPP.  Note that the  UV  plane is only half as
		 *  high as the Y.
		 */
		stride |= (priv->stride << 16);
		priv->work[OVA][IDX_STRIDE] = stride;
		priv->work[OVC][IDX_STRIDE] = stride;
		*flg |= FLG_STRIDE;

		/*
		 *  Calculate the remainder of the mapping values for NV12.
		 */
		swidthy = swidth(priv->yoffset, w);
		swidthuv = swidth(priv->uoffset, w / 2);
		priv->work[OVA][IDX_SWIDTH] = w | (((w / 2) & 0x7FFF) << 16);
		priv->work[OVC][IDX_SWIDTH] = w | (((w / 2) & 0x7FFF) << 16);
		*flg |= FLG_SWIDTH;
		priv->work[OVA][IDX_SWIDTHSW] = swidthy | (swidthuv << 16);
		priv->work[OVC][IDX_SWIDTHSW] = swidthy | (swidthuv << 16);
		*flg |= FLG_SWIDTHSW;
		priv->work[OVA][IDX_SHEIGHT] = h | ((h / 2) << 16);
		priv->work[OVC][IDX_SHEIGHT] = h | ((h / 2) << 16);
		*flg |= FLG_SHEIGHT;
		break;

	case OX_YUV420:
		/*
		 *  This is a full planar mode. Both the U and V buffers must
		 *  be set.
		 */
		priv->work[OVA][IDX_BUF0U] = priv->uoffset;
		priv->work[OVA][IDX_BUF1U] = priv->uoffset;
		priv->work[OVA][IDX_BUF0V] = priv->voffset;
		priv->work[OVA][IDX_BUF1V] = priv->voffset;

		/*
		 *  The U and V stride for this mode is half the Y stride.
		 */
		stride |= (priv->stride << 15);
		priv->work[OVA][IDX_STRIDE] = stride;
		priv->work[OVC][IDX_STRIDE] = stride;
		*flg |= FLG_STRIDE;

		/*
		 *  Calculate the remainder of the mapping registers for the
		 *  YUV420 mode.
		 */
		swidthy = swidth(priv->yoffset, w);
		swidthuv = swidth(priv->uoffset, w / 2);
		priv->work[OVA][IDX_SWIDTH] = w | (((w / 2) & 0x7FFF) << 16);
		priv->work[OVC][IDX_SWIDTH] = w | (((w / 2) & 0x7FFF) << 16);
		*flg |= FLG_SWIDTH;
		priv->work[OVA][IDX_SWIDTHSW] = swidthy | (swidthuv << 16);
		priv->work[OVC][IDX_SWIDTHSW] = swidthy | (swidthuv << 16);
		*flg |= FLG_SWIDTHSW;
		priv->work[OVA][IDX_SHEIGHT] = h | ((h / 2) << 16);
		priv->work[OVC][IDX_SHEIGHT] = h | ((h / 2) << 16);
		*flg |= FLG_SHEIGHT;
		break;

	case OX_YUV422:
	default:
		/*
		 *  This is a packed format.  There is no need for any buffer
		 *  offset other than the Y. The stride for this mode has no
		 *  UV value.
		 */
		priv->work[OVA][IDX_STRIDE] = stride;
		priv->work[OVC][IDX_STRIDE] = stride;
		*flg |= FLG_STRIDE;
		priv->work[OVA][IDX_SWIDTH] = w;
		priv->work[OVC][IDX_SWIDTH] = w;
		*flg |= FLG_SWIDTH;
		swidthy = priv->yoffset + (w << 1) + 0x3F;
		swidthy >>= 5;
		swidthy -= 1;
		swidthy <<= 2;
		priv->work[OVA][IDX_SWIDTHSW] = swidthy;
		priv->work[OVC][IDX_SWIDTHSW] = swidthy;
		*flg |= FLG_SWIDTHSW;
		priv->work[OVA][IDX_SHEIGHT] = h;
		priv->work[OVC][IDX_SHEIGHT] = h;
		*flg |= FLG_SHEIGHT;
		break;
	}

	return 0;
}				/* mapOvl */

/*********************************
 *  Attach surfaces  to the overlays  in accordance with  'priv'  and
 *      'update'. Update the adjustment bits in 'flg'. Return zero on
 *      success or a negative error code.
 */
static int surfaceOvl(ovl_s3d_priv_p priv,
		      ovl_s3d_update_p update, unsigned int *flg)
{
#define     CSZ     (64 * sizeof(uint32_t))

	uint32_t ovadda, ovaddc;

	/*
	 *  Find the overlay address for each of the source images on the
	 *  left and right.  These preserve the lower 16-bits cleared  as
	 *  they are 64K aligned.
	 */
	ovadda = GADDR(update->lbuf);
	ovaddc = GADDR(update->rbuf);

	/*
	 *  If the user has provided horizontal scale coefficients,  then
	 *  copy that into the register working data.
	 */
	if ((update->valid & OU_HCOEFF) != 0) {
		if ((copy_from_user(&priv->work[OVA][IDX_UVHCOEF],
				    update->hcoef,
				    CSZ) != CSZ) ||
		    (copy_from_user(&priv->work[OVC][IDX_UVHCOEF],
				    update->hcoef, CSZ) != CSZ)) {

			printk(OVL_MSG "failed to copy UVHCOEF data\n");

			return -EFAULT;
		}

		/*
		 *  These bits indicate  that the coefficients  should be up-
		 *  dated on page flip.
		 */
		ovadda |= 1;
		ovaddc |= 1;
		*flg |= FLG_UVHCOEF;
	} else {
		ovadda &= ~0x00000001;
		ovaddc &= ~0x00000001;
	}

	priv->work[OVA][IDX_OVADD] |= ovadda;
	priv->work[OVC][IDX_OVADD] |= ovaddc;
	*flg |= FLG_OVADD;

	return 0;
}				/* surfaceOvl */

/*********************************
 *  Based upon a similar function in psb_video/src/psb_overlay.c. Re-
 *      turns the SWIDTH register value.
 */
static int swidth(unsigned int offset, unsigned int width)
{
#define SHIFT   6
#define MASK    0x3f

	int swidth;

	swidth = ((offset + width + MASK) >> SHIFT) - (offset >> SHIFT);
	swidth <<= 1;
	swidth -= 1;

	return swidth << 2;
}				/* swidth */
