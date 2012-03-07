/*
 * Support for OmniVision ov8830 1080p HD camera sensor.
 *
 * Copyright (c) 2011 Intel Corporation. All Rights Reserved.
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include "ov8830.h"

#define to_ov8830_sensor(sd) container_of(sd, struct ov8830_device, sd)

#define HOME_POS 255

/* 816x612_2lane_30fps 100 99 816 612 */
static const struct ov8830_reg ov8830_mode1[] = {
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0103 }, 0x01 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0102 }, 0x01 },
	{ OV8830_8BIT, { 0x3001 }, 0x2a },
	{ OV8830_8BIT, { 0x3002 }, 0x88 },
	{ OV8830_8BIT, { 0x3005 }, 0x00 },
	{ OV8830_8BIT, { 0x3011 }, 0x41 },
	{ OV8830_8BIT, { 0x3015 }, 0x08 },
	{ OV8830_8BIT, { 0x301b }, 0xb4 },
	{ OV8830_8BIT, { 0x301d }, 0x02 },
	{ OV8830_8BIT, { 0x3021 }, 0x00 },
	{ OV8830_8BIT, { 0x3022 }, 0x00 },
	{ OV8830_8BIT, { 0x3081 }, 0x02 },
	{ OV8830_8BIT, { 0x3083 }, 0x01 },
	{ OV8830_8BIT, { 0x3090 }, 0x02 },	/* PLL2_prediv: 2 */
	{ OV8830_8BIT, { 0x3091 }, 22 },	/* PLL2_multiplier */
	{ OV8830_8BIT, { 0x3092 }, 0 },		/* PLL2_divs: 0+1: 1 ok */
	{ OV8830_8BIT, { 0x3093 }, 0x00 },	/* PLL2_seld5: 0 */
	{ OV8830_8BIT, { 0x3098 }, 0x03 },	/* PLL3_prediv: 3 */
	{ OV8830_8BIT, { 0x3099 }, 19 },	/* PLL3_mult2: 30 */
	{ OV8830_8BIT, { 0x309a }, 0x00 },	/* PLL3_divs: 0 */
	{ OV8830_8BIT, { 0x309b }, 0x00 },	/* PLL3_div: 0 */
	{ OV8830_8BIT, { 0x309c }, 0x01 },	/* PLL3_mult1: 0 (default) */
	{ OV8830_8BIT, { 0x30a2 }, 0x01 },
	{ OV8830_8BIT, { 0x30b0 }, 0x05 },
	{ OV8830_8BIT, { 0x30b2 }, 0x00 },
	{ OV8830_8BIT, { 0x30b3 }, 94 },	/* PLL1_multiplier */
	{ OV8830_8BIT, { 0x30b4 }, 0x03 },	/* PLL1_prediv: 3 */
	{ OV8830_8BIT, { 0x30b5 }, 0x04 },	/* PLL1_op_pix_div: 4 *2  */
	{ OV8830_8BIT, { 0x30b6 }, 0x01 },	/* PLL1_op_sys_div: 1 */
	{ OV8830_8BIT, { 0x3104 }, 0xa1 },
	{ OV8830_8BIT, { 0x3106 }, 0x01 },
	{ OV8830_8BIT, { 0x3400 }, 0x04 },
	{ OV8830_8BIT, { 0x3401 }, 0x00 },
	{ OV8830_8BIT, { 0x3402 }, 0x04 },
	{ OV8830_8BIT, { 0x3403 }, 0x00 },
	{ OV8830_8BIT, { 0x3404 }, 0x04 },
	{ OV8830_8BIT, { 0x3405 }, 0x00 },
	{ OV8830_8BIT, { 0x3406 }, 0x01 },
	{ OV8830_8BIT, { 0x3500 }, 0x00 },
	{ OV8830_8BIT, { 0x3501 }, 0x25 },
	{ OV8830_8BIT, { 0x3502 }, 0x46 },
	{ OV8830_8BIT, { 0x3503 }, 0x07 },
	{ OV8830_8BIT, { 0x3504 }, 0x00 },
	{ OV8830_8BIT, { 0x3505 }, 0x30 },
	{ OV8830_8BIT, { 0x3506 }, 0x00 },
	{ OV8830_8BIT, { 0x3507 }, 0x04 },
	{ OV8830_8BIT, { 0x3508 }, 0x80 },
	{ OV8830_8BIT, { 0x3509 }, 0x10 },
	{ OV8830_8BIT, { 0x350a }, 0x00 },
	{ OV8830_8BIT, { 0x350b }, 0x38 },
	{ OV8830_8BIT, { 0x3600 }, 0x78 },
	{ OV8830_8BIT, { 0x3601 }, 0x02 },
	{ OV8830_8BIT, { 0x3602 }, 0x1c },
	{ OV8830_8BIT, { 0x3604 }, 0x38 },
	{ OV8830_8BIT, { 0x3620 }, 0x64 },
	{ OV8830_8BIT, { 0x3621 }, 0xb5 },
	{ OV8830_8BIT, { 0x3622 }, 0x03 },
	{ OV8830_8BIT, { 0x3625 }, 0x64 },
	{ OV8830_8BIT, { 0x3630 }, 0x55 },
	{ OV8830_8BIT, { 0x3631 }, 0xd2 },
	{ OV8830_8BIT, { 0x3632 }, 0x00 },
	{ OV8830_8BIT, { 0x3633 }, 0x34 },
	{ OV8830_8BIT, { 0x3634 }, 0x03 },
	{ OV8830_8BIT, { 0x3660 }, 0x80 },
	{ OV8830_8BIT, { 0x3662 }, 0x10 },
	{ OV8830_8BIT, { 0x3665 }, 0x00 },
	{ OV8830_8BIT, { 0x3666 }, 0x00 },
	{ OV8830_8BIT, { 0x3667 }, 0x00 },
	{ OV8830_8BIT, { 0x366a }, 0x80 },
	{ OV8830_8BIT, { 0x366c }, 0x00 },
	{ OV8830_8BIT, { 0x366d }, 0x00 },
	{ OV8830_8BIT, { 0x366e }, 0x00 },
	{ OV8830_8BIT, { 0x366f }, 0x20 },
	{ OV8830_8BIT, { 0x3680 }, 0xe0 },
	{ OV8830_8BIT, { 0x3681 }, 0x00 },
	{ OV8830_8BIT, { 0x3701 }, 0x14 },
	{ OV8830_8BIT, { 0x3702 }, 0xbf },
	{ OV8830_8BIT, { 0x3703 }, 0x8c },
	{ OV8830_8BIT, { 0x3704 }, 0x78 },
	{ OV8830_8BIT, { 0x3705 }, 0x02 },
	{ OV8830_8BIT, { 0x3708 }, 0xe8 },
	{ OV8830_8BIT, { 0x3709 }, 0x03 },
	{ OV8830_8BIT, { 0x370a }, 0x00 },
	{ OV8830_8BIT, { 0x370b }, 0x20 },
	{ OV8830_8BIT, { 0x370c }, 0x0c },
	{ OV8830_8BIT, { 0x370d }, 0x11 },
	{ OV8830_8BIT, { 0x370e }, 0x00 },
	{ OV8830_8BIT, { 0x370f }, 0x00 },
	{ OV8830_8BIT, { 0x3710 }, 0x00 },
	{ OV8830_8BIT, { 0x371c }, 0x01 },
	{ OV8830_8BIT, { 0x371f }, 0x0c },
	{ OV8830_8BIT, { 0x3721 }, 0x00 },
	{ OV8830_8BIT, { 0x3724 }, 0x10 },
	{ OV8830_8BIT, { 0x3726 }, 0x00 },
	{ OV8830_8BIT, { 0x372a }, 0x01 },
	{ OV8830_8BIT, { 0x3730 }, 0x18 },
	{ OV8830_8BIT, { 0x3738 }, 0x22 },
	{ OV8830_8BIT, { 0x3739 }, 0x08 },
	{ OV8830_8BIT, { 0x373a }, 0x51 },
	{ OV8830_8BIT, { 0x373b }, 0x02 },
	{ OV8830_8BIT, { 0x373c }, 0x20 },
	{ OV8830_8BIT, { 0x373f }, 0x02 },
	{ OV8830_8BIT, { 0x3740 }, 0x42 },
	{ OV8830_8BIT, { 0x3741 }, 0x02 },
	{ OV8830_8BIT, { 0x3742 }, 0x18 },
	{ OV8830_8BIT, { 0x3743 }, 0x01 },
	{ OV8830_8BIT, { 0x3744 }, 0x02 },
	{ OV8830_8BIT, { 0x3747 }, 0x10 },
	{ OV8830_8BIT, { 0x374c }, 0x04 },
	{ OV8830_8BIT, { 0x3751 }, 0xf0 },
	{ OV8830_8BIT, { 0x3752 }, 0x00 },
	{ OV8830_8BIT, { 0x3753 }, 0x00 },
	{ OV8830_8BIT, { 0x3754 }, 0xc0 },
	{ OV8830_8BIT, { 0x3755 }, 0x00 },
	{ OV8830_8BIT, { 0x3756 }, 0x1a },
	{ OV8830_8BIT, { 0x3758 }, 0x00 },
	{ OV8830_8BIT, { 0x3759 }, 0x0f },
	{ OV8830_8BIT, { 0x375c }, 0x04 },
	{ OV8830_8BIT, { 0x3767 }, 0x01 },
	{ OV8830_8BIT, { 0x376b }, 0x44 },
	{ OV8830_8BIT, { 0x3774 }, 0x10 },
	{ OV8830_8BIT, { 0x3776 }, 0x00 },
	{ OV8830_8BIT, { 0x377f }, 0x08 },
	{ OV8830_8BIT, { 0x3780 }, 0x22 },
	{ OV8830_8BIT, { 0x3781 }, 0x0c },
	{ OV8830_8BIT, { 0x3784 }, 0x2c },
	{ OV8830_8BIT, { 0x3785 }, 0x1e },
	{ OV8830_8BIT, { 0x378f }, 0xf5 },
	{ OV8830_8BIT, { 0x3791 }, 0xb0 },
	{ OV8830_8BIT, { 0x3795 }, 0x00 },
	{ OV8830_8BIT, { 0x3796 }, 0x64 },
	{ OV8830_8BIT, { 0x3797 }, 0x11 },
	{ OV8830_8BIT, { 0x3798 }, 0x30 },
	{ OV8830_8BIT, { 0x3799 }, 0x41 },
	{ OV8830_8BIT, { 0x379a }, 0x07 },
	{ OV8830_8BIT, { 0x379b }, 0xb0 },
	{ OV8830_8BIT, { 0x379c }, 0x0c },
	{ OV8830_8BIT, { 0x37c5 }, 0x00 },
	{ OV8830_8BIT, { 0x37c6 }, 0xa0 },
	{ OV8830_8BIT, { 0x37c7 }, 0x00 },
	{ OV8830_8BIT, { 0x37c9 }, 0x00 },
	{ OV8830_8BIT, { 0x37ca }, 0x00 },
	{ OV8830_8BIT, { 0x37cb }, 0x00 },
	{ OV8830_8BIT, { 0x37cc }, 0x00 },
	{ OV8830_8BIT, { 0x37cd }, 0x00 },
	{ OV8830_8BIT, { 0x37ce }, 0x01 },
	{ OV8830_8BIT, { 0x37cf }, 0x00 },
	{ OV8830_8BIT, { 0x37d1 }, 0x01 },
	{ OV8830_8BIT, { 0x37de }, 0x00 },
	{ OV8830_8BIT, { 0x37df }, 0x00 },
	{ OV8830_8BIT, { 0x3800 }, 0x00 },	/* TIMING_X_ADDR_START = 0 */
	{ OV8830_8BIT, { 0x3801 }, 0x00 },
	{ OV8830_8BIT, { 0x3802 }, 0x00 },	/* TIMING_Y_ADDR_START = 0 */
	{ OV8830_8BIT, { 0x3803 }, 0x00 },
	{ OV8830_8BIT, { 0x3804 }, 0x0c },
	{ OV8830_8BIT, { 0x3805 }, 0xdf },
	{ OV8830_8BIT, { 0x3806 }, 0x09 },
	{ OV8830_8BIT, { 0x3807 }, 0xaf },
	{ OV8830_8BIT, { 0x3808 }, 0x03 },	/* TIMING_X_OUTPUT_SIZE = 816 */
	{ OV8830_8BIT, { 0x3809 }, 0x30 },
	{ OV8830_8BIT, { 0x380a }, 0x02 },	/* TIMING_Y_OUTPUT_SIZE = 612 */
	{ OV8830_8BIT, { 0x380b }, 0x64 },
	{ OV8830_8BIT, { 0x380c }, 0x0e },	/* TIMING_HTS = 3608 */
	{ OV8830_8BIT, { 0x380d }, 0x18 },
	{ OV8830_8BIT, { 0x380e }, 0x07 },	/* TIMING_VTS = 1996 */
	{ OV8830_8BIT, { 0x380f }, 0xcc },
	{ OV8830_8BIT, { 0x3810 }, 0x00 },
	{ OV8830_8BIT, { 0x3811 }, 0x04 },
	{ OV8830_8BIT, { 0x3812 }, 0x00 },
	{ OV8830_8BIT, { 0x3813 }, 0x04 },
	{ OV8830_8BIT, { 0x3814 }, 0x71 },
	{ OV8830_8BIT, { 0x3815 }, 0x71 },
	{ OV8830_8BIT, { 0x3820 }, 0x11 },
	{ OV8830_8BIT, { 0x3821 }, 0x0f },
	{ OV8830_8BIT, { 0x3823 }, 0x00 },
	{ OV8830_8BIT, { 0x3824 }, 0x00 },
	{ OV8830_8BIT, { 0x3825 }, 0x00 },
	{ OV8830_8BIT, { 0x3826 }, 0x00 },
	{ OV8830_8BIT, { 0x3827 }, 0x00 },
	{ OV8830_8BIT, { 0x382a }, 0x04 },
	{ OV8830_8BIT, { 0x3a04 }, 0x02 },
	{ OV8830_8BIT, { 0x3a05 }, 0x69 },
	{ OV8830_8BIT, { 0x3a06 }, 0x00 },
	{ OV8830_8BIT, { 0x3a07 }, 0xf8 },
	{ OV8830_8BIT, { 0x3b00 }, 0x00 },
	{ OV8830_8BIT, { 0x3b02 }, 0x00 },
	{ OV8830_8BIT, { 0x3b03 }, 0x00 },
	{ OV8830_8BIT, { 0x3b04 }, 0x00 },
	{ OV8830_8BIT, { 0x3b05 }, 0x00 },
	{ OV8830_8BIT, { 0x3d00 }, 0x00 },
	{ OV8830_8BIT, { 0x3d01 }, 0x00 },
	{ OV8830_8BIT, { 0x3d02 }, 0x00 },
	{ OV8830_8BIT, { 0x3d03 }, 0x00 },
	{ OV8830_8BIT, { 0x3d04 }, 0x00 },
	{ OV8830_8BIT, { 0x3d05 }, 0x00 },
	{ OV8830_8BIT, { 0x3d06 }, 0x00 },
	{ OV8830_8BIT, { 0x3d07 }, 0x00 },
	{ OV8830_8BIT, { 0x3d08 }, 0x00 },
	{ OV8830_8BIT, { 0x3d09 }, 0x00 },
	{ OV8830_8BIT, { 0x3d0a }, 0x00 },
	{ OV8830_8BIT, { 0x3d0b }, 0x00 },
	{ OV8830_8BIT, { 0x3d0c }, 0x00 },
	{ OV8830_8BIT, { 0x3d0d }, 0x00 },
	{ OV8830_8BIT, { 0x3d0e }, 0x00 },
	{ OV8830_8BIT, { 0x3d0f }, 0x00 },
	{ OV8830_8BIT, { 0x3d80 }, 0x00 },
	{ OV8830_8BIT, { 0x3d81 }, 0x00 },
	{ OV8830_8BIT, { 0x3d84 }, 0x00 },
	{ OV8830_8BIT, { 0x4000 }, 0x18 },
	{ OV8830_8BIT, { 0x4001 }, 0x04 },
	{ OV8830_8BIT, { 0x4002 }, 0x45 },
	{ OV8830_8BIT, { 0x4004 }, 0x02 },
	{ OV8830_8BIT, { 0x4005 }, 0x1A },
	{ OV8830_8BIT, { 0x4006 }, 0x16 },
	{ OV8830_8BIT, { 0x4008 }, 0x20 },
	{ OV8830_8BIT, { 0x4009 }, 0x10 },
	{ OV8830_8BIT, { 0x4101 }, 0x12 },
	{ OV8830_8BIT, { 0x4104 }, 0x5b },
	{ OV8830_8BIT, { 0x4307 }, 0x30 },
	{ OV8830_8BIT, { 0x4315 }, 0x00 },
	{ OV8830_8BIT, { 0x4511 }, 0x05 },
	{ OV8830_8BIT, { 0x4512 }, 0x00 },
	{ OV8830_8BIT, { 0x4805 }, 0x01 },
	{ OV8830_8BIT, { 0x4806 }, 0x00 },
	{ OV8830_8BIT, { 0x481f }, 0x36 },
	{ OV8830_8BIT, { 0x4831 }, 0x6c },
	{ OV8830_8BIT, { 0x4837 }, 0x0d },
	{ OV8830_8BIT, { 0x4a00 }, 0xaa },
	{ OV8830_8BIT, { 0x4a03 }, 0x01 },
	{ OV8830_8BIT, { 0x4a05 }, 0x08 },
	{ OV8830_8BIT, { 0x4a0a }, 0x88 },
	{ OV8830_8BIT, { 0x5000 }, 0x06 },
	{ OV8830_8BIT, { 0x5001 }, 0x01 },
	{ OV8830_8BIT, { 0x5002 }, 0x80 },
	{ OV8830_8BIT, { 0x5003 }, 0x20 },
	{ OV8830_8BIT, { 0x5013 }, 0x00 },
	{ OV8830_8BIT, { 0x5046 }, 0x4a },
	{ OV8830_8BIT, { 0x5780 }, 0x1c },
	{ OV8830_8BIT, { 0x5786 }, 0x20 },
	{ OV8830_8BIT, { 0x5787 }, 0x10 },
	{ OV8830_8BIT, { 0x5788 }, 0x18 },
	{ OV8830_8BIT, { 0x578a }, 0x04 },
	{ OV8830_8BIT, { 0x578b }, 0x02 },
	{ OV8830_8BIT, { 0x578c }, 0x02 },
	{ OV8830_8BIT, { 0x578e }, 0x06 },
	{ OV8830_8BIT, { 0x578f }, 0x02 },
	{ OV8830_8BIT, { 0x5790 }, 0x02 },
	{ OV8830_8BIT, { 0x5791 }, 0xff },
	{ OV8830_8BIT, { 0x5a08 }, 0x02 },
	{ OV8830_8BIT, { 0x5e00 }, 0x00 },
	{ OV8830_8BIT, { 0x5e10 }, 0x0c },
	{ OV8830_TOK_TERM, {0}, 0}
};

/* 40 RES_1600x1200_BIN_48fps_4lane OV8830_AM35.ovd */
static const struct ov8830_reg ov8830_mode_1600[] = {
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0103 }, 0x01 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0102 }, 0x01 },
	{ OV8830_8BIT, { 0x3000 }, 0x00 },
	{ OV8830_8BIT, { 0x3001 }, 0x2A },
	{ OV8830_8BIT, { 0x3002 }, 0x88 },
	{ OV8830_8BIT, { 0x3003 }, 0x00 },
	{ OV8830_8BIT, { 0x3004 }, 0x00 },
	{ OV8830_8BIT, { 0x3005 }, 0x00 },
	{ OV8830_8BIT, { 0x3006 }, 0x00 },
	{ OV8830_8BIT, { 0x3007 }, 0x00 },
	{ OV8830_8BIT, { 0x3008 }, 0x00 },
	{ OV8830_8BIT, { 0x3009 }, 0x00 },
	{ OV8830_8BIT, { 0x3011 }, 0x41 },
	{ OV8830_8BIT, { 0x3012 }, 0x08 },
	{ OV8830_8BIT, { 0x3013 }, 0x10 },
	{ OV8830_8BIT, { 0x3014 }, 0x00 },
	{ OV8830_8BIT, { 0x3015 }, 0x08 },
	{ OV8830_8BIT, { 0x3016 }, 0xF0 },
	{ OV8830_8BIT, { 0x3017 }, 0xF0 },
	{ OV8830_8BIT, { 0x3018 }, 0xF0 },
	{ OV8830_8BIT, { 0x301B }, 0xB4 },
	{ OV8830_8BIT, { 0x301D }, 0x02 },
	{ OV8830_8BIT, { 0x3021 }, 0x00 },
	{ OV8830_8BIT, { 0x3022 }, 0x00 },
	{ OV8830_8BIT, { 0x3024 }, 0x00 },
	{ OV8830_8BIT, { 0x3026 }, 0x00 },
	{ OV8830_8BIT, { 0x3027 }, 0x00 },
	{ OV8830_8BIT, { 0x3081 }, 0x02 },
	{ OV8830_8BIT, { 0x3083 }, 0x01 },
	{ OV8830_8BIT, { 0x3090 }, 0x02	},	/* PLL2_prediv */
	{ OV8830_8BIT, { 0x3091 }, 22	},	/* PLL2_multiplier */
	{ OV8830_8BIT, { 0x3092 }, 0x00	},	/* PLL2_divs */
	{ OV8830_8BIT, { 0x3093 }, 0x00	},	/* PLL2_seld5 */
	{ OV8830_8BIT, { 0x3098 }, 0x03	},	/* PLL3_prediv */
	{ OV8830_8BIT, { 0x3099 }, 19	},	/* PLL3_mult2 */
	{ OV8830_8BIT, { 0x309A }, 0x00	},	/* PLL3_divs */
	{ OV8830_8BIT, { 0x309B }, 0x00	},	/* PLL3_div */
	{ OV8830_8BIT, { 0x309C }, 1	},	/* PLL3_div */
	{ OV8830_8BIT, { 0x30A2 }, 0x01 },
	{ OV8830_8BIT, { 0x30B0 }, 0x05 },
	{ OV8830_8BIT, { 0x30B2 }, 0x00 },
	{ OV8830_8BIT, { 0x30B3 }, 93	},	/* PLL1_multiplier */
	{ OV8830_8BIT, { 0x30B4 }, 0x03	},	/* PLL1_prediv */
	{ OV8830_8BIT, { 0x30B5 }, 0x04	},	/* PLL1_op_pix_div */
	{ OV8830_8BIT, { 0x30B6 }, 0x01	},	/* PLL1_op_sys_div */
	{ OV8830_8BIT, { 0x3104 }, 0xA1 },
	{ OV8830_8BIT, { 0x3106 }, 0x01 },
	{ OV8830_8BIT, { 0x3300 }, 0x00 },
	{ OV8830_8BIT, { 0x3400 }, 0x04 },
	{ OV8830_8BIT, { 0x3401 }, 0x00 },
	{ OV8830_8BIT, { 0x3402 }, 0x04 },
	{ OV8830_8BIT, { 0x3403 }, 0x00 },
	{ OV8830_8BIT, { 0x3404 }, 0x04 },
	{ OV8830_8BIT, { 0x3405 }, 0x00 },
	{ OV8830_8BIT, { 0x3406 }, 0x01 },
	{ OV8830_8BIT, { 0x3500 }, 0x00 },
	{ OV8830_8BIT, { 0x3501 }, 0x16 },
	{ OV8830_8BIT, { 0x3502 }, 0xf3 },
	{ OV8830_8BIT, { 0x3503 }, 0x07 },
	{ OV8830_8BIT, { 0x3504 }, 0x00 },
	{ OV8830_8BIT, { 0x3505 }, 0x30 },
	{ OV8830_8BIT, { 0x3506 }, 0x00 },
	{ OV8830_8BIT, { 0x3507 }, 0x08 },
	{ OV8830_8BIT, { 0x3508 }, 0x80 },
	{ OV8830_8BIT, { 0x3509 }, 0x10 },
	{ OV8830_8BIT, { 0x350A }, 0x00 },
	{ OV8830_8BIT, { 0x350B }, 0x38 },
	{ OV8830_8BIT, { 0x350C }, 0x00 },
	{ OV8830_8BIT, { 0x350D }, 0x00 },
	{ OV8830_8BIT, { 0x3600 }, 0x78 },
	{ OV8830_8BIT, { 0x3601 }, 0x02 },
	{ OV8830_8BIT, { 0x3602 }, 0x1C },
	{ OV8830_8BIT, { 0x3604 }, 0x38 },
	{ OV8830_8BIT, { 0x3620 }, 0x64 },
	{ OV8830_8BIT, { 0x3621 }, 0xB5 },
	{ OV8830_8BIT, { 0x3622 }, 0x03 },
	{ OV8830_8BIT, { 0x3625 }, 0x64 },
	{ OV8830_8BIT, { 0x3630 }, 0x55 },
	{ OV8830_8BIT, { 0x3631 }, 0xD2 },
	{ OV8830_8BIT, { 0x3632 }, 0x00 },
	{ OV8830_8BIT, { 0x3633 }, 0x34 },
	{ OV8830_8BIT, { 0x3634 }, 0x03 },
	{ OV8830_8BIT, { 0x3660 }, 0x80 },
	{ OV8830_8BIT, { 0x3662 }, 0x10 },
	{ OV8830_8BIT, { 0x3665 }, 0x00 },
	{ OV8830_8BIT, { 0x3666 }, 0x00 },
	{ OV8830_8BIT, { 0x3667 }, 0x00 },
	{ OV8830_8BIT, { 0x366A }, 0x80 },
	{ OV8830_8BIT, { 0x366C }, 0x00 },
	{ OV8830_8BIT, { 0x366D }, 0x00 },
	{ OV8830_8BIT, { 0x366E }, 0x00 },
	{ OV8830_8BIT, { 0x366F }, 0x20 },
	{ OV8830_8BIT, { 0x3680 }, 0xE0 },
	{ OV8830_8BIT, { 0x3681 }, 0x00 },
	{ OV8830_8BIT, { 0x3701 }, 0x14 },
	{ OV8830_8BIT, { 0x3702 }, 0xBF },
	{ OV8830_8BIT, { 0x3703 }, 0x8C },
	{ OV8830_8BIT, { 0x3704 }, 0x78 },
	{ OV8830_8BIT, { 0x3705 }, 0x02 },
	{ OV8830_8BIT, { 0x3708 }, 0xE4 },
	{ OV8830_8BIT, { 0x3709 }, 0x03 },
	{ OV8830_8BIT, { 0x370A }, 0x00 },
	{ OV8830_8BIT, { 0x370B }, 0x20 },
	{ OV8830_8BIT, { 0x370C }, 0x0C },
	{ OV8830_8BIT, { 0x370D }, 0x11 },
	{ OV8830_8BIT, { 0x370E }, 0x00 },
	{ OV8830_8BIT, { 0x370F }, 0x00 },
	{ OV8830_8BIT, { 0x3710 }, 0x00 },
	{ OV8830_8BIT, { 0x371C }, 0x01 },
	{ OV8830_8BIT, { 0x371F }, 0x0C },
	{ OV8830_8BIT, { 0x3721 }, 0x00 },
	{ OV8830_8BIT, { 0x3724 }, 0x10 },
	{ OV8830_8BIT, { 0x3726 }, 0x00 },
	{ OV8830_8BIT, { 0x372A }, 0x01 },
	{ OV8830_8BIT, { 0x3730 }, 0x18 },
	{ OV8830_8BIT, { 0x3738 }, 0x22 },
	{ OV8830_8BIT, { 0x3739 }, 0x08 },
	{ OV8830_8BIT, { 0x373A }, 0x51 },
	{ OV8830_8BIT, { 0x373B }, 0x02 },
	{ OV8830_8BIT, { 0x373C }, 0x20 },
	{ OV8830_8BIT, { 0x373F }, 0x02 },
	{ OV8830_8BIT, { 0x3740 }, 0x42 },
	{ OV8830_8BIT, { 0x3741 }, 0x02 },
	{ OV8830_8BIT, { 0x3742 }, 0x18 },
	{ OV8830_8BIT, { 0x3743 }, 0x01 },
	{ OV8830_8BIT, { 0x3744 }, 0x02 },
	{ OV8830_8BIT, { 0x3747 }, 0x10 },
	{ OV8830_8BIT, { 0x374C }, 0x04 },
	{ OV8830_8BIT, { 0x3751 }, 0xF0 },
	{ OV8830_8BIT, { 0x3752 }, 0x00 },
	{ OV8830_8BIT, { 0x3753 }, 0x00 },
	{ OV8830_8BIT, { 0x3754 }, 0xC0 },
	{ OV8830_8BIT, { 0x3755 }, 0x00 },
	{ OV8830_8BIT, { 0x3756 }, 0x1A },
	{ OV8830_8BIT, { 0x3758 }, 0x00 },
	{ OV8830_8BIT, { 0x3759 }, 0x0F },
	{ OV8830_8BIT, { 0x375C }, 0x04 },
	{ OV8830_8BIT, { 0x3767 }, 0x01 },
	{ OV8830_8BIT, { 0x376B }, 0x44 },
	{ OV8830_8BIT, { 0x3774 }, 0x10 },
	{ OV8830_8BIT, { 0x3776 }, 0x00 },
	{ OV8830_8BIT, { 0x377F }, 0x08 },
	{ OV8830_8BIT, { 0x3780 }, 0x22 },
	{ OV8830_8BIT, { 0x3781 }, 0x0C },
	{ OV8830_8BIT, { 0x3784 }, 0x2C },
	{ OV8830_8BIT, { 0x3785 }, 0x1E },
	{ OV8830_8BIT, { 0x378F }, 0xF5 },
	{ OV8830_8BIT, { 0x3791 }, 0xB0 },
	{ OV8830_8BIT, { 0x3795 }, 0x00 },
	{ OV8830_8BIT, { 0x3796 }, 0x64 },
	{ OV8830_8BIT, { 0x3797 }, 0x11 },
	{ OV8830_8BIT, { 0x3798 }, 0x30 },
	{ OV8830_8BIT, { 0x3799 }, 0x41 },
	{ OV8830_8BIT, { 0x379A }, 0x07 },
	{ OV8830_8BIT, { 0x379B }, 0xB0 },
	{ OV8830_8BIT, { 0x379C }, 0x0C },
	{ OV8830_8BIT, { 0x37C0 }, 0x00 },
	{ OV8830_8BIT, { 0x37C1 }, 0x00 },
	{ OV8830_8BIT, { 0x37C2 }, 0x00 },
	{ OV8830_8BIT, { 0x37C3 }, 0x00 },
	{ OV8830_8BIT, { 0x37C4 }, 0x00 },
	{ OV8830_8BIT, { 0x37C5 }, 0x00 },
	{ OV8830_8BIT, { 0x37C6 }, 0xA0 },
	{ OV8830_8BIT, { 0x37C7 }, 0x00 },
	{ OV8830_8BIT, { 0x37C8 }, 0x00 },
	{ OV8830_8BIT, { 0x37C9 }, 0x00 },
	{ OV8830_8BIT, { 0x37CA }, 0x00 },
	{ OV8830_8BIT, { 0x37CB }, 0x00 },
	{ OV8830_8BIT, { 0x37CC }, 0x00 },
	{ OV8830_8BIT, { 0x37CD }, 0x00 },
	{ OV8830_8BIT, { 0x37CE }, 0x01 },
	{ OV8830_8BIT, { 0x37CF }, 0x00 },
	{ OV8830_8BIT, { 0x37D1 }, 0x01 },
	{ OV8830_8BIT, { 0x37DE }, 0x00 },
	{ OV8830_8BIT, { 0x37DF }, 0x00 },
	{ OV8830_8BIT, { 0x3800 }, 0x00 },
	{ OV8830_8BIT, { 0x3801 }, 0x28 },
	{ OV8830_8BIT, { 0x3802 }, 0x00 },
	{ OV8830_8BIT, { 0x3803 }, 0x20 },
	{ OV8830_8BIT, { 0x3804 }, 0x0C },
	{ OV8830_8BIT, { 0x3805 }, 0xB7 },
	{ OV8830_8BIT, { 0x3806 }, 0x09 },
	{ OV8830_8BIT, { 0x3807 }, 0x8F },
	{ OV8830_8BIT, { 0x3808 }, 0x06 },
	{ OV8830_8BIT, { 0x3809 }, 0x40 },
	{ OV8830_8BIT, { 0x380A }, 0x04 },
	{ OV8830_8BIT, { 0x380B }, 0xB0 },
	{ OV8830_8BIT, { 0x380C }, 0x0E },
	{ OV8830_8BIT, { 0x380D }, 0x18 },
	{ OV8830_8BIT, { 0x380E }, 0x04 },
	{ OV8830_8BIT, { 0x380F }, 0xCC },
	{ OV8830_8BIT, { 0x3810 }, 0x00 },
	{ OV8830_8BIT, { 0x3811 }, 0x04 },
	{ OV8830_8BIT, { 0x3812 }, 0x00 },
	{ OV8830_8BIT, { 0x3813 }, 0x04 },
	{ OV8830_8BIT, { 0x3814 }, 0x31 },
	{ OV8830_8BIT, { 0x3815 }, 0x31 },
	{ OV8830_8BIT, { 0x3820 }, 0x11 },
	{ OV8830_8BIT, { 0x3821 }, 0x0F },
	{ OV8830_8BIT, { 0x3823 }, 0x00 },
	{ OV8830_8BIT, { 0x3824 }, 0x00 },
	{ OV8830_8BIT, { 0x3825 }, 0x00 },
	{ OV8830_8BIT, { 0x3826 }, 0x00 },
	{ OV8830_8BIT, { 0x3827 }, 0x00 },
	{ OV8830_8BIT, { 0x382A }, 0x04 },
	{ OV8830_8BIT, { 0x3A04 }, 0x04 },
	{ OV8830_8BIT, { 0x3A05 }, 0xC9 },
	{ OV8830_8BIT, { 0x3A06 }, 0x00 },
	{ OV8830_8BIT, { 0x3A07 }, 0xF8 },
	{ OV8830_8BIT, { 0x3A18 }, 0x00 },
	{ OV8830_8BIT, { 0x3A19 }, 0x00 },
	{ OV8830_8BIT, { 0x3B00 }, 0x00 },
	{ OV8830_8BIT, { 0x3B01 }, 0x00 },
	{ OV8830_8BIT, { 0x3B02 }, 0x00 },
	{ OV8830_8BIT, { 0x3B03 }, 0x00 },
	{ OV8830_8BIT, { 0x3B04 }, 0x00 },
	{ OV8830_8BIT, { 0x3B05 }, 0x00 },
	{ OV8830_8BIT, { 0x3D00 }, 0x00 },
	{ OV8830_8BIT, { 0x3D01 }, 0x00 },
	{ OV8830_8BIT, { 0x3D02 }, 0x00 },
	{ OV8830_8BIT, { 0x3D03 }, 0x00 },
	{ OV8830_8BIT, { 0x3D04 }, 0x00 },
	{ OV8830_8BIT, { 0x3D05 }, 0x00 },
	{ OV8830_8BIT, { 0x3D06 }, 0x00 },
	{ OV8830_8BIT, { 0x3D07 }, 0x00 },
	{ OV8830_8BIT, { 0x3D08 }, 0x00 },
	{ OV8830_8BIT, { 0x3D09 }, 0x00 },
	{ OV8830_8BIT, { 0x3D0A }, 0x00 },
	{ OV8830_8BIT, { 0x3D0B }, 0x00 },
	{ OV8830_8BIT, { 0x3D0C }, 0x00 },
	{ OV8830_8BIT, { 0x3D0D }, 0x00 },
	{ OV8830_8BIT, { 0x3D0E }, 0x00 },
	{ OV8830_8BIT, { 0x3D0F }, 0x00 },
	{ OV8830_8BIT, { 0x3D80 }, 0x00 },
	{ OV8830_8BIT, { 0x3D81 }, 0x00 },
	{ OV8830_8BIT, { 0x3D84 }, 0x00 },
	{ OV8830_8BIT, { 0x3E07 }, 0x20 },
	{ OV8830_8BIT, { 0x4000 }, 0x18 },
	{ OV8830_8BIT, { 0x4001 }, 0x04 },
	{ OV8830_8BIT, { 0x4002 }, 0x45 },
	{ OV8830_8BIT, { 0x4004 }, 0x02 },
	{ OV8830_8BIT, { 0x4005 }, 0x18 },
	{ OV8830_8BIT, { 0x4006 }, 0x16 },
	{ OV8830_8BIT, { 0x4008 }, 0x20 },
	{ OV8830_8BIT, { 0x4009 }, 0x10 },
	{ OV8830_8BIT, { 0x400C }, 0x00 },
	{ OV8830_8BIT, { 0x400D }, 0x00 },
	{ OV8830_8BIT, { 0x4058 }, 0x01 },
	{ OV8830_8BIT, { 0x4101 }, 0x12 },
	{ OV8830_8BIT, { 0x4104 }, 0x5B },
	{ OV8830_8BIT, { 0x4303 }, 0x00 },
	{ OV8830_8BIT, { 0x4304 }, 0x08 },
	{ OV8830_8BIT, { 0x4307 }, 0x30 },
	{ OV8830_8BIT, { 0x4315 }, 0x00 },
	{ OV8830_8BIT, { 0x4511 }, 0x05 },
	{ OV8830_8BIT, { 0x4512 }, 0x00 },
	{ OV8830_8BIT, { 0x4750 }, 0x00 },
	{ OV8830_8BIT, { 0x4751 }, 0x00 },
	{ OV8830_8BIT, { 0x4752 }, 0x00 },
	{ OV8830_8BIT, { 0x4753 }, 0x00 },
	{ OV8830_8BIT, { 0x4805 }, 0x01 },
	{ OV8830_8BIT, { 0x4806 }, 0x00 },
	{ OV8830_8BIT, { 0x481F }, 0x36 },
	{ OV8830_8BIT, { 0x4831 }, 0x6C },
	{ OV8830_8BIT, { 0x4837 }, 0x0D },
	{ OV8830_8BIT, { 0x4A00 }, 0xAA },
	{ OV8830_8BIT, { 0x4A03 }, 0x01 },
	{ OV8830_8BIT, { 0x4A05 }, 0x08 },
	{ OV8830_8BIT, { 0x4A0A }, 0x88 },
	{ OV8830_8BIT, { 0x5000 }, 0x06 },
	{ OV8830_8BIT, { 0x5001 }, 0x01 },
	{ OV8830_8BIT, { 0x5002 }, 0x80 },
	{ OV8830_8BIT, { 0x5003 }, 0x20 },
	{ OV8830_8BIT, { 0x5013 }, 0x00 },
	{ OV8830_8BIT, { 0x5046 }, 0x4A },
	{ OV8830_8BIT, { 0x5780 }, 0x1C },
	{ OV8830_8BIT, { 0x5786 }, 0x20 },
	{ OV8830_8BIT, { 0x5787 }, 0x10 },
	{ OV8830_8BIT, { 0x5788 }, 0x18 },
	{ OV8830_8BIT, { 0x578A }, 0x04 },
	{ OV8830_8BIT, { 0x578B }, 0x02 },
	{ OV8830_8BIT, { 0x578C }, 0x02 },
	{ OV8830_8BIT, { 0x578E }, 0x06 },
	{ OV8830_8BIT, { 0x578F }, 0x02 },
	{ OV8830_8BIT, { 0x5790 }, 0x02 },
	{ OV8830_8BIT, { 0x5791 }, 0xFF },
	{ OV8830_8BIT, { 0x5A08 }, 0x02 },
	{ OV8830_8BIT, { 0x5E00 }, 0x00 },
	{ OV8830_8BIT, { 0x5E10 }, 0x0C },
	{ OV8830_TOK_TERM, {0}, 0}
};

/* 30 RES_3264x2448_15fps_2lane OV8830_AM35.ovd */
static const struct ov8830_reg ov8830_mode2[] = {
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0103 }, 0x01 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0100 }, 0x00 },
	{ OV8830_8BIT, { 0x0102 }, 0x01 },
	{ OV8830_8BIT, { 0x3000 }, 0x00 },
	{ OV8830_8BIT, { 0x3001 }, 0x2a },
	{ OV8830_8BIT, { 0x3002 }, 0x88 },
	{ OV8830_8BIT, { 0x3003 }, 0x00 },
	{ OV8830_8BIT, { 0x3004 }, 0x00 },
	{ OV8830_8BIT, { 0x3005 }, 0x00 },
	{ OV8830_8BIT, { 0x3006 }, 0x00 },
	{ OV8830_8BIT, { 0x3007 }, 0x00 },
	{ OV8830_8BIT, { 0x3008 }, 0x00 },
	{ OV8830_8BIT, { 0x3009 }, 0x00 },
	{ OV8830_8BIT, { 0x3011 }, 0x41 },
	{ OV8830_8BIT, { 0x3012 }, 0x08 },
	{ OV8830_8BIT, { 0x3013 }, 0x10 },
	{ OV8830_8BIT, { 0x3014 }, 0x00 },
	{ OV8830_8BIT, { 0x3015 }, 0x08 },
	{ OV8830_8BIT, { 0x3016 }, 0xf0 },
	{ OV8830_8BIT, { 0x3017 }, 0xf0 },
	{ OV8830_8BIT, { 0x3018 }, 0xf0 },
	{ OV8830_8BIT, { 0x301b }, 0xb4 },
	{ OV8830_8BIT, { 0x301d }, 0x02 },
	{ OV8830_8BIT, { 0x3021 }, 0x00 },
	{ OV8830_8BIT, { 0x3022 }, 0x00 },
	{ OV8830_8BIT, { 0x3024 }, 0x00 },
	{ OV8830_8BIT, { 0x3026 }, 0x00 },
	{ OV8830_8BIT, { 0x3027 }, 0x00 },
	{ OV8830_8BIT, { 0x3081 }, 0x02 },
	{ OV8830_8BIT, { 0x3083 }, 0x01 },
	{ OV8830_8BIT, { 0x3090 }, 0x03 },
	{ OV8830_8BIT, { 0x3091 }, 42 },
	{ OV8830_8BIT, { 0x3092 }, 0x01 },
	{ OV8830_8BIT, { 0x3093 }, 0x00 },
	{ OV8830_8BIT, { 0x3098 }, 0x03 },
	{ OV8830_8BIT, { 0x3099 }, 19 },
	{ OV8830_8BIT, { 0x309a }, 0x00 },
	{ OV8830_8BIT, { 0x309b }, 0x00 },
	{ OV8830_8BIT, { 0x309c }, 0x01 },
	{ OV8830_8BIT, { 0x30a2 }, 0x01 },
	{ OV8830_8BIT, { 0x30b0 }, 0x05 },
	{ OV8830_8BIT, { 0x30b2 }, 0x00 },
	{ OV8830_8BIT, { 0x30b3 }, 106 },
	{ OV8830_8BIT, { 0x30b4 }, 0x03 },
	{ OV8830_8BIT, { 0x30b5 }, 0x04 },
	{ OV8830_8BIT, { 0x30b6 }, 0x01 },
	{ OV8830_8BIT, { 0x3104 }, 0xa1 },
	{ OV8830_8BIT, { 0x3106 }, 0x01 },
	{ OV8830_8BIT, { 0x3300 }, 0x00 },
	{ OV8830_8BIT, { 0x3400 }, 0x04 },
	{ OV8830_8BIT, { 0x3401 }, 0x00 },
	{ OV8830_8BIT, { 0x3402 }, 0x04 },
	{ OV8830_8BIT, { 0x3403 }, 0x00 },
	{ OV8830_8BIT, { 0x3404 }, 0x04 },
	{ OV8830_8BIT, { 0x3405 }, 0x00 },
	{ OV8830_8BIT, { 0x3406 }, 0x01 },
	{ OV8830_8BIT, { 0x3500 }, 0x00 },
	{ OV8830_8BIT, { 0x3501 }, 0x30 },
	{ OV8830_8BIT, { 0x3502 }, 0x00 },
	{ OV8830_8BIT, { 0x3503 }, 0x07 },
	{ OV8830_8BIT, { 0x3504 }, 0x00 },
	{ OV8830_8BIT, { 0x3505 }, 0x30 },
	{ OV8830_8BIT, { 0x3506 }, 0x00 },
	{ OV8830_8BIT, { 0x3507 }, 0x10 },
	{ OV8830_8BIT, { 0x3508 }, 0x80 },
	{ OV8830_8BIT, { 0x3509 }, 0x10 },
	{ OV8830_8BIT, { 0x350a }, 0x00 },
	{ OV8830_8BIT, { 0x350b }, 0x38 },
	{ OV8830_8BIT, { 0x350c }, 0x00 },
	{ OV8830_8BIT, { 0x350d }, 0x00 },
	{ OV8830_8BIT, { 0x3600 }, 0x78 },
	{ OV8830_8BIT, { 0x3601 }, 0x02 },
	{ OV8830_8BIT, { 0x3602 }, 0x1c },
	{ OV8830_8BIT, { 0x3604 }, 0x38 },
	{ OV8830_8BIT, { 0x3620 }, 0x64 },
	{ OV8830_8BIT, { 0x3621 }, 0xb5 },
	{ OV8830_8BIT, { 0x3622 }, 0x03 },
	{ OV8830_8BIT, { 0x3625 }, 0x64 },
	{ OV8830_8BIT, { 0x3630 }, 0x55 },
	{ OV8830_8BIT, { 0x3631 }, 0xd2 },
	{ OV8830_8BIT, { 0x3632 }, 0x00 },
	{ OV8830_8BIT, { 0x3633 }, 0x34 },
	{ OV8830_8BIT, { 0x3634 }, 0x03 },
	{ OV8830_8BIT, { 0x3660 }, 0x80 },
	{ OV8830_8BIT, { 0x3662 }, 0x10 },
	{ OV8830_8BIT, { 0x3665 }, 0x00 },
	{ OV8830_8BIT, { 0x3666 }, 0x00 },
	{ OV8830_8BIT, { 0x3667 }, 0x00 },
	{ OV8830_8BIT, { 0x366a }, 0x80 },
	{ OV8830_8BIT, { 0x366c }, 0x00 },
	{ OV8830_8BIT, { 0x366d }, 0x00 },
	{ OV8830_8BIT, { 0x366e }, 0x00 },
	{ OV8830_8BIT, { 0x366f }, 0x20 },
	{ OV8830_8BIT, { 0x3680 }, 0xe0 },
	{ OV8830_8BIT, { 0x3681 }, 0x00 },
	{ OV8830_8BIT, { 0x3701 }, 0x14 },
	{ OV8830_8BIT, { 0x3702 }, 0xbf },
	{ OV8830_8BIT, { 0x3703 }, 0x8c },
	{ OV8830_8BIT, { 0x3704 }, 0x78 },
	{ OV8830_8BIT, { 0x3705 }, 0x02 },
	{ OV8830_8BIT, { 0x3708 }, 0xe4 },
	{ OV8830_8BIT, { 0x3709 }, 0x03 },
	{ OV8830_8BIT, { 0x370a }, 0x00 },
	{ OV8830_8BIT, { 0x370b }, 0x20 },
	{ OV8830_8BIT, { 0x370c }, 0x0c },
	{ OV8830_8BIT, { 0x370d }, 0x11 },
	{ OV8830_8BIT, { 0x370e }, 0x00 },
	{ OV8830_8BIT, { 0x370f }, 0x00 },
	{ OV8830_8BIT, { 0x3710 }, 0x00 },
	{ OV8830_8BIT, { 0x371c }, 0x01 },
	{ OV8830_8BIT, { 0x371f }, 0x0c },
	{ OV8830_8BIT, { 0x3721 }, 0x00 },
	{ OV8830_8BIT, { 0x3724 }, 0x10 },
	{ OV8830_8BIT, { 0x3726 }, 0x00 },
	{ OV8830_8BIT, { 0x372a }, 0x01 },
	{ OV8830_8BIT, { 0x3730 }, 0x18 },
	{ OV8830_8BIT, { 0x3738 }, 0x22 },
	{ OV8830_8BIT, { 0x3739 }, 0x08 },
	{ OV8830_8BIT, { 0x373a }, 0x51 },
	{ OV8830_8BIT, { 0x373b }, 0x02 },
	{ OV8830_8BIT, { 0x373c }, 0x20 },
	{ OV8830_8BIT, { 0x373f }, 0x02 },
	{ OV8830_8BIT, { 0x3740 }, 0x42 },
	{ OV8830_8BIT, { 0x3741 }, 0x02 },
	{ OV8830_8BIT, { 0x3742 }, 0x18 },
	{ OV8830_8BIT, { 0x3743 }, 0x01 },
	{ OV8830_8BIT, { 0x3744 }, 0x02 },
	{ OV8830_8BIT, { 0x3747 }, 0x10 },
	{ OV8830_8BIT, { 0x374c }, 0x04 },
	{ OV8830_8BIT, { 0x3751 }, 0xf0 },
	{ OV8830_8BIT, { 0x3752 }, 0x00 },
	{ OV8830_8BIT, { 0x3753 }, 0x00 },
	{ OV8830_8BIT, { 0x3754 }, 0xc0 },
	{ OV8830_8BIT, { 0x3755 }, 0x00 },
	{ OV8830_8BIT, { 0x3756 }, 0x1a },
	{ OV8830_8BIT, { 0x3758 }, 0x00 },
	{ OV8830_8BIT, { 0x3759 }, 0x0f },
	{ OV8830_8BIT, { 0x375c }, 0x04 },
	{ OV8830_8BIT, { 0x3767 }, 0x01 },
	{ OV8830_8BIT, { 0x376b }, 0x44 },
	{ OV8830_8BIT, { 0x3774 }, 0x10 },
	{ OV8830_8BIT, { 0x3776 }, 0x00 },
	{ OV8830_8BIT, { 0x377f }, 0x08 },
	{ OV8830_8BIT, { 0x3780 }, 0x22 },
	{ OV8830_8BIT, { 0x3781 }, 0x0c },
	{ OV8830_8BIT, { 0x3784 }, 0x2c },
	{ OV8830_8BIT, { 0x3785 }, 0x1e },
	{ OV8830_8BIT, { 0x378f }, 0xf5 },
	{ OV8830_8BIT, { 0x3791 }, 0xb0 },
	{ OV8830_8BIT, { 0x3795 }, 0x00 },
	{ OV8830_8BIT, { 0x3796 }, 0x64 },
	{ OV8830_8BIT, { 0x3797 }, 0x11 },
	{ OV8830_8BIT, { 0x3798 }, 0x30 },
	{ OV8830_8BIT, { 0x3799 }, 0x41 },
	{ OV8830_8BIT, { 0x379a }, 0x07 },
	{ OV8830_8BIT, { 0x379b }, 0xb0 },
	{ OV8830_8BIT, { 0x379c }, 0x0c },
	{ OV8830_8BIT, { 0x37c0 }, 0x00 },
	{ OV8830_8BIT, { 0x37c1 }, 0x00 },
	{ OV8830_8BIT, { 0x37c2 }, 0x00 },
	{ OV8830_8BIT, { 0x37c3 }, 0x00 },
	{ OV8830_8BIT, { 0x37c4 }, 0x00 },
	{ OV8830_8BIT, { 0x37c5 }, 0x00 },
	{ OV8830_8BIT, { 0x37c6 }, 0xa0 },
	{ OV8830_8BIT, { 0x37c7 }, 0x00 },
	{ OV8830_8BIT, { 0x37c8 }, 0x00 },
	{ OV8830_8BIT, { 0x37c9 }, 0x00 },
	{ OV8830_8BIT, { 0x37ca }, 0x00 },
	{ OV8830_8BIT, { 0x37cb }, 0x00 },
	{ OV8830_8BIT, { 0x37cc }, 0x00 },
	{ OV8830_8BIT, { 0x37cd }, 0x00 },
	{ OV8830_8BIT, { 0x37ce }, 0x01 },
	{ OV8830_8BIT, { 0x37cf }, 0x00 },
	{ OV8830_8BIT, { 0x37d1 }, 0x01 },
	{ OV8830_8BIT, { 0x37de }, 0x00 },
	{ OV8830_8BIT, { 0x37df }, 0x00 },
	{ OV8830_8BIT, { 0x3800 }, 0x00 },
	{ OV8830_8BIT, { 0x3801 }, 0x04 },
	{ OV8830_8BIT, { 0x3802 }, 0x00 },
	{ OV8830_8BIT, { 0x3803 }, 0x04 },
	{ OV8830_8BIT, { 0x3804 }, 0x0c },
	{ OV8830_8BIT, { 0x3805 }, 0xdb },
	{ OV8830_8BIT, { 0x3806 }, 0x09 },
	{ OV8830_8BIT, { 0x3807 }, 0xab },
	{ OV8830_8BIT, { 0x3808 }, 0x0c },
	{ OV8830_8BIT, { 0x3809 }, 0xd0 },
	{ OV8830_8BIT, { 0x380a }, 0x09 },
	{ OV8830_8BIT, { 0x380b }, 0xa0 },
	{ OV8830_8BIT, { 0x380c }, 0x0e },
	{ OV8830_8BIT, { 0x380d }, 0x18 },
	{ OV8830_8BIT, { 0x380e }, 0x0a },
	{ OV8830_8BIT, { 0x380f }, 0x00 },
	{ OV8830_8BIT, { 0x3810 }, 0x00 },
	{ OV8830_8BIT, { 0x3811 }, 0x04 },
	{ OV8830_8BIT, { 0x3812 }, 0x00 },
	{ OV8830_8BIT, { 0x3813 }, 0x04 },
	{ OV8830_8BIT, { 0x3814 }, 0x11 },
	{ OV8830_8BIT, { 0x3815 }, 0x11 },
	{ OV8830_8BIT, { 0x3820 }, 0x10 },
	{ OV8830_8BIT, { 0x3821 }, 0x0e },
	{ OV8830_8BIT, { 0x3823 }, 0x00 },
	{ OV8830_8BIT, { 0x3824 }, 0x00 },
	{ OV8830_8BIT, { 0x3825 }, 0x00 },
	{ OV8830_8BIT, { 0x3826 }, 0x00 },
	{ OV8830_8BIT, { 0x3827 }, 0x00 },
	{ OV8830_8BIT, { 0x382a }, 0x04 },
	{ OV8830_8BIT, { 0x3a04 }, 0x09 },
	{ OV8830_8BIT, { 0x3a05 }, 0xa9 },
	{ OV8830_8BIT, { 0x3a06 }, 0x00 },
	{ OV8830_8BIT, { 0x3a07 }, 0xf8 },
	{ OV8830_8BIT, { 0x3a18 }, 0x00 },
	{ OV8830_8BIT, { 0x3a19 }, 0x00 },
	{ OV8830_8BIT, { 0x3b00 }, 0x00 },
	{ OV8830_8BIT, { 0x3b01 }, 0x00 },
	{ OV8830_8BIT, { 0x3b02 }, 0x00 },
	{ OV8830_8BIT, { 0x3b03 }, 0x00 },
	{ OV8830_8BIT, { 0x3b04 }, 0x00 },
	{ OV8830_8BIT, { 0x3b05 }, 0x00 },
	{ OV8830_8BIT, { 0x3d00 }, 0x00 },
	{ OV8830_8BIT, { 0x3d01 }, 0x00 },
	{ OV8830_8BIT, { 0x3d02 }, 0x00 },
	{ OV8830_8BIT, { 0x3d03 }, 0x00 },
	{ OV8830_8BIT, { 0x3d04 }, 0x00 },
	{ OV8830_8BIT, { 0x3d05 }, 0x00 },
	{ OV8830_8BIT, { 0x3d06 }, 0x00 },
	{ OV8830_8BIT, { 0x3d07 }, 0x00 },
	{ OV8830_8BIT, { 0x3d08 }, 0x00 },
	{ OV8830_8BIT, { 0x3d09 }, 0x00 },
	{ OV8830_8BIT, { 0x3d0a }, 0x00 },
	{ OV8830_8BIT, { 0x3d0b }, 0x00 },
	{ OV8830_8BIT, { 0x3d0c }, 0x00 },
	{ OV8830_8BIT, { 0x3d0d }, 0x00 },
	{ OV8830_8BIT, { 0x3d0e }, 0x00 },
	{ OV8830_8BIT, { 0x3d0f }, 0x00 },
	{ OV8830_8BIT, { 0x3d80 }, 0x00 },
	{ OV8830_8BIT, { 0x3d81 }, 0x00 },
	{ OV8830_8BIT, { 0x3d84 }, 0x00 },
	{ OV8830_8BIT, { 0x3e07 }, 0x20 },
	{ OV8830_8BIT, { 0x4000 }, 0x18 },
	{ OV8830_8BIT, { 0x4001 }, 0x04 },
	{ OV8830_8BIT, { 0x4002 }, 0x45 },
	{ OV8830_8BIT, { 0x4004 }, 0x02 },
	{ OV8830_8BIT, { 0x4005 }, 0x18 },
	{ OV8830_8BIT, { 0x4006 }, 0x16 },
	{ OV8830_8BIT, { 0x4008 }, 0x20 },
	{ OV8830_8BIT, { 0x4009 }, 0x10 },
	{ OV8830_8BIT, { 0x400c }, 0x00 },
	{ OV8830_8BIT, { 0x400d }, 0x00 },
	{ OV8830_8BIT, { 0x4058 }, 0x00 },
	{ OV8830_8BIT, { 0x4101 }, 0x12 },
	{ OV8830_8BIT, { 0x4104 }, 0x5b },
	{ OV8830_8BIT, { 0x4303 }, 0x00 },
	{ OV8830_8BIT, { 0x4304 }, 0x08 },
	{ OV8830_8BIT, { 0x4307 }, 0x30 },
	{ OV8830_8BIT, { 0x4315 }, 0x00 },
	{ OV8830_8BIT, { 0x4511 }, 0x05 },
	{ OV8830_8BIT, { 0x4512 }, 0x01 },
	{ OV8830_8BIT, { 0x4750 }, 0x00 },
	{ OV8830_8BIT, { 0x4751 }, 0x00 },
	{ OV8830_8BIT, { 0x4752 }, 0x00 },
	{ OV8830_8BIT, { 0x4753 }, 0x00 },
	{ OV8830_8BIT, { 0x4805 }, 0x01 },
	{ OV8830_8BIT, { 0x4806 }, 0x00 },
	{ OV8830_8BIT, { 0x481f }, 0x36 },
	{ OV8830_8BIT, { 0x4831 }, 0x6c },
	{ OV8830_8BIT, { 0x4837 }, 0x0c },
	{ OV8830_8BIT, { 0x4a00 }, 0xaa },
	{ OV8830_8BIT, { 0x4a03 }, 0x01 },
	{ OV8830_8BIT, { 0x4a05 }, 0x08 },
	{ OV8830_8BIT, { 0x4a0a }, 0x88 },
	{ OV8830_8BIT, { 0x5000 }, 0x06 },
	{ OV8830_8BIT, { 0x5001 }, 0x01 },
	{ OV8830_8BIT, { 0x5002 }, 0x80 },
	{ OV8830_8BIT, { 0x5003 }, 0x20 },
	{ OV8830_8BIT, { 0x5013 }, 0x00 },
	{ OV8830_8BIT, { 0x5046 }, 0x4a },
	{ OV8830_8BIT, { 0x5780 }, 0x1c },
	{ OV8830_8BIT, { 0x5786 }, 0x20 },
	{ OV8830_8BIT, { 0x5787 }, 0x10 },
	{ OV8830_8BIT, { 0x5788 }, 0x18 },
	{ OV8830_8BIT, { 0x578a }, 0x04 },
	{ OV8830_8BIT, { 0x578b }, 0x02 },
	{ OV8830_8BIT, { 0x578c }, 0x02 },
	{ OV8830_8BIT, { 0x578e }, 0x06 },
	{ OV8830_8BIT, { 0x578f }, 0x02 },
	{ OV8830_8BIT, { 0x5790 }, 0x02 },
	{ OV8830_8BIT, { 0x5791 }, 0xff },
	{ OV8830_8BIT, { 0x5a08 }, 0x02 },
	{ OV8830_8BIT, { 0x5e00 }, 0x00 },
	{ OV8830_8BIT, { 0x5e10 }, 0x0c },
	{ OV8830_TOK_TERM, {0}, 0}
};

/* divides a by b using half up rounding and div/0 prevention
 * (result is 0 if b == 0) */
#define divsave_rounded(a, b)	(((b) != 0) ? (((a)+((b)>>1))/(b)) : (-1))

typedef unsigned int sensor_register;
struct sensor_mode_data {
	sensor_register coarse_integration_time_min;
	sensor_register coarse_integration_time_max_margin;
	sensor_register fine_integration_time_min;
	sensor_register fine_integration_time_max_margin;
	sensor_register fine_integration_time_def;
	sensor_register frame_length_lines;
	sensor_register line_length_pck;
	sensor_register read_mode;
	int vt_pix_clk_freq_mhz;
};

/*
 * TODO: use debug parameter to actually define when debug messages should
 * be printed.
 */
static int debug;
static u16 real_model_id;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug messages");

struct ov8830_resolution ov8830_res_preview[] = {
	{
		 .desc =	"PREVIEW_30fps"	,
		 .width =	816	,
		 .height =	612	,
		 .fps =		30	,
		 .used =	0	,
		 .pixels_per_line = 0x20F0, /* consistent with regs arrays */
		 .lines_per_frame = 0x02F7, /* consistent with regs arrays */
		 .regs =	ov8830_mode1	,
	},
	{
		 .desc =	"MODE1600x1200"	,
		 .width =	1600	,
		 .height =	1200	,
		 .fps =		48	,
		 .used =	0	,
		 .pixels_per_line = 3608, /* consistent with regs arrays */
		 .lines_per_frame = 1228, /* consistent with regs arrays */
		 .regs =	ov8830_mode_1600	,
	},
	{
		.desc =        "STILL_8M_15fps"        ,
		.width =       3280    ,
		.height =      2464    ,
		.fps =         15      ,
		.used =        0       ,
		.pixels_per_line = 3608, /* consistent with regs arrays */
		.lines_per_frame = 2560, /* consistent with regs arrays */
		.regs =        ov8830_mode2  ,
	},
};

#define N_RES_PREVIEW (ARRAY_SIZE(ov8830_res_preview))

struct ov8830_resolution ov8830_res_still[] = {
	{
		 .desc =	"STILL_30fps"	,
		 .width =	816	,
		 .height =	612	,
		 .fps =		30	,
		 .used =	0	,
		 .pixels_per_line = 0x20F0, /* consistent with regs arrays */
		 .lines_per_frame = 0x02F7, /* consistent with regs arrays */
		 .regs =	ov8830_mode1	,
	},
	{
		 .desc =	"MODE1600x1200"	,
		 .width =	1600	,
		 .height =	1200	,
		 .fps =		48	,
		 .used =	0	,
		 .pixels_per_line = 3608, /* consistent with regs arrays */
		 .lines_per_frame = 1228, /* consistent with regs arrays */
		 .regs =	ov8830_mode_1600	,
	},
	{
		.desc =        "STILL_8M_15fps"        ,
		.width =       3280    ,
		.height =      2464    ,
		.fps =         15      ,
		.used =        0       ,
		.pixels_per_line = 3608, /* consistent with regs arrays */
		.lines_per_frame = 2560, /* consistent with regs arrays */
		.regs =        ov8830_mode2  ,
	},
};

#define N_RES_STILL (ARRAY_SIZE(ov8830_res_still))

struct ov8830_resolution ov8830_res_video[] = {
	{
		 .desc =	"VIDEO_30fps"	,
		 .width =	816	,
		 .height =	612	,
		 .fps =		30	,
		 .used =	0	,
		 .pixels_per_line = 0x20F0, /* consistent with regs arrays */
		 .lines_per_frame = 0x02F7, /* consistent with regs arrays */
		 .regs =	ov8830_mode1	,
	},
	{
		 .desc =	"MODE1600x1200"	,
		 .width =	1600	,
		 .height =	1200	,
		 .fps =		48	,
		 .used =	0	,
		 .pixels_per_line = 3608, /* consistent with regs arrays */
		 .lines_per_frame = 1228, /* consistent with regs arrays */
		 .regs =	ov8830_mode_1600	,
	},
	{
		.desc =        "STILL_8M_15fps"        ,
		.width =       3280    ,
		.height =      2464    ,
		.fps =         15      ,
		.used =        0       ,
		.pixels_per_line = 3608, /* consistent with regs arrays */
		.lines_per_frame = 2560, /* consistent with regs arrays */
		.regs =        ov8830_mode2  ,
	},
};

#define N_RES_VIDEO (ARRAY_SIZE(ov8830_res_video))

struct ov8830_resolution *ov8830_res = ov8830_res_preview;
static int N_RES = N_RES_PREVIEW;

static int
ov8830_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[OV8830_SHORT_MAX];
	int err, i;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	/* @len should be even when > 1 */
	if (len > OV8830_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0)
		goto error;

	/* high byte comes first */
	if (len == OV8830_8BIT) {
		*val = (u8)data[0];
	} else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int ov8830_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;
	int retry = 0;

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * It is said that Rev 2 sensor needs some delay here otherwise
	 * registers do not seem to load correctly. But tests show that
	 * removing the delay would not cause any in-stablility issue and the
	 * delay will cause serious performance down, so, removed previous
	 * mdelay(1) here.
	 */

	if (ret == num_msg)
		return 0;

	if (retry <= I2C_RETRY_COUNT) {
		dev_err(&client->dev, "retrying i2c write transfer... %d",
			retry);
		retry++;
		msleep(20);
		goto again;
	}

	return ret;
}

static int
ov8830_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != OV8830_8BIT && data_length != OV8830_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);

	if (data_length == OV8830_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* OV8830_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = be16_to_cpu(val);
	}

	ret = ov8830_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}


/**
 * ov8830_rmw_reg - Read/Modify/Write a value to a register in the sensor
 * device
 * @client: i2c driver client structure
 * @data_length: 8/16-bits length
 * @reg: register address
 * @mask: masked out bits
 * @set: bits set
 *
 * Read/modify/write a value to a register in the  sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov8830_rmw_reg(struct i2c_client *client, u16 data_length, u16 reg,
			   u16 mask, u16 set)
{
	int err;
	u16 val;

	/* Exit when no mask */
	if (mask == 0)
		return 0;

	/* @mask must not exceed data length */
	if (data_length == OV8830_8BIT && mask & ~0xff)
		return -EINVAL;

	err = ov8830_read_reg(client, data_length, reg, &val);
	if (err) {
		v4l2_err(client, "ov8830_rmw_reg error exit, read failed\n");
		return -EINVAL;
	}

	val &= ~mask;

	/*
	 * Perform the OR function if the @set exists.
	 * Shift @set value to target bit location. @set should set only
	 * bits included in @mask.
	 *
	 * REVISIT: This function expects @set to be non-shifted. Its shift
	 * value is then defined to be equal to mask's LSB position.
	 * How about to inform values in their right offset position and avoid
	 * this unneeded shift operation?
	 */
	set <<= ffs(mask) - 1;
	val |= set & mask;

	err = ov8830_write_reg(client, data_length, reg, val);
	if (err) {
		v4l2_err(client, "ov8830_rmw_reg error exit, write failed\n");
		return -EINVAL;
	}

	return 0;
}


/*
 * ov8830_write_reg_array - Initializes a list of MT9M114 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __ov8830_flush_reg_array, __ov8830_buf_reg_array() and
 * __ov8830_write_reg_is_consecutive() are internal functions to
 * ov8830_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __ov8830_flush_reg_array(struct i2c_client *client,
				     struct ov8830_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return ov8830_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __ov8830_buf_reg_array(struct i2c_client *client,
				   struct ov8830_write_ctrl *ctrl,
				   const struct ov8830_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case OV8830_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case OV8830_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg.sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= OV8830_MAX_WRITE_BUF_SIZE)
		__ov8830_flush_reg_array(client, ctrl);

	return 0;
}

static int
__ov8830_write_reg_is_consecutive(struct i2c_client *client,
				   struct ov8830_write_ctrl *ctrl,
				   const struct ov8830_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg.sreg;
}

static int ov8830_write_reg_array(struct i2c_client *client,
				   const struct ov8830_reg *reglist)
{
	const struct ov8830_reg *next = reglist;
	struct ov8830_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != OV8830_TOK_TERM; next++) {
		switch (next->type & OV8830_TOK_MASK) {
		case OV8830_TOK_DELAY:
			err = __ov8830_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;

		case OV8830_RMW:
			err = __ov8830_flush_reg_array(client, &ctrl);
			err |= ov8830_rmw_reg(client,
					       next->type & ~OV8830_RMW,
					       next->reg.sreg, next->val,
					       next->val2);
			if (err) {
				v4l2_err(client, "%s: rwm error, "
						"aborted\n", __func__);
				return err;
			}
			break;

		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__ov8830_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __ov8830_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __ov8830_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __ov8830_flush_reg_array(client, &ctrl);
}


static int ov8830_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int ret;

	value = min(value, OV8830_MAX_FOCUS_POS);

	ret = ov8830_write_reg(client, OV8830_16BIT, OV8830_VCM_CODE,
				OV8830_MAX_FOCUS_POS - value);
	if (ret == 0) {
		dev->focus = value;
		do_gettimeofday(&(dev->timestamp_t_focus_abs));
	}
	return ret;
}

static int ov8830_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 val;



	ret = ov8830_read_reg(client, OV8830_16BIT,
			       OV8830_VCM_CODE, &val);
	*value = OV8830_MAX_FOCUS_POS - val;

	return ret;
}

static int ov8830_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	return ov8830_t_focus_abs(sd, dev->focus + value);
}

#define WAIT_FOR_VCM_MOTOR	60000
static int ov8830_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	u32 status = 0;
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct timeval current_time;
	bool stillmoving = false;

	do_gettimeofday(&current_time);
	if (current_time.tv_sec == (dev->timestamp_t_focus_abs).tv_sec) {
		if (current_time.tv_usec < ((dev->timestamp_t_focus_abs).tv_usec+WAIT_FOR_VCM_MOTOR)) {
			stillmoving = true;
		}
	} else {
		if ((current_time.tv_usec+1000000) < ((dev->timestamp_t_focus_abs).tv_usec+WAIT_FOR_VCM_MOTOR)) {
			/* assuming the delay betwee calls does not take more than a second. */
			stillmoving = true;
		}
	}

	if (stillmoving) {
		status |= ATOMISP_FOCUS_STATUS_MOVING;
		status |= ATOMISP_FOCUS_HP_IN_PROGRESS;
	} else {
		status |= ATOMISP_FOCUS_HP_COMPLETE;
		status |= ATOMISP_FOCUS_STATUS_ACCEPTS_NEW_MOVE;
	}
	*value = status;
	return 0;
}

/*
static int ov8830_vcm_enable(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov8830_rmw_reg(client, OV8830_16BIT, OV8830_VCM_SLEW_STEP,
				OV8830_VCM_ENABLE, 0x1);
}
*/
static long ov8830_set_exposure(struct v4l2_subdev *sd, u16 coarse_itg,
				 u16 fine_itg, u16 gain)

{
	return -ENXIO;		/* Not supported yet */
}

static long ov8830_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	u16 coarse_itg, fine_itg, gain;

	coarse_itg = exposure->integration_time[0];
	fine_itg = exposure->integration_time[1];
	gain = exposure->gain[0];

	return ov8830_set_exposure(sd, coarse_itg, fine_itg, gain);
}

static long ov8830_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return ov8830_s_exposure(sd, (struct atomisp_exposure *)arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int ov8830_init_registers(struct v4l2_subdev *sd)
{
	return 0;
}

static int ov8830_init(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* set inital registers */
	ret = ov8830_init_registers(sd);

	/* restore settings */
	ov8830_res = ov8830_res_preview;
	N_RES = N_RES_PREVIEW;

	return ret;
}

static void ov8830_uninit(struct v4l2_subdev *sd)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);

	dev->coarse_itg = 0;
	dev->fine_itg   = 0;
	dev->gain       = 0;
	dev->focus      = OV8830_INVALID_CONFIG;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int ret;

	/* Enable power */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* Release reset */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* Enable clock */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* Minumum delay is 8192 clock cycles before first i2c transaction,
	 * which is 1.37 ms at the lowest allowed clock rate 6 MHz */
	msleep(2);
	return 0;

fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	return ret;
}

static int ov8830_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	int ret;

	if (on == 0) {
		ov8830_uninit(sd);
		ret = power_down(sd);
		dev->power = 0;
	} else {
		ret = power_up(sd);
		if (!ret) {
			dev->power = 1;
			/* init motor initial position */
			return ov8830_init(sd, 0);
		}
	}

	return ret;
}

static int ov8830_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!chip)
		return -EINVAL;

	v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV8830, 0);

	return 0;
}

static int ov8830_get_intg_factor(struct i2c_client *client,
								   struct camera_mipi_info *info,
								   const struct ov8830_reg *reglist)
{
	sensor_register	vt_pix_clk_div;
	sensor_register	vt_sys_clk_div;
	sensor_register	pre_pll_clk_div;
	sensor_register	pll_multiplier;
	sensor_register	op_pix_clk_div;
	sensor_register	op_sys_clk_div;

    /* TODO: this should not be a constant but should be set by a call to
     * MSIC's driver to get the ext_clk that MSIC supllies to the sensor.
     */
	const int ext_clk_freq_mhz = 19200000;
	struct sensor_mode_data buf;
	const struct ov8830_reg *next = reglist;
	int vt_pix_clk_freq_mhz;
	u16 data[OV8830_SHORT_MAX];

	sensor_register coarse_integration_time_min;
	sensor_register coarse_integration_time_max_margin;
	sensor_register fine_integration_time_min;
	sensor_register fine_integration_time_max_margin;
	sensor_register frame_length_lines;
	sensor_register line_length_pck;
	sensor_register read_mode;

	if (info == NULL)
		return -EINVAL;

	memset(data, 0, OV8830_SHORT_MAX * sizeof(u16));
	if (ov8830_read_reg(client, 12, OV8830_VT_PIX_CLK_DIV, data))
		return -EINVAL;
	vt_pix_clk_div = data[0];
	vt_sys_clk_div = data[1];
	pre_pll_clk_div = data[2];
	pll_multiplier = data[3];
	op_pix_clk_div = data[4];
	op_sys_clk_div = data[5];

	memset(data, 0, OV8830_SHORT_MAX * sizeof(u16));
	if (ov8830_read_reg(client, 4, OV8830_FRAME_LENGTH_LINES, data))
		return -EINVAL;
	frame_length_lines = data[0];
	line_length_pck = data[1];

	memset(data, 0, OV8830_SHORT_MAX * sizeof(u16));
	if (ov8830_read_reg(client, 8, OV8830_COARSE_INTG_TIME_MIN, data))
		return -EINVAL;
	coarse_integration_time_min = data[0];
	coarse_integration_time_max_margin = data[1];
	fine_integration_time_min = data[2];
	fine_integration_time_max_margin = data[3];

	memset(data, 0, OV8830_SHORT_MAX * sizeof(u16));
	if (ov8830_read_reg(client, 2, OV8830_READ_MODE, data))
		return -EINVAL;
	read_mode = data[0];

	vt_pix_clk_freq_mhz = divsave_rounded(ext_clk_freq_mhz*pll_multiplier,
								pre_pll_clk_div*vt_sys_clk_div*vt_pix_clk_div);

	memset(data, 0, OV8830_SHORT_MAX * sizeof(u16));
	if (ov8830_read_reg(client, 2, OV8830_FINE_INTEGRATION_TIME, data))
		return -EINVAL;
	v4l2_info(client, "fine_integration_time_i2c: %d", data[0]);

	for (; next->type != OV8830_TOK_TERM; next++) {
		if (next->type == OV8830_16BIT) {
			if (next->reg.sreg == OV8830_FINE_INTEGRATION_TIME) {
				buf.fine_integration_time_def = next->val;
				break;
			}
		}
	}

    /* something's wrong here, this mode does not have fine_igt set! */
	if (next->type == OV8830_TOK_TERM)
		return -EINVAL;

	buf.coarse_integration_time_min = coarse_integration_time_min;
	buf.coarse_integration_time_max_margin = coarse_integration_time_max_margin;
	buf.fine_integration_time_min = fine_integration_time_min;
	buf.fine_integration_time_max_margin = fine_integration_time_max_margin;
	buf.vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	buf.line_length_pck = line_length_pck;
	buf.frame_length_lines = frame_length_lines;
	buf.read_mode = read_mode;

	memcpy(&info->data, &buf, sizeof(buf));

	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int ov8830_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	int ret;

	/* the fine integration time is currently not calculated */
	ret = ov8830_read_reg(client, OV8830_16BIT,
			       OV8830_COARSE_INTEGRATION_TIME, &coarse);
	*value = coarse;

	return ret;
}

static int ov8830_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov8830_write_reg(client, OV8830_16BIT, 0x3070, value);
}

static int ov8830_v_flip(struct v4l2_subdev *sd, s32 value)
{
	return -ENXIO;
}


static int ov8830_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (value > OV8830_VCM_SLEW_STEP_MAX)
		return -EINVAL;

	return ov8830_rmw_reg(client, OV8830_16BIT, OV8830_VCM_SLEW_STEP,
				OV8830_VCM_SLEW_STEP_MASK, value);
}

static int ov8830_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Max 16 bits */
	if (value > OV8830_VCM_SLEW_TIME_MAX)
		return -EINVAL;

	return ov8830_write_reg(client, OV8830_16BIT, OV8830_VCM_SLEW_TIME,
				 value);
}

static int ov8830_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV8830_FOCAL_LENGTH_NUM << 16) | OV8830_FOCAL_LENGTH_DEM;
	return 0;
}

static int ov8830_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for ov8830*/
	*val = (OV8830_F_NUMBER_DEFAULT_NUM << 16) | OV8830_F_NUMBER_DEM;
	return 0;
}

static int ov8830_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV8830_F_NUMBER_DEFAULT_NUM << 24) |
		(OV8830_F_NUMBER_DEM << 16) |
		(OV8830_F_NUMBER_DEFAULT_NUM << 8) | OV8830_F_NUMBER_DEM;
	return 0;
}

struct ov8830_control ov8830_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = ov8830_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_TEST_PATTERN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Test pattern",
			.minimum = 0,
			.maximum = 0xffff,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov8830_test_pattern,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov8830_v_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move absolute",
			.minimum = 0,
			.maximum = OV8830_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ov8830_t_focus_abs,
		.query = ov8830_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.minimum = OV8830_MAX_FOCUS_NEG,
			.maximum = OV8830_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ov8830_t_focus_rel,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_STATUS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus status",
			.minimum = 0,
			.maximum = 100, /* allow enum to grow in the future */
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ov8830_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_SLEW,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm slew",
			.minimum = 0,
			.maximum = OV8830_VCM_SLEW_STEP_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ov8830_t_vcm_slew,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_TIMEING,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm step time",
			.minimum = 0,
			.maximum = OV8830_VCM_SLEW_TIME_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ov8830_t_vcm_timing,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = OV8830_FOCAL_LENGTH_DEFAULT,
			.maximum = OV8830_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = OV8830_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = ov8830_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = OV8830_F_NUMBER_DEFAULT,
			.maximum = OV8830_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = OV8830_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = ov8830_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = OV8830_F_NUMBER_RANGE,
			.maximum =  OV8830_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = OV8830_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = ov8830_g_fnumber_range,
	}
};
#define N_CONTROLS (ARRAY_SIZE(ov8830_controls))

static struct ov8830_control *ov8830_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov8830_controls[i].qc.id == id)
			return &ov8830_controls[i];
	return NULL;
}

static int ov8830_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ov8830_control *ctrl = ov8830_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;

	*qc = ctrl->qc;

	return 0;
}

/* ov8830 control set/get */
static int ov8830_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov8830_control *s_ctrl;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = ov8830_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	return s_ctrl->query(sd, &ctrl->value);
}

static int ov8830_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov8830_control *octrl = ov8830_find_control(ctrl->id);

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	return octrl->tweak(sd, ctrl->value);
}

struct ov8830_format ov8830_formats[] = {
	{
	 .desc = "RGB Bayer Format",
	 .regs = NULL,
	 },
};

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 140   /* tune this value so that the DVS resolutions get selected properly, but make sure 16:9 do not match 4:3*/
static int distance(struct ov8830_resolution *res, u32 w, u32 h)
{
	unsigned int w_ratio = ((res->width<<13)/w);
	unsigned int h_ratio = ((res->height<<13)/h);
	int match   = abs(((w_ratio<<13)/h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)  || (match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/* Return the nearest higher resolution index */
static int nearest_resolution_index(int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct ov8830_resolution *tmp_res = NULL;

	for (i = 0; i < N_RES; i++) {
		tmp_res = &ov8830_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}

	if (idx == -1)
		return -1;

	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != ov8830_res[i].width)
			continue;
		if (h != ov8830_res[i].height)
			continue;
		/* Found it */
		return i;
	}
	return -1;
}

static int ov8830_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	int idx;

	if (!fmt)
		return -EINVAL;

	if ((fmt->width > OV8830_RES_WIDTH_MAX) || (fmt->height > OV8830_RES_HEIGHT_MAX)) {
		fmt->width = OV8830_RES_WIDTH_MAX;
		fmt->height = OV8830_RES_HEIGHT_MAX;
	} else {
		idx = nearest_resolution_index(fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller resolutions.
		 * If it fails, it means the requested resolution is higher than we
		 * can support. Fallback to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = N_RES - 1;

		fmt->width = ov8830_res[idx].width;
		fmt->height = ov8830_res[idx].height;
	}

	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;


	return 0;
}

static int ov8830_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	const struct ov8830_reg *ov8830_def_reg;
	struct camera_mipi_info *ov8830_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ov8830_info = v4l2_get_subdev_hostdata(sd);
	if (ov8830_info == NULL)
		return -EINVAL;

	ret = ov8830_try_mbus_fmt(sd, fmt);
	if (ret) {
		v4l2_err(sd, "try fmt fail\n");
		return ret;
	}
	dev->fmt_idx = get_resolution_index(fmt->width, fmt->height);

	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		v4l2_err(sd, "get resolution fail\n");
		return -EINVAL;
	}

	ov8830_def_reg = ov8830_res[dev->fmt_idx].regs;
	ret = ov8830_write_reg_array(client, ov8830_def_reg);
	if (ret)
		return -EINVAL;

	dev->fps = ov8830_res[dev->fmt_idx].fps;
	dev->pixels_per_line = ov8830_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = ov8830_res[dev->fmt_idx].lines_per_frame;
#if 0
	ret = ov8830_get_intg_factor(client, ov8830_info, ov8830_def_reg);
	if (ret) {
		v4l2_err(sd, "failed to get integration_factor\n");
		return -EINVAL;
	}

	/* restore exposure, gain settings */
	if (dev->coarse_itg)
		ov8830_set_exposure(sd, dev->coarse_itg, dev->fine_itg,
				     dev->gain);
#endif
	return 0;
}

static int ov8830_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);

	if (!fmt)
		return -EINVAL;

	fmt->width = ov8830_res[dev->fmt_idx].width;
	fmt->height = ov8830_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int ov8830_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip ID	 */
	if (ov8830_read_reg(client, OV8830_8BIT, 0x3001,
			     &high)) {
		v4l2_err(client, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	if (ov8830_read_reg(client, OV8830_8BIT, 0x3002,
			     &low)) {
		v4l2_err(client, "sensor_id_low = 0x%x\n", high);
		return -ENODEV;
	}
	*id = (((u8) high) << 8) | (u8) low;
	v4l2_info(client, "sensor_id = 0x%x\n", *id);
	real_model_id = *id;

	if (*id != 0x0A08) {
		v4l2_err(client, "sensor ID error\n");
		return -ENODEV;
	}

	v4l2_info(client, "detect ov8830 success\n");

	/* REVISIT: HACK: Driver is currently forcing revision to 0 */
	*revision = 0;

	return 0;
}

/*
 * ov8830 stream on/off
 */
static int ov8830_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	int ret = ov8830_write_reg(client, OV8830_8BIT, 0x0100, enable ? 1 : 0);
	if (ret != 0) {
		v4l2_err(client, "failed to set streaming\n");
		return ret;
	}

	dev->streaming = enable;

	/* restore settings */
	ov8830_res = ov8830_res_preview;
	N_RES = N_RES_PREVIEW;

	return 0;
}

/*
 * ov8830 enum frame size, frame intervals
 */
static int ov8830_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ov8830_res[index].width;
	fsize->discrete.height = ov8830_res[index].height;
	fsize->reserved[0] = ov8830_res[index].used;

	return 0;
}

static int ov8830_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;

	if (index >= N_RES)
		return -EINVAL;

	/* since the isp will donwscale the resolution to the right size, find the nearest one that will allow the isp to do so
	 * important to ensure that the resolution requested is padded correctly by the requester, which is the atomisp driver in this case.
	 */
	index = nearest_resolution_index(fival->width, fival->height);

	if (-1 == index)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
/*	fival->width = ov8830_res[index].width;
	fival->height = ov8830_res[index].height; */
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ov8830_res[index].fps;

	return 0;
}

static int ov8830_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	*code = V4L2_MBUS_FMT_SGRBG10_1X10;
	return 0;
}

static int ov8830_s_config(struct v4l2_subdev *sd,
			    int irq, void *pdata)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id;
	int ret;

	if (pdata == NULL)
		return -ENODEV;

	dev->platform_data = pdata;

	ret = ov8830_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "ov8830 power-up err.\n");
		return ret;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = ov8830_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "ov8830_detect err s_config.\n");
		goto fail_detect;
	}

	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;

	/* power off sensor */
	ret = ov8830_s_power(sd, 0);
	if (ret) {
		v4l2_err(client, "ov8830 power-down err.\n");
		return ret;
	}

	return 0;

fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	ov8830_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int
ov8830_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;
	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int
ov8830_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = ov8830_res[index].width;
	fse->min_height = ov8830_res[index].height;
	fse->max_width = ov8830_res[index].width;
	fse->max_height = ov8830_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ov8830_get_pad_format(struct ov8830_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		v4l2_err(client, "%s err. pad %x\n", __func__, pad);
		return NULL;
	}

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
ov8830_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ov8830_get_pad_format(dev, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
ov8830_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ov8830_get_pad_format(dev, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	return 0;
}

static int
ov8830_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);

	dev->run_mode = param->parm.capture.capturemode;

	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		ov8830_res = ov8830_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		ov8830_res = ov8830_res_still;
		N_RES = N_RES_STILL;
		break;
	default:
		ov8830_res = ov8830_res_preview;
		N_RES = N_RES_PREVIEW;
	}
	return 0;
}

int
ov8830_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct ov8830_device *dev = to_ov8830_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 lines_per_frame;

	/*
	 * if no specific information to calculate the fps,
	 * just used the value in sensor settings
	 */
	if (!dev->pixels_per_line || !dev->lines_per_frame) {
		interval->interval.numerator = 1;
		interval->interval.denominator = dev->fps;
		return 0;
	}

	/*
	 * DS: if coarse_integration_time is set larger than
	 * lines_per_frame the frame_size will be expanded to
	 * coarse_integration_time+1
	 */
	if (dev->coarse_itg > dev->lines_per_frame) {
		if (dev->coarse_itg == 0xFFFF) {
			/*
			 * we can not add 1 according to ds, as this will
			 * cause over flow
			 */
			v4l2_warn(client, "%s: abnormal coarse_itg:0x%x\n",
				  __func__, dev->coarse_itg);
			lines_per_frame = dev->coarse_itg;
		} else
			lines_per_frame = dev->coarse_itg + 1;
	} else
		lines_per_frame = dev->lines_per_frame;

	interval->interval.numerator = dev->pixels_per_line *
					lines_per_frame;
	interval->interval.denominator = OV8830_MCLK * 1000000;

	return 0;
}

static const struct v4l2_subdev_video_ops ov8830_video_ops = {
	.s_stream = ov8830_s_stream,
	.enum_framesizes = ov8830_enum_framesizes,
	.enum_frameintervals = ov8830_enum_frameintervals,
	.enum_mbus_fmt = ov8830_enum_mbus_fmt,
	.try_mbus_fmt = ov8830_try_mbus_fmt,
	.g_mbus_fmt = ov8830_g_mbus_fmt,
	.s_mbus_fmt = ov8830_s_mbus_fmt,
	.s_parm = ov8830_s_parm,
	.g_frame_interval = ov8830_g_frame_interval,
};

static const struct v4l2_subdev_core_ops ov8830_core_ops = {
	.g_chip_ident = ov8830_g_chip_ident,
	.queryctrl = ov8830_queryctrl,
	.g_ctrl = ov8830_g_ctrl,
	.s_ctrl = ov8830_s_ctrl,
	.s_power = ov8830_s_power,
	.ioctl = ov8830_ioctl,
	.init = ov8830_init,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops ov8830_pad_ops = {
	.enum_mbus_code = ov8830_enum_mbus_code,
	.enum_frame_size = ov8830_enum_frame_size,
	.get_fmt = ov8830_get_pad_format,
	.set_fmt = ov8830_set_pad_format,
};

static const struct v4l2_subdev_ops ov8830_ops = {
	.core = &ov8830_core_ops,
	.video = &ov8830_video_ops,
	.pad = &ov8830_pad_ops,
};

static const struct media_entity_operations ov8830_entity_ops = {
	.link_setup = NULL,
};

static int ov8830_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov8830_device *dev = to_ov8830_sensor(sd);

	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	kfree(dev);

	return 0;
}

static int ov8830_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ov8830_device *dev;
	int ret;

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&(dev->sd), client, &ov8830_ops);

	if (client->dev.platform_data) {
		ret = ov8830_s_config(&dev->sd, client->irq,
				      client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			return ret;
		}
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.ops = &ov8830_entity_ops;
	dev->format.code = V4L2_MBUS_FMT_SGRBG10_1X10;

	/* REVISIT: Do we need media controller? */
	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		ov8830_remove(client);
		return ret;
	}

	return 0;
}

static const struct i2c_device_id ov8830_id[] = {
	{OV8830_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov8830_id);

static struct i2c_driver ov8830_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = OV8830_NAME,
	},
	.probe = ov8830_probe,
	.remove = ov8830_remove,
	.id_table = ov8830_id,
};

static __init int ov8830_init_mod(void)
{
	return i2c_add_driver(&ov8830_driver);
}

static __exit void ov8830_exit_mod(void)
{
	i2c_del_driver(&ov8830_driver);
}

module_init(ov8830_init_mod);
module_exit(ov8830_exit_mod);

MODULE_DESCRIPTION("A low-level driver for Omnivision OV8830 sensors");
MODULE_LICENSE("GPL");
