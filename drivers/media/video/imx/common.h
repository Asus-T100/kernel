#ifndef __COMMON_H__
#define __COMMON_H__

enum imx_tok_type {
	IMX_8BIT  = 0x0001,
	IMX_16BIT = 0x0002,
	IMX_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	IMX_TOK_DELAY  = 0xfe00	/* delay token for reg list */
};

/**
 * struct imx_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct imx_reg {
	enum imx_tok_type type;
	u16 sreg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

struct imx_resolution {
	u8 *desc;
	const struct imx_reg *regs;
	int res;
	int width;
	int height;
	int fps;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	unsigned short skip_frames;
	u8 bin_factor_x;
	u8 bin_factor_y;
	bool used;
};
#define GROUPED_PARAMETER_HOLD_ENABLE  {IMX_8BIT, 0x0104, 0x1}
#define GROUPED_PARAMETER_HOLD_DISABLE  {IMX_8BIT, 0x0104, 0x0}
#endif
