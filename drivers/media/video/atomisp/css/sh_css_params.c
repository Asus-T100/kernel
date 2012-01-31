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

#include <hmm/hmm.h>

#include "sh_css.h"
#include "sh_css_params.h"
#include "sh_css_internal.h"
#include "sh_css_hrt.h"
#include "sh_css_defs.h"
#include "sh_css_sp.h"

#define sISP_REG_BIT              ISP_VEC_ELEMBITS
#define uISP_REG_BIT              ((unsigned)(sISP_REG_BIT-1))
#define sSHIFT                    (16-sISP_REG_BIT)
#define uSHIFT                    ((unsigned)(16-uISP_REG_BIT))
#define sFRACTION_BITS_FITTING(a) (a-sSHIFT)
#define uFRACTION_BITS_FITTING(a) ((unsigned)(a-uSHIFT))
#define sISP_VAL_MIN              (-(1<<uISP_REG_BIT))
#define sISP_VAL_MAX              ((1<<uISP_REG_BIT)-1)
#define uISP_VAL_MIN              ((unsigned)0)
#define uISP_VAL_MAX              ((unsigned)((1<<uISP_REG_BIT)-1))
/* a:fraction bits for 16bit precision, b:fraction bits for ISP precision */
#define sDIGIT_FITTING(v, a, b) \
	min(max((((v)>>sSHIFT) >> (sFRACTION_BITS_FITTING(a)-(b))), \
	  sISP_VAL_MIN), sISP_VAL_MAX)
#define uDIGIT_FITTING(v, a, b) \
	min((unsigned)max((unsigned)(((v)>>uSHIFT) >> (uFRACTION_BITS_FITTING(a)-(b))), \
	  uISP_VAL_MIN), uISP_VAL_MAX)

#define FPNTBL_BYTES(binary) \
	(sizeof(char) * (binary)->in_frame_info.height * \
	 (binary)->in_frame_info.padded_width)
#define SCTBL_BYTES(binary) \
	(sizeof(unsigned short) * (binary)->sctbl_height * \
	 (binary)->sctbl_aligned_width_per_color * SH_CSS_SC_NUM_COLORS)
#define S3ATBL_BYTES(binary) \
	(sizeof(struct sh_css_3a_output) * (binary)->s3atbl_isp_width * \
	 (binary)->s3atbl_isp_height)
/* TODO: check if the stride is always the same max value or whether
 * it varies per resolution. */
#define S3ATBL_HI_LO_BYTES(binary) \
	(ISP_S3ATBL_HI_LO_STRIDE_BYTES * (binary)->s3atbl_isp_height)
/* SDIS */
#define SDIS_VER_COEF_TBL__IN_DMEM(b) \
	_SDIS_VER_COEF_TBL_USE_DMEM(b->info->mode, b->info->enable_dis)

#define SH_CSS_DIS_VER_NUM_COEF_TYPES(b) \
	(SDIS_VER_COEF_TBL__IN_DMEM(b) ? \
		SH_CSS_DIS_COEF_TYPES_ON_DMEM : \
		SH_CSS_DIS_NUM_COEF_TYPES)

#define SDIS_HOR_COEF_TBL_BYTES(b) \
	(sizeof(short) * SH_CSS_DIS_NUM_COEF_TYPES * (b)->dis_hor_coef_num_isp)
#define SDIS_VER_COEF_TBL_BYTES(b) \
	(sizeof(short) * SH_CSS_DIS_VER_NUM_COEF_TYPES(b) * \
		(b)->dis_ver_coef_num_isp)
#define SDIS_HOR_PROJ_TBL_BYTES(b) \
	(sizeof(int)   * SH_CSS_DIS_NUM_COEF_TYPES * (b)->dis_hor_proj_num_isp)
#define SDIS_VER_PROJ_TBL_BYTES(b) \
	(sizeof(int)   * SH_CSS_DIS_NUM_COEF_TYPES * (b)->dis_ver_proj_num_isp)
#define MORPH_PLANE_BYTES(binary) \
	(SH_CSS_MORPH_TABLE_ELEM_BYTES * (binary)->morph_tbl_aligned_width * \
	 (binary)->morph_tbl_height)

static struct sh_css_isp_params isp_parameters;
static struct sh_css_fpn_table fpn_table;
static const struct sh_css_morph_table   *morph_table;
static const struct sh_css_shading_table *sc_table;
static const struct sh_css_macc_table    *macc_table;
static const struct sh_css_gamma_table   *gamma_table;
static const struct sh_css_ctc_table     *ctc_table;
static const struct sh_css_3a_config     *s3a_config;
static const struct sh_css_wb_config     *wb_config;
static const struct sh_css_cc_config     *cc_config;
static const struct sh_css_tnr_config    *tnr_config;
static const struct sh_css_ob_config     *ob_config;
static const struct sh_css_dp_config     *dp_config;
static const struct sh_css_nr_config     *nr_config;
static const struct sh_css_ee_config     *ee_config;
static const struct sh_css_de_config     *de_config;
static const struct sh_css_gc_config     *gc_config;
static const struct sh_css_anr_config    *anr_config;
static const struct sh_css_ce_config     *ce_config;
static bool isp_params_changed,
	    fpn_table_changed,
	    dis_coef_table_changed,
	    morph_table_changed,
	    sc_table_changed,
	    macc_table_changed,
	    gamma_table_changed,
	    ctc_table_changed,
	    s3a_config_changed,
	    wb_config_changed,
	    cc_config_changed,
	    tnr_config_changed,
	    ob_config_changed,
	    dp_config_changed,
	    nr_config_changed,
	    ee_config_changed,
	    de_config_changed,
	    gc_config_changed,
	    anr_config_changed,
	    ce_config_changed;

static size_t fpn_tbl_size,
	      sc_tbl_size,
	      s3a_tbl_size,
	      s3a_tbl_hi_size,
	      s3a_tbl_lo_size,
	      sdis_hor_coef_size,
	      sdis_ver_coef_size,
	      sdis_hor_proj_size,
	      sdis_ver_proj_size,
	      tetra_r_x_size,
	      tetra_r_y_size,
	      tetra_gr_x_size,
	      tetra_gr_y_size,
	      tetra_gb_x_size,
	      tetra_gb_y_size,
	      tetra_b_x_size,
	      tetra_b_y_size,
	      tetra_batr_x_size,
	      tetra_batr_y_size,
	      tetra_ratb_x_size,
	      tetra_ratb_y_size;

/* Double buffering for 3A */
static void *s3a_tables[2],
	    *s3a_tables_hi[2],
	    *s3a_tables_lo[2],
	    *dis_hor_projections[2],
	    *dis_ver_projections[2];
static unsigned int curr_valid_buffer;

/* local buffers, used to re-order the 3a statistics in vmem-format */
static unsigned short s3a_tbl_hi_buf[ISP_S3ATBL_HI_LO_STRIDE *
				     SH_CSS_MAX_BQ_GRID_HEIGHT],
		      s3a_tbl_lo_buf[ISP_S3ATBL_HI_LO_STRIDE *
				     SH_CSS_MAX_BQ_GRID_HEIGHT];
static struct sh_css_macc_table converted_macc_table;
static const short *dis_hor_coef_tbl,
		   *dis_ver_coef_tbl;
static const struct sh_css_binary *current_3a_binary;
/* for csim and fpga, we need to keep a copy of all data in DDR.
   On the chip this is not necessary through.
   TODO: remove this for the chip */
static struct sh_css_ddr_address_map ddr_ptrs;
/* We keep a second copy of the ptr struct for the SP to access.
   Again, this would not be necessary on the chip. */
static void *sp_ddr_ptrs;

/* sp group address on DDR */
static void *xmem_sp_group_ptrs;

/* Default Parameters */
static const struct sh_css_gamma_table default_gamma_table = {
	.data = {
		0, 1, 2, 3, 4, 5, 6, 7,
		8, 9, 10, 11, 12, 13, 14, 16,
		17, 18, 19, 20, 21, 23, 24, 25,
		27, 28, 29, 31, 32, 33, 35, 36,
		38, 39, 41, 42, 44, 45, 47, 48,
		49, 51, 52, 54, 55, 57, 58, 60,
		61, 62, 64, 65, 66, 68, 69, 70,
		71, 72, 74, 75, 76, 77, 78, 79,
		80, 81, 82, 83, 84, 85, 86, 87,
		88, 89, 90, 91, 92, 93, 93, 94,
		95, 96, 97, 98, 98, 99, 100, 101,
		102, 102, 103, 104, 105, 105, 106, 107,
		108, 108, 109, 110, 110, 111, 112, 112,
		113, 114, 114, 115, 116, 116, 117, 118,
		118, 119, 120, 120, 121, 121, 122, 123,
		123, 124, 125, 125, 126, 126, 127, 127,	/* 128 */
		128, 129, 129, 130, 130, 131, 131, 132,
		132, 133, 134, 134, 135, 135, 136, 136,
		137, 137, 138, 138, 139, 139, 140, 140,
		141, 141, 142, 142, 143, 143, 144, 144,
		145, 145, 145, 146, 146, 147, 147, 148,
		148, 149, 149, 150, 150, 150, 151, 151,
		152, 152, 152, 153, 153, 154, 154, 155,
		155, 155, 156, 156, 156, 157, 157, 158,
		158, 158, 159, 159, 160, 160, 160, 161,
		161, 161, 162, 162, 162, 163, 163, 163,
		164, 164, 164, 165, 165, 165, 166, 166,
		166, 167, 167, 167, 168, 168, 168, 169,
		169, 169, 170, 170, 170, 170, 171, 171,
		171, 172, 172, 172, 172, 173, 173, 173,
		174, 174, 174, 174, 175, 175, 175, 176,
		176, 176, 176, 177, 177, 177, 177, 178,	/* 256 */
		178, 178, 178, 179, 179, 179, 179, 180,
		180, 180, 180, 181, 181, 181, 181, 182,
		182, 182, 182, 182, 183, 183, 183, 183,
		184, 184, 184, 184, 184, 185, 185, 185,
		185, 186, 186, 186, 186, 186, 187, 187,
		187, 187, 187, 188, 188, 188, 188, 188,
		189, 189, 189, 189, 189, 190, 190, 190,
		190, 190, 191, 191, 191, 191, 191, 192,
		192, 192, 192, 192, 192, 193, 193, 193,
		193, 193, 194, 194, 194, 194, 194, 194,
		195, 195, 195, 195, 195, 195, 196, 196,
		196, 196, 196, 196, 197, 197, 197, 197,
		197, 197, 198, 198, 198, 198, 198, 198,
		198, 199, 199, 199, 199, 199, 199, 200,
		200, 200, 200, 200, 200, 200, 201, 201,
		201, 201, 201, 201, 201, 202, 202, 202,	/* 384 */
		202, 202, 202, 202, 203, 203, 203, 203,
		203, 203, 203, 204, 204, 204, 204, 204,
		204, 204, 204, 205, 205, 205, 205, 205,
		205, 205, 205, 206, 206, 206, 206, 206,
		206, 206, 206, 207, 207, 207, 207, 207,
		207, 207, 207, 208, 208, 208, 208, 208,
		208, 208, 208, 209, 209, 209, 209, 209,
		209, 209, 209, 209, 210, 210, 210, 210,
		210, 210, 210, 210, 210, 211, 211, 211,
		211, 211, 211, 211, 211, 211, 212, 212,
		212, 212, 212, 212, 212, 212, 212, 213,
		213, 213, 213, 213, 213, 213, 213, 213,
		214, 214, 214, 214, 214, 214, 214, 214,
		214, 214, 215, 215, 215, 215, 215, 215,
		215, 215, 215, 216, 216, 216, 216, 216,
		216, 216, 216, 216, 216, 217, 217, 217,	/* 512 */
		217, 217, 217, 217, 217, 217, 217, 218,
		218, 218, 218, 218, 218, 218, 218, 218,
		218, 219, 219, 219, 219, 219, 219, 219,
		219, 219, 219, 220, 220, 220, 220, 220,
		220, 220, 220, 220, 220, 221, 221, 221,
		221, 221, 221, 221, 221, 221, 221, 221,
		222, 222, 222, 222, 222, 222, 222, 222,
		222, 222, 223, 223, 223, 223, 223, 223,
		223, 223, 223, 223, 223, 224, 224, 224,
		224, 224, 224, 224, 224, 224, 224, 224,
		225, 225, 225, 225, 225, 225, 225, 225,
		225, 225, 225, 226, 226, 226, 226, 226,
		226, 226, 226, 226, 226, 226, 226, 227,
		227, 227, 227, 227, 227, 227, 227, 227,
		227, 227, 228, 228, 228, 228, 228, 228,
		228, 228, 228, 228, 228, 228, 229, 229,
		229, 229, 229, 229, 229, 229, 229, 229,
		229, 229, 230, 230, 230, 230, 230, 230,
		230, 230, 230, 230, 230, 230, 231, 231,
		231, 231, 231, 231, 231, 231, 231, 231,
		231, 231, 231, 232, 232, 232, 232, 232,
		232, 232, 232, 232, 232, 232, 232, 233,
		233, 233, 233, 233, 233, 233, 233, 233,
		233, 233, 233, 233, 234, 234, 234, 234,
		234, 234, 234, 234, 234, 234, 234, 234,
		234, 235, 235, 235, 235, 235, 235, 235,
		235, 235, 235, 235, 235, 235, 236, 236,
		236, 236, 236, 236, 236, 236, 236, 236,
		236, 236, 236, 236, 237, 237, 237, 237,
		237, 237, 237, 237, 237, 237, 237, 237,
		237, 237, 238, 238, 238, 238, 238, 238,
		238, 238, 238, 238, 238, 238, 238, 238,
		239, 239, 239, 239, 239, 239, 239, 239,
		239, 239, 239, 239, 239, 239, 240, 240,
		240, 240, 240, 240, 240, 240, 240, 240,
		240, 240, 240, 240, 241, 241, 241, 241,
		241, 241, 241, 241, 241, 241, 241, 241,
		241, 241, 241, 242, 242, 242, 242, 242,
		242, 242, 242, 242, 242, 242, 242, 242,
		242, 242, 243, 243, 243, 243, 243, 243,
		243, 243, 243, 243, 243, 243, 243, 243,
		243, 244, 244, 244, 244, 244, 244, 244,
		244, 244, 244, 244, 244, 244, 244, 244,
		245, 245, 245, 245, 245, 245, 245, 245,
		245, 245, 245, 245, 245, 245, 245, 246,
		246, 246, 246, 246, 246, 246, 246, 246,
		246, 246, 246, 246, 246, 246, 246, 247,
		247, 247, 247, 247, 247, 247, 247, 247,
		247, 247, 247, 247, 247, 247, 247, 248,
		248, 248, 248, 248, 248, 248, 248, 248,
		248, 248, 248, 248, 248, 248, 248, 249,
		249, 249, 249, 249, 249, 249, 249, 249,
		249, 249, 249, 249, 249, 249, 249, 250,
		250, 250, 250, 250, 250, 250, 250, 250,
		250, 250, 250, 250, 250, 250, 250, 251,
		251, 251, 251, 251, 251, 251, 251, 251,
		251, 251, 251, 251, 251, 251, 251, 252,
		252, 252, 252, 252, 252, 252, 252, 252,
		252, 252, 252, 252, 252, 252, 252, 253,
		253, 253, 253, 253, 253, 253, 253, 253,
		253, 253, 253, 253, 253, 253, 253, 253,
		254, 254, 254, 254, 254, 254, 254, 254,
		254, 254, 254, 254, 254, 254, 254, 254,
		255, 255, 255, 255, 255, 255, 255, 255
	}
};

static const struct sh_css_ctc_table default_ctc_table = {
	.data = {
		0, 0, 256, 384, 384, 497, 765, 806,
		837, 851, 888, 901, 957, 981, 993, 1001,
		1011, 1029, 1028, 1039, 1062, 1059, 1073, 1080,
		1083, 1085, 1085, 1098, 1080, 1084, 1085, 1093,
		1078, 1073, 1070, 1069, 1077, 1066, 1072, 1063,
		1053, 1044, 1046, 1053, 1039, 1028, 1025, 1024,
		1012, 1013, 1016, 996, 992, 990, 990, 980,
		969, 968, 961, 955, 951, 949, 933, 930,
		929, 925, 921, 916, 906, 901, 895, 893,
		886, 877, 872, 869, 866, 861, 857, 849,
		845, 838, 836, 832, 823, 821, 815, 813,
		809, 805, 796, 793, 790, 785, 784, 778,
		772, 768, 766, 763, 758, 752, 749, 745,
		741, 740, 736, 730, 726, 724, 723, 718,
		711, 709, 706, 704, 701, 698, 691, 689,
		688, 683, 683, 678, 675, 673, 671, 669,
		666, 663, 661, 660, 656, 656, 653, 650,
		648, 647, 646, 643, 639, 638, 637, 635,
		633, 632, 629, 627, 626, 625, 622, 621,
		618, 618, 614, 614, 612, 609, 606, 606,
		603, 600, 600, 597, 594, 591, 590, 586,
		582, 581, 578, 575, 572, 569, 563, 560,
		557, 554, 551, 548, 545, 539, 536, 533,
		529, 527, 524, 519, 516, 513, 510, 507,
		504, 501, 498, 493, 491, 488, 485, 484,
		480, 476, 474, 471, 467, 466, 464, 460,
		459, 455, 453, 449, 447, 446, 443, 441,
		438, 435, 432, 432, 429, 427, 426, 422,
		419, 418, 416, 414, 412, 410, 408, 406,
		404, 402, 401, 398, 397, 395, 393, 390,
		389, 388, 387, 384, 382, 380, 378, 377,
		376, 375, 372, 370, 368, 368, 366, 364,
		363, 361, 360, 358, 357, 355, 354, 352,
		351, 350, 349, 346, 345, 344, 344, 342,
		340, 339, 337, 337, 336, 335, 333, 331,
		330, 329, 328, 326, 326, 324, 324, 322,
		321, 320, 318, 318, 318, 317, 315, 313,
		312, 311, 311, 310, 308, 307, 306, 306,
		304, 304, 302, 301, 300, 300, 299, 297,
		297, 296, 296, 294, 294, 292, 291, 291,
		291, 290, 288, 287, 286, 286, 287, 285,
		284, 283, 282, 282, 281, 281, 279, 278,
		278, 278, 276, 276, 275, 274, 274, 273,
		271, 270, 269, 268, 268, 267, 265, 262,
		261, 260, 260, 259, 257, 254, 252, 252,
		251, 251, 249, 246, 245, 244, 243, 242,
		240, 239, 239, 237, 235, 235, 233, 231,
		232, 230, 229, 226, 225, 224, 225, 224,
		223, 220, 219, 219, 218, 217, 217, 214,
		213, 213, 212, 211, 209, 209, 209, 208,
		206, 205, 204, 203, 204, 203, 201, 200,
		199, 197, 198, 198, 197, 195, 194, 194,
		193, 192, 192, 191, 189, 190, 189, 188,
		186, 187, 186, 185, 185, 184, 183, 181,
		183, 182, 181, 180, 179, 178, 178, 178,
		177, 176, 175, 176, 175, 174, 174, 173,
		172, 173, 172, 171, 170, 170, 169, 169,
		169, 168, 167, 166, 167, 167, 166, 165,
		164, 164, 164, 163, 164, 163, 162, 163,
		162, 161, 160, 161, 160, 160, 160, 159,
		158, 157, 158, 158, 157, 157, 156, 156,
		156, 156, 155, 155, 154, 154, 154, 154,
		154, 153, 152, 153, 152, 152, 151, 152,
		151, 152, 151, 150, 150, 149, 149, 150,
		149, 149, 148, 148, 148, 149, 148, 147,
		146, 146, 147, 146, 147, 146, 145, 146,
		146, 145, 144, 145, 144, 145, 144, 144,
		143, 143, 143, 144, 143, 142, 142, 142,
		142, 142, 142, 141, 141, 141, 141, 140,
		140, 141, 140, 140, 141, 140, 139, 139,
		139, 140, 139, 139, 138, 138, 137, 139,
		138, 138, 138, 137, 138, 137, 137, 137,
		137, 136, 137, 136, 136, 136, 136, 135,
		136, 135, 135, 135, 135, 136, 135, 135,
		134, 134, 133, 135, 134, 134, 134, 133,
		134, 133, 134, 133, 133, 132, 133, 133,
		132, 133, 132, 132, 132, 132, 131, 131,
		131, 132, 131, 131, 130, 131, 130, 132,
		131, 130, 130, 129, 130, 129, 130, 129,
		129, 129, 130, 129, 128, 128, 128, 128,
		129, 128, 128, 127, 127, 128, 128, 127,
		127, 126, 126, 127, 127, 126, 126, 126,
		127, 126, 126, 126, 125, 125, 126, 125,
		125, 124, 124, 124, 125, 125, 124, 124,
		123, 124, 124, 123, 123, 122, 122, 122,
		122, 122, 121, 120, 120, 119, 118, 118,
		118, 117, 117, 116, 115, 115, 115, 114,
		114, 113, 113, 112, 111, 111, 111, 110,
		110, 109, 109, 108, 108, 108, 107, 107,
		106, 106, 105, 105, 105, 104, 104, 103,
		103, 102, 102, 102, 102, 101, 101, 100,
		100, 99, 99, 99, 99, 99, 99, 98,
		97, 98, 97, 97, 97, 96, 96, 95,
		96, 95, 96, 95, 95, 94, 94, 95,
		94, 94, 94, 93, 93, 92, 93, 93,
		93, 93, 92, 92, 91, 92, 92, 92,
		91, 91, 90, 90, 91, 91, 91, 90,
		90, 90, 90, 91, 90, 90, 90, 89,
		89, 89, 90, 89, 89, 89, 89, 89,
		88, 89, 89, 88, 88, 88, 88, 87,
		89, 88, 88, 88, 88, 88, 87, 88,
		88, 88, 87, 87, 87, 87, 87, 88,
		87, 87, 87, 87, 87, 87, 88, 87,
		87, 87, 87, 86, 86, 87, 87, 87,
		87, 86, 86, 86, 87, 87, 86, 87,
		86, 86, 86, 87, 87, 86, 86, 86,
		86, 86, 87, 87, 86, 85, 85, 85,
		84, 85, 85, 84, 84, 83, 83, 82,
		82, 82, 81, 81, 80, 79, 79, 79,
		78, 77, 77, 76, 76, 76, 75, 74,
		74, 74, 73, 73, 72, 71, 71, 71,
		70, 70, 69, 69, 68, 68, 67, 67,
		67, 66, 66, 65, 65, 64, 64, 63,
		62, 62, 62, 61, 60, 60, 59, 59,
		58, 58, 57, 57, 56, 56, 56, 55,
		55, 54, 55, 55, 54, 53, 53, 52,
		53, 53, 52, 51, 51, 50, 51, 50,
		49, 49, 50, 49, 49, 48, 48, 47,
		47, 48, 46, 45, 45, 45, 46, 45,
		45, 44, 45, 45, 45, 43, 42, 42,
		41, 43, 41, 40, 40, 39, 40, 41,
		39, 39, 39, 39, 39, 38, 35, 35,
		34, 37, 36, 34, 33, 33, 33, 35,
		34, 32, 32, 31, 32, 30, 29, 26,
		25, 25, 27, 26, 23, 23, 23, 25,
		24, 24, 22, 21, 20, 19, 16, 14,
		13, 13, 13, 10, 9, 7, 7, 7,
		12, 12, 12, 7, 0, 0, 0, 0
	}
};

/* multiple axis color correction table,
 * 64values = 2x2matrix for 16area, [s2.11].
 */
static const struct sh_css_macc_table default_macc_table = {
	.data = {
		8192, 0, 0, 8192, 8192, 0, 0, 8192,
		8192, 0, 0, 8192, 8192, 0, 0, 8192,
		8192, 0, 0, 8192, 8192, 0, 0, 8192,
		8192, 0, 0, 8192, 8192, 0, 0, 8192,
		8192, 0, 0, 8192, 8192, 0, 0, 8192,
		8192, 0, 0, 8192, 8192, 0, 0, 8192,
		8192, 0, 0, 8192, 8192, 0, 0, 8192,
		8192, 0, 0, 8192, 8192, 0, 0, 8192
	}
};

/* Digital Zoom lookup table. See documentation for more details about the
 * contents of this table.
 */
static const int zoom_table[4][HRT_GDC_N] = {
	{0, 0, 0, 0, 0, 0, -1, -1,
	 -1, -2, -2, -3, -3, -4, -4, -5,
	 -6, -6, -7, -7, -8, -9, -9, -10,
	 -11, -11, -12, -13, -13, -14, -14, -15,
	 -16, -16, -16, -17, -17, -18, -18, -18,
	 -18, -18, -18, -18, -18, -18, -18, -18,
	 -18, -17, -17, -16, -15, -15, -14, -13,
	 -12, -11, -9, -8, -7, -5, -3, -1},
	{0, 2, 4, 7, 9, 12, 16, 19,
	 23, 27, 31, 35, 39, 43, 48, 53,
	 58, 62, 67, 73, 78, 83, 88, 94,
	 99, 105, 110, 116, 121, 127, 132, 138,
	 144, 149, 154, 160, 165, 170, 176, 181,
	 186, 191, 195, 200, 205, 209, 213, 218,
	 222, 225, 229, 232, 236, 239, 241, 244,
	 246, 248, 250, 252, 253, 254, 255, 255},
	{256, 255, 255, 254, 253, 252, 250, 248,
	 246, 244, 241, 239, 236, 232, 229, 225,
	 222, 218, 213, 209, 205, 200, 195, 191,
	 186, 181, 176, 170, 165, 160, 154, 149,
	 144, 138, 132, 127, 121, 116, 110, 105,
	 99, 94, 88, 83, 78, 73, 67, 62,
	 58, 53, 48, 43, 39, 35, 31, 27,
	 23, 19, 16, 12, 9, 7, 4, 2},
	{0, -1, -3, -5, -6, -8, -9, -10,
	 -12, -13, -14, -15, -16, -15, -17, -17,
	 -18, -18, -17, -19, -19, -18, -18, -19,
	 -18, -19, -18, -17, -17, -17, -16, -16,
	 -16, -15, -14, -14, -13, -12, -12, -12,
	 -11, -11, -9, -9, -9, -8, -6, -6,
	 -6, -5, -4, -3, -4, -3, -2, -2,
	 -1, 0, -1, 0, 1, 0, 0, 0}
};

static const struct sh_css_3a_config default_3a_config = {
	.ae_y_coef_r  = 25559,
	.ae_y_coef_g  = 32768,
	.ae_y_coef_b  = 7209,
	.af_fir1_coef = {-3344, -6104, -19143, 19143, 6104, 3344, 0},
	.af_fir2_coef = {1027, 0, -9219, 16384, -9219, 1027, 0}
};

static const struct sh_css_3a_config disabled_3a_config = {
	.ae_y_coef_r  = 25559,
	.ae_y_coef_g  = 32768,
	.ae_y_coef_b  = 7209,
	.af_fir1_coef = {-6689, -12207, -32768, 32767, 12207, 6689, 0},
	.af_fir2_coef = {2053, 0, -18437, 32767, -18437, 2053, 0}
};

static const struct sh_css_wb_config default_wb_config = {
	.integer_bits = 1,
	.gr           = 32768,
	.r            = 32768,
	.b            = 32768,
	.gb           = 32768
};

static const struct sh_css_wb_config disabled_wb_config = {
	.integer_bits = 1,
	.gr           = 32768,
	.r            = 32768,
	.b            = 32768,
	.gb           = 32768
};

static const struct sh_css_cc_config default_cc_config = {
	.fraction_bits = 8,
	.matrix        = {255, 29, 120, 0, -374, -342, 0, -672, 301},
};

static const struct sh_css_cc_config disabled_cc_config = {
	.fraction_bits = 8,
	.matrix        = {256, 44, 47, 0, -169, -171, 0, -214, 148},
};

static const struct sh_css_tnr_config default_tnr_config = {
	.gain         = 32768,
	.threshold_y  = 32,
	.threshold_uv = 32,
};

static const struct sh_css_tnr_config disabled_tnr_config = {
	.gain         = 0,
	.threshold_y  = 0,
	.threshold_uv = 0,
};

static const struct sh_css_ob_config default_ob_config = {
	.mode           = sh_css_ob_mode_none,
	.level_gr       = 0,
	.level_r        = 0,
	.level_b        = 0,
	.level_gb       = 0,
	.start_position = 0,
	.end_position   = 0
};

static const struct sh_css_ob_config disabled_ob_config = {
	.mode           = sh_css_ob_mode_none,
	.level_gr       = 0,
	.level_r        = 0,
	.level_b        = 0,
	.level_gb       = 0,
	.start_position = 0,
	.end_position   = 0
};

static const struct sh_css_dp_config default_dp_config = {
	.threshold = 8192,
	.gain      = 2048
};

static const struct sh_css_dp_config disabled_dp_config = {
	.threshold = 65535,
	.gain      = 65535
};

static const struct sh_css_nr_config default_nr_config = {
	.bnr_gain     = 16384,
	.ynr_gain     = 8192,
	.direction    = 1280,
	.threshold_cb = 0,
	.threshold_cr = 0
};

static const struct sh_css_nr_config disabled_nr_config = {
	.bnr_gain     = 0,
	.ynr_gain     = 0,
	.direction    = 0,
	.threshold_cb = 0,
	.threshold_cr = 0
};

static const struct sh_css_ee_config default_ee_config = {
	.gain        = 8192,
	.threshold   = 128,
	.detail_gain = 2048
};

static const struct sh_css_ee_config disabled_ee_config = {
	.gain        = 0,
	.threshold   = 0,
	.detail_gain = 0
};

static const struct sh_css_de_config default_de_config = {
	.pixelnoise          = 0,
	.c1_coring_threshold = 0,
	.c2_coring_threshold = 0
};

static const struct sh_css_de_config disabled_de_config = {
	.pixelnoise          = 65535,
	.c1_coring_threshold = 0,
	.c2_coring_threshold = 0
};

static const struct sh_css_gc_config default_gc_config = {
	.gain_k1 = 0,
	.gain_k2 = 0
};

static const struct sh_css_gc_config disabled_gc_config = {
	.gain_k1 = 0,
	.gain_k2 = 0
};

static const struct sh_css_anr_config default_anr_config = {
	.threshold   = 20,
};

static const struct sh_css_ce_config default_ce_config = {
	.uv_level_min = 0,
	.uv_level_max = 255
};

int
sh_css_get_gdc_coord_one(void)
{
	return HRT_GDC_COORD_ONE;
}

void
sh_css_set_dis_coefficients(const short *horizontal_coefficients,
			    const short *vertical_coefficients)
{
	dis_hor_coef_tbl = horizontal_coefficients;
	dis_ver_coef_tbl = vertical_coefficients;
	dis_coef_table_changed = true;
}

#if 1
/* This is the optimized code that uses the aligned_width and aligned_height
 * for the projections. This should be enabled in the same patch set that
 * adds the correct handling of these strides to the DIS IA code.
 */
void
sh_css_get_dis_projections(int *horizontal_projections,
			   int *vertical_projections)
{
	unsigned int hor_num_isp, ver_num_isp,
		     hor_bytes, ver_bytes;
	int *hor_ptr_isp = dis_hor_projections[curr_valid_buffer],
	    *ver_ptr_isp = dis_ver_projections[curr_valid_buffer];

	if (current_3a_binary == NULL)
		return;

	hor_num_isp = current_3a_binary->dis_hor_proj_num_isp;
	ver_num_isp = current_3a_binary->dis_ver_proj_num_isp;

	hor_bytes = hor_num_isp * sizeof(*horizontal_projections) *
		    SH_CSS_DIS_NUM_COEF_TYPES;
	ver_bytes = ver_num_isp * sizeof(*vertical_projections) *
		    SH_CSS_DIS_NUM_COEF_TYPES;

	hrt_isp_css_mm_load(hor_ptr_isp, horizontal_projections, hor_bytes);
	hrt_isp_css_mm_load(ver_ptr_isp, vertical_projections, ver_bytes);
}
#else
void
sh_css_get_dis_projections(int *horizontal_projections,
			   int *vertical_projections)
{
	unsigned int hor_num_isp, ver_num_isp,
		     hor_num_3a, ver_num_3a, i;
	int *hor_ptr_3a  = horizontal_projections,
	    *ver_ptr_3a  = vertical_projections,
	    *hor_ptr_isp = dis_hor_projections[curr_valid_buffer],
	    *ver_ptr_isp = dis_ver_projections[curr_valid_buffer];

	if (current_3a_binary == NULL)
		return;

	hor_num_isp = current_3a_binary->dis_hor_proj_num_isp;
	ver_num_isp = current_3a_binary->dis_ver_proj_num_isp;
	hor_num_3a  = current_3a_binary->dis_hor_proj_num_3a;
	ver_num_3a  = current_3a_binary->dis_ver_proj_num_3a;

	for (i = 0; i < SH_CSS_DIS_NUM_COEF_TYPES; i++) {
		hrt_isp_css_mm_load(hor_ptr_isp, hor_ptr_3a,
				    hor_num_3a * sizeof(int));
		hor_ptr_isp += hor_num_isp;
		hor_ptr_3a  += hor_num_3a;

		hrt_isp_css_mm_load(ver_ptr_isp, ver_ptr_3a,
				    ver_num_3a * sizeof(int));
		ver_ptr_isp += ver_num_isp;
		ver_ptr_3a  += ver_num_3a;
	}
}
#endif

static void
get_3a_stats_from_dmem(struct sh_css_3a_output *output)
{
	int ddr_width  = current_3a_binary->s3atbl_isp_width,
		  out_width  = current_3a_binary->s3atbl_width,
			out_height = current_3a_binary->s3atbl_height,
			i;
	struct sh_css_3a_output
			*ddr_ptr = s3a_tables[curr_valid_buffer],
			*out_ptr = output;

	for (i = 0; i < out_height; i++) {
		hrt_isp_css_mm_load(ddr_ptr, out_ptr,
			out_width * sizeof(*output));
		ddr_ptr += ddr_width;
		out_ptr += out_width;
	}
}

static inline int
merge_hi14bit_lo14bit(unsigned short hi, unsigned short lo)
{
	int val = (int) ((((unsigned int) hi << 14) & 0xfffc000) |
			 ((unsigned int) lo & 0x3fff));
	return val;
}

static void
get_3a_stats_from_vmem(struct sh_css_3a_output *output)
{
	int out_width  = current_3a_binary->s3atbl_width,
	    out_height = current_3a_binary->s3atbl_height;
	unsigned short *hi, *lo;
	int chunk, rest, kmax, y, x, k, elm_start, elm, ofs, bytes;

	hi = s3a_tbl_hi_buf;
	lo = s3a_tbl_lo_buf;

	chunk = (ISP_VEC_NELEMS >> current_3a_binary->deci_factor_log2);
	chunk = max(chunk, 1);
	bytes = ISP_S3ATBL_HI_LO_STRIDE_BYTES * out_height;

	hrt_isp_css_mm_load(s3a_tables_hi[curr_valid_buffer], hi, bytes);
	hrt_isp_css_mm_load(s3a_tables_lo[curr_valid_buffer], lo, bytes);

	for (y = 0; y < out_height; y++) {
		elm_start = y * ISP_S3ATBL_HI_LO_STRIDE;
		rest = out_width;
		x = 0;
		while (x < out_width) {
			kmax = (rest > chunk) ? chunk : rest;
			ofs = y * out_width + x;
			elm = elm_start + x * sizeof(*output) / sizeof(int);
			for (k = 0; k < kmax; k++, elm++) {
				output[ofs + k].ae_y =
				    merge_hi14bit_lo14bit
				    (hi[elm], lo[elm]);
				output[ofs + k].awb_cnt =
				    merge_hi14bit_lo14bit
				    (hi[elm + chunk], lo[elm + chunk]);
				output[ofs + k].awb_gr =
				    merge_hi14bit_lo14bit
				    (hi[elm + chunk * 2],
				     lo[elm + chunk * 2]);
				output[ofs + k].awb_r =
				    merge_hi14bit_lo14bit
				    (hi[elm + chunk * 3],
				     lo[elm + chunk * 3]);
				output[ofs + k].awb_b =
				    merge_hi14bit_lo14bit
				    (hi[elm + chunk * 4],
				     lo[elm + chunk * 4]);
				output[ofs + k].awb_gb =
				    merge_hi14bit_lo14bit
				    (hi[elm + chunk * 5],
				     lo[elm + chunk * 5]);
				output[ofs + k].af_hpf1 =
				    merge_hi14bit_lo14bit
				    (hi[elm + chunk * 6],
				     lo[elm + chunk * 6]);
				output[ofs + k].af_hpf2 =
				    merge_hi14bit_lo14bit
				    (hi[elm + chunk * 7],
				     lo[elm + chunk * 7]);
			}
			x += chunk;
			rest -= chunk;
		}
	}
}

static void
sh_css_process_3a(void)
{
	unsigned int i;
	/* coefficients to calculate Y */
	isp_parameters.ae_y_coef_r =
	    uDIGIT_FITTING(s3a_config->ae_y_coef_r, 16, SH_CSS_AE_YCOEF_SHIFT);
	isp_parameters.ae_y_coef_g =
	    uDIGIT_FITTING(s3a_config->ae_y_coef_g, 16, SH_CSS_AE_YCOEF_SHIFT);
	isp_parameters.ae_y_coef_b =
	    uDIGIT_FITTING(s3a_config->ae_y_coef_b, 16, SH_CSS_AE_YCOEF_SHIFT);

	/* af fir coefficients */
	for (i = 0; i < 7; ++i) {
		isp_parameters.af_fir1[i] =
		  sDIGIT_FITTING(s3a_config->af_fir1_coef[i], 15,
				 SH_CSS_AF_FIR_SHIFT);
		isp_parameters.af_fir2[i] =
		  sDIGIT_FITTING(s3a_config->af_fir2_coef[i], 15,
				 SH_CSS_AF_FIR_SHIFT);
	}
	isp_params_changed = true;
	s3a_config_changed = false;
}

void *
sh_css_params_ddr_address_map(void)
{
	return sp_ddr_ptrs;
}

/* ****************************************************
 * Each coefficient is stored as 7bits to fit 2 of them into one
 * ISP vector element, so we will store 4 coefficents on every
 * memory word (32bits)
 *
 * 0: Coefficient 0 used bits
 * 1: Coefficient 1 used bits
 * 2: Coefficient 2 used bits
 * 3: Coefficient 3 used bit3
 * x: not used
 *
 * xx33333332222222 | xx11111110000000
 *
 * ***************************************************
 */
static void
store_fpntbl(void *ptr)
{
	unsigned int i, j;
	short *data_ptr = fpn_table.data;

	for (i = 0; i < fpn_table.height; i++) {
		for (j = 0;
		     j < fpn_table.width;
		     j += 4, ptr += 4, data_ptr += 4) {
			int data = data_ptr[0] << 0 |
				   data_ptr[1] << 7 |
				   data_ptr[2] << 16 |
				   data_ptr[3] << 23;
			hrt_isp_css_mm_store_int(ptr, data);
		}
	}
}

static void
convert_raw_to_fpn(void)
{
	short maxval = 0;
	unsigned int i;

	/* Find the maximum value in the table */
	for (i = 0; i < fpn_table.height * fpn_table.width; i++) {
		short val = fpn_table.data[i];
		/* Make sure FPN value can be represented in 13-bit unsigned
		 * number (ISP precision - 1), but note that actual input range
		 * depends on precision of input frame data.
		 */
		if (val < 0)
			val = 0;
		if (val >= (1 << 13))
			val = (1 << 13) - 1;
		maxval = max(maxval, val);
	}
	/* Find the lowest shift value to remap the values in the range
	 * 0..maxval to 0..2^shiftval*63.
	 */
	fpn_table.shift = 0;
	while (maxval > 63) {
		maxval /= 2;
		fpn_table.shift++;
	}
	/* Adjust the values in the table for the shift value */
	for (i = 0; i < fpn_table.height * fpn_table.width; i++)
		((unsigned short *) fpn_table.data)[i] >>= fpn_table.shift;
}

enum sh_css_err
sh_css_set_black_frame(const struct sh_css_frame *raw_black_frame)
{
	/* this function desperately needs to be moved to the ISP or SP such
	 * that it can use the DMA.
	 */
	unsigned int height = raw_black_frame->info.height,
		     width = raw_black_frame->info.padded_width,
		     y, x, k, data;
	void *ptr = raw_black_frame->planes.raw.data;

	if (fpn_table.data &&
	    (fpn_table.width != width || fpn_table.height != height)) {
		sh_css_free(fpn_table.data);
		fpn_table.data = NULL;
	}
	if (fpn_table.data == NULL) {
		fpn_table.data = sh_css_malloc(height * width * sizeof(short));
		if (!fpn_table.data)
			return sh_css_err_cannot_allocate_memory;
		fpn_table.width = width;
		fpn_table.height = height;
		fpn_table.shift = 0;
	}

	/* store raw to fpntbl */
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x += (ISP_VEC_NELEMS * 2)) {
			int ofs = y * width + x;
			for (k = 0; k < ISP_VEC_NELEMS; k += 2) {
				hrt_isp_css_mm_load(ptr, &data, sizeof(int));
				fpn_table.data[ofs + 2 * k] =
				    (short) (data & 0xFFFF);
				fpn_table.data[ofs + 2 * k + 2] =
				    (short) ((data >> 16) & 0xFFFF);
				ptr += 4;	/* byte system address */
			}
			for (k = 0; k < ISP_VEC_NELEMS; k += 2) {
				hrt_isp_css_mm_load(ptr, &data, sizeof(int));
				fpn_table.data[ofs + 2 * k + 1] =
				    (short) (data & 0xFFFF);
				fpn_table.data[ofs + 2 * k + 3] =
				    (short) ((data >> 16) & 0xFFFF);
				ptr += 4;	/* byte system address */
			}
		}
	}

	/* raw -> fpn */
	convert_raw_to_fpn();

	/* overwrite isp parameter */
	isp_parameters.fpn_shift = fpn_table.shift;
	isp_parameters.fpn_enabled = 1;
	fpn_table_changed = true;
	isp_params_changed = true;
	return sh_css_success;
}

struct sh_css_shading_table *
sh_css_shading_table_alloc(unsigned int width,
			   unsigned int height)
{
	unsigned int i;
	struct sh_css_shading_table *me = sh_css_malloc(sizeof(*me));

	if (me == NULL)
		return NULL;
	me->width         = width;
	me->height        = height;
	me->sensor_width  = 0;
	me->sensor_height = 0;
	me->fraction_bits = 0;
	for (i = 0; i < SH_CSS_SC_NUM_COLORS; i++) {
		me->data[i] =
		    sh_css_malloc(width * height * sizeof(*me->data[0]));
		if (me->data[i] == NULL) {
			unsigned int j;
			for (j = 0; j < i; j++)
				sh_css_free(me->data[j]);
			sh_css_free(me);
			return NULL;
		}
	}
	return me;
}

void
sh_css_shading_table_free(struct sh_css_shading_table *table)
{
	unsigned int i;

	if (table == NULL)
		return;
	for (i = 0; i < SH_CSS_SC_NUM_COLORS; i++) {
		if (table->data[i])
			sh_css_free(table->data[i]);
	}
	sh_css_free(table);
	sc_table = NULL;
}

void
sh_css_params_set_shading_table(const struct sh_css_shading_table *table)
{
	if (table != sc_table) {
		sc_table = table;
		sc_table_changed = true;
	}
}

static void
store_sctbl(const struct sh_css_binary *binary,
	    void *ddr_addr)
{
	unsigned int i, j, aligned_width, row_padding;

	if (!sc_table)
		return;

	aligned_width = binary->sctbl_aligned_width_per_color;
	isp_parameters.sc_gain_shift = sc_table->fraction_bits;
	row_padding = aligned_width - sc_table->width;

	for (i = 0; i < sc_table->height; i++) {
		for (j = 0; j < SH_CSS_SC_NUM_COLORS; j++) {
			hrt_isp_css_mm_store(ddr_addr,
					&sc_table->data[j][i*sc_table->width],
					sc_table->width * sizeof(short));
			ddr_addr += sc_table->width * sizeof(short);
			hrt_isp_css_mm_set(ddr_addr, 0,
						row_padding * sizeof(short));
			ddr_addr += row_padding * sizeof(short);
		}
	}
	isp_params_changed = true;
}

static void
sh_css_process_wb(void)
{
	isp_parameters.wb_gain_shift =
	    uISP_REG_BIT - wb_config->integer_bits;
	isp_parameters.wb_gain_gr =
	    uDIGIT_FITTING(wb_config->gr, 16 - wb_config->integer_bits,
			   isp_parameters.wb_gain_shift);
	isp_parameters.wb_gain_r =
	    uDIGIT_FITTING(wb_config->r, 16 - wb_config->integer_bits,
			   isp_parameters.wb_gain_shift);
	isp_parameters.wb_gain_b =
	    uDIGIT_FITTING(wb_config->b, 16 - wb_config->integer_bits,
			   isp_parameters.wb_gain_shift);
	isp_parameters.wb_gain_gb =
	    uDIGIT_FITTING(wb_config->gb, 16 - wb_config->integer_bits,
			   isp_parameters.wb_gain_shift);
	isp_params_changed = true;
	wb_config_changed = false;
}

static void
sh_css_process_cc(void)
{
	isp_parameters.csc_coef_shift    = (int) cc_config->fraction_bits;
	isp_parameters.yc1c2_to_ycbcr_00 = (int) cc_config->matrix[0];
	isp_parameters.yc1c2_to_ycbcr_01 = (int) cc_config->matrix[1];
	isp_parameters.yc1c2_to_ycbcr_02 = (int) cc_config->matrix[2];
	isp_parameters.yc1c2_to_ycbcr_10 = (int) cc_config->matrix[3];
	isp_parameters.yc1c2_to_ycbcr_11 = (int) cc_config->matrix[4];
	isp_parameters.yc1c2_to_ycbcr_12 = (int) cc_config->matrix[5];
	isp_parameters.yc1c2_to_ycbcr_20 = (int) cc_config->matrix[6];
	isp_parameters.yc1c2_to_ycbcr_21 = (int) cc_config->matrix[7];
	isp_parameters.yc1c2_to_ycbcr_22 = (int) cc_config->matrix[8];
	isp_params_changed = true;
	cc_config_changed = false;
}

static void
sh_css_process_tnr(void)
{
	isp_parameters.tnr_coef =
	    uDIGIT_FITTING(tnr_config->gain, 16, SH_CSS_TNR_COEF_SHIFT);
	isp_parameters.tnr_threshold_Y =
	    uDIGIT_FITTING(tnr_config->threshold_y, 16, SH_CSS_ISP_YUV_BITS);
	isp_parameters.tnr_threshold_C =
	    uDIGIT_FITTING(tnr_config->threshold_uv, 16, SH_CSS_ISP_YUV_BITS);
	isp_params_changed = true;
	tnr_config_changed = false;
}

static void
sh_css_process_ob(void)
{
	unsigned int raw_bit_depth = 16;
	switch (ob_config->mode) {
	case sh_css_ob_mode_fixed:
		if (current_3a_binary)
			raw_bit_depth
			  = current_3a_binary->in_frame_info.raw_bit_depth;
		isp_parameters.ob_blacklevel_gr
			= ob_config->level_gr >> (16 - raw_bit_depth);
		isp_parameters.ob_blacklevel_r
			= ob_config->level_r  >> (16 - raw_bit_depth);
		isp_parameters.ob_blacklevel_b
			= ob_config->level_b  >> (16 - raw_bit_depth);
		isp_parameters.ob_blacklevel_gb
			= ob_config->level_gb >> (16 - raw_bit_depth);
		isp_parameters.obarea_start_bq = 0;
		isp_parameters.obarea_length_bq = 0;
		isp_parameters.obarea_length_bq_inverse = 0;
		break;
	case sh_css_ob_mode_raster:
		isp_parameters.ob_blacklevel_gr = 0;
		isp_parameters.ob_blacklevel_r = 0;
		isp_parameters.ob_blacklevel_b = 0;
		isp_parameters.ob_blacklevel_gb = 0;
		isp_parameters.obarea_start_bq =
		    ob_config->start_position;
		isp_parameters.obarea_length_bq =
		    ((ob_config->end_position - ob_config->start_position) + 1);
		isp_parameters.obarea_length_bq_inverse =
		    (1 << 12) / isp_parameters.obarea_length_bq;
		break;
	default:
		isp_parameters.ob_blacklevel_gr = 0;
		isp_parameters.ob_blacklevel_r = 0;
		isp_parameters.ob_blacklevel_b = 0;
		isp_parameters.ob_blacklevel_gb = 0;
		isp_parameters.obarea_start_bq = 0;
		isp_parameters.obarea_length_bq = 0;
		isp_parameters.obarea_length_bq_inverse = 0;
		break;
	}
	isp_params_changed = true;
	ob_config_changed = false;
}

static void
sh_css_process_dp(void)
{
	isp_parameters.dp_threshold_single_when_2adjacent_on =
	    SH_CSS_BAYER_MAXVAL;
	isp_parameters.dp_threshold_2adjacent_when_2adjacent_on =
	    uDIGIT_FITTING(dp_config->threshold, 16, SH_CSS_BAYER_BITS);
	isp_parameters.dp_threshold_single_when_2adjacent_off =
	    uDIGIT_FITTING(dp_config->threshold, 16, SH_CSS_BAYER_BITS);
	isp_parameters.dp_threshold_2adjacent_when_2adjacent_off =
	    SH_CSS_BAYER_MAXVAL;
	isp_parameters.dp_gain =
	    uDIGIT_FITTING(dp_config->gain, 8, SH_CSS_DP_GAIN_SHIFT);
	isp_params_changed = true;
	dp_config_changed = false;
}

static void
sh_css_process_nr_ee(void)
{
	int asiWk1, asiWk2, asiWk3;

	/* BNR (Bayer Noise Reduction) */
	isp_parameters.bnr_threshold_low =
	    uDIGIT_FITTING(nr_config->direction, 16, SH_CSS_BAYER_BITS);
	isp_parameters.bnr_threshold_width_log2 = uFRACTION_BITS_FITTING(8);
	isp_parameters.bnr_threshold_width =
	    1 << isp_parameters.bnr_threshold_width_log2;
	isp_parameters.bnr_gain_all =
	    uDIGIT_FITTING(nr_config->bnr_gain, 16, SH_CSS_BNR_GAIN_SHIFT);
	isp_parameters.bnr_gain_dir =
	    uDIGIT_FITTING(nr_config->bnr_gain, 16, SH_CSS_BNR_GAIN_SHIFT);
	isp_parameters.bnr_clip = uDIGIT_FITTING(
					(unsigned)16384, 16, SH_CSS_BAYER_BITS);

	/* YNR (Y Noise Reduction), YEE (Y Edge Enhancement) */
	asiWk1 = (int) ee_config->gain;
	asiWk2 = asiWk1 / 8;
	asiWk3 = asiWk1 / 4;
	isp_parameters.ynr_threshold =
		uDIGIT_FITTING((unsigned)8192, 16, SH_CSS_BAYER_BITS);
	isp_parameters.ynr_gain_all =
	    uDIGIT_FITTING(nr_config->ynr_gain, 16, SH_CSS_YNR_GAIN_SHIFT);
	isp_parameters.ynr_gain_dir =
	    uDIGIT_FITTING(nr_config->ynr_gain, 16, SH_CSS_YNR_GAIN_SHIFT);
	isp_parameters.ynryee_dirthreshold_s =
	    min((uDIGIT_FITTING(nr_config->direction, 16, SH_CSS_BAYER_BITS)
				    << 1),
		SH_CSS_BAYER_MAXVAL);
	isp_parameters.ynryee_dirthreshold_g =
	    min((uDIGIT_FITTING(nr_config->direction, 16, SH_CSS_BAYER_BITS)
				    << 4),
		SH_CSS_BAYER_MAXVAL);
	isp_parameters.ynryee_dirthreshold_width_log2 =
	    uFRACTION_BITS_FITTING(8);
	isp_parameters.ynryee_dirthreshold_width =
	    1 << isp_parameters.ynryee_dirthreshold_width_log2;
	isp_parameters.yee_detailgain =
	    uDIGIT_FITTING(ee_config->detail_gain, 11,
			   SH_CSS_YEE_DETAIL_GAIN_SHIFT);
	isp_parameters.yee_coring_s =
	    (uDIGIT_FITTING((unsigned)56, 16, SH_CSS_BAYER_BITS) *
	     ee_config->threshold) >> 8;
	isp_parameters.yee_coring_g =
	    (uDIGIT_FITTING((unsigned)224, 16, SH_CSS_BAYER_BITS) *
	     ee_config->threshold) >> 8;
	/* 8; // *1.125 ->[s4.8] */
	isp_parameters.yee_scale_plus_s =
	    (asiWk1 + asiWk2) >> (11 - SH_CSS_YEE_SCALE_SHIFT);
	/* 8; // ( * -.25)->[s4.8] */
	isp_parameters.yee_scale_plus_g =
	    (0 - asiWk3) >> (11 - SH_CSS_YEE_SCALE_SHIFT);
	/* 8; // *0.875 ->[s4.8] */
	isp_parameters.yee_scale_minus_s =
	    (asiWk1 - asiWk2) >> (11 - SH_CSS_YEE_SCALE_SHIFT);
	/* 8; // ( *.25 ) ->[s4.8] */
	isp_parameters.yee_scale_minus_g =
	    (asiWk3) >> (11 - SH_CSS_YEE_SCALE_SHIFT);
	isp_parameters.yee_clip_plus_s =
	    uDIGIT_FITTING((unsigned)32760, 16, SH_CSS_BAYER_BITS);
	isp_parameters.yee_clip_plus_g = 0;
	isp_parameters.yee_clip_minus_s =
	    uDIGIT_FITTING((unsigned)504, 16, SH_CSS_BAYER_BITS);
	isp_parameters.yee_clip_minus_g =
	    uDIGIT_FITTING((unsigned)32256, 16, SH_CSS_BAYER_BITS);
	isp_parameters.ynryee_Yclip = SH_CSS_BAYER_MAXVAL;
	isp_params_changed = true;
	nr_config_changed = false;
	ee_config_changed = false;
}

static void
sh_css_process_de(void)
{
	isp_parameters.de_pixelnoise =
	    uDIGIT_FITTING(de_config->pixelnoise, 16, SH_CSS_BAYER_BITS);
	isp_parameters.de_c1_coring_threshold =
	    uDIGIT_FITTING(de_config->c1_coring_threshold, 16,
			   SH_CSS_BAYER_BITS);
	isp_parameters.de_c2_coring_threshold =
	    uDIGIT_FITTING(de_config->c2_coring_threshold, 16,
			   SH_CSS_BAYER_BITS);
	isp_params_changed = true;
	de_config_changed = false;
}

static void
sh_css_process_gc(void)
{
	isp_parameters.gamma_gain_k1 =
	    uDIGIT_FITTING(gc_config->gain_k1, 16, SH_CSS_GAMMA_GAIN_K_SHIFT);
	isp_parameters.gamma_gain_k2 =
	    uDIGIT_FITTING(gc_config->gain_k2, 16, SH_CSS_GAMMA_GAIN_K_SHIFT);
	isp_params_changed = true;
	gc_config_changed = false;
}

static void
sh_css_process_anr(void)
{
	isp_parameters.anr_threshold = anr_config->threshold;
	isp_params_changed = true;
	anr_config_changed = false;
}

static void
sh_css_process_ce(void)
{
	isp_parameters.ce_uv_level_min = ce_config->uv_level_min;
	isp_parameters.ce_uv_level_max = ce_config->uv_level_max;
	isp_params_changed = true;
	ce_config_changed = false;
}

void
sh_css_set_gamma_table(const struct sh_css_gamma_table *table)
{
	gamma_table = table;
	gamma_table_changed = true;
}

void
sh_css_get_gamma_table(const struct sh_css_gamma_table **table)
{
	*table = gamma_table;
}

void
sh_css_set_ctc_table(const struct sh_css_ctc_table *table)
{
	ctc_table = table;
	ctc_table_changed = true;
}

void
sh_css_get_ctc_table(const struct sh_css_ctc_table **table)
{
	*table = ctc_table;
}

void
sh_css_set_macc_table(const struct sh_css_macc_table *table)
{
	macc_table = table;
	macc_table_changed = true;
}

void
sh_css_get_macc_table(const struct sh_css_macc_table **table)
{
	*table = macc_table;
}

void
sh_css_morph_table_free(struct sh_css_morph_table *me)
{
	unsigned int i;

	if (me == NULL)
		return;

	for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
		if (me->coordinates_x[i])
			sh_css_free(me->coordinates_x[i]);
		if (me->coordinates_y[i])
			sh_css_free(me->coordinates_y[i]);
	}
	sh_css_free(me);
}

struct sh_css_morph_table *
sh_css_morph_table_allocate(unsigned int width, unsigned int height)
{
	unsigned int i;
	struct sh_css_morph_table *me = sh_css_malloc(sizeof(*me));

	if (!me)
		return NULL;

	for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
		me->coordinates_x[i] = NULL;
		me->coordinates_y[i] = NULL;
	}

	for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
		me->coordinates_x[i] =
		    sh_css_malloc(height * width *
				  sizeof(*me->coordinates_x[i]));
		me->coordinates_y[i] =
		    sh_css_malloc(height * width *
				  sizeof(*me->coordinates_y[i]));
		if (!me->coordinates_x[i] || !me->coordinates_y[i]) {
			sh_css_morph_table_free(me);
			return NULL;
		}
	}
	me->width = width;
	me->height = height;
	return me;
}

static enum sh_css_err
sh_css_params_default_morph_table(struct sh_css_morph_table **table,
				  const struct sh_css_binary *binary)
{
	unsigned int i, j, k,
		     step = (ISP_VEC_NELEMS / 16) * 128,
		     width = binary->morph_tbl_width,
		     height = binary->morph_tbl_height;
	short start_x[SH_CSS_MORPH_TABLE_NUM_PLANES] = { -8, 0, -8, 0, 0, -8 },
	      start_y[SH_CSS_MORPH_TABLE_NUM_PLANES] = { 0, 0, -8, -8, -8, 0 };
	struct sh_css_morph_table *tab;

	tab = sh_css_morph_table_allocate(width, height);
	if (!tab)
		return sh_css_err_cannot_allocate_memory;

	for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
		short val_y = start_y[i];
		for (j = 0; j < height; j++) {
			short val_x = start_x[i];
			unsigned short *x_ptr, *y_ptr;

			x_ptr = &tab->coordinates_x[i][j * width];
			y_ptr = &tab->coordinates_y[i][j * width];
			for (k = 0; k < width;
			     k++, x_ptr++, y_ptr++, val_x += step) {
				if (k == 0)
					*x_ptr = 0;
				else if (k == width - 1)
					*x_ptr = val_x + 2 * start_x[i];
				else
					*x_ptr = val_x;
				if (j == 0)
					*y_ptr = 0;
				else
					*y_ptr = val_y;
			}
			val_y += step;
		}
	}
	*table = tab;

	return sh_css_success;
}

void
sh_css_invalidate_morph_table(void)
{
	morph_table_changed = true;
}

void
sh_css_set_morph_table(const struct sh_css_morph_table *table)
{
	morph_table = table;
	morph_table_changed = true;
}

void
sh_css_get_morph_table(const struct sh_css_morph_table **table)
{
	*table = morph_table;
}

enum sh_css_err
sh_css_get_3a_statistics(struct sh_css_3a_output *output)
{
	if (!current_3a_binary)
		return sh_css_err_internal_error;

	if (current_3a_binary->info->s3atbl_use_dmem)
		get_3a_stats_from_dmem(output);
	else
		get_3a_stats_from_vmem(output);
	return sh_css_success;
}

void
sh_css_set_3a_config(const struct sh_css_3a_config *config)
{
	if (config)
		s3a_config = config;
	else
		s3a_config = &disabled_3a_config;
	s3a_config_changed = true;
}

void
sh_css_get_3a_config(const struct sh_css_3a_config **config)
{
	*config = s3a_config;
}

void
sh_css_set_wb_config(const struct sh_css_wb_config *config)
{
	if (config)
		wb_config = config;
	else
		wb_config = &disabled_wb_config;
	wb_config_changed = true;
}

void
sh_css_get_wb_config(const struct sh_css_wb_config **config)
{
	*config = wb_config;
}

void
sh_css_set_cc_config(const struct sh_css_cc_config *config)
{
	if (config)
		cc_config = config;
	else
		cc_config = &disabled_cc_config;
	cc_config_changed = true;
}

void
sh_css_get_cc_config(const struct sh_css_cc_config **config)
{
	*config = cc_config;
}

void
sh_css_set_tnr_config(const struct sh_css_tnr_config *config)
{
	if (config)
		tnr_config = config;
	else
		tnr_config = &disabled_tnr_config;
	tnr_config_changed = true;
}

void
sh_css_get_tnr_config(const struct sh_css_tnr_config **config)
{
	*config = tnr_config;
}

void
sh_css_set_ob_config(const struct sh_css_ob_config *config)
{
	if (config)
		ob_config = config;
	else
		ob_config = &disabled_ob_config;
	ob_config_changed = true;
}

void
sh_css_get_ob_config(const struct sh_css_ob_config **config)
{
	*config = ob_config;
}

void
sh_css_set_dp_config(const struct sh_css_dp_config *config)
{
	if (config)
		dp_config = config;
	else
		dp_config = &disabled_dp_config;
	dp_config_changed = true;
}

void
sh_css_get_dp_config(const struct sh_css_dp_config **config)
{
	*config = dp_config;
}

void
sh_css_set_nr_config(const struct sh_css_nr_config *config)
{
	if (config)
		nr_config = config;
	else
		nr_config = &disabled_nr_config;
	nr_config_changed = true;
}

void
sh_css_get_nr_config(const struct sh_css_nr_config **config)
{
	*config = nr_config;
}

void
sh_css_set_ee_config(const struct sh_css_ee_config *config)
{
	if (config)
		ee_config = config;
	else
		ee_config = &disabled_ee_config;
	ee_config_changed = true;
}

void
sh_css_get_ee_config(const struct sh_css_ee_config **config)
{
	*config = ee_config;
}

void
sh_css_set_de_config(const struct sh_css_de_config *config)
{
	if (config)
		de_config = config;
	else
		de_config = &disabled_de_config;
	de_config_changed = true;
}

void
sh_css_get_de_config(const struct sh_css_de_config **config)
{
	*config = de_config;
}

void
sh_css_set_gc_config(const struct sh_css_gc_config *config)
{
	if (config)
		gc_config = config;
	else
		gc_config = &disabled_gc_config;
	gc_config_changed = true;
}

void
sh_css_get_gc_config(const struct sh_css_gc_config **config)
{
	*config = gc_config;
}

void
sh_css_set_anr_config(const struct sh_css_anr_config *config)
{
	if (config)
		anr_config = config;
	else
		anr_config = &default_anr_config;
	anr_config_changed = true;
}

void
sh_css_get_anr_config(const struct sh_css_anr_config **config)
{
	*config = anr_config;
}

void
sh_css_set_ce_config(const struct sh_css_ce_config *config)
{
	if (config)
		ce_config = config;
	else
		ce_config = &default_ce_config;
	ce_config_changed = true;
}

void
sh_css_get_ce_config(const struct sh_css_ce_config **config)
{
	*config = ce_config;
}

static bool
alloc(void **ptr, unsigned int bytes)
{
	void *p = hrt_isp_css_mm_alloc(bytes);
	if (p == NULL)
		return false;
	*ptr = p;
	return true;
}

static inline bool
realloc_buf(void **curr_buf, size_t *curr_size,
	    size_t needed_size, enum sh_css_err *err,
	    bool cached)
{
	/* Possible optimization: add a function hrt_isp_css_mm_realloc()
	 * and implement on top of hmm. */
	if (*curr_size >= needed_size)
		return false;
	if (*curr_buf)
		hrt_isp_css_mm_free(*curr_buf);
	if (cached)
		*curr_buf = hrt_isp_css_mm_alloc_cached(needed_size);
	else
		*curr_buf = hrt_isp_css_mm_alloc(needed_size);
	if (!*curr_buf) {
		*err = sh_css_err_cannot_allocate_memory;
		*curr_size = 0;
	} else {
		*curr_size = needed_size;
	}
	return true;
}

static inline bool
reallocate_buffer(void **curr_buf, size_t *curr_size,
		  size_t needed_size, enum sh_css_err *err)
{
	return realloc_buf(curr_buf, curr_size, needed_size, err, false);
}

static inline bool
reallocate_cached_buffer(void **curr_buf, size_t *curr_size,
			 size_t needed_size, enum sh_css_err *err)
{
	return realloc_buf(curr_buf, curr_size, needed_size, err, true);
}

static enum sh_css_err
reallocate_buffers(const struct sh_css_binary *binary)
{
	bool changed = false;
	enum sh_css_err err = sh_css_success;

	if (binary->info->enable_fpnr) {
		changed |= reallocate_buffer(&ddr_ptrs.fpn_tbl, &fpn_tbl_size,
					     FPNTBL_BYTES(binary), &err);
	}
	if (binary->info->enable_sc) {
		changed |= reallocate_buffer(&ddr_ptrs.sc_tbl, &sc_tbl_size,
					     SCTBL_BYTES(binary), &err);
	}
	if (binary->info->enable_s3a && binary->info->s3atbl_use_dmem) {
		unsigned int size = s3a_tbl_size;
		changed |= reallocate_cached_buffer(&s3a_tables[0],
						    &size,
						    S3ATBL_BYTES(binary), &err);
		changed |= reallocate_cached_buffer(&s3a_tables[1],
						    &s3a_tbl_size,
						    S3ATBL_BYTES(binary), &err);
	}
	if (binary->info->enable_s3a && !binary->info->s3atbl_use_dmem) {
		unsigned int hi_size = s3a_tbl_hi_size,
			     lo_size = s3a_tbl_lo_size;
		changed |= reallocate_cached_buffer(&s3a_tables_hi[0],
						    &hi_size,
						    S3ATBL_HI_LO_BYTES(binary),
						    &err);
		changed |= reallocate_cached_buffer(&s3a_tables_hi[1],
						    &s3a_tbl_hi_size,
						    S3ATBL_HI_LO_BYTES(binary),
						    &err);
		changed |= reallocate_cached_buffer(&s3a_tables_lo[0],
						    &lo_size,
						    S3ATBL_HI_LO_BYTES(binary),
						    &err);
		changed |= reallocate_cached_buffer(&s3a_tables_lo[1],
						    &s3a_tbl_lo_size,
						    S3ATBL_HI_LO_BYTES(binary),
						    &err);
	}
	if (binary->info->enable_dis) {
		unsigned int hor_size = sdis_hor_proj_size,
			     ver_size = sdis_ver_proj_size;
		changed |= reallocate_buffer(&ddr_ptrs.sdis_hor_coef,
					     &sdis_hor_coef_size,
					     SDIS_HOR_COEF_TBL_BYTES(binary),
					     &err);
		changed |= reallocate_buffer(&ddr_ptrs.sdis_ver_coef,
					     &sdis_ver_coef_size,
					     SDIS_VER_COEF_TBL_BYTES(binary),
					     &err);
		changed |= reallocate_cached_buffer(&dis_hor_projections[0],
					     &hor_size,
					     SDIS_HOR_PROJ_TBL_BYTES(binary),
					     &err);
		changed |= reallocate_cached_buffer(&dis_hor_projections[1],
					     &sdis_hor_proj_size,
					     SDIS_HOR_PROJ_TBL_BYTES(binary),
					     &err);
		changed |= reallocate_cached_buffer(&dis_ver_projections[0],
					     &ver_size,
					     SDIS_VER_PROJ_TBL_BYTES(binary),
					     &err);
		changed |= reallocate_cached_buffer(&dis_ver_projections[1],
					     &sdis_ver_proj_size,
					     SDIS_VER_PROJ_TBL_BYTES(binary),
					     &err);
	}
	if (binary->info->mode == SH_CSS_BINARY_MODE_GDC) {
		changed |= reallocate_buffer(&ddr_ptrs.tetra_r_x,
					     &tetra_r_x_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_r_y,
					     &tetra_r_y_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_gr_x,
					     &tetra_gr_x_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_gr_y,
					     &tetra_gr_y_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_gb_x,
					     &tetra_gb_x_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_gb_y,
					     &tetra_gb_y_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_b_x,
					     &tetra_b_x_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_b_y,
					     &tetra_b_y_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_ratb_x,
					     &tetra_ratb_x_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_ratb_y,
					     &tetra_ratb_y_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_batr_x,
					     &tetra_batr_x_size,
					     MORPH_PLANE_BYTES(binary), &err);
		changed |= reallocate_buffer(&ddr_ptrs.tetra_batr_y,
					     &tetra_batr_y_size,
					     MORPH_PLANE_BYTES(binary), &err);
	}
	if (changed)
		hrt_isp_css_mm_store(sp_ddr_ptrs, &ddr_ptrs, sizeof(ddr_ptrs));
	return err;
}

void
sh_css_params_swap_3a_buffers(void)
{
	curr_valid_buffer = 1-curr_valid_buffer;
}

enum sh_css_err
sh_css_params_init(void)
{
	bool succ = true;

	memset(&ddr_ptrs, 0, sizeof(ddr_ptrs));
	succ &= alloc(&ddr_ptrs.isp_param, sizeof(struct sh_css_isp_params));
	succ &= alloc(&ddr_ptrs.ctc_tbl,   sizeof(struct sh_css_ctc_table));
	succ &= alloc(&ddr_ptrs.gamma_tbl, sizeof(struct sh_css_gamma_table));
	succ &= alloc(&ddr_ptrs.macc_tbl,  sizeof(struct sh_css_macc_table));
	fpn_tbl_size = 0;
	sc_tbl_size = 0;
	s3a_tbl_size = 0;
	s3a_tbl_hi_size = 0;
	s3a_tbl_lo_size = 0;
	sdis_hor_coef_size = 0;
	sdis_ver_coef_size = 0;
	sdis_hor_proj_size = 0;
	sdis_ver_proj_size = 0;
	tetra_r_x_size = 0;
	tetra_r_y_size = 0;
	tetra_gr_x_size = 0;
	tetra_gr_y_size = 0;
	tetra_gb_x_size = 0;
	tetra_gb_y_size = 0;
	tetra_b_x_size = 0;
	tetra_b_y_size = 0;
	tetra_batr_x_size = 0;
	tetra_batr_y_size = 0;
	tetra_ratb_x_size = 0;
	tetra_ratb_y_size = 0;

	sp_ddr_ptrs = hrt_isp_css_mm_calloc(CEIL_MUL(sizeof(ddr_ptrs),
						     HIVE_ISP_DDR_WORD_BYTES));
	xmem_sp_group_ptrs = hrt_isp_css_mm_calloc
		(sizeof(struct sh_css_sp_group));
	if (!succ || !sp_ddr_ptrs || !xmem_sp_group_ptrs) {
		sh_css_uninit();
		return sh_css_err_cannot_allocate_memory;
	}
	sh_css_set_3a_config(&default_3a_config);
	sh_css_set_wb_config(&default_wb_config);
	sh_css_set_cc_config(&default_cc_config);
	sh_css_set_tnr_config(&default_tnr_config);
	sh_css_set_ob_config(&default_ob_config);
	sh_css_set_dp_config(&default_dp_config);
	sh_css_set_nr_config(&default_nr_config);
	sh_css_set_ee_config(&default_ee_config);
	sh_css_set_de_config(&default_de_config);
	sh_css_set_gc_config(&default_gc_config);
	sh_css_set_anr_config(&default_anr_config);
	sh_css_set_ce_config(&default_ce_config);
	sh_css_set_macc_table(&default_macc_table);
	sh_css_set_gamma_table(&default_gamma_table);
	sh_css_set_ctc_table(&default_ctc_table);
	sh_css_hrt_gdc_set_lut(zoom_table);
	fpn_table_changed = true;
	isp_parameters.fpn_enabled = 0;
	morph_table = NULL;
	morph_table_changed = true;
	sc_table = NULL;
	sc_table_changed = false;
	curr_valid_buffer = 0;
	return sh_css_success;
}

void
sh_css_params_reconfigure_gdc_lut(void)
{
	sh_css_hrt_gdc_set_lut(zoom_table);
}

#define safe_free(x)	do {                                    \
				if (x)                          \
					hrt_isp_css_mm_free(x); \
				(x) = NULL;                     \
			} while (0)

void
sh_css_params_uninit(void)
{
	safe_free(ddr_ptrs.isp_param);
	safe_free(ddr_ptrs.ctc_tbl);
	safe_free(ddr_ptrs.gamma_tbl);
	safe_free(ddr_ptrs.macc_tbl);
	safe_free(ddr_ptrs.fpn_tbl);
	safe_free(ddr_ptrs.sc_tbl);
	safe_free(s3a_tables[0]);
	safe_free(s3a_tables[1]);
	safe_free(s3a_tables_hi[0]);
	safe_free(s3a_tables_hi[1]);
	safe_free(s3a_tables_lo[0]);
	safe_free(s3a_tables_lo[1]);
	safe_free(dis_hor_projections[0]);
	safe_free(dis_hor_projections[1]);
	safe_free(dis_ver_projections[0]);
	safe_free(dis_ver_projections[1]);
	safe_free(ddr_ptrs.sdis_hor_coef);
	safe_free(ddr_ptrs.sdis_ver_coef);
	safe_free(ddr_ptrs.tetra_r_x);
	safe_free(ddr_ptrs.tetra_r_y);
	safe_free(ddr_ptrs.tetra_gr_x);
	safe_free(ddr_ptrs.tetra_gr_y);
	safe_free(ddr_ptrs.tetra_gb_x);
	safe_free(ddr_ptrs.tetra_gb_y);
	safe_free(ddr_ptrs.tetra_b_x);
	safe_free(ddr_ptrs.tetra_b_y);
	safe_free(ddr_ptrs.tetra_ratb_x);
	safe_free(ddr_ptrs.tetra_ratb_y);
	safe_free(ddr_ptrs.tetra_batr_x);
	safe_free(ddr_ptrs.tetra_batr_y);
	safe_free(sp_ddr_ptrs);
	safe_free(xmem_sp_group_ptrs);
	if (fpn_table.data)
		sh_css_free(fpn_table.data);
}

static void write_morph_plane(unsigned short *data,
			      unsigned int width,
			      unsigned int height,
			      void *dest,
			      unsigned int aligned_width)
{
	unsigned int i, padding, w;

	/* currently we don't have morph table interpolation yet,
	 * so we allow a wider table to be used. This will be removed
	 * in the future. */
	if (width > aligned_width) {
		padding = 0;
		w = aligned_width;
	} else {
		padding = aligned_width - width;
		w = width;
	}

	for (i = 0; i < height; i++) {
		hrt_isp_css_mm_store(dest, data, w * sizeof(short));
		dest += w * sizeof(short);
		hrt_isp_css_mm_set(dest, 0, padding * sizeof(short));
		dest += padding * sizeof(short);
		data += width;
	}
}

/* Store the DIS coefficients from the 3A library to DDR where the ISP
   will read them from. The ISP works on a grid that can be larger than
   that of the 3a library. If that is the case, we padd the difference
   with zeroes. */
static void
store_dis_coefficients(const struct sh_css_binary *binary)
{
	unsigned int hor_num_isp = binary->dis_hor_coef_num_isp,
		     ver_num_isp = binary->dis_ver_coef_num_isp,
		     hor_num_3a  = binary->dis_hor_coef_num_3a,
		     ver_num_3a  = binary->dis_ver_coef_num_3a,
		     hor_padding = hor_num_isp - hor_num_3a,
		     ver_padding = ver_num_isp - ver_num_3a,
		     i;
	const short *hor_ptr_3a = dis_hor_coef_tbl,
		    *ver_ptr_3a = dis_ver_coef_tbl;
	short *hor_ptr_isp = ddr_ptrs.sdis_hor_coef,
	      *ver_ptr_isp = ddr_ptrs.sdis_ver_coef;

	for (i = 0; i < SH_CSS_DIS_NUM_COEF_TYPES; i++) {
		hrt_isp_css_mm_store(hor_ptr_isp, hor_ptr_3a,
				     hor_num_3a * sizeof(*hor_ptr_3a));
		hor_ptr_3a  += hor_num_3a;
		hor_ptr_isp += hor_num_3a;
		hrt_isp_css_mm_set(hor_ptr_isp, 0,
					hor_padding * sizeof(short));
		hor_ptr_isp += hor_padding;
	}
	for (i = 0; i < SH_CSS_DIS_VER_NUM_COEF_TYPES(binary); i++) {
		hrt_isp_css_mm_store(ver_ptr_isp, ver_ptr_3a,
				     ver_num_3a * sizeof(*ver_ptr_3a));
		ver_ptr_3a  += ver_num_3a;
		ver_ptr_isp += ver_num_3a;
		hrt_isp_css_mm_set(ver_ptr_isp, 0,
					ver_padding * sizeof(short));
		ver_ptr_isp += ver_padding;
	}
	dis_coef_table_changed = false;
}

enum sh_css_err
sh_css_params_write_to_ddr(const struct sh_css_binary *binary)
{
	enum sh_css_err err;
	unsigned int free_buffer = 1-curr_valid_buffer;

	err = reallocate_buffers(binary);
	if (err != sh_css_success)
		return err;

	/* Make sure the SP firmware uses the right (free) buffer */
	ddr_ptrs.s3a_tbl       = s3a_tables[free_buffer];
	ddr_ptrs.s3a_tbl_hi    = s3a_tables_hi[free_buffer];
	ddr_ptrs.s3a_tbl_lo    = s3a_tables_lo[free_buffer];
	ddr_ptrs.sdis_hor_proj = dis_hor_projections[free_buffer];
	ddr_ptrs.sdis_ver_proj = dis_ver_projections[free_buffer];

	hrt_isp_css_mm_store(sp_ddr_ptrs, &ddr_ptrs, sizeof(ddr_ptrs));

	if (fpn_table_changed && binary->info->enable_fpnr) {
		if (isp_parameters.fpn_enabled) {
			store_fpntbl(ddr_ptrs.fpn_tbl);
		} else if (SH_CSS_PREVENT_UNINIT_READS) {
			int *ptr = ddr_ptrs.fpn_tbl;
			/* prevent warnings when reading fpn table in csim.
			   Actual values are not used when fpn is disabled. */
			hrt_isp_css_mm_set(ptr, 0, fpn_tbl_size);
		}
		fpn_table_changed = false;
	}
	if (sc_table_changed && binary->info->enable_sc) {
		store_sctbl(binary, ddr_ptrs.sc_tbl);
		sc_table_changed = false;
	}

	if (s3a_config && s3a_config_changed)
		sh_css_process_3a();
	if (wb_config && wb_config_changed)
		sh_css_process_wb();
	if (cc_config && cc_config_changed)
		sh_css_process_cc();
	if (tnr_config && tnr_config_changed)
		sh_css_process_tnr();
	if (ob_config && ob_config_changed)
		sh_css_process_ob();
	if (dp_config && dp_config_changed)
		sh_css_process_dp();
	if (nr_config && ee_config && (nr_config_changed || ee_config_changed))
		sh_css_process_nr_ee();
	if (de_config && de_config_changed)
		sh_css_process_de();
	if (gc_config && gc_config_changed)
		sh_css_process_gc();
	if (anr_config && anr_config_changed)
		sh_css_process_anr();
	if (ce_config && ce_config_changed)
		sh_css_process_ce();

	if (isp_params_changed) {
		if (SH_CSS_PREVENT_UNINIT_READS) {
			/* ispparm struct is read with DMA which reads
			 * multiples of the DDR word with (32 bytes):
			 * So we pad with zeroes to prevent warnings in csim.
			 */
			unsigned int aligned_width, padding_bytes;
			char *pad_ptr;

			aligned_width = CEIL_MUL(
					  sizeof(struct sh_css_isp_params),
					  HIVE_ISP_DDR_WORD_BYTES);
			padding_bytes = aligned_width -
					sizeof(struct sh_css_isp_params);
			pad_ptr = ddr_ptrs.isp_param +
					sizeof(struct sh_css_isp_params);
			hrt_isp_css_mm_set(pad_ptr, 0, padding_bytes);
		}
		hrt_isp_css_mm_store(ddr_ptrs.isp_param,
				     &isp_parameters,
				     sizeof(struct sh_css_isp_params));
		isp_params_changed = false;
	}

	if (ctc_table && ctc_table_changed) {
		hrt_isp_css_mm_store(ddr_ptrs.ctc_tbl,
				     ctc_table->data,
				     sizeof(ctc_table->data));
		ctc_table_changed = false;
	}
	if (gamma_table && gamma_table_changed) {
		hrt_isp_css_mm_store(ddr_ptrs.gamma_tbl,
				     gamma_table->data,
				     sizeof(gamma_table->data));
		gamma_table_changed = false;
	}
	if (macc_table && macc_table_changed) {
		unsigned int i, j, idx;
		unsigned int idx_map[] = {
			0, 1, 3, 2, 6, 7, 5, 4, 12, 13, 15, 14, 10, 11, 9, 8};

		for (i = 0; i < SH_CSS_MACC_NUM_AXES; i++) {
			idx = 4*idx_map[i];
			j   = 4*i;

			converted_macc_table.data[idx] =
			    sDIGIT_FITTING(macc_table->data[j], 13,
					   SH_CSS_MACC_COEF_SHIFT);
			converted_macc_table.data[idx+1] =
			    sDIGIT_FITTING(macc_table->data[j+1], 13,
					   SH_CSS_MACC_COEF_SHIFT);
			converted_macc_table.data[idx+2] =
			    sDIGIT_FITTING(macc_table->data[j+2], 13,
					   SH_CSS_MACC_COEF_SHIFT);
			converted_macc_table.data[idx+3] =
			    sDIGIT_FITTING(macc_table->data[j+3], 13,
					   SH_CSS_MACC_COEF_SHIFT);
		}
		hrt_isp_css_mm_store(ddr_ptrs.macc_tbl,
				     converted_macc_table.data,
				     sizeof(converted_macc_table.data));
		macc_table_changed = false;
	}

	if (dis_coef_table_changed && binary->info->enable_dis) {
		store_dis_coefficients(binary);
		dis_coef_table_changed = false;
	}

	if (binary->info->mode == SH_CSS_BINARY_MODE_GDC &&
	    morph_table_changed) {
		unsigned int i;
		void *virt_addr_tetra_x[SH_CSS_MORPH_TABLE_NUM_PLANES] = {
			ddr_ptrs.tetra_r_x,
			ddr_ptrs.tetra_gr_x,
			ddr_ptrs.tetra_gb_x,
			ddr_ptrs.tetra_b_x,
			ddr_ptrs.tetra_ratb_x,
			ddr_ptrs.tetra_batr_x
		};
		void *virt_addr_tetra_y[SH_CSS_MORPH_TABLE_NUM_PLANES] = {
			ddr_ptrs.tetra_r_y,
			ddr_ptrs.tetra_gr_y,
			ddr_ptrs.tetra_gb_y,
			ddr_ptrs.tetra_b_y,
			ddr_ptrs.tetra_ratb_y,
			ddr_ptrs.tetra_batr_y
		};
		const struct sh_css_morph_table *table = morph_table;
		struct sh_css_morph_table *id_table = NULL;
		if (table &&
		    (table->width < binary->morph_tbl_width ||
		     table->height < binary->morph_tbl_height)) {
			table = NULL;
		}
		if (!table) {
			sh_css_params_default_morph_table(&id_table, binary);
			table = id_table;
		}

		for (i = 0; i < SH_CSS_MORPH_TABLE_NUM_PLANES; i++) {
			write_morph_plane(table->coordinates_x[i],
					  table->width,
					  table->height,
					  virt_addr_tetra_x[i],
					  binary->morph_tbl_aligned_width);
			write_morph_plane(table->coordinates_y[i],
					  table->width,
					  table->height,
					  virt_addr_tetra_y[i],
					  binary->morph_tbl_aligned_width);
		}
		if (id_table)
			sh_css_morph_table_free(id_table);
		morph_table_changed = false;
	}
	return sh_css_success;
}

void
sh_css_params_set_current_binary(const struct sh_css_binary *binary)
{
	if (binary->info->enable_s3a)
		current_3a_binary = binary;
}

const struct sh_css_fpn_table *
sh_css_get_fpn_table(void)
{
	return &fpn_table;
}

const struct sh_css_shading_table *
sh_css_get_shading_table(void)
{
	return sc_table;
}

const struct sh_css_isp_params *
sh_css_get_isp_params(void)
{
	return &isp_parameters;
}

const struct sh_css_binary *
sh_css_get_3a_binary(void)
{
	return current_3a_binary;
}

void
sh_css_get_isp_dis_coefficients(short *horizontal_coefficients,
				short *vertical_coefficients)
{
	unsigned int hor_num_isp, ver_num_isp, i;
	short *hor_ptr     = horizontal_coefficients,
	      *ver_ptr     = vertical_coefficients,
	      *hor_ptr_isp = ddr_ptrs.sdis_hor_coef,
	      *ver_ptr_isp = ddr_ptrs.sdis_ver_coef;

	if (current_3a_binary == NULL)
		return;

	hor_num_isp = current_3a_binary->dis_hor_coef_num_isp;
	ver_num_isp = current_3a_binary->dis_ver_coef_num_isp;

	for (i = 0; i < SH_CSS_DIS_NUM_COEF_TYPES; i++) {
		hrt_isp_css_mm_load(hor_ptr_isp, hor_ptr,
			hor_num_isp * sizeof(short));
		hor_ptr_isp += hor_num_isp;
		hor_ptr     += hor_num_isp;
	}
	for (i = 0; i < SH_CSS_DIS_VER_NUM_COEF_TYPES(current_3a_binary); i++) {
		hrt_isp_css_mm_load(ver_ptr_isp, ver_ptr,
			ver_num_isp * sizeof(short));
		ver_ptr_isp += ver_num_isp;
		ver_ptr     += ver_num_isp;
	}
}

void
sh_css_get_isp_dis_projections(int *horizontal_projections,
			       int *vertical_projections)
{
	unsigned int hor_num_isp, ver_num_isp, i;
	int *hor_ptr     = horizontal_projections,
	    *ver_ptr     = vertical_projections,
	    *hor_ptr_isp = dis_hor_projections[curr_valid_buffer],
	    *ver_ptr_isp = dis_ver_projections[curr_valid_buffer];

	if (current_3a_binary == NULL)
		return;

	hor_num_isp = current_3a_binary->dis_hor_proj_num_isp;
	ver_num_isp = current_3a_binary->dis_ver_proj_num_isp;

	for (i = 0; i < SH_CSS_DIS_NUM_COEF_TYPES; i++) {
		hrt_isp_css_mm_load(hor_ptr_isp, hor_ptr,
				    hor_num_isp * sizeof(int));
		hor_ptr_isp += hor_num_isp;
		hor_ptr     += hor_num_isp;

		hrt_isp_css_mm_load(ver_ptr_isp, ver_ptr,
				    ver_num_isp * sizeof(int));
		ver_ptr_isp += ver_num_isp;
		ver_ptr     += ver_num_isp;
	}
}

void *
sh_css_store_sp_group_to_ddr(void)
{
	hrt_isp_css_mm_store(xmem_sp_group_ptrs,
			     &sh_css_sp_group,
			     sizeof(struct sh_css_sp_group));
	return xmem_sp_group_ptrs;
}
