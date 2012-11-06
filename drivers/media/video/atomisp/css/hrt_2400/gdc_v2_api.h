#ifndef HRT_GDC_V2_api_h_
#define HRT_GDC_V2_api_h_

#include "gdc_v2_defs.h"

/* In case of crun, the CRUN bit is set to 1                */
/* In case of all the other runs, the NCRUN bit is set to 0 */
#ifdef __HIVECC
#define HRT_GDC_PACK_CMD(cmd) (cmd)
#else
#define HRT_GDC_PACK_CMD(cmd) ((cmd) | (1 << HRT_GDC_CRUN_POS))
#endif

#ifdef __HIVECC
#define HRT_GDC_ADDR(addr) ((unsigned int)(addr))
#else
#define HRT_GDC_ADDR(addr) ((unsigned int)(addr))
#endif

typedef enum {
  gdc_8_bpp  = 8,
  gdc_10_bpp = 10,
  gdc_12_bpp = 12,
  gdc_14_bpp = 14
} gdc_bits_per_pixel;


#define _hive_gdc_sru(gdc_id)         HIVE_ ## gdc_id ## _SRU
#define _hive_gdc_fifo(gdc_id)        HIVE_ ## gdc_id ## _FIFO
#define _hive_gdc_snd(gdc_id, value)  std_snd(_hive_gdc_sru(gdc_id), _hive_gdc_fifo(gdc_id), value)
#define _hive_gdc_rcv(gdc_id)         (OP_std_rcv(_hive_gdc_fifo(gdc_id)) ON(_hive_gdc_sru(gdc_id)))

#define hive_gdc_wait_for_ack(gdc_id) \
  _hive_gdc_rcv(gdc_id)

#ifdef __HIVECC
#define _hive_gdc_snd_if_crun(gdc_id, is_var)
#else
#define _hive_gdc_snd_if_crun(gdc_id, is_var) _hive_gdc_snd(gdc_id, is_var)
#endif

/* Only in the first token of the data (kick) command, we set the HRT_GDC_DATA_CMD id */
/* CRUN bit is not and should not be modified                                         */
#define hive_gdc_scaling(gdc_id, ipxfrx, fry, src_address, dst_address, src_is_var, dst_is_var)\
{\
  _hive_gdc_snd(gdc_id, HRT_GDC_PACK_CMD( ipxfrx | (fry<<HRT_GDC_FRY_BIT_OFFSET) | (HRT_GDC_DATA_CMD << HRT_GDC_CMD_POS) ) );\
  _hive_gdc_snd(gdc_id, (unsigned) src_address);\
  _hive_gdc_snd(gdc_id, (unsigned) dst_address);\
  _hive_gdc_snd_if_crun(gdc_id, src_is_var);\
  _hive_gdc_snd_if_crun(gdc_id, dst_is_var);\
}

/* Only in the first token of the data (kick) command, we set the HRT_GDC_DATA_CMD id */
/* CRUN bit is not and should not be modified                                         */
#define hive_gdc_tetragon(gdc_id, p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y, src_address, dst_address, src_is_var, dst_is_var)\
{\
  _hive_gdc_snd(gdc_id, HRT_GDC_PACK_CMD( p0x | (HRT_GDC_DATA_CMD << HRT_GDC_CMD_POS) ) );\
  _hive_gdc_snd(gdc_id, p0y);\
  _hive_gdc_snd(gdc_id, p1x);\
  _hive_gdc_snd(gdc_id, p1y);\
  _hive_gdc_snd(gdc_id, p2x);\
  _hive_gdc_snd(gdc_id, p2y);\
  _hive_gdc_snd(gdc_id, p3x);\
  _hive_gdc_snd(gdc_id, p3y);\
  _hive_gdc_snd(gdc_id, (unsigned) src_address);\
  _hive_gdc_snd(gdc_id, (unsigned) dst_address);\
  _hive_gdc_snd_if_crun(gdc_id, src_is_var);\
  _hive_gdc_snd_if_crun(gdc_id, dst_is_var);\
}

/* Only the Config command contains the CRUN bit                                      */
#define hrt_gdc_config(gdc_id, reg_id, value)\
{\
  _hive_gdc_snd(gdc_id, HRT_GDC_PACK_CMD((((unsigned int)(value)) << HRT_GDC_DATA_POS) | (((unsigned int)(reg_id)) << HRT_GDC_REG_ID_POS) | (HRT_GDC_CONFIG_CMD << HRT_GDC_CMD_POS)));\
}

// Theoretically could be set by using the set register command from the FIFO (or Slave I/F), but 



//#define hive_gdc_set_fryipxfrx(gdc_id, value)      hrt_gdc_config(gdc_id, HRT_GDC_FRYIPXFRX_IDX, value) 
//#define hive_gdc_set_src_addr(gdc_id, value)       hrt_gdc_config(gdc_id, HRT_GDC_SRC_ADDR_IDX, value)
//#define hive_gdc_set_dst_addr(gdc_id, value)       hrt_gdc_config(gdc_id, HRT_GDC_DST_ADDR_IDX, value)

//#define hive_gdc_set_p0x(gdc_id, value)            hrt_gdc_config(gdc_id, HRT_GDC_P0X_IDX, value)
//#define hive_gdc_set_p0y(gdc_id, value)            hrt_gdc_config(gdc_id, HRT_GDC_P0Y_IDX, value)
//#define hive_gdc_set_p1x(gdc_id, value)            hrt_gdc_config(gdc_id, HRT_GDC_P1X_IDX, value)
//#define hive_gdc_set_p1y(gdc_id, value)            hrt_gdc_config(gdc_id, HRT_GDC_P1Y_IDX, value)
//#define hive_gdc_set_p2x(gdc_id, value)            hrt_gdc_config(gdc_id, HRT_GDC_P2X_IDX, value)
//#define hive_gdc_set_p2y(gdc_id, value)            hrt_gdc_config(gdc_id, HRT_GDC_P2Y_IDX, value)
//#define hive_gdc_set_p3x(gdc_id, value)            hrt_gdc_config(gdc_id, HRT_GDC_P3X_IDX, value)
//#define hive_gdc_set_p3y(gdc_id, value)            hrt_gdc_config(gdc_id, HRT_GDC_P3Y_IDX, value)


#define hive_gdc_set_bpp(gdc_id, value)            hrt_gdc_config(gdc_id, HRT_GDC_BPP_IDX, value)
#define hive_gdc_set_oxdim(gdc_id, value)          hrt_gdc_config(gdc_id, HRT_GDC_OXDIM_IDX, value)
#define hive_gdc_set_oydim(gdc_id, value)          hrt_gdc_config(gdc_id, HRT_GDC_OYDIM_IDX, value)

#ifdef __HIVECC
#define hive_gdc_set_src_end_addr(gdc_id, value)   hrt_gdc_config(gdc_id, HRT_GDC_SRC_END_ADDR_IDX, (unsigned) value)
#define hive_gdc_set_src_wrap_addr(gdc_id, value)  hrt_gdc_config(gdc_id, HRT_GDC_SRC_WRAP_ADDR_IDX, (unsigned) value)
#else
#define hive_gdc_set_src_end_addr(gdc_id, value) \
hrt_gdc_config(gdc_id, HRT_GDC_SRC_END_ADDR_IDX,  0xBEEFED) \
_hive_gdc_snd(gdc_id, (unsigned) value);
#define hive_gdc_set_src_wrap_addr(gdc_id, value) \
hrt_gdc_config(gdc_id, HRT_GDC_SRC_WRAP_ADDR_IDX, 0xBEEFED) \
_hive_gdc_snd(gdc_id, (unsigned) value);
#endif

#define hive_gdc_set_src_stride(gdc_id, value)     hrt_gdc_config(gdc_id, HRT_GDC_SRC_STRIDE_IDX, value)
#define hive_gdc_set_dst_stride(gdc_id, value)     hrt_gdc_config(gdc_id, HRT_GDC_DST_STRIDE_IDX, value)
#define hive_gdc_set_dx(gdc_id, value)             hrt_gdc_config(gdc_id, HRT_GDC_DX_IDX, value)
#define hive_gdc_set_dy(gdc_id, value)             hrt_gdc_config(gdc_id, HRT_GDC_DY_IDX, value)
#define hive_gdc_set_ixdim(gdc_id, value)          hrt_gdc_config(gdc_id, HRT_GDC_P0X_IDX, value)
#define hive_gdc_set_iydim(gdc_id, value)          hrt_gdc_config(gdc_id, HRT_GDC_P0Y_IDX, value)


#define hive_gdc_set_perf_mode(gdc_id, value)      hrt_gdc_config(gdc_id, HRT_GDC_PERF_POINT_IDX, value)
#define hive_gdc_set_interp_type(gdc_id, value)    hrt_gdc_config(gdc_id, HRT_GDC_INTERP_TYPE_IDX, value)
#define hive_gdc_set_scan(gdc_id, value)           hrt_gdc_config(gdc_id, HRT_GDC_SCAN_IDX, value)
#define hive_gdc_set_proc_shape(gdc_id, value)     hrt_gdc_config(gdc_id, HRT_GDC_PROC_MODE_IDX , value)


#define hive_gdc_set_perf_xy_11(gdc_id)            hive_gdc_set_perf_mode(gdc_id, HRT_GDC_PERF_1_1_pix)
#define hive_gdc_set_perf_xy_12(gdc_id)            hive_gdc_set_perf_mode(gdc_id, HRT_GDC_PERF_1_2_pix)
#define hive_gdc_set_perf_xy_21(gdc_id)            hive_gdc_set_perf_mode(gdc_id, HRT_GDC_PERF_2_1_pix)
#define hive_gdc_set_perf_xy_22(gdc_id)            hive_gdc_set_perf_mode(gdc_id, HRT_GDC_PERF_2_2_pix)

#define hive_gdc_set_interp_nnd(gdc_id)            hive_gdc_set_interp_type(gdc_id, HRT_GDC_NND_MODE)
#define hive_gdc_set_interp_bli(gdc_id)            hive_gdc_set_interp_type(gdc_id, HRT_GDC_BLI_MODE)
#define hive_gdc_set_interp_bci(gdc_id)            hive_gdc_set_interp_type(gdc_id, HRT_GDC_BCI_MODE)
#define hive_gdc_set_interp_lut(gdc_id)            hive_gdc_set_interp_type(gdc_id, HRT_GDC_LUT_MODE)

#define hive_gdc_set_scan_stb(gdc_id)              hive_gdc_set_scan(gdc_id, HRT_GDC_SCAN_STB)
#define hive_gdc_set_scan_str(gdc_id)              hive_gdc_set_scan(gdc_id, HRT_GDC_SCAN_STR)

#define hive_gdc_set_shape_scaling(gdc_id)         hive_gdc_set_proc_shape(gdc_id, HRT_GDC_MODE_SCALING)
#define hive_gdc_set_shape_tetragon(gdc_id)        hive_gdc_set_proc_shape(gdc_id, HRT_GDC_MODE_TETRAGON)


#endif /* HRT_GDC_V2_api_h_ */
