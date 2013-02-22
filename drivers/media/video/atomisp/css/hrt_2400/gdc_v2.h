#ifndef _hrt_gdc_v2_h
#define _hrt_gdc_v2_h

#include "gdc_v2_defs.h"


#define hrt_gdc_slave_port(gdc_id)    HRTCAT(gdc_id,_sl_in)
#define hrt_gdc_register_address(reg) (_HRT_GDC_REG_ALIGN * (reg))

#define _hrt_gdc_set_register(gdc_id, reg, val) \
  _hrt_slave_port_store_32_volatile(hrt_gdc_slave_port(gdc_id), hrt_gdc_register_address(reg), (val))

/* Sets one entry of gdc LUT memory (range of entries is from 0 to 4096) */
#define _hrt_gdc_set_lut_entry(gdc_id, idx, val) \
  _hrt_slave_port_store_16_volatile(hrt_gdc_slave_port(gdc_id),hrt_gdc_register_address(HRT_GDC_LUT_IDX) + 2*idx , (val))


#define _hrt_gdc_get_register(gdc_id, reg) \
  _hrt_slave_port_load_32_volatile(hrt_gdc_slave_port(gdc_id), hrt_gdc_register_address(reg))

#define HRT_GDC_LUT_BASE_ADDR hrt_gdc_register_address(HRT_GDC_LUT_IDX)

/* register addresses          */
#define hrt_gdc_status_register_address()        hrt_gdc_register_address(HRT_GDC_CHK_ENGINE_IDX)
#define hrt_gdc_woix_register_address()          hrt_gdc_register_address(HRT_GDC_WOIX_IDX)
#define hrt_gdc_woiy_register_address()          hrt_gdc_register_address(HRT_GDC_WOIY_IDX)
#define hrt_gdc_bpp_register_address()           hrt_gdc_register_address(HRT_GDC_BPP_IDX)
#define hrt_gdc_fryipxfrx_register_address()     hrt_gdc_register_address(HRT_GDC_FRYIPXFRX_IDX)
#define hrt_gdc_oxdim_register_address()         hrt_gdc_register_address(HRT_GDC_OXDIM_IDX)
#define hrt_gdc_oydim_register_address()         hrt_gdc_register_address(HRT_GDC_OYDIM_IDX)
#define hrt_gdc_src_addr_register_address()      hrt_gdc_register_address(HRT_GDC_SRC_ADDR_IDX)
#define hrt_gdc_src_end_addr_register_address()  hrt_gdc_register_address(HRT_GDC_SRC_END_ADDR_IDX)
#define hrt_gdc_src_wrap_addr_register_address() hrt_gdc_register_address(HRT_GDC_SRC_WRAP_ADDR_IDX)
#define hrt_gdc_src_stride_register_address()    hrt_gdc_register_address(HRT_GDC_SRC_STRIDE_IDX)
#define hrt_gdc_dst_addr_register_address()      hrt_gdc_register_address(HRT_GDC_DST_ADDR_IDX)
#define hrt_gdc_dst_stride_register_address()    hrt_gdc_register_address(HRT_GDC_DST_STRIDE_IDX)
#define hrt_gdc_dx_register_address()            hrt_gdc_register_address(HRT_GDC_DX_IDX)
#define hrt_gdc_dy_register_address()            hrt_gdc_register_address(HRT_GDC_DY_IDX)
#define hrt_gdc_p0x_register_address()           hrt_gdc_register_address(HRT_GDC_P0X_IDX)
#define hrt_gdc_p1x_register_address()           hrt_gdc_register_address(HRT_GDC_P1X_IDX)
#define hrt_gdc_p2x_register_address()           hrt_gdc_register_address(HRT_GDC_P2X_IDX)
#define hrt_gdc_p3x_register_address()           hrt_gdc_register_address(HRT_GDC_P3X_IDX)
#define hrt_gdc_p0y_register_address()           hrt_gdc_register_address(HRT_GDC_P0Y_IDX)
#define hrt_gdc_p1y_register_address()           hrt_gdc_register_address(HRT_GDC_P1Y_IDX)
#define hrt_gdc_p2y_register_address()           hrt_gdc_register_address(HRT_GDC_P2Y_IDX)
#define hrt_gdc_p3y_register_address()           hrt_gdc_register_address(HRT_GDC_P3Y_IDX)
#define hrt_gdc_perf_point_register_address()    hrt_gdc_register_address(HRT_GDC_PERF_POINT_IDX)
#define hrt_gdc_interp_type_register_address()   hrt_gdc_register_address(HRT_GDC_INTERP_TYPE_IDX)
#define hrt_gdc_scan_register_address()          hrt_gdc_register_address(HRT_GDC_SCAN_IDX)
#define hrt_gdc_proc_mode_register_address()     hrt_gdc_register_address(HRT_GDC_PROC_MODE_IDX)


/* set registers                        */
/* status register cannot be set (ro)   */
/* FRYIPXFRX, Tetragon and SRC/DST address */
/* should not be set from here,         */
/* but only as part of the data tokens, */
/* while kicking the GDC_V2             */

//#define hrt_gdc_set_fryipxfrx(gdc_id)                _hrt_gdc_set_register(gdc_id, HRT_GDC_FRYIPXFRX_ID, value)
//#define hrt_gdc_set_p0x(gdc_id, value)            _hrt_gdc_set_register(gdc_id, HRT_GDC_P0X_IDX, value)
//#define hrt_gdc_set_p0y(gdc_id, value)            _hrt_gdc_set_register(gdc_id, HRT_GDC_P0Y_IDX, value)
//#define hrt_gdc_set_p1x(gdc_id, value)            _hrt_gdc_set_register(gdc_id, HRT_GDC_P1X_IDX, value)
//#define hrt_gdc_set_p1y(gdc_id, value)            _hrt_gdc_set_register(gdc_id, HRT_GDC_P1Y_IDX, value)
//#define hrt_gdc_set_p2x(gdc_id, value)            _hrt_gdc_set_register(gdc_id, HRT_GDC_P2X_IDX, value)
//#define hrt_gdc_set_p2y(gdc_id, value)            _hrt_gdc_set_register(gdc_id, HRT_GDC_P2Y_IDX, value)
//#define hrt_gdc_set_p3x(gdc_id, value)            _hrt_gdc_set_register(gdc_id, HRT_GDC_P3X_IDX, value)
//#define hrt_gdc_set_p3y(gdc_id, value)            _hrt_gdc_set_register(gdc_id, HRT_GDC_P3Y_IDX, value)
//#define hrt_gdc_set_src_addr(gdc_id, value)       _hrt_gdc_set_register(gdc_id, HRT_GDC_SRC_ADDR_IDX, value)
//#define hrt_gdc_set_dst_addr(gdc_id, value)       _hrt_gdc_set_register(gdc_id, HRT_GDC_DST_ADDR_IDX, value)

#define hrt_gdc_set_bpp(gdc_id, value)            _hrt_gdc_set_register(gdc_id, HRT_GDC_BPP_IDX, value)
#define hrt_gdc_set_oxdim(gdc_id, value)          _hrt_gdc_set_register(gdc_id, HRT_GDC_OXDIM_IDX, value)
#define hrt_gdc_set_oydim(gdc_id, value)          _hrt_gdc_set_register(gdc_id, HRT_GDC_OYDIM_IDX, value)
#define hrt_gdc_set_ixdim(gdc_id, value)          _hrt_gdc_set_register(gdc_id, HRT_GDC_P0X_IDX, value)
#define hrt_gdc_set_iydim(gdc_id, value)          _hrt_gdc_set_register(gdc_id, HRT_GDC_P0Y_IDX, value)
#define hrt_gdc_set_src_end_addr(gdc_id, value)   _hrt_gdc_set_register(gdc_id, HRT_GDC_SRC_END_ADDR_IDX, value)
#define hrt_gdc_set_src_wrap_addr(gdc_id, value)  _hrt_gdc_set_register(gdc_id, HRT_GDC_SRC_WRAP_ADDR_IDX, value)
#define hrt_gdc_set_src_stride(gdc_id, value)     _hrt_gdc_set_register(gdc_id, HRT_GDC_SRC_STRIDE_IDX, value)
#define hrt_gdc_set_dst_stride(gdc_id, value)     _hrt_gdc_set_register(gdc_id, HRT_GDC_DST_STRIDE_IDX, value)
#define hrt_gdc_set_dx(gdc_id, value)             _hrt_gdc_set_register(gdc_id, HRT_GDC_DX_IDX, value)
#define hrt_gdc_set_dy(gdc_id, value)             _hrt_gdc_set_register(gdc_id, HRT_GDC_DY_IDX, value)
#define hrt_gdc_set_perf_point(gdc_id, value)     _hrt_gdc_set_register(gdc_id, HRT_GDC_PERF_POINT_IDX, value)
#define hrt_gdc_set_interp_type(gdc_id, value)    _hrt_gdc_set_register(gdc_id, HRT_GDC_INTERP_TYPE_IDX, value)
#define hrt_gdc_set_scan(gdc_id, value)           _hrt_gdc_set_register(gdc_id, HRT_GDC_SCAN_IDX, value)
#define hrt_gdc_set_proc_mode(gdc_id, value)      _hrt_gdc_set_register(gdc_id, HRT_GDC_PROC_MODE_IDX, value)


#define hrt_gdc_set_perf_xy_11(gdc_id)            hrt_gdc_set_perf_point(gdc_id, HRT_GDC_PERF_1_1_pix)
#define hrt_gdc_set_perf_xy_12(gdc_id)            hrt_gdc_set_perf_point(gdc_id, HRT_GDC_PERF_1_2_pix)
#define hrt_gdc_set_perf_xy_21(gdc_id)            hrt_gdc_set_perf_point(gdc_id, HRT_GDC_PERF_2_1_pix)
#define hrt_gdc_set_perf_xy_22(gdc_id)            hrt_gdc_set_perf_point(gdc_id, HRT_GDC_PERF_2_2_pix)

#define hrt_gdc_set_interp_nnd(gdc_id)            hrt_gdc_set_interp_type(gdc_id, HRT_GDC_NND_MODE)
#define hrt_gdc_set_interp_bli(gdc_id)            hrt_gdc_set_interp_type(gdc_id, HRT_GDC_BLI_MODE)
#define hrt_gdc_set_interp_bci(gdc_id)            hrt_gdc_set_interp_type(gdc_id, HRT_GDC_BCI_MODE)
#define hrt_gdc_set_interp_lut(gdc_id)            hrt_gdc_set_interp_type(gdc_id, HRT_GDC_LUT_MODE)

#define hrt_gdc_set_scan_stb(gdc_id)              hrt_gdc_set_scan(gdc_id, HRT_GDC_SCAN_STB)
#define hrt_gdc_set_scan_str(gdc_id)              hrt_gdc_set_scan(gdc_id, HRT_GDC_SCAN_STR)

#define hrt_gdc_set_mode_scaling(gdc_id)          hrt_gdc_set_proc_mode(gdc_id, HRT_GDC_MODE_SCALING)
#define hrt_gdc_set_mode_tetragon(gdc_id)         hrt_gdc_set_proc_mode(gdc_id, HRT_GDC_MODE_TETRAGON)



#define hrt_gdc_set_lut_32(gdc_id, lut)\
{\
  unsigned int i;\
  for (i=0; i<HRT_GDC_N; i++) { \
    unsigned int entry_0 = lut[0][i] & HRT_GDC_BCI_COEF_MASK, \
                 entry_1 = lut[1][i] & HRT_GDC_BCI_COEF_MASK, \
                 entry_2 = lut[2][i] & HRT_GDC_BCI_COEF_MASK, \
                 entry_3 = lut[3][i] & HRT_GDC_BCI_COEF_MASK, \
                 word_0  = entry_0 | (entry_1 << HRT_GDC_LUT_COEFF_OFFSET), \
                 word_1  = entry_2 | (entry_3 << HRT_GDC_LUT_COEFF_OFFSET); \
    _hrt_gdc_set_register(gdc_id, (HRT_GDC_LUT_IDX + i*2 + 0), word_0); \
    _hrt_gdc_set_register(gdc_id, (HRT_GDC_LUT_IDX + i*2 + 1), word_1); \
  } \
}

#define hrt_gdc_set_lut_16(gdc_id, lut)\
{\
  unsigned int i;\
  for (i=0; i<HRT_GDC_N; i++) { \
    _hrt_gdc_set_lut_entry(gdc_id, (4*i + 0), (lut[0][i])); \
    _hrt_gdc_set_lut_entry(gdc_id, (4*i + 1), (lut[1][i])); \
    _hrt_gdc_set_lut_entry(gdc_id, (4*i + 2), (lut[2][i])); \
    _hrt_gdc_set_lut_entry(gdc_id, (4*i + 3), (lut[3][i])); \
  } \
}


/* get registers */
#define hrt_gdc_get_status(gdc_id)         _hrt_gdc_get_register(gdc_id, HRT_GDC_CHK_ENGINE_IDX)
#define hrt_gdc_get_woix(gdc_id)           _hrt_gdc_get_register(gdc_id, HRT_GDC_WOIX_IDX)
#define hrt_gdc_get_woiy(gdc_id)           _hrt_gdc_get_register(gdc_id, HRT_GDC_WOIY_IDX)
#define hrt_gdc_get_bpp(gdc_id)            _hrt_gdc_get_register(gdc_id, HRT_GDC_BPP_IDX)
#define hrt_gdc_get_fryipxfrx(gdc_id)      _hrt_gdc_get_register(gdc_id, HRT_GDC_FRYIPXFRX_IDX)
#define hrt_gdc_get_oxdim(gdc_id)          _hrt_gdc_get_register(gdc_id, HRT_GDC_OXDIM_IDX)
#define hrt_gdc_get_oydim(gdc_id)          _hrt_gdc_get_register(gdc_id, HRT_GDC_OYDIM_IDX)
#define hrt_gdc_get_ixdim(gdc_id)          _hrt_gdc_get_register(gdc_id, HRT_GDC_P0X_IDX)
#define hrt_gdc_get_iydim(gdc_id)          _hrt_gdc_get_register(gdc_id, HRT_GDC_P0Y_IDX)
#define hrt_gdc_get_src_addr(gdc_id)       _hrt_gdc_get_register(gdc_id, HRT_GDC_SRC_ADDR_IDX)
#define hrt_gdc_get_src_end_addr(gdc_id)   _hrt_gdc_get_register(gdc_id, HRT_GDC_SRC_END_ADDR_IDX)
#define hrt_gdc_get_src_wrap_addr(gdc_id)  _hrt_gdc_get_register(gdc_id, HRT_GDC_SRC_WRAP_ADDR_IDX)
#define hrt_gdc_get_src_stride(gdc_id)     _hrt_gdc_get_register(gdc_id, HRT_GDC_SRC_STRIDE_IDX)
#define hrt_gdc_get_dst_addr(gdc_id)       _hrt_gdc_get_register(gdc_id, HRT_GDC_DST_ADDR_IDX)
#define hrt_gdc_get_dst_stride(gdc_id)     _hrt_gdc_get_register(gdc_id, HRT_GDC_DST_STRIDE_IDX)
#define hrt_gdc_get_dx(gdc_id)             _hrt_gdc_get_register(gdc_id, HRT_GDC_DX_IDX)
#define hrt_gdc_get_dy(gdc_id)             _hrt_gdc_get_register(gdc_id, HRT_GDC_DY_IDX)
#define hrt_gdc_get_p0x(gdc_id)            _hrt_gdc_get_register(gdc_id, HRT_GDC_P0X_IDX)
#define hrt_gdc_get_p1x(gdc_id)            _hrt_gdc_get_register(gdc_id, HRT_GDC_P1X_IDX)
#define hrt_gdc_get_p2x(gdc_id)            _hrt_gdc_get_register(gdc_id, HRT_GDC_P2X_IDX)
#define hrt_gdc_get_p3x(gdc_id)            _hrt_gdc_get_register(gdc_id, HRT_GDC_P3X_IDX)
#define hrt_gdc_get_p0y(gdc_id)            _hrt_gdc_get_register(gdc_id, HRT_GDC_P0Y_IDX)
#define hrt_gdc_get_p1y(gdc_id)            _hrt_gdc_get_register(gdc_id, HRT_GDC_P1Y_IDX)
#define hrt_gdc_get_p2y(gdc_id)            _hrt_gdc_get_register(gdc_id, HRT_GDC_P2Y_IDX)
#define hrt_gdc_get_p3y(gdc_id)            _hrt_gdc_get_register(gdc_id, HRT_GDC_P3Y_IDX)
#define hrt_gdc_get_perf_point(gdc_id)     _hrt_gdc_get_register(gdc_id, HRT_GDC_PERF_POINT_IDX)
#define hrt_gdc_get_interp_type(gdc_id)    _hrt_gdc_get_register(gdc_id, HRT_GDC_INTERP_TYPE_IDX)
#define hrt_gdc_get_scan(gdc_id)           _hrt_gdc_get_register(gdc_id, HRT_GDC_SCAN_IDX)
#define hrt_gdc_get_proc_mode(gdc_id)      _hrt_gdc_get_register(gdc_id, HRT_GDC_PROC_MODE_IDX)

#endif /* _hrt_gdc_v2_h */
