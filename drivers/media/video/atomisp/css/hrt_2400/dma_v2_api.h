#ifndef _hss_dma_v2_api_h
#define _hss_dma_v2_api_h

#include "dma_v2_defs.h"

#define _hive_dma_sru(dma_id)        HIVE_ ## dma_id ## _SRU
#define _hive_dma_fifo(dma_id)       HIVE_ ## dma_id ## _FIFO
#define _hive_dma_snd(dma_id, token) std_snd(_hive_dma_sru(dma_id), _hive_dma_fifo(dma_id), (unsigned int)(token))
#define _hive_dma_rcv(dma_id)        OP_std_rcv(_hive_dma_fifo(dma_id))
/* ON(_hive_dma_sru(dma_id)) -> commented ,syntax error at token "__attribute__" to be checked */

#ifdef __HIVECC
#define _hive_dma_snd_if_crun(dma_id, is_var)
#else
#define _hive_dma_snd_if_crun(dma_id, is_var) _hive_dma_snd(dma_id, is_var)
#endif

#define _DMA_V2_PROP_SHIFT(val, param)       ((val) << _DMA_V2_ ## param ## _IDX)
#define _DMA_V2_PROP_MASK(param)             ((1U<<_DMA_V2_ ## param ## _BITS)-1)
#define _DMA_V2_PACK(val, param)             _DMA_V2_PROP_SHIFT((val) & _DMA_V2_PROP_MASK(param), param)

#define _DMA_V2_PACK_COMMAND(cmd)            _DMA_V2_PACK(cmd, CMD)
#define _DMA_V2_PACK_CHANNEL(ch)             _DMA_V2_PACK(ch,  CHANNEL)
#define _DMA_V2_PACK_PARAM(par)              _DMA_V2_PACK(par, PARAM)
#define _DMA_V2_PACK_EXTENSION(ext)          _DMA_V2_PACK(ext, EXTENSION)
#define _DMA_V2_PACK_LEFT_CROPPING(lc)       _DMA_V2_PACK(lc,  LEFT_CROPPING)
#define _DMA_V2_PACK_WIDTH_A(w)              _DMA_V2_PACK(w,   SPEC_DEV_A_XB)
#define _DMA_V2_PACK_WIDTH_B(w)              _DMA_V2_PACK(w,   SPEC_DEV_B_XB)
#define _DMA_V2_PACK_HEIGHT(h)               _DMA_V2_PACK(h,   SPEC_YB)

#define _DMA_V2_PACK_CHANNEL_CMD(cmd, ch)    (_DMA_V2_PACK_COMMAND(cmd) | _DMA_V2_PACK_CHANNEL(ch))
#define _DMA_V2_PACK_SETUP(conn, ext)        ((conn) | _DMA_V2_PACK_EXTENSION(ext))
#define _DMA_V2_PACK_CROP_ELEMS(elems, crop) ((elems) | _DMA_V2_PACK_LEFT_CROPPING(crop))

#define _DMA_V2_PACK_BLOCK_CMD(cmd, ch, width_a, width_b, height) \
  (_DMA_V2_PACK_COMMAND(cmd)     | \
   _DMA_V2_PACK_CHANNEL(ch)      | \
   _DMA_V2_PACK_WIDTH_A(width_a) | \
   _DMA_V2_PACK_WIDTH_B(width_b) | \
   _DMA_V2_PACK_HEIGHT(height))

#define dma_v2_zero_extension     _DMA_V2_ZERO_EXTEND
#define dma_v2_sign_extension     _DMA_V2_SIGN_EXTEND

#define dma_true 1
#define dma_false 0

#define hive_dma_v2_wait_for_ack(dma_id) _hive_dma_rcv(dma_id)

#define _hive_dma_v2_move_b2a_data(dma_id, cmd, channel, to_addr, from_addr, to_is_var, from_is_var) \
{ \
  _hive_dma_snd_if_crun(dma_id, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_CHANNEL_CMD(cmd, channel)); \
  _hive_dma_snd(dma_id, from_addr); \
  _hive_dma_snd(dma_id, to_addr); \
  _hive_dma_snd_if_crun(dma_id, to_is_var); \
  _hive_dma_snd_if_crun(dma_id, from_is_var); \
}

#define hive_dma_v2_move_b2a_data(dma_id, channel, to_addr, from_addr, to_is_var, from_is_var) \
  _hive_dma_v2_move_b2a_data(dma_id, _DMA_V2_MOVE_B2A_COMMAND, channel, to_addr, from_addr, to_is_var, from_is_var)

#define hive_dma_v2_move_b2a_data_no_sync_check(dma_id, channel, to_addr, from_addr, to_is_var, from_is_var) \
  _hive_dma_v2_move_b2a_data(dma_id, _DMA_V2_MOVE_B2A_NO_SYNC_CHK_COMMAND, channel, to_addr, from_addr, to_is_var, from_is_var)

#define hive_dma_v2_move_b2a_data_no_ack(dma_id, channel, to_addr, from_addr, to_is_var, from_is_var) \
  _hive_dma_v2_move_b2a_data(dma_id, _DMA_V2_NO_ACK_MOVE_B2A_NO_SYNC_CHK_COMMAND, channel, to_addr, from_addr, to_is_var, from_is_var)

#define _hive_dma_v2_move_b2a_block(dma_id, cmd, channel, to_addr, from_addr, to_width, from_width, height, to_is_var, from_is_var) \
{ \
  _hive_dma_snd_if_crun(dma_id, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_BLOCK_CMD(cmd, channel, to_width, from_width, height)); \
  _hive_dma_snd(dma_id, from_addr); \
  _hive_dma_snd(dma_id, to_addr); \
  _hive_dma_snd_if_crun(dma_id, to_is_var); \
  _hive_dma_snd_if_crun(dma_id, from_is_var); \
}

#define hive_dma_v2_move_b2a_block(dma_id, channel, to_addr, from_addr, to_width, from_width, height, to_is_var, from_is_var) \
  _hive_dma_v2_move_b2a_block(dma_id, _DMA_V2_MOVE_B2A_BLOCK_COMMAND, channel, to_addr, from_addr, to_width, from_width, height, to_is_var, from_is_var)

#define hive_dma_v2_move_b2a_block_no_sync_check(dma_id, channel, to_addr, from_addr, to_width, from_width, height, to_is_var, from_is_var) \
  _hive_dma_v2_move_b2a_block(dma_id, _DMA_V2_MOVE_B2A_BLOCK_NO_SYNC_CHK_COMMAND, channel, to_addr, from_addr, to_width, from_width, height, to_is_var, from_is_var)

#define hive_dma_v2_move_b2a_block_no_ack(dma_id, channel, to_addr, from_addr, to_width, from_width, height, to_is_var, from_is_var) \
  _hive_dma_v2_move_b2a_block(dma_id, _DMA_V2_NO_ACK_MOVE_B2A_BLOCK_NO_SYNC_CHK_COMMAND, channel, to_addr, from_addr, to_width, from_width, height, to_is_var, from_is_var)

#define _hive_dma_v2_move_a2b_data(dma_id, cmd, channel, from_addr, to_addr, from_is_var, to_is_var) \
{ \
  _hive_dma_snd_if_crun(dma_id, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_CHANNEL_CMD(cmd, channel)); \
  _hive_dma_snd(dma_id, from_addr); \
  _hive_dma_snd(dma_id, to_addr); \
  _hive_dma_snd_if_crun(dma_id, from_is_var); \
  _hive_dma_snd_if_crun(dma_id, to_is_var); \
}

#define hive_dma_v2_move_a2b_data(dma_id, channel, from_addr, to_addr, from_is_var, to_is_var) \
  _hive_dma_v2_move_a2b_data(dma_id, _DMA_V2_MOVE_A2B_COMMAND, channel, from_addr, to_addr, from_is_var, to_is_var)

#define hive_dma_v2_move_a2b_data_no_sync_check(dma_id, channel, from_addr, to_addr, from_is_var, to_is_var) \
  _hive_dma_v2_move_a2b_data(dma_id, _DMA_V2_MOVE_A2B_NO_SYNC_CHK_COMMAND, channel, from_addr, to_addr, from_is_var, to_is_var)

#define hive_dma_v2_move_a2b_data_no_ack(dma_id, channel, from_addr, to_addr, from_is_var, to_is_var) \
  _hive_dma_v2_move_a2b_data(dma_id, _DMA_V2_NO_ACK_MOVE_A2B_NO_SYNC_CHK_COMMAND, channel, from_addr, to_addr, from_is_var, to_is_var)

#define _hive_dma_v2_move_a2b_block(dma_id, cmd, channel, from_addr, to_addr, from_width, to_width, height, from_is_var, to_is_var) \
{ \
  _hive_dma_snd_if_crun(dma_id, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_BLOCK_CMD(cmd, channel, from_width, to_width, height)); \
  _hive_dma_snd(dma_id, from_addr); \
  _hive_dma_snd(dma_id, to_addr); \
  _hive_dma_snd_if_crun(dma_id, from_is_var); \
  _hive_dma_snd_if_crun(dma_id, to_is_var); \
}

#define hive_dma_v2_move_a2b_block(dma_id, channel, from_addr, to_addr, from_width, to_width, height, from_is_var, to_is_var) \
  _hive_dma_v2_move_a2b_block(dma_id, _DMA_V2_MOVE_A2B_BLOCK_COMMAND, channel, from_addr, to_addr, from_width, to_width, height, from_is_var, to_is_var)

#define hive_dma_v2_move_a2b_block_no_sync_check(dma_id, channel, from_addr, to_addr, from_width, to_width, height, from_is_var, to_is_var) \
  _hive_dma_v2_move_a2b_block(dma_id, _DMA_V2_MOVE_A2B_BLOCK_NO_SYNC_CHK_COMMAND, channel, from_addr, to_addr, from_width, to_width, height, from_is_var, to_is_var)

#define hive_dma_v2_move_a2b_block_no_ack(dma_id, channel, from_addr, to_addr, from_width, to_width, height, from_is_var, to_is_var) \
  _hive_dma_v2_move_a2b_block(dma_id, _DMA_V2_NO_ACK_MOVE_A2B_BLOCK_NO_SYNC_CHK_COMMAND, channel, from_addr, to_addr, from_width, to_width, height, from_is_var, to_is_var)

#define _hive_dma_v2_init_a_data(cmd, dma_id, channel, address, value, is_var) \
{ \
  _hive_dma_snd_if_crun(dma_id, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_CHANNEL_CMD(cmd, channel)); \
  _hive_dma_snd(dma_id, value); \
  _hive_dma_snd(dma_id, address); \
  _hive_dma_snd_if_crun(dma_id, is_var); \
}

#define hive_dma_v2_init_a_data(dma_id, channel, address, value, is_var) \
  _hive_dma_v2_init_a_data(_DMA_V2_INIT_A_COMMAND, dma_id, channel, address, value, is_var)

#define hive_dma_v2_init_a_data_no_sync_check(dma_id, channel, address, value, is_var) \
  _hive_dma_v2_init_a_data(_DMA_V2_INIT_A_NO_SYNC_CHK_COMMAND, dma_id, channel, address, value, is_var)

#define hive_dma_v2_init_a_data_no_ack(dma_id, channel, address, value, is_var) \
  _hive_dma_v2_init_a_data(_DMA_V2_NO_ACK_INIT_A_NO_SYNC_CHK_COMMAND, dma_id, channel, address, value, is_var)

#define _hive_dma_v2_init_a_block(cmd, dma_id, channel, address, value, width, height, is_var) \
{ \
  _hive_dma_snd_if_crun(dma_id, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_BLOCK_CMD(cmd, channel, width, 0, height)); \
  _hive_dma_snd(dma_id, value); \
  _hive_dma_snd(dma_id, address); \
  _hive_dma_snd_if_crun(dma_id, is_var); \
}

#define hive_dma_v2_init_a_block(dma_id, channel, address, value, width, height, is_var) \
  _hive_dma_v2_init_a_block(_DMA_V2_INIT_A_BLOCK_COMMAND, dma_id, channel, address, value, width, height, is_var)

#define hive_dma_v2_init_a_block_no_sync_check(dma_id, channel, address, value, width, height, is_var) \
  _hive_dma_v2_init_a_block(_DMA_V2_INIT_A_BLOCK_NO_SYNC_CHK_COMMAND, dma_id, channel, address, value, width, height, is_var)

#define hive_dma_v2_init_a_block_no_ack(dma_id, channel, address, value, width, height, is_var) \
  _hive_dma_v2_init_a_block(_DMA_V2_NO_ACK_INIT_A_BLOCK_NO_SYNC_CHK_COMMAND, dma_id, channel, address, value, width, height, is_var)

#define _hive_dma_v2_init_b_data(cmd, dma_id, channel, address, value, is_var) \
{ \
  _hive_dma_snd_if_crun(dma_id, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_CHANNEL_CMD(cmd, channel)); \
  _hive_dma_snd(dma_id, value); \
  _hive_dma_snd(dma_id, address); \
  _hive_dma_snd_if_crun(dma_id, is_var); \
}

#define hive_dma_v2_init_b_data(dma_id, channel, address, value, is_var) \
  _hive_dma_v2_init_b_data(_DMA_V2_INIT_B_COMMAND, dma_id, channel, address, value, is_var)

#define hive_dma_v2_init_b_data_no_sync_check(dma_id, channel, address, value, is_var) \
  _hive_dma_v2_init_b_data(_DMA_V2_INIT_B_NO_SYNC_CHK_COMMAND, dma_id, channel, address, value, is_var)

#define hive_dma_v2_init_b_data_no_ack(dma_id, channel, address, value, is_var) \
  _hive_dma_v2_init_b_data(_DMA_V2_NO_ACK_INIT_B_NO_SYNC_CHK_COMMAND, dma_id, channel, address, value, is_var)

#define _hive_dma_v2_init_b_block(cmd, dma_id, channel, address, value, width, height, is_var) \
{ \
  _hive_dma_snd_if_crun(dma_id, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_BLOCK_CMD(cmd , channel, 0, width, height)); \
  _hive_dma_snd(dma_id, value); \
  _hive_dma_snd(dma_id, address); \
  _hive_dma_snd_if_crun(dma_id, is_var); \
}
#define hive_dma_v2_init_b_block(dma_id, channel, address, value, width, height, is_var) \
  _hive_dma_v2_init_b_data(_DMA_V2_INIT_B_BLOCK_COMMAND, dma_id, channel, address, value, is_var)

#define hive_dma_v2_init_b_block_no_sync_check(dma_id, channel, address, value, width, height, is_var) \
  _hive_dma_v2_init_b_data(_DMA_V2_INIT_B_BLOCK_NO_SYNC_CHK_COMMAND, dma_id, channel, address, value, is_var)

#define hive_dma_v2_init_b_block_no_ack(dma_id, channel, address, value, width, height, is_var) \
  _hive_dma_v2_init_b_data(_DMA_V2_NO_ACK_INIT_B_BLOCK_NO_SYNC_CHK_COMMAND, dma_id, channel, address, value, is_var)

#define hive_dma_v2_configure_channel(dma_id, channel, connection, extension, height, \
                                 stride_A, elems_A, cropping_A, width_A, \
                                 stride_B, elems_B, cropping_B, width_B) \
{ \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_CHANNEL_CMD(_DMA_V2_CONFIG_CHANNEL_COMMAND, channel)); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_SETUP(connection, extension)); \
  _hive_dma_snd(dma_id, stride_A); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_CROP_ELEMS(elems_A, cropping_A)); \
  _hive_dma_snd(dma_id, width_A); \
  _hive_dma_snd(dma_id, stride_B); \
  _hive_dma_snd(dma_id, _DMA_V2_PACK_CROP_ELEMS(elems_B, cropping_B)); \
  _hive_dma_snd(dma_id, width_B); \
  _hive_dma_snd(dma_id, height); \
}

#define _dma_v2_set_channel_parameter(dma_id, channel, param, value) \
{ \
  _hive_dma_snd(dma_id, _DMA_V2_SET_CHANNEL_PARAM_COMMAND | _DMA_V2_PACK_CHANNEL(channel) | _DMA_V2_PACK_PARAM(param)); \
  _hive_dma_snd(dma_id, value); \
}

#define hive_dma_v2_set_channel_packing(dma_id, channel, connection, extension) \
  _dma_v2_set_channel_parameter(dma_id, channel, _DMA_V2_PACKING_SETUP_PARAM, _DMA_V2_PACK_SETUP(connection, extension))

#define hive_dma_v2_set_channel_stride_A(dma_id, channel, stride) \
  _dma_v2_set_channel_parameter(dma_id, channel, _DMA_V2_STRIDE_A_PARAM, stride)

#define hive_dma_v2_set_channel_elements_and_cropping_A(dma_id, channel, elements, cropping) \
  _dma_v2_set_channel_parameter(dma_id, channel, _DMA_V2_ELEM_CROPPING_A_PARAM, _DMA_V2_PACK_CROP_ELEMS(elements, cropping))
      
#define hive_dma_v2_set_channel_width_A(dma_id, channel, width) \
  _dma_v2_set_channel_parameter(dma_id, channel, _DMA_V2_WIDTH_A_PARAM, width)
      
#define hive_dma_v2_set_channel_stride_B(dma_id, channel, stride) \
  _dma_v2_set_channel_parameter(dma_id, channel, _DMA_V2_STRIDE_B_PARAM, stride)

#define hive_dma_v2_set_channel_elements_and_cropping_B(dma_id, channel, elements, cropping) \
  _dma_v2_set_channel_parameter(dma_id, channel, _DMA_V2_ELEM_CROPPING_B_PARAM, _DMA_V2_PACK_CROP_ELEMS(elements, cropping))

#define hive_dma_v2_set_channel_width_B(dma_id, channel, width) \
  _dma_v2_set_channel_parameter(dma_id, channel, _DMA_V2_WIDTH_B_PARAM, width)
      
#define hive_dma_v2_set_channel_height(dma_id, channel, height) \
  _dma_v2_set_channel_parameter(dma_id, channel, _DMA_V2_HEIGHT_PARAM, height)
      
#define hive_dma_v2_reset(dma_id) \
{ \
  _hive_dma_snd(dma_id, _DMA_V2_RESET_COMMAND); \
}

#endif /* _hss_dma_v2_api_h */
