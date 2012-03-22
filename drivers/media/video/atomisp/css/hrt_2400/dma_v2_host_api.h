#ifndef _dma_v2_hots_api_h
#define _dma_v2_hots_api_h

#include <dma_v2_defs.h>
#include <dma_v2.h>
#include <dma_v2_api.h>

#define sf_cmd0 sf_hostcmd0_2_dma_in
#define sf_cmd1 sf_hostcmd1_2_dma_in


#ifndef C_RUN 
#define _hrt_fifo_snd_if_crun(sf_cmd, is_var) 
#else
#define _hrt_fifo_snd_if_crun(sf_cmd, is_var) hrt_fifo_snd(sf_cmd, is_var)
#endif

#define _hrt_dma_v2_host_api_wait_for_ack(sf_ack) \
{ \
  hrt_fifo_rcv(sf_ack); \
} \

#define _hrt_dma_v2_host_api_reset_from_stream(sf_cmd) \
{ \
  hrt_fifo_snd(sf_cmd, _DMA_V2_RESET_COMMAND); \
} \

#define _hrt_dma_v2_host_api_reset_from_slave(dma) \
{ \
  hrt_dma_v2_slave_reset(dma); \
} \

#define _hrt_dma_v2_host_api_configure_channel(sf_cmd, \
                                          channel, connection, extension, \
                                          height, stride_A, elems_A, \
                                          cropping_A, width_A, stride_B,  \
                                          elems_B, cropping_B, width_B) \
{ \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_CHANNEL_CMD(_DMA_V2_CONFIG_CHANNEL_COMMAND, channel)); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_SETUP(connection, extension)); \
  hrt_fifo_snd(sf_cmd, stride_A); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_CROP_ELEMS(elems_A, cropping_A)); \
  hrt_fifo_snd(sf_cmd, width_A); \
  hrt_fifo_snd(sf_cmd, stride_B); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_CROP_ELEMS(elems_B, cropping_B)); \
  hrt_fifo_snd(sf_cmd, width_B); \
  hrt_fifo_snd(sf_cmd, height); \
}

#define dma_true  1
#define dma_false 0

#define _hrt_dma_v2_host_api_move_b2a_data(sf_cmd, cmd, channel, to_addr, from_addr) \
{ \
  _hrt_fifo_snd_if_crun(sf_cmd, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_CHANNEL_CMD(cmd, channel)); \
  hrt_fifo_snd(sf_cmd, from_addr); \
  hrt_fifo_snd(sf_cmd, to_addr); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
}

#define _hrt_dma_v2_host_api_move_b2a_data_block(sf_cmd, cmd, channel, to_addr, from_addr, to_width, from_width, height) \
{ \
  _hrt_fifo_snd_if_crun(sf_cmd, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_BLOCK_CMD(cmd, channel, to_width, from_width, height)); \
  hrt_fifo_snd(sf_cmd, from_addr); \
  hrt_fifo_snd(sf_cmd, to_addr); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
}

#define _hrt_dma_v2_host_api_move_a2b_data(sf_cmd, cmd, channel, from_addr, to_addr) \
{ \
  _hrt_fifo_snd_if_crun(sf_cmd, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_CHANNEL_CMD(cmd, channel)); \
  hrt_fifo_snd(sf_cmd, from_addr); \
  hrt_fifo_snd(sf_cmd, to_addr); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
}

#define _hrt_dma_v2_host_api_move_a2b_data_block(sf_cmd, cmd, channel, from_addr, to_addr, from_width, to_width, height) \
{ \
  _hrt_fifo_snd_if_crun(sf_cmd, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_BLOCK_CMD(cmd, channel, from_width, to_width, height)); \
  hrt_fifo_snd(sf_cmd, from_addr); \
  hrt_fifo_snd(sf_cmd, to_addr); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
}

#define _hrt_dma_v2_host_api_init_data(sf_cmd, cmd, channel, address, value) \
{ \
  _hrt_fifo_snd_if_crun(sf_cmd, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_CHANNEL_CMD(cmd, channel)); \
  hrt_fifo_snd(sf_cmd, value); \
  hrt_fifo_snd(sf_cmd, address); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
}

#define _hrt_dma_v2_host_api_init_a_data_block(sf_cmd, cmd, channel, addr, value, width, height) \
{ \
  _hrt_fifo_snd_if_crun(sf_cmd, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_BLOCK_CMD(cmd, channel, width, 0, height)); \
  hrt_fifo_snd(sf_cmd, value); \
  hrt_fifo_snd(sf_cmd, address); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
}

#define _hrt_dma_v2_host_api_init_b_data_block(sf_cmd, cmd, channel, addr, value, width, height) \
{ \
  _hrt_fifo_snd_if_crun(sf_cmd, _DMA_V2_PACK_COMMAND(_DMA_V2_SET_CRUN_COMMAND)); \
  hrt_fifo_snd(sf_cmd, _DMA_V2_PACK_BLOCK_CMD(cmd, channel, 0, width, height)); \
  hrt_fifo_snd(sf_cmd, value); \
  hrt_fifo_snd(sf_cmd, address); \
  _hrt_fifo_snd_if_crun(sf_cmd, dma_false); \
}

#define _dma_v2_host_api_set_channel_parameter(sf_cmd, channel, param, value) \
{ \
  hrt_fifo_snd(sf_cmd, _DMA_V2_SET_CHANNEL_PARAM_COMMAND | _DMA_V2_PACK_CHANNEL(channel) | _DMA_V2_PACK_PARAM(param)); \
  hrt_fifo_snd(sf_cmd, value); \
}

#define _hrt_dma_v2_host_api_set_channel_packing(sf_cmd, channel, connection, extension) \
  _dma_v2_host_api_set_channel_parameter(sf_cmd, channel, _DMA_V2_PACKING_SETUP_PARAM, _DMA_V2_PACK_SETUP(connection, extension))

#define _hrt_dma_v2_host_api_set_channel_stride_A(sf_cmd, channel, stride) \
  _dma_v2_host_api_set_channel_parameter(sf_cmd, channel, _DMA_V2_STRIDE_A_PARAM, stride)

#define _hrt_dma_v2_host_api_set_channel_elements_and_cropping_A(sf_cmd, channel, elements, cropping) \
  _dma_v2_host_api_set_channel_parameter(sf_cmd, channel, _DMA_V2_ELEM_CROPPING_A_PARAM, _DMA_V2_PACK_CROP_ELEMS(elements, cropping))
      
#define _hrt_dma_v2_host_api_set_channel_width_A(sf_cmd, channel, width) \
  _dma_v2_host_api_set_channel_parameter(sf_cmd, channel, _DMA_V2_WIDTH_A_PARAM, width)
      
#define _hrt_dma_v2_host_api_set_channel_stride_B(sf_cmd, channel, stride) \
  _dma_v2_host_api_set_channel_parameter(sf_cmd, channel, _DMA_V2_STRIDE_B_PARAM, stride)

#define _hrt_dma_v2_host_api_set_channel_elements_and_cropping_B(sf_cmd, channel, elements, cropping) \
  _dma_v2_host_api_set_channel_parameter(sf_cmd, channel, _DMA_V2_ELEM_CROPPING_B_PARAM, _DMA_V2_PACK_CROP_ELEMS(elements, cropping))

#define _hrt_dma_v2_host_api_set_channel_width_B(sf_cmd, channel, width) \
  _dma_v2_host_api_set_channel_parameter(sf_cmd, channel, _DMA_V2_WIDTH_B_PARAM, width)
      
#define _hrt_dma_v2_host_api_set_channel_height(sf_cmd, channel, height) \
  _dma_v2_host_api_set_channel_parameter(sf_cmd, channel, _DMA_V2_HEIGHT_PARAM, height)


#endif /* _dma_v2_hots_api_h */
