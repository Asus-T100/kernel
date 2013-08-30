/* Release Version: ci_master_byt_20130820_2200 */
/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2013 Intel Corporation. All Rights Reserved.
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

#ifndef __IA_CSS_I_RMGR_GEN_HOST_SHARED_H_INCLUDED__
#define __IA_CSS_I_RMGR_GEN_HOST_SHARED_H_INCLUDED__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif
#include <ia_css.h>
#include <sh_css_debug.h>


#define IA_CSS_RESOURCE_LIST_MAX_AMOUNT 10 
#define IA_CSS_RESOURCE_RESERVED  1
#define IA_CSS_RESOURCE_AVAILABLE 0


typedef enum ia_css_resource_type
{
    IA_CSS_RESOURCE_START  = 1230,     // start at random number for code robustness, 
    IA_CSS_RESOURCE_SP_THREAD,        // register the threads of the sp
    IA_CSS_RESOURCE_HOST2SP_QUEUE,    // register the host to sp queues
    IA_CSS_RESOURCE_END               
}ia_css_resource_type_t;

typedef struct ia_css_resource_list_item
{
    uint32_t id;
    uint16_t reserved;
}ia_css_resource_list_item_t;

typedef struct ia_css_resource_list
{
    ia_css_resource_list_item_t list[IA_CSS_RESOURCE_LIST_MAX_AMOUNT];
    uint16_t size;
}ia_css_resource_list_t;
void test_resource_manager(void);


enum ia_css_err ia_css_i_host_rmgr_init_gen(ia_css_resource_type_t resource_type);
enum ia_css_err ia_css_i_host_rmgr_setup_gen(ia_css_resource_type_t resource_type, uint32_t size);
enum ia_css_err ia_css_i_host_rmgr_acq_gen(ia_css_resource_type_t resource_type, uint32_t* resource_id);
enum ia_css_err ia_css_i_host_rmgr_rel_gen(ia_css_resource_type_t resource_type, uint32_t size);



#endif /* __IA_CSS_I_RMGR_VBUF_HOST_SHARED_H_INCLUDED__ */
