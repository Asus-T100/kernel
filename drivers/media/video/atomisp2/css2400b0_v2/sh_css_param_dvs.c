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

#include "assert_support.h"
#include "ia_css.h"
#include "sh_css_param_dvs.h"
#include "sh_css_debug.h"
#ifdef __KERNEL__
#include <linux/string.h>		/* memcpy() */
#else
#include <string.h>		/* memcpy() */
#endif


struct ia_css_dvs_6axis_config *
generate_dvs_6axis_table(const struct ia_css_resolution	*frame_res, const struct ia_css_resolution *dvs_offset)
{
	
	unsigned int x,y;
	unsigned int width_y;
	unsigned int height_y;
	unsigned int width_uv;
	unsigned int height_uv;
	enum ia_css_err err = IA_CSS_SUCCESS;	
	struct ia_css_dvs_6axis_config  *dvs_config = NULL;

	assert(frame_res != NULL);
	assert(dvs_offset != NULL);
	
	dvs_config = (struct ia_css_dvs_6axis_config *)sh_css_malloc(sizeof(struct ia_css_dvs_6axis_config));
	if(dvs_config == NULL)
	{
		sh_css_dtrace(SH_DBG_TRACE, "out of memory\n");
		err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
	}
	else
	{	/*Initialize new struct with latest config settings*/	
		dvs_config->width_y = width_y = DVS_TABLE_IN_BLOCKDIM_X_LUMA(frame_res->width);
		dvs_config->height_y = height_y = DVS_TABLE_IN_BLOCKDIM_Y_LUMA(frame_res->height);
		dvs_config->width_uv = width_uv = DVS_TABLE_IN_BLOCKDIM_X_CHROMA(frame_res->width / 2); /* UV = Y/2, depens on colour format YUV 4.2.0*/
		dvs_config->height_uv = height_uv = DVS_TABLE_IN_BLOCKDIM_Y_CHROMA(frame_res->height / 2);/* UV = Y/2, depens on colour format YUV 4.2.0*/

		sh_css_dtrace(SH_DBG_TRACE, "generate_dvs_6axis_table: Env_X %d Env_Y %d\n",dvs_offset->width,dvs_offset->height);
		sh_css_dtrace(SH_DBG_TRACE, "generate_dvs_6axis_table Y: W %d H %d\n",width_y,height_y);
		/* Generate Y buffers  */
		dvs_config->xcoords_y = (uint32_t *)sh_css_malloc( width_y * height_y * sizeof(uint32_t));	
		if(dvs_config->xcoords_y == NULL)
		{
			sh_css_dtrace(SH_DBG_TRACE, "out of memory\n");
			err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		}
		else
		{
			for(y = 0; y < height_y; y++)
			{
				for(x=0;x<width_y;x++)
				{
					dvs_config->xcoords_y[y*width_y + x] =  (  ( dvs_offset->width + x*DVS_BLOCKDIM_X) << DVS_COORD_FRAC_BITS );
				}
			}
		}

		dvs_config->ycoords_y = (uint32_t *)sh_css_malloc( width_y * height_y * sizeof(uint32_t));
		if(dvs_config->ycoords_y == NULL)
		{
			sh_css_dtrace(SH_DBG_TRACE, "out of memory\n");
			err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		}
		else
		{			
			for(y = 0; y < height_y; y++)
			{
				for(x=0;x<width_y;x++)
				{
					dvs_config->ycoords_y[y*width_y + x] =  (  ( dvs_offset->height + y*DVS_BLOCKDIM_Y_LUMA) << DVS_COORD_FRAC_BITS );
				}
			}
		}
		
		/* Generate UV buffers  */		
		sh_css_dtrace(SH_DBG_TRACE, "generate_dvs_6axis_table UV W %d H %d\n",width_uv,height_uv);
		
		dvs_config->xcoords_uv = (uint32_t *)sh_css_malloc( width_uv * height_uv * sizeof(uint32_t));	
		if(dvs_config->xcoords_uv == NULL)
		{
			sh_css_dtrace(SH_DBG_TRACE, "out of memory\n");
			err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		}
		else
		{			
			for(y = 0; y < height_uv; y++)
			{
				for(x=0;x<width_uv;x++)
				{	/* Envelope dimesions set in Ypixels hence offset UV = offset Y/2 */
					dvs_config->xcoords_uv[y*width_uv + x] =  (  ( (dvs_offset->width / 2) + x*DVS_BLOCKDIM_X) << DVS_COORD_FRAC_BITS ); 
				}
			}
		}
		    
		dvs_config->ycoords_uv = (uint32_t *)sh_css_malloc( width_uv * height_uv * sizeof(uint32_t));
		if(dvs_config->ycoords_uv == NULL)
		{
			sh_css_dtrace(SH_DBG_TRACE, "out of memory\n");
			err = IA_CSS_ERR_CANNOT_ALLOCATE_MEMORY;
		}
		else
		{	
			for(y = 0; y < height_uv; y++)
			{
				for(x=0;x<width_uv;x++)
				{ 	/* Envelope dimesions set in Ypixels hence offset UV = offset Y/2 */
					dvs_config->ycoords_uv[y*width_uv + x] =  (  ( (dvs_offset->height / 2) + y*DVS_BLOCKDIM_Y_CHROMA) << DVS_COORD_FRAC_BITS );
				}
			}
		}
		
#if 0
		for(y = 0; y < height_y; y++)
		{
			for(x=0;x<width_y;x++)
				sh_css_dtrace(SH_DBG_TRACE, "xcoords_y: %d \n",dvs_config->xcoords_y[y*width_y + x]); 
			sh_css_dtrace(SH_DBG_TRACE, "\n");
		}
		
		for(y = 0; y < height_y; y++)
		{
			for(x=0;x<width_y;x++)
				sh_css_dtrace(SH_DBG_TRACE, "ycoords_y: %d \n",dvs_config->ycoords_y[y*width_y + x]); 
			sh_css_dtrace(SH_DBG_TRACE, "\n");
		}	
		
				for(y = 0; y < height_y; y++)
		{
			for(x=0;x<width_uv;x++)
				sh_css_dtrace(SH_DBG_TRACE, "xcoords_uv: %d \n",dvs_config->xcoords_uv[y*width_uv + x]); 
			sh_css_dtrace(SH_DBG_TRACE, "\n");
		}
		
		for(y = 0; y < height_uv; y++)
		{
			for(x=0;x<width_uv;x++)
				sh_css_dtrace(SH_DBG_TRACE, "ycoords_uv: %d \n",dvs_config->ycoords_uv[y*width_uv + x]); 
			sh_css_dtrace(SH_DBG_TRACE, "\n");
		}				
#endif		    
		if (err != IA_CSS_SUCCESS)
		{
			sh_css_dtrace(SH_DBG_TRACE, "generate_dvs_6axis_table: err %d\n, leave() ",(int)err);	
			free_dvs_6axis_table(&dvs_config); /* we might have allocated some memory, release this */
		}
		else
		{
			sh_css_dtrace(SH_DBG_TRACE, "generate_dvs_6axis_table leave() , dvs_config %p\n",dvs_config);
		}
	}
	
	return dvs_config;
}

void
free_dvs_6axis_table(struct ia_css_dvs_6axis_config  **dvs_6axis_config)
{
	assert(dvs_6axis_config != NULL);
	assert(*dvs_6axis_config != NULL);

	if( (dvs_6axis_config != NULL) && (*dvs_6axis_config != NULL) ) 
	{
		sh_css_dtrace(SH_DBG_TRACE, "-> free_dvs_6axis_table dvs_6axis_config %p\n",(*dvs_6axis_config));
		if((*dvs_6axis_config)->xcoords_y != NULL)
		{
			 sh_css_free((*dvs_6axis_config)->xcoords_y);
			 (*dvs_6axis_config)->xcoords_y = NULL;
		}
		
		if((*dvs_6axis_config)->ycoords_y != NULL)
		{
			sh_css_free((*dvs_6axis_config)->ycoords_y);
			(*dvs_6axis_config)->ycoords_y = NULL;
		}	
		
		/* Free up UV buffers */
		if((*dvs_6axis_config)->xcoords_uv != NULL)
		{
			sh_css_free((*dvs_6axis_config)->xcoords_uv);
			(*dvs_6axis_config)->xcoords_uv = NULL;
		}
		
		if((*dvs_6axis_config)->ycoords_uv != NULL)
		{
			sh_css_free((*dvs_6axis_config)->ycoords_uv);
			(*dvs_6axis_config)->ycoords_uv = NULL;
		}
		
		sh_css_free(*dvs_6axis_config);
		*dvs_6axis_config = NULL;
	}
	sh_css_dtrace(SH_DBG_TRACE, "<- free_dvs_6axis_table dvs_6axis_config %p\n",(*dvs_6axis_config));
}

void copy_dvs_6axis_table(struct ia_css_dvs_6axis_config *dvs_config_dst,
			const struct ia_css_dvs_6axis_config *dvs_config_src)
{	
	unsigned int width_y;
	unsigned int height_y;
	unsigned int width_uv;
	unsigned int height_uv;

	assert(dvs_config_dst != NULL);
	assert(dvs_config_src!= NULL);

	width_y = dvs_config_src->width_y;
	height_y =  dvs_config_src->height_y;
	width_uv = dvs_config_src->width_uv; /* = Y/2, depens on colour format YUV 4.2.0*/
	height_uv = dvs_config_src->height_uv;
		
	memcpy(dvs_config_dst->xcoords_y,dvs_config_src->xcoords_y, (width_y * height_y * sizeof(uint32_t)));
	memcpy(dvs_config_dst->ycoords_y,dvs_config_src->ycoords_y, (width_y * height_y * sizeof(uint32_t)));

	memcpy(dvs_config_dst->xcoords_uv,dvs_config_src->xcoords_uv, (width_uv * height_uv * sizeof(uint32_t)));
	memcpy(dvs_config_dst->ycoords_uv,dvs_config_src->ycoords_uv, (width_uv * height_uv * sizeof(uint32_t)));

}
