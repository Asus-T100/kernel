/* Release Version: ci_master_byt_20130905_2200 */
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

#ifndef _SH_CSS_DEBUG_INTERNAL_H_
#define _SH_CSS_DEBUG_INTERNAL_H_

/*! \file */

#include "ia_css.h"
#include "sh_css_internal.h"

/**
 * @brief Internal debug support for constructing a pipe graph.
 *
 * @return	None
 */
extern void sh_css_debug_pipe_graph_dump_prologue(void);

/**
 * @brief Internal debug support for constructing a pipe graph.
 *
 * @return	None
 */
extern void sh_css_debug_pipe_graph_dump_epilogue(void);

/**
 * @brief Internal debug support for constructing a pipe graph.
 * @param[in]	stage		Pipeline stage.
 * @param[in]	id		Pipe id.
 *
 * @return	None
 */
extern void sh_css_debug_pipe_graph_dump_stage(
		struct sh_css_pipeline_stage *stage,
		enum ia_css_pipe_id id);

/**
 * @brief Internal debug support for constructing a pipe graph.
 * @param[in]	cc_frame	Output frame of SP raw copy.
 *
 * @return	None
 */
extern void sh_css_debug_pipe_graph_dump_sp_raw_copy(
		struct ia_css_frame *cc_frame);



#endif /* _SH_CSS_DEBUG_INTERNAL_H_ */
