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

#ifndef __PIPELINE_H_INCLUDED__
#define __PIPELINE_H_INCLUDED__

/*
 * This file is included on every cell {SP,ISP,host} and on every system
 * that uses the pipeline.
 *
 * Problematic is the definition of the pipeline. For the moment it refers
 * to the algorithmic pipeline (i.e. a DAG of kernels) as the user would see
 * it. The pipeline thus aggregates all parameter sets of all kernels
 * irrespective of the partitioning of the pipeline over stages or sections
 *
 * System and cell specific interfaces and inline code are included
 * conditionally through Makefile path settings.
 *
 *  - .        system and cell agnostic interfaces, constants and identifiers
 *	- public:  system agnostic, cell specific interfaces
 *	- private: system dependent, cell specific interfaces & inline implementations
 *	- global:  system specific constants and identifiers
 *	- local:   system and cell specific constants and identifiers
 */

/* #define __PARAM_BY_ADDRESS__ 1 */
#define __PARAM_BY_TEMPLATE__ 1

#include "storage_class.h"

#include "pipeline_local.h"

#ifndef __INLINE_PIPELINE__
#define STORAGE_CLASS_PIPELINE_H STORAGE_CLASS_EXTERN
#define STORAGE_CLASS_PIPELINE_C 
#include "pipeline_public.h"
#else  /* __INLINE_PIPELINE__ */
#define STORAGE_CLASS_PIPELINE_H STORAGE_CLASS_INLINE
#define STORAGE_CLASS_PIPELINE_C STORAGE_CLASS_INLINE
#include "pipeline_private.h"
#endif /* __INLINE_PIPELINE__ */

#endif /* __PIPELINE_H_INCLUDED__ */
