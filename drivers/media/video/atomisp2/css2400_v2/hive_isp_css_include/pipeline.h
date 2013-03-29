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
