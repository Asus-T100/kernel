/*!****************************************************************************
@File			resman.h

@Title			Resource Manager API

@Author			Imagination Technologies

@date   		27/11/03
 
@Copyright     	Copyright 2003-2005 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either
                material or conceptual may be copied or distributed,
                transmitted, transcribed, stored in a retrieval system
                or translated into any human or computer language in any
                form by any means, electronic, mechanical, manual or
                other-wise, or disclosed to third parties without the
                express written permission of Imagination Technologies
                Limited, Unit 8, HomePark Industrial Estate,
                King's Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform		generic

@Description	Provide resource management

@DoxygenVer		

******************************************************************************/

#ifndef __RESMAN_H__
#define __RESMAN_H__

#if defined (__cplusplus)
extern "C" {
#endif

#include "servicesext.h"

/******************************************************************************
 * resman definitions 
 *****************************************************************************/

	enum {
		/* SGX: */
		RESMAN_TYPE_SHARED_PB_DESC = 1,	/*!< Parameter buffer kernel stubs */
		RESMAN_TYPE_SHARED_PB_DESC_CREATE_LOCK,	/*!< Shared parameter buffer creation lock */

		/* MSVDX: */
		/* FIXME: TBD */

		/* DISPLAY CLASS: */
		RESMAN_TYPE_DISPLAYCLASS_SWAPCHAIN_REF,	/*!< Display Class Swapchain Reference Resource */
		RESMAN_TYPE_DISPLAYCLASS_DEVICE,	/*!< Display Class Device Resource */

		/* BUFFER CLASS: */
		RESMAN_TYPE_BUFFERCLASS_DEVICE,	/*!< Buffer Class Device Resource */

		/* OS specific User mode Mappings: */
		RESMAN_TYPE_OS_USERMODE_MAPPING,	/*!< OS specific User mode mappings */

		/* COMMON: */
		RESMAN_TYPE_DC_DEVICE,
		RESMAN_TYPE_DC_DISPLAY_CONTEXT,
		RESMAN_TYPE_DC_PIN_HANDLE,
		RESMAN_TYPE_DC_BUFFER,
		RESMAN_TYPE_DEVMEM_MEM_EXPORT,
		RESMAN_TYPE_PMR,
		RESMAN_TYPE_PMR_EXPORT,
		RESMAN_TYPE_PMR_PAGELIST,	/*!< Device Memory page list Resource */
		RESMAN_TYPE_SERVER_EXPORTCOOKIE,
		RESMAN_TYPE_DEVICEMEM2_CONTEXT,	/*!< Device Memory Context Resource */
		RESMAN_TYPE_DEVICEMEM2_HEAP,	/*!< Device Memory Heap Resource */
		RESMAN_TYPE_DEVICEMEM2_RESERVATION,	/*!< Device Memory Reservation Resource */
		RESMAN_TYPE_DEVICEMEM2_MAPPING,	/*!< Device Memory Mapping Resource */
		RESMAN_TYPE_DEVICEMEM_CONTEXT,	/*!< Device Memory Context Resource */
		RESMAN_TYPE_DEVICECLASSMEM_MAPPING,	/*!< Device Memory Mapping Resource */
		RESMAN_TYPE_DEVICEMEM_MAPPING,	/*!< Device Memory Mapping Resource */
		RESMAN_TYPE_DEVICEMEM_WRAP,	/*!< Device Memory Wrap Resource */
		RESMAN_TYPE_DEVICEMEM_ALLOCATION,	/*!< Device Memory Allocation Resource */
		RESMAN_TYPE_EVENT_OBJECT,	/*!< Event Object */
		RESMAN_TYPE_SHARED_MEM_INFO,	/*!< Shared system memory meminfo */
		RESMAN_TYPE_MODIFY_SYNC_OPS,	/*!< Syncobject synchronisation Resource */
		RESMAN_TYPE_SYNC_INFO,	/*!< Syncobject Resource */
		PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA,	/*!< Private Data Resource */
		RESMAN_TYPE_SYNC_PRIMITIVE,	/*!< Sync primitive resource */
		RESMAN_TYPE_SYNC_PRIMITIVE_BLOCK,	/*!< Sync primitive block resource */
		RESMAN_TYPE_SERVER_SYNC_PRIMITIVE,	/*!< Server sync primitive resource */
		RESMAN_TYPE_SERVER_SYNC_EXPORT,	/*!< Server sync export resource */
		RESMAN_TYPE_SERVER_OP_COOKIE,	/*!< Server  operation cookie resource */

		/* RGX: */
		RESMAN_TYPE_RGX_RENDER_CONTEXT,	/*!< RGX Render Context Resource */
		RESMAN_TYPE_RGX_TQ3D_CONTEXT,	/*!< RGX Transfer Queue 3D Context Resource */
		RESMAN_TYPE_RGX_TQ2D_CONTEXT,	/*!< RGX Transfer Queue 2D Context Resource */
		RESMAN_TYPE_RGX_COMPUTE_CONTEXT,	/*!< RGX Compute Context Resource */
		RESMAN_TYPE_RGX_CCB,	/*!< RGX Circular Command Buffer Resource */
		RESMAN_TYPE_RGX_FWIF_HWRTDATA,	/*! < FW HWRTDATA structure */
		RESMAN_TYPE_RGX_FWIF_RENDERTARGET,	/*! < FW RENDER_TARGET structure */
		RESMAN_TYPE_RGX_FWIF_FREELIST,	/*! < FW FREELIST structure */
		/* KERNEL: */
		RESMAN_TYPE_KERNEL_DEVICEMEM_ALLOCATION	/*!< Device Memory Allocation Resource */
	};

#define RESMAN_CRITERIA_ALL				0x00000000	/*!< match by criteria all */
#define RESMAN_CRITERIA_RESTYPE			0x00000001	/*!< match by criteria type */
#define RESMAN_CRITERIA_PVOID_PARAM		0x00000002	/*!< match by criteria param1 */
#define RESMAN_CRITERIA_UI32_PARAM		0x00000004	/*!< match by criteria param2 */

	typedef PVRSRV_ERROR(*RESMAN_FREE_FN) (IMG_PVOID pvParam,
					       IMG_UINT32 ui32Param);

	typedef struct _RESMAN_ITEM_ *PRESMAN_ITEM;
	typedef struct _RESMAN_CONTEXT_ *PRESMAN_CONTEXT;

/******************************************************************************
 * resman functions 
 *****************************************************************************/

	PVRSRV_ERROR ResManInit(IMG_VOID);
	IMG_VOID ResManDeInit(IMG_VOID);

	PRESMAN_ITEM ResManRegisterRes(PRESMAN_CONTEXT hResManContext,
				       IMG_UINT32 ui32ResType,
				       IMG_PVOID pvParam,
				       IMG_UINT32 ui32Param,
				       RESMAN_FREE_FN pfnFreeResource);

	PVRSRV_ERROR ResManFreeResByPtr(PRESMAN_ITEM psResItem);

/*!
******************************************************************************
 @Function	 	ResManFindPrivateDataByPtr

 @Description   finds the private date for a resource by matching on pointer type

 @inputs        psResItem - pointer to resource item

 @Return   		PVRSRV_ERROR
**************************************************************************/
	extern PVRSRV_ERROR
	    ResManFindPrivateDataByPtr(PRESMAN_ITEM psResItem,
				       IMG_PVOID * ppvParam1,
				       IMG_UINT32 * pui32Param2);

	PVRSRV_ERROR ResManFreeResByCriteria(PRESMAN_CONTEXT hResManContext,
					     IMG_UINT32 ui32SearchCriteria,
					     IMG_UINT32 ui32ResType,
					     IMG_PVOID pvParam,
					     IMG_UINT32 ui32Param);

	PVRSRV_ERROR ResManDissociateRes(PRESMAN_ITEM psResItem,
					 PRESMAN_CONTEXT psNewResManContext);

	PVRSRV_ERROR ResManFindResourceByPtr(PRESMAN_CONTEXT hResManContext,
					     PRESMAN_ITEM psItem);

	PVRSRV_ERROR PVRSRVResManConnect(PRESMAN_CONTEXT * phResManContext);
	IMG_VOID PVRSRVResManDisconnect(PRESMAN_CONTEXT hResManContext,
					IMG_BOOL bKernelContext);

#if defined (__cplusplus)
}
#endif
#endif				/* __RESMAN_H__ */
/******************************************************************************
 End of file (resman.h)
******************************************************************************/
