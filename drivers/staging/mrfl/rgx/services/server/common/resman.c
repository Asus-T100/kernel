									    /*************************************************************************//*!
									       @File
									       @Title          Resource Manager
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description     Provide resource management
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "resman.h"
#include "allocmem.h"
#include "pvr_debug.h"

/* FIXME: use mutex here if required */
#define ACQUIRE_SYNC_OBJ
#define RELEASE_SYNC_OBJ

#define RESMAN_SIGNATURE 0x12345678

/******************************************************************************
 * resman structures
 *****************************************************************************/

/* resman item structure */
typedef struct _RESMAN_ITEM_ {
	IMG_UINT32 ui32Signature;

	struct _RESMAN_ITEM_ **ppsThis;	/*!< list navigation */
	struct _RESMAN_ITEM_ *psNext;	/*!< list navigation */

	IMG_UINT32 ui32Flags;	/*!< flags */
	IMG_UINT32 ui32ResType;	/*!< res type */

	IMG_PVOID pvParam;	/*!< param1 for callback */
	IMG_UINT32 ui32Param;	/*!< param2 for callback */

	RESMAN_FREE_FN pfnFreeResource;	/*!< resman item free callback */
} RESMAN_ITEM;

/* resman context structure */
typedef struct _RESMAN_CONTEXT_ {

	IMG_UINT32 ui32Signature;

	struct _RESMAN_CONTEXT_ **ppsThis;	/*!< list navigation */
	struct _RESMAN_CONTEXT_ *psNext;	/*!< list navigation */

	RESMAN_ITEM *psResItemList;	/*!< res item list for context */

} RESMAN_CONTEXT;

/* resman list structure */
typedef struct {
	RESMAN_CONTEXT *psContextList;	/*!< resman context list */

} RESMAN_LIST, *PRESMAN_LIST;	/* PRQA S 3205 */

PRESMAN_LIST gpsResList = IMG_NULL;

#include "lists.h"	/* PRQA S 5087 */	/* include lists.h required here */

static IMPLEMENT_LIST_ANY_VA(RESMAN_ITEM)
static IMPLEMENT_LIST_ANY_VA_2(RESMAN_ITEM, IMG_BOOL, IMG_FALSE)
static IMPLEMENT_LIST_INSERT(RESMAN_ITEM)
static IMPLEMENT_LIST_REMOVE(RESMAN_ITEM)
static IMPLEMENT_LIST_REVERSE(RESMAN_ITEM)

static IMPLEMENT_LIST_REMOVE(RESMAN_CONTEXT)
static IMPLEMENT_LIST_INSERT(RESMAN_CONTEXT)

/******************************************************** Forword references */
 static PVRSRV_ERROR FreeResourceByPtr(RESMAN_ITEM * psItem,
				       IMG_BOOL bExecuteCallback);

static PVRSRV_ERROR FreeResourceByCriteria(PRESMAN_CONTEXT psContext,
					   IMG_UINT32 ui32SearchCriteria,
					   IMG_UINT32 ui32ResType,
					   IMG_PVOID pvParam,
					   IMG_UINT32 ui32Param,
					   IMG_BOOL bExecuteCallback);

#if defined(DEBUG)
static IMG_VOID ValidateResList(PRESMAN_LIST psResList);
#define VALIDATERESLIST() ValidateResList(gpsResList)
#else
#define VALIDATERESLIST()
#endif

/*!
******************************************************************************

 @Function	ResManInit

 @Description initialises the resman

 @Return   none

******************************************************************************/
PVRSRV_ERROR ResManInit(IMG_VOID)
{
	if (gpsResList == IMG_NULL) {
		/* If not already initialised */
		gpsResList = OSAllocMem(sizeof(*gpsResList));
		if (gpsResList == IMG_NULL) {
			return PVRSRV_ERROR_OUT_OF_MEMORY;
		}

		/* Init list, the linked list has dummy entries at both ends */
		gpsResList->psContextList = IMG_NULL;

		/* Check resource list */
		VALIDATERESLIST();
	}

	return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	ResManDeInit

 @Description de-initialises the resman

 @Return   none

******************************************************************************/
IMG_VOID ResManDeInit(IMG_VOID)
{
	if (gpsResList != IMG_NULL) {
		/* FIXME: ensure all contexts have been freed. */
		OSFreeMem(gpsResList);
		gpsResList = IMG_NULL;
	}
}

/*!
******************************************************************************

 @Function	PVRSRVResManConnect

 @Description Opens a connection to the Resource Manager

 @output 	phResManContext - Resman context

 @Return    error code or PVRSRV_OK

******************************************************************************/
PVRSRV_ERROR PVRSRVResManConnect(PRESMAN_CONTEXT * phResManContext)
{
	PRESMAN_CONTEXT psResManContext;

	/*Acquire resource list sync object */
	ACQUIRE_SYNC_OBJ;

	/*Check resource list */
	VALIDATERESLIST();

	/* Allocate memory for the new context. */
	psResManContext = OSAllocMem(sizeof(*psResManContext));
	if (psResManContext == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVResManConnect: ERROR allocating new RESMAN context struct"));

		/* Check resource list */
		VALIDATERESLIST();

		/* Release resource list sync object */
		RELEASE_SYNC_OBJ;

		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psResManContext->ui32Signature = RESMAN_SIGNATURE;
	psResManContext->psResItemList = IMG_NULL;

	/* Insert new context struct after the dummy first entry */
	List_RESMAN_CONTEXT_Insert(&gpsResList->psContextList, psResManContext);

	/* Check resource list */
	VALIDATERESLIST();

	/* Release resource list sync object */
	RELEASE_SYNC_OBJ;

	*phResManContext = psResManContext;

	return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	PVRSRVResManDisconnect

 @Description Closes a Resource Manager connection and frees all resources

 @input 	hResManContext - Resman context
 @input 	bKernelContext - IMG_TRUE for kernel contexts

 @Return	IMG_VOID

******************************************************************************/
IMG_VOID PVRSRVResManDisconnect(PRESMAN_CONTEXT psResManContext,
				IMG_BOOL bKernelContext)
{

	/* Acquire resource list sync object */
	ACQUIRE_SYNC_OBJ;

	/* Check resource list */
	VALIDATERESLIST();

	/* Free all auto-freed resources in order */

	if (!bKernelContext) {
		/* OS specific User-mode Mappings: */
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_OS_USERMODE_MAPPING, 0, 0,
				       IMG_TRUE);

		/* Event Object */
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_EVENT_OBJECT, 0, 0,
				       IMG_TRUE);

		/* syncobject state (Read/Write Complete values) */
		/* Must be FIFO, so we reverse the list, twice */
		List_RESMAN_ITEM_Reverse(&psResManContext->psResItemList);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_MODIFY_SYNC_OPS, 0, 0,
				       IMG_TRUE);
		List_RESMAN_ITEM_Reverse(&psResManContext->psResItemList);	// (could survive without this - all following items would be cleared up "fifo" too)

		/* RGX types: */
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_RGX_RENDER_CONTEXT, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_RGX_FWIF_FREELIST, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_RGX_FWIF_HWRTDATA, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_RGX_FWIF_RENDERTARGET, 0, 0,
				       IMG_TRUE);

		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_RGX_TQ3D_CONTEXT, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_RGX_TQ2D_CONTEXT, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_RGX_COMPUTE_CONTEXT, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_RGX_CCB, 0, 0, IMG_TRUE);

		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_SHARED_PB_DESC_CREATE_LOCK,
				       0, 0, IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_SHARED_PB_DESC, 0, 0,
				       IMG_TRUE);

		/* MSVDX types: FIXME - TBD */

		/* COMMON types: */
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVMEM_MEM_EXPORT, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_SERVER_OP_COOKIE, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_SYNC_PRIMITIVE, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_SERVER_SYNC_PRIMITIVE, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_SERVER_SYNC_EXPORT, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_SYNC_PRIMITIVE_BLOCK, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_SYNC_INFO, 0, 0, IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVICECLASSMEM_MAPPING, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVICEMEM_WRAP, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVICEMEM_MAPPING, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_KERNEL_DEVICEMEM_ALLOCATION,
				       0, 0, IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVICEMEM_ALLOCATION, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVICEMEM_CONTEXT, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_SHARED_MEM_INFO, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVICEMEM2_MAPPING, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVICEMEM2_RESERVATION, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVICEMEM2_HEAP, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DEVICEMEM2_CONTEXT, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_PMR_PAGELIST, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_PMR_EXPORT, 0, 0, IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_PMR, 0, 0, IMG_TRUE);

		/* DISPLAY CLASS types: */
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DC_PIN_HANDLE, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DC_BUFFER, 0, 0, IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DC_DISPLAY_CONTEXT, 0, 0,
				       IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DC_DEVICE, 0, 0, IMG_TRUE);

		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DISPLAYCLASS_SWAPCHAIN_REF,
				       0, 0, IMG_TRUE);
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_DISPLAYCLASS_DEVICE, 0, 0,
				       IMG_TRUE);

		/* BUFFER CLASS types: */
		FreeResourceByCriteria(psResManContext, RESMAN_CRITERIA_RESTYPE,
				       RESMAN_TYPE_BUFFERCLASS_DEVICE, 0, 0,
				       IMG_TRUE);
	}

	/* Ensure that there are no resources left */
	PVR_ASSERT(psResManContext->psResItemList == IMG_NULL);

	/* Remove the context struct from the list */
	List_RESMAN_CONTEXT_Remove(psResManContext);

	/* Free the context struct */
	OSFreeMem(psResManContext);
	/*not nulling pointer, copy on stack */

	/* Check resource list */
	VALIDATERESLIST();

	/* Release resource list sync object */
	RELEASE_SYNC_OBJ;
}

/*!
******************************************************************************
 @Function	 ResManRegisterRes

 @Description    : Inform the resource manager that the given resource has
				   been alloacted and freeing of it will be the responsibility
				   of the resource manager

 @input 	psResManContext - resman context
 @input 	ui32ResType - identify what kind of resource it is
 @input 	pvParam - address of resource
 @input 	ui32Param - size of resource
 @input 	pfnFreeResource - pointer to function that frees this resource

 @Return   On success a pointer to an opaque data structure that represents
						the allocated resource, else NULL

**************************************************************************/
PRESMAN_ITEM ResManRegisterRes(PRESMAN_CONTEXT psResManContext,
			       IMG_UINT32 ui32ResType,
			       IMG_PVOID pvParam,
			       IMG_UINT32 ui32Param,
			       RESMAN_FREE_FN pfnFreeResource)
{
	PRESMAN_ITEM psNewResItem;

	PVR_ASSERT(psResManContext != IMG_NULL);
	PVR_ASSERT(ui32ResType != 0);

	if (psResManContext == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "ResManRegisterRes: invalid parameter - psResManContext"));
		return (PRESMAN_ITEM) IMG_NULL;
	}

	/* Acquire resource list sync object */
	ACQUIRE_SYNC_OBJ;

	/* Check resource list */
	VALIDATERESLIST();

	PVR_DPF((PVR_DBG_MESSAGE, "ResManRegisterRes: register resource "
		 "Context %p, ResType 0x%x, pvParam %p, ui32Param 0x%x, "
		 "FreeFunc %p",
		 psResManContext, ui32ResType, pvParam, ui32Param,
		 pfnFreeResource));

	/* Allocate memory for the new resource structure */
	psNewResItem = OSAllocMem(sizeof(RESMAN_ITEM));
	if (psNewResItem == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR, "ResManRegisterRes: "
			 "ERROR allocating new resource item"));

		/* Release resource list sync object */
		RELEASE_SYNC_OBJ;

		return ((PRESMAN_ITEM) IMG_NULL);
	}

	/* Fill in details about this resource */
	psNewResItem->ui32Signature = RESMAN_SIGNATURE;
	psNewResItem->ui32ResType = ui32ResType;
	psNewResItem->pvParam = pvParam;
	psNewResItem->ui32Param = ui32Param;
	psNewResItem->pfnFreeResource = pfnFreeResource;
	psNewResItem->ui32Flags = 0;

	/* Insert new structure after dummy first entry */
	List_RESMAN_ITEM_Insert(&psResManContext->psResItemList, psNewResItem);

	/* Check resource list */
	VALIDATERESLIST();

	/* Release resource list sync object */
	RELEASE_SYNC_OBJ;

	return (psNewResItem);
}

/*!
******************************************************************************
 @Function	 	ResManFreeResByPtr

 @Description   frees a resource by matching on pointer type

 @inputs        psResItem - pointer to resource item to free

 @Return   		PVRSRV_ERROR
**************************************************************************/
PVRSRV_ERROR ResManFreeResByPtr(RESMAN_ITEM * psResItem)
{
	PVRSRV_ERROR eError;

	PVR_ASSERT(psResItem != IMG_NULL);

	if (psResItem == IMG_NULL) {
		PVR_DPF((PVR_DBG_MESSAGE,
			 "ResManFreeResByPtr: NULL ptr - nothing to do"));
		return PVRSRV_OK;
	}

	PVR_DPF((PVR_DBG_MESSAGE, "ResManFreeResByPtr: freeing resource at %p",
		 psResItem));

	/*Acquire resource list sync object */
	ACQUIRE_SYNC_OBJ;

	/*Check resource list */
	VALIDATERESLIST();

	/*Free resource */
	eError = FreeResourceByPtr(psResItem, IMG_TRUE);

	/*Check resource list */
	VALIDATERESLIST();

	/*Release resource list sync object */
	RELEASE_SYNC_OBJ;

	return (eError);
}

/*
  FIXME:
   
  following comment should be in .h file
*/
/*!
******************************************************************************
 @Function	 	ResManFindPrivateDataByPtr

 @Description   finds the private date for a resource by matching on pointer type

 @inputs        psResItem - pointer to resource item

 @Return   		PVRSRV_ERROR
**************************************************************************/
PVRSRV_ERROR
ResManFindPrivateDataByPtr(RESMAN_ITEM * psResItem,
			   IMG_PVOID * ppvParam1, IMG_UINT32 * pui32Param2)
{
	PVR_ASSERT(psResItem != IMG_NULL);

	if (psResItem == IMG_NULL) {
		PVR_DPF((PVR_DBG_MESSAGE,
			 "ResManFindPrivateDataByPtr: NULL ptr - nothing to do"));
		return PVRSRV_OK;
	}

	PVR_DPF((PVR_DBG_MESSAGE,
		 "ResManFindPrivateDataByPtr: looking up private data for resource at %p",
		 psResItem));

	/*Acquire resource list sync object */
	ACQUIRE_SYNC_OBJ;

	/*Check resource list */
	VALIDATERESLIST();

	/* verify signature */
	PVR_ASSERT(psResItem->ui32Signature == RESMAN_SIGNATURE);

	/* lookup params */
	if (ppvParam1 != IMG_NULL) {
		*ppvParam1 = psResItem->pvParam;
	}
	if (pui32Param2 != IMG_NULL) {
		*pui32Param2 = psResItem->ui32Param;
	}

	/*Check resource list */
	VALIDATERESLIST();

	/*Release resource list sync object */
	RELEASE_SYNC_OBJ;

	return PVRSRV_OK;
}

/*!
******************************************************************************
 @Function	 	ResManFreeResByCriteria

 @Description   frees a resource by matching on criteria

 @inputs	 	hResManContext - handle for resman context
 @inputs        ui32SearchCriteria - indicates which parameters should be
 				used in search for resources to free
 @inputs        ui32ResType - identify what kind of resource to free
 @inputs        pvParam - address of resource to be free
 @inputs        ui32Param - size of resource to be free

 @Return   		PVRSRV_ERROR
**************************************************************************/
PVRSRV_ERROR ResManFreeResByCriteria(PRESMAN_CONTEXT psResManContext,
				     IMG_UINT32 ui32SearchCriteria,
				     IMG_UINT32 ui32ResType,
				     IMG_PVOID pvParam, IMG_UINT32 ui32Param)
{
	PVRSRV_ERROR eError;

	PVR_ASSERT(psResManContext != IMG_NULL);

	/* Acquire resource list sync object */
	ACQUIRE_SYNC_OBJ;

	/* Check resource list */
	VALIDATERESLIST();

	PVR_DPF((PVR_DBG_MESSAGE, "ResManFreeResByCriteria: "
		 "Context %p, Criteria 0x%x, Type 0x%x, Addr %p, Param 0x%x",
		 psResManContext, ui32SearchCriteria, ui32ResType,
		 pvParam, ui32Param));

	/* Free resources by criteria for this context */
	eError = FreeResourceByCriteria(psResManContext, ui32SearchCriteria,
					ui32ResType, pvParam, ui32Param,
					IMG_TRUE);

	/* Check resource list */
	VALIDATERESLIST();

	/* Release resource list sync object */
	RELEASE_SYNC_OBJ;

	return eError;
}

/*!
******************************************************************************
 @Function	 	ResManDissociateRes

 @Description   Moves a resource from one context to another.

 @inputs        psResItem - pointer to resource item to dissociate
 @inputs	 	psNewResManContext - new resman context for the resource

 @Return   		IMG_VOID
**************************************************************************/
PVRSRV_ERROR ResManDissociateRes(RESMAN_ITEM * psResItem,
				 PRESMAN_CONTEXT psNewResManContext)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	PVR_ASSERT(psResItem != IMG_NULL);

	if (psResItem == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "ResManDissociateRes: invalid parameter - psResItem"));
		PVR_DBG_BREAK;
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	PVR_ASSERT(psResItem->ui32Signature == RESMAN_SIGNATURE);

	if (psNewResManContext != IMG_NULL) {
		/* Remove this item from its old resource list */
		List_RESMAN_ITEM_Remove(psResItem);

		/* Re-insert into new list */
		List_RESMAN_ITEM_Insert(&psNewResManContext->psResItemList,
					psResItem);

	} else {
		eError = FreeResourceByPtr(psResItem, IMG_FALSE);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "ResManDissociateRes: failed to free resource by pointer"));
			return eError;
		}
	}

	return eError;
}

/*!
******************************************************************************
 @Function	 	ResManFindResourceByPtr_AnyVaCb

 @Description
 					Compares the resman item with a given pointer.

 @inputs	 	psCurItem - theThe item to check
 @inputs        va - Variable argument list with:
					 psItem - pointer to resource item to find

 @Return   		IMG_BOOL
**************************************************************************/
static IMG_BOOL ResManFindResourceByPtr_AnyVaCb(RESMAN_ITEM * psCurItem,
						va_list va)
{
	RESMAN_ITEM *psItem;

	psItem = va_arg(va, RESMAN_ITEM *);

	return (IMG_BOOL) (psCurItem == psItem);
}

/*!
******************************************************************************
 @Function	 	ResManFindResourceByPtr

 @Description
 					Attempts to find a resource in the list for this context

 @inputs	 	hResManContext - handle for resman context
 @inputs        psItem - pointer to resource item to find

 @Return   		PVRSRV_ERROR
**************************************************************************/
IMG_INTERNAL PVRSRV_ERROR ResManFindResourceByPtr(PRESMAN_CONTEXT
						  psResManContext,
						  RESMAN_ITEM * psItem)
{
/*	RESMAN_ITEM		*psCurItem;*/

	PVRSRV_ERROR eResult;

	PVR_ASSERT(psResManContext != IMG_NULL);
	PVR_ASSERT(psItem != IMG_NULL);

	if ((psItem == IMG_NULL) || (psResManContext == IMG_NULL)) {
		PVR_DPF((PVR_DBG_ERROR,
			 "ResManFindResourceByPtr: invalid parameter"));
		PVR_DBG_BREAK;
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	PVR_ASSERT(psItem->ui32Signature == RESMAN_SIGNATURE);

	/* Acquire resource list sync object */
	ACQUIRE_SYNC_OBJ;

	PVR_DPF((PVR_DBG_MESSAGE,
		 "FindResourceByPtr: psItem=%p, psItem->psNext=%p",
		 psItem, psItem->psNext));

	PVR_DPF((PVR_DBG_MESSAGE,
		 "FindResourceByPtr: Resource Ctx %p, Type 0x%x, Addr %p, "
		 "Param 0x%x, FnCall %p, Flags 0x%x",
		 psResManContext, psItem->ui32ResType, psItem->pvParam,
		 psItem->ui32Param, psItem->pfnFreeResource,
		 psItem->ui32Flags));

	/* Search resource items starting at after the first dummy item */
	if (List_RESMAN_ITEM_IMG_BOOL_Any_va(psResManContext->psResItemList,
					     &ResManFindResourceByPtr_AnyVaCb,
					     psItem)) {
		eResult = PVRSRV_OK;
	} else {
		eResult = PVRSRV_ERROR_NOT_OWNER;
	}

	/* Release resource list sync object */
	RELEASE_SYNC_OBJ;

/*	return PVRSRV_ERROR_NOT_OWNER;*/
	return eResult;
}

/*!
******************************************************************************
 @Function	 	FreeResourceByPtr

 @Description
 					Frees a resource and move it from the list
					NOTE : this function must be called with the resource
					list sync object held

 @inputs        psItem - pointer to resource item to free
 				bExecuteCallback - execute callback?

 @Return   		PVRSRV_ERROR
**************************************************************************/
static PVRSRV_ERROR FreeResourceByPtr(RESMAN_ITEM * psItem,
				      IMG_BOOL bExecuteCallback)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	PVR_ASSERT(psItem != IMG_NULL);

	if (psItem == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "FreeResourceByPtr: invalid parameter"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	PVR_ASSERT(psItem->ui32Signature == RESMAN_SIGNATURE);

	PVR_DPF((PVR_DBG_MESSAGE,
		 "FreeResourceByPtr: psItem=%p, psItem->psNext=%p",
		 psItem, psItem->psNext));

	PVR_DPF((PVR_DBG_MESSAGE,
		 "FreeResourceByPtr: Type 0x%x, Addr %p, "
		 "Param 0x%x, FnCall %p, Flags 0x%x",
		 psItem->ui32ResType, psItem->pvParam,
		 psItem->ui32Param, psItem->pfnFreeResource,
		 psItem->ui32Flags));

	/* Remove this item from the resource list */
	List_RESMAN_ITEM_Remove(psItem);

	/* Release resource list sync object just in case the free routine calls the resource manager */
	RELEASE_SYNC_OBJ;

	/* Call the freeing routine */
	if (bExecuteCallback) {
		eError =
		    psItem->pfnFreeResource(psItem->pvParam, psItem->ui32Param);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "FreeResourceByPtr: ERROR calling FreeResource function"));
		}
	}

	/* Acquire resource list sync object */
	ACQUIRE_SYNC_OBJ;

	/* Free memory for the resource item */
	OSFreeMem(psItem);

	return (eError);
}

/*!
******************************************************************************
 @Function	 	FreeResourceByCriteria_AnyVaCb

 @Description
 					Matches a resource manager item with a given criteria.

 @inputs        psCuItem - the item to be matched
 @inputs		va - a variable argument list with:.
					ui32SearchCriteria - indicates which parameters should be used
					search for resources to free
					ui32ResType - identify what kind of resource to free
					pvParam - address of resource to be free
					ui32Param - size of resource to be free

 @Return   		psCurItem if matched, IMG_NULL otherwise.
**************************************************************************/
static IMG_VOID *FreeResourceByCriteria_AnyVaCb(RESMAN_ITEM * psCurItem,
						va_list va)
{
	IMG_UINT32 ui32SearchCriteria;
	IMG_UINT32 ui32ResType;
	IMG_PVOID pvParam;
	IMG_UINT32 ui32Param;

	ui32SearchCriteria = va_arg(va, IMG_UINT32);
	ui32ResType = va_arg(va, IMG_UINT32);
	pvParam = va_arg(va, IMG_PVOID);
	ui32Param = va_arg(va, IMG_UINT32);

	/*check that for all conditions are either disabled or eval to true */
	if (
		   /* Check resource type */
		   (((ui32SearchCriteria & RESMAN_CRITERIA_RESTYPE) == 0UL) ||
		    (psCurItem->ui32ResType == ui32ResType))
		   &&
		   /* Check address */
		   (((ui32SearchCriteria & RESMAN_CRITERIA_PVOID_PARAM) == 0UL)
		    || (psCurItem->pvParam == pvParam))
		   &&
		   /* Check size */
		   (((ui32SearchCriteria & RESMAN_CRITERIA_UI32_PARAM) == 0UL)
		    || (psCurItem->ui32Param == ui32Param))
	    ) {
		return psCurItem;
	} else {
		return IMG_NULL;
	}
}

/*!
******************************************************************************
 @Function	 	FreeResourceByCriteria

 @Description
 					Frees all resources that match the given criteria for the
					context.
					NOTE : this function must be called with the resource
					list sync object held

 @inputs        psResManContext - pointer to resman context
 @inputs        ui32SearchCriteria - indicates which parameters should be used
 @inputs        search for resources to free
 @inputs        ui32ResType - identify what kind of resource to free
 @inputs        pvParam - address of resource to be free
 @inputs        ui32Param - size of resource to be free
 @inputs        ui32AutoFreeLev - auto free level to free
 @inputs        bExecuteCallback - execute callback?

 @Return   		PVRSRV_ERROR
**************************************************************************/
static PVRSRV_ERROR FreeResourceByCriteria(PRESMAN_CONTEXT psResManContext,
					   IMG_UINT32 ui32SearchCriteria,
					   IMG_UINT32 ui32ResType,
					   IMG_PVOID pvParam,
					   IMG_UINT32 ui32Param,
					   IMG_BOOL bExecuteCallback)
{
	PRESMAN_ITEM psCurItem;
	PVRSRV_ERROR eError = PVRSRV_OK;

	/* Search resource items starting at after the first dummy item */
	/*while we get a match and not an error */
	while ((psCurItem = (PRESMAN_ITEM)
		List_RESMAN_ITEM_Any_va(psResManContext->psResItemList,
					&FreeResourceByCriteria_AnyVaCb,
					ui32SearchCriteria,
					ui32ResType,
					pvParam,
					ui32Param)) != IMG_NULL
	       && eError == PVRSRV_OK) {
		eError = FreeResourceByPtr(psCurItem, bExecuteCallback);
	}

	return eError;
}

#if defined(DEBUG)
/*!
******************************************************************************
 @Function	 	ValidateResList

 @Description
 					Walks the resource list check the pointers
					NOTE : this function must be called with the resource
					list sync object held

 @Return   		none
**************************************************************************/
static IMG_VOID ValidateResList(PRESMAN_LIST psResList)
{
	PRESMAN_ITEM psCurItem, *ppsThisItem;
	PRESMAN_CONTEXT psCurContext, *ppsThisContext;

	/* check we're initialised */
	if (psResList == IMG_NULL) {
		PVR_DPF((PVR_DBG_MESSAGE,
			 "ValidateResList: resman not initialised yet"));
		return;
	}

	psCurContext = psResList->psContextList;
	ppsThisContext = &psResList->psContextList;

	/* Walk the context list */
	while (psCurContext != IMG_NULL) {
		/* Check current item */
		PVR_ASSERT(psCurContext->ui32Signature == RESMAN_SIGNATURE);
		if (psCurContext->ppsThis != ppsThisContext) {
			PVR_DPF((PVR_DBG_WARNING,
				 "psCC=%p psCC->ppsThis=%p psCC->psNext=%p ppsTC=%p",
				 psCurContext, psCurContext->ppsThis,
				 psCurContext->psNext, ppsThisContext));
			PVR_ASSERT(psCurContext->ppsThis == ppsThisContext);
		}

		/* Walk the list for this context */
		psCurItem = psCurContext->psResItemList;
		ppsThisItem = &psCurContext->psResItemList;
		while (psCurItem != IMG_NULL) {
			/* Check current item */
			PVR_ASSERT(psCurItem->ui32Signature ==
				   RESMAN_SIGNATURE);
			if (psCurItem->ppsThis != ppsThisItem) {
				PVR_DPF((PVR_DBG_WARNING,
					 "psCurItem=%p psCurItem->ppsThis=%p psCurItem->psNext=%p ppsThisItem=%p",
					 psCurItem, psCurItem->ppsThis,
					 psCurItem->psNext, ppsThisItem));
				PVR_ASSERT(psCurItem->ppsThis == ppsThisItem);
			}

			/* Move to next item */
			ppsThisItem = &psCurItem->psNext;
			psCurItem = psCurItem->psNext;
		}

		/* Move to next context */
		ppsThisContext = &psCurContext->psNext;
		psCurContext = psCurContext->psNext;
	}
}
#endif				/* DEBUG */

/******************************************************************************
 End of file (resman.c)
******************************************************************************/
