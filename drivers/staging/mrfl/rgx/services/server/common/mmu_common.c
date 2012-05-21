									    /*************************************************************************//*!
									       @File
									       @Title          Common MMU Management
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements basic low level control of MMU.
									       @License        Strictly Confidential.
    *//***************************************************************************/

/* Our own interface */
#include "mmu_common.h"

/*
Interfaces to other modules:

Let's keep this graph up-to-date:

   +-----------+
   | devicemem |
   +-----------+
         |
   +============+
   | mmu_common |
   +============+
         |
         +-----------------+
         |                 |
    +---------+      +----------+
    |   pmr   |      |  device  |
    +---------+      +----------+
*/

#include "img_types.h"
#include "osfunc.h"
#include "allocmem.h"
#if defined(PDUMP)
#include "pdump_km.h"
#endif
#include "pmr.h"
/* include/ */
#include "pvr_debug.h"
#include "pvrsrv_error.h"
#include "pvrsrv.h"

/* 
    FIXME: we should probably have a special place for such things.
*/
#ifndef MAX
#define MAX(a,b) 					(((a) > (b)) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a,b) 					(((a) < (b)) ? (a) : (b))
#endif

// #define MMU_OBJECT_REFCOUNT_DEBUGING 1
#if defined (MMU_OBJECT_REFCOUNT_DEBUGING)
#define MMU_OBJ_DBG(x)	PVR_DPF(x);
#else
#define MMU_OBJ_DBG(x)
#endif

typedef IMG_UINT32 MMU_FLAGS_T;

/*!
	All physical allocations and frees are relative to this context, so
	we would get all the allocations of PCs, PDs, and PTs from the same
	RA.

	We have one per MMU context in case we have mixed UMA/LMA devices
	within the same system.
*/
typedef struct _MMU_PHYSMEM_CONTEXT_ {
	/*! Parent device node */
	PVRSRV_DEVICE_NODE *psDevNode;

	/*! Refcount so we know when to free up the arena */
	IMG_UINT32 uiNumAllocations;

	/*! Arena from which physical memory is derived */
	RA_ARENA *psPhysMemRA;
	/*! Arena name */
	IMG_CHAR *pszPhysMemRAName;
	/*! Size of arena name string */
	IMG_SIZE_T uiPhysMemRANameAllocSize;

} MMU_PHYSMEM_CONTEXT;

/*!
	Mapping structure for MMU memory allocation
*/
typedef struct _MMU_MEMORY_MAPPING_ {
	/*! Physmem context to allocate from */
	MMU_PHYSMEM_CONTEXT *psContext;
	/*! OS/system Handle for this allocation */
	Px_HANDLE sMemHandle;
	/*! CPU virtual address of this allocation */
	IMG_VOID *pvCpuVAddr;
	/*! CPU physical address of this allocation */
	IMG_CPU_PHYADDR sCpuPAddr;
	/*! Device physical address of this allocation */
	IMG_DEV_PHYADDR sDevPAddr;
	/*! Size of this alloction */
	IMG_SIZE_T uiSize;
	/*! Number of current mappings of this allocation */
	IMG_UINT32 uiCpuVAddrRefCount;
} MMU_MEMORY_MAPPING;

/*!
	Memory descriptor for MMU objects. There can be more then one memory
	descriptor per MMU memory allocation.
*/
typedef struct _MMU_MEMORY_DESC_ {
	/* NB: bValid is set if this descriptor describes physical
	   memory.  This allows "empty" descriptors to exist, such that we
	   can allocate them in batches.  */
	/*! Does this MMU object have physical backing */
	IMG_BOOL bValid;
	/*! Device Physcial address of physical backing */
	IMG_DEV_PHYADDR sDevPAddr;
	/*! CPU virtual address of physical backing */
	IMG_VOID *pvCpuVAddr;
	/*! Mapping data for this MMU object */
	MMU_MEMORY_MAPPING *psMapping;
} MMU_MEMORY_DESC;

/*!
	MMU levelx structure. This is generic and is used
	for all levels (PC, PD, PT).
*/
typedef struct _MMU_Levelx_INFO_ {
	/*! The Number of enteries in this level */
	IMG_UINT32 ui32NumOfEntries;

	/*! Number of times this level has been reference. Note: For Level1 (PTE)
	   we still take/drop the reference when setting up the page tables rather
	   then at map/unmap time as this simplifies things */
	IMG_UINT32 ui32RefCount;

	/*! MemDesc for this level */
	MMU_MEMORY_DESC sMemDesc;

	/*! Array of infos for the next level. Must be last member in structure */
	struct _MMU_Levelx_INFO_ *apsNextLevel[1];
} MMU_Levelx_INFO;

/*!
	MMU context structure
*/
struct _MMU_CONTEXT_ {
	/*! Parent device node */
	PVRSRV_DEVICE_NODE *psDevNode;

	MMU_DEVICEATTRIBS *psDevAttrs;

	/*! For allocation and deallocation of the physical memory where
	   the pagetables live */
	struct _MMU_PHYSMEM_CONTEXT_ *psPhysMemCtx;

#if defined(PDUMP)
	/*! PDump context ID (required for PDump commands with virtual addresses) */
	IMG_UINT32 uiPDumpContextID;

	/*! The refcount of the PDump context ID */
	IMG_UINT32 ui32PDumpContextIDRefCount;
#endif

	/*! Data that is passed back during device specific callbacks */
	IMG_HANDLE hDevData;

	/*! Base level info structure. Must be last member in structure */
	MMU_Levelx_INFO sBaseLevelInfo;
};

/* useful macros */
/* units represented in a bitfield */
#define UNITS_IN_BITFIELD(Mask, Shift)	((Mask >> Shift) + 1)

#if IMG_ADDRSPACE_PHYSADDR_BITS == 32
//#warning you are still using 32-bit device physicial addresses
#define MMU_BAD_PHYS_ADDR 0xbadbad00
#else
#define MMU_BAD_PHYS_ADDR 0xbadbad00badULL
#endif
static const IMG_DEV_PHYADDR gsBadDevPhyAddr = { MMU_BAD_PHYS_ADDR };

/*****************************************************************************
 *                          Utility functions                                *
 *****************************************************************************/

									    /*************************************************************************//*!
									       @Function       _CalcPCEIdx

									       @Description    Calculate the page catalogue index

									       @Input          sDevVAddr           Device virtual address

									       @Input          psDevVAddrConfig    Configuration of the virtual address

									       @Input          bRoundUp            Round up the index

									       @Return         The page catalogue index
									     */
/*****************************************************************************/
static IMG_UINT32 _CalcPCEIdx(IMG_DEV_VIRTADDR sDevVAddr,
			      const MMU_DEVVADDR_CONFIG * psDevVAddrConfig,
			      IMG_BOOL bRoundUp)
{
	IMG_DEV_VIRTADDR sTmpDevVAddr;
	IMG_UINT32 ui32RetVal;

	sTmpDevVAddr = sDevVAddr;

	if (bRoundUp) {
		sTmpDevVAddr.uiAddr--;
	}
	ui32RetVal =
	    (IMG_UINT32) ((sTmpDevVAddr.uiAddr & psDevVAddrConfig->
			   uiPCIndexMask)
			  >> psDevVAddrConfig->uiPCIndexShift);

	if (bRoundUp) {
		ui32RetVal++;
	}

	return ui32RetVal;
}

									    /*************************************************************************//*!
									       @Function       _CalcPCEIdx

									       @Description    Calculate the page directory index

									       @Input          sDevVAddr           Device virtual address

									       @Input          psDevVAddrConfig    Configuration of the virtual address

									       @Input          bRoundUp            Round up the index

									       @Return         The page directory index
									     */
/*****************************************************************************/
static IMG_UINT32 _CalcPDEIdx(IMG_DEV_VIRTADDR sDevVAddr,
			      const MMU_DEVVADDR_CONFIG * psDevVAddrConfig,
			      IMG_BOOL bRoundUp)
{
	IMG_DEV_VIRTADDR sTmpDevVAddr;
	IMG_UINT32 ui32RetVal;

	sTmpDevVAddr = sDevVAddr;

	if (bRoundUp) {
		sTmpDevVAddr.uiAddr--;
	}
	ui32RetVal =
	    (IMG_UINT32) ((sTmpDevVAddr.uiAddr & psDevVAddrConfig->
			   uiPDIndexMask)
			  >> psDevVAddrConfig->uiPDIndexShift);

	if (bRoundUp) {
		ui32RetVal++;
	}

	return ui32RetVal;
}

									    /*************************************************************************//*!
									       @Function       _CalcPCEIdx

									       @Description    Calculate the page entry index

									       @Input          sDevVAddr           Device virtual address

									       @Input          psDevVAddrConfig    Configuration of the virtual address

									       @Input          bRoundUp            Round up the index

									       @Return         The page entry index
									     */
/*****************************************************************************/
static IMG_UINT32 _CalcPTEIdx(IMG_DEV_VIRTADDR sDevVAddr,
			      const MMU_DEVVADDR_CONFIG * psDevVAddrConfig,
			      IMG_BOOL bRoundUp)
{
	IMG_DEV_VIRTADDR sTmpDevVAddr;
	IMG_UINT32 ui32RetVal;

	sTmpDevVAddr = sDevVAddr;

	if (bRoundUp) {
		sTmpDevVAddr.uiAddr--;
	}
	ui32RetVal =
	    (IMG_UINT32) ((sTmpDevVAddr.uiAddr & psDevVAddrConfig->
			   uiPTIndexMask)
			  >> psDevVAddrConfig->uiPTIndexShift);

	if (bRoundUp) {
		ui32RetVal++;
	}

	return ui32RetVal;
}

/*****************************************************************************
 *         MMU memory allocation/maganement functions (mem desc)             *
 *****************************************************************************/

									    /*************************************************************************//*!
									       @Function       _MMU_PhysMem_RAImportAlloc

									       @Description    Imports MMU Px memory into the RA. This is where the
									       actual allocation of physical memory happens.

									       @Input          hArenaHandle    Handle that was passed in during the
									       creation of the RA

									       @Input          uiSize          Size of the memory to import

									       @Input          uiFlags         Flags that where passed in the allocation.

									       @Output         puiBase         The address of where to insert this import

									       @Output         puiActualSize   The actual size of the import

									       @Output         phPriv          Handle which will be passed back when
									       this import is freed

									       @Return         IMG_TURE if import alloc was successful, otherwise IMG_FALSE
									     */
/*****************************************************************************/
static IMG_BOOL _MMU_PhysMem_RAImportAlloc(RA_PERARENA_HANDLE hArenaHandle,
					   RA_LENGTH_T uiSize,
					   RA_FLAGS_T uiFlags,
					   RA_BASE_T * puiBase,
					   RA_LENGTH_T * puiActualSize,
					   RA_PERISPAN_HANDLE * phPriv)
{
	MMU_PHYSMEM_CONTEXT *psCtx = (MMU_PHYSMEM_CONTEXT *) hArenaHandle;
	PVRSRV_DEVICE_NODE *psDevNode = (PVRSRV_DEVICE_NODE *) psCtx->psDevNode;
	MMU_MEMORY_MAPPING *psMapping;
	PVRSRV_ERROR eError;

	/* FIXME?: Might we want to pass in the MMU level this way so we
	   don't mix PC/PD/PT in the same allocation. Might make
	   cache managament easier */
	PVR_UNREFERENCED_PARAMETER(uiFlags);

	psMapping = OSAllocMem(sizeof(MMU_MEMORY_MAPPING));
	if (psMapping == IMG_NULL) {
		goto e0;
	}

	eError =
	    psDevNode->pfnMMUPxAlloc(psDevNode, uiSize, &psMapping->sMemHandle,
				     &psMapping->sDevPAddr);
	if (eError != PVRSRV_OK) {
		goto e1;
	}

	psMapping->psContext = psCtx;
	psMapping->uiSize = uiSize;

	/* FIXME: Assumes UMA */
	psMapping->sCpuPAddr.uiAddr = psMapping->sDevPAddr.uiAddr;
	psMapping->uiCpuVAddrRefCount = 0;

	*phPriv = (RA_PERISPAN_HANDLE) psMapping;

	/* Note: This assumes this memory never gets paged out */
	*puiBase = (RA_BASE_T) psMapping->sDevPAddr.uiAddr;
	*puiActualSize = uiSize;

	return IMG_TRUE;

 e1:
	OSFreeMem(psMapping);
 e0:
	return IMG_FALSE;
}

									    /*************************************************************************//*!
									       @Function       _MMU_PhysMem_RAImportFree

									       @Description    Imports MMU Px memory into the RA. This is where the
									       actual free of physical memory happens.

									       @Input          hArenaHandle    Handle that was passed in during the
									       creation of the RA

									       @Input          puiBase         The address of where to insert this import

									       @Output         phPriv          Private data that the import alloc provided

									       @Return         None
									     */
/*****************************************************************************/
static IMG_VOID _MMU_PhysMem_RAImportFree(RA_PERARENA_HANDLE hArenaHandle,
					  RA_BASE_T uiBase,
					  RA_PERISPAN_HANDLE hPriv)
{
	MMU_MEMORY_MAPPING *psMapping = (MMU_MEMORY_MAPPING *) hPriv;
	MMU_PHYSMEM_CONTEXT *psCtx = (MMU_PHYSMEM_CONTEXT *) hArenaHandle;
	PVRSRV_DEVICE_NODE *psDevNode = (PVRSRV_DEVICE_NODE *) psCtx->psDevNode;

	PVR_UNREFERENCED_PARAMETER(uiBase);

	/* Check we have dropped all CPU mappings */
	PVR_ASSERT(psMapping->uiCpuVAddrRefCount == 0);

	psDevNode->pfnMMUPxFree(psDevNode, &psMapping->sMemHandle);
	OSFreeMem(psMapping);
}

									    /*************************************************************************//*!
									       @Function       _MMU_PhysMemAlloc

									       @Description    Allocates physical memory for MMU objects

									       @Input          psCtx           Physmem context to do the allocation from

									       @Output         psMemDesc       Allocation discription

									       @Input          uiBytes         Size of the allocation in bytes

									       @Input          uiAlignment     Alignment requirement of this allocation

									       @Return         PVRSRV_OK if allocation was successful
									     */
/*****************************************************************************/

static PVRSRV_ERROR _MMU_PhysMemAlloc(MMU_PHYSMEM_CONTEXT * psCtx,
				      MMU_MEMORY_DESC * psMemDesc,
				      IMG_SIZE_T uiBytes,
				      IMG_SIZE_T uiAlignment)
{
	RA_BASE_T uiPhysAddr;
	IMG_BOOL bStatus;

	if (!psMemDesc || psMemDesc->bValid) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	bStatus = RA_Alloc(psCtx->psPhysMemRA, uiBytes, 0,	// flags
			   uiAlignment,
			   &uiPhysAddr,
			   IMG_NULL,
			   (RA_PERISPAN_HANDLE *) & psMemDesc->psMapping);
	if (!bStatus) {
		PVR_DPF((PVR_DBG_ERROR,
			 "_MMU_PhysMemAlloc: ERROR call to RA_Alloc() failed"));
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMemDesc->bValid = IMG_TRUE;
	psMemDesc->pvCpuVAddr = IMG_NULL;
	psMemDesc->sDevPAddr.uiAddr = (IMG_UINTPTR_T) uiPhysAddr;

	return PVRSRV_OK;
}

									    /*************************************************************************//*!
									       @Function       _MMU_PhysMemFree

									       @Description    Allocates physical memory for MMU objects

									       @Input          psCtx           Physmem context to do the free on

									       @Input          psMemDesc       Allocation discription

									       @Return         None
									     */
/*****************************************************************************/
static IMG_VOID _MMU_PhysMemFree(MMU_PHYSMEM_CONTEXT * psCtx,
				 MMU_MEMORY_DESC * psMemDesc)
{
	RA_BASE_T uiPhysAddr;

	PVR_ASSERT(psMemDesc->bValid);

	uiPhysAddr = psMemDesc->sDevPAddr.uiAddr;	/* FIXME:  translate addr if necc */
	RA_Free(psCtx->psPhysMemRA, uiPhysAddr);

	psMemDesc->bValid = IMG_FALSE;
}

									    /*************************************************************************//*!
									       @Function       _MMU_MapCPUVAddr

									       @Description    Map an allocation of physical memory for MMU objects
									       into the CPU address space

									       @Input          psMemDesc       Allocation discription

									       @Return         PVRSRV_OK if map was successful
									     */
/*****************************************************************************/
static PVRSRV_ERROR _MMU_MapCPUVAddr(MMU_MEMORY_DESC * psMMUMemDesc)
{
	MMU_MEMORY_MAPPING *psMapping = psMMUMemDesc->psMapping;
	MMU_PHYSMEM_CONTEXT *psCtx = psMapping->psContext;
	PVRSRV_DEVICE_NODE *psDevNode = psCtx->psDevNode;
	PVRSRV_ERROR eError = PVRSRV_OK;

	/* There should only be one call to map */
	PVR_ASSERT(psMMUMemDesc->pvCpuVAddr == IMG_NULL);

	/* FIXME: Lock */
	if (psMapping->uiCpuVAddrRefCount == 0) {

		eError = psDevNode->pfnMMUPxMap(psDevNode,
						&psMapping->sMemHandle,
						psMapping->uiSize,
						&psMapping->sDevPAddr,
						&psMapping->pvCpuVAddr);
	}
	psMapping->uiCpuVAddrRefCount++;
	/* FIXME: Unlock */

	PVR_ASSERT(psMapping->pvCpuVAddr != IMG_NULL);

	/* Workout the address for this mem desc */
	psMMUMemDesc->pvCpuVAddr = ((IMG_UINT8 *) psMapping->pvCpuVAddr) +
	    (psMMUMemDesc->psMapping->sDevPAddr.uiAddr -
	     psMapping->sDevPAddr.uiAddr);

	return eError;
}

									    /*************************************************************************//*!
									       @Function       _MMU_UnmapCPUVAddr

									       @Description    Unmap an allocation of physical memory for MMU objects
									       from the CPU address space

									       @Input          psMemDesc       Allocation discription

									       @Return         PVRSRV_OK if map was successful
									     */
/*****************************************************************************/
static PVRSRV_ERROR _MMU_UnmapCPUVAddr(MMU_MEMORY_DESC * psMMUMemDesc)
{
	MMU_MEMORY_MAPPING *psMapping = psMMUMemDesc->psMapping;
	MMU_PHYSMEM_CONTEXT *psCtx = psMapping->psContext;
	PVRSRV_DEVICE_NODE *psDevNode = psCtx->psDevNode;

	/* FIXME: Lock */
	if (--psMapping->uiCpuVAddrRefCount == 0) {
		psDevNode->pfnMMUPxUnmap(psDevNode, &psMapping->sMemHandle,
					 psMMUMemDesc->pvCpuVAddr);
	}
	/* FIXME: Unlock */

	psMMUMemDesc->pvCpuVAddr = IMG_NULL;

	return PVRSRV_OK;
}

/*****************************************************************************
 *              MMU object allocation/management functions                   *
 *****************************************************************************/

									    /*************************************************************************//*!
									       @Function       _PxMemAlloc

									       @Description    Allocates physical memory for MMU objects, initialises
									       and PDumps it.

									       @Input          psMMUContext    MMU context

									       @Input          uiNumEntries    Number of entries to allocate

									       @Input          psConfig        MMU Px config

									       @Input          eMMULevel       MMU level that that allocation is for

									       @Output         psMemDesc       Description of allocation

									       @Return         PVRSRV_OK if allocation was successful
									     */
/*****************************************************************************/
static PVRSRV_ERROR _PxMemAlloc(MMU_CONTEXT * psMMUContext,
				IMG_UINT32 uiNumEntries,
				const MMU_PxE_CONFIG * psConfig,
				MMU_LEVEL eMMULevel,
				MMU_MEMORY_DESC * psMemDesc)
{
	PVRSRV_ERROR eError;
	IMG_SIZE_T uiBytes;
	IMG_SIZE_T uiAlign;
#if defined(PDUMP)
	PVRSRV_DEVICE_NODE *psDevNode = psMMUContext->psDevNode;
#endif
	PVR_ASSERT(psConfig->uiBytesPerEntry != 0);

	uiBytes = uiNumEntries * psConfig->uiBytesPerEntry;
	uiAlign = 1 << psConfig->uiLog2Align;

	/*  allocate the object */
	eError = _MMU_PhysMemAlloc(psMMUContext->psPhysMemCtx,
				   psMemDesc, uiBytes, uiAlign);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "_PxMemAlloc: failed to allocate memory for the  MMU object"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	/*
	   some OS's can alloc memory without a CPU ptr.
	   Map the memory to the CPU (may be a no-op) 
	 */
	eError = _MMU_MapCPUVAddr(psMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "_PxMemAlloc: failed to map MMU object to CPU"));
		eError = PVRSRV_ERROR_FAILED_TO_MAP_PAGE_TABLE;
		goto e1;
	}

	/*
	   Clear the object
	   Note: if any MMUs are cleared with non-zero values then will need a custom
	   clear function
	 */
	OSMemSet(psMemDesc->pvCpuVAddr, 0, uiBytes);

#if defined(PDUMP)
	PDUMPCOMMENT("Alloc MMU object");

	/* new pdump API */
	/* FIXME: I'd rather not have this switch statment. Review this and
	   see if we can rework pdump to not require it */
	switch (eMMULevel) {
	case MMU_LEVEL_3:
		PDUMP_MMU_MALLOC_PC(psDevNode->pszMMUPxPDumpMemSpaceName,
				    &psMemDesc->sDevPAddr, uiBytes, uiAlign);

		PDUMP_MMU_DUMP_PC_ENTRIES(psDevNode->pszMMUPxPDumpMemSpaceName,
					  psMemDesc->pvCpuVAddr,
					  psMemDesc->sDevPAddr,
					  0,
					  uiNumEntries,
					  psConfig->uiBytesPerEntry,
					  psConfig->uiLog2Align,
					  psConfig->uiAddrShift,
					  psConfig->uiAddrMask,
					  psConfig->uiProtMask, 0);
		break;

	case MMU_LEVEL_2:
		PDUMP_MMU_MALLOC_PD(psDevNode->pszMMUPxPDumpMemSpaceName,
				    &psMemDesc->sDevPAddr, uiBytes, uiAlign);

		PDUMP_MMU_DUMP_PD_ENTRIES(psDevNode->pszMMUPxPDumpMemSpaceName,
					  psMemDesc->pvCpuVAddr,
					  psMemDesc->sDevPAddr, 0, uiNumEntries,
					  /* FIXME: should it be psMMUHeap->sPDumpAttrib.sHeap...? */
					  psConfig->uiBytesPerEntry,
					  psConfig->uiLog2Align,
					  psConfig->uiAddrShift,
					  psConfig->uiAddrMask,
					  psConfig->uiProtMask, 0);
		break;

	case MMU_LEVEL_1:
		PDUMP_MMU_MALLOC_PT(psDevNode->pszMMUPxPDumpMemSpaceName,
				    &psMemDesc->sDevPAddr, uiBytes, uiAlign);

		PDUMP_MMU_DUMP_PT_ENTRIES(psDevNode->pszMMUPxPDumpMemSpaceName, psMemDesc->pvCpuVAddr, psMemDesc->sDevPAddr, 0, uiNumEntries, IMG_NULL, IMG_NULL, 0,	/* pdump symbolic info is irrelevant here */
					  psConfig->uiBytesPerEntry,
					  psConfig->uiLog2Align,
					  psConfig->uiAddrShift,
					  psConfig->uiAddrMask,
					  psConfig->uiProtMask, 0);
		break;
	default:
		PVR_DPF((PVR_DBG_ERROR, "_PxMemAlloc: Invalid MMU level"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e2;
	}
#endif

	/* unmap the memory from the CPU (may be a no-op) */
	_MMU_UnmapCPUVAddr(psMemDesc);

	return PVRSRV_OK;

#if defined(PDUMP)
 e2:
	_MMU_UnmapCPUVAddr(psMemDesc);
#endif
 e1:
	_MMU_PhysMemFree(psMMUContext->psPhysMemCtx, psMemDesc);
 e0:
	PVR_ASSERT(eError == PVRSRV_OK);
	return eError;
}

									    /*************************************************************************//*!
									       @Function       _PxMemFree

									       @Description    Frees physical memory for MMU objects, de-initialises
									       and PDumps it.

									       @Input          psMemDesc       Description of allocation

									       @Return         PVRSRV_OK if allocation was successful
									     */
/*****************************************************************************/

static IMG_VOID _PxMemFree(MMU_CONTEXT * psMMUContext,
			   MMU_MEMORY_DESC * psMemDesc, MMU_LEVEL eMMULevel)
{
#if defined(PDUMP)
	PVRSRV_DEVICE_NODE *psDevNode = psMMUContext->psDevNode;
#endif
#if defined(MMU_CLEARMEM_ON_FREE)
	PVRSRV_ERROR eError;
	/*
	   some OS's can alloc memory without a CPU ptr.
	   Map the memory to the CPU (may be a no-op) 
	 */
	eError = _MMU_MapCPUVAddr(psMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "_PxMemFree: failed to map MMU object to CPU"));
		PVR_ASSERT(0);
	}

	/*
	   Clear the MMU object
	   Note: if any MMUs are cleared with non-zero values then will need a custom
	   clear function
	 */
	OSMemSet(psMemDesc->pvCpuVAddr, 0, psMemDesc->ui32Bytes);

#if defined(PDUMP)
	PDUMPCOMMENT("Clear MMU object before freeing it");
#endif

	/* unmap the memory from the CPU (may be a no-op) */
	_MMU_UnmapCPUVAddr(psMemDesc);
#endif				/* MMU_CLEARMEM_ON_FREE */

#if defined(PDUMP)
	PDUMPCOMMENT("Free MMU object");

	/* new pdump API */
	/* FIXME: I'd rather not have this switch statment. Review this and
	   see if we can rework pdump to not require it */

	switch (eMMULevel) {
	case MMU_LEVEL_3:
		PDUMP_MMU_FREE_PC(psDevNode->pszMMUPxPDumpMemSpaceName,
				  &psMemDesc->sDevPAddr);
		break;

	case MMU_LEVEL_2:
		PDUMP_MMU_FREE_PD(psDevNode->pszMMUPxPDumpMemSpaceName,
				  &psMemDesc->sDevPAddr);
		break;

	case MMU_LEVEL_1:
		PDUMP_MMU_FREE_PT(psDevNode->pszMMUPxPDumpMemSpaceName,
				  &psMemDesc->sDevPAddr);
		break;

	default:
		PVR_DPF((PVR_DBG_ERROR, "_PxMemAlloc: Invalid MMU level"));
		PVR_ASSERT(0);
	}
#else
	PVR_UNREFERENCED_PARAMETER(eMMULevel);
#endif
	/*  free the PC */
	_MMU_PhysMemFree(psMMUContext->psPhysMemCtx, psMemDesc);
}

									    /*************************************************************************//*!
									       @Function       _SetupPxE

									       @Description    Sets up an entery of an MMU object to point to the
									       provided address

									       @Input          psMMUContext    MMU context to operate on

									       @Input          psMemDesc       Allocation description of MMU object

									       @Input          uiIndex         Index into the MMU object to setup

									       @Input          psConfig        MMU Px config

									       @Input          eMMULevel       Level of MMU object

									       @Input          psDevPAddr      Address to setup the MMU object to point to

									       @Input          pszMemspaceName Name of the PDump memory space that the entry
									       will point to

									       @Input          pszSymbolicAddr PDump symbolic address that the entry will
									       point to

									       @Input          uiProtFlags     MMU protection flags

									       @Return         PVRSRV_OK if the setup was successful
									     */
/*****************************************************************************/
static PVRSRV_ERROR _SetupPxE(MMU_CONTEXT * psMMUContext,
			      MMU_MEMORY_DESC * psMemDesc,
			      IMG_UINT32 uiIndex,
			      const MMU_PxE_CONFIG * psConfig,
			      MMU_LEVEL eMMULevel,
			      const IMG_DEV_PHYADDR * psDevPAddr,
#if defined(PDUMP)
			      const IMG_CHAR * pszMemspaceName,
			      const IMG_CHAR * pszSymbolicAddr,
			      IMG_DEVMEM_OFFSET_T uiSymbolicAddrOffset,
#endif
			      MMU_FLAGS_T uiProtFlags)
{
	PVRSRV_DEVICE_NODE *psDevNode = psMMUContext->psDevNode;
	PVRSRV_ERROR eError;

	IMG_UINT32(*pfnDerivePxEProt4) (IMG_UINT32);
	IMG_UINT64(*pfnDerivePxEProt8) (IMG_UINT32);

	if (!psDevPAddr) {
		/* Invalidate entry */
		if (~uiProtFlags & MMU_PROTFLAGS_INVALID) {
			PVR_DPF((PVR_DBG_ERROR,
				 "Error, no physical address specified, but not invalidating entry"));
			uiProtFlags |= MMU_PROTFLAGS_INVALID;
		}
		psDevPAddr = &gsBadDevPhyAddr;
	} else {
		if (uiProtFlags & MMU_PROTFLAGS_INVALID) {
			PVR_DPF((PVR_DBG_ERROR,
				 "A physical address was specified when requesting invalidation of entry"));
			uiProtFlags |= MMU_PROTFLAGS_INVALID;
		}
	}

	switch (eMMULevel) {
	case MMU_LEVEL_3:
		pfnDerivePxEProt4 = psMMUContext->psDevAttrs->pfnDerivePCEProt4;
		pfnDerivePxEProt8 = psMMUContext->psDevAttrs->pfnDerivePCEProt8;
		break;

	case MMU_LEVEL_2:
		pfnDerivePxEProt4 = psMMUContext->psDevAttrs->pfnDerivePDEProt4;
		pfnDerivePxEProt8 = psMMUContext->psDevAttrs->pfnDerivePDEProt8;
		break;

	case MMU_LEVEL_1:
		pfnDerivePxEProt4 = psMMUContext->psDevAttrs->pfnDerivePTEProt4;
		pfnDerivePxEProt8 = psMMUContext->psDevAttrs->pfnDerivePTEProt8;
		break;

	default:
		PVR_DPF((PVR_DBG_ERROR, "_SetupPxE: invalid MMU level"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Map the Page Catalogue into CPU virtual memory */
	/* FIXME: consider making this the caller's
	   responsibility... */
	eError = _MMU_MapCPUVAddr(psMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "_SetupPxE: failed to map Px to CPU"));
		return PVRSRV_ERROR_FAILED_TO_MAP_PAGE_TABLE;
	}

	/* how big is a PxE in bytes? */
	switch (psConfig->uiBytesPerEntry) {
	case 4:
		{
			IMG_UINT32 *pui32Px;
			IMG_UINT64 ui64PxE64;

			pui32Px = psMemDesc->pvCpuVAddr;

			ui64PxE64 = psDevPAddr->uiAddr
			    >> psConfig->uiLog2Align
			    << psConfig->uiAddrShift & psConfig->uiAddrMask;

			ui64PxE64 |= pfnDerivePxEProt4(uiProtFlags);
			/* assert that the result fits into 32 bits before writing
			   it into the 32-bit array with a cast */
			PVR_ASSERT(ui64PxE64 == (ui64PxE64 & 0xffffffffU));

			/* We should never invalidate an invalid page */
			if (uiProtFlags & MMU_PROTFLAGS_INVALID) {
				PVR_ASSERT(pui32Px[uiIndex] != ui64PxE64);
			}
			pui32Px[uiIndex] = (IMG_UINT32) ui64PxE64;
			break;
		}
	case 8:
		{
			IMG_UINT64 *pui64Px = psMemDesc->pvCpuVAddr;
			pui64Px[uiIndex] = psDevPAddr->uiAddr
			    >> psConfig->uiLog2Align
			    << psConfig->uiAddrShift & psConfig->uiAddrMask;
			pui64Px[uiIndex] |= pfnDerivePxEProt8(uiProtFlags);
			break;
		}
	default:
		PVR_DPF((PVR_DBG_ERROR,
			 "_SetupPxE: PxE size not supported (%d) for level %d",
			 psConfig->uiBytesPerEntry, eMMULevel));

		_MMU_UnmapCPUVAddr(psMemDesc);

		return PVRSRV_ERROR_INVALID_PARAMS;
	}

#if defined (PDUMP)
	switch (eMMULevel) {
	case MMU_LEVEL_3:
		PDUMP_MMU_DUMP_PC_ENTRIES(psDevNode->pszMMUPxPDumpMemSpaceName,
					  psMemDesc->pvCpuVAddr,
					  psMemDesc->sDevPAddr,
					  uiIndex,
					  1,
					  psConfig->uiBytesPerEntry,
					  psConfig->uiLog2Align,
					  psConfig->uiAddrShift,
					  psConfig->uiAddrMask,
					  psConfig->uiProtMask, 0);
		break;

	case MMU_LEVEL_2:
		PDUMP_MMU_DUMP_PD_ENTRIES(psDevNode->pszMMUPxPDumpMemSpaceName,
					  psMemDesc->pvCpuVAddr,
					  psMemDesc->sDevPAddr,
					  uiIndex,
					  1,
					  psConfig->uiBytesPerEntry,
					  psConfig->uiLog2Align,
					  psConfig->uiAddrShift,
					  psConfig->uiAddrMask,
					  psConfig->uiProtMask, 0);
		break;

	case MMU_LEVEL_1:
		PDUMP_MMU_DUMP_PT_ENTRIES(psDevNode->pszMMUPxPDumpMemSpaceName,
					  psMemDesc->pvCpuVAddr,
					  psMemDesc->sDevPAddr,
					  uiIndex,
					  1,
					  pszMemspaceName,
					  pszSymbolicAddr,
					  uiSymbolicAddrOffset,
					  psConfig->uiBytesPerEntry,
					  psConfig->uiLog2Align,
					  psConfig->uiAddrShift,
					  psConfig->uiAddrMask,
					  psConfig->uiProtMask, 0);
		break;
	default:
		PVR_DPF((PVR_DBG_ERROR,
			 "_SetupPxE: PxE size not supported (%d) for level %d",
			 psConfig->uiBytesPerEntry, eMMULevel));

		_MMU_UnmapCPUVAddr(psMemDesc);

		return PVRSRV_ERROR_INVALID_PARAMS;
	}
#endif

	psDevNode->pfnMMUCacheInvalidate(psDevNode, psMMUContext->hDevData,
					 eMMULevel,
					 (uiProtFlags & MMU_PROTFLAGS_INVALID) ?
					 IMG_TRUE : IMG_FALSE);

	/* unmap the memory from the CPU (may be a no-op) */
	eError = _MMU_UnmapCPUVAddr(psMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "_SetupPCE: failed to release the CPU mapping"));
		return PVRSRV_ERROR_MMU_FAILED_TO_UNMAP_PAGE_TABLE;
	}

	return PVRSRV_OK;
}

/*****************************************************************************
 *                   MMU host control functions (Level Info)                 *
 *****************************************************************************/

									    /*************************************************************************//*!
									       @Function       _MMU_FreeLevel

									       @Description    Recursivley frees the specified range of Px entries. If any
									       level has it's last reference dropped then the MMU object
									       memory and the MMU_Levelx_Info will be freed.

									       @Input          psMMUContext    MMU context to operate on

									       @Input          psLevel                 Level info on which to to free the
									       specified range

									       @Input          auiStartArray           Array of start indexs (one for each level)

									       @Input          auiEndArray             Array of end indexs (one for each level)

									       @Input          auiEntriesPerPxArray    Array of number of enteries for the Px
									       (one for each level)

									       @Input          apsConfig               Array of PxE configs (one for each level)

									       @Input          aeMMULevel              Array of MMU levels (one for each level)

									       @Input          pui32CurrentLevel       Pointer to a varible which is set to our
									       current level 

									       @Input          uiStartIndex            Start index of the range to free

									       @Input          uiEndIndex              End index of the range to free

									       @Return         IMG_TRUE if the last reference to psLevel was dropped
									     */
/*****************************************************************************/
static IMG_BOOL _MMU_FreeLevel(MMU_CONTEXT * psMMUContext,
			       MMU_Levelx_INFO * psLevel,
			       IMG_UINT32 auiStartArray[],
			       IMG_UINT32 auiEndArray[],
			       IMG_UINT32 auiEntriesPerPxArray[],
			       const MMU_PxE_CONFIG * apsConfig[],
			       MMU_LEVEL aeMMULevel[],
			       IMG_UINT32 * pui32CurrentLevel,
			       IMG_UINT32 uiStartIndex, IMG_UINT32 uiEndIndex)
{
	IMG_UINT32 uiThisLevel = *pui32CurrentLevel;
	const MMU_PxE_CONFIG *psConfig = apsConfig[uiThisLevel];
	IMG_UINT32 i;
	IMG_BOOL bFreed = IMG_FALSE;

	/* Sanity check */
	PVR_ASSERT(*pui32CurrentLevel < MMU_MAX_LEVEL);
	PVR_ASSERT(psLevel != IMG_NULL);

	MMU_OBJ_DBG((PVR_DBG_ERROR,
		     "_MMU_FreeLevel: level = %d, range %d - %d, refcount = %d",
		     aeMMULevel[uiThisLevel], uiStartIndex, uiEndIndex,
		     psLevel->ui32RefCount));

	for (i = uiStartIndex; i < uiEndIndex; i++) {
		if (aeMMULevel[uiThisLevel] != MMU_LEVEL_1) {
			MMU_Levelx_INFO *psNextLevel = psLevel->apsNextLevel[i];
			IMG_UINT32 uiNextStartIndex;
			IMG_UINT32 uiNextEndIndex;

			/* If we're crossing a Px then the start index chanages */
			if (i == uiStartIndex) {
				uiNextStartIndex =
				    auiStartArray[uiThisLevel + 1];
			} else {
				uiNextStartIndex = 0;
			}

			/* If we're crossing a Px then the end index chanages */
			if (i == (uiEndIndex - 1)) {
				uiNextEndIndex = auiEndArray[uiThisLevel + 1];
			} else {
				uiNextEndIndex =
				    auiEntriesPerPxArray[uiThisLevel + 1];
			}

			/* Recurse into the next level */
			(*pui32CurrentLevel)++;
			if (_MMU_FreeLevel
			    (psMMUContext, psNextLevel, auiStartArray,
			     auiEndArray, auiEntriesPerPxArray, apsConfig,
			     aeMMULevel, pui32CurrentLevel, uiNextStartIndex,
			     uiNextEndIndex)) {
				/* The level below us is empty, drop the refcount and clear the pointer */
				psLevel->ui32RefCount--;
				psLevel->apsNextLevel[i] = IMG_NULL;

				/* Check we haven't wraped around */
				PVR_ASSERT(psLevel->ui32RefCount <=
					   psLevel->ui32NumOfEntries);
			}
			(*pui32CurrentLevel)--;
		} else {
			psLevel->ui32RefCount--;
		}

		/*
		   Free this level if it is no longer referenced, unless it's the base
		   level in which case it's part of the MMU context and should be freed
		   when the MMU contexted is freed
		 */
		if ((psLevel->ui32RefCount == 0)
		    && (psLevel != &psMMUContext->sBaseLevelInfo)) {
			/* Level 1 PTE reprogramming is done in the unmap */
			if (aeMMULevel[uiThisLevel] != MMU_LEVEL_1) {
				PVRSRV_ERROR eError;

				/* Wire up the entry */
				eError = _SetupPxE(psMMUContext,
						   &psLevel->sMemDesc,
						   i,
						   psConfig,
						   aeMMULevel[uiThisLevel],
						   IMG_NULL,
#if defined(PDUMP)
						   IMG_NULL,	/* Only required for data page */
						   IMG_NULL,	/* Only required for data page */
						   0,	/* Only required for data page */
#endif
						   MMU_PROTFLAGS_INVALID);	/* Really? Valid is 0?? */

				PVR_ASSERT(eError == PVRSRV_OK);
			}

			_PxMemFree(psMMUContext, &psLevel->sMemDesc,
				   aeMMULevel[uiThisLevel]);
			OSFreeMem(psLevel);
			bFreed = IMG_TRUE;
		}
	}
	MMU_OBJ_DBG((PVR_DBG_ERROR,
		     "_MMU_FreeLevel end: level = %d, refcount = %d",
		     aeMMULevel[uiThisLevel], psLevel->ui32RefCount));

	return bFreed;
}

									    /*************************************************************************//*!
									       @Function       _MMU_AllocLevel

									       @Description    Recursivley allocates the specified range of Px entries. If any
									       level has it's last reference dropped then the MMU object
									       memory and the MMU_Levelx_Info will be freed.

									       @Input          psMMUContext    MMU context to operate on

									       @Input          psLevel                 Level info on which to to free the
									       specified range

									       @Input          auiStartArray           Array of start indexs (one for each level)

									       @Input          auiEndArray             Array of end indexs (one for each level)

									       @Input          auiEntriesPerPxArray    Array of number of enteries for the Px
									       (one for each level)

									       @Input          apsConfig               Array of PxE configs (one for each level)

									       @Input          aeMMULevel              Array of MMU levels (one for each level)

									       @Input          pui32CurrentLevel       Pointer to a varible which is set to our
									       current level 

									       @Input          uiStartIndex            Start index of the range to free

									       @Input          uiEndIndex              End index of the range to free

									       @Return         IMG_TRUE if the last reference to psLevel was dropped
									     */
/*****************************************************************************/
static PVRSRV_ERROR _MMU_AllocLevel(MMU_CONTEXT * psMMUContext,
				    MMU_Levelx_INFO * psLevel,
				    IMG_UINT32 auiStartArray[],
				    IMG_UINT32 auiEndArray[],
				    IMG_UINT32 auiEntriesPerPxArray[],
				    const MMU_PxE_CONFIG * apsConfig[],
				    MMU_LEVEL aeMMULevel[],
				    IMG_UINT32 * pui32CurrentLevel,
				    IMG_UINT32 uiStartIndex,
				    IMG_UINT32 uiEndIndex)
{
	IMG_UINT32 uiThisLevel = *pui32CurrentLevel;
	const MMU_PxE_CONFIG *psConfig = apsConfig[uiThisLevel];
	PVRSRV_ERROR eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	IMG_UINT32 uiAllocState = 99;
	IMG_UINT32 i;

	/* Sanity check */
	PVR_ASSERT(*pui32CurrentLevel < MMU_MAX_LEVEL);

	MMU_OBJ_DBG((PVR_DBG_ERROR,
		     "_MMU_AllocLevel: level = %d, range %d - %d, refcount = %d",
		     aeMMULevel[uiThisLevel], uiStartIndex, uiEndIndex,
		     psLevel->ui32RefCount));

	for (i = uiStartIndex; i < uiEndIndex; i++) {
		/* Only try and allocation if this is not the last level */
		if (aeMMULevel[uiThisLevel] != MMU_LEVEL_1) {
			IMG_UINT32 uiNextStartIndex;
			IMG_UINT32 uiNextEndIndex;

			/* No Px setup for this index, allocate one */
			if (!psLevel->apsNextLevel[i]) {
				MMU_Levelx_INFO *psNextLevel;
				IMG_UINT32 ui32AllocSize;
				IMG_UINT32 uiNextEntries;

				/* Allocate and setup the next level */
				uiNextEntries =
				    auiEntriesPerPxArray[uiThisLevel + 1];
				ui32AllocSize = sizeof(MMU_Levelx_INFO);
				if (aeMMULevel[uiThisLevel + 1] != MMU_LEVEL_1) {
					ui32AllocSize +=
					    sizeof(MMU_Levelx_INFO *) *
					    (uiNextEntries - 1);
				}
				psNextLevel = OSAllocMem(ui32AllocSize);
				if (psNextLevel == IMG_NULL) {
					uiAllocState = 0;
					goto e0;
				}
				OSMemSet(psNextLevel, 0, ui32AllocSize);

				/* Hook in this level for next time */
				psLevel->apsNextLevel[i] = psNextLevel;

				psNextLevel->ui32NumOfEntries = uiNextEntries;
				psNextLevel->ui32RefCount = 0;
				/* Allocate Px memory */
				eError =
				    _PxMemAlloc(psMMUContext, uiNextEntries,
						apsConfig[uiThisLevel + 1],
						aeMMULevel[uiThisLevel + 1],
						&psNextLevel->sMemDesc);
				if (eError != PVRSRV_OK) {
					uiAllocState = 1;
					goto e0;
				}

				/* Wire up the entry */
				eError = _SetupPxE(psMMUContext,
						   &psLevel->sMemDesc,
						   i,
						   psConfig,
						   aeMMULevel[uiThisLevel],
						   &psNextLevel->sMemDesc.
						   sDevPAddr,
#if defined(PDUMP)
						   IMG_NULL,	/* Only required for data page */
						   IMG_NULL,	/* Only required for data page */
						   0,	/* Only required for data page */
#endif
						   0);	/* Really? Valid is 0?? */
				if (eError != PVRSRV_OK) {
					uiAllocState = 1;
					goto e0;
				}

				psLevel->ui32RefCount++;
			}

			/* If we're crossing a Px then the start index chanages */
			if (i == uiStartIndex) {
				uiNextStartIndex =
				    auiStartArray[uiThisLevel + 1];
			} else {
				uiNextStartIndex = 0;
			}

			/* If we're crossing a Px then the end index chanages */
			if (i == (uiEndIndex - 1)) {
				uiNextEndIndex = auiEndArray[uiThisLevel + 1];
			} else {
				uiNextEndIndex =
				    auiEntriesPerPxArray[uiThisLevel + 1];
			}

			/* Recurse into the next level */
			(*pui32CurrentLevel)++;
			eError =
			    _MMU_AllocLevel(psMMUContext,
					    psLevel->apsNextLevel[i],
					    auiStartArray, auiEndArray,
					    auiEntriesPerPxArray, apsConfig,
					    aeMMULevel, pui32CurrentLevel,
					    uiNextStartIndex, uiNextEndIndex);
			if (eError != PVRSRV_OK) {
				uiAllocState = 2;
				goto e0;
			}
			(*pui32CurrentLevel)--;
		} else {
			/* All we need to do for level 1 is bump the refcount */
			psLevel->ui32RefCount++;
		}
		PVR_ASSERT(psLevel->ui32RefCount <= psLevel->ui32NumOfEntries);
	}
	MMU_OBJ_DBG((PVR_DBG_ERROR,
		     "_MMU_AllocLevel end: level = %d, refcount = %d",
		     aeMMULevel[uiThisLevel], psLevel->ui32RefCount));
	return PVRSRV_OK;

 e0:
	/* Sanity check that we've not come down this route unexpectedly */
	PVR_ASSERT(uiAllocState == 99);
	PVR_DPF((PVR_DBG_ERROR,
		 "_MMU_AllocLevel: Error %d alllocating Px for level %d in stage %d",
		 eError, aeMMULevel[uiThisLevel], uiAllocState));

	for (i = i; i != uiStartIndex; i--) {
		switch (uiAllocState) {
			IMG_UINT32 uiNextStartIndex;
			IMG_UINT32 uiNextEndIndex;

		case 3:
			/* If we're crossing a Px then the start index chanages */
			if (i == uiStartIndex) {
				uiNextStartIndex =
				    auiStartArray[uiThisLevel + 1];
			} else {
				uiNextStartIndex = 0;
			}

			/* If we're crossing a Px then the end index chanages */
			if (i == (uiEndIndex - 1)) {
				uiNextEndIndex = auiEndArray[uiThisLevel + 1];
			} else {
				uiNextEndIndex =
				    auiEntriesPerPxArray[uiThisLevel + 1];
			}

			if (aeMMULevel[uiThisLevel] != MMU_LEVEL_1) {
				if (_MMU_FreeLevel
				    (psMMUContext, psLevel->apsNextLevel[i],
				     auiStartArray, auiEndArray,
				     auiEntriesPerPxArray, apsConfig,
				     aeMMULevel, pui32CurrentLevel,
				     uiNextStartIndex, uiNextEndIndex)) {
					psLevel->ui32RefCount--;
					psLevel->apsNextLevel[i] = IMG_NULL;

					/* Check we haven't wraped around */
					PVR_ASSERT(psLevel->ui32RefCount <=
						   psLevel->ui32NumOfEntries);
				}
			} else {
				/* We should never come down this path, but it's here
				   for completeness */
				psLevel->ui32RefCount--;

				/* Check we haven't wraped around */
				PVR_ASSERT(psLevel->ui32RefCount <=
					   psLevel->ui32NumOfEntries);
			}
		case 2:
			if (psLevel->ui32RefCount == 0) {
				_PxMemFree(psMMUContext, &psLevel->sMemDesc,
					   aeMMULevel[uiThisLevel]);
			}
		case 1:
			if (psLevel->ui32RefCount == 0) {
				OSFreeMem(psLevel->apsNextLevel[i]);
			}
		case 0:
			uiAllocState = 3;
			break;
		}
	}
	return eError;
}

/*****************************************************************************
 *                   MMU page table functions                                *
 *****************************************************************************/

									    /*************************************************************************//*!
									       @Function       _MMU_GetLevelData

									       @Description    Get the all the level data and calculates the indexes for the
									       specified address range

									       @Input          psMMUContext            MMU context to operate on

									       @Input          sDevVAddrStart          Start device virtual address

									       @Input          sDevVAddrEnd            End device virtual address

									       @Input          uiLog2DataPageSize      Log2 of the page size to use

									       @Input          auiStartArray           Array of start indexes (one for each level)

									       @Input          auiEndArray             Array of end indexes (one for each level)

									       @Input          uiEntriesPerPxArray     Array of number of enteries for the Px
									       (one for each level)

									       @Input          apsConfig               Array of PxE configs (one for each level)

									       @Input          aeMMULevel              Array of MMU levels (one for each level)
									       @Input          apsConfig               Array of PxE configs (one for each level)

									       @Input          aeMMULevel              Array of MMU levels (one for each level)

									       @Input          pui32CurrentLevel       Pointer to a variable which is set to our
									       current level 

									       @Input          uiStartIndex            Start index of the range to free

									       @Input          uiEndIndex              End index of the range to free

									       @Return         IMG_TRUE if the last reference to psLevel was dropped
									     */
/*****************************************************************************/
static IMG_VOID _MMU_GetLevelData(MMU_CONTEXT * psMMUContext,
				  IMG_DEV_VIRTADDR sDevVAddrStart,
				  IMG_DEV_VIRTADDR sDevVAddrEnd,
				  IMG_UINT32 uiLog2DataPageSize,
				  IMG_UINT32 auiStartArray[],
				  IMG_UINT32 auiEndArray[],
				  IMG_UINT32 auiEntriesPerPx[],
				  const MMU_PxE_CONFIG * apsConfig[],
				  MMU_LEVEL aeMMULevel[],
				  const MMU_DEVVADDR_CONFIG **
				  ppsMMUDevVAddrConfig, IMG_HANDLE * phPriv)
{
	const MMU_PxE_CONFIG *psMMUPDEConfig;
	const MMU_PxE_CONFIG *psMMUPTEConfig;
	const MMU_DEVVADDR_CONFIG *psDevVAddrConfig;
	MMU_DEVICEATTRIBS *psDevAttrs = psMMUContext->psDevAttrs;
	PVRSRV_ERROR eError;
	IMG_UINT32 i = 0;

	eError = psDevAttrs->pfnGetPageSizeConfiguration(uiLog2DataPageSize,
							 &psMMUPDEConfig,
							 &psMMUPTEConfig,
							 ppsMMUDevVAddrConfig,
							 phPriv);
	PVR_ASSERT(eError == PVRSRV_OK);

	psDevVAddrConfig = *ppsMMUDevVAddrConfig;

	if (psDevVAddrConfig->uiPCIndexMask != 0) {
		auiStartArray[i] =
		    _CalcPCEIdx(sDevVAddrStart, psDevVAddrConfig, IMG_FALSE);
		auiEndArray[i] =
		    _CalcPCEIdx(sDevVAddrEnd, psDevVAddrConfig, IMG_TRUE);
		auiEntriesPerPx[i] =
		    (IMG_UINT32) UNITS_IN_BITFIELD(psDevVAddrConfig->
						   uiPCIndexMask,
						   psDevVAddrConfig->
						   uiPCIndexShift);
		apsConfig[i] = psDevAttrs->psBaseConfig;
		aeMMULevel[i] = MMU_LEVEL_3;
		i++;
	}

	if (psDevVAddrConfig->uiPDIndexMask != 0) {
		auiStartArray[i] =
		    _CalcPDEIdx(sDevVAddrStart, psDevVAddrConfig, IMG_FALSE);
		auiEndArray[i] =
		    _CalcPDEIdx(sDevVAddrEnd, psDevVAddrConfig, IMG_TRUE);
		auiEntriesPerPx[i] =
		    (IMG_UINT32) UNITS_IN_BITFIELD(psDevVAddrConfig->
						   uiPDIndexMask,
						   psDevVAddrConfig->
						   uiPDIndexShift);
		if (i == 0) {
			apsConfig[i] = psDevAttrs->psBaseConfig;
		} else {
			apsConfig[i] = psMMUPDEConfig;
		}
		aeMMULevel[i] = MMU_LEVEL_2;
		i++;
	}

	if (psDevVAddrConfig->uiPTIndexMask != 0) {
		auiStartArray[i] =
		    _CalcPTEIdx(sDevVAddrStart, psDevVAddrConfig, IMG_FALSE);
		auiEndArray[i] =
		    _CalcPTEIdx(sDevVAddrEnd, psDevVAddrConfig, IMG_TRUE);
		auiEntriesPerPx[i] =
		    (IMG_UINT32) UNITS_IN_BITFIELD(psDevVAddrConfig->
						   uiPTIndexMask,
						   psDevVAddrConfig->
						   uiPTIndexShift);
		if (i == 0) {
			apsConfig[i] = psDevAttrs->psBaseConfig;
		} else {
			apsConfig[i] = psMMUPTEConfig;
		}
		aeMMULevel[i] = MMU_LEVEL_1;
		i++;
	}

	PVR_ASSERT(i != 0);
}

static IMG_VOID _MMU_PutLevelData(MMU_CONTEXT * psMMUContext, IMG_HANDLE hPriv)
{
	MMU_DEVICEATTRIBS *psDevAttrs = psMMUContext->psDevAttrs;

	psDevAttrs->pfnPutPageSizeConfiguration(hPriv);
}

									    /*************************************************************************//*!
									       @Function       _AllocPageTables

									       @Description    Allocate page tables and any higher level MMU objects required
									       for the specified virtual range

									       @Input          psMMUContext            MMU context to operate on

									       @Input          sDevVAddrStart          Start device virtual address

									       @Input          sDevVAddrEnd            End device virtual address

									       @Input          uiProtFlags             Generic MMU protoecton flags

									       @Return         PVRSRV_OK if the allocation was successful
									     */
/*****************************************************************************/
static PVRSRV_ERROR
_AllocPageTables(MMU_CONTEXT * psMMUContext,
		 IMG_DEV_VIRTADDR sDevVAddrStart,
		 IMG_DEV_VIRTADDR sDevVAddrEnd, MMU_FLAGS_T uiProtFlags)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 auiStartArray[MMU_MAX_LEVEL];
	IMG_UINT32 auiEndArray[MMU_MAX_LEVEL];
	IMG_UINT32 auiEntriesPerPx[MMU_MAX_LEVEL];
	MMU_LEVEL aeMMULevel[MMU_MAX_LEVEL];
	const MMU_PxE_CONFIG *apsConfig[MMU_MAX_LEVEL];
	const MMU_DEVVADDR_CONFIG *psDevVAddrConfig;
	IMG_HANDLE hPriv;
	IMG_UINT32 ui32CurrentLevel = 0;

	/* FIXME: To support variable page size this needs to be passed in from above */
	IMG_UINT32 uiLog2PageSize = 12;

	PVR_DPF((PVR_DBG_ALLOC,
		 "_AllocPageTables: vaddr range: 0x%010llx:0x%010llx",
		 sDevVAddrStart.uiAddr, sDevVAddrEnd.uiAddr));

#if defined(PDUMP)
	PDUMPCOMMENT
	    ("Allocating page tables for virtual range: 0x%010llX to 0x%010llX",
	     (IMG_UINT64) sDevVAddrStart.uiAddr,
	     (IMG_UINT64) sDevVAddrEnd.uiAddr);
#endif

	_MMU_GetLevelData(psMMUContext, sDevVAddrStart, sDevVAddrEnd,
			  uiLog2PageSize, auiStartArray, auiEndArray,
			  auiEntriesPerPx, apsConfig, aeMMULevel,
			  &psDevVAddrConfig, &hPriv);

	eError = _MMU_AllocLevel(psMMUContext, &psMMUContext->sBaseLevelInfo,
				 auiStartArray, auiEndArray, auiEntriesPerPx,
				 apsConfig, aeMMULevel, &ui32CurrentLevel,
				 auiStartArray[0], auiEndArray[0]);

	_MMU_PutLevelData(psMMUContext, hPriv);

	return eError;
}

									    /*************************************************************************//*!
									       @Function       _FreePageTables

									       @Description    Free page tables and any higher level MMU objects at are no
									       longer referenced for the specified virtual range

									       @Input          psMMUContext            MMU context to operate on

									       @Input          sDevVAddrStart          Start device virtual address

									       @Input          sDevVAddrEnd            End device virtual address

									       @Return         None
									     */
/*****************************************************************************/
static IMG_VOID _FreePageTables(MMU_CONTEXT * psMMUContext,
				IMG_DEV_VIRTADDR sDevVAddrStart,
				IMG_DEV_VIRTADDR sDevVAddrEnd)
{
	IMG_UINT32 auiStartArray[MMU_MAX_LEVEL];
	IMG_UINT32 auiEndArray[MMU_MAX_LEVEL];
	IMG_UINT32 auiEntriesPerPx[MMU_MAX_LEVEL];
	MMU_LEVEL aeMMULevel[MMU_MAX_LEVEL];
	const MMU_PxE_CONFIG *apsConfig[MMU_MAX_LEVEL];
	const MMU_DEVVADDR_CONFIG *psDevVAddrConfig;
	IMG_UINT32 ui32CurrentLevel = 0;
	IMG_HANDLE hPriv;

	/* FIXME: To support variable page size this needs to be passed in from above */
	IMG_UINT32 uiLog2PageSize = 12;

	PVR_DPF((PVR_DBG_ALLOC,
		 "_FreePageTables: vaddr range: 0x%010llx:0x%010llx",
		 sDevVAddrStart.uiAddr, sDevVAddrEnd.uiAddr));

	_MMU_GetLevelData(psMMUContext, sDevVAddrStart, sDevVAddrEnd,
			  uiLog2PageSize, auiStartArray, auiEndArray,
			  auiEntriesPerPx, apsConfig, aeMMULevel,
			  &psDevVAddrConfig, &hPriv);

	_MMU_FreeLevel(psMMUContext, &psMMUContext->sBaseLevelInfo,
		       auiStartArray, auiEndArray, auiEntriesPerPx,
		       apsConfig, aeMMULevel, &ui32CurrentLevel,
		       auiStartArray[0], auiEndArray[0]);

	_MMU_PutLevelData(psMMUContext, hPriv);
}

									    /*************************************************************************//*!
									       @Function       _MMU_GetPTEInfo

									       @Description    Get the PTE config and level information for the specificed
									       virtual address

									       @Input          psMMUContext            MMU context to operate on

									       @Input          psDevVAddr              Device virtual address to get the PTE info
									       for

									       @Input          uiLog2DataPageSize      Log 2 of the page size

									       @Output         psLevel                 Level info of the PT

									       @Output         pui32PTEIndex           Index into the PT the address corresponds to

									       @Output         ppsConfig               Config of the PTE

									       @Output         phPriv                  Private data handle to be passed back
									       when the info is put

									       @Return         None
									     */
/*****************************************************************************/
static IMG_VOID _MMU_GetPTEInfo(MMU_CONTEXT * psMMUContext,
				IMG_DEV_VIRTADDR sDevVAddr,
				IMG_UINT32 uiLog2DataPageSize,
				MMU_Levelx_INFO ** psLevel,
				IMG_UINT32 * pui32PTEIndex,
				const MMU_PxE_CONFIG ** ppsConfig,
				IMG_HANDLE * phPriv)
{
	MMU_DEVICEATTRIBS *psDevAttrs = psMMUContext->psDevAttrs;
	MMU_Levelx_INFO *psLocalLevel = IMG_NULL;

	/* FIXME: I don't want all of these, but the API assumes I do */
	const MMU_DEVVADDR_CONFIG *psDevVAddrConfig;
	const MMU_PxE_CONFIG *psPDEConfig;
	const MMU_PxE_CONFIG *psPTEConfig;

	IMG_UINT32 uiPCEIndex;
	IMG_UINT32 uiPDEIndex;

	if (psDevAttrs->pfnGetPageSizeConfiguration(uiLog2DataPageSize,
						    &psPDEConfig,
						    &psPTEConfig,
						    &psDevVAddrConfig,
						    phPriv) != PVRSRV_OK) {
		/*
		   There should be no way we got here unless uiLog2DataPageSize
		   has changed after the MMU_Alloc call (in which case it's a bug in
		   the MM code)
		 */
		PVR_ASSERT(0);
	}

	switch (psMMUContext->psDevAttrs->eTopLevel) {
	case MMU_LEVEL_3:
		/* find the page directory containing the PCE */
		uiPCEIndex =
		    _CalcPCEIdx(sDevVAddr, psDevVAddrConfig, IMG_FALSE);
		psLocalLevel =
		    psMMUContext->sBaseLevelInfo.apsNextLevel[uiPCEIndex];

	case MMU_LEVEL_2:
		/* find the page table containing the PDE */
		uiPDEIndex =
		    _CalcPDEIdx(sDevVAddr, psDevVAddrConfig, IMG_FALSE);
		if (psLocalLevel != IMG_NULL) {
			psLocalLevel = psLocalLevel->apsNextLevel[uiPDEIndex];
		} else {
			psLocalLevel =
			    psMMUContext->sBaseLevelInfo.
			    apsNextLevel[uiPDEIndex];
		}

	case MMU_LEVEL_1:
		/* find PTE index into page table */
		*pui32PTEIndex =
		    _CalcPTEIdx(sDevVAddr, psDevVAddrConfig, IMG_FALSE);
		if (psLocalLevel == IMG_NULL) {
			psLocalLevel = &psMMUContext->sBaseLevelInfo;
		}
		break;

	default:
		PVR_DPF((PVR_DBG_ERROR, "_MMU_GetPTEInfo: Invalid MMU level"));
		return;
	}

	*psLevel = psLocalLevel;
	*ppsConfig = psPTEConfig;
}

									    /*************************************************************************//*!
									       @Function       _MMU_PutPTEInfo

									       @Description    Put the level info.

									       @Input          psMMUContext            MMU context to operate on

									       @Input          phPriv                  Private data handle

									       @Return         None
									     */
/*****************************************************************************/
static IMG_VOID _MMU_PutPTEInfo(MMU_CONTEXT * psMMUContext, IMG_HANDLE hPriv)
{
	MMU_DEVICEATTRIBS *psDevAttrs = psMMUContext->psDevAttrs;

	psDevAttrs->pfnPutPageSizeConfiguration(hPriv);
}

									    /*************************************************************************//*!
									       @Function       _MMU_MapPage

									       @Description    Map a page into the MMU. MMU_Alloc must have been called before
									       this function to allocate the page tables.

									       @Input          psMMUContext            MMU context to operate on

									       @Input          psDevVAddr              Device virtual address to map the page
									       into

									       @Input          sDevPAddr               Device physical address of the memory
									       to map

									       @Output         pszMemspaceName         PDump memory space name of the memory
									       to map

									       @Output         pszSymbolicAddr         PDump symbolic address of the memory to
									       map

									       @Output         uiSymbolicAddrOffset    Offset from the PDump symbolic address
									       of the memory to map

									       @Output         uiProtFlags             Generic MMU protection flags

									       @Return         None
									     */
/*****************************************************************************/
static IMG_VOID
_MMU_MapPage(MMU_CONTEXT * psMMUContext,
	     IMG_DEV_VIRTADDR sDevVAddr, IMG_DEV_PHYADDR sDevPAddr,
#if defined(PDUMP)
	     const IMG_CHAR * pszMemspaceName,
	     const IMG_CHAR * pszSymbolicAddr,
	     IMG_DEVMEM_OFFSET_T uiSymbolicAddrOffset,
#endif
	     MMU_FLAGS_T uiProtFlags)
{
	const MMU_PxE_CONFIG *psConfig;
	MMU_Levelx_INFO *psLevel;
	PVRSRV_ERROR eError;
	IMG_UINT32 uiPTEIndex;
	IMG_HANDLE hPriv;

	/* FIXME: This should be passed in */
	IMG_UINT32 uiLog2DataPageSize = 12;

	_MMU_GetPTEInfo(psMMUContext, sDevVAddr,
			uiLog2DataPageSize, &psLevel, &uiPTEIndex, &psConfig,
			&hPriv);

	eError = _SetupPxE(psMMUContext, &psLevel->sMemDesc, uiPTEIndex,
			   psConfig, MMU_LEVEL_1, &sDevPAddr,
#if defined(PDUMP)
			   pszMemspaceName, pszSymbolicAddr,
			   uiSymbolicAddrOffset,
#endif
			   uiProtFlags);

	_MMU_PutPTEInfo(psMMUContext, hPriv);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "_MMU_MapPage: _SetupPxE failed"));
		PVR_ASSERT(0);
		return;
	}

	PVR_ASSERT(psLevel->ui32RefCount <= psLevel->ui32NumOfEntries);
}

									    /*************************************************************************//*!
									       @Function       _MMU_UnmapPage

									       @Description    Unmap a page from the MMU.

									       @Input          psMMUContext            MMU context to operate on

									       @Input          psDevVAddr              Device virtual address to unmap the page
									       from

									       @Return         None
									     */
/*****************************************************************************/
static IMG_VOID
_MMU_UnmapPage(MMU_CONTEXT * psMMUContext, IMG_DEV_VIRTADDR sDevVAddr)
{
	const MMU_PxE_CONFIG *psConfig = IMG_NULL;
	MMU_Levelx_INFO *psLevel;
	PVRSRV_ERROR eError;
	IMG_UINT32 uiPTEIndex;
	IMG_HANDLE hPriv;

	/* FIXME: This should be passed in */
	IMG_UINT32 uiLog2DataPageSize = 12;

	_MMU_GetPTEInfo(psMMUContext, sDevVAddr, uiLog2DataPageSize,
			&psLevel, &uiPTEIndex, &psConfig, &hPriv);

	eError =
	    _SetupPxE(psMMUContext, &psLevel->sMemDesc, uiPTEIndex, psConfig,
		      MMU_LEVEL_1, IMG_NULL,
#if defined(PDUMP)
		      IMG_NULL, IMG_NULL, 0U,
#endif
		      MMU_PROTFLAGS_INVALID);

	_MMU_PutPTEInfo(psMMUContext, hPriv);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "MMU_UnmapPage: _SetupPxE failed"));
		PVR_ASSERT(0);
		return;		//eError;
	}

	/* Check we haven't wraped around */
	PVR_ASSERT(psLevel->ui32RefCount <= psLevel->ui32NumOfEntries);
}

/*****************************************************************************
 *                     Public interface functions                            *
 *****************************************************************************/

/*
	MMU_ContextCreate
*/
PVRSRV_ERROR
MMU_ContextCreate(PVRSRV_DEVICE_NODE * psDevNode, MMU_CONTEXT ** ppsMMUContext)
{
	MMU_CONTEXT *psMMUContext;
	MMU_DEVICEATTRIBS *psDevAttrs;
	const MMU_DEVVADDR_CONFIG *psDevVAddrConfig;
	const MMU_PxE_CONFIG *psConfig;
	MMU_PHYSMEM_CONTEXT *psCtx;
	IMG_UINT32 ui32BaseObjects;
	IMG_UINT32 ui32Size;
	IMG_CHAR sBuf[40];
	PVRSRV_ERROR eError = PVRSRV_OK;

	psDevAttrs = psDevNode->psMMUDevAttrs;
	psConfig = psDevAttrs->psBaseConfig;
	psDevVAddrConfig = psDevAttrs->psTopLevelDevVAddrConfig;

	switch (psDevAttrs->eTopLevel) {
	case MMU_LEVEL_3:
		ui32BaseObjects =
		    UNITS_IN_BITFIELD(psDevVAddrConfig->uiPCIndexMask,
				      psDevVAddrConfig->uiPCIndexShift);
		break;

	case MMU_LEVEL_2:
		ui32BaseObjects =
		    UNITS_IN_BITFIELD(psDevVAddrConfig->uiPDIndexMask,
				      psDevVAddrConfig->uiPDIndexShift);
		break;

	case MMU_LEVEL_1:
		ui32BaseObjects =
		    UNITS_IN_BITFIELD(psDevVAddrConfig->uiPTIndexMask,
				      psDevVAddrConfig->uiPTIndexShift);
		break;

	default:
		PVR_DPF((PVR_DBG_ERROR,
			 "MMU_ContextCreate: Invalid MMU config"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	/* Allocate the MMU context with the Level 1 Px info's */
	ui32Size = sizeof(MMU_CONTEXT) +
	    ((ui32BaseObjects - 1) * sizeof(MMU_Levelx_INFO *));

	psMMUContext = OSAllocMem(ui32Size);
	if (psMMUContext == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "MMU_ContextCreate: ERROR call to OSAllocMem failed"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	OSMemSet(psMMUContext, 0, ui32Size);

#if defined(PDUMP)
	/* Clear the refcount */
	psMMUContext->ui32PDumpContextIDRefCount = 0;
#endif
	/* Record Device specific attributes in the context for subsequent use */
	psMMUContext->psDevAttrs = psDevAttrs;
	psMMUContext->psDevNode = psDevNode;

	/* 
	   Allocate physmem context and set it up
	 */
	psCtx = OSAllocMem(sizeof(MMU_PHYSMEM_CONTEXT));
	if (psCtx == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "MMU_ContextCreate: ERROR call to OSAllocMem failed"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e1;
	}
	psMMUContext->psPhysMemCtx = psCtx;

	OSMemSet(psCtx, 0, sizeof(MMU_PHYSMEM_CONTEXT));
	psCtx->psDevNode = psDevNode;

	OSSNPrintf(sBuf, sizeof(sBuf) - 1, "pgtables %p", psCtx);
	psCtx->uiPhysMemRANameAllocSize = OSStringLength(sBuf) + 1;
	psCtx->pszPhysMemRAName = OSAllocMem(psCtx->uiPhysMemRANameAllocSize);
	if (psCtx->pszPhysMemRAName == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR, "MMU_ContextCreate: Out of memory"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e2;
	}

	OSStringCopy(psCtx->pszPhysMemRAName, sBuf);

	psCtx->psPhysMemRA = RA_Create(psCtx->pszPhysMemRAName,
				       /* "initial" import (is none!) */
				       0,	/* base */
				       0,	/* len */
				       0,	/* flags */
				       IMG_NULL,	/* priv */
				       /* subsequent import */
				       psDevNode->uiMMUPxLog2AllocGran,
				       _MMU_PhysMem_RAImportAlloc,
				       _MMU_PhysMem_RAImportFree,
				       psCtx /* priv */ );
	if (psCtx->psPhysMemRA == IMG_NULL) {
		OSFreeMem(psCtx->pszPhysMemRAName);
		psCtx->pszPhysMemRAName = IMG_NULL;
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e3;
	}

	/* allocate the base level object */
	/*
	   Note: Although this is not required by the this file until
	   the 1st allocation is made, a device specific callback
	   might request the base object address so we allocate
	   it up front.
	 */
	if (_PxMemAlloc(psMMUContext,
			ui32BaseObjects,
			psConfig,
			psDevAttrs->eTopLevel,
			&psMMUContext->sBaseLevelInfo.sMemDesc)) {
		PVR_DPF((PVR_DBG_ERROR,
			 "MMU_ContextCreate: Failed to alloc level 1 object"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e4;
	}

	psMMUContext->sBaseLevelInfo.ui32NumOfEntries = ui32BaseObjects;
	psMMUContext->sBaseLevelInfo.ui32RefCount = 0;

	/* return context */
	*ppsMMUContext = psMMUContext;

	return PVRSRV_OK;

 e4:
	RA_Delete(psCtx->psPhysMemRA);
 e3:
	OSFreeMem(psCtx->pszPhysMemRAName);
 e2:
	OSFreeMem(psCtx);
 e1:
	OSFreeMem(psMMUContext);
 e0:
	return eError;
}

/*
	MMU_ContextDestroy
*/
IMG_VOID MMU_ContextDestroy(MMU_CONTEXT * psMMUContext)
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();

	PVR_DPF((PVR_DBG_MESSAGE, "MMU_ContextDestroy: Enter"));

	if (psPVRSRVData->eServicesState == PVRSRV_SERVICES_STATE_OK) {
		/* There should be no way to get here with live pages unless
		   there is a bug in this module or the MM code */
		PVR_ASSERT(psMMUContext->sBaseLevelInfo.ui32RefCount == 0);
	}

	/* Free the top level MMU object */
	_PxMemFree(psMMUContext,
		   &psMMUContext->sBaseLevelInfo.sMemDesc,
		   psMMUContext->psDevAttrs->eTopLevel);

	/* Free physmem context */
	RA_Delete(psMMUContext->psPhysMemCtx->psPhysMemRA);
	psMMUContext->psPhysMemCtx->psPhysMemRA = IMG_NULL;
	OSFreeMem(psMMUContext->psPhysMemCtx->pszPhysMemRAName);
	psMMUContext->psPhysMemCtx->pszPhysMemRAName = IMG_NULL;

	OSFreeMem(psMMUContext->psPhysMemCtx);

	/* free the context itself. */
	OSFreeMem(psMMUContext);
	/*not nulling pointer, copy on stack */

	PVR_DPF((PVR_DBG_MESSAGE, "MMU_ContextDestroy: Exit"));
}

/*
	MMU_Alloc
*/
PVRSRV_ERROR
MMU_Alloc(MMU_CONTEXT * psMMUContext,
	  IMG_DEVMEM_SIZE_T uSize,
	  IMG_DEVMEM_SIZE_T * puActualSize,
	  IMG_UINT32 uiProtFlags,
	  IMG_DEVMEM_SIZE_T uDevVAddrAlignment, IMG_DEV_VIRTADDR * psDevVAddr)
{
	PVRSRV_ERROR eError;
	IMG_DEV_VIRTADDR sDevVAddrEnd;

	/* FIXME: This should be passed in */
	IMG_UINT32 uiLog2DataPageSize = 12;

	/* FIXME: I don't want all of these, but the current API assumes I do */
	const MMU_PxE_CONFIG *psPDEConfig;
	const MMU_PxE_CONFIG *psPTEConfig;
	const MMU_DEVVADDR_CONFIG *psDevVAddrConfig;

	MMU_DEVICEATTRIBS *psDevAttrs = psMMUContext->psDevAttrs;
	IMG_HANDLE hPriv;

	PVR_DPF((PVR_DBG_MESSAGE,
		 "MMU_Alloc: uSize=0x%010llx, uiProtFlags=0x%x, align=0x%010llx",
		 uSize, uiProtFlags, uDevVAddrAlignment));

	/* check params */
	if (!psMMUContext || !psDevVAddr || !puActualSize) {
		PVR_DPF((PVR_DBG_ERROR, "MMU_Alloc: invalid params"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	eError = psDevAttrs->pfnGetPageSizeConfiguration(uiLog2DataPageSize,
							 &psPDEConfig,
							 &psPTEConfig,
							 &psDevVAddrConfig,
							 &hPriv);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "MMU_Alloc: Failed to get config info (%d)", eError));
		return eError;
	}

	/* size and alignment must be datapage granular */
	if (((psDevVAddr->uiAddr & psDevVAddrConfig->uiPageOffsetMask) != 0)
	    || ((uSize & psDevVAddrConfig->uiPageOffsetMask) != 0)) {
		PVR_DPF((PVR_DBG_ERROR,
			 "MMU_Alloc: invalid address or size granularity"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	sDevVAddrEnd = *psDevVAddr;
	sDevVAddrEnd.uiAddr += uSize;
	eError =
	    _AllocPageTables(psMMUContext, *psDevVAddr, sDevVAddrEnd,
			     uiProtFlags);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "MMU_Alloc: _DeferredAllocPagetables failed"));
		return PVRSRV_ERROR_MMU_FAILED_TO_ALLOCATE_PAGETABLES;
	}

	psDevAttrs->pfnPutPageSizeConfiguration(hPriv);

	return PVRSRV_OK;
}

/*
	MMU_Free
*/
IMG_VOID
MMU_Free(MMU_CONTEXT * psMMUContext, IMG_DEV_VIRTADDR sDevVAddr,
	 IMG_DEVMEM_SIZE_T uiSize)
{
	IMG_DEV_VIRTADDR sDevVAddrEnd;

	if (psMMUContext == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR, "MMU_Free: invalid parameter"));
		return;
	}

	PVR_DPF((PVR_DBG_MESSAGE, "MMU_Free: Freeing DevVAddr 0x%010llX",
		 sDevVAddr.uiAddr));

	/* ensure the address range to free is inside the heap */
	sDevVAddrEnd = sDevVAddr;
	sDevVAddrEnd.uiAddr += uiSize;

	_FreePageTables(psMMUContext, sDevVAddr, sDevVAddrEnd);
}

/*
	MMU_UnmapPages
*/
IMG_VOID
MMU_UnmapPages(MMU_CONTEXT * psMMUContext,
	       IMG_DEV_VIRTADDR sDevVAddr, IMG_UINT32 ui32PageCount)
{
	/* FIXME: This should be passed in */
	IMG_UINT32 uiLog2DataPageSize = 12;
	IMG_UINT32 uiPageSize = 1 << uiLog2DataPageSize;

#if defined PDUMP
	PDUMPCOMMENT
	    ("Invalidate the entry in %d Page Tables that referred to the Data Page",
	     ui32PageCount);
#endif
	while (ui32PageCount != 0) {
		_MMU_UnmapPage(psMMUContext, sDevVAddr);
		sDevVAddr.uiAddr += uiPageSize;
		ui32PageCount--;
	}
}

/*
	MMU_MapPMR
*/
PVRSRV_ERROR
MMU_MapPMR(MMU_CONTEXT * psMMUContext,
	   IMG_DEV_VIRTADDR sDevVAddr,
	   const PMR * psPMR,
	   IMG_DEVMEM_SIZE_T uiSizeBytes, PVRSRV_MEMALLOCFLAGS_T uiMappingFlags)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 uiCount, i;
#if defined (MAPPING_PRIV_CHECK)
	PMR_FLAGS_T uiPMRFlags;
#endif
#if defined (MAPPING_PRIV_CHECK)
	MMU_PROTFLAGS_T uiProtFlags;
#endif
	IMG_DEV_PHYADDR sDevPAddr;
#if defined(PDUMP)
	IMG_CHAR aszMemspaceName[PMR_MAX_MEMSPACE_NAME_LENGTH_DEFAULT];
	IMG_CHAR aszSymbolicAddress[PMR_MAX_SYMBOLIC_ADDRESS_LENGTH_DEFAULT];
	IMG_DEVMEM_OFFSET_T uiSymbolicAddrOffset;
#endif	 /*PDUMP*/
	    PVRSRV_MEMALLOCFLAGS_T uiMMUProtFlags = 0;

	/* FIXME: This should be passed in */
	IMG_UINT32 uiLog2DataPageSize = 12;
	IMG_UINT32 uiPageSize = 1 << uiLog2DataPageSize;

	PVR_ASSERT(psMMUContext != IMG_NULL);

#if defined (MAPPING_PRIV_CHECK)
	/* N.B.  The caller _must_ have already called
	   "PMRLockSysPhysAddr()" on this PMR _before_ calling MMU_MapPMR.
	   Why?  (i) Because we really want this module to concentrate on
	   page table management, and interacts the absolute minimum with
	   the PMR; and (ii) because in the future we may map PMRs in
	   partially (e.g. demand-paging scenario) and it would not be
	   right to call locksysphysaddr on each individual mapping; and
	   (iii) we've already got "unmap pages" where we don't have the
	   PMR handle (we could change the API, but I can't justify this
	   just for this).  However, it may be worth rethinking this,
	   because we'll eventually want to support mixed page sizes
	   within one allocation (rather than it being a heap attribute)
	   so we may have to move more logic into the mmu code. */

	eError = PMR_Flags(psPMR, &uiPMRFlags);
	/* guaranteed to not error as we've called Lock already */
	PVR_ASSERT(eError == PVRSRV_OK);

	uiProtFlags =
	    (MMU_PROTFLAGS_T) (uiMapFlags & uiPMRFlags & MMU_PROTFLAGS_MASK);
	/* check that interesting bits don't get lost in cast (due to
	   differing flag widths in s/w stack) */
	PVR_ASSERT(uiProtFlags ==
		   (uiMapFlags & uiPMRFlags & MMU_PROTFLAGS_MASK));

	/* Check that the requested mapping mode can be honoured, after
	   applying the permissions */
	if (uiProtFlags != (uiMapFlags & MMU_PROTFLAGS_MASK)) {
		/* FIXME: perhaps ought to catch this one level higher
		   (devicemem2.c) and simply ASSERT it here. */
		eError = PVRSRV_ERROR_INVALID_PARAMS;	// perhaps prefer: NEWDEVMEM_BAD_MAPPING_MODE?
		goto e0;
	}
#endif

	/* Do flag conversion between devmem flags and MMU generic flags */

	uiMMUProtFlags |=
	    ((uiMappingFlags & PVRSRV_MEMALLOCFLAG_DEVICE_FLAGS_MASK)
	     >> PVRSRV_MEMALLOCFLAG_DEVICE_FLAGS_OFFSET)
	    << MMU_PROTFLAGS_DEVICE_OFFSET;

	if (uiMappingFlags & PVRSRV_MEMALLOCFLAG_GPU_READABLE) {
		uiMMUProtFlags |= MMU_PROTFLAGS_READABLE;
	}
	if (uiMappingFlags & PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE) {
		uiMMUProtFlags |= MMU_PROTFLAGS_WRITEABLE;
	}
	switch (uiMappingFlags & PVRSRV_MEMALLOCFLAG_GPU_CACHE_MODE_MASK) {
	case PVRSRV_MEMALLOCFLAG_GPU_UNCACHED:
	case PVRSRV_MEMALLOCFLAG_GPU_WRITE_COMBINE:
		break;
	case PVRSRV_MEMALLOCFLAG_GPU_CACHE_COHERENT:
		uiMMUProtFlags |= MMU_PROTFLAGS_CACHE_COHERENT;
	case PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT:
		uiMMUProtFlags |= MMU_PROTFLAGS_CACHED;
		break;
	default:
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	/* FIXME: should we verify the size and contiguity (i.e. page size)? */

#if defined(PDUMP)
	PDUMPCOMMENT
	    ("Wire up Page Table entries to point to the Data Pages (%lld bytes)",
	     uiSizeBytes);
#endif

	for (i = 0, uiCount = 0;
	     uiCount < uiSizeBytes; i++, uiCount += uiPageSize) {
#if defined(PDUMP)
		IMG_DEVMEM_OFFSET_T uiNextSymName;
#endif
		/* "uiSize" is the amount of contiguity in the underlying
		   page.  Normally this would be constant for the system, but,
		   that constant needs to be communicated, in case it's ever
		   different. */
		/* Caller guarantees that PMRLockSysPhysAddr() has already
		   been called */
		eError = PMR_DevPhysAddr(psPMR, uiCount, &sDevPAddr);
		PVR_ASSERT(eError == PVRSRV_OK);

		/* check the physical alignment of the memory to map */
		PVR_ASSERT((sDevPAddr.uiAddr & (uiPageSize - 1)) == 0);

#if defined(PDUMP)
		eError = PMR_PDumpSymbolicAddr(psPMR, uiCount,
					       sizeof(aszMemspaceName),
					       &aszMemspaceName[0],
					       sizeof(aszSymbolicAddress),
					       &aszSymbolicAddress[0],
					       &uiSymbolicAddrOffset,
					       &uiNextSymName);
		PVR_ASSERT(eError == PVRSRV_OK);
#endif

		_MMU_MapPage(	/* Map this page into this heap: */
				    psMMUContext,
				    /* To this virtual address */
				    sDevVAddr,
				    /* Here's the physical address */
				    sDevPAddr,
#if defined(PDUMP)
				    /* PDump symbolic address */
				    aszMemspaceName, aszSymbolicAddress,
				    uiSymbolicAddrOffset,
#endif
				    /* and with these flags: */
				    uiMMUProtFlags);

		sDevVAddr.uiAddr += uiPageSize;

		PVR_DPF((PVR_DBG_MESSAGE,
			 "MMU_MapPMR: devVAddr=%10llX, size=0x%x/0x%010llx",
			 sDevVAddr.uiAddr, uiCount, uiSizeBytes));
	}
#if defined(PDUMP)
	PDUMPCOMMENT("Wired up %d Page Table entries", i);
#endif

	return PVRSRV_OK;

	/*
	   error paths follow
	 */

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*
	MMU_AcquireBaseAddr
*/
PVRSRV_ERROR
MMU_AcquireBaseAddr(MMU_CONTEXT * psMMUContext, IMG_DEV_PHYADDR * psPhysAddr)
{
	if (!psMMUContext)
		return PVRSRV_ERROR_INVALID_PARAMS;

	/* FIXME: Do we want to add refcount here? */

	*psPhysAddr = psMMUContext->sBaseLevelInfo.sMemDesc.sDevPAddr;
	return PVRSRV_OK;
}

/*
	MMU_ReleaseBaseAddr
*/
IMG_VOID MMU_ReleaseBaseAddr(MMU_CONTEXT * psMMUContext)
{
	/* FIXME: Do we want to add refcount here? */
	PVR_UNREFERENCED_PARAMETER(psMMUContext);
}

/*
	MMU_SetDeviceData
*/
IMG_VOID MMU_SetDeviceData(MMU_CONTEXT * psMMUContext, IMG_HANDLE hDevData)
{
	psMMUContext->hDevData = hDevData;
}

#if defined(PDUMP)
/*
	MMU_ContextDerivePCPDumpSymAddr
*/
PVRSRV_ERROR MMU_ContextDerivePCPDumpSymAddr(MMU_CONTEXT * psMMUContext,
					     IMG_CHAR *
					     pszPDumpSymbolicNameBuffer,
					     IMG_SIZE_T
					     uiPDumpSymbolicNameBufferSize)
{
	IMG_SIZE_T uiCount;
	IMG_UINT64 ui64PhysAddr;
	PVRSRV_DEVICE_IDENTIFIER *psDevId = &psMMUContext->psDevNode->sDevId;

	if (!psMMUContext->sBaseLevelInfo.sMemDesc.bValid) {
		/* We don't have any allocations.  You're not allowed to ask
		   for the page catalogue base address until you've made at
		   least one allocation */
		/* FIXME! Perhaps it may be better to have a "zero" page, or a
		   pretend page catalogue that would always result in a page
		   fault, and return that instead? */
		return PVRSRV_ERROR_MMU_API_PROTOCOL_ERROR;
	}

	ui64PhysAddr =
	    (IMG_UINT64) psMMUContext->sBaseLevelInfo.sMemDesc.sDevPAddr.uiAddr;

	PVR_ASSERT(uiPDumpSymbolicNameBufferSize >=
		   (IMG_UINT32) (21 +
				 OSStringLength(psDevId->pszPDumpDevName)));

	/* Page table Symbolic Name is formed from page table phys addr
	   prefixed with MMUPT_.  FIXME: should this really be a PMR? */

	uiCount = OSSNPrintf(pszPDumpSymbolicNameBuffer,
			     uiPDumpSymbolicNameBufferSize,
			     ":%s:%s%016llX",
			     psDevId->pszPDumpDevName,
			     psMMUContext->sBaseLevelInfo.sMemDesc.
			     bValid ? "MMUPT_" : "XXX", ui64PhysAddr);

	if (uiCount + 1 > uiPDumpSymbolicNameBufferSize) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	return PVRSRV_OK;
}

/*
	MMU_PDumpWritePageCatBase
*/
PVRSRV_ERROR
MMU_PDumpWritePageCatBase(MMU_CONTEXT * psMMUContext,
			  const IMG_CHAR * pszSpaceName,
			  IMG_DEVMEM_OFFSET_T uiOffset)
{
	PVRSRV_ERROR eError;
	IMG_CHAR aszPageCatBaseSymbolicAddr[100];

	eError = MMU_ContextDerivePCPDumpSymAddr(psMMUContext,
						 &aszPageCatBaseSymbolicAddr[0],
						 sizeof
						 (aszPageCatBaseSymbolicAddr));
	PVR_ASSERT(eError == PVRSRV_OK);

	eError = PDumpWriteSymbAddress(pszSpaceName, uiOffset, aszPageCatBaseSymbolicAddr, 0,	/* offset -- FIXME: could be non-zero for var. pgsz */
				       8,	/* word size (bytes) i.e. is a 64-bit reg */
				       PDUMP_FLAGS_CONTINUOUS);
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

/*
	MMU_AcquirePDumpMMUContext
*/
PVRSRV_ERROR MMU_AcquirePDumpMMUContext(MMU_CONTEXT * psMMUContext,
					IMG_UINT32 * pui32PDumpMMUContextID)
{
	PVRSRV_DEVICE_IDENTIFIER *psDevId = &psMMUContext->psDevNode->sDevId;

	if (!psMMUContext->ui32PDumpContextIDRefCount) {
		PDUMP_MMU_ALLOC_MMUCONTEXT(psDevId->pszPDumpDevName,
					   psMMUContext->sBaseLevelInfo.
					   sMemDesc.sDevPAddr,
					   &psMMUContext->uiPDumpContextID);
	}

	psMMUContext->ui32PDumpContextIDRefCount++;
	*pui32PDumpMMUContextID = psMMUContext->uiPDumpContextID;

	return PVRSRV_OK;
}

/*
	MMU_ReleasePDumpMMUContext
*/
PVRSRV_ERROR MMU_ReleasePDumpMMUContext(MMU_CONTEXT * psMMUContext)
{
	PVRSRV_DEVICE_IDENTIFIER *psDevId = &psMMUContext->psDevNode->sDevId;

	PVR_ASSERT(psMMUContext->ui32PDumpContextIDRefCount != 0);
	psMMUContext->ui32PDumpContextIDRefCount--;

	if (psMMUContext->ui32PDumpContextIDRefCount == 0) {
		PDUMP_MMU_FREE_MMUCONTEXT(psDevId->pszPDumpDevName,
					  psMMUContext->uiPDumpContextID);
	}

	return PVRSRV_OK;
}
#endif

/******************************************************************************
 End of file (mmu_common.c)
******************************************************************************/
