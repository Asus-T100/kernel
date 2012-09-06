									    /*************************************************************************//*!
									       @File
									       @Title          Physmem (PMR) abstraction
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Part of the memory management.  This module is responsible for
									       the "PMR" abstraction.  A PMR (Physical Memory Resource)
									       represents some unit of physical memory which is
									       allocated/freed/mapped/unmapped as an indivisible unit
									       (higher software levels provide an abstraction above that
									       to deal with dividing this down into smaller manageable units).
									       Importantly, this module knows nothing of virtual memory, or
									       of MMUs etc., with one excuseable exception.  We have the
									       concept of a "page size", which really means nothing in
									       physical memory, but represents a "contiguity quantum" such
									       that the higher level modules which map this memory are able
									       to verify that it matches the needs of the page size for the
									       virtual realm into which it is being mapped.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "img_types.h"
#include "pdumpdefs.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"

#include "pdump.h"

#include "osfunc.h"
#include "pdump_km.h"
#include "pdump_physmem.h"
#include "pmr_impl.h"
#include "pvrsrv.h"

#include "allocmem.h"

#if defined(SUPPORT_SECURE_EXPORT)
#include "secure_export.h"
#include "ossecure_export.h"
#endif

/* ourselves */
#include "pmr.h"

/* A "context" for the physical memory block resource allocator.

   Context is probably the wrong word.

   There is almost certainly only one of these, ever, in the system.
   But, let's keep the notion of a context anyway, "just-in-case".
*/
struct _PMR_CTX_ {
	/* For debugging, and PDump, etc., let's issue a forever
	   incrementing serial number to each allocation. */
	IMG_UINT64 uiNextSerialNum;

	/* For security, we only allow a PMBR to be mapped if the caller
	   knows its key.  We can pseudo-randomly generate keys */
	IMG_UINT64 uiNextKey;

	/* For debugging only, I guess:  Number of live PMBRs */
	IMG_UINT32 uiNumLivePMRs;

	/* 
	   FIXME:

	   We need an OS independent mutex to protect uiNextKey and
	   uiNextSerialNum.  This could be a spinlock, as we'd only hold
	   it very briefly while turning the state machine to generate
	   these numbers.
	 */

	/* In order to seed the uiNextKey, we enforce initialisation at
	   driver load time.  Also, we can debug check at driver unload
	   that the PMR count is zero. */
	IMG_BOOL bModuleInitialised;
} _gsSingletonPMRContext = {
1, 0, 0, IMG_FALSE};

/* A "PMBR" or PMR, or whatever

   One per physical allocation.  May be "shared".

   "shared" is ambiguous.  We need to be careful with terminology.
   There are two ways in which a PMR may be "shared" and we need to be
   sure that we are clear which we mean.

   i)   multiple small allocations living together inside one PMR;

   ii)  one single allocation filling a PMR but mapped into multiple
        memory contexts.

   This is more important further up the stack - at this level, all we
   care is that the PMR is being referenced multiple times.
*/
struct _PMR_ {
	/* This object is strictly refcounted.  References include:
	   - mapping
	   - live handles (to this object)
	   - live export handles
	   (thus it is normal for allocated and exported memory to have a refcount of 3)
	   The object is destroyed when and only when the refcount reaches 0
	 */
	/*
	   Physical address translation (device <> cpu) is done on a per device
	   basis which means we need the physcial heap info
	 */
	PHYS_HEAP *psPhysHeap;
	/* FIXME: need mutex or spinlock around access to
	   uiRefCount.  When incrementing, all we care is that the
	   increment is atomic.  When decrementing, not only do we care
	   about whether it is atomic, but also whether it reached zero.
	   Testing for its zeroness as a separate step is not
	   sufficient. */
	IMG_UINT32 uiRefCount;

	/* lock count - this is the number of times
	   PMRLockSysPhysAddresses() has been called, less the number of
	   PMRUnlockSysPhysAddresses() calls.  This is arguably here for
	   debug reasons only, as the refcount is already incremented as a
	   matter of course.  Really, this just allows us to trap protocol
	   errors: i.e. calling PMRSysPhysAddr(),
	   without a lock, or calling PMRUnlockSysPhysAddresses() too many
	   or too few times. */
	IMG_UINT32 uiLockCount;

	/* Incrementing serial number to each allocation. */
	IMG_UINT64 uiSerialNum;

	/* For security, we only allow a PMBR to be mapped if the caller
	   knows its key.  We can pseudo-randomly generate keys */
	PMR_PASSWORD_T uiKey;

	/* Callbacks for per-flavour functions */
	const PMR_IMPL_FUNCTAB *psFuncTab;

	/* Data associated with the "subtype" */
	PMR_IMPL_PRIVDATA pvFlavourData;

	/* And for pdump */
	const IMG_CHAR *pszPDumpDefaultMemspaceName;
	const IMG_CHAR *pszPDumpFlavour;

	/* Logical size of allocation.  "logical", because a PMR can
	   represent memory that will never physically exist.  This is the
	   amount of virtual space that the PMR would consume when it's
	   mapped into a virtual allocation. */
	PMR_SIZE_T uiLogicalSize;

	/* Minimum Physical Contiguity Guarantee.  Might be called "page
	   size", but that would be incorrect, as page size is something
	   meaningful only in virtual realm.  This contiguity guarantee
	   provides an inequality that can be verified/asserted/whatever
	   to ensure that this PMR conforms to the page size requirement
	   of the place the PMR gets mapped.  (May be used to select an
	   appropriate heap in variable page size systems)

	   The absolutely necessary condition is this:

	   device MMU page size <= actual physical contiguity.

	   We go one step further in order to be able to provide an early warning / early compatibility check and say this:

	   device MMU page size <= 2**(uiLog2ContiguityGuarantee) <= actual physical contiguity.

	   In this way, it is possible to make the page table reservation
	   in the device MMU without even knowing the granularity of the
	   physical memory (i.e. useful for being able to allocate virtual
	   before physical)
	 */
	PMR_LOG2ALIGN_T uiLog2ContiguityGuarantee;

	/* Flags.  We store a copy of the "PMR flags" (usually a subset of
	   the flags given at allocation time) and return them to any
	   caller of PMR_Flags().  The intention of these flags is that
	   the ones stored here are used to represent permissions, such
	   that noone is able to map a PMR in a mode in which they are not
	   allowed, e.g. writeable for a read-only PMR, etc. */
	PMR_FLAGS_T uiFlags;

	/* Do we really need this? For now we'll keep it, until we know we don't. */
	/* NB: this is not the "memory context" in client terms - this is
	   _purely_ the "PMR" context, of which there is almost certainly only
	   ever one per system as a whole, but we'll keep the concept
	   anyway, just-in-case. */
	struct _PMR_CTX_ *psContext;
};

/* do we need a struct for the export handle?  I'll use one for now, but if nothing goes in it, we'll lose it */
struct _PMR_EXPORT_ {
	struct _PMR_ *psPMR;
};

struct _PMR_PAGELIST_ {
	struct _PMR_ *psReferencePMR;
};

#define MIN3(a,b,c)	(((a) < (b)) ? (((a) < (c)) ? (a):(c)) : (((b) < (c)) ? (b):(c)))

static PVRSRV_ERROR
_PMRCreate(PMR_SIZE_T uiLogicalSize,
	   PMR_LOG2ALIGN_T uiLog2ContiguityGuarantee,
	   PMR_FLAGS_T uiFlags, PMR ** ppsPMR)
{
	IMG_VOID *pvLinAddr;
	PMR *psPMR;
	struct _PMR_CTX_ *psContext;

	psContext = &_gsSingletonPMRContext;

	pvLinAddr = OSAllocMem(sizeof(*psPMR));
	if (pvLinAddr == IMG_NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	} else {
		psPMR = (PMR *) pvLinAddr;
		psPMR->uiRefCount = 0;
		psPMR->uiLockCount = 0;
		psPMR->psContext = psContext;
		psPMR->uiLogicalSize = uiLogicalSize;
		psPMR->uiLog2ContiguityGuarantee = uiLog2ContiguityGuarantee;
		psPMR->uiFlags = uiFlags;
		psPMR->uiKey = psContext->uiNextKey;
		psPMR->uiSerialNum = psContext->uiNextSerialNum;
		psContext->uiNextKey = (0x80200003 * psContext->uiNextKey)
		    ^ (0xf00f0081 * (IMG_UINTPTR_T) pvLinAddr);
		psContext->uiNextSerialNum++;

		*ppsPMR = psPMR;

		PVR_DPF((PVR_DBG_MESSAGE, "pmr.c: created PMR @0x%p", psPMR));

		/* Increment live PMR count.  Probably only of interest for debugging */
		/* FIXME: ATOMIC increment required */
		psContext->uiNumLivePMRs++;

		return PVRSRV_OK;
	}
}

static IMG_VOID _UnrefAndMaybeDestroy(PMR * psPMR)
{
	PVRSRV_ERROR eError2;
	struct _PMR_CTX_ *psCtx;

	PVR_ASSERT(psPMR != IMG_NULL);
	PVR_ASSERT(psPMR->uiRefCount > 0);

	/* FIXME: need mutex / spinlock / atomic decrement and test zero */
	psPMR->uiRefCount--;

	if (psPMR->uiRefCount == 0) {
		if (psPMR->psFuncTab->pfnFinalize != IMG_NULL) {
			eError2 =
			    psPMR->psFuncTab->pfnFinalize(psPMR->pvFlavourData);
			PVR_ASSERT(eError2 == PVRSRV_OK);	/* can we do better? */
		}

		PVR_ASSERT(psPMR->uiLockCount == 0);

		psCtx = psPMR->psContext;

		OSFreeMem(psPMR);

		/* Decrement live PMR count.  Probably only of interest for debugging */
		PVR_ASSERT(psCtx->uiNumLivePMRs > 0);
		/* FIXME: mutex / spinlock / atomic decrement required */
		psCtx->uiNumLivePMRs--;
	}
}

PVRSRV_ERROR
PMRCreatePMR(PHYS_HEAP * psPhysHeap,
	     PMR_SIZE_T uiLogicalSize,
	     PMR_LOG2ALIGN_T uiLog2ContiguityGuarantee,
	     PMR_FLAGS_T uiFlags,
	     const IMG_CHAR * pszPDumpFlavour,
	     const PMR_IMPL_FUNCTAB * psFuncTab,
	     PMR_IMPL_PRIVDATA pvPrivData, PMR ** ppsPMRPtr)
{
	PVRSRV_ERROR eError;
	PMR *psPMR;

	eError = _PMRCreate(uiLogicalSize, uiLog2ContiguityGuarantee, uiFlags,
			    &psPMR);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	psPMR->psPhysHeap = psPhysHeap;
	psPMR->psFuncTab = psFuncTab;
	psPMR->pszPDumpDefaultMemspaceName =
	    PhysHeapPDumpMemspaceName(psPhysHeap);
	psPMR->pszPDumpFlavour = pszPDumpFlavour;
	psPMR->pvFlavourData = pvPrivData;
	psPMR->uiRefCount++;

	*ppsPMRPtr = psPMR;

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR
PMRLockSysPhysAddresses(PMR * psPMR, IMG_UINT32 uiLog2RequiredContiguity)
{
	PVRSRV_ERROR eError;

	PVR_ASSERT(psPMR != IMG_NULL);

	if (uiLog2RequiredContiguity > psPMR->uiLog2ContiguityGuarantee) {
		eError = PVRSRV_ERROR_PMR_INCOMPATIBLE_CONTIGUITY;
		goto e0;
	}

	/* We also count the locks as references, so that the PMR is not
	   freed while someone is using a physical address. */
	/* "lock" here simply means incrementing the refcount.  It means
	   the refcount is multipurpose, but that's okay.  We only have to
	   promise that physical addresses are valid after this point, and
	   remain valid until the corresponding
	   PMRUnlockSysPhysAddressesOSMem() */
	psPMR->uiRefCount++;	/* FIXME: need atomic increment */

	/* Also count locks separately from other types of references, to
	   allow for debug assertions */
	psPMR->uiLockCount++;
	/* FIXME: protect with mutex/spinlock/atomic inc and test/etc */
	/* Only call callback if lockcount transitions from 0 to 1 */
	if (psPMR->uiLockCount == 1) {
		if (psPMR->psFuncTab->pfnLockPhysAddresses != IMG_NULL) {
			/* must always have lock and unlock in pairs! */
			PVR_ASSERT(psPMR->psFuncTab->pfnUnlockPhysAddresses !=
				   IMG_NULL);

			eError =
			    psPMR->psFuncTab->pfnLockPhysAddresses(psPMR->
								   pvFlavourData,
								   uiLog2RequiredContiguity);

			if (eError != PVRSRV_OK) {
				goto e1;
			}
		}
	}

	return PVRSRV_OK;

 e1:
	psPMR->uiLockCount--;
	psPMR->uiRefCount--;

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR PMRUnlockSysPhysAddresses(PMR * psPMR)
{
	PVRSRV_ERROR eError2;

	PVR_ASSERT(psPMR != IMG_NULL);

	/* FIXME: protect with mutex/spinlock/atomic dec and test/etc */
	/* Only call callback if lockcount transitions from 0 to 1 */
	if (psPMR->uiLockCount == 1) {
		if (psPMR->psFuncTab->pfnUnlockPhysAddresses != IMG_NULL) {
			PVR_ASSERT(psPMR->psFuncTab->pfnLockPhysAddresses !=
				   IMG_NULL);

			eError2 =
			    psPMR->psFuncTab->pfnUnlockPhysAddresses(psPMR->
								     pvFlavourData);
			/* must never fail */
			PVR_ASSERT(eError2 == PVRSRV_OK);
		}
	}

	PVR_ASSERT(psPMR->uiLockCount > 0);
	psPMR->uiLockCount--;

	/* We also count the locks as references, so that the PMR is not
	   freed while someone is using a physical address. */
	_UnrefAndMaybeDestroy(psPMR);

	return PVRSRV_OK;
}

PVRSRV_ERROR
PMRExportPMR(PMR * psPMR,
	     PMR_EXPORT ** ppsPMRExportPtr,
	     PMR_SIZE_T * puiSize,
	     PMR_LOG2ALIGN_T * puiLog2Contig, PMR_PASSWORD_T * puiPassword)
{
	IMG_UINT64 uiPassword;
	PMR_EXPORT *psPMRExport;

	uiPassword = psPMR->uiKey;

	psPMRExport = OSAllocMem(sizeof(*psPMRExport));
	if (psPMRExport == IMG_NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psPMRExport->psPMR = psPMR;
	psPMR->uiRefCount++;

	/* FIXME: add resman */

	*ppsPMRExportPtr = psPMRExport;
	*puiSize = psPMR->uiLogicalSize;
	*puiLog2Contig = psPMR->uiLog2ContiguityGuarantee;
	*puiPassword = uiPassword;

	/*
	   FIXME:

	   Should we should have a pointer in the PMR to the
	   PMRExport?  This way, if the export hasn't been unexported,
	   but the PMR gets freed, then the export could be undone.  Also,
	   it would enforce only one export at a time (is this
	   wise/useful?)
	 */
	return PVRSRV_OK;
}

PVRSRV_ERROR
PMRMakeServerExportClientExport(DEVMEM_EXPORTCOOKIE * psPMRExportIn,
				PMR_EXPORT ** ppsPMRExportPtr,
				PMR_SIZE_T * puiSize,
				PMR_LOG2ALIGN_T * puiLog2Contig,
				PMR_PASSWORD_T * puiPassword)
{
	*ppsPMRExportPtr = (PMR_EXPORT *) psPMRExportIn->hPMRExportHandle;
	*puiSize = psPMRExportIn->uiSize;
	*puiLog2Contig = psPMRExportIn->uiLog2ContiguityGuarantee;
	*puiPassword = psPMRExportIn->uiPMRExportPassword;

	return PVRSRV_OK;
}

PVRSRV_ERROR PMRUnmakeServerExportClientExport(PMR_EXPORT * psPMRExport)
{
	PVR_UNREFERENCED_PARAMETER(psPMRExport);

	/*
	   There is nothing to do here, the server will call unexport
	   regardless of the type of shutdown. In order to play ball
	   with resman (where it's used) we need to pair functions
	   and this is PMRMakeServerExportClientExport counter part.
	 */
	return PVRSRV_OK;
}

PVRSRV_ERROR PMRUnexportPMR(PMR_EXPORT * psPMRExport)
{

	/* FIXME: probably shouldn't be assertions? */
	PVR_ASSERT(psPMRExport != IMG_NULL);
	PVR_ASSERT(psPMRExport->psPMR != IMG_NULL);
	PVR_ASSERT(psPMRExport->psPMR->uiRefCount > 0);

	_UnrefAndMaybeDestroy(psPMRExport->psPMR);

	OSFreeMem(psPMRExport);

	return PVRSRV_OK;
}

PVRSRV_ERROR
PMRImportPMR(PMR_EXPORT * psPMRExport,
	     PMR_PASSWORD_T uiPassword,
	     PMR_SIZE_T uiSize, PMR_LOG2ALIGN_T uiLog2Contig, PMR ** ppsPMR)
{
	PMR *psPMR;

	/* FIXME: probably shouldn't be assertions? */
	PVR_ASSERT(psPMRExport != IMG_NULL);
	PVR_ASSERT(psPMRExport->psPMR != IMG_NULL);
	PVR_ASSERT(psPMRExport->psPMR->uiRefCount > 0);

	psPMR = psPMRExport->psPMR;

	if (psPMR->uiKey != uiPassword) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PMRImport: password given = %016llx, expected = %016llx\n",
			 uiPassword, psPMR->uiKey));
		return PVRSRV_ERROR_PMR_WRONG_PASSWORD_OR_STALE_PMR;
	}

	/* FIXME: Race condition. Need atomic inc */

	psPMR->uiRefCount++;

	/* recheck, at least to guard against aforementioned race */
	/* FIXME: Remove when above FIXME is fixed */
	PVR_ASSERT(psPMR->uiKey == uiPassword);

	if (psPMR->uiLogicalSize != uiSize
	    || psPMR->uiLog2ContiguityGuarantee != uiLog2Contig) {
		psPMR->uiRefCount--;
		return PVRSRV_ERROR_PMR_MISMATCHED_ATTRIBUTES;
	}

	*ppsPMR = psPMR;

	return PVRSRV_OK;
}

PVRSRV_ERROR PMRUnimportPMR(PMR * psPMR)
{
	_UnrefAndMaybeDestroy(psPMR);

	return PVRSRV_OK;
}

/*
	Note:
	We pass back the PMR as it was passed in as a different handle type
	(DEVMEM_MEM_IMPORT) and it allows us to change the import structure
	type if we should need to embed any meta data in it.
*/
PVRSRV_ERROR
PMRLocalImportPMR(PMR * psPMR,
		  PMR ** ppsPMR,
		  IMG_DEVMEM_SIZE_T * puiSize, IMG_DEVMEM_ALIGN_T * puiAlign)
{
	/* FIXME: Race condition. Need atomic inc */
	psPMR->uiRefCount++;

	/* Return the PMR */
	*ppsPMR = psPMR;
	*puiSize = psPMR->uiLogicalSize;
	*puiAlign = 1 << psPMR->uiLog2ContiguityGuarantee;
	return PVRSRV_OK;
}

PVRSRV_ERROR PMRGetUID(PMR * psPMR, IMG_UINT64 * pui64UID)
{
	PVR_ASSERT(psPMR != IMG_NULL);

	*pui64UID = psPMR->uiSerialNum;

	return PVRSRV_OK;
}

#if defined(SUPPORT_SECURE_EXPORT)
PVRSRV_ERROR PMRSecureExportPMR(CONNECTION_DATA * psConnection,
				PMR * psPMR,
				IMG_SECURE_TYPE * phSecure,
				PMR ** ppsPMR,
				CONNECTION_DATA ** ppsSecureConnection)
{
	PVRSRV_ERROR eError;

	eError = OSSecureExport(psConnection,
				(IMG_PVOID) psPMR,
				phSecure, ppsSecureConnection);

	if (eError != PVRSRV_OK) {
		goto e0;
	}

	/* Make sure the PMR won't get freed before it's imported */
	PMRRefPMR(psPMR);

	/* Pass back the PMR for resman */
	*ppsPMR = psPMR;

	return PVRSRV_OK;
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR PMRSecureUnexportPMR(PMR * psPMR)
{
	PMRUnrefPMR(psPMR);
	return PVRSRV_OK;
}

PVRSRV_ERROR PMRSecureImportPMR(IMG_SECURE_TYPE hSecure,
				PMR ** ppsPMR,
				IMG_DEVMEM_SIZE_T * puiSize,
				IMG_DEVMEM_ALIGN_T * puiAlign)
{
	PVRSRV_ERROR eError;
	PMR *psPMR;

	eError = OSSecureImport(hSecure, (IMG_PVOID *) & psPMR);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	PMRRefPMR(psPMR);

	/* Return the PMR */
	*ppsPMR = psPMR;
	*puiSize = psPMR->uiLogicalSize;
	*puiAlign = 1 << psPMR->uiLog2ContiguityGuarantee;
	return PVRSRV_OK;
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR PMRSecureUnimportPMR(PMR * psPMR)
{
	PMRUnrefPMR(psPMR);
	return PVRSRV_OK;
}
#endif

PVRSRV_ERROR
PMRAcquireKernelMappingData(PMR * psPMR,
			    IMG_SIZE_T uiOffset,
			    IMG_SIZE_T uiSize,
			    IMG_VOID ** ppvKernelAddressOut,
			    IMG_SIZE_T * puiLengthOut, IMG_HANDLE * phPrivOut)
{
	PVRSRV_ERROR eError;
	IMG_VOID *pvKernelAddress;
	IMG_HANDLE hPriv;
	PMR_FLAGS_T ulFlags;

	PVR_ASSERT(psPMR != IMG_NULL);

	/* Acquire/Release functions must be overridden in pairs */
	if (psPMR->psFuncTab->pfnAcquireKernelMappingData == IMG_NULL) {
		PVR_ASSERT(psPMR->psFuncTab->pfnReleaseKernelMappingData ==
			   IMG_NULL);

		/* If PMR implementation does not supply this pair of
		   functions, it means they do not permit the PMR to be mapped
		   into kernel memory at all */
		eError = PVRSRV_ERROR_PMR_NOT_PERMITTED;
		goto e0;
	}
	PVR_ASSERT(psPMR->psFuncTab->pfnReleaseKernelMappingData != IMG_NULL);

	PMR_Flags(psPMR, &ulFlags);

	eError =
	    psPMR->psFuncTab->pfnAcquireKernelMappingData(psPMR->pvFlavourData,
							  uiOffset, uiSize,
							  &pvKernelAddress,
							  &hPriv, ulFlags);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	*ppvKernelAddressOut = pvKernelAddress;
	if (uiSize == 0) {
		/* Zero size means map the whole PMR in ... */
		*puiLengthOut = (IMG_SIZE_T) psPMR->uiLogicalSize;
	} else {
		/* ... otherwise we just map in one page */
		*puiLengthOut = 1 << psPMR->uiLog2ContiguityGuarantee;
	}
	*phPrivOut = hPriv;

	return PVRSRV_OK;

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR PMRReleaseKernelMappingData(PMR * psPMR, IMG_HANDLE hPriv)
{
	PVR_ASSERT(psPMR->psFuncTab->pfnAcquireKernelMappingData != IMG_NULL);
	PVR_ASSERT(psPMR->psFuncTab->pfnReleaseKernelMappingData != IMG_NULL);

	psPMR->psFuncTab->pfnReleaseKernelMappingData(psPMR->pvFlavourData,
						      hPriv);

	return PVRSRV_OK;
}

PVRSRV_ERROR
PMR_ReadBytes(PMR * psPMR,
	      IMG_DEVMEM_OFFSET_T uiOffset,
	      IMG_UINT8 * pcBuffer,
	      IMG_SIZE_T uiBufSz, IMG_SIZE_T * puiNumBytes)
{
	PVRSRV_ERROR eError;
	if (uiOffset + uiBufSz > psPMR->uiLogicalSize) {
		uiBufSz = psPMR->uiLogicalSize - uiOffset;
	}
	PVR_ASSERT(uiBufSz > 0);
	PVR_ASSERT(uiBufSz <= psPMR->uiLogicalSize);

	/*
	   PMR implementations can override this.  If they don't, a
	   "default" handler uses kernel virtual mappings.  If the kernel
	   can't provide a kernel virtual mapping, this function fails
	 */
	PVR_ASSERT(psPMR->psFuncTab->pfnAcquireKernelMappingData != IMG_NULL ||
		   psPMR->psFuncTab->pfnReadBytes != IMG_NULL);

	if (psPMR->psFuncTab->pfnReadBytes != IMG_NULL) {
		/* defer to callback if present */

		eError = PMRLockSysPhysAddresses(psPMR,
						 psPMR->
						 uiLog2ContiguityGuarantee);
		if (eError != PVRSRV_OK) {
			goto e0;
		}

		eError = psPMR->psFuncTab->pfnReadBytes(psPMR->pvFlavourData,
							uiOffset,
							pcBuffer,
							uiBufSz, puiNumBytes);
		PMRUnlockSysPhysAddresses(psPMR);
		if (eError != PVRSRV_OK) {
			goto e0;
		}
	} else if (psPMR->psFuncTab->pfnAcquireKernelMappingData) {
		/* "default" handler for reading bytes */

		IMG_HANDLE hKernelMappingHandle;
		IMG_UINT8 *pcKernelAddress;
		PMR_FLAGS_T ulFlags;

		PMR_Flags(psPMR, &ulFlags);

		eError =
		    psPMR->psFuncTab->pfnAcquireKernelMappingData(psPMR->
								  pvFlavourData,
								  0, 0,
								  (IMG_VOID **)
								  &
								  pcKernelAddress,
								  &hKernelMappingHandle,
								  ulFlags);
		if (eError != PVRSRV_OK) {
			goto e0;
		}

		OSMemCopy(&pcBuffer[0], &pcKernelAddress[uiOffset], uiBufSz);
		*puiNumBytes = uiBufSz;

		psPMR->psFuncTab->pfnReleaseKernelMappingData(psPMR->
							      pvFlavourData,
							      hKernelMappingHandle);
	} else {
		PVR_DPF((PVR_DBG_ERROR,
			 "PMR_ReadBytes: can't read from this PMR"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		OSPanic();
		goto e0;
	}

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	OSPanic();
	return eError;
}

IMG_VOID PMRRefPMR(PMR * psPMR)
{
	PVR_ASSERT(psPMR != IMG_NULL);
	psPMR->uiRefCount++;
}

PVRSRV_ERROR PMRUnrefPMR(PMR * psPMR)
{
	_UnrefAndMaybeDestroy(psPMR);
	return PVRSRV_OK;
}

PVRSRV_ERROR PMR_Flags(const PMR * psPMR, PMR_FLAGS_T * puiPMRFlags)
{
	PVR_ASSERT(psPMR != IMG_NULL);

	*puiPMRFlags = psPMR->uiFlags;
	return PVRSRV_OK;
}

PVRSRV_ERROR
PMR_LogicalSize(const PMR * psPMR, IMG_DEVMEM_SIZE_T * puiLogicalSize)
{
	PVR_ASSERT(psPMR != IMG_NULL);

	*puiLogicalSize = psPMR->uiLogicalSize;
	return PVRSRV_OK;
}

/* must have called PMRLockSysPhysAddresses() before calling this! */
PVRSRV_ERROR
PMR_DevPhysAddr(const PMR * psPMR,
		IMG_DEVMEM_OFFSET_T uiOffset, IMG_DEV_PHYADDR * psDevAddrPtr)
{
	PVRSRV_ERROR eError;

	PVR_ASSERT(psPMR != IMG_NULL);
	PVR_ASSERT(psPMR->psFuncTab->pfnDevPhysAddr != IMG_NULL);

	PVR_ASSERT(psPMR->uiLockCount > 0);

	eError = psPMR->psFuncTab->pfnDevPhysAddr(psPMR->pvFlavourData,
						  uiOffset, psDevAddrPtr);

	if (eError != PVRSRV_OK) {
		goto e0;
	}

	return PVRSRV_OK;

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR
PMR_CpuPhysAddr(const PMR * psPMR,
		IMG_DEVMEM_OFFSET_T uiOffset, IMG_CPU_PHYADDR * psCpuAddrPtr)
{
	IMG_DEV_PHYADDR sDevPAddr;
	PVRSRV_ERROR eError;

	eError = PMR_DevPhysAddr(psPMR, uiOffset, &sDevPAddr);
	if (eError != PVRSRV_OK) {
		goto e0;
	}
	PhysHeapDevPAddrToCpuPAddr(psPMR->psPhysHeap, psCpuAddrPtr, &sDevPAddr);

	return PVRSRV_OK;

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

#if defined(PDUMP)
PVRSRV_ERROR
PMR_PDumpSymbolicAddr(const PMR * psPMR,
		      IMG_DEVMEM_OFFSET_T uiOffset,
		      IMG_UINT32 ui32MemspaceNameLen,
		      IMG_CHAR * pszMemspaceName,
		      IMG_UINT32 ui32SymbolicAddrLen,
		      IMG_CHAR * pszSymbolicAddr,
		      IMG_DEVMEM_OFFSET_T * puiNewOffset,
		      IMG_DEVMEM_OFFSET_T * puiNextSymName)
{
	const IMG_CHAR *pszPrefix;

	PVR_ASSERT(uiOffset < psPMR->uiLogicalSize);

	if (psPMR->psFuncTab->pfnPDumpSymbolicAddr != IMG_NULL) {
		/* defer to callback if present */
		return psPMR->psFuncTab->pfnPDumpSymbolicAddr(psPMR->
							      pvFlavourData,
							      uiOffset,
							      pszMemspaceName,
							      ui32MemspaceNameLen,
							      pszSymbolicAddr,
							      ui32SymbolicAddrLen,
							      puiNewOffset,
							      puiNextSymName);
	} else {
		OSSNPrintf(pszMemspaceName, ui32MemspaceNameLen, "%s",
			   psPMR->pszPDumpDefaultMemspaceName);

		if (psPMR->pszPDumpFlavour != IMG_NULL) {
			pszPrefix = psPMR->pszPDumpFlavour;
		} else {
			pszPrefix = PMR_DEFAULT_PREFIX;
		}
		OSSNPrintf(pszSymbolicAddr, ui32SymbolicAddrLen,
			   PMR_SYMBOLICADDR_FMTSPEC, pszPrefix,
			   psPMR->uiSerialNum);
		*puiNewOffset = uiOffset;
		*puiNextSymName = (IMG_DEVMEM_OFFSET_T) psPMR->uiLogicalSize;

		return PVRSRV_OK;
	}
}

/*!
 * @brief Writes a WRW command to the script2 buffer, representing a
 * 		  dword write to a physical allocation. Size is always
 * 		  sizeof(IMG_UINT32).
 * @param psPMR - PMR object representing allocation
 * @param uiOffset - offset
 * @param ui32Value - value to write
 * @param uiPDumpFlags - pdump flags
 * @return PVRSRV_ERROR
 */
PVRSRV_ERROR
PMRPDumpLoadMemValue(PMR * psPMR,
		     IMG_DEVMEM_OFFSET_T uiOffset,
		     IMG_UINT32 ui32Value, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;
	IMG_CHAR aszMemspaceName[PMR_MAX_MEMSPACE_NAME_LENGTH_DEFAULT];
	IMG_CHAR aszSymbolicName[PMR_MAX_SYMBOLIC_ADDRESS_LENGTH_DEFAULT];
	IMG_DEVMEM_OFFSET_T uiPDumpSymbolicOffset;
	IMG_DEVMEM_OFFSET_T uiNextSymName;

	PVR_ASSERT(uiOffset + sizeof(IMG_UINT32) <= psPMR->uiLogicalSize);

	eError = PMRLockSysPhysAddresses(psPMR,
					 psPMR->uiLog2ContiguityGuarantee);
	PVR_ASSERT(eError == PVRSRV_OK);

	/* Get the symbolic address of the PMR */
	eError = PMR_PDumpSymbolicAddr(psPMR,
				       uiOffset,
				       sizeof(aszMemspaceName),
				       &aszMemspaceName[0],
				       sizeof(aszSymbolicName),
				       &aszSymbolicName[0],
				       &uiPDumpSymbolicOffset, &uiNextSymName);
	PVR_ASSERT(eError == PVRSRV_OK);

	/* Write the WRW script command */
	eError = PDumpPMRWRW(aszMemspaceName,
			     aszSymbolicName,
			     uiPDumpSymbolicOffset, ui32Value, uiPDumpFlags);
	PVR_ASSERT(eError == PVRSRV_OK);

	eError = PMRUnlockSysPhysAddresses(psPMR);
	PVR_ASSERT(eError == PVRSRV_OK);

	return PVRSRV_OK;
}

PVRSRV_ERROR
PMRPDumpLoadMem(PMR * psPMR,
		IMG_DEVMEM_OFFSET_T uiOffset,
		IMG_DEVMEM_SIZE_T uiSize, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;
	IMG_CHAR aszMemspaceName[PMR_MAX_MEMSPACE_NAME_LENGTH_DEFAULT];
	IMG_CHAR aszSymbolicName[PMR_MAX_SYMBOLIC_ADDRESS_LENGTH_DEFAULT];
	IMG_DEVMEM_OFFSET_T uiPDumpSymbolicOffset;
	IMG_CHAR
	    aszParamStreamFilename[PMR_MAX_PARAMSTREAM_FILENAME_LENGTH_DEFAULT];
	PDUMP_FILEOFFSET_T uiParamStreamFileOffset;
	IMG_UINT8 *pcBuffer;
	IMG_SIZE_T uiBufSz;
	IMG_SIZE_T uiNumBytes;
	IMG_DEVMEM_OFFSET_T uiNextSymName;

	PVR_ASSERT(uiOffset + uiSize <= psPMR->uiLogicalSize);

#define PMR_MAX_PDUMP_BUFSZ 16384
	uiBufSz = PMR_MAX_PDUMP_BUFSZ;
	if (uiBufSz > uiSize) {
		uiBufSz = uiSize;
	}

	pcBuffer = OSAllocMem(uiBufSz);
	PVR_ASSERT(pcBuffer != IMG_NULL);

	eError = PMRLockSysPhysAddresses(psPMR,
					 psPMR->uiLog2ContiguityGuarantee);
	PVR_ASSERT(eError == PVRSRV_OK);

	while (uiSize > 0) {
		eError = PMR_PDumpSymbolicAddr(psPMR,
					       uiOffset,
					       sizeof(aszMemspaceName),
					       &aszMemspaceName[0],
					       sizeof(aszSymbolicName),
					       &aszSymbolicName[0],
					       &uiPDumpSymbolicOffset,
					       &uiNextSymName);
		PVR_ASSERT(eError == PVRSRV_OK);

		/* Reads enough to fill buffer, or until next symbolic name
		   change, or until end of PMR, whichever comes first */
		eError = PMR_ReadBytes(psPMR,
				       uiOffset,
				       pcBuffer,
				       MIN3(uiBufSz, uiSize,
					    uiNextSymName - uiOffset),
				       &uiNumBytes);
		PVR_ASSERT(eError == PVRSRV_OK);
		PVR_ASSERT(uiNumBytes > 0);

		eError = PDumpWriteBuffer(pcBuffer,
					  uiNumBytes,
					  uiPDumpFlags,
					  &aszParamStreamFilename[0],
					  sizeof(aszParamStreamFilename),
					  &uiParamStreamFileOffset);
		PVR_ASSERT(eError == PVRSRV_OK);

		eError = PDumpPMRLDB(aszMemspaceName,
				     aszSymbolicName,
				     uiPDumpSymbolicOffset,
				     uiNumBytes,
				     aszParamStreamFilename,
				     uiParamStreamFileOffset, uiPDumpFlags);
		PVR_ASSERT(eError == PVRSRV_OK);

		uiOffset += uiNumBytes;
		PVR_ASSERT(uiNumBytes <= uiSize);
		uiSize -= uiNumBytes;
	}

	eError = PMRUnlockSysPhysAddresses(psPMR);
	PVR_ASSERT(eError == PVRSRV_OK);

	OSFreeMem(pcBuffer);

	return PVRSRV_OK;
}

PVRSRV_ERROR
PMRPDumpSaveToFile(const PMR * psPMR,
		   IMG_DEVMEM_OFFSET_T uiOffset,
		   IMG_DEVMEM_SIZE_T uiSize,
		   IMG_UINT32 uiArraySize, const IMG_CHAR * pszFilename)
{
	PVRSRV_ERROR eError;
	IMG_CHAR aszMemspaceName[PMR_MAX_MEMSPACE_NAME_LENGTH_DEFAULT];
	IMG_CHAR aszSymbolicName[PMR_MAX_SYMBOLIC_ADDRESS_LENGTH_DEFAULT];
	IMG_DEVMEM_OFFSET_T uiOutOffset;
	IMG_DEVMEM_OFFSET_T uiNextSymName;

	PVR_UNREFERENCED_PARAMETER(uiArraySize);

	PVR_ASSERT(uiOffset + uiSize <= psPMR->uiLogicalSize);

	eError = PMR_PDumpSymbolicAddr(psPMR,
				       uiOffset,
				       sizeof(aszMemspaceName),
				       &aszMemspaceName[0],
				       sizeof(aszSymbolicName),
				       &aszSymbolicName[0],
				       &uiOutOffset, &uiNextSymName);
	PVR_ASSERT(eError == PVRSRV_OK);
	PVR_ASSERT(uiOffset + uiSize <= uiNextSymName);

	eError = PDumpPMRSAB(aszMemspaceName,
			     aszSymbolicName,
			     uiOutOffset, uiSize, pszFilename, 0);
	PVR_ASSERT(eError == PVRSRV_OK);

	return PVRSRV_OK;
}
#endif				/* PDUMP */

/*
   FIXME: Find a better way to do this
 */

IMG_VOID *PMRGetPrivateDataHack(const PMR * psPMR,
				const PMR_IMPL_FUNCTAB * psFuncTab)
{
	PVR_ASSERT(psFuncTab == psPMR->psFuncTab);

	return psPMR->pvFlavourData;
}

PVRSRV_ERROR PMRWritePMPageList(	/* Target PMR, offset, and length */
				       PMR * psPageListPMR,
				       IMG_DEVMEM_OFFSET_T uiTableOffset,
				       IMG_DEVMEM_SIZE_T uiTableLength,
				       /* Referenced PMR, and "page" granularity */
				       PMR * psReferencePMR,
				       IMG_DEVMEM_LOG2ALIGN_T uiLog2PageSize,
				       PMR_PAGELIST ** ppsPageList)
{
	PVRSRV_ERROR eError;
	IMG_DEVMEM_SIZE_T uiWordSize;
	IMG_UINT32 uiNumPages;
	IMG_UINT32 uiPageIndex;
	PMR_FLAGS_T uiFlags;
	PMR_PAGELIST *psPageList;
#if defined(PDUMP)
	IMG_CHAR aszTableEntryMemspaceName[100];
	IMG_CHAR aszTableEntrySymbolicName[100];
	IMG_DEVMEM_OFFSET_T uiTableEntryPDumpOffset;
	IMG_CHAR aszPageMemspaceName[100];
	IMG_CHAR aszPageSymbolicName[100];
	IMG_DEVMEM_OFFSET_T uiPagePDumpOffset;
	IMG_DEVMEM_OFFSET_T uiNextSymName;
#endif
#if !defined(NO_HARDWARE)
	IMG_UINT32 uiPageListPageSize =
	    1 << psPageListPMR->uiLog2ContiguityGuarantee;
	IMG_BOOL bPageIsMapped = IMG_FALSE;
	IMG_UINT32 uiPageListPMRPage = 0;
	IMG_UINT32 uiPrevPageListPMRPage = 0;
	IMG_HANDLE hPrivData;
	IMG_VOID *pvKernAddr = IMG_NULL;
	IMG_UINT32 *pui32DataPtr;
#endif
	/* FIXME: should this be configurable? */
	uiWordSize = 4;

	if (uiWordSize != 4) {
		eError = PVRSRV_ERROR_NOT_SUPPORTED;
		goto e0;
	}

	/* check we're being asked to write the same number of 4-byte units as there are pages */
	uiNumPages =
	    (IMG_UINT32) (psReferencePMR->uiLogicalSize >> uiLog2PageSize);
	if (uiNumPages << uiLog2PageSize != psReferencePMR->uiLogicalSize) {
		/* Strictly speaking, it's possible to provoke this error in two ways:
		   (i) if it's not a whole multiple of the page size; or
		   (ii) if there are more than 4 billion pages.
		   The latter is unlikely. :)  but the check is required in order to justify the cast.
		 */
		eError = PVRSRV_ERROR_PMR_NOT_PAGE_MULTIPLE;
		goto e0;
	}
	if (uiNumPages * uiWordSize != uiTableLength) {
		eError = PVRSRV_ERROR_PMR_NOT_PAGE_MULTIPLE;
		goto e0;
	}
	if (psReferencePMR->uiLogicalSize != uiNumPages << uiLog2PageSize) {
		eError = PVRSRV_ERROR_PMR_NOT_PAGE_MULTIPLE;
		goto e0;
	}

	/* Check we're not being asked to write off the end of the PMR */
	if (uiTableOffset + uiTableLength > psPageListPMR->uiLogicalSize) {
		/* table memory insufficient to store all the entries */
		/* table insufficient to store addresses of whole block */
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	/* the PMR into which we are writing must not be user CPU mappable: */
	eError = PMR_Flags(psPageListPMR, &uiFlags);
	if ((eError != PVRSRV_OK) ||
	    ((uiFlags &
	      (PVRSRV_MEMALLOCFLAG_CPU_READABLE |
	       PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE)) != 0)) {
		PVR_DPF((PVR_DBG_ERROR, "eError = %d", eError));
		PVR_DPF((PVR_DBG_ERROR, "masked flags = 0x%08x",
			 (uiFlags &
			  (PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			   PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE))));
		PVR_DPF((PVR_DBG_ERROR,
			 "Page list PMR allows CPU mapping (0x%08x)", uiFlags));
		eError = PVRSRV_ERROR_DEVICEMEM_INVALID_PMR_FLAGS;
		goto e0;
	}

	psPageList = OSAllocMem(sizeof(PMR_PAGELIST));
	if (psPageList == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR, "Failed to allocation PMR page list"));
		goto e0;
	}
	psPageList->psReferencePMR = psReferencePMR;

	/* Need to lock down the physical addresses of the reference PMR */
	/* N.B.  This also checks that the requested "contiguity" is achievable */
	eError = PMRLockSysPhysAddresses(psReferencePMR, uiLog2PageSize);
	if (eError != PVRSRV_OK) {
		goto e1;
	}

	for (uiPageIndex = 0; uiPageIndex < uiNumPages; uiPageIndex++) {
#if !defined(NO_HARDWARE)
		IMG_DEV_PHYADDR sDevAddrPtr;
#endif
		IMG_DEVMEM_OFFSET_T uiPMROffset =
		    uiTableOffset + (uiWordSize * uiPageIndex);
#if defined(PDUMP)
		eError = PMR_PDumpSymbolicAddr(psPageListPMR,
					       uiPMROffset,
					       sizeof
					       (aszTableEntryMemspaceName),
					       &aszTableEntryMemspaceName[0],
					       sizeof
					       (aszTableEntrySymbolicName),
					       &aszTableEntrySymbolicName[0],
					       &uiTableEntryPDumpOffset,
					       &uiNextSymName);
		PVR_ASSERT(eError == PVRSRV_OK);

		eError = PMR_PDumpSymbolicAddr(psReferencePMR,
					       uiPageIndex << uiLog2PageSize,
					       sizeof(aszPageMemspaceName),
					       &aszPageMemspaceName[0],
					       sizeof(aszPageSymbolicName),
					       &aszPageSymbolicName[0],
					       &uiPagePDumpOffset,
					       &uiNextSymName);
		PVR_ASSERT(eError == PVRSRV_OK);

		eError = PDumpWriteShiftedMaskedValue(	/* destination */
							     aszTableEntryMemspaceName,
							     aszTableEntrySymbolicName,
							     uiTableEntryPDumpOffset,
							     /* source */
							     aszPageMemspaceName,
							     aszPageSymbolicName,
							     uiPagePDumpOffset,
							     /* shift right */
							     uiLog2PageSize,
							     /* shift left */
							     0,
							     /* mask */
							     0xffffffff,
							     /* word size */
							     uiWordSize,
							     /* flags */
							     PDUMP_FLAGS_CONTINUOUS);
		PVR_ASSERT(eError == PVRSRV_OK);
#else
		PVR_UNREFERENCED_PARAMETER(uiPMROffset);
#endif
#if !defined(NO_HARDWARE)
		PMR_DevPhysAddr(psReferencePMR,
				uiPageIndex << uiLog2PageSize, &sDevAddrPtr);

		uiPageListPMRPage =
		    uiPMROffset >> psReferencePMR->uiLog2ContiguityGuarantee;

		if ((bPageIsMapped == IMG_FALSE)
		    || (uiPageListPMRPage != uiPrevPageListPMRPage)) {
			IMG_SIZE_T uiMappingOffset =
			    uiPMROffset & (~(uiPageListPageSize - 1));
			IMG_SIZE_T uiMapedSize;

			if (bPageIsMapped == IMG_TRUE) {
				PMRReleaseKernelMappingData(psPageListPMR,
							    hPrivData);
			}

			eError = PMRAcquireKernelMappingData(psPageListPMR,
							     uiMappingOffset,
							     uiPageListPageSize,
							     &pvKernAddr,
							     &uiMapedSize,
							     &hPrivData);
			if (eError != PVRSRV_OK) {
				PVR_DPF((PVR_DBG_ERROR,
					 "Error mapping page list PMR page (%d) into kernel (%d)",
					 uiPageListPMRPage, eError));
				goto e2;
			}

			bPageIsMapped = IMG_TRUE;
			uiPrevPageListPMRPage = uiPageListPMRPage;
			PVR_ASSERT(uiMapedSize >= uiPageListPageSize);
			PVR_ASSERT(pvKernAddr != IMG_NULL);
		}

		/* Write the physcial page index into the page list PMR */
		pui32DataPtr =
		    (IMG_UINT32 *) (pvKernAddr +
				    (uiPMROffset & (uiPageListPageSize - 1)));
		*pui32DataPtr = sDevAddrPtr.uiAddr >> uiLog2PageSize;

		/* Last page so unmap */
		if (uiPageIndex == (uiNumPages - 1)) {
			PMRReleaseKernelMappingData(psPageListPMR, hPrivData);
		}
#endif
	}
	*ppsPageList = psPageList;
	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */
#if !defined(NO_HARDWARE)
 e2:
	PMRUnlockSysPhysAddresses(psReferencePMR);
#endif
 e1:
	OSFreeMem(psPageList);
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR			/* FIXME: should be IMG_VOID */
PMRUnwritePMPageList(PMR_PAGELIST * psPageList)
{
	PVRSRV_ERROR eError2;

	eError2 = PMRUnlockSysPhysAddresses(psPageList->psReferencePMR);
	PVR_ASSERT(eError2 == PVRSRV_OK);
	OSFreeMem(psPageList);

	return PVRSRV_OK;
}

#if defined(PDUMP)
extern PVRSRV_ERROR
PMRPDumpPol32(const PMR * psPMR,
	      IMG_DEVMEM_OFFSET_T uiOffset,
	      IMG_UINT32 ui32Value,
	      IMG_UINT32 ui32Mask,
	      PDUMP_POLL_OPERATOR eOperator, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;
	IMG_CHAR aszMemspaceName[100];
	IMG_CHAR aszSymbolicName[100];
	IMG_DEVMEM_OFFSET_T uiPDumpOffset;
	IMG_DEVMEM_OFFSET_T uiNextSymName;

	eError = PMR_PDumpSymbolicAddr(psPMR,
				       uiOffset,
				       sizeof(aszMemspaceName),
				       &aszMemspaceName[0],
				       sizeof(aszSymbolicName),
				       &aszSymbolicName[0],
				       &uiPDumpOffset, &uiNextSymName);
	if (eError != PVRSRV_OK) {
		goto e0;
	}
#define _MEMPOLL_DELAY		(1000)
#define _MEMPOLL_COUNT		(2000000000 / _MEMPOLL_DELAY)

	eError = PDumpPMRPOL(aszMemspaceName,
			     aszSymbolicName,
			     uiPDumpOffset,
			     ui32Value,
			     ui32Mask,
			     eOperator,
			     _MEMPOLL_COUNT, _MEMPOLL_DELAY, uiPDumpFlags);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR
PMRPDumpCBP(const PMR * psPMR,
	    IMG_DEVMEM_OFFSET_T uiReadOffset,
	    IMG_DEVMEM_OFFSET_T uiWriteOffset,
	    IMG_DEVMEM_SIZE_T uiPacketSize, IMG_DEVMEM_SIZE_T uiBufferSize)
{
	PVRSRV_ERROR eError;
	IMG_CHAR aszMemspaceName[100];
	IMG_CHAR aszSymbolicName[100];
	IMG_DEVMEM_OFFSET_T uiPDumpOffset;
	IMG_DEVMEM_OFFSET_T uiNextSymName;

	eError = PMR_PDumpSymbolicAddr(psPMR,
				       uiReadOffset,
				       sizeof(aszMemspaceName),
				       &aszMemspaceName[0],
				       sizeof(aszSymbolicName),
				       &aszSymbolicName[0],
				       &uiPDumpOffset, &uiNextSymName);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	eError = PDumpPMRCBP(aszMemspaceName,
			     aszSymbolicName,
			     uiPDumpOffset,
			     uiWriteOffset, uiPacketSize, uiBufferSize);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}
#endif

PVRSRV_ERROR PMRInit()
{
	if (_gsSingletonPMRContext.bModuleInitialised) {
		PVR_DPF((PVR_DBG_ERROR, "pmr.c:  oops, already initialized"));
		return PVRSRV_ERROR_PMR_UNRECOVERABLE_ERROR;
	}

	_gsSingletonPMRContext.uiNextSerialNum = 1;

	_gsSingletonPMRContext.uiNextKey =
	    0x8300f001 * (IMG_UINTPTR_T) & _gsSingletonPMRContext;

	/*
	   FIXME:

	   should initialise a mutex here
	 */

	_gsSingletonPMRContext.bModuleInitialised = IMG_TRUE;

	_gsSingletonPMRContext.uiNumLivePMRs = 0;

	return PVRSRV_OK;
}

PVRSRV_ERROR PMRDeInit()
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();

	if (psPVRSRVData->eServicesState != PVRSRV_SERVICES_STATE_OK) {
		return PVRSRV_OK;
	}

	PVR_ASSERT(_gsSingletonPMRContext.bModuleInitialised);
	if (!_gsSingletonPMRContext.bModuleInitialised) {
		PVR_DPF((PVR_DBG_ERROR, "pmr.c:  oops, not initialized"));
		return PVRSRV_ERROR_PMR_UNRECOVERABLE_ERROR;
	}

	PVR_ASSERT(_gsSingletonPMRContext.uiNumLivePMRs == 0);
	if (_gsSingletonPMRContext.uiNumLivePMRs != 0) {
		PVR_DPF((PVR_DBG_ERROR, "pmr.c:  %d live PMR(s) remain(s)",
			 _gsSingletonPMRContext.uiNumLivePMRs));
		PVR_DPF((PVR_DBG_ERROR,
			 "pmr.c:  This is an unrecoverable error; a subsequent crash is inevitable"));
		return PVRSRV_ERROR_PMR_UNRECOVERABLE_ERROR;
	}

	_gsSingletonPMRContext.bModuleInitialised = IMG_FALSE;

	/*
	   FIXME:

	   should deinitialise the mutex here
	 */

	return PVRSRV_OK;
}
