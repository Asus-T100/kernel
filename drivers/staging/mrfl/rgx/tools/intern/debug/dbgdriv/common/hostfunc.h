									    /*************************************************************************//*!
									       @File
									       @Description    32 Bit Highlander kernel manager VxD Services
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _HOSTFUNC_
#define _HOSTFUNC_

/*****************************************************************************
 Defines
*****************************************************************************/
#define HOST_PAGESIZE			(4096)
#define DBG_MEMORY_INITIALIZER	(0xe2)

/*****************************************************************************
 Function prototypes
*****************************************************************************/
IMG_UINT32 HostReadRegistryDWORDFromString(IMG_CHAR * pcKey,
					   IMG_CHAR * pcValueName,
					   IMG_UINT32 * pui32Data);

IMG_VOID *HostPageablePageAlloc(IMG_UINT32 ui32Pages);
IMG_VOID HostPageablePageFree(IMG_VOID * pvBase);
IMG_VOID *HostNonPageablePageAlloc(IMG_UINT32 ui32Pages);
IMG_VOID HostNonPageablePageFree(IMG_VOID * pvBase);

IMG_VOID *HostMapKrnBufIntoUser(IMG_VOID * pvKrnAddr, IMG_UINT32 ui32Size,
				IMG_VOID * *ppvMdl);
IMG_VOID HostUnMapKrnBufFromUser(IMG_VOID * pvUserAddr, IMG_VOID * pvMdl,
				 IMG_VOID * pvProcess);

IMG_VOID HostCreateRegDeclStreams(IMG_VOID);

IMG_VOID *HostCreateMutex(IMG_VOID);
IMG_VOID HostAquireMutex(IMG_VOID * pvMutex);
IMG_VOID HostReleaseMutex(IMG_VOID * pvMutex);
IMG_VOID HostDestroyMutex(IMG_VOID * pvMutex);

#if defined(SUPPORT_DBGDRV_EVENT_OBJECTS)
IMG_INT32 HostCreateEventObjects(IMG_VOID);
IMG_VOID HostWaitForEvent(DBG_EVENT eEvent);
IMG_VOID HostSignalEvent(DBG_EVENT eEvent);
IMG_VOID HostDestroyEventObjects(IMG_VOID);
#endif				/*defined(SUPPORT_DBGDRV_EVENT_OBJECTS) */

#endif

/*****************************************************************************
 End of file (HOSTFUNC.H)
*****************************************************************************/
