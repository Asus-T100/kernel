									    /*************************************************************************//*!
									       @File           scp.c
									       @Title          Software Command Processor
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    The software command processor allows commands queued and
									       deferred until their synchronisation requirements have been meet.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "scp.h"
#include "lists.h"
#include "allocmem.h"
#include "pvrsrv.h"
#include "pvr_debug.h"
#include "osfunc.h"
#include "sync_server.h"

struct _SCP_CONTEXT_ {
	IMG_VOID *pvCCB;	/*!< Pointer to the command circler buffer */
	volatile IMG_UINT32 ui32DepOffset;	/*!< Dependency offset  */
	volatile IMG_UINT32 ui32ReadOffset;	/*!< Read offset */
	volatile IMG_UINT32 ui32WriteOffset;	/*!< Write offset */
	IMG_UINT32 ui32CCBSize;	/*!< CCB size */
};

typedef struct _SCP_SYNC_DATA_ {
	SERVER_SYNC_PRIMITIVE *psSync;	/*!< Server sync */
	IMG_UINT32 ui32Fence;	/*!< Fence value to check for */
	IMG_UINT32 ui32Update;	/*!< Fence update value */
} SCP_SYNC_DATA;

#define SCP_COMMAND_INVALID     0	/*!< Invalid command */
#define SCP_COMMAND_CALLBACK    1	/*!< Command with callbacks */
#define SCP_COMMAND_PADDING     2	/*!< Padding */
typedef struct _SCP_COMMAND_ {
	IMG_UINT32 ui32CmdType;	/*!< Command type */
	IMG_UINT32 ui32CmdSize;	/*!< Total size of the command (i.e. includes header) */
	IMG_UINT32 ui32SyncCount;	/*!< Total number of syncs in pasSync */
	SCP_SYNC_DATA *pasSCPSyncData;	/*!< Pointer to the array of sync data (allocated in the CCB) */
	SCPReady pfnReady;	/*!< Pointer to the funtion to call when the command is ready */
	IMG_PVOID pvReadyData;	/*!< Data to pass into pfnReady */
	IMG_PVOID pvCompleteData;	/*!< Data to pass into pfnComplete */
} SCP_COMMAND;

#define GET_CCB_SPACE(WOff, ROff, CCBSize) \
	((((ROff) - (WOff)) + ((CCBSize) - 1)) & ((CCBSize) - 1))

#define UPDATE_CCB_OFFSET(Off, PacketSize, CCBSize) \
	(Off) = (((Off) + (PacketSize)) & ((CCBSize) - 1))

#define PADDING_COMMAND_SIZE	(sizeof(SCP_COMMAND))

#if defined(SCP_DEBUG)
#define SCP_DEBUG_PRINT(fmt, ...) \
	PVRSRVDebugPrintf(PVR_DBG_WARNING, \
					  fmt, \
					  __VA_ARGS__)
#else
#define SCP_DEBUG_PRINT(fmt, ...)
#endif

/*****************************************************************************
 *                          Internal functions                               *
 *****************************************************************************/

									    /*************************************************************************//*!
									       @Function       __SCPAlloc

									       @Description    Allocate space in the software command processor.

									       @Input          psContext            Context to allocate from

									       @Input          ui32Size                Size to allocate

									       @Output         ppvBufferSpace          Pointer to space allocated

									       @Return         PVRSRV_OK if the allocation was successful
									     */
/*****************************************************************************/
static
PVRSRV_ERROR __SCPAlloc(SCP_CONTEXT * psContext,
			IMG_UINT32 ui32Size, IMG_PVOID * ppvBufferSpace)
{
	IMG_UINT32 ui32FreeSpace;

	ui32FreeSpace = GET_CCB_SPACE(psContext->ui32WriteOffset,
				      psContext->ui32ReadOffset,
				      psContext->ui32CCBSize);
	if (ui32FreeSpace >= ui32Size) {
		*ppvBufferSpace = (IMG_PVOID) ((IMG_UINT8 *) psContext->pvCCB +
					       psContext->ui32WriteOffset);
		return PVRSRV_OK;
	} else {
		return PVRSRV_ERROR_RETRY;
	}
}

									    /*************************************************************************//*!
									       @Function       _SCPAlloc

									       @Description    Allocate space in the software command processor, handling the
									       case where we wrap around the CCB.

									       @Input          psContext            Context to allocate from

									       @Input          ui32Size                Size to allocate

									       @Output         ppvBufferSpace          Pointer to space allocated

									       @Return         PVRSRV_OK if the allocation was successful
									     */
/*****************************************************************************/
static
PVRSRV_ERROR _SCPAlloc(SCP_CONTEXT * psContext,
		       IMG_UINT32 ui32Size, IMG_PVOID * ppvBufferSpace)
{
	if ((ui32Size + PADDING_COMMAND_SIZE) > psContext->ui32CCBSize) {
		PVR_DPF((PVR_DBG_WARNING, "Command size (%d) too big for CCB\n",
			 ui32Size));
		return PVRSRV_ERROR_CMD_TOO_BIG;
	}

	/*
	   Check we don't overflow the end of the buffer and make sure we have
	   enough for the padding command
	 */
	if ((psContext->ui32WriteOffset + ui32Size + PADDING_COMMAND_SIZE) >
	    psContext->ui32CCBSize) {
		SCP_COMMAND *psCommand;
		IMG_PVOID pvCommand;
		PVRSRV_ERROR eError;
		IMG_UINT32 ui32Remain =
		    psContext->ui32CCBSize - psContext->ui32WriteOffset;

		/* We're at the end of the buffer without enough contiguous space */
		eError = __SCPAlloc(psContext, ui32Remain, &pvCommand);
		if (eError != PVRSRV_OK) {
			PVR_ASSERT(eError == PVRSRV_ERROR_RETRY);
			return eError;
		}
		psCommand = pvCommand;
		psCommand->ui32CmdType = SCP_COMMAND_PADDING;
		psCommand->ui32CmdSize = ui32Remain;

		UPDATE_CCB_OFFSET(psContext->ui32WriteOffset, ui32Remain,
				  psContext->ui32CCBSize);
	}

	return __SCPAlloc(psContext, ui32Size, ppvBufferSpace);
}

									    /*************************************************************************//*!
									       @Function       _SCPInsert

									       @Description    Insert the a finished command that was written into the CCB
									       space allocated in a previous call to _SCPAlloc.
									       This makes the command ready to be processed.

									       @Input          psContext               Context to allocate from

									       @Input          ui32Size                Size to allocate

									       @Return         None
									     */
/*****************************************************************************/
static
IMG_VOID _SCPInsert(SCP_CONTEXT * psContext, IMG_UINT32 ui32Size)
{
	/*
	 * Update the write offset.
	 */
	UPDATE_CCB_OFFSET(psContext->ui32WriteOffset,
			  ui32Size, psContext->ui32CCBSize);
}

									    /*************************************************************************//*!
									       @Function       _SCPProcessCommand

									       @Description    Try and process a command. Checks to see if the command
									       has had it's fences meet and is ready to go.

									       @Input          psCommand               Command to run

									       @Return         PVRSRV_OK if the command has been made ready
									     */
/*****************************************************************************/
static
PVRSRV_ERROR _SCPProcessCommand(SCP_COMMAND * psCommand)
{
	IMG_UINT32 i;

	PVR_ASSERT(psCommand->ui32CmdType != SCP_COMMAND_INVALID);

	if (psCommand->ui32CmdType == SCP_COMMAND_PADDING) {
		return PVRSRV_OK;
	}

	for (i = 0; i < psCommand->ui32SyncCount; i++) {
		SCP_SYNC_DATA *psSCPSyncData = &psCommand->pasSCPSyncData[i];

		if (!ServerSyncFenceIsMeet
		    (psSCPSyncData->psSync, psSCPSyncData->ui32Fence)) {
			return PVRSRV_ERROR_FAILED_DEPENDENCIES;
		}
	}

	/* Command is ready */
	if (psCommand->
	    pfnReady(psCommand->pvReadyData, psCommand->pvCompleteData)) {
		return PVRSRV_OK;
	}

	/*
	   If we got here it means the command is ready to go, but the SCP client
	   isn't ready for the command
	 */
	return PVRSRV_ERROR_NOT_READY;
}

/*****************************************************************************
 *                    Public interface functions                             *
 *****************************************************************************/

/*
	SCPCreate
*/
IMG_EXPORT
    PVRSRV_ERROR IMG_CALLCONV SCPCreate(IMG_UINT32 ui32CCBSizeLog2,
					SCP_CONTEXT ** ppsContext)
{
	SCP_CONTEXT *psContext;
	IMG_UINT32 ui32Power2QueueSize = 1 << ui32CCBSizeLog2;
	PVRSRV_ERROR eError;

	/* allocate an internal queue info structure */
	psContext = OSAllocMem(sizeof(SCP_CONTEXT));
	if (psContext == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "SCPCreate: Failed to alloc queue struct"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorExit;
	}
	OSMemSet(psContext, 0, sizeof(SCP_CONTEXT));

	/* allocate the command queue buffer - allow for overrun */
	psContext->pvCCB = OSAllocMem(ui32Power2QueueSize);
	if (psContext->pvCCB == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "SCPCreate: Failed to alloc queue buffer"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorExit;
	}

	/* Sanity check: Should be zeroed by OSMemSet */
	PVR_ASSERT(psContext->ui32ReadOffset == 0);
	PVR_ASSERT(psContext->ui32WriteOffset == 0);

	psContext->ui32CCBSize = ui32Power2QueueSize;

	SCP_DEBUG_PRINT("%s: New SCP %p of size %d",
			__FUNCTION__, psContext, ui32Power2QueueSize);

	*ppsContext = psContext;

	return PVRSRV_OK;

 ErrorExit:
	if (psContext) {
		if (psContext->pvCCB) {
			OSFreeMem(psContext->pvCCB);
			psContext->pvCCB = IMG_NULL;
		}

		OSFreeMem(psContext);
	}

	return eError;
}

/*
	SCPAllocCommand
*/
IMG_EXPORT
    PVRSRV_ERROR IMG_CALLCONV SCPAllocCommand(SCP_CONTEXT * psContext,
					      IMG_UINT32 ui32SyncPrimCount,
					      SERVER_SYNC_PRIMITIVE ** papsSync,
					      SCPReady pfnCommandReady,
					      IMG_SIZE_T ui32ReadyDataByteSize,
					      IMG_SIZE_T
					      ui32CompleteDataByteSize,
					      IMG_PVOID * ppvReadyData,
					      IMG_PVOID * ppvCompleteData)
{
	PVRSRV_ERROR eError;
	SCP_COMMAND *psCommand;
	IMG_UINT32 ui32CommandSize;
	IMG_UINT32 ui32SyncOpSize;
	IMG_UINT32 i;

	/* Round up the incoming data sizes to be pointer granular */
	ui32ReadyDataByteSize =
	    (ui32ReadyDataByteSize & (~(sizeof(IMG_PVOID) - 1))) +
	    sizeof(IMG_PVOID);
	ui32CompleteDataByteSize =
	    (ui32CompleteDataByteSize & (~(sizeof(IMG_PVOID) - 1))) +
	    sizeof(IMG_PVOID);

	ui32SyncOpSize =
	    (sizeof(PVRSRV_CLIENT_SYNC_PRIM_OP) * ui32SyncPrimCount);

	/* Total command size */
	ui32CommandSize = sizeof(SCP_COMMAND) +
	    ui32SyncOpSize + ui32ReadyDataByteSize + ui32CompleteDataByteSize;

	eError =
	    _SCPAlloc(psContext, ui32CommandSize, (IMG_VOID **) & psCommand);
	if (eError != PVRSRV_OK) {
		SCP_DEBUG_PRINT
		    ("%s: Failed to allocate command of size %d for ctx %p (%d)",
		     __FUNCTION__, ui32CommandSize, psContext, eError);
		return eError;
	}

	SCP_DEBUG_PRINT
	    ("%s: New Command %p for ctx %p of size %d, syncCount: %d",
	     __FUNCTION__, psCommand, psContext, ui32CommandSize,
	     ui32SyncPrimCount);

	/* setup the command */
	psCommand->ui32CmdSize = ui32CommandSize;
	psCommand->ui32CmdType = SCP_COMMAND_CALLBACK;
	psCommand->ui32SyncCount = ui32SyncPrimCount;

	/* Set up command pointers */
	psCommand->pasSCPSyncData =
	    (SCP_SYNC_DATA *) (((IMG_CHAR *) psCommand) + sizeof(SCP_COMMAND));

	psCommand->pfnReady = pfnCommandReady;

	psCommand->pvReadyData = ((IMG_CHAR *) psCommand) +
	    sizeof(SCP_COMMAND) + ui32SyncOpSize;
	psCommand->pvCompleteData = ((IMG_CHAR *) psCommand) +
	    sizeof(SCP_COMMAND) + ui32SyncOpSize + ui32ReadyDataByteSize;

	/* Copy over the sync data */
	for (i = 0; i < ui32SyncPrimCount; i++) {
		SCP_SYNC_DATA *psSCPSyncData = &psCommand->pasSCPSyncData[i];

		psSCPSyncData->psSync = papsSync[i];
		PVRSRVServerSyncQueueSWOpKM(papsSync[i],
					    &psSCPSyncData->ui32Fence,
					    &psSCPSyncData->ui32Update);
	}

	*ppvReadyData = psCommand->pvReadyData;
	*ppvCompleteData = psCommand->pvCompleteData;

	return PVRSRV_OK;
}

/*
	SCPSubmitCommand
*/
IMG_EXPORT PVRSRV_ERROR SCPSubmitCommand(SCP_CONTEXT * psContext)
{
	SCP_COMMAND *psCommand;
	PVRSRV_ERROR eError;

	if (psContext == IMG_NULL) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psCommand = (SCP_COMMAND *) ((IMG_UINT8 *) psContext->pvCCB
				     + psContext->ui32WriteOffset);

	SCP_DEBUG_PRINT("%s: Submit command %p for ctx %p",
			__FUNCTION__, psCommand, psContext);

	_SCPInsert(psContext, psCommand->ui32CmdSize);

	/* The command we inserted might already be ready to run */
	eError = SCPRun(psContext);
	PVR_ASSERT(eError == PVRSRV_OK);

	return PVRSRV_OK;
}

/*
	SCPRun
*/
IMG_EXPORT PVRSRV_ERROR SCPRun(SCP_CONTEXT * psContext)
{
	SCP_COMMAND *psCommand;

	if (psContext == IMG_NULL) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	while (psContext->ui32DepOffset != psContext->ui32WriteOffset) {
		PVRSRV_ERROR eError;

		psCommand = (SCP_COMMAND *) ((IMG_UINT8 *) psContext->pvCCB +
					     psContext->ui32DepOffset);

		eError = _SCPProcessCommand(psCommand);

		SCP_DEBUG_PRINT("%s: Processes command %p for ctx %p (%d)",
				__FUNCTION__, psCommand, psContext, eError);

		if (eError == PVRSRV_OK) {
			/* processed cmd so update queue */
			UPDATE_CCB_OFFSET(psContext->ui32DepOffset,
					  psCommand->ui32CmdSize,
					  psContext->ui32CCBSize);
		} else {
			/* As soon as we hit a command that can't run break out */
			break;
		}
	}

	return PVRSRV_OK;
}

IMG_EXPORT IMG_VOID SCPFlush(SCP_CONTEXT * psContext)
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	IMG_HANDLE hOSEvent;

	OSEventObjectOpen(psPVRSRVData->hGlobalEventObject, &hOSEvent);

	/* 
	   Loop until we've flush out all commands from the SCP.
	   There is no benefit adding some arbitrary time-out as it means we would
	   end up freeing the SCP with commands unprocessed and thus 
	   unprocessed fence updates which potentiality cause much harder
	   to diagnose issues.
	 */

	while (psPVRSRVData->eServicesState == PVRSRV_SERVICES_STATE_OK) {
		SCPRun(psContext);

		if (psContext->ui32DepOffset != psContext->ui32WriteOffset) {
			OSEventObjectWait(hOSEvent);
		} else {
			break;
		}
	}
	OSEventObjectClose(hOSEvent);
}

/*
	SCPCommandComplete
*/
IMG_EXPORT IMG_VOID SCPCommandComplete(SCP_CONTEXT * psContext)
{
	SCP_COMMAND *psCommand;
	IMG_UINT32 i;
	IMG_BOOL bContinue = IMG_TRUE;

	if (psContext == IMG_NULL) {
		return;
	}

	if (psContext->ui32ReadOffset == psContext->ui32DepOffset) {
		PVR_DPF((PVR_DBG_ERROR,
			 "SCPCommandComplete: Called with no work to do!"));
		return;
	}

	while (bContinue) {
		psCommand = (SCP_COMMAND *) ((IMG_UINT8 *) psContext->pvCCB +
					     psContext->ui32ReadOffset);

		if (psCommand->ui32CmdType == SCP_COMMAND_CALLBACK) {
			/* Do any fence updates */
			for (i = 0; i < psCommand->ui32SyncCount; i++) {
				SCP_SYNC_DATA *psSCPSyncData =
				    &psCommand->pasSCPSyncData[i];

				ServerSyncCompleteOp(psSCPSyncData->psSync,
						     psSCPSyncData->ui32Update);
			}

			bContinue = IMG_FALSE;
		}
		/* processed cmd so update queue */
		UPDATE_CCB_OFFSET(psContext->ui32ReadOffset,
				  psCommand->ui32CmdSize,
				  psContext->ui32CCBSize);

		SCP_DEBUG_PRINT
		    ("%s: Complete command %p for ctx %p (continue: %d)",
		     __FUNCTION__, psCommand, psContext, bContinue);

	}

}

/*
	SCPDestroy
*/
IMG_EXPORT IMG_VOID IMG_CALLCONV SCPDestroy(SCP_CONTEXT * psContext)
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	IMG_HANDLE hOSEvent;

	OSEventObjectOpen(psPVRSRVData->hGlobalEventObject, &hOSEvent);

	/*
	   Loop until all commands have been completed
	   We can't time-out as in-flight commands have pointers
	   into our CCB
	 */
	while (psContext->ui32ReadOffset != psContext->ui32WriteOffset) {
		if (psPVRSRVData->eServicesState == PVRSRV_SERVICES_STATE_OK) {
			OSEventObjectWait(hOSEvent);
		} else {
			break;
		}
	}
	OSEventObjectClose(hOSEvent);

	OSFreeMem(psContext->pvCCB);
	psContext->pvCCB = IMG_NULL;
	OSFreeMem(psContext);
}
