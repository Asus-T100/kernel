									    /*************************************************************************//*!
									       @File
									       @Title          Device specific pdump routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Device specific pdump functions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "devicemem_pdump.h"
#include "rgxpdump.h"

/*
 * PVRSRVPDumpSignatureBufferKM
 */
PVRSRV_ERROR PVRSRVPDumpSignatureBufferKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					  IMG_UINT32 ui32PDumpFlags)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;

	/* TA signatures */
	PDumpCommentWithFlags(ui32PDumpFlags,
			      "** Dump TA signatures and checksums Buffer");

	DevmemPDumpSaveToFileVirtual(psDevInfo->psRGXFWSigTAChecksMemDesc,
				     0,
				     psDevInfo->ui32SigTAChecksSize,
				     "out.tasig", 0, ui32PDumpFlags);

	/* 3D signatures */
	PDumpCommentWithFlags(ui32PDumpFlags,
			      "** Dump 3D signatures and checksums Buffer");
	DevmemPDumpSaveToFileVirtual(psDevInfo->psRGXFWSig3DChecksMemDesc, 0,
				     psDevInfo->ui32Sig3DChecksSize,
				     "out.3dsig", 0, ui32PDumpFlags);

	return PVRSRV_OK;
}

IMG_EXPORT
    PVRSRV_ERROR PVRSRVPDumpTraceBufferKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					  IMG_UINT32 ui32PDumpFlags)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;

	/* Dump trace buffer */
	PDumpCommentWithFlags(ui32PDumpFlags, "** Dump trace Buffer");
	DevmemPDumpSaveToFileVirtual(psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
				     offsetof(RGXFWIF_TRACEBUF,
					      ui32TracePointer),
				     (RGXFW_TRACE_BUFFER_SIZE +
				      1) * sizeof(IMG_UINT32), "out.trace", 0,
				     ui32PDumpFlags);

	return PVRSRV_OK;
}

/******************************************************************************
 End of file (rgxpdump.c)
******************************************************************************/
