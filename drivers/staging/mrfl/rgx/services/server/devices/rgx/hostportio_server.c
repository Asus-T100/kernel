									    /*************************************************************************//*!
									       @File
									       @Title          Hostport services functions inmplementation
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Kernel services functions for hostport I/O:
									       Reading from and Writing to the hostport from the host.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "pvrsrv.h"
#include "pvr_debug.h"
#include "hostportio_server.h"
#include "rgxfwutils.h"
#include "mm.h"
#include "osfunc.h"
#include "pdump_km.h"
#include "sync_internal.h"

#if defined(PDUMP)
static PVRSRV_ERROR
PDumpHostPortWrite(IMG_CHAR * pcBuffer,
		   IMG_UINT32 ui32NumBytes,
		   IMG_UINT32 ui32WriteOffset, PDUMP_FLAGS_T uiPDumpFlags);

static PVRSRV_ERROR
PDumpHostPortRead(IMG_UINT32 ui32HPReadOffset,
		  IMG_UINT32 ui32NumLoadBytes, PDUMP_FLAGS_T uiPDumpFlags);

static PVRSRV_ERROR
PDumpHostPortWrite(IMG_CHAR * pcBuffer,
		   IMG_UINT32 ui32NumBytes,
		   IMG_UINT32 ui32WriteOffset, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;

	if ((eError = PDumpLDW(pcBuffer,
			       "HOSTPORT",
			       ui32WriteOffset, ui32NumBytes, uiPDumpFlags))) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDumpHostPortWrite: PDumpLDW failed. Error:%u",
			 eError));
		goto cleanup;
	}

 cleanup:
	return eError;
}

static PVRSRV_ERROR
PDumpHostPortRead(IMG_UINT32 ui32HPReadOffset,
		  IMG_UINT32 ui32NumLoadBytes, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVRSRV_ERROR eError;
	static IMG_UINT32 uiReadFileNum = 0;
	IMG_CHAR aszOutfileName[32];

	PVR_DPF((PVR_DBG_ERROR, "PDumpHostPortRead\n"));

	if ((eError = PDumpOSSprintf(aszOutfileName, sizeof(aszOutfileName),
				     "outfile%u.out", uiReadFileNum))) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDumpHostPortRead: PDumpOSSprintf failed. Error:%u",
			 eError));
		goto cleanup;
	}

	++uiReadFileNum;

	if ((eError = PDumpSAW("HOSTPORT",
			       ui32HPReadOffset,
			       ui32NumLoadBytes,
			       aszOutfileName, 0, uiPDumpFlags))) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PDumpHostPortRead: PDumpSAW failed. Error:%u",
			 eError));
		goto cleanup;
	}

 cleanup:
	return eError;
}
#endif				/* PDUMP */

static PVRSRV_ERROR
LocalHostportAcquireWrapper(DEVMEM_MEMDESC * psFWMemContextMemDesc,
			    IMG_UINT32 uiCRHostIFVal,
			    PVRSRV_DEVICE_NODE * psDeviceNode,
			    IMG_VOID ** ppApertureBase,
			    IMG_UINT32 * pui32ApertureSizeBytes)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD sHPCCBCmd;

	/* Schedule a hostport acquire command... */

	sHPCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_HPCTL;
	sHPCCBCmd.uCmdData.sHostPortData.ui32RgxCRHostIFVal = uiCRHostIFVal;
	sHPCCBCmd.uCmdData.sHostPortData.bAcquire = IMG_TRUE;
	RGXSetFirmwareAddress(&sHPCCBCmd.uCmdData.sHostPortData.psMemContext,
			      psFWMemContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);

	PDUMPCOMMENT("Scheduling hostport acquire command");
	if ((eError = RGXScheduleCommand(psDeviceNode->pvDevice,
					 RGXFWIF_DM_GP,
					 &sHPCCBCmd,
					 sizeof(sHPCCBCmd), IMG_FALSE))) {
		PVR_DPF((PVR_DBG_ERROR,
			 "LocalHostportAcquireWrapper: RGXScheduleCommand failed. Error:%u",
			 eError));
		goto cleanup;
	}

	/* Map the hostport aperture into our virtual memory space... */

	*pui32ApertureSizeBytes =
	    psDeviceNode->psDevConfig->ui32HPApertureSizeBytes;

#if !defined(NO_HARDWARE)
	{
		IMG_CPU_PHYADDR sApertureCPUPAddr =
		    psDeviceNode->psDevConfig->sHPApertureBasePAddr;

		PVR_DPF((PVR_DBG_ERROR,
			 "LocalHostportAcquireWrapper: OSMapPhysToLin %lu",
			 (unsigned long)sApertureCPUPAddr.uiAddr));
		if (IMG_NULL ==
		    (*ppApertureBase =
		     OSMapPhysToLin(sApertureCPUPAddr, *pui32ApertureSizeBytes,
				    0x0))) {
			PVR_DPF((PVR_DBG_ERROR,
				 "LocalHostportAcquireWrapper: OSMapPhysToLin failed. Error:%u",
				 eError));
			goto cleanup;
		}
	}
#else
	*ppApertureBase = IMG_NULL;
#endif

	/* Wait for FW to complete */
	eError =
	    RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, IMG_FALSE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "LocalHostportAcquireWrapper: Wait for completion aborted with error:%u",
			 eError));
	}

 cleanup:
	return eError;
}

static PVRSRV_ERROR
LocalHostportReleaseWrapper(IMG_PVOID pvApertureBase,
			    IMG_DEVMEM_SIZE_T uiApertureSizeBytes,
			    DEVMEM_MEMDESC * psFWMemContextMemDesc,
			    PVRSRV_DEVICE_NODE * psDeviceNode)
{
	RGXFWIF_KCCB_CMD sHPCCBCmd;
	PVRSRV_ERROR eError = PVRSRV_OK;

	/* Unmap the hortport aperture from our VM space ... */

#if !defined(NO_HARDWARE)
	OSUnMapPhysToLin(pvApertureBase, uiApertureSizeBytes, 0);
#endif

	/* Schedule a hostport release command ... */

	sHPCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_HPCTL;
	sHPCCBCmd.uCmdData.sHostPortData.ui32RgxCRHostIFVal = 0;
	sHPCCBCmd.uCmdData.sHostPortData.bAcquire = IMG_FALSE;
	RGXSetFirmwareAddress(&sHPCCBCmd.uCmdData.sHostPortData.psMemContext,
			      psFWMemContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);

	if ((eError = RGXScheduleCommand(psDeviceNode->pvDevice,
					 RGXFWIF_DM_GP,
					 &sHPCCBCmd,
					 sizeof(sHPCCBCmd), IMG_FALSE))) {

		PVR_DPF((PVR_DBG_ERROR,
			 "LocalHostportReleaseWrapper: RGXScheduleCommandfailed. Error:%u",
			 eError));
		goto cleanup;
	}

 cleanup:
	return eError;
}

									     /**************************************************************************//*!
									        @Function       PVRSRVHostPortReadKM

									        @Description    Read data from the hostport into the host.
									        NB: Consult PVRSRV_DEVICE_NODE.psDevConfig.ui32HPApertureSizeBytes
									        for the maximum number of bytes that can be read for each call.
									        If a request to read greater than that number of bytes, it will
									        be clamped to that value. You may need to make additional calls
									        to read all the bytes you need.

									        @Input          psDeviceNode -- device node associated with device memory context

									        @input          hMemCtxPrivData -- memory context to write into

									        @input          ui32CRHostIFVal -- value to set the CR_HOSTIF register to

									        @input          ui32ReadOffset -- offset from mapping base to begin reading. For
									        working around any granularity restrictions imposed by the
									        ui32CFHostIFVal argument.

									        @input          uiDstBufLen -- number of bytes to read

									        @input          pDstBuffer -- where to read the bytes into

									        @Output         pui32NumBytesRead -- number of bytes actually read through
									        the host port.

									        @Return         PVRSRV_ERROR
    *//***************************************************************************/

IMG_EXPORT PVRSRV_ERROR
PVRSRVHostPortReadKM(PVRSRV_DEVICE_NODE * psDeviceNode,
		     IMG_HANDLE hMemCtxPrivData,
		     IMG_UINT32 ui32CRHostIFVal,
		     IMG_UINT32 ui32ReadOffset,
		     IMG_DEVMEM_SIZE_T uiDstBufLen,
		     IMG_CHAR * pDstBuffer, IMG_DEVMEM_SIZE_T * puiNumBytesRead)
{
	PVRSRV_ERROR eError;
	IMG_VOID *pvApertureBase;
	DEVMEM_MEMDESC *psFWMemContextMemDesc = hMemCtxPrivData;

	IMG_UINT32 ui32ApertureSizeBytes;

	PVR_DPF((PVR_DBG_ERROR, "PVRSRVHostPortReadKM offset=%u buflen=%llu\n",
		 ui32CRHostIFVal, uiDstBufLen));

	/* Acquire the hostport, map it into our virtual memory space, copy data from it, and clean up. */

	PDUMPCOMMENT("Acquiring hostport");
	if ((eError = LocalHostportAcquireWrapper(psFWMemContextMemDesc,
						  ui32CRHostIFVal,
						  psDeviceNode,
						  &pvApertureBase,
						  &ui32ApertureSizeBytes))) {
		goto fail;
	}

	PVR_DPF((PVR_DBG_ERROR, "PVRSRVHostPortReadKM aperture size=%u\n",
		 ui32ApertureSizeBytes));

	/* Copy data from the hostport to the destination buffer... */

	/* don't read too much */
	if (uiDstBufLen + ui32ReadOffset > ui32ApertureSizeBytes) {
		eError = PVRSRV_ERROR_HP_REQUEST_TOO_LONG;
		goto cleanup;
	}

	PDUMPCOMMENT("copying data from hostport");

#if defined(PDUMP)
	if ((eError = PDumpHostPortRead(ui32ReadOffset, uiDstBufLen, 0))) {
		goto cleanup;
	}
#endif

	*puiNumBytesRead =
	    OSReadHWRegBank(pvApertureBase, ui32ReadOffset, pDstBuffer,
			    uiDstBufLen);

	/* Unmap and release the hostport ... */
	if ((eError = LocalHostportReleaseWrapper(pvApertureBase,
						  ui32ApertureSizeBytes,
						  psFWMemContextMemDesc,
						  psDeviceNode))) {
		goto fail;
	}

	return PVRSRV_OK;

 cleanup:
	/* no error checking, we're doing the best we can if we get here... */
	LocalHostportReleaseWrapper(pvApertureBase,
				    ui32ApertureSizeBytes,
				    psFWMemContextMemDesc, psDeviceNode);

 fail:
	return eError;
}

									     /**************************************************************************//*!
									        @Function       PVRSRVHostPortWriteKM

									        @Description    Write data from the host into the hostport.
									        NB: Consult PVRSRV_DEVICE_NODE.psDevConfig.ui32HPApertureSizeBytes
									        for the maximum number of bytes that can be written for each
									        call. If a request to write greater than that number of bytes,
									        it will be clamped to that value. You may need to make
									        additional calls to write the entirety of your buffer.

									        @Input          psDeviceNode -- device node associated with device memory context

									        @input          hMemCtxPrivData -- memory context to write into

									        @input          ui32CRHostIFVal -- value to set the CR_HOSTIF register to

									        @input          ui32WriteOffset -- offset from mapping base to begin writing. For
									        working around any granularity restrictions imposed by the
									        ui32CFHostIFVal argument.

									        @input          ui32SrcBufLen -- number of bytes to write

									        @input          pSrcBuffer -- the buffer to write into the host port.

									        @Output         pui32NumBytesWritten -- number of bytes actually written through
									        the host port.

									        @Return         PVRSRV_ERROR
    *//***************************************************************************/

IMG_EXPORT PVRSRV_ERROR
PVRSRVHostPortWriteKM(PVRSRV_DEVICE_NODE * psDeviceNode,
		      IMG_HANDLE hMemCtxPrivData,
		      IMG_UINT32 ui32CRHostIFVal,
		      IMG_UINT32 ui32WriteOffset,
		      IMG_DEVMEM_SIZE_T uiSrcBufLen,
		      IMG_CHAR * pSrcBuffer,
		      IMG_DEVMEM_SIZE_T * puiNumBytesWritten)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32ApertureSizeBytes = 0;
	IMG_VOID *pvApertureBase;
	DEVMEM_MEMDESC *psFWMemContextMemDesc = hMemCtxPrivData;

	PVR_DPF((PVR_DBG_ERROR, "PVRSRVHostPortWriteKM offset=%u buflen=%llu\n",
		 ui32CRHostIFVal, uiSrcBufLen));

	/* Acquire the hostport, map it into our virtual memory space, copy data from it, and clean up. */

	PDUMPCOMMENT("Acquiring hostport");
	if ((eError = LocalHostportAcquireWrapper(psFWMemContextMemDesc,
						  ui32CRHostIFVal,
						  psDeviceNode,
						  &pvApertureBase,
						  &ui32ApertureSizeBytes))) {
		goto fail;
	}

	PVR_DPF((PVR_DBG_ERROR, "PVRSRVHostPortWriteKM aperture size=%u\n",
		 ui32ApertureSizeBytes));

	/* Copy data from the hostport to the destination buffer... */

	/* don't write too much */
	if (uiSrcBufLen + ui32WriteOffset > ui32ApertureSizeBytes) {
		eError = PVRSRV_ERROR_HP_REQUEST_TOO_LONG;
		goto cleanup;
	}

	PDUMPCOMMENT("copying data into hostport");

#if defined(PDUMP)
	if ((eError =
	     PDumpHostPortWrite(pSrcBuffer, uiSrcBufLen, ui32WriteOffset, 0))) {
		goto cleanup;
	}
#endif

	*puiNumBytesWritten =
	    OSWriteHWRegBank(pvApertureBase, ui32WriteOffset, pSrcBuffer,
			     uiSrcBufLen);

	/* Unmap and release the hostport ... */
	if ((eError = LocalHostportReleaseWrapper(pvApertureBase,
						  ui32ApertureSizeBytes,
						  psFWMemContextMemDesc,
						  psDeviceNode))) {
		goto fail;
	}

	return PVRSRV_OK;

 cleanup:
	/* no error checking, we're doing the best we can if we get here... */
	LocalHostportReleaseWrapper(pvApertureBase,
				    ui32ApertureSizeBytes,
				    psFWMemContextMemDesc, psDeviceNode);

 fail:
	return eError;
}
