									    /*************************************************************************//*!
									       @File
									       @Title          RGX breakpoint functionality
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the RGX breakpoint functionality
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXBREAKPOINT_H__)
#define __RGXBREAKPOINT_H__

#include "pvr_debug.h"
#include "rgxutils.h"
#include "rgxfwutils.h"
#include "rgx_fwif_km.h"

/*!
*******************************************************************************
 @Function	PVRSRVRGXSetBreakpointKM

 @Description
	Server-side implementation of RGXSetBreakpoint

 @Input psDeviceNode - RGX Device node
 @Input eDataMaster - Data Master to schedule command for
 @Input hMemCtxPrivData - FIXME
 @Input ui32BPAddr - Address of breakpoint
 @Input ui32HandlerAddr - Address of breakpoint handler
 @Input ui32BPCtl - Breakpoint controls

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXSetBreakpointKM(PVRSRV_DEVICE_NODE * psDeviceNode,
				      IMG_HANDLE hMemCtxPrivData,
				      RGXFWIF_DM eFWDataMaster,
				      IMG_UINT32 ui32BPAddr,
				      IMG_UINT32 ui32HandlerAddr,
				      IMG_UINT32 ui32DataMaster);

/*!
*******************************************************************************
 @Function	PVRSRVRGXClearBreakpointKM

 @Description
	Server-side implementation of RGXClearBreakpoint

 @Input psDeviceNode - RGX Device node
 @Input hMemCtxPrivData - FIXME

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXClearBreakpointKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					IMG_HANDLE hMemCtxPrivData);

/*!
*******************************************************************************
 @Function	PVRSRVRGXEnableBreakpointKM

 @Description
	Server-side implementation of RGXEnableBreakpoint

 @Input psDeviceNode - RGX Device node
 @Input hMemCtxPrivData - FIXME

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXEnableBreakpointKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					 IMG_HANDLE hMemCtxPrivData);

/*!
*******************************************************************************
 @Function	PVRSRVRGXDisableBreakpointKM

 @Description
	Server-side implementation of RGXDisableBreakpoint

 @Input psDeviceNode - RGX Device node
 @Input hMemCtxPrivData - FIXME

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXDisableBreakpointKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					  IMG_HANDLE hMemCtxPrivData);

/*!
*******************************************************************************
 @Function	PVRSRVRGXOverallocateBPRegistersKM

 @Description
	Server-side implementation of RGXOverallocateBPRegisters

 @Input psDeviceNode - RGX Device node
 @Input ui32TempRegs - Number of temporary registers to overallocate
 @Input ui32SharedRegs - Number of shared registers to overallocate

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXOverallocateBPRegistersKM(PVRSRV_DEVICE_NODE *
						psDeviceNode,
						IMG_UINT32 ui32TempRegs,
						IMG_UINT32 ui32SharedRegs);
#endif				/* __RGXBREAKPOINT_H__ */
