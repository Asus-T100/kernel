									    /*************************************************************************//*!
									       @File                    pci_support.h

									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved

									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "img_types.h"
#include "pvrsrv_error.h"
typedef enum _HOST_PCI_INIT_FLAGS_ {
	HOST_PCI_INIT_FLAG_BUS_MASTER = 0x00000001,
	HOST_PCI_INIT_FLAG_MSI = 0x00000002,
	HOST_PCI_INIT_FLAG_FORCE_I32 = 0x7fffffff
} HOST_PCI_INIT_FLAGS;

struct _PVRSRV_PCI_DEV_OPAQUE_STRUCT_;
typedef struct _PVRSRV_PCI_DEV_OPAQUE_STRUCT_ *PVRSRV_PCI_DEV_HANDLE;

PVRSRV_PCI_DEV_HANDLE OSPCIAcquireDev(IMG_UINT16 ui16VendorID,
				      IMG_UINT16 ui16DeviceID,
				      HOST_PCI_INIT_FLAGS eFlags);
PVRSRV_PCI_DEV_HANDLE OSPCISetDev(IMG_VOID * pvPCICookie,
				  HOST_PCI_INIT_FLAGS eFlags);
PVRSRV_ERROR OSPCIReleaseDev(PVRSRV_PCI_DEV_HANDLE hPVRPCI);
PVRSRV_ERROR OSPCIIRQ(PVRSRV_PCI_DEV_HANDLE hPVRPCI, IMG_UINT32 * pui32IRQ);
IMG_UINT32 OSPCIAddrRangeLen(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
			     IMG_UINT32 ui32Index);
IMG_UINT32 OSPCIAddrRangeStart(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
			       IMG_UINT32 ui32Index);
IMG_UINT32 OSPCIAddrRangeEnd(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
			     IMG_UINT32 ui32Index);
PVRSRV_ERROR OSPCIRequestAddrRange(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
				   IMG_UINT32 ui32Index);
PVRSRV_ERROR OSPCIReleaseAddrRange(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
				   IMG_UINT32 ui32Index);
PVRSRV_ERROR OSPCISuspendDev(PVRSRV_PCI_DEV_HANDLE hPVRPCI);
PVRSRV_ERROR OSPCIResumeDev(PVRSRV_PCI_DEV_HANDLE hPVRPCI);
