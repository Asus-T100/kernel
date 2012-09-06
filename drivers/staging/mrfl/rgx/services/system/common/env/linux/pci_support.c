									    /*************************************************************************//*!
									       @File                    pci_support.c

									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved

									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <linux/pci.h>
#include <linux/version.h>

#include "pci_support.h"

typedef struct _PVR_PCI_DEV_TAG {
	struct pci_dev *psPCIDev;
	HOST_PCI_INIT_FLAGS ePCIFlags;
	IMG_BOOL abPCIResourceInUse[DEVICE_COUNT_RESOURCE];
} PVR_PCI_DEV;

/*!
******************************************************************************

 @Function	OSPCISetDev
 
 @Description 
 
 Set a PCI device for subsequent use.

 @input pvPCICookie :	Pointer to OS specific PCI structure/cookie

 @input eFlags :	Flags

 @Return   Pointer to PCI device handle

******************************************************************************/
PVRSRV_PCI_DEV_HANDLE OSPCISetDev(IMG_VOID * pvPCICookie,
				  HOST_PCI_INIT_FLAGS eFlags)
{
	int err;
	IMG_UINT32 i;
	PVR_PCI_DEV *psPVRPCI;

	psPVRPCI = kmalloc(sizeof(*psPVRPCI), GFP_KERNEL);
	if (psPVRPCI == IMG_NULL) {
		printk(KERN_ERR
		       "OSPCISetDev: Couldn't allocate PVR PCI structure\n");
		return IMG_NULL;
	}

	psPVRPCI->psPCIDev = (struct pci_dev *)pvPCICookie;
	psPVRPCI->ePCIFlags = eFlags;

	err = pci_enable_device(psPVRPCI->psPCIDev);
	if (err != 0) {
		printk(KERN_ERR "OSPCISetDev: Couldn't enable device (%d)\n",
		       err);
		return IMG_NULL;
	}

	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_BUS_MASTER) {	/* PRQA S 3358 */
		/* misuse of enums */
		pci_set_master(psPVRPCI->psPCIDev);
	}

	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_MSI) {	/* PRQA S 3358 */
		/* misuse of enums */
#if defined(CONFIG_PCI_MSI)
		err = pci_enable_msi(psPVRPCI->psPCIDev);
		if (err != 0) {
			printk(KERN_ERR "OSPCISetDev: Couldn't enable MSI (%d)",
			       err);
			psPVRPCI->ePCIFlags &= ~HOST_PCI_INIT_FLAG_MSI;	/* PRQA S 1474,3358,4130 *//* misuse of enums */
		}
#else
		printk(KERN_ERR
		       "OSPCISetDev: MSI support not enabled in the kernel");
#endif
	}

	/* Initialise the PCI resource tracking array */
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		psPVRPCI->abPCIResourceInUse[i] = IMG_FALSE;
	}

	return (PVRSRV_PCI_DEV_HANDLE) psPVRPCI;
}

/*!
******************************************************************************

 @Function	OSPCIAcquireDev
 
 @Description 
 
 Acquire a PCI device for subsequent use.

 @input ui16VendorID :	Vendor PCI ID

 @input ui16VendorID :	Device PCI ID

 @input eFlags :	Flags

 @Return   PVESRV_ERROR

******************************************************************************/
PVRSRV_PCI_DEV_HANDLE OSPCIAcquireDev(IMG_UINT16 ui16VendorID,
				      IMG_UINT16 ui16DeviceID,
				      HOST_PCI_INIT_FLAGS eFlags)
{
	struct pci_dev *psPCIDev;

	psPCIDev = pci_get_device(ui16VendorID, ui16DeviceID, NULL);
	if (psPCIDev == NULL) {
		printk(KERN_ERR "OSPCIAcquireDev: Couldn't acquire device");
		return IMG_NULL;
	}

	return OSPCISetDev((IMG_VOID *) psPCIDev, eFlags);
}

/*!
******************************************************************************

 @Function	OSPCIIRQ
 
 @Description 
 
 Get the interrupt number for the device.

 @input hPVRPCI :	PCI device handle

 @input pui32IRQ :	Pointer to where the interrupt number should be returned

 @Return   PVESRV_ERROR

******************************************************************************/
PVRSRV_ERROR OSPCIIRQ(PVRSRV_PCI_DEV_HANDLE hPVRPCI, IMG_UINT32 * pui32IRQ)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *) hPVRPCI;

	*pui32IRQ = psPVRPCI->psPCIDev->irq;

	return PVRSRV_OK;
}

/* Functions supported by OSPCIAddrRangeFunc */
enum HOST_PCI_ADDR_RANGE_FUNC {
	HOST_PCI_ADDR_RANGE_FUNC_LEN,
	HOST_PCI_ADDR_RANGE_FUNC_START,
	HOST_PCI_ADDR_RANGE_FUNC_END,
	HOST_PCI_ADDR_RANGE_FUNC_REQUEST,
	HOST_PCI_ADDR_RANGE_FUNC_RELEASE
};

/*!
******************************************************************************

 @Function	OSPCIAddrRangeFunc
 
 @Description 
 
 Internal support function for various address range related functions

 @input eFunc :	Function to perform

 @input hPVRPCI :	PCI device handle

 @input ui32Index :	Address range index

 @Return   function dependent

******************************************************************************/
static IMG_UINT32 OSPCIAddrRangeFunc(enum HOST_PCI_ADDR_RANGE_FUNC eFunc,
				     PVRSRV_PCI_DEV_HANDLE hPVRPCI,
				     IMG_UINT32 ui32Index)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *) hPVRPCI;

	if (ui32Index >= DEVICE_COUNT_RESOURCE) {
		printk(KERN_ERR "OSPCIAddrRangeFunc: Index out of range");
		return 0;

	}

	switch (eFunc) {
	case HOST_PCI_ADDR_RANGE_FUNC_LEN:
		return pci_resource_len(psPVRPCI->psPCIDev, ui32Index);
	case HOST_PCI_ADDR_RANGE_FUNC_START:
		return pci_resource_start(psPVRPCI->psPCIDev, ui32Index);
	case HOST_PCI_ADDR_RANGE_FUNC_END:
		return pci_resource_end(psPVRPCI->psPCIDev, ui32Index);
	case HOST_PCI_ADDR_RANGE_FUNC_REQUEST:
		{
			int err;

			err =
			    pci_request_region(psPVRPCI->psPCIDev,
					       (IMG_INT) ui32Index,
					       PVRSRV_MODNAME);
			if (err != 0) {
				printk(KERN_ERR
				       "OSPCIAddrRangeFunc: pci_request_region_failed (%d)",
				       err);
				return 0;
			}
			psPVRPCI->abPCIResourceInUse[ui32Index] = IMG_TRUE;
			return 1;
		}
	case HOST_PCI_ADDR_RANGE_FUNC_RELEASE:
		if (psPVRPCI->abPCIResourceInUse[ui32Index]) {
			pci_release_region(psPVRPCI->psPCIDev,
					   (IMG_INT) ui32Index);
			psPVRPCI->abPCIResourceInUse[ui32Index] = IMG_FALSE;
		}
		return 1;
	default:
		printk(KERN_ERR "OSPCIAddrRangeFunc: Unknown function");
		break;
	}

	return 0;
}

/*!
******************************************************************************

 @Function	OSPCIAddrRangeLen
 
 @Description 
 
 Returns length of a given address range length

 @input hPVRPCI :	PCI device handle

 @input ui32Index :	Address range index

 @Return   Length of address range, or 0 if no such range

******************************************************************************/
IMG_UINT32 OSPCIAddrRangeLen(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
			     IMG_UINT32 ui32Index)
{
	return OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_LEN, hPVRPCI,
				  ui32Index);
}

/*!
******************************************************************************

 @Function	OSPCIAddrRangeStart
 
 @Description 
 
 Returns the start of a given address range

 @input hPVRPCI :	PCI device handle

 @input ui32Index :	Address range index

 @Return   Start of address range, or 0 if no such range

******************************************************************************/
IMG_UINT32 OSPCIAddrRangeStart(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
			       IMG_UINT32 ui32Index)
{
	return OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_START, hPVRPCI,
				  ui32Index);
}

/*!
******************************************************************************

 @Function	OSPCIAddrRangeEnd
 
 @Description 
 
 Returns the end of a given address range

 @input hPVRPCI :	PCI device handle"ayy

 @input ui32Index :	Address range index

 @Return   End of address range, or 0 if no such range

******************************************************************************/
IMG_UINT32 OSPCIAddrRangeEnd(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
			     IMG_UINT32 ui32Index)
{
	return OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_END, hPVRPCI,
				  ui32Index);
}

/*!
******************************************************************************

 @Function	OSPCIRequestAddrRange
 
 @Description 
 
 Request a given address range index for subsequent use

 @input hPVRPCI :	PCI device handle

 @input ui32Index :	Address range index

 @Return   PVESRV_ERROR

******************************************************************************/
PVRSRV_ERROR OSPCIRequestAddrRange(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
				   IMG_UINT32 ui32Index)
{
	return OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_REQUEST, hPVRPCI,
				  ui32Index) ==
	    0 ? PVRSRV_ERROR_PCI_CALL_FAILED : PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	OSPCIReleaseAddrRange
 
 @Description 
 
 Release a given address range that is no longer being used

 @input hPVRPCI :	PCI device handle

 @input ui32Index :	Address range index

 @Return   PVESRV_ERROR

******************************************************************************/
PVRSRV_ERROR OSPCIReleaseAddrRange(PVRSRV_PCI_DEV_HANDLE hPVRPCI,
				   IMG_UINT32 ui32Index)
{
	return OSPCIAddrRangeFunc(HOST_PCI_ADDR_RANGE_FUNC_RELEASE, hPVRPCI,
				  ui32Index) ==
	    0 ? PVRSRV_ERROR_PCI_CALL_FAILED : PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	OSPCIReleaseDev
 
 @Description 
 
 Release a PCI device that is no longer being used

 @input hPVRPCI :	PCI device handle

 @Return   PVESRV_ERROR

******************************************************************************/
PVRSRV_ERROR OSPCIReleaseDev(PVRSRV_PCI_DEV_HANDLE hPVRPCI)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *) hPVRPCI;
	int i;

	/* Release all PCI regions that are currently in use */
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		if (psPVRPCI->abPCIResourceInUse[i]) {
			pci_release_region(psPVRPCI->psPCIDev, i);
			psPVRPCI->abPCIResourceInUse[i] = IMG_FALSE;
		}
	}

#if defined(CONFIG_PCI_MSI)
	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_MSI) {	/* PRQA S 3358 */
		/* misuse of enums */
		pci_disable_msi(psPVRPCI->psPCIDev);
	}
#endif

	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_BUS_MASTER) {	/* PRQA S 3358 */
		/* misuse of enums */
		pci_clear_master(psPVRPCI->psPCIDev);
	}

	pci_disable_device(psPVRPCI->psPCIDev);

	kfree((IMG_VOID *) psPVRPCI);
	/*not nulling pointer, copy on stack */

	return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	OSPCISuspendDev
 
 @Description 
 
 Prepare PCI device to be turned off by power management

 @input hPVRPCI :	PCI device handle

 @Return   PVESRV_ERROR

******************************************************************************/
PVRSRV_ERROR OSPCISuspendDev(PVRSRV_PCI_DEV_HANDLE hPVRPCI)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *) hPVRPCI;
	int i;
	int err;

	/* Release all PCI regions that are currently in use */
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		if (psPVRPCI->abPCIResourceInUse[i]) {
			pci_release_region(psPVRPCI->psPCIDev, i);
		}
	}

	err = pci_save_state(psPVRPCI->psPCIDev);
	if (err != 0) {
		printk(KERN_ERR "OSPCISuspendDev: pci_save_state_failed (%d)",
		       err);
		return PVRSRV_ERROR_PCI_CALL_FAILED;
	}

	pci_disable_device(psPVRPCI->psPCIDev);

	err =
	    pci_set_power_state(psPVRPCI->psPCIDev,
				pci_choose_state(psPVRPCI->psPCIDev,
						 PMSG_SUSPEND));
	switch (err) {
	case 0:
		break;
	case -EIO:
		printk(KERN_ERR
		       "OSPCISuspendDev: device doesn't support PCI PM");
		break;
	case -EINVAL:
		printk(KERN_ERR
		       "OSPCISuspendDev: can't enter requested power state");
		break;
	default:
		printk(KERN_ERR
		       "OSPCISuspendDev: pci_set_power_state failed (%d)", err);
		break;
	}

	return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	OSPCIResumeDev
 
 @Description 
 
 Prepare a PCI device to be resumed by power management

 @input hPVRPCI :	PCI device handle

 @input pvPCICookie :	Pointer to OS specific PCI structure/cookie

 @input eFlags :	Flags

 @Return   PVESRV_ERROR

******************************************************************************/
PVRSRV_ERROR OSPCIResumeDev(PVRSRV_PCI_DEV_HANDLE hPVRPCI)
{
	PVR_PCI_DEV *psPVRPCI = (PVR_PCI_DEV *) hPVRPCI;
	int err;
	int i;

	err =
	    pci_set_power_state(psPVRPCI->psPCIDev,
				pci_choose_state(psPVRPCI->psPCIDev, PMSG_ON));
	switch (err) {
	case 0:
		break;
	case -EIO:
		printk(KERN_ERR
		       "OSPCIResumeDev: device doesn't support PCI PM");
		break;
	case -EINVAL:
		printk(KERN_ERR
		       "OSPCIResumeDev: can't enter requested power state");
		return PVRSRV_ERROR_UNKNOWN_POWER_STATE;
	default:
		printk(KERN_ERR
		       "OSPCIResumeDev: pci_set_power_state failed (%d)", err);
		return PVRSRV_ERROR_UNKNOWN_POWER_STATE;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38))
	pci_restore_state(psPVRPCI->psPCIDev);
#else
	err = pci_restore_state(psPVRPCI->psPCIDev);
	if (err != 0) {
		printk(KERN_ERR "OSPCIResumeDev: pci_restore_state failed (%d)",
		       err);
		return PVRSRV_ERROR_PCI_CALL_FAILED;
	}
#endif
	err = pci_enable_device(psPVRPCI->psPCIDev);
	if (err != 0) {
		printk(KERN_ERR "OSPCIResumeDev: Couldn't enable device (%d)",
		       err);
		return PVRSRV_ERROR_PCI_CALL_FAILED;
	}

	if (psPVRPCI->ePCIFlags & HOST_PCI_INIT_FLAG_BUS_MASTER)	/* PRQA S 3358 */
		/* misuse of enums */
		pci_set_master(psPVRPCI->psPCIDev);

	/* Restore the PCI resource tracking array */
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		if (psPVRPCI->abPCIResourceInUse[i]) {
			err =
			    pci_request_region(psPVRPCI->psPCIDev, i,
					       PVRSRV_MODNAME);
			if (err != 0) {
				printk(KERN_ERR
				       "OSPCIResumeDev: pci_request_region_failed (region %d, error %d)",
				       i, err);
			}
		}

	}

	return PVRSRV_OK;
}
