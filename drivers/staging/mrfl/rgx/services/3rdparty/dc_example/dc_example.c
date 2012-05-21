									    /*************************************************************************//*!
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

/* Linux headers */
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/spinlock_types.h>	/* Required for pgtable.h */
#include <linux/module.h>

#if defined(SUPPORT_DRI_DRM)
#include <drm/drmP.h>
#endif

#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/div64.h>
#include <asm/page.h>

/* for MODULE_LICENSE */
#include "pvrmodule.h"

/* Services headers */
#include "kerneldisplay.h"
#include "imgpixfmts_km.h"

#if defined (LMA)
#include "physheap.h"
#define DC_PHYS_HEAP_ID		1
#else
#define DC_PHYS_HEAP_ID		0
#endif

#if defined(SUPPORT_DRI_DRM)
#include "pvr_drm.h"
#endif

#if defined (LMA)
#define DRVNAME	"dc_example_LMA"
#else
#define DRVNAME	"dc_example_UMA"
#endif

#define ASSERT(n) \
do \
{ \
	if (!(n)) \
	{ \
		printk(KERN_ERR DRVNAME "Assertion faild @ %s:%d\n", __FUNCTION__, __LINE__); \
		BUG(); \
	} \
} while(0)

/*
	Enable to track contexts and buffers
*/
/* #define DCEX_DEBUG 1 */

/*
	Enable to get more debug. Only supported on UMA
*/
/* #define DCEX_VERBOSE 1*/

#if defined(DCEX_DEBUG)
#define DCEX_DEBUG_PRINT(fmt, ...) \
	printk(KERN_WARNING DRVNAME " " fmt, __VA_ARGS__)
#else
#define DCEX_DEBUG_PRINT(fmt, ...)
#endif

/*
	The number of inflight commands this display driver can handle
*/
#define MAX_COMMANDS_INFLIGHT 2

MODULE_SUPPORTED_DEVICE(DEVNAME);

static unsigned long width = DC_EXAMPLE_WIDTH;
static unsigned long height = DC_EXAMPLE_HEIGHT;
static unsigned long depth = DC_EXAMPLE_BIT_DEPTH;
static IMG_UINT32 ui32ByteStride;
static IMG_UINT32 ePixelFormat;

module_param(width, ulong, S_IRUGO);
module_param(height, ulong, S_IRUGO);
module_param(depth, ulong, S_IRUGO);

static IMG_BOOL CheckBufferDimensions(IMG_VOID)
{
	IMG_UINT32 ui32BytesPP;
	if (width == 0 || height == 0 || depth == 0) {
		printk(KERN_WARNING DRVNAME
		       ": Illegal module parameters (width %lu, height %lu, depth %lu)\n",
		       width, height, depth);
		return IMG_FALSE;
	}

	switch (depth) {
	case 32:
		ePixelFormat = IMG_PIXFMT_B8G8R8A8_UNORM;
		ui32BytesPP = 4;
		break;
	case 16:
		ePixelFormat = IMG_PIXFMT_B5G6R5_UNORM;
		ui32BytesPP = 2;
		break;
	default:
		printk(KERN_WARNING DRVNAME
		       ": Display depth %lu not supported\n", depth);
		ePixelFormat = IMG_PIXFMT_UNKNOWN;
		return IMG_FALSE;
	}

	ui32ByteStride = width * ui32BytesPP;

	printk(KERN_INFO DRVNAME " Width: %lu\n", (unsigned long)width);
	printk(KERN_INFO DRVNAME " Height: %lu\n", (unsigned long)height);
	printk(KERN_INFO DRVNAME " Depth: %lu bits\n", depth);
	printk(KERN_INFO DRVNAME " Stride: %lu bytes\n",
	       (unsigned long)ui32ByteStride);

	return IMG_TRUE;
}

/*
	As we only allow one display context to be created the config data
	fifo can be global data
*/
IMG_HANDLE g_hConfigData[MAX_COMMANDS_INFLIGHT];
IMG_UINT32 g_ui32Head = 0;
IMG_UINT32 g_ui32Tail = 0;

IMG_BOOL g_DisplayContextActive = IMG_FALSE;

static inline IMG_VOID DCExampleConfigPush(IMG_HANDLE hConfigData)
{
	g_hConfigData[g_ui32Head] = hConfigData;
	g_ui32Head++;
	if (g_ui32Head >= MAX_COMMANDS_INFLIGHT) {
		g_ui32Head = 0;
	}
}

static inline IMG_HANDLE DCExampleConfigPop(IMG_VOID)
{
	IMG_HANDLE hConfigData = g_hConfigData[g_ui32Tail];
	g_ui32Tail++;
	if (g_ui32Tail >= MAX_COMMANDS_INFLIGHT) {
		g_ui32Tail = 0;
	}
	return hConfigData;
}

static inline IMG_BOOL DCExampleConfigIsEmpty(IMG_VOID)
{
	return (g_ui32Tail == g_ui32Head);
}

typedef struct _DCEX_DEVICE_ {
	IMG_HANDLE hSrvHandle;
#if defined (LMA)
	PHYS_HEAP *psPhysHeap;
	IMG_CPU_PHYADDR sDispStartAddr;
	IMG_UINT64 uiDispMemSize;
	IMG_UINT32 ui32BufferSize;
	IMG_UINT32 ui32BufferCount;
	IMG_UINT32 ui32BufferUseMask;
#endif
} DCEX_DEVICE;

typedef struct _DCEX_BUFFER_ {
	IMG_UINT32 ui32RefCount;	/* Only required for system buffer */
	IMG_UINT32 ePixFormat;
	IMG_UINT32 ui32Width;
	IMG_UINT32 ui32Height;
	IMG_UINT32 ui32ByteStride;
	IMG_UINT32 ui32Size;
	IMG_UINT32 ui32PageCount;
	IMG_HANDLE hImport;
	IMG_DEV_PHYADDR *pasDevPAddr;
#if defined (LMA)
	IMG_UINT64 uiAllocHandle;
	DCEX_DEVICE *psDevice;
#else
	IMG_PVOID pvAllocHandle;
#endif
} DCEX_BUFFER;

DCEX_DEVICE *g_psDeviceData = IMG_NULL;
DCEX_BUFFER *g_psSystemBuffer = IMG_NULL;

#if defined (LMA)
/*
	Simple unit size allocator
*/
static
IMG_UINT64 _DCExampleAllocLMABuffer(DCEX_DEVICE * psDevice)
{
	IMG_UINT32 i;
	IMG_UINT64 pvRet = 0;

	for (i = 0; i < psDevice->ui32BufferCount; i++) {
		if ((psDevice->ui32BufferUseMask & (1UL << i)) == 0) {
			pvRet = psDevice->sDispStartAddr.uiAddr +
			    (i * psDevice->ui32BufferSize);
			psDevice->ui32BufferUseMask |= (1UL << i);
			break;
		}
	}

	return pvRet;
}

static
IMG_VOID _DCExampleFreeLMABuffer(DCEX_DEVICE * psDevice, IMG_UINT64 uiAddr)
{
	uint64_t ui64Offset;

	ASSERT(uiAddr >= psDevice->sDispStartAddr.uiAddr);

	ui64Offset = uiAddr - psDevice->sDispStartAddr.uiAddr;
	do_div(ui64Offset, psDevice->ui32BufferSize);
	ASSERT(ui64Offset <= psDevice->ui32BufferCount);
	psDevice->ui32BufferUseMask &= ~(1UL << ui64Offset);
}
#endif				/* LMA */

static
IMG_VOID DCExampleGetInfo(IMG_HANDLE hDeviceData,
			  DC_DISPLAY_INFO * psDisplayInfo)
{
	PVR_UNREFERENCED_PARAMETER(hDeviceData);

	/*
	   Copy our device name
	 */
	strncpy(psDisplayInfo->szDisplayName, DRVNAME " 1", DC_NAME_SIZE);

	/*
	   Report what our minimum and maximum display period is.
	 */
	psDisplayInfo->ui32MinDisplayPeriod = 0;
	psDisplayInfo->ui32MaxDisplayPeriod = 1;
}

static
PVRSRV_ERROR DCExamplePanelQueryCount(IMG_HANDLE hDeviceData,
				      IMG_UINT32 * pui32NumPanels)
{
	PVR_UNREFERENCED_PARAMETER(hDeviceData);
	/*
	   If you know the panel count at compile time just hardcode it, if it's
	   dynamic you should probe it here
	 */
	*pui32NumPanels = 1;

	return PVRSRV_OK;
}

static
PVRSRV_ERROR DCExamplePanelQuery(IMG_HANDLE hDeviceData,
				 IMG_UINT32 ui32PanelsArraySize,
				 IMG_UINT32 * pui32NumPanels,
				 PVRSRV_SURFACE_INFO * psSurfInfo)
{
	PVR_UNREFERENCED_PARAMETER(hDeviceData);

	/*
	   If we have hotplug displays then there is a chance a display could
	   have been removed so return the number of panels we have queryed
	 */
	*pui32NumPanels = 1;

	/*
	   Either hard code the values here or probe each panel here. If a new
	   panel has been hotpluged then ignore it as we've not been given
	   room to store it's data
	 */
	psSurfInfo[0].sFormat.ePixFormat = IMG_PIXFMT_B8G8R8A8_UNORM;

	psSurfInfo[0].sDims.ui32Width = width;
	psSurfInfo[0].sDims.ui32Height = height;

	return PVRSRV_OK;
}

static
PVRSRV_ERROR DCExampleFormatQuery(IMG_HANDLE hDeviceData,
				  IMG_UINT32 ui32NumFormats,
				  PVRSRV_SURFACE_FORMAT * pasFormat,
				  IMG_UINT32 * pui32Supported)
{
	IMG_UINT32 i;
	PVR_UNREFERENCED_PARAMETER(hDeviceData);

	for (i = 0; i < ui32NumFormats; i++) {
		pui32Supported[i] = 0;

		/*
		   If the display controller has multiple display pipes (DMA engines)
		   each one should be checked to see if it supports the specified
		   format.
		 */
		if (pasFormat[i].ePixFormat == IMG_PIXFMT_B8G8R8A8_UNORM) {
			pui32Supported[i]++;
		}
	}

	return PVRSRV_OK;
}

static
PVRSRV_ERROR DCExampleDimQuery(IMG_HANDLE hDeviceData,
			       IMG_UINT32 ui32NumDims,
			       PVRSRV_SURFACE_DIMS * psDim,
			       IMG_UINT32 * pui32Supported)
{
	IMG_UINT32 i;
	PVR_UNREFERENCED_PARAMETER(hDeviceData);

	for (i = 0; i < ui32NumDims; i++) {
		pui32Supported[i] = 0;

		/*
		   If the display controller has multiple display pipes (DMA engines)
		   each one should be checked to see if it supports the specified
		   dimentation.
		 */
		if ((psDim[i].ui32Width == width)
		    && (psDim[i].ui32Height == height)) {
			pui32Supported[i]++;
		}
	}

	return PVRSRV_OK;
}

static
PVRSRV_ERROR DCExampleBufferSystemAcquire(IMG_HANDLE hDeviceData,
					  IMG_DEVMEM_LOG2ALIGN_T *
					  puiLog2PageSize,
					  IMG_UINT32 * pui32PageCount,
					  IMG_UINT32 * pui32PhysHeapID,
					  IMG_UINT32 * pui32ByteStride,
					  IMG_HANDLE * phSystemBuffer)
{
	DCEX_BUFFER *psBuffer;
	PVR_UNREFERENCED_PARAMETER(hDeviceData);

	/*
	   This function is optionial. It provides a method for services
	   to acquire a display buffer which it didn't setup but was created
	   by the OS (e.g. Linux frame buffer).
	   If the OS should trigger a mode change then it's not allowed to free
	   the previous buffer until services has released it via BufferSystemRelease
	 */

	/*
	   Take a reference to the system buffer 1st to make sure it isn't freed
	 */
	g_psSystemBuffer->ui32RefCount++;
	psBuffer = g_psSystemBuffer;

	*puiLog2PageSize = PAGE_SHIFT;
	*pui32PageCount = psBuffer->ui32Size >> PAGE_SHIFT;
	*pui32PhysHeapID = DC_PHYS_HEAP_ID;
	*pui32ByteStride = psBuffer->ui32ByteStride;
	*phSystemBuffer = psBuffer;

	return PVRSRV_OK;
}

static
IMG_VOID DCExampleBufferSystemRelease(IMG_HANDLE hSystemBuffer)
{
	DCEX_BUFFER *psBuffer = hSystemBuffer;

	psBuffer->ui32RefCount--;

	/*
	   If the system buffer has changed and we've just dropped the last
	   refcount then free the buffer
	 */
	if ((g_psSystemBuffer != psBuffer) && (psBuffer->ui32RefCount == 0)) {
		/* Free the buffer and it's memory (if the memory was allocated) */
	}
}

static
PVRSRV_ERROR DCExampleContextCreate(IMG_HANDLE hDeviceData,
				    IMG_HANDLE * hDisplayContext)
{
	/*
	   The creation of a display context is a software concept and
	   it's an "agreament" between the services client and the DC driver
	   as to what this means (if anything)
	 */
	if (!g_DisplayContextActive) {
		*hDisplayContext = hDeviceData;

		g_DisplayContextActive = IMG_TRUE;
		DCEX_DEBUG_PRINT("Create context (%p)\n", *hDisplayContext);
		return PVRSRV_OK;
	}

	return PVRSRV_ERROR_RESOURCE_UNAVAILIBLE;
}

static
PVRSRV_ERROR DCExampleContextConfigureCheck(IMG_HANDLE hDisplayContext,
					    IMG_UINT32 ui32PipeCount,
					    PVRSRV_SURFACE_CONFIG_INFO *
					    pasSurfAttrib,
					    IMG_HANDLE * ahBuffers)
{
	IMG_UINT32 i;
	IMG_BOOL bFail = IMG_FALSE;
	PVR_UNREFERENCED_PARAMETER(hDisplayContext);

	/*
	   This optional function allows the display driver to check if the display
	   configuration passed in is valid.
	   It's possible that due to HW contraints that although the client has
	   honoured the DimQuery and FormatQuery results the configuration it
	   has requested is still not possible (e.g. there isn't enough space in
	   the display controllers's MMU, or due restrictions on display pipes.
	 */

	for (i = 0; i < ui32PipeCount; i++) {
		DCEX_BUFFER *psBuffer = ahBuffers[i];
		PVRSRV_SURFACE_INFO *psSurfInfo = &pasSurfAttrib[i].sSurface;

		if (psSurfInfo->sFormat.ePixFormat != IMG_PIXFMT_B8G8R8A8_UNORM) {
			bFail = IMG_TRUE;
		}
		if (psSurfInfo->sDims.ui32Width != width) {
			bFail = IMG_TRUE;
		}
		if (psSurfInfo->sDims.ui32Height != height) {
			bFail = IMG_TRUE;
		}
		if (pasSurfAttrib[i].ui32XOffset != 0) {
			bFail = IMG_TRUE;
		}
		if (pasSurfAttrib[i].ui32YOffset != 0) {
			bFail = IMG_TRUE;
		}

		if (psBuffer->ui32Width != width) {
			bFail = IMG_TRUE;
		}
		if (psBuffer->ui32Height != height) {
			bFail = IMG_TRUE;
		}

		if (bFail) {
			break;
		}
	}

	if (bFail) {
		return PVRSRV_ERROR_INVALID_CONFIG;
	}
	return PVRSRV_OK;
}

static
IMG_VOID DCExampleContextConfigure(IMG_HANDLE hDisplayContext,
				   IMG_UINT32 ui32PipeCount,
				   PVRSRV_SURFACE_CONFIG_INFO * pasSurfAttrib,
				   IMG_HANDLE * ahBuffers,
				   IMG_UINT32 ui32DisplayPeriod,
				   IMG_HANDLE hConfigData)
{
	IMG_UINT32 i;
	PVR_UNREFERENCED_PARAMETER(hDisplayContext);

	/*
	   As we have no HW and thus no VSync IRQ we just activate the
	   new config here
	 */
	for (i = 0; i < ui32PipeCount; i++) {
#if defined(DCEX_DEBUG)
		DCEX_BUFFER *psBuffer = ahBuffers[i];
		/*
		   There is no checking to be done here as we can't fail,
		   any checking should have been DCExampleContextConfigureCheck.
		 */

		/*
		   If the display controller supports scaling it should set it up
		   here.
		 */

		/*
		   Setup the DMA from the display buffer
		 */

		/*
		   Save the config data as we need to pass it back once this
		   confiuration gets retired
		 */
		DCEX_DEBUG_PRINT("Display buffer (%p)\n", psBuffer);
#endif				/* DCEX_DEBUG */
	}

	/*
	   As we have no HW and thus no VSync IRQ we just retire the
	   previous config as soon as we get a new one
	 */
	if (!DCExampleConfigIsEmpty()) {
		/* Retire the current config */
		DCDisplayConfigurationRetired(DCExampleConfigPop());
	}

	if (ui32PipeCount != 0) {
		/* Save our new config data */
		DCExampleConfigPush(hConfigData);
	} else {
		/*
		   When the client requests the display context to be destroyed
		   services will issue a "NULL" flip to us so we can retire
		   the current configuration.

		   In this simple example we just need to pop the current (and last)
		   configuration off the our stack and retire it which we're already
		   done above as it's our default behaviour to retire the previous config
		   immediately when we get a new one.
		   In real devices we could have a number of configurations in flight
		   and we would have to block here until they have all been retired.

		   At this point there is nothing that the display is being asked to
		   display by services and it's the DC driver implementation decision
		   as to what it should then do. Typically, for systems that have a
		   system surface the DC would switch back to displaying that.
		 */
		DCEX_DEBUG_PRINT("Display flushed (%p)\n", hDisplayContext);
	}
}

static
IMG_VOID DCExampleContextDestroy(IMG_HANDLE hDisplayContext)
{
	PVR_UNREFERENCED_PARAMETER(hDisplayContext);

	ASSERT(DCExampleConfigIsEmpty());

	/*
	   Counter part to ContextCreate. Any buffers created/imported
	   on this display context will have been freed before this call
	   so all the display driver needs to do is release any resources
	   allocated at ContextCreate time.
	 */
	g_DisplayContextActive = IMG_FALSE;
	DCEX_DEBUG_PRINT("Destroy display context (%p)\n", hDisplayContext);
}

#define BYTE_TO_PAGES(range) (((range) + (PAGE_SIZE - 1)) >> PAGE_SHIFT)

static
PVRSRV_ERROR DCExampleBufferAlloc(IMG_HANDLE hDisplayContext,
				  DC_BUFFER_CREATE_INFO * psCreateInfo,
				  IMG_DEVMEM_LOG2ALIGN_T * puiLog2PageSize,
				  IMG_UINT32 * pui32PageCount,
				  IMG_UINT32 * pui32PhysHeapID,
				  IMG_UINT32 * pui32ByteStride,
				  IMG_HANDLE * phBuffer)
{
	DCEX_BUFFER *psBuffer;
	PVRSRV_SURFACE_INFO *psSurfInfo = &psCreateInfo->sSurface;
	PVRSRV_ERROR eError;
#if defined (LMA)
	DCEX_DEVICE *psDevice = hDisplayContext;
#else
	PVR_UNREFERENCED_PARAMETER(hDisplayContext);
#endif
	/*
	   Allocate the buffer control structure
	 */
	psBuffer = kmalloc(sizeof(DCEX_BUFFER), GFP_KERNEL);
	if (psBuffer == NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_bufferalloc;
	}
	memset(psBuffer, 0, sizeof(DCEX_BUFFER));

	/*
	   As we're been asked to allocate this buffer we decide what it's
	   stride should be.
	 */
	psBuffer->ui32RefCount = 1;
	psBuffer->ePixFormat = psSurfInfo->sFormat.ePixFormat;
	psBuffer->ui32ByteStride =
	    psSurfInfo->sDims.ui32Width * psCreateInfo->ui32BPP;
	psBuffer->ui32Width = psSurfInfo->sDims.ui32Width;
	psBuffer->ui32Height = psSurfInfo->sDims.ui32Height;
	psBuffer->ui32Size = psBuffer->ui32Height * psBuffer->ui32ByteStride;

	/*
	   Allocate display adressable memory. We only need physcial addresses
	   at this stage.

	   Note: This could be defered till the 1st map or acquire call.
	 */
#if defined (LMA)
	psBuffer->psDevice = psDevice;
	psBuffer->uiAllocHandle = _DCExampleAllocLMABuffer(psDevice);

	if (psBuffer->uiAllocHandle == 0) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_buffermemalloc;
	}
#else
	psBuffer->pvAllocHandle = __vmalloc(psBuffer->ui32Size,
					    GFP_KERNEL | __GFP_HIGHMEM,
					    pgprot_noncached(PAGE_KERNEL));

	if (psBuffer->pvAllocHandle == NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_buffermemalloc;
	}
#endif
	*pui32ByteStride = psBuffer->ui32ByteStride;
	*puiLog2PageSize = PAGE_SHIFT;
	*pui32PageCount = BYTE_TO_PAGES(psBuffer->ui32Size);
	*pui32PhysHeapID = DC_PHYS_HEAP_ID;
	*phBuffer = psBuffer;

	DCEX_DEBUG_PRINT("Allocate buffer (%p)\n", psBuffer);
	return PVRSRV_OK;
 fail_buffermemalloc:
#if defined (LMA)
	_DCExampleFreeLMABuffer(psDevice, psBuffer->uiAllocHandle);
#else
	vfree(psBuffer->pvAllocHandle);
#endif
	kfree(psBuffer);
 fail_bufferalloc:
	return eError;
}

#if !defined (LMA)
static
PVRSRV_ERROR DCExampleBufferImport(IMG_HANDLE hDisplayContext,
				   IMG_UINT32 ui32NumPlanes,
				   IMG_HANDLE ** paphImport,
				   DC_BUFFER_IMPORT_INFO * psSurfAttrib,
				   IMG_HANDLE * phBuffer)
{
	/*
	   This it optional and should only be provided if the display controller
	   can access "general" memory (e.g. the memory doesn't have to contiguesus)
	 */
	DCEX_BUFFER *psBuffer;

	/*
	   Check to see if our display hardware supports this buffer
	 */
	if ((psSurfAttrib->ePixFormat != IMG_PIXFMT_B8G8R8A8_UNORM) ||
	    (psSurfAttrib->ui32Width[0] != width) ||
	    (psSurfAttrib->ui32Height[0] != height) ||
	    (psSurfAttrib->ui32Stride[0] != ui32ByteStride)) {
		return PVRSRV_ERROR_UNSUPPORTED_PIXEL_FORMAT;
	}

	psBuffer = kmalloc(sizeof(DCEX_BUFFER), GFP_KERNEL);
	if (psBuffer == NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	psBuffer->ui32Width = psSurfAttrib->ui32Width[0];
	psBuffer->ui32RefCount = 1;
	psBuffer->ePixFormat = psSurfAttrib->ePixFormat;
	psBuffer->ui32ByteStride = psSurfAttrib->ui32Stride[0];
	psBuffer->ui32Width = psSurfAttrib->ui32Width[0];
	psBuffer->ui32Height = psSurfAttrib->ui32Height[0];

	/*
	   If the display controller supports mapping "general" memory, but has
	   limitations (e.g. if it doesn't have full range addressing) these
	   should be checked here by calling DCImportBufferAcquire. In this case
	   it lock down the physcial address of the buffer at this stange rather
	   then being able to defer it to map time.
	 */
	psBuffer->hImport = paphImport[0];

	*phBuffer = psBuffer;
	DCEX_DEBUG_PRINT("Import buffer (%p)\n", psBuffer);
	return PVRSRV_OK;
}
#endif

#define	VMALLOC_TO_PAGE_PHYS(vAddr) page_to_phys(vmalloc_to_page(vAddr))

static
PVRSRV_ERROR DCExampleBufferAcquire(IMG_HANDLE hBuffer,
				    IMG_DEV_PHYADDR * pasDevPAddr,
				    IMG_PVOID * ppvLinAddr)
{
	DCEX_BUFFER *psBuffer = hBuffer;
	unsigned long ulPages = BYTE_TO_PAGES(psBuffer->ui32Size);
	IMG_UINT32 i;
#if defined (LMA)
	PHYS_HEAP *psPhysHeap = psBuffer->psDevice->psPhysHeap;
	IMG_CPU_PHYADDR sCpuPAddr;
#else
	IMG_PVOID pvLinAddr;
#endif
	/*
	   If we didn't allocate the display memory at buffer alloc time
	   we would have to do it here.
	 */

	/*
	   Fill in the array of addresses we where passed
	 */
#if defined (LMA)
	sCpuPAddr.uiAddr = psBuffer->uiAllocHandle;
	for (i = 0; i < ulPages; i++) {
		PhysHeapCpuPAddrToDevPAddr(psPhysHeap, &pasDevPAddr[i],
					   &sCpuPAddr);
		sCpuPAddr.uiAddr += PAGE_SIZE;
	}
	*ppvLinAddr = IMG_NULL;
#else
	pvLinAddr = psBuffer->pvAllocHandle;
	for (i = 0; i < ulPages; i++) {
		pasDevPAddr[i].uiAddr = VMALLOC_TO_PAGE_PHYS(pvLinAddr);

		pvLinAddr += PAGE_SIZE;
	}
	*ppvLinAddr = psBuffer->pvAllocHandle;
#endif

	DCEX_DEBUG_PRINT("Acquire buffer (%p) memory\n", psBuffer);
	return PVRSRV_OK;
}

static
IMG_VOID DCExampleBufferRelease(IMG_HANDLE hBuffer)
{
#if defined(DCEX_DEBUG)
	DCEX_BUFFER *psBuffer = hBuffer;
#endif
	/*
	   We could release the display memory here (assuming it wasn't
	   still mapped into the display controller).

	   As the buffer hasn't been freed the contents must be preserved, i.e.
	   in the next call to Acquire different physcial pages can be returned,
	   but they must have the same contents as the old pages had at Release
	   time.
	 */
	DCEX_DEBUG_PRINT("Release buffer (%p) memory\n", psBuffer);
}

static
IMG_VOID DCExampleBufferFree(IMG_HANDLE hBuffer)
{
	DCEX_BUFFER *psBuffer = hBuffer;

	DCEX_DEBUG_PRINT("Free buffer (%p)\n", psBuffer);
#if defined (LMA)
	_DCExampleFreeLMABuffer(psBuffer->psDevice, psBuffer->uiAllocHandle);
#else
	vfree(psBuffer->pvAllocHandle);
#endif
	kfree(psBuffer);
}

static
PVRSRV_ERROR DCExampleBufferMap(IMG_HANDLE hBuffer)
{
	DCEX_BUFFER *psBuffer = hBuffer;
	IMG_UINT32 ui32PageCount;
	PVRSRV_ERROR eError;
#if defined (DCEX_VERBOSE)
	IMG_UINT32 i;
#endif
	/*
	   If the display controller needs memory to be mapped into it
	   (e.g. it has an MMU) and didn't do it in the alloc and import then it
	   should provide this function.
	 */

	if (psBuffer->hImport) {
		IMG_DEV_PHYADDR *pasDevPAddr;
		/*
		   In the case of an import buffer we didn't allocate the buffer and
		   so need to ask for it's pages
		 */
		eError = DCImportBufferAcquire(psBuffer->hImport,
					       PAGE_SHIFT,
					       &ui32PageCount, &pasDevPAddr);
		if (eError != PVRSRV_OK) {
			goto fail_import;
		}
#if defined (DCEX_VERBOSE)
		for (i = 0; i < ui32PageCount; i++) {
			printk(KERN_WARNING DRVNAME
			       ": DCExampleBufferMap: DCExample map address 0x%016llx\n",
			       pasDevPAddr[i].uiAddr);
		}
#endif
		psBuffer->pasDevPAddr = pasDevPAddr;
		psBuffer->ui32PageCount = ui32PageCount;
	}
#if defined (DCEX_VERBOSE)
	else {
		unsigned long ulPages = BYTE_TO_PAGES(psBuffer->ui32Size);
		IMG_CPU_VIRTADDR pvLinAddr = psBuffer->pvAllocHandle;
		IMG_DEV_PHYADDR sDevPAddr;

		for (i = 0; i < ulPages; i++) {
			sDevPAddr.uiAddr = VMALLOC_TO_PAGE_PHYS(pvLinAddr);
			pvLinAddr += PAGE_SIZE;
			printk(KERN_WARNING DRVNAME
			       ": DCExampleBufferMap: DCExample map address 0x%016llx\n",
			       sDevPAddr.uiAddr);
		}
	}
#endif
	DCEX_DEBUG_PRINT("Map buffer (%p) into display\n", psBuffer);
	return PVRSRV_OK;

 fail_import:
	return eError;
}

static
IMG_VOID DCExampleBufferUnmap(IMG_HANDLE hBuffer)
{
	DCEX_BUFFER *psBuffer = hBuffer;
#if defined (DCEX_VERBOSE)
	IMG_UINT32 i;
#endif
	/*
	   If the display controller provided buffer map then it must provide
	   this function
	 */

	/*
	   Unmap the memory from the display controller's MMU
	 */
	if (psBuffer->hImport) {
#if defined (DCEX_VERBOSE)
		for (i = 0; i < psBuffer->ui32PageCount; i++) {
			printk(KERN_WARNING DRVNAME
			       ": DCExampleBufferUnmap: DCExample unmap address 0x%016llx\n",
			       psBuffer->pasDevPAddr[i].uiAddr);
		}
#endif
		/*
		   As this was an imported buffer we need to release it
		 */
		DCImportBufferRelease(psBuffer->hImport, psBuffer->pasDevPAddr);
	}
#if defined (DCEX_VERBOSE)
	else {
		unsigned long ulPages = BYTE_TO_PAGES(psBuffer->ui32Size);
		IMG_CPU_VIRTADDR pvLinAddr = psBuffer->pvAllocHandle;
		IMG_DEV_PHYADDR sDevPAddr;

		for (i = 0; i < ulPages; i++) {
			sDevPAddr.uiAddr = VMALLOC_TO_PAGE_PHYS(pvLinAddr);

			pvLinAddr += PAGE_SIZE;
			printk(KERN_WARNING DRVNAME
			       ": DCExampleBufferUnmap: DCExample unmap address 0x%016llx\n",
			       sDevPAddr.uiAddr);
		}
	}
#endif
	DCEX_DEBUG_PRINT("Unmap buffer (%p) from display\n", psBuffer);
}

/*
	In this example driver we provide the full range of functions
*/

static DC_DEVICE_FUNCTIONS sDCFunctions = {
	.pfnGetInfo = DCExampleGetInfo,
	.pfnPanelQueryCount = DCExamplePanelQueryCount,
	.pfnPanelQuery = DCExamplePanelQuery,
	.pfnFormatQuery = DCExampleFormatQuery,
	.pfnDimQuery = DCExampleDimQuery,
	.pfnContextCreate = DCExampleContextCreate,
	.pfnContextDestroy = DCExampleContextDestroy,
	.pfnContextConfigure = DCExampleContextConfigure,
	.pfnContextConfigureCheck = DCExampleContextConfigureCheck,
	.pfnBufferAlloc = DCExampleBufferAlloc,
	.pfnBufferAcquire = DCExampleBufferAcquire,
	.pfnBufferRelease = DCExampleBufferRelease,
	.pfnBufferFree = DCExampleBufferFree,
#if !defined (LMA)
	.pfnBufferImport = DCExampleBufferImport,
#endif
	.pfnBufferMap = DCExampleBufferMap,
	.pfnBufferUnmap = DCExampleBufferUnmap,
	.pfnBufferSystemAcquire = DCExampleBufferSystemAcquire,
	.pfnBufferSystemRelease = DCExampleBufferSystemRelease,
};

/*
	If a display controller only supported the basic's it would only need:

static DC_DEVICE_FUNCTIONS sDCFunctions = {

Must always be provided
	.pfnPanelQueryCount			= DCExamplePanelQueryCount,
	.pfnPanelQuery				= DCExamplePanelQuery,
	.pfnFormatQuery				= DCExampleFormatQuery,
	.pfnDimQuery				= DCExampleDimQuery,
	.pfnBufferQuery				= DCExampleBufferQuery,

Only provide these five functions if your controller can be reprogrammed.
Reprogramming means that it can be told to scan out a different physical
address (or has an MMU which translates to the same thing). Most display
controllers will be of this type and require the functionality.
	.pfnContextCreate			= DCExampleContextCreate,
	.pfnContextDestroy			= DCExampleContextDestroy,
	.pfnContextConfigure		= DCExampleContextConfigure,
	.pfnBufferAlloc				= DCExampleBufferAlloc,
	.pfnBufferFree				= DCExampleBufferFree,

Provide these functions if your controller has an MMU and does not (or
cannot) map/unmap buffers at alloc/free time
	.pfnBufferMap				= DCExampleBufferMap,
	.pfnBufferUnmap				= DCExampleBufferUnmap,

Provide this function if your controller can scan out arbitrary memory,
allocated for another purpose by services. Probably implies MMU
capability
	.pfnBufferImport			= DCExampleBufferImport,

Only provide these two callbacks if you have a system surface
	.pfnBufferSystemAcquire		= DCExampleBufferSystemAcquire,
	.pfnBufferSystemRelease		= DCExampleBufferSystemRelease,

};

*/

/*
	functions exported by kernel services for use by 3rd party kernel display
	class device driver
*/
static
PVRSRV_ERROR DCExampleInit(IMG_VOID)
{
	DCEX_BUFFER *psBuffer;
	DCEX_DEVICE *psDeviceData = IMG_NULL;
	PVRSRV_ERROR eError;
#if defined (LMA)
	int64_t ui64BufferCount;
#endif

	/* Check the module params and setup global buffer size state */
	if (!CheckBufferDimensions()) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/*
	   If the display controller hasn't already been initilised elsewhere in
	   the system then it should initilised here.

	   Create the private data structure (psDeviceData) and store all the
	   device specific data we will need later (e.g. pointer to mapped registered)
	   This device specific private data will be passed into our callbacks so
	   we don't need global data and can have more then one display controller
	   driven by the same driver (we would just create an "instance" of a device
	   by registering the same callbacks with different private data)
	 */
	psDeviceData = kmalloc(sizeof(DCEX_DEVICE), GFP_KERNEL);
	if (psDeviceData == NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_devicealloc;
	}
#if defined (LMA)
	/*
	   If the display is using card memory then we need to know
	   where that memory is so we have to acquire the heap we want
	   to use (a carveout of the card memory) so we can get it's address
	 */
	eError = PhysHeapAcquire(DC_PHYS_HEAP_ID, &psDeviceData->psPhysHeap);
	if (eError != PVRSRV_OK) {
		goto fail_heapacquire;
	}

	/* Sanity check we're operating on a LMA heap */
	ASSERT(PhysHeapGetType(psDeviceData->psPhysHeap) == PHYS_HEAP_TYPE_LMA);

	eError = PhysHeapGetAddress(psDeviceData->psPhysHeap,
				    &psDeviceData->sDispStartAddr);
	ASSERT(eError == PVRSRV_OK);

	eError = PhysHeapGetSize(psDeviceData->psPhysHeap,
				 &psDeviceData->uiDispMemSize);
	ASSERT(eError == PVRSRV_OK);
#endif
	/*
	   If the display driver has a system surface create the buffer structure
	   that describes it here.

	   Note:
	   All this data and the buffer should be querryed from the OS, but in this
	   example we don't have an OS driver to hook into and so we create the
	   data.
	 */
	psBuffer = kmalloc(sizeof(DCEX_BUFFER), GFP_KERNEL);
	if (psBuffer == NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_bufferalloc;
	}
	memset(psBuffer, 0, sizeof(DCEX_BUFFER));

	psBuffer->ui32RefCount = 1;
	psBuffer->ePixFormat = IMG_PIXFMT_B8G8R8A8_UNORM;
	psBuffer->ui32Width = width;
	psBuffer->ui32Height = height;
	psBuffer->ui32ByteStride = ui32ByteStride;
	psBuffer->ui32Size = psBuffer->ui32Height * psBuffer->ui32ByteStride;
	psBuffer->ui32Size = (psBuffer->ui32Size + PAGE_SIZE - 1) & (PAGE_MASK);

#if defined (LMA)
	/*
	   Simple allocator, assume all buffers are going to be the same size.
	 */
	ui64BufferCount = psDeviceData->uiDispMemSize;
	do_div(ui64BufferCount, psBuffer->ui32Size);

	psDeviceData->ui32BufferCount = (IMG_UINT32) ui64BufferCount;
	ASSERT((IMG_UINT32) ui64BufferCount == psDeviceData->ui32BufferCount);

	if (psDeviceData->ui32BufferCount > 32)
		psDeviceData->ui32BufferCount = 32;

	psDeviceData->ui32BufferSize = psBuffer->ui32Size;
	psDeviceData->ui32BufferUseMask = 0;

	psBuffer->psDevice = psDeviceData;
	psBuffer->uiAllocHandle = _DCExampleAllocLMABuffer(psDeviceData);
	if (psBuffer->uiAllocHandle == 0) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_buffermemalloc;
	}
#else
	psBuffer->pvAllocHandle =
	    __vmalloc(psBuffer->ui32Size, GFP_KERNEL | __GFP_HIGHMEM,
		      pgprot_noncached(PAGE_KERNEL));
	if (psBuffer->pvAllocHandle == NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_buffermemalloc;
	}
#endif
	DCEX_DEBUG_PRINT("Allocate system buffer = %p\n", psBuffer);

	/*
	   Register our DC driver with services
	 */
	eError = DCRegisterDevice(&sDCFunctions,
				  MAX_COMMANDS_INFLIGHT,
				  psDeviceData, &psDeviceData->hSrvHandle);
	if (eError != PVRSRV_OK) {
		goto fail_register;
	}

	/* Save the device data somewhere we can retreaive it */
	g_psDeviceData = psDeviceData;

	/* Store the system buffer on the global hook */
	g_psSystemBuffer = psBuffer;

	return PVRSRV_OK;

 fail_register:
#if defined (LMA)
	_DCExampleFreeLMABuffer(psDeviceData, psBuffer->uiAllocHandle);
#else
	vfree(psBuffer->pvAllocHandle);
#endif
 fail_buffermemalloc:
	kfree(psBuffer);
 fail_bufferalloc:
#if defined (LMA)
	PhysHeapRelease(psDeviceData->psPhysHeap);
 fail_heapacquire:
#endif
	kfree(psDeviceData);
 fail_devicealloc:
	return eError;
}

static
IMG_VOID DCExampleDeinit(IMG_VOID)
{
	DCEX_DEVICE *psDeviceData = g_psDeviceData;
	DCEX_BUFFER *psBuffer = g_psSystemBuffer;

	DCUnregisterDevice(psDeviceData->hSrvHandle);
	DCEX_DEBUG_PRINT("Free system buffer = %p\n", psBuffer);
#if defined (LMA)
	_DCExampleFreeLMABuffer(psDeviceData, psBuffer->uiAllocHandle);
	PhysHeapRelease(psDeviceData->psPhysHeap);
#else
	vfree(psBuffer->pvAllocHandle);
#endif
	kfree(psBuffer);
	kfree(psDeviceData);
}

#if defined(SUPPORT_DRI_DRM)
int PVR_DRM_MAKENAME(DISPLAY_CONTROLLER,
		     _Init) (struct drm_device unref__ * dev)
#else
static int __init dc_example_init(void)
#endif
{
	if (DCExampleInit() != PVRSRV_OK) {
		return -ENODEV;
	}

	return 0;
}

/*****************************************************************************
 Function Name:	DC_NOHW_Cleanup
 Description  :	Remove the driver from the kernel.

				__exit places the function in a special memory section that
				the kernel frees once the function has been run.  Refer also
				to module_exit() macro call below.

*****************************************************************************/
#if defined(SUPPORT_DRI_DRM)
void PVR_DRM_MAKENAME(DISPLAY_CONTROLLER,
		      _Cleanup) (struct drm_device unref__ * dev)
#else
static void __exit dc_example_deinit(void)
#endif
{
	DCExampleDeinit();
}

#if !defined(SUPPORT_DRI_DRM)
module_init(dc_example_init);
module_exit(dc_example_deinit);
#endif
