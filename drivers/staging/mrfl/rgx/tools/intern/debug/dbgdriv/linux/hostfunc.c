									    /*************************************************************************//*!
									       @File
									       @Title          Debug driver file
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <asm/page.h>
#include <linux/vmalloc.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15))
#include <linux/mutex.h>
#else
#include <asm/semaphore.h>
#endif
#include <linux/hardirq.h>

#if defined(SUPPORT_DBGDRV_EVENT_OBJECTS)
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#endif				/* defined(SUPPORT_DBGDRV_EVENT_OBJECTS) */

#include "img_types.h"
#include "pvr_debug.h"

#include "dbgdrvif.h"
#include "hostfunc.h"
#include "dbgdriv.h"

#if defined(DEBUG) && !defined(SUPPORT_DRI_DRM)
IMG_UINT32 gPVRDebugLevel = (DBGPRIV_FATAL | DBGPRIV_ERROR | DBGPRIV_WARNING);

#define PVR_STRING_TERMINATOR		'\0'
#define PVR_IS_FILE_SEPARATOR(character) ( ((character) == '\\') || ((character) == '/') )

/******************************************************************************/

/*----------------------------------------------------------------------------
<function>
	FUNCTION   : PVRSRVDebugPrintf
	PURPOSE    : To output a debug message to the user
	PARAMETERS : In : uDebugLevel - The current debug level
	             In : pszFile - The source file generating the message
	             In : uLine - The line of the source file
	             In : pszFormat - The message format string
	             In : ... - Zero or more arguments for use by the format string
	RETURNS    : None
</function>
------------------------------------------------------------------------------*/
void PVRSRVDebugPrintf(IMG_UINT32 ui32DebugLevel,
		       const IMG_CHAR * pszFileName,
		       IMG_UINT32 ui32Line, const IMG_CHAR * pszFormat, ...
    )
{
	IMG_BOOL bTrace;
#if !defined(__sh__)
	IMG_CHAR *pszLeafName;

	pszLeafName = (char *)strrchr(pszFileName, '\\');

	if (pszLeafName) {
		pszFileName = pszLeafName;
	}
#endif				/* __sh__ */

	bTrace =
	    (IMG_BOOL) (ui32DebugLevel & DBGPRIV_CALLTRACE) ? IMG_TRUE :
	    IMG_FALSE;

	if (gPVRDebugLevel & ui32DebugLevel) {
		va_list vaArgs;
		static char szBuffer[256];

		va_start(vaArgs, pszFormat);

		/* Add in the level of warning */
		if (bTrace == IMG_FALSE) {
			switch (ui32DebugLevel) {
			case DBGPRIV_FATAL:
				{
					strcpy(szBuffer, "PVR_K:(Fatal): ");
					break;
				}
			case DBGPRIV_ERROR:
				{
					strcpy(szBuffer, "PVR_K:(Error): ");
					break;
				}
			case DBGPRIV_WARNING:
				{
					strcpy(szBuffer, "PVR_K:(Warning): ");
					break;
				}
			case DBGPRIV_MESSAGE:
				{
					strcpy(szBuffer, "PVR_K:(Message): ");
					break;
				}
			case DBGPRIV_VERBOSE:
				{
					strcpy(szBuffer, "PVR_K:(Verbose): ");
					break;
				}
			default:
				{
					strcpy(szBuffer,
					       "PVR_K:(Unknown message level)");
					break;
				}
			}
		} else {
			strcpy(szBuffer, "PVR_K: ");
		}

		vsprintf(&szBuffer[strlen(szBuffer)], pszFormat, vaArgs);

		/*
		 * Metrics and Traces don't need a location
		 */
		if (bTrace == IMG_FALSE) {
			sprintf(&szBuffer[strlen(szBuffer)], " [%d, %s]",
				(int)ui32Line, pszFileName);
		}

		printk(KERN_INFO "%s\r\n", szBuffer);

		va_end(vaArgs);
	}
}
#endif				/* defined(DEBUG) && !defined(SUPPORT_DRI_DRM) */

/*!
******************************************************************************

 @Function	HostMemSet

 @Description Function that does the same as the C memset() functions

 @Modified *pvDest :	pointer to start of buffer to be set

 @Input    ui8Value:	value to set each byte to

 @Input    ui32Size :	number of bytes to set

 @Return   IMG_VOID

******************************************************************************/
IMG_VOID HostMemSet(IMG_VOID * pvDest, IMG_UINT8 ui8Value, IMG_UINT32 ui32Size)
{
	memset(pvDest, (int)ui8Value, (size_t) ui32Size);
}

/*!
******************************************************************************

 @Function		HostMemCopy

 @Description	Copies memory around

 @Input    pvDst - pointer to dst
 @Output   pvSrc - pointer to src
 @Input    ui32Size - bytes to copy

 @Return  none

******************************************************************************/
IMG_VOID HostMemCopy(IMG_VOID * pvDst, IMG_VOID * pvSrc, IMG_UINT32 ui32Size)
{
#if defined(USE_UNOPTIMISED_MEMCPY)
	unsigned char *src, *dst;
	int i;

	src = (unsigned char *)pvSrc;
	dst = (unsigned char *)pvDst;
	for (i = 0; i < ui32Size; i++) {
		dst[i] = src[i];
	}
#else
	memcpy(pvDst, pvSrc, ui32Size);
#endif
}

IMG_UINT32 HostReadRegistryDWORDFromString(char *pcKey, char *pcValueName,
					   IMG_UINT32 * pui32Data)
{
	/* FIXME: Not yet implemented */
	return 0;
}

IMG_VOID *HostPageablePageAlloc(IMG_UINT32 ui32Pages)
{
	return (void *)vmalloc(ui32Pages * PAGE_SIZE);	/*, GFP_KERNEL); */
}

IMG_VOID HostPageablePageFree(IMG_VOID * pvBase)
{
	vfree(pvBase);
}

IMG_VOID *HostNonPageablePageAlloc(IMG_UINT32 ui32Pages)
{
	return (void *)vmalloc(ui32Pages * PAGE_SIZE);	/*, GFP_KERNEL); */
}

IMG_VOID HostNonPageablePageFree(IMG_VOID * pvBase)
{
	vfree(pvBase);
}

IMG_VOID *HostMapKrnBufIntoUser(IMG_VOID * pvKrnAddr, IMG_UINT32 ui32Size,
				IMG_VOID ** ppvMdl)
{
	/* XXX Not yet implemented */
	return IMG_NULL;
}

IMG_VOID HostUnMapKrnBufFromUser(IMG_VOID * pvUserAddr, IMG_VOID * pvMdl,
				 IMG_VOID * pvProcess)
{
	/* XXX Not yet implemented */
}

IMG_VOID HostCreateRegDeclStreams(IMG_VOID)
{
	/* XXX Not yet implemented */
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
typedef struct mutex MUTEX;
#define	INIT_MUTEX(m)		mutex_init(m)
#define	DOWN_TRYLOCK(m)		(!mutex_trylock(m))
#define	DOWN(m)			mutex_lock(m)
#define UP(m)			mutex_unlock(m)
#else
typedef struct semaphore MUTEX;
#define	INIT_MUTEX(m)		init_MUTEX(m)
#define	DOWN_TRYLOCK(m)		down_trylock(m)
#define	DOWN(m)			down(m)
#define UP(m)			up(m)
#endif

IMG_VOID *HostCreateMutex(IMG_VOID)
{
	MUTEX *psMutex;

	psMutex = kmalloc(sizeof(*psMutex), GFP_KERNEL);
	if (psMutex) {
		INIT_MUTEX(psMutex);
	}

	return psMutex;
}

IMG_VOID HostAquireMutex(IMG_VOID * pvMutex)
{
	BUG_ON(in_interrupt());

#if defined(PVR_DEBUG_DBGDRV_DETECT_HOST_MUTEX_COLLISIONS)
	if (DOWN_TRYLOCK((MUTEX *) pvMutex)) {
		printk(KERN_INFO "HostAquireMutex: Waiting for mutex\n");
		DOWN((MUTEX *) pvMutex);
	}
#else
	DOWN((MUTEX *) pvMutex);
#endif
}

IMG_VOID HostReleaseMutex(IMG_VOID * pvMutex)
{
	UP((MUTEX *) pvMutex);
}

IMG_VOID HostDestroyMutex(IMG_VOID * pvMutex)
{
	if (pvMutex) {
		kfree(pvMutex);
	}
}

#if defined(SUPPORT_DBGDRV_EVENT_OBJECTS)

#define	EVENT_WAIT_TIMEOUT_MS	500
#define	EVENT_WAIT_TIMEOUT_JIFFIES	(EVENT_WAIT_TIMEOUT_MS * HZ / 1000)

static int iStreamData;
static wait_queue_head_t sStreamDataEvent;

IMG_INT32 HostCreateEventObjects(IMG_VOID)
{
	init_waitqueue_head(&sStreamDataEvent);

	return 0;
}

IMG_VOID HostWaitForEvent(DBG_EVENT eEvent)
{
	switch (eEvent) {
	case DBG_EVENT_STREAM_DATA:
		/*
		 * More than one process may be woken up.
		 * Any process that wakes up should consume
		 * all the data from the streams.
		 */
		wait_event_interruptible_timeout(sStreamDataEvent,
						 iStreamData != 0,
						 EVENT_WAIT_TIMEOUT_JIFFIES);
		iStreamData = 0;
		break;
	default:
		/*
		 * For unknown events, enter an interruptible sleep.
		 */
		msleep_interruptible(EVENT_WAIT_TIMEOUT_MS);
		break;
	}
}

IMG_VOID HostSignalEvent(DBG_EVENT eEvent)
{
	switch (eEvent) {
	case DBG_EVENT_STREAM_DATA:
		iStreamData = 1;
		wake_up_interruptible(&sStreamDataEvent);
		break;
	default:
		break;
	}
}

IMG_VOID HostDestroyEventObjects(IMG_VOID)
{
}
#endif				/* defined(SUPPORT_DBGDRV_EVENT_OBJECTS) */
