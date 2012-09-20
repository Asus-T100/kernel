									    /*************************************************************************//*!
									       @File
									       @Title          Debug Functionality
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description     Provides kernel side Debug Functionality.
									       @License        Strictly Confidential.
    *//**************************************************************************/
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/hardirq.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/string.h>	// strncpy, strlen
#include <stdarg.h>
#include "img_types.h"
#include "servicesext.h"
#include "pvr_debug.h"
#include "srvkm.h"
#include "proc.h"
#include "mutex.h"
#include "linkage.h"
#include "pvr_uaccess.h"
#include "pvrsrv.h"

static IMG_BOOL VBAppend(IMG_CHAR * pszBuf, IMG_UINT32 ui32BufSiz,
			 const IMG_CHAR * pszFormat, va_list VArgs)
IMG_FORMAT_PRINTF(3, 0);

#if defined(PVRSRV_NEED_PVR_DPF)

#define PVR_MAX_FILEPATH_LEN 256

static IMG_BOOL BAppend(IMG_CHAR * pszBuf, IMG_UINT32 ui32BufSiz,
			const IMG_CHAR * pszFormat, ...)
IMG_FORMAT_PRINTF(3, 4);

/* NOTE: Must NOT be static! Used in module.c.. */
IMG_UINT32 gPVRDebugLevel = (DBGPRIV_FATAL | DBGPRIV_ERROR | DBGPRIV_WARNING);

#endif				/* defined(PVRSRV_NEED_PVR_DPF) || defined(PVRSRV_NEED_PVR_TRACE) */

#define	PVR_MAX_MSG_LEN PVR_MAX_DEBUG_MESSAGE_LEN

/* Message buffer for non-IRQ messages */
static IMG_CHAR gszBufferNonIRQ[PVR_MAX_MSG_LEN + 1];

/* Message buffer for IRQ messages */
static IMG_CHAR gszBufferIRQ[PVR_MAX_MSG_LEN + 1];

/* The lock is used to control access to gszBufferNonIRQ */
static PVRSRV_LINUX_MUTEX gsDebugMutexNonIRQ;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
/* The lock is used to control access to gszBufferIRQ */
			 /* PRQA S 0671,0685 1 *//* ignore warnings about C99 style initialisation */
static spinlock_t gsDebugLockIRQ = SPIN_LOCK_UNLOCKED;
#else
static DEFINE_SPINLOCK(gsDebugLockIRQ);
#endif

#if !defined (USE_SPIN_LOCK)	/* to keep QAC happy */
#define	USE_SPIN_LOCK (in_interrupt() || !preemptible())
#endif

static inline void GetBufferLock(unsigned long *pulLockFlags)
{
	if (USE_SPIN_LOCK) {
		spin_lock_irqsave(&gsDebugLockIRQ, *pulLockFlags);
	} else {
		LinuxLockMutex(&gsDebugMutexNonIRQ);
	}
}

static inline void ReleaseBufferLock(unsigned long ulLockFlags)
{
	if (USE_SPIN_LOCK) {
		spin_unlock_irqrestore(&gsDebugLockIRQ, ulLockFlags);
	} else {
		LinuxUnLockMutex(&gsDebugMutexNonIRQ);
	}
}

static inline void SelectBuffer(IMG_CHAR ** ppszBuf, IMG_UINT32 * pui32BufSiz)
{
	if (USE_SPIN_LOCK) {
		*ppszBuf = gszBufferIRQ;
		*pui32BufSiz = sizeof(gszBufferIRQ);
	} else {
		*ppszBuf = gszBufferNonIRQ;
		*pui32BufSiz = sizeof(gszBufferNonIRQ);
	}
}

/*
 * Append a string to a buffer using formatted conversion.
 * The function takes a variable number of arguments, pointed
 * to by the var args list.
 */
static IMG_BOOL VBAppend(IMG_CHAR * pszBuf, IMG_UINT32 ui32BufSiz,
			 const IMG_CHAR * pszFormat, va_list VArgs)
{
	IMG_UINT32 ui32Used;
	IMG_UINT32 ui32Space;
	IMG_INT32 i32Len;

	ui32Used = strlen(pszBuf);
	BUG_ON(ui32Used >= ui32BufSiz);
	ui32Space = ui32BufSiz - ui32Used;

	i32Len = vsnprintf(&pszBuf[ui32Used], ui32Space, pszFormat, VArgs);
	pszBuf[ui32BufSiz - 1] = 0;

	/* Return true if string was truncated */
	return (i32Len < 0
		|| i32Len >= (IMG_INT32) ui32Space) ? IMG_TRUE : IMG_FALSE;
}

/* Actually required for ReleasePrintf too */

IMG_VOID PVRDPFInit(IMG_VOID)
{
	LinuxInitMutex(&gsDebugMutexNonIRQ);
}

									    /*************************************************************************//*!
									       @Function       PVRSRVReleasePrintf
									       @Description    To output an important message to the user in release builds
									       @Input          pszFormat   The message format string
									       @Input          ...         Zero or more arguments for use by the format string
    *//**************************************************************************/
IMG_VOID PVRSRVReleasePrintf(const IMG_CHAR * pszFormat, ...)
{
	va_list vaArgs;
	unsigned long ulLockFlags = 0;
	IMG_CHAR *pszBuf;
	IMG_UINT32 ui32BufSiz;

	SelectBuffer(&pszBuf, &ui32BufSiz);

	va_start(vaArgs, pszFormat);

	GetBufferLock(&ulLockFlags);
	strncpy(pszBuf, "PVR_K: ", (ui32BufSiz - 1));

	if (VBAppend(pszBuf, ui32BufSiz, pszFormat, vaArgs)) {
		printk(KERN_ERR "PVR_K:(Message Truncated): %s\n", pszBuf);
	} else {
		printk(KERN_ERR "%s\n", pszBuf);
	}

	ReleaseBufferLock(ulLockFlags);
	va_end(vaArgs);

}

#if defined(PVRSRV_NEED_PVR_TRACE)

									    /*************************************************************************//*!
									       @Function       PVRTrace
									       @Description    To output a debug message to the user
									       @Input          pszFormat   The message format string
									       @Input          ...         Zero or more arguments for use by the format string
    *//**************************************************************************/
IMG_VOID PVRSRVTrace(const IMG_CHAR * pszFormat, ...)
{
	va_list VArgs;
	unsigned long ulLockFlags = 0;
	IMG_CHAR *pszBuf;
	IMG_UINT32 ui32BufSiz;

	SelectBuffer(&pszBuf, &ui32BufSiz);

	va_start(VArgs, pszFormat);

	GetBufferLock(&ulLockFlags);

	strncpy(pszBuf, "PVR: ", (ui32BufSiz - 1));

	if (VBAppend(pszBuf, ui32BufSiz, pszFormat, VArgs)) {
		printk(KERN_ERR "PVR_K:(Message Truncated): %s\n", pszBuf);
	} else {
		printk(KERN_ERR "%s\n", pszBuf);
	}

	ReleaseBufferLock(ulLockFlags);

	va_end(VArgs);
}

#endif				/* defined(PVRSRV_NEED_PVR_TRACE) */

#if defined(PVRSRV_NEED_PVR_DPF)

/*
 * Append a string to a buffer using formatted conversion.
 * The function takes a variable number of arguments, calling
 * VBAppend to do the actual work.
 */
static IMG_BOOL BAppend(IMG_CHAR * pszBuf, IMG_UINT32 ui32BufSiz,
			const IMG_CHAR * pszFormat, ...)
{
	va_list VArgs;
	IMG_BOOL bTrunc;

	va_start(VArgs, pszFormat);

	bTrunc = VBAppend(pszBuf, ui32BufSiz, pszFormat, VArgs);

	va_end(VArgs);

	return bTrunc;
}

									    /*************************************************************************//*!
									       @Function       PVRSRVDebugPrintf
									       @Description    To output a debug message to the user
									       @Input          uDebugLevel The current debug level
									       @Input          pszFile     The source file generating the message
									       @Input          uLine       The line of the source file
									       @Input          pszFormat   The message format string
									       @Input          ...         Zero or more arguments for use by the format string
    *//**************************************************************************/
IMG_VOID PVRSRVDebugPrintf(IMG_UINT32 ui32DebugLevel,
			   const IMG_CHAR * pszFullFileName,
			   IMG_UINT32 ui32Line, const IMG_CHAR * pszFormat, ...
    )
{
	IMG_BOOL bTrace;
	const IMG_CHAR *pszFileName = pszFullFileName;
	IMG_CHAR *pszLeafName;

	bTrace =
	    (IMG_BOOL) (ui32DebugLevel & DBGPRIV_CALLTRACE) ? IMG_TRUE :
	    IMG_FALSE;

	if (gPVRDebugLevel & ui32DebugLevel) {
		va_list vaArgs;
		unsigned long ulLockFlags = 0;
		IMG_CHAR *pszBuf;
		IMG_UINT32 ui32BufSiz;

		SelectBuffer(&pszBuf, &ui32BufSiz);

		va_start(vaArgs, pszFormat);

		GetBufferLock(&ulLockFlags);

		/* Add in the level of warning */
		if (bTrace == IMG_FALSE) {
			switch (ui32DebugLevel) {
			case DBGPRIV_FATAL:
				{
					strncpy(pszBuf, "PVR_K:(Fatal): ",
						(ui32BufSiz - 1));
					break;
				}
			case DBGPRIV_ERROR:
				{
					strncpy(pszBuf, "PVR_K:(Error): ",
						(ui32BufSiz - 1));
					break;
				}
			case DBGPRIV_WARNING:
				{
					strncpy(pszBuf, "PVR_K:(Warning): ",
						(ui32BufSiz - 1));
					break;
				}
			case DBGPRIV_MESSAGE:
				{
					strncpy(pszBuf, "PVR_K:(Message): ",
						(ui32BufSiz - 1));
					break;
				}
			case DBGPRIV_VERBOSE:
				{
					strncpy(pszBuf, "PVR_K:(Verbose): ",
						(ui32BufSiz - 1));
					break;
				}
			default:
				{
					strncpy(pszBuf,
						"PVR_K:(Unknown message level)",
						(ui32BufSiz - 1));
					break;
				}
			}
		} else {
			strncpy(pszBuf, "PVR_K: ", (ui32BufSiz - 1));
		}

		if (VBAppend(pszBuf, ui32BufSiz, pszFormat, vaArgs)) {
			printk(KERN_ERR "PVR_K:(Message Truncated): %s\n",
			       pszBuf);
		} else {
			/* Traces don't need a location */
			if (bTrace == IMG_FALSE) {
#ifdef DEBUG_LOG_PATH_TRUNCATE
				/* Buffer for rewriting filepath in log messages */
				static IMG_CHAR
				    szFileNameRewrite[PVR_MAX_FILEPATH_LEN];

				IMG_CHAR *pszTruncIter;
				IMG_CHAR *pszTruncBackInter;

				/* Truncate path (DEBUG_LOG_PATH_TRUNCATE shoud be set to EURASIA env var) */
				pszFileName =
				    pszFullFileName +
				    strlen(DEBUG_LOG_PATH_TRUNCATE) + 1;

				/* Try to find '/../' entries and remove it together with
				   previous entry. Repeat unit all removed */
				strncpy(szFileNameRewrite, pszFileName,
					PVR_MAX_FILEPATH_LEN);

				if (strlen(szFileNameRewrite) ==
				    PVR_MAX_FILEPATH_LEN - 1) {
					IMG_CHAR szTruncateMassage[] =
					    "FILENAME TRUNCATED";
					strcpy(szFileNameRewrite +
					       (PVR_MAX_FILEPATH_LEN - 1 -
						strlen(szTruncateMassage)),
					       szTruncateMassage);
				}

				pszTruncIter = szFileNameRewrite;
				while (*pszTruncIter++ != 0) {
					IMG_CHAR *pszNextStartPoint;
					/* Find '/../' pattern */
					if (!
					    ((*pszTruncIter == '/'
					      && (pszTruncIter - 4 >=
						  szFileNameRewrite))
					     && (*(pszTruncIter - 1) == '.')
					     && (*(pszTruncIter - 2) == '.')
					     && (*(pszTruncIter - 3) == '/'))
					    )
						continue;

					/* Find previous '/' */
					pszTruncBackInter = pszTruncIter - 3;
					while (*(--pszTruncBackInter) != '/') {
						if (pszTruncBackInter <=
						    szFileNameRewrite)
							break;
					}
					pszNextStartPoint = pszTruncBackInter;

					/* Remove found region */
					while (*pszTruncIter != 0) {
						*pszTruncBackInter++ =
						    *pszTruncIter++;
					}
					*pszTruncBackInter = 0;

					/* Start again */
					pszTruncIter = pszNextStartPoint;
				}

				pszFileName = szFileNameRewrite;
				/* Remove first '/' if exist (it's always relative path */
				if (*pszFileName == '/')
					pszFileName++;
#endif

#if !defined(__sh__)
				pszLeafName =
				    (IMG_CHAR *) strrchr(pszFileName, '\\');

				if (pszLeafName) {
					pszFileName = pszLeafName;
				}
#endif				/* __sh__ */
				pszLeafName =
				    (IMG_CHAR *) strrchr(pszFileName, '/');
				if (pszLeafName)
					pszFileName = pszLeafName;

				if (BAppend
				    (pszBuf, ui32BufSiz, " [%u, %s]", ui32Line,
				     pszFileName)) {
					printk(KERN_ERR
					       "PVR_K:(Message Truncated): %s\n",
					       pszBuf);
				} else {
					printk(KERN_ERR "%s\n", pszBuf);
				}
			} else {
				printk(KERN_ERR "%s\n", pszBuf);
			}
		}

		ReleaseBufferLock(ulLockFlags);

		va_end(vaArgs);
	}
}

#endif				/* PVRSRV_NEED_PVR_DPF */

#if defined(DEBUG)

IMG_INT PVRDebugProcSetLevel(struct file *file, const IMG_CHAR * buffer,
			     IMG_UINT32 count, IMG_VOID * data)
{
#define	_PROC_SET_BUFFER_SZ		2
	IMG_CHAR data_buffer[_PROC_SET_BUFFER_SZ];

	PVR_UNREFERENCED_PARAMETER(file);
	PVR_UNREFERENCED_PARAMETER(data);

	if (count != _PROC_SET_BUFFER_SZ) {
		return -EINVAL;
	} else {
		if (pvr_copy_from_user(data_buffer, buffer, count))
			return -EINVAL;
		if (data_buffer[count - 1] != '\n')
			return -EINVAL;
		/* FIXME: debug_level shouldn't be misused to set the services state */
		if ((data_buffer[0] == 'k') || ((data_buffer[0] == 'K'))) {
			PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
			psPVRSRVData->eServicesState =
			    PVRSRV_SERVICES_STATE_BAD;
		} else {
			gPVRDebugLevel = data_buffer[0] - '0';
		}

	}
	return (count);
}

void ProcSeqShowDebugLevel(struct seq_file *sfile, void *el)
{
	PVR_UNREFERENCED_PARAMETER(el);

	seq_printf(sfile, "%u\n", gPVRDebugLevel);
}

#endif				/* defined(DEBUG) */
