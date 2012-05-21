									    /*************************************************************************//*!
									       @File
									       @Title          Services API Header
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Exported services API details
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __SERVICES_H__
#define __SERVICES_H__

#if defined (__cplusplus)
extern "C" {
#endif

#include "img_defs.h"
#include "servicesext.h"
#include "pdumpdefs.h"
#include "pvrsrv_miscinfo.h"
/* FIXME: Can't do this as dc_client includes services.h
#include "dc_client.h"
*/

#if defined(LDDM)
/* LDDM build needs to include this for the allocation structure */
#include "umallocation.h"
#endif

#include "pvrsrv_device_types.h"

/* The comment below is the front page for code-generated doxygen documentation */
/*!
 ******************************************************************************
 @mainpage
 This document details the APIs and implementation of the Consumer Services.
 It is intended to be used in conjunction with the Consumer Services
 Software Architectural Specification and the Consumer Services Software
 Functional Specification.
 *****************************************************************************/

/******************************************************************************
 * 	#defines
 *****************************************************************************/

/*! 4k page size definition */
#define PVRSRV_4K_PAGE_SIZE					4096UL	/*!< Size of a 4K Page */
#define PVRSRV_4K_PAGE_SIZE_ALIGNSHIFT		12	/*!< Amount to shift an address by so that
							   it is always page-aligned */

#define EVENTOBJNAME_MAXLENGTH (50)	/*!< Max length of an event object name */

/*
	Flags for Services connection.
	Allows to define per-client policy for Services
*/
#define SRV_FLAGS_PERSIST		0x1	/* DOXYGEN_FIXME */

/*
	Pdump flags which are accessible to Services clients
*/
/* FIXME: defined to be the same as
 * #define PDUMP_FLAGS_CONTINUOUS		0x40000000UL
 * (from services/include/pdump.h)
 * The flags need to either be moved here, or e.g. all PDump functions need a bContinuous parameter
 */
#define PVRSRV_PDUMP_FLAGS_CONTINUOUS		0x40000000UL	/* DOXYGEN_FIXME */

#define PVRSRV_UNDEFINED_HEAP_ID			(~0LU)

/*!
 ******************************************************************************
 * User Module type
 *****************************************************************************/
	typedef enum {
		IMG_EGL = 0x00000001,	/*!< EGL Module */
		IMG_OPENGLES1 = 0x00000002,	/*!< OGLES1 Module */
		IMG_OPENGLES3 = 0x00000003,	/*!< OGLES3 Module */
		IMG_D3DM = 0x00000004,	/*!< D3DM Module */
		IMG_SRV_UM = 0x00000005,	/*!< Services User-Mode */
		IMG_SRV_INIT = 0x00000006,	/*!< Services initialisation */
		IMG_SRVCLIENT = 0x00000007,	/*!< Services Client */
		IMG_WDDMKMD = 0x00000008,	/*!< WDDM KMD */
		IMG_WDDM3DNODE = 0x00000009,	/*!< WDDM 3D Node */
		IMG_WDDMMVIDEONODE = 0x0000000A,	/*!< WDDM MVideo Node */
		IMG_WDDMVPBNODE = 0x0000000B,	/*!< WDDM VPB Node */
		IMG_OPENGL = 0x0000000C,	/*!< OpenGL */
		IMG_D3D = 0x0000000D,	/*!< D3D */
		IMG_OPENCL = 0x0000000E,	/*!< OpenCL */
		IMG_ANDROID_HAL = 0x0000000F,	/*!< Graphics HAL */

	} IMG_MODULE_ID;

/*! Max length of an App-Hint string */
#define APPHINT_MAX_STRING_SIZE	256

/*!
 ******************************************************************************
 * IMG data types
 *****************************************************************************/
	typedef enum {
		IMG_STRING_TYPE = 1,	/*!< String type */
		IMG_FLOAT_TYPE,	/*!< Float type */
		IMG_UINT_TYPE,	/*!< Unsigned Int type */
		IMG_INT_TYPE,	/*!< (Signed) Int type */
		IMG_FLAG_TYPE	/*!< Flag Type */
	} IMG_DATA_TYPE;

/******************************************************************************
 * Structure definitions.
 *****************************************************************************/

/*!
 * Forward declaration
 */
	typedef struct _PVRSRV_DEV_DATA_ *PPVRSRV_DEV_DATA;
/*!
 * Forward declaration (look on connection.h)
 */
	typedef struct _PVRSRV_CONNECTION_ PVRSRV_CONNECTION;

	/*************************************************************************//*!
	 * Client dev info
     *//*************************************************************************/
	typedef struct _PVRSRV_CLIENT_DEV_DATA_ {
		IMG_UINT32 ui32NumDevices;	/*!< Number of services-managed devices connected */
		PVRSRV_DEVICE_IDENTIFIER asDevID[PVRSRV_MAX_DEVICES];	/*!< Device identifiers */
		 PVRSRV_ERROR(*apfnDevConnect[PVRSRV_MAX_DEVICES]) (PPVRSRV_DEV_DATA);	/*!< device-specific connection callback */
		 PVRSRV_ERROR(*apfnDumpTrace[PVRSRV_MAX_DEVICES]) (PPVRSRV_DEV_DATA);	/*!< device-specific debug trace callback */

	} PVRSRV_CLIENT_DEV_DATA;

/*!
 ******************************************************************************
 * This structure allows the user mode glue code to have an OS independent
 * set of prototypes.
 *****************************************************************************/
	typedef struct _PVRSRV_DEV_DATA_ {
		IMG_CONST PVRSRV_CONNECTION *psConnection;	/*!< Services connection info */
		IMG_HANDLE hDevCookie;	/*!< Dev cookie */

		IMG_BOOL bSyncPrimAlreadyPdumped;

	} PVRSRV_DEV_DATA;

/*
	FIXME:
	Remove ASAP as we don't use these in services, but clients still
	use the structures
*/
/*!
 ******************************************************************************
 * address:value register structure
 *****************************************************************************/
	typedef struct _PVRSRV_HWREG_ {
		IMG_UINT32 ui32RegAddr;	/*!< Address */
		IMG_UINT32 ui32RegVal;	/*!< value */
	} PVRSRV_HWREG;

/*!
 ******************************************************************************
 * address:value register structure
 *****************************************************************************/
	typedef struct _PVRSRV_HWREG64_ {
		IMG_UINT32 ui32RegAddr;	/*!< Address */
		IMG_UINT64 ui64RegVal;	/*!< value */
	} PVRSRV_HWREG64;

	/*************************************************************************//*! 
	   PVR Client Event handling in Services
    *//**************************************************************************/
	typedef enum _PVRSRV_CLIENT_EVENT_ {
		PVRSRV_CLIENT_EVENT_HWTIMEOUT = 0,	/* DOXYGEN_FIXME */
	} PVRSRV_CLIENT_EVENT;

	/**************************************************************************//*!
	   @Function       PVRSRVClientEvent
	   @Description    Handles timeouts occurring in client drivers
	   @Input          eEvent          event type
	   @Input          psDevData       pointer to the PVRSRV_DEV_DATA context
	   @Input          pvData          client-specific data
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_ 
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVClientEvent(IMG_CONST
							PVRSRV_CLIENT_EVENT
							eEvent,
							PVRSRV_DEV_DATA *
							psDevData,
							IMG_PVOID pvData);

/******************************************************************************
 * PVR Services API prototypes.
 *****************************************************************************/

	/**************************************************************************//*!
	   @Function       PVRSRVConnect
	   @Description    Creates a connection from an application to the services 
	   module and initialises device-specific call-back functions.
	   @Output         ppsConnection   on Success, *ppsConnection is set to the new 
	   PVRSRV_CONNECTION instance.
	   @Input          ui32SrvFlags    a bit-wise OR of the following:
	   SRV_FLAGS_PERSIST
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_ 
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVConnect(PVRSRV_CONNECTION **
						    ppsConnection,
						    IMG_UINT32 ui32SrvFlags);

	/**************************************************************************//*!
	   @Function       PVRSRVDisconnect 
	   @Description    Disconnects from the services module
	   @Input          psConnection    the connection to be disconnected
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVDisconnect(PVRSRV_CONNECTION *
						       psConnection);

	/**************************************************************************//*!
	   @Function       PVRSRVEnumerateDevices
	   @Description    Enumerate all PowerVR services supported devices in the 
	   system.

	   The function returns a list of the device IDs stored either
	   in the services (or constructed in the user mode glue 
	   component in certain environments). The number of devices 
	   in the list is also returned.

	   The user is required to provide a buffer large enough to 
	   receive an array of MAX_NUM_DEVICE_IDS *
	   PVRSRV_DEVICE_IDENTIFIER structures.

	   In a binary layered component which does not support dynamic
	   runtime selection, the glue code should compile to return 
	   the supported devices statically, e.g. multiple instances of
	   the same device if multiple devices are supported, or the 
	   target combination of RGX and display device.

	   ** FIXME:  should we really have device specific references here?  FIXME! **

	   In the case of an environment (for instance) where one RGX
	   may connect to two display devices this code would enumerate
	   all three devices and even non-dynamic RGX selection code 
	   should retain the facility to parse the list to find the
	   index of the RGX device.}
	   @Input          psConnection    Services connection
	   @Output         puiNumDevices   Number of devices present in the system
	   @Output         puiDevIDs       Pointer to called supplied array of
	   PVRSRV_DEVICE_IDENTIFIER structures. The
	   array is assumed to be at least
	   PVRSRV_MAX_DEVICES long.
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVEnumerateDevices(IMG_CONST
							     PVRSRV_CONNECTION *
							     psConnection,
							     IMG_UINT32 *
							     puiNumDevices,
							     PVRSRV_DEVICE_IDENTIFIER
							     * puiDevIDs);

	/**************************************************************************//*!
	   @Function       PVRSRVAcquireDeviceData
	   @Description    Returns device info structure pointer for the requested device.
	   This populates a PVRSRV_DEV_DATA structure with appropriate 
	   pointers to the DevInfo structure for the device requested.

	   In a non-plug-and-play the first call to GetDeviceInfo for a
	   device causes device initialisation

	   Calls to GetDeviceInfo are reference counted
	   @Input          psConnection    Services connection
	   @Input          uiDevIndex      Index to the required device obtained from the 
	   PVRSRVEnumerateDevice function 
	   @Output         psDevData       The returned Device Data
	   @Input          eDeviceType     Required device type. If type is unknown use 
	   uiDevIndex to locate device data
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVAcquireDeviceData(IMG_CONST
							      PVRSRV_CONNECTION
							      * psConnection,
							      IMG_UINT32
							      uiDevIndex,
							      PVRSRV_DEV_DATA *
							      psDevData,
							      PVRSRV_DEVICE_TYPE
							      eDeviceType);

	/**************************************************************************//*!
	   @Function       PVRSRVGetMiscInfo
	   @Description    Retrieves miscellaneous information from services
	   @Input          psConnection    Services connection
	   @Output         psMiscInfo      the returned miscellaneous info
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVGetMiscInfo(IMG_CONST
							PVRSRV_CONNECTION *
							psConnection,
							PVRSRV_MISC_INFO *
							psMiscInfo);

	/**************************************************************************//*!
	   @Function       PVRSRVReleaseMiscInfo
	   @Description    Releases the miscellaneous information previously provided by
	   PVRSRVGetMiscInfo
	   @Input          psConnection    Services connection
	   @Input          psMiscInfo      the misc info to be released
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVReleaseMiscInfo(IMG_CONST
							    PVRSRV_CONNECTION *
							    psConnection,
							    PVRSRV_MISC_INFO *
							    psMiscInfo);

	/**************************************************************************//*!
	   @Function       PVRSRVPollForValue
	   @Description    Polls for a value to match a masked read of System Memory.
	   The function returns when either (1) the value read back
	   matches ui32Value, or (2) the maximum number of tries has
	   been reached.
	   @Input          psConnection        Services connection
	   @Input          hOSEvent            Handle to OS event object to wait for
	   @Input          pui32LinMemAddr     the address of the memory to poll
	   @Input          ui32Value           the required value
	   @Input          ui32Mask            the mask to use
	   @Input          ui32Waitus          interval between tries (us)
	   @Input          ui32Tries           number of tries to make before giving up
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a 
	   PVRSRV_ error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR PVRSRVPollForValue(const PVRSRV_CONNECTION *
					    psConnection, IMG_HANDLE hOSEvent,
					    volatile IMG_UINT32 *
					    pui32LinMemAddr,
					    IMG_UINT32 ui32Value,
					    IMG_UINT32 ui32Mask,
					    IMG_UINT32 ui32Waitus,
					    IMG_UINT32 ui32Tries);

/******************************************************************************
 * PDUMP Function prototypes...
 *****************************************************************************/
#if defined(PDUMP)
	/**************************************************************************//*!
	   @Function       PVRSRVPDumpInit
	   @Description    Pdump initialisation
	   @Input          psConnection    Services connection
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVPDumpInit(IMG_CONST
						      PVRSRV_CONNECTION *
						      psConnection);

	/**************************************************************************//*!
	   @Function       PVRSRVPDumpStartInitPhase
	   @Description    Resume the pdump init phase state   
	   @Input          psConnection    Services connection
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVPDumpStartInitPhase(IMG_CONST
								PVRSRV_CONNECTION
								* psConnection);

	/**************************************************************************//*!
	   @Function       PVRSRVPDumpStopInitPhase
	   @Description    Stop the pdump init phase state
	   @Input          psConnection    Services connection
	   @Input          eModuleID       Which module is requesting to stop the init phase
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVPDumpStopInitPhase(IMG_CONST
							       PVRSRV_CONNECTION
							       * psConnection,
							       IMG_MODULE_ID
							       eModuleID);

	/**************************************************************************//*!
	   @Function       PVRSRVPDumpSetFrame
	   @Description    Sets the pdump frame
	   @Input          psConnection    Services connection
	   @Input          ui32Frame       DOXYGEN_FIXME
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
    *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVPDumpSetFrame(IMG_CONST
							  PVRSRV_CONNECTION *
							  psConnection,
							  IMG_UINT32 ui32Frame);

	/**************************************************************************//*!
	   @Function       PVRSRVPDumpIsLastCaptureFrame
	   @Description    Returns whether this is the last frame of the capture range
	   @Input          psConnection    Services connection
	   @Return         IMG_BOOL:               IMG_TRUE if last frame,
	   IMG_FALSE otherwise
    *//**************************************************************************/
	 IMG_IMPORT
	    IMG_BOOL IMG_CALLCONV PVRSRVPDumpIsLastCaptureFrame(IMG_CONST
								PVRSRV_CONNECTION
								* psConnection);

	/**************************************************************************//*!
	   @Function       PVRSRVPDumpComment
	   @Description    PDumps a comment
	   @Input          psConnection        Services connection
	   @Input          pszComment          Comment to be inserted
	   @Input          bContinuous         DOXYGEN_FIXME
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVPDumpComment(IMG_CONST
							 PVRSRV_CONNECTION *
							 psConnection,
							 IMG_CONST IMG_CHAR *
							 pszComment,
							 IMG_BOOL bContinuous);

	/**************************************************************************//*!
	   @Function       PVRSRVPDumpCommentf
	   @Description    PDumps a formatted comment
	   @Input          psConnection        Services connection
	   @Input          bContinuous         DOXYGEN_FIXME
	   @Input          pszFormat           Format string
	   @Input          ...                 vararg list
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVPDumpCommentf(IMG_CONST
							  PVRSRV_CONNECTION *
							  psConnection,
							  IMG_BOOL bContinuous,
							  IMG_CONST IMG_CHAR *
							  pszFormat, ...)
	 IMG_FORMAT_PRINTF(3, 4);

	/**************************************************************************//*!
	   @Function       PVRSRVPDumpCommentWithFlagsf
	   @Description    PDumps a formatted comment, passing in flags
	   @Input          psConnection        Services connection
	   @Input          ui32Flags           Flags
	   @Input          pszFormat           Format string
	   @Input          ...                 vararg list
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVPDumpCommentWithFlagsf(IMG_CONST
								   PVRSRV_CONNECTION
								   *
								   psConnection,
								   IMG_UINT32
								   ui32Flags,
								   IMG_CONST
								   IMG_CHAR *
								   pszFormat,
								   ...)
	 IMG_FORMAT_PRINTF(3, 4);

	/**************************************************************************//*!
	   @Function       PVRSRVPDumpIsCapturing
	   @Description    Reports whether PDump is currently capturing or not
	   @Input          psConnection        Services connection
	   @Output         pbIsCapturing       Indicates whether PDump is currently
	   capturing
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVPDumpIsCapturing(IMG_CONST
							     PVRSRV_CONNECTION *
							     psConnection,
							     IMG_BOOL *
							     pbIsCapturing);

	/**************************************************************************//*!
	   @Function       PVRSRVPDumpIsCapturingTest
	   @Description    DOXYGEN_FIXME
	   @Input          psConnection        Services connection
	   @Return         IMG_BOOL
     *//**************************************************************************/
	 IMG_IMPORT
	    IMG_BOOL IMG_CALLCONV PVRSRVPDumpIsCapturingTest(IMG_CONST
							     PVRSRV_CONNECTION *
							     psConnection);
#else				/* PDUMP */

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpInit)
#endif
	static INLINE PVRSRV_ERROR
	    PVRSRVPDumpInit(IMG_CONST PVRSRV_CONNECTION * psConnection) {
		PVR_UNREFERENCED_PARAMETER(psConnection);
		return PVRSRV_OK;
	}
#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpStartInitPhase)
#endif
	static INLINE PVRSRV_ERROR
	    PVRSRVPDumpStartInitPhase(IMG_CONST PVRSRV_CONNECTION *
				      psConnection) {
		PVR_UNREFERENCED_PARAMETER(psConnection);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpStopInitPhase)
#endif
	static INLINE PVRSRV_ERROR
	    PVRSRVPDumpStopInitPhase(IMG_CONST PVRSRV_CONNECTION * psConnection)
	{
		PVR_UNREFERENCED_PARAMETER(psConnection);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpSetFrame)
#endif
	static INLINE PVRSRV_ERROR
	    PVRSRVPDumpSetFrame(IMG_CONST PVRSRV_CONNECTION * psConnection,
				IMG_UINT32 ui32Frame) {
		PVR_UNREFERENCED_PARAMETER(psConnection);
		PVR_UNREFERENCED_PARAMETER(ui32Frame);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpIsLastCaptureFrame)
#endif
	static INLINE IMG_BOOL
	    PVRSRVPDumpIsLastCaptureFrame(IMG_CONST PVRSRV_CONNECTION *
					  psConnection) {
		PVR_UNREFERENCED_PARAMETER(psConnection);
		return IMG_FALSE;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpComment)
#endif
	static INLINE PVRSRV_ERROR
	    PVRSRVPDumpComment(IMG_CONST PVRSRV_CONNECTION * psConnection,
			       IMG_CONST IMG_CHAR * pszComment,
			       IMG_BOOL bContinuous) {
		PVR_UNREFERENCED_PARAMETER(psConnection);
		PVR_UNREFERENCED_PARAMETER(pszComment);
		PVR_UNREFERENCED_PARAMETER(bContinuous);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpCommentf)
#endif
	static INLINE PVRSRV_ERROR
	    PVRSRVPDumpCommentf(IMG_CONST PVRSRV_CONNECTION * psConnection,
				IMG_BOOL bContinuous,
				IMG_CONST IMG_CHAR * pszFormat, ...) {
		PVR_UNREFERENCED_PARAMETER(psConnection);
		PVR_UNREFERENCED_PARAMETER(bContinuous);
		PVR_UNREFERENCED_PARAMETER(pszFormat);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpCommentWithFlagsf)
#endif
	static INLINE PVRSRV_ERROR
	    PVRSRVPDumpCommentWithFlagsf(IMG_CONST PVRSRV_CONNECTION *
					 psConnection, IMG_UINT32 ui32Flags,
					 IMG_CONST IMG_CHAR * pszFormat, ...) {
		PVR_UNREFERENCED_PARAMETER(psConnection);
		PVR_UNREFERENCED_PARAMETER(ui32Flags);
		PVR_UNREFERENCED_PARAMETER(pszFormat);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpIsCapturing)
#endif
	static INLINE PVRSRV_ERROR
	    PVRSRVPDumpIsCapturing(IMG_CONST PVRSRV_CONNECTION * psConnection,
				   IMG_BOOL * pbIsCapturing) {
		PVR_UNREFERENCED_PARAMETER(psConnection);
		*pbIsCapturing = IMG_FALSE;
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpIsCapturingTest)
#endif
	static INLINE IMG_BOOL
	    PVRSRVPDumpIsCapturingTest(IMG_CONST PVRSRV_CONNECTION *
				       psConnection) {
		PVR_UNREFERENCED_PARAMETER(psConnection);
		return IMG_FALSE;
	}
#endif				/* PDUMP */

	/**************************************************************************//*!
	   @Function       PVRSRVLoadLibrary
	   @Description    Load the named Dynamic-Link (Shared) Library. This will perform
	   reference counting in association with PVRSRVUnloadLibrary,
	   so for example if the same library is loaded twice and unloaded once,
	   a reference to the library will remain.
	   @Input          pszLibraryName      the name of the library to load
	   @Return         IMG_HANDLE          On success, the handle of the newly-loaded
	   library. Otherwise, zero.
     *//**************************************************************************/
	IMG_IMPORT IMG_HANDLE PVRSRVLoadLibrary(const IMG_CHAR *
						pszLibraryName);

	/**************************************************************************//*!
	   @Function       PVRSRVUnloadLibrary
	   @Description    Unload the Dynamic-Link (Shared) Library which had previously been
	   loaded using PVRSRVLoadLibrary(). See PVRSRVLoadLibrary() for
	   information regarding reference counting.
	   @Input          hExtDrv             handle of the Dynamic-Link / Shared library
	   to unload, as returned by PVRSRVLoadLibrary().
	   @Return         PVRSRV_ERROR        PVRSRV_OK if successful. Otherwise,
	   PVRSRV_ERROR_UNLOAD_LIBRARY_FAILED.
     *//**************************************************************************/
	IMG_IMPORT PVRSRV_ERROR PVRSRVUnloadLibrary(IMG_HANDLE hExtDrv);

	/**************************************************************************//*!
	   @Function       PVRSRVGetLibFuncAddr
	   @Description    Returns the address of a function in a Dynamic-Link / Shared
	   Library.
	   @Input          hExtDrv             handle of the Dynamic-Link / Shared Library
	   in which the function resides
	   @Input          pszFunctionName     the name of the function
	   @Output         ppvFuncAddr         on success, the address of the function
	   requested. Otherwise, NULL.
	   @Return         PVRSRV_ERROR        PVRSRV_OK if successful. Otherwise,
	   PVRSRV_ERROR_UNABLE_TO_GET_FUNC_ADDR.
     *//**************************************************************************/
	IMG_IMPORT PVRSRV_ERROR PVRSRVGetLibFuncAddr(IMG_HANDLE hExtDrv,
						     const IMG_CHAR *
						     pszFunctionName,
						     IMG_VOID ** ppvFuncAddr);

	/**************************************************************************//*!
	   @Function       PVRSRVClockus
	   @Description    Returns the current system clock time, in microseconds.  Note 
	   that this does not necessarily guarantee microsecond accuracy.
	   @Return         IMG_UINT32          the curent system clock time, in 
	   microseconds
     *//**************************************************************************/
	IMG_IMPORT IMG_UINT32 PVRSRVClockus(void);

	/**************************************************************************//*!
	   @Function       PVRSRVWaitus
	   @Description    Waits for the specified number of microseconds
	   @Input          ui32Timeus          the time to wait for, in microseconds 
	   @Return         None
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID PVRSRVWaitus(IMG_UINT32 ui32Timeus);

	/**************************************************************************//*!
	   @Function       PVRSRVReleaseThreadQuanta
	   @Description    Releases thread quanta
	   @Return         None
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID PVRSRVReleaseThreadQuanta(void);

	/**************************************************************************//*!
	   @Function       PVRSRVGetCurrentProcessID
	   @Description    Returns handle for current process
	   @Return         ID of current process
     *//**************************************************************************/
	IMG_IMPORT IMG_PID IMG_CALLCONV PVRSRVGetCurrentProcessID(void);

	/**************************************************************************//*!
	   @Function       PVRSRVSetLocale
	   @Description    Thin wrapper on posix setlocale
	   @Input          pszLocale
	   @Return         IMG_CHAR *      DOXYGEN_FIXME
     *//**************************************************************************/
	IMG_IMPORT IMG_CHAR *IMG_CALLCONV PVRSRVSetLocale(const IMG_CHAR *
							  pszLocale);

	/**************************************************************************//*!
	   @Function       PVRSRVCreateAppHintState
	   @Description    Create app hint state
	   @Input          eModuleID       DOXYGEN_FIXME
	   @Input          pszAppName      DOXYGEN_FIXME
	   @Output         ppvState        DOXYGEN_FIXME
	   @Return         None
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVCreateAppHintState(IMG_MODULE_ID
								  eModuleID,
								  const IMG_CHAR
								  * pszAppName,
								  IMG_VOID **
								  ppvState);
	/**************************************************************************//*!
	   @Function       PVRSRVFreeAppHintState
	   @Description    Free the app hint state, if it was created
	   @Input          eModuleID       DOXYGEN_FIXME
	   @Input          pvHintState     DOXYGEN_FIXME
	   @Return         None
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVFreeAppHintState(IMG_MODULE_ID
								eModuleID,
								IMG_VOID *
								pvHintState);

	/**************************************************************************//*!
	   @Function       PVRSRVGetAppHint
	   @Description    Return the value of this hint from state or use default
	   @Input          pvHintState     DOXYGEN_FIXME
	   @Input          pszHintName     DOXYGEN_FIXME
	   @Input          eDataType       DOXYGEN_FIXME
	   @Input          pvDefault       DOXYGEN_FIXME
	   @Output         pvReturn        DOXYGEN_FIXME
	   @Return         IMG_BOOL        True if hint read, False if used default
     *//**************************************************************************/
	IMG_IMPORT IMG_BOOL IMG_CALLCONV PVRSRVGetAppHint(IMG_VOID *
							  pvHintState,
							  const IMG_CHAR *
							  pszHintName,
							  IMG_DATA_TYPE
							  eDataType,
							  const IMG_VOID *
							  pvDefault,
							  IMG_VOID * pvReturn);

/******************************************************************************
 * Memory API(s)
 *****************************************************************************/

/* Exported APIs */
	/**************************************************************************//*!
	   @Function       PVRSRVAllocUserModeMem
	   @Description    Allocate a block of user-mode memory
	   @Input          ui32Size    the amount of memory to allocate
	   @Return         IMG_PVOID   On success, a pointer to the memory allocated.
	   Otherwise, NULL.
     *//**************************************************************************/
	IMG_IMPORT IMG_PVOID IMG_CALLCONV PVRSRVAllocUserModeMem(IMG_SIZE_T
								 ui32Size);

	/**************************************************************************//*!
	   @Function       PVRSRVCallocUserModeMem
	   @Description    Allocate a block of user-mode memory
	   @Input          ui32Size    the amount of memory to allocate
	   @Return         IMG_PVOID   On success, a pointer to the memory allocated.
	   Otherwise, NULL.
     *//**************************************************************************/
	IMG_IMPORT IMG_PVOID IMG_CALLCONV PVRSRVCallocUserModeMem(IMG_SIZE_T
								  ui32Size);

	/**************************************************************************//*!
	   @Function       PVRSRVReallocUserModeMem
	   @Description    Re-allocate a block of memory
	   @Input          pvBase      the address of the existing memory, previously
	   allocated with PVRSRVAllocUserModeMem
	   @Input          uNewSize    the newly-desired size of the memory chunk
	   @Return         IMG_PVOID   On success, a pointer to the memory block. If the
	   size of the block could not be changed, the
	   return value is NULL.
     *//**************************************************************************/
	IMG_IMPORT IMG_PVOID IMG_CALLCONV PVRSRVReallocUserModeMem(IMG_PVOID
								   pvBase,
								   IMG_SIZE_T
								   uNewSize);
	/**************************************************************************//*!
	   @Function       PVRSRVFreeUserModeMem
	   @Description    Free a block of memory previously allocated with
	   PVRSRVAllocUserModeMem
	   @Input          pvMem       pointer to the block of memory to be freed
	   @Return         None
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVFreeUserModeMem(IMG_PVOID pvMem);
	/**************************************************************************//*!
	   @Function       PVRSRVMemCopy
	   @Description    Copy a block of memory
	   @Input          pvDst       pointer to the destination
	   @Input          pvSrc       pointer to the source location
	   @Input          ui32Size    the amount of memory to copy
	   @Return         None
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID PVRSRVMemCopy(IMG_VOID * pvDst,
					  const IMG_VOID * pvSrc,
					  IMG_SIZE_T ui32Size);

	/**************************************************************************//*!
	   @Function       PVRSRVMemSet
	   @Description    Set all bytes in a region of memory to the specified value
	   @Input          pvDest      pointer to the start of the memory region
	   @Input          ui8Value    the value to be written
	   @Input          ui32Size    the number of bytes to be set to ui8Value
	   @Return         None
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID PVRSRVMemSet(IMG_VOID * pvDest, IMG_UINT8 ui8Value,
					 IMG_SIZE_T ui32Size);

	/**************************************************************************//*!
	   @Function       PVRSRVLockProcessGlobalMutex
	   @Description    Locking function for non-recursive coarse-grained mutex shared
	   between all threads in a proccess.
	   @Output         phMutex             DOXYGEN_FIXME
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVLockProcessGlobalMutex(IMG_VOID);

	/**************************************************************************//*!
	   @Function       PVRSRVUnlockProcessGlobalMutex
	   @Description    Unlocking function for non-recursive coarse-grained mutex shared
	   between all threads in a proccess.
	   @Output         phMutex             DOXYGEN_FIXME
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID IMG_CALLCONV
	    PVRSRVUnlockProcessGlobalMutex(IMG_VOID);

	struct _PVRSRV_MUTEX_OPAQUE_STRUCT_;
	typedef struct _PVRSRV_MUTEX_OPAQUE_STRUCT_ *PVRSRV_MUTEX_HANDLE;	/*!< Convenience typedef */

	/**************************************************************************//*!
	   @Function       PVRSRVCreateMutex
	   @Description    DOXYGEN_FIXME
	   @Output         phMutex             DOXYGEN_FIXME
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
#if !defined(PVR_DEBUG_MUTEXES)
	IMG_IMPORT PVRSRV_ERROR IMG_CALLCONV
	    PVRSRVCreateMutex(PVRSRV_MUTEX_HANDLE * phMutex);
#else
	IMG_IMPORT PVRSRV_ERROR IMG_CALLCONV
	    PVRSRVCreateMutex(PVRSRV_MUTEX_HANDLE * phMutex,
			      IMG_CHAR pszMutexName[], IMG_CHAR pszFilename[],
			      IMG_INT iLine);
#define PVRSRVCreateMutex(phMutex) \
	PVRSRVCreateMutex(phMutex, #phMutex, __FILE__, __LINE__)
#endif

	/**************************************************************************//*!
	   @Function       PVRSRVDestroyMutex
	   @Description    Create a mutex.
	   @Input          hMutex              On success, filled with the new Mutex
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**********************************************************************/
#if !defined(PVR_DEBUG_MUTEXES)
	IMG_IMPORT PVRSRV_ERROR IMG_CALLCONV
	    PVRSRVDestroyMutex(PVRSRV_MUTEX_HANDLE hMutex);
#else
	IMG_IMPORT PVRSRV_ERROR IMG_CALLCONV
	    PVRSRVDestroyMutex(PVRSRV_MUTEX_HANDLE hMutex,
			       IMG_CHAR pszMutexName[], IMG_CHAR pszFilename[],
			       IMG_INT iLine);
#define PVRSRVDestroyMutex(hMutex) \
	PVRSRVDestroyMutex(hMutex, #hMutex, __FILE__, __LINE__)
#endif

	/**************************************************************************//*!
	   @Function       PVRSRVLockMutex
	   @Description    Lock the mutex passed
	   @Input          hMutex              handle of the mutex to be locked
	   @Return         None
     *//**********************************************************************/
#if !defined(PVR_DEBUG_MUTEXES)
	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVLockMutex(PVRSRV_MUTEX_HANDLE
							 hMutex);
#else
	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVLockMutex(PVRSRV_MUTEX_HANDLE
							 hMutex,
							 IMG_CHAR
							 pszMutexName[],
							 IMG_CHAR pszFilename[],
							 IMG_INT iLine);
#define PVRSRVLockMutex(hMutex) \
	PVRSRVLockMutex(hMutex, #hMutex, __FILE__, __LINE__)
#endif

	/**************************************************************************//*!
	   @Function       PVRSRVUnlockMutex
	   @Description    Unlock the mutex passed
	   @Input          hMutex              handle of the mutex to be unlocked
	   @Return         None
     *//**********************************************************************/
#if !defined(PVR_DEBUG_MUTEXES)
	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVUnlockMutex(PVRSRV_MUTEX_HANDLE
							   hMutex);
#else
	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVUnlockMutex(PVRSRV_MUTEX_HANDLE
							   hMutex,
							   IMG_CHAR
							   pszMutexName[],
							   IMG_CHAR
							   pszFilename[],
							   IMG_INT iLine);
#define PVRSRVUnlockMutex(hMutex) \
	PVRSRVUnlockMutex(hMutex, #hMutex, __FILE__, __LINE__)
#endif

	struct _PVRSRV_SEMAPHORE_OPAQUE_STRUCT_;
	typedef struct _PVRSRV_SEMAPHORE_OPAQUE_STRUCT_ *PVRSRV_SEMAPHORE_HANDLE;	/*!< Convenience typedef */

#if defined(_MSC_VER)
	/*! 
	   Used when waiting for a semaphore to become unlocked. Indicates that 
	   the caller is willing to wait forever.
	 */
#define IMG_SEMAPHORE_WAIT_INFINITE       ((IMG_UINT64)0xFFFFFFFFFFFFFFFF)
#else
	/*! 
	   Used when waiting for a semaphore to become unlocked. Indicates that 
	   the caller is willing to wait forever.
	 */
#define IMG_SEMAPHORE_WAIT_INFINITE       ((IMG_UINT64)0xFFFFFFFFFFFFFFFFull)
#endif

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVCreateSemaphore)
#endif
	/**************************************************************************//*!
	   @Function       PVRSRVCreateSemaphore
	   @Description    Create a semaphore with an initial count
	   @Output         phSemaphore         on success, ptr to the handle of the new 
	   semaphore. Otherwise, zero.
	   @Input          iInitialCount       DOXYGEN_FIXME
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	static INLINE PVRSRV_ERROR PVRSRVCreateSemaphore(PVRSRV_SEMAPHORE_HANDLE
							 * phSemaphore,
							 IMG_INT iInitialCount)
	{
		PVR_UNREFERENCED_PARAMETER(iInitialCount);
		*phSemaphore = 0;
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVDestroySemaphore)
#endif
	/**************************************************************************//*!
	   @Function       PVRSRVDestroySemaphore
	   @Description    destroy the semaphore passed
	   @Input          hSemaphore          the semaphore to be destroyed
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	static INLINE PVRSRV_ERROR
	    PVRSRVDestroySemaphore(PVRSRV_SEMAPHORE_HANDLE hSemaphore) {
		PVR_UNREFERENCED_PARAMETER(hSemaphore);
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVWaitSemaphore)
#endif
	/**************************************************************************//*!
	   @Function       PVRSRVWaitSemaphore
	   @Description    wait on the specified semaphore
	   @Input          hSemaphore          the semephore on which to wait
	   @Input          ui64TimeoutMicroSeconds the time to wait for the semaphore to
	   become unlocked, if locked when the function
	   is called.
	   @Return         PVRSRV_ERROR:       PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	static INLINE PVRSRV_ERROR PVRSRVWaitSemaphore(PVRSRV_SEMAPHORE_HANDLE
						       hSemaphore,
						       IMG_UINT64
						       ui64TimeoutMicroSeconds)
	{
		PVR_UNREFERENCED_PARAMETER(hSemaphore);
		PVR_UNREFERENCED_PARAMETER(ui64TimeoutMicroSeconds);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPostSemaphore)
#endif
	/**************************************************************************//*!
	   @Function       PVRSRVPostSemaphore
	   @Description    DOXYGEN_FIXME
	   @Input          hSemaphore      DOXYGEN_FIXME
	   @Input          iPostCount      DOXYGEN_FIXME
	   @Return         None
     *//**************************************************************************/
	static INLINE IMG_VOID PVRSRVPostSemaphore(PVRSRV_SEMAPHORE_HANDLE
						   hSemaphore,
						   IMG_INT iPostCount) {
		PVR_UNREFERENCED_PARAMETER(hSemaphore);
		PVR_UNREFERENCED_PARAMETER(iPostCount);
	}

/* Non-exported APIs */
#if defined(DEBUG) && (defined(__linux__) || defined(_WIN32))
	/**************************************************************************//*!
	   @Function       PVRSRVAllocUserModeMemTracking
	   @Description    Wrapper function for malloc, used for memory-leak detection
	   @Input          ui32Size            number of bytes to be allocated
	   @Input          pszFileName         filename of the calling code
	   @Input          ui32LineNumber      line number of the calling code
	   @Return         IMG_PVOID           On success, a ptr to the newly-allocated
	   memory. Otherwise, NULL.
     *//**************************************************************************/
	IMG_IMPORT IMG_PVOID IMG_CALLCONV
	    PVRSRVAllocUserModeMemTracking(IMG_SIZE_T ui32Size,
					   IMG_CHAR * pszFileName,
					   IMG_UINT32 ui32LineNumber);

	/**************************************************************************//*!
	   @Function       PVRSRVCallocUserModeMemTracking
	   @Description    Wrapper function for calloc, used for memory-leak detection
	   @Input          ui32Size            number of bytes to be allocated
	   @Input          pszFileName         filename of the calling code
	   @Input          ui32LineNumber      line number of the calling code
	   @Return         IMG_PVOID           On success, a ptr to the newly-allocated
	   memory. Otherwise, NULL.
     *//**************************************************************************/
	IMG_IMPORT IMG_PVOID IMG_CALLCONV
	    PVRSRVCallocUserModeMemTracking(IMG_SIZE_T ui32Size,
					    IMG_CHAR * pszFileName,
					    IMG_UINT32 ui32LineNumber);

	/**************************************************************************//*!
	   @Function       PVRSRVFreeUserModeMemTracking
	   @Description    Wrapper for free - see PVRSRVAllocUserModeMemTracking
	   @Input          pvMem               pointer to the memory to be freed
	   @Return         None
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVFreeUserModeMemTracking(IMG_VOID
								       * pvMem);

	/**************************************************************************//*!
	   @Function       PVRSRVReallocUserModeMemTracking
	   @Description    Wrapper for realloc, used in memory-leak detection
	   @Input          pvMem           pointer to the existing memory block
	   @Input          ui32NewSize     the desired new size of the block
	   @Input          pszFileName     the filename of the calling code
	   @Input          ui32LineNumber  the line number of the calling code
	   @Return         IMG_PVOID       on success, a pointer to the memory block.
	   This may not necessarily be the same
	   location as the block was at before the
	   call. On failure, NULL is returned.
     *//**************************************************************************/
	IMG_IMPORT IMG_PVOID IMG_CALLCONV
	    PVRSRVReallocUserModeMemTracking(IMG_VOID * pvMem,
					     IMG_SIZE_T ui32NewSize,
					     IMG_CHAR * pszFileName,
					     IMG_UINT32 ui32LineNumber);
#endif				/* defined(DEBUG) && (defined(__linux__) || defined(_WIN32)) */

	/**************************************************************************//*!
	   @Function       PVRSRVDumpDebugInfo
	   @Description    Dump debug information to kernel log
	   @Input          psConnection    Services connection
	   @Return         IMG_VOID
     *//**************************************************************************/
	IMG_IMPORT IMG_VOID
	    PVRSRVDumpDebugInfo(const PVRSRV_CONNECTION * psConnection);

/******************************************************************************
 * PVR Event Object API(s)
 *****************************************************************************/
	/**************************************************************************//*!
	   @Function       PVRSRVEventObjectWait
	   @Description    Wait (block) on the OS-specific event object passed
	   @Input          psConnection    Services connection
	   @Input          hOSEvent        the event object to wait on
	   @Return         PVRSRV_ERROR:   PVRSRV_OK on success. Otherwise, a PVRSRV_
	   error code
     *//**************************************************************************/
	IMG_IMPORT PVRSRV_ERROR PVRSRVEventObjectWait(const PVRSRV_CONNECTION *
						      psConnection,
						      IMG_HANDLE hOSEvent);

	/**************************************************************************//*!
	   @Function               PVRSRVGetErrorString
	   @Description    Returns a text string relating to the PVRSRV_ERROR enum. Note:
	   must be kept in sync with servicesext.h
	   @Input          eError                  DOXYGEN_FIXME
	   @Return         IMG_CHAR *              DOXYGEN_FIXME
     *//**************************************************************************/
	IMG_IMPORT const IMG_CHAR *PVRSRVGetErrorString(PVRSRV_ERROR eError);

/*!
 Time wrapping macro
*/
#define TIME_NOT_PASSED_UINT32(a,b,c)		(((a) - (b)) < (c))

#if defined (__cplusplus)
}
#endif
#endif				/* __SERVICES_H__ */

/******************************************************************************
 End of file (services.h)
******************************************************************************/
