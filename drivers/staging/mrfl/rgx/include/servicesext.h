									    /*************************************************************************//*!
									       @File
									       @Title          Services definitions required by external drivers
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Provides services data structures, defines and prototypes
									       required by external drivers
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined (__SERVICESEXT_H__)
#define __SERVICESEXT_H__

/* include/ */
#include "pvrsrv_error.h"
#include "img_types.h"
#include "imgpixfmts_km.h"
#include "pvrsrv_device_types.h"

#if defined(_WIN32)
#include "pvrsrvformats.h"
#endif

/*
 * Lock buffer read/write flags
 */
#define PVRSRV_LOCKFLG_READONLY     	(1)	/*!< The locking process will only read the locked surface */

/*!
 *****************************************************************************
 *	Services State
 *****************************************************************************/
typedef enum _PVRSRV_SERVICES_STATE_ {
	PVRSRV_SERVICES_STATE_OK = 0,
	PVRSRV_SERVICES_STATE_BAD,
} PVRSRV_SERVICES_STATE;

/*!
 *****************************************************************************
 *	States for power management
 *****************************************************************************/
typedef enum _PVRSRV_SYS_POWER_STATE_ {
	PVRSRV_SYS_POWER_STATE_Unspecified = -1,	/*!< Unspecified : Uninitialised */
	PVRSRV_SYS_POWER_STATE_D0 = 0,	/*!< On */
	PVRSRV_SYS_POWER_STATE_D1 = 1,	/*!< User Idle */
	PVRSRV_SYS_POWER_STATE_D2 = 2,	/*!< System Idle / sleep */
	PVRSRV_SYS_POWER_STATE_D3 = 3,	/*!< Suspend / Hibernate */
	PVRSRV_SYS_POWER_STATE_D4 = 4,	/*!< shutdown */

	PVRSRV_SYS_POWER_STATE_FORCE_I32 = 0x7fffffff	/*!< Force enum to be at least 32-bits wide */
} PVRSRV_SYS_POWER_STATE, *PPVRSRV_SYS_POWER_STATE;	/*!< Typedef for ptr to PVRSRV_SYS_POWER_STATE */

/*!
  Device Power State Enum
 */
typedef enum _PVRSRV_DEV_POWER_STATE_ {
	PVRSRV_DEV_POWER_STATE_DEFAULT = -1,	/*!< Default state for the device */
	PVRSRV_DEV_POWER_STATE_ON = 0,	/*!< Running */
	PVRSRV_DEV_POWER_STATE_IDLE = 1,	/*!< Powered but operation paused */
	PVRSRV_DEV_POWER_STATE_OFF = 2,	/*!< Unpowered */

	PVRSRV_DEV_POWER_STATE_FORCE_I32 = 0x7fffffff	/*!< Force enum to be at least 32-bits wide */
} PVRSRV_DEV_POWER_STATE, *PPVRSRV_DEV_POWER_STATE;	/*!< Typedef for ptr to PVRSRV_DEV_POWER_STATE *//* PRQA S 3205 */

/* Power transition handler prototypes */

/*! 
  Typedef for a pointer to a Function that will be called before a transition
  from one power state to another. See also PFN_POST_POWER.
 */
typedef PVRSRV_ERROR(*PFN_PRE_POWER) (IMG_HANDLE hDevHandle,
				      PVRSRV_DEV_POWER_STATE eNewPowerState,
				      PVRSRV_DEV_POWER_STATE
				      eCurrentPowerState);
/*! 
  Typedef for a pointer to a Function that will be called after a transition
  from one power state to another. See also PFN_PRE_POWER.
 */
typedef PVRSRV_ERROR(*PFN_POST_POWER) (IMG_HANDLE hDevHandle,
				       PVRSRV_DEV_POWER_STATE eNewPowerState,
				       PVRSRV_DEV_POWER_STATE
				       eCurrentPowerState);

/* Clock speed handler prototypes */

/*!
  Typedef for a pointer to a Function that will be caled before a transition
  from one clockspeed to another. See also PFN_POST_CLOCKSPEED_CHANGE.
 */
typedef PVRSRV_ERROR(*PFN_PRE_CLOCKSPEED_CHANGE) (IMG_HANDLE hDevHandle,
						  IMG_BOOL bIdleDevice,
						  PVRSRV_DEV_POWER_STATE
						  eCurrentPowerState);

/*!
  Typedef for a pointer to a Function that will be caled after a transition
  from one clockspeed to another. See also PFN_PRE_CLOCKSPEED_CHANGE.
 */
typedef PVRSRV_ERROR(*PFN_POST_CLOCKSPEED_CHANGE) (IMG_HANDLE hDevHandle,
						   IMG_BOOL bIdleDevice,
						   PVRSRV_DEV_POWER_STATE
						   eCurrentPowerState);

/*!
 *****************************************************************************
 * Enumeration of possible alpha types.
 *****************************************************************************/
typedef enum _PVRSRV_ALPHA_FORMAT_ {
	PVRSRV_ALPHA_FORMAT_UNKNOWN = 0x00000000,	/*!< Alpha Format: Unknown */
	PVRSRV_ALPHA_FORMAT_PRE = 0x00000001,	/*!< Alpha Format: Pre-Alpha */
	PVRSRV_ALPHA_FORMAT_NONPRE = 0x00000002,	/*!< Alpha Format: Non-Pre-Alpha */
	PVRSRV_ALPHA_FORMAT_MASK = 0x0000000F,	/*!< Alpha Format Mask */
} PVRSRV_ALPHA_FORMAT;

/*!
 *****************************************************************************
 * Enumeration of possible alpha types.
 *****************************************************************************/
typedef enum _PVRSRV_COLOURSPACE_FORMAT_ {
	PVRSRV_COLOURSPACE_FORMAT_UNKNOWN = 0x00000000,	/*!< Colourspace Format: Unknown */
	PVRSRV_COLOURSPACE_FORMAT_LINEAR = 0x00010000,	/*!< Colourspace Format: Linear */
	PVRSRV_COLOURSPACE_FORMAT_NONLINEAR = 0x00020000,	/*!< Colourspace Format: Non-Linear */
	PVRSRV_COLOURSPACE_FORMAT_MASK = 0x000F0000,	/*!< Colourspace Format Mask */
} PVRSRV_COLOURSPACE_FORMAT;

/*!
 * Drawable orientation (in degrees clockwise).
 */
typedef enum _PVRSRV_ROTATION_ {
	PVRSRV_ROTATE_0 = 0,	/*!< Rotate by 0 degres */
	PVRSRV_ROTATE_90 = 1,	/*!< Rotate by 90 degrees */
	PVRSRV_ROTATE_180 = 2,	/*!< Rotate by 180 degrees */
	PVRSRV_ROTATE_270 = 3,	/*!< Rotate by 270 degrees */
	PVRSRV_FLIP_Y
} PVRSRV_ROTATION;

/*!
 *****************************************************************************
 * Resource locking structure
 *****************************************************************************/
typedef struct PVRSRV_RESOURCE_TAG {
	volatile IMG_UINT32 ui32Lock;	/*!< the lock around the resource */
	IMG_UINT32 ui32ID;	/*!< the ID of the resource */
} PVRSRV_RESOURCE;
typedef PVRSRV_RESOURCE PVRSRV_RES_HANDLE;	/*!< Typedef: PVRSRV_RES_HANDLE is a PVRSRV_RESOURCE */

/*!
 *****************************************************************************
 * This structure is used for OS independent registry (profile) access
 *****************************************************************************/

typedef struct _PVRSRV_REGISTRY_INFO {
	IMG_UINT32 ui32DevCookie;
	IMG_PCHAR pszKey;
	IMG_PCHAR pszValue;
	IMG_PCHAR pszBuf;
	IMG_UINT32 ui32BufSize;
} PVRSRV_REGISTRY_INFO, *PPVRSRV_REGISTRY_INFO;

#define MAX_BUFFER_DEVICE_NAME_SIZE	(50)	/*!< Max size of the buffer device name */

/*! buffer information structure */
typedef struct BUFFER_INFO_TAG {
	IMG_UINT32 ui32BufferCount;	/*!< Number of supported buffers */
	IMG_UINT32 ui32BufferDeviceID;	/*!< DeviceID assigned by Services */
	IMG_PIXFMT eIMGPixFmt;	/*!< Pixel format of the buffer */
	IMG_UINT32 ui32ByteStride;	/*!< Byte stride of the buffer */
	IMG_UINT32 ui32Width;	/*!< Width of the buffer, in pixels */
	IMG_UINT32 ui32Height;	/*!< Height of the buffer, in pixels */
	IMG_UINT32 ui32Flags;	/*!< Flags */
	IMG_CHAR szDeviceName[MAX_BUFFER_DEVICE_NAME_SIZE];	/*!< Name of the device */
} BUFFER_INFO;

#endif				/* __SERVICESEXT_H__ */
/*****************************************************************************
 End of file (servicesext.h)
*****************************************************************************/
