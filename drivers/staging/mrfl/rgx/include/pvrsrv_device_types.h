									    /*************************************************************************//*!
									       @File
									       @Title          PowerVR device type definitions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__PVRSRV_DEVICE_TYPES_H__)
#define __PVRSRV_DEVICE_TYPES_H__

#define PVRSRV_MAX_DEVICES		16	/*!< Largest supported number of devices on the system */

/*!
 ******************************************************************************
 * List of known device types.
 *****************************************************************************/
typedef enum _PVRSRV_DEVICE_TYPE_ {
	PVRSRV_DEVICE_TYPE_UNKNOWN = 0,	/*!< Unknown device type */
	PVRSRV_DEVICE_TYPE_MBX1 = 1,	/*!< MBX1 */
	PVRSRV_DEVICE_TYPE_MBX1_LITE = 2,	/*!< MBX1 Lite */
	PVRSRV_DEVICE_TYPE_M24VA = 3,	/*!< M24VA */
	PVRSRV_DEVICE_TYPE_MVDA2 = 4,	/*!< MVDA2 */
	PVRSRV_DEVICE_TYPE_MVED1 = 5,	/*!< MVED1 */
	PVRSRV_DEVICE_TYPE_MSVDX = 6,	/*!< MSVDX */
	PVRSRV_DEVICE_TYPE_SGX = 7,	/*!< SGX */
	PVRSRV_DEVICE_TYPE_VGX = 8,	/*!< VGX */
	PVRSRV_DEVICE_TYPE_EXT = 9,	/*!< 3rd party devices take ext type */
	PVRSRV_DEVICE_TYPE_RGX = 10,	/*!< RGX */
	PVRSRV_DEVICE_TYPE_TOPAZ = 11,	/*!< TOPAZ */

	PVRSRV_DEVICE_TYPE_LAST = 11,	/*!< Last device type */

	PVRSRV_DEVICE_TYPE_FORCE_I32 = 0x7fffffff	/*!< Force enum to be 32-bit width */
} PVRSRV_DEVICE_TYPE;

/*!
 *****************************************************************************
 * List of known device classes.
 *****************************************************************************/
typedef enum _PVRSRV_DEVICE_CLASS_ {
	PVRSRV_DEVICE_CLASS_3D = 0,	/*!< 3D Device Class */
	PVRSRV_DEVICE_CLASS_DISPLAY = 1,	/*!< Display Device Class */
	PVRSRV_DEVICE_CLASS_BUFFER = 2,	/*!< Buffer Class */
	PVRSRV_DEVICE_CLASS_VIDEO = 3,	/*!< Video Device Class */

	PVRSRV_DEVICE_CLASS_FORCE_I32 = 0x7fffffff	/* Force enum to be at least 32-bits wide */
} PVRSRV_DEVICE_CLASS;

/*!
 ******************************************************************************
 * Device identifier structure
 *****************************************************************************/
typedef struct _PVRSRV_DEVICE_IDENTIFIER_ {
	PVRSRV_DEVICE_TYPE eDeviceType;	/*!< Identifies the type of the device */
	PVRSRV_DEVICE_CLASS eDeviceClass;	/*!< Identifies more general class of device - display/3d/mpeg etc */
	IMG_UINT32 ui32DeviceIndex;	/*!< Index of the device within the system */
	IMG_CHAR *pszPDumpDevName;	/*!< Pdump memory bank name */
	IMG_CHAR *pszPDumpRegName;	/*!< Pdump register bank name */

} PVRSRV_DEVICE_IDENTIFIER;

#endif				/* __PVRSRV_DEVICE_TYPES_H__ */
