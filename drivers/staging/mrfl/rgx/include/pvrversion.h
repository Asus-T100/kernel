									    /*************************************************************************//*!
									       @File
									       @Title          Version numbers and strings.
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Version numbers and strings for PVR Consumer services
									       components.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _PVRVERSION_H_
#define _PVRVERSION_H_

#define PVR_STR(X) #X
#define PVR_STR2(X) PVR_STR(X)

#define PVRVERSION_MAJ               1
#define PVRVERSION_MIN               0

#define PVRVERSION_FAMILY           "rogueddk"
#define PVRVERSION_BRANCHNAME       "MAIN"
#define PVRVERSION_BUILD             870514
#define PVRVERSION_BSCONTROL        "pc_i686_linux_nohw"

#define PVRVERSION_STRING           "pc_i686_linux_nohw rogueddk MAIN@" PVR_STR2(PVRVERSION_BUILD)
#define PVRVERSION_STRING_SHORT     "MAIN@" PVR_STR2(PVRVERSION_BUILD)

#define COPYRIGHT_TXT               "Copyright (c) Imagination Technologies Ltd. All Rights Reserved."

#define PVRVERSION_BUILD_HI          87
#define PVRVERSION_BUILD_LO          514
#define PVRVERSION_STRING_NUMERIC    PVR_STR2(PVRVERSION_MAJ) "." PVR_STR2(PVRVERSION_MIN) "." PVR_STR2(PVRVERSION_BUILD_HI) "." PVR_STR2(PVRVERSION_BUILD_LO)

#endif				/* _PVRVERSION_H_ */
