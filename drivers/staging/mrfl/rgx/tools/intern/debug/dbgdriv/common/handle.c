									    /*************************************************************************//*!
									       @File
									       @Title          Resource Handle Manager
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Provide resource handle management
									       @License        Strictly Confidential.
    *//**************************************************************************/
#include "img_defs.h"
#include "dbgdrvif.h"
#include "dbgdriv.h"

/* max number of streams held in SID info table */
#define MAX_SID_ENTRIES		8

typedef struct _SID_INFO {
	PDBG_STREAM psStream;
} SID_INFO, *PSID_INFO;

static SID_INFO gaSID_Xlat_Table[MAX_SID_ENTRIES];

IMG_SID PStream2SID(PDBG_STREAM psStream)
{
	if (psStream != (PDBG_STREAM) IMG_NULL) {
		IMG_INT32 iIdx;

		for (iIdx = 0; iIdx < MAX_SID_ENTRIES; iIdx++) {
			if (psStream == gaSID_Xlat_Table[iIdx].psStream) {
				/* idx is one based */
				return (IMG_SID) iIdx + 1;
			}
		}
	}

	return (IMG_SID) 0;
}

PDBG_STREAM SID2PStream(IMG_SID hStream)
{
	/* changed to zero based */
	IMG_INT32 iIdx = (IMG_INT32) hStream - 1;

	if (iIdx >= 0 && iIdx < MAX_SID_ENTRIES) {
		return gaSID_Xlat_Table[iIdx].psStream;
	} else {
		return (PDBG_STREAM) IMG_NULL;
	}
}

IMG_BOOL AddSIDEntry(PDBG_STREAM psStream)
{
	if (psStream != (PDBG_STREAM) IMG_NULL) {
		IMG_INT32 iIdx;

		for (iIdx = 0; iIdx < MAX_SID_ENTRIES; iIdx++) {
			if (psStream == gaSID_Xlat_Table[iIdx].psStream) {
				/* already created */
				return IMG_TRUE;
			}

			if (gaSID_Xlat_Table[iIdx].psStream ==
			    (PDBG_STREAM) IMG_NULL) {
				/* free entry */
				gaSID_Xlat_Table[iIdx].psStream = psStream;
				return IMG_TRUE;
			}
		}
	}

	return IMG_FALSE;
}

IMG_BOOL RemoveSIDEntry(PDBG_STREAM psStream)
{
	if (psStream != (PDBG_STREAM) IMG_NULL) {
		IMG_INT32 iIdx;

		for (iIdx = 0; iIdx < MAX_SID_ENTRIES; iIdx++) {
			if (psStream == gaSID_Xlat_Table[iIdx].psStream) {
				gaSID_Xlat_Table[iIdx].psStream =
				    (PDBG_STREAM) IMG_NULL;
				return IMG_TRUE;
			}
		}
	}

	return IMG_FALSE;
}

/******************************************************************************
 End of file (handle.c)
******************************************************************************/
