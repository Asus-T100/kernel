									    /*************************************************************************//*!
									       @File
									       @Title          rgx kernel services structues/functions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX initialisation script definitions.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __RGXSCRIPT_H__
#define __RGXSCRIPT_H__

#if defined (__cplusplus)
extern "C" {
#endif

#define	RGX_MAX_INIT_COMMANDS	(256)
#define	RGX_MAX_DEINIT_COMMANDS	(16)

#define RGX_DBG_CMD_NAME_SIZE	(32)

	typedef enum _RGX_INIT_OPERATION {
		RGX_INIT_OP_ILLEGAL = 0,
		RGX_INIT_OP_WRITE_HW_REG,
		RGX_INIT_OP_POLL_HW_REG,
		RGX_INIT_OP_LOOP_POINT,
		RGX_INIT_OP_COND_BRANCH,
		RGX_INIT_OP_HALT,
		RGX_INIT_OP_DBG_READ32_HW_REG,
		RGX_INIT_OP_DBG_READ64_HW_REG,
#if defined(PDUMP)
		RGX_INIT_OP_PDUMP_HW_REG,
#endif
	} RGX_INIT_OPERATION;

	typedef union _RGX_INIT_COMMAND_ {
		RGX_INIT_OPERATION eOp;

		struct {
			RGX_INIT_OPERATION eOp;
			IMG_UINT32 ui32Offset;
			IMG_UINT32 ui32Value;
		} sWriteHWReg;

#if defined(PDUMP)
		struct {
			RGX_INIT_OPERATION eOp;
			IMG_UINT32 ui32Offset;
			IMG_UINT32 ui32Value;
		} sPDumpHWReg;
#endif

		struct {
			RGX_INIT_OPERATION eOp;
			IMG_UINT32 ui32Offset;
			IMG_UINT32 ui32Value;
			IMG_UINT32 ui32Mask;
		} sPollHWReg;

		struct {
			RGX_INIT_OPERATION eOp;
		} sLoopPoint;

		struct {
			RGX_INIT_OPERATION eOp;
			IMG_UINT32 ui32Offset;
			IMG_UINT32 ui32Value;
			IMG_UINT32 ui32Mask;

		} sConditionalBranchPoint;

		struct {
			RGX_INIT_OPERATION eOp;
			IMG_UINT32 ui32Offset;
			IMG_CHAR aszName[RGX_DBG_CMD_NAME_SIZE];
		} sDBGReadHWReg;

	} RGX_INIT_COMMAND;

	typedef struct _RGX_INIT_SCRIPTS_ {
		RGX_INIT_COMMAND asInitCommands[RGX_MAX_INIT_COMMANDS];
		RGX_INIT_COMMAND asDbgCommands[RGX_MAX_INIT_COMMANDS];
		RGX_INIT_COMMAND asDeinitCommands[RGX_MAX_DEINIT_COMMANDS];
	} RGX_SCRIPTS;

#if defined(__cplusplus)
}
#endif
#endif				/* __RGXSCRIPT_H__ */
/*****************************************************************************
 End of file (rgxscript.h)
*****************************************************************************/
