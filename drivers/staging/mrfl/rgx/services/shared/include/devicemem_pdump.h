									     /**************************************************************************//*!
									        @File           devicemem_pdump.h
									        @Title          Device Memory Management PDump internal
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Services internal interface to PDump device memory management
									        functions that are shared between client and server code.
    *//***************************************************************************/

#ifndef _DEVICEMEM_PDUMP_H_
#define _DEVICEMEM_PDUMP_H_

#include "devicemem.h"
#include "pdumpdefs.h"
#include "pdump.h"

#if defined(PDUMP)
/*
 * DevmemPDumpMem()
 *
 * takes a memory descriptor, offset, and size, and takes the current
 * contents of the memory at that location and writes it to the prm
 * pdump file, and emits a pdump LDB to load the data from that file.
 * The intention here is that the contents of the simulated buffer
 * upon pdump playback will be made to be the same as they are when
 * this command is run, enabling pdump of cases where the memory has
 * been modified externally, i.e. by the host cpu or by a third
 * party.
 */
extern IMG_VOID
DevmemPDumpLoadMem(DEVMEM_MEMDESC * psMemDesc,
		   IMG_DEVMEM_OFFSET_T uiOffset,
		   IMG_DEVMEM_SIZE_T uiSize, PDUMP_FLAGS_T uiPDumpFlags);

/*
 * DevmemPDumpMemValue()
 * 
 * As above but dumps the value at a dword-aligned address in plain
 * text to the pdump script2 file. Useful for patching a buffer at
 * pdump playback by simply editing the script output file.
 * 
 * (The same functionality can be achieved by the above function but
 *  the binary PARAM file must be patched in that case.)
 */
IMG_INTERNAL IMG_VOID
DevmemPDumpLoadMemValue(DEVMEM_MEMDESC * psMemDesc,
			IMG_DEVMEM_OFFSET_T uiOffset,
			IMG_UINT32 ui32Value, PDUMP_FLAGS_T uiPDumpFlags);

/*
 * DevmemPDumpPageCatBaseToSAddr()
 *
 * Returns the symbolic address of a piece of memory represented
 * by an offset into the mem descriptor.
 */
extern PVRSRV_ERROR
DevmemPDumpPageCatBaseToSAddr(DEVMEM_MEMDESC * psMemDesc,
			      IMG_DEVMEM_OFFSET_T * puiMemOffset,
			      IMG_CHAR * pszName, IMG_UINT32 ui32Size);

/*
 * DevmemPDumpSaveToFile()
 *
 * emits a pdump SAB to cause the current contents of the memory to be
 * written to the given file during playback
 */
extern IMG_VOID
DevmemPDumpSaveToFile(DEVMEM_MEMDESC * psMemDesc,
		      IMG_DEVMEM_OFFSET_T uiOffset,
		      IMG_DEVMEM_SIZE_T uiSize, const IMG_CHAR * pszFilename);

/*
 * DevmemPDumpSaveToFileVirtual()
 *
 * emits a pdump SAB, just like DevmemPDumpSaveToFile(), but uses the
 * virtual address and device MMU context to cause the pdump player to
 * traverse the MMU page tables itself.
 */
extern IMG_VOID
DevmemPDumpSaveToFileVirtual(DEVMEM_MEMDESC * psMemDesc,
			     IMG_DEVMEM_OFFSET_T uiOffset,
			     IMG_DEVMEM_SIZE_T uiSize,
			     const IMG_CHAR * pszFilename,
			     IMG_UINT32 ui32FileOffset,
			     IMG_UINT32 ui32PdumpFlags);

/*
 *
 * Devmem_PDumpDevmemPol32()
 *
 * writes a PDump 'POL' command to wait for a masked 32-bit memory
 * location to become the specified value
 */
extern PVRSRV_ERROR
DevmemPDumpDevmemPol32(const DEVMEM_MEMDESC * psMemDesc,
		       IMG_DEVMEM_OFFSET_T uiOffset,
		       IMG_UINT32 ui32Value,
		       IMG_UINT32 ui32Mask,
		       PDUMP_POLL_OPERATOR eOperator,
		       PDUMP_FLAGS_T ui32PDumpFlags);

/*
 * DevmemPDumpCBP()
 *
 * Polls for space in circular buffer. Reads the read offset
 * from memory and waits until there is enough space to write
 * the packet.
 *
 * hMemDesc      - MemDesc which contains the read offset
 * uiReadOffset  - Offset into MemDesc to the read offset
 * uiWriteOffset - Current write offset
 * uiPacketSize  - Size of packet to write
 * uiBufferSize  - Size of circular buffer
 */
extern PVRSRV_ERROR
DevmemPDumpCBP(const DEVMEM_MEMDESC * psMemDesc,
	       IMG_DEVMEM_OFFSET_T uiReadOffset,
	       IMG_DEVMEM_OFFSET_T uiWriteOffset,
	       IMG_DEVMEM_SIZE_T uiPacketSize, IMG_DEVMEM_SIZE_T uiBufferSize);

#else				/* PDUMP */

#ifdef INLINE_IS_PRAGMA
#pragma inline(DevmemPDumpLoadMem)
#endif
static INLINE IMG_VOID
DevmemPDumpLoadMem(DEVMEM_MEMDESC * psMemDesc,
		   IMG_DEVMEM_OFFSET_T uiOffset,
		   IMG_DEVMEM_SIZE_T uiSize, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psMemDesc);
	PVR_UNREFERENCED_PARAMETER(uiOffset);
	PVR_UNREFERENCED_PARAMETER(uiSize);
	PVR_UNREFERENCED_PARAMETER(uiPDumpFlags);
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(DevmemPDumpLoadMemValue)
#endif
static INLINE IMG_VOID
DevmemPDumpLoadMemValue(DEVMEM_MEMDESC * psMemDesc,
			IMG_DEVMEM_OFFSET_T uiOffset,
			IMG_UINT32 ui32Value, PDUMP_FLAGS_T uiPDumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psMemDesc);
	PVR_UNREFERENCED_PARAMETER(uiOffset);
	PVR_UNREFERENCED_PARAMETER(ui32Value);
	PVR_UNREFERENCED_PARAMETER(uiPDumpFlags);
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(DevmemPDumpPageCatBaseToSAddr)
#endif
static INLINE PVRSRV_ERROR
DevmemPDumpPageCatBaseToSAddr(DEVMEM_MEMDESC * psMemDesc,
			      IMG_DEVMEM_OFFSET_T * puiMemOffset,
			      IMG_CHAR * pszName, IMG_UINT32 ui32Size)
{
	PVR_UNREFERENCED_PARAMETER(psMemDesc);
	PVR_UNREFERENCED_PARAMETER(puiMemOffset);
	PVR_UNREFERENCED_PARAMETER(pszName);
	PVR_UNREFERENCED_PARAMETER(ui32Size);

	return PVRSRV_OK;
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(DevmemPDumpSaveToFile)
#endif
static INLINE IMG_VOID
DevmemPDumpSaveToFile(DEVMEM_MEMDESC * psMemDesc,
		      IMG_DEVMEM_OFFSET_T uiOffset,
		      IMG_DEVMEM_SIZE_T uiSize, const IMG_CHAR * pszFilename)
{
	PVR_UNREFERENCED_PARAMETER(psMemDesc);
	PVR_UNREFERENCED_PARAMETER(uiOffset);
	PVR_UNREFERENCED_PARAMETER(uiSize);
	PVR_UNREFERENCED_PARAMETER(pszFilename);
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(DevmemPDumpSaveToFileVirtual)
#endif
static INLINE IMG_VOID
DevmemPDumpSaveToFileVirtual(DEVMEM_MEMDESC * psMemDesc,
			     IMG_DEVMEM_OFFSET_T uiOffset,
			     IMG_DEVMEM_SIZE_T uiSize,
			     const IMG_CHAR * pszFilename,
			     IMG_UINT32 ui32FileOffset,
			     IMG_UINT32 ui32PdumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psMemDesc);
	PVR_UNREFERENCED_PARAMETER(uiOffset);
	PVR_UNREFERENCED_PARAMETER(uiSize);
	PVR_UNREFERENCED_PARAMETER(pszFilename);
	PVR_UNREFERENCED_PARAMETER(ui32FileOffset);
	PVR_UNREFERENCED_PARAMETER(ui32PdumpFlags);
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(DevmemPDumpDevmemPol32)
#endif
static INLINE PVRSRV_ERROR
DevmemPDumpDevmemPol32(const DEVMEM_MEMDESC * psMemDesc,
		       IMG_DEVMEM_OFFSET_T uiOffset,
		       IMG_UINT32 ui32Value,
		       IMG_UINT32 ui32Mask,
		       PDUMP_POLL_OPERATOR eOperator,
		       PDUMP_FLAGS_T ui32PDumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psMemDesc);
	PVR_UNREFERENCED_PARAMETER(uiOffset);
	PVR_UNREFERENCED_PARAMETER(ui32Value);
	PVR_UNREFERENCED_PARAMETER(ui32Mask);
	PVR_UNREFERENCED_PARAMETER(eOperator);
	PVR_UNREFERENCED_PARAMETER(ui32PDumpFlags);

	return PVRSRV_OK;
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(DevmemPDumpCBP)
#endif
static INLINE PVRSRV_ERROR
DevmemPDumpCBP(const DEVMEM_MEMDESC * psMemDesc,
	       IMG_DEVMEM_OFFSET_T uiReadOffset,
	       IMG_DEVMEM_OFFSET_T uiWriteOffset,
	       IMG_DEVMEM_SIZE_T uiPacketSize, IMG_DEVMEM_SIZE_T uiBufferSize)
{
	PVR_UNREFERENCED_PARAMETER(psMemDesc);
	PVR_UNREFERENCED_PARAMETER(uiReadOffset);
	PVR_UNREFERENCED_PARAMETER(uiWriteOffset);
	PVR_UNREFERENCED_PARAMETER(uiPacketSize);
	PVR_UNREFERENCED_PARAMETER(uiBufferSize);

	return PVRSRV_OK;
}
#endif				/* PDUMP */
#endif				/* _DEVICEMEM_PDUMP_H_ */
