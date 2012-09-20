									    /*************************************************************************//*!
									       @File
									       @Title          Functions for creating and reading proc filesystem entries.
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __SERVICES_PROC_H__
#define __SERVICES_PROC_H__

#include <asm/system.h>		// va_list etc
#include <linux/proc_fs.h>	// read_proc_t etc
#include <linux/seq_file.h>	// seq_file

#define END_OF_FILE (off_t) -1

typedef off_t(pvr_read_proc_t) (IMG_CHAR *, size_t, off_t);

#define PVR_PROC_SEQ_START_TOKEN (void*)1
typedef void *(pvr_next_proc_seq_t) (struct seq_file *, void *, loff_t);
typedef void *(pvr_off2element_proc_seq_t) (struct seq_file *, loff_t);
typedef void (pvr_show_proc_seq_t) (struct seq_file *, void *);
typedef void (pvr_startstop_proc_seq_t) (struct seq_file *, IMG_BOOL start);

typedef struct _PVR_PROC_SEQ_HANDLERS_ {
	pvr_next_proc_seq_t *next;
	pvr_show_proc_seq_t *show;
	pvr_off2element_proc_seq_t *off2element;
	pvr_startstop_proc_seq_t *startstop;
	IMG_VOID *data;
} PVR_PROC_SEQ_HANDLERS;

/** off2element function for elements with only ONE element (no header) */
void *ProcSeq1ElementOff2Element(struct seq_file *sfile, loff_t off);

/** off2element function for elements with only ONE element (+ header) */
void *ProcSeq1ElementHeaderOff2Element(struct seq_file *sfile, loff_t off);

off_t printAppend(IMG_CHAR * buffer, size_t size, off_t off,
		  const IMG_CHAR * format, ...)
    __attribute__ ((format(printf, 4, 5)));

IMG_INT CreateProcEntries(IMG_VOID);

IMG_INT CreateProcReadEntry(const IMG_CHAR * name, pvr_read_proc_t handler);

IMG_INT CreateProcEntry(const IMG_CHAR * name, read_proc_t rhandler,
			write_proc_t whandler, IMG_VOID * data);

IMG_VOID RemoveProcEntry(const IMG_CHAR * name);

IMG_VOID RemoveProcEntries(IMG_VOID);

struct proc_dir_entry *CreateProcReadEntrySeq(const IMG_CHAR * name,
					      IMG_VOID * data,
					      pvr_next_proc_seq_t next_handler,
					      pvr_show_proc_seq_t show_handler,
					      pvr_off2element_proc_seq_t
					      off2element_handler,
					      pvr_startstop_proc_seq_t
					      startstop_handler);

struct proc_dir_entry *CreateProcEntrySeq(const IMG_CHAR * name,
					  IMG_VOID * data,
					  pvr_next_proc_seq_t next_handler,
					  pvr_show_proc_seq_t show_handler,
					  pvr_off2element_proc_seq_t
					  off2element_handler,
					  pvr_startstop_proc_seq_t
					  startstop_handler,
					  write_proc_t whandler);

IMG_VOID RemoveProcEntrySeq(struct proc_dir_entry *proc_entry);

#endif
