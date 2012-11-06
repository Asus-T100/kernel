#ifndef __SYSTEM_LOCAL_H_INCLUDED__
#define __SYSTEM_LOCAL_H_INCLUDED__

#ifdef HRT_ISP_CSS_CUSTOM_HOST
#ifndef HRT_USE_VIR_ADDRS
#define HRT_USE_VIR_ADDRS
#endif
#include "hive_isp_css_custom_host_hrt.h"
#endif

#include "system_global.h"

#define HRT_ADDRESS_WIDTH	64		/* Surprise, this is a local property*/

#if !defined(__KERNEL__) || (1==1)
/* This interface is deprecated */
#include "hrt/hive_types.h"
#else  /* __KERNEL__ */
#include <stdint.h>

typedef uint64_t			hrt_address;
typedef uint32_t			hrt_vaddress;
typedef uint32_t			hrt_data;
#endif /* __KERNEL__ */

/*
 * Cell specific address maps
 */
#define GP_FIFO_BASE   ((hrt_address)0x0000000000090104)		/* This is NOT a base address */

/* DDR */
static const hrt_address DDR_BASE[N_DDR_ID] = {
	0x0000000120000000ULL};

/* ISP */
static const hrt_address ISP_CTRL_BASE[N_ISP_ID] = {
	0x0000000000020000ULL};

static const hrt_address ISP_DMEM_BASE[N_ISP_ID] = {
	0xffffffffffffffffULL};

static const hrt_address ISP_BAMEM_BASE[N_BAMEM_ID] = {
	0xffffffffffffffffULL};

static const hrt_address ISP_VAMEM_BASE[N_VAMEM_ID] = {
	0xffffffffffffffffULL,
	0xffffffffffffffffULL,
	0xffffffffffffffffULL};

static const hrt_address ISP_HMEM_BASE[N_HMEM_ID] = {
	0xffffffffffffffffULL};

/* SP */
static const hrt_address SP_CTRL_BASE[N_SP_ID] = {
	0x0000000000010000ULL};

static const hrt_address SP_DMEM_BASE[N_SP_ID] = {
	0x0000000000300000ULL};

/* MMU */
static const hrt_address MMU_BASE[N_MMU_ID] = {
	0x0000000000070000ULL};

/* DMA */
static const hrt_address DMA_BASE[N_DMA_ID] = {
	0x0000000000040000ULL};

/* IRQ */
static const hrt_address IRQ_BASE[N_IRQ_ID] = {
	0x0000000000000500ULL};
/*
	0x0000000000030A00ULL,
	0x000000000008C000ULL,
	0x0000000000090200ULL};
 */

/* GDC */
static const hrt_address GDC_BASE[N_GDC_ID] = {
	0x0000000000050000ULL,
	0x0000000000060000ULL};

/* FIFO_MONITOR (not a subset of GP_DEVICE) */
static const hrt_address FIFO_MONITOR_BASE[N_FIFO_MONITOR_ID] = {
	0x0000000000000000ULL};

/*
static const hrt_address GP_REGS_BASE[N_GP_REGS_ID] = {
	0x0000000000000000ULL};

static const hrt_address GP_DEVICE_BASE[N_GP_DEVICE_ID] = {
	0x0000000000090000ULL};
*/

/* GP_DEVICE (single base for all separate GP_REG instances) */
static const hrt_address GP_DEVICE_BASE[N_GP_DEVICE_ID] = {
	0x0000000000000000ULL};

/* GPIO */
static const hrt_address GPIO_BASE[N_GPIO_ID] = {
	0x00000400UL};

/* TIMED_CTRL */
static const hrt_address TIMED_CTRL_BASE[N_TIMED_CTRL_ID] = {
	0x00000100UL};


/* INPUT_FORMATTER */
static const hrt_address INPUT_FORMATTER_BASE[N_INPUT_FORMATTER_ID] = {
	0x0000000000030000ULL,
	0x0000000000030200ULL,
	0x0000000000030400ULL};
/*	0x0000000000030600ULL, */ /* memcpy() */

/* INPUT_SYSTEM */
static const hrt_address INPUT_SYSTEM_BASE[N_INPUT_SYSTEM_ID] = {
	0x0000000000080000ULL};
/*	0x0000000000081000ULL, */ /* capture A */
/*	0x0000000000082000ULL, */ /* capture B */
/*	0x0000000000083000ULL, */ /* capture C */
/*	0x0000000000084000ULL, */ /* Acquisition */
/*	0x0000000000085000ULL, */ /* DMA */
/*	0x0000000000089000ULL, */ /* ctrl */
/*	0x000000000008A000ULL, */ /* GP regs */
/*	0x000000000008B000ULL, */ /* FIFO */
/*	0x000000000008C000ULL, */ /* IRQ */

/* RX, the MIPI lane control regs start at offset 0 */
static const hrt_address RX_BASE[N_RX_ID] = {
	0x0000000000080100ULL};

#endif /* __SYSTEM_LOCAL_H_INCLUDED__ */
