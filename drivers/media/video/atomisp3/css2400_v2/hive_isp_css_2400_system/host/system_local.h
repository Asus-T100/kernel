#ifndef __SYSTEM_LOCAL_H_INCLUDED__
#define __SYSTEM_LOCAL_H_INCLUDED__

#ifdef HRT_ISP_CSS_CUSTOM_HOST
#ifndef HRT_USE_VIR_ADDRS
#define HRT_USE_VIR_ADDRS
#endif
/* This interface is deprecated */
/*#include "hive_isp_css_custom_host_hrt.h"*/
#endif

#include "system_global.h"

#ifdef __FIST__
#define HRT_ADDRESS_WIDTH	32		/* Surprise, this is a local property and even differs per platform */
#else
#define HRT_ADDRESS_WIDTH	64		/* Surprise, this is a local property */
#endif

#if !defined(__KERNEL__) || (1==1)
/* This interface is deprecated */
#include "hrt/hive_types.h"
#else  /* __KERNEL__ */
#include <linux/types.h>

#if HRT_ADDRESS_WIDTH==64
typedef uint64_t			hrt_address;
#elif HRT_ADDRESS_WIDTH==32
typedef uint32_t			hrt_address;
#else
#error "system_local.h: HRT_ADDRESS_WIDTH must be one of {32,64}"
#endif

typedef uint32_t			hrt_vaddress;
typedef uint32_t			hrt_data;
#endif /* __KERNEL__ */

/*
 * Cell specific address maps
 */
#if HRT_ADDRESS_WIDTH==64

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
#if defined (IS_ISP_2400_MAMOIADA_SYSTEM) || defined (IS_ISP_2401_MAMOIADA_SYSTEM)
/*
 * MMU0_ID: The data MMU
 * MMU1_ID: The icache MMU
 */
static const hrt_address MMU_BASE[N_MMU_ID] = {
	0x0000000000070000ULL,
	0x00000000000A0000ULL};
#elif defined (IS_ISP_2400A0_MAMOIADA_SYSTEM)
static const hrt_address MMU_BASE[N_MMU_ID] = {
	0x0000000000070000ULL};
#else
#error "system_local.h: SYSTEM must be one of {2400, 2400A0, 2401 }"
#endif

/* DMA */
static const hrt_address DMA_BASE[N_DMA_ID] = {
	0x0000000000040000ULL};

/* IRQ */
static const hrt_address IRQ_BASE[N_IRQ_ID] = {
	0x0000000000000500ULL,
	0x0000000000030A00ULL,
	0x000000000008C000ULL,
	0x0000000000090200ULL};
/*
	0x0000000000000500ULL};
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
	0x0000000000000400ULL};

/* TIMED_CTRL */
static const hrt_address TIMED_CTRL_BASE[N_TIMED_CTRL_ID] = {
	0x0000000000000100ULL};


/* INPUT_FORMATTER */
static const hrt_address INPUT_FORMATTER_BASE[N_INPUT_FORMATTER_ID] = {
	0x0000000000030000ULL,
	0x0000000000030200ULL,
	0x0000000000030400ULL,
	0x0000000000030600ULL}; /* memcpy() */

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

#elif HRT_ADDRESS_WIDTH==32

#define GP_FIFO_BASE   ((hrt_address)0x00090104)		/* This is NOT a base address */

/* DDR : Attention, this value not defined in 32-bit */
static const hrt_address DDR_BASE[N_DDR_ID] = {
	0x00000000UL};

/* ISP */
static const hrt_address ISP_CTRL_BASE[N_ISP_ID] = {
	0x00020000UL};

static const hrt_address ISP_DMEM_BASE[N_ISP_ID] = {
	0xffffffffUL};

static const hrt_address ISP_BAMEM_BASE[N_BAMEM_ID] = {
	0xffffffffUL};

static const hrt_address ISP_VAMEM_BASE[N_VAMEM_ID] = {
	0xffffffffUL,
	0xffffffffUL,
	0xffffffffUL};

static const hrt_address ISP_HMEM_BASE[N_HMEM_ID] = {
	0xffffffffUL};

/* SP */
static const hrt_address SP_CTRL_BASE[N_SP_ID] = {
	0x00010000UL};

static const hrt_address SP_DMEM_BASE[N_SP_ID] = {
	0x00300000UL};

/* MMU */
#if defined (IS_ISP_2400_MAMOIADA_SYSTEM) || defined (IS_ISP_2401_MAMOIADA_SYSTEM)
/*
 * MMU0_ID: The data MMU
 * MMU1_ID: The icache MMU
 */
static const hrt_address MMU_BASE[N_MMU_ID] = {
	0x00070000UL,
	0x000A0000UL};
#elif defined (IS_ISP_2400A0_MAMOIADA_SYSTEM)
static const hrt_address MMU_BASE[N_MMU_ID] = {
	0x00070000UL};
#else
#error "system_local.h: SYSTEM must be one of {2400, 2400A0, 2401 }"
#endif

/* DMA */
static const hrt_address DMA_BASE[N_DMA_ID] = {
	0x00040000UL};

/* IRQ */
static const hrt_address IRQ_BASE[N_IRQ_ID] = {
	0x00000500UL,
	0x00030A00UL,
	0x0008C000UL,
	0x00090200UL};
/*
	0x00000500UL};
 */

/* GDC */
static const hrt_address GDC_BASE[N_GDC_ID] = {
	0x00050000UL,
	0x00060000UL};

/* FIFO_MONITOR (not a subset of GP_DEVICE) */
static const hrt_address FIFO_MONITOR_BASE[N_FIFO_MONITOR_ID] = {
	0x00000000UL};

/*
static const hrt_address GP_REGS_BASE[N_GP_REGS_ID] = {
	0x00000000UL};

static const hrt_address GP_DEVICE_BASE[N_GP_DEVICE_ID] = {
	0x00090000UL};
*/

/* GP_DEVICE (single base for all separate GP_REG instances) */
static const hrt_address GP_DEVICE_BASE[N_GP_DEVICE_ID] = {
	0x00000000UL};

/* GPIO */
static const hrt_address GPIO_BASE[N_GPIO_ID] = {
	0x00000400UL};

/* TIMED_CTRL */
static const hrt_address TIMED_CTRL_BASE[N_TIMED_CTRL_ID] = {
	0x00000100UL};


/* INPUT_FORMATTER */
static const hrt_address INPUT_FORMATTER_BASE[N_INPUT_FORMATTER_ID] = {
	0x00030000UL,
	0x00030200UL,
	0x00030400UL};
/*	0x00030600UL, */ /* memcpy() */

/* INPUT_SYSTEM */
static const hrt_address INPUT_SYSTEM_BASE[N_INPUT_SYSTEM_ID] = {
	0x00080000UL};
/*	0x00081000UL, */ /* capture A */
/*	0x00082000UL, */ /* capture B */
/*	0x00083000UL, */ /* capture C */
/*	0x00084000UL, */ /* Acquisition */
/*	0x00085000UL, */ /* DMA */
/*	0x00089000UL, */ /* ctrl */
/*	0x0008A000UL, */ /* GP regs */
/*	0x0008B000UL, */ /* FIFO */
/*	0x0008C000UL, */ /* IRQ */

/* RX, the MIPI lane control regs start at offset 0 */
static const hrt_address RX_BASE[N_RX_ID] = {
	0x00080100UL};

#else
#error "system_local.h: HRT_ADDRESS_WIDTH must be one of {32,64}"
#endif

#endif /* __SYSTEM_LOCAL_H_INCLUDED__ */
