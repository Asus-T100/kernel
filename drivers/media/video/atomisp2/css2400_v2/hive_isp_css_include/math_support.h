/* Release Version: ci_master_byt_20130905_2200 */
/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2013 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __MATH_SUPPORT_H_INCLUDED__
#define __MATH_SUPPORT_H_INCLUDED__

/* ceil((real)a / b) */
#define ceil_div(a,b) (((a)+(b)-1)/(b))

#if defined(_MSC_VER)

/* MSC already provides min/max */
/*#define min(a, b) ((a) < (b) ? (a) : (b)) */
/*#define max(a, b) ((a) > (b) ? (a) : (b)) */
#define clamp(a, min_val, max_val) min(max(a, min_val), max_val)
#define bound(min_val, x, max_val) min(max(x, min_val), max_val)

#define _MAX(a, b)        ((a) > (b) ? (a) : (b))
#define _MIN(a, b)        ((a) < (b) ? (a) : (b))
#define _CEIL_MUL(a, b)   (CEIL_DIV(a, b) * (b))
#define _CEIL_DIV(a, b)   ((b) ? ((a)+(b)-1)/(b) : 0)
#define _CEIL_SHIFT(a, b) (((a)+(1<<(b))-1)>>(b))
#define _CEIL_SHIFT_MUL(a, b) (CEIL_SHIFT(a, b) << (b))
#define _CEIL_MUL2(a, b)  (((a)+(b)-1) & ~((b)-1))

#ifndef SH_CSS_CEIL_INLINE
#define MAX(a, b)	 	_MAX(a,b)
#define CEIL_MUL(a, b)		_CEIL_MUL(a, b) 
#define CEIL_DIV(a, b)   	_CEIL_DIV(a, b)
#define CEIL_SHIFT(a, b) 	_CEIL_SHIFT(a, b)
#define CEIL_SHIFT_MUL(a, b)  	_CEIL_SHIFT_MUL(a, b)
#define CEIL_MUL2(a, b)  	_CEIL_MUL2(a, b)

#else /* SH_CSS_CEIL_INLINE */

#define MAX(a, b)	 	_max(a,b)
#define CEIL_MUL(a, b)		_ceil_mul(a, b) 
#define CEIL_DIV(a, b)   	_ceil_div(a, b)
#define CEIL_SHIFT(a, b) 	_ceil_shift(a, b)
#define CEIL_SHIFT_MUL(a, b)  	_ceil_shift_mul(a, b)
#define CEIL_MUL2(a, b)  	_ceil_mul2(a, b)

static __inline unsigned _max(unsigned a, unsigned b)
{
	return _MAX(a,b);
}

static __inline unsigned _min(unsigned a, unsigned b)
{
	return _MIN(a,b);
}

static __inline unsigned _ceil_div(unsigned a, unsigned b)
{
	return _CEIL_DIV(a,b);
}

static inline unsigned _ceil_mul(unsigned a, unsigned b)
{
	return _CEIL_MUL(a,b);
}

static __inline unsigned _ceil_shift(unsigned a, unsigned b)
{
	return _CEIL_SHIFT(a,b);
}

static __inline unsigned _ceil_shift_mul(unsigned a, unsigned b)
{
	return _CEIL_SHIFT_MUL(a,b);
}

static __inline unsigned _ceil_mul2(unsigned a, unsigned b)
{
	return _CEIL_MUL2(a,b);
}

#endif /* SH_CSS_CEIL_INLINE */

#elif defined(__HIVECC)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define clamp(a, min_val, max_val) min(max(a, min_val), max_val)
/* the HIVE operator clip() is an assymetric bound() */
#define bound(min_val, x, max_val) min(max(x, min_val), max_val)

#define _MAX(a, b)        ((a) > (b) ? (a) : (b))
#define _MIN(a, b)        ((a) < (b) ? (a) : (b))
#define _CEIL_MUL(a, b)   (CEIL_DIV(a, b) * (b))
#define _CEIL_DIV(a, b)   ((b) ? ((a)+(b)-1)/(b) : 0)
#define _CEIL_SHIFT(a, b) (((a)+(1<<(b))-1)>>(b))
#define _CEIL_SHIFT_MUL(a, b) (CEIL_SHIFT(a, b) << (b))
#define _CEIL_MUL2(a, b)  (((a)+(b)-1) & ~((b)-1))

#ifndef SH_CSS_CEIL_INLINE
#define MAX(a, b)	 	_MAX(a,b)
#define CEIL_MUL(a, b)		_CEIL_MUL(a, b) 
#define CEIL_DIV(a, b)   	_CEIL_DIV(a, b)
#define CEIL_SHIFT(a, b) 	_CEIL_SHIFT(a, b)
#define CEIL_SHIFT_MUL(a, b)  	_CEIL_SHIFT_MUL(a, b)
#define CEIL_MUL2(a, b)  	_CEIL_MUL2(a, b)

#else /* SH_CSS_CEIL_INLINE */

#define MAX(a, b)	 	_max(a,b)
#define CEIL_MUL(a, b)		_ceil_mul(a, b) 
#define CEIL_DIV(a, b)   	_ceil_div(a, b)
#define CEIL_SHIFT(a, b) 	_ceil_shift(a, b)
#define CEIL_SHIFT_MUL(a, b)  	_ceil_shift_mul(a, b)
#define CEIL_MUL2(a, b)  	_ceil_mul2(a, b)

static inline unsigned _max(unsigned a, unsigned b)
{
	return _MAX(a,b);
}

static inline unsigned _min(unsigned a, unsigned b)
{
	return _MIN(a,b);
}

static inline unsigned _ceil_div(unsigned a, unsigned b)
{
	return _CEIL_DIV(a,b);
}

static inline unsigned _ceil_mul(unsigned a, unsigned b)
{
	return _CEIL_MUL(a,b);
}

static inline unsigned _ceil_shift(unsigned a, unsigned b)
{
	return _CEIL_SHIFT(a,b);
}

static inline unsigned _ceil_shift_mul(unsigned a, unsigned b)
{
	return _CEIL_SHIFT_MUL(a,b);
}

static inline unsigned _ceil_mul2(unsigned a, unsigned b)
{
	return _CEIL_MUL2(a,b);
}

#endif /* SH_CSS_CEIL_INLINE */

#elif defined(__KERNEL__)

#define _MAX(a, b)        ((a) > (b) ? (a) : (b))
#define _MIN(a, b)        ((a) < (b) ? (a) : (b))
#define _CEIL_MUL(a, b)   (CEIL_DIV(a, b) * (b))
#define _CEIL_DIV(a, b)   ((b) ? ((a)+(b)-1)/(b) : 0)
#define _CEIL_SHIFT(a, b) (((a)+(1<<(b))-1)>>(b))
#define _CEIL_SHIFT_MUL(a, b) (CEIL_SHIFT(a, b) << (b))
#define _CEIL_MUL2(a, b)  (((a)+(b)-1) & ~((b)-1))

#ifndef SH_CSS_CEIL_INLINE
#define MAX(a, b)	 	_MAX(a,b)
#define CEIL_MUL(a, b)		_CEIL_MUL(a, b) 
#define CEIL_DIV(a, b)   	_CEIL_DIV(a, b)
#define CEIL_SHIFT(a, b) 	_CEIL_SHIFT(a, b)
#define CEIL_SHIFT_MUL(a, b)  	_CEIL_SHIFT_MUL(a, b)
#define CEIL_MUL2(a, b)  	_CEIL_MUL2(a, b)

#else /* SH_CSS_CEIL_INLINE */

#define MAX(a, b)	 	_max(a,b)
#define CEIL_MUL(a, b)		_ceil_mul(a, b) 
#define CEIL_DIV(a, b)   	_ceil_div(a, b)
#define CEIL_SHIFT(a, b) 	_ceil_shift(a, b)
#define CEIL_SHIFT_MUL(a, b)  	_ceil_shift_mul(a, b)
#define CEIL_MUL2(a, b)  	_ceil_mul2(a, b)

static inline unsigned _max(unsigned a, unsigned b)
{
	return _MAX(a,b);
}

static inline unsigned _min(unsigned a, unsigned b)
{
	return _MIN(a,b);
}

static inline unsigned _ceil_div(unsigned a, unsigned b)
{
	return _CEIL_DIV(a,b);
}

static inline unsigned _ceil_mul(unsigned a, unsigned b)
{
	return _CEIL_MUL(a,b);
}

static inline unsigned _ceil_shift(unsigned a, unsigned b)
{
	return _CEIL_SHIFT(a,b);
}

static inline unsigned _ceil_shift_mul(unsigned a, unsigned b)
{
	return _CEIL_SHIFT_MUL(a,b);
}

static inline unsigned _ceil_mul2(unsigned a, unsigned b)
{
	return _CEIL_MUL2(a,b);
}

#endif /* SH_CSS_CEIL_INLINE */

#elif defined(__FIST__)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define clamp(a, min_val, max_val) min(max(a, min_val), max_val)
#define bound(min_val, x, max_val) min(max(x, min_val), max_val)

#define _MAX(a, b)        ((a) > (b) ? (a) : (b))
#define _MIN(a, b)        ((a) < (b) ? (a) : (b))
#define _CEIL_MUL(a, b)   (CEIL_DIV(a, b) * (b))
#define _CEIL_DIV(a, b)   ((b) ? ((a)+(b)-1)/(b) : 0)
#define _CEIL_SHIFT(a, b) (((a)+(1<<(b))-1)>>(b))
#define _CEIL_SHIFT_MUL(a, b) (CEIL_SHIFT(a, b) << (b))
#define _CEIL_MUL2(a, b)  (((a)+(b)-1) & ~((b)-1))

#ifndef SH_CSS_CEIL_INLINE
#define MAX(a, b)	 	_MAX(a,b)
#define CEIL_MUL(a, b)		_CEIL_MUL(a, b) 
#define CEIL_DIV(a, b)   	_CEIL_DIV(a, b)
#define CEIL_SHIFT(a, b) 	_CEIL_SHIFT(a, b)
#define CEIL_SHIFT_MUL(a, b)  	_CEIL_SHIFT_MUL(a, b)
#define CEIL_MUL2(a, b)  	_CEIL_MUL2(a, b)

#else /* SH_CSS_CEIL_INLINE */

#define MAX(a, b)	 	_max(a,b)
#define CEIL_MUL(a, b)		_ceil_mul(a, b) 
#define CEIL_DIV(a, b)   	_ceil_div(a, b)
#define CEIL_SHIFT(a, b) 	_ceil_shift(a, b)
#define CEIL_SHIFT_MUL(a, b)  	_ceil_shift_mul(a, b)
#define CEIL_MUL2(a, b)  	_ceil_mul2(a, b)

static inline unsigned _max(unsigned a, unsigned b)
{
	return _MAX(a,b);
}

static inline unsigned _min(unsigned a, unsigned b)
{
	return _MIN(a,b);
}

static inline unsigned _ceil_div(unsigned a, unsigned b)
{
	return _CEIL_DIV(a,b);
}

static inline unsigned _ceil_mul(unsigned a, unsigned b)
{
	return _CEIL_MUL(a,b);
}

static inline unsigned _ceil_shift(unsigned a, unsigned b)
{
	return _CEIL_SHIFT(a,b);
}

static inline unsigned _ceil_shift_mul(unsigned a, unsigned b)
{
	return _CEIL_SHIFT_MUL(a,b);
}

static inline unsigned _ceil_mul2(unsigned a, unsigned b)
{
	return _CEIL_MUL2(a,b);
}

#endif /* SH_CSS_CEIL_INLINE */

#elif defined(__GNUC__)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
/*
#define min(a, b) ({ \
	__typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a < _b ? _a : _b; }) 

#define max(a, b) ({ \
	__typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a > _b ? _a : _b; }) 
 */
#define clamp(a, min_val, max_val) min(max(a, min_val), max_val)
#define bound(min_val, x, max_val) min(max(x, min_val), max_val)

#define _MAX(a, b)        ((a) > (b) ? (a) : (b))
#define _MIN(a, b)        ((a) < (b) ? (a) : (b))
#define _CEIL_MUL(a, b)   (CEIL_DIV(a, b) * (b))
#define _CEIL_DIV(a, b)   ((b) ? ((a)+(b)-1)/(b) : 0)
#define _CEIL_SHIFT(a, b) (((a)+(1<<(b))-1)>>(b))
#define _CEIL_SHIFT_MUL(a, b) (CEIL_SHIFT(a, b) << (b))
#define _CEIL_MUL2(a, b)  (((a)+(b)-1) & ~((b)-1))

#ifndef SH_CSS_CEIL_INLINE
#define MAX(a, b)	 	_MAX(a,b)
#define CEIL_MUL(a, b)		_CEIL_MUL(a, b) 
#define CEIL_DIV(a, b)   	_CEIL_DIV(a, b)
#define CEIL_SHIFT(a, b) 	_CEIL_SHIFT(a, b)
#define CEIL_SHIFT_MUL(a, b)  	_CEIL_SHIFT_MUL(a, b)
#define CEIL_MUL2(a, b)  	_CEIL_MUL2(a, b)

#else /* SH_CSS_CEIL_INLINE */

#define MAX(a, b)	 	_max(a,b)
#define CEIL_MUL(a, b)		_ceil_mul(a, b) 
#define CEIL_DIV(a, b)   	_ceil_div(a, b)
#define CEIL_SHIFT(a, b) 	_ceil_shift(a, b)

#define CEIL_SHIFT_MUL(a, b)  	_ceil_shift_mul(a, b)
#define CEIL_MUL2(a, b)  	_ceil_mul2(a, b)

static inline unsigned _max(unsigned a, unsigned b)
{
	return _MAX(a,b);
}

static inline unsigned _min(unsigned a, unsigned b)
{
	return _MIN(a,b);
}

static inline unsigned _ceil_div(unsigned a, unsigned b)
{
	return _CEIL_DIV(a,b);
}

static inline unsigned _ceil_mul(unsigned a, unsigned b)
{
	return _CEIL_MUL(a,b);
}

static inline unsigned _ceil_shift(unsigned a, unsigned b)
{
	return _CEIL_SHIFT(a,b);
}

static inline unsigned _ceil_shift_mul(unsigned a, unsigned b)
{
	return _CEIL_SHIFT_MUL(a,b);
}

static inline unsigned _ceil_mul2(unsigned a, unsigned b)
{
	return _CEIL_MUL2(a,b);
}

#endif /* SH_CSS_CEIL_INLINE */

#else /* default is for an unknown environment */

/* already defined */

#endif

#endif /* __MATH_SUPPORT_H_INCLUDED__ */
