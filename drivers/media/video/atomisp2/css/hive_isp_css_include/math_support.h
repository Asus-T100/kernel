#ifndef __MATH_SUPPORT_H_INCLUDED__
#define __MATH_SUPPORT_H_INCLUDED__

#if defined(_MSC_VER)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define clamp(a, min_val, max_val) min(max(a, min_val), max_val)
#define bound(min_val, x, max_val) min(max(x, min_val), max_val)

#elif defined(__HIVECC)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define clamp(a, min_val, max_val) min(max(a, min_val), max_val)
/* the HIVE operator clip() is an assymetric bound() */
#define bound(min_val, x, max_val) min(max(x, min_val), max_val)

#elif defined(__KERNEL__)

/* already defined */

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

#else /* default is for the FIST environment */

/* already defined */

#endif

#endif /* __MATH_SUPPORT_H_INCLUDED__ */
