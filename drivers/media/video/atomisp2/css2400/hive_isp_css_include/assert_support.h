#ifndef __ASSERT_SUPPORT_H_INCLUDED__
#define __ASSERT_SUPPORT_H_INCLUDED__

#if defined(_MSC_VER)
#include "assert.h"
#define OP___assert(cnd) assert(cnd)
#elif defined(__HIVECC)
#define OP___assert(cnd) ((void)0)
#elif defined(__KERNEL__) /* a.o. Android builds */

#include "sh_css_debug.h"
#define __symbol2value( x ) #x
#define __symbol2string( x ) __symbol2value( x )
#define assert( expression )                                            \
	do {                                                            \
		if (!(expression))                                      \
			sh_css_dtrace(SH_DBG_ERROR, "%s",               \
				"Assertion failed: " #expression        \
				  ", file " __FILE__                    \
				  ", line " __symbol2string( __LINE__ ) \
				  ".\n" );                              \
	} while (0)

#define OP___assert(cnd) assert(cnd)

#elif defined(__FIST__)

#include <fist/fist.h>
#include <cyg/hal/plf_intr.h> /* for HAL_DELAY_US(us) */
#define _FIST
#define HRT_HW
#define __HOST__
#define assert(ignore)((void) 0)
#define OP___assert(cnd)

#elif defined(__GNUC__)
#include "assert.h"
#define OP___assert(cnd) assert(cnd)
#else /* default is for unknown environments */
#define assert(cnd) ((void)0)
#endif

#endif /* __ASSERT_SUPPORT_H_INCLUDED__ */
