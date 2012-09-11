#ifndef __ASSERT_SUPPORT_H_INCLUDED__
#define __ASSERT_SUPPORT_H_INCLUDED__

#if defined(_MSC_VER)
#include "assert.h"
#define OP___assert(cnd) assert(cnd)
#elif defined(__HIVECC)
#define OP___assert(cnd)
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

#elif defined(__GNUC__)
#include "assert.h"
#define OP___assert(cnd) assert(cnd)
#else /* default is for the FIST environment */
#define assert(cnd)
#endif

#endif /* __ASSERT_SUPPORT_H_INCLUDED__ */
