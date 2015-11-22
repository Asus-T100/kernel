#ifndef _UTIL_H_
#define _UTIL_H_

#define TAG "batt"

#define DBG_ERR  0x1
#define DBG_INFO 0x2
#define DBG_DBG  0x3

#define DEBUG_LEVEL DBG_INFO

#if (DEBUG_LEVEL == DBG_ERR)
#define dbg_d(format, arg...)
#define dbg_i(format, arg...)
#define dub_i(format, arg...) printk("%s: " format, TAG , ## arg)
#elif (DEBUG_LEVEL == DBG_INFO)
#define dbg_d(format, arg...)
#define dbg_i(format, arg...) printk("%s: " format, TAG , ## arg)
#define dbg_e(format, arg...) printk("%s: " format, TAG , ## arg)
#elif (DEBUG_LEVEL == DBG_DBG)
#define dbg_d(format, arg...)			    		\
	do {				            			\
			printk("%s: " format,	\
				TAG , ## arg); 		        \
	} while (0)
#define dbg_i(format, arg...) printk("%s: " format, TAG , ## arg)
#define dbg_e(format, arg...) printk("%s: " format, TAG , ## arg)
#endif

#define BYTETOBINARYPATTERN "%d%d%d%d-%d%d%d%db"
#define BYTETOBINARY(byte) \
        (byte & 0x80 ? 1 : 0), \
        (byte & 0x40 ? 1 : 0), \
        (byte & 0x20 ? 1 : 0), \
        (byte & 0x10 ? 1 : 0), \
        (byte & 0x08 ? 1 : 0), \
        (byte & 0x04 ? 1 : 0), \
        (byte & 0x02 ? 1 : 0), \
        (byte & 0x01 ? 1 : 0)

void generate_key(char *buffer);

#endif
