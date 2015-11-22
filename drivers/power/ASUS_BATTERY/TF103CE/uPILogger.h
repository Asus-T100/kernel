/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom Shen Tom_Shen@asus.com
 */

#ifndef __UPILOGGER_H__
#define __UPILOGGER_H__

#define BQ_DUMP(...) \
do { \
        local_len = sprintf(page, __VA_ARGS__); \
        len += local_len; \
        page += local_len; \
}while(0);


int ug31xx_register_upilogger_proc_fs(void);

#endif //#define __UPILOGGER_H__
