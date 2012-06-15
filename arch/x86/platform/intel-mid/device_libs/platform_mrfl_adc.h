/*
 * platform_mrfl_adc.h: Platform data for Merrifield GPADC driver
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_MRFL_ADC_H_
#define _PLATFORM_MRFL_ADC_H_

#define MRFL_ADC_DEV_NAME	"bcove_adc"

extern void __init *mrfl_adc_platform_data(void *info) __attribute__((weak));
#endif
