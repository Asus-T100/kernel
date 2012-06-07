#ifndef __INTEL_BASINCOVE_GPADC_H__
#define __INTEL_BASINCOVE_GPADC_H__

#define GPADC_VBAT	(1 << 0)
#define GPADC_BATID	(1 << 1)
#define GPADC_IBAT	(1 << 2)
#define GPADC_BATTEMP0	(1 << 3)
#define GPADC_BATTEMP1	(1 << 4)
#define GPADC_SYSTEMP0	(1 << 5)
#define GPADC_SYSTEMP1	(1 << 6)
#define GPADC_SYSTEMP2	(1 << 7)
#define GPADC_PMICTEMP	(1 << 8)
#define GPADC_CH_NUM	9

#define MBATTEMP	(1 << 2)
#define MSYSTEMP	(1 << 3)
#define MBATT		(1 << 4)
#define MVIBATT		(1 << 5)
#define MCCTICK		(1 << 7)

#define GPADC_RSL(channel, res)		\
	({				\
		int order = -1;		\
		int ch = channel;	\
		do {			\
			ch >>= 1;	\
			order++;	\
		} while (ch);		\
		res->data[order];	\
	})

struct intel_basincove_gpadc_platform_data {
	unsigned long intr;
};

struct gpadc_result {
	int data[GPADC_CH_NUM];
};

int intel_basincove_gpadc_sample(int ch, struct gpadc_result *res);
#endif
