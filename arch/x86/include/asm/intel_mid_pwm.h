#ifndef __INTEL_MID_PWM_H__
#define __INTEL_MID_PWM_H__

#define	MSIC_REG_PWM0CLKDIV1	0x061
#define	MSIC_REG_PWM0CLKDIV0	0x062
#define	MSIC_REG_PWM0DUTYCYCLE	0x067

#define MAX_DUTYCYCLE_PERCENTAGE	100

enum {
	PWM_LED = 0,
	PWM_VIBRATOR,
	PWM_LCD_BACKLIGHT,
	PWM_NUM,
};

struct intel_mid_pwm_platform_data {
	int reg_clkdiv0[PWM_NUM];
	int reg_clkdiv1[PWM_NUM];
	int reg_dutycyc[PWM_NUM];
};

int intel_mid_pwm(int id, int value);
#endif

