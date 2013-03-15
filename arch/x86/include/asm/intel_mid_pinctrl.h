#ifndef _ASM_X86_INTEL_MID_PINCTRL_H_
#define _ASM_X86_INTEL_MID_PINCTRL_H_

struct intel_mid_pinctrl_platform_data {
	char name[16];
	struct pinstruct_t *pin_t;
	int pin_num;
};

enum platform_descs {
	ctp_pin_desc,
	mrfl_pin_desc,
};

#endif


