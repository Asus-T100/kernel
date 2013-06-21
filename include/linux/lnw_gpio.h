#ifndef _H_GPIO_LANGWELL_H
#define _H_GPIO_LANGWELL_H
enum {
	LNW_GPIO = 0,
	LNW_ALT_1 = 1,
	LNW_ALT_2 = 2,
	LNW_ALT_3 = 3,
};

void lnw_gpio_set_alt(int gpio, int alt);
int gpio_get_alt(int gpio);
#endif
