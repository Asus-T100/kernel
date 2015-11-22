#include <linux/power_supply.h>
#include <linux/HWVersion.h>

#ifdef CONFIG_PF400CG
extern void led_set_for_driver(int brightness,
                unsigned long blink_delay_on,
                unsigned long blink_delay_off);

void set_off() { led_set_for_driver(0, 0, 0); }
void set_red() { led_set_for_driver(1, 0, 0); }
void set_green() { led_set_for_driver(2, 0, 0); }
void set_orange() { led_set_for_driver(3, 0, 0); }
void set_blink() { led_set_for_driver(1, 500, 3500); }
#else
void set_off() {}
void set_red() {}
void set_green() {}
void set_orange() {}
void set_blink() {}
#endif
EXPORT_SYMBOL(set_off);
EXPORT_SYMBOL(set_red);
EXPORT_SYMBOL(set_green);
EXPORT_SYMBOL(set_orange);
EXPORT_SYMBOL(set_blink);

