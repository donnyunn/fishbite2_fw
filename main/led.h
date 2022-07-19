#ifndef __LED_H
#define __LED_H

#include "main.h"

typedef enum {
    LED_STANDBY,
    LED_OFF,
    LED_RED_ON,
    LED_GREEN_ON,
    LED_ADVERTISING,
} led_indicate_t;

void led_init(void);
void led_red_on(void);
void led_grn_on(void);
void led_off(void);
void led_advertising(void);
void led_indicate_poweron(void);
void led_indicate_poweroff(void);
void led_indicate_pairing(bool onoff);

#endif /* __LED_H */