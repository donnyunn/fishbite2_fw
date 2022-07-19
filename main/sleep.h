#ifndef __SLEEP_H
#define __SLEEP_H

#include "main.h"

#define WAKEUP_PIN CONFIG_GPIO_WAKEUP_PIN

void sleep_init(void);
void lightSleep(void);
void deepSleep(void);
bool isPushedPwrbtn(void);

#endif /* __SLEEP_H */