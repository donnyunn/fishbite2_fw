#ifndef __GATTS_H
#define __GATTS_H

#include "main.h"

/// A:
/// B:
/// C: led control service
#define PROFILE_NUM 3
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1
#define PROFILE_C_APP_ID 2

void ble_set_onSensitivityChange(void(*fnPtr));
void ble_set_onBrightnessChanged(void(*fnPtr));

void ble_sendIndication(int ID, uint8_t* val);
bool isBleConnected(void);
void gatts_init(void);

#endif /* __GATTS_H */