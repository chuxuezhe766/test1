#ifndef __DJI_Motor_H__
#define __DJI_Motor_H__

#include "stm32h7xx.h"
#include "bsp_can.h"

void spd_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id, int16_t vel);

#endif
