#include "DJI_Motor.h"
#include "bsp_buzzer.h"
void spd_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t id, int16_t vel)
{
    uint8_t data[8];
    uint8_t result;

    data[0] = vel >> 8;
    data[1] = vel;
    data[2] = vel >> 8;
    data[3] = vel;
    data[4] = vel >> 8;
    data[5] = vel;
    data[6] = vel >> 8;
    data[7] = vel;

    fdcanx_send_data(hfdcan, 0x200, data);

}