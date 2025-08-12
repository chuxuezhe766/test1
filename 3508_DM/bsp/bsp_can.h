#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"

typedef struct {
    uint16_t rotor_mechanical_angle; // 转子机械角度
    int16_t rotor_speed;             // 转子转速
    int16_t actual_torque_current;   // 实际转矩电流
    uint8_t motor_temperature;       // 电机温度
} MotorFeedbackData;

// 添加CAN消息结构体定义
typedef struct {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    uint8_t port;
} bsp_can_msg_t;

void bsp_can_init(void);
void can_filter_init(FDCAN_HandleTypeDef *hfdcan);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data);
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void get_djimotor_measure(MotorFeedbackData *motor, uint8_t *buf);

#endif 