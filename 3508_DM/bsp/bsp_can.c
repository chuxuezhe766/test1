#include "bsp_can.h"

//接收电机的数据
MotorFeedbackData motor_name[7];

/*
 * 函数功能：初始化CAN通信接口
 * 参数说明：无
 * 返回值：无
 */
void bsp_can_init(void)
{
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	can_filter_init(&hfdcan1);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,ENABLE,ENABLE);
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef fdcan_filter;
    
    fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
    fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    fdcan_filter.FilterID1 = 0x00;                               
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(hfdcan,&fdcan_filter);                                                    //接收ID2
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data)
{	
	uint32_t send_mail_box;
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	pTxHeader.DataLength = FDCAN_DLC_BYTES_8;

    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_OFF;
    pTxHeader.FDFormat=FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
	{
		return 1;//发送
	}
	return 0;
}
/**
  * @brief  接收FDCAN数据帧并解析电机测量信息
  * @param  hfdcan: FDCAN句柄指针
  * @param  rec_id: 接收报文ID存储地址
  * @param  buf: 数据缓冲区指针
  * @retval 接收到的有效数据长度(字节)，返回0表示接收失败或数据无效
  */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		switch(pRxHeader.DataLength) 
		{
    		case FDCAN_DLC_BYTES_8:  len = 8;  break;
   		 	case FDCAN_DLC_BYTES_12: len = 12; break;
    		case FDCAN_DLC_BYTES_16: len = 16; break;
    		case FDCAN_DLC_BYTES_20: len = 20; break;
    		case FDCAN_DLC_BYTES_24: len = 24; break;
    		case FDCAN_DLC_BYTES_32: len = 32; break;
    		case FDCAN_DLC_BYTES_48: len = 48; break;
    		case FDCAN_DLC_BYTES_64: len = 64; break;
    		default: len = 0; // 无效 DLC 值时返回 0
		}
		switch (pRxHeader.Identifier)
		{
			case 0x201:
			{
				get_djimotor_measure(&motor_name[0], buf); 
        		break;
			}
			case 0x202:
			{
				get_djimotor_measure(&motor_name[1], buf); 
        		break;
			}
			case 0x203:
			{
				get_djimotor_measure(&motor_name[1], buf); 
        		break;
			}
			case 0x204:
			{
				get_djimotor_measure(&motor_name[1], buf); 
        		break;
			}
		}

		return len;//接收数据
	}
	return 0;	
}
/*
 * @brief FDCAN1接收回调函数，用于处理接收到的数据帧
 *
 * 该函数作为FDCAN1外设的接收中断回调函数，负责接收并存储
 * 接收到的CAN数据帧信息。函数内部通过调用通用接收函数
 * 完成实际的数据接收操作。
 *
 * @param 无
 * @return 无
 */
void fdcan1_rx_callback(void)
{
	uint8_t rx_data[8] = {0};
	uint16_t rec_id;
	fdcanx_receive(&hfdcan1, &rec_id, rx_data);
}
/**
  * @brief  FDCAN接收FIFO0中断回调函数
  * @param  hfdcan：指向FDCAN句柄的指针，用于指定操作的FDCAN实例
  * @param  RxFifo0ITs：接收FIFO0中断标志位集合
  * @retval 无返回值
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
}
/**
 * @brief 解析DJIMotor反馈数据到结构体
 * 
 * @param motor 用于存储解析后数据的MotorFeedbackData结构体指针（输出参数）
 * @param buf   包含原始反馈数据的缓冲区，长度至少为7字节（输入参数）
 * 
 * @return 无
 */
void get_djimotor_measure(MotorFeedbackData *motor, uint8_t *buf)
{
    // 直接解析数据到结构体
    motor->rotor_mechanical_angle = (buf[0] << 8) | buf[1];
    motor->rotor_speed = (int16_t)((buf[2] << 8) | buf[3]);
    motor->actual_torque_current = (int16_t)((buf[4] << 8) | buf[5]);
    motor->motor_temperature = buf[6];
}












