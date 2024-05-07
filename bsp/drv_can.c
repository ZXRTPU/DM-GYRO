#include "drv_can.h"
#include "remote_control.h"
#include "string.h"
#include "miniPC_process.h"
#include "VideoTransmitter.h"
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl[2];
extern chassis_t chassis;

//接收上C板的陀螺仪数据
int16_t UP_Roll;
int16_t UP_Pitch;
int16_t UP_Yaw;
float gimbal_Yaw;
float gimbal_Pitch;
extern uint8_t vision_is_tracking;
uint8_t friction_mode;

static uint8_t sbus_buf[18u]; // 遥控器接收的buffer
static uint8_t video_buf[12]; // 图传接收的buffer

uint16_t can_cnt_1 = 0;

float powerdata[4];
uint16_t pPowerdata[8];

uint8_t rx_data2[8];

uint16_t setpower = 5500;
int canerror = 0;

void CAN1_Init(void)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank = 0;                      // filter 0
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;                    // 标识符寄存器
  can_filter.FilterIdLow = 0;                     // 标识符寄存器
  can_filter.FilterMaskIdHigh = 0;                // 屏蔽寄存器
  can_filter.FilterMaskIdLow = 0;                 // 屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter);                         // init can filter
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能can的FIFO0中断
  HAL_CAN_Start(&hcan1);                                             // 启动can1
}

void CAN2_Init(void)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank = 14;                     // filter 14
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;                    // 标识符寄存器
  can_filter.FilterIdLow = 0;                     // 标识符寄存器
  can_filter.FilterMaskIdHigh = 0;                // 屏蔽寄存器
  can_filter.FilterMaskIdLow = 0;                 // 屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan2, &can_filter); // init can filter
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2); // 启动can2
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接受中断回调函数
{
  CAN_RxHeaderTypeDef rx_header;

  if (hcan->Instance == CAN2)
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		

    // 底盘电机信息接收
    if ((rx_header.StdId >= 0x201)     // 201-204
        && (rx_header.StdId <= 0x204)) // 判断标识符，标识符为0x200+ID
    {
      uint8_t index = rx_header.StdId - 0x201; // get motor index by can_id
      chassis.motor_info[index].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
      chassis.motor_info[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
      chassis.motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
      chassis.motor_info[index].temp = rx_data[6];
      if (index == 0)
      {
        can_cnt_1++;
      }
    }
		    if (rx_header.StdId == 0x211)
    {
      extern float powerdata[4];
      uint16_t *pPowerdata = (uint16_t *)rx_data;

      powerdata[0] = (float)pPowerdata[0] / 100.f; // 输入电压
      powerdata[1] = (float)pPowerdata[1] / 100.f; // 电容电压
      powerdata[2] = (float)pPowerdata[2] / 100.f; // 输入电流
      powerdata[3] = (float)pPowerdata[3] / 100.f; // P
  }
	
  if (hcan->Instance == CAN1)
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); 
		
		// 接收上C板传来的陀螺仪数据
		if (rx_header.StdId == 0x55) 
    { 
			memcpy(sbus_buf + 16, rx_data, 2);
      //sbus_to_rc(sbus_buf);
      UP_Yaw = (int16_t)((rx_data[2] << 8) | rx_data[3]) / 50.0f;
      UP_Pitch = (int16_t)((rx_data[4] << 8) | rx_data[5]) / 50.0f;
			gimbal_Yaw=UP_Yaw;//小数部分
		  gimbal_Pitch = UP_Pitch;
      vision_is_tracking = rx_data[6];
      friction_mode = rx_data[7];
    }
		
		if (rx_header.StdId == 0x36) // 接收上C传来的图传数据
    {
      memcpy(video_buf, rx_data, 8);
    }

    if (rx_header.StdId == 0x37) // 接收上C传来的图传数据
    {
      memcpy(video_buf + 8, rx_data, 4);
      VideoRead(video_buf);
      gimbal_Pitch = (int16_t)((rx_data[4] << 8) | rx_data[5]) / 50.0f;
      gimbal_Yaw = (int16_t)((rx_data[6] << 8) | rx_data[7]) / 50.0f;
    }

    if (rx_header.StdId == 0x38) // 接收上C传来的图传数据
    {
      vision_is_tracking = rx_data[0];
      friction_mode = rx_data[1];
    }
  }
}
}


// 向C板发送数据
void can_remote(uint8_t sbus_buf[], uint32_t can_send_id, uint32_t len) // 调用can来发送遥控器数据
{
  CAN_TxHeaderTypeDef tx_header;

  tx_header.StdId = can_send_id;                       // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE = CAN_ID_STD;                          // 标准帧
  tx_header.RTR = CAN_RTR_DATA;                        // 数据帧
  tx_header.DLC = len;                                   // 发送数据长度（字节）
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) // 等待邮箱空闲
  {
  }
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf, (uint32_t *)CAN_TX_MAILBOX0);
}

// 底盘电机控制
void set_motor_current_chassis(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];

  tx_header.StdId = (id_range == 0) ? (0x200) : (0x1ff); // 如果id_range==0则等于0x200,id_range==1则等于0x1ff（ID号）
  tx_header.IDE = CAN_ID_STD;                            // 标准帧
  tx_header.RTR = CAN_RTR_DATA;                          // 数据帧

  tx_header.DLC = 8; // 发送数据长度（字节）

  tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  tx_data[1] = (v1) & 0xff;
  tx_data[2] = (v2 >> 8) & 0xff;
  tx_data[3] = (v2) & 0xff;
  tx_data[4] = (v3 >> 8) & 0xff;
  tx_data[5] = (v3) & 0xff;
  tx_data[6] = (v4 >> 8) & 0xff;
  tx_data[7] = (v4) & 0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

// 雲臺電機控制
void set_motor_current_gimbal(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  // CAN_TxHeaderTypeDef tx_header;
  // uint8_t tx_data[8];

  // tx_header.StdId = (id_range == 0) ? (0x1ff) : (0x2ff); // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  // tx_header.IDE = CAN_ID_STD;                            // 标准帧
  // tx_header.RTR = CAN_RTR_DATA;                          // 数据帧

  // tx_header.DLC = 8; // 发送数据长度（字节）

  // tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  // tx_data[1] = (v1) & 0xff;
  // tx_data[2] = (v2 >> 8) & 0xff;
  // tx_data[3] = (v2) & 0xff;
  // tx_data[4] = (v3 >> 8) & 0xff;
  // tx_data[5] = (v3) & 0xff;
  // tx_data[6] = (v4 >> 8) & 0xff;
  // tx_data[7] = (v4) & 0xff;
  // HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

// 雲臺電機控制 -- CAN2
void set_motor_current_gimbal2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  // CAN_TxHeaderTypeDef tx_header;
  // uint8_t tx_data[8];

  // tx_header.StdId = (id_range == 0) ? (0x1ff) : (0x2ff); // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  // tx_header.IDE = CAN_ID_STD;                            // 标准帧
  // tx_header.RTR = CAN_RTR_DATA;                          // 数据帧

  // tx_header.DLC = 8; // 发送数据长度（字节）

  // tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  // tx_data[1] = (v1) & 0xff;
  // tx_data[2] = (v2 >> 8) & 0xff;
  // tx_data[3] = (v2) & 0xff;
  // tx_data[4] = (v3 >> 8) & 0xff;
  // tx_data[5] = (v3) & 0xff;
  // tx_data[6] = (v4 >> 8) & 0xff;
  // tx_data[7] = (v4) & 0xff;
  // HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

// 發射機構電機控制
void set_motor_current_shoot(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  // CAN_TxHeaderTypeDef tx_header;
  // uint8_t tx_data[8];

  // tx_header.StdId = (id_range == 0) ? (0x200) : (0x1ff); // 如果id_range==0则等于0x200,id_range==1则等于0x1ff（ID号）
  // tx_header.IDE = CAN_ID_STD;                            // 标准帧
  // tx_header.RTR = CAN_RTR_DATA;                          // 数据帧

  // tx_header.DLC = 8; // 发送数据长度（字节）

  // tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  // tx_data[1] = (v1) & 0xff;
  // tx_data[2] = (v2 >> 8) & 0xff;
  // tx_data[3] = (v2) & 0xff;
  // tx_data[4] = (v3 >> 8) & 0xff;
  // tx_data[5] = (v3) & 0xff;
  // tx_data[6] = (v4 >> 8) & 0xff;
  // tx_data[7] = (v4) & 0xff;
  // HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

// 试运行代码，一步解决所有电机控制
void set_curruent(uint32_t motor_range, CAN_HandleTypeDef can_id, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  // CAN_TxHeaderTypeDef tx_header;
  // uint8_t tx_data[8];

  // tx_header.StdId = motor_range; // 控制电机的ID号
  // tx_header.IDE = CAN_ID_STD;    // 标准帧
  // tx_header.RTR = CAN_RTR_DATA;  // 数据帧

  // tx_header.DLC = 8; // 发送数据长度（字节）

  // tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  // tx_data[1] = (v1) & 0xff;
  // tx_data[2] = (v2 >> 8) & 0xff;
  // tx_data[3] = (v2) & 0xff;
  // tx_data[4] = (v3 >> 8) & 0xff;
  // tx_data[5] = (v3) & 0xff;
  // tx_data[6] = (v4 >> 8) & 0xff;
  // tx_data[7] = (v4) & 0xff;
  // HAL_CAN_AddTxMessage(&can_id, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}