/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @author modified by wexhi
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "drv_usart.h"
#include "task.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "daemon.h"
#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USART_Instance *referee_usart_instance; // 裁判系统串口实例
static Daemon_Instance *referee_daemon;		   // 裁判系统守护进程
static referee_info_t referee_info;			   // 裁判系统数据

referee_infantry_t referee_infantry; // 裁判系统英雄机器人数据

// 读取裁判系统数据到结构体
static void judge_read();

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度(byte),用于CR16校验
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				// 2个8位拼成16位int
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				switch (referee_info.CmdID)
				{
				case ID_game_state: // 0x0001
					memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
					break;
				case ID_game_result: // 0x0002
					memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
					break;
				case ID_game_robot_survivors: // 0x0003
					memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
					break;
				case ID_event_data: // 0x0101
					memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
					break;
				case ID_supply_projectile_action: // 0x0102
					memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
					break;
				case ID_game_warning: // 0x0104
					memcpy(&referee_info.GameWarning, (buff + DATA_Offset), LEN_game_warning);
					break;
				case ID_dart_info: // 0x0105
					memcpy(&referee_info.DartInfo, (buff + DATA_Offset), LEN_dart_info);
					break;
				case ID_game_robot_state: // 0x0201
					memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
					break;
				case ID_power_heat_data: // 0x0202
					memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
					break;
				case ID_game_robot_pos: // 0x0203
					memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
					break;
				case ID_buff_musk: // 0x0204
					memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
					break;
				case ID_aerial_robot_energy: // 0x0205
					memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
					break;
				case ID_robot_hurt: // 0x0206
					memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
					break;
				case ID_shoot_data: // 0x0207
					memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
					break;
				case ID_projectile_allowance: // 0x0208
					memcpy(&referee_info.ProjectileAllowance, (buff + DATA_Offset), LEN_projectile_allowance);
					break;
				case ID_rfid_status: // 0x0209
					memcpy(&referee_info.RFIDStatus, (buff + DATA_Offset), LEN_rfid_status);
					break;
				// @todo 本demo未添加哨兵、飞镖、雷达的裁判系统数据解析
				case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
					memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
					break;
				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	JudgeReadData(referee_usart_instance->recv_buff);
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	if (!referee_info.init_flag)
		referee_info.init_flag = 1;
	else
		return &referee_info;

	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}

// 读取裁判系统数据到结构体
static void judge_read()
{
	referee_infantry.robot_level = referee_info.GameRobotState.robot_level;
	referee_infantry.chassis_power_limit = referee_info.GameRobotState.chassis_power_limit;
	referee_infantry.chassis_power = referee_info.PowerHeatData.chassis_power;
	referee_infantry.buffer_energy = referee_info.PowerHeatData.buffer_energy;
}