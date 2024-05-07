#include "Chassis_task.h"
#include "cmsis_os.h"
#include "exchange.h"
#include "drv_can.h"
#include "usart.h"
#include "pid.h"
#include "math.h"
#include "arm_math.h"
#include "struct_typedef.h"
#include "rm_referee.h"
#include "referee_protocol.h"
#include "remote_control.h"
#include "robot_def.h"
#include "VideoTransmitter.h"

#define KEY_START_OFFSET 10
#define KEY_STOP_OFFSET 20
#define CHASSIS_WZ_MAX 4000
#define FOLLOW_WEIGHT 160

#define RC_MAX 660
#define RC_MIN -660
#define motor_max 4000
#define motor_min -4000
#define angle_valve 5
#define angle_weight 55
#define CHASSIS_MAX_SPEED 8000

#define CHASSIS_SPEED_MAX_1 5000
#define CHASSIS_SPEED_MAX_2 5500
#define CHASSIS_SPEED_MAX_3 6000
#define CHASSIS_SPEED_MAX_4 6500
#define CHASSIS_SPEED_MAX_5 7500
#define CHASSIS_SPEED_MAX_6 8500
#define CHASSIS_SPEED_MAX_7 9000
#define CHASSIS_SPEED_MAX_8 10000
#define CHASSIS_SPEED_MAX_9 11000
#define CHASSIS_SPEED_MAX_10 12000
/// @brief ///这个是用来调试功率限制的//////////////////////////////////
int16_t chassis_speed_max = 0;//暂时没用到
/// @brief //////
int16_t chassis_wz_max = 4000;
// #define chassis_speed_max 2000
#define CHASSIS_WZ_MAX_1 4000 // 低速，g键触发
#define CHASSIS_WZ_MAX_2 6000 // 高速，b键触发

chassis_t chassis;
//extern double yaw12;//DM陀螺仪
//extern float Yaw1;//外接陀螺仪
extern float gimbal_Yaw;

//yaw校准参数
int yaw_correction_flag = 1; // yaw值校正标志
static uint8_t cycle = 0; // do while循环一次的条件

motor_info_t motor_info_chassis[10]; // 电机信息结构体

extern referee_infantry_t referee_infantry;
extern int superop; // 超电
extern uint8_t rx_buffer_c[49];
extern uint8_t rx_buffer_d[128];

int8_t chassis_mode;
uint8_t supercap_flag = 0;   
extern Video_ctrl_t video_ctrl[2];
extern RC_ctrl_t rc_ctrl[2]; // 遥控器信息结构体
extern float powerdata[4];
extern uint16_t shift_flag;
static int16_t key_x_fast, key_y_fast, key_x_slow, key_y_slow, key_Wz_acw, key_Wz_cw;
// uint8_t rc[18];
static referee_info_t *referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data;     

//功率限制
float Watch_Power_Max;                                                 // 限制值
float Watch_Power;                                                     // 实时功率
uint16_t Watch_Buffer;                                                    // 缓冲能量值
double Chassis_pidout;                                                 // 输出值
double Chassis_pidout_target;                                          // 目标值
static double Scaling1 = 0, Scaling2 = 0, Scaling3 = 0, Scaling4 = 0;  // 比例
float Klimit = 1;                                                      // 限制值
float Plimit = 0;                                                      // 约束比例
float Chassis_pidout_max;                                              // 输出值限制


static void Chassis_Init();
static void Chassis_loop_Init();
static void level_judge();
// 读取键鼠数据控制底盘模式
static void read_keyboard(void);
static void key_control();
//底盘yaw值校正
static void yaw_correct();
//static void manual_yaw_correct();
// 模式选择
static void mode_chooce();
// 遥控器控制底盘电机
static void chassis_rc_mode();
// 小陀螺模式
static void chassis_gyro_mode();
//底盘跟随云台
static void chassis_follow_mode();
//底盘停止模式
static void chassis_stop_mode();
// 运动解算
static void chassis_motol_speed_calculate();
//底盘功率限制
static void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed);
static void Chassis_Power_Limit(double Chassis_pidout_target_limit);   
// 电机电流控制
static void chassis_current_give();
//过零处理
static void detel_calc(fp32 *angle);

void Chassis_task(void const * argument)
{
  Chassis_Init();

 for (;;) // 底盘运动任务
  {
    Chassis_loop_Init();

		//陀螺仪校准
    yaw_correct(); 

    // 选择底盘运动模式
    mode_chooce();

    // 电机速度解算
    chassis_motol_speed_calculate();

    // 电机功率限制
    // Motor_Speed_limiting(chassis.speed_target,motor_max);

    chassis_current_give();

    osDelay(1);
  }
}

static void Chassis_Init()
{
  chassis.pid_parameter[0] = 30, chassis.pid_parameter[1] = 0.5, chassis.pid_parameter[2] = 2;
    // referee_data = RefereeInit(&huart5); // 裁判系统初始化


  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&chassis.pid[i], chassis.pid_parameter, 16384, 16384); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
  }
  // pid_init(&supercap_pid, superpid, 3000, 3000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384

  chassis.Vx = 0, chassis.Vy = 0, chassis.Wz = 0;
}

static void Chassis_loop_Init()
{
  chassis.Vx = 0;
  chassis.Vy = 0;
  chassis.Wz = 0;
}

static void mode_chooce()
{
    // 遥控器链路
    if (rc_ctrl[TEMP].rc.switch_left)
    {
      // 右拨杆下，遥控操作
      if (switch_is_down(rc_ctrl[TEMP].rc.switch_right))
      {
        chassis_follow_mode();
      }

      // 右拨杆中，键鼠操作
      else if (switch_is_mid(rc_ctrl[TEMP].rc.switch_right))
      {
        // 底盘模式读取
        read_keyboard();
        key_control();

        // 底盘跟随云台模式，r键触发
        if (chassis_mode == 1)
        {
          chassis_follow_mode();
        }

        // 正常运动模式，f键触发
        else if (chassis_mode == 2)
        {
          //manual_yaw_correct(); // 手动校正yaw值，头对正，按下V键
          chassis_rc_mode();
        }

        else
        {
          chassis_stop_mode();
        }
      }

      // 停止模式
      else
      {
        chassis_stop_mode();
      }
    }

    // 图传链路
    else
    {
      // 底盘模式读取
      read_keyboard();
      key_control();

      switch (ui_data.chassis_mode)
      {
      // 底盘跟随云台模式，r键触发
      case CHASSIS_FOLLOW_GIMBAL_YAW:
        chassis_follow_mode();
        break;

      // 正常运动模式，f键触发
      case CHASSIS_NO_FOLLOW:
        //manual_yaw_correct(); // 手动校正yaw值，头对正，按下V键
        chassis_rc_mode();
        break;
			
			case CHASSIS_GYRO:
        chassis_gyro_mode();
        break;

      // 停止模式
      case CHASSIS_ZERO_FORCE:
        chassis_stop_mode();
        break;

      default:
        chassis_stop_mode();
        break;
      }
    }
}

static void read_keyboard()
{
  // 遥控器链路
  if (rc_ctrl[TEMP].rc.switch_left)
  {
    // F键控制底盘模式
    if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_F] % 3 == 0)
      ui_data.chassis_mode = CHASSIS_NO_FOLLOW; // normal
    else if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_F] % 3 == 1)
      ui_data.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // follow
		else if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_F] % 3 == 2)
			ui_data.chassis_mode = CHASSIS_GYRO;
    else
      ui_data.chassis_mode = CHASSIS_ZERO_FORCE; // stop
    
    // C键控制超级电容
    if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_C] % 2 == 1)
    {
      supercap_flag = 1;
      ui_data.supcap_mode = SUPCAP_ON;
    }
    else
    {
      supercap_flag = 0;
      ui_data.supcap_mode = SUPCAP_OFF;
    }

    // Q键切换发射模式，单发和爆破
    if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_Q] % 2 == 1)
      ui_data.shoot_mode = SHOOT_BUFF;
    else
      ui_data.shoot_mode = SHOOT_NORMAL;
		
//    // R键控制底盘小陀螺速度
//    if (rc_ctrl[TEMP].key_count[KEY_PRESS][Key_R] % 2 == 1)
//    {
//      chassis_wz_max = CHASSIS_WZ_MAX_2; // 因为默认为1，这里保证第一次按下就能切换
//      ui_data.top_mode = TOP_HIGH;
//    }
//    else
//    {
//      chassis_wz_max = CHASSIS_WZ_MAX_1;
//      ui_data.top_mode = TOP_LOW;
//    }

//    // E键切换摩擦轮速度，012分别为low，normal，high
//    switch (friction_mode)
//    {
//    case 0:
//      ui_data.friction_mode = FRICTION_LOW;
//      break;
//    case 1:
//      ui_data.friction_mode = FRICTION_NORMAL;
//      break;
//    case 2:
//      ui_data.friction_mode = FRICTION_HIGH;
//      break;
//    }
  }
	
	// 图传链路
  else
  {
    // F键控制底盘模式
    if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_F] % 3 == 1)
      ui_data.chassis_mode = CHASSIS_NO_FOLLOW; // normal
    else if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_F] % 3 == 2)
      ui_data.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // follow
    else
      ui_data.chassis_mode = CHASSIS_ZERO_FORCE; // stop

    // C键控制超级电容
    if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_C] % 2 == 1)
    {
      supercap_flag = 1;
      ui_data.supcap_mode = SUPCAP_ON;
    }
    else
    {
      supercap_flag = 0;
      ui_data.supcap_mode = SUPCAP_OFF;
    }

    // Q键切换发射模式，单发和爆破
    if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_Q] % 2 == 1)
      ui_data.shoot_mode = SHOOT_BUFF;
    else
      ui_data.shoot_mode = SHOOT_NORMAL;
		
//		// R键控制底盘小陀螺速度
//    if (video_ctrl[TEMP].key_count[KEY_PRESS][Key_R] % 2 == 1)
//    {
//      chassis_wz_max = CHASSIS_WZ_MAX_2; // 因为默认为1，这里保证第一次按下就能切换
//      ui_data.top_mode = TOP_HIGH;
//    }
//    else
//    {
//      chassis_wz_max = CHASSIS_WZ_MAX_1;
//      ui_data.top_mode = TOP_LOW;
//    }

//    // E键切换摩擦轮速度，012分别为low，normal，high
//    switch (friction_mode)
//    {
//    case 0:
//      ui_data.friction_mode = FRICTION_LOW;
//      break;
//    case 1:
//      ui_data.friction_mode = FRICTION_NORMAL;
//      break;
//    case 2:
//      ui_data.friction_mode = FRICTION_HIGH;
//      break;
//    }
//  }
   }
}

static void key_control()
{
	// 遥控器链路
  if (rc_ctrl[TEMP].rc.switch_left)
  {
    if (rc_ctrl[TEMP].key[KEY_PRESS].d)
      key_x_fast += KEY_START_OFFSET;
    else
      key_x_fast -= KEY_STOP_OFFSET;

    if (rc_ctrl[TEMP].key[KEY_PRESS].a)
      key_x_slow += KEY_START_OFFSET;
    else
      key_x_slow -= KEY_STOP_OFFSET;

    if (rc_ctrl[TEMP].key[KEY_PRESS].w)
      key_y_fast += KEY_START_OFFSET;
    else
      key_y_fast -= KEY_STOP_OFFSET;

    if (rc_ctrl[TEMP].key[KEY_PRESS].s)
      key_y_slow += KEY_START_OFFSET;
    else
      key_y_slow -= KEY_STOP_OFFSET;

    // 正转
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift)
      key_Wz_acw += KEY_START_OFFSET;
    else
      key_Wz_acw -= KEY_STOP_OFFSET;

    // 反转
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl)
      key_Wz_cw -= KEY_START_OFFSET;
    else
      key_Wz_cw += KEY_STOP_OFFSET;
  }

  // 图传链路
  else
  {
    if (video_ctrl[TEMP].key[KEY_PRESS].d)
      key_x_fast += KEY_START_OFFSET;
    else
      key_x_fast -= KEY_STOP_OFFSET;

    if (video_ctrl[TEMP].key[KEY_PRESS].a)
      key_x_slow += KEY_START_OFFSET;
    else
      key_x_slow -= KEY_STOP_OFFSET;

    if (video_ctrl[TEMP].key[KEY_PRESS].w)
      key_y_fast += KEY_START_OFFSET;
    else
      key_y_fast -= KEY_STOP_OFFSET;

    if (video_ctrl[TEMP].key[KEY_PRESS].s)
      key_y_slow += KEY_START_OFFSET;
    else
      key_y_slow -= KEY_STOP_OFFSET;

    // 正转
    if (video_ctrl[TEMP].key[KEY_PRESS].shift)
      key_Wz_acw += KEY_START_OFFSET;
    else
      key_Wz_acw -= KEY_STOP_OFFSET;

    // 反转
    if (video_ctrl[TEMP].key[KEY_PRESS].ctrl)
      key_Wz_cw -= KEY_START_OFFSET;
    else
      key_Wz_cw += KEY_STOP_OFFSET;
  }

  if (key_x_fast > chassis_speed_max)
    key_x_fast = chassis_speed_max;
  if (key_x_fast < 0)
    key_x_fast = 0;
  if (key_x_slow > chassis_speed_max)
    key_x_slow = chassis_speed_max;
  if (key_x_slow < 0)
    key_x_slow = 0;
  if (key_y_fast > chassis_speed_max)
    key_y_fast = chassis_speed_max;
  if (key_y_fast < 0)
    key_y_fast = 0;
  if (key_y_slow > chassis_speed_max)
    key_y_slow = chassis_speed_max;
  if (key_y_slow < 0)
    key_y_slow = 0;
  if (key_Wz_acw > chassis_wz_max)
    key_Wz_acw = chassis_wz_max;
  if (key_Wz_acw < 0)
    key_Wz_acw = 0;
  if (key_Wz_cw < -chassis_wz_max)
    key_Wz_cw = -chassis_wz_max;
  if (key_Wz_cw > 0)
    key_Wz_cw = 0;
}

// 线性映射函数
static int16_t map_range(int value, int from_min, int from_max, int to_min, int to_max)
{
  // 首先将输入值映射到[0, 1]的范围
  double normalized_value = (value * 1.0 - from_min * 1.0) / (from_max * 1.0 - from_min * 1.0);

  // 然后将[0, 1]的范围映射到[to_min, to_max]的范围
  int16_t mapped_value = (int16_t)(normalized_value * (to_max - to_min) + to_min);

  return mapped_value;
}

//自由控制模式
static void chassis_rc_mode(void)
{
  // 从遥控器获取控制输入
  chassis.Vx = rc_ctrl[TEMP].rc.rocker_l_ / 660.0f * chassis_speed_max + key_x_fast - key_x_slow;
  chassis.Vy = rc_ctrl[TEMP].rc.rocker_l1/ 660.0f * chassis_speed_max + key_y_fast - key_y_slow; 
  chassis.Wz = rc_ctrl[TEMP].rc.dial / 660.0f * chassis_wz_max + key_Wz_acw + key_Wz_cw;

//  /*************记得加上线性映射***************/
//  chassis.Vx = map_range(chassis.Vx, RC_MIN, RC_MAX, motor_min, motor_max)+key_x_fast - key_x_slow;
//  chassis.Vy = map_range(chassis.Vy, RC_MIN, RC_MAX, motor_min, motor_max)+ key_y_fast - key_y_slow;
//  chassis.Wz = map_range(chassis.Wz, RC_MIN, RC_MAX, motor_min, motor_max)+key_Wz_acw + key_Wz_cw;
  int16_t Temp_Vx = chassis.Vx;
  int16_t Temp_Vy = chassis.Vy;

  chassis.err_angle= chassis.INS.yaw_update - gimbal_Yaw;
  chassis.err_angle = -chassis.err_angle / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反

  chassis.Vx = cos(chassis.err_angle) * Temp_Vx - sin(chassis.err_angle) * Temp_Vy;
  chassis.Vy = sin(chassis.err_angle) * Temp_Vx + cos(chassis.err_angle) * Temp_Vy;
	
   cycle = 1; // 记录的模式状态的变量，以便切换到 follow 模式的时候，可以知道分辨已经切换模式，计算一次 yaw 的差值

}

// 小陀螺模式
static void chassis_gyro_mode()
{
  chassis.Vx = rc_ctrl[TEMP].rc.rocker_r_ / 660.0f * chassis_speed_max + key_x_fast - key_x_slow; // left and right
  chassis.Vy = rc_ctrl[TEMP].rc.rocker_r1 / 660.0f * chassis_speed_max + key_y_fast - key_y_slow; // front and back
  chassis.Wz = chassis_wz_max;

  int16_t Temp_Vx = chassis.Vx;
  int16_t Temp_Vy = chassis.Vy;

  chassis.err_angle= chassis.INS.yaw_update - gimbal_Yaw;
  chassis.err_angle = -chassis.err_angle / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反

  chassis.Vx = cos(chassis.err_angle) * Temp_Vx - sin(chassis.err_angle) * Temp_Vy;
  chassis.Vy = sin(chassis.err_angle) * Temp_Vx + cos(chassis.err_angle) * Temp_Vy;

  cycle = 1; // 记录的模式状态的变量，以便切换到 follow 模式的时候，可以知道分辨已经切换模式，计算一次 yaw 的差值

}

// 底盘跟随云台
static void chassis_follow_mode(void)
{
  chassis.Vx = rc_ctrl[TEMP].rc.rocker_r_ / 660.0f * chassis_speed_max + key_x_fast - key_x_slow; // left and right
  chassis.Vy = rc_ctrl[TEMP].rc.rocker_r1 / 660.0f * chassis_speed_max + key_y_fast - key_y_slow; // front and back

  chassis.err_angle = chassis.INS.yaw_update - gimbal_Yaw;

  // 消除静态旋转
  if (chassis.err_angle  > -5 && chassis.err_angle  < 5)
  {
    chassis.Wz = 0;
  }
  else
  {
    detel_calc(&chassis.err_angle);
    chassis.Wz = -chassis.err_angle  * FOLLOW_WEIGHT;

    if (chassis.Wz > 2 * chassis_wz_max)
      chassis.Wz = 2 * chassis_wz_max;
    if (chassis.Wz < -2 * chassis_wz_max)
      chassis.Wz = -2 * chassis_wz_max;
  }

  int16_t Temp_Vx = chassis.Vx;
  int16_t Temp_Vy = chassis.Vy;
  chassis.err_angle = -chassis.err_angle / 57.3f;  // 此处加负是因为旋转角度后，旋转方向相反
  chassis.Vx = cos(chassis.err_angle) * Temp_Vx - sin(chassis.err_angle) * Temp_Vy;
  chassis.Vy = sin(chassis.err_angle) * Temp_Vx + cos(chassis.err_angle) * Temp_Vy;
    
  cycle = 1; // 记录的模式状态的变量，以便切换到 follow 模式的时候，可以知道分辨已经切换模式，计算一次 yaw 的差值
}

//底盘停止模式
static void chassis_stop_mode()
{
  chassis.Vx = 0;
  chassis.Vy = 0;
  chassis.Wz = 0;
}

// // 运动解算
static void chassis_motol_speed_calculate()
{
  chassis.speed_target[CHAS_LF] = 3*(-chassis.Wz)*0.4 + chassis.Vx - chassis.Vy;
  chassis.speed_target[CHAS_RF] = 3*(-chassis.Wz)*0.4 - chassis.Vx - chassis.Vy;
  chassis.speed_target[CHAS_RB] = 3*(-chassis.Wz)*0.4 + chassis.Vx + chassis.Vy;
  chassis.speed_target[CHAS_LB] = 3*(-chassis.Wz)*0.4 - chassis.Vx + chassis.Vy;
}

// 速度限制函数
static void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed)
{
  uint8_t i = 0;
  int16_t max = 0;
  int16_t temp = 0;
  int16_t max_speed = limit_speed;
  fp32 rate = 0;
  for (i = 0; i < 4; i++)
  {
    temp = (motor_speed[i] > 0) ? (motor_speed[i]) : (-motor_speed[i]); // 求绝对值

    if (temp > max)
    {
      max = temp;
    }
  }

  if (max > max_speed)
  {
    rate = max_speed * 1.0 / max; //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
    for (i = 0; i < 4; i++)
    {
      motor_speed[i] *= rate;
    }
  }
}

// 电机电流控制
static void chassis_current_give()
{
  Motor_Speed_limiting(chassis.speed_target, CHASSIS_MAX_SPEED); // 限制最大期望速度，输入参数是限制速度值(同比缩放)

  uint8_t i = 0;

  for (i = 0; i < 4; i++)
  {
    chassis.motor_info[i].set_current = pid_calc(&chassis.pid[i], chassis.motor_info[i].rotor_speed, chassis.speed_target[i]);
  }
  
  Chassis_Power_Limit(CHASSIS_MAX_SPEED * 4); // 限制底盘功率

  set_motor_current_chassis(0, chassis.motor_info[0].set_current, chassis.motor_info[1].set_current, chassis.motor_info[2].set_current, chassis.motor_info[3].set_current);
}


// /*************************yaw值校正*******************************/
static void yaw_correct(void)
{
  // 只执行一次
  if (yaw_correction_flag)
  {
    yaw_correction_flag = 0;
    chassis.INS.yaw_init = chassis.INS.Yaw;//下面的
  }
  // Wz为负，顺时针旋转，陀螺仪飘 60°/min（以3000为例转出的，根据速度不同调整）
  // 解决yaw偏移，完成校正
  if (rc_ctrl[TEMP].key[KEY_PRESS].shift || rc_ctrl[TEMP].key[KEY_PRESS].ctrl|| rc_ctrl[TEMP].rc.switch_right == 3)
  {
    if (chassis.Wz > 500)
      chassis.imu_err -= 0.001f;
    // imu_err_yaw -= 0.001f * chassis_speed_max / 3000.0f;
    if (chassis.Wz < -500)
      chassis.imu_err += 0.001f;
    // imu_err_yaw += 0.001f * chassis_speed_max / 3000.0f;
  }
  chassis.INS.yaw_update = chassis.INS.Yaw - chassis.INS.yaw_init + chassis.imu_err;
}

static void detel_calc(fp32 *angle)
{
  // 如果角度大于180度，则减去360度
  if (*angle > 180)
  {
    *angle -= 360;
  }

  // 如果角度小于-180度，则加上360度
  else if (*angle < -180)
  {
    *angle += 360;
  }
}
static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{
  // 819.2/A，假设最大功率为120W，那么能够通过的最大电流为5A，取一个保守值：800.0 * 5 = 4000
  Watch_Power_Max = Klimit;
  Watch_Power = referee_infantry.chassis_power;
  Watch_Buffer =referee_infantry.buffer_energy; // 限制值，功率值，缓冲能量值，初始值是1，0，0
  // get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//通过裁判系统和编码器值获取（限制值，实时功率，实时缓冲能量）

  Chassis_pidout_max = 61536; // 32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

  if (Watch_Power > 600)
  {
     Motor_Speed_limiting(chassis.speed_target, 4096);
  }
  else
  {
		//此处应该是绝对值相加
    Chassis_pidout = ((chassis.speed_target[0] - chassis.motor_info[0].rotor_speed) +
                      (chassis.speed_target[1] - chassis.motor_info[1].rotor_speed) +
                      (chassis.speed_target[2] - chassis.motor_info[2].rotor_speed) +
                      (chassis.speed_target[3] - chassis.motor_info[3].rotor_speed)); // fabs是求绝对值，这里获取了4个轮子的差值求和
    //	Chassis_pidout_target = fabs(motor_speed_target[0]) + fabs(motor_speed_target[1]) + fabs(motor_speed_target[2]) + fabs(motor_speed_target[3]);

    /*期望滞后占比环，增益个体加速度*/
    if (Chassis_pidout)
    {
      Scaling1 = (chassis.speed_target[0] - chassis.motor_info[0].rotor_speed) / Chassis_pidout;
      Scaling2 = (chassis.speed_target[1] - chassis.motor_info[1].rotor_speed) / Chassis_pidout;
      Scaling3 = (chassis.speed_target[2] - chassis.motor_info[2].rotor_speed) / Chassis_pidout;
      Scaling4 = (chassis.speed_target[3] - chassis.motor_info[3].rotor_speed) / Chassis_pidout; // 求比例，4个scaling求和为1
    }
    else
    {
      Scaling1 = 0.25, Scaling2 = 0.25, Scaling3 = 0.25, Scaling4 = 0.25;
    }

    /*功率满输出占比环，车总增益加速度*/
    //		if(Chassis_pidout_target) Klimit=Chassis_pidout/Chassis_pidout_target;	//375*4 = 1500
    //		else{Klimit = 0;}
    Klimit = Chassis_pidout / Chassis_pidout_target_limit;

    if (Klimit > 1)
      Klimit = 1;
    else if (Klimit < -1)
      Klimit = -1; // 限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

    // /*缓冲能量占比环，总体约束*/
    // if (Watch_Buffer < 50 && Watch_Buffer >= 40)
    //   Plimit = 0.9; // 近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
    // else if (Watch_Buffer < 40 && Watch_Buffer >= 35)
    //   Plimit = 0.75;
    // else if (Watch_Buffer < 35 && Watch_Buffer >= 30)
    //   Plimit = 0.5;
    // else if (Watch_Buffer < 30 && Watch_Buffer >= 20)
    //   Plimit = 0.25;
    // else if (Watch_Buffer < 20 && Watch_Buffer >= 10)
    //   Plimit = 0.125;
    // else if (Watch_Buffer < 10 && Watch_Buffer >= 0)
    //   Plimit = 0.05;
    // else
    // {
    //   Plimit = 1;
    // }
    if (!supercap_flag)
    {
      if (powerdata[1]) // 如果接入supercap
      {
        if (powerdata[1] < 24 && powerdata[1] > 23)
          Plimit = 0.9;
        else if (powerdata[1] < 23 && powerdata[1] > 22)
          Plimit = 0.8;
        else if (powerdata[1] < 22 && powerdata[1] > 21)
          Plimit = 0.7;
        else if (powerdata[1] < 21 && powerdata[1] > 20)
          Plimit = 0.6;
        else if (powerdata[1] < 20 && powerdata[1] > 18)
          Plimit = 0.5;
        else if (powerdata[1] < 18 && powerdata[1] > 15)
          Plimit = 0.3;
        else if (powerdata[1] < 15)
          Plimit = 0.1;
      }
      else // 防止不接入supercap时，Plimit为0.1
      {
        Plimit = 1;
      }
    }
    else
    {
      // if (powerdata[1] < 24 && powerdata[1] > 16)
      Plimit = 1;
      // else if (powerdata[1] < 16 && powerdata[1] > 12)
      //   Plimit = 0.5;
    }

    chassis.motor_info[0].set_current = Scaling1 * (Chassis_pidout_max * Klimit) * Plimit; // 输出值
    chassis.motor_info[1].set_current = Scaling2 * (Chassis_pidout_max * Klimit) * Plimit;
    chassis.motor_info[2].set_current = Scaling3 * (Chassis_pidout_max * Klimit) * Plimit;
    chassis.motor_info[3].set_current = Scaling4 * (Chassis_pidout_max * Klimit) * Plimit; /*同比缩放电流*/
  }
}

