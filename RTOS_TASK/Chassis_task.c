#include "Chassis_task.h"
#include "cmsis_os.h"
#include "exchange.h"
#include "drv_can.h"
#include "imu_task.h"
#include "usart.h"
#include "pid.h"
#include "math.h"
#include "arm_math.h"
#include "struct_typedef.h"
#include "rm_referee.h"
#include "referee_protocol.h"

#define KEY_START_OFFSET 10
#define KEY_STOP_OFFSET 20
#define CHASSIS_WZ_MAX 4000

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
extern double yaw12;//下面的yaw
extern float Yaw1;//新陀螺仪的
extern float Yaw_top ;//float Yaw_top;上面的 yaw
//yaw校准参数
float Yaw;//用来计算
float Yaw_update;
float Yaw_init;
int yaw_correction_flag = 1; // yaw值校正标志
float relative_yaw=0;
static uint8_t cycle = 0; // do while循环一次的条件
float imu_err_yaw = 0;    // 记录yaw飘移的数值便于进行校正

// pid_struct_t supercap_pid;
motor_info_t motor_info_chassis[10]; // 电机信息结构体
// fp32 superpid[3] = {120, 0.1, 0};

extern float Hero_chassis_power;
extern uint16_t Hero_chassis_power_buffer;
extern uint8_t rx_buffer_c[49];
extern uint8_t rx_buffer_d[128];
extern uint8_t Hero_level;

int8_t chassis_mode;
uint8_t supercap_flag = 0;         
extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体
extern float powerdata[4];
extern uint16_t shift_flag;
static int16_t key_x_fast, key_y_fast, key_x_slow, key_y_slow, key_Wz_acw, key_Wz_cw;
// uint8_t rc[18];
static referee_info_t *referee_data; // 用于获取裁判系统的数据

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
//

static void Chassis_Init();

static void Chassis_loop_Init();

static void level_judge();

// 读取键鼠数据控制底盘模式
static void read_keyboard(void);

// 模式选择
static void mode_chooce();

// 遥控器控制底盘电机
static void RC_Move(void);

// 小陀螺模式
static void gyroscope(void);

// 速度限制函数
static void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed);

// 电机电流控制
static void chassis_current_give();

// 运动解算
static void chassis_motol_speed_calculate();

static void detel_calc(fp32 *angle);

static void chassis_follow();

static void yaw_correct();

static void key_control();

static void Chassis_Power_Limit(double Chassis_pidout_target_limit);                  // 底盘功率限制

static void manual_yaw_correct();

void Chassis_task(void const *pvParameters)
{
  Chassis_Init();
  // imu_task_init();
  // imu_task();
 for (;;) // 底盘运动任务
  {
    // level_judge();

    Chassis_loop_Init();

    yaw_correct(); // 校准
    
    //读取底盘
    read_keyboard();

    // 选择底盘运动模式
    mode_chooce();

    // 电机速度解算
    chassis_motol_speed_calculate();

    // 电机电流控制
// Motor_Speed_limiting(chassis.speed_target,motor_max);

    chassis_current_give();
    // imu_task();
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
/*嘿嘿 *///键盘控制
static void read_keyboard(void)
{
  if (r_flag)
    chassis_mode = 1;
  else if (f_flag)
    chassis_mode = 2;

  if (x_flag)
    supercap_flag = 0;
  else if (c_flag)
    supercap_flag = 1;

  if (g_flag)//不同等级功率
    chassis_wz_max = CHASSIS_WZ_MAX_1;
  else if (b_flag)
    chassis_wz_max = CHASSIS_WZ_MAX_2;
}
/*嘿嘿回来*/
/*static void mode_chooce()
{
  // 遥控器控制
  // chanel 0 left max==-660,right max==660
  // chanel 1 up max==660,down max==-660
  // chanel 2 left max==-660,right max==660
  // chanel 3 up max==660,down max==-660
  // chanel 4 The remote control does not have this channel

  if (rc_ctrl.rc.s[0] == 1)
  {
    key_control();
    gyroscope();
  }
  else if (rc_ctrl.rc.s[0] == 2|| chassis_mode == 2)
  {
    key_control();
    RC_Move();
  }
  else if (rc_ctrl.rc.s[0] == 3 || chassis_mode == 1)
  {
    key_control();
    chassis_follow();

  }
  // else
  // {
  // }
}*/

static void mode_chooce()
{
  // 右拨杆中，键鼠操作
  if (rc_ctrl.rc.s[0] == 3)
  {
      // 底盘跟随云台模式，r键触发
      if (chassis_mode == 1)
      {
        key_control();
        chassis_follow();
      }

      // 正常运动模式，f键触发
      else if (chassis_mode == 2)
      {
        key_control();
        manual_yaw_correct(); // 手动校正yaw值，头对正，按下V键
        RC_Move();
      }
  }
    // 右拨杆下，遥控操作
    else if (rc_ctrl.rc.s[0] == 2)
    {
      chassis_follow();
      // chassis_mode_normal();
    }

    else
    {
      key_control();
      RC_Move();
    }
}

/*
static void mode_chooce()
{
  // 右拨杆中，键鼠操作
  if (rc_ctrl.rc.s[0] == 1)
  {
      // 底盘跟随云台模式，r键触发
      //if (chassis_mode == 1)
      //{
        //key_control();
      chassis_follow();
      //}

      // 正常运动模式，f键触发
      //else if (chassis_mode == 2)
     // {
      //  key_control();
      //  manual_yaw_correct(); // 手动校正yaw值，头对正，按下V键
      //  RC_Move();
      //}
  }
    // 右拨杆下，遥控操作
    else if (rc_ctrl.rc.s[0] == 2)
    {
      //chassis_follow();
      // chassis_mode_normal();
      gyroscope();
    }
    else
    {
      //key_control();
      //manual_yaw_correct();
      RC_Move();
    }
}
*/

// // 运动解算
static void chassis_motol_speed_calculate()
{

  // 根据分解的速度调整电机速度目标
  // chassis.speed_target[CHAS_LF] = 3*(-chassis.Wz)*0.4 + chassis.Vx + chassis.Vy;
  // chassis.speed_target[CHAS_RF] = 3*(-chassis.Wz)*0.4 + chassis.Vx - chassis.Vy;
  // chassis.speed_target[CHAS_RB] = 3*(-chassis.Wz)*0.4 - chassis.Vx - chassis.Vy;
  // chassis.speed_target[CHAS_LB] = 3*(-chassis.Wz)*0.4 - chassis.Vx + chassis.Vy;

  chassis.speed_target[CHAS_LF] = 3*(-chassis.Wz)*0.4 + chassis.Vx - chassis.Vy;
  chassis.speed_target[CHAS_RF] = 3*(-chassis.Wz)*0.4 - chassis.Vx - chassis.Vy;
  chassis.speed_target[CHAS_RB] = 3*(-chassis.Wz)*0.4 + chassis.Vx + chassis.Vy;
  chassis.speed_target[CHAS_LB] = 3*(-chassis.Wz)*0.4 - chassis.Vx + chassis.Vy;
}
// 运动解算
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
  // set_curruent(MOTOR_3508_0, hcan1, chassis.motor_info[0].set_current, chassis.motor_info[1].set_current, chassis.motor_info[2].set_current, chassis.motor_info[3].set_current);
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

static void RC_Move(void)
{
  // 从遥控器获取控制输入
  chassis.Vx = rc_ctrl.rc.ch[3]; // 前后输入
  chassis.Vy = rc_ctrl.rc.ch[2]; // 左右输入
  chassis.Wz = rc_ctrl.rc.ch[4]; // 旋转输入

  /*************记得加上线性映射***************/
  chassis.Vx = map_range(chassis.Vx, RC_MIN, RC_MAX, motor_min, motor_max)+key_x_fast - key_x_slow;
  chassis.Vy = map_range(chassis.Vy, RC_MIN, RC_MAX, motor_min, motor_max)+ key_y_fast - key_y_slow;
  chassis.Wz = map_range(chassis.Wz, RC_MIN, RC_MAX, motor_min, motor_max)+key_Wz_acw + key_Wz_cw;

      // relative_yaw = (yaw12 - Yaw_top) / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反
relative_yaw = (Yaw_update - Yaw_top) / 57.3f;
    int16_t temp_Vx = 0;
  int16_t temp_Vy = 0;
  //  temp_Vx = chassis.Vx * cosf(0) - chassis.Vy * sinf(0);
  // temp_Vy = chassis.Vx * sinf(0) + chassis.Vy * cosf(0);
  temp_Vx = chassis.Vx * cosf(relative_yaw) - chassis.Vy * sinf(relative_yaw);
  temp_Vy = chassis.Vx * sinf(relative_yaw) + chassis.Vy * cosf(relative_yaw);
  chassis.Vx = temp_Vx;
  chassis.Vy = temp_Vy;
    cycle = 1; // 记录的模式状态的变量，以便切换到 follow 模式的时候，可以知道分辨已经切换模式，计算一次 yaw 的差值

}

// 小陀螺模式
static void gyroscope(void)
{

  chassis.Wz = 3000;
  // chassis.Vx = rc_ctrl.rc.ch[3]; // 前后输入
  // chassis.Vy = rc_ctrl.rc.ch[2]; // 左右输入
  /*************记得加上线性映射***************/
  chassis.Vx = map_range(rc_ctrl.rc.ch[3], RC_MIN, RC_MAX, motor_min, motor_max)+ key_x_fast - key_x_slow;
  chassis.Vy = map_range(rc_ctrl.rc.ch[2], RC_MIN, RC_MAX, motor_min, motor_max)+ key_y_fast - key_y_slow;
  // int16_t Temp_Vx = chassis.Vx;
  // int16_t Temp_Vy = chassis.Vy;
  //  relative_yaw = Yaw - INS_top.Yaw;//改变量 这里是下面的减去上面的 但是按照五号的来 需要下面的陀螺仪减去上面的c板
  //  relative_yaw = yaw12 - Yaw_top; //暂时没有加陀螺仪代码 需要改
  // relative_yaw = -(yaw12 - Yaw_top) ; // 此处加负是因为旋转角度后，旋转方向相反
    //////////////////////////////////////////////// /*relative_yaw = (yaw12 - Yaw_top) / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反
    relative_yaw = (Yaw_update - Yaw_top) / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反

  // chassis.Vx = cosf(relative_yaw) * Temp_Vx - sinf(relative_yaw) * Temp_Vy;
  // chassis.Vy = sinf(relative_yaw) * Temp_Vx + cosf(relative_yaw) * Temp_Vy;


  int16_t temp_Vx = 0;
  int16_t temp_Vy = 0;
  //  temp_Vx = chassis.Vx * cosf(0) - chassis.Vy * sinf(0);
  // temp_Vy = chassis.Vx * sinf(0) + chassis.Vy * cosf(0);
  temp_Vx = chassis.Vx * cosf(relative_yaw) - chassis.Vy * sinf(relative_yaw);
  temp_Vy = chassis.Vx * sinf(relative_yaw) + chassis.Vy * cosf(relative_yaw);
  chassis.Vx = temp_Vx;
  chassis.Vy = temp_Vy;


  cycle = 1; // 记录的模式状态的变量，以便切换到 follow 模式的时候，可以知道分辨已经切换模式，计算一次 yaw 的差值

}

// 底盘跟随云台
static void chassis_follow(void)
{

  chassis.Vx = rc_ctrl.rc.ch[3]; // 前后输入
  chassis.Vy = rc_ctrl.rc.ch[2]; // 左右输入
  // chassis.Wz = rc_ctrl.rc.ch[4]; // 旋转输入
  /*************记得加上线性映射***************/
  chassis.Vx = map_range(chassis.Vx, RC_MIN, RC_MAX, motor_min, motor_max)+ key_x_fast - key_x_slow;
  chassis.Vy = map_range(chassis.Vy, RC_MIN, RC_MAX, motor_min, motor_max)+ key_y_fast - key_y_slow;

  // int16_t relative_yaw = Yaw - INS.Yaw_update; // 最新的减去上面的
 relative_yaw = Yaw_update-Yaw_top;
  // int16_t yaw_speed = pid_calc(&pid_yaw_angle, 0, relative_yaw);
  // int16_t rotate_w = (chassis.motor_info[0].rotor_speed + chassis.motor_info[1].rotor_speed + chassis.motor_info[2].rotor_speed + chassis.motor_info[3].rotor_speed) / (4 * 19);
  // 消除静态旋转
  if (relative_yaw > -5 && relative_yaw < 5)
  {
    chassis.Wz = 0;
  }
  else
  {
        detel_calc(&relative_yaw);
        chassis.Wz = -relative_yaw*160;
    // chassis.Wz = pid_calc(&pid_yaw_speed, yaw_speed, rotate_w);

    if(chassis.Wz > 2 * chassis_wz_max)
    chassis.Wz =2 * chassis_wz_max;
    if(chassis.Wz < -2 * chassis_wz_max)
    chassis.Wz = -2 * chassis_wz_max;
  }
  int16_t Temp_Vx = chassis.Vx;
  int16_t Temp_Vy = chassis.Vy;

  chassis.Vx = cos(-relative_yaw / 57.3f) * Temp_Vx - sin(-relative_yaw / 57.3f) * Temp_Vy;
  chassis.Vy = sin(-relative_yaw / 57.3f) * Temp_Vx + cos(-relative_yaw / 57.3f) * Temp_Vy;
    cycle = 1; // 记录的模式状态的变量，以便切换到 follow 模式的时候，可以知道分辨已经切换模式，计算一次 yaw 的差值

}

// /*************************yaw值校正*******************************/
static void yaw_correct(void)
{
  // 只执行一次
  if (yaw_correction_flag)
  {
    yaw_correction_flag = 0;
    Yaw_init = Yaw1;//下面的
  }
  // Wz为负，顺时针旋转，陀螺仪飘 60°/min（以3000为例转出的，根据速度不同调整）
  // 解决yaw偏移，完成校正
  if (shift_flag || ctrl_flag || rc_ctrl.rc.s[0] == 3)
  {
    if (chassis.Wz > 500)
      imu_err_yaw -= 0.001f;
    // imu_err_yaw -= 0.001f * chassis_speed_max / 3000.0f;
    if (chassis.Wz < -500)
      imu_err_yaw += 0.001f;
    // imu_err_yaw += 0.001f * chassis_speed_max / 3000.0f;
  }
  Yaw_update = Yaw1 - Yaw_init + imu_err_yaw;
}
/*************************** 键盘控制函数 ************************/
static void key_control(void)
{
  if (d_flag)
    key_y_fast += KEY_START_OFFSET;
  else
    key_y_fast -= KEY_STOP_OFFSET;
  if (a_flag)
    key_y_slow += KEY_START_OFFSET;
  else
    key_y_slow -= KEY_STOP_OFFSET;
  if (w_flag)
    key_x_fast += KEY_START_OFFSET;
  else
    key_x_fast -= KEY_STOP_OFFSET;
  if (s_flag)
    key_x_slow += KEY_START_OFFSET;
  else
    key_x_slow -= KEY_STOP_OFFSET;
//*******************************************//
//正转
  if (shift_flag)
    key_Wz_acw += KEY_START_OFFSET;
  else
    key_Wz_acw -= KEY_STOP_OFFSET;

  // 反转
  if (ctrl_flag)
    key_Wz_cw -= KEY_START_OFFSET;
  else
    key_Wz_cw += KEY_STOP_OFFSET;
    //
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

      if (key_Wz_acw > CHASSIS_WZ_MAX)
    key_Wz_acw = CHASSIS_WZ_MAX;
  if (key_Wz_acw < 0)
    key_Wz_acw = 0;
  if (key_Wz_cw < -CHASSIS_WZ_MAX)
    key_Wz_cw = -CHASSIS_WZ_MAX;
  if (key_Wz_cw > 0)
    key_Wz_cw = 0;
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
    Watch_Power = Hero_chassis_power;
  Watch_Buffer = Hero_chassis_power_buffer; // 限制值，功率值，缓冲能量值，初始值是1，0，0
  // Watch_Power = referee_data->PowerHeatData.chassis_power; // powerd.chassis_power
  // Watch_Buffer = referee_data->PowerHeatData.buffer_energy; // powerd.chassis_power_buffer
// Watch_Power =0;
// Watch_Buffer=60;
  // Hero_chassis_power_buffer;//限制值，功率值，缓冲能量值，初始值是1，0，0
  // get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//通过裁判系统和编码器值获取（限制值，实时功率，实时缓冲能量）

  Chassis_pidout_max = 61536; // 32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

  if (Watch_Power > 600)
    Motor_Speed_limiting(chassis.speed_target, 4096); // 限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
  else
  {
    Chassis_pidout = (fabs(chassis.speed_target[0] - chassis.motor_info[0].rotor_speed) +
                      fabs(chassis.speed_target[1] - chassis.motor_info[1].rotor_speed) +
                      fabs(chassis.speed_target[2] - chassis.motor_info[2].rotor_speed) +
                      fabs(chassis.speed_target[3] - chassis.motor_info[3].rotor_speed)); // fabs是求绝对值，这里获取了4个轮子的差值求和

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

    /*缓冲能量占比环，总体约束*/
    if (Watch_Buffer < 50 && Watch_Buffer >= 40)
      Plimit = 0.9; // 近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
    else if (Watch_Buffer < 40 && Watch_Buffer >= 35)
      Plimit = 0.75;
    else if (Watch_Buffer < 35 && Watch_Buffer >= 30)
      Plimit = 0.5;
    else if (Watch_Buffer < 30 && Watch_Buffer >= 20)
      Plimit = 0.25;
    else if (Watch_Buffer < 20 && Watch_Buffer >= 10)
      Plimit = 0.125;
    else if (Watch_Buffer < 10 && Watch_Buffer > 0)
      Plimit = 0.05;
    else
    {
      Plimit = 1;
    }

    chassis.motor_info[0].set_current = Scaling1 * (Chassis_pidout_max * Klimit) * Plimit; // 输出值
    chassis.motor_info[1].set_current = Scaling2 * (Chassis_pidout_max * Klimit) * Plimit;
    chassis.motor_info[2].set_current = Scaling3 * (Chassis_pidout_max * Klimit) * Plimit;
    chassis.motor_info[3].set_current = Scaling4 * (Chassis_pidout_max * Klimit) * Plimit; /*同比缩放电流*/
  }
}
/***************** 手动yaw值校正（仅在正常运动模式生效） ******************/
static void manual_yaw_correct()
{
  if (v_flag)
  {
    float manual_err_yaw = Yaw_update - Yaw_top;
    imu_err_yaw -= manual_err_yaw;
  }
}


static void level_judge()
{
  if (Hero_level)
  {
    switch (Hero_level)
    {
    case 1:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_1;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_1 + 3000;
      break;
    case 2:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_2;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_2 + 3000;
      break;
    case 3:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_3;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_3 + 3000;
      break;
    case 4:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_4;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_4 + 3000;
      break;
    case 5:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_5;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_5 + 3000;
      break;
    case 6:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_6;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_6 + 3000;
      break;
    case 7:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_7;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_7 + 3000;
      break;
    case 8:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_8;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_8 + 3000;
      break;
    case 9:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_9;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_9 + 3000;
      break;
    case 10:
      if (!supercap_flag)
        chassis_speed_max = CHASSIS_SPEED_MAX_10;
      else
        chassis_speed_max = CHASSIS_SPEED_MAX_10 + 3000;
      break;
    }
  }
  else
    chassis_speed_max = CHASSIS_SPEED_MAX_1;
}
