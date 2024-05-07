#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef struct
{
    uint16_t can_id;        // ID号
    int16_t set_current;    // 发送信息
    uint16_t rotor_angle;   // 现在的角度
    int16_t rotor_speed;    // 现在的转速
    int16_t torque_current; // 实际转矩电流
    uint8_t temp;           // 电机温度
} motor_info_t;

typedef struct
{

    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;
    fp32 max_iout;

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];
    fp32 error[3];

} pid_struct_t;

typedef struct
{
    motor_info_t motor_info;     // 电机信息结构体
    fp32 pid_parameter[3];       // 云台电机的pid参数
    fp32 pid_angle_parameter[3]; // 云台电机的pid参数
    pid_struct_t pid;            // 云台电机的pid结构体
    pid_struct_t pid_angle;      // 云台电机的pid结构体
    fp32 speed_target;           // 云台电机的目标速度
    fp32 angle_target;           // 云台电机的目标角度
    fp32 err_angle;              // 云台电机的目标角度
} gimbal_t;


typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];          // 角速度
    float Accel[3];         // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    double Roll;
    double Pitch;
    double Yaw;
		
    double yaw_init;   // 初始化yaw的初始值
    double yaw_update; // 使用这个得到真正的yaw值
		
    double YawTotalAngle;
} INS_t;

typedef struct
{
    /* data */
    motor_info_t motor_info[4]; // 电机信息结构体
    fp32 pid_parameter[3];      // 底盘电机的pid参数
    pid_struct_t pid[4];        // 底盘电机的pid结构体
    int16_t speed_target[4];    // 底盘电机的目标速度
    int16_t Vx, Vy, Wz;         // 底盘电机的目标速度
	
	  fp32 err_angle;             // 下板与上板的角度差
    fp32 err_angle_rad;         // 下板与上板的角度差(弧度制)
	
    fp32 imu_err;               // 修正陀螺仪漂移量
    
	  /*底盘陀螺仪的数据*/
	  INS_t INS;
	  
} chassis_t;

typedef struct
{
    /* data */
    motor_info_t motor_info[4]; // 电机信息结构体

    fp32 pid_dial_para[3];     // 拨盘电机的pid参数
    fp32 pid_friction_para[3]; // 摩擦轮电机的pid参数
    fp32 pid_bay_para[3];      // 弹舱电机的pid参数

    pid_struct_t pid_dial;     // 拨盘电机的pid结构体
    pid_struct_t pid_friction; // 摩擦轮电机的pid结构体
    pid_struct_t pid_bay;      // 弹舱电机的pid结构体

    int16_t dial_speed_target;     // 拨盘电机的目标速度
    int16_t friction_speed_target[2]; // 摩擦轮电机的目标速度
    int16_t bay_speed_target;      // 弹舱电机的目标速度

} shooter_t;

#endif
