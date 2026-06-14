/**
  * @file       robot_param_mecannum_hero.h
  * @brief      这里是麦克纳姆轮英雄机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_MODE_CHANNEL 0  // 选择底盘状态 开关通道号

#define CHASSIS_TYPE CHASSIS_MECANUM_WHEEL       // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT      // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER            // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  //选择机械臂类型

// 机器人物理参数
typedef enum {
    // 底盘CAN1
    WHEEL1 = 0,
    WHEEL2 = 1,
    WHEEL3 = 2,
    WHEEL4 = 3,
    // 云台CAN2
    YAW = 4,
    PITCH = 5,
    TRIGGER = 6,
    FRIC1 = 0,
    FRIC2 = 1,
} DJIMotorIndex_e;  //DJI电机在接收数据数组中的索引

// 底盘的遥控器相关宏定义 ---------------------
#define CHASSIS_MODE_CHANNEL 0        // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL 1           // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0           // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 0          // 旋转的遥控器通道号码
#define CHASSIS_ROLL_CHANNEL 4        // ROLL角的遥控器通道号码
#define CHASSIS_RC_DEADLINE 3        // 摇杆死区
#define CHASSIS_ANGLE_DEADBAND 0.05f  // 底盘跟随云台角度死区(rad)，约2.87度

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define WHEEL_RADIUS 0.106f  //(m)轮子半径
#define WHEEL_DIRECTION 1    //轮子方向
#define CAR_RADIUS 0.23f     //(m)车中轴到轮子的距离
//upper_limit parameters ---------------------
#define MAX_SPEED_VECTOR_VX 1.5f
#define MAX_SPEED_VECTOR_VY 1.5f
#define MAX_SPEED_VECTOR_WZ 3.0f

//lower_limit parameters ---------------------
#define MIN_SPEED_VECTOR_VX -MAX_SPEED_VECTOR_VX
#define MIN_SPEED_VECTOR_VY -MAX_SPEED_VECTOR_VY
#define MIN_SPEED_VECTOR_WZ -MAX_SPEED_VECTOR_WZ

//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_CHASSIS_WHEEL_SPEED_0 45.0f
// #define KP_CHASSIS_WHEEL_SPEED_0 15.0f
#define KI_CHASSIS_WHEEL_SPEED_0 0.0f
#define KD_CHASSIS_WHEEL_SPEED_0 0.3f

#define KP_CHASSIS_WHEEL_SPEED 45.0f
#define KI_CHASSIS_WHEEL_SPEED 0.0f
#define KD_CHASSIS_WHEEL_SPEED 0.3f
#define MAX_IOUT_CHASSIS_WHEEL_SPEED 10000.0f
#define MAX_OUT_CHASSIS_WHEEL_SPEED 30000.0f

#define KP_CHASSIS_WHEEL_SPEED_2 20.0f
#define KI_CHASSIS_WHEEL_SPEED_2 0.0f
#define KD_CHASSIS_WHEEL_SPEED_2 0.3f

//云台跟随角度环PID参数
#define KP_CHASSIS_GIMBAL_FOLLOW_ANGLE -373.0f
#define KI_CHASSIS_GIMBAL_FOLLOW_ANGLE 0.0f
#define KD_CHASSIS_GIMBAL_FOLLOW_ANGLE 0.0f
#define MAX_IOUT_CHASSIS_GIMBAL_FOLLOW_ANGLE 10000.0f
#define MAX_OUT_CHASSIS_GIMBAL_FOLLOW_ANGLE 30000.0f

#define MAX_ROLL (0.3f)
#define MIN_ROLL (-MAX_ROLL)

#define CHASSIS_RC_MAX_SPEED 150
//rocker value (max 660) change to vertial speed (m/s)
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.2f
//rocker value (max 660) change to horizontal speed (m/s)
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.2f  //0.5f
#define CHASSIS_WX_RC_SEN 0.2f

#define CHASSIS_WZ_SET_SCALE 0.1f
#define MOTOR_DISTANCE_TO_CENTER 0.2f

#define CHASSIA_SPIN_SPEED 5.5f  //小陀螺旋转速度设定
#define CHASSIA_STOP_SPEED 0.0f

//chassis forward or back max speed
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 500.0f
//chassis left or right max speed
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 500.0f
//底盘小陀螺速度
// #define NORMAL_MAX_CHASSIS_SPEED_WX 650.0f

#define NORMAL_MAX_CHASSIS_SPEED_WX 450.0f
#define NORMAL_MIN_CHASSIS_SPEED_WX 0.0f
//当底盘数据错误时电流值给0
#define CHASSIA_CURR_ZERO 0.0f

/*-------------------- Gimbal --------------------*/
//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)2000

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (20000.0f)
//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY (100000.0f)
#define REMOTE_CONTROLLER_SENSITIVITY_2 (700000.0f)
//motor parameters ---------------------
//电机id
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)2)

//电机can口
#define GIMBAL_DIRECT_YAW_CAN ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DM_4310)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DM_4310)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (-1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE (0)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH (0.47f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.3f)

//电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (0.0f)  //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID (0.0f)    //云台初始化正对齐的时候使用的yaw轴正中心量

//PID parameters ---------------------
//YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE (20.0f)
#define KI_GIMBAL_YAW_ANGLE (0.0f)
#define KD_GIMBAL_YAW_ANGLE (1.0f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.8f)
#define MAX_OUT_GIMBAL_YAW_ANGLE (5.0f)
//VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY (20.0f)
#define KI_GIMBAL_YAW_VELOCITY (0.0f)
#define KD_GIMBAL_YAW_VELOCITY (2.0f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (8000.0f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY (30000.0f)

//PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE (43.0f)
#define KI_GIMBAL_PITCH_ANGLE (0.00f)
#define KD_GIMBAL_PITCH_ANGLE (12.0f)
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (1.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE (3.0f)
//VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY (12.0f)
#define KI_GIMBAL_PITCH_VELOCITY (0.0f)
#define KD_GIMBAL_PITCH_VELOCITY (1.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (10000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (30000.0f)

/*----------------此为pitch轴-------------*/
#define GIMBAL_DM4310_MIT_KP (0.0f)
#define GIMBAL_DM4310_MIT_KI (0.0f)
#define GIMBAL_DM4310_MIT_KD (8.0f)
/*----------------换成新yaw轴-------------*/
#define GIMBAL_DM6006_MIT_KP (0.0f)
#define GIMBAL_DM6006_MIT_KI (0.0f)
#define GIMBAL_DM6006_MIT_KD (7.1f)
/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.03f              // (m)摩擦轮半径
#define BULLET_NUM 6                   // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1                      // 定义枪管个数（一个枪管2个摩擦轮）
#define TRIGGER_REDUCTION_RATIO 10.0f  // 定义英雄电机到拨弹盘的齿轮减速比

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DM_4310)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机ID
#define TRIGGER_MOTOR_ID 5
#define FRIC_MOTOR_R_ID 1
#define FRIC_MOTOR_L_ID 2
#define FRIC_MOTOR_R1_ID 3
#define FRIC_MOTOR_L1_ID 4

//电机can口
#define TRIGGER_MOTOR_CAN 1
#define FRIC_MOTOR_R_CAN 2
#define FRIC_MOTOR_L_CAN 2
#define FRIC_MOTOR_R1_CAN 2
#define FRIC_MOTOR_L1_CAN 2

//电机std_id
#define STD_ID 0x200

//单环拨弹速度
#define TRIGGER_SPEED (80.0f)
//摩擦轮速度
//测速模块是前
// #define FRIC_R_SPEED (640.0f)    //左后
// #define FRIC_R_1_SPEED (575.0f)  //左前

// #define FRIC_L_SPEED (-575.0f)    //右前
// #define FRIC_L_1_SPEED (-640.0f)  //右后

#define FRIC_R_SPEED (416.0f)    //左前
#define FRIC_R_1_SPEED (435.0f)  //左后

#define FRIC_L_SPEED (-416.0f)    //右前
#define FRIC_L_1_SPEED (-435.0f)  //右后

#define FRIC_SPEED_LIMIT (1000.0f)

/*ECD parameters------------*/
//电机反馈码盘值范围  (none)
#define HALF_ECD_RANGE 0
#define ECD_RANGE 0

//电机rpm 变化成 旋转速度的比例  (none)
#define MOTOR_RPM_TO_SPEED (0.0f)
#define MOTOR_ECD_TO_ANGLE 0.0f
#define FULL_COUNT 0.0f

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED 0.1f
#define BLOCK_TIME 1000
#define REVERSE_TIME 700
#define REVERSE_SPEED (1.0f)  // (rad/s)

/*MIT parameters ---------------------*/
// #define TRIGGER_SPEED_MIT_KP (25.0f)
//#define TRIGGER_SPEED_MIT_KP (3.0f)
#define TRIGGER_SPEED_MIT_KD (1.40f)

/*PID parameters ---------------------*/

//拨弹轮电机PID速度环 -
#define TRIGGER_SPEED_PID_KP (54.0f)
#define TRIGGER_SPEED_PID_KI (1.7f)
#define TRIGGER_SPEED_PID_KD (6.2f)
//15满
#define TRIGGER_SPEED_PID_MAX_OUT (16000.0f)   // 提高输出上限保证控制能力
#define TRIGGER_SPEED_PID_MAX_IOUT (15000.0f)  // 大幅提高积分限幅

//拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (27.0f)
#define TRIGGER_ANGEL_PID_KI (0.0f)
#define TRIGGER_ANGEL_PID_KD (3.0f)

#define TRIGGER_ANGEL_PID_MAX_OUT (15000.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (1000.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (130.0f)
#define FIRC_SPEED_PID_KI (0.0f)
#define FRIC_SPEED_PID_KD (0.8f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

#define SHOOT_HEAT_REMAIN_VALUE 80  //80

// 摩擦轮数量
#define FRIC_MOTOR_NUM 4

/*-------------------- Hip Joint --------------------*/
// 髋关节电机类型
#define CHASSIS_HIP_MOTOR_TYPE ((MotorType_e)DM_4310)

// 髋关节电机PID参数 (角度环)
#define KP_CHASSIS_HIP_ANGLE 15.0f
#define KI_CHASSIS_HIP_ANGLE 2.0f
#define KD_CHASSIS_HIP_ANGLE 0.0f
#define MAX_IOUT_CHASSIS_HIP_ANGLE 400.0f
#define MAX_OUT_CHASSIS_HIP_ANGLE 5.0f  // 速度输出限制

// 髋关节MIT模式参数
#define CHASSIS_HIP_MIT_KP (49.0f)
#define CHASSIS_HIP_MIT_KD (75.0f)
//#define CHASSIS_HIP_MIT_KD (45.0f)

// 髋关节控制参数
#define CHASSIS_HIP_CONTROL_SENSITIVITY 0.005f  // 遥控器通道灵敏度
#define CHASSIS_HIP_MAX_ANGLE (M_PI / 3.0f)     // 最大角度 60度
#define CHASSIS_HIP_MIN_ANGLE 0
#define CHASSIS_HIP_INIT_ANGLE (0.0f)  // 初始角度

// 髋关节IMU自动模式 角度环PID参数 (外环，基于陀螺仪euler_x/pitch角度，输出目标角速度)
#define KP_CHASSIS_HIP_IMU 3.0f
#define KI_CHASSIS_HIP_IMU 0.0f
#define KD_CHASSIS_HIP_IMU 0.0f
#define MAX_IOUT_CHASSIS_HIP_IMU 0.0f
#define MAX_OUT_CHASSIS_HIP_IMU 5.0f        // 角度环输出限制（目标角速度 rad/s）
#define CHASSIS_HIP_AUTO_TARGET_PITCH 0.0f  // 自动模式目标pitch角度（水平为0）
#define CHASSIS_HIP_IMU_DEADLINE 0.09f      // IMU euler_x死区（rad），约1.7度

// 髋关节IMU自动模式 角速度环PID参数 (内环，基于陀螺仪gyro_x角速度，输出电机速度)
#define KP_CHASSIS_HIP_IMU_GYRO 3.0f
#define KI_CHASSIS_HIP_IMU_GYRO 0.0f
#define KD_CHASSIS_HIP_IMU_GYRO 0.0f
#define MAX_IOUT_CHASSIS_HIP_IMU_GYRO 1.0f
#define MAX_OUT_CHASSIS_HIP_IMU_GYRO 5.0f  // 角速度环输出限制（电机速度）

#endif /* INCLUDED_ROBOT_PARAM_H */
