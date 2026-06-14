/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.c/h
  * @brief      平衡底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#ifndef CHASSIS_MECANUM_H
#define CHASSIS_MECANUM_H
#include "CAN_cmd_dji.h"
#include "CAN_receive.h"
#include "SupCap.h"
#include "chassis.h"
#include "custom_typedef.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"

/*-------------------- Structural definition --------------------*/

typedef enum {
    CHASSIS_OFF,                // 底盘关闭
    CHASSIS_ZERO_FORCE,         // 底盘无力，所有控制量置0
    CHASSIS_FOLLOW_GIMBAL_YAW,  // 底盘跟随云台（运动方向为云台坐标系方向，需进行坐标转换）
    CHASSIS_STOP,               // 底盘停止运动(速度为0)
    CHASSIS_FREE,               // 底盘不跟随云台
    CHASSIS_SPIN,               // 底盘小陀螺模式
    CHASSIS_AUTO,               // 底盘自动模式
    CHASSIS_OPEN                // 遥控器的值乘以比例成电流值开环控制
} ChassisMode_e;

/**
 * @brief 状态、期望和限制值
 */

typedef struct
{
    float wheel_speed[4];  // (m/s)轮子速度
    ChassisSpeedVector_t speed_vector;
} Values_t_chassis;

// typedef struct
// {
//     pid_type_def wheel_pid_speed[4];
//     pid_type_def gimbal_follow_pid_angle[4];
// } PID_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;           // 底盘使用的遥控器指针
    const ImuMeasure_s * imu_data;  // CAN通讯IMU数据指针
    ChassisMode_e mode;             // 底盘模式
    ChassisState_e state;           // 底盘状态
    uint8_t error_code;             // 底盘错误代码

    /*-------------------- Motors --------------------*/
    // 定义4个麦克纳姆轮
    Motor_s wheel_motor[4];  // 驱动轮电机
    // 定义2个髋关节电机
    Motor_s hip_motor[2];  // 髋关节电机，0-左，1-右

    /*-------------------- Super Capacitor --------------------*/
    SupCap_s super_cap;  // 超级电容
    /*-------------------- Values --------------------*/

    Values_t_chassis ref;          // 期望值
    Values_t_chassis fdb;          // 状态值
    Values_t_chassis upper_limit;  // 上限值
    Values_t_chassis lower_limit;  // 下限值

    pid_type_def pid;                 // PID控制器
    pid_type_def motor_chassis[4];    //chassis motor data.底盘电机数据
    pid_type_def motor_speed_pid[4];  //motor speed PID.底盘电机速度pid
    pid_type_def chassis_angle_pid;   //follow angle PID.底盘跟随云台角度pid
    pid_type_def hip_angle_pid[2];    //髋关节角度PID控制器，0-左，1-右
    pid_type_def hip_imu_pid;         //髋关节IMU自动模式 角度环PID（外环，基于euler_x角度）
    pid_type_def hip_imu_gyro_pid;    //髋关节IMU自动模式 角速度环PID（内环，基于gyro_x角速度）

    float dyaw;        // (rad)(feedback)当前位置与云台中值角度差（用于坐标转换）
    uint16_t yaw_mid;  // (ecd)(preset)云台中值角度
    uint16_t current_set;

    fp32 vx_rc_set;  //底盘设定速度，遥控器控制云台坐标系下前进方向
    fp32 vy_rc_set;  //底盘设定速度，遥控器控制云台坐标系下左右方向
    fp32 wz_rc_set;  //底盘设定旋转速度，遥控器控制云台坐标系下
    fp32 vx_set;     //底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set;     //底盘设定速度 左右方向 左为正，单位 m/s
    fp32 wz_set;     //底盘设定旋转角速度，逆时针为正 单位 rad/s

    // 髋关节相关变量
    fp32 hip_angle_rc_set;       //髋关节遥控器设定角度
    fp32 hip_reference[2];       //髋关节期望角度，0-左，1-右
    fp32 hip_feedback[2];        //髋关节反馈角度，0-左，1-右
    fp32 hip_auto_target_pitch;  //髋关节自动模式目标pitch角度（水平为0）

} Chassis_s;

extern void ChassisInit(void);

extern void ChassisSetMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void ChassisSendCmd(void);

#endif  //CHASSIS_MECANUM_H
#endif  //CHASSIS_MECANUM_WHEEL
