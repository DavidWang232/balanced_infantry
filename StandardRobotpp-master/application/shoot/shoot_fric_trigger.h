/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_fric.c/h
  * @brief      使用摩擦轮的发射机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.1.0     2025-1-15       CJH             1. 实现基本功能
  *  V2.0.0     2025-3-3        CJH             1. 兼容了达妙4310拨弹盘和大疆2006拨弹盘
  *                                             2. 完善了单发功能，上位机火控功能
  *                                             3. 增加了热量限制
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "robot_param.h"

#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)
#ifndef SHOOT_FRIC_H
#define SHOOT_FRIC_H
#include "CAN_communication.h"
#include "arm_math.h"
#include "detect_task.h"
#include "gpio.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "referee.h"
#include "remote_control.h"
#include "shoot.h"
#include "supervisory_computer_cmd.h"
#include "tim.h"
#include "usb_debug.h"
#include "user_lib.h"

/* TB6612 控制参数 */
#define TB6612_PWM_ARR 19999       // TIM1 ARR值
#define TB6612_PWM_PRESCALER 167   // TIM1 Prescaler值
#define TB6612_FORWARD_SPEED 16999  // 正转PWM值
#define TB6612_REVERSE_SPEED 15999  // 反转PWM值

typedef enum {
    LOAD_STOP,       // 停止拨盘
    LAOD_BULLET,     // 单发模式
    LOAD_BURSTFIRE,  // 连发模式,对速度闭环
    LOAD_BLOCK       // 堵转，模式
} LoadMode_e;

typedef enum {
    FRIC_NOT_READY,  // 未准备发射
    FRIC_READY,      // 准备发射
} FricState_e;

typedef struct feedback
{
    fp32 trigger_angel_fdb;  // 拨弹盘输出轴位置
    fp32 trigger_speed_fdb;  // 拨弹盘输出轴速度
    fp32 fric_speed_fdb_L;   // 摩擦轮输出轴速度
    fp32 fric_speed_fdb_R;
    fp32 fric_speed_fdb_R1;
    fp32 fric_speed_fdb_L1;
} Fdb;

typedef struct reference
{
    fp32 trigger_angel_ref;  // 拨弹盘位置期望
    fp32 trigger_speed_ref;  // 拨弹盘速度期望
    fp32 fric_speed_ref_L;   // 摩擦轮速度期望
    fp32 fric_speed_ref_R;
    fp32 fric_speed_ref_R1;
    fp32 fric_speed_ref_L1;
} Ref;

typedef struct
{
    const RC_ctrl_t * rc;  // 射击使用的遥控器指针
    LoadMode_e mode;       // 射击模式
    FricState_e state;     // 摩擦轮状态

    Motor_s fric_motor[4];  // 摩擦轮电机
    Motor_s trigger_motor;  // 拨弹盘电机

    //pid
    pid_type_def trigger_angel_pid;
    pid_type_def trigger_speed_pid;
    pid_type_def fric_pid[4];

    //block_reverse
    uint16_t reverse_time;
    uint16_t block_time;
    fp32 last_trigger_vel;
    fp32 last_fric_vel;

    //feedback
    Fdb FDB;

    //reference
    Ref REF;

    //flag
    uint16_t fric_flag;   //    摩擦轮状态
    uint16_t move_flag;   //    拨弹盘角度状态，用于判断单发射击执行情况
    uint16_t shoot_flag;  //    鼠标左键状态，用于判断弹发射击启动

    //ecd
    int16_t last_ecd;   //     上一个ecd
    int16_t ecd_count;  //     ecd计数

    // heat
    uint16_t heat_limit;
    uint16_t heat;
    uint16_t mr_time;

    // N20 motor control for bullet feed
    uint8_t n20_state;                // 0: 反转, 1: 正转
    uint32_t n20_forward_start_time;  // 正转开始时间
    uint8_t bullet_feed_flag;         // 拨弹标志
} Shoot_s;

extern void ShootInit(void);

extern void ShootSetMode(void);

extern void ShootObserver(void);

extern void ShootReference(void);

extern void ShootConsole(void);

extern void ShootSendCmd(void);

extern uint8_t GetChainStatus(void);
extern void SetChainStatus(uint8_t status);

#endif  // SHOOT_FRIC_H
#endif  // SHOOT_TYPE == SHOOT_FRIC
