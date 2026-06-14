/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_mecanum.c/h
  * @brief      麦轮轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-9-2024      Tina_Lin         1. done
  *  V1.0.1     Dec-9-2024      Tina_Lin         1. 完成基本控制
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#include "CAN_cmd_SupCap.h"
#include "CAN_cmd_damiao.h"  // 添加达妙电机控制头文件
#include "CAN_receive.h"
#include "SupCap.h"
#include "chassis.h"
#include "chassis_mecanum.h"
#include "chassis_power_control.h"  // 添加功率控制头文件
#include "cmsis_os.h"               // FreeRTOS延时函数
#include "communication.h"          // 添加通信头文件以获取YKQ遥控器指针
#include "detect_task.h"
#include "gimbal.h"
#include "math.h"
#include "motor.h"
#include "referee.h"         // 添加裁判系统头文件
#include "remote_control.h"  // 添加遥控器头文件以支持键盘控制
#include "usb_debug.h"
#include "usb_task.h"
#include "user_lib.h"  // 添加用户库，使用 fp32_deadline 函数
Motor_s __Motor;
Chassis_s CHASSIS;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    //CHASSIS.rc = get_remote_control_point();  // 获取遥控器指针
    CHASSIS.rc = get_remote_control_tuchaun_point();  //获取图传遥控器指针
    CHASSIS.imu_data = GetImuMeasurePoint();          // 获取IMU数据指针
    /*-------------------- 初始化PID --------------------*/
    //速度环pid
    float chassis_speed_pid_0[3] = {
        KP_CHASSIS_WHEEL_SPEED_0, KI_CHASSIS_WHEEL_SPEED_0, KD_CHASSIS_WHEEL_SPEED_0};
    float chassis_speed_pid_1[3] = {
        KP_CHASSIS_WHEEL_SPEED, KI_CHASSIS_WHEEL_SPEED, KD_CHASSIS_WHEEL_SPEED};
    float chassis_speed_pid_2[3] = {
        KP_CHASSIS_WHEEL_SPEED_2, KI_CHASSIS_WHEEL_SPEED_2, KD_CHASSIS_WHEEL_SPEED_2};

    //yaw轴跟踪pid
    float chassis_yaw_pid[3] = {
        KP_CHASSIS_GIMBAL_FOLLOW_ANGLE, KI_CHASSIS_GIMBAL_FOLLOW_ANGLE,
        KD_CHASSIS_GIMBAL_FOLLOW_ANGLE};

    // //获取底盘电机数据指针，初始化PID
    uint8_t i;
    // for (i = 0; i < 4; i++) {
    //     //底盘不跟随云台下的pid初始化
    //     PID_init(
    //         &CHASSIS.motor_speed_pid[i], PID_POSITION, chassis_speed_pid,
    //         MAX_OUT_CHASSIS_WHEEL_SPEED, MAX_IOUT_CHASSIS_WHEEL_SPEED);
    // }
    PID_init(
        &CHASSIS.motor_speed_pid[0], PID_POSITION, chassis_speed_pid_2, MAX_OUT_CHASSIS_WHEEL_SPEED,
        MAX_IOUT_CHASSIS_WHEEL_SPEED);
    PID_init(
        &CHASSIS.motor_speed_pid[3], PID_POSITION, chassis_speed_pid_2, MAX_OUT_CHASSIS_WHEEL_SPEED,
        MAX_IOUT_CHASSIS_WHEEL_SPEED);
    PID_init(
        &CHASSIS.motor_speed_pid[2], PID_POSITION, chassis_speed_pid_2, MAX_OUT_CHASSIS_WHEEL_SPEED,
        MAX_IOUT_CHASSIS_WHEEL_SPEED);
    PID_init(
        &CHASSIS.motor_speed_pid[1], PID_POSITION, chassis_speed_pid_2, MAX_OUT_CHASSIS_WHEEL_SPEED,
        MAX_IOUT_CHASSIS_WHEEL_SPEED);
    //底盘跟随云台pid初始化
    PID_init(
        &CHASSIS.chassis_angle_pid, PID_POSITION, chassis_yaw_pid,
        MAX_OUT_CHASSIS_GIMBAL_FOLLOW_ANGLE, MAX_IOUT_CHASSIS_GIMBAL_FOLLOW_ANGLE);

    //髋关节PID初始化
    float hip_angle_pid[3] = {KP_CHASSIS_HIP_ANGLE, KI_CHASSIS_HIP_ANGLE, KD_CHASSIS_HIP_ANGLE};
    for (i = 0; i < 2; i++) {
        PID_init(
            &CHASSIS.hip_angle_pid[i], PID_POSITION, hip_angle_pid, MAX_OUT_CHASSIS_HIP_ANGLE,
            MAX_IOUT_CHASSIS_HIP_ANGLE);
    }

    //髋关节IMU自动模式 角度环PID初始化（外环，基于陀螺仪euler_x/pitch角度，输出目标角速度）
    float hip_imu_pid[3] = {KP_CHASSIS_HIP_IMU, KI_CHASSIS_HIP_IMU, KD_CHASSIS_HIP_IMU};
    PID_init(
        &CHASSIS.hip_imu_pid, PID_POSITION, hip_imu_pid, MAX_OUT_CHASSIS_HIP_IMU,
        MAX_IOUT_CHASSIS_HIP_IMU);
    //髋关节IMU自动模式 角速度环PID初始化（内环，基于陀螺仪gyro_x角速度，输出电机速度）
    float hip_imu_gyro_pid[3] = {
        KP_CHASSIS_HIP_IMU_GYRO, KI_CHASSIS_HIP_IMU_GYRO, KD_CHASSIS_HIP_IMU_GYRO};
    PID_init(
        &CHASSIS.hip_imu_gyro_pid, PID_POSITION, hip_imu_gyro_pid, MAX_OUT_CHASSIS_HIP_IMU_GYRO,
        MAX_IOUT_CHASSIS_HIP_IMU_GYRO);
    CHASSIS.hip_auto_target_pitch = CHASSIS_HIP_AUTO_TARGET_PITCH;

    /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(&CHASSIS.wheel_motor[0], 1, 1, DJI_M3508, 1, 1, 1);
    MotorInit(&CHASSIS.wheel_motor[1], 2, 1, DJI_M3508, 1, 1, 1);
    MotorInit(&CHASSIS.wheel_motor[2], 3, 1, DJI_M3508, 1, 1, 1);
    MotorInit(&CHASSIS.wheel_motor[3], 4, 1, DJI_M3508, 1, 1, 1);

    /*-------------------- 初始化髋关节电机 --------------------*/
    // 第0个髋关节电机（左电机）：ID=0x03，反馈报文=0x53 (DM_M3_ID)
    MotorInit(
        &CHASSIS.hip_motor[0], 0x03, 1, DM_4310, -1, 1.0f,
        DM_MODE_MIT);  // 左电机，方向反向，使用CAN1
    // 第1个髋关节电机（右电机）：ID=0x04，反馈报文=0x54 (DM_M4_ID)
    MotorInit(
        &CHASSIS.hip_motor[1], 0x04, 1, DM_4310, 1, 1.0f,
        DM_MODE_MIT);  // 右电机，方向正向，使用CAN1

    // 初始化髋关节角度
    CHASSIS.hip_reference[0] = CHASSIS_HIP_INIT_ANGLE;
    CHASSIS.hip_reference[1] = CHASSIS_HIP_INIT_ANGLE;
    CHASSIS.hip_angle_rc_set = 0.0f;

    /*-------------------- 初始化超级电容 --------------------*/
    SupCapInit(&CHASSIS.super_cap, 1);  // 初始化超级电容，使用CAN1

    /*-------------------- 初始化功率控制 --------------------*/
    chassis_power_control_init(&CHASSIS);  // 初始化功率控制系统

    // // 发送初始化超级电容控制命令
    // // 初始化时从裁判系统获取缓冲能量数据
    // fp32 init_power, init_buffer;
    // get_chassis_power_and_buffer(&init_power, &init_buffer);
    // CanCmdSupCap(1, (uint16_t)init_buffer, 50, 1);  // 使用裁判系统的实际缓冲能量
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
    // 遥控器下档为安全档（无力模式）
    if (switch_is_down(CHASSIS.rc->rc.s[0])) {
            CHASSIS.mode = CHASSIS_ZERO_FORCE;  // 下档 - 安全档（无力）
       }
        if (switch_is_up(CHASSIS.rc->rc.s[0])) {
           CHASSIS.mode = CHASSIS_SPIN;
        }
		if (switch_is_mid(CHASSIS.rc->rc.s[0])) {
           CHASSIS.mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        }
		
        // 使用键盘 SHIFT 键控制模式
        else if (GetDt7Keyboard(KEY_SHIFT)) {
            CHASSIS.mode = CHASSIS_SPIN;  // 按着 SHIFT 进入小陀螺模式
        } 
    }


/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        GetMotorMeasure(&CHASSIS.wheel_motor[i]);
    }

    // 更新髋关节电机数据
    for (uint8_t i = 0; i < 2; i++) {
        GetMotorMeasure(&CHASSIS.hip_motor[i]);
        // 更新髋关节反馈角度
        CHASSIS.hip_feedback[i] = CHASSIS.hip_motor[i].fdb.pos;
    }

    // 更新超级电容反馈数据
    GetSupCapMeasure(&CHASSIS.super_cap);

    // IMU数据通过指针直接访问，无需拷贝

    // 调试打印两个髋关节的扭矩
    //ModifyDebugDataPackage(4, CHASSIS.hip_motor[0].fdb.tor, "hip0_tor");
    //ModifyDebugDataPackage(5, CHASSIS.hip_motor[1].fdb.tor, "hip1_tor");
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
    fp32 rc_x, rc_y;
    fp32 vx_channel, vy_channel;
    rc_deadband_limit(CHASSIS.rc->rc.ch[3], rc_y, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[2], rc_x, CHASSIS_RC_DEADLINE);

    vx_channel = -rc_x * MAX_SPEED_VECTOR_VX;
    vy_channel = rc_y * MAX_SPEED_VECTOR_VY;

    CHASSIS.dyaw = GetGimbalDeltaYawMid();
    //ModifyDebugDataPackage(3, CHASSIS.dyaw, "dyaw");

    // DM_IMU 欧拉角调试输出
    //ModifyDebugDataPackage(1, CHASSIS.imu_data->euler_z, "imu_yaw");
    //ModifyDebugDataPackage(2, CHASSIS.imu_data->euler_y, "imu_pitch");
    //ModifyDebugDataPackage(3, CHASSIS.imu_data->euler_x, "imu_roll");

    //给定摇杆值(无死区)
    // CHASSIS.vx_rc_set = CHASSIS_VX_RC_SEN * CHASSIS.rc->rc.ch[3];
    // CHASSIS.vy_rc_set = CHASSIS_VY_RC_SEN * CHASSIS.rc->rc.ch[2];

    CHASSIS.vx_rc_set = vx_channel;
    CHASSIS.vy_rc_set = vy_channel;

    //具体模式设定
    switch (CHASSIS.mode) {
        case CHASSIS_ZERO_FORCE: {
            CHASSIS.wz_set = CHASSIA_STOP_SPEED;
            CHASSIS.vx_set = CHASSIA_STOP_SPEED;
            CHASSIS.vy_set = CHASSIA_STOP_SPEED;

            // 髋关节停止控制
            CHASSIS.hip_reference[0] = -0.05;
            CHASSIS.hip_reference[1] = 0.05;
            break;
        }
        case CHASSIS_FOLLOW_GIMBAL_YAW: {  //云台跟随模式，只控制底盘
            // 键盘控制输入
            if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_W) {
                CHASSIS.vy_rc_set = NORMAL_MAX_CHASSIS_SPEED_Y;
            } else if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_S) {
                CHASSIS.vy_rc_set = -NORMAL_MAX_CHASSIS_SPEED_Y;
            }

            if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_A) {
                CHASSIS.vx_rc_set = NORMAL_MAX_CHASSIS_SPEED_X;
            } else if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_D) {
                CHASSIS.vx_rc_set = -NORMAL_MAX_CHASSIS_SPEED_X;
            }

            //乘旋转矩阵 将云台坐标系转换到底盘坐标系
            fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
            sin_yaw = sinf(CHASSIS.dyaw);
            cos_yaw = cosf(CHASSIS.dyaw);

            CHASSIS.vy_set = cos_yaw * CHASSIS.vy_rc_set - sin_yaw * CHASSIS.vx_rc_set;
            CHASSIS.vx_set = sin_yaw * CHASSIS.vy_rc_set + cos_yaw * CHASSIS.vx_rc_set;

            // 云台跟随：底盘追赶云台方向，让dyaw趋于0（底盘正面对准云台指向）
            fp32 dyaw_after_deadband =
                fp32_deadline(CHASSIS.dyaw, -CHASSIS_ANGLE_DEADBAND, CHASSIS_ANGLE_DEADBAND);
            CHASSIS.wz_set =
                PID_calc(&CHASSIS.chassis_angle_pid, dyaw_after_deadband, 0);  //让底盘跟随云台转动
            break;
        }
        case CHASSIS_AUTO: {  //髋关节自动模式（基于IMU陀螺仪保持底盘水平）

            // 底盘跟随云台（与FOLLOW_GIMBAL_YAW相同的轮子控制）
            fp32 sin_yaw_auto = 0.0f, cos_yaw_auto = 0.0f;
            sin_yaw_auto = sinf(CHASSIS.dyaw);
            cos_yaw_auto = cosf(CHASSIS.dyaw);
            CHASSIS.vy_set = cos_yaw_auto * CHASSIS.vy_rc_set - sin_yaw_auto * CHASSIS.vx_rc_set;
            CHASSIS.vx_set = sin_yaw_auto * CHASSIS.vy_rc_set + cos_yaw_auto * CHASSIS.vx_rc_set;

            fp32 dyaw_auto =
                fp32_deadline(CHASSIS.dyaw, -CHASSIS_ANGLE_DEADBAND, CHASSIS_ANGLE_DEADBAND);
            CHASSIS.wz_set = PID_calc(&CHASSIS.chassis_angle_pid, dyaw_auto, 0);

            // 髋关节自动模式：目标pitch角度由IMU控制，Console中计算
            // 此处不需要通过遥控器设置hip_reference
            break;
        }
        case CHASSIS_SPIN: {  //小陀螺模式，可以控制髋关节和底盘
            // 键盘控制输入
            if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_W) {
                CHASSIS.vy_rc_set = NORMAL_MAX_CHASSIS_SPEED_Y;
            } else if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_S) {
                CHASSIS.vy_rc_set = -NORMAL_MAX_CHASSIS_SPEED_Y;
            }

            if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_A) {
                CHASSIS.vx_rc_set = NORMAL_MAX_CHASSIS_SPEED_X;
            } else if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_D) {
                CHASSIS.vx_rc_set = -NORMAL_MAX_CHASSIS_SPEED_X;
            }

            fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
            // 控制vx vy
            sin_yaw = sinf(CHASSIS.dyaw);
            cos_yaw = cosf(CHASSIS.dyaw);
            CHASSIS.vy_set = cos_yaw * CHASSIS.vy_rc_set - sin_yaw * CHASSIS.vx_rc_set;
            CHASSIS.vx_set = sin_yaw * CHASSIS.vy_rc_set + cos_yaw * CHASSIS.vx_rc_set;

            CHASSIS.wz_set = -NORMAL_MAX_CHASSIS_SPEED_WX;
            // 髋关节停止控制
            CHASSIS.hip_reference[0] = -0.05;
            CHASSIS.hip_reference[1] = 0.05;
            break;
        }
        default:
            // 髋关节保持当前位置
            // CHASSIS.hip_reference[0] = CHASSIS.hip_feedback[0];
            // CHASSIS.hip_reference[1] = CHASSIS.hip_feedback[1];
            break;
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    uint8_t i;

    // 图传遥控离线保护：离线时将电流全部置零
    if (GetYkqRcOffline()) {
        for (i = 0; i < 4; i++) {
            CHASSIS.wheel_motor[i].set.curr = CHASSIA_CURR_ZERO;
        }
        // 髋关节电机也停止
        for (i = 0; i < 2; i++) {
            CHASSIS.hip_motor[i].set.vel = 0.0f;
            CHASSIS.hip_motor[i].set.pos = 0.0f;
            CHASSIS.hip_motor[i].set.tor = 0.0f;
        }
        return;
    }

    //麦轮解算
    CHASSIS.wheel_motor[0].set.vel =
        -CHASSIS.vx_set + CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;
    CHASSIS.wheel_motor[1].set.vel =
        CHASSIS.vx_set + CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;
    CHASSIS.wheel_motor[2].set.vel =
        CHASSIS.vx_set - CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;
    CHASSIS.wheel_motor[3].set.vel =
        -CHASSIS.vx_set - CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;

    //pid速度计算
    for (i = 0; i < 4; i++) {
        CHASSIS.wheel_motor[i].set.curr = PID_calc(
            &CHASSIS.motor_speed_pid[i], CHASSIS.wheel_motor[i].fdb.vel,
            CHASSIS.wheel_motor[i].set.vel);
    }

    // 髋关节控制
    if (CHASSIS.mode == CHASSIS_AUTO) {
    } else {
        // 手动模式/其他模式：使用电机位置反馈
        // PID角度环计算期望速度
        CHASSIS.hip_motor[0].set.pos = CHASSIS.hip_reference[0];
        CHASSIS.hip_motor[0].set.vel =
            PID_calc(&CHASSIS.hip_angle_pid[0], CHASSIS.hip_feedback[0], CHASSIS.hip_reference[0]);
        CHASSIS.hip_motor[0].set.tor = 4;
        //CHASSIS.hip_motor[0].set.tor =0;
        CHASSIS.hip_motor[1].set.pos = CHASSIS.hip_reference[1];
        CHASSIS.hip_motor[1].set.vel =
            PID_calc(&CHASSIS.hip_angle_pid[1], CHASSIS.hip_feedback[1], CHASSIS.hip_reference[1]);
        //CHASSIS.hip_motor[1].set.tor =0;
        CHASSIS.hip_motor[1].set.tor = -4;
    }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */

void ChassisSendCmd(void)
{
    // 发送底盘电机控制命令
    //    CanCmdDjiMotor(
    //        1, 0x200, g_chassis_power_control.motor_speed_pid[0]->out,
    //        g_chassis_power_control.motor_speed_pid[1]->out,
    //        g_chassis_power_control.motor_speed_pid[2]->out,
    //        g_chassis_power_control.motor_speed_pid[3]->out);
    CanCmdDjiMotor(
        1, 0x200, CHASSIS.wheel_motor[0].set.curr, CHASSIS.wheel_motor[1].set.curr,
        CHASSIS.wheel_motor[2].set.curr, CHASSIS.wheel_motor[3].set.curr);
    // 发送髋关节电机控制命令
    for (uint8_t i = 0; i < 2; i++) {
        if (CHASSIS.hip_motor[i].type == DM_4310) {
            // 检查电机状态，如果失能则使能电机
            if (CHASSIS.hip_motor[i].fdb.state == DM_STATE_DISABLE) {
                DmEnable(&CHASSIS.hip_motor[i]);
            }
        }
    }
    DmMitCtrl(&CHASSIS.hip_motor[0], CHASSIS_HIP_MIT_KP, CHASSIS_HIP_MIT_KD);
    osDelay(1);
    DmMitCtrl(&CHASSIS.hip_motor[1], CHASSIS_HIP_MIT_KP, CHASSIS_HIP_MIT_KD);
    osDelay(1);
    // // 发送DM IMU欧拉角数据请求
    osDelay(1);
    DM_IMUsend(0x03);  // reg=0x03 请求欧拉角数据
    osDelay(1);
    DM_IMUsend(0x02);  // reg=0x02 请求角速度数据
    //    // // 发送超级电容控制命令
    //    // // 可以根据底盘运动状态动态调整超级电容参数
    //    fp32 current_power, current_buffer;
    //    get_chassis_power_and_buffer(&current_power, &current_buffer);  // 从裁判系统获取实时数据

    //    uint16_t power_buffer = (uint16_t)current_buffer;  // 从裁判系统获取的当前缓冲能量
    //    uint16_t power_limit = 40;                         // 功率限制值，可根据比赛规则调整
    //    uint8_t automode_stage = 0;                        // 启用超级电容

    //    CanCmdSupCap(1, power_buffer, power_limit, automode_stage);
}

#endif  //CHASSIS_OMNI_WHEEL
