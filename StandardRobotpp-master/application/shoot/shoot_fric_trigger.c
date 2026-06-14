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

#include "shoot_fric_trigger.h"

#include "stm32f4xx_hal.h"

#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)

/* TB6612 控制函数声明 */
static void TB6612_Init(void);
static void TB6612_SetMotor(uint8_t direction, uint16_t speed);
static void N20_Control_Task(void);

static Shoot_s SHOOT = {
    .mode = LOAD_STOP,
    .state = FRIC_NOT_READY,
    .fric_flag = 0,
    .move_flag = 0,
    .ecd_count = 0,
    .shoot_flag = 0,
    .heat = 0,
    .heat_limit = 0,
    .n20_state = 0,  // 初始状态为反转
    .n20_forward_start_time = 0,
    .bullet_feed_flag = 0,
};

//static uint32_t last_shoot_time = 0; // 记录上次单发时间(ms)
static uint16_t last_switch_state = RC_SW_MID;  // 记录上次开关状态，初始化为中档
static uint8_t last_mouse_press_l = 0;          // 记录上次鼠标左键状态，初始化为未按下

uint8_t fric_ui;
fp32 delta;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ShootInit(void)
{
    //获取遥控器指针
    SHOOT.rc = get_remote_control_point();

    //TB6612初始化
    TB6612_Init();

    //摩擦轮相关
    MotorInit(
        &SHOOT.fric_motor[0], FRIC_MOTOR_R_ID, FRIC_MOTOR_R_CAN, FRIC_MOTOR_TYPE, 1, 1.0f,
        0);  //初始化R摩擦轮电机结构体
    MotorInit(
        &SHOOT.fric_motor[1], FRIC_MOTOR_L_ID, FRIC_MOTOR_L_CAN, FRIC_MOTOR_TYPE, 1, 1.0f,
        0);  //初始化L摩擦轮电机结构体
    MotorInit(
        &SHOOT.fric_motor[2], FRIC_MOTOR_R1_ID, FRIC_MOTOR_R1_CAN, FRIC_MOTOR_TYPE, 1, 1.0f,
        0);  //初始化R1摩擦轮电机结构体
    MotorInit(
        &SHOOT.fric_motor[3], FRIC_MOTOR_L1_ID, FRIC_MOTOR_L1_CAN, FRIC_MOTOR_TYPE, 1, 1.0f,
        0);  //初始化L1摩擦轮电机结构体

    const fp32 pid_fric[3] = {
        FRIC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FRIC_SPEED_PID_KD};  //摩擦轮速度环

    PID_init(&SHOOT.fric_pid[0], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(
        &SHOOT.fric_pid[1], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT,
        FRIC_PID_MAX_IOUT);  //摩擦轮初始化pid
    PID_init(&SHOOT.fric_pid[2], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&SHOOT.fric_pid[3], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);

    //拨弹盘相关
    MotorInit(
        &SHOOT.trigger_motor, TRIGGER_MOTOR_ID, TRIGGER_MOTOR_CAN, TRIGGER_MOTOR_TYPE, 1, 1.0f,
        0);  //初始化拨弹盘电机结构体
    if (TRIGGER_MOTOR_TYPE == DJI_M2006) {
        const fp32 pid_angel_trigger[3] = {
            TRIGGER_ANGEL_PID_KP, TRIGGER_ANGEL_PID_KI, TRIGGER_ANGEL_PID_KD};  //拨弹盘角度环
        const fp32 pid_speed_trigger[3] = {
            TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};  //拨弹盘速度环

        PID_init(
            &SHOOT.trigger_angel_pid, PID_POSITION, pid_angel_trigger, TRIGGER_ANGEL_PID_MAX_OUT,
            TRIGGER_ANGEL_PID_MAX_IOUT);
        PID_init(
            &SHOOT.trigger_speed_pid, PID_POSITION, pid_speed_trigger, TRIGGER_SPEED_PID_MAX_OUT,
            TRIGGER_SPEED_PID_MAX_IOUT);  //拨弹盘初始化pid
    } else if (TRIGGER_MOTOR_TYPE == DM_4310) {
        const fp32 pid_angel_trigger[3] = {
            TRIGGER_ANGEL_PID_KP, TRIGGER_ANGEL_PID_KI, TRIGGER_ANGEL_PID_KD};  //拨弹盘角度环

        PID_init(
            &SHOOT.trigger_angel_pid, PID_POSITION, pid_angel_trigger, TRIGGER_ANGEL_PID_MAX_OUT,
            TRIGGER_ANGEL_PID_MAX_IOUT);  //拨弹盘初始化pid
    }
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ShootSetMode(void)
{
    /*键鼠遥控器控制方式初版----------------------------*/
    if (switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))  //上档防止误触
    {
        // rc.s[0]遥控器开关边沿检测：检测从中档到上档的跳变
        uint16_t current_switch_s0 = SHOOT.rc->rc.s[0];
        bool_t switch_edge_trigger =
            (last_switch_state == RC_SW_MID) && (current_switch_s0 == RC_SW_UP);

        // 如果是从中档→上档的跳变，保持摩擦轮开启并继续拨弹
        if (switch_edge_trigger && SHOOT.move_flag == 0) {
            SHOOT.state = FRIC_READY;  // 保持摩擦轮开启
            SHOOT.mode = LAOD_BULLET;  // 保持拨弹模式
            SHOOT.shoot_flag = 1;      // 设置拨弹标志
        } else {
            // 其他情况下正常上档逻辑
            SHOOT.state = FRIC_NOT_READY;
            SHOOT.mode = LOAD_STOP;
        }

        // 更新遥控器开关状态
        last_switch_state = current_switch_s0;
    }

    else if (switch_is_mid(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])) {
        // 摩擦轮控制
        if (SHOOT.rc->key.v & KEY_PRESSED_OFFSET_Q)  //Q启动摩擦轮
        {
            SHOOT.fric_flag = 1;
        } else if (SHOOT.rc->key.v & KEY_PRESSED_OFFSET_E)  //E关闭摩擦轮
        {
            SHOOT.fric_flag = 0;
        }
        SHOOT.state = FRIC_READY;

        // 中档一直保持LAOD_BULLET模式
        SHOOT.mode = LAOD_BULLET;

        // 鼠标左键边沿检测：检测从未按下到按下的跳变
        uint8_t current_mouse_press_l = SHOOT.rc->mouse.press_l;
        bool_t mouse_edge_trigger = (last_mouse_press_l == 0) && (current_mouse_press_l == 1);

        // 只用鼠标左键控制拨弹标志
        if (mouse_edge_trigger && SHOOT.move_flag == 0) {
            SHOOT.shoot_flag = 1;  // 设置拨弹标志
        }

        // 更新鼠标状态
        last_mouse_press_l = current_mouse_press_l;

        // rc.s[0]遥控器开关边沿检测：检测从中档到上档的跳变
        uint16_t current_switch_s0 = SHOOT.rc->rc.s[0];
        bool_t switch_edge_trigger =
            (last_switch_state == RC_SW_MID) && (current_switch_s0 == RC_SW_UP);

        // 遥控器开关从中档到上档触发拨弹
        if (switch_edge_trigger && SHOOT.move_flag == 0) {
            SHOOT.shoot_flag = 1;  // 设置拨弹标志
        }

        // 更新遥控器开关状态
        last_switch_state = current_switch_s0;

    } else if (switch_is_down(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])) {
        SHOOT.state = FRIC_NOT_READY;
        SHOOT.mode = LOAD_STOP;
        // 更新遥控器开关状态
        last_switch_state = SHOOT.rc->rc.s[0];

        //上位机测试
        // SHOOT.state = FRIC_READY;

        // if (GetScCmdFire())
        // {
        //   SHOOT.mode = LOAD_BURSTFIRE;
        // }
        // else
        // {
        //   SHOOT.mode = LOAD_STOP;
        // }
    }

    /*
 //防堵转
    if (SHOOT.mode == LOAD_BURSTFIRE||SHOOT.mode == LAOD_BULLET)
    {
      if(SHOOT.block_time >= BLOCK_TIME)
      {
        SHOOT.mode = LOAD_BLOCK;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
      }
  
      if(fabs(SHOOT.last_trigger_vel)<BLOCK_TRIGGER_SPEED&&SHOOT.block_time<BLOCK_TIME)
      {
        SHOOT.block_time++;
        SHOOT.reverse_time = 0;
      }
      else if(SHOOT.block_time== BLOCK_TIME&& SHOOT.reverse_time< REVERSE_TIME)
      {
        SHOOT.reverse_time++;  
      }
      else
      {
        SHOOT.block_time = 0;
      }
      
    }

    //过热保护
    if (fabs(SHOOT.last_fric_vel) < FRIC_SPEED_LIMIT)
    {
      SHOOT.mode = LOAD_STOP;
      fric_ui = 0;
    }
    else
    {
      fric_ui = 1;
    }
    
    //热量限制
    if (TRIGGER_MOTOR_TYPE == DJI_M2006)
    {
      get_shoot_heat0_limit_and_heat0(&SHOOT.heat_limit, &SHOOT.heat);
    }
    else if (TRIGGER_MOTOR_TYPE == DM_4310)
    {
      get_shoot_heat42_limit_and_heat42(&SHOOT.heat_limit, &SHOOT.heat);
    }

    if ((SHOOT.heat + SHOOT_HEAT_REMAIN_VALUE) > SHOOT.heat_limit)
    {
      SHOOT.mode = LOAD_STOP;
    }



    */

    //安全档
    /*
        if ((switch_is_down(SHOOT.rc->rc.s[0])))
    {
      SHOOT.mode = LOAD_STOP;
      SHOOT.state = FRIC_NOT_READY;
    }
    
    */

    //遥控器离线保护
    if (toe_is_error(DBUS_TOE)) {
        SHOOT.state = FRIC_NOT_READY;
        SHOOT.mode = LOAD_STOP;
    }
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ShootObserver(void)
{
    GetMotorMeasure(&SHOOT.trigger_motor);
    GetMotorMeasure(&SHOOT.fric_motor[0]);
    GetMotorMeasure(&SHOOT.fric_motor[1]);
    GetMotorMeasure(&SHOOT.fric_motor[2]);
    GetMotorMeasure(&SHOOT.fric_motor[3]);

    SHOOT.FDB.fric_speed_fdb_R = SHOOT.fric_motor[0].fdb.vel;
    SHOOT.FDB.fric_speed_fdb_L = SHOOT.fric_motor[1].fdb.vel;
    SHOOT.FDB.fric_speed_fdb_R1 = SHOOT.fric_motor[2].fdb.vel;
    SHOOT.FDB.fric_speed_fdb_L1 = SHOOT.fric_motor[3].fdb.vel;

    SHOOT.FDB.trigger_speed_fdb = SHOOT.trigger_motor.fdb.vel;

    if (TRIGGER_MOTOR_TYPE == DJI_M2006) {
        if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE) {
            SHOOT.ecd_count--;
        } else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE) {
            SHOOT.ecd_count++;
        }

        if (SHOOT.ecd_count == FULL_COUNT) {
            SHOOT.ecd_count = -(FULL_COUNT - 1);
        } else if (SHOOT.ecd_count == -FULL_COUNT) {
            SHOOT.ecd_count = FULL_COUNT - 1;
        }
        //计算输出轴角度
        SHOOT.FDB.trigger_angel_fdb =
            (SHOOT.ecd_count * ECD_RANGE + SHOOT.trigger_motor.fdb.ecd) * MOTOR_ECD_TO_ANGLE;

        //记录上一个ecd值
        SHOOT.last_ecd = SHOOT.trigger_motor.fdb.ecd;

        //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
        //if(FULL_COUNT%2 == 0)
        //{
        // if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE)
        // {
        //     SHOOT.ecd_count--;
        // }
        // else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE)
        // {

        //     SHOOT.ecd_count++;
        // }

        // if (SHOOT.ecd_count == FULL_COUNT)
        // {
        //     SHOOT.ecd_count = -(FULL_COUNT - 1);
        // }
        // else if (SHOOT.ecd_count == -FULL_COUNT)
        // {
        //     SHOOT.ecd_count = FULL_COUNT-1;
        // }
        //}
        // //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 51圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
        // else
        // {
        //   if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE)
        //   {
        //       SHOOT.ecd_count--;
        //   }
        //   else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE)
        //   {

        //       SHOOT.ecd_count++;
        //   }

        //   if (SHOOT.ecd_count == FULL_COUNT)
        //   {
        //       SHOOT.ecd_count = -FULL_COUNT;
        //   }
        //   else if (SHOOT.ecd_count == -FULL_COUNT)
        //   {
        //       SHOOT.ecd_count = FULL_COUNT;
        //   }
        //}

    } else if (TRIGGER_MOTOR_TYPE == DM_4310) {
        SHOOT.FDB.trigger_angel_fdb = theta_format(SHOOT.trigger_motor.fdb.pos);
    }

    //记录上一个拨弹盘vel,用于堵转模式判断
    SHOOT.last_trigger_vel = SHOOT.trigger_motor.fdb.vel;

    //记录上一个摩擦轮vel,用于过热保护
    SHOOT.last_fric_vel = SHOOT.fric_motor[0].fdb.vel;
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ShootReference(void)
{
    // 在函数开始时进行全局的开关状态检测
    uint16_t current_switch_state = SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL_TRG];
    bool_t switch_edge_trigger =
        (last_switch_state == RC_SW_DOWN) && (current_switch_state == RC_SW_MID);
    last_switch_state = current_switch_state;

    switch (SHOOT.state) {
        case FRIC_NOT_READY:
            SHOOT.REF.fric_speed_ref_R = 0.0f;
            SHOOT.REF.fric_speed_ref_L = 0.0f;
            SHOOT.REF.fric_speed_ref_R1 = 0.0f;
            SHOOT.REF.fric_speed_ref_L1 = 0.0f;
            break;

        case FRIC_READY:
            //后
            SHOOT.REF.fric_speed_ref_R = FRIC_R_SPEED;
            SHOOT.REF.fric_speed_ref_L = FRIC_L_SPEED;
            //前
            SHOOT.REF.fric_speed_ref_R1 = FRIC_R_1_SPEED;
            SHOOT.REF.fric_speed_ref_L1 = FRIC_L_1_SPEED;
            break;

        default:
            break;
    }

    switch (SHOOT.mode) {
        case LOAD_STOP:
            //SHOOT.REF.trigger_speed_ref = 0.0f;
            // 在STOP模式下，设置目标位置为当前位置，
            SHOOT.REF.trigger_angel_ref = SHOOT.FDB.trigger_angel_fdb;
            break;

        case LAOD_BULLET:
            if (TRIGGER_MOTOR_TYPE == DJI_M2006) {
                if (SHOOT.move_flag == 0) {
                    SHOOT.REF.trigger_angel_ref = theta_format(
                        SHOOT.FDB.trigger_angel_fdb +
                        2 * PI / BULLET_NUM / TRIGGER_REDUCTION_RATIO);

                    // N20电机拨弹控制：当触发单发时同时控制N20
                    if (!SHOOT.bullet_feed_flag) {
                        SHOOT.bullet_feed_flag = 1;
                        SHOOT.n20_state = 1;  // 正转
                        SHOOT.n20_forward_start_time = HAL_GetTick();
                    }
                }

                if (theta_format(SHOOT.REF.trigger_angel_ref - SHOOT.FDB.trigger_angel_fdb) >
                    0.01f) {
                    SHOOT.move_flag = 1;
                } else {
                    SHOOT.move_flag = 0;
                }
            } else if (TRIGGER_MOTOR_TYPE == DM_4310) {
                // 使用全局的边沿检测结果或鼠标触发单发
                bool_t trigger_fire = switch_edge_trigger || SHOOT.shoot_flag;

                if (SHOOT.move_flag == 0 && trigger_fire) {
                    SHOOT.REF.trigger_angel_ref =
                        theta_format(SHOOT.FDB.trigger_angel_fdb - 2 * PI / BULLET_NUM);
                    // SHOOT.REF.trigger_angel_ref =
                    //     SHOOT.FDB.trigger_angel_fdb -
                    //     2 * PI / BULLET_NUM * TRIGGER_REDUCTION_RATIO;

                    // N20电机拨弹控制：下档切换到中档时触发
                    if (!SHOOT.bullet_feed_flag) {
                        SHOOT.bullet_feed_flag = 1;
                        SHOOT.n20_state = 1;  // 正转
                        SHOOT.n20_forward_start_time = HAL_GetTick();
                    }

                    // 清除拨弹标志，允许下次拨弹
                    SHOOT.shoot_flag = 0;
                }

                if (theta_format(SHOOT.FDB.trigger_angel_fdb - SHOOT.REF.trigger_angel_ref) >
                    0.01f) {
                    SHOOT.move_flag = 1;
                } else {
                    SHOOT.move_flag = 0;
                }
            }
            break;

        case LOAD_BURSTFIRE:
            SHOOT.REF.trigger_speed_ref = TRIGGER_SPEED;
            break;

        case LOAD_BLOCK:
            SHOOT.REF.trigger_speed_ref = REVERSE_SPEED;
            break;

        default:
            break;
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ShootConsole(void)
{
    SHOOT.fric_motor[0].set.curr =
        PID_calc(&SHOOT.fric_pid[0], SHOOT.FDB.fric_speed_fdb_R, SHOOT.REF.fric_speed_ref_R);
    SHOOT.fric_motor[1].set.curr =
        PID_calc(&SHOOT.fric_pid[1], SHOOT.FDB.fric_speed_fdb_L, SHOOT.REF.fric_speed_ref_L);
    SHOOT.fric_motor[2].set.curr =
        PID_calc(&SHOOT.fric_pid[2], SHOOT.FDB.fric_speed_fdb_R1, SHOOT.REF.fric_speed_ref_R1);
    SHOOT.fric_motor[3].set.curr =
        PID_calc(&SHOOT.fric_pid[3], SHOOT.FDB.fric_speed_fdb_L1, SHOOT.REF.fric_speed_ref_L1);

    if (TRIGGER_MOTOR_TYPE == DM_4310) {
        if (SHOOT.mode == LOAD_STOP) {
            delta = theta_format(SHOOT.REF.trigger_angel_ref - SHOOT.FDB.trigger_angel_fdb);
            SHOOT.trigger_motor.set.vel = PID_calc(&SHOOT.trigger_angel_pid, 0, delta);
        } else if (SHOOT.mode == LOAD_BURSTFIRE) {
            SHOOT.trigger_motor.set.vel = SHOOT.REF.trigger_speed_ref;
        } else if (SHOOT.mode == LAOD_BULLET) {
            //这么使电机转的又快 同时扭矩加大
            delta = theta_format(SHOOT.REF.trigger_angel_ref - SHOOT.FDB.trigger_angel_fdb);
            //SHOOT.trigger_motor.set.pos =SHOOT.REF.trigger_angel_ref;
            SHOOT.trigger_motor.set.vel = PID_calc(&SHOOT.trigger_angel_pid, 0, delta);
            //SHOOT.trigger_motor.set.vel =0;
            SHOOT.trigger_motor.set.tor = 15;
        } else if (SHOOT.mode == LOAD_BLOCK) {
            SHOOT.trigger_motor.set.vel = SHOOT.REF.trigger_speed_ref;
        }
    }

    // N20电机控制任务
    N20_Control_Task();
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ShootSendCmd(void)
{
    if (TRIGGER_MOTOR_TYPE == DJI_M2006) {
        CanCmdDjiMotor(
            FRIC_MOTOR_R_CAN, STD_ID, SHOOT.fric_motor[0].set.curr, SHOOT.fric_motor[1].set.curr,
            SHOOT.fric_motor[2].set.curr, SHOOT.fric_motor[3].set.curr);

    } else if (TRIGGER_MOTOR_TYPE == DM_4310) {
        DmEnable(&SHOOT.trigger_motor);
        //DmMitCtrlPosition(&SHOOT.trigger_motor, TRIGGER_SPEED_MIT_KP,TRIGGER_SPEED_MIT_KD);
        DmMitCtrlVelocity(&SHOOT.trigger_motor, TRIGGER_SPEED_MIT_KD);
        CanCmdDjiMotor(
            FRIC_MOTOR_R_CAN, STD_ID, SHOOT.fric_motor[0].set.curr, SHOOT.fric_motor[1].set.curr,
            SHOOT.fric_motor[2].set.curr, SHOOT.fric_motor[3].set.curr);
    }
}

/*-------------------- TB6612 N20电机控制函数 --------------------*/

/**
 * @brief          TB6612初始化
 * @param[in]      none
 * @retval         none
 */
static void TB6612_Init(void)
{
    // 启用TB6612 (STBY = 1)
    HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);

    // 初始状态：反转
    TB6612_SetMotor(0, TB6612_REVERSE_SPEED);
}

/**
 * @brief          设置TB6612电机转向和速度
 * @param[in]      direction: 0=反转(CCW), 1=正转(CW)
 * @param[in]      speed: PWM占空比值 (0-TB6612_PWM_ARR)
 * @retval         none
 */
static void TB6612_SetMotor(uint8_t direction, uint16_t speed)
{
    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);

    if (direction == 0) {
        // 反转: BIN1=0, BIN2=1
        HAL_GPIO_WritePin(TB6612_BIN1_GPIO_Port, TB6612_BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TB6612_BIN2_GPIO_Port, TB6612_BIN2_Pin, GPIO_PIN_SET);
    } else {
        // 正转: BIN1=1, BIN2=0
        HAL_GPIO_WritePin(TB6612_BIN1_GPIO_Port, TB6612_BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TB6612_BIN2_GPIO_Port, TB6612_BIN2_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief          N20电机控制任务
 * @param[in]      none
 * @retval         none
 */
static void N20_Control_Task(void)
{
    uint32_t current_time = HAL_GetTick();

    if (SHOOT.bullet_feed_flag) {
        // 拨弹模式
        if (SHOOT.n20_state == 1) {
            // 正转状态
            if (current_time - SHOOT.n20_forward_start_time >= 450) {
                // 正转0.5秒后回到反转
                SHOOT.n20_state = 0;
                SHOOT.bullet_feed_flag = 0;
                TB6612_SetMotor(0, TB6612_REVERSE_SPEED);
            } else {
                // 继续正转
                TB6612_SetMotor(1, TB6612_FORWARD_SPEED);
            }
        }
    } else {
        // 非拨弹状态，保持反转
        TB6612_SetMotor(0, TB6612_REVERSE_SPEED);
    }
}

#endif  // SHOOT_TYPE == SHOOT_FRIC
