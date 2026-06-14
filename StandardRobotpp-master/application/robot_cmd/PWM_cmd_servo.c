/**
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  * @file       PWM_cmd_servo.c/h
  * @brief      PWM发送函数，通过PWM信号控制舵机.
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     2025/2/11       YZX             1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  */

#include "PWM_cmd_servo.h"

#include "bsp_pwm.h"
#include "main.h"

#define SERVO_MIN_PWM 500
#define SERVO_MAX_PWM 2500
/*-------------------- Public functions --------------------*/

/**
   * @brief          通过PWM发送PWM信号控制舵机
   * @param[in]      pump_id 舵机ID
   * @param[in]      pwm pwm信号占空比
   * @retval         none
   */
void PwmCmdServo(uint8_t pump_id, uint16_t pwm)
{
    if (pwm > SERVO_MIN_PWM) {
        pwm = SERVO_MAX_PWM;
    } else if (pwm < SERVO_MIN_PWM) {
        pwm = SERVO_MIN_PWM;
    }

    servo_pwm_set(pwm, pump_id);
}
