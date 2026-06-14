# 达妙4310（DM_4310）电机参数完整配置文档

## 1. 电机基本信息

- **电机类型枚举值**: `DM_4310` (在 `robot_typedef.h` 中定义)
- **电机支持的控制模式**:
  - MIT模式（混合控制）：位置+速度+扭矩混合控制
  - 位置模式
  - 速度模式
  - 位置速度模式

## 2. MIT模式参数范围（标准定义）

所有参数定义位于 [motor.h](application/robot_cmd/motor.h) 中：

### 位置参数（Position）
```c
#define DM_P_MIN   -12.5f    // 最小位置 (rad)
#define DM_P_MAX    12.5f    // 最大位置 (rad)
```

### 速度参数（Velocity）
```c
#define DM_V_MIN   -30.0f    // 最小速度 (rad/s)
#define DM_V_MAX    30.0f    // 最大速度 (rad/s)
```

### 比例系数（Kp）- 位置刚度
```c
#define DM_KP_MIN   0.0f     // 最小Kp
#define DM_KP_MAX   500.0f   // 最大Kp
```

### 微分系数（Kd）- 位置阻尼
```c
#define DM_KD_MIN   0.0f     // 最小Kd
#define DM_KD_MAX   5.0f     // 最大Kd
```

### 扭矩参数（Torque）
```c
#define DM_T_MIN   -10.0f    // 最小扭矩 (Nm)
#define DM_T_MAX    10.0f    // 最大扭矩 (Nm)
```

### 反馈扭矩范围
根据CAN接收处理 [CAN_receive.c:93](application/robot_cmd/CAN_receive.c#L93)：
- 反馈扭矩范围: `-18.0 ~ 18.0` Nm （12位精度）

## 3. CAN通信模式定义

位于 [motor.h](application/robot_cmd/motor.h) 中：

```c
#define DM_MODE_MIT      0x000  // MIT模式
#define DM_MODE_POS      0x100  // 位置模式
#define DM_MODE_SPEED    0x200  // 速度模式
#define DM_MODE_POSI     0x300  // 位置模式（另一种）
```

## 4. 电机状态定义

```c
#define DM_STATE_DISABLE                0x00  // 禁用
#define DM_STATE_ENABLE                 0x01  // 使能
#define DM_STATE_OVERVOLTAGE            0x08  // 过压
#define DM_STATE_UNDERVOLTAGE           0x09  // 欠压
#define DM_STATE_OVERCURRENT            0x0A  // 过流
#define DM_STATE_MOS_OVER_TEMPERATURE   0x0B  // MOS管过温
#define DM_STATE_COIL_OVER_TEMPERATURE  0x0C  // 线圈过温
#define DM_STATE_COMMUNICATION_LOSS     0x0D  // 通信丢失
#define DM_STATE_OVERLOAD               0x0E  // 过载
```

## 5. CAN反馈数据解析（DmFdbData）

根据 [CAN_receive.c:85-96](application/robot_cmd/CAN_receive.c#L85-L96)：

### 反馈帧结构
```c
rx_data[0] = ID(4bit) + State(4bit)
rx_data[1:2] = Position (16bit) -> 范围: -12.5~12.5 rad
rx_data[3:4(高4bit)] = Velocity (12bit) -> 范围: -30.0~30.0 rad/s
rx_data[4(低4bit):5] = Torque (12bit) -> 范围: -18.0~18.0 Nm
rx_data[6] = T_MOS (温度)
rx_data[7] = T_Rotor (温度)
```

## 6. MIT控制函数签名

位于 [CAN_cmd_damiao.h](application/robot_cmd/CAN_cmd_damiao.h) 中：

```c
// MIT混合控制（位置+速度+扭矩）
void DmMitCtrl(Motor_s * motor, float kp, float kd);

// 纯扭矩控制
void DmMitCtrlTorque(Motor_s * motor);

// 速度控制
void DmMitCtrlVelocity(Motor_s * motor, float kd);

// 位置控制
void DmMitCtrlPosition(Motor_s * motor, float kp, float kd);
```

### 控制函数实现
[CAN_cmd_damiao.c:149-178](application/robot_cmd/CAN_cmd_damiao.c#L149-L178)

MIT模式控制帧构成：
- 位置目标 (16bit)
- 速度目标 (12bit)
- Kp值 (12bit)
- Kd值 (12bit)
- 扭矩给定 (12bit)

## 7. 实机应用配置示例

### 7.1 英雄机器人（拨弹盘用）
[robot_param_mecannum_hero.h](application/robot_param_mecannum_hero.h)
```c
// 拨弹盘电机
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DM_4310)
#define TRIGGER_MOTOR_ID 5
#define TRIGGER_MOTOR_CAN 1

// MIT控制参数
#define TRIGGER_SPEED_MIT_KD (1.40f)

// 拨弹轮电机PID速度环
#define TRIGGER_SPEED_PID_KP (54.0f)
#define TRIGGER_SPEED_PID_KI (1.7f)
#define TRIGGER_SPEED_PID_KD (6.2f)
#define TRIGGER_SPEED_PID_MAX_OUT (16000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (15000.0f)

// 拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (27.0f)
#define TRIGGER_ANGEL_PID_KI (0.0f)
#define TRIGGER_ANGEL_PID_KD (3.0f)
#define TRIGGER_ANGEL_PID_MAX_OUT (15000.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (1000.0f)
```

### 7.2 英雄机器人（云台用）
[robot_param_mecannum_hero.h:130-131](application/robot_param_mecannum_hero.h#L130-L131)
```c
// Yaw轴和Pitch轴均采用DM_4310
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DM_4310)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DM_4310)

// Pitch轴MIT参数
#define GIMBAL_DM4310_MIT_KP (0.0f)
#define GIMBAL_DM4310_MIT_KI (0.0f)
#define GIMBAL_DM4310_MIT_KD (8.0f)

// Pitch轴PID参数（角度环）
#define KP_GIMBAL_PITCH_ANGLE (43.0f)
#define KI_GIMBAL_PITCH_ANGLE (0.00f)
#define KD_GIMBAL_PITCH_ANGLE (12.0f)

// Yaw轴PID参数（角度环）
#define KP_GIMBAL_YAW_ANGLE (20.0f)
#define KI_GIMBAL_YAW_ANGLE (0.0f)
#define KD_GIMBAL_YAW_ANGLE (1.0f)
```

### 7.3 工程机器人（机械臂关节0用）
[robot_param_engineer.h:47](application/robot_param_engineer.h#L47)
```c
#define JOINT_MOTOR_0_TYPE DM_4310
#define JOINT_MOTOR_0_MODE DM_MODE_MIT
#define JOINT_MOTOR_0_ID 1
#define JOINT_MOTOR_0_CAN 2
#define JOINT_MOTOR_0_REDUCATION_RATIO 2.24f  // 减速比

// 角度环PID参数
#define KP_JOINT_0_ANGLE 10.0f
#define KI_JOINT_0_ANGLE 0.1f
#define KD_JOINT_0_ANGLE 0.0f
#define MAX_IOUT_JOINT_0_ANGLE 0.3f
#define MAX_OUT_JOINT_0_ANGLE 5.0f

// 位置限幅
#define MAX_JOINT_0_POSITION  M_PI * 3 / 2     // 上限
#define MIN_JOINT_0_POSITION -MAX_JOINT_0_POSITION  // 下限
```

### 7.4 英雄机器人（髋关节用）
[robot_param_mecannum_hero.h:293-297](application/robot_param_mecannum_hero.h#L293-L297)
```c
#define CHASSIS_HIP_MOTOR_TYPE ((MotorType_e)DM_4310)

// MIT模式参数
#define CHASSIS_HIP_MIT_KP (49.0f)
#define CHASSIS_HIP_MIT_KD (75.0f)

// 角度环PID参数
#define KP_CHASSIS_HIP_ANGLE 15.0f
#define KI_CHASSIS_HIP_ANGLE 2.0f
#define KD_CHASSIS_HIP_ANGLE 0.0f
```

## 8. 代码中的DM_4310使用位置

### 8.1 条件编译检查
- [shoot_fric_trigger.c:109](application/shoot/shoot_fric_trigger.c#L109) - 拨弹盘控制
- [shoot_fric_trigger.c:296](application/shoot/shoot_fric_trigger.c#L296) - 拨弹盘反馈处理
- [shoot_fric_trigger.c:413](application/shoot/shoot_fric_trigger.c#L413) - 拨弹盘角度反馈
- [gimbal_yaw_pitch_direct.c:402-413](application/gimbal/gimbal_yaw_pitch_direct.c#L402-L413) - 云台控制
- [chassis_mecanum.c:121-125](application/chassis/chassis_mecanum.c#L121-L125) - 髋关节

### 8.2 MIT控制调用示例
[shoot_fric_trigger.c:576](application/shoot/shoot_fric_trigger.c#L576)
```c
DmMitCtrlVelocity(&SHOOT.trigger_motor, TRIGGER_SPEED_MIT_KD);
```

## 9. 参数单位说明

| 参数 | 单位 | 测量范围 | 控制范围 | 说明 |
|------|------|---------|---------|------|
| 位置 (pos) | rad | -12.5~12.5 | -12.5~12.5 | 单圈位置 |
| 速度 (vel) | rad/s | -30.0~30.0 | -30.0~30.0 | 角速度 |
| 扭矩 (tor) | Nm | -18.0~18.0 | -10.0~10.0 | 控制扭矩上限为±10Nm |
| Kp | - | 0.0~500.0 | 0.0~500.0 | 位置刚度系数 |
| Kd | - | 0.0~5.0 | 0.0~5.0 | 位置阻尼系数 |

## 10. 重要提示

1. **扭矩限制**: 控制时扭矩给定范围为 ±10Nm，但反馈扭矩范围可达 ±18Nm
2. **MIT模式**: 支持位置、速度、扭矩的同时给定，电机内部计算合成控制
3. **CAN通信**: 使用16位位置编码、12位速度和扭矩编码
4. **减速比**: 不同应用中减速比不同（工程臂为2.24:1，其他为1:1）
5. **反馈更新**: 位置从编码器反馈，速度和扭矩通过状态估计得到

## 11. 相关文件清单

- [motor.h](application/robot_cmd/motor.h) - 电机参数定义
- [CAN_cmd_damiao.h](application/robot_cmd/CAN_cmd_damiao.h) - 达妙电机控制函数声明
- [CAN_cmd_damiao.c](application/robot_cmd/CAN_cmd_damiao.c) - 达妙电机控制实现
- [CAN_receive.c](application/robot_cmd/CAN_receive.c) - CAN反馈数据解析
- [robot_param_mecannum_hero.h](application/robot_param_mecannum_hero.h) - 英雄机器人参数
- [robot_param_engineer.h](application/robot_param_engineer.h) - 工程机器人参数
- [shoot_fric_trigger.c](application/shoot/shoot_fric_trigger.c) - 拨弹盘控制代码
- [gimbal_yaw_pitch_direct.c](application/gimbal/gimbal_yaw_pitch_direct.c) - 云台控制代码
