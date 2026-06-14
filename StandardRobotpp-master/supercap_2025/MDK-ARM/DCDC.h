#ifndef DCDC
#define DCDC

#include "stm32g4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "arm_math.h"
#include "tim.h"
#include "PID.h"

#define TURNOFF 0
#define OUTPUT 1
#define CHARGE 2

typedef struct
{
	float u[2];
	float y[2];
}ChargeLoop;

typedef struct
{
	float integral;
}OutPutPI;

typedef struct
{
	float targetCurrent;
	float current;	
	float error;
	float output;
	ChargeLoop chargeLoop;
	OutPutPI OutPutPi;
}CurrentLoop;

typedef struct
{
	float BUSCurrent;
	float BUSVoltage;
	float CapVoltage;
	float BUSPower;
	float MAXpower;
}DCDCStage;

typedef struct
{
	uint8_t stage;//自动模式 0关闭 1打开
	uint8_t lastStage;
	float powerLimit;
	float powerBuffer;
	PID powerPID;
	PID Bus_currentPID;
	PID ChargePID;
}AutoMode;

typedef struct
{
	uint8_t mode;// 环路模式 0关闭 1 放电 2充电 3死亡 不作为对外接口
	uint8_t lastMode;
	AutoMode autoMode;//自动模式 0关闭 1打开 作为对外接口
	
}DCDCMode;


void Cap_Init();
void DCDC_CALLBACK();







#endif