
#include "PID.h"

//初始化pid参数
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut,float minOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->maxIntegral=maxI;
	pid->maxOutput=maxOut;
	pid->minOutput=minOut;
	pid->deadzone=0;
}

//单级pid计算
void PID_SingleCalc(PID *pid,float reference,float feedback)
{
	//更新数据
	pid->lastError=pid->error;
	if(ABS(reference-feedback) < pid->deadzone)//若误差在死区内则error直接置0
		pid->error=0;
	
	else
	{
		if(reference-feedback>pid->deadzone)
			pid->error=reference-feedback-pid->deadzone;
		if(reference-feedback<-pid->deadzone)
			pid->error=reference-feedback+pid->deadzone;		
			//计算微分
			pid->output=(pid->error-pid->lastError)*pid->kd;
			//计算比例
			pid->output+=pid->error*pid->kp;
			//计算积分
			pid->integral+=pid->error*pid->ki;
			LIMIT(pid->integral,-pid->maxIntegral,pid->maxIntegral);//积分限幅
			pid->output+=pid->integral;
			//输出限幅
			LIMIT(pid->output,-pid->minOutput,pid->maxOutput);
	}
}

//清空一个pid的历史数据
void PID_Clear(PID *pid)
{
	pid->error=0;
	pid->lastError=0;
	pid->integral=0;
	pid->output=0;
}

//设置PID死区
void PID_SetDeadzone(PID *pid,float deadzone)
{
	pid->deadzone=deadzone;
}
