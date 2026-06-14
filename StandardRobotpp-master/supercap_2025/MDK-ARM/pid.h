#ifndef _USER_PID_H_
#define _USER_PID_H_

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#ifndef ABS
#define ABS(x) ((x)>=0?(x):-(x))
#endif

typedef struct _PID
{
	float kp,ki,kd;
	float error,lastError;//误差、上次误差
	float integral,maxIntegral;//积分、积分限幅
	float output,maxOutput,minOutput;//输出、输出限幅
	float deadzone;//死区
}PID;

void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut,float minOut);
void PID_SingleCalc(PID *pid,float reference,float feedback);
void PID_Clear(PID *pid);
void PID_SetDeadzone(PID *pid,float deadzone);





























#endif
