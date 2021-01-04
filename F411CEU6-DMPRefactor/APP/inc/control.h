/**
  ******************************************************************************
  * 文件名程: control.h
  * 作    者: Jason_xy
  * 个人博客：https://jason-xy.cn
  * 版    本: V1.0
  * 编写日期: 2020-12-15
  * 功    能: PID控制
  ******************************************************************************
  * 说明：
  * 1.需要提前配置好gy-86。
  * 2.需要提前配置好电机。
  * 
  * 功能：
  * 1.角度环控制。
  * 2.利用重力校准。
  ******************************************************************************
  */

#ifndef __CONTROL_H__ 
#define __CONTROL_H__
#include "math.h"
#include "gy-86.h"
#include "motor.h"
//#define M_PI 3.14159

#define CAR_ANGLE_SET   0
#define CAR_ANGLE_SPEED_SET 0
#define CAR_SPEED_SET         CarSpeedSet
#define CAR_POSITION_MAX	700
#define CAR_POSITION_MIN	(-900) 
#define SPEED_CONTROL_PERIOD	 25	    //速度环控制周期

extern unsigned char SpeedControlCount;
extern unsigned char SpeedControlPeriod;

//PID结构体
typedef struct {
    float P;
    float I;
    float D;
}PID;

//PID参数
extern PID AngleRingPID;
extern PID SpeedRingPID;
extern PID MotorRingPID[4];

//角度控制参数
extern float GyroAngleSpeed;
extern float GyroAngleSpeedBefore;
extern float GyroAccle;
extern float CarAngle;
extern float CarAngleOld;

//电机控制参数
extern float SpeedControlOut[4];
extern float SpeedControlOutOld[4];
extern float SpeedControlOutNew[4];
extern float AngleControlOut;
extern float MotorOut[4];
extern int SpeedDirection;

//速度控制参数
extern short  MotorPulse[4];
extern int  MotorPulseOld[4];
extern int  MotorPulseSigma[4];

extern float CarSpeed[4];
extern float CarSpeedSet;
extern float CarSpeedOld[4];
extern float CarPosition[4];

//电机脉冲捕获
extern uint8_t CapFlag[4];
extern uint32_t CapVal[4];																	
extern float SpeedOfWheel[4];    //°/s
extern int cap;

//状态标记
extern int status;

float Scale(float input, float inputMin, float inputMax, float outputMin, float outputMax);
void AngleCalculate(void);
void AngleControl(void);
void MotorOutput(void);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
float SpeedInnerControlCalculate(short Pulse,int Target,int MotorNum);
void SpeedInnerControl(void);
void SpeedControlOutput(void);
void SpeedControl(void);

void Get_Init_Gravity(void);
void Gyroscope_Balance_Calibration(void);
void Acceleromter_Balance_Calibration(void);

#endif /* __CONTROL_H__ */


