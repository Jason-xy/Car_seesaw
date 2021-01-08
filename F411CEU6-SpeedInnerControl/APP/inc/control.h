/**
  ******************************************************************************
  * �ļ�����: control.h
  * ��    ��: Jason_xy
  * ���˲��ͣ�https://jason-xy.cn
  * ��    ��: V1.0
  * ��д����: 2020-12-15
  * ��    ��: PID����
  ******************************************************************************
  * ˵����
  * 1.��Ҫ��ǰ���ú�gy-86��
  * 2.��Ҫ��ǰ���úõ����
  * 
  * ���ܣ�
  * 1.�ǶȻ����ơ�
  * 2.��������У׼��
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
#define SPEED_CONTROL_PERIOD	 25	    //�ٶȻ���������

extern unsigned char SpeedControlCount;
extern unsigned char SpeedControlPeriod;

//PID�ṹ��
typedef struct {
    float P;
    float I;
    float D;
}PID;

//PID����
extern PID AngleRingPID;
extern PID SpeedRingPID;
extern PID MotorRingPID;

//�Ƕȿ��Ʋ���
extern float GyroAngleSpeed;
extern float GyroAngleSpeedOld;
extern float GyroAccle;
extern float CarAngle;
extern float CarAngleOld;

//������Ʋ���
extern float SpeedControlOut;
extern float SpeedControlOutOld;
extern float SpeedControlOutNew;
extern float AngleControlOut;
extern float MotorOut;

//�ٶȿ��Ʋ���
extern short  MotorPulse;
extern int  MotorPulseOld;
extern int  MotorPulseSigma;

extern float CarSpeed;
extern float CarSpeedSet;
extern float CarSpeedOld;
extern float CarPosition;

//������岶��																	
extern float SpeedOfWheel[2];   


float Scale(float input, float inputMin, float inputMax, float outputMin, float outputMax);
void AngleCalculate(void);
void AngleControl(void);
void MotorOutput(void);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
float SpeedInnerControlCalculate(short Pulse,int Target);
void SpeedInnerControl(void);
void SpeedControlOutput(void);
void SpeedControl(void);

void Get_Init_Gravity(void);
void Gyroscope_Balance_Calibration(void);
void Acceleromter_Balance_Calibration(void);
void getSpeed(void);

#endif /* __CONTROL_H__ */


