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
extern PID MotorRingPID[4];

//�Ƕȿ��Ʋ���
extern float GyroAngleSpeed;
extern float GyroAngleSpeedBefore;
extern float GyroAccle;
extern float CarAngle;
extern float CarAngleOld;

//������Ʋ���
extern float SpeedControlOut[4];
extern float SpeedControlOutOld[4];
extern float SpeedControlOutNew[4];
extern float AngleControlOut;
extern float MotorOut[4];
extern int SpeedDirection;

//�ٶȿ��Ʋ���
extern short  MotorPulse[4];
extern int  MotorPulseOld[4];
extern int  MotorPulseSigma[4];

extern float CarSpeed[4];
extern float CarSpeedSet;
extern float CarSpeedOld[4];
extern float CarPosition[4];

//������岶��
extern uint8_t CapFlag[4];
extern uint32_t CapVal[4];																	
extern float SpeedOfWheel[4];    //��/s
extern int cap;

//״̬���
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


