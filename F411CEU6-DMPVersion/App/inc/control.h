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
#include "mpu6050.h"
#include "motor.h"
#define M_PI 3.14159

#define CAR_ANGLE_SET   0
#define CAR_ANGLE_SPEED_SET 0

//PID�ṹ��
typedef struct {
    float P;
    float I;
    float D;
}PID;

//Ϊ��ʵ���ٶȵ�100��
typedef struct{
    short ax;
    short ay;
    short az;
}Acceleromter_accstruct;

extern PID AngleRingPID;
extern float GyroAngleSpeed;
extern float CarAngle;
extern float AngleControlOut;
extern float MotorOut;

void AngleCalculate(void);
void AngleControl(void);
void MotorOutput(void);

void Get_Init_Gravity(void);
void Gyroscope_Balance_Calibration(void);
void Acceleromter_Balance_Calibration(void);

#endif /* __CONTROL_H__ */


