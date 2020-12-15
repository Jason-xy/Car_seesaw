/**
  ******************************************************************************
  * �ļ�����: motor.c 
  * ��    ��: Jason_xy
  * ���˲��ͣ�https://jason-xy.cn
  * ��    ��: V1.0
  * ��д����: 2020-12-15
  * ��    ��: ���ٵ������
  ******************************************************************************
  * ˵����
  * 1.��Ҫ��ǰ���ú�TIM��
  * 
  * ���ܣ�
  * 1.�����ʼ����
  * 2.�ı����ٶȡ�
  ******************************************************************************
  */

#ifndef __CONTROL_H__ 
#define __CONTROL_H__
#include "math.h"
#include "gy-86.h"
#include "motor.h"

#define CAR_ANGLE_SET   0
#define CAR_ANGLE_SPEED_SET 0

//PID�ṹ��
typedef struct {
    float P;
    float I;
    float D;
}PID;

extern PID AngleRingPID;
extern float GyroAngleSpeed;
extern float CarAngle;
extern float AngleControlOut;
extern float MotorOut;

void AngleCalculate(void);
void AngleControl(void);
void MotorOutput(void);

#endif /* __CONTROL_H__ */


