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
#include "mpu6050.h"
#include "motor.h"
#define M_PI 3.14159

#define CAR_ANGLE_SET   0
#define CAR_ANGLE_SPEED_SET 0

//PID结构体
typedef struct {
    float P;
    float I;
    float D;
}PID;

//为真实加速度的100倍
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


