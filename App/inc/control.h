/**
  ******************************************************************************
  * 文件名程: motor.c 
  * 作    者: Jason_xy
  * 个人博客：https://jason-xy.cn
  * 版    本: V1.0
  * 编写日期: 2020-12-15
  * 功    能: 减速电机控制
  ******************************************************************************
  * 说明：
  * 1.需要提前设置好TIM。
  * 
  * 功能：
  * 1.电机初始化。
  * 2.改变电机速度。
  ******************************************************************************
  */

#ifndef __CONTROL_H__ 
#define __CONTROL_H__
#include "math.h"
#include "gy-86.h"
#include "motor.h"

#define CAR_ANGLE_SET   0
#define CAR_ANGLE_SPEED_SET 0

//PID结构体
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


