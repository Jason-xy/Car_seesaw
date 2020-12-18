/**
  ******************************************************************************
  * 文件名程: motor.h 
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
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f1xx_hal.h"
#include "tim.h"

#define MOTOR_TIM   htim2
#define DIRECTION_PORT GPIOA
#define DIRECTION_PIN_0 GPIO_PIN_4
#define DIRECTION_PIN_1 GPIO_PIN_5

#define MOTOR_OUT_MAX 100
#define MOTOR_OUT_MIN -100
#define MOTOR_OUT_DEAD_VAL  25

void SetMotorDutyCycle(float DutyCycle);//DutyCycle > 0 正转; DutyCycle < 0 反转; DutyCycle = 0 停转
void Motor_Init(void);

#endif /* __MOTOR_H__ */
