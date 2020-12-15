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
#include "motor.h"

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&MOTOR_TIM,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&MOTOR_TIM,TIM_CHANNEL_2);
//    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
}

void SetMotorDutyCycle(float DutyCycle)
{
//    if(DutyCycle<0)
//    {
//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
//        DutyCycle = (-DutyCycle);
//    }else
//    {
//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//    }
//    __HAL_TIM_SetCompare(&MOTOR_TIM,TIM_CHANNEL_1,(int)DutyCycle);
	if(DutyCycle<0)
	{
		DutyCycle = (-DutyCycle);
		HAL_TIM_PWM_Start(&MOTOR_TIM,TIM_CHANNEL_2);
		__HAL_TIM_SetCompare(&MOTOR_TIM,TIM_CHANNEL_2,(int)DutyCycle*10);
		HAL_TIM_PWM_Stop(&MOTOR_TIM,TIM_CHANNEL_1);
	}
	else
	{
		HAL_TIM_PWM_Start(&MOTOR_TIM,TIM_CHANNEL_1);
		__HAL_TIM_SetCompare(&MOTOR_TIM,TIM_CHANNEL_1,(int)DutyCycle*10);
		HAL_TIM_PWM_Stop(&MOTOR_TIM,TIM_CHANNEL_2);
	}
}
