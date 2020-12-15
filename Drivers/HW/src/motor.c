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
