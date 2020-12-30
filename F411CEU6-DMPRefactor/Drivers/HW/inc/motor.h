/**
  ******************************************************************************
  * �ļ�����: motor.h 
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
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"
#include "tim.h"

#define MOTOR_TIM   htim2
#define DIRECTION_PORT GPIOA
#define DIRECTION_PIN_0 GPIO_PIN_4
#define DIRECTION_PIN_1 GPIO_PIN_5

#define MOTOR_OUT_MAX 300
#define MOTOR_OUT_MIN -300
#define MOTOR_OUT_DEAD_VAL  100

void SetMotorDutyCycle(float DutyCycle, uint8_t Channel);//DutyCycle > 0 ��ת; DutyCycle < 0 ��ת; DutyCycle = 0 ͣת
void Motor_Init(void);

#endif /* __MOTOR_H__ */
