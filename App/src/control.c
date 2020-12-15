/**
  ******************************************************************************
  * �ļ�����: control.c 
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
  ******************************************************************************
  */
 #include "control.h"

 //ȫ�ֱ���
PID AngleRingPID={50,0,1};
float GyroAngleSpeed=0;
float CarAngle=0;
float AngleControlOut=0;
float MotorOut=0;

//�ǶȻ�����
void AngleCalculate(void)
{
    //���ٶ�
	//��ΧΪ2000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
    GyroAngleSpeed=Gyro_x/16.4f;

    //�Ƕ�
    CarAngle=CarAngle+GyroAngleSpeed*0.005; 
    
}

//�ǶȻ�����
void AngleControl(void)
{
    AngleControlOut=(CAR_ANGLE_SET-CarAngle)*AngleRingPID.P*5+\
    (CAR_ANGLE_SPEED_SET-GyroAngleSpeed)*(AngleRingPID.D);
}

//������
void MotorOutput(void)
{
    MotorOut=AngleControlOut;

    //�����������
    if(MotorOut>0)
        MotorOut+=MOTOR_OUT_DEAD_VAL;
    else if(MotorOut<0)
        MotorOut-=MOTOR_OUT_DEAD_VAL;

    //���ʹ���
    if(MotorOut>MOTOR_OUT_MAX)
        MotorOut=MOTOR_OUT_MAX;
    if(MotorOut<MOTOR_OUT_MIN)
        MotorOut=MOTOR_OUT_MIN;

    SetMotorDutyCycle(MotorOut);
}
