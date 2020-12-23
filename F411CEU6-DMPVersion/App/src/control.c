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
  * 2.��������У׼��
  ******************************************************************************
  */
 #include "control.h"

//ȫ�ֱ���
PID AngleRingPID={2.5,-1,1}; //3��-1��1
float GyroAngleSpeed=0;
float GyroAngleSpeedBefore=0;
float GyroAccle=0;
float CarAngle=0;
float AngleControlOut=0;
float MotorOut=0;

//����У׼
short gravity=0;
short gravityE2=0;

//Ϊ��ʵ���ٶȵ�100��
Acceleromter_accstruct init_gravity;
Acceleromter_accstruct accel_struct;
short angle_x=0;
short angle_z=0;

//�ǶȻ�����
void AngleCalculate(void)
{
    //���ٶ�
	//��ΧΪ250deg/sʱ�������ϵ��131.2 LSB/(deg/s)
	//ʹ����ʽ���������赥λ���㡣
	MPU_Get_Gyroscope(&Gx,&Gy,&Gz);
  GyroAngleSpeed=Gx/131.2f;
	GyroAccle=(GyroAngleSpeed-GyroAngleSpeedBefore)/0.05f;

    //�Ƕȡ�ʱ������ʵ�ʲ������㡿
    //CarAngle=CarAngle+GyroAngleSpeed*0.248f;
	
	//ֱ��ʹ��DMP��̬��������
	CarAngle=roll;
	GyroAngleSpeedBefore=GyroAngleSpeed;
}

//�ǶȻ�����
void AngleControl(void)
{
    AngleControlOut=(CAR_ANGLE_SET-CarAngle)*AngleRingPID.P+\
		GyroAccle*AngleRingPID.I+\
    (CAR_ANGLE_SPEED_SET-GyroAngleSpeed)*(AngleRingPID.D);
}

//������
void MotorOutput(void)
{
    MotorOut=AngleControlOut;

    //�����������
    if(MotorOut>1)
        MotorOut+=MOTOR_OUT_DEAD_VAL;
    else if(MotorOut<-1)
        MotorOut-=MOTOR_OUT_DEAD_VAL;

    //���ʹ���
    if(MotorOut>MOTOR_OUT_MAX)
        MotorOut=MOTOR_OUT_MAX;
    if(MotorOut<MOTOR_OUT_MIN)
        MotorOut=MOTOR_OUT_MIN;

    SetMotorDutyCycle(MotorOut);
}

//��ȡ�ʼУ׼��ʱ�򣬻�ȡƽ�ؽ��ٶ���������
void Get_Init_Gravity(void)
{

    read_Accelerometer_MPS();
    init_gravity.ax=Ax*100;
    init_gravity.ay=Ay*100;
    init_gravity.az=Az*100;
    gravityE2=init_gravity.ax*init_gravity.ax+init_gravity.ay*init_gravity.ay+init_gravity.az+init_gravity.az;
    gravity=sqrt(gravityE2);
}

//���ͣתʱ����������Ϊƽ�⣬У�ԽǶ�
void Gyroscope_Balance_Calibration(void)
{   
    accel_struct.ax=Ax*100;
    accel_struct.ay=Ay*100;
//    accel_struct.az=Az*100;

    if(gravityE2+2000>accel_struct.ax*accel_struct.ax+accel_struct.az*accel_struct.az\
                &&gravityE2-2000<accel_struct.ax*accel_struct.ax+accel_struct.az*accel_struct.az){  //������������ٶ����ֵ�Ƚϣ�ȷ����pwmΪ0

        angle_z=accel_struct.az/gravity;
        angle_x=accel_struct.ax/gravity;

        if((angle_x-0.1)<angle_z&&angle_z<(angle_x+0.1)) {//x��z�������б������Χ�ڣ���ʹ��
            CarAngle=(90-acos(angle_x)*180/M_PI); 
        }   
    }
}

//���ٶȼƲ�Ϊ�㣬���ýǳ�ʼ�Ƕ�
void Acceleromter_Balance_Calibration(void)
{
    //ƫ��
    short errorz=0;
    read_Accelerometer_MPS();
//    accel_struct.ax=Ax*100;
//    accel_struct.ay=Ay*100;
    accel_struct.az=Az*100;
    errorz=accel_struct.az-init_gravity.az;


    if(errorz<5&&errorz>-5)
    {
        CarAngle=CarAngle/2;
        if(errorz<1&&errorz>-1)
        {           //ֻ��z����ٶȣ��������Χ����֤��ƽ�⣬����
            CarAngle=0;
        }
    }
}

