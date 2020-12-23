/**
  ******************************************************************************
  * 文件名程: control.c 
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
 #include "control.h"

//全局变量
PID AngleRingPID={2.5,-1,1}; //3，-1，1
float GyroAngleSpeed=0;
float GyroAngleSpeedBefore=0;
float GyroAccle=0;
float CarAngle=0;
float AngleControlOut=0;
float MotorOut=0;

//重力校准
short gravity=0;
short gravityE2=0;

//为真实加速度的100倍
Acceleromter_accstruct init_gravity;
Acceleromter_accstruct accel_struct;
short angle_x=0;
short angle_z=0;

//角度环计算
void AngleCalculate(void)
{
    //角速度
	//范围为250deg/s时，换算关系：131.2 LSB/(deg/s)
	//使用显式读出，无需单位换算。
	MPU_Get_Gyroscope(&Gx,&Gy,&Gz);
  GyroAngleSpeed=Gx/131.2f;
	GyroAccle=(GyroAngleSpeed-GyroAngleSpeedBefore)/0.05f;

    //角度【时间差根据实际测量计算】
    //CarAngle=CarAngle+GyroAngleSpeed*0.248f;
	
	//直接使用DMP姿态解算数据
	CarAngle=roll;
	GyroAngleSpeedBefore=GyroAngleSpeed;
}

//角度环控制
void AngleControl(void)
{
    AngleControlOut=(CAR_ANGLE_SET-CarAngle)*AngleRingPID.P+\
		GyroAccle*AngleRingPID.I+\
    (CAR_ANGLE_SPEED_SET-GyroAngleSpeed)*(AngleRingPID.D);
}

//电机输出
void MotorOutput(void)
{
    MotorOut=AngleControlOut;

    //添加死区常数
    if(MotorOut>1)
        MotorOut+=MOTOR_OUT_DEAD_VAL;
    else if(MotorOut<-1)
        MotorOut-=MOTOR_OUT_DEAD_VAL;

    //饱和处理
    if(MotorOut>MOTOR_OUT_MAX)
        MotorOut=MOTOR_OUT_MAX;
    if(MotorOut<MOTOR_OUT_MIN)
        MotorOut=MOTOR_OUT_MIN;

    SetMotorDutyCycle(MotorOut);
}

//获取最开始校准的时候，获取平地角速度三轴数据
void Get_Init_Gravity(void)
{

    read_Accelerometer_MPS();
    init_gravity.ax=Ax*100;
    init_gravity.ay=Ay*100;
    init_gravity.az=Az*100;
    gravityE2=init_gravity.ax*init_gravity.ax+init_gravity.ay*init_gravity.ay+init_gravity.az+init_gravity.az;
    gravity=sqrt(gravityE2);
}

//电机停转时，陀螺仪认为平衡，校对角度
void Gyroscope_Balance_Calibration(void)
{   
    accel_struct.ax=Ax*100;
    accel_struct.ay=Ay*100;
//    accel_struct.az=Az*100;

    if(gravityE2+2000>accel_struct.ax*accel_struct.ax+accel_struct.az*accel_struct.az\
                &&gravityE2-2000<accel_struct.ax*accel_struct.ax+accel_struct.az*accel_struct.az){  //测量的三轴加速度与初值比较，确定是pwm为0

        angle_z=accel_struct.az/gravity;
        angle_x=accel_struct.ax/gravity;

        if((angle_x-0.1)<angle_z&&angle_z<(angle_x+0.1)) {//x，z轴算出倾斜角在误差范围内，则使用
            CarAngle=(90-acos(angle_x)*180/M_PI); 
        }   
    }
}

//加速度计测为零，设置角初始角度
void Acceleromter_Balance_Calibration(void)
{
    //偏移
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
        {           //只用z轴加速度，如果在误差范围内则证明平衡，重置
            CarAngle=0;
        }
    }
}

