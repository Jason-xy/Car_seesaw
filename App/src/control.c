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
  ******************************************************************************
  */
 #include "control.h"

 //全局变量
PID AngleRingPID={4.8,0,1};
float GyroAngleSpeed=0;
float CarAngle=0;
float AngleControlOut=0;
float MotorOut=0;

//角度环计算
void AngleCalculate(void)
{
    //角速度
	//范围为250deg/s时，换算关系：131.2 LSB/(deg/s)
	//使用显式读出，无需单位换算。
    GyroAngleSpeed=Gx;

    //角度【时间差根据实际测量计算】
    CarAngle=CarAngle+GyroAngleSpeed*0.12857f; 
    
}

//角度环控制
void AngleControl(void)
{
    AngleControlOut=(CAR_ANGLE_SET-CarAngle)*AngleRingPID.P*1+\
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
