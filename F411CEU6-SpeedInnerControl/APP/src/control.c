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
  * 2.电机内环PID。
  * 3.速度外环PID。
  ******************************************************************************
  */
 #include "control.h"

unsigned char SpeedControlCount;
unsigned char SpeedControlPeriod;

//中断触发通道
volatile HAL_TIM_ActiveChannel ActiveChannel[4]={	HAL_TIM_ACTIVE_CHANNEL_1, 
                                                    HAL_TIM_ACTIVE_CHANNEL_2, 
                                                    HAL_TIM_ACTIVE_CHANNEL_3, 
                                                    HAL_TIM_ACTIVE_CHANNEL_4};
//TIM通道
volatile uint8_t Channel[4] = { 0x00000000U, //TIM_CHANNEL_1
                                0x00000004U, //TIM_CHANNEL_2
                                0x00000008U, //TIM_CHANNEL_3
                                0x0000000CU};//TIM_CHANNEL_4

//全局变量
PID AngleRingPID={20,-0,4}; //0.8 0 0.4
PID SpeedRingPID={0,0,0};   //速度开环50 10 0
PID MotorRingPID={15,0,0};    //电机闭环
//角度控制参数
float GyroAngleSpeed=0;
float GyroAngleSpeedBefore=0;
float GyroAccle=0;
float GyroAccleBefore=0;
float CarAngle=0;
float CarAngleOld=0;

//电机控制参数
float AngleControlOut=0;
float SpeedControlOut=0;
float SpeedControlOutOld=0;
float SpeedControlOutNew=0;
float MotorOut=0;
float ErrorRrev=0;
float PWMBais=0;
float PWM=0;
int SpeedDirection=0;

//速度控制参数
short  MotorPulse=0;
int  MotorPulseOld=0;
int  MotorPulseSigma=0;

float CarSpeed=0;
float CarSpeedSet=0;
float CarSpeedOld=0;
float CarPosition=0;

//电机脉冲捕获
uint8_t CapFlag[4] = { 0 };
uint32_t CapVal[4] = { 0 };																	
float SpeedOfWheel[4]={0};    //°/s
int cap = 0;

//状态标记
int status=0;

//量程归一化处理
float Scale(float input, float inputMin, float inputMax, float outputMin, float outputMax)
{ 
  float output;
  if (inputMin < inputMax)
    output = (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output = (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}


//角度环计算
void AngleCalculate(void)
{
    //角速度
	//范围为2000deg/s时，换算关系：131.2 LSB/(deg/s)
	//使用显式读出，无需单位换算。
	if(roll-CarAngle<0.001f||CarAngle-roll<0.001f)
	CarAngle=roll;
	GyroAngleSpeed=(CarAngle*1000-CarAngleOld*1000)/5.0f*0.7f+0.3f*GyroAngleSpeedBefore;
	GyroAccle=(GyroAngleSpeed-GyroAngleSpeedBefore)/0.05f*0.7f+0.3f*GyroAccleBefore;
	if(GyroAccle<0)GyroAccle=-GyroAccle;

    //角度【时间差根据实际测量计算】
    //CarAngle=CarAngle+GyroAngleSpeed*0.248f;
	
	//直接使用DMP姿态解算数据
	CarAngleOld=CarAngle;                        //[-5~5]
	GyroAngleSpeedBefore=GyroAngleSpeed;
	GyroAccleBefore=GyroAccle;
}

//角度环控制
void AngleControl(void)
{
    AngleControlOut=CarAngle*AngleRingPID.P+\
    GyroAngleSpeed*AngleRingPID.D/1.0f;         //数量级10^1
}

//速度开环计算									//目标转动速度0
void SpeedControl(void)
{
  float fP,fI;   	
	float fDelta;	
		CarSpeed = (SpeedOfWheel[0]+SpeedOfWheel[1])/2;
		CarSpeed = 0.7f * CarSpeedOld + 0.3f * CarSpeed ;//低通滤波，使速度更平滑
		CarSpeedOld = CarSpeed;                                     
		
		fDelta = CAR_SPEED_SET;
		fDelta -= CarSpeed;            //数量级10
		
		fP = fDelta * (SpeedRingPID.P);
		fI = fDelta * (SpeedRingPID.I/10.0f);

		CarPosition += fI;
		
		//积分上限设限
		if((short)CarPosition > CAR_POSITION_MAX)    CarPosition = CAR_POSITION_MAX;
		if((short)CarPosition < CAR_POSITION_MIN)    CarPosition = CAR_POSITION_MIN;
		
		SpeedControlOutOld = SpeedControlOutNew;
		SpeedControlOutNew = fP + CarPosition;
}

//速度开环控制函数
void SpeedControlOutput(void)
{
  float fValue;
	
		fValue = SpeedControlOutNew - SpeedControlOutOld ;
		SpeedControlOut = fValue * (SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + SpeedControlOutOld;
		
}

//速度闭环计算 4路电机
float SpeedInnerControlCalculate(short Pulse,int Target)
{
	int Error;
	PWM=Target;
	Error=Pulse-Target;
	PWMBais=MotorRingPID.P*(Error-ErrorRrev)+MotorRingPID.I*Error;
	ErrorRrev=Error;
	PWM+=PWMBais;
	if(PWM>MOTOR_OUT_MAX)PWM=MOTOR_OUT_MAX+300;
	if(PWM<MOTOR_OUT_MIN)PWM=MOTOR_OUT_MIN-300;
	return PWM;
}

//速度闭环控制
void SpeedInnerControl(void)
{
		PWM=SpeedInnerControlCalculate(CarSpeed*2,MotorOut);
		SetMotorDutyCycle(PWM,Channel[0]);
		SetMotorDutyCycle(PWM,Channel[1]);
		SetMotorDutyCycle(PWM,Channel[2]);
		SetMotorDutyCycle(PWM,Channel[3]);
}

//电机输出
void MotorOutput(void)
{
    MotorOut=AngleControlOut-SpeedControlOut/100.0f;

    //添加死区常数
    if(MotorOut>27.0f)
        MotorOut+=MOTOR_OUT_DEAD_VAL;
    else if(MotorOut<-27.0f)
        MotorOut-=MOTOR_OUT_DEAD_VAL;
		
		if(GyroAccle>10||GyroAccle<-10)
		{
			if(CarAngle>0.5f)
			MotorOut+=GyroAccle*AngleRingPID.I;
			else if(CarAngle<-0.5f)
			MotorOut-=GyroAccle*AngleRingPID.I;
		}

    //饱和处理
    if(MotorOut>MOTOR_OUT_MAX)
        MotorOut=MOTOR_OUT_MAX;
    if(MotorOut<MOTOR_OUT_MIN)
        MotorOut=MOTOR_OUT_MIN;
	SpeedInnerControl();//速度闭环控制
}

//编码器模式获取速度
void getSpeed(void)
{
	SpeedOfWheel[0] = (short)(__HAL_TIM_GET_COUNTER(&htim4));//先读取脉冲数
	SpeedOfWheel[1] = (short)(__HAL_TIM_GET_COUNTER(&htim3));
	__HAL_TIM_SET_COUNTER(&htim4,0);//再计数器清零
	__HAL_TIM_SET_COUNTER(&htim3,0);
}

