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
PID AngleRingPID={6,0,80}; //40 0 20   20  40                  [20,0,40]					[13,0,60]		[11,0,80]			[8,0,80]
PID SpeedRingPID={0,0,0};   //速度开环50 10 0                   [0,0,0]						[0,0,0]
PID MotorRingPID={3.05,0.53,0};    //电机闭环5 6                      [6,0,0]				[6.6,0.5,0]  [2.5,0.5,0]	[2.8,0.53]
//角度控制参数
float GyroAngleSpeed=0;
float GyroAngleSpeedOld=0;
float GyroAccle=0;
float GyroAccleOld=0;
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

//速度控制参数
short  MotorPulse=0;
int  MotorPulseOld=0;
int  MotorPulseSigma=0;

float CarSpeed=0;
float CarSpeedSet=0;
float CarSpeedOld=0;
float CarPosition=0;

//电机脉冲捕获																	
float SpeedOfWheel[2]={0};    //°/s

int PWM_OUT_DEAD_VAL=25;


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
    //角度
	//范围为2000deg/s时，换算关系：131.2 LSB/(deg/s)
	//直接使用DMP姿态解算数据
	if(roll-CarAngle<0.001f||CarAngle-roll<0.001f)
	CarAngle=roll;
	
	//低通滤波
	GyroAngleSpeed=(CarAngle*1000-CarAngleOld*1000)/5.0f*0.7f+0.3f*GyroAngleSpeedOld;
	GyroAccle=(GyroAngleSpeed-GyroAngleSpeedOld)/0.05f*0.7f+0.3f*GyroAccleOld;
	
	//if(GyroAccle<0)GyroAccle=-GyroAccle;
	
	
	CarAngleOld=CarAngle;                        //[-5~5]
	GyroAngleSpeedOld=GyroAngleSpeed;
	GyroAccleOld=GyroAccle;
}

//角度环控制
void AngleControl(void)
{
    AngleControlOut=CarAngle*AngleRingPID.P+\
    GyroAngleSpeed*AngleRingPID.D/10.0f;         //数量级10^1
}

//速度开环计算									//目标转动速度0
void SpeedControl(void)
{
  float fP,fI;   	
	float fDelta;	                                     
		
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

//速度闭环计算 2路电机
float SpeedInnerControlCalculate(short Pulse,int Target)
{
	int Error;
	PWM=Target;
	Error=Pulse-Target;
	PWMBais=MotorRingPID.P*(Error-ErrorRrev)+MotorRingPID.I*Error;
	ErrorRrev=Error;
	PWM+=PWMBais;
	if(PWM>MOTOR_OUT_MAX+300)PWM=MOTOR_OUT_MAX+300;
	if(PWM<MOTOR_OUT_MIN-300)PWM=MOTOR_OUT_MIN-300;
	if(PWM>27.0f)
	 PWM+=200;
  else if(MotorOut<-27.0f)
		PWM-=(200+PWM_OUT_DEAD_VAL);
	return PWM;
}

//速度闭环控制
void SpeedInnerControl(void)
{
	getSpeed();
	PWM=SpeedInnerControlCalculate(CarSpeed,MotorOut);
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
	CarSpeed = (SpeedOfWheel[0]+SpeedOfWheel[1])/2;
	CarSpeed = 0.7f * CarSpeedOld + 0.3f * CarSpeed ;//低通滤波，使速度更平滑
	CarSpeedOld = CarSpeed;
}

