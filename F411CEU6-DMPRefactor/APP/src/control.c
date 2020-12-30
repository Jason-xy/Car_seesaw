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
PID AngleRingPID={4,0,4}; //3，-1，1
PID SpeedRingPID={0,0,0};   //速度开环
PID MotorRingPID[4]={{10,0.9,0},{10,0.9,0},{10,0.9,0},{10,0.9,0}};    //电机闭环
//角度控制参数
float GyroAngleSpeed=0;
float GyroAngleSpeedBefore=0;
float GyroAccle=0;
float CarAngle=0;
float CarAngleOld=0;

//电机控制参数
float AngleControlOut=0;
float SpeedControlOut[4]={0};
float SpeedControlOutOld[4]={0};
float SpeedControlOutNew[4]={0};
float MotorOut[4]={0};
float ErrorRrev[4]={0};
float PWMBais[4]={0};
float PWM[4]={0};

//速度控制参数
short  MotorPulse[4]={0};
int  MotorPulseOld[4]={0};
int  MotorPulseSigma[4]={0};

float CarSpeed[4]={0};
float CarSpeedSet=0;
float CarSpeedOld[4]={0};
float CarPosition[4]={0};

//电机脉冲捕获
uint8_t CapFlag[4] = { 0 };
uint32_t CapVal[4] = { 0 };																	
float SpeedOfWheel[4]={0};    //°/s
int cap = 0;

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
	GyroAngleSpeed=(CarAngle*1000-CarAngleOld*1000)/5.0f;
	GyroAccle=(GyroAngleSpeed-GyroAngleSpeedBefore)/0.05f;

    //角度【时间差根据实际测量计算】
    //CarAngle=CarAngle+GyroAngleSpeed*0.248f;
	
	//直接使用DMP姿态解算数据
	CarAngleOld=CarAngle;                        //[-5~5]
	GyroAngleSpeedBefore=GyroAngleSpeed;
}

//角度环控制
void AngleControl(void)
{
    AngleControlOut=CarAngle*AngleRingPID.P+\
		GyroAccle*AngleRingPID.I+\
    GyroAngleSpeed*AngleRingPID.D/1.0f;         //数量级10^1
}

//电机速度捕获
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//选择对应定时器，对应通道
	for(int i=0;i<4;i++){
		if(htim->Channel == ActiveChannel[i])
		{
			cap=1;
			switch(CapFlag[i])
			{//读取当前通道的状态				
				case 0://没有读到过中断
					__HAL_TIM_SET_COUNTER(htim,0);  //计数器置0
					CapFlag[i]++;  //修改捕获状态为捕获过一次
					break;
				case 1:
					CapVal[i] = HAL_TIM_ReadCapturedValue(htim,Channel[i]); //读取计数器值	
					SpeedOfWheel[i]=375000/CapVal[i];//360*10^6/(Cap*960)
					CapFlag[i] = 0;    //重置捕获状
					break;
			}	
		}
		if(cap)
        {
			cap = 0;
            break;
        }
	}
}

//速度开环计算									//目标转动速度0
void SpeedControl(void)
{
  float fP[4],fI[4];   	
	float fDelta[4];
	for(int i=0;i<4;i++)
	{	
		CarSpeed[i] = SpeedOfWheel[i]; 
		CarSpeed[i] = 0.7f * CarSpeedOld[i] + 0.3f * CarSpeed[i] ;//低通滤波，使速度更平滑
		CarSpeedOld[i] = CarSpeed[i];                                     
		
		fDelta[i] = CAR_SPEED_SET;
		fDelta[i] -= CarSpeed[i];            //数量级10
		
		fP[i] = fDelta[i] * (SpeedRingPID.P);
		fI[i] = fDelta[i] * (SpeedRingPID.I/10.0f);

		CarPosition[i] += fI[i];
		
		//积分上限设限
		if((short)CarPosition[i] > CAR_POSITION_MAX)    CarPosition[i] = CAR_POSITION_MAX;
		if((short)CarPosition[i] < CAR_POSITION_MIN)    CarPosition[i] = CAR_POSITION_MIN;
		
		SpeedControlOutOld[i] = SpeedControlOutNew[i];
		SpeedControlOutNew[i] = fP[i] + CarPosition[i];
	}
}

//速度开环控制函数
void SpeedControlOutput(void)
{
  float fValue[4];
	
	for(int i=0;i<4;i++)
	{
		fValue[i] = SpeedControlOutNew[i] - SpeedControlOutOld[i] ;
		SpeedControlOut[i] = fValue[i] * (SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + SpeedControlOutOld[i];
	}		
}

//速度闭环计算 4路电机
float SpeedInnerControlCalculate(short Pulse,int Target,int MotorNum)
{
	int Error;
	Error=Pulse-Target;
	PWMBais[MotorNum]=MotorRingPID[MotorNum].P*(Error-ErrorRrev[MotorNum])+MotorRingPID[MotorNum].I*Error;
	ErrorRrev[MotorNum]=Error;
	PWM[MotorNum]+=PWMBais[MotorNum];
	if(PWM[MotorNum]>100)PWM[MotorNum]=100;
	if(PWM[MotorNum]<-100)PWM[MotorNum]=-100;
	return PWM[MotorNum];
}

//速度闭环控制
void SpeedInnerControl(void)
{
	for(int i=0;i<4;i++)
	{
		MotorOut[i]=SpeedInnerControlCalculate(SpeedOfWheel[i],MotorOut[i],i);
		SetMotorDutyCycle(MotorOut[i],Channel[i]);
	}
}

//电机输出
void MotorOutput(void)
{
	for(int i=0;i<4;i++)
	{
    MotorOut[i]=AngleControlOut-SpeedControlOut[i];
	//MotorOut[i]=Scale(MotorOut[i], -300, 300, -100, 100);

    //添加死区常数
    if(MotorOut[i]>5.4f)
        MotorOut[i]+=MOTOR_OUT_DEAD_VAL;
    else if(MotorOut[i]<-5.4f)
        MotorOut[i]-=MOTOR_OUT_DEAD_VAL;

    //饱和处理
    if(MotorOut[i]>MOTOR_OUT_MAX)
        MotorOut[i]=MOTOR_OUT_MAX;
    if(MotorOut[i]<MOTOR_OUT_MIN)
        MotorOut[i]=MOTOR_OUT_MIN;

    SetMotorDutyCycle(MotorOut[i],Channel[i]);
	}
	//SpeedInnerControl();//闭环输出
}

