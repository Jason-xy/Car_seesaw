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
  * 2.����ڻ�PID��
  * 3.�ٶ��⻷PID��
  ******************************************************************************
  */
 #include "control.h"

unsigned char SpeedControlCount;
unsigned char SpeedControlPeriod;

//�жϴ���ͨ��
volatile HAL_TIM_ActiveChannel ActiveChannel[4]={	HAL_TIM_ACTIVE_CHANNEL_1, 
                                                    HAL_TIM_ACTIVE_CHANNEL_2, 
                                                    HAL_TIM_ACTIVE_CHANNEL_3, 
                                                    HAL_TIM_ACTIVE_CHANNEL_4};
//TIMͨ��
volatile uint8_t Channel[4] = { 0x00000000U, //TIM_CHANNEL_1
                                0x00000004U, //TIM_CHANNEL_2
                                0x00000008U, //TIM_CHANNEL_3
                                0x0000000CU};//TIM_CHANNEL_4

//ȫ�ֱ���
PID AngleRingPID={10,-0,4}; //0.8 0 0.4
PID SpeedRingPID={0,0,0};   //�ٶȿ���50 10 0
const float P=10,I=0.05,D=0;
PID MotorRingPID[4]={{P,I,D},{P,I,D},{P,I,D},{P,I,D}};    //����ջ�
//�Ƕȿ��Ʋ���
float GyroAngleSpeed=0;
float GyroAngleSpeedBefore=0;
float GyroAccle=0;
float GyroAccleBefore=0;
float CarAngle=0;
float CarAngleOld=0;

//������Ʋ���
float AngleControlOut=0;
float SpeedControlOut[4]={0};
float SpeedControlOutOld[4]={0};
float SpeedControlOutNew[4]={0};
float MotorOut[4]={0};
float ErrorRrev[4]={0};
float PWMBais[4]={0};
float PWM[4]={0};
int SpeedDirection=0;

//�ٶȿ��Ʋ���
short  MotorPulse[4]={0};
int  MotorPulseOld[4]={0};
int  MotorPulseSigma[4]={0};

float CarSpeed[4]={0};
float CarSpeedSet=0;
float CarSpeedOld[4]={0};
float CarPosition[4]={0};

//������岶��
uint8_t CapFlag[4] = { 0 };
uint32_t CapVal[4] = { 0 };																	
float SpeedOfWheel[4]={0};    //��/s
int cap = 0;

//״̬���
int status=0;

//���̹�һ������
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


//�ǶȻ�����
void AngleCalculate(void)
{
    //���ٶ�
	//��ΧΪ2000deg/sʱ�������ϵ��131.2 LSB/(deg/s)
	//ʹ����ʽ���������赥λ���㡣
	if(roll-CarAngle<0.001f||CarAngle-roll<0.001f)
	CarAngle=roll;
	GyroAngleSpeed=(CarAngle*1000-CarAngleOld*1000)/5.0f*0.7f+0.3f*GyroAngleSpeedBefore;
	GyroAccle=(GyroAngleSpeed-GyroAngleSpeedBefore)/0.05f*0.7f+0.3f*GyroAccleBefore;
	if(GyroAccle<0)GyroAccle=-GyroAccle;

    //�Ƕȡ�ʱ������ʵ�ʲ������㡿
    //CarAngle=CarAngle+GyroAngleSpeed*0.248f;
	
	//ֱ��ʹ��DMP��̬��������
	CarAngleOld=CarAngle;                        //[-5~5]
	GyroAngleSpeedBefore=GyroAngleSpeed;
	GyroAccleBefore=GyroAccle;
}

//�ǶȻ�����
void AngleControl(void)
{
    AngleControlOut=CarAngle*AngleRingPID.P+\
    GyroAngleSpeed*AngleRingPID.D/1.0f;         //������10^1
}

//����ٶȲ���
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//ѡ���Ӧ��ʱ������Ӧͨ��
	for(int i=0;i<4;i++){
		if(htim->Channel == ActiveChannel[i])
		{
			cap=1;
			switch(CapFlag[i])
			{//��ȡ��ǰͨ����״̬				
				case 0://û�ж������ж�
					if(htim->Channel == TIM_CHANNEL_3)
					//SpeedDirection=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
					__HAL_TIM_SET_COUNTER(htim,0);  //��������0
					CapFlag[i]++;  //�޸Ĳ���״̬Ϊ�����һ��
					break;
				case 1:
					if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2))
					{
						if(htim->Channel == ActiveChannel[2])
						SpeedDirection=1;
					}
					else if(htim->Channel == ActiveChannel[2])
						SpeedDirection=0;
					CapVal[i] = HAL_TIM_ReadCapturedValue(htim,Channel[i]); //��ȡ������ֵ
					if(CapVal[i]>750)
					{
						if(SpeedDirection==1)
						SpeedOfWheel[i]=375000/CapVal[i];//360*10^6/(Cap*960)  ����Ԥ��Ƶ������Ϊ8
						else
						SpeedOfWheel[i]=0.0f-375000/CapVal[i];
					}
					CapFlag[i] = 0;    //���ò���״
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

//�ٶȿ�������									//Ŀ��ת���ٶ�0
void SpeedControl(void)
{
  float fP[4],fI[4];   	
	float fDelta[4];
	for(int i=0;i<4;i++)
	{	
		if(SpeedOfWheel[i]>1000||SpeedOfWheel[i]<-1000)
		CarSpeed[i]=0;
		else
		CarSpeed[i] = SpeedOfWheel[i];
		SpeedOfWheel[i]=0;
		CarSpeed[i] = 0.7f * CarSpeedOld[i] + 0.3f * CarSpeed[i] ;//��ͨ�˲���ʹ�ٶȸ�ƽ��
		CarSpeedOld[i] = CarSpeed[i];                                     
		
		fDelta[i] = CAR_SPEED_SET;
		fDelta[i] -= CarSpeed[i];            //������10
		
		fP[i] = fDelta[i] * (SpeedRingPID.P);
		fI[i] = fDelta[i] * (SpeedRingPID.I/10.0f);

		CarPosition[i] += fI[i];
		
		//������������
		if((short)CarPosition[i] > CAR_POSITION_MAX)    CarPosition[i] = CAR_POSITION_MAX;
		if((short)CarPosition[i] < CAR_POSITION_MIN)    CarPosition[i] = CAR_POSITION_MIN;
		
		SpeedControlOutOld[i] = SpeedControlOutNew[i];
		SpeedControlOutNew[i] = fP[i] + CarPosition[i];
	}
}

//�ٶȿ������ƺ���
void SpeedControlOutput(void)
{
  float fValue[4];
	
	for(int i=0;i<4;i++)
	{
		fValue[i] = SpeedControlOutNew[i] - SpeedControlOutOld[i] ;
		SpeedControlOut[i] = fValue[i] * (SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + SpeedControlOutOld[i];
	}		
}

//�ٶȱջ����� 4·���
float SpeedInnerControlCalculate(short Pulse,int Target,int MotorNum)
{
	int Error;
	PWM[MotorNum]=Target;
	Error=Pulse-Target;
	PWMBais[MotorNum]=MotorRingPID[MotorNum].I*(Error-ErrorRrev[MotorNum])+MotorRingPID[MotorNum].P*Error;
	ErrorRrev[MotorNum]=Error;
	PWM[MotorNum]+=PWMBais[MotorNum];
	if(PWM[MotorNum]>MOTOR_OUT_MAX)PWM[MotorNum]=MOTOR_OUT_MAX;
	if(PWM[MotorNum]<MOTOR_OUT_MIN)PWM[MotorNum]=MOTOR_OUT_MAX;
	return PWM[MotorNum];
}

//�ٶȱջ�����
void SpeedInnerControl(void)
{
	for(int i=0;i<4;i++)
	{
		PWM[i]=SpeedInnerControlCalculate(SpeedOfWheel[i]/2,MotorOut[i],i);
		SetMotorDutyCycle(PWM[i],Channel[i]);
	}
}

//������
void MotorOutput(void)
{
	for(int i=0;i<4;i++)
	{
    MotorOut[i]=AngleControlOut-SpeedControlOut[i]/100.0f;
	//MotorOut[i]=Scale(MotorOut[i], -300, 300, -100, 100);

    //�����������
    if(MotorOut[i]>7.0f)
        MotorOut[i]+=MOTOR_OUT_DEAD_VAL;
    else if(MotorOut[i]<-7.0f)
        MotorOut[i]-=MOTOR_OUT_DEAD_VAL;
		
		if(GyroAccle>10||GyroAccle<-10)
		{
			if(CarAngle>0.5f)
			MotorOut[i]+=GyroAccle*AngleRingPID.I;
			else if(CarAngle<-0.5f)
			MotorOut[i]-=GyroAccle*AngleRingPID.I;
		}

    //���ʹ���
    if(MotorOut[i]>MOTOR_OUT_MAX)
        MotorOut[i]=MOTOR_OUT_MAX;
    if(MotorOut[i]<MOTOR_OUT_MIN)
        MotorOut[i]=MOTOR_OUT_MIN;
//		if(i==0)
//		MotorOut[0]+=50;
//		if(MotorOut[i]<-5)MotorOut[i]-=10;
//    SetMotorDutyCycle(MotorOut[i],Channel[i]);
	}
	SpeedInnerControl();//�ٶȱջ�����
	//SpeedInnerControl();//�ջ����
}

