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
PID AngleRingPID={4,0,4}; //3��-1��1
PID SpeedRingPID={0,0,0};   //�ٶȿ���
PID MotorRingPID[4]={{10,0.9,0},{10,0.9,0},{10,0.9,0},{10,0.9,0}};    //����ջ�
//�Ƕȿ��Ʋ���
float GyroAngleSpeed=0;
float GyroAngleSpeedBefore=0;
float GyroAccle=0;
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
	GyroAngleSpeed=(CarAngle*1000-CarAngleOld*1000)/5.0f;
	GyroAccle=(GyroAngleSpeed-GyroAngleSpeedBefore)/0.05f;

    //�Ƕȡ�ʱ������ʵ�ʲ������㡿
    //CarAngle=CarAngle+GyroAngleSpeed*0.248f;
	
	//ֱ��ʹ��DMP��̬��������
	CarAngleOld=CarAngle;                        //[-5~5]
	GyroAngleSpeedBefore=GyroAngleSpeed;
}

//�ǶȻ�����
void AngleControl(void)
{
    AngleControlOut=CarAngle*AngleRingPID.P+\
		GyroAccle*AngleRingPID.I+\
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
					__HAL_TIM_SET_COUNTER(htim,0);  //��������0
					CapFlag[i]++;  //�޸Ĳ���״̬Ϊ�����һ��
					break;
				case 1:
					CapVal[i] = HAL_TIM_ReadCapturedValue(htim,Channel[i]); //��ȡ������ֵ	
					SpeedOfWheel[i]=375000/CapVal[i];//360*10^6/(Cap*960)
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
		CarSpeed[i] = SpeedOfWheel[i]; 
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
	Error=Pulse-Target;
	PWMBais[MotorNum]=MotorRingPID[MotorNum].P*(Error-ErrorRrev[MotorNum])+MotorRingPID[MotorNum].I*Error;
	ErrorRrev[MotorNum]=Error;
	PWM[MotorNum]+=PWMBais[MotorNum];
	if(PWM[MotorNum]>100)PWM[MotorNum]=100;
	if(PWM[MotorNum]<-100)PWM[MotorNum]=-100;
	return PWM[MotorNum];
}

//�ٶȱջ�����
void SpeedInnerControl(void)
{
	for(int i=0;i<4;i++)
	{
		MotorOut[i]=SpeedInnerControlCalculate(SpeedOfWheel[i],MotorOut[i],i);
		SetMotorDutyCycle(MotorOut[i],Channel[i]);
	}
}

//������
void MotorOutput(void)
{
	for(int i=0;i<4;i++)
	{
    MotorOut[i]=AngleControlOut-SpeedControlOut[i];
	//MotorOut[i]=Scale(MotorOut[i], -300, 300, -100, 100);

    //�����������
    if(MotorOut[i]>5.4f)
        MotorOut[i]+=MOTOR_OUT_DEAD_VAL;
    else if(MotorOut[i]<-5.4f)
        MotorOut[i]-=MOTOR_OUT_DEAD_VAL;

    //���ʹ���
    if(MotorOut[i]>MOTOR_OUT_MAX)
        MotorOut[i]=MOTOR_OUT_MAX;
    if(MotorOut[i]<MOTOR_OUT_MIN)
        MotorOut[i]=MOTOR_OUT_MIN;

    SetMotorDutyCycle(MotorOut[i],Channel[i]);
	}
	//SpeedInnerControl();//�ջ����
}

