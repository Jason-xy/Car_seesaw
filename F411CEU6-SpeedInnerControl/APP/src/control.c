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
PID AngleRingPID={6,0,80}; //40 0 20   20  40                  [20,0,40]					[13,0,60]		[11,0,80]			[8,0,80]
PID SpeedRingPID={0,0,0};   //�ٶȿ���50 10 0                   [0,0,0]						[0,0,0]
PID MotorRingPID={3.05,0.53,0};    //����ջ�5 6                      [6,0,0]				[6.6,0.5,0]  [2.5,0.5,0]	[2.8,0.53]
//�Ƕȿ��Ʋ���
float GyroAngleSpeed=0;
float GyroAngleSpeedOld=0;
float GyroAccle=0;
float GyroAccleOld=0;
float CarAngle=0;
float CarAngleOld=0;

//������Ʋ���
float AngleControlOut=0;
float SpeedControlOut=0;
float SpeedControlOutOld=0;
float SpeedControlOutNew=0;
float MotorOut=0;
float ErrorRrev=0;
float PWMBais=0;
float PWM=0;

//�ٶȿ��Ʋ���
short  MotorPulse=0;
int  MotorPulseOld=0;
int  MotorPulseSigma=0;

float CarSpeed=0;
float CarSpeedSet=0;
float CarSpeedOld=0;
float CarPosition=0;

//������岶��																	
float SpeedOfWheel[2]={0};    //��/s

int PWM_OUT_DEAD_VAL=25;


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
    //�Ƕ�
	//��ΧΪ2000deg/sʱ�������ϵ��131.2 LSB/(deg/s)
	//ֱ��ʹ��DMP��̬��������
	if(roll-CarAngle<0.001f||CarAngle-roll<0.001f)
	CarAngle=roll;
	
	//��ͨ�˲�
	GyroAngleSpeed=(CarAngle*1000-CarAngleOld*1000)/5.0f*0.7f+0.3f*GyroAngleSpeedOld;
	GyroAccle=(GyroAngleSpeed-GyroAngleSpeedOld)/0.05f*0.7f+0.3f*GyroAccleOld;
	
	//if(GyroAccle<0)GyroAccle=-GyroAccle;
	
	
	CarAngleOld=CarAngle;                        //[-5~5]
	GyroAngleSpeedOld=GyroAngleSpeed;
	GyroAccleOld=GyroAccle;
}

//�ǶȻ�����
void AngleControl(void)
{
    AngleControlOut=CarAngle*AngleRingPID.P+\
    GyroAngleSpeed*AngleRingPID.D/10.0f;         //������10^1
}

//�ٶȿ�������									//Ŀ��ת���ٶ�0
void SpeedControl(void)
{
  float fP,fI;   	
	float fDelta;	                                     
		
		fDelta = CAR_SPEED_SET;
		fDelta -= CarSpeed;            //������10
		
		fP = fDelta * (SpeedRingPID.P);
		fI = fDelta * (SpeedRingPID.I/10.0f);

		CarPosition += fI;
		
		//������������
		if((short)CarPosition > CAR_POSITION_MAX)    CarPosition = CAR_POSITION_MAX;
		if((short)CarPosition < CAR_POSITION_MIN)    CarPosition = CAR_POSITION_MIN;
		
		SpeedControlOutOld = SpeedControlOutNew;
		SpeedControlOutNew = fP + CarPosition;
}

//�ٶȿ������ƺ���
void SpeedControlOutput(void)
{
  float fValue;
	
		fValue = SpeedControlOutNew - SpeedControlOutOld ;
		SpeedControlOut = fValue * (SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + SpeedControlOutOld;
		
}

//�ٶȱջ����� 2·���
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

//�ٶȱջ�����
void SpeedInnerControl(void)
{
	getSpeed();
	PWM=SpeedInnerControlCalculate(CarSpeed,MotorOut);
	SetMotorDutyCycle(PWM,Channel[0]);
	SetMotorDutyCycle(PWM,Channel[1]);
	SetMotorDutyCycle(PWM,Channel[2]);
	SetMotorDutyCycle(PWM,Channel[3]);
}

//������
void MotorOutput(void)
{
    MotorOut=AngleControlOut-SpeedControlOut/100.0f;

    //�����������
    if(MotorOut>27.0f)
        MotorOut+=MOTOR_OUT_DEAD_VAL;
    else if(MotorOut<-27.0f)
        MotorOut-=MOTOR_OUT_DEAD_VAL;

    //���ʹ���
    if(MotorOut>MOTOR_OUT_MAX)
        MotorOut=MOTOR_OUT_MAX;
    if(MotorOut<MOTOR_OUT_MIN)
        MotorOut=MOTOR_OUT_MIN;
	SpeedInnerControl();//�ٶȱջ�����
}

//������ģʽ��ȡ�ٶ�
void getSpeed(void)
{
	SpeedOfWheel[0] = (short)(__HAL_TIM_GET_COUNTER(&htim4));//�ȶ�ȡ������
	SpeedOfWheel[1] = (short)(__HAL_TIM_GET_COUNTER(&htim3));
	__HAL_TIM_SET_COUNTER(&htim4,0);//�ټ���������
	__HAL_TIM_SET_COUNTER(&htim3,0);
	CarSpeed = (SpeedOfWheel[0]+SpeedOfWheel[1])/2;
	CarSpeed = 0.7f * CarSpeedOld + 0.3f * CarSpeed ;//��ͨ�˲���ʹ�ٶȸ�ƽ��
	CarSpeedOld = CarSpeed;
}

