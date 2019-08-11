 /**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f30x.h"
#include <stm32f30x_gpio.h>
#include "stm32f3xx_nucleo.h"
#define  ARM_MATH_CM4
#include "arm_math.h"
#include "arm_const_structs.h"


#define DATASIZE 180
#define PRESCALER_VALUE 5
#define DEAD_TIME_NANO_SEC 3000.0
uint32_t f_carry = 800;


uint32_t counter1 = 0;
uint32_t counter2 = 0;
uint32_t theta1 = 0;
uint16_t Uref = 100,Vref = 100,Wref = 100;
float sine[DATASIZE];
uint16_t flag = 0;


static void NVIC_PriorityGroup_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); //Use only Subpriority
/*	---------------------| PreemptionPriority | SubPriority |
	---------------------------------------------------------
	NVIC_PriorityGroup_0 |      0 (0bit)      | 0-15 (4bit) |
	NVIC_PriorityGroup_1 |     0-1 (1bit)     | 0-7 (3bit)  |
	NVIC_PriorityGroup_2 |     0-3 (2bit)     | 0-3 (2bit)  |
	NVIC_PriorityGroup_3 |     0-7 (3bit)     | 0-1 (1bit)  |
	NVIC_PriorityGroup_4 |     0-15(4bit)     |  0  (0bit)  |
	---------------------------------------------------------*/
}

static void LED_Blink_Init(void)
{
	//GPIOB Clocks enable
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	//GPIOSetting
	GPIO_InitTypeDef pin_PB13_12;
	pin_PB13_12.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_12; //LED2:PB13, For check terminal :PB12
	pin_PB13_12.GPIO_Mode = GPIO_Mode_OUT;			//OUT
	pin_PB13_12.GPIO_OType = GPIO_OType_PP;			//Push-Pull mode
	pin_PB13_12.GPIO_PuPd = GPIO_PuPd_DOWN;			// Pull-DOWN
	pin_PB13_12.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB,&pin_PB13_12);
}

static void TIM1_Init(uint32_t f_car)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	uint16_t TimerPeriod = 0;	//variable for compute carrier freq. MIN 98[Hz]@PRESCALER_VALUE=5
	uint16_t Channel1Pulse = 0 ,Channel2Pulse = 0,Channel3Pulse = 0,Channel4Pulse = 0; //var for compute Duty
	uint16_t DeadTimePeriodCalc = 0;//variable for compute Dead-Time.
	uint8_t  DeadTimePeriod = 0; //variable for compute Dead-Time.
	//GPIOA,B Clocks enable
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA |RCC_AHBPeriph_GPIOB,ENABLE);

	//GPIOA Config : Channel1N,1P,2P,3P and 4 as AF(alternate function) push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11; //1N,1P,2P,3P,4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;													//Alternative Function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;													//Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;													//Pull-down
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_11);

	//GPIOB Config : Channel2N,3N as AF(alternate function) push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //2N,3N
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_6);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_6);

	//TIM1 Config

	//Compute the value to be set in ARR register to generate signal frequency at fcar[Hz]
	SystemCoreClockUpdate();
//	TimerPeriod = (64000000 / ((uint16_t)f_car * 8) - 1 );
	TimerPeriod = (SystemCoreClock / ((uint16_t)f_car * PRESCALER_VALUE * 2)  ) - 1;
	// Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N
	Channel1Pulse = (uint16_t) (((uint32_t)  (TimerPeriod - 1)) / 2);
	// Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 and 2N
	Channel2Pulse = (uint16_t) (((uint32_t) 375 * (TimerPeriod - 1)) / 1000);
	// Compute CCR3 value to generate a duty cycle at 25%  for channel 3 and 3N
	Channel3Pulse = (uint16_t) (((uint32_t) 250 * (TimerPeriod - 1)) / 1000);
	// Compute CCR4 value to generate a duty cycle at 12.5%  for channel 4
	Channel4Pulse = (uint16_t) (((uint32_t) 125 * (TimerPeriod - 1)) / 1000);

	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER_VALUE -1 ;//Prescaler 1/(PRESCALER_VALUE + 1)
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;							//Update Event prescaler
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	//Channel 1,2,3 and 4 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	//Dead-time and Break Setting

//	TIM_DeadTime[7:5]=0xx => DT=TIM_DeadTime[7:0]x tdtg with tdtg=tDTS
//	TIM_DeadTime[7:5]=10x => DT=(64+TIM_DeadTime[5:0])xtdtg with Tdtg=2xtDTS
//	TIM_DeadTime[7:5]=110 => DT=(32+TIM_DeadTime[4:0])xtdtg with Tdtg=8xtDTS
//	TIM_DeadTime[7:5]=111 => DT=(32+TIM_DeadTime[4:0])xtdtg with Tdtg=16xtDTS

	DeadTimePeriodCalc = (uint16_t)((DEAD_TIME_NANO_SEC / 1000000000.0) * SystemCoreClock ) ;
	if(DeadTimePeriodCalc >= 0 && DeadTimePeriodCalc < 128 )
	{
		DeadTimePeriod = DeadTimePeriodCalc;
	}else if(DeadTimePeriodCalc >= 128 && DeadTimePeriodCalc < 256)
	{
		DeadTimePeriodCalc = (DeadTimePeriodCalc / 2) - 64;
		DeadTimePeriod = ((DeadTimePeriodCalc & 0b00111111 ) | 0b10000000);
	}

	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = 0b10100000;//3000[ns]@sysclock=(1/tDTS)=64[MHz]
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);


	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	// TIM1 counter enable
	TIM_Cmd(TIM1, ENABLE);
	//TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

}

void TIM1_Update(uint32_t Channel1Duty, uint32_t Channel2Duty, uint32_t Channel3Duty ,uint32_t Channel4Duty, uint32_t f_car)
{

//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;

	uint16_t TimerPeriod = 0;//variable for compute carrier freq. MIN 98[Hz]@PRESCALER_VALUE=5
	uint16_t Channel1Pulse = 0 ,Channel2Pulse = 0,Channel3Pulse = 0,Channel4Pulse = 0;
	//TIM1 Config:
	//Compute the value to be set in ARR register to generate signal frequency at fcar[Hz]
//	SystemCoreClockUpdate();
	TimerPeriod = (SystemCoreClock / ((uint16_t)f_car * PRESCALER_VALUE * 2 )  ) - 1;
	// Compute CCR1 value to generate a duty cycle for channel 1 and 1N
	Channel1Pulse = (uint16_t) ((  Channel1Duty * (TimerPeriod - 1)) / 1000);
	// Compute CCR2 value to generate a duty cycle for channel 2 and 2N
	Channel2Pulse = (uint16_t) ((  Channel2Duty * (TimerPeriod - 1)) / 1000);
	// Compute CCR3 value to generate a duty cycle for channel 3 and 3N
	Channel3Pulse = (uint16_t) ((  Channel3Duty * (TimerPeriod - 1)) / 1000);
	// Compute CCR4 value to generate a duty cycle for channel 4
	Channel4Pulse = (uint16_t) (( Channel4Duty * (TimerPeriod - 1)) / 1000);

	TIM1->ARR = TimerPeriod;	//Reconfigure Auto Reload Register

	TIM1->CCR1 = Channel1Pulse;	//Reconfigure Capture/Compare Register 1
	TIM1->CCR2 = Channel2Pulse; //Reconfigure Capture/Compare Register 2
	TIM1->CCR3 = Channel3Pulse; //Reconfigure Capture/Compare Register 3
	TIM1->CCR4 = Channel4Pulse; //Reconfigure Capture/Compare Register 4

}

void TIM1_IntrptInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //0 is highest priority, order to number.
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void TIM6_Init(uint32_t f_cont)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 ,ENABLE);
	SystemCoreClockUpdate();
	TIM_TimeBaseStructure.TIM_Period = 6;//;
	TIM_TimeBaseStructure.TIM_Prescaler = 64000000/6400 -1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6,ENABLE);
}

static void TIM6_IntrptInit()
{
	NVIC_InitTypeDef NVIC_InitStructure;


	NVIC_InitStructure.NVIC_IRQChannel =  TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //0 is highest priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

int main(void)
{
	for(uint32_t i=0;i<DATASIZE;i++)
		{
		sine[i] = arm_sin_f32((float)i *( 2.0 * PI / (float)DATASIZE) );
		}
	f_carry = 98; //[Hz]
	uint32_t f_cont = 1;//[Hz]

	NVIC_PriorityGroup_Init();
	LED_Blink_Init();
	TIM6_Init(f_cont);
	TIM6_IntrptInit();
	TIM1_Init(f_carry);
	TIM1_IntrptInit();

	for(;;){
	};
}

void TIM1_UP_TIM16_IRQHandler(void)//use TIM1 IRQHandler
{

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //Do Not delete
	  {
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);		//Do Not delete

	    if(flag == 0)
	     {
	 		GPIO_WriteBit(GPIOB,GPIO_Pin_13|GPIO_Pin_12,Bit_SET);
	 		flag = 1;
	     }else{
	     	GPIO_WriteBit(GPIOB,GPIO_Pin_13|GPIO_Pin_12,Bit_RESET);
	     	flag = 0;
	     }
		counter1++;
//		f_carry = f_carry + 1;
		TIM1_Update(Uref,Vref,Wref,0,f_carry);

		if(counter1 > 10){
//		    f_carry = f_carry + 1;
	//		TIM1_Update(Uref,Vref,Wref,0,f_carry);
		    counter1 = 0;
		}
		if(f_carry > 20000){
			f_carry = 100;
		}
	  }

}

void TIM6_DAC_IRQHandler(void)//use TIM6 IRQHandler
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)	//Do Not delete
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);		//Do Not delete

		counter2++;
		if(counter2 > 2){
			theta1++;
			counter2 = 0;
		}
		if(theta1>DATASIZE)
		{
			theta1 = 0;
		}

		Uref =(uint16_t)((0.5*sine[theta1] +0.5) * 1000);
		Vref =(uint16_t)((0.5*sine[theta1] +0.5) * 1000);
		Wref =(uint16_t)((0.5*sine[theta1] +0.5) * 1000);
	}
}


