#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course

#include "gpio.h"

struct Time { //struct for timer data
	uint8_t volatile hours;
	uint8_t volatile minutes;
	uint8_t volatile seconds;
	uint8_t volatile seconds100;
};

struct Time timerTime; //declaration of timer struct

uint8_t timerStat; //variable for timer state

void initInterrupt(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource5); // sets port B pin 5 to the IRQ
	// define and set setting for EXTI
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	// setup NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource0); // sets port C pin 0 to the IRQ
	// define and set setting for EXTI
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	// setup NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource4); // sets port A pin 4 to the IRQ
	// define and set setting for EXTI
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	// setup NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource1); // sets port C pin 1 to the IRQ
	// define and set setting for EXTI
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	// setup NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
}

void initTimer(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;

	// NVIC for timer
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2,ENABLE);

	timerStat=1; //extra variable to keep timer state

	RCC->APB1ENR |= RCC_APB1Periph_TIM2; // Enable clock line to timer 2
	TIM2->CR1=0xB01;
	TIM2->PSC=6399; //change pre-scaler frequency to 10kHz
	TIM2->ARR=99; //count up to 100
	TIM2->DIER |= 0x0001; // Enable timer 2 interrupts

	NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority interrupts
	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt
	TIM_Cmd(TIM2,DISABLE);
}

void showTime(void){ // format the output data
	printf("%d: %d: %d: %d\n",timerTime.hours, \
							timerTime.minutes, \
							timerTime.seconds, \
							timerTime.seconds100);
	//print the timer value to the console
}

void TIM2_IRQHandler(void) { //timer interrupt handler
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){ //if interrupt occurs
//	timerTime.seconds100 += 1;
//		if( timerTime.seconds100 == 100 ){
//			timerTime.seconds100 = 0;
//			timerTime.seconds += 1;
//			if(timerTime.seconds == 60){
//				timerTime.seconds = 0;
//				timerTime.minutes += 1;
//				if(timerTime.minutes == 60){
//					timerTime.minutes = 0;
//					timerTime.hours += 1;
//				}
//			}
//		} //store the timer data in the struct
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // Clear interrupt bit
//		if( timerTime.seconds100 == 0 ){ // printf every second
//			showTime();
//			printf("a\n");
//		}
		printf("a\n");
	}
}

void EXTI9_5_IRQHandler(void){ // for pausing timer
	if(EXTI_GetITStatus(EXTI_Line5) != RESET){
		if(timerStat==1){
			showTime();
			TIM_Cmd(TIM2,DISABLE); //stop timer
			timerStat=0;
		}
		else{
			TIM_Cmd(TIM2,ENABLE); //start timer
			timerStat=1;
		}
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

void EXTI4_IRQHandler(void){ // reset the timer
	if(EXTI_GetITStatus(EXTI_Line4) != RESET){
		timerTime.seconds100 = 0; // set timer struct to 0
		timerTime.seconds = 0;
		timerTime.minutes = 0;
		timerTime.hours = 0;
		TIM_Cmd(TIM2,DISABLE);
		timerStat=0;
		showTime(); //printf the current time
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

int main(void)
{
	initJoystick();
	initLed();
	initInterrupt();
	initTimer();
	uart_init( 9600 ); // Initialize USB serial at 9600 baud

	while(1){

	}
}
