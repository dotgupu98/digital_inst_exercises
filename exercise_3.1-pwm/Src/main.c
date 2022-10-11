#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" 		// Input/output library for this course
#include "flash.h"
#include "lcd.h"

void timer16_pwm_init(void){
	//step1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
	//step3
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_ClockDivision = 0;
	TIM_InitStructure.TIM_Period = 1000; //set the maximum period
	TIM_InitStructure.TIM_Prescaler = 64; //for 1MHz counting frequency
	TIM_TimeBaseInit(TIM16,&TIM_InitStructure);
	//step4
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; //pwm mode
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 250; //25% pwm
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High; // polarity
	//step5
	TIM_OC1Init(TIM16,&TIM_OCInitStruct);
	//step6
	TIM_OC1PreloadConfig(TIM16,TIM_OCPreload_Enable);
	//step7
	TIM_CtrlPWMOutputs(TIM16, ENABLE); //pwm output enable
	TIM_Cmd(TIM16,ENABLE); // timer enable
}

void GPIO_set_AF1_PA6(void){
	//step2
	// Enable clock for GPIO Port B
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins
	GPIO_StructInit(&GPIO_InitStructAll);
	// Then set things that are not default.
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	// Setup of GPIO with the settings chosen
	GPIO_Init(GPIOB, &GPIO_InitStructAll);
	//set gpio af
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);
}

int main(void)
{
	uart_init(9600);
	GPIO_set_AF1_PA6();
	timer16_pwm_init();
	while(1){

	}
}

//int main(void)
//{
//	uart_init(9600);
//	GPIO_set_AF1_PA6();
//	timer16_pwm_init();
//	TIM_OCInitTypeDef TIM_OCInitStruct;
//	while(1){
//		for(int i=0;i<1000;i++){
//			for(int j=0;j<50000;j++);
//			TIM_OCInitStruct.TIM_Pulse = i;
//			TIM_OC1Init(TIM16,&TIM_OCInitStruct);
//		}
//	}
//}
