#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" 		// Input/output library for this course
#include "flash.h"
#include "lcd.h"
#include <string.h>

#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))
float volatile x_global=0;
float volatile y_global=0;
float volatile V_ABS;
char  str[16];
uint8_t volatile fbuffer[512];
uint16_t volatile x;
uint16_t volatile y;

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

	//timerStat=1; //extra variable to keep timer state

	RCC->APB1ENR |= RCC_APB1Periph_TIM2; // Enable clock line to timer 2
	TIM2->CR1=0xB01;
	TIM2->PSC=6399; //change pre-scaler frequency to 10kHz
	TIM2->ARR=9999; //count up to 100
	TIM2->DIER |= 0x0001; // Enable timer 2 interrupts

	NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority interrupts
	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt
	TIM_Cmd(TIM2,ENABLE);
}

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
}

//void EXTI9_5_IRQHandler(void){ // for pausing timer
//	if(EXTI_GetITStatus(EXTI_Line5) != RESET){
//		//printf("test\n");
//		for(uint8_t j =0;j<16;j++){
//
//			//for(uint32_t i = 0; i<100000;i++);
//			// set channel 1 to active
//			printf("test\n");
//			ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
//			ADC_StartConversion(ADC1); // Start ADC read
//			while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
//			x_global += (float)ADC_GetConversionValue(ADC1) ; // save measured data
//
//			//for(uint32_t i = 0; i<100000;i++);
//			// set channel 2 to active
//			//printf("%d\n",j);
//			ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_1Cycles5);
//			ADC_StartConversion(ADC1); // Start ADC read
//			while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
//			y_global += (float)ADC_GetConversionValue(ADC1) ; // save measured data
//
//		}
//		//printf("test\n");
//		x_global = x_global/16*V_ABS;
//		y_global = y_global/16*V_ABS;
//		//printf("values\n");
//
//
//
////			FLASH_Unlock(); //unlock flash
////
////			//ERASE FLASH
////			FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
////
////			//WRITE UINT8 TO FLASH
////			FLASH_ErasePage(PG31_BASE); //erase flash
////			write_float_flash(PG31_BASE, 0, x_global);
////			write_float_flash(PG31_BASE, 1, y_global);
////
////			FLASH_Lock();  //lock flash
//
//		EXTI_ClearITPendingBit(EXTI_Line5);
//	}
//	//printf("exti\n");
//	//printf("PA0: %f, PA1 %f\n",x_global,y_global);
//	//printf("PA0: %f, PA1 %f\n",x_global,y_global);
//}

//void TIM2_IRQHandler(void) { //timer interrupt handler
//	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){ //if interrupt occurs
//		printf("a\n");
//		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // Clear interrupt bit
//	}
//}

void TIM2_IRQHandler(void) { //timer interrupt handler
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){ //if interrupt occurs
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // Clear interrupt bit
		//char str[7];
		//uint8_t fbuffer[512];
		//for(uint32_t i = 0; i<100000;i++);
		// set channel 1 to active
		ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
		ADC_StartConversion(ADC1); // Start ADC read
		while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
		x = ADC_GetConversionValue(ADC1) ; // save measured data

		// set channel 2 to active
		ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_1Cycles5);
		ADC_StartConversion(ADC1); // Start ADC read
		while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
		y = ADC_GetConversionValue(ADC1) ; // save measured data

		//printf( "a\n" );
				//printf("Pot1: %0.2f, Pot2: %0.2f\n",(double)x* V_ABS, (double)y* V_ABS);

		printf("PA0: %f, PA1 %f\n",x_global,y_global);
		//memset(fbuffer,0x00,512);
				lcd_write_string((uint8_t*)"ADC data", fbuffer, 0, 0);
				sprintf(str,"Pot1: %0.3f",(double)x* (double)V_ABS*((double)3.2/(double)x_global));
				lcd_write_string(str, fbuffer, 0, 1);
				sprintf(str,"Pot1: %0.3f",(float)y* (float)V_ABS*((float)3.2/(float)y_global));
				lcd_write_string(str, fbuffer, 0, 2);
				sprintf(str,"%0.3f, %0.3f",x_global,y_global);
				lcd_write_string(str, fbuffer, 0, 3);
				lcd_push_buffer(fbuffer);

		//TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // Clear interrupt bit
	}
}

int main(void)
{
	init_spi_lcd();
	uart_init(9600);
	printf(".................................................\n");

	//uint8_t fbuffer[512];
	memset(fbuffer,0x00,512); // Sets each element of the buffer to 0xAA
	lcd_push_buffer(fbuffer);

	uint32_t address = 0x0800F800;
	float tempVal;
	for ( int i = 0 ; i < 2 ; i++ ){
	tempVal = *(float *)(address + i * 4); // Read Command
	printf("%f \n", tempVal);
	}

	x_global = *(float *)(address + 0 * 4);
	y_global = *(float *)(address + 1 * 4);

	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div8); //adc clk
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE); //enable adc

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); //gpio clock
	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN; // Set as input
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0; // Set so the configuration is on pin 4
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AN; // Set as input
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_1; // Set so the configuration is on pin 4
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	ADC_InitTypeDef ADC_InitStructAll; //struct for adc config

	ADC_StructInit(&ADC_InitStructAll); //settings for the adc
	ADC_InitStructAll.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructAll.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructAll.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructAll.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructAll.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1,&ADC_InitStructAll); // init the adc settings
	ADC_Cmd(ADC1,ENABLE); //enable adc

	// set internal reference voltage source and wait
	ADC_VoltageRegulatorCmd(ADC1,ENABLE);
	//Wait for at least 10uS before continuing...
	for(uint32_t i = 0; i<10000;i++);

	ADC_Cmd(ADC1,DISABLE);
	while(ADC_GetDisableCmdStatus(ADC1)){} // wait for disable of ADC

	ADC_SelectCalibrationMode(ADC1,ADC_CalibrationMode_Single); //select calibration mode
	ADC_StartCalibration(ADC1); //calibrate adc

	while(ADC_GetCalibrationStatus(ADC1)){} //wait for calibration

	for(uint32_t i = 0; i<100;i++);//wait more

	//2.5 from here

	ADC_VrefintCmd(ADC1,ENABLE); // setup ref voltage to channel 18
	for(uint32_t i = 0; i<10000;i++); // wait for some time

	ADC_Cmd(ADC1,ENABLE);// turn on ADC
	while((!ADC_GetFlagStatus(ADC1,ADC_FLAG_RDY))){	} //wait for adc to turn on



	ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_19Cycles5);
	//wait for 2.2us
	ADC_StartConversion(ADC1); // Start ADC read
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
	uint16_t VREFINT_DATA = ADC_GetConversionValue(ADC1); // save measured data
	// calculate the voltage/adc step
	V_ABS = ((3.3 * (VREFINT_CAL / VREFINT_DATA)) / 4095);

	char str[7];

	//initInterrupt();
	initTimer();
	while(1){
//		for(uint32_t i = 0; i<100000;i++);
//		printf("PA0: %f, PA1 %f\n",x_global,y_global);
//
//		lcd_write_string((uint8_t*)"ADC data", fbuffer, 20, 0);
//		sprintf(str,"Pot1: %0.3f",(double)x* V_ABS);
//		lcd_write_string(str, fbuffer, 20, 2);
//		sprintf(str,"Pot1: %0.3f",(double)y* V_ABS);
//		lcd_write_string(str, fbuffer, 20, 3);
//		sprintf(str,"val %0.3f",x_global);
//		lcd_write_string(str, fbuffer, 20, 1);
//		lcd_push_buffer(fbuffer);
	}
}
