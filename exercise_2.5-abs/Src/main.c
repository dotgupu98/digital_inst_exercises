#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" 		// Input/output library for this course
#include "flash.h"
#include "lcd.h"
#include <string.h>

#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))
float volatile V_ABS;
char  str[16];
uint8_t fbuffer[512];
uint16_t volatile adc1;
uint16_t volatile adc2;
uint8_t volatile ADCupdate=0;

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
	//Settings timer
	RCC->APB1ENR |= RCC_APB1Periph_TIM2; // Enable clock line to timer 2
	TIM2->CR1=0xB01;
	TIM2->PSC=6399; //change pre-scaler frequency to 10kHz
	TIM2->ARR=999; //count up to 100
	TIM2->DIER |= 0x0001; // Enable timer 2 interrupts
    //NVIC settings
	NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority interrupts
	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt
	TIM_Cmd(TIM2,DISABLE);
}

void ADC_setup_pA(void){
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
}

void ADC_Calibrate(){
	ADC_VoltageRegulatorCmd(ADC1,ENABLE);
	//Wait for at least 10uS before continuing...
	for(uint32_t i = 0; i<10000;i++);

	ADC_Cmd(ADC1,DISABLE);
	while(ADC_GetDisableCmdStatus(ADC1)){} // wait for disable of ADC

	ADC_SelectCalibrationMode(ADC1,ADC_CalibrationMode_Single); //select calibration mode
	ADC_StartCalibration(ADC1); //calibrate adc
	while(ADC_GetCalibrationStatus(ADC1)){} //wait for calibration
	for(uint32_t i = 0; i<100;i++);//wait more

	ADC_VrefintCmd(ADC1,ENABLE); // setup ref voltage to channel 18
	for(uint32_t i = 0; i<10000;i++); // wait for some time

	ADC_Cmd(ADC1,ENABLE);// turn on ADC
	while((!ADC_GetFlagStatus(ADC1,ADC_FLAG_RDY))){	} //wait for adc to turn on

	ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_19Cycles5); //wait for 2.2us
	ADC_StartConversion(ADC1); // Start ADC read
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read

	uint16_t VREFINT_DATA = ADC_GetConversionValue(ADC1); // save measured data
	V_ABS = ((3.3 * (VREFINT_CAL / VREFINT_DATA)) / 4095); // calculate the voltage/adc step
}

uint16_t ADC_measure_PA(uint8_t channel){
	uint16_t x;
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);
	ADC_StartConversion(ADC1); // Start ADC read
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
	x = ADC_GetConversionValue(ADC1) ; // savemeasured data
	return x;
}

void TIM2_IRQHandler(void) { //timer interrupt handler
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){ //if interrupt occurs
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // Clear interrupt bit
		ADCupdate=1;
	}
}

void LCD_data_print(void){
	lcd_write_string((uint8_t*)"ADC data", fbuffer, 20, 0);
	sprintf(str,"Pot1: %0.3f",(double)adc1* (double)V_ABS);
	lcd_write_string(str, fbuffer, 20, 2);
	sprintf(str,"Pot2: %0.3f",(float)adc2* (float)V_ABS);
	lcd_write_string(str, fbuffer, 20, 3);
	lcd_push_buffer(fbuffer);
}

//void LCD_data_print(void){
//	lcd_write_string((uint8_t*)"ADC data", fbuffer, 20, 0);
//	sprintf(str,"Pot1: %0.3f",(double)adc1* (double)V_ABS*((double)3.2/(double)x_global));
//	lcd_write_string(str, fbuffer, 20, 2);
//	sprintf(str,"Pot2: %0.3f",(float)adc2* (float)V_ABS*((double)3.2/(double)y_global));
//	lcd_write_string(str, fbuffer, 20, 3);
//	lcd_push_buffer(fbuffer);
//}

int main(void)
{
	uart_init(9600);
	ADC_setup_pA();
	ADC_Calibrate();
	initTimer();
	init_spi_lcd();
	memset(fbuffer,0x00,512); // Sets each element of the buffer to 0xAA
	lcd_push_buffer(fbuffer);
	TIM_Cmd(TIM2,ENABLE);
	while(1){
		if(ADCupdate == 1){
			adc1 = ADC_measure_PA(1);
			adc2 = ADC_measure_PA(2);
			LCD_data_print();
			ADCupdate=0;
		}
	}
}
