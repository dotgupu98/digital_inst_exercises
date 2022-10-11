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
uint32_t map1=0;
uint32_t map2=0;

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

void LCD_data_print(void){
	lcd_write_string((uint8_t*)"ADC data", fbuffer, 20, 0);
	sprintf(str,"Pot1: %0.3f ms",(double)map1/1000);
	lcd_write_string(str, fbuffer, 20, 2);
	sprintf(str,"Pot2: %0.3f ms",(double)map2/1000);
	lcd_write_string(str, fbuffer, 20, 3);
	lcd_push_buffer(fbuffer);
}

void timer16_pwm_init(void){
	//step1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
	//step3
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_ClockDivision = 0;
	TIM_InitStructure.TIM_Period = 20000; //set the maximum period
	TIM_InitStructure.TIM_Prescaler = 64; //for 1MHz counting frequency
	TIM_TimeBaseInit(TIM16,&TIM_InitStructure);
	//step4
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	//TIM_OCInitStruct.TIM_Pulse = 0x01f4; // 0x0000 <-> 0xFFFF
	TIM_OCInitStruct.TIM_Pulse = 1500; // 0 - 20000, 1000-2000
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	//step5
	TIM_OC1Init(TIM16,&TIM_OCInitStruct);
	//step6
	TIM_OC1PreloadConfig(TIM16,TIM_OCPreload_Enable);
	//step7
	TIM_CtrlPWMOutputs(TIM16, ENABLE);
	TIM_Cmd(TIM16,ENABLE);
}

void GPIO_set_AF1_PA6(void){
	//step2
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port B
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins
	GPIO_StructInit(&GPIO_InitStructAll);
	// Then set things that are not default.
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_6;
	//GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
	//GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);
}

void timer2_pwm_init(void){
	//step1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//step3
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_ClockDivision = 0;
	TIM_InitStructure.TIM_Period = 20000; //set the maximum period
	TIM_InitStructure.TIM_Prescaler = 64; //for 1MHz counting frequency
	TIM_TimeBaseInit(TIM2,&TIM_InitStructure);
	//step4
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 1500; // 0 - 20000, 1000-2000
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	//step5
	TIM_OC4Init(TIM2,&TIM_OCInitStruct);
	//step6
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
	//step7
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}

void GPIO_set_AF1_PB11(void){
	//step2
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port B
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins
	GPIO_StructInit(&GPIO_InitStructAll);
	// Then set things that are not default.
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1);
}

int main(void)
{
	uart_init(9600);
	init_spi_lcd();
	memset(fbuffer,0x00,512); // Sets each element of the buffer to 0xAA
	lcd_push_buffer(fbuffer);
	ADC_setup_pA();
	ADC_Calibrate();
	GPIO_set_AF1_PA6();
	timer16_pwm_init();
	GPIO_set_AF1_PB11();
	timer2_pwm_init();
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);

	while(1){
		printf("alma\n");
		adc1 = ADC_measure_PA(1);
		adc2 = ADC_measure_PA(2);
		map1 = 1000+(adc1*1000)/4096;
		map2 = 1000+(adc2*1000)/4096;
		//printf("%d\n",map1);
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_Pulse = map1;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC1Init(TIM16,&TIM_OCInitStruct);

		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_Pulse = map2;
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC4Init(TIM2,&TIM_OCInitStruct);

		LCD_data_print();
	}
}
