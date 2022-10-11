#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" 		// Input/output library for this course
#include "flash.h"
#include "lcd.h"
#include <string.h>

//uint8_t positions[4] = { 0b0101, 0b1001, 0b1010, 0b0110 };
uint16_t interval=100;
uint8_t current_step=0;

void initTimer2(void){
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
	TIM2->ARR=interval; //count up to 100
	TIM2->DIER |= 0x0001; // Enable timer 2 interrupts
    //NVIC settings
	NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority interrupts
	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt
	TIM_Cmd(TIM2,DISABLE);
}

void TIM2_IRQHandler(void) { //timer interrupt handler
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){ //if interrupt occurs
		stepper_func(current_step);
		current_step=(current_step+1)%4;
		//printf("%d\n", current_step);
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // Clear interrupt bit
	}
}

void initGPIO(void){
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); // Enable clock for GPIO Port C
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A

	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

	// Sets PA9 to output
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_5; // Set so the configuration is on pin 9
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PA9 to output
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_6; // Set so the configuration is on pin 9
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PA9 to output
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_7; // Set so the configuration is on pin 9
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PA9 to output
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_1; // Set so the configuration is on pin 9
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
}

void stepper_func(uint8_t step){
	switch(step){
		case 0 :
			GPIO_WriteBit(GPIOA , GPIO_Pin_5, 0); //set red led to enabled or disabled
			GPIO_WriteBit(GPIOA , GPIO_Pin_6, 1); //set green led to enabled or disabled
			GPIO_WriteBit(GPIOA , GPIO_Pin_7, 0); //set blue led to enabled or disabled
			GPIO_WriteBit(GPIOB , GPIO_Pin_1, 1); //set red led to enabled or disabled
		break;

		case 1 :
			GPIO_WriteBit(GPIOA , GPIO_Pin_5, 1); //set red led to enabled or disabled
			GPIO_WriteBit(GPIOA , GPIO_Pin_6, 0); //set green led to enabled or disabled
			GPIO_WriteBit(GPIOA , GPIO_Pin_7, 0); //set blue led to enabled or disabled
			GPIO_WriteBit(GPIOB , GPIO_Pin_1, 1); //set red led to enabled or disabled
		break;

		case 2 :
			GPIO_WriteBit(GPIOA , GPIO_Pin_5, 1); //set red led to enabled or disabled
			GPIO_WriteBit(GPIOA , GPIO_Pin_6, 0); //set green led to enabled or disabled
			GPIO_WriteBit(GPIOA , GPIO_Pin_7, 1); //set blue led to enabled or disabled
			GPIO_WriteBit(GPIOB , GPIO_Pin_1, 0); //set red led to enabled or disabled
		break;

		case 3 :
			GPIO_WriteBit(GPIOA , GPIO_Pin_5, 0); //set red led to enabled or disabled
			GPIO_WriteBit(GPIOA , GPIO_Pin_6, 1); //set green led to enabled or disabled
			GPIO_WriteBit(GPIOA , GPIO_Pin_7, 1); //set blue led to enabled or disabled
			GPIO_WriteBit(GPIOB , GPIO_Pin_1, 0); //set red led to enabled or disabled
		break;
	}
}

int main(void)
{
	uart_init(9600);
	initGPIO();
	initTimer2();
	TIM_Cmd(TIM2,ENABLE);

	while(1){
		switch(uart_get_char()){
		case 'w' : if(interval<=1000){interval+=1;TIM2->ARR=interval;printf("%d\n",interval);};
		break;

		case 's' : if(interval>30){interval-=1;TIM2->ARR=interval;printf("%d\n",interval);}
		break;
		}
	}
}
