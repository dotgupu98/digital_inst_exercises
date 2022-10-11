#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course

struct Time {
	uint8_t volatile hours;
	uint8_t volatile minutes;
	uint8_t volatile seconds;
	uint8_t volatile seconds100;
};

struct Time timerTime, timerSplit1, timerSplit2;

uint8_t timerStat;
uint32_t time=0;

void initJoystick(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); // Enable clock for GPIO Port C
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A

	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_4; // Set so the configuration is on pin 4
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0; // Set so the configuration is on pin 0
	GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_5; // Set so the configuration is on pin 5
	GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0; // Set so the configuration is on pin 0
	GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_IN; // Set as input
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN; // Set as pull down
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_1; // Set so the configuration is on pin 1
	GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

}

void initLed(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE); // Enable clock for GPIO Port C
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port B
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A

	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

	// Sets PA9 to output
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_9; // Set so the configuration is on pin 9
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PA9 to output
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_4; // Set so the configuration is on pin 9
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PA9 to output
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_7; // Set so the configuration is on pin 9
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	GPIO_WriteBit(GPIOB , GPIO_Pin_4, Bit_SET);
	GPIO_WriteBit(GPIOC , GPIO_Pin_7, Bit_SET);
	GPIO_WriteBit(GPIOA , GPIO_Pin_9, Bit_SET);
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

initTimer(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	/*TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//TIM_InitStructure.TIM_Period = 6138;
	TIM_InitStructure.TIM_Period = 6399;
	TIM_InitStructure.TIM_Prescaler = 100;
	TIM_TimeBaseInit(TIM2,&TIM_InitStructure);*/
	// NVIC for timer
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	timerStat=1;
	TIM_Cmd(TIM2,ENABLE);

	////////////////////////////////////////////////////////////////////////////
	//RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1Periph_TIM2; // Enable clock line to timer 2
	TIM2->CR1=0xB01;
	TIM2->PSC=6399;
	TIM2->ARR=99;
	TIM2->DIER |= 0x0001; // Enable timer 2 interrupts
	//TIM2->CNT=1000;
	//TIM2->CR1=0xB01;




	NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority interrupts
	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt
	TIM_Cmd(TIM2,DISABLE);
}

uint8_t readJoystick(void){
	uint8_t up=GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_4);
	uint8_t down=GPIO_ReadInputDataBit ( GPIOB, GPIO_Pin_0);
	uint8_t left=GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_1);
	uint8_t right=GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_0);
	uint8_t center=GPIO_ReadInputDataBit ( GPIOB, GPIO_Pin_5);

	uint8_t array = (up << 7) | (down << 6) | (left << 5) | (right << 4) | (center << 3) | (0 << 2) | (0 << 1) | (0 << 0);

	return(array);
}

setLed(uint8_t red, uint8_t green, uint8_t blue){
	GPIO_WriteBit(GPIOB , GPIO_Pin_4, red);
	GPIO_WriteBit(GPIOC , GPIO_Pin_7, green);
	GPIO_WriteBit(GPIOA , GPIO_Pin_9, blue);
}

/*void TIM2_IRQHandler(void) {
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){
	//Do whatever you want here, but make sure it doesn’t take too much time
	timerTime.seconds100 += 1;
	if( timerTime.seconds100 == 100 ){
		timerTime.seconds100 = 0;
		timerTime.seconds += 1;
		printf("%d: %d: %d: %d\n",timerTime.hours, timerTime.minutes, timerTime.seconds, timerTime.seconds100);
		if(timerTime.seconds == 60){
			timerTime.seconds = 0;
			timerTime.minutes += 1;
			if(timerTime.minutes == 60){
				timerTime.minutes = 0;
				timerTime.hours += 1;
			}
		}
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // Clear interrupt bit
}}*/

void TIM2_IRQHandler(void) {
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){
	//Do whatever you want here, but make sure it doesn’t take too much time
	time += 1;
	if( time % 100 == 0 ){
		printf("%d\n",time);
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // Clear interrupt bit
}}

void EXTI9_5_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line5) != RESET){
		if(timerStat==1){
			TIM_Cmd(TIM2,DISABLE);
			timerStat=0;
		}
		else{
			TIM_Cmd(TIM2,ENABLE);
			timerStat=1;
		}
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

void EXTI0_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){
		timerSplit1.hours = timerTime.hours;
		timerSplit1.minutes = timerTime.minutes;
		timerSplit1.seconds = timerTime.seconds;
		timerSplit1.seconds100 = timerTime.seconds100;
		printf("%d: %d: %d: %d\n",timerSplit1.hours, timerSplit1.minutes, timerSplit1.seconds, timerSplit1.seconds100);

		EXTI_ClearITPendingBit(EXTI_Line0);

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}

void EXTI1_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line1) != RESET){
		timerSplit2.hours = timerTime.hours;
		timerSplit2.minutes = timerTime.minutes;
		timerSplit2.seconds = timerTime.seconds;
		timerSplit2.seconds100 = timerTime.seconds100;
		printf("%d: %d: %d: %d\n",timerSplit2.hours, timerSplit2.minutes, timerSplit2.seconds, timerSplit2.seconds100);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void EXTI4_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line4) != RESET){
		timerTime.seconds100 = 0;
		timerTime.seconds = 0;
		timerTime.minutes = 0;
		timerTime.hours = 0;
		TIM_Cmd(TIM2,DISABLE);
		timerStat=0;
		time=0;
		printf("%d: %d: %d: %d\n",timerTime.hours, timerTime.minutes, timerTime.seconds, timerTime.seconds100);

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

int main(void)
{
	initJoystick();
	initLed();
	initInterrupt();
	initTimer();
	//lcd_init();
	//0b00111100010000100011110000000000;
	//0b11000011101111011100001111111111;
	//initTimer();
	uint8_t array=0;
	uart_init( 9600 ); // Initialize USB serial at 9600 baud

	while(1){
		//TIM2_IRQHandler();
		//EXTI4_IRQHandler();
		//EXTI1_IRQHandler();
		//EXTI1_IRQHandler();
		//EXTI9_5_IRQHandler();
		//TIM2_IRQHandler;

	}
}
