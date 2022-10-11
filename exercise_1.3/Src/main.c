#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course

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

uint8_t readJoystick(void){
	uint8_t up=GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_4);
	uint8_t down=GPIO_ReadInputDataBit ( GPIOB, GPIO_Pin_0);
	uint8_t left=GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_1);
	uint8_t right=GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_0);
	uint8_t center=GPIO_ReadInputDataBit ( GPIOB, GPIO_Pin_5);

	uint8_t array = (up << 7) | (down << 6) | (left << 5) | (right << 4) | (center << 3) | (0 << 2) | (0 << 1) | (0 << 0);

	return(array);
}

void setLed(uint8_t r, uint8_t g, uint8_t b){
	GPIO_WriteBit(GPIOB , GPIO_Pin_4, r); //set red led to enabled or disabled
	GPIO_WriteBit(GPIOC , GPIO_Pin_7, g); //set green led to enabled or disabled
	GPIO_WriteBit(GPIOA , GPIO_Pin_9, b); //set blue led to enabled or disabled
}

void EXTI9_5_IRQHandler(void){ //interrupt handler for joystick center input
	if(EXTI_GetITStatus(EXTI_Line5) != RESET){
		printf("Right : %d | Up : %d | Center : %d | Left : %d | Down : %d\n",\
			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0),\
			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4),\
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5),\
			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1),\
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)); //prints the direction data
		setLed(1,1,1); //set led to off
		EXTI_ClearITPendingBit(EXTI_Line5); //clear pending bit
	}
}

void EXTI0_IRQHandler(void){ //interrupt handler for joystick right input
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){
		printf("Right : %d | Up : %d | Center : %d | Left : %d | Down : %d\n",\
			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0),\
			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4),\
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5),\
			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1),\
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)); //prints the direction data
		setLed(1,0,1); //set led to green color
		EXTI_ClearITPendingBit(EXTI_Line0); //clear pending bit
	}
}

void EXTI4_IRQHandler(void){ //interrupt handler for joystick up input
	if(EXTI_GetITStatus(EXTI_Line4) != RESET){
		printf("Right : %d | Up : %d | Center : %d | Left : %d | Down : %d\n",\
			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0),\
			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4),\
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5),\
			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1),\
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)); //prints the direction data
		setLed(1,1,0); //set led to blue color
		EXTI_ClearITPendingBit(EXTI_Line4); //clear pending bit
	}
}

void EXTI1_IRQHandler(void){ //interrupt handler for joystick left input
	if(EXTI_GetITStatus(EXTI_Line1) != RESET){
		printf("Right : %d | Up : %d | Center : %d | Left : %d | Down : %d\n",\
			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0),\
			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4),\
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5),\
			GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1),\
			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)); //prints the direction data
		setLed(0,1,1); //set led to red color
		EXTI_ClearITPendingBit(EXTI_Line1); //clear pending bit
	}
}

int main(void)
{
	initJoystick(); //initialize the input pins
	initLed(); //initialize the output pins
	setLed(1,1,1); //turn off all leds
	initInterrupt(); //initialize the interrupts
	uart_init( 9600 ); // Initialize USB serial at 9600 baud

	while(1){

	}
}
