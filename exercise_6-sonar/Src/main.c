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


void initTrigger(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port A

	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins

	// Sets PA9 to output
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_8; // Set so the configuration is on pin 9
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen


}

void initInterrupt(void){ //init ext interrupt for echo pin on PB5
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource5); // sets port B pin 5 to the IRQ
	// define and set setting for EXTI
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStructure);
	// setup NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
}

void initTimer(void){
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
	TIM2->PSC=9;
	TIM2->ARR=63;
	TIM2->DIER |= 0x0001; // Enable timer 2 interrupts
	//TIM2->CNT=1000;
	//TIM2->CR1=0xB01;

	NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority interrupts
	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt
	TIM_Cmd(TIM2,DISABLE);
}

void TIM2_IRQHandler(void) {
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET){
	//Do whatever you want here, but make sure it doesn’t take too much time
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); // Clear interrupt bit
		time+=1;
	}

}

void showTime(void){
	printf("%d: %d: %d: %d\n",timerTime.hours, timerTime.minutes, timerTime.seconds, timerTime.seconds100);
}

void EXTI9_5_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line5) != RESET){
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 1){
			time=0;
			TIM_Cmd(TIM2,ENABLE);
		}
		else{
			TIM_Cmd(TIM2,DISABLE);
		}
		EXTI_ClearITPendingBit(EXTI_Line5);
	}

}

int main(void)
{
  uart_init( 9600 ); // Initialize USB serial at 9600 baud
  initInterrupt();
  initTimer();
  initTrigger();
  //GPIO_ReadInputDataBit ( GPIOB, GPIO_Pin_0);
  GPIO_WriteBit(GPIOB , GPIO_Pin_8, SET);
  //for(int i=0; i < 10000; i++);

  //initTrigger();

  //GPIO_WriteBit(GPIOB , GPIO_Pin_8, RESET);
  //for(int i=0; i < 10000; i++);
  //GPIO_WriteBit(GPIOB , GPIO_Pin_8, SET);
  void delay(int ms){
  	int clock_cycles = 1.2 * ms * 12.8/3;

    for(int i = 0; i < clock_cycles; i++){
    }
  }

	while(1){
		delay(5000000);
		printf("Distance: %d cm\n",time*10/58);
		GPIO_WriteBit(GPIOB , GPIO_Pin_8, SET);
		delay(20);
		GPIO_WriteBit(GPIOB , GPIO_Pin_8, RESET);
	}
}
