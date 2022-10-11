#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course

int ICValue1 = 0;
int ICValue2 = 0;
int ICValid = 0;

void initCounter(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); //Enable clock for timer
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port B

	GPIO_InitTypeDef GPIO_InitStructAll; // Define typedef struct for setting pins
	GPIO_StructInit(&GPIO_InitStructAll);
	// Then set things that are not default.
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructAll.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	GPIOA->AFR[0] |= GPIO_AF_1; //Sets pin y at port x to alternative function z

	//ICInitStruct
	TIM_ICInitTypeDef ICStruct;
	ICStruct.TIM_ICFilter = 0x0;
	ICStruct.TIM_ICPrescaler = 0x0;
	TIM_ICInit(TIM2, &ICStruct);

	// Timer
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_TimeBaseStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_ClockDivision = 0;
	TIM_InitStructure.TIM_Period = 0xFFFF; //set the maximum period
	TIM_InitStructure.TIM_Prescaler = 63; //for 1MHz counting frequency
	TIM_TimeBaseInit(TIM2,&TIM_InitStructure);

	// Set Input Capture in TIM2 to PWM mode
	TIM2->CCMR1 = TIM_CCMR1_CC2S_1 | TIM_CCMR1_CC1S_0; //CC1 channel as input, IC1 is mapped on TI1
	TIM2->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_TS_0;        //set trigger to TI1FP1
	TIM2->SMCR |= TIM_SMCR_SMS_2;                      //slave to reset mode
	TIM2->CCER = TIM_CCER_CC2P | TIM_CCER_CC1E | TIM_CCER_CC2E; //enable capture and compare modules

	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); //set priority
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); // NVIC for timer
	TIM_Cmd(TIM2,ENABLE); //enable the timer
}

void TIM2_IRQHandler(void){
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2); //clear pending bit
	ICValue1 = TIM_GetCapture1(TIM2); // save period
	ICValue2 = TIM_GetCapture2(TIM2); // save duty cycle
	ICValid = 1; // save that the measurement happened
	TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE); // disable
}

/*int main(void)
{
	SystemInit(); //reset the RCC clock configuration to the default
	initCounter(); //initialize the timer
	uart_init( 9600 ); // Initialize USB serial at 9600 baud

    while(1)
    {
        if (1) // printf if the measurement is valid
        {
            printf("F: %fkHz T: %fms D: %f\n",
            		1000/(double)ICValue1,  //calculate the frequency
					//cast type to double for division
					ICValue1*1e-3,  //calculate the period
					(double)((double)ICValue2/(double)ICValue1)); //calculate the duty cycle
            ICValid = 0; //set to 0 for new measurement
            TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE); // enable the timer
        }
        for (int i = 0; i < 10000000; i++)
            ;
    	} //delay for slower printing
}*/

int main(void)
{
	SystemInit(); //reset the RCC clock configuration to the default
	initCounter(); //initialize the timer
	uart_init( 9600 ); // Initialize USB serial at 9600 baud

    while(1)
    {
        if (ICValid) // printf if the measurement is valid
        {
            printf("Freq.: %f kHz, Period: %f ms, Dutycycle: %f \%\n",
            		1000/(double)ICValue1,  //calculate the frequency
					//cast type to double for division
					ICValue1*1e-3,  //calculate the period
					(double)((double)ICValue2/(double)ICValue1)); //calculate the duty cycle
            ICValid = 0; //set to 0 for new measurement
            TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE); // enable the timer
        }
        for (int i = 0; i < 10000000; i++)
            ;
    	} //delay for slower printing
}
