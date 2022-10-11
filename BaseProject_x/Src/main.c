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

uint8_t readJoystick(void){ //fuction to load input data to the array
	uint8_t array = (GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_4) << 7) |\
			(GPIO_ReadInputDataBit ( GPIOB, GPIO_Pin_0) << 6) |\
			(GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_1) << 5) |\
			(GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_0) << 4) |\
			(GPIO_ReadInputDataBit ( GPIOB, GPIO_Pin_5) << 3) |\
			(0 << 2) | (0 << 1) | (0 << 0); //Create array for serial data
	return(array);
}

int main(void)
{
	initJoystick(); //GPIO initialization
	uint8_t array=0; //variable to display
	uart_init( 9600 ); // Initialize USB serial at 9600 baud

	while(1){

		if(array==readJoystick()){ //display only when input change occurs
			array=readJoystick();
			for(int i=7; i>=0; i--) printf("%1d", (array & (1 << i)) >> i );
			printf("\n");
		}
	}
}
