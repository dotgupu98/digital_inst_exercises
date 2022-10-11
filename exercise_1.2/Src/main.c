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
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_4; // Set so the configuration is on pin 4
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOB, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen

	// Sets PA9 to output
	GPIO_StructInit(&GPIO_InitStructAll); // Initialize GPIO struct
	GPIO_InitStructAll.GPIO_Mode = GPIO_Mode_OUT; // Set as output
	GPIO_InitStructAll.GPIO_OType = GPIO_OType_PP; // Set as Push-Pull
	GPIO_InitStructAll.GPIO_Pin = GPIO_Pin_7; // Set so the configuration is on pin 7
	GPIO_InitStructAll.GPIO_Speed = GPIO_Speed_2MHz; // Set speed to 2 MHz
	// For all options see SPL/inc/stm32f30x_gpio.h
	GPIO_Init(GPIOC, &GPIO_InitStructAll); // Setup of GPIO with the settings chosen
}

uint8_t readJoystick(void){ //function to load input data to the array
	uint8_t array = (GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_4) << 7) |\
			(GPIO_ReadInputDataBit ( GPIOB, GPIO_Pin_0) << 6) |\
			(GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_1) << 5) |\
			(GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_0) << 4) |\
			(GPIO_ReadInputDataBit ( GPIOB, GPIO_Pin_5) << 3) |\
			(0 << 2) | (0 << 1) | (0 << 0); //Create array for serial data

	switch (array){
			case 8: //array value=8 when center pressed
				setLed(0,1,1);
				break;

			case 16: //array value=16 when right pressed
				setLed(1,0,1);
				break;

			case 32: //array value=32 when left pressed
				setLed(0,0,1);
				break;

			case 64: //array value=64 when down pressed
				setLed(1,0,0);
				break;

			case 128: //array value=128 when up pressed
				setLed(0,1,0);
				break;

			default: //joystick not used or faulty (several directions at the same time)
				setLed(1,1,1);
			}

	return(array);
}

void setLed(uint8_t r, uint8_t g, uint8_t b){
	GPIO_WriteBit(GPIOB , GPIO_Pin_4, r); //set red led to enabled or disabled
	GPIO_WriteBit(GPIOC , GPIO_Pin_7, g); //set green led to enabled or disabled
	GPIO_WriteBit(GPIOA , GPIO_Pin_9, b); //set blue led to enabled or disabled
}

int main(void)
{
	initJoystick();
	initLed();
	uint8_t array=0;
	uart_init( 9600 ); // Initialize USB serial at 9600 baud
	setLed(1,1,1); //set all leds to off state

	while(1){ // set led color depending on joystick position

		if(array != readJoystick()){ //display only when input change occurs
			array=readJoystick();
			for(int i=7; i>=0; i--) printf("%1d", (array & (1 << i)) >> i );
			printf("\n");
		}
	}
}
