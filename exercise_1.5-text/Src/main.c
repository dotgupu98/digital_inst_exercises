#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" 		// Input/output library for this course

int main(void)
{
	uart_init(9600); //init uart
	uint8_t a = 10;
	float f = 2.7645;
	char str[7];
	sprintf(str, "a = %2d, f = %0.2f", a, f); // format the string
	printf("%s\n",str); // print to uart console
	while(1){
	}
}
