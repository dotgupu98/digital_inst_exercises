#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" 		// Input/output library for this course
#include "flash.h"
#include "lcd.h"

int main(void)
{
	init_spi_lcd();
	uart_init(9600);

	uint8_t fbuffer[512];
	memset(fbuffer,0xAA,512); // Sets each element of the buffer to 0xAA
	lcd_push_buffer(fbuffer);

	lcd_write_string("1line", fbuffer, 0, 0);
	lcd_write_string("2line", fbuffer, 10, 1);
	lcd_write_string("3line", fbuffer, 20, 2);
	lcd_write_string("4line", fbuffer, 30, 3);
	lcd_push_buffer(fbuffer);

	uint32_t address = 0x0800F800;
	uint16_t tempVal;
	for ( int i = 0 ; i < 10 ; i++ ){
	tempVal = *(uint16_t *)(address + i * 2); // Read Command
	printf("%d \n", tempVal);
	}

	uint8_t a = 10;
	char str[7];
	//sprintf(str, “a = %2d”, a);

	while(1){
		for(int i=0;i<=100;i++){
			//lcd_write_string("                               ", fbuffer, 0, 0);
			lcd_write_string("x", fbuffer, i, 0);
			lcd_push_buffer(fbuffer);
			for(int j=0;j<200000;j++){;}
		}
	}
}
