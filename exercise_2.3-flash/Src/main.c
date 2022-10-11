#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" 		// Input/output library for this course

#include "flash.h"

int main(void)
{
	uart_init(9600); // Initialize USB serial at 9600 baud
	printf("Flash Demo\n");

	//Create 4 Variables that are supposed to be written to the flash
	uint8_t uint8_flash = 0xAD;	//8-bit Integer
	uint16_t uint16_flash = 0xADAD; //16-bit Integer
	uint32_t uint32_flash = 0xADADADAD; //32-bit Integer
	float float_flash = 1.5; //float
	uint32_t uint32_flash2 = 0x00000000; //32-bit Integer



	FLASH_Unlock(); //unlock flash

	//ERASE FLASH
	printf("Write Float to Flash\n");
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

	//WRITE UINT8 TO FLASH
	FLASH_ErasePage(PG31_BASE); //erase flash
	write_word_flash(PG31_BASE, 0, (uint32_t)uint8_flash);
	printf("Write done.\n");

	//WRITE UINT16 TO FLASH
	printf("Write Float to Flash\n");
	write_word_flash(PG31_BASE, 1, (uint32_t)uint16_flash);
	printf("Write done.\n");

	//WRITE UINT32 TO FLASH
	printf("Write Float to Flash\n");
	write_word_flash(PG31_BASE, 2, uint32_flash);
	printf("Write done.\n");

	//WRITE FLOAT TO FLASH
	printf("Write Float to Flash\n");
	write_float_flash(PG31_BASE, 3, float_flash);
	printf("Write done.\n");

	FLASH_Lock();  //lock flash

	FLASH_Unlock(); //unlock flash

	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	FLASH_ErasePage(PG31_BASE); //erase flash

	for(uint32_t i=0;i<512;i++){
		//printf("Write Float to Flash\n");
		write_word_flash(PG31_BASE, i, (uint32_t)(uint32_flash2+i));
		//printf("Write done.\n");
	}

	FLASH_Lock();  //lock flash
	printf("Write done.\n");


	while(1){}
}
