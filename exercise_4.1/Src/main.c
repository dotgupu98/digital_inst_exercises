
#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" 		// Input/output library for this course
#include <string.h>

/*************************
    L3G4200D Registers
*************************/
/*#define WHO_AM_I 0x0F
#define CTRL_REG1 0x10
#define CTRL_REG2 0x11
#define CTRL_REG3 0x12
#define CTRL_REG4 0x13
#define CTRL_REG5 0x14
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38*/

#define ACCEL_RANGE 2
#define GYRO_RANGE 245
#define MAGNETOMETER_RANGE 4

#define WHO_AM_I 0x0F
#define CTRL_REG1_G 0x10
#define CTRL_REG2_G 0x11
#define CTRL_REG3_G 0x12
#define CTRL_REG4_G 0x1E
#define CTRL_REG5_XL 0x1F
#define CTRL_REG6_XL 0x20
#define CTRL_REG7_XL 0x21
#define CTRL_REG8_G  0x22

#define OUT_X_L_G 0x18
#define OUT_X_H_G 0x19
#define OUT_Y_L_G 0x1A
#define OUT_Y_H_G 0x1B
#define OUT_Z_L_G 0x1C
#define OUT_Z_H_G 0x1D

#define OUT_TEMP_L 0x15
#define OUT_TEMP_H 0x16

#define OUT_X_L_XL 0x28
#define OUT_X_H_XL 0x29
#define OUT_Y_L_XL 0x2A
#define OUT_Y_H_XL 0x2B
#define OUT_Z_L_XL 0x2C
#define OUT_Z_H_XL 0x2D

#define CTRL_REG1_M 0x20
#define CTRL_REG2_M 0x21
#define CTRL_REG3_M 0x22
#define CTRL_REG4_M 0x23
#define CTRL_REG5_M 0x24

#define OUT_X_L_M 0x28
#define OUT_X_H_M 0x29
#define OUT_Y_L_M 0x2A
#define OUT_Y_H_M 0x2B
#define OUT_Z_L_M 0x2C
#define OUT_Z_H_M 0x2D

int int2pin = 6;
int int1pin = 7;
int chipSelect = 10;

struct sensor_reading {
  float x;
  float y;
  float z;
};

struct sensor_reading values, gyro_values, acc_values, mag_values;

void init_spi() {
    // Enable Clocks
    RCC->AHBENR  |= 0x00020000 | 0x00040000;    // Enable Clock for GPIO Banks A and B
    RCC->APB1ENR |= 0x00004000;                 // Enable Clock for SPI2

    // Connect pins to SPI2
    GPIOB->AFR[13 >> 0x03] &= ~(0x0000000F << ((13 & 0x00000007) * 4)); // Clear alternate function for PB13
    GPIOB->AFR[13 >> 0x03] |=  (0x00000005 << ((13 & 0x00000007) * 4)); // Set alternate 5 function for PB13 - SCLK
    GPIOB->AFR[15 >> 0x03] &= ~(0x0000000F << ((15 & 0x00000007) * 4)); // Clear alternate function for PB15
    GPIOB->AFR[15 >> 0x03] |=  (0x00000005 << ((15 & 0x00000007) * 4)); // Set alternate 5 function for PB15 - MOSI
    GPIOB->AFR[14 >> 0x03] &= ~(0x0000000F << ((14 & 0x00000007) * 4)); // Clear alternate function for PB14
        GPIOB->AFR[14 >> 0x03] |=  (0x00000005 << ((14 & 0x00000007) * 4)); // Set alternate 5 function for PB14 - MISO

    // Configure pins PB13 and PB15 for 10 MHz alternate function
    GPIOB->OSPEEDR &= ~(0x00000003 << (13 * 2) | 0x00000003 << (15 * 2));    // Clear speed register
    GPIOB->OSPEEDR |=  (0x00000001 << (13 * 2) | 0x00000001 << (15 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOB->OTYPER  &= ~(0x0001     << (13)     | 0x0001     << (15));        // Clear output type register
    GPIOB->OTYPER  |=  (0x0000     << (13)     | 0x0000     << (15));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOB->MODER   &= ~(0x00000003 << (13 * 2) | 0x00000003 << (15 * 2));    // Clear mode register
    GPIOB->MODER   |=  (0x00000002 << (13 * 2) | 0x00000002 << (15 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR   &= ~(0x00000003 << (13 * 2) | 0x00000003 << (15 * 2));    // Clear push/pull register
    GPIOB->PUPDR   |=  (0x00000000 << (13 * 2) | 0x00000000 << (15 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    // Configure pins PB13 and PB15 for 10 MHz alternate function
    GPIOB->OSPEEDR &= ~(0x00000003 << (14 * 2) );    // Clear speed register
    GPIOB->OSPEEDR |=  (0x00000001 << (14 * 2) );    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOB->OTYPER  &= ~(0x0001     << (14)     );        // Clear output type register
    GPIOB->OTYPER  |=  (0x0000     << (14)     );        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOB->MODER   &= ~(0x00000003 << (14 * 2) );    // Clear mode register
    GPIOB->MODER   |=  (0x00000002 << (14 * 2) );    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR   &= ~(0x00000003 << (14 * 2) );    // Clear push/pull register
    GPIOB->PUPDR   |=  (0x00000000 << (14 * 2) );    // Set push/pull register (0x00 - No pull, 0x01

    // Initialize REEST, nCS, and A0
    // Configure pins PB6 and PB14 for 10 MHz output
  	// Configure PB14 --> CS
    GPIOB->OSPEEDR &= ~(0x00000003 << (6 * 2) );    // Clear speed register
    GPIOB->OSPEEDR |=  (0x00000001 << (6 * 2) );    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOB->OTYPER  &= ~(0x0001     << (6)     );        // Clear output type register
    GPIOB->OTYPER  |=  (0x0000     << (6)     );        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOB->MODER   &= ~(0x00000003 << (6 * 2));    // Clear mode register
    GPIOB->MODER   |=  (0x00000001 << (6 * 2) );    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR   &= ~(0x00000003 << (6 * 2) );    // Clear push/pull register
    GPIOB->PUPDR   |=  (0x00000000 << (6 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    // Initialize REEST, nCS, and A0
    // Configure pins PB6 and PB14 for 10 MHz output
  	// Configure PA1 --> CS
    GPIOA->OSPEEDR &= ~(0x00000003 << (1 * 2) );    // Clear speed register
    GPIOA->OSPEEDR |=  (0x00000001 << (1 * 2) );    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOA->OTYPER  &= ~(0x0001     << (1)     );    // Clear output type register
    GPIOA->OTYPER  |=  (0x0000     << (1)     );    // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOA->MODER   &= ~(0x00000003 << (1 * 2) );    // Clear mode register
    GPIOA->MODER   |=  (0x00000001 << (1 * 2) );    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOA->PUPDR   &= ~(0x00000003 << (1 * 2) );    // Clear push/pull register
    GPIOA->PUPDR   |=  (0x00000000 << (1 * 2) );    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    // Configure pin PA8 for 10 MHz output
    GPIOA->OSPEEDR &= ~0x00000003 << (8 * 2);    // Clear speed register
    GPIOA->OSPEEDR |=  0x00000001 << (8 * 2);    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOA->OTYPER  &= ~0x0001     << (8);        // Clear output type register
    GPIOA->OTYPER  |=  0x0000     << (8);        // Set output type register (0x00 - Push pull, 0x01 - Open drain)


    GPIOA->MODER   &= ~0x00000003 << (8 * 2);    // Clear mode register
    GPIOA->MODER   |=  0x00000001 << (8 * 2);    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)

    GPIOA->MODER   &= ~(0x00000003 << (2 * 2) | 0x00000003 << (3 * 2));    // This is needed for UART to work. It makes no sense.
    GPIOA->MODER   |=  (0x00000002 << (2 * 2) | 0x00000002 << (3 * 2));

    GPIOA->PUPDR   &= ~0x00000003 << (8 * 2);    // Clear push/pull register
    GPIOA->PUPDR   |=  0x00000000 << (8 * 2);    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    GPIOB->ODR |=  (0x0001 << 6); // CS = 1
    GPIOA->ODR |=  (0x0001 << 1); // CS = 1

    // Configure SPI2
    SPI2->CR1 &= 0x3040; // Clear CR1 Register
    SPI2->CR1 |= 0x0000; // Configure direction (0x0000 - 2 Lines Full Duplex, 0x0400 - 2 Lines RX Only, 0x8000 - 1 Line RX, 0xC000 - 1 Line TX)
    SPI2->CR1 |= 0x0104; // Configure mode (0x0000 - Slave, 0x0104 - Master)
    SPI2->CR1 |= 0x0002; // Configure clock polarity (0x0000 - Low, 0x0002 - High)
    SPI2->CR1 |= 0x0001; // Configure clock phase (0x0000 - 1 Edge, 0x0001 - 2 Edge)
    SPI2->CR1 |= 0x0200; // Configure chip select (0x0000 - Hardware based, 0x0200 - Software based)
    SPI2->CR1 |= 0x0008; // Set Baud Rate Prescaler (0x0000 - 2, 0x0008 - 4, 0x0018 - 8, 0x0020 - 16, 0x0028 - 32, 0x0028 - 64, 0x0030 - 128, 0x0038 - 128)
    SPI2->CR1 |= 0x0000; // Set Bit Order (0x0000 - MSB First, 0x0080 - LSB First)
    SPI2->CR2 &= ~0x0F00; // Clear CR2 Register
    SPI2->CR2 |= 0x0700; // Set Number of Bits (0x0300 - 4, 0x0400 - 5, 0x0500 - 6, ...);
    SPI2->I2SCFGR &= ~0x0800; // Disable I2S
    SPI2->CRCPR = 7; // Set CRC polynomial order
    SPI2->CR2 &= ~0x1000;
    SPI2->CR2 |= 0x1000; // Configure RXFIFO return at (0x0000 - Half-full (16 bits), 0x1000 - Quarter-full (8 bits))
    SPI2->CR1 |= 0x0040; // Enable SPI2
}

uint16_t transmit_byte(uint8_t data) {//CS is PB6
    //GPIOB->ODR &= ~(0x0001 << 6); // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) != SET) { }
    SPI_SendData8(SPI2, data);
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) != SET) { }
    //return SPI_ReceiveData8(SPI1);
    //GPIOB->ODR |=  (0x0001 << 6); // CS = 1 - End Transmission
  	uint16_t out;
    while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
  	out = SPI_ReceiveData8(SPI2);
  	SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
    return out;
}

uint16_t readRegister(uint8_t address, uint8_t chip)
{
  //GPIOA->ODR &= ~(0x0001 << 1); // CS = 0 - Start Transmission
  //GPIO_WriteBit(GPIOA , GPIO_Pin_1, RESET);
  uint16_t toRead;
  address |= 0x80;  // This tells the L3G4200D we're reading;
  //GPIO_WriteBit(GPIOA , GPIO_Pin_1, RESET);
  if(chip == 0){
    GPIO_WriteBit(GPIOA , GPIO_Pin_1, RESET);
	}
  else GPIO_WriteBit(GPIOB , GPIO_Pin_6, RESET);
  transmit_byte(address);
  //GPIOB->ODR |=  (0x0001 << 6); // CS = 1 - End Transmission
  toRead = transmit_byte(0x00);
  //GPIOA->ODR |=  (0x0001 << 1); // CS = 1 - End Transmission
  //GPIO_WriteBit(GPIOA , GPIO_Pin_1, SET);
  if(chip == 0){
    GPIO_WriteBit(GPIOA , GPIO_Pin_1, SET);
	}
  else GPIO_WriteBit(GPIOB , GPIO_Pin_6, SET);
  //printf("%d\n",toRead);
  return toRead;
}

//writeRegister for acc
void writeRegister(uint8_t address, uint8_t data, uint8_t chip)
{
  if(chip == 0){
    GPIO_WriteBit(GPIOA , GPIO_Pin_1, RESET);
	}
  else GPIO_WriteBit(GPIOB , GPIO_Pin_6, RESET);
  //GPIOA->ODR &= ~(0x0001 << 1); // CS = 0 - Start Transmission
  //GPIO_WriteBit(GPIOA , GPIO_Pin_1, RESET);
  address &= 0x7F;  // This to tell the L3G4200D we're writing
  transmit_byte(address);
  transmit_byte(data);
  //GPIOA->ODR |=  (0x0001 << 1); // CS = 1 - End Transmission
  if(chip ==0){
    GPIO_WriteBit(GPIOA , GPIO_Pin_1, SET);
	}
  else GPIO_WriteBit(GPIOB , GPIO_Pin_6, SET);
  //GPIO_WriteBit(GPIOA , GPIO_Pin_1, SET);
}

uint8_t setup_lsm9ds1(){
  //hand shake gyro
  writeRegister(CTRL_REG8_G, 0x05, 0); //reset gyro
  if (readRegister(WHO_AM_I, 0) != 0x68) { //second argument specifies gyro
    return -1;
  }

  //setup gyro
  writeRegister(CTRL_REG1_G, 0x78, 0); // 119 Hz, 2000 dps, 16 Hz BW
  writeRegister(CTRL_REG6_XL, 0x70, 0); // 119 Hz, 4G

  //hand shake magnetometer
  writeRegister(CTRL_REG2_M, 0x0C, 1); //reset magnetometer
  if (readRegister(WHO_AM_I, 1) != 0x3D) { //second argument specifies magnetometer
     return -1;
  }

  //setup magnetometer
  writeRegister(CTRL_REG1_M, 0xb4, 1); // Temperature compensation enable, medium performance, 20 Hz
  writeRegister(CTRL_REG2_M, 0b00000000, 1); // 4 Gauss
  writeRegister(CTRL_REG3_M, 0x00, 1); // Continuous conversion mode
}

struct sensor_reading getGyroValues()
{
//  uint16_t x = (readRegister(0x29,0)&0xFF)<<8;
//  x |= (readRegister(0x28,0)&0xFF);
//
//  uint16_t y = (readRegister(0x2B,0)&0xFF)<<8;
//  y |= (readRegister(0x2A,0)&0xFF);
//
//  uint16_t z = (readRegister(0x2D,0)&0xFF)<<8;
//  z |= (readRegister(0x2C,0)&0xFF);
//
//  values.x = x;
//  values.y = x;
//  values.z = z;
//
	uint8_t x_low, x_high;
	uint8_t y_low, y_high;
	uint8_t z_low, z_high;

	x_low   = readRegister(0x18,0);
	x_high  = readRegister(0x19,0);
	y_low   = readRegister(0x1A,0);
	y_high  = readRegister(0x1B,0);
	z_low   = readRegister(0x1C,0);
	z_high  = readRegister(0x1D,0);

	int16_t x = (signed)((x_high<<8)|x_low);
	int16_t y = (signed)((y_high<<8)|y_low);
	int16_t z = (signed)((z_high<<8)|z_low);

	values.x = ((float)x*2000/32768);
	values.y = ((float)y*2000/32768);
	values.z = ((float)z*2000/32768);

	return values;

  return values;
}

struct sensor_reading getAccValues()
{
//	uint16_t x = (readRegister(0x19,0)&0xFF)<<8;
//	x |= (readRegister(0x18,0)&0xFF);
//
//	uint16_t y = (readRegister(0x1B,0)&0xFF)<<8;
//	y |= (readRegister(0x1A,0)&0xFF);
//
//	uint16_t z = (readRegister(0x1D,0)&0xFF)<<8;
//	z |= (readRegister(0x1C,0)&0xFF);
//
//	values.x = x;
//	values.y = x;
//	values.z = z;
	uint8_t x_low, x_high;
	uint8_t y_low, y_high;
	uint8_t z_low, z_high;

	x_low   = readRegister(0x28,0);
	x_high  = readRegister(0x29,0);
	y_low   = readRegister(0x2A,0);
	y_high  = readRegister(0x2B,0);
	z_low   = readRegister(0x2C,0);
	z_high  = readRegister(0x2D,0);

	int16_t x = (signed)((x_high<<8)|x_low);
	int16_t y = (signed)((y_high<<8)|y_low);
	int16_t z = (signed)((z_high<<8)|z_low);

	values.x = ((float)x*4/32768);
	values.y = ((float)y*4/32768);
	values.z = ((float)z*4/32768);

	return values;
}

struct sensor_reading getMagValues()
{
//	uint16_t x = (readRegister(0x29,1)&0xFF)<<8;
//	x |= (readRegister(0x28,1)&0xFF);
//
//	uint16_t y = (readRegister(0x2B,1)&0xFF)<<8;
//	y |= (readRegister(0x2A,1)&0xFF);
//
//	uint16_t z = (readRegister(0x2D,1)&0xFF)<<8;
//	z |= (readRegister(0x2C,1)&0xFF);
//
//	int16_t

	uint8_t x_low, x_high;
	uint8_t y_low, y_high;
	uint8_t z_low, z_high;

	x_low   = readRegister(0x28,1);
	x_high  = readRegister(0x29,1);
	y_low   = readRegister(0x2A,1);
	y_high  = readRegister(0x2B,1);
	z_low   = readRegister(0x2C,1);
	z_high  = readRegister(0x2D,1);

	int16_t x = (signed)((x_high<<8)|x_low);
	int16_t y = (signed)((y_high<<8)|y_low);
	int16_t z = (signed)((z_high<<8)|z_low);

	values.x = ((float)x*4/32768);
	values.y = ((float)y*4/32768);
	values.z = ((float)z*4/32768);

	return values;
}

float getTempValue(){
  uint8_t t_low, t_high;

  t_low   = readRegister(OUT_TEMP_L, 0);
	t_high  = readRegister(OUT_TEMP_H, 0);

  int16_t temp_binary = (signed)((t_high<<8)|t_low);
  float temp = ((float)temp_binary/16+27.5);
  temp = temp;
  return temp;
}


void init_cs(){
	//setup CS for Gyro PA1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE); // Enable clock for GPIO Port A
	GPIO_InitTypeDef GPIO_InitStruct_PA1; // Define typedef struct for setting pins

	GPIO_StructInit(&GPIO_InitStruct_PA1); // Initialize GPIO struct
	GPIO_InitStruct_PA1.GPIO_Mode = GPIO_Mode_OUT; // Set as input
	GPIO_InitStruct_PA1.GPIO_PuPd = GPIO_PuPd_NOPULL; // Set as pull down
	GPIO_InitStruct_PA1.GPIO_Pin = GPIO_Pin_1; // Set so the configuration is on pin 4
	GPIO_Init(GPIOA, &GPIO_InitStruct_PA1); // Setup of GPIO with the settings chosen

	//setup CS for Magnetometer PB6
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); // Enable clock for GPIO Port A
	GPIO_InitTypeDef GPIO_InitStruct_PB6; // Define typedef struct for setting pins

	GPIO_StructInit(&GPIO_InitStruct_PB6); // Initialize GPIO struct
	GPIO_InitStruct_PB6.GPIO_Mode = GPIO_Mode_OUT; // Set as input
	GPIO_InitStruct_PB6.GPIO_PuPd = GPIO_PuPd_NOPULL; // Set as pull down
	GPIO_InitStruct_PB6.GPIO_Pin = GPIO_Pin_6; // Set so the configuration is on pin 4
	GPIO_Init(GPIOB, &GPIO_InitStruct_PB6); // Setup of GPIO with the settings chosen
}

#define UART3_BUFFER_LENGTH 256
volatile uint8_t UART3_BUFFER[UART3_BUFFER_LENGTH] = {0};
volatile uint8_t UART3_END_IDX = 0;
volatile uint8_t UART3_START_IDX = 0;
volatile uint8_t UART3_COUNT = 0;

uint8_t uart3_get_char(){
    uint8_t val = 0;
    if (UART3_COUNT > 0) {
        val = UART3_BUFFER[UART3_START_IDX++];
        UART3_COUNT--;
    }
    return val;
}

void uart3_put_char(uint8_t c) {
    USART_SendData(USART3, (uint8_t)c);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)  == RESET){}
}

int _write3_r(struct _reent *r, int file, char *ptr, int len) {
    int n;

    for (n = 0; n < len; n++) {
        if (ptr[n] == '\n') {
            uart3_put_char('\r');
        }
        uart3_put_char(ptr[n] & (uint16_t)0x01FF);
    }

    return len;
}

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        UART3_BUFFER[UART3_END_IDX++] = (uint8_t)(USART3->RDR & 0xFF);
        if (UART3_COUNT == UART3_BUFFER_LENGTH-1){
            UART3_START_IDX++;
        } else {
            UART3_COUNT++;
        }
    }
}

void uart3_clear(){
    UART3_START_IDX = 0;
    UART3_END_IDX = 0;
    UART3_COUNT = 0;
}

uint8_t uart3_get_count(){
    return UART3_COUNT;
}

void uart3_init(uint32_t baud) {
    setbuf(stdout, NULL); // Set stdout to disable line buffering
    setbuf(stdin,  NULL); // Set stdin  to disable line buffering

    // Enable Clocks
    RCC->AHBENR  |= RCC_AHBPeriph_GPIOB;    // Enable Clock for GPIO Bank A
    RCC->APB1ENR |= RCC_APB1Periph_USART3;  // Enable Clock for USART2

    // Connect pins to USART2
    GPIOB->AFR[8 >> 0x03] &= ~(0x0000000F << ((8 & 0x00000007) * 4)); // Clear alternate function for PA2
    GPIOB->AFR[8 >> 0x03] |=  (0x00000007 << ((8 & 0x00000007) * 4)); // Set alternate 7 function for PA2
    GPIOB->AFR[9 >> 0x03] &= ~(0x0000000F << ((9 & 0x00000007) * 4)); // Clear alternate function for PA3
    GPIOB->AFR[9 >> 0x03] |=  (0x00000007 << ((9 & 0x00000007) * 4)); // Set alternate 7 function for PA3

    // Configure pins PA2 and PA3 for 10 MHz alternate function
    GPIOB->OSPEEDR &= ~(0x00000003 << (8 * 2) | 0x00000003 << (9 * 2));    // Clear speed register
    GPIOB->OSPEEDR |=  (0x00000001 << (8 * 2) | 0x00000001 << (9 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOB->OTYPER  &= ~(0x0001     << (8)     | 0x0001     << (9));        // Clear output type register
    GPIOB->OTYPER  |=  (0x0000     << (8)     | 0x0000     << (9));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOB->MODER   &= ~(0x00000003 << (8 * 2) | 0x00000003 << (9 * 2));    // Clear mode register
    GPIOB->MODER   |=  (0x00000002 << (8 * 2) | 0x00000002 << (9 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR   &= ~(0x00000003 << (8 * 2) | 0x00000003 << (9 * 2));    // Clear push/pull register
    GPIOB->PUPDR   |=  (0x00000001 << (8 * 2) | 0x00000001 << (9 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    //Configure USART2
    USART3->CR1 &= ~0x00000001; // Disable USART2
    USART3->CR2 &= ~0x00003000; // Clear CR2 Configuration
    USART3->CR2 |=  0x00000000; // Set 1 stop bits
    USART3->CR1 &= ~(0x00001000 | 0x00000400 | 0x00000200 | 0x00000008 | 0x00000004); // Clear CR1 Configuration
    USART3->CR1 |=  0x00000000; // Set word length to 8 bits
    USART3->CR1 |=  0x00000000; // Set parity bits to none
    USART3->CR1 |=  0x00000004 | 0x00000008; // Set mode to RX and TX
    USART3->CR3 &= ~(0x00000100 | 0x00000200); // Clear CR3 Configuration
    USART3->CR3 |=  0x00000000; // Set hardware flow control to none

    uint32_t divider = 0, apbclock = 0, tmpreg = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus); // Get USART2 Clock frequency
    apbclock = RCC_ClocksStatus.USART3CLK_Frequency;

    if ((USART3->CR1 & 0x00008000) != 0) {
      // (divider * 10) computing in case Oversampling mode is 8 Samples
      divider = (2 * apbclock) / baud;
      tmpreg  = (2 * apbclock) % baud;
    } else {
      // (divider * 10) computing in case Oversampling mode is 16 Samples
      divider = apbclock / baud;
      tmpreg  = apbclock % baud;
    }

    if (tmpreg >=  baud / 2) {
        divider++;
    }

    if ((USART3->CR1 & 0x00008000) != 0) {
        // get the LSB of divider and shift it to the right by 1 bit
        tmpreg = (divider & (uint16_t)0x000F) >> 1;
        // update the divider value
        divider = (divider & (uint16_t)0xFFF0) | tmpreg;
    }

    USART3->BRR = (uint16_t)divider; // Configure baud rate
    USART3->CR1 |= 0x00000001; // Enable USART2

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART3_IRQn);
}


//int main(void)
//{
//  uart_init(9600);
//  init_spi();
//  init_cs();
//
//  while(setup_lsm9ds1() == -1){
//	  printf("Waiting for connection to LSM9DS1");
//  }
//
//  while(1){
//	for (int i=0; i<1000000;i++);
//
//	gyro_values = getGyroValues();
//	acc_values = getAccValues();
//	mag_values = getMagValues();
//  float temp = getTempValue();
//
//	printf("GYRO: x: %f, y: %f, z: %f\n", gyro_values.x, gyro_values.y, gyro_values.z);
//	printf("ACCEL: x: %f g, y: %f g, z: %f g\n", acc_values.x, acc_values.y, acc_values.z);
//	printf("MAG: x: %f Gs, y: %f Gs, z: %f Gs\n", mag_values.x, mag_values.y, mag_values.z);
//  printf("Temp: %f °C\n", temp);
//
//
//	printf("\n");
//  }
//}
void printf_usart3(char source[]){
	for (int i = 0; i < strlen(source); i++) {
    uart3_put_char(source[i]);
  }
}

int main(void)
{
  uart3_init(9600);

  printf_usart3("abcds");
  printf_usart3("abcds");
  printf_usart3("abcds\n");

  while(1){


	  for (int i=0; i<1000000;i++);
	  //uart3_put_char(0x0F);
  }
}

