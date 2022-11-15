
#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h"         // Input/output library for this course
#include <string.h>

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
  }
}


