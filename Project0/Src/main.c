#include "stm32f30x_conf.h" // STM32 config
#include "30010_io.h" // Input/output library for this course
int16_t power(int8_t a, int8_t exp) {
// calculates a^exp
int16_t i, r = a;
for (i = 1; i < exp; i++)
r *= a;
return(r);
}
int main(void)
{
int16_t a;
uart_init( 9600 ); // Initialize USB serial at 9600 baud
printf("\n\n x x^2 x^3 x^4\n");
for (a = 0; a < 10; a++)
printf("%8d%8d%8d%8d\n",a, power(a, 2), power(a, 3), power(a, 4));
while(1){}
}
