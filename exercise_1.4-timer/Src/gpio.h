#ifndef GPIO_H_
#define GPIO_H_

void initJoystick(void);
void initLed(void);
uint8_t readJoystick(void);
void setLed(uint8_t red, uint8_t green, uint8_t blue);

#endif
