#ifndef     PLATFORM_PULT_APP
#define     PLATFORM_PULT_APP

#define MODBUS_USART 0

#define MB_CONVERTER

// rs485 converter i/o port registers
#define MB_CTRL_PORT_SEL P1SEL
#define MB_CTRL_PORT_DIR P1DIR
#define MB_CTRL_PORT_OUT P1OUT

// rs485 converter control pins
#define MB_RE 1
#define MB_DE 2

#endif

