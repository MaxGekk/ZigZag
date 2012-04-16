#ifndef     PLATFORM_ANGARSK
#define     PLATFORM_ANGARSK

#define     IPORT   2       /* Номер порта цифровых входов */
#define     IPORT_IRQ   IRQ_PORT2
#define     IPINS   0x01    /* XXX Маска цифровых входов */

#define     OPORT   6       /* Номер порта цифровых выходов */
#define     OPINS   0x00    /* Маска цифровых выходов */

#define     LED_PORT    2       /* номер порта для светодиодов */
#define     LED1        (1<<4)
#define     LED2        (1<<6)
#define     LED3        (1<<7)

#endif

