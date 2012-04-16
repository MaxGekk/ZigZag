#ifndef     PLATFORM_UBPD
#define     PLATFORM_UBPD

#define     IPORT   2       /* Номер порта цифровых входов */
#define     IPORT_IRQ   IRQ_PORT2
#define     IPINS   0x0F    /* XXX Маска цифровых входов */

#define     OPORT   5       /* Номер порта цифровых выходов */
#define     OPINS   0x0F    /* Маска цифровых выходов */

#define     LED_PORT    6       /* номер порта для светодиодов */
#define     LED1        1
#define     LED2        2
#define     LED3        4

#endif

