#ifndef     PLATFORM_FIRE_BUTTON
#define     PLATFORM_FIRE_BUTTON

#define     IPORT   1
#define     IPORT_IRQ   IRQ_PORT1
#define     IPINS   0x20

#define     OPORT   5
#define     OPINS   0xFF

#define     CONTROL_PORT    2
#define     PINT1           0x02
#define     PINT2           0x01
#define     CONTROL_PINS    (PINT1|PINT2)

#endif

