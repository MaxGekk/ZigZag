/*
 * kbrd.h - inteface file to keyboard routines and interrupts
 */
#ifndef _KBRD_H_
#define _KBRD_H_

#define KBRD_SOUND_PORT              1
#define KBRD_PORT2                   2
#define KBRD_PORT6                   6

#define KBRD_ANTIRING_TIMER          0
#define KBRD_SOUND_TIMER             2

#define _KBRD_ANTIRING_DELAY         20 // ms
#define _KBRD_SOUND_DURATION         10 // ms
//#define _KBRD_ANTIRING_DELAY         1250 // symbols
#define _KBRD_SOUND_TIME             31
#define _KBRD_BUF_SIZE               16

#define KBRD_SOUND_PIN               0x80

//#define _KBRD_ENABLE_INT()           (P2IE  |=  0x0f)
//#define _KBRD_DISABLE_INT()          (P2IE  &= ~0x0f)
//#define _KBRD_IS_INT_ENABLED()       (P2IE & 0x0f ? 1 : 0)

//#define _KBRD_READ_COLUMNS()       (P2IN & 0x0f)
//#define _KBRD_SET_ROWS(row)          P6OUT  &= ~0x0f; P6OUT  |= row;
//#define _KBRD_CLR_INT_FLAGS()        (P2IFG &= ~0x0f)
//#define _KBRD_SETUP_EDGES(edges)     (P2IES = (P2IES & 0xf0) | edges)
//#define _KBRD_ARE_EDGES_SET()        (!((P2IES ^ P2IN) & 0x0f))
/*
#define _KBRD_SETUP_PORTS()        P6OUT |=  0x0f; \
                                   P6DIR |=  0x0f; \
                                   P6SEL &= ~0x0f; \
                                   P2DIR &= ~0x0f; \
                                   P2SEL &= ~0x0f; \
                                   P2IES &= ~0x0f; \
                                   P2IFG &= ~0x0f; 
*/

extern unsigned char kbrd_pressed;
extern port_attr_t port2_attr;

void init_kbrd(void);
void kbrd_clear_ring_buf(void);
unsigned char kbrd_getch(void);
int kbrd_service(void);
unsigned char kbrd_getSymbolCount(void);
unsigned char get_digit(unsigned char code);

#endif  // _KBRD_H_
