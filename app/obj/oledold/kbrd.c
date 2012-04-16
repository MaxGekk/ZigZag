/*
 * kbrd.c - keyboard routines and interrupts
 */
//#include <msp430x16x.h>
#include <zzPort.h>
#include "kbrd.h"

port_attr_t port2_attr;
port_attr_t port1_attr;

extern port_attr_t port6_attr;

static unsigned int kbrd_sound;

unsigned char kbrd_buf[_KBRD_BUF_SIZE];
unsigned char kbrd_buf_pos;
unsigned char kbrd_buf_symbols;
unsigned int  kbrd_prev_state;
unsigned char kbrd_pressed;
unsigned char kbrd_ring_buf[16];

//extern void start_TB(unsigned int period);
extern result_t set_stimer(uint8_t tnum, uint16_t period);
//extern result_t set_atimer(uint8_t tnum, uint16_t period);
extern result_t set_atimer( const uint8_t tnum, const uint32_t tpoint);
extern uint32_t get_atimer_counter(void);

volatile uint8_t P2IN  asm("0x0028");
volatile uint8_t P2DIR asm("0x002a");
volatile uint8_t P2IFG asm("0x002B");
volatile uint8_t P2IES asm("0x002c");
volatile uint8_t P2IE  asm("0x002d");
volatile uint8_t P2SEL asm("0x002e");
volatile uint8_t P6OUT asm("0x0035");
volatile uint8_t P6DIR asm("0x0036");
volatile uint8_t P6SEL asm("0x0037");

static unsigned int kbrd_scan(void);


static inline _KBRD_CLR_INT_FLAGS(void)
{
   P2IFG &= ~0x0f;
}

static inline void _KBRD_SET_ROWS(unsigned char row)
{
   port_write(KBRD_PORT6, 0x0f, PIN_LO);
   port_write(KBRD_PORT6,  row, PIN_HI);
}

static inline port_t _KBRD_READ_COLUMNS(void)
{
   port_t port_val = 0;
//   port_read(KBRD_PORT2, 0x0f, &port_val);
   port_val = P2IN & 0x0f;
   return port_val;
}

static inline void _KBRD_SETUP_EDGES(unsigned char edges)
{
   port2_attr.ies = (port2_attr.ies & 0xf0) | edges;
   port_set_attr(KBRD_PORT2, 0x0f, &port2_attr);
}

static inline unsigned char _KBRD_ARE_EDGES_SET(void)
{
   port_t p2 = 0;
   //port_read(KBRD_PORT2, 0x0f, &p2);
   p2 = P2IN & 0x0f;
   return !((port2_attr.ies ^ p2) & 0x0f);
}

static inline void _KBRD_ENABLE_INT(void)
{
   PIN_SET(port2_attr.ie, 0x0f, PIN_HI);
   port_set_attr(KBRD_PORT2, 0x0f, &port2_attr);
}

static inline void _KBRD_DISABLE_INT(void)
{
   PIN_CLEAR(port2_attr.ie, 0x0f);
   port_set_attr(KBRD_PORT2, 0x0f, &port2_attr);
}

static inline unsigned char _KBRD_IS_INT_ENABLED(void)
{
   return (port2_attr.ie & 0x0f ? 1 : 0);
}

// Start timer to fight against button ringing
static void kbrd_startAntiRingTimer(void)
{
// start_TB(_KBRD_ANTIRING_DELAY);

   if(set_stimer(KBRD_ANTIRING_TIMER, /*_KBRD_ANTIRING_DELAY */ 20) == ENOERR)
   {
      port_write(4, 0x08, PIN_LO);
   }
}

// Start timer to stop Button press/release sound
static void kbrd_startSoundTimer(void)
{
   //uint32_t tpoint = get_atimer_counter() + 16384;//(uint32_t)_KBRD_SOUND_DURATION * 1000 / 32;
//   if(set_stimer(KBRD_SOUND_TIMER, _KBRD_SOUND_DURATION ) == ENOERR)
   //if(set_atimer(KBRD_SOUND_TIMER, tpoint ) == ENOERR)
//   if(set_atimer(0, get_atimer_counter() + 16384 ) == ENOERR)
   {
      kbrd_sound = 1;
//      port_write(KBRD_SOUND_PORT, KBRD_SOUND_PIN, PIN_HI);
   }
//   start_TA(_KBRD_SOUND_TIME);
}

// Function to clear antiringing buffer
void kbrd_clear_ring_buf(void)
{
   unsigned int i;

   for(i = sizeof(kbrd_buf); i--; )
      kbrd_ring_buf[i] = 0;
}

static inline void _KBRD_SETUP_PORTS(void)
{
   _KBRD_CLR_INT_FLAGS();

   /*
   PIN_SET(port6_attr.dir, 0x0f, PIN_HI);
   PIN_CLEAR(port6_attr.sel, 0x0f);
   port_set_attr(KBRD_PORT6, 0x0f, &port6_attr);
   port_write(KBRD_PORT6, 0x0f, PIN_HI);
   */
   P6OUT |=  0x0f;
   P6DIR |=  0x0f;
   P6SEL &= ~0x0f;

   P2DIR &= ~0x0f;
   P2SEL &= ~0x0f;
   P2IES &= ~0x0f;
   P2IFG &= ~0x0f;

   /*
   PIN_CLEAR(port2_attr.ie,  0x0f);
   PIN_CLEAR(port2_attr.dir, 0x0f);
   PIN_CLEAR(port2_attr.sel, 0x0f);
   PIN_CLEAR(port2_attr.ies, 0x0f);
   port_set_attr(KBRD_PORT2, 0x0f, &port2_attr);
   */
}


// Kyeboard hardware and driver initialization
void init_kbrd(void)
{
   P2IE  &= ~0x0f; // Disable interrupts
//   _KBRD_DISABLE_INT();

   kbrd_buf_pos     = 0;
   kbrd_buf_symbols = 0;
   kbrd_prev_state  = 0;
   kbrd_pressed     = 0;
   kbrd_sound = 0;

   PIN_SET(port1_attr.dir, KBRD_SOUND_PIN, PIN_HI);
   PIN_CLEAR(port1_attr.sel, KBRD_SOUND_PIN);
   port_set_attr(KBRD_SOUND_PORT, KBRD_SOUND_PIN, &port1_attr);
   port_write(KBRD_SOUND_PORT, KBRD_SOUND_PORT, PIN_LO);

   kbrd_clear_ring_buf();

   _KBRD_SETUP_PORTS();
   /*
   P6OUT |=  0x0f; // Row0-3
   P6DIR |=  0x0f; // Outputs
   P6SEL &= ~0x0f;

   P2DIR &= ~0x0f; // Col0-3
   P2SEL &= ~0x0f; // Inputs
   P2IES &= ~0x0f; // Wait for low-to-high transition
   P2IFG &= ~0x0f; // Clear interrupt flags
   */
   
   P2IE  |=  0x0f; // Enable interrupts
   //_KBRD_ENABLE_INT();
}

unsigned char kbrd_getSymbolCount(void)
{
   return kbrd_buf_symbols;
}

unsigned char kbrd_putch(unsigned char ch)
{
   kbrd_buf[kbrd_buf_pos++] = ch;
   kbrd_buf_pos &= (_KBRD_BUF_SIZE - 1);
   if(++kbrd_buf_symbols > _KBRD_BUF_SIZE)
      kbrd_buf_symbols = _KBRD_BUF_SIZE;
   return kbrd_buf_symbols;
}

unsigned char kbrd_getch(void)
{
   unsigned char ch;
   unsigned char pos = kbrd_buf_pos;
   if(!kbrd_buf_symbols)
      return 0;
   ch = kbrd_buf[(pos - 1) & (_KBRD_BUF_SIZE - 1)];
   kbrd_buf_symbols--;
   return ch;
}

int kbrd_service(void)
{
   unsigned char P2_val;
   unsigned int t, i;
//   port_write(4, 0x01, PIN_LO);
//   P2IE  &= ~0x0f; // Disable interrupts
   do
   {
      P2_val = P2IN & 0x0f;
      //P2_val = _KBRD_READ_COLUMNS();
      t = kbrd_scan();
      //_KBRD_CLR_INT_FLAGS();
      //P2IFG &= ~0x0f;
      //_KBRD_SETUP_EDGES(P2_val);
      P2IES = (P2IES & 0xf0) | P2_val;
      P2IFG &= ~0x0f;
   }while(/*!_KBRD_ARE_EDGES_SET()*/((P2IES ^ P2IN) & 0x0f));
   if(kbrd_sound)
   {
      kbrd_sound = 0;
      port_write(KBRD_SOUND_PORT, KBRD_SOUND_PIN, PIN_HI);
      for(i = 0xffff / 4; i--;);
      port_write(KBRD_SOUND_PORT, KBRD_SOUND_PIN, PIN_LO);
   }
//   P2IE  |=  0x0f; // Enable interrupts
//   port_write(4, 0x01, PIN_HI);
/*
   P4OUT |= 0x01;
   if(P2IES & 0x08)
      P4OUT &= ~0x01;
*/
//   start_TB(625 - 1); // Antiringing
//   kbrd_startAntiRingTimer();
   return t;
}


static unsigned int kbrd_scan(void)
{
   unsigned char i, j;
   unsigned char row_mask;
   unsigned char cols[4];
   unsigned int kbrd_mask;
   unsigned int kbrd_state = 0, changes = 0;
   unsigned int tmp;

   
   row_mask = 0x01;
   for(i = 0; i < 4; i++)
   {
      P6OUT  &= ~0x0f;
      P6OUT  |= row_mask;
//      _KBRD_SET_ROWS(row_mask);
      cols[i] = P2IN & 0x0f;
      cols[i] = P2IN & 0x0f;
      cols[i] = P2IN & 0x0f;
      cols[i] = P2IN & 0x0f;
      cols[i] = P2IN & 0x0f;
//      cols[i] = _KBRD_READ_COLUMNS(); // P2IN & 0x0f;
      row_mask <<= 1;
   }
   //_KBRD_SET_ROWS(0x0f);
   P6OUT |= 0x0f;

   kbrd_state = 0;
   changes = 0;
   for(i = 4; i--; )
      kbrd_state |= (unsigned int)cols[i] << (i * 4);

//   tmp = kbrd_state;
   
   changes = kbrd_prev_state ^ kbrd_state;
   kbrd_mask = 0x01;
   for(i = 1; i < 17; i++, kbrd_mask <<= 1)
   {
      if(!kbrd_ring_buf[i - 1])
      {
         j = kbrd_state & kbrd_mask ? i : 0x80 | i;
         if(changes & kbrd_mask)
         {
            kbrd_putch(j);
            kbrd_ring_buf[i - 1] = 1;
            kbrd_startAntiRingTimer();
            if(kbrd_state & kbrd_mask)
               kbrd_startSoundTimer();
//            start_TA(31 - 1);
            kbrd_pressed = 1;
         }
      }
      else
         kbrd_state = (kbrd_state & (~kbrd_mask)) | (kbrd_prev_state & kbrd_mask);
//         kbrd_state >>= 1;
//         changes >>= 1;
   }
   kbrd_prev_state = kbrd_state;

   return tmp;
}
/*
void __attribute((wakeup)) __attribute((interrupt(2))) P2_IRQ(void)
{
   unsigned int i;
   if(P2IFG & 0x0f) // Any button
      kbrd_service();
}
*/

unsigned char get_digit(unsigned char code)
{
   if(code == 0x10)
      return '0';
   if(code == 0x0f)
      return '5';
   if((code >= 5) && (code <= 8))
      return '1' + code - 5;
   if((code >= 9) && (code <= 0x0c))
      return '6' + code - 9;
   return code;
}

