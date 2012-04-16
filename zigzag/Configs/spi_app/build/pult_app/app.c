#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 38 "/opt/msp430-3.3.6/msp430/include/sys/inttypes.h"
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 290 "/usr/lib/ncc/nesc_nx.h"
typedef struct { char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;



typedef struct { char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 151 "/opt/msp430-3.3.6/lib/gcc-lib/msp430/3.3.6/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 41 "/opt/msp430-3.3.6/msp430/include/sys/types.h"
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 64 "/opt/msp430-3.3.6/msp430/include/string.h"
extern void bzero(void *, size_t );
# 59 "/opt/msp430-3.3.6/msp430/include/stdlib.h"
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/opt/msp430-3.3.6/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/opt/msp430-3.3.6/msp430/include/sys/_types.h"
typedef long _off_t;
typedef long _ssize_t;
# 19 "/opt/msp430-3.3.6/msp430/include/sys/reent.h"
typedef unsigned long __ULong;
# 31 "/opt/msp430-3.3.6/msp430/include/sys/reent.h" 3
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int );




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/opt/msp430-3.3.6/msp430/include/math.h"
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 91 "/home/max/tinyos/tinyos-1.x/tos/system/tos.h"
typedef unsigned char bool;






enum __nesc_unnamed4247 {
  FALSE = 0, 
  TRUE = 1
};



enum __nesc_unnamed4248 {
  FAIL = 0, 
  SUCCESS = 1
};


static inline uint8_t rcombine(uint8_t r1, uint8_t r2);
typedef uint8_t result_t  ;







static inline result_t rcombine(result_t r1, result_t r2);
#line 140
enum __nesc_unnamed4249 {
  NULL = 0x0
};
# 39 "/opt/msp430-3.3.6/msp430/include/msp430/iostructures.h"
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4250 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
# 107 "/opt/msp430-3.3.6/msp430/include/msp430/iostructures.h" 3
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};









struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 110 "/opt/msp430-3.3.6/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1IN __asm ("0x0020");

volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");

volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");

volatile unsigned char P2IFG __asm ("0x002B");



volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");
#line 169
volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 87 "/opt/msp430-3.3.6/msp430/include/msp430/usart.h"
volatile unsigned char U0TCTL __asm ("0x0071");
#line 256
volatile unsigned char U1CTL __asm ("0x0078");

volatile unsigned char U1TCTL __asm ("0x0079");



volatile unsigned char U1MCTL __asm ("0x007B");

volatile unsigned char U1BR0 __asm ("0x007C");

volatile unsigned char U1BR1 __asm ("0x007D");

volatile unsigned char U1RXBUF __asm ("0x007E");
# 24 "/opt/msp430-3.3.6/msp430/include/msp430/flash.h"
volatile unsigned int FCTL3 __asm ("0x012C");
# 20 "/opt/msp430-3.3.6/msp430/include/msp430/compa.h"
volatile unsigned char CACTL1 __asm ("0x0059");
# 25 "/opt/msp430-3.3.6/msp430/include/msp430/timera.h"
volatile unsigned int TA0IV __asm ("0x012E");

volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");
# 127 "/opt/msp430-3.3.6/msp430/include/msp430/timera.h" 3
#line 118
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 22 "/opt/msp430-3.3.6/msp430/include/msp430/timerb.h"
volatile unsigned int TBIV __asm ("0x011E");

volatile unsigned int TBCTL __asm ("0x0180");

volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");



volatile unsigned int TBCCTL2 __asm ("0x0186");

volatile unsigned int TBCCR0 __asm ("0x0192");



volatile unsigned int TBCCR2 __asm ("0x0196");




volatile unsigned int TBCCTL3 __asm ("0x0188");

volatile unsigned int TBCCTL4 __asm ("0x018A");

volatile unsigned int TBCCTL5 __asm ("0x018C");

volatile unsigned int TBCCTL6 __asm ("0x018E");
#line 76
#line 64
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/opt/msp430-3.3.6/msp430/include/msp430/basic_clock.h"
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/opt/msp430-3.3.6/msp430/include/msp430/adc12.h"
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4257 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 18 "/opt/msp430-3.3.6/msp430/include/msp430/dac12.h"
volatile unsigned int DAC12_0CTL __asm ("0x01C0");
# 56 "/opt/msp430-3.3.6/msp430/include/msp430x16x.h"
volatile unsigned char IE1 __asm ("0x0000");








volatile unsigned char IFG1 __asm ("0x0002");







volatile unsigned char IE2 __asm ("0x0001");









volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 196 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
static inline void __nesc_disable_interrupt(void);





static inline void __nesc_enable_interrupt(void);




static inline bool are_interrupts_enabled(void);




typedef bool __nesc_atomic_t;

static inline __nesc_atomic_t __nesc_atomic_start(void );
static inline void __nesc_atomic_end(__nesc_atomic_t oldSreg);



static inline __nesc_atomic_t __nesc_atomic_start(void );






static inline void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);








 bool LPMode_disabled = FALSE;

static inline void LPMode_enable(void);







static __inline void __nesc_atomic_sleep(void);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_app/hardware.h"
static inline void TOSH_SET_SATELLITE_INTA_PIN(void);
#line 37
static inline void TOSH_CLR_SATELLITE_INTA_PIN(void);
#line 56
static inline void TOSH_SEL_SIMO1_MODFUNC(void);
static inline void TOSH_SEL_SOMI1_MODFUNC(void);
static inline void TOSH_SEL_UCLK1_MODFUNC(void);




static inline void TOSH_SET_SATELLITE_INTR_2_PIN(void);
#line 63
static inline void TOSH_CLR_SATELLITE_INTR_2_PIN(void);
#line 63
static inline void TOSH_MAKE_SATELLITE_INTR_2_OUTPUT(void);


static inline void TOSH_SET_PIN_DIRECTIONS(void );
# 54 "/home/max/tinyos/tinyos-1.x/tos/types/dbg_modes.h"
typedef long long TOS_dbg_mode;



enum __nesc_unnamed4258 {
  DBG_ALL = ~0ULL, 


  DBG_BOOT = 1ULL << 0, 
  DBG_CLOCK = 1ULL << 1, 
  DBG_TASK = 1ULL << 2, 
  DBG_SCHED = 1ULL << 3, 
  DBG_SENSOR = 1ULL << 4, 
  DBG_LED = 1ULL << 5, 
  DBG_CRYPTO = 1ULL << 6, 


  DBG_ROUTE = 1ULL << 7, 
  DBG_AM = 1ULL << 8, 
  DBG_CRC = 1ULL << 9, 
  DBG_PACKET = 1ULL << 10, 
  DBG_ENCODE = 1ULL << 11, 
  DBG_RADIO = 1ULL << 12, 


  DBG_LOG = 1ULL << 13, 
  DBG_ADC = 1ULL << 14, 
  DBG_I2C = 1ULL << 15, 
  DBG_UART = 1ULL << 16, 
  DBG_PROG = 1ULL << 17, 
  DBG_SOUNDER = 1ULL << 18, 
  DBG_TIME = 1ULL << 19, 
  DBG_POWER = 1ULL << 20, 



  DBG_SIM = 1ULL << 21, 
  DBG_QUEUE = 1ULL << 22, 
  DBG_SIMRADIO = 1ULL << 23, 
  DBG_HARD = 1ULL << 24, 
  DBG_MEM = 1ULL << 25, 



  DBG_USR1 = 1ULL << 27, 
  DBG_USR2 = 1ULL << 28, 
  DBG_USR3 = 1ULL << 29, 
  DBG_TEMP = 1ULL << 30, 

  DBG_ERROR = 1ULL << 31, 
  DBG_NONE = 0, 

  DBG_DEFAULT = DBG_ALL
};
# 61 "/home/max/tinyos/tinyos-1.x/tos/system/sched.c"
#line 59
typedef struct __nesc_unnamed4259 {
  void (*tp)(void);
} TOSH_sched_entry_T;

enum __nesc_unnamed4260 {




  TOSH_MAX_TASKS = 1 << 4, 



  TOSH_TASK_BITMASK = TOSH_MAX_TASKS - 1
};

volatile TOSH_sched_entry_T TOSH_queue[TOSH_MAX_TASKS];
uint8_t TOSH_sched_full;
volatile uint8_t TOSH_sched_free;

static inline void TOSH_sched_init(void );








bool TOS_post(void (*tp)(void));
#line 102
bool TOS_post(void (*tp)(void))  ;
#line 136
static inline bool TOSH_run_next_task(void);
#line 159
static inline void TOSH_run_task(void);
# 28 "/home/max/tinyos/tinyos-1.x/tos/system/Ident.h"
enum __nesc_unnamed4261 {

  IDENT_MAX_PROGRAM_NAME_LENGTH = 16
};






#line 33
typedef struct __nesc_unnamed4262 {

  uint32_t unix_time;
  uint32_t user_hash;
  char program_name[IDENT_MAX_PROGRAM_NAME_LENGTH];
} Ident_t;
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.h"
enum __nesc_unnamed4263 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4264 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} MSP430CompareControl_t;
#line 76
#line 66
typedef struct __nesc_unnamed4265 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} MSP430TimerAControl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4266 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} MSP430TimerBControl_t;
# 24 "../../../include/irq.h"
void __critical_enter(void);




void __critical_exit(void);
#line 46
void __process_irq(uint16_t irq_num);
# 19 "../../../include/time.h"
uint64_t __sys_time(void);




int16_t __set_sys_time(uint64_t time);
#line 39
int16_t __stimer_set(const uint8_t tnum, const uint32_t milli_sec);





void __stimer_fired(const uint8_t tnum);
# 16 "../../../include/task.h"
void __app_init(void);




typedef void (*task_ft)(void);




int16_t __post_task(task_ft task_func);
# 17 "../../../include/net.h"
void __net_enter(uint16_t pan_id);




void __net_exit(void);
#line 36
int16_t __net_send(const uint16_t dst_addr, const uint16_t data_size, 
const uint8_t *const data, uint8_t handle);







void __net_send_done(uint8_t handle, int8_t status);









void __net_recv(uint16_t src_addr, uint64_t src_ext_addr, uint16_t data_size, uint8_t *data, uint8_t lqi);




uint16_t __net_addr(void);










void __net_child_join(uint16_t short_addr, uint64_t ext_addr);





void __net_child_leave(uint64_t ext_addr);
# 14 "../../../include/port.h"
enum __nesc_unnamed4267 {
  OP_WRITE = 0, 
  OP_READ, 
  OP_SET_ATTR, 
  OP_GET_ATTR, 
  OP_GET_IFLAG, 
  OP_RESET_IFLAG
};








int16_t __port_perm(const uint8_t port_num, const uint8_t mask, const uint8_t op);
# 18 "../../../include/syszig.h"
typedef uint16_t faddr_t;


extern uint16_t __module_load;
# 4 "../../../zigzag/IEEE802_15_4/SysTimer/public/SysTime.h"
typedef uint32_t TSysTime;

typedef uint32_t TMilliSec;
# 6 "../../../zigzag/IEEE802_15_4/SysCommon/public/SysCommon.h"
typedef uint16_t TUniData;
# 4 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/SysTimer.h"
enum __nesc_unnamed4268 {
  MAX_SYS_JIFFY_HI = 0x7fffUL, 
  MAX_SYS_JIFFY_LO = 0xffffUL, 
  MAX_SYS_JIFFY = (uint32_t )(MAX_SYS_JIFFY_HI << 16) | MAX_SYS_JIFFY_LO
};
# 4 "../../../zigzag/IEEE802_15_4/Phy/public/PhyConst.h"
enum __nesc_unnamed4269 {

  PHY_AMAX_PHY_PACKET_SIZE = 127, 


  PHY_ATURNAROUND_TIME = 12
};
# 6 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPrivate.h"
enum __nesc_unnamed4270 {
#line 6
  PHY_SIZE_OF_FRAME_LENGTH = 1
};
#line 7
enum __nesc_unnamed4271 {
#line 7
  MAX_PPDU_LENGTH = PHY_SIZE_OF_FRAME_LENGTH + PHY_AMAX_PHY_PACKET_SIZE
};
enum __nesc_unnamed4272 {
#line 9
  CRC16_LENGTH = 2
};
typedef uint8_t _TPhyFrameLength;

typedef uint32_t _TPhyTimeStamp;





#line 15
typedef struct __nesc_unnamed4273 {
  _TPhyTimeStamp timeStamp;
  uint8_t current;
  uint8_t data[MAX_PPDU_LENGTH];
} _TPhyFrame;

typedef uint8_t _TPhyLinkQuality;

typedef uint8_t _TPhyChannel;

typedef uint32_t _TPhyChannelsSupported;




#line 27
typedef struct __nesc_unnamed4274 {
  unsigned tolerance : 2;
  signed txPower : 6;
} _TPhyTransmitPower;







#line 32
typedef enum __nesc_unnamed4275 {
  _PHY_CCA_MODE_FIRST = 1, 
  _PHY_CCA_MODE_1 = 1, 
  _PHY_CCA_MODE_2, 
  _PHY_CCA_MODE_3 = 3, 
  _PHY_CCA_MODE_LAST = 3
} _TPhyCCAMode;

typedef uint8_t _TPhyEnergyLevel;
# 31 "../../../zigzag/IEEE802_15_4/Phy/public/Phy.h"
#line 7
typedef enum __nesc_unnamed4276 {

  PHY_BUSY = 0x00, 

  PHY_BUSY_RX = 0x01, 

  PHY_BUSY_TX = 0x02, 

  PHY_FORCE_TRX_OFF = 0x03, 

  PHY_IDLE = 0x04, 


  PHY_INVALID_PARAMETER = 0x05, 

  PHY_RX_ON = 0x06, 

  PHY_SUCCESS = 0x07, 

  PHY_TRX_OFF = 0x08, 

  PHY_TX_ON = 0x09, 

  PHY_UNSUPPORTED_ATTRIBUTE = 0x0a
} TPhyStatus;

typedef _TPhyLinkQuality TPhyLinkQuality;

typedef _TPhyFrameLength TPhyFrameLength;

typedef _TPhyChannel TPhyChannel;

typedef _TPhyChannelsSupported TPhyChannelsSupported;

typedef _TPhyTransmitPower TPhyTransmitPower;







#line 43
typedef enum __nesc_unnamed4277 {
  PHY_CCA_MODE_FIRST = _PHY_CCA_MODE_FIRST, 
  PHY_CCA_MODE_1 = _PHY_CCA_MODE_1, 
  PHY_CCA_MODE_2 = _PHY_CCA_MODE_2, 
  PHY_CCA_MODE_3 = _PHY_CCA_MODE_3, 
  PHY_CCA_MODE_LAST = _PHY_CCA_MODE_LAST
} TPhyCCAMode;

typedef _TPhyEnergyLevel TPhyEnergyLevel;

typedef _TPhyTimeStamp TPhyTimeStamp;

typedef _TPhyFrame TPhyFrame;

typedef uint8_t TPhyPoolHandle;
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/MacCommon.h"
enum __nesc_unnamed4278 {

  MAC_ABASE_SLOT_DURATION = 60, 


  MAC_ANUM_SUPERFRAME_SLOTS = 16, 


  MAC_ABASE_SUPERFRAME_DURATION = MAC_ABASE_SLOT_DURATION * MAC_ANUM_SUPERFRAME_SLOTS, 
#line 35
  MAC_AMAX_BE = 5, 


  MAC_AMAX_BEACON_OVERHEAD = 75, 


  MAC_AMAX_BEACON_PAYLOAD_LENGTH = PHY_AMAX_PHY_PACKET_SIZE - MAC_AMAX_BEACON_OVERHEAD, 



  MAC_AGTS_DESC_PERSISTENCE_TIME = 4, 


  MAC_AMAX_FRAME_OVERHEAD = 25, 



  MAC_AMAX_FRAME_RESPONSE_TIME = 1220, 


  MAC_AMAX_FRAME_RETRIES = 3, 



  MAC_AMAX_LOST_BEACONS = 8, 



  MAC_AMAX_MAC_FRAME_SIZE = PHY_AMAX_PHY_PACKET_SIZE - MAC_AMAX_FRAME_OVERHEAD, 



  MAC_AMAX_SIFS_FRAME_SIZE = 18, 


  MAC_AMIN_CAP_LENGTH = 440, 


  MAC_AMIN_LIFS_PERIOD = 40, 


  MAC_AMIN_SIFS_PERIOD = 12, 



  MAC_ARESPONSE_WAIT_TIME = 32 * MAC_ABASE_SUPERFRAME_DURATION, 



  MAC_AUNIT_BACKOFF_PERIOD = 20
};
#line 140
#line 89
typedef enum __nesc_unnamed4279 {

  MAC_SUCCESS = 0x00, 

  MAC_PAN_AT_CAPACITY = 0x01, 

  MAC_PAN_ACCESS_DENIED = 0x02, 

  MAC_BEACON_LOSS = 0xe0, 

  MAC_CHANNEL_ACCESS_FAILURE = 0xe1, 

  MAC_DENIED = 0xe2, 

  MAC_DISABLE_TRX_FAILURE = 0xe3, 

  MAC_FAILED_SECURITY_CHECK = 0xe4, 

  MAC_FRAME_TOO_LONG = 0xe5, 


  MAC_INVALID_GTS = 0xe6, 


  MAC_INVALID_HANDLE = 0xe7, 

  MAC_INVALID_PARAMETER = 0xe8, 

  MAC_NO_ACK = 0xe9, 

  MAC_NO_BEACON = 0xea, 

  MAC_NO_DATA = 0xeb, 

  MAC_NO_SHORT_ADDRESS = 0xec, 

  MAC_OUT_OF_CAP = 0xed, 

  MAC_PAN_ID_CONFLICT = 0xee, 

  MAC_REALIGNMENT = 0xef, 

  MAC_TRANSACTION_EXPIRED = 0xf0, 

  MAC_TRANSACTION_OVERFLOW = 0xf1, 

  MAC_TX_ACTIVE = 0xf2, 

  MAC_UNAVAILABLE_KEY = 0xf3, 

  MAC_UNSUPPORTED_ATTRIBUTE = 0xf4
} TMacStatus;
#line 152
#line 142
typedef enum __nesc_unnamed4280 {
  MAC_CMD_ASSOC_REQUEST = 0x01, 
  MAC_CMD_ASSOC_RESPONSE = 0x02, 
  MAC_CMD_DISASSOC_NOTIFICATION = 0x03, 
  MAC_CMD_DATA_REQUEST = 0x04, 
  MAC_CMD_PAN_ID_CONFLICT = 0x05, 
  MAC_CMD_ORPHAN_NOTIFICATION = 0x06, 
  MAC_CMD_BEACON_REQUEST = 0x07, 
  MAC_CMD_COORDINATOR_REALIGNMENT = 0x08, 
  MAC_CMD_GTS_REQUEST = 0x09
} TMacCommand;

typedef uint8_t TMacDSN;




#line 156
typedef enum __nesc_unnamed4281 {
  MAC_ACK_WAIT_DURATION_54 = 54 * 4, 
  MAC_ACK_WAIT_DURATION_120 = 120
} TMacAckWaitDuration;





#line 161
typedef enum __nesc_unnamed4282 {
  MAC_UNSECURED_MODE = 0x00, 
  MAC_ACL_MODE = 0x01, 
  MAC_SECURED_MODE = 0x02
} TMacSecurityMode;
# 4 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressPrv.h"
enum __nesc_unnamed4283 {
#line 4
  _MAC_PANID_LENGTH = 2
};
enum __nesc_unnamed4284 {
#line 6
  _MAC_SHORT_ADDRESS_LENGTH = 2
};
enum __nesc_unnamed4285 {
#line 8
  _MAC_EXTENDED_ADDRESS_LENGTH = 8
};
typedef uint16_t _TMacPANId;





#line 12
typedef enum __nesc_unnamed4286 {
  _MAC_ADDRESS_NOT_PRESENT = 0x0, 
  _MAC_ADDRESS_SHORT = 0x2, 
  _MAC_ADDRESS_EXTENDED = 0x3
} _TMacAddressMode;

typedef uint16_t _TMacShortAddress;

typedef uint64_t _TMacExtendedAddress;







#line 22
typedef struct __nesc_unnamed4287 {
  _TMacAddressMode mode;
  union __nesc_unnamed4288 {
    _TMacShortAddress shortAddress;
    _TMacExtendedAddress extendedAddress;
  } address;
} _TMacAddress;
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/MacAddress.h"
enum __nesc_unnamed4289 {
#line 6
  MAC_SHORT_ADDRESS_LENGTH = _MAC_SHORT_ADDRESS_LENGTH
};
enum __nesc_unnamed4290 {
#line 8
  MAC_EXTENDED_ADDRESS_LENGTH = _MAC_EXTENDED_ADDRESS_LENGTH
};
enum __nesc_unnamed4291 {
#line 10
  MAC_PANID_LENGTH = _MAC_PANID_LENGTH
};




#line 12
typedef enum __nesc_unnamed4292 {
  MAC_ADDRESS_NOT_PRESENT = _MAC_ADDRESS_NOT_PRESENT, 
  MAC_ADDRESS_SHORT = _MAC_ADDRESS_SHORT, 
  MAC_ADDRESS_EXTENDED = _MAC_ADDRESS_EXTENDED
} TMacAddressMode;

typedef _TMacPANId TMacPANId;

typedef _TMacShortAddress TMacShortAddress;

typedef _TMacExtendedAddress TMacExtendedAddress;

typedef _TMacAddress TMacAddress;
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormat.h"
enum __nesc_unnamed4293 {
#line 7
  MAC_FRAME_CONTROL_LENGTH = 2
};






#line 8
typedef struct __nesc_unnamed4294 {
  unsigned frameType : 3;
  unsigned securityEnabled : 1;
  unsigned framePending : 1;
  unsigned ackRequest : 1;
  unsigned intraPAN : 1;
  unsigned  : 1;
} TMacLSBFrameControl;






#line 17
typedef struct __nesc_unnamed4295 {
  unsigned  : 2;
  unsigned dstAddressMode : 2;
  unsigned  : 2;
  unsigned srcAddressMode : 2;
} TMacMSBFrameControl;










#line 24
typedef struct __nesc_unnamed4296 {
  union __nesc_unnamed4297 {
    TMacLSBFrameControl value;
    uint8_t raw;
  } lsb;
  union __nesc_unnamed4298 {
    TMacMSBFrameControl value;
    uint8_t raw;
  } msb;
} TMacFrameControl;
typedef TMacFrameControl TMacFrameControlField;

enum __nesc_unnamed4299 {
#line 36
  MAC_SEQUENCE_NUMBER_LENGTH = 1
};
#line 37
typedef uint8_t _TMacSequenceNumber;



#line 38
typedef union __nesc_unnamed4300 {
  uint8_t raw[MAC_SEQUENCE_NUMBER_LENGTH];
  _TMacSequenceNumber value;
} TMacSequenceNumberField;




#line 43
typedef union __nesc_unnamed4301 {
  uint8_t raw[MAC_PANID_LENGTH];
  TMacPANId value;
} TMacPANIdField;




#line 48
typedef union __nesc_unnamed4302 {
  uint8_t raw[MAC_SHORT_ADDRESS_LENGTH];
  TMacShortAddress value;
} TMacShortAddressField;




#line 53
typedef union __nesc_unnamed4303 {
  uint8_t raw[MAC_EXTENDED_ADDRESS_LENGTH];
  TMacExtendedAddress value;
} TMacExtendedAddressField;




#line 58
typedef union __nesc_unnamed4304 {
  TMacShortAddressField _short;
  TMacExtendedAddressField _extended;
} TMacAddressField;

typedef uint8_t _TMacPayloadLength;
enum __nesc_unnamed4305 {
#line 64
  MAC_PAYLOAD_LENGTH = MAC_AMAX_MAC_FRAME_SIZE
};


#line 65
typedef struct __nesc_unnamed4306 {
  _TMacPayloadLength length;
  uint8_t raw[MAC_PAYLOAD_LENGTH];
} TMacPayloadField;

enum __nesc_unnamed4307 {
#line 70
  MAC_FRAME_CHECK_SEQUENCE_LENGTH = 2
};
#line 71
typedef uint16_t _TMacFrameCheckSequence;



#line 72
typedef union __nesc_unnamed4308 {
  uint8_t raw[MAC_FRAME_CHECK_SEQUENCE_LENGTH];
  _TMacFrameCheckSequence value;
} TMacFrameCheckSequenceField;

typedef uint8_t _TMacRawFrameLength;

typedef uint32_t _TMacTimeStamp;

typedef uint8_t _TMacLinkQuality;
#line 93
#line 83
typedef struct __nesc_unnamed4309 {
  _TMacTimeStamp timeStamp;
  TMacFrameControlField frameControl;
  TMacSequenceNumberField sequenceNumber;
  TMacPANIdField dstPANId;
  TMacAddressField dstAddress;
  TMacPANIdField srcPANId;
  TMacAddressField srcAddress;
  TMacPayloadField payload;
  _TMacLinkQuality linkQuality;
} _TMacFrame;
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/MacFrame.h"
#line 7
typedef enum __nesc_unnamed4310 {
  MAC_FRAME_TYPE_BEACON, 
  MAC_FRAME_TYPE_DATA, 
  MAC_FRAME_TYPE_ACK, 
  MAC_FRAME_TYPE_COMMAND
} TMacFrameType;

typedef _TMacSequenceNumber TMacSequenceNumber;

typedef _TMacPayloadLength TMacPayloadLength;

typedef _TMacFrame TMacFrame;

typedef _TMacRawFrameLength TMacRawFrameLength;

typedef _TMacLinkQuality TMacLinkQuality;

typedef _TMacTimeStamp TMacTimeStamp;




#line 26
typedef enum __nesc_unnamed4311 {
  MAC_SEND_MODE_DIRECT, 
  MAC_SEND_MODE_CSMA
} TMacSendMode;
# 4 "../../../zigzag/IEEE802_15_4/MacCommon/public/MacSecurity.h"
typedef uint8_t TACLEntry;
# 4 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/MacSuperframe.h"
typedef uint8_t TMacBeaconOrder;

typedef uint8_t TMacSuperframeOrder;







#line 8
typedef enum __nesc_unnamed4312 {
  MAC_SLOT_TYPE_UNKNOWN = 0, 
  MAC_SLOT_TYPE_FREE = 1, 
  MAC_SLOT_TYPE_BEACON = 2, 
  MAC_SLOT_TYPE_CAP = 3, 
  MAC_SLOT_TYPE_CFP = 4
} TMacSlotType;
enum __nesc_unnamed4313 {
#line 15
  SLOT_TYPE_NUMBER = 5
};
#line 31
#line 26
typedef enum __nesc_unnamed4314 {
  MAC_SUPERFRAME_MODE_INVALID = 0, 
  MAC_SUPERFRAME_MODE_UNSLOTTED, 
  MAC_SUPERFRAME_MODE_SLOTTED, 
  MAC_SUPERFRAME_MODE_PASSIVE
} TMacSuperframeMode;
# 4 "../../../zigzag/IEEE802_15_4/MacPool/public/MacPool.h"
typedef uint16_t TMacPoolHandle;

typedef uint16_t TMacTransactionPersistenceTime;
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/MacCAP.h"
#line 4
typedef enum __nesc_unnamed4315 {
  MAC_BATT_LIFE_EXT_PERIOD_6 = 6, 
  MAC_BATT_LIFE_EXT_PERIOD_8 = 8
} TMacBattLifeExtPeriods;










#line 9
typedef enum __nesc_unnamed4316 {
  MAC_MAX_CSMA_BACKOFFS_FIST = 0, 
  MAC_MAX_CSMA_BACKOFFS_0 = 0, 
  MAC_MAX_CSMA_BACKOFFS_1, 
  MAC_MAX_CSMA_BACKOFFS_2, 
  MAC_MAX_CSMA_BACKOFFS_3, 
  MAC_MAX_CSMA_BACKOFFS_4, 
  MAC_MAX_CSMA_BACKOFFS_5 = 5, 
  MAC_MAX_CSMA_BACKOFFS_LAST = 5
} TMacMaxCSMABackoffs;










#line 20
typedef enum __nesc_unnamed4317 {
  MAC_BACKOFF_EXPONENT_FIRST = 0, 
  MAC_BACKOFF_EXPONENT_0 = 0, 
  MAC_BACKOFF_EXPONENT_1, 
  MAC_BACKOFF_EXPONENT_2, 
  MAC_BACKOFF_EXPONENT_3, 
  MAC_BACKOFF_EXPONENT_4, 
  MAC_BACKOFF_EXPONENT_5, 
  MAC_BACKOFF_EXPONENT_LAST = 5
} TMacBackoffExponent;
# 4 "../../../zigzag/IEEE802_15_4/MacAssoc/public/MacAssoc.h"
enum __nesc_unnamed4318 {
  MAC_DEVICE_TYPE_RFD = 0, 
  MAC_DEVICE_TYPE_FFD = 1
};









#line 9
typedef struct __nesc_unnamed4319 {
  unsigned int alternatePANCoordinator : 1;
  unsigned int deviceType : 1;
  unsigned int powerSource : 1;
  unsigned int receiverOnWhenIdle : 1;
  unsigned int  : 2;
  unsigned int securityCapability : 1;
  unsigned int allocateAddress : 1;
} TCapabilityInformation;




#line 19
typedef enum __nesc_unnamed4320 {
  MAC_DISASSOCIATE_BY_COORD = 0x01, 
  MAC_DISASSOCIATE_BY_DEVICE = 0x02
} TDisassociateReason;
#line 54
#line 24
typedef enum __nesc_unnamed4321 {
  MAC_LOGICAL_CHANNEL_FIRST = 0, 
  MAC_LOGICAL_CHANNEL_0 = 0, 
  MAC_LOGICAL_CHANNEL_1, 
  MAC_LOGICAL_CHANNEL_2, 
  MAC_LOGICAL_CHANNEL_3, 
  MAC_LOGICAL_CHANNEL_4, 
  MAC_LOGICAL_CHANNEL_5, 
  MAC_LOGICAL_CHANNEL_6, 
  MAC_LOGICAL_CHANNEL_7, 
  MAC_LOGICAL_CHANNEL_8, 
  MAC_LOGICAL_CHANNEL_9, 
  MAC_LOGICAL_CHANNEL_10, 
  MAC_LOGICAL_CHANNEL_11, 
  MAC_LOGICAL_CHANNEL_12, 
  MAC_LOGICAL_CHANNEL_13, 
  MAC_LOGICAL_CHANNEL_14, 
  MAC_LOGICAL_CHANNEL_15, 
  MAC_LOGICAL_CHANNEL_16, 
  MAC_LOGICAL_CHANNEL_17, 
  MAC_LOGICAL_CHANNEL_18, 
  MAC_LOGICAL_CHANNEL_19, 
  MAC_LOGICAL_CHANNEL_20, 
  MAC_LOGICAL_CHANNEL_21, 
  MAC_LOGICAL_CHANNEL_22, 
  MAC_LOGICAL_CHANNEL_23, 
  MAC_LOGICAL_CHANNEL_24, 
  MAC_LOGICAL_CHANNEL_25, 
  MAC_LOGICAL_CHANNEL_26 = 26, 
  MAC_LOGICAL_CHANNEL_LAST = 26
} TMacLogicalChannel;
# 14 "../../../zigzag/IEEE802_15_4/MacBeacon/public/MacBeacon.h"
#line 8
typedef struct __nesc_unnamed4322 {

  unsigned shortAddrNum : 3;
  unsigned  : 1;
  unsigned extAddrNum : 3;
} 
TMacPendAddrSpec;


typedef uint8_t *TMacAddrList;










#line 19
typedef struct __nesc_unnamed4323 {

  unsigned beaconOrder : 4;
  unsigned superframeOrder : 4;
  unsigned finalCapSlot : 4;
  unsigned battLifeExtension : 1;
  unsigned  : 1;
  unsigned panCoordinator : 1;
  unsigned associationPermit : 1;
} TMacSuperframeSpec;
#line 42
#line 30
typedef struct __nesc_unnamed4324 {

  TMacAddress coordAddr;
  TMacPANId coordPANId;
  TMacSuperframeSpec superframeSpec;
  TMacTimeStamp timeStamp;
  bool gtsPermit;
  bool securityUse;
  bool securityFailure;
  uint8_t logicalChannel;
  uint8_t linkQuality;
  uint8_t aclEntry;
} TMacPanDescriptor;

typedef uint8_t TMacBSN;
typedef uint8_t TMacBeaconPayloadLength;
typedef uint8_t *TMacBeaconPayload;
typedef uint32_t TMacBeaconTxTime;
# 15 "../../../zigzag/IEEE802_15_4/MacScan/public/MacScan.h"
#line 9
typedef enum __nesc_unnamed4325 {

  MAC_ED_SCAN = 0x0, 
  MAC_ACTIVE_SCAN = 0x1, 
  MAC_PASSIVE_SCAN = 0x2, 
  MAC_ORPHAN_SCAN = 0x3
} TMacScanType;
# 21 "../../../zigzag/ZigBee/interface/types.h"
typedef TMacShortAddress NwkAddr;
typedef TMacPANId PanID_t;
typedef TMacExtendedAddress IEEEAddr;
#line 47
#line 25
typedef enum __nesc_unnamed4326 {
  NWK_SUCCESS = 0x00, 

  NWK_INVALID_PARAMETER = 0xc1, 
  NWK_INVALID_REQUEST = 0xc2, 

  NWK_NOT_PERMITTED = 0xc3, 
  NWK_STARTUP_FAILURE = 0xc4, 
  NWK_ALREADY_PRESENT = 0xc5, 

  NWK_SYNC_FAILURE = 0xc6, 
  NWK_TABLE_FULL = 0xc7, 
  NWK_UNKNOWN_DEVICE = 0xc8, 

  NWK_UNSUPPORTED_ATTRIBUTE = 0xc9, 
  NWK_NO_NETWORKS = 0xca, 
  NWK_LEAVE_UNCONFIRMED = 0xcb, 

  NWK_MAX_FRM_CNTR = 0xcc, 
  NWK_NO_KEY = 0xcd, 
  NWK_BAD_CCM_OUTPUT = 0xce
} 
NwkStatus;
#line 63
#line 52
typedef struct __nesc_unnamed4327 {


  uint16_t panID;
  unsigned stackProfile : 4;
  unsigned zigBeeVersion : 4;
  unsigned beaconOrder : 4;
  unsigned superframeOrder : 4;
  bool permitJoining;
  uint8_t logicalChannel;
} 
NwkNetworkDescriptor;







#line 66
typedef enum __nesc_unnamed4328 {

  ZIGBEE_COORDINATOR = 0, 
  ZIGBEE_ROUTER = 1, 
  ZIGBEE_END_DEVICE = 2
} NwkDeviceType;









#line 74
typedef enum __nesc_unnamed4329 {

  NWK_PARENT = 0, 
  NWK_CHILD = 1, 
  NWK_SIBLING = 2, 
  NWK_OTHER = 3
} 
NwkRelationship;
#line 106
#line 87
typedef struct __nesc_unnamed4330 {

  PanID_t panID;
  IEEEAddr extendedAddr;
  NwkAddr networkAddr;
  TSysTime incomingBeaconTimestamp;
  TSysTime beaconTransmissionTimeOffset;
  TSysTime lastFrameTime;
  NwkDeviceType deviceType;
  NwkRelationship relationship;
  bool rxOnWhenIdle;
  bool permitJoining;
  bool potentialParent;
  uint8_t depth;
  uint8_t beaconOrder;
  uint8_t transmitFailure;
  uint8_t lqi;
  uint8_t logicalChannel;
} 
NwkNeighbor;








#line 108
typedef enum __nesc_unnamed4331 {

  ROUTE_ACTIVE = 0, 
  ROUTE_DISCOVERY_UNDERWAY = 1, 
  ROUTE_DISCOVERY_FAILED = 2, 
  ROUTE_INACTIVE = 3
} 
NwkRouteStatus;









#line 120
typedef struct __nesc_unnamed4332 {

  NwkAddr destinationAddr;
  NwkAddr nextHopAddr;
  NwkRouteStatus status;
} NwkRoute;


typedef TCapabilityInformation NwkCapabilityInfo;









#line 131
typedef enum __nesc_unnamed4333 {

  NWK_OFFLINE, 
  NWK_JOINED_AS_ED, 
  NWK_JOINED_AS_ROUTER, 
  NWK_ROUTING
} 
NwkOnlineStatus;


enum __nesc_unnamed4334 {

  NWK_DATA_FRAME = 0, 
  NWK_COMMAND_FRAME = 1
};


enum __nesc_unnamed4335 {

  SUPPRESS_DISCOVERY = 0, 
  ENABLE_DISCOVERY = 1, 
  FORCE_DISCOVERY = 2
};

enum __nesc_unnamed4336 {

  NWK_BROADCAST = 0xffff
};
#line 178
#line 160
typedef struct __nesc_unnamed4337 {

  unsigned frameType : 2;
  unsigned protocolVersion : 4;
  unsigned discoverRoute : 2;
  unsigned multicast : 1;
  unsigned security : 1;
  unsigned sourceRoute : 1;
  unsigned dstIEEEAddr : 1;
  unsigned srcIEEEAddr : 1;
  unsigned  : 3;

  NwkAddr destinationAddr;
  NwkAddr sourceAddr;
  uint8_t radius;
  uint8_t sequenceNumber;
  uint64_t sourceExtAddr;
} 
__attribute((packed))  NwkHeader;
#line 190
#line 182
typedef struct __nesc_unnamed4338 {

  uint8_t routeRequestID;
  NwkAddr sourceAddr;
  NwkAddr senderAddr;
  uint8_t forwardCost;
  uint8_t residualCost;
  uint16_t expirationTime;
} NwkRouteDiscovery;
# 16 "../../../zigzag/ZigBee/implementation/constants.h"
enum __nesc_unnamed4339 {
  NWK_COORDINATOR_CAPABLE = 0x1, 

  NWK_DISCOVERY_RETRY_LIMIT = 0x3, 
  NWK_MAX_DEPTH = 0x0f, 
  NWK_MIN_HEADER_OVERHEAD = 0x08, 
  NWK_PROTOCOL_VERSION = 0x02, 
  NWK_REPAIR_THRESHOLD = 0x03, 
  NWK_ROUTE_DISCOVERY_TIME = 0x2710, 
  NWK_MAX_BROADCAST_JITTER = 0x40, 
  NWK_INITIAL_RREQ_RETRIES = 0x03, 
  NWK_RREQ_RETRIES = 0x02, 
  NWK_RREQ_RETRY_INTERVAL = 0xfe, 
  NWK_MIN_RREQ_JITTER = 0x01, 
  NWK_MAX_RREQ_JITTER = 0x40
};
# 8 "../../../zigzag/ZigParam.h"
struct info_param_t {
  uint64_t MAC_ADDRESS;
  uint32_t Z_CHANNELS;
  uint16_t Z_PAN_ID;
  uint8_t Z_MAX_CHILDREN;
  uint8_t Z_MAX_ROUTERS;
  uint8_t Z_MAX_DEPTH;
  uint8_t Z_BEACON_ORDER;
  uint8_t Z_SUPERFRAME_ORDER;
  uint8_t Z_SCAN_ORDER;
  uint8_t ALIVE_SEND_PERIOD;
  uint8_t ALIVE_SEND_ATTEMPTS;
} __attribute((packed)) ;

struct info_param_t;
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430usart.h"
#line 31
typedef enum __nesc_unnamed4340 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;
# 43 "/opt/msp430-3.3.6/lib/gcc-lib/msp430/3.3.6/include/stdarg.h"
typedef __builtin_va_list __gnuc_va_list;
# 105 "/opt/msp430-3.3.6/lib/gcc-lib/msp430/3.3.6/include/stdarg.h" 3
typedef __gnuc_va_list va_list;
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLInitM.nc"
static  result_t HPLInitM$init(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MSP430ClockM$StdControl$init(void);






static  result_t MSP430ClockM$StdControl$start(void);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430DCOCalibM$Timer32khz$overflow(void);
#line 33
static   void MSP430DCOCalibM$TimerMicro$overflow(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlA2$getControl(void);
#line 30
static   MSP430CompareControl_t MSP430TimerM$ControlB0$getControl(void);







static   void MSP430TimerM$ControlB0$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB0$setControlAsCompare(void);



static   void MSP430TimerM$ControlB0$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB0$clearPendingInterrupt(void);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureA1$getEvent(void);
#line 32
static   uint16_t MSP430TimerM$CaptureB3$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB3$default$captured(uint16_t arg_0x40695b78);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB3$setEvent(uint16_t arg_0x40686010);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB6$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB6$default$captured(uint16_t arg_0x40695b78);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB4$getControl(void);







static   void MSP430TimerM$ControlB4$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB4$setControlAsCompare(void);



static   void MSP430TimerM$ControlB4$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB4$clearPendingInterrupt(void);
#line 30
static   MSP430CompareControl_t MSP430TimerM$ControlA0$getControl(void);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB1$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB1$default$captured(uint16_t arg_0x40695b78);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB1$default$fired(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB1$getControl(void);








static   void MSP430TimerM$ControlB1$disableEvents(void);
#line 34
static   void MSP430TimerM$ControlB1$setControl(MSP430CompareControl_t arg_0x4068c990);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureA2$getEvent(void);
#line 32
static   uint16_t MSP430TimerM$CaptureB4$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB4$default$captured(uint16_t arg_0x40695b78);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB2$getControl(void);







static   void MSP430TimerM$ControlB2$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB2$setControlAsCompare(void);



static   void MSP430TimerM$ControlB2$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB2$clearPendingInterrupt(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerA$setClockSource(uint16_t arg_0x40682668);
#line 38
static   void MSP430TimerM$TimerA$disableEvents(void);
#line 32
static   void MSP430TimerM$TimerA$clearOverflow(void);


static   void MSP430TimerM$TimerA$setMode(int arg_0x40684860);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB4$setEvent(uint16_t arg_0x40686010);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlA1$getControl(void);
#line 30
static   MSP430CompareControl_t MSP430TimerM$ControlB5$getControl(void);







static   void MSP430TimerM$ControlB5$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB5$setControlAsCompare(void);



static   void MSP430TimerM$ControlB5$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB5$clearPendingInterrupt(void);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureA0$getEvent(void);
#line 32
static   uint16_t MSP430TimerM$CaptureB2$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB2$default$captured(uint16_t arg_0x40695b78);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB2$setEvent(uint16_t arg_0x40686010);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB5$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB5$default$captured(uint16_t arg_0x40695b78);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB3$getControl(void);







static   void MSP430TimerM$ControlB3$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB3$setControlAsCompare(void);



static   void MSP430TimerM$ControlB3$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB3$clearPendingInterrupt(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerB$setClockSource(uint16_t arg_0x40682668);
#line 30
static   uint16_t MSP430TimerM$TimerB$read(void);

static   void MSP430TimerM$TimerB$clearOverflow(void);
#line 31
static   bool MSP430TimerM$TimerB$isOverflowPending(void);



static   void MSP430TimerM$TimerB$setMode(int arg_0x40684860);




static   void MSP430TimerM$TimerB$setInputDivider(uint16_t arg_0x40682b10);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB5$setEvent(uint16_t arg_0x40686010);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB0$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB0$default$captured(uint16_t arg_0x40695b78);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB6$setEvent(uint16_t arg_0x40686010);

static   void MSP430TimerM$CompareB0$setEventFromNow(uint16_t arg_0x40686958);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB6$getControl(void);







static   void MSP430TimerM$ControlB6$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB6$setControlAsCompare(void);



static   void MSP430TimerM$ControlB6$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB6$clearPendingInterrupt(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t PowerManagerM$StdControl$init(void);






static  result_t PowerManagerM$StdControl$start(void);
# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void ZigIrqM$Port14$fired(void);
#line 59
static   void ZigIrqM$Port26$fired(void);
#line 59
static   void ZigIrqM$Port17$fired(void);
#line 59
static   void ZigIrqM$Port21$fired(void);
#line 59
static   void ZigIrqM$Port12$fired(void);
#line 59
static   void ZigIrqM$Port24$fired(void);
#line 59
static   void ZigIrqM$Port15$fired(void);
#line 59
static   void ZigIrqM$Port27$fired(void);
#line 59
static   void ZigIrqM$Port10$fired(void);
#line 59
static   void ZigIrqM$Port22$fired(void);
#line 59
static   void ZigIrqM$Port13$fired(void);
#line 59
static   void ZigIrqM$Port25$fired(void);
#line 59
static   void ZigIrqM$Port16$fired(void);
#line 59
static   void ZigIrqM$Port20$fired(void);
#line 59
static   void ZigIrqM$Port11$fired(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ZigIrqM$StdControl$init(void);






static  result_t ZigIrqM$StdControl$start(void);
# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void ZigIrqM$Port23$fired(void);
#line 40
static   void MSP430InterruptM$Port14$clear(void);
#line 35
static   void MSP430InterruptM$Port14$disable(void);
#line 54
static   void MSP430InterruptM$Port14$edge(bool arg_0x407cd270);
#line 30
static   void MSP430InterruptM$Port14$enable(void);
#line 47
static   bool MSP430InterruptM$Port14$getValue(void);
#line 40
static   void MSP430InterruptM$ACCV$clear(void);
#line 59
static   void MSP430InterruptM$ACCV$default$fired(void);
#line 40
static   void MSP430InterruptM$Port15$clear(void);
#line 54
static   void MSP430InterruptM$Port15$edge(bool arg_0x407cd270);
#line 47
static   bool MSP430InterruptM$Port15$getValue(void);
#line 40
static   void MSP430InterruptM$Port10$clear(void);
#line 54
static   void MSP430InterruptM$Port10$edge(bool arg_0x407cd270);
#line 30
static   void MSP430InterruptM$Port10$enable(void);









static   void MSP430InterruptM$OF$clear(void);
#line 59
static   void MSP430InterruptM$OF$default$fired(void);
#line 40
static   void MSP430InterruptM$NMI$clear(void);
#line 59
static   void MSP430InterruptM$NMI$default$fired(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ZigSysM$StdControl$init(void);






static  result_t ZigSysM$StdControl$start(void);
#line 63
static  result_t TimerSymbol2M$IStdControl$init(void);






static  result_t TimerSymbol2M$IStdControl$start(void);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t TimerSymbol2M$TimerMilli$default$fired(
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x408cf3d0);
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t TimerSymbol2M$TimerMilli$setOneShot(
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x408cf3d0, 
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
int32_t arg_0x408c0088);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbol2M$AlarmCompare$fired(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime TimerSymbol2M$ILocalTime$Read(void);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void TimerSymbol2M$AlarmTimer$overflow(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t TimerSymbol2M$ITimerSymbol$Stop(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x408d07c0);
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t TimerSymbol2M$ITimerSymbol$SetPeriodic(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x408d07c0, 
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TSysTime arg_0x408aa5d8, TUniData arg_0x408aa760);









static  result_t TimerSymbol2M$ITimerSymbol$default$Fired(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x408d07c0, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TUniData arg_0x408c7120);
#line 11
static  bool TimerSymbol2M$ITimerSymbol$IsSet(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x408d07c0);

static   uint16_t TimerSymbol2M$overflowCount(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t TimerSymbolAsyncM$IStdControl$init(void);






static  result_t TimerSymbolAsyncM$IStdControl$start(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare1$fired(void);
#line 34
static   void TimerSymbolAsyncM$AlarmCompare4$fired(void);
#line 34
static   void TimerSymbolAsyncM$AlarmCompare2$fired(void);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void TimerSymbolAsyncM$ITimer$overflow(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare0$fired(void);
#line 34
static   void TimerSymbolAsyncM$AlarmCompare3$fired(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$default$Fired(
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
uint8_t arg_0x4091d3d8, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
TUniData arg_0x408c4dd8);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void TimerTuningM$ITimerB$overflow(void);
#line 33
static   void TimerTuningM$ITimerA$overflow(void);
# 29 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ClockInit.nc"
static  void TimerTuningM$IMSP430ClockInit$initTimerB(void);
#line 28
static  void TimerTuningM$IMSP430ClockInit$initTimerA(void);
#line 27
static  void TimerTuningM$IMSP430ClockInit$initClocks(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t LocalTime64M$ITimerSymbol$Fired(TUniData arg_0x408c7120);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  uint64_t LocalTime64M$ILocalTime64$getLocalTimeAt(TSysTime arg_0x408961b8);
static  void LocalTime64M$ILocalTime64$setLocalTimeAt(TSysTime arg_0x40896638, uint64_t arg_0x408967c0);

static  uint64_t LocalTime64M$ILocalTime64$getLocalTime(void);
static  void LocalTime64M$ILocalTime64$setLocalTime(uint64_t arg_0x40895010);
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   uint32_t TimeCastM$ITimeCast$SymbolsToJiffies(TSysTime arg_0x4089d868);
#line 10
static   TMilliSec TimeCastM$ITimeCast$SymbolsToMillis(TSysTime arg_0x4089f708);



static   uint32_t TimeCastM$ITimeCast$MillisToJiffies(TMilliSec arg_0x4089d3b0);
#line 7
static   TSysTime TimeCastM$ITimeCast$JiffiesToSymbols(uint32_t arg_0x408a1f18);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t ZigSTimerM$Timer0$fired(void);
#line 37
static  result_t ZigSTimerM$Timer1$fired(void);
#line 37
static  result_t ZigSTimerM$Timer2$fired(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void ZigTimerAIrqM$Compare1$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void ZigTimerAIrqM$Capture1$captured(uint16_t arg_0x40695b78);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void ZigTimerAIrqM$Compare2$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void ZigTimerAIrqM$Capture2$captured(uint16_t arg_0x40695b78);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void ZigTimerAIrqM$Compare0$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void ZigTimerAIrqM$Capture0$captured(uint16_t arg_0x40695b78);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void ZigTimerAIrqM$Timer$overflow(void);
# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void ZigSpi2AppM$INTR$fired(void);
#line 59
static   void ZigSpi2AppM$INTA_2$fired(void);
#line 59
static   void ZigSpi2AppM$INTJOIN$fired(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ZigSpi2AppM$StdControl$init(void);






static  result_t ZigSpi2AppM$StdControl$start(void);
# 73 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
static   result_t UniSART1M$USARTControl$disableRxIntr(void);
static   result_t UniSART1M$USARTControl$disableTxIntr(void);
#line 65
static   void UniSART1M$USARTControl$setModeSPI(bool arg_0x40a03898);
#line 81
static   result_t UniSART1M$USARTControl$isTxIntrPending(void);
#line 103
static   result_t UniSART1M$USARTControl$tx(uint8_t arg_0x409fd010);






static   uint8_t UniSART1M$USARTControl$rx(void);
#line 86
static   result_t UniSART1M$USARTControl$isRxIntrPending(void);
# 47 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MainM.nc"
static  result_t MainM$hardwareInit(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MainM$StdControl$init(void);






static  result_t MainM$StdControl$start(void);
# 52 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MainM.nc"
int main(void)   ;
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t HPLInitM$MSP430ClockControl$init(void);






static  result_t HPLInitM$MSP430ClockControl$start(void);
# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLInitM.nc"
static inline  result_t HPLInitM$init(void);
# 29 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ClockInit.nc"
static  void MSP430ClockM$MSP430ClockInit$initTimerB(void);
#line 28
static  void MSP430ClockM$MSP430ClockInit$initTimerA(void);
#line 27
static  void MSP430ClockM$MSP430ClockInit$initClocks(void);
# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
 static volatile uint16_t MSP430ClockM$TA0CTL __asm ("0x0160");
 static volatile uint16_t MSP430ClockM$TA0IV __asm ("0x012E");
 static volatile uint16_t MSP430ClockM$TBCTL __asm ("0x0180");
 static volatile uint16_t MSP430ClockM$TBIV __asm ("0x011E");

enum MSP430ClockM$__nesc_unnamed4341 {

  MSP430ClockM$ACLK_CALIB_PERIOD = 8, 
  MSP430ClockM$ACLK_KHZ = 32, 
  MSP430ClockM$TARGET_DCO_KHZ = 4096, 
  MSP430ClockM$TARGET_DCO_DELTA = MSP430ClockM$TARGET_DCO_KHZ / MSP430ClockM$ACLK_KHZ * MSP430ClockM$ACLK_CALIB_PERIOD
};
#line 115
static inline void MSP430ClockM$startTimerA(void);
#line 127
static inline void MSP430ClockM$startTimerB(void);
#line 139
static void MSP430ClockM$set_dco_calib(int calib);





static inline uint16_t MSP430ClockM$test_calib_busywait_delta(int calib);
#line 168
static inline void MSP430ClockM$busyCalibrateDCO(void);
#line 201
static inline  result_t MSP430ClockM$StdControl$init(void);
#line 220
static inline  result_t MSP430ClockM$StdControl$start(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   uint16_t MSP430DCOCalibM$Timer32khz$read(void);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430DCOCalibM.nc"
uint16_t MSP430DCOCalibM$m_prev;

enum MSP430DCOCalibM$__nesc_unnamed4342 {

  MSP430DCOCalibM$TARGET_DELTA = 2048, 
  MSP430DCOCalibM$MAX_DEVIATION = 7
};


static inline   void MSP430DCOCalibM$TimerMicro$overflow(void);
#line 75
static inline   void MSP430DCOCalibM$Timer32khz$overflow(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureA1$captured(uint16_t arg_0x40695b78);
#line 74
static   void MSP430TimerM$CaptureB3$captured(uint16_t arg_0x40695b78);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA1$fired(void);
#line 34
static   void MSP430TimerM$CompareB3$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureB6$captured(uint16_t arg_0x40695b78);
#line 74
static   void MSP430TimerM$CaptureB1$captured(uint16_t arg_0x40695b78);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB1$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureA2$captured(uint16_t arg_0x40695b78);
#line 74
static   void MSP430TimerM$CaptureB4$captured(uint16_t arg_0x40695b78);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA2$fired(void);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerA$overflow(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB4$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureA0$captured(uint16_t arg_0x40695b78);
#line 74
static   void MSP430TimerM$CaptureB2$captured(uint16_t arg_0x40695b78);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA0$fired(void);
#line 34
static   void MSP430TimerM$CompareB2$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureB5$captured(uint16_t arg_0x40695b78);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerB$overflow(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB5$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureB0$captured(uint16_t arg_0x40695b78);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB6$fired(void);
#line 34
static   void MSP430TimerM$CompareB0$fired(void);
# 67 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
 static volatile uint16_t MSP430TimerM$TA0CTL __asm ("0x0160");

 static volatile uint16_t MSP430TimerM$TA0CCTL0 __asm ("0x0162");
 static volatile uint16_t MSP430TimerM$TA0CCTL1 __asm ("0x0164");
 static volatile uint16_t MSP430TimerM$TA0CCTL2 __asm ("0x0166");

 static volatile uint16_t MSP430TimerM$TA0CCR0 __asm ("0x0172");
 static volatile uint16_t MSP430TimerM$TA0CCR1 __asm ("0x0174");
 static volatile uint16_t MSP430TimerM$TA0CCR2 __asm ("0x0176");

 static volatile uint16_t MSP430TimerM$TBCCTL0 __asm ("0x0182");
 static volatile uint16_t MSP430TimerM$TBCCTL1 __asm ("0x0184");
 static volatile uint16_t MSP430TimerM$TBCCTL2 __asm ("0x0186");
 static volatile uint16_t MSP430TimerM$TBCCTL3 __asm ("0x0188");
 static volatile uint16_t MSP430TimerM$TBCCTL4 __asm ("0x018A");
 static volatile uint16_t MSP430TimerM$TBCCTL5 __asm ("0x018C");
 static volatile uint16_t MSP430TimerM$TBCCTL6 __asm ("0x018E");

 static volatile uint16_t MSP430TimerM$TBCCR0 __asm ("0x0192");
 static volatile uint16_t MSP430TimerM$TBCCR1 __asm ("0x0194");
 static volatile uint16_t MSP430TimerM$TBCCR2 __asm ("0x0196");
 static volatile uint16_t MSP430TimerM$TBCCR3 __asm ("0x0198");
 static volatile uint16_t MSP430TimerM$TBCCR4 __asm ("0x019A");
 static volatile uint16_t MSP430TimerM$TBCCR5 __asm ("0x019C");
 static volatile uint16_t MSP430TimerM$TBCCR6 __asm ("0x019E");

typedef MSP430CompareControl_t MSP430TimerM$CC_t;

static inline uint16_t MSP430TimerM$CC2int(MSP430TimerM$CC_t x);
static inline MSP430TimerM$CC_t MSP430TimerM$int2CC(uint16_t x);

static uint16_t MSP430TimerM$compareControl(void);
#line 123
void sig_TIMERA0_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(12))) ;







void sig_TIMERA1_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(10))) ;
#line 166
static inline   uint16_t MSP430TimerM$TimerB$read(void);


static inline   bool MSP430TimerM$TimerB$isOverflowPending(void);

static inline   void MSP430TimerM$TimerA$clearOverflow(void);
static inline   void MSP430TimerM$TimerB$clearOverflow(void);

static inline   void MSP430TimerM$TimerA$setMode(int mode);
static inline   void MSP430TimerM$TimerB$setMode(int mode);






static inline   void MSP430TimerM$TimerA$disableEvents(void);


static inline   void MSP430TimerM$TimerA$setClockSource(uint16_t clockSource);




static inline   void MSP430TimerM$TimerB$setClockSource(uint16_t clockSource);









static inline   void MSP430TimerM$TimerB$setInputDivider(uint16_t inputDivider);




static inline   MSP430TimerM$CC_t MSP430TimerM$ControlA0$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlA1$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlA2$getControl(void);
#line 253
static inline   uint16_t MSP430TimerM$CaptureA0$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureA1$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureA2$getEvent(void);
#line 277
void sig_TIMERB0_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(26))) ;







void sig_TIMERB1_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(24))) ;
#line 332
static inline    void MSP430TimerM$CompareB1$default$fired(void);





static inline    void MSP430TimerM$CaptureB0$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB1$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB2$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB3$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB4$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB5$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB6$default$captured(uint16_t time);


static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB0$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB1$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB2$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB3$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB4$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB5$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB6$getControl(void);









static inline   void MSP430TimerM$ControlB0$clearPendingInterrupt(void);

static inline   void MSP430TimerM$ControlB2$clearPendingInterrupt(void);
static inline   void MSP430TimerM$ControlB3$clearPendingInterrupt(void);
static inline   void MSP430TimerM$ControlB4$clearPendingInterrupt(void);
static inline   void MSP430TimerM$ControlB5$clearPendingInterrupt(void);
static inline   void MSP430TimerM$ControlB6$clearPendingInterrupt(void);


static inline   void MSP430TimerM$ControlB1$setControl(MSP430TimerM$CC_t x);






static inline   void MSP430TimerM$ControlB0$setControlAsCompare(void);

static inline   void MSP430TimerM$ControlB2$setControlAsCompare(void);
static inline   void MSP430TimerM$ControlB3$setControlAsCompare(void);
static inline   void MSP430TimerM$ControlB4$setControlAsCompare(void);
static inline   void MSP430TimerM$ControlB5$setControlAsCompare(void);
static inline   void MSP430TimerM$ControlB6$setControlAsCompare(void);
#line 411
static inline   void MSP430TimerM$ControlB0$enableEvents(void);

static inline   void MSP430TimerM$ControlB2$enableEvents(void);
static inline   void MSP430TimerM$ControlB3$enableEvents(void);
static inline   void MSP430TimerM$ControlB4$enableEvents(void);
static inline   void MSP430TimerM$ControlB5$enableEvents(void);
static inline   void MSP430TimerM$ControlB6$enableEvents(void);

static inline   void MSP430TimerM$ControlB0$disableEvents(void);
static inline   void MSP430TimerM$ControlB1$disableEvents(void);
static inline   void MSP430TimerM$ControlB2$disableEvents(void);
static inline   void MSP430TimerM$ControlB3$disableEvents(void);
static inline   void MSP430TimerM$ControlB4$disableEvents(void);
static inline   void MSP430TimerM$ControlB5$disableEvents(void);
static inline   void MSP430TimerM$ControlB6$disableEvents(void);
#line 443
static inline   uint16_t MSP430TimerM$CaptureB0$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB1$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB2$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB3$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB4$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB5$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB6$getEvent(void);



static inline   void MSP430TimerM$CompareB2$setEvent(uint16_t x);
static inline   void MSP430TimerM$CompareB3$setEvent(uint16_t x);
static inline   void MSP430TimerM$CompareB4$setEvent(uint16_t x);
static inline   void MSP430TimerM$CompareB5$setEvent(uint16_t x);
static inline   void MSP430TimerM$CompareB6$setEvent(uint16_t x);









static inline   void MSP430TimerM$CompareB0$setEventFromNow(uint16_t x);
# 5 "../../../zigzag/PowerManagerM.nc"
static inline  result_t PowerManagerM$StdControl$init(void);
static inline  result_t PowerManagerM$StdControl$start(void);
# 30 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port10$fired(void);
static inline   void ZigIrqM$Port11$fired(void);
static inline   void ZigIrqM$Port12$fired(void);
static inline   void ZigIrqM$Port13$fired(void);
static inline   void ZigIrqM$Port14$fired(void);
static inline   void ZigIrqM$Port15$fired(void);
static inline   void ZigIrqM$Port16$fired(void);
static inline   void ZigIrqM$Port17$fired(void);

static inline   void ZigIrqM$Port20$fired(void);
static inline   void ZigIrqM$Port21$fired(void);
static inline   void ZigIrqM$Port22$fired(void);
static inline   void ZigIrqM$Port23$fired(void);
static inline   void ZigIrqM$Port24$fired(void);
static inline   void ZigIrqM$Port25$fired(void);
static inline   void ZigIrqM$Port26$fired(void);
static inline   void ZigIrqM$Port27$fired(void);

void sig_DACDMA_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(0))) ;
void sig_ADC12_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(14))) ;
void sig_COMPARATORA_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(22))) ;


void sig_UART0RX_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(18))) ;
void sig_UART0TX_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(16))) ;



void sig_UART1RX_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(6))) ;
void sig_UART1TX_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(4))) ;


static inline  result_t ZigIrqM$StdControl$init(void);
static inline  result_t ZigIrqM$StdControl$start(void);
# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void MSP430InterruptM$Port14$fired(void);
#line 59
static   void MSP430InterruptM$Port26$fired(void);
#line 59
static   void MSP430InterruptM$Port17$fired(void);
#line 59
static   void MSP430InterruptM$Port21$fired(void);
#line 59
static   void MSP430InterruptM$Port12$fired(void);
#line 59
static   void MSP430InterruptM$Port24$fired(void);
#line 59
static   void MSP430InterruptM$ACCV$fired(void);
#line 59
static   void MSP430InterruptM$Port15$fired(void);
#line 59
static   void MSP430InterruptM$Port27$fired(void);
#line 59
static   void MSP430InterruptM$Port10$fired(void);
#line 59
static   void MSP430InterruptM$Port22$fired(void);
#line 59
static   void MSP430InterruptM$OF$fired(void);
#line 59
static   void MSP430InterruptM$Port13$fired(void);
#line 59
static   void MSP430InterruptM$Port25$fired(void);
#line 59
static   void MSP430InterruptM$Port16$fired(void);
#line 59
static   void MSP430InterruptM$NMI$fired(void);
#line 59
static   void MSP430InterruptM$Port20$fired(void);
#line 59
static   void MSP430InterruptM$Port11$fired(void);
#line 59
static   void MSP430InterruptM$Port23$fired(void);
# 51 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
 static volatile uint8_t MSP430InterruptM$P1IE __asm ("0x0025");
 static volatile uint8_t MSP430InterruptM$P2IE __asm ("0x002D");
 static volatile uint8_t MSP430InterruptM$P1IFG __asm ("0x0023");
 static volatile uint8_t MSP430InterruptM$P2IFG __asm ("0x002B");

void sig_PORT1_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(8))) ;
#line 71
void sig_PORT2_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(2))) ;
#line 85
void sig_NMI_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(28))) ;
#line 111
static inline    void MSP430InterruptM$NMI$default$fired(void);
static inline    void MSP430InterruptM$OF$default$fired(void);
static inline    void MSP430InterruptM$ACCV$default$fired(void);

static inline   void MSP430InterruptM$Port10$enable(void);



static inline   void MSP430InterruptM$Port14$enable(void);
#line 150
static inline   void MSP430InterruptM$Port14$disable(void);
#line 177
static inline   void MSP430InterruptM$Port10$clear(void);



static inline   void MSP430InterruptM$Port14$clear(void);
static inline   void MSP430InterruptM$Port15$clear(void);
#line 195
static inline   void MSP430InterruptM$NMI$clear(void);
static inline   void MSP430InterruptM$OF$clear(void);
static inline   void MSP430InterruptM$ACCV$clear(void);





static inline   bool MSP430InterruptM$Port14$getValue(void);
static   bool MSP430InterruptM$Port15$getValue(void);
#line 221
static inline   void MSP430InterruptM$Port10$edge(bool l2h);
#line 245
static   void MSP430InterruptM$Port14$edge(bool l2h);





static   void MSP430InterruptM$Port15$edge(bool l2h);
# 8 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  uint64_t ZigSysM$ILocalTime64$getLocalTime(void);
static  void ZigSysM$ILocalTime64$setLocalTime(uint64_t arg_0x40895010);
# 9 "../../../zigzag/ZigSysM.nc"
static const uint8_t ZigSysM$en_mask[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

int16_t __port_perm(const uint8_t port_num, const uint8_t mask, const uint8_t op)   ;










uint64_t __sys_time(void)   ;




int16_t __set_sys_time(uint64_t time)   ;






static inline  void ZigSysM$task_app_init(void);






bool TOS_post(void (*tp)(void))   ;
int16_t __post_task(task_ft task_func)   ;






static volatile __nesc_atomic_t ZigSysM$reenable_interrupt;
void __critical_enter(void)   ;





void __critical_exit(void)   ;





static inline  result_t ZigSysM$StdControl$init(void);
static inline  result_t ZigSysM$StdControl$start(void);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t TimerSymbol2M$TimerMilli$fired(
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x408cf3d0);
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   uint32_t TimerSymbol2M$ITimeCast$SymbolsToJiffies(TSysTime arg_0x4089d868);
#line 14
static   uint32_t TimerSymbol2M$ITimeCast$MillisToJiffies(TMilliSec arg_0x4089d3b0);
#line 7
static   TSysTime TimerSymbol2M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x408a1f18);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbol2M$AlarmControl$enableEvents(void);
#line 35
static   void TimerSymbol2M$AlarmControl$setControlAsCompare(void);



static   void TimerSymbol2M$AlarmControl$disableEvents(void);
#line 32
static   void TimerSymbol2M$AlarmControl$clearPendingInterrupt(void);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbol2M$AlarmCompare$setEventFromNow(uint16_t arg_0x40686958);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   uint16_t TimerSymbol2M$AlarmTimer$read(void);
static   bool TimerSymbol2M$AlarmTimer$isOverflowPending(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t TimerSymbol2M$ITimerSymbol$Fired(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x408d07c0, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TUniData arg_0x408c7120);
# 21 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
enum TimerSymbol2M$__nesc_unnamed4343 {
  TimerSymbol2M$NUM_SYMBOL_TIMERS = 1U, 
  TimerSymbol2M$NUM_MILLIS_TIMERS = 3U, 
  TimerSymbol2M$NUM_TIMERS = TimerSymbol2M$NUM_SYMBOL_TIMERS + TimerSymbol2M$NUM_MILLIS_TIMERS, 
  TimerSymbol2M$EMPTY_LIST = 255, 
  TimerSymbol2M$MAX_HW_COUNTER = 0xffffL, 
  TimerSymbol2M$MIN_HW_COUNTER = 2
};

typedef uint32_t TimerSymbol2M$TJiffy;










#line 32
typedef struct TimerSymbol2M$__nesc_unnamed4344 {
  TUniData uniData;
  TimerSymbol2M$TJiffy wakeUpTime;
  TimerSymbol2M$TJiffy period;
  uint8_t next;
  unsigned isSet : 1;
  unsigned isPeriodic : 1;
  unsigned isQueued : 1;
  unsigned isOverflow : 1;
} TimerSymbol2M$TSyncTimer;

TimerSymbol2M$TSyncTimer TimerSymbol2M$timers[TimerSymbol2M$NUM_TIMERS];
uint8_t TimerSymbol2M$shortTimersHead;
uint8_t TimerSymbol2M$longTimersHead;
bool TimerSymbol2M$m_posted_checkShortTimers;


uint16_t hiLocalTime  ;

static inline  result_t TimerSymbol2M$IStdControl$init(void);










static inline  result_t TimerSymbol2M$IStdControl$start(void);


static inline   uint16_t TimerSymbol2M$overflowCount(void);








static TimerSymbol2M$TJiffy TimerSymbol2M$ReadLocalTime(void);
#line 88
static void TimerSymbol2M$InsertTimer(uint8_t num, bool isShort);
#line 103
static inline void TimerSymbol2M$RemoveTimer(uint8_t num);




static inline void TimerSymbol2M$SignalTimerFired(uint8_t num);








static void TimerSymbol2M$SetWakeUpTime(TimerSymbol2M$TSyncTimer *timer, TimerSymbol2M$TJiffy base, TimerSymbol2M$TJiffy delta);
#line 129
static TimerSymbol2M$TJiffy TimerSymbol2M$GetWakeUpDelta(TimerSymbol2M$TSyncTimer *timer, TimerSymbol2M$TJiffy localTime);
#line 143
static void TimerSymbol2M$ExecuteTimers(uint8_t head);
#line 187
static inline  void TimerSymbol2M$CheckShortTimers(void);

static void TimerSymbol2M$Post_checkShortTimers(void);









static void TimerSymbol2M$SetNextShortEvent(void);
#line 237
static inline  void TimerSymbol2M$CheckShortTimers(void);








static void TimerSymbol2M$CheckLongTimers(void);






static inline  void TimerSymbol2M$TaskCheckLTimers(void);




static inline void TimerSymbol2M$ClearOverflow(void);






static inline  void TimerSymbol2M$TaskCheckAndClearOverflowLTimers(void);





static inline   void TimerSymbol2M$AlarmCompare$fired(void);




static inline   void TimerSymbol2M$AlarmTimer$overflow(void);










static result_t TimerSymbol2M$SetTimer(uint8_t num, TimerSymbol2M$TJiffy jiffy, bool isPeriodic, TUniData uniData);
#line 306
static inline   TSysTime TimerSymbol2M$ILocalTime$Read(void);






static inline  result_t TimerSymbol2M$ITimerSymbol$SetPeriodic(uint8_t num, TSysTime symbols, TUniData uniData);









static inline  result_t TimerSymbol2M$ITimerSymbol$Stop(uint8_t num);





static inline  bool TimerSymbol2M$ITimerSymbol$IsSet(uint8_t num);
#line 349
static inline   result_t TimerSymbol2M$ITimerSymbol$default$Fired(uint8_t num, TUniData uniData);



static inline uint8_t TimerSymbol2M$fromNumMilli(uint8_t num);
#line 368
static inline  result_t TimerSymbol2M$TimerMilli$setOneShot(uint8_t num, int32_t millis);
#line 399
static inline   result_t TimerSymbol2M$TimerMilli$default$fired(uint8_t num);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl2$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl2$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl2$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl2$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare1$setEvent(uint16_t arg_0x40686010);
#line 30
static   void TimerSymbolAsyncM$AlarmCompare4$setEvent(uint16_t arg_0x40686010);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl3$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl3$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl3$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl3$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare2$setEvent(uint16_t arg_0x40686010);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl0$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl0$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl0$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl0$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare0$setEvent(uint16_t arg_0x40686010);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl4$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl4$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl4$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl4$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare3$setEvent(uint16_t arg_0x40686010);
# 28 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static   uint16_t TimerSymbolAsyncM$overflowCount(void);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl1$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl1$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl1$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl1$clearPendingInterrupt(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$Fired(
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
uint8_t arg_0x4091d3d8, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
TUniData arg_0x408c4dd8);
# 33 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
enum TimerSymbolAsyncM$__nesc_unnamed4345 {
  TimerSymbolAsyncM$ASYNC_TIMER_NUM = 5, 
  TimerSymbolAsyncM$MAX_HW_COUNTER = 0xFFFF, 
  TimerSymbolAsyncM$MIN_HW_COUNTER = 2, 
  TimerSymbolAsyncM$WARNING_INTERVAL = 10
};








#line 40
struct TimerSymbolAsyncM$__nesc_unnamed4346 {
  TUniData uniData;
  TSysTime shotTime;
  TSysTime period;
  TSysTime remain;
  unsigned isSet : 1;
  unsigned isPeriodic : 1;
} TimerSymbolAsyncM$timer[TimerSymbolAsyncM$ASYNC_TIMER_NUM];


static inline void TimerSymbolAsyncM$SetControlAsCompare(const uint8_t timerNumber);
static void TimerSymbolAsyncM$DisableEvents(const uint8_t timerNumber);
static void TimerSymbolAsyncM$EnableEvents(const uint8_t timerNumber);
static void TimerSymbolAsyncM$ClearPendingInterrupt(const uint8_t timerNumber);
static inline void TimerSymbolAsyncM$SetEvent(const uint8_t timerNumber, const uint16_t jiffy);
#line 66
static inline   void TimerSymbolAsyncM$ITimer$overflow(void);
#line 86
static inline void TimerSymbolAsyncM$SetTimerAt(const uint8_t timerNumber, const uint32_t time);
#line 104
static void TimerSymbolAsyncM$HandleAlarmFire(const uint8_t timerNumber);
#line 244
static inline  result_t TimerSymbolAsyncM$IStdControl$init(void);









static inline  result_t TimerSymbolAsyncM$IStdControl$start(void);
#line 274
static inline void TimerSymbolAsyncM$SetControlAsCompare(const uint8_t timerNumber);










static void TimerSymbolAsyncM$DisableEvents(const uint8_t timerNumber);










static void TimerSymbolAsyncM$EnableEvents(const uint8_t timerNumber);










static void TimerSymbolAsyncM$ClearPendingInterrupt(const uint8_t timerNumber);










static inline void TimerSymbolAsyncM$SetEvent(const uint8_t timerNumber, const uint16_t jiffy);
#line 340
static inline   void TimerSymbolAsyncM$AlarmCompare0$fired(void);
static inline   void TimerSymbolAsyncM$AlarmCompare1$fired(void);
static inline   void TimerSymbolAsyncM$AlarmCompare2$fired(void);
static inline   void TimerSymbolAsyncM$AlarmCompare3$fired(void);
static inline   void TimerSymbolAsyncM$AlarmCompare4$fired(void);


static inline    
#line 346
result_t TimerSymbolAsyncM$ITimerSymbolAsync$default$Fired(uint8_t timerNumber, 
TUniData uniData);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerTuningM$ITimerB2Control$disableEvents(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void TimerTuningM$ITimerB$setClockSource(uint16_t arg_0x40682668);
#line 32
static   void TimerTuningM$ITimerB$clearOverflow(void);


static   void TimerTuningM$ITimerB$setMode(int arg_0x40684860);




static   void TimerTuningM$ITimerB$setInputDivider(uint16_t arg_0x40682b10);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerTuningM$ITimerB4Control$disableEvents(void);
#line 39
static   void TimerTuningM$ITimerB5Control$disableEvents(void);
#line 39
static   void TimerTuningM$ITimerB6Control$disableEvents(void);
#line 39
static   void TimerTuningM$ITimerB1Control$disableEvents(void);
#line 34
static   void TimerTuningM$ITimerB1Control$setControl(MSP430CompareControl_t arg_0x4068c990);




static   void TimerTuningM$ITimerB3Control$disableEvents(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void TimerTuningM$ITimerA$setClockSource(uint16_t arg_0x40682668);
#line 38
static   void TimerTuningM$ITimerA$disableEvents(void);
#line 32
static   void TimerTuningM$ITimerA$clearOverflow(void);


static   void TimerTuningM$ITimerA$setMode(int arg_0x40684860);
# 21 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerTuningM.nc"
static inline  void TimerTuningM$IMSP430ClockInit$initClocks(void);
#line 43
static inline  void TimerTuningM$IMSP430ClockInit$initTimerA(void);









typedef MSP430CompareControl_t TimerTuningM$CC_t;
static inline uint16_t TimerTuningM$CC2uint(TimerTuningM$CC_t x);

static inline  void TimerTuningM$IMSP430ClockInit$initTimerB(void);
#line 96
static inline   void TimerTuningM$ITimerA$overflow(void);
static inline   void TimerTuningM$ITimerB$overflow(void);
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   TMilliSec LocalTime64M$ITimeCast$SymbolsToMillis(TSysTime arg_0x4089f708);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime LocalTime64M$ILocalTime$Read(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t LocalTime64M$ITimerSymbol$Stop(void);
#line 6
static  result_t LocalTime64M$ITimerSymbol$SetPeriodic(TSysTime arg_0x408aa5d8, TUniData arg_0x408aa760);




static  bool LocalTime64M$ITimerSymbol$IsSet(void);
# 17 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
uint64_t LocalTime64M$baseTime = 0;
TSysTime LocalTime64M$firedCount;

static inline  uint64_t LocalTime64M$ILocalTime64$getLocalTimeAt(TSysTime at);









static  uint64_t LocalTime64M$ILocalTime64$getLocalTime(void);




static inline  void LocalTime64M$ILocalTime64$setLocalTimeAt(TSysTime at, uint64_t time);
#line 47
static  void LocalTime64M$ILocalTime64$setLocalTime(uint64_t time);




static inline  result_t LocalTime64M$ITimerSymbol$Fired(TUniData ud);
# 16 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   TSysTime TimeCastM$ITimeCast$JiffiesToSymbols(uint32_t jiffies);








static inline   TMilliSec TimeCastM$ITimeCast$SymbolsToMillis(TSysTime symbols);
#line 38
static inline   uint32_t TimeCastM$ITimeCast$MillisToJiffies(TMilliSec millis);



static inline   uint32_t TimeCastM$ITimeCast$SymbolsToJiffies(TSysTime symbols);
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t ZigSTimerM$Timer0$setOneShot(int32_t arg_0x408c0088);
#line 28
static  result_t ZigSTimerM$Timer1$setOneShot(int32_t arg_0x408c0088);
#line 28
static  result_t ZigSTimerM$Timer2$setOneShot(int32_t arg_0x408c0088);
# 11 "../../../zigzag/ZigSTimerM.nc"
int16_t __stimer_set(const uint8_t timer_num, const uint32_t milli_sec)   ;
#line 26
static inline  result_t ZigSTimerM$Timer0$fired(void);






static inline  result_t ZigSTimerM$Timer1$fired(void);






static inline  result_t ZigSTimerM$Timer2$fired(void);
# 17 "../../../zigzag/ZigTimerAIrqM.nc"
static inline   void ZigTimerAIrqM$Timer$overflow(void);
static inline   void ZigTimerAIrqM$Compare0$fired(void);
static inline   void ZigTimerAIrqM$Compare1$fired(void);
static inline   void ZigTimerAIrqM$Compare2$fired(void);
static inline   void ZigTimerAIrqM$Capture0$captured(uint16_t time);
static inline   void ZigTimerAIrqM$Capture1$captured(uint16_t time);
static inline   void ZigTimerAIrqM$Capture2$captured(uint16_t time);
# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void ZigSpi2AppM$INTR$clear(void);
#line 54
static   void ZigSpi2AppM$INTR$edge(bool arg_0x407cd270);
#line 30
static   void ZigSpi2AppM$INTR$enable(void);









static   void ZigSpi2AppM$INTA_2$clear(void);
#line 35
static   void ZigSpi2AppM$INTA_2$disable(void);
#line 54
static   void ZigSpi2AppM$INTA_2$edge(bool arg_0x407cd270);
#line 30
static   void ZigSpi2AppM$INTA_2$enable(void);
#line 47
static   bool ZigSpi2AppM$INTA_2$getValue(void);
# 73 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
static   result_t ZigSpi2AppM$USARTControl$disableRxIntr(void);
static   result_t ZigSpi2AppM$USARTControl$disableTxIntr(void);
#line 65
static   void ZigSpi2AppM$USARTControl$setModeSPI(bool arg_0x40a03898);
#line 81
static   result_t ZigSpi2AppM$USARTControl$isTxIntrPending(void);
#line 103
static   result_t ZigSpi2AppM$USARTControl$tx(uint8_t arg_0x409fd010);






static   uint8_t ZigSpi2AppM$USARTControl$rx(void);
#line 86
static   result_t ZigSpi2AppM$USARTControl$isRxIntrPending(void);
# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void ZigSpi2AppM$INTJOIN$clear(void);
#line 54
static   void ZigSpi2AppM$INTJOIN$edge(bool arg_0x407cd270);
#line 47
static   bool ZigSpi2AppM$INTJOIN$getValue(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  void ZigSpi2AppM$ILocalTime64$setLocalTime(uint64_t arg_0x40895010);
# 17 "../../../zigzag/ZigSpi2AppM.nc"
size_t deserialize(const void *src, size_t size, const char *fmt, ...)   ;
size_t serialize(void *dst, size_t max_size, const char *fmt, ...)   ;

 bool ZigSpi2AppM$spi_busy = FALSE;







 
#line 22
static struct ZigSpi2AppM$__nesc_unnamed4347 {
  uint16_t dst_addr;
  uint16_t data_size;
  uint8_t *data;
  uint8_t handle;
  bool serv;
} ZigSpi2AppM$tx;

static void ZigSpi2AppM$tx_request(void);
#line 64
 
#line 59
static struct ZigSpi2AppM$__nesc_unnamed4348 {
  uint16_t src_addr;
  uint64_t ext_addr;
  uint16_t data_size;
  uint8_t data[MAC_AMAX_MAC_FRAME_SIZE - NWK_MIN_HEADER_OVERHEAD];
} ZigSpi2AppM$rx;

static inline  void ZigSpi2AppM$loop_tx(void);









int16_t __net_send(const uint16_t dst_addr, const uint16_t data_size, 
const uint8_t *const data, uint8_t handle)   ;
#line 111
static uint8_t ZigSpi2AppM$set_time_msg[12];
volatile uint64_t *ZigSpi2AppM$__workaround;
int16_t __set_sys_time2(uint64_t time)   ;
#line 134
static inline  void ZigSpi2AppM$task_tx_done(void);







static __inline void ZigSpi2AppM$tx_data(void);
#line 183
static inline   void ZigSpi2AppM$INTA_2$fired(void);








static inline  void ZigSpi2AppM$task_net_enter(void);






static inline  void ZigSpi2AppM$task_net_exit(void);






static void ZigSpi2AppM$process_join(void);
#line 220
static inline   void ZigSpi2AppM$INTJOIN$fired(void);








static inline  void ZigSpi2AppM$task_rx_done(void);
#line 269
static inline   void ZigSpi2AppM$INTR$fired(void);
#line 332
uint16_t __net_addr(void)   ;




static inline  result_t ZigSpi2AppM$StdControl$init(void);




static inline  result_t ZigSpi2AppM$StdControl$start(void);
# 45 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
 static volatile uint8_t UniSART1M$ME2 __asm ("0x0005");
 static volatile uint8_t UniSART1M$IFG2 __asm ("0x0003");
 static volatile uint8_t UniSART1M$U1TCTL __asm ("0x0079");
 static volatile uint8_t UniSART1M$U1TXBUF __asm ("0x007F");

uint16_t UniSART1M$l_br;

uint8_t UniSART1M$l_ssel;
#line 77
static inline   void UniSART1M$USARTControl$setModeSPI(bool master);
#line 145
static   result_t UniSART1M$USARTControl$isTxIntrPending(void);
#line 160
static   result_t UniSART1M$USARTControl$isRxIntrPending(void);







static inline   result_t UniSART1M$USARTControl$disableRxIntr(void);




static inline   result_t UniSART1M$USARTControl$disableTxIntr(void);
#line 194
static inline   result_t UniSART1M$USARTControl$tx(uint8_t data);






static inline   uint8_t UniSART1M$USARTControl$rx(void);
# 8 "../../../zigzag/SerializeM.nc"
static void SerializeM$memcopy(void *const dst, const void *const src, const uint8_t len);






size_t serialize(void *dst, size_t max_size, const char *fmt, ...)   ;
#line 56
size_t deserialize(const void *src, size_t size, const char *fmt, ...)   ;
# 127 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
static inline void MSP430ClockM$startTimerB(void)
{

  MSP430ClockM$TBCTL = 0x0020 | (MSP430ClockM$TBCTL & ~(0x0020 | 0x0010));
}

#line 115
static inline void MSP430ClockM$startTimerA(void)
{

  MSP430ClockM$TA0CTL = 0x0020 | (MSP430ClockM$TA0CTL & ~(0x0020 | 0x0010));
}

#line 220
static inline  result_t MSP430ClockM$StdControl$start(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      MSP430ClockM$startTimerA();
      MSP430ClockM$startTimerB();
    }
#line 226
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t HPLInitM$MSP430ClockControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = MSP430ClockM$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 54 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerTuningM.nc"
static inline uint16_t TimerTuningM$CC2uint(TimerTuningM$CC_t x)
#line 54
{
#line 54
  union __nesc_unnamed4349 {
#line 54
    TimerTuningM$CC_t f;
#line 54
    uint16_t t;
  } 
#line 54
  c = { .f = x };

#line 54
  return c.t;
}

# 425 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB6$disableEvents(void)
#line 425
{
#line 425
  MSP430TimerM$TBCCTL6 &= ~0x0010;
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerTuningM$ITimerB6Control$disableEvents(void){
#line 39
  MSP430TimerM$ControlB6$disableEvents();
#line 39
}
#line 39
# 424 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB5$disableEvents(void)
#line 424
{
#line 424
  MSP430TimerM$TBCCTL5 &= ~0x0010;
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerTuningM$ITimerB5Control$disableEvents(void){
#line 39
  MSP430TimerM$ControlB5$disableEvents();
#line 39
}
#line 39
# 423 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB4$disableEvents(void)
#line 423
{
#line 423
  MSP430TimerM$TBCCTL4 &= ~0x0010;
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerTuningM$ITimerB4Control$disableEvents(void){
#line 39
  MSP430TimerM$ControlB4$disableEvents();
#line 39
}
#line 39
# 422 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB3$disableEvents(void)
#line 422
{
#line 422
  MSP430TimerM$TBCCTL3 &= ~0x0010;
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerTuningM$ITimerB3Control$disableEvents(void){
#line 39
  MSP430TimerM$ControlB3$disableEvents();
#line 39
}
#line 39
# 421 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB2$disableEvents(void)
#line 421
{
#line 421
  MSP430TimerM$TBCCTL2 &= ~0x0010;
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerTuningM$ITimerB2Control$disableEvents(void){
#line 39
  MSP430TimerM$ControlB2$disableEvents();
#line 39
}
#line 39
# 95 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline uint16_t MSP430TimerM$CC2int(MSP430TimerM$CC_t x)
#line 95
{
#line 95
  union __nesc_unnamed4350 {
#line 95
    MSP430TimerM$CC_t f;
#line 95
    uint16_t t;
  } 
#line 95
  c = { .f = x };

#line 95
  return c.t;
}

#line 372
static inline   void MSP430TimerM$ControlB1$setControl(MSP430TimerM$CC_t x)
#line 372
{
#line 372
  MSP430TimerM$TBCCTL1 = MSP430TimerM$CC2int(x);
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerTuningM$ITimerB1Control$setControl(MSP430CompareControl_t arg_0x4068c990){
#line 34
  MSP430TimerM$ControlB1$setControl(arg_0x4068c990);
#line 34
}
#line 34
# 420 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB1$disableEvents(void)
#line 420
{
#line 420
  MSP430TimerM$TBCCTL1 &= ~0x0010;
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerTuningM$ITimerB1Control$disableEvents(void){
#line 39
  MSP430TimerM$ControlB1$disableEvents();
#line 39
}
#line 39
# 175 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerB$setMode(int mode)
#line 175
{
#line 175
  TBCTL = (TBCTL & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerB$setMode(int arg_0x40684860){
#line 35
  MSP430TimerM$TimerB$setMode(arg_0x40684860);
#line 35
}
#line 35
# 200 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerB$setInputDivider(uint16_t inputDivider)
{
  TBCTL = (TBCTL & ~((1 << 6) | (3 << 6))) | ((inputDivider << 8) & ((1 << 6) | (3 << 6)));
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerB$setInputDivider(uint16_t arg_0x40682b10){
#line 40
  MSP430TimerM$TimerB$setInputDivider(arg_0x40682b10);
#line 40
}
#line 40
# 190 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerB$setClockSource(uint16_t clockSource)
{
  TBCTL = (TBCTL & ~(0x0100 | 0x0200)) | ((clockSource << 8) & (0x0100 | 0x0200));
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerB$setClockSource(uint16_t arg_0x40682668){
#line 39
  MSP430TimerM$TimerB$setClockSource(arg_0x40682668);
#line 39
}
#line 39
# 56 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerTuningM.nc"
static inline  void TimerTuningM$IMSP430ClockInit$initTimerB(void)
{
  TimerTuningM$CC_t cfgCapture = { .cm = 3, .ccis = 1, .scs = 1, .clld = 0, .cap = 1, .ccie = 1 };
  TimerTuningM$CC_t cfgCompare = { .clld = 0, .cap = 0, .ccie = 0 };


  TimerTuningM$ITimerB$setClockSource(0x1);
  TimerTuningM$ITimerB$setInputDivider(0x0);
  TBR = 0xffff;
  TimerTuningM$ITimerB$setMode(0x2);


  TBCCTL0 = TimerTuningM$CC2uint(cfgCompare);

  TimerTuningM$ITimerB1Control$disableEvents();
  TimerTuningM$ITimerB1Control$setControl(cfgCapture);

  TimerTuningM$ITimerB2Control$disableEvents();
  TBCCTL2 = TimerTuningM$CC2uint(cfgCompare);

  TimerTuningM$ITimerB3Control$disableEvents();
  TBCCTL3 = TimerTuningM$CC2uint(cfgCompare);

  TimerTuningM$ITimerB4Control$disableEvents();
  TBCCTL4 = TimerTuningM$CC2uint(cfgCompare);

  TimerTuningM$ITimerB5Control$disableEvents();
  TBCCTL5 = TimerTuningM$CC2uint(cfgCompare);

  TimerTuningM$ITimerB6Control$disableEvents();
  TBCCTL6 = TimerTuningM$CC2uint(cfgCompare);

  TBCTL |= 0x0002;
}

# 29 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ClockInit.nc"
inline static  void MSP430ClockM$MSP430ClockInit$initTimerB(void){
#line 29
  TimerTuningM$IMSP430ClockInit$initTimerB();
#line 29
}
#line 29
# 185 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerA$setClockSource(uint16_t clockSource)
{
  MSP430TimerM$TA0CTL = (MSP430TimerM$TA0CTL & ~(0x0100 | 0x0200)) | ((clockSource << 8) & (0x0100 | 0x0200));
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerA$setClockSource(uint16_t arg_0x40682668){
#line 39
  MSP430TimerM$TimerA$setClockSource(arg_0x40682668);
#line 39
}
#line 39
# 174 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerA$setMode(int mode)
#line 174
{
#line 174
  MSP430TimerM$TA0CTL = (MSP430TimerM$TA0CTL & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerA$setMode(int arg_0x40684860){
#line 35
  MSP430TimerM$TimerA$setMode(arg_0x40684860);
#line 35
}
#line 35
# 182 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerA$disableEvents(void)
#line 182
{
#line 182
  MSP430TimerM$TA0CTL &= ~0x0002;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerA$disableEvents(void){
#line 38
  MSP430TimerM$TimerA$disableEvents();
#line 38
}
#line 38
# 43 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerTuningM.nc"
static inline  void TimerTuningM$IMSP430ClockInit$initTimerA(void)
{



  TimerTuningM$ITimerA$disableEvents();
  TimerTuningM$ITimerA$setMode(0x0);
  TimerTuningM$ITimerA$setClockSource(0x0);
}

# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ClockInit.nc"
inline static  void MSP430ClockM$MSP430ClockInit$initTimerA(void){
#line 28
  TimerTuningM$IMSP430ClockInit$initTimerA();
#line 28
}
#line 28
# 21 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerTuningM.nc"
static inline  void TimerTuningM$IMSP430ClockInit$initClocks(void)
{
#line 35
  DCOCTL = 0x40;
  BCSCTL1 = 0x80 | ((0x04 | 0x02) | 0x01);
  BCSCTL2 = 0x01;

  IE1 &= ~(1 << 1);
}

# 27 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ClockInit.nc"
inline static  void MSP430ClockM$MSP430ClockInit$initClocks(void){
#line 27
  TimerTuningM$IMSP430ClockInit$initClocks();
#line 27
}
#line 27
# 145 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
static inline uint16_t MSP430ClockM$test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  MSP430ClockM$set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + MSP430ClockM$ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void MSP430ClockM$busyCalibrateDCO(void)
{

  int calib;
  int step;



  MSP430ClockM$TA0CTL = 0x0200 | 0x0020;
  MSP430ClockM$TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (MSP430ClockM$test_calib_busywait_delta(calib | step) <= MSP430ClockM$TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  MSP430ClockM$set_dco_calib(calib);
}

static inline  result_t MSP430ClockM$StdControl$init(void)
{

  MSP430ClockM$TA0CTL = 0x0004;
  MSP430ClockM$TA0IV = 0;
  MSP430ClockM$TBCTL = 0x0004;
  MSP430ClockM$TBIV = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      MSP430ClockM$busyCalibrateDCO();
      MSP430ClockM$MSP430ClockInit$initClocks();
      MSP430ClockM$MSP430ClockInit$initTimerA();
      MSP430ClockM$MSP430ClockInit$initTimerB();
    }
#line 215
    __nesc_atomic_end(__nesc_atomic); }

  return SUCCESS;
}

# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t HPLInitM$MSP430ClockControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = MSP430ClockM$StdControl$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 66 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_app/hardware.h"
static inline void TOSH_SET_PIN_DIRECTIONS(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 68
    {
      P1IE = 0x00;
#line 69
      P1OUT = 0x42;
#line 69
      P1SEL = 0x00;
#line 69
      P1DIR = 0x4e;
#line 69
      P1IFG = 0x00;
      P2IE = 0x00;
#line 70
      P2OUT = 0x00;
#line 70
      P2SEL = 0x00;
#line 70
      P2DIR = 0xd0;
#line 70
      P2IFG = 0x00;
      P4OUT = 0x10;
#line 71
      P4SEL = 0x00;
#line 71
      P4DIR = 0x18;
      P5OUT = 0x80;
#line 72
      P5DIR = 0x84;
#line 72
      P5SEL = 0x0e;
      P6OUT = 0x00;
#line 73
      P6SEL = 0x00;
#line 73
      P6DIR = 0xa0;
    }
#line 74
    __nesc_atomic_end(__nesc_atomic); }
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLInitM.nc"
static inline  result_t HPLInitM$init(void)
{
  TOSH_SET_PIN_DIRECTIONS();
  HPLInitM$MSP430ClockControl$init();
  HPLInitM$MSP430ClockControl$start();
  return SUCCESS;
}

# 47 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MainM.nc"
inline static  result_t MainM$hardwareInit(void){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLInitM$init();
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 79 "/home/max/tinyos/tinyos-1.x/tos/system/sched.c"
static inline void TOSH_sched_init(void )
{
  int i;

#line 82
  TOSH_sched_free = 0;
  TOSH_sched_full = 0;
  for (i = 0; i < TOSH_MAX_TASKS; i++) 
    TOSH_queue[i].tp = NULL;
}

# 120 "/home/max/tinyos/tinyos-1.x/tos/system/tos.h"
static inline result_t rcombine(result_t r1, result_t r2)



{
  return r1 == FAIL ? FAIL : r2;
}

# 62 "../../../zigzag/ZigSysM.nc"
static inline  result_t ZigSysM$StdControl$init(void)
#line 62
{
#line 62
  return SUCCESS;
}

# 62 "../../../zigzag/ZigIrqM.nc"
static inline  result_t ZigIrqM$StdControl$init(void)
#line 62
{
#line 62
  return SUCCESS;
}

# 337 "../../../zigzag/ZigSpi2AppM.nc"
static inline  result_t ZigSpi2AppM$StdControl$init(void)
{
  return SUCCESS;
}

# 385 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB6$setControlAsCompare(void)
#line 385
{
#line 385
  MSP430TimerM$TBCCTL6 = MSP430TimerM$compareControl();
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl4$setControlAsCompare(void){
#line 35
  MSP430TimerM$ControlB6$setControlAsCompare();
#line 35
}
#line 35
# 384 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB5$setControlAsCompare(void)
#line 384
{
#line 384
  MSP430TimerM$TBCCTL5 = MSP430TimerM$compareControl();
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl3$setControlAsCompare(void){
#line 35
  MSP430TimerM$ControlB5$setControlAsCompare();
#line 35
}
#line 35
# 383 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB4$setControlAsCompare(void)
#line 383
{
#line 383
  MSP430TimerM$TBCCTL4 = MSP430TimerM$compareControl();
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl2$setControlAsCompare(void){
#line 35
  MSP430TimerM$ControlB4$setControlAsCompare();
#line 35
}
#line 35
# 382 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB3$setControlAsCompare(void)
#line 382
{
#line 382
  MSP430TimerM$TBCCTL3 = MSP430TimerM$compareControl();
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl1$setControlAsCompare(void){
#line 35
  MSP430TimerM$ControlB3$setControlAsCompare();
#line 35
}
#line 35
# 381 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB2$setControlAsCompare(void)
#line 381
{
#line 381
  MSP430TimerM$TBCCTL2 = MSP430TimerM$compareControl();
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl0$setControlAsCompare(void){
#line 35
  MSP430TimerM$ControlB2$setControlAsCompare();
#line 35
}
#line 35
# 274 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline void TimerSymbolAsyncM$SetControlAsCompare(const uint8_t timerNumber)
{
  switch (timerNumber) {
      case 0: TimerSymbolAsyncM$AlarmControl0$setControlAsCompare();
#line 277
      return;
      case 1: TimerSymbolAsyncM$AlarmControl1$setControlAsCompare();
#line 278
      return;
      case 2: TimerSymbolAsyncM$AlarmControl2$setControlAsCompare();
#line 279
      return;
      case 3: TimerSymbolAsyncM$AlarmControl3$setControlAsCompare();
#line 280
      return;
      case 4: TimerSymbolAsyncM$AlarmControl4$setControlAsCompare();
#line 281
      return;
    }
}

#line 244
static inline  result_t TimerSymbolAsyncM$IStdControl$init(void)
{
  uint8_t i;

#line 247
  for (i = 0; i < TimerSymbolAsyncM$ASYNC_TIMER_NUM; i++) {
      TimerSymbolAsyncM$SetControlAsCompare(i);
      TimerSymbolAsyncM$DisableEvents(i);
    }
  return SUCCESS;
}

# 419 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB0$disableEvents(void)
#line 419
{
#line 419
  MSP430TimerM$TBCCTL0 &= ~0x0010;
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbol2M$AlarmControl$disableEvents(void){
#line 39
  MSP430TimerM$ControlB0$disableEvents();
#line 39
}
#line 39
# 379 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB0$setControlAsCompare(void)
#line 379
{
#line 379
  MSP430TimerM$TBCCTL0 = MSP430TimerM$compareControl();
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbol2M$AlarmControl$setControlAsCompare(void){
#line 35
  MSP430TimerM$ControlB0$setControlAsCompare();
#line 35
}
#line 35
# 51 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  result_t TimerSymbol2M$IStdControl$init(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 53
    hiLocalTime = 0;
#line 53
    __nesc_atomic_end(__nesc_atomic); }
  TimerSymbol2M$shortTimersHead = TimerSymbol2M$EMPTY_LIST;
  TimerSymbol2M$longTimersHead = TimerSymbol2M$EMPTY_LIST;
  bzero(TimerSymbol2M$timers, sizeof TimerSymbol2M$timers);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    TimerSymbol2M$m_posted_checkShortTimers = FALSE;
#line 57
    __nesc_atomic_end(__nesc_atomic); }
  TimerSymbol2M$AlarmControl$setControlAsCompare();
  TimerSymbol2M$AlarmControl$disableEvents();
  return SUCCESS;
}

# 5 "../../../zigzag/PowerManagerM.nc"
static inline  result_t PowerManagerM$StdControl$init(void)
#line 5
{
#line 5
  return SUCCESS;
}

# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t MainM$StdControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = PowerManagerM$StdControl$init();
#line 63
  result = rcombine(result, TimerSymbol2M$IStdControl$init());
#line 63
  result = rcombine(result, TimerSymbolAsyncM$IStdControl$init());
#line 63
  result = rcombine(result, ZigSpi2AppM$StdControl$init());
#line 63
  result = rcombine(result, ZigIrqM$StdControl$init());
#line 63
  result = rcombine(result, ZigSysM$StdControl$init());
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl0$disableEvents(void){
#line 39
  MSP430TimerM$ControlB2$disableEvents();
#line 39
}
#line 39
inline static   void TimerSymbolAsyncM$AlarmControl1$disableEvents(void){
#line 39
  MSP430TimerM$ControlB3$disableEvents();
#line 39
}
#line 39
inline static   void TimerSymbolAsyncM$AlarmControl2$disableEvents(void){
#line 39
  MSP430TimerM$ControlB4$disableEvents();
#line 39
}
#line 39
inline static   void TimerSymbolAsyncM$AlarmControl3$disableEvents(void){
#line 39
  MSP430TimerM$ControlB5$disableEvents();
#line 39
}
#line 39
inline static   void TimerSymbolAsyncM$AlarmControl4$disableEvents(void){
#line 39
  MSP430TimerM$ControlB6$disableEvents();
#line 39
}
#line 39
# 34 "../../../zigzag/ZigSysM.nc"
static inline  void ZigSysM$task_app_init(void)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __app_init();
    }
#line 38
  return;
}

#line 63
static inline  result_t ZigSysM$StdControl$start(void)
{

  TOS_post(ZigSysM$task_app_init);

  return SUCCESS;
}

# 63 "../../../zigzag/ZigIrqM.nc"
static inline  result_t ZigIrqM$StdControl$start(void)
{
#line 76
  DAC12_0CTL &= 0xfff7;
  CACTL1 &= 0xfd;
  ADC12CTL0 &= 0xfff3;

  return SUCCESS;
}

# 173 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static inline   result_t UniSART1M$USARTControl$disableTxIntr(void)
#line 173
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 174
    IE2 &= ~(1 << 5);
#line 174
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   result_t ZigSpi2AppM$USARTControl$disableTxIntr(void){
#line 74
  unsigned char result;
#line 74

#line 74
  result = UniSART1M$USARTControl$disableTxIntr();
#line 74

#line 74
  return result;
#line 74
}
#line 74
# 168 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static inline   result_t UniSART1M$USARTControl$disableRxIntr(void)
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    IE2 &= ~(1 << 4);
#line 169
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 73 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   result_t ZigSpi2AppM$USARTControl$disableRxIntr(void){
#line 73
  unsigned char result;
#line 73

#line 73
  result = UniSART1M$USARTControl$disableRxIntr();
#line 73

#line 73
  return result;
#line 73
}
#line 73
# 58 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_app/hardware.h"
static inline void TOSH_SEL_UCLK1_MODFUNC(void)
#line 58
{
#line 58
   static volatile uint8_t r __asm ("0x0033");

#line 58
  r |= 1 << 3;
}

#line 57
static inline void TOSH_SEL_SOMI1_MODFUNC(void)
#line 57
{
#line 57
   static volatile uint8_t r __asm ("0x0033");

#line 57
  r |= 1 << 2;
}

#line 56
static inline void TOSH_SEL_SIMO1_MODFUNC(void)
#line 56
{
#line 56
   static volatile uint8_t r __asm ("0x0033");

#line 56
  r |= 1 << 1;
}

# 77 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static inline   void UniSART1M$USARTControl$setModeSPI(bool master)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      TOSH_SEL_SIMO1_MODFUNC();
      TOSH_SEL_SOMI1_MODFUNC();
      TOSH_SEL_UCLK1_MODFUNC();

      IE2 &= ~((1 << 5) | (1 << 4));

      if (TRUE == master) {
          U1CTL = ((0x01 | 0x10) | 0x04) | 0x02;
          U1CTL &= ~0x20;

          UniSART1M$U1TCTL = 0x02;
          UniSART1M$U1TCTL |= 0x80;

          if (UniSART1M$l_ssel & 0x80) {
              UniSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
              UniSART1M$U1TCTL |= UniSART1M$l_ssel & 0x7F;
            }
          else {
              UniSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
              UniSART1M$U1TCTL |= 0x20;
            }

          if (UniSART1M$l_br != 0) {
              U1BR0 = UniSART1M$l_br & 0x0FF;
              U1BR1 = (UniSART1M$l_br >> 8) & 0x0FF;
            }
          else {
              U1BR0 = 0x02;
              U1BR1 = 0x00;
            }
        }
      else {
          U1CTL = (0x01 | 0x10) | 0x04;
          UniSART1M$U1TCTL = 0x02 | 0x80;
        }
      U1MCTL = 0;

      UniSART1M$ME2 &= ~((1 << 5) | (1 << 4));
      UniSART1M$ME2 |= 1 << 4;
      U1CTL &= ~0x01;

      UniSART1M$IFG2 &= ~((1 << 5) | (1 << 4));
      IE2 &= ~((1 << 5) | (1 << 4));
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 65 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   void ZigSpi2AppM$USARTControl$setModeSPI(bool arg_0x40a03898){
#line 65
  UniSART1M$USARTControl$setModeSPI(arg_0x40a03898);
#line 65
}
#line 65
# 150 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port14$disable(void)
#line 150
{
#line 150
  MSP430InterruptM$P1IE &= ~(1 << 4);
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigSpi2AppM$INTA_2$disable(void){
#line 35
  MSP430InterruptM$Port14$disable();
#line 35
}
#line 35
# 115 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port10$enable(void)
#line 115
{
#line 115
  MSP430InterruptM$P1IE |= 1 << 0;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigSpi2AppM$INTR$enable(void){
#line 30
  MSP430InterruptM$Port10$enable();
#line 30
}
#line 30
# 177 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port10$clear(void)
#line 177
{
#line 177
  MSP430InterruptM$P1IFG &= ~(1 << 0);
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigSpi2AppM$INTR$clear(void){
#line 40
  MSP430InterruptM$Port10$clear();
#line 40
}
#line 40
# 221 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port10$edge(bool l2h)
#line 221
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 222
    {
      if (l2h) {
#line 223
        P1IES &= ~(1 << 0);
        }
      else {
#line 224
        P1IES |= 1 << 0;
        }
    }
#line 226
    __nesc_atomic_end(__nesc_atomic); }
}

# 54 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigSpi2AppM$INTR$edge(bool arg_0x407cd270){
#line 54
  MSP430InterruptM$Port10$edge(arg_0x407cd270);
#line 54
}
#line 54
# 63 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_app/hardware.h"
static inline void TOSH_CLR_SATELLITE_INTR_2_PIN(void)
#line 63
{
#line 63
   static volatile uint8_t r __asm ("0x0035");

#line 63
  r &= ~(1 << 7);
}

#line 63
static inline void TOSH_MAKE_SATELLITE_INTR_2_OUTPUT(void)
#line 63
{
#line 63
   static volatile uint8_t r __asm ("0x0036");

#line 63
  r |= 1 << 7;
}

# 342 "../../../zigzag/ZigSpi2AppM.nc"
static inline  result_t ZigSpi2AppM$StdControl$start(void)
{
  ZigSpi2AppM$spi_busy = FALSE;

  TOSH_MAKE_SATELLITE_INTR_2_OUTPUT();
  TOSH_CLR_SATELLITE_INTR_2_PIN();

  ZigSpi2AppM$INTR$edge(TRUE);
  ZigSpi2AppM$INTR$clear();
  ZigSpi2AppM$INTR$enable();

  ZigSpi2AppM$INTA_2$disable();

  ZigSpi2AppM$process_join();

  ZigSpi2AppM$USARTControl$setModeSPI(FALSE);
  ZigSpi2AppM$USARTControl$disableRxIntr();
  ZigSpi2AppM$USARTControl$disableTxIntr();



  return SUCCESS;
}

# 254 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline  result_t TimerSymbolAsyncM$IStdControl$start(void)
{
  uint8_t i;

#line 257
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 257
    for (i = 0; i < TimerSymbolAsyncM$ASYNC_TIMER_NUM; i++) {
        TimerSymbolAsyncM$timer[i].isSet = FALSE;
        TimerSymbolAsyncM$DisableEvents(i);
      }
#line 260
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 62 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  result_t TimerSymbol2M$IStdControl$start(void)
#line 62
{
#line 62
  return SUCCESS;
}

# 237 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
static inline void LPMode_enable(void)
#line 237
{
  LPMode_disabled = FALSE;
}

# 6 "../../../zigzag/PowerManagerM.nc"
static inline  result_t PowerManagerM$StdControl$start(void)
{
  LPMode_enable();
  return SUCCESS;
}

# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t MainM$StdControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = PowerManagerM$StdControl$start();
#line 70
  result = rcombine(result, TimerSymbol2M$IStdControl$start());
#line 70
  result = rcombine(result, TimerSymbolAsyncM$IStdControl$start());
#line 70
  result = rcombine(result, ZigSpi2AppM$StdControl$start());
#line 70
  result = rcombine(result, ZigIrqM$StdControl$start());
#line 70
  result = rcombine(result, ZigSysM$StdControl$start());
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 54 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigSpi2AppM$INTJOIN$edge(bool arg_0x407cd270){
#line 54
  MSP430InterruptM$Port15$edge(arg_0x407cd270);
#line 54
}
#line 54
# 182 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port15$clear(void)
#line 182
{
#line 182
  MSP430InterruptM$P1IFG &= ~(1 << 5);
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigSpi2AppM$INTJOIN$clear(void){
#line 40
  MSP430InterruptM$Port15$clear();
#line 40
}
#line 40







inline static   bool ZigSpi2AppM$INTJOIN$getValue(void){
#line 47
  unsigned char result;
#line 47

#line 47
  result = MSP430InterruptM$Port15$getValue();
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 199 "../../../zigzag/ZigSpi2AppM.nc"
static inline  void ZigSpi2AppM$task_net_exit(void)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __net_exit();
    }
#line 203
  return;
}

#line 192
static inline  void ZigSpi2AppM$task_net_enter(void)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __net_enter(0x0000);
    }
#line 196
  return;
}

# 202 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
static inline void __nesc_enable_interrupt(void)
{
   __asm volatile ("eint");}

#line 226
static inline void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

#line 245
static __inline void __nesc_atomic_sleep(void)
#line 245
{








  uint16_t LPMode_bits = 0;

  if (LPMode_disabled) {
      __nesc_enable_interrupt();
      return;
    }
  else 
#line 259
    {
      LPMode_bits = 0x0080 + 0x0040 + 0x0010;



      if ((((
#line 262
      TA0CTL & (3 << 4)) != 0 << 4 && (TA0CTL & (3 << 8)) == 2 << 8)
       || (ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20))
       || (ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20)) {






        LPMode_bits = 0x0040 + 0x0010;
        }


      if (ADC12CTL1 & 0x0001) {
          if (!(ADC12CTL0 & 0x0080) && (TA0CTL & (3 << 8)) == 2 << 8) {
            LPMode_bits = 0x0040 + 0x0010;
            }
          else {
#line 279
            switch (ADC12CTL1 & (3 << 3)) {
                case 2 << 3: LPMode_bits = 0;
#line 280
                break;
                case 3 << 3: LPMode_bits = 0x0040 + 0x0010;
#line 281
                break;
              }
            }
        }

      LPMode_bits |= 0x0008;
       __asm volatile ("bis  %0, r2" :  : "m"((uint16_t )LPMode_bits));}
}

#line 196
static inline void __nesc_disable_interrupt(void)
{
   __asm volatile ("dint");
   __asm volatile ("nop");}







static inline bool are_interrupts_enabled(void)
{
  return (({
#line 209
    uint16_t __x;

#line 209
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 209
   & 0x0008) != 0;
}








static inline __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = are_interrupts_enabled();

#line 222
  __nesc_disable_interrupt();
  return result;
}

# 136 "/home/max/tinyos/tinyos-1.x/tos/system/sched.c"
static inline bool TOSH_run_next_task(void)
{
  __nesc_atomic_t fInterruptFlags;
  uint8_t old_full;
  void (*func)(void );

  fInterruptFlags = __nesc_atomic_start();
  old_full = TOSH_sched_full;
  func = TOSH_queue[old_full].tp;
  if (func == NULL) 
    {
      __nesc_atomic_sleep();
      return 0;
    }

  TOSH_queue[old_full].tp = NULL;
  TOSH_sched_full = (old_full + 1) & TOSH_TASK_BITMASK;
  __nesc_atomic_end(fInterruptFlags);
  func();

  return 1;
}

static inline void TOSH_run_task(void)
#line 159
{
  for (; ; ) 
    TOSH_run_next_task();
}

# 96 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline MSP430TimerM$CC_t MSP430TimerM$int2CC(uint16_t x)
#line 96
{
#line 96
  union __nesc_unnamed4351 {
#line 96
    uint16_t f;
#line 96
    MSP430TimerM$CC_t t;
  } 
#line 96
  c = { .f = x };

#line 96
  return c.t;
}

#line 205
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlA0$getControl(void)
#line 205
{
#line 205
  return MSP430TimerM$int2CC(MSP430TimerM$TA0CCTL0);
}

# 21 "../../../zigzag/ZigTimerAIrqM.nc"
static inline   void ZigTimerAIrqM$Capture0$captured(uint16_t time)
#line 21
{
#line 21
  {
#line 21
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 21
      __process_irq(12);
      }
  }
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureA0$captured(uint16_t arg_0x40695b78){
#line 74
  ZigTimerAIrqM$Capture0$captured(arg_0x40695b78);
#line 74
}
#line 74
# 253 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureA0$getEvent(void)
#line 253
{
#line 253
  return MSP430TimerM$TA0CCR0;
}

# 18 "../../../zigzag/ZigTimerAIrqM.nc"
static inline   void ZigTimerAIrqM$Compare0$fired(void)
#line 18
{
#line 18
  {
#line 18
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 18
      __process_irq(12);
      }
  }
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA0$fired(void){
#line 34
  ZigTimerAIrqM$Compare0$fired();
#line 34
}
#line 34
# 206 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlA1$getControl(void)
#line 206
{
#line 206
  return MSP430TimerM$int2CC(MSP430TimerM$TA0CCTL1);
}

# 22 "../../../zigzag/ZigTimerAIrqM.nc"
static inline   void ZigTimerAIrqM$Capture1$captured(uint16_t time)
#line 22
{
#line 22
  {
#line 22
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 22
      __process_irq(10);
      }
  }
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureA1$captured(uint16_t arg_0x40695b78){
#line 74
  ZigTimerAIrqM$Capture1$captured(arg_0x40695b78);
#line 74
}
#line 74
# 254 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureA1$getEvent(void)
#line 254
{
#line 254
  return MSP430TimerM$TA0CCR1;
}

# 19 "../../../zigzag/ZigTimerAIrqM.nc"
static inline   void ZigTimerAIrqM$Compare1$fired(void)
#line 19
{
#line 19
  {
#line 19
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 19
      __process_irq(10);
      }
  }
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA1$fired(void){
#line 34
  ZigTimerAIrqM$Compare1$fired();
#line 34
}
#line 34
# 207 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlA2$getControl(void)
#line 207
{
#line 207
  return MSP430TimerM$int2CC(MSP430TimerM$TA0CCTL2);
}

# 23 "../../../zigzag/ZigTimerAIrqM.nc"
static inline   void ZigTimerAIrqM$Capture2$captured(uint16_t time)
#line 23
{
#line 23
  {
#line 23
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 23
      __process_irq(10);
      }
  }
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureA2$captured(uint16_t arg_0x40695b78){
#line 74
  ZigTimerAIrqM$Capture2$captured(arg_0x40695b78);
#line 74
}
#line 74
# 255 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureA2$getEvent(void)
#line 255
{
#line 255
  return MSP430TimerM$TA0CCR2;
}

# 20 "../../../zigzag/ZigTimerAIrqM.nc"
static inline   void ZigTimerAIrqM$Compare2$fired(void)
#line 20
{
#line 20
  {
#line 20
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 20
      __process_irq(10);
      }
  }
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA2$fired(void){
#line 34
  ZigTimerAIrqM$Compare2$fired();
#line 34
}
#line 34
# 166 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$TimerB$read(void)
#line 166
{
#line 166
  return TBR;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   uint16_t MSP430DCOCalibM$Timer32khz$read(void){
#line 30
  unsigned int result;
#line 30

#line 30
  result = MSP430TimerM$TimerB$read();
#line 30

#line 30
  return result;
#line 30
}
#line 30
# 41 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430DCOCalibM.nc"
static inline   void MSP430DCOCalibM$TimerMicro$overflow(void)
{
  uint16_t now = MSP430DCOCalibM$Timer32khz$read();
  uint16_t delta = now - MSP430DCOCalibM$m_prev;

#line 45
  MSP430DCOCalibM$m_prev = now;

  if (delta > MSP430DCOCalibM$TARGET_DELTA + MSP430DCOCalibM$MAX_DEVIATION) 
    {

      if (DCOCTL < 0xe0) 
        {
          DCOCTL++;
        }
      else {
#line 54
        if ((BCSCTL1 & 7) < 7) 
          {
            BCSCTL1++;
            DCOCTL = 96;
          }
        }
    }
  else {
#line 60
    if (delta < MSP430DCOCalibM$TARGET_DELTA - MSP430DCOCalibM$MAX_DEVIATION) 
      {

        if (DCOCTL > 0) 
          {
            DCOCTL--;
          }
        else {
#line 67
          if ((BCSCTL1 & 7) > 0) 
            {
              BCSCTL1--;
              DCOCTL = 128;
            }
          }
      }
    }
}

# 171 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerA$clearOverflow(void)
#line 171
{
#line 171
  MSP430TimerM$TA0CTL &= ~0x0001;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerA$clearOverflow(void){
#line 32
  MSP430TimerM$TimerA$clearOverflow();
#line 32
}
#line 32
# 96 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerTuningM.nc"
static inline   void TimerTuningM$ITimerA$overflow(void)
#line 96
{
#line 96
  TimerTuningM$ITimerA$clearOverflow();
}

# 17 "../../../zigzag/ZigTimerAIrqM.nc"
static inline   void ZigTimerAIrqM$Timer$overflow(void)
#line 17
{
#line 17
  {
#line 17
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 17
      __process_irq(10);
      }
  }
}

# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void MSP430TimerM$TimerA$overflow(void){
#line 33
  ZigTimerAIrqM$Timer$overflow();
#line 33
  TimerTuningM$ITimerA$overflow();
#line 33
  MSP430DCOCalibM$TimerMicro$overflow();
#line 33
}
#line 33
# 347 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB0$getControl(void)
#line 347
{
#line 347
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL0);
}

#line 338
static inline    void MSP430TimerM$CaptureB0$default$captured(uint16_t time)
#line 338
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB0$captured(uint16_t arg_0x40695b78){
#line 74
  MSP430TimerM$CaptureB0$default$captured(arg_0x40695b78);
#line 74
}
#line 74
# 443 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureB0$getEvent(void)
#line 443
{
#line 443
  return MSP430TimerM$TBCCR0;
}

# 271 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline   void TimerSymbol2M$AlarmCompare$fired(void)
{
  TimerSymbol2M$Post_checkShortTimers();
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB0$fired(void){
#line 34
  TimerSymbol2M$AlarmCompare$fired();
#line 34
}
#line 34
# 237 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  void TimerSymbol2M$CheckShortTimers(void)
{
  uint8_t head = TimerSymbol2M$shortTimersHead;

#line 240
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 240
    TimerSymbol2M$m_posted_checkShortTimers = FALSE;
#line 240
    __nesc_atomic_end(__nesc_atomic); }
  TimerSymbol2M$shortTimersHead = TimerSymbol2M$EMPTY_LIST;
  TimerSymbol2M$ExecuteTimers(head);
  TimerSymbol2M$SetNextShortEvent();
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   uint16_t TimerSymbol2M$AlarmTimer$read(void){
#line 30
  unsigned int result;
#line 30

#line 30
  result = MSP430TimerM$TimerB$read();
#line 30

#line 30
  return result;
#line 30
}
#line 30
# 25 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   TMilliSec TimeCastM$ITimeCast$SymbolsToMillis(TSysTime symbols)
{
  return (TMilliSec )(((uint64_t )symbols << 1) / 125);
}

# 10 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TMilliSec LocalTime64M$ITimeCast$SymbolsToMillis(TSysTime arg_0x4089f708){
#line 10
  unsigned long result;
#line 10

#line 10
  result = TimeCastM$ITimeCast$SymbolsToMillis(arg_0x4089f708);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 16 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   TSysTime TimeCastM$ITimeCast$JiffiesToSymbols(uint32_t jiffies)
{
  return jiffies << 1;
}

# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TSysTime TimerSymbol2M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x408a1f18){
#line 7
  unsigned long result;
#line 7

#line 7
  result = TimeCastM$ITimeCast$JiffiesToSymbols(arg_0x408a1f18);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 306 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline   TSysTime TimerSymbol2M$ILocalTime$Read(void)
{
  return TimerSymbol2M$ITimeCast$JiffiesToSymbols(TimerSymbol2M$ReadLocalTime());
}

# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
inline static   TSysTime LocalTime64M$ILocalTime$Read(void){
#line 5
  unsigned long result;
#line 5

#line 5
  result = TimerSymbol2M$ILocalTime$Read();
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 52 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static inline  result_t LocalTime64M$ITimerSymbol$Fired(TUniData ud)
{
  TSysTime now = LocalTime64M$ILocalTime$Read();

  LocalTime64M$baseTime += LocalTime64M$ITimeCast$SymbolsToMillis(now - LocalTime64M$firedCount);
  LocalTime64M$firedCount = now;
  return SUCCESS;
}

# 349 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline   result_t TimerSymbol2M$ITimerSymbol$default$Fired(uint8_t num, TUniData uniData)
#line 349
{
#line 349
  return SUCCESS;
}

# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t TimerSymbol2M$ITimerSymbol$Fired(uint8_t arg_0x408d07c0, TUniData arg_0x408c7120){
#line 16
  unsigned char result;
#line 16

#line 16
  switch (arg_0x408d07c0) {
#line 16
    case 0U:
#line 16
      result = LocalTime64M$ITimerSymbol$Fired(arg_0x408c7120);
#line 16
      break;
#line 16
    default:
#line 16
      result = TimerSymbol2M$ITimerSymbol$default$Fired(arg_0x408d07c0, arg_0x408c7120);
#line 16
      break;
#line 16
    }
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 40 "../../../zigzag/ZigSTimerM.nc"
static inline  result_t ZigSTimerM$Timer2$fired(void)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __stimer_fired(2);
    }
#line 44
  return SUCCESS;
}

#line 33
static inline  result_t ZigSTimerM$Timer1$fired(void)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __stimer_fired(1);
    }
#line 37
  return SUCCESS;
}

#line 26
static inline  result_t ZigSTimerM$Timer0$fired(void)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __stimer_fired(0);
    }
#line 30
  return SUCCESS;
}

# 399 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline   result_t TimerSymbol2M$TimerMilli$default$fired(uint8_t num)
#line 399
{
#line 399
  return SUCCESS;
}

# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
inline static  result_t TimerSymbol2M$TimerMilli$fired(uint8_t arg_0x408cf3d0){
#line 37
  unsigned char result;
#line 37

#line 37
  switch (arg_0x408cf3d0) {
#line 37
    case 0U:
#line 37
      result = ZigSTimerM$Timer0$fired();
#line 37
      break;
#line 37
    case 1U:
#line 37
      result = ZigSTimerM$Timer1$fired();
#line 37
      break;
#line 37
    case 2U:
#line 37
      result = ZigSTimerM$Timer2$fired();
#line 37
      break;
#line 37
    default:
#line 37
      result = TimerSymbol2M$TimerMilli$default$fired(arg_0x408cf3d0);
#line 37
      break;
#line 37
    }
#line 37

#line 37
  return result;
#line 37
}
#line 37
# 108 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline void TimerSymbol2M$SignalTimerFired(uint8_t num)
{
  uint16_t num16 = num;

#line 111
  if (TimerSymbol2M$NUM_MILLIS_TIMERS > 0 && num16 >= TimerSymbol2M$NUM_SYMBOL_TIMERS) {
    TimerSymbol2M$TimerMilli$fired(num - TimerSymbol2M$NUM_SYMBOL_TIMERS);
    }
  else {
#line 114
    TimerSymbol2M$ITimerSymbol$Fired(num, TimerSymbol2M$timers[num].uniData);
    }
}

# 467 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$CompareB0$setEventFromNow(uint16_t x)
#line 467
{
#line 467
  MSP430TimerM$TBCCR0 = TBR + x;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerSymbol2M$AlarmCompare$setEventFromNow(uint16_t arg_0x40686958){
#line 32
  MSP430TimerM$CompareB0$setEventFromNow(arg_0x40686958);
#line 32
}
#line 32
# 363 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB0$clearPendingInterrupt(void)
#line 363
{
#line 363
  MSP430TimerM$TBCCTL0 &= ~0x0001;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbol2M$AlarmControl$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB0$clearPendingInterrupt();
#line 32
}
#line 32
# 411 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB0$enableEvents(void)
#line 411
{
#line 411
  MSP430TimerM$TBCCTL0 |= 0x0010;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbol2M$AlarmControl$enableEvents(void){
#line 38
  MSP430TimerM$ControlB0$enableEvents();
#line 38
}
#line 38
# 348 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB1$getControl(void)
#line 348
{
#line 348
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL1);
}

#line 339
static inline    void MSP430TimerM$CaptureB1$default$captured(uint16_t time)
#line 339
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB1$captured(uint16_t arg_0x40695b78){
#line 74
  MSP430TimerM$CaptureB1$default$captured(arg_0x40695b78);
#line 74
}
#line 74
# 444 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureB1$getEvent(void)
#line 444
{
#line 444
  return MSP430TimerM$TBCCR1;
}

#line 332
static inline    void MSP430TimerM$CompareB1$default$fired(void)
#line 332
{
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB1$fired(void){
#line 34
  MSP430TimerM$CompareB1$default$fired();
#line 34
}
#line 34
# 349 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB2$getControl(void)
#line 349
{
#line 349
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL2);
}

#line 340
static inline    void MSP430TimerM$CaptureB2$default$captured(uint16_t time)
#line 340
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB2$captured(uint16_t arg_0x40695b78){
#line 74
  MSP430TimerM$CaptureB2$default$captured(arg_0x40695b78);
#line 74
}
#line 74
# 445 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureB2$getEvent(void)
#line 445
{
#line 445
  return MSP430TimerM$TBCCR2;
}

# 340 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline   void TimerSymbolAsyncM$AlarmCompare0$fired(void)
#line 340
{
#line 340
  TimerSymbolAsyncM$HandleAlarmFire(0);
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB2$fired(void){
#line 34
  TimerSymbolAsyncM$AlarmCompare0$fired();
#line 34
}
#line 34
# 169 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   bool MSP430TimerM$TimerB$isOverflowPending(void)
#line 169
{
#line 169
  return TBCTL & 0x0001;
}

# 31 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   bool TimerSymbol2M$AlarmTimer$isOverflowPending(void){
#line 31
  unsigned char result;
#line 31

#line 31
  result = MSP430TimerM$TimerB$isOverflowPending();
#line 31

#line 31
  return result;
#line 31
}
#line 31
# 65 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline   uint16_t TimerSymbol2M$overflowCount(void)
{
  uint16_t oc;

  /* atomic removed: atomic calls only */
#line 68
  oc = hiLocalTime;
  if (TimerSymbol2M$AlarmTimer$isOverflowPending()) {
    oc = oc < MAX_SYS_JIFFY_HI ? oc + 1 : 0;
    }
#line 71
  return oc;
}

# 28 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
inline static   uint16_t TimerSymbolAsyncM$overflowCount(void){
#line 28
  unsigned int result;
#line 28

#line 28
  result = TimerSymbol2M$overflowCount();
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 457 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$CompareB6$setEvent(uint16_t x)
#line 457
{
#line 457
  MSP430TimerM$TBCCR6 = x;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerSymbolAsyncM$AlarmCompare4$setEvent(uint16_t arg_0x40686010){
#line 30
  MSP430TimerM$CompareB6$setEvent(arg_0x40686010);
#line 30
}
#line 30
# 456 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$CompareB5$setEvent(uint16_t x)
#line 456
{
#line 456
  MSP430TimerM$TBCCR5 = x;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerSymbolAsyncM$AlarmCompare3$setEvent(uint16_t arg_0x40686010){
#line 30
  MSP430TimerM$CompareB5$setEvent(arg_0x40686010);
#line 30
}
#line 30
# 455 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$CompareB4$setEvent(uint16_t x)
#line 455
{
#line 455
  MSP430TimerM$TBCCR4 = x;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerSymbolAsyncM$AlarmCompare2$setEvent(uint16_t arg_0x40686010){
#line 30
  MSP430TimerM$CompareB4$setEvent(arg_0x40686010);
#line 30
}
#line 30
# 454 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$CompareB3$setEvent(uint16_t x)
#line 454
{
#line 454
  MSP430TimerM$TBCCR3 = x;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerSymbolAsyncM$AlarmCompare1$setEvent(uint16_t arg_0x40686010){
#line 30
  MSP430TimerM$CompareB3$setEvent(arg_0x40686010);
#line 30
}
#line 30
# 453 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$CompareB2$setEvent(uint16_t x)
#line 453
{
#line 453
  MSP430TimerM$TBCCR2 = x;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerSymbolAsyncM$AlarmCompare0$setEvent(uint16_t arg_0x40686010){
#line 30
  MSP430TimerM$CompareB2$setEvent(arg_0x40686010);
#line 30
}
#line 30
# 318 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline void TimerSymbolAsyncM$SetEvent(const uint8_t timerNumber, const uint16_t jiffy)
{
  switch (timerNumber) {
      case 0: TimerSymbolAsyncM$AlarmCompare0$setEvent(jiffy);
#line 321
      return;
      case 1: TimerSymbolAsyncM$AlarmCompare1$setEvent(jiffy);
#line 322
      return;
      case 2: TimerSymbolAsyncM$AlarmCompare2$setEvent(jiffy);
#line 323
      return;
      case 3: TimerSymbolAsyncM$AlarmCompare3$setEvent(jiffy);
#line 324
      return;
      case 4: TimerSymbolAsyncM$AlarmCompare4$setEvent(jiffy);
#line 325
      return;
    }
}

#line 86
static inline void TimerSymbolAsyncM$SetTimerAt(const uint8_t timerNumber, const uint32_t time)
{
  uint32_t hi_time;
#line 88
  uint32_t oc;

#line 89
  TimerSymbolAsyncM$ClearPendingInterrupt(timerNumber);
  TimerSymbolAsyncM$SetEvent(timerNumber, time & 0xffff);
  oc = TimerSymbolAsyncM$overflowCount();
  hi_time = (uint32_t )time >> 16;
  if (oc <= hi_time) {
    TimerSymbolAsyncM$timer[timerNumber].remain = hi_time - oc;
    }
  else {
#line 96
    TimerSymbolAsyncM$timer[timerNumber].remain = hi_time + MAX_SYS_JIFFY_HI + 1 - oc;
    }
#line 97
  if (0 == TimerSymbolAsyncM$timer[timerNumber].remain) {
    TimerSymbolAsyncM$EnableEvents(timerNumber);
    }
  else {
#line 100
    TimerSymbolAsyncM$DisableEvents(timerNumber);
    }
#line 101
  return;
}

# 365 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB2$clearPendingInterrupt(void)
#line 365
{
#line 365
  MSP430TimerM$TBCCTL2 &= ~0x0001;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl0$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB2$clearPendingInterrupt();
#line 32
}
#line 32
# 366 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB3$clearPendingInterrupt(void)
#line 366
{
#line 366
  MSP430TimerM$TBCCTL3 &= ~0x0001;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl1$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB3$clearPendingInterrupt();
#line 32
}
#line 32
# 367 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB4$clearPendingInterrupt(void)
#line 367
{
#line 367
  MSP430TimerM$TBCCTL4 &= ~0x0001;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl2$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB4$clearPendingInterrupt();
#line 32
}
#line 32
# 368 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB5$clearPendingInterrupt(void)
#line 368
{
#line 368
  MSP430TimerM$TBCCTL5 &= ~0x0001;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl3$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB5$clearPendingInterrupt();
#line 32
}
#line 32
# 369 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB6$clearPendingInterrupt(void)
#line 369
{
#line 369
  MSP430TimerM$TBCCTL6 &= ~0x0001;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl4$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB6$clearPendingInterrupt();
#line 32
}
#line 32
# 413 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB2$enableEvents(void)
#line 413
{
#line 413
  MSP430TimerM$TBCCTL2 |= 0x0010;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl0$enableEvents(void){
#line 38
  MSP430TimerM$ControlB2$enableEvents();
#line 38
}
#line 38
# 414 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB3$enableEvents(void)
#line 414
{
#line 414
  MSP430TimerM$TBCCTL3 |= 0x0010;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl1$enableEvents(void){
#line 38
  MSP430TimerM$ControlB3$enableEvents();
#line 38
}
#line 38
# 415 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB4$enableEvents(void)
#line 415
{
#line 415
  MSP430TimerM$TBCCTL4 |= 0x0010;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl2$enableEvents(void){
#line 38
  MSP430TimerM$ControlB4$enableEvents();
#line 38
}
#line 38
# 416 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB5$enableEvents(void)
#line 416
{
#line 416
  MSP430TimerM$TBCCTL5 |= 0x0010;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl3$enableEvents(void){
#line 38
  MSP430TimerM$ControlB5$enableEvents();
#line 38
}
#line 38
# 417 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB6$enableEvents(void)
#line 417
{
#line 417
  MSP430TimerM$TBCCTL6 |= 0x0010;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerSymbolAsyncM$AlarmControl4$enableEvents(void){
#line 38
  MSP430TimerM$ControlB6$enableEvents();
#line 38
}
#line 38
# 347 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline    
#line 346
result_t TimerSymbolAsyncM$ITimerSymbolAsync$default$Fired(uint8_t timerNumber, 
TUniData uniData)
{
  return SUCCESS;
}

# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
inline static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$Fired(uint8_t arg_0x4091d3d8, TUniData arg_0x408c4dd8){
#line 16
  unsigned char result;
#line 16

#line 16
    result = TimerSymbolAsyncM$ITimerSymbolAsync$default$Fired(arg_0x4091d3d8, arg_0x408c4dd8);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 350 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB3$getControl(void)
#line 350
{
#line 350
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL3);
}

#line 341
static inline    void MSP430TimerM$CaptureB3$default$captured(uint16_t time)
#line 341
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB3$captured(uint16_t arg_0x40695b78){
#line 74
  MSP430TimerM$CaptureB3$default$captured(arg_0x40695b78);
#line 74
}
#line 74
# 446 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureB3$getEvent(void)
#line 446
{
#line 446
  return MSP430TimerM$TBCCR3;
}

# 341 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline   void TimerSymbolAsyncM$AlarmCompare1$fired(void)
#line 341
{
#line 341
  TimerSymbolAsyncM$HandleAlarmFire(1);
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB3$fired(void){
#line 34
  TimerSymbolAsyncM$AlarmCompare1$fired();
#line 34
}
#line 34
# 351 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB4$getControl(void)
#line 351
{
#line 351
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL4);
}

#line 342
static inline    void MSP430TimerM$CaptureB4$default$captured(uint16_t time)
#line 342
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB4$captured(uint16_t arg_0x40695b78){
#line 74
  MSP430TimerM$CaptureB4$default$captured(arg_0x40695b78);
#line 74
}
#line 74
# 447 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureB4$getEvent(void)
#line 447
{
#line 447
  return MSP430TimerM$TBCCR4;
}

# 342 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline   void TimerSymbolAsyncM$AlarmCompare2$fired(void)
#line 342
{
#line 342
  TimerSymbolAsyncM$HandleAlarmFire(2);
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB4$fired(void){
#line 34
  TimerSymbolAsyncM$AlarmCompare2$fired();
#line 34
}
#line 34
# 352 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB5$getControl(void)
#line 352
{
#line 352
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL5);
}

#line 343
static inline    void MSP430TimerM$CaptureB5$default$captured(uint16_t time)
#line 343
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB5$captured(uint16_t arg_0x40695b78){
#line 74
  MSP430TimerM$CaptureB5$default$captured(arg_0x40695b78);
#line 74
}
#line 74
# 448 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureB5$getEvent(void)
#line 448
{
#line 448
  return MSP430TimerM$TBCCR5;
}

# 343 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline   void TimerSymbolAsyncM$AlarmCompare3$fired(void)
#line 343
{
#line 343
  TimerSymbolAsyncM$HandleAlarmFire(3);
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB5$fired(void){
#line 34
  TimerSymbolAsyncM$AlarmCompare3$fired();
#line 34
}
#line 34
# 353 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB6$getControl(void)
#line 353
{
#line 353
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL6);
}

#line 344
static inline    void MSP430TimerM$CaptureB6$default$captured(uint16_t time)
#line 344
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB6$captured(uint16_t arg_0x40695b78){
#line 74
  MSP430TimerM$CaptureB6$default$captured(arg_0x40695b78);
#line 74
}
#line 74
# 449 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$CaptureB6$getEvent(void)
#line 449
{
#line 449
  return MSP430TimerM$TBCCR6;
}

# 344 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline   void TimerSymbolAsyncM$AlarmCompare4$fired(void)
#line 344
{
#line 344
  TimerSymbolAsyncM$HandleAlarmFire(4);
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB6$fired(void){
#line 34
  TimerSymbolAsyncM$AlarmCompare4$fired();
#line 34
}
#line 34
# 75 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430DCOCalibM.nc"
static inline   void MSP430DCOCalibM$Timer32khz$overflow(void)
{
}

# 172 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerB$clearOverflow(void)
#line 172
{
#line 172
  TBCTL &= ~0x0001;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerB$clearOverflow(void){
#line 32
  MSP430TimerM$TimerB$clearOverflow();
#line 32
}
#line 32
# 97 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerTuningM.nc"
static inline   void TimerTuningM$ITimerB$overflow(void)
#line 97
{
#line 97
  TimerTuningM$ITimerB$clearOverflow();
}

# 66 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline   void TimerSymbolAsyncM$ITimer$overflow(void)
{
  uint8_t i;
  uint16_t tbr = 0;

#line 70
  for (i = 0; i < TimerSymbolAsyncM$ASYNC_TIMER_NUM; i++) {
      if (FALSE == TimerSymbolAsyncM$timer[i].isSet) {
#line 71
        continue;
        }
#line 72
      tbr = TBR + 1;
      if (0 == TimerSymbolAsyncM$timer[i].remain) {
        *((uint16_t *)&TBCCTL2 + i) |= 1;
        }
      else {
#line 76
        TimerSymbolAsyncM$timer[i].remain -= 1;
        }
#line 77
      if (tbr < *((uint16_t *)&TBCCR2 + i)) {
        TimerSymbolAsyncM$ClearPendingInterrupt(i);
        }
#line 79
      if (0 == TimerSymbolAsyncM$timer[i].remain) {
        TimerSymbolAsyncM$EnableEvents(i);
        }
    }
#line 82
  while (TBR < tbr) ;
  return;
}

# 258 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline void TimerSymbol2M$ClearOverflow(void)
{
  uint8_t i;

#line 261
  for (i = 0; i < TimerSymbol2M$NUM_TIMERS; i++) 
    if (TimerSymbol2M$timers[i].isSet && TimerSymbol2M$timers[i].isQueued) {
      TimerSymbol2M$timers[i].isOverflow = FALSE;
      }
}

#line 265
static inline  void TimerSymbol2M$TaskCheckAndClearOverflowLTimers(void)
{
  TimerSymbol2M$ClearOverflow();
  TimerSymbol2M$CheckLongTimers();
}

#line 253
static inline  void TimerSymbol2M$TaskCheckLTimers(void)
{
  TimerSymbol2M$CheckLongTimers();
}

#line 276
static inline   void TimerSymbol2M$AlarmTimer$overflow(void)
{
  if (hiLocalTime < MAX_SYS_JIFFY_HI) {
      hiLocalTime += 1;
      TOS_post(TimerSymbol2M$TaskCheckLTimers);
    }
  else 
#line 281
    {
      hiLocalTime = 0;
      TOS_post(TimerSymbol2M$TaskCheckAndClearOverflowLTimers);
    }
}

# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void MSP430TimerM$TimerB$overflow(void){
#line 33
  TimerSymbol2M$AlarmTimer$overflow();
#line 33
  TimerSymbolAsyncM$ITimer$overflow();
#line 33
  TimerTuningM$ITimerB$overflow();
#line 33
  MSP430DCOCalibM$Timer32khz$overflow();
#line 33
}
#line 33
# 30 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port10$fired(void)
#line 30
{
#line 30
  {
#line 30
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 30
      __process_irq(8);
      }
  }
}

# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_app/hardware.h"
static inline void TOSH_CLR_SATELLITE_INTA_PIN(void)
#line 37
{
#line 37
   static volatile uint8_t r __asm ("0x0021");

#line 37
  r &= ~(1 << 3);
}

# 229 "../../../zigzag/ZigSpi2AppM.nc"
static inline  void ZigSpi2AppM$task_rx_done(void)
{


  if (ZigSpi2AppM$rx.src_addr == 0xfffe) {
      uint8_t msg_type;


      deserialize(ZigSpi2AppM$rx.data, 4, "1:1:1:1:", 
      (void *)0, (void *)0, &msg_type, (void *)0);
      if (msg_type == 0x04) {
          uint8_t event_type;

#line 241
          deserialize(ZigSpi2AppM$rx.data + 4, 9, "8:1:", 
          (void *)0, &event_type);
          if (event_type == 0x20) {
              uint16_t saddr;

#line 245
              deserialize(ZigSpi2AppM$rx.data + 4 + 9, 2 + 8, "2:", 
              &saddr);
              ZigSpi2AppM$spi_busy = FALSE;
              if ((faddr_t )__module_load != (faddr_t )-1) {
                  __net_child_join(saddr, ZigSpi2AppM$rx.ext_addr);
                }

              return;
            }
          else {
#line 253
            if (event_type == 0x21) {
                ZigSpi2AppM$spi_busy = FALSE;
                if ((faddr_t )__module_load != (faddr_t )-1) {
                  __net_child_leave(ZigSpi2AppM$rx.ext_addr);
                  }
#line 257
                return;
              }
            }
        }
#line 260
      ZigSpi2AppM$spi_busy = FALSE;
      return;
    }
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __net_recv(ZigSpi2AppM$rx.src_addr, ZigSpi2AppM$rx.ext_addr, ZigSpi2AppM$rx.data_size, ZigSpi2AppM$rx.data, 0);
    }
#line 265
  ZigSpi2AppM$spi_busy = FALSE;
  return;
}

# 201 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static inline   uint8_t UniSART1M$USARTControl$rx(void)
#line 201
{
  uint8_t value;

  /* atomic removed: atomic calls only */
#line 203
  {
    value = U1RXBUF;
  }
  return value;
}

# 110 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   uint8_t ZigSpi2AppM$USARTControl$rx(void){
#line 110
  unsigned char result;
#line 110

#line 110
  result = UniSART1M$USARTControl$rx();
#line 110

#line 110
  return result;
#line 110
}
#line 110
#line 86
inline static   result_t ZigSpi2AppM$USARTControl$isRxIntrPending(void){
#line 86
  unsigned char result;
#line 86

#line 86
  result = UniSART1M$USARTControl$isRxIntrPending();
#line 86

#line 86
  return result;
#line 86
}
#line 86
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_app/hardware.h"
static inline void TOSH_SET_SATELLITE_INTA_PIN(void)
#line 37
{
#line 37
   static volatile uint8_t r __asm ("0x0021");

#line 37
  r |= 1 << 3;
}

# 81 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   result_t ZigSpi2AppM$USARTControl$isTxIntrPending(void){
#line 81
  unsigned char result;
#line 81

#line 81
  result = UniSART1M$USARTControl$isTxIntrPending();
#line 81

#line 81
  return result;
#line 81
}
#line 81
# 269 "../../../zigzag/ZigSpi2AppM.nc"
static inline   void ZigSpi2AppM$INTR$fired(void)
{
  register uint8_t *data;

  if (ZigSpi2AppM$spi_busy == TRUE) {
      ZigSpi2AppM$INTR$clear();
      return;
    }
  /* atomic removed: atomic calls only */
#line 277
  {
    P5SEL = 0x0e;
    U1CTL = (0x01 | 0x10) | 0x04;
    U1TCTL = 0x02 | 0x80;
    U1MCTL = 0;
    ME2 |= 1 << 4;
    U1CTL &= ~0x01;
  }
  ZigSpi2AppM$USARTControl$isTxIntrPending();
  ZigSpi2AppM$USARTControl$rx();

  TOSH_SET_SATELLITE_INTA_PIN();


  data = (uint8_t *)& ZigSpi2AppM$rx.src_addr;
  while (!ZigSpi2AppM$USARTControl$isRxIntrPending()) ;

  * data++ = ZigSpi2AppM$USARTControl$rx();
  while (!ZigSpi2AppM$USARTControl$isRxIntrPending()) ;
  *data = ZigSpi2AppM$USARTControl$rx();


  {
    uint8_t i;

#line 301
    data = & ZigSpi2AppM$rx.ext_addr;
    for (i = 0; i < sizeof(IEEEAddr ); i++) {
        while (!ZigSpi2AppM$USARTControl$isRxIntrPending()) ;
        * data++ = ZigSpi2AppM$USARTControl$rx();
      }
  }


  data = (uint8_t *)& ZigSpi2AppM$rx.data_size;
  while (!ZigSpi2AppM$USARTControl$isRxIntrPending()) ;
  * data++ = ZigSpi2AppM$USARTControl$rx();
  while (!ZigSpi2AppM$USARTControl$isRxIntrPending()) ;
  *data = ZigSpi2AppM$USARTControl$rx();


  if (0 < ZigSpi2AppM$rx.data_size) {
      uint8_t i;

#line 318
      data = ZigSpi2AppM$rx.data;
      for (i = 0; i < ZigSpi2AppM$rx.data_size; i++) {
          while (!ZigSpi2AppM$USARTControl$isRxIntrPending()) ;
          * data++ = ZigSpi2AppM$USARTControl$rx();
        }
      if (TOS_post(ZigSpi2AppM$task_rx_done)) {
        ZigSpi2AppM$spi_busy = TRUE;
        }
    }
  ZigSpi2AppM$INTR$clear();
  TOSH_CLR_SATELLITE_INTA_PIN();
  return;
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port10$fired(void){
#line 59
  ZigSpi2AppM$INTR$fired();
#line 59
  ZigIrqM$Port10$fired();
#line 59
}
#line 59
# 31 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port11$fired(void)
#line 31
{
#line 31
  {
#line 31
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 31
      __process_irq(8);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port11$fired(void){
#line 59
  ZigIrqM$Port11$fired();
#line 59
}
#line 59
# 32 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port12$fired(void)
#line 32
{
#line 32
  {
#line 32
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 32
      __process_irq(8);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port12$fired(void){
#line 59
  ZigIrqM$Port12$fired();
#line 59
}
#line 59
# 33 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port13$fired(void)
#line 33
{
#line 33
  {
#line 33
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 33
      __process_irq(8);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port13$fired(void){
#line 59
  ZigIrqM$Port13$fired();
#line 59
}
#line 59
# 34 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port14$fired(void)
#line 34
{
#line 34
  {
#line 34
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 34
      __process_irq(8);
      }
  }
}

# 181 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port14$clear(void)
#line 181
{
#line 181
  MSP430InterruptM$P1IFG &= ~(1 << 4);
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigSpi2AppM$INTA_2$clear(void){
#line 40
  MSP430InterruptM$Port14$clear();
#line 40
}
#line 40
# 134 "../../../zigzag/ZigSpi2AppM.nc"
static inline  void ZigSpi2AppM$task_tx_done(void)
{
  ZigSpi2AppM$spi_busy = FALSE;
  if (ZigSpi2AppM$tx.serv == FALSE && (faddr_t )__module_load != (faddr_t )-1) {
    __net_send_done(ZigSpi2AppM$tx.handle, 0);
    }
#line 139
  return;
}

# 194 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static inline   result_t UniSART1M$USARTControl$tx(uint8_t data)
#line 194
{
  /* atomic removed: atomic calls only */
#line 195
  {
    UniSART1M$U1TXBUF = data;
  }
  return SUCCESS;
}

# 103 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   result_t ZigSpi2AppM$USARTControl$tx(uint8_t arg_0x409fd010){
#line 103
  unsigned char result;
#line 103

#line 103
  result = UniSART1M$USARTControl$tx(arg_0x409fd010);
#line 103

#line 103
  return result;
#line 103
}
#line 103
# 142 "../../../zigzag/ZigSpi2AppM.nc"
static __inline void ZigSpi2AppM$tx_data(void)
{
  register uint8_t *data;
  uint8_t i;


  ZigSpi2AppM$USARTControl$isTxIntrPending();
  ZigSpi2AppM$USARTControl$rx();

  data = (uint8_t *)& ZigSpi2AppM$tx.dst_addr;
  ZigSpi2AppM$USARTControl$tx(* data++);
  TOSH_CLR_SATELLITE_INTR_2_PIN();

  while (!ZigSpi2AppM$USARTControl$isTxIntrPending()) ;

  ZigSpi2AppM$USARTControl$tx(*data);
  while (!ZigSpi2AppM$USARTControl$isTxIntrPending()) ;



  data = (uint8_t *)& ZigSpi2AppM$tx.data_size;
  ZigSpi2AppM$USARTControl$tx(* data++);
  while (!ZigSpi2AppM$USARTControl$isTxIntrPending()) ;
  ZigSpi2AppM$USARTControl$tx(*data);
  while (!ZigSpi2AppM$USARTControl$isTxIntrPending()) ;


  data = ZigSpi2AppM$tx.data;

  for (i = 0; i < ZigSpi2AppM$tx.data_size; i++) {
      ZigSpi2AppM$USARTControl$tx(* data++);
      while (!ZigSpi2AppM$USARTControl$isTxIntrPending()) ;
    }

  if (!TOS_post(ZigSpi2AppM$task_tx_done)) {
    ZigSpi2AppM$spi_busy = FALSE;
    }

  return;
}

static inline   void ZigSpi2AppM$INTA_2$fired(void)
{

  ZigSpi2AppM$tx_data();
  ZigSpi2AppM$INTA_2$clear();
  ZigSpi2AppM$INTA_2$disable();
  return;
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port14$fired(void){
#line 59
  ZigSpi2AppM$INTA_2$fired();
#line 59
  ZigIrqM$Port14$fired();
#line 59
}
#line 59
# 35 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port15$fired(void)
#line 35
{
#line 35
  {
#line 35
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 35
      __process_irq(8);
      }
  }
}

# 220 "../../../zigzag/ZigSpi2AppM.nc"
static inline   void ZigSpi2AppM$INTJOIN$fired(void)
{
  ZigSpi2AppM$process_join();
  return;
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port15$fired(void){
#line 59
  ZigSpi2AppM$INTJOIN$fired();
#line 59
  ZigIrqM$Port15$fired();
#line 59
}
#line 59
# 36 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port16$fired(void)
#line 36
{
#line 36
  {
#line 36
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 36
      __process_irq(8);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port16$fired(void){
#line 59
  ZigIrqM$Port16$fired();
#line 59
}
#line 59
# 37 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port17$fired(void)
#line 37
{
#line 37
  {
#line 37
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 37
      __process_irq(8);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port17$fired(void){
#line 59
  ZigIrqM$Port17$fired();
#line 59
}
#line 59
# 39 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port20$fired(void)
#line 39
{
#line 39
  {
#line 39
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 39
      __process_irq(2);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port20$fired(void){
#line 59
  ZigIrqM$Port20$fired();
#line 59
}
#line 59
# 40 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port21$fired(void)
#line 40
{
#line 40
  {
#line 40
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 40
      __process_irq(2);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port21$fired(void){
#line 59
  ZigIrqM$Port21$fired();
#line 59
}
#line 59
# 41 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port22$fired(void)
#line 41
{
#line 41
  {
#line 41
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 41
      __process_irq(2);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port22$fired(void){
#line 59
  ZigIrqM$Port22$fired();
#line 59
}
#line 59
# 42 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port23$fired(void)
#line 42
{
#line 42
  {
#line 42
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 42
      __process_irq(2);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port23$fired(void){
#line 59
  ZigIrqM$Port23$fired();
#line 59
}
#line 59
# 43 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port24$fired(void)
#line 43
{
#line 43
  {
#line 43
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 43
      __process_irq(2);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port24$fired(void){
#line 59
  ZigIrqM$Port24$fired();
#line 59
}
#line 59
# 44 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port25$fired(void)
#line 44
{
#line 44
  {
#line 44
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 44
      __process_irq(2);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port25$fired(void){
#line 59
  ZigIrqM$Port25$fired();
#line 59
}
#line 59
# 45 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port26$fired(void)
#line 45
{
#line 45
  {
#line 45
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 45
      __process_irq(2);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port26$fired(void){
#line 59
  ZigIrqM$Port26$fired();
#line 59
}
#line 59
# 46 "../../../zigzag/ZigIrqM.nc"
static inline   void ZigIrqM$Port27$fired(void)
#line 46
{
#line 46
  {
#line 46
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 46
      __process_irq(2);
      }
  }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port27$fired(void){
#line 59
  ZigIrqM$Port27$fired();
#line 59
}
#line 59
# 195 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$NMI$clear(void)
#line 195
{
#line 195
  IFG1 &= ~(1 << 4);
}

#line 111
static inline    void MSP430InterruptM$NMI$default$fired(void)
#line 111
{
#line 111
  MSP430InterruptM$NMI$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$NMI$fired(void){
#line 59
  MSP430InterruptM$NMI$default$fired();
#line 59
}
#line 59
# 196 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$OF$clear(void)
#line 196
{
#line 196
  IFG1 &= ~(1 << 1);
}

#line 112
static inline    void MSP430InterruptM$OF$default$fired(void)
#line 112
{
#line 112
  MSP430InterruptM$OF$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$OF$fired(void){
#line 59
  MSP430InterruptM$OF$default$fired();
#line 59
}
#line 59
# 197 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$ACCV$clear(void)
#line 197
{
#line 197
  FCTL3 &= ~0x0004;
}

#line 113
static inline    void MSP430InterruptM$ACCV$default$fired(void)
#line 113
{
#line 113
  MSP430InterruptM$ACCV$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$ACCV$fired(void){
#line 59
  MSP430InterruptM$ACCV$default$fired();
#line 59
}
#line 59
# 8 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
inline static  uint64_t ZigSysM$ILocalTime64$getLocalTime(void){
#line 8
  unsigned long long result;
#line 8

#line 8
  result = LocalTime64M$ILocalTime64$getLocalTime();
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 20 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static inline  uint64_t LocalTime64M$ILocalTime64$getLocalTimeAt(TSysTime at)
{
  TSysTime curCount = at;
  TMilliSec t = LocalTime64M$ITimeCast$SymbolsToMillis(curCount - LocalTime64M$firedCount);
  uint64_t localTime = LocalTime64M$baseTime + t;


  return localTime;
}

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
inline static  void ZigSysM$ILocalTime64$setLocalTime(uint64_t arg_0x40895010){
#line 9
  LocalTime64M$ILocalTime64$setLocalTime(arg_0x40895010);
#line 9
}
#line 9
# 42 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   uint32_t TimeCastM$ITimeCast$SymbolsToJiffies(TSysTime symbols)
{
  return symbols >> 1;
}

# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   uint32_t TimerSymbol2M$ITimeCast$SymbolsToJiffies(TSysTime arg_0x4089d868){
#line 15
  unsigned long result;
#line 15

#line 15
  result = TimeCastM$ITimeCast$SymbolsToJiffies(arg_0x4089d868);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 313 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  result_t TimerSymbol2M$ITimerSymbol$SetPeriodic(uint8_t num, TSysTime symbols, TUniData uniData)
{
  return TimerSymbol2M$SetTimer(num, TimerSymbol2M$ITimeCast$SymbolsToJiffies(symbols), TRUE, uniData);
}

# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t LocalTime64M$ITimerSymbol$SetPeriodic(TSysTime arg_0x408aa5d8, TUniData arg_0x408aa760){
#line 6
  unsigned char result;
#line 6

#line 6
  result = TimerSymbol2M$ITimerSymbol$SetPeriodic(0U, arg_0x408aa5d8, arg_0x408aa760);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 103 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline void TimerSymbol2M$RemoveTimer(uint8_t num)
{
  TimerSymbol2M$timers[num].isSet = FALSE;
}

#line 323
static inline  result_t TimerSymbol2M$ITimerSymbol$Stop(uint8_t num)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 325
    TimerSymbol2M$RemoveTimer(num);
#line 325
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t LocalTime64M$ITimerSymbol$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(0U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 329 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  bool TimerSymbol2M$ITimerSymbol$IsSet(uint8_t num)
{
  return TimerSymbol2M$timers[num].isSet;
}

# 11 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  bool LocalTime64M$ITimerSymbol$IsSet(void){
#line 11
  unsigned char result;
#line 11

#line 11
  result = TimerSymbol2M$ITimerSymbol$IsSet(0U);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 35 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static inline  void LocalTime64M$ILocalTime64$setLocalTimeAt(TSysTime at, uint64_t time)
{
  if (TRUE == LocalTime64M$ITimerSymbol$IsSet()) {
    LocalTime64M$ITimerSymbol$Stop();
    }
  LocalTime64M$baseTime = time;
  LocalTime64M$firedCount = at;

  LocalTime64M$ITimerSymbol$SetPeriodic(225000000L, 0);
  return;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   uint32_t TimeCastM$ITimeCast$MillisToJiffies(TMilliSec millis)
{
  return (uint32_t )((uint64_t )millis * 125) >> 2;
}

# 14 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   uint32_t TimerSymbol2M$ITimeCast$MillisToJiffies(TMilliSec arg_0x4089d3b0){
#line 14
  unsigned long result;
#line 14

#line 14
  result = TimeCastM$ITimeCast$MillisToJiffies(arg_0x4089d3b0);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 353 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline uint8_t TimerSymbol2M$fromNumMilli(uint8_t num)
{
  return num + TimerSymbol2M$NUM_SYMBOL_TIMERS;
}











static inline  result_t TimerSymbol2M$TimerMilli$setOneShot(uint8_t num, int32_t millis)
{
  return TimerSymbol2M$SetTimer(TimerSymbol2M$fromNumMilli(num), TimerSymbol2M$ITimeCast$MillisToJiffies(millis), FALSE, 0);
}

# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
inline static  result_t ZigSTimerM$Timer0$setOneShot(int32_t arg_0x408c0088){
#line 28
  unsigned char result;
#line 28

#line 28
  result = TimerSymbol2M$TimerMilli$setOneShot(0U, arg_0x408c0088);
#line 28

#line 28
  return result;
#line 28
}
#line 28
inline static  result_t ZigSTimerM$Timer1$setOneShot(int32_t arg_0x408c0088){
#line 28
  unsigned char result;
#line 28

#line 28
  result = TimerSymbol2M$TimerMilli$setOneShot(1U, arg_0x408c0088);
#line 28

#line 28
  return result;
#line 28
}
#line 28
inline static  result_t ZigSTimerM$Timer2$setOneShot(int32_t arg_0x408c0088){
#line 28
  unsigned char result;
#line 28

#line 28
  result = TimerSymbol2M$TimerMilli$setOneShot(2U, arg_0x408c0088);
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 66 "../../../zigzag/ZigSpi2AppM.nc"
static inline  void ZigSpi2AppM$loop_tx(void)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
      __net_send_done(ZigSpi2AppM$tx.handle, 0);
      __net_recv(ZigSpi2AppM$rx.src_addr, ZigSpi2AppM$rx.ext_addr, ZigSpi2AppM$rx.data_size, ZigSpi2AppM$rx.data, 0);
    }
  ZigSpi2AppM$spi_busy = FALSE;
  return;
}

# 54 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigSpi2AppM$INTA_2$edge(bool arg_0x407cd270){
#line 54
  MSP430InterruptM$Port14$edge(arg_0x407cd270);
#line 54
}
#line 54
# 203 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   bool MSP430InterruptM$Port14$getValue(void)
#line 203
{
#line 203
  bool b;

#line 203
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 203
    b = (P1IN >> 4) & 1;
#line 203
    __nesc_atomic_end(__nesc_atomic); }
#line 203
  return b;
}

# 47 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   bool ZigSpi2AppM$INTA_2$getValue(void){
#line 47
  unsigned char result;
#line 47

#line 47
  result = MSP430InterruptM$Port14$getValue();
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 119 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port14$enable(void)
#line 119
{
#line 119
  MSP430InterruptM$P1IE |= 1 << 4;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigSpi2AppM$INTA_2$enable(void){
#line 30
  MSP430InterruptM$Port14$enable();
#line 30
}
#line 30
# 63 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_app/hardware.h"
static inline void TOSH_SET_SATELLITE_INTR_2_PIN(void)
#line 63
{
#line 63
   static volatile uint8_t r __asm ("0x0035");

#line 63
  r |= 1 << 7;
}

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
inline static  void ZigSpi2AppM$ILocalTime64$setLocalTime(uint64_t arg_0x40895010){
#line 9
  LocalTime64M$ILocalTime64$setLocalTime(arg_0x40895010);
#line 9
}
#line 9
# 102 "/home/max/tinyos/tinyos-1.x/tos/system/sched.c"
 bool TOS_post(void (*tp)(void))
#line 102
{
  __nesc_atomic_t fInterruptFlags;
  uint8_t tmp;



  fInterruptFlags = __nesc_atomic_start();

  tmp = TOSH_sched_free;

  if (TOSH_queue[tmp].tp == NULL) {
      TOSH_sched_free = (tmp + 1) & TOSH_TASK_BITMASK;
      TOSH_queue[tmp].tp = tp;
      __nesc_atomic_end(fInterruptFlags);

      return TRUE;
    }
  else {
      __nesc_atomic_end(fInterruptFlags);

      return FALSE;
    }
}

# 52 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MainM.nc"
  int main(void)
{
  MainM$hardwareInit();
  TOSH_sched_init();

  MainM$StdControl$init();
  MainM$StdControl$start();
  __nesc_enable_interrupt();

  for (; ; ) {
#line 61
      TOSH_run_task();
    }
}

# 139 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
static void MSP430ClockM$set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 98 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static uint16_t MSP430TimerM$compareControl(void)
{
  MSP430TimerM$CC_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return MSP430TimerM$CC2int(x);
}

# 285 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static void TimerSymbolAsyncM$DisableEvents(const uint8_t timerNumber)
{
  switch (timerNumber) {
      case 0: TimerSymbolAsyncM$AlarmControl0$disableEvents();
#line 288
      return;
      case 1: TimerSymbolAsyncM$AlarmControl1$disableEvents();
#line 289
      return;
      case 2: TimerSymbolAsyncM$AlarmControl2$disableEvents();
#line 290
      return;
      case 3: TimerSymbolAsyncM$AlarmControl3$disableEvents();
#line 291
      return;
      case 4: TimerSymbolAsyncM$AlarmControl4$disableEvents();
#line 292
      return;
    }
}

# 206 "../../../zigzag/ZigSpi2AppM.nc"
static void ZigSpi2AppM$process_join(void)
{
  ZigSpi2AppM$INTJOIN$edge(TRUE);
  ZigSpi2AppM$INTJOIN$clear();
  if (ZigSpi2AppM$INTJOIN$getValue() == FALSE) {
      TOS_post(ZigSpi2AppM$task_net_exit);
      return;
    }
  ZigSpi2AppM$INTJOIN$edge(FALSE);
  ZigSpi2AppM$INTJOIN$clear();
  TOS_post(ZigSpi2AppM$task_net_enter);
  return;
}

# 251 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static   void MSP430InterruptM$Port15$edge(bool l2h)
#line 251
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 252
    {
      if (l2h) {
#line 253
        P1IES &= ~(1 << 5);
        }
      else {
#line 254
        P1IES |= 1 << 5;
        }
    }
#line 256
    __nesc_atomic_end(__nesc_atomic); }
}

#line 204
static   bool MSP430InterruptM$Port15$getValue(void)
#line 204
{
#line 204
  bool b;

#line 204
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 204
    b = (P1IN >> 5) & 1;
#line 204
    __nesc_atomic_end(__nesc_atomic); }
#line 204
  return b;
}

# 123 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
 __attribute((wakeup)) __attribute((interrupt(12))) void sig_TIMERA0_VECTOR(void)
{
  if (MSP430TimerM$ControlA0$getControl().cap) {
    MSP430TimerM$CaptureA0$captured(MSP430TimerM$CaptureA0$getEvent());
    }
  else {
#line 128
    MSP430TimerM$CompareA0$fired();
    }
}

#line 131
 __attribute((wakeup)) __attribute((interrupt(10))) void sig_TIMERA1_VECTOR(void)
{
  int n = TA0IV;

#line 134
  switch (n) 
    {
      case 0: break;
      case 2: 
        if (MSP430TimerM$ControlA1$getControl().cap) {
          MSP430TimerM$CaptureA1$captured(MSP430TimerM$CaptureA1$getEvent());
          }
        else {
#line 141
          MSP430TimerM$CompareA1$fired();
          }
#line 142
      break;
      case 4: 
        if (MSP430TimerM$ControlA2$getControl().cap) {
          MSP430TimerM$CaptureA2$captured(MSP430TimerM$CaptureA2$getEvent());
          }
        else {
#line 147
          MSP430TimerM$CompareA2$fired();
          }
#line 148
      break;
      case 6: break;
      case 8: break;
      case 10: MSP430TimerM$TimerA$overflow();
#line 151
      break;
      case 12: break;
      case 14: break;
    }
}

#line 277
 __attribute((wakeup)) __attribute((interrupt(26))) void sig_TIMERB0_VECTOR(void)
{
  if (MSP430TimerM$ControlB0$getControl().cap) {
    MSP430TimerM$CaptureB0$captured(MSP430TimerM$CaptureB0$getEvent());
    }
  else {
#line 282
    MSP430TimerM$CompareB0$fired();
    }
}

# 189 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static void TimerSymbol2M$Post_checkShortTimers(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      if (!TimerSymbol2M$m_posted_checkShortTimers) {
          if (TOS_post(TimerSymbol2M$CheckShortTimers)) {
            TimerSymbol2M$m_posted_checkShortTimers = TRUE;
            }
        }
    }
#line 197
    __nesc_atomic_end(__nesc_atomic); }
}

#line 143
static void TimerSymbol2M$ExecuteTimers(uint8_t head)
{
  bool signalTimer;
  TimerSymbol2M$TSyncTimer *timer;
  uint8_t num;
  TimerSymbol2M$TJiffy wakeUpDelta;
  TimerSymbol2M$TJiffy localTime = TimerSymbol2M$ReadLocalTime();

#line 150
  while (TimerSymbol2M$EMPTY_LIST != head) {
      signalTimer = FALSE;
      num = head;
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 153
        {
          timer = &TimerSymbol2M$timers[num];
          head = timer->next;
          timer->isQueued = FALSE;
          if (timer->isSet) {
              int32_t remain = timer->wakeUpTime - localTime;

#line 159
              timer->isSet = FALSE;


              if (0 < remain) {

                  TimerSymbol2M$InsertTimer(num, remain <= TimerSymbol2M$MAX_HW_COUNTER);
                }
              else 
#line 165
                {
                  if (timer->isOverflow) {
                      wakeUpDelta = TimerSymbol2M$GetWakeUpDelta(timer, localTime);

                      TimerSymbol2M$InsertTimer(num, wakeUpDelta <= TimerSymbol2M$MAX_HW_COUNTER);
                    }
                  else 
#line 170
                    {
                      if (timer->isPeriodic) {
                          TimerSymbol2M$SetWakeUpTime(timer, timer->wakeUpTime, timer->period);
                          wakeUpDelta = TimerSymbol2M$GetWakeUpDelta(timer, localTime);

                          TimerSymbol2M$InsertTimer(num, wakeUpDelta <= TimerSymbol2M$MAX_HW_COUNTER);
                        }
                      signalTimer = TRUE;
                    }
                }
            }
        }
#line 181
        __nesc_atomic_end(__nesc_atomic); }
      if (signalTimer) {
        TimerSymbol2M$SignalTimerFired(num);
        }
    }
}

#line 74
static TimerSymbol2M$TJiffy TimerSymbol2M$ReadLocalTime(void)
{
  TimerSymbol2M$TJiffy localTime;
  uint16_t lonow;
#line 77
  uint16_t hinow;

#line 78
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 78
    {
      hinow = hiLocalTime;
      if (TimerSymbol2M$AlarmTimer$isOverflowPending()) {
        hinow = hinow < MAX_SYS_JIFFY_HI ? hinow + 1 : 0;
        }
#line 82
      lonow = TimerSymbol2M$AlarmTimer$read();
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
  localTime = ((TimerSymbol2M$TJiffy )hinow << 16) | lonow;
  return localTime;
}

static void TimerSymbol2M$InsertTimer(uint8_t num, bool isShort)
{
  if (FALSE == TimerSymbol2M$timers[num].isQueued) {
      if (isShort) {
          TimerSymbol2M$timers[num].next = TimerSymbol2M$shortTimersHead;
          TimerSymbol2M$shortTimersHead = num;
        }
      else 
#line 94
        {
          TimerSymbol2M$timers[num].next = TimerSymbol2M$longTimersHead;
          TimerSymbol2M$longTimersHead = num;
        }
      TimerSymbol2M$timers[num].isQueued = TRUE;
    }
  TimerSymbol2M$timers[num].isSet = TRUE;
}

#line 129
static TimerSymbol2M$TJiffy TimerSymbol2M$GetWakeUpDelta(TimerSymbol2M$TSyncTimer *timer, TimerSymbol2M$TJiffy localTime)
{
  TimerSymbol2M$TJiffy delta;

#line 132
  if (FALSE == timer->isOverflow) {
      if (localTime <= timer->wakeUpTime) {
        delta = timer->wakeUpTime - localTime;
        }
      else {
#line 136
        delta = 0;
        }
    }
  else 
#line 137
    {
      delta = MAX_SYS_JIFFY - localTime + timer->wakeUpTime;
    }
  return delta;
}

#line 117
static void TimerSymbol2M$SetWakeUpTime(TimerSymbol2M$TSyncTimer *timer, TimerSymbol2M$TJiffy base, TimerSymbol2M$TJiffy delta)
{
  TimerSymbol2M$TJiffy dt = MAX_SYS_JIFFY - base;

#line 120
  if (dt < delta) {
      timer->isOverflow = TRUE;
      timer->wakeUpTime = delta - dt;
    }
  else 
#line 123
    {
      timer->isOverflow = FALSE;
      timer->wakeUpTime = base + delta;
    }
}

#line 199
static void TimerSymbol2M$SetNextShortEvent(void)
{
  TimerSymbol2M$TJiffy localTime = TimerSymbol2M$ReadLocalTime();

#line 202
  if (TimerSymbol2M$EMPTY_LIST != TimerSymbol2M$shortTimersHead) {
      uint8_t head = TimerSymbol2M$shortTimersHead;
      uint8_t soon = head;
      TimerSymbol2M$TJiffy remaining = TimerSymbol2M$GetWakeUpDelta(&TimerSymbol2M$timers[head], localTime);

#line 206
      head = TimerSymbol2M$timers[head].next;
      while (TimerSymbol2M$EMPTY_LIST != head) {
          TimerSymbol2M$TJiffy dt = TimerSymbol2M$GetWakeUpDelta(&TimerSymbol2M$timers[head], localTime);

#line 209
          if (dt < remaining) {
              remaining = dt;
              soon = head;
            }
          head = TimerSymbol2M$timers[head].next;
        }

      localTime = TimerSymbol2M$ReadLocalTime();
      remaining = TimerSymbol2M$GetWakeUpDelta(&TimerSymbol2M$timers[soon], localTime);

      if (0 < remaining) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 220
            {
              if (TimerSymbol2M$MIN_HW_COUNTER < remaining) {
                TimerSymbol2M$AlarmCompare$setEventFromNow(remaining);
                }
              else {
#line 224
                TimerSymbol2M$AlarmCompare$setEventFromNow(TimerSymbol2M$MIN_HW_COUNTER);
                }
#line 225
              TimerSymbol2M$AlarmControl$clearPendingInterrupt();
              TimerSymbol2M$AlarmControl$enableEvents();
            }
#line 227
            __nesc_atomic_end(__nesc_atomic); }
        }
      else 
#line 228
        {
          TimerSymbol2M$AlarmControl$disableEvents();
          TimerSymbol2M$Post_checkShortTimers();
        }
    }
  else 
#line 232
    {
      TimerSymbol2M$AlarmControl$disableEvents();
    }
}

# 285 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
 __attribute((wakeup)) __attribute((interrupt(24))) void sig_TIMERB1_VECTOR(void)
{
  int n = TBIV;

#line 288
  switch (n) 
    {
      case 0: break;
      case 2: 
        if (MSP430TimerM$ControlB1$getControl().cap) {
          MSP430TimerM$CaptureB1$captured(MSP430TimerM$CaptureB1$getEvent());
          }
        else {
#line 295
          MSP430TimerM$CompareB1$fired();
          }
#line 296
      break;
      case 4: 
        if (MSP430TimerM$ControlB2$getControl().cap) {
          MSP430TimerM$CaptureB2$captured(MSP430TimerM$CaptureB2$getEvent());
          }
        else {
#line 301
          MSP430TimerM$CompareB2$fired();
          }
#line 302
      break;
      case 6: 
        if (MSP430TimerM$ControlB3$getControl().cap) {
          MSP430TimerM$CaptureB3$captured(MSP430TimerM$CaptureB3$getEvent());
          }
        else {
#line 307
          MSP430TimerM$CompareB3$fired();
          }
#line 308
      break;
      case 8: 
        if (MSP430TimerM$ControlB4$getControl().cap) {
          MSP430TimerM$CaptureB4$captured(MSP430TimerM$CaptureB4$getEvent());
          }
        else {
#line 313
          MSP430TimerM$CompareB4$fired();
          }
#line 314
      break;
      case 10: 
        if (MSP430TimerM$ControlB5$getControl().cap) {
          MSP430TimerM$CaptureB5$captured(MSP430TimerM$CaptureB5$getEvent());
          }
        else {
#line 319
          MSP430TimerM$CompareB5$fired();
          }
#line 320
      break;
      case 12: 
        if (MSP430TimerM$ControlB6$getControl().cap) {
          MSP430TimerM$CaptureB6$captured(MSP430TimerM$CaptureB6$getEvent());
          }
        else {
#line 325
          MSP430TimerM$CompareB6$fired();
          }
#line 326
      break;
      case 14: MSP430TimerM$TimerB$overflow();
#line 327
      break;
    }
}

# 104 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static void TimerSymbolAsyncM$HandleAlarmFire(const uint8_t timerNumber)
{
  if (TRUE == TimerSymbolAsyncM$timer[timerNumber].isPeriodic) {
      TimerSymbolAsyncM$timer[timerNumber].shotTime += TimerSymbolAsyncM$timer[timerNumber].period;
      TimerSymbolAsyncM$SetTimerAt(timerNumber, TimerSymbolAsyncM$timer[timerNumber].shotTime);
    }
  else 
#line 109
    {
      TimerSymbolAsyncM$timer[timerNumber].isSet = FALSE;
      TimerSymbolAsyncM$DisableEvents(timerNumber);
    }
  TimerSymbolAsyncM$ITimerSymbolAsync$Fired(timerNumber, TimerSymbolAsyncM$timer[timerNumber].uniData);
}

#line 307
static void TimerSymbolAsyncM$ClearPendingInterrupt(const uint8_t timerNumber)
{
  switch (timerNumber) {
      case 0: TimerSymbolAsyncM$AlarmControl0$clearPendingInterrupt();
#line 310
      return;
      case 1: TimerSymbolAsyncM$AlarmControl1$clearPendingInterrupt();
#line 311
      return;
      case 2: TimerSymbolAsyncM$AlarmControl2$clearPendingInterrupt();
#line 312
      return;
      case 3: TimerSymbolAsyncM$AlarmControl3$clearPendingInterrupt();
#line 313
      return;
      case 4: TimerSymbolAsyncM$AlarmControl4$clearPendingInterrupt();
#line 314
      return;
    }
}

#line 296
static void TimerSymbolAsyncM$EnableEvents(const uint8_t timerNumber)
{
  switch (timerNumber) {
      case 0: TimerSymbolAsyncM$AlarmControl0$enableEvents();
#line 299
      return;
      case 1: TimerSymbolAsyncM$AlarmControl1$enableEvents();
#line 300
      return;
      case 2: TimerSymbolAsyncM$AlarmControl2$enableEvents();
#line 301
      return;
      case 3: TimerSymbolAsyncM$AlarmControl3$enableEvents();
#line 302
      return;
      case 4: TimerSymbolAsyncM$AlarmControl4$enableEvents();
#line 303
      return;
    }
}

# 246 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static void TimerSymbol2M$CheckLongTimers(void)
{
  uint8_t head = TimerSymbol2M$longTimersHead;

#line 249
  TimerSymbol2M$longTimersHead = TimerSymbol2M$EMPTY_LIST;
  TimerSymbol2M$ExecuteTimers(head);
  TimerSymbol2M$SetNextShortEvent();
}

# 48 "../../../zigzag/ZigIrqM.nc"
 __attribute((wakeup)) __attribute((interrupt(0))) void sig_DACDMA_VECTOR(void)
#line 48
{
#line 48
  {
#line 48
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 48
      __process_irq(0);
      }
  }
}

#line 49
 __attribute((wakeup)) __attribute((interrupt(14))) void sig_ADC12_VECTOR(void)
#line 49
{
#line 49
  {
#line 49
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 49
      __process_irq(14);
      }
  }
}

#line 50
 __attribute((wakeup)) __attribute((interrupt(22))) void sig_COMPARATORA_VECTOR(void)
#line 50
{
#line 50
  {
#line 50
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 50
      __process_irq(22);
      }
  }
}

#line 53
 __attribute((wakeup)) __attribute((interrupt(18))) void sig_UART0RX_VECTOR(void)
#line 53
{
#line 53
  {
#line 53
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 53
      __process_irq(18);
      }
  }
}

#line 54
 __attribute((wakeup)) __attribute((interrupt(16))) void sig_UART0TX_VECTOR(void)
#line 54
{
#line 54
  {
#line 54
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 54
      __process_irq(16);
      }
  }
}

#line 58
 __attribute((wakeup)) __attribute((interrupt(6))) void sig_UART1RX_VECTOR(void)
#line 58
{
#line 58
  {
#line 58
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 58
      __process_irq(6);
      }
  }
}

#line 59
 __attribute((wakeup)) __attribute((interrupt(4))) void sig_UART1TX_VECTOR(void)
#line 59
{
#line 59
  {
#line 59
    if ((faddr_t )__module_load != (faddr_t )-1) {
#line 59
      __process_irq(4);
      }
  }
}

# 56 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
 __attribute((wakeup)) __attribute((interrupt(8))) void sig_PORT1_VECTOR(void)
{
  volatile int n = MSP430InterruptM$P1IFG & MSP430InterruptM$P1IE;

  if (n & (1 << 0)) {
#line 60
      MSP430InterruptM$Port10$fired();
#line 60
      return;
    }
#line 61
  if (n & (1 << 1)) {
#line 61
      MSP430InterruptM$Port11$fired();
#line 61
      return;
    }
#line 62
  if (n & (1 << 2)) {
#line 62
      MSP430InterruptM$Port12$fired();
#line 62
      return;
    }
#line 63
  if (n & (1 << 3)) {
#line 63
      MSP430InterruptM$Port13$fired();
#line 63
      return;
    }
#line 64
  if (n & (1 << 4)) {
#line 64
      MSP430InterruptM$Port14$fired();
#line 64
      return;
    }
#line 65
  if (n & (1 << 5)) {
#line 65
      MSP430InterruptM$Port15$fired();
#line 65
      return;
    }
#line 66
  if (n & (1 << 6)) {
#line 66
      MSP430InterruptM$Port16$fired();
#line 66
      return;
    }
#line 67
  if (n & (1 << 7)) {
#line 67
      MSP430InterruptM$Port17$fired();
#line 67
      return;
    }
}

# 145 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static   result_t UniSART1M$USARTControl$isTxIntrPending(void)
#line 145
{
  if (UniSART1M$IFG2 & (1 << 5)) {
      UniSART1M$IFG2 &= ~(1 << 5);
      return SUCCESS;
    }
  return FAIL;
}








static   result_t UniSART1M$USARTControl$isRxIntrPending(void)
#line 160
{
  if (UniSART1M$IFG2 & (1 << 4)) {
      UniSART1M$IFG2 &= ~(1 << 4);
      return SUCCESS;
    }
  return FAIL;
}

# 56 "../../../zigzag/SerializeM.nc"
  size_t deserialize(const void *src, size_t size, const char *fmt, ...)
{
  uint8_t num = 0;
  void *dst;
  va_list args;

  __builtin_va_start(args, fmt);

  while ('\0' != *fmt) {
      if (':' == *fmt) {
          if (0 < num) {
              if (size < num) {
                break;
                }
#line 69
              dst = (void *)(__builtin_va_arg(args, unsigned int ));
              SerializeM$memcopy(dst, src, num);
              size -= num;
              src = (uint8_t *)src + num;
            }
          num = 0;
        }
      else {
#line 76
        num = *fmt - '0';
        }
#line 77
      ++fmt;
    }

  __builtin_va_end(args);

  return size;
}

#line 8
static void SerializeM$memcopy(void *const dst, const void *const src, const uint8_t len)
{
  uint8_t i;

#line 11
  for (i = 0; i < len; i++) (
    (uint8_t *)dst)[i] = ((uint8_t *)src)[i];
}

# 71 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
 __attribute((wakeup)) __attribute((interrupt(2))) void sig_PORT2_VECTOR(void)
{
  volatile int n = MSP430InterruptM$P2IFG & MSP430InterruptM$P2IE;

  if (n & (1 << 0)) {
#line 75
      MSP430InterruptM$Port20$fired();
#line 75
      return;
    }
#line 76
  if (n & (1 << 1)) {
#line 76
      MSP430InterruptM$Port21$fired();
#line 76
      return;
    }
#line 77
  if (n & (1 << 2)) {
#line 77
      MSP430InterruptM$Port22$fired();
#line 77
      return;
    }
#line 78
  if (n & (1 << 3)) {
#line 78
      MSP430InterruptM$Port23$fired();
#line 78
      return;
    }
#line 79
  if (n & (1 << 4)) {
#line 79
      MSP430InterruptM$Port24$fired();
#line 79
      return;
    }
#line 80
  if (n & (1 << 5)) {
#line 80
      MSP430InterruptM$Port25$fired();
#line 80
      return;
    }
#line 81
  if (n & (1 << 6)) {
#line 81
      MSP430InterruptM$Port26$fired();
#line 81
      return;
    }
#line 82
  if (n & (1 << 7)) {
#line 82
      MSP430InterruptM$Port27$fired();
#line 82
      return;
    }
}

#line 85
 __attribute((wakeup)) __attribute((interrupt(28))) void sig_NMI_VECTOR(void)
{
  volatile int n = IFG1;

#line 88
  if (n & (1 << 4)) {
#line 88
      MSP430InterruptM$NMI$fired();
#line 88
      return;
    }
#line 89
  if (n & (1 << 1)) {
#line 89
      MSP430InterruptM$OF$fired();
#line 89
      return;
    }
#line 90
  if (FCTL3 & 0x0004) {
#line 90
      MSP430InterruptM$ACCV$fired();
#line 90
      return;
    }
}

# 11 "../../../zigzag/ZigSysM.nc"
  int16_t __port_perm(const uint8_t port_num, const uint8_t mask, const uint8_t op)
{
  if (6 < port_num) {
    return -1;
    }
#line 15
  if ((ZigSysM$en_mask[port_num - 1] | mask) ^ ZigSysM$en_mask[port_num - 1]) {
    return -1;
    }
#line 17
  return 0;
}



  uint64_t __sys_time(void)
{
  return (uint64_t )ZigSysM$ILocalTime64$getLocalTime();
}

# 30 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static  uint64_t LocalTime64M$ILocalTime64$getLocalTime(void)
{
  return LocalTime64M$ILocalTime64$getLocalTimeAt(LocalTime64M$ILocalTime$Read());
}

# 27 "../../../zigzag/ZigSysM.nc"
  int16_t __set_sys_time(uint64_t time)
{
  ZigSysM$ILocalTime64$setLocalTime(time);
  return 0;
}

# 47 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static  void LocalTime64M$ILocalTime64$setLocalTime(uint64_t time)
{
  LocalTime64M$ILocalTime64$setLocalTimeAt(LocalTime64M$ILocalTime$Read(), time);
}

# 287 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static result_t TimerSymbol2M$SetTimer(uint8_t num, TimerSymbol2M$TJiffy jiffy, bool isPeriodic, TUniData uniData)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 289
    {
      TimerSymbol2M$TSyncTimer *timer = &TimerSymbol2M$timers[num];
      TimerSymbol2M$TJiffy localTime;

#line 292
      if (timer->isSet) {
        TimerSymbol2M$RemoveTimer(num);
        }
#line 294
      timer->period = jiffy;
      timer->isPeriodic = isPeriodic;
      timer->uniData = uniData;
      localTime = TimerSymbol2M$ReadLocalTime();
      TimerSymbol2M$SetWakeUpTime(timer, localTime, jiffy);
      TimerSymbol2M$InsertTimer(num, jiffy <= TimerSymbol2M$MAX_HW_COUNTER);
      TimerSymbol2M$SetNextShortEvent();
    }
#line 301
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 42 "../../../zigzag/ZigSysM.nc"
  int16_t __post_task(task_ft task_func)
{
  if (TRUE == TOS_post(task_func)) {
    return 0;
    }
#line 46
  return -1;
}


  void __critical_enter(void)
{
  ZigSysM$reenable_interrupt = __nesc_atomic_start();
  return;
}

  void __critical_exit(void)
{
  __nesc_atomic_end(ZigSysM$reenable_interrupt);
  return;
}

# 11 "../../../zigzag/ZigSTimerM.nc"
  int16_t __stimer_set(const uint8_t timer_num, const uint32_t milli_sec)
{
  result_t res;

#line 14
  switch (timer_num) {
      case 0: res = ZigSTimerM$Timer0$setOneShot(milli_sec);
#line 15
      break;
      case 1: res = ZigSTimerM$Timer1$setOneShot(milli_sec);
#line 16
      break;
      case 2: res = ZigSTimerM$Timer2$setOneShot(milli_sec);
#line 17
      break;
      default: res = FAIL;
    }
  if (SUCCESS == res) {
    return 0;
    }
#line 22
  return -1;
}

# 15 "../../../zigzag/SerializeM.nc"
  size_t serialize(void *dst, size_t max_size, const char *fmt, ...)
{
  uint8_t len = 0;
#line 17
  uint8_t num = 0;
  va_list args;

#line 19
  __builtin_va_start(args, fmt);
  while ('\0' != *fmt) {
      if (*fmt == ':') {
          if (0 < num) {
              if (max_size < len + num) {
                break;
                }
#line 25
              switch (num) {
                  case 1: {
#line 26
                      unsigned char src = (unsigned char )(__builtin_va_arg(args, unsigned int ));

#line 27
                      SerializeM$memcopy(dst, &src, sizeof src);
                    }
                  break;
                  case 2: {
#line 30
                      unsigned int src = (__builtin_va_arg(args, unsigned int ));

#line 31
                      SerializeM$memcopy(dst, &src, sizeof src);
                    }
                  break;
                  case 4: {
#line 34
                      unsigned int src = (__builtin_va_arg(args, unsigned long int ));

#line 35
                      SerializeM$memcopy(dst, &src, sizeof src);
                    }
                  break;
                  case 8: {
#line 38
                      unsigned long long int src = (__builtin_va_arg(args, unsigned long long int ));

#line 39
                      SerializeM$memcopy(dst, &src, sizeof src);
                    }
                  break;
                  default: break;
                }
              len += num;
              dst = (uint8_t *)dst + num;
            }
          num = 0;
        }
      else {
#line 49
        num = *fmt - '0';
        }
#line 50
      ++fmt;
    }
  __builtin_va_end(args);
  return len;
}

# 76 "../../../zigzag/ZigSpi2AppM.nc"
  int16_t __net_send(const uint16_t dst_addr, const uint16_t data_size, 
const uint8_t *const data, uint8_t handle)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      if (ZigSpi2AppM$spi_busy == TRUE) 
        {
          int __nesc_temp = 
#line 81
          -1;

          {
#line 81
            __nesc_atomic_end(__nesc_atomic); 
#line 81
            return __nesc_temp;
          }
        }
#line 82
      ZigSpi2AppM$spi_busy = TRUE;
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
  ZigSpi2AppM$tx.dst_addr = dst_addr;
  ZigSpi2AppM$tx.data_size = data_size;
  ZigSpi2AppM$tx.data = (uint8_t *)data;
  ZigSpi2AppM$tx.handle = handle;
  ZigSpi2AppM$tx.serv = FALSE;
  if (dst_addr == __net_addr()) {
      uint16_t i;


      for (i = 0; i < data_size; i++) 
        ZigSpi2AppM$rx.data[i] = data[i];
      ZigSpi2AppM$rx.src_addr = __net_addr();
      ZigSpi2AppM$rx.ext_addr = 0;
      ZigSpi2AppM$rx.data_size = data_size;
      if (!TOS_post(ZigSpi2AppM$loop_tx)) {
          ZigSpi2AppM$spi_busy = FALSE;
          return -1;
        }
    }
  else 
#line 102
    {

      if (ZigSpi2AppM$INTJOIN$getValue() == TRUE) {
        ZigSpi2AppM$tx_request();
        }
    }
#line 107
  return 0;
}

#line 332
  uint16_t __net_addr(void)
{
  return 0x0000;
}

#line 30
static void ZigSpi2AppM$tx_request(void)
{
  TOSH_CLR_SATELLITE_INTR_2_PIN();



  ZigSpi2AppM$INTA_2$disable();
  ZigSpi2AppM$INTA_2$edge(FALSE);
  ZigSpi2AppM$INTA_2$clear();
  if (ZigSpi2AppM$INTA_2$getValue() == TRUE) {
      ZigSpi2AppM$INTA_2$enable();
      return;
    }

  ZigSpi2AppM$INTA_2$edge(TRUE);
  ZigSpi2AppM$INTA_2$clear();
  TOSH_SET_SATELLITE_INTR_2_PIN();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    {
      P5SEL = 0x0e;
      U1CTL = (0x01 | 0x10) | 0x04;
      U1TCTL = 0x02 | 0x80;
      U1MCTL = 0;
      ME2 |= 1 << 4;
      U1CTL &= ~0x01;
    }
#line 54
    __nesc_atomic_end(__nesc_atomic); }
  ZigSpi2AppM$INTA_2$enable();
  return;
}

# 245 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static   void MSP430InterruptM$Port14$edge(bool l2h)
#line 245
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 246
    {
      if (l2h) {
#line 247
        P1IES &= ~(1 << 4);
        }
      else {
#line 248
        P1IES |= 1 << 4;
        }
    }
#line 250
    __nesc_atomic_end(__nesc_atomic); }
}

# 113 "../../../zigzag/ZigSpi2AppM.nc"
  int16_t __set_sys_time2(uint64_t time)
{
  ZigSpi2AppM$__workaround = &time;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 117
    {
      if (ZigSpi2AppM$spi_busy == TRUE) 
        {
          int __nesc_temp = 
#line 119
          -1;

          {
#line 119
            __nesc_atomic_end(__nesc_atomic); 
#line 119
            return __nesc_temp;
          }
        }
#line 120
      ZigSpi2AppM$spi_busy = TRUE;
    }
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  serialize(ZigSpi2AppM$set_time_msg, 12, "1:1:1:1:8:1:", 
  (uint8_t )0, (uint8_t )0, (uint8_t )0x04, (uint8_t )9, time, (uint8_t )0xff);
  ZigSpi2AppM$ILocalTime64$setLocalTime(time);
  ZigSpi2AppM$tx.dst_addr = 0xfffe;
  ZigSpi2AppM$tx.data_size = 12;
  ZigSpi2AppM$tx.data = ZigSpi2AppM$set_time_msg;
  ZigSpi2AppM$tx.serv = TRUE;
  if (ZigSpi2AppM$INTJOIN$getValue() == TRUE) {
    ZigSpi2AppM$tx_request();
    }
#line 131
  return 0;
}

