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
typedef struct __nesc_unnamed4243 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4244 {
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

  union __nesc_unnamed4245 {

    struct __nesc_unnamed4246 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4247 {


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






enum __nesc_unnamed4248 {
  FALSE = 0, 
  TRUE = 1
};



enum __nesc_unnamed4249 {
  FAIL = 0, 
  SUCCESS = 1
};


static inline uint8_t rcombine(uint8_t r1, uint8_t r2);
typedef uint8_t result_t  ;







static inline result_t rcombine(result_t r1, result_t r2);
#line 140
enum __nesc_unnamed4250 {
  NULL = 0x0
};
# 39 "/opt/msp430-3.3.6/msp430/include/msp430/iostructures.h"
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4251 {
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
# 112 "/opt/msp430-3.3.6/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");

volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");

volatile unsigned char P2IFG __asm ("0x002B");

volatile unsigned char P2IES __asm ("0x002C");

volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");










volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 85 "/opt/msp430-3.3.6/msp430/include/msp430/usart.h"
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");

volatile unsigned char U0TXBUF __asm ("0x0077");
#line 256
volatile unsigned char U1CTL __asm ("0x0078");

volatile unsigned char U1TCTL __asm ("0x0079");



volatile unsigned char U1MCTL __asm ("0x007B");

volatile unsigned char U1BR0 __asm ("0x007C");

volatile unsigned char U1BR1 __asm ("0x007D");

volatile unsigned char U1RXBUF __asm ("0x007E");
# 24 "/opt/msp430-3.3.6/msp430/include/msp430/flash.h"
volatile unsigned int FCTL3 __asm ("0x012C");
# 25 "/opt/msp430-3.3.6/msp430/include/msp430/timera.h"
volatile unsigned int TA0IV __asm ("0x012E");

volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");
# 127 "/opt/msp430-3.3.6/msp430/include/msp430/timera.h" 3
#line 118
typedef struct __nesc_unnamed4252 {
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
typedef struct __nesc_unnamed4253 {
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
typedef struct __nesc_unnamed4254 {
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
typedef struct __nesc_unnamed4255 {
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
typedef struct __nesc_unnamed4256 {
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
typedef struct __nesc_unnamed4257 {
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
typedef struct __nesc_unnamed4258 {
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
# 56 "/opt/msp430-3.3.6/msp430/include/msp430x16x.h"
volatile unsigned char IE1 __asm ("0x0000");








volatile unsigned char IFG1 __asm ("0x0002");







volatile unsigned char IE2 __asm ("0x0001");









volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 161 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
static __inline void TOSH_wait(void );
#line 174
static __inline void TOSH_uwait(uint16_t u);
#line 196
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
# 58 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/CC2420Const.h"
enum __nesc_unnamed4259 {
  CC2420_TIME_BIT = 4, 
  CC2420_TIME_BYTE = CC2420_TIME_BIT << 3, 
  CC2420_TIME_SYMBOL = 16
};
#line 76
enum __nesc_unnamed4260 {
  CC2420_MIN_CHANNEL = 11, 
  CC2420_MAX_CHANNEL = 26
};
#line 261
enum __nesc_unnamed4261 {
  CP_MAIN = 0, 
  CP_MDMCTRL0, 
  CP_MDMCTRL1, 
  CP_RSSI, 
  CP_SYNCWORD, 
  CP_TXCTRL, 
  CP_RXCTRL0, 
  CP_RXCTRL1, 
  CP_FSCTRL, 
  CP_SECCTRL0, 
  CP_SECCTRL1, 
  CP_BATTMON, 
  CP_IOCFG0, 
  CP_IOCFG1
};
# 44 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline uint8_t TOSH_READ_CC_FIFOP_PIN(void);

static inline uint8_t TOSH_READ_CC_FIFO_PIN(void);
static inline uint8_t TOSH_READ_RADIO_CCA_PIN(void);


static inline uint8_t TOSH_READ_SATELLITE_INTR_2_PIN(void);


static inline void TOSH_SEL_SIMO0_MODFUNC(void);
#line 53
static inline void TOSH_SEL_SIMO0_IOFUNC(void);
static inline void TOSH_SEL_SOMI0_MODFUNC(void);
#line 54
static inline void TOSH_SEL_SOMI0_IOFUNC(void);
static inline void TOSH_SEL_UCLK0_MODFUNC(void);
#line 55
static inline void TOSH_SEL_UCLK0_IOFUNC(void);
static inline void TOSH_SEL_UTXD0_IOFUNC(void);
#line 56
static inline bool TOSH_IS_UTXD0_MODFUNC(void);
#line 56
static inline bool TOSH_IS_UTXD0_IOFUNC(void);
static inline void TOSH_SEL_URXD0_IOFUNC(void);
#line 57
static inline bool TOSH_IS_URXD0_MODFUNC(void);
#line 57
static inline bool TOSH_IS_URXD0_IOFUNC(void);






static inline void TOSH_SEL_CC_SFD_MODFUNC(void);
#line 64
static inline void TOSH_SEL_CC_SFD_IOFUNC(void);
static inline void TOSH_SET_RADIO_CSN_PIN(void);
#line 65
static inline void TOSH_CLR_RADIO_CSN_PIN(void);
#line 65
static inline void TOSH_MAKE_RADIO_CSN_OUTPUT(void);

static inline void TOSH_SET_CC_VREN_PIN(void);
#line 67
static inline void TOSH_CLR_CC_VREN_PIN(void);

static inline void TOSH_SET_CC_RSTN_PIN(void);
#line 69
static inline void TOSH_CLR_CC_RSTN_PIN(void);
static inline void TOSH_SET_RADIO_POWER_PIN(void);
#line 70
static inline void TOSH_CLR_RADIO_POWER_PIN(void);
#line 70
static inline void TOSH_MAKE_RADIO_POWER_OUTPUT(void);
#line 70
static inline void TOSH_SEL_RADIO_POWER_IOFUNC(void);


static inline void TOSH_SET_SATELLITE_CSN_PIN(void);
#line 73
static inline void TOSH_CLR_SATELLITE_CSN_PIN(void);
#line 73
static inline void TOSH_MAKE_SATELLITE_CSN_OUTPUT(void);
static inline void TOSH_SEL_SIMO1_MODFUNC(void);
static inline void TOSH_SEL_SOMI1_MODFUNC(void);
static inline void TOSH_SEL_UCLK1_MODFUNC(void);
static inline void TOSH_SET_SATELLITE_INTR_PIN(void);
#line 77
static inline void TOSH_CLR_SATELLITE_INTR_PIN(void);
static inline uint8_t TOSH_READ_SATELLITE_INTA_PIN(void);
static inline void TOSH_SET_SATELLITE_INTA_2_PIN(void);
#line 79
static inline void TOSH_CLR_SATELLITE_INTA_2_PIN(void);
#line 79
static inline uint8_t TOSH_READ_SATELLITE_INTA_2_PIN(void);
static inline void TOSH_SET_JOIN_PIN(void);
#line 80
static inline void TOSH_CLR_JOIN_PIN(void);
#line 80
static inline uint8_t TOSH_READ_JOIN_PIN(void);


static inline void TOSH_TOGGLE_LED0_PIN(void);

static inline void TOSH_TOGGLE_LED2_PIN(void);



static inline void TOSH_SET_PIN_DIRECTIONS(void );
# 54 "/home/max/tinyos/tinyos-1.x/tos/types/dbg_modes.h"
typedef long long TOS_dbg_mode;



enum __nesc_unnamed4262 {
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
typedef struct __nesc_unnamed4263 {
  void (*tp)(void);
} TOSH_sched_entry_T;

enum __nesc_unnamed4264 {




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
# 149 "/home/max/tinyos/tinyos-1.x/tos/system/tos.h"
static void *nmemcpy(void *to, const void *from, size_t n);
# 28 "/home/max/tinyos/tinyos-1.x/tos/system/Ident.h"
enum __nesc_unnamed4265 {

  IDENT_MAX_PROGRAM_NAME_LENGTH = 16
};






#line 33
typedef struct __nesc_unnamed4266 {

  uint32_t unix_time;
  uint32_t user_hash;
  char program_name[IDENT_MAX_PROGRAM_NAME_LENGTH];
} Ident_t;
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.h"
enum __nesc_unnamed4267 {
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
typedef struct __nesc_unnamed4268 {

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
typedef struct __nesc_unnamed4269 {

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
typedef struct __nesc_unnamed4270 {

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
# 4 "../../../zigzag/IEEE802_15_4/SysCommon/public/ByteOrder.h"
static __inline bool IsTargetHostLSB(void);





static __inline uint16_t ToLSB16(uint16_t a);
# 6 "../../../zigzag/IEEE802_15_4/SysCommon/public/SysCommon.h"
typedef uint16_t TUniData;
# 4 "../../../zigzag/IEEE802_15_4/SysTimer/public/SysTime.h"
typedef uint32_t TSysTime;

typedef uint32_t TMilliSec;
# 4 "../../../zigzag/IEEE802_15_4/Phy/public/PhyConst.h"
enum __nesc_unnamed4271 {

  PHY_AMAX_PHY_PACKET_SIZE = 127, 


  PHY_ATURNAROUND_TIME = 12
};
# 6 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPrivate.h"
enum __nesc_unnamed4272 {
#line 6
  PHY_SIZE_OF_FRAME_LENGTH = 1
};
#line 7
enum __nesc_unnamed4273 {
#line 7
  MAX_PPDU_LENGTH = PHY_SIZE_OF_FRAME_LENGTH + PHY_AMAX_PHY_PACKET_SIZE
};
enum __nesc_unnamed4274 {
#line 9
  CRC16_LENGTH = 2
};
typedef uint8_t _TPhyFrameLength;

typedef uint32_t _TPhyTimeStamp;





#line 15
typedef struct __nesc_unnamed4275 {
  _TPhyTimeStamp timeStamp;
  uint8_t current;
  uint8_t data[MAX_PPDU_LENGTH];
} _TPhyFrame;

typedef uint8_t _TPhyLinkQuality;

typedef uint8_t _TPhyChannel;

typedef uint32_t _TPhyChannelsSupported;




#line 27
typedef struct __nesc_unnamed4276 {
  unsigned tolerance : 2;
  signed txPower : 6;
} _TPhyTransmitPower;







#line 32
typedef enum __nesc_unnamed4277 {
  _PHY_CCA_MODE_FIRST = 1, 
  _PHY_CCA_MODE_1 = 1, 
  _PHY_CCA_MODE_2, 
  _PHY_CCA_MODE_3 = 3, 
  _PHY_CCA_MODE_LAST = 3
} _TPhyCCAMode;

typedef uint8_t _TPhyEnergyLevel;
# 31 "../../../zigzag/IEEE802_15_4/Phy/public/Phy.h"
#line 7
typedef enum __nesc_unnamed4242 {

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
typedef enum __nesc_unnamed4278 {
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
enum __nesc_unnamed4279 {

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
typedef enum __nesc_unnamed4280 {

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
typedef enum __nesc_unnamed4281 {
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
typedef enum __nesc_unnamed4282 {
  MAC_ACK_WAIT_DURATION_54 = 54 * 4, 
  MAC_ACK_WAIT_DURATION_120 = 120
} TMacAckWaitDuration;





#line 161
typedef enum __nesc_unnamed4283 {
  MAC_UNSECURED_MODE = 0x00, 
  MAC_ACL_MODE = 0x01, 
  MAC_SECURED_MODE = 0x02
} TMacSecurityMode;
# 4 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressPrv.h"
enum __nesc_unnamed4284 {
#line 4
  _MAC_PANID_LENGTH = 2
};
enum __nesc_unnamed4285 {
#line 6
  _MAC_SHORT_ADDRESS_LENGTH = 2
};
enum __nesc_unnamed4286 {
#line 8
  _MAC_EXTENDED_ADDRESS_LENGTH = 8
};
typedef uint16_t _TMacPANId;





#line 12
typedef enum __nesc_unnamed4287 {
  _MAC_ADDRESS_NOT_PRESENT = 0x0, 
  _MAC_ADDRESS_SHORT = 0x2, 
  _MAC_ADDRESS_EXTENDED = 0x3
} _TMacAddressMode;

typedef uint16_t _TMacShortAddress;

typedef uint64_t _TMacExtendedAddress;







#line 22
typedef struct __nesc_unnamed4288 {
  _TMacAddressMode mode;
  union __nesc_unnamed4289 {
    _TMacShortAddress shortAddress;
    _TMacExtendedAddress extendedAddress;
  } address;
} _TMacAddress;
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/MacAddress.h"
enum __nesc_unnamed4290 {
#line 6
  MAC_SHORT_ADDRESS_LENGTH = _MAC_SHORT_ADDRESS_LENGTH
};
enum __nesc_unnamed4291 {
#line 8
  MAC_EXTENDED_ADDRESS_LENGTH = _MAC_EXTENDED_ADDRESS_LENGTH
};
enum __nesc_unnamed4292 {
#line 10
  MAC_PANID_LENGTH = _MAC_PANID_LENGTH
};




#line 12
typedef enum __nesc_unnamed4293 {
  MAC_ADDRESS_NOT_PRESENT = _MAC_ADDRESS_NOT_PRESENT, 
  MAC_ADDRESS_SHORT = _MAC_ADDRESS_SHORT, 
  MAC_ADDRESS_EXTENDED = _MAC_ADDRESS_EXTENDED
} TMacAddressMode;

typedef _TMacPANId TMacPANId;

typedef _TMacShortAddress TMacShortAddress;

typedef _TMacExtendedAddress TMacExtendedAddress;

typedef _TMacAddress TMacAddress;
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormat.h"
enum __nesc_unnamed4294 {
#line 7
  MAC_FRAME_CONTROL_LENGTH = 2
};






#line 8
typedef struct __nesc_unnamed4295 {
  unsigned frameType : 3;
  unsigned securityEnabled : 1;
  unsigned framePending : 1;
  unsigned ackRequest : 1;
  unsigned intraPAN : 1;
  unsigned  : 1;
} TMacLSBFrameControl;






#line 17
typedef struct __nesc_unnamed4296 {
  unsigned  : 2;
  unsigned dstAddressMode : 2;
  unsigned  : 2;
  unsigned srcAddressMode : 2;
} TMacMSBFrameControl;










#line 24
typedef struct __nesc_unnamed4297 {
  union __nesc_unnamed4298 {
    TMacLSBFrameControl value;
    uint8_t raw;
  } lsb;
  union __nesc_unnamed4299 {
    TMacMSBFrameControl value;
    uint8_t raw;
  } msb;
} TMacFrameControl;
typedef TMacFrameControl TMacFrameControlField;

enum __nesc_unnamed4300 {
#line 36
  MAC_SEQUENCE_NUMBER_LENGTH = 1
};
#line 37
typedef uint8_t _TMacSequenceNumber;



#line 38
typedef union __nesc_unnamed4301 {
  uint8_t raw[MAC_SEQUENCE_NUMBER_LENGTH];
  _TMacSequenceNumber value;
} TMacSequenceNumberField;




#line 43
typedef union __nesc_unnamed4302 {
  uint8_t raw[MAC_PANID_LENGTH];
  TMacPANId value;
} TMacPANIdField;




#line 48
typedef union __nesc_unnamed4303 {
  uint8_t raw[MAC_SHORT_ADDRESS_LENGTH];
  TMacShortAddress value;
} TMacShortAddressField;




#line 53
typedef union __nesc_unnamed4304 {
  uint8_t raw[MAC_EXTENDED_ADDRESS_LENGTH];
  TMacExtendedAddress value;
} TMacExtendedAddressField;




#line 58
typedef union __nesc_unnamed4305 {
  TMacShortAddressField _short;
  TMacExtendedAddressField _extended;
} TMacAddressField;

typedef uint8_t _TMacPayloadLength;
enum __nesc_unnamed4306 {
#line 64
  MAC_PAYLOAD_LENGTH = MAC_AMAX_MAC_FRAME_SIZE
};


#line 65
typedef struct __nesc_unnamed4307 {
  _TMacPayloadLength length;
  uint8_t raw[MAC_PAYLOAD_LENGTH];
} TMacPayloadField;

enum __nesc_unnamed4308 {
#line 70
  MAC_FRAME_CHECK_SEQUENCE_LENGTH = 2
};
#line 71
typedef uint16_t _TMacFrameCheckSequence;



#line 72
typedef union __nesc_unnamed4309 {
  uint8_t raw[MAC_FRAME_CHECK_SEQUENCE_LENGTH];
  _TMacFrameCheckSequence value;
} TMacFrameCheckSequenceField;

typedef uint8_t _TMacRawFrameLength;

typedef uint32_t _TMacTimeStamp;

typedef uint8_t _TMacLinkQuality;
#line 93
#line 83
typedef struct __nesc_unnamed4310 {
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
typedef enum __nesc_unnamed4311 {
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
typedef enum __nesc_unnamed4312 {
  MAC_SEND_MODE_DIRECT, 
  MAC_SEND_MODE_CSMA
} TMacSendMode;
# 4 "../../../zigzag/IEEE802_15_4/MacCommon/public/MacSecurity.h"
typedef uint8_t TACLEntry;
# 4 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/MacSuperframe.h"
typedef uint8_t TMacBeaconOrder;

typedef uint8_t TMacSuperframeOrder;







#line 8
typedef enum __nesc_unnamed4313 {
  MAC_SLOT_TYPE_UNKNOWN = 0, 
  MAC_SLOT_TYPE_FREE = 1, 
  MAC_SLOT_TYPE_BEACON = 2, 
  MAC_SLOT_TYPE_CAP = 3, 
  MAC_SLOT_TYPE_CFP = 4
} TMacSlotType;
enum __nesc_unnamed4314 {
#line 15
  SLOT_TYPE_NUMBER = 5
};
#line 31
#line 26
typedef enum __nesc_unnamed4315 {
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
typedef enum __nesc_unnamed4316 {
  MAC_BATT_LIFE_EXT_PERIOD_6 = 6, 
  MAC_BATT_LIFE_EXT_PERIOD_8 = 8
} TMacBattLifeExtPeriods;










#line 9
typedef enum __nesc_unnamed4317 {
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
typedef enum __nesc_unnamed4318 {
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
enum __nesc_unnamed4319 {
  MAC_DEVICE_TYPE_RFD = 0, 
  MAC_DEVICE_TYPE_FFD = 1
};









#line 9
typedef struct __nesc_unnamed4320 {
  unsigned int alternatePANCoordinator : 1;
  unsigned int deviceType : 1;
  unsigned int powerSource : 1;
  unsigned int receiverOnWhenIdle : 1;
  unsigned int  : 2;
  unsigned int securityCapability : 1;
  unsigned int allocateAddress : 1;
} TCapabilityInformation;




#line 19
typedef enum __nesc_unnamed4321 {
  MAC_DISASSOCIATE_BY_COORD = 0x01, 
  MAC_DISASSOCIATE_BY_DEVICE = 0x02
} TDisassociateReason;
#line 54
#line 24
typedef enum __nesc_unnamed4322 {
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
typedef struct __nesc_unnamed4323 {

  unsigned shortAddrNum : 3;
  unsigned  : 1;
  unsigned extAddrNum : 3;
} 
TMacPendAddrSpec;


typedef uint8_t *TMacAddrList;










#line 19
typedef struct __nesc_unnamed4324 {

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
typedef struct __nesc_unnamed4325 {

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
typedef enum __nesc_unnamed4326 {

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
typedef enum __nesc_unnamed4327 {
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
typedef struct __nesc_unnamed4328 {


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
typedef enum __nesc_unnamed4329 {

  ZIGBEE_COORDINATOR = 0, 
  ZIGBEE_ROUTER = 1, 
  ZIGBEE_END_DEVICE = 2
} NwkDeviceType;









#line 74
typedef enum __nesc_unnamed4330 {

  NWK_PARENT = 0, 
  NWK_CHILD = 1, 
  NWK_SIBLING = 2, 
  NWK_OTHER = 3
} 
NwkRelationship;
#line 106
#line 87
typedef struct __nesc_unnamed4331 {

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
typedef enum __nesc_unnamed4332 {

  ROUTE_ACTIVE = 0, 
  ROUTE_DISCOVERY_UNDERWAY = 1, 
  ROUTE_DISCOVERY_FAILED = 2, 
  ROUTE_INACTIVE = 3
} 
NwkRouteStatus;









#line 120
typedef struct __nesc_unnamed4333 {

  NwkAddr destinationAddr;
  NwkAddr nextHopAddr;
  NwkRouteStatus status;
} NwkRoute;


typedef TCapabilityInformation NwkCapabilityInfo;









#line 131
typedef enum __nesc_unnamed4334 {

  NWK_OFFLINE, 
  NWK_JOINED_AS_ED, 
  NWK_JOINED_AS_ROUTER, 
  NWK_ROUTING
} 
NwkOnlineStatus;


enum __nesc_unnamed4335 {

  NWK_DATA_FRAME = 0, 
  NWK_COMMAND_FRAME = 1
};


enum __nesc_unnamed4336 {

  SUPPRESS_DISCOVERY = 0, 
  ENABLE_DISCOVERY = 1, 
  FORCE_DISCOVERY = 2
};

enum __nesc_unnamed4337 {

  NWK_BROADCAST = 0xffff
};
#line 178
#line 160
typedef struct __nesc_unnamed4338 {

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
typedef struct __nesc_unnamed4339 {

  uint8_t routeRequestID;
  NwkAddr sourceAddr;
  NwkAddr senderAddr;
  uint8_t forwardCost;
  uint8_t residualCost;
  uint16_t expirationTime;
} NwkRouteDiscovery;
# 16 "../../../zigzag/ZigBee/implementation/constants.h"
enum __nesc_unnamed4340 {
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

static struct info_param_t info_param __attribute((section(".infomem")))  = { 
.MAC_ADDRESS = 1003, 
.Z_CHANNELS = 1 << 15, 
.Z_PAN_ID = 0xf3, 
.Z_MAX_CHILDREN = 4, 
.Z_MAX_ROUTERS = 4, 
.Z_MAX_DEPTH = 7, 
.Z_BEACON_ORDER = 6, 
.Z_SUPERFRAME_ORDER = 1, 
.Z_SCAN_ORDER = 6, 
.ALIVE_SEND_PERIOD = 10, 
.ALIVE_SEND_ATTEMPTS = 3 };
# 17 "../../../zigzag/ZigBee/implementation/NwkBeacon.h"
#line 6
typedef struct __nesc_unnamed4341 {

  unsigned protocolID : 8;
  unsigned stkProf : 4;
  unsigned prVer : 4;
  unsigned  : 2;
  unsigned rtrCap : 1;
  unsigned devDep : 4;
  unsigned endCap : 1;
  IEEEAddr extendedPANID;
  uint8_t txOffset[3];
} __attribute((packed))  NwkBeaconPayload;
# 12 "../../../zigzag/IEEE802_15_4/MacData/public/MacData.h"
#line 5
typedef struct __nesc_unnamed4342 {

  unsigned  : 4;
  unsigned secure : 1;
  unsigned indirect : 1;
  unsigned gts : 1;
  unsigned acknowledged : 1;
} TxOptions;
# 4 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/SysTimer.h"
enum __nesc_unnamed4343 {
  MAX_SYS_JIFFY_HI = 0x7fffUL, 
  MAX_SYS_JIFFY_LO = 0xffffUL, 
  MAX_SYS_JIFFY = (uint32_t )(MAX_SYS_JIFFY_HI << 16) | MAX_SYS_JIFFY_LO
};
# 4 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420.h"
const uint16_t CC2420_EXTADDR_ADDRESS = 0x160;
const uint16_t CC2420_PANID_ADDRESS = 0x168;
const uint16_t CC2420_SHORTADDR_ADDRESS = 0x16a;
#line 36
#line 24
typedef union __nesc_unnamed4344 {
  struct __nesc_unnamed4345 {
    unsigned int  : 1;
    unsigned int rssi_valid : 1;
    unsigned int lock : 1;
    unsigned int tx_active : 1;
    unsigned int enc_busy : 1;
    unsigned int tx_underflow : 1;
    unsigned int xosc16m_stable : 1;
    unsigned int  : 1;
  } value;
  uint8_t raw;
} TCC2420_STATUS;

const uint8_t CC2420_MAIN_ADDRESS = 0x10;
#line 50
#line 39
typedef union __nesc_unnamed4346 {
  struct __nesc_unnamed4347 {
    unsigned xosc16m_bypass : 1;
    unsigned reservW0 : 10;
    unsigned fs_resetn : 1;
    unsigned mod_resetn : 1;
    unsigned demod_resetn : 1;
    unsigned enc_resetn : 1;
    unsigned resetn : 1;
  } value;
  uint16_t raw;
} TCC2420_MAIN;
const TCC2420_MAIN CC2420_MAIN_DEFAULT = 
{ { .xosc16m_bypass = 0, 
.reservW0 = 0, 
.fs_resetn = 1, 
.mod_resetn = 1, 
.demod_resetn = 1, 
.enc_resetn = 1, 
.resetn = 1 } };


const uint8_t CC2420_MDMCTRL0_ADDRESS = 0x11;
#line 75
#line 62
typedef union __nesc_unnamed4348 {
  struct __nesc_unnamed4349 {
    unsigned preamble_length : 4;
    unsigned autoack : 1;
    unsigned autocrc : 1;
    unsigned cca_mode : 2;
    unsigned cca_hyst : 3;
    unsigned adr_decode : 1;
    unsigned pan_coordinator : 1;
    unsigned reserved_frame_mode : 1;
    unsigned reservW0 : 2;
  } value;
  uint16_t raw;
} TCC2420_MDMCTRL0;
const TCC2420_MDMCTRL0 CC2420_MDMCTRL0_DEFAULT = 
{ { .preamble_length = 0x2, 
.autoack = 1, 
.autocrc = 1, 
.cca_mode = 3, 
.cca_hyst = 2, 
.adr_decode = 1, 
.pan_coordinator = 0, 
.reserved_frame_mode = 0, 
.reservW0 = 0 } };


const uint8_t CC2420_MDMCTRL1_ADDRESS = 0x12;










#line 89
typedef union __nesc_unnamed4350 {
  struct __nesc_unnamed4351 {
    unsigned rx_mode : 2;
    unsigned tx_mode : 2;
    unsigned modulation_mode : 1;
    unsigned demod_avg_mode : 1;
    unsigned corr_thr : 5;
    unsigned reservW0 : 5;
  } value;
  uint16_t raw;
} TCC2420_MDMCTRL1;
const TCC2420_MDMCTRL1 CC2420_MDMCTRL1_DEFAULT = 
{ { .rx_mode = 0, 
.tx_mode = 0, 
.modulation_mode = 0, 
.demod_avg_mode = 0, 
.corr_thr = 20, 
.reservW0 = 0 } };


const uint8_t CC2420_RSSI_ADDRESS = 0x13;






#line 110
typedef union __nesc_unnamed4352 {
  struct __nesc_unnamed4353 {
    signed rssi_val : 8;
    signed cca_thr : 8;
  } value;
  uint16_t raw;
} TCC2420_RSSI;
const TCC2420_RSSI CC2420_RSSI_DEFAULT = 
{ { .cca_thr = -32, 
.rssi_val = -128 } };


const uint8_t CC2420_SYNCWORD_ADDRESS = 0x14;





#line 123
typedef union __nesc_unnamed4354 {
  struct __nesc_unnamed4355 {
    unsigned int syncword : 16;
  } value;
  uint16_t raw;
} TCC2420_SYNCWORD;
const TCC2420_SYNCWORD CC2420_SYNCWORD_DEFAULT = 
{ { 
.syncword = 0xA70F } };


const uint8_t CC2420_TXCTRL_ADDRESS = 0x15;
#line 146
#line 135
typedef union __nesc_unnamed4356 {
  struct __nesc_unnamed4357 {
    unsigned pa_level : 5;
    unsigned reservW1 : 1;
    unsigned pa_current : 3;
    unsigned txmix_current : 2;
    unsigned txmix_cap_array : 2;
    unsigned tx_turnaround : 1;
    unsigned txmixbuf_cur : 2;
  } value;
  uint16_t raw;
} TCC2420_TXCTRL;
const TCC2420_TXCTRL CC2420_TXCTRL_DEFAULT = 
{ { .pa_level = 31, 
.reservW1 = 1, 
.pa_current = 3, 
.txmix_current = 0, 
.txmix_cap_array = 0, 
.tx_turnaround = 1, 
.txmixbuf_cur = 1 } };


const uint8_t CC2420_RXCTRL0_ADDRESS = 0x16;
#line 170
#line 158
typedef union __nesc_unnamed4358 {
  struct __nesc_unnamed4359 {
    unsigned int low_lna_current : 2;
    unsigned int med_lna_current : 2;
    unsigned int high_lna_current : 2;
    unsigned int low_lna_gain : 2;
    unsigned int med_lna_gain : 2;
    unsigned int high_lna_gain : 2;
    unsigned int rxmixbuf_cur : 2;
    unsigned int reservW0 : 2;
  } value;
  uint16_t raw;
} TCC2420_RXCTRL0;
const TCC2420_RXCTRL0 CC2420_RXCTRL0_DEFAULT = 
{ { .low_lna_current = 1, 
.med_lna_current = 1, 
.high_lna_current = 2, 
.low_lna_gain = 3, 
.med_lna_gain = 2, 
.high_lna_gain = 0, 
.rxmixbuf_cur = 1, 
.reservW0 = 0 } };


const uint8_t CC2420_RXCTRL1_ADDRESS = 0x17;
#line 198
#line 183
typedef union __nesc_unnamed4360 {
  struct __nesc_unnamed4361 {
    unsigned int rxmix_current : 2;
    unsigned int rxmix_vcm : 2;
    unsigned int rxmix_tail : 2;
    unsigned int lna_cap_array : 2;
    unsigned int med_hgm : 1;
    unsigned int high_hgm : 1;
    unsigned int med_lowgain : 1;
    unsigned int low_lowgain : 1;
    unsigned int rxbpf_midcur : 1;
    unsigned int rxbpf_locur : 1;
    unsigned int reservW0 : 2;
  } value;
  uint16_t raw;
} TCC2420_RXCTRL1;
const TCC2420_RXCTRL1 CC2420_RXCTRL1_DEFAULT = 
{ { .rxmix_current = 2, 
.rxmix_vcm = 1, 
.rxmix_tail = 1, 
.lna_cap_array = 1, 
.med_hgm = 0, 
.high_hgm = 1, 
.med_lowgain = 0, 
.low_lowgain = 1, 
.rxbpf_midcur = 0, 
.rxbpf_locur = 0, 
.reservW0 = 0 } };


const uint8_t CC2420_FSCTRL_ADDRESS = 0x18;










#line 214
typedef union __nesc_unnamed4362 {
  struct __nesc_unnamed4363 {
    unsigned int freq : 10;
    unsigned int lock_status : 1;
    unsigned int lock_length : 1;
    unsigned int cal_running : 1;
    unsigned int cal_done : 1;
    unsigned int lock_thr : 2;
  } value;
  uint16_t raw;
} TCC2420_FSCTRL;
const TCC2420_FSCTRL CC2420_FSCTRL_DEFAULT = 
{ { .freq = 357, 
.lock_status = 0, 
.lock_length = 0, 
.cal_running = 0, 
.cal_done = 0, 
.lock_thr = 1 } };


const uint8_t CC2420_SECCTRL0_ADDRESS = 0x19;
#line 247
#line 235
typedef union __nesc_unnamed4364 {
  struct __nesc_unnamed4365 {
    unsigned int sec_mode : 2;
    unsigned int sec_m : 3;
    unsigned int sec_rxkeysel : 1;
    unsigned int sec_txkeysel : 1;
    unsigned int sec_sakeysel : 1;
    unsigned int sec_cbc_head : 1;
    unsigned int rxfifo_protection : 1;
    unsigned int reservW0 : 6;
  } value;
  uint16_t raw;
} TCC2420_SECCTRL0;
const TCC2420_SECCTRL0 CC2420_SECCTRL0_DEFAULT = 
{ { .sec_mode = 0, 
.sec_m = 1, 
.sec_rxkeysel = 0, 
.sec_txkeysel = 1, 
.sec_sakeysel = 1, 
.sec_cbc_head = 1, 
.rxfifo_protection = 0, 
.reservW0 = 0 } };


const uint8_t CC2420_SECCTRL1_ADDRESS = 0x1A;








#line 260
typedef union __nesc_unnamed4366 {
  struct __nesc_unnamed4367 {
    unsigned int sec_rxl : 7;
    unsigned int reserv1W0 : 1;
    unsigned int sec_txl : 7;
    unsigned int reserv2W0 : 1;
  } value;
  uint16_t raw;
} TCC2420_SECCTRL1;
const TCC2420_SECCTRL1 CC2420_SECCTRL1_DEFAULT = 
{ { .sec_rxl = 0, 
.reserv1W0 = 0, 
.sec_txl = 0, 
.reserv2W0 = 0 } };


const uint8_t CC2420_BATTMON_ADDRESS = 0x1B;








#line 277
typedef union __nesc_unnamed4368 {
  struct __nesc_unnamed4369 {
    unsigned int battmon_voltage : 5;
    unsigned int battmon_en : 1;
    unsigned int batt_ok : 1;
    unsigned int reservW0 : 9;
  } value;
  uint16_t raw;
} TCC2420_BATTMON;
const TCC2420_BATTMON CC2420_BATTMON_DEFAULT = 
{ { .battmon_voltage = 0, 
.battmon_en = 0, 
.batt_ok = 0, 
.reservW0 = 0 } };


const uint8_t CC2420_IOCFG0_ADDRESS = 0x1C;
#line 305
#line 294
typedef union __nesc_unnamed4370 {
  struct __nesc_unnamed4371 {
    unsigned int fifop_thr : 7;
    unsigned int cca_polarity : 1;
    unsigned int sfd_polarity : 1;
    unsigned int fifop_polarity : 1;
    unsigned int fifo_polarity : 1;
    unsigned int bcn_accept : 1;
    unsigned int reservW0 : 4;
  } value;
  uint16_t raw;
} TCC2420_IOCFG0;
const TCC2420_IOCFG0 CC2420_IOCFG0_DEFAULT = 
{ { .fifop_thr = 127, 
.cca_polarity = 0, 
.sfd_polarity = 0, 
.fifop_polarity = 0, 
.fifo_polarity = 0, 
.bcn_accept = 1, 
.reservW0 = 0 } };


const uint8_t CC2420_IOCFG1_ADDRESS = 0x1D;








#line 317
typedef union __nesc_unnamed4372 {
  struct __nesc_unnamed4373 {
    unsigned int ccamux : 5;
    unsigned int sfdmux : 5;
    unsigned int hssd_src : 3;
    unsigned int reservW0 : 3;
  } value;
  uint16_t raw;
} TCC2420_IOCFG1;
const TCC2420_IOCFG1 CC2420_IOCFG1_DEFAULT = 
{ { .ccamux = 0, 
.sfdmux = 0, 
.hssd_src = 0, 
.reservW0 = 0 } };









#line 334
typedef union __nesc_unnamed4374 {
  struct __nesc_unnamed4375 {
    unsigned int fsm_cur_state : 6;
    unsigned int  : 10;
  } value;
  uint16_t raw;
} TCC2420_FSMSTATE;
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430usart.h"
#line 31
typedef enum __nesc_unnamed4376 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacSlottedCAP.h"
#line 4
typedef enum __nesc_unnamed4377 {
  FFMAC_PARENT_CAP = 0, 
  FFMAC_OWN_CAP = 2
} TMacCAPType;
# 4 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacOwnCAP.h"
enum __nesc_unnamed4378 {

  MAC_OWN_SF = 0U
};
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
#line 74
static   void MSP430TimerM$CaptureA1$default$captured(uint16_t arg_0x40750170);
#line 32
static   uint16_t MSP430TimerM$CaptureB3$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB3$default$captured(uint16_t arg_0x40750170);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA1$default$fired(void);
#line 30
static   void MSP430TimerM$CompareB3$setEvent(uint16_t arg_0x4073c680);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB6$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB6$default$captured(uint16_t arg_0x40750170);
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
#line 56
static   void MSP430TimerM$CaptureB1$clearOverflow(void);
#line 51
static   bool MSP430TimerM$CaptureB1$isOverflowPending(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB1$default$fired(void);
# 36 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void MSP430TimerM$ControlB1$setControlAsCapture(bool arg_0x407417d0);
#line 30
static   MSP430CompareControl_t MSP430TimerM$ControlB1$getControl(void);







static   void MSP430TimerM$ControlB1$enableEvents(void);
static   void MSP430TimerM$ControlB1$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB1$clearPendingInterrupt(void);

static   void MSP430TimerM$ControlB1$setControl(MSP430CompareControl_t arg_0x40741010);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureA2$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureA2$default$captured(uint16_t arg_0x40750170);
#line 32
static   uint16_t MSP430TimerM$CaptureB4$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB4$default$captured(uint16_t arg_0x40750170);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB2$getControl(void);







static   void MSP430TimerM$ControlB2$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB2$setControlAsCompare(void);



static   void MSP430TimerM$ControlB2$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB2$clearPendingInterrupt(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA2$default$fired(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerA$setClockSource(uint16_t arg_0x40735dd8);
#line 38
static   void MSP430TimerM$TimerA$disableEvents(void);
#line 32
static   void MSP430TimerM$TimerA$clearOverflow(void);


static   void MSP430TimerM$TimerA$setMode(int arg_0x40735010);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB4$setEvent(uint16_t arg_0x4073c680);
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
#line 74
static   void MSP430TimerM$CaptureA0$default$captured(uint16_t arg_0x40750170);
#line 32
static   uint16_t MSP430TimerM$CaptureB2$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB2$default$captured(uint16_t arg_0x40750170);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA0$default$fired(void);
#line 30
static   void MSP430TimerM$CompareB2$setEvent(uint16_t arg_0x4073c680);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB5$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB5$default$captured(uint16_t arg_0x40750170);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB3$getControl(void);







static   void MSP430TimerM$ControlB3$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB3$setControlAsCompare(void);



static   void MSP430TimerM$ControlB3$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB3$clearPendingInterrupt(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerB$setClockSource(uint16_t arg_0x40735dd8);
#line 30
static   uint16_t MSP430TimerM$TimerB$read(void);

static   void MSP430TimerM$TimerB$clearOverflow(void);
#line 31
static   bool MSP430TimerM$TimerB$isOverflowPending(void);



static   void MSP430TimerM$TimerB$setMode(int arg_0x40735010);




static   void MSP430TimerM$TimerB$setInputDivider(uint16_t arg_0x40733358);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB5$setEvent(uint16_t arg_0x4073c680);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB0$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB0$default$captured(uint16_t arg_0x40750170);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB6$setEvent(uint16_t arg_0x4073c680);

static   void MSP430TimerM$CompareB0$setEventFromNow(uint16_t arg_0x40754010);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB6$getControl(void);







static   void MSP430TimerM$ControlB6$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB6$setControlAsCompare(void);



static   void MSP430TimerM$ControlB6$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB6$clearPendingInterrupt(void);
# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
static  void ZigCoordM$NLME_Leave$indication(IEEEAddr arg_0x408d6548);
# 25 "../../../zigzag/ZigBee/interface/NLME_NetworkFormation.nc"
static  void ZigCoordM$IZigNetFormation$confirm(NwkStatus arg_0x408b09a8);
# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void ZigCoordM$IZigReset$confirm(NwkStatus arg_0x408d5c68);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ZigCoordM$StdControl$init(void);






static  result_t ZigCoordM$StdControl$start(void);
# 38 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t NLME_JoinParentM$IMacASSOCIATE$Indication(TMacExtendedAddress arg_0x408ef610, 
TCapabilityInformation arg_0x408ef7c0, 
bool arg_0x408ef958, 
TACLEntry arg_0x408efaf8);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCOMM_STATUS.nc"
static  void NLME_JoinParentM$IMacCOMM_STATUS$Indication(TMacPANId arg_0x409059c0, 
TMacAddress arg_0x40905b60, TMacAddress arg_0x40905cf0, 
TMacStatus arg_0x40905e90);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NLME_JoinParentM$Reset$reset(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t NLME_JoinParentM$ITimerSymbol$Fired(TUniData arg_0x4090ac28);
# 15 "../../../zigzag/ZigBee/interface/NLME_NetworkFormation.nc"
static  void NLME_NetworkFormationM$NLME_NetworkFormation$request(
uint32_t arg_0x408aecd0, 
uint8_t arg_0x408aee68, 
uint8_t arg_0x408b0030, 
uint8_t arg_0x408b01c8, 

PanID_t arg_0x408b0358, 
bool arg_0x408b0508);
# 22 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
static  result_t NLME_NetworkFormationM$IMacSTART$Confirm(TMacStatus arg_0x40928820);
# 66 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  uint8_t NIBM$NIB$getNwkMaxRouters(void);
#line 191
static  void NIBM$NIB$setOnline(NwkOnlineStatus arg_0x408bf800);
#line 20
static  uint8_t NIBM$NIB$getNwkSequenceNumber(void);
#line 55
static  void NIBM$NIB$setNwkMaxDepth(uint8_t arg_0x408c7328);
#line 204
static  void NIBM$NIB$setBeaconOffset(TSysTime arg_0x408da228);
#line 192
static  NwkOnlineStatus NIBM$NIB$getOnlineStatus(void);



static  uint8_t NIBM$NIB$getDepth(void);
#line 65
static  void NIBM$NIB$setNwkMaxRouters(uint8_t arg_0x408c7aa0);
#line 197
static  void NIBM$NIB$setDepth(uint8_t arg_0x408bd620);
#line 47
static  void NIBM$NIB$setNwkMaxChildren(uint8_t arg_0x408c8b40);
#line 201
static  void NIBM$NIB$setChannel(uint8_t arg_0x408bdd90);



static  TSysTime NIBM$NIB$getBeaconOffset(void);
#line 19
static  void NIBM$NIB$setNwkSequenceNumber(uint8_t arg_0x408ca478);
#line 200
static  uint8_t NIBM$NIB$getChannel(void);
#line 48
static  uint8_t NIBM$NIB$getNwkMaxChildren(void);







static  uint8_t NIBM$NIB$getNwkMaxDepth(void);
#line 193
static  bool NIBM$NIB$isJoined(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NIBM$Reset$reset(void);
#line 46
static  void NwkAddressingM$Reset$reset(void);
# 15 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  result_t NwkAddressingM$NwkAddressing$allocEdAddress(NwkAddr *arg_0x40914ba8);



static  bool NwkAddressingM$NwkAddressing$hasVacantAddrs(void);
static  bool NwkAddressingM$NwkAddressing$hasVacantEdAddrs(void);




static  void NwkAddressingM$NwkAddressing$getDirection(NwkAddr arg_0x40913dd8, NwkRelationship *arg_0x40911010, NwkAddr *arg_0x409111b0);
#line 17
static  result_t NwkAddressingM$NwkAddressing$allocRouterAddress(NwkAddr *arg_0x40913068);



static  bool NwkAddressingM$NwkAddressing$hasVacantRouterAddrs(void);
# 21 "../../../zigzag/ZigBee/implementation/NwkBeaconParentM.nc"
static  void NwkBeaconParentM$updateBeacon(void);
# 16 "../../../zigzag/ZigBee/interface/NLME_PermitJoining.nc"
static  NwkStatus NLME_PermitJoiningM$NLME_PermitJoining$request(
uint8_t arg_0x408cb030);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t NLME_PermitJoiningM$ITimerSymbol$Fired(TUniData arg_0x4090ac28);
# 15 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void NLME_ResetM$NLME_Reset$request(void);
static  void NLME_ResetM$MacReset$confirm(NwkStatus arg_0x408d5c68);
#line 15
static  void NwkResetMacSingleM$NLME_Reset$request(void);
# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t NwkResetMacSingleM$IMacRESET$Confirm(TMacStatus arg_0x409c24b8);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NeighborTableM$Reset$reset(void);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NeighborTableM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x408dea50);










static  result_t NeighborTableM$NeighborTable$remove(NwkNeighbor *arg_0x408dd580);
#line 15
static  result_t NeighborTableM$NeighborTable$update(NwkNeighbor arg_0x408de578);
#line 29
static  result_t NeighborTableM$NeighborTable$getByAddr(TMacAddress arg_0x408deee0, NwkNeighbor **arg_0x408dd0c8);
# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t RouterM$LowerIf$Confirm(uint8_t arg_0x409e1a00, TMacStatus arg_0x409e1b90);
#line 16
static  result_t RouterM$LowerIf$Indication(TMacPANId arg_0x409e6810, 
TMacAddress arg_0x409e69a8, 
TMacPANId arg_0x409e6b48, 
TMacAddress arg_0x409e6ce0, 
uint8_t arg_0x409e6e78, 
uint8_t *arg_0x409e1068, 
TMacLinkQuality arg_0x409e1208, 
bool arg_0x409e13a0, 
uint8_t arg_0x409e1538);
# 15 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static  uint8_t RouterM$newMsduHandle(void);
# 23 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
static  result_t RouterM$NLDE_Data$request(
NwkAddr arg_0x408e5938, 
uint8_t arg_0x408e5ad0, 
uint8_t *arg_0x408e5c80, 
uint8_t arg_0x408e5e18, 
uint8_t arg_0x408e4010, 
uint8_t arg_0x408e41a8, 
bool arg_0x408e4340);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void RouterM$Reset$reset(void);
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t RouterM$UpperIf$default$Request(TMacPANId arg_0x409e2778, 
TMacAddress arg_0x409e2910, 
TMacPANId arg_0x409e2ab0, 
TMacAddress arg_0x409e2c48, 
uint8_t arg_0x409e2de0, 
const uint8_t *const arg_0x409e6010, 
uint8_t arg_0x409e61a8, 
TxOptions arg_0x409e6340);
# 7 "../../../zigzag/ZigBee/implementation/NwkFrame.nc"
static  uint8_t NwkFrameM$NwkFrame$mkDataFrameHeader(uint8_t *arg_0x409fc010, 
NwkAddr arg_0x409fc1a0, 
uint8_t arg_0x409fc330, 
uint8_t arg_0x409fc4c8, 
bool arg_0x409fc670);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t TimerSymbol2M$IStdControl$init(void);






static  result_t TimerSymbol2M$IStdControl$start(void);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t TimerSymbol2M$TimerMilli$default$fired(
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a643d0);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbol2M$AlarmCompare$fired(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime TimerSymbol2M$ILocalTime$Read(void);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void TimerSymbol2M$AlarmTimer$overflow(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t TimerSymbol2M$ITimerSymbol$Stop(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a667c0);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t TimerSymbol2M$ITimerSymbol$SetOneShot(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a667c0, 
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868);
#line 6
static  result_t TimerSymbol2M$ITimerSymbol$SetPeriodic(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a667c0, 
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TSysTime arg_0x4090c0c0, TUniData arg_0x4090c248);









static  result_t TimerSymbol2M$ITimerSymbol$default$Fired(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a667c0, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TUniData arg_0x4090ac28);
#line 11
static  bool TimerSymbol2M$ITimerSymbol$IsSet(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a667c0);

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
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$Stop(
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
uint8_t arg_0x40aac3d8);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$default$Fired(
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
uint8_t arg_0x40aac3d8, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
TUniData arg_0x40a53dd8);
#line 7
static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$SetOneShotAt(
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
uint8_t arg_0x40aac3d8, 
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
TSysTime arg_0x40a317e0, TUniData arg_0x40a31968);
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
static  result_t LocalTime64M$ITimerSymbol$Fired(TUniData arg_0x4090ac28);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  uint64_t LocalTime64M$ILocalTime64$getLocalTimeAt(TSysTime arg_0x40a39010);
static  void LocalTime64M$ILocalTime64$setLocalTimeAt(TSysTime arg_0x40a39490, uint64_t arg_0x40a39618);

static  uint64_t LocalTime64M$ILocalTime64$getLocalTime(void);
static  void LocalTime64M$ILocalTime64$setLocalTime(uint64_t arg_0x40a39da0);
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   uint32_t TimeCastM$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a346a0);
#line 10
static   TMilliSec TimeCastM$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a35558);
#line 7
static   TSysTime TimeCastM$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a37d70);
#line 6
static   TSysTime TimeCastM$ITimeCast$MillisToSymbols(TMilliSec arg_0x40a378b8);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t PhyCC2420M$IStdControl$init(void);






static  result_t PhyCC2420M$IStdControl$start(void);
# 49 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
static   result_t PhyCC2420M$IChipconRAM$writeDone(uint16_t arg_0x40b90e80, uint8_t arg_0x40b8f030, uint8_t *arg_0x40b8f1d0);
# 11 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
static  result_t PhyCC2420M$IPhyAttr$SetAutoAck(bool arg_0x40b38dd0);
#line 7
static  result_t PhyCC2420M$IPhyAttr$SetextendedAddress(uint64_t arg_0x40b384b0);
#line 5
static  result_t PhyCC2420M$IPhyAttr$SetpanId(uint16_t arg_0x40b38010);
#line 3
static  result_t PhyCC2420M$IPhyAttr$SetshortAddress(uint16_t arg_0x40b39ad8);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static   result_t PhyCC2420M$IPhyTxDATA$Request(
# 9 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b4e010, 
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
const TPhyPoolHandle arg_0x40b2ab78, 
const TUniData arg_0x40b2ad30);

static  result_t PhyCC2420M$IPhyTxDATA$default$Confirm(
# 9 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b4e010, 
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
TPhyPoolHandle arg_0x40b2c1d8, 
TPhyStatus arg_0x40b2c378, 
TUniData arg_0x40b2c510);
# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t PhyCC2420M$IChipconFIFOSignal$fired(void);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
static  TPhyStatus PhyCC2420M$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b4a180);
# 18 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static   void PhyCC2420M$FreePoolItem(TPhyPoolHandle arg_0x40b77408);
# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t PhyCC2420M$IChipconFIFOP$fired(void);
# 53 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
static   result_t PhyCC2420M$IChipconSFD$captured(uint16_t arg_0x40b95360);
# 50 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
static   result_t PhyCC2420M$IChipconFIFO$TXFIFODone(uint8_t arg_0x40b97118, uint8_t *arg_0x40b972b8);
#line 39
static   result_t PhyCC2420M$IChipconFIFO$RXFIFODone(uint8_t arg_0x40b989a8, uint8_t *arg_0x40b98b48);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t PhyCC2420M$IPhySET_TRX_STATE$Request(
# 13 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b79340, 
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960);

static  result_t PhyCC2420M$IPhySET_TRX_STATE$default$Confirm(
# 13 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b79340, 
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
TPhyStatus arg_0x40b47df8, TUniData arg_0x40b46010);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
static   result_t PhyCC2420M$IPhyTxFIFO$Write(
# 17 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b78bf8, 
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
TPhyPoolHandle arg_0x40b2edd8, TUniData arg_0x40b29010);



static   void PhyCC2420M$IPhyTxFIFO$default$WriteDone(
# 17 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b78bf8, 
# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
TPhyPoolHandle arg_0x40b29960, 
result_t arg_0x40b29af8, TUniData arg_0x40b29c80);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyCCA.nc"
static   TPhyStatus PhyCC2420M$IPhyCCA$Request(void);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   uint8_t *PhyFrameM$IPhyFrame$GetPPDU(const TPhyFrame *const arg_0x40b41c08);
#line 17
static   result_t PhyFrameM$IPhyFrame$ResetPosition(TPhyFrame *const arg_0x40b3fed8);







static   result_t PhyFrameM$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b3c3c8, TPhyTimeStamp arg_0x40b3c550);
#line 9
static   uint8_t PhyFrameM$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b406b8);









static   result_t PhyFrameM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b3d418);
#line 13
static   result_t PhyFrameM$IPhyFrame$AppendOctet(TPhyFrame *const arg_0x40b3f118, const uint8_t arg_0x40b3f2b8);

static   result_t PhyFrameM$IPhyFrame$ReadOctet(TPhyFrame *const arg_0x40b3f7d8, uint8_t *const arg_0x40b3f9b8);
#line 27
static   result_t PhyFrameM$IPhyFrame$GetTimeStamp(const TPhyFrame *const arg_0x40b3ca90, TPhyTimeStamp *const arg_0x40b3cc78);
#line 7
static   uint8_t PhyFrameM$IPhyFrame$GetMPDULength(const TPhyFrame *const arg_0x40b40188);
#line 23
static   result_t PhyFrameM$IPhyFrame$CheckCRC(TPhyFrame *const arg_0x40b3de40);
#line 21
static   result_t PhyFrameM$IPhyFrame$CalcCRC(TPhyFrame *const arg_0x40b3d928);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t PhyPoolM$IStdControl$init(void);






static  result_t PhyPoolM$IStdControl$start(void);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t PhyPoolM$IPhyPool$Alloc(const uint8_t arg_0x40b5e6b8, const uint16_t arg_0x40b5e860, 
TPhyPoolHandle *const arg_0x40b5ea58);

static   TPhyFrame *PhyPoolM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010);



static   result_t PhyPoolM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b5cbc0);
#line 10
static   result_t PhyPoolM$IPhyPool$GetUniData(const TPhyPoolHandle arg_0x40b5c500, uint16_t *const arg_0x40b5c6f0);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void PhyPoolM$IPhyPoolReset$reset(void);
# 61 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420.nc"
static   uint16_t HPLCC2420M$HPLCC2420$read(uint8_t arg_0x40b9b248);
#line 54
static   uint8_t HPLCC2420M$HPLCC2420$write(uint8_t arg_0x40b9cb78, uint16_t arg_0x40b9cd00);
#line 47
static   uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t arg_0x40b9c678);
# 29 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
static   result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t arg_0x40b98260, uint8_t *arg_0x40b98400);
#line 19
static   result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t arg_0x40b99a48, uint8_t *arg_0x40b99be8);
# 47 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
static   result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t arg_0x40b90638, uint8_t arg_0x40b907b8, uint8_t *arg_0x40b90958);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t HPLCC2420M$StdControl$init(void);






static  result_t HPLCC2420M$StdControl$start(void);







static  result_t HPLCC2420M$StdControl$stop(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static  result_t HPLCC2420M$BusArbitration$busFree(void);
# 43 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLI2CInterrupt.nc"
static   void HPLUSART0M$HPLI2CInterrupt$default$fired(void);
# 53 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
static   result_t HPLUSART0M$USARTData$default$rxDone(uint8_t arg_0x40c8e570);
#line 46
static   result_t HPLUSART0M$USARTData$default$txDone(void);
# 191 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
static   result_t HPLUSART0M$USARTControl$isTxEmpty(void);
#line 130
static   bool HPLUSART0M$USARTControl$isSPI(void);
#line 115
static   void HPLUSART0M$USARTControl$disableSPI(void);
#line 85
static   void HPLUSART0M$USARTControl$disableUART(void);
#line 75
static   bool HPLUSART0M$USARTControl$isUART(void);
#line 159
static   bool HPLUSART0M$USARTControl$isI2C(void);
#line 172
static   result_t HPLUSART0M$USARTControl$disableRxIntr(void);
static   result_t HPLUSART0M$USARTControl$disableTxIntr(void);
#line 65
static   bool HPLUSART0M$USARTControl$isUARTtx(void);
#line 125
static   void HPLUSART0M$USARTControl$disableI2C(void);









static   void HPLUSART0M$USARTControl$setModeSPI(void);
#line 52
static   msp430_usartmode_t HPLUSART0M$USARTControl$getMode(void);
#line 180
static   result_t HPLUSART0M$USARTControl$isTxIntrPending(void);
#line 202
static   result_t HPLUSART0M$USARTControl$tx(uint8_t arg_0x40c4b760);






static   uint8_t HPLUSART0M$USARTControl$rx(void);
#line 185
static   result_t HPLUSART0M$USARTControl$isRxIntrPending(void);
#line 70
static   bool HPLUSART0M$USARTControl$isUARTrx(void);
# 59 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t HPLCC2420InterruptM$FIFO$disable(void);
#line 43
static   result_t HPLCC2420InterruptM$FIFO$startWait(bool arg_0x40b92010);
#line 59
static   result_t HPLCC2420InterruptM$FIFOP$disable(void);
#line 43
static   result_t HPLCC2420InterruptM$FIFOP$startWait(bool arg_0x40b92010);
# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void HPLCC2420InterruptM$CCAInterrupt$fired(void);
#line 59
static   void HPLCC2420InterruptM$FIFOInterrupt$fired(void);
# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t HPLCC2420InterruptM$CCA$default$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void HPLCC2420InterruptM$SFDCapture$captured(uint16_t arg_0x40750170);
# 60 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
static   result_t HPLCC2420InterruptM$SFD$disable(void);
#line 43
static   result_t HPLCC2420InterruptM$SFD$enableCapture(bool arg_0x40b76d48);
# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void HPLCC2420InterruptM$FIFOPInterrupt$fired(void);
#line 40
static   void MSP430InterruptM$Port14$clear(void);
#line 35
static   void MSP430InterruptM$Port14$disable(void);




static   void MSP430InterruptM$Port26$clear(void);
#line 59
static   void MSP430InterruptM$Port26$default$fired(void);
#line 40
static   void MSP430InterruptM$Port17$clear(void);
#line 59
static   void MSP430InterruptM$Port17$default$fired(void);
#line 40
static   void MSP430InterruptM$Port21$clear(void);
#line 35
static   void MSP430InterruptM$Port21$disable(void);
#line 54
static   void MSP430InterruptM$Port21$edge(bool arg_0x40cd4068);
#line 30
static   void MSP430InterruptM$Port21$enable(void);









static   void MSP430InterruptM$Port12$clear(void);
#line 59
static   void MSP430InterruptM$Port12$default$fired(void);
#line 40
static   void MSP430InterruptM$Port24$clear(void);
#line 59
static   void MSP430InterruptM$Port24$default$fired(void);
#line 40
static   void MSP430InterruptM$ACCV$clear(void);
#line 59
static   void MSP430InterruptM$ACCV$default$fired(void);
#line 40
static   void MSP430InterruptM$Port15$clear(void);
#line 59
static   void MSP430InterruptM$Port15$default$fired(void);
#line 40
static   void MSP430InterruptM$Port27$clear(void);
#line 59
static   void MSP430InterruptM$Port27$default$fired(void);
#line 40
static   void MSP430InterruptM$Port10$clear(void);
#line 35
static   void MSP430InterruptM$Port10$disable(void);
#line 54
static   void MSP430InterruptM$Port10$edge(bool arg_0x40cd4068);
#line 30
static   void MSP430InterruptM$Port10$enable(void);









static   void MSP430InterruptM$Port22$clear(void);
#line 59
static   void MSP430InterruptM$Port22$default$fired(void);
#line 40
static   void MSP430InterruptM$OF$clear(void);
#line 59
static   void MSP430InterruptM$OF$default$fired(void);
#line 40
static   void MSP430InterruptM$Port13$clear(void);
#line 35
static   void MSP430InterruptM$Port13$disable(void);
#line 54
static   void MSP430InterruptM$Port13$edge(bool arg_0x40cd4068);
#line 30
static   void MSP430InterruptM$Port13$enable(void);









static   void MSP430InterruptM$Port25$clear(void);
#line 59
static   void MSP430InterruptM$Port25$default$fired(void);
#line 40
static   void MSP430InterruptM$Port16$clear(void);
#line 59
static   void MSP430InterruptM$Port16$default$fired(void);
#line 40
static   void MSP430InterruptM$NMI$clear(void);
#line 59
static   void MSP430InterruptM$NMI$default$fired(void);
#line 40
static   void MSP430InterruptM$Port20$clear(void);
#line 59
static   void MSP430InterruptM$Port20$default$fired(void);
#line 40
static   void MSP430InterruptM$Port11$clear(void);
#line 59
static   void MSP430InterruptM$Port11$default$fired(void);
#line 40
static   void MSP430InterruptM$Port23$clear(void);
#line 59
static   void MSP430InterruptM$Port23$default$fired(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t BusArbitrationM$StdControl$init(void);






static  result_t BusArbitrationM$StdControl$start(void);







static  result_t BusArbitrationM$StdControl$stop(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static  result_t BusArbitrationM$BusArbitration$default$busFree(
# 31 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
uint8_t arg_0x40d9aa88);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static   result_t BusArbitrationM$BusArbitration$releaseBus(
# 31 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
uint8_t arg_0x40d9aa88);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static   result_t BusArbitrationM$BusArbitration$getBus(
# 31 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
uint8_t arg_0x40d9aa88);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacAddressM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409bc8b0, const TMacAddress arg_0x409bca60);
#line 8
static  result_t MacAddressM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x409bc1e8, const TMacAddress arg_0x409bc398);







static  result_t MacAddressM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x409bad90);

static  bool MacAddressM$IMacAddress$Equal(const TMacAddress *const arg_0x409b92b0, const TMacAddress *const arg_0x409b94c0);
#line 14
static  result_t MacAddressM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x409ba6d0, const TMacExtendedAddress arg_0x409ba890);
#line 12
static  result_t MacAddressM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x409ba010, const TMacShortAddress arg_0x409ba1c8);
#line 6
static  TMacAddressMode MacAddressM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409bdc98);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacFrameFormatM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200);
#line 27
static  result_t MacFrameFormatM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40da5b38, const TMacAddress arg_0x40da5ce8);
#line 17
static  result_t MacFrameFormatM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40da80a8, const bool arg_0x40da8250);
#line 42
static  result_t MacFrameFormatM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40dd1530, uint8_t *const arg_0x40dd1718);




static  result_t MacFrameFormatM$IMacFrame$SetTimeStamp(TMacFrame *const arg_0x40dcfb90, TMacTimeStamp arg_0x40dcfd20);
#line 30
static  result_t MacFrameFormatM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40da3908, const TMacPANId arg_0x40da3ab8);








static  result_t MacFrameFormatM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40dd2df0, 
TMacPayloadLength *const arg_0x40dd1010);
#line 36
static  result_t MacFrameFormatM$IMacFrame$SetPayload(TMacFrame *const arg_0x40dd2510, 
const uint8_t *const arg_0x40dd2728, TMacPayloadLength arg_0x40dd28c0);
#line 15
static  result_t MacFrameFormatM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40da99a0, bool *const arg_0x40da9b88);
#line 8
static  result_t MacFrameFormatM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40dac708, const bool arg_0x40dac8b0);
#line 31
static  result_t MacFrameFormatM$IMacFrame$GetSrcPANId(const TMacFrame *const arg_0x40da2010, TMacPANId *const arg_0x40da2200);
#line 48
static  result_t MacFrameFormatM$IMacFrame$GetTimeStamp(const TMacFrame *const arg_0x40dce260, TMacTimeStamp *const arg_0x40dce450);
#line 44
static  TMacRawFrameLength MacFrameFormatM$IMacFrame$Pack(const TMacFrame *const arg_0x40dd1c38, TPhyFrame *const arg_0x40dd1e28);
#line 18
static  result_t MacFrameFormatM$IMacFrame$GetIntraPAN(const TMacFrame *const arg_0x40da8770, bool *const arg_0x40da8958);
#line 11
static  result_t MacFrameFormatM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40dab510, const bool arg_0x40dab6b8);
#line 45
static  result_t MacFrameFormatM$IMacFrame$UnPack(TMacFrame *const arg_0x40dcf3a0, TPhyFrame *const arg_0x40dcf590);
#line 21
static  result_t MacFrameFormatM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40da6558, 
TMacSequenceNumber *const arg_0x40da6760);










static  result_t MacFrameFormatM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40da2700, const TMacAddress arg_0x40da28b0);
#line 24
static  result_t MacFrameFormatM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40da6c60, const TMacPANId arg_0x40da6e10);



static  result_t MacFrameFormatM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40da3218, TMacAddress *const arg_0x40da3408);
#line 9
static  result_t MacFrameFormatM$IMacFrame$GetSecurityEnabled(const TMacFrame *const arg_0x40dacdd8, bool *const arg_0x40dab010);




static  result_t MacFrameFormatM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40da92d8, const bool arg_0x40da9480);
#line 5
static  result_t MacFrameFormatM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40dad8e8, const TMacFrameType arg_0x40dada98);
#line 50
static  result_t MacFrameFormatM$IMacFrame$GetLinkQuality(const TMacFrame *const arg_0x40dce970, TMacLinkQuality *const arg_0x40dceb60);
#line 34
static  result_t MacFrameFormatM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40da2dd0, TMacAddress *const arg_0x40dd2010);
#line 20
static  result_t MacFrameFormatM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40da8e60, const TMacSequenceNumber arg_0x40da6030);




static  result_t MacFrameFormatM$IMacFrame$GetDstPANId(const TMacFrame *const arg_0x40da5448, TMacPANId *const arg_0x40da5638);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacCommonAttrM$IStdControl$init(void);






static  result_t MacCommonAttrM$IStdControl$start(void);
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void);
#line 8
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPANId(void);
#line 7
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dc9198);


static  TMacShortAddress MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dc99a8);

static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void);
#line 24
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void);



static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void);


static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(bool arg_0x40dc4f00);
#line 6
static  TMacPANId MacCommonAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dcacf0);
#line 18
static  TMacDSN MacCommonAttrM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dc63a0);
#line 11
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacShortAddress(TMacShortAddress arg_0x40dc9e50);


static  TMacAckWaitDuration MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dc86c8);





static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacDSN(void);
#line 19
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dc6828);
#line 32
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacRandomM$IStdControl$init(void);






static  result_t MacRandomM$IStdControl$start(void);
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
static   uint16_t MacRandomM$IMacRandom$Rand(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
static   result_t MacSuperframesM$Timer$Fired(TUniData arg_0x40a53dd8);
# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
static   void MacSuperframesM$ParentCAP$default$BeginCAP(void);

static   void MacSuperframesM$ParentCAP$default$EndCAP(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacSuperframesM$StdControl$init(void);






static  result_t MacSuperframesM$StdControl$start(void);







static  result_t MacSuperframesM$StdControl$stop(void);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC_LOSS.nc"
static  void MacSuperframesM$IMacSYNC_LOSS$default$Indication(
TMacStatus arg_0x40e9fb50);
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static  result_t MacSuperframesM$IPhyTxDATA$Confirm(TPhyPoolHandle arg_0x40b2c1d8, 
TPhyStatus arg_0x40b2c378, 
TUniData arg_0x40b2c510);
# 29 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static  result_t MacSuperframesM$runOwnSF(TSysTime arg_0x40e6f120);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacSuperframesM$Reset$reset(void);
# 35 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static  void MacSuperframesM$default$sleepIndication(TMilliSec arg_0x40e6d338);
# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
static   void MacSuperframesM$IPhyTxFIFO$WriteDone(TPhyPoolHandle arg_0x40b29960, 
result_t arg_0x40b29af8, TUniData arg_0x40b29c80);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacSuperframesM$IPhyTrxParent$Confirm(TPhyStatus arg_0x40b47df8, TUniData arg_0x40b46010);
#line 8
static  result_t MacSuperframesM$IPhyTrxOwn$Confirm(TPhyStatus arg_0x40b47df8, TUniData arg_0x40b46010);
# 11 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetmacSuperframeOrder(TMacSuperframeOrder arg_0x40e80f10);
static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void);
#line 8
static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void);
#line 6
static  TMacBeaconOrder MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90);



static  TMacSuperframeOrder MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e80a60);
#line 7
static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e80248);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacSuperframeAttrC$Reset$reset(void);
# 9 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  void MacBeaconPackerM$IMacPool$FreeDone(const TMacPoolHandle arg_0x40ee5030, const TMacStatus arg_0x40ee51d8);
# 23 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconPackerM.nc"
static  void MacBeaconPackerM$packBeacon(TPhyFrame *arg_0x40ee8010, uint64_t *arg_0x40ee81b8);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacCAPM$IBackoff$Fired(TUniData arg_0x4090ac28);
# 21 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$Send(const uint8_t arg_0x40f20b58, 
const TMacFrame *const arg_0x40f20d78, 
const uint8_t arg_0x40f20f28, 
const TUniData arg_0x40f1f0f8);
# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
static   void MacCAPM$IMacCAP$BeginCAP(
# 13 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40f23e98);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
static   void MacCAPM$IMacCAP$EndCAP(
# 13 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40f23e98);
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static  result_t MacCAPM$IPhyTxDATA$Confirm(TPhyPoolHandle arg_0x40b2c1d8, 
TPhyStatus arg_0x40b2c378, 
TUniData arg_0x40b2c510);
# 26 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$SendFromPhyPool(const uint8_t arg_0x40f1f5b8, 
const TPhyPoolHandle arg_0x40f1f778, 
const uint8_t arg_0x40f1f928);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
static  TMacStatus MacCAPM$IMacCAPAttr$SetmacBattLifeExt(bool arg_0x40ebce20);
#line 20
static  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs(void);



static  TMacStatus MacCAPM$IMacCAPAttr$SetmacMinBE(
TMacBackoffExponent arg_0x40ef0570);
#line 8
static  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExt(void);





static  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods(void);



static  TMacStatus MacCAPM$IMacCAPAttr$SetmacMaxCSMABackoffs(
TMacMaxCSMABackoffs arg_0x40ebd898);
#line 6
static  bool MacCAPM$IMacCAPAttr$GetmacBattLifeExt(TMacStatus *const arg_0x40ebc988);





static  TMacStatus MacCAPM$IMacCAPAttr$SetmacBattLifeExtPeriods(
TMacBattLifeExtPeriods arg_0x40ebeba8);
#line 26
static  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacMinBE(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacCAPM$Reset$reset(
# 17 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40f20240);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacCAPM$ICAPControl$init(
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40f23728);
# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacCAPM$ICAPControl$start(
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40f23728);
# 78 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacCAPM$ICAPControl$stop(
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40f23728);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacCAPM$IPhySET_TRX_STATE$Confirm(TPhyStatus arg_0x40b47df8, TUniData arg_0x40b46010);
# 33 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  void MacCAPM$BeaconOrderChanged(uint8_t arg_0x40f1e258);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacCAPM$IMacReceive$Receive(TMacFrame *const arg_0x40ef5318);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyRxDATA.nc"
static  result_t MacRxM$IPhyRxDATA$Indication(const TPhyPoolHandle arg_0x40b26580);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacRxM$BeaconReceive$default$Receive(TMacFrame *const arg_0x40ef5318);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f88168, 
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
const TMacFrame *const arg_0x40ef9010, 
const TUniData arg_0x40ef91c8);



static  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$default$SendDone(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f88168, 
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
TMacStatus arg_0x40ef9b08, const TUniData arg_0x40ef9cb0);
#line 10
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendFromPhyPool(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f88168, 
# 10 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
const TPhyPoolHandle arg_0x40ef9680);
# 8 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendDone(const uint8_t arg_0x40f888f0, uint8_t arg_0x40f88a70, TMacStatus arg_0x40f88c00, const TUniData arg_0x40f88da8);
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Receive(const uint8_t arg_0x40f86280, TMacFrame *const arg_0x40f86470);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacPoolM$IStdControl$init(void);






static  result_t MacPoolM$IStdControl$start(void);
# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  TMacFrame *MacPoolM$IMacPool$GetFrame(const TMacPoolHandle arg_0x40ee5698);
#line 8
static  result_t MacPoolM$IMacPool$Free(const TMacPoolHandle arg_0x40ee69b8, const TMacStatus arg_0x40ee6b60);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacPoolM$IExpiredTimer$Fired(TUniData arg_0x4090ac28);
# 10 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPoolAttr.nc"
static  TMacStatus MacPoolM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void);
# 13 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static  bool MacPoolM$SearchDstAddress(const TMacAddress arg_0x40fb1ea8, 
TMacPoolHandle *const arg_0x40fb00c8);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacPoolM$Reset$reset(void);
# 9 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  void MacPendingM$IMacPool$FreeDone(const TMacPoolHandle arg_0x40ee5030, const TMacStatus arg_0x40ee51d8);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacPendingM$IMacCSMA$SendDone(TMacStatus arg_0x40ef9b08, const TUniData arg_0x40ef9cb0);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacPendingM$IMacReceive$Receive(TMacFrame *const arg_0x40ef5318);
# 13 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
static  uint8_t *MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayload(
uint8_t *const arg_0x40ec53b8, 
TMacStatus *const arg_0x40ec55b8);
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBeaconPayload(
uint8_t *arg_0x40ec5ab0, 
uint8_t arg_0x40ec5c50);


static  TMacBSN MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBSN(TMacStatus *const arg_0x40ec3480);
#line 19
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconPayload(void);









static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconTxTime(void);
#line 23
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBSN(void);
#line 22
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBSN(TMacBSN arg_0x40ec3908);
#line 11
static  uint8_t MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayloadLength(
TMacStatus *const arg_0x40ec6e20);
#line 9
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacAutoRequest(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacAssocCoordM$IStdControl$init(void);






static  result_t MacAssocCoordM$IStdControl$start(void);
# 51 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t MacAssocCoordM$IMacASSOCIATE$Response(TMacExtendedAddress arg_0x408ee228, 
TMacShortAddress arg_0x408ee3d8, 
TMacStatus arg_0x408ee570, 
bool arg_0x408ee708);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacAssocCoordM$IAssocRequest$Receive(TMacFrame *const arg_0x40ef5318);
# 10 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
static  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void);







static  bool MacAssocCoordM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40eec9e8);
static  TMacStatus MacAssocCoordM$IMacAssocAttr$SetmacAssociationPermit(bool arg_0x40eece88);
static  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacAssociationPermit(void);
#line 16
static  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacAssocCoordM$Reset$reset(void);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacAssocCoordM$IMacSendOwn$SendDone(TMacStatus arg_0x40ef9b08, const TUniData arg_0x40ef9cb0);
# 4 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconCommonM.nc"
static  uint8_t MacBeaconCommonM$getLastBSN(void);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacStartM$IPhySET_TRX_STATE$Confirm(TPhyStatus arg_0x40b47df8, TUniData arg_0x40b46010);
# 10 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
static  result_t MacStartM$IMacSTART$Request(
TMacPANId arg_0x409455b0, 
TMacLogicalChannel arg_0x40945758, 
TMacBeaconOrder arg_0x409458f8, 
TMacSuperframeOrder arg_0x40945aa0, 
bool arg_0x40945c38, 
bool arg_0x40945dd8, 
bool arg_0x40928010, 
bool arg_0x409281a8, 
TSysTime arg_0x40928358);
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t MacDataM$IMacDataOwn$Request(TMacPANId arg_0x409e2778, 
TMacAddress arg_0x409e2910, 
TMacPANId arg_0x409e2ab0, 
TMacAddress arg_0x409e2c48, 
uint8_t arg_0x409e2de0, 
const uint8_t *const arg_0x409e6010, 
uint8_t arg_0x409e61a8, 
TxOptions arg_0x409e6340);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacDataM$AckWaitTimer$Fired(TUniData arg_0x4090ac28);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacDataM$IMacReceiveOwn$Receive(TMacFrame *const arg_0x40ef5318);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacDataM$IMacSendParent$default$Send(const TMacFrame *const arg_0x40ef9010, 
const TUniData arg_0x40ef91c8);
# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t MacDataM$IMacDataParent$default$Confirm(uint8_t arg_0x409e1a00, TMacStatus arg_0x409e1b90);
#line 16
static  result_t MacDataM$IMacDataParent$default$Indication(TMacPANId arg_0x409e6810, 
TMacAddress arg_0x409e69a8, 
TMacPANId arg_0x409e6b48, 
TMacAddress arg_0x409e6ce0, 
uint8_t arg_0x409e6e78, 
uint8_t *arg_0x409e1068, 
TMacLinkQuality arg_0x409e1208, 
bool arg_0x409e13a0, 
uint8_t arg_0x409e1538);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacDataM$Reset$reset(void);
# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacDataM$StdControl$start(void);







static  result_t MacDataM$StdControl$stop(void);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacDataM$IMacSendOwn$SendDone(TMacStatus arg_0x40ef9b08, const TUniData arg_0x40ef9cb0);
#line 12
static  void MacScanBeaconM$CSMA$SendDone(TMacStatus arg_0x40ef9b08, const TUniData arg_0x40ef9cb0);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacScanBeaconM$Reset$reset(void);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacScanBeaconM$IMacReceive$Receive(TMacFrame *const arg_0x40ef5318);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress MacCoordAttrM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408f9440);
#line 7
static  bool MacCoordAttrM$IMacGET$GetmacAssociationPermit(TMacStatus *const arg_0x409026d0);
#line 19
static  TMacBeaconOrder MacCoordAttrM$IMacGET$GetmacBeaconOrder(TMacStatus *const arg_0x408ff728);
#line 37
static  TMacPANId MacCoordAttrM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408fb528);
# 11 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
static  TMacStatus MacCoordAttrM$IMacSET$SetmacBattLifeExt(bool arg_0x409346b8);
#line 36
static  TMacStatus MacCoordAttrM$IMacSET$SetmacPANId(TMacPANId arg_0x4092e1c0);



static  TMacStatus MacCoordAttrM$IMacSET$SetmacRxOnWhenIdle(bool arg_0x4092eaf8);
#line 15
static  TMacStatus MacCoordAttrM$IMacSET$SetmacBeaconPayload(TMacBeaconPayload arg_0x40932030, 
TMacBeaconPayloadLength arg_0x409321e0);
#line 42
static  TMacStatus MacCoordAttrM$IMacSET$SetmacShortAddress(TMacShortAddress arg_0x4092d010);
#line 7
static  TMacStatus MacCoordAttrM$IMacSET$SetmacAssociationPermit(bool arg_0x40935d78);
# 5 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t MacCoordAttrM$IMacRESET$Request(bool arg_0x409c2030);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacCoordAttrM$IPhySET_TRX_STATE$Confirm(TPhyStatus arg_0x40b47df8, TUniData arg_0x40b46010);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t ChildSupervisorM$ITimerSymbol$Fired(TUniData arg_0x4090ac28);
# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ChildSupervisorM$StdControl$start(void);
#line 63
static  result_t PowerManagerM$StdControl$init(void);






static  result_t PowerManagerM$StdControl$start(void);
# 33 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
static  void ZigNet2SpiM$IZigDATA$confirm(
uint8_t arg_0x408e47f8, 
NwkStatus arg_0x408e4990);



static  void ZigNet2SpiM$IZigDATA$indication(
NwkAddr arg_0x408e4e20, 
IEEEAddr arg_0x408e2010, 
uint8_t arg_0x408e21a8, 
uint8_t *arg_0x408e2358, 
uint8_t arg_0x408e24f0);
# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
static  void ZigNet2SpiM$IZigLeave$indication(IEEEAddr arg_0x408d6548);
# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void ZigNet2SpiM$INTR_2$fired(void);
# 25 "../../../zigzag/ZigBee/interface/NLME_NetworkFormation.nc"
static  void ZigNet2SpiM$IZigNetFormation$confirm(NwkStatus arg_0x408b09a8);
# 15 "../../../zigzag/ZigBee/interface/NLME_JoinParent.nc"
static  void ZigNet2SpiM$IZigParentJoin$indication(
NwkAddr arg_0x408e7808, 
IEEEAddr arg_0x408e79a8, 
NwkCapabilityInfo arg_0x408e7b58, 
bool arg_0x408e7d00);
# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void ZigNet2SpiM$IZigReset$confirm(NwkStatus arg_0x408d5c68);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ZigNet2SpiM$StdControl$init(void);






static  result_t ZigNet2SpiM$StdControl$start(void);
# 70 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
static   void UniSART1M$USARTControl$setClockRate(uint16_t arg_0x41157ca8, uint8_t arg_0x41157e28);
#line 68
static   void UniSART1M$USARTControl$setClockSource(uint8_t arg_0x41157800);
#line 92
static   result_t UniSART1M$USARTControl$isTxEmpty(void);
#line 73
static   result_t UniSART1M$USARTControl$disableRxIntr(void);
static   result_t UniSART1M$USARTControl$disableTxIntr(void);
#line 65
static   void UniSART1M$USARTControl$setModeSPI(bool arg_0x41157348);
#line 81
static   result_t UniSART1M$USARTControl$isTxIntrPending(void);
#line 103
static   result_t UniSART1M$USARTControl$tx(uint8_t arg_0x41154ab0);






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

enum MSP430ClockM$__nesc_unnamed4379 {

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

enum MSP430DCOCalibM$__nesc_unnamed4380 {

  MSP430DCOCalibM$TARGET_DELTA = 2048, 
  MSP430DCOCalibM$MAX_DEVIATION = 7
};


static inline   void MSP430DCOCalibM$TimerMicro$overflow(void);
#line 75
static inline   void MSP430DCOCalibM$Timer32khz$overflow(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureA1$captured(uint16_t arg_0x40750170);
#line 74
static   void MSP430TimerM$CaptureB3$captured(uint16_t arg_0x40750170);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA1$fired(void);
#line 34
static   void MSP430TimerM$CompareB3$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureB6$captured(uint16_t arg_0x40750170);
#line 74
static   void MSP430TimerM$CaptureB1$captured(uint16_t arg_0x40750170);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB1$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureA2$captured(uint16_t arg_0x40750170);
#line 74
static   void MSP430TimerM$CaptureB4$captured(uint16_t arg_0x40750170);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA2$fired(void);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerA$overflow(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB4$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureA0$captured(uint16_t arg_0x40750170);
#line 74
static   void MSP430TimerM$CaptureB2$captured(uint16_t arg_0x40750170);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA0$fired(void);
#line 34
static   void MSP430TimerM$CompareB2$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureB5$captured(uint16_t arg_0x40750170);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerB$overflow(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB5$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureB0$captured(uint16_t arg_0x40750170);
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
#line 110
static inline uint16_t MSP430TimerM$captureControl(uint8_t l_cm);
#line 123
void sig_TIMERA0_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(12))) ;







void sig_TIMERA1_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(10))) ;
#line 157
static inline    void MSP430TimerM$CompareA0$default$fired(void);
static inline    void MSP430TimerM$CompareA1$default$fired(void);
static inline    void MSP430TimerM$CompareA2$default$fired(void);
static inline    void MSP430TimerM$CaptureA0$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureA1$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureA2$default$captured(uint16_t time);



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
static inline   void MSP430TimerM$ControlB1$clearPendingInterrupt(void);
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


static inline   void MSP430TimerM$ControlB1$setControlAsCapture(uint8_t cm);
#line 411
static inline   void MSP430TimerM$ControlB0$enableEvents(void);
static inline   void MSP430TimerM$ControlB1$enableEvents(void);
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








static inline   bool MSP430TimerM$CaptureB1$isOverflowPending(void);







static inline   void MSP430TimerM$CaptureB1$clearOverflow(void);
# 55 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  void ZigCoordM$NIB$setNwkMaxDepth(uint8_t arg_0x408c7328);









static  void ZigCoordM$NIB$setNwkMaxRouters(uint8_t arg_0x408c7aa0);
#line 47
static  void ZigCoordM$NIB$setNwkMaxChildren(uint8_t arg_0x408c8b40);
# 15 "../../../zigzag/ZigBee/interface/NLME_NetworkFormation.nc"
static  void ZigCoordM$IZigNetFormation$request(
uint32_t arg_0x408aecd0, 
uint8_t arg_0x408aee68, 
uint8_t arg_0x408b0030, 
uint8_t arg_0x408b01c8, 

PanID_t arg_0x408b0358, 
bool arg_0x408b0508);
# 15 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void ZigCoordM$IZigReset$request(void);
# 16 "../../../zigzag/ZigBee/interface/NLME_PermitJoining.nc"
static  NwkStatus ZigCoordM$IZigPermitJoining$request(
uint8_t arg_0x408cb030);
# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ZigCoordM$ChildSupervisorControl$start(void);
# 23 "../../../zigzag/ZigCoordM.nc"
static inline  void ZigCoordM$startDemo(void);






static inline  void ZigCoordM$networkFormation(void);
#line 48
static inline  void ZigCoordM$IZigReset$confirm(NwkStatus status);






static  void ZigCoordM$IZigNetFormation$confirm(NwkStatus status);
#line 70
static inline  result_t ZigCoordM$StdControl$init(void);



static inline  result_t ZigCoordM$StdControl$start(void);










static inline  void ZigCoordM$NLME_Leave$indication(IEEEAddr DeviceAddr);
# 196 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  uint8_t NLME_JoinParentM$NIB$getDepth(void);



static  uint8_t NLME_JoinParentM$NIB$getChannel(void);
# 51 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t NLME_JoinParentM$IMacASSOCIATE$Response(TMacExtendedAddress arg_0x408ee228, 
TMacShortAddress arg_0x408ee3d8, 
TMacStatus arg_0x408ee570, 
bool arg_0x408ee708);
# 19 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacBeaconOrder NLME_JoinParentM$IMacGET$GetmacBeaconOrder(TMacStatus *const arg_0x408ff728);
#line 37
static  TMacPANId NLME_JoinParentM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408fb528);
# 28 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
static  void NLME_JoinParentM$updateBeacon(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime NLME_JoinParentM$ILocalTime$Read(void);
# 15 "../../../zigzag/ZigBee/interface/NLME_JoinParent.nc"
static  void NLME_JoinParentM$NLME_JoinParent$indication(
NwkAddr arg_0x408e7808, 
IEEEAddr arg_0x408e79a8, 
NwkCapabilityInfo arg_0x408e7b58, 
bool arg_0x408e7d00);
# 15 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NLME_JoinParentM$NeighborTable$update(NwkNeighbor arg_0x408de578);
# 15 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  result_t NLME_JoinParentM$NwkAddressing$allocEdAddress(NwkAddr *arg_0x40914ba8);

static  result_t NLME_JoinParentM$NwkAddressing$allocRouterAddress(NwkAddr *arg_0x40913068);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t NLME_JoinParentM$ITimerSymbol$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868);
# 34 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
NwkAddr NLME_JoinParentM$joiningNwkAddr;
IEEEAddr NLME_JoinParentM$joiningExtAddr;
NwkCapabilityInfo NLME_JoinParentM$joiningCapability;

enum NLME_JoinParentM$__nesc_unnamed4381 {
#line 38
  NLME_JoinParentM$WAIT_BEACONS = 3
};
NwkStatus NLME_JoinParentM$addr_allocated;
enum NLME_JoinParentM$__nesc_unnamed4382 {
#line 41
  NLME_JoinParentM$IDLE, NLME_JoinParentM$WAIT_COMM_STATUS
} 
#line 41
NLME_JoinParentM$state;



static inline  result_t NLME_JoinParentM$IMacASSOCIATE$Indication(TMacExtendedAddress deviceAddr, 
TCapabilityInformation capability, 
bool securityUse, 
TACLEntry aclEntry);
#line 87
static  void NLME_JoinParentM$IMacCOMM_STATUS$Indication(TMacPANId panID, TMacAddress src, TMacAddress dst, TMacStatus status);
#line 141
static inline  result_t NLME_JoinParentM$ITimerSymbol$Fired(TUniData u);










static inline  void NLME_JoinParentM$Reset$reset(void);
# 25 "../../../zigzag/ZigBee/interface/NLME_NetworkFormation.nc"
static  void NLME_NetworkFormationM$NLME_NetworkFormation$confirm(NwkStatus arg_0x408b09a8);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NLME_NetworkFormationM$NwkAddressingReset$reset(void);
# 191 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  void NLME_NetworkFormationM$NIB$setOnline(NwkOnlineStatus arg_0x408bf800);
#line 204
static  void NLME_NetworkFormationM$NIB$setBeaconOffset(TSysTime arg_0x408da228);
#line 192
static  NwkOnlineStatus NLME_NetworkFormationM$NIB$getOnlineStatus(void);




static  void NLME_NetworkFormationM$NIB$setDepth(uint8_t arg_0x408bd620);



static  void NLME_NetworkFormationM$NIB$setChannel(uint8_t arg_0x408bdd90);
# 11 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
static  TMacStatus NLME_NetworkFormationM$IMacSET$SetmacBattLifeExt(bool arg_0x409346b8);
#line 36
static  TMacStatus NLME_NetworkFormationM$IMacSET$SetmacPANId(TMacPANId arg_0x4092e1c0);



static  TMacStatus NLME_NetworkFormationM$IMacSET$SetmacRxOnWhenIdle(bool arg_0x4092eaf8);

static  TMacStatus NLME_NetworkFormationM$IMacSET$SetmacShortAddress(TMacShortAddress arg_0x4092d010);
# 24 "../../../zigzag/ZigBee/implementation/NLME_NetworkFormationM.nc"
static  void NLME_NetworkFormationM$updateBeacon(void);
# 19 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  bool NLME_NetworkFormationM$NwkAddressing$hasVacantAddrs(void);
# 10 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
static  result_t NLME_NetworkFormationM$IMacSTART$Request(
TMacPANId arg_0x409455b0, 
TMacLogicalChannel arg_0x40945758, 
TMacBeaconOrder arg_0x409458f8, 
TMacSuperframeOrder arg_0x40945aa0, 
bool arg_0x40945c38, 
bool arg_0x40945dd8, 
bool arg_0x40928010, 
bool arg_0x409281a8, 
TSysTime arg_0x40928358);
# 32 "../../../zigzag/ZigBee/implementation/NLME_NetworkFormationM.nc"
PanID_t NLME_NetworkFormationM$request_panID;
uint8_t NLME_NetworkFormationM$request_beaconOrder;
uint8_t NLME_NetworkFormationM$request_superframeOrder;
uint8_t NLME_NetworkFormationM$channel;

static inline void NLME_NetworkFormationM$start_mac(void);

static inline  void NLME_NetworkFormationM$NLME_NetworkFormation$request(
uint32_t scanChannels, 
uint8_t scanDuration, 
uint8_t beaconOrder, 
uint8_t superframeOrder, 

PanID_t panID, 
bool batteryLifeExtension);
#line 94
static inline void NLME_NetworkFormationM$start_mac(void);
#line 136
static  result_t NLME_NetworkFormationM$IMacSTART$Confirm(TMacStatus status);
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
static   uint16_t NIBM$Rand$Rand(void);
# 21 "../../../zigzag/ZigBee/implementation/NIBM.nc"
enum NIBM$__nesc_unnamed4383 {

  NIBM$DEFAULT_nwkPassiveAckTimeout = 0x03, 
  NIBM$DEFAULT_nwkMaxBroadcastRetries = 0x03, 
  NIBM$DEFAULT_nwkMaxChildren = 0x07, 
  NIBM$DEFAULT_nwkMaxDepth = 0x05, 
  NIBM$DEFAULT_nwkMaxRouters = 0x05, 
  NIBM$DEFAULT_nwkNetworkBroadcastDeliveryTime = NIBM$DEFAULT_nwkPassiveAckTimeout, 
  NIBM$DEFAULT_nwkReportConstantCost = 0x00, 
  NIBM$DEFAULT_nwkRouteDiscoveryRetriesPermitted = NWK_DISCOVERY_RETRY_LIMIT, 
  NIBM$DEFAULT_nwkSymLink = FALSE, 
  NIBM$DEFAULT_nwkCapabilityInformation = 0x00, 
  NIBM$DEFAULT_nwkUseTreeAddrAlloc = TRUE, 
  NIBM$DEFAULT_nwkUseTreeRouting = TRUE, 
  NIBM$DEFAULT_nwkNextAddress = 0x0000, 
  NIBM$DEFAULT_nwkAvailableAddresses = 0x0000, 
  NIBM$DEFAULT_nwkAddressIncrement = 0x0001, 
  NIBM$DEFAULT_nwkTransactionPersistenceTime = 0x01f4
};

uint8_t NIBM$nwkSequenceNumber;
uint8_t NIBM$nwkPassiveAckTimeout;
uint8_t NIBM$nwkMaxBroadcastRetries;
uint8_t NIBM$nwkMaxChildren;
uint8_t NIBM$nwkMaxDepth;
uint8_t NIBM$nwkMaxRouters;
uint8_t NIBM$nwkNetworkBroadcastDeliveryTime;
uint8_t NIBM$nwkReportConstantCost;
uint8_t NIBM$nwkRouteDiscoveryRetriesPermitted;
bool NIBM$nwkSymLink;

bool NIBM$nwkUseTreeAddrAlloc;
bool NIBM$nwkUseTreeRouting;
uint16_t NIBM$nwkNextAddress;
uint16_t NIBM$nwkAvailableAddresses;
uint16_t NIBM$nwkAddressIncrement;
uint16_t NIBM$nwkTransactionPersistenceTime;

TSysTime NIBM$beaconOffset;



NwkOnlineStatus NIBM$onlineStatus;
uint8_t NIBM$channel;
uint8_t NIBM$depth;



static inline  void NIBM$NIB$setNwkSequenceNumber(uint8_t attr);



static inline  uint8_t NIBM$NIB$getNwkSequenceNumber(void);
#line 99
static inline  void NIBM$NIB$setNwkMaxChildren(uint8_t attr);



static inline  uint8_t NIBM$NIB$getNwkMaxChildren(void);





static inline  void NIBM$NIB$setNwkMaxDepth(uint8_t attr);



static inline  uint8_t NIBM$NIB$getNwkMaxDepth(void);





static inline  void NIBM$NIB$setNwkMaxRouters(uint8_t attr);



static inline  uint8_t NIBM$NIB$getNwkMaxRouters(void);
#line 238
static inline  void NIBM$Reset$reset(void);
#line 269
static inline  void NIBM$NIB$setOnline(NwkOnlineStatus status);
static inline  NwkOnlineStatus NIBM$NIB$getOnlineStatus(void);

static inline  bool NIBM$NIB$isJoined(void);

static inline  uint8_t NIBM$NIB$getDepth(void);
static inline  void NIBM$NIB$setDepth(uint8_t d);




static inline  uint8_t NIBM$NIB$getChannel(void);
static inline  void NIBM$NIB$setChannel(uint8_t c);



static inline  void NIBM$NIB$setBeaconOffset(TSysTime offs);
static inline  TSysTime NIBM$NIB$getBeaconOffset(void);
# 66 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  uint8_t NwkAddressingM$NIB$getNwkMaxRouters(void);
#line 196
static  uint8_t NwkAddressingM$NIB$getDepth(void);
#line 48
static  uint8_t NwkAddressingM$NIB$getNwkMaxChildren(void);







static  uint8_t NwkAddressingM$NIB$getNwkMaxDepth(void);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress NwkAddressingM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408f9440);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NwkAddressingM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x408dea50);
# 24 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
NwkAddr NwkAddressingM$cskip = 0;
NwkAddr NwkAddressingM$nextRouterAddr;
NwkAddr NwkAddressingM$nextEDAddr;
uint8_t NwkAddressingM$edAddrNum;
uint8_t NwkAddressingM$routerAddrNum = 0;


static NwkAddr NwkAddressingM$countCskip(uint8_t depth);
#line 55
static  void NwkAddressingM$Reset$reset(void);
#line 69
static inline void NwkAddressingM$calcNextEDAddr(void);
#line 81
static inline void NwkAddressingM$calcNextRouterAddr(void);
#line 93
static bool NwkAddressingM$checkFree(NwkAddr addr);
#line 105
static inline  result_t NwkAddressingM$NwkAddressing$allocEdAddress(NwkAddr *newAddr);
#line 126
static inline  result_t NwkAddressingM$NwkAddressing$allocRouterAddress(NwkAddr *newAddr);
#line 148
static void NwkAddressingM$countChildren(uint8_t *routers, uint8_t *eds);
#line 164
static  bool NwkAddressingM$NwkAddressing$hasVacantAddrs(void);







static inline  bool NwkAddressingM$NwkAddressing$hasVacantEdAddrs(void);







static inline  bool NwkAddressingM$NwkAddressing$hasVacantRouterAddrs(void);
#line 204
static inline  void NwkAddressingM$NwkAddressing$getDirection(NwkAddr dest, NwkRelationship *relationship, NwkAddr *next_hop);
# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  bool NwkBeaconParentM$IMacGET$GetmacAssociationPermit(TMacStatus *const arg_0x409026d0);
# 15 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
static  TMacStatus NwkBeaconParentM$IMacSET$SetmacBeaconPayload(TMacBeaconPayload arg_0x40932030, 
TMacBeaconPayloadLength arg_0x409321e0);
# 196 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  uint8_t NwkBeaconParentM$NIB$getDepth(void);








static  TSysTime NwkBeaconParentM$NIB$getBeaconOffset(void);
# 20 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  bool NwkBeaconParentM$NwkAddressing$hasVacantEdAddrs(void);
static  bool NwkBeaconParentM$NwkAddressing$hasVacantRouterAddrs(void);
# 27 "../../../zigzag/ZigBee/implementation/NwkBeaconParentM.nc"
static  void NwkBeaconParentM$updateBeacon(void);
# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
static  TMacStatus NLME_PermitJoiningM$IMacSET$SetmacAssociationPermit(bool arg_0x40935d78);
# 23 "../../../zigzag/ZigBee/implementation/NLME_PermitJoiningM.nc"
static  void NLME_PermitJoiningM$updateBeacon(void);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t NLME_PermitJoiningM$ITimerSymbol$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868);
# 19 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  bool NLME_PermitJoiningM$NwkAddressing$hasVacantAddrs(void);
# 30 "../../../zigzag/ZigBee/implementation/NLME_PermitJoiningM.nc"
static inline  NwkStatus NLME_PermitJoiningM$NLME_PermitJoining$request(
uint8_t permitDuration);
#line 53
static inline  result_t NLME_PermitJoiningM$ITimerSymbol$Fired(TUniData u);
# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void NLME_ResetM$NLME_Reset$confirm(NwkStatus arg_0x408d5c68);
#line 15
static  void NLME_ResetM$MacReset$request(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NLME_ResetM$Reset$reset(void);
# 21 "../../../zigzag/ZigBee/implementation/NLME_ResetM.nc"
bool NLME_ResetM$waitConfirm = FALSE;
static inline  void NLME_ResetM$NLME_Reset$request(void);






static inline  void NLME_ResetM$MacReset$confirm(NwkStatus status);
# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void NwkResetMacSingleM$NLME_Reset$confirm(NwkStatus arg_0x408d5c68);
# 5 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t NwkResetMacSingleM$IMacRESET$Request(bool arg_0x409c2030);
# 14 "../../../zigzag/ZigBee/implementation/NwkResetMacSingleM.nc"
static inline  void NwkResetMacSingleM$NLME_Reset$request(void);




static inline  result_t NwkResetMacSingleM$IMacRESET$Confirm(TMacStatus status);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t NeighborTableM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409bc8b0, const TMacAddress arg_0x409bca60);
#line 8
static  result_t NeighborTableM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x409bc1e8, const TMacAddress arg_0x409bc398);
#line 6
static  TMacAddressMode NeighborTableM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409bdc98);
# 32 "../../../zigzag/ZigBee/implementation/NeighborTableM.nc"
NwkNeighbor NeighborTableM$Table[20];

int NeighborTableM$free_idx = 0;

static inline  void NeighborTableM$Reset$reset(void);
#line 49
static inline  result_t NeighborTableM$NeighborTable$update(NwkNeighbor new_neighbor);
#line 84
static  result_t NeighborTableM$NeighborTable$getNextPtr(NwkNeighbor **neighbor);
#line 102
static  result_t NeighborTableM$NeighborTable$getByAddr(TMacAddress addr, NwkNeighbor **pneighbor);
#line 137
static inline  result_t NeighborTableM$NeighborTable$remove(NwkNeighbor *pneighbor);
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t RouterM$LowerIf$Request(TMacPANId arg_0x409e2778, 
TMacAddress arg_0x409e2910, 
TMacPANId arg_0x409e2ab0, 
TMacAddress arg_0x409e2c48, 
uint8_t arg_0x409e2de0, 
const uint8_t *const arg_0x409e6010, 
uint8_t arg_0x409e61a8, 
TxOptions arg_0x409e6340);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress RouterM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408f9440);
#line 37
static  TMacPANId RouterM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408fb528);
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t RouterM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x409ba010, const TMacShortAddress arg_0x409ba1c8);
# 192 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  NwkOnlineStatus RouterM$NIB$getOnlineStatus(void);
static  bool RouterM$NIB$isJoined(void);
# 7 "../../../zigzag/ZigBee/implementation/NwkFrame.nc"
static  uint8_t RouterM$NwkFrame$mkDataFrameHeader(uint8_t *arg_0x409fc010, 
NwkAddr arg_0x409fc1a0, 
uint8_t arg_0x409fc330, 
uint8_t arg_0x409fc4c8, 
bool arg_0x409fc670);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime RouterM$ILocalTime$Read(void);
# 33 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
static  void RouterM$NLDE_Data$confirm(
uint8_t arg_0x408e47f8, 
NwkStatus arg_0x408e4990);



static  void RouterM$NLDE_Data$indication(
NwkAddr arg_0x408e4e20, 
IEEEAddr arg_0x408e2010, 
uint8_t arg_0x408e21a8, 
uint8_t *arg_0x408e2358, 
uint8_t arg_0x408e24f0);
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t RouterM$UpperIf$Request(TMacPANId arg_0x409e2778, 
TMacAddress arg_0x409e2910, 
TMacPANId arg_0x409e2ab0, 
TMacAddress arg_0x409e2c48, 
uint8_t arg_0x409e2de0, 
const uint8_t *const arg_0x409e6010, 
uint8_t arg_0x409e61a8, 
TxOptions arg_0x409e6340);
# 25 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  void RouterM$NwkAddressing$getDirection(NwkAddr arg_0x40913dd8, NwkRelationship *arg_0x40911010, NwkAddr *arg_0x409111b0);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t RouterM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x408dea50);







static  result_t RouterM$NeighborTable$getByAddr(TMacAddress arg_0x408deee0, NwkNeighbor **arg_0x408dd0c8);
# 34 "../../../zigzag/ZigBee/implementation/RouterM.nc"
enum RouterM$__nesc_unnamed4384 {

  RouterM$NWK_FRAME_BUFFER_SIZE = 8, 
  RouterM$NWK_MAX_PAYLOAD_SIZE = MAC_AMAX_MAC_FRAME_SIZE - sizeof(NwkHeader ), 


  RouterM$NWK_FRAME_NULL = 0, 
  RouterM$NWK_FRAME_PENDING = 1
};








#line 44
typedef struct RouterM$__nesc_unnamed4385 {

  TSysTime timestamp;
  uint8_t state;
  uint8_t msduLength;
  uint8_t msdu[MAC_AMAX_MAC_FRAME_SIZE];
} 
RouterM$FrameEntry;

static  void RouterM$relayNext(void);
static bool RouterM$sendFrame(uint8_t *frame, uint8_t length);
static inline result_t RouterM$get_relaying_options(NwkAddr destinationAddr, NwkAddr *nextHop, NwkRelationship *relationship, bool *indirect);

static inline  uint8_t RouterM$newMsduHandle(void);





RouterM$FrameEntry RouterM$frames[RouterM$NWK_FRAME_BUFFER_SIZE];
uint8_t RouterM$cur_frame;

uint8_t RouterM$current_msduHandle;
uint8_t RouterM$current_nsduHandle;
bool RouterM$wait_confirm = FALSE;
bool RouterM$relaying;



bool RouterM$reportErrorWhenGetFree = FALSE;
uint8_t RouterM$reportErrorNsduHandle;








static inline  result_t RouterM$NLDE_Data$request(
NwkAddr dstAddr, 
uint8_t nsduLength, 
uint8_t *nsdu, 
uint8_t nsduHandle, 
uint8_t radius, 
uint8_t discoverRoute, 
bool securityEnable);
#line 156
static bool RouterM$sendFrame(uint8_t *frame, uint8_t length);
#line 223
static inline result_t RouterM$IMacDATA_Confirm(uint8_t msduHandle, TMacStatus status);
#line 292
static inline result_t RouterM$find_place(uint8_t *idx);

static inline result_t RouterM$IMacDATA_Indication(
TMacPANId srcPanID, 
TMacAddress src, 
TMacPANId dstPanID, 
TMacAddress dst, 
uint8_t msduLength, 
uint8_t *msdu, 
TMacLinkQuality mpduLinkQuality, 
bool sequrityUse, 
uint8_t aclEntry);
#line 370
static inline result_t RouterM$find_place(uint8_t *idx);
#line 410
static  void RouterM$relayNext(void);
#line 438
static inline result_t RouterM$get_relaying_options(NwkAddr destinationAddr, NwkAddr *nextHop, NwkRelationship *relationship, bool *indirect);
#line 475
static inline  void RouterM$Reset$reset(void);
#line 494
static inline  result_t RouterM$LowerIf$Confirm(uint8_t msduHandle, TMacStatus status);




static inline  result_t RouterM$LowerIf$Indication(
TMacPANId srcPanID, 
TMacAddress src, 
TMacPANId dstPanID, 
TMacAddress dst, 
uint8_t msduLength, 
uint8_t *msdu, 
TMacLinkQuality mpduLinkQuality, 
bool securityUse, 
uint8_t aclEntry);
#line 559
static inline   result_t RouterM$UpperIf$default$Request(TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
const uint8_t *const pmsduFrame, 
uint8_t msduHandle, 
TxOptions bmTxOpt);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress NwkFrameM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408f9440);
# 20 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  uint8_t NwkFrameM$NIB$getNwkSequenceNumber(void);
#line 19
static  void NwkFrameM$NIB$setNwkSequenceNumber(uint8_t arg_0x408ca478);
# 15 "../../../zigzag/ZigBee/implementation/NwkFrameM.nc"
static inline  uint8_t NwkFrameM$NwkFrame$mkDataFrameHeader(uint8_t *msdu, 
NwkAddr dstAddr, 
uint8_t radius, 
uint8_t discoverRoute, 
bool securityEnable);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t TimerSymbol2M$TimerMilli$fired(
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a643d0);
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   uint32_t TimerSymbol2M$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a346a0);
#line 7
static   TSysTime TimerSymbol2M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a37d70);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbol2M$AlarmControl$enableEvents(void);
#line 35
static   void TimerSymbol2M$AlarmControl$setControlAsCompare(void);



static   void TimerSymbol2M$AlarmControl$disableEvents(void);
#line 32
static   void TimerSymbol2M$AlarmControl$clearPendingInterrupt(void);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbol2M$AlarmCompare$setEventFromNow(uint16_t arg_0x40754010);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   uint16_t TimerSymbol2M$AlarmTimer$read(void);
static   bool TimerSymbol2M$AlarmTimer$isOverflowPending(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t TimerSymbol2M$ITimerSymbol$Fired(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a667c0, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TUniData arg_0x4090ac28);
# 21 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
enum TimerSymbol2M$__nesc_unnamed4386 {
  TimerSymbol2M$NUM_SYMBOL_TIMERS = 7U, 
  TimerSymbol2M$NUM_MILLIS_TIMERS = 0U, 
  TimerSymbol2M$NUM_TIMERS = TimerSymbol2M$NUM_SYMBOL_TIMERS + TimerSymbol2M$NUM_MILLIS_TIMERS, 
  TimerSymbol2M$EMPTY_LIST = 255, 
  TimerSymbol2M$MAX_HW_COUNTER = 0xffffL, 
  TimerSymbol2M$MIN_HW_COUNTER = 2
};

typedef uint32_t TimerSymbol2M$TJiffy;










#line 32
typedef struct TimerSymbol2M$__nesc_unnamed4387 {
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


static   uint16_t TimerSymbol2M$overflowCount(void);








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




static inline  result_t TimerSymbol2M$ITimerSymbol$SetOneShot(uint8_t num, TSysTime symbols, TUniData uniData);




static inline  result_t TimerSymbol2M$ITimerSymbol$Stop(uint8_t num);





static inline  bool TimerSymbol2M$ITimerSymbol$IsSet(uint8_t num);
#line 349
static inline   result_t TimerSymbol2M$ITimerSymbol$default$Fired(uint8_t num, TUniData uniData);
#line 399
static inline   result_t TimerSymbol2M$TimerMilli$default$fired(uint8_t num);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl2$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl2$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl2$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl2$clearPendingInterrupt(void);
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   uint32_t TimerSymbolAsyncM$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a346a0);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare1$setEvent(uint16_t arg_0x4073c680);
#line 30
static   void TimerSymbolAsyncM$AlarmCompare4$setEvent(uint16_t arg_0x4073c680);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl3$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl3$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl3$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl3$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare2$setEvent(uint16_t arg_0x4073c680);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl0$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl0$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl0$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl0$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare0$setEvent(uint16_t arg_0x4073c680);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl4$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl4$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl4$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl4$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare3$setEvent(uint16_t arg_0x4073c680);
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
uint8_t arg_0x40aac3d8, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
TUniData arg_0x40a53dd8);
# 33 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
enum TimerSymbolAsyncM$__nesc_unnamed4388 {
  TimerSymbolAsyncM$ASYNC_TIMER_NUM = 5, 
  TimerSymbolAsyncM$MAX_HW_COUNTER = 0xFFFF, 
  TimerSymbolAsyncM$MIN_HW_COUNTER = 2, 
  TimerSymbolAsyncM$WARNING_INTERVAL = 10
};








#line 40
struct TimerSymbolAsyncM$__nesc_unnamed4389 {
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
static void TimerSymbolAsyncM$SetTimerAt(const uint8_t timerNumber, const uint32_t time);
#line 104
static void TimerSymbolAsyncM$HandleAlarmFire(const uint8_t timerNumber);
#line 142
static result_t TimerSymbolAsyncM$StartTimerAt(const uint8_t timerNumber, 
const uint32_t time, const uint32_t period, 
const TUniData uniData, const bool isPeriodic);
#line 183
static inline   
#line 182
result_t TimerSymbolAsyncM$ITimerSymbolAsync$SetOneShotAt(uint8_t timerNumber, 
TSysTime time, TUniData uniData);
#line 234
static inline   result_t TimerSymbolAsyncM$ITimerSymbolAsync$Stop(uint8_t timerNumber);









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
static   void TimerTuningM$ITimerB$setClockSource(uint16_t arg_0x40735dd8);
#line 32
static   void TimerTuningM$ITimerB$clearOverflow(void);


static   void TimerTuningM$ITimerB$setMode(int arg_0x40735010);




static   void TimerTuningM$ITimerB$setInputDivider(uint16_t arg_0x40733358);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerTuningM$ITimerB4Control$disableEvents(void);
#line 39
static   void TimerTuningM$ITimerB5Control$disableEvents(void);
#line 39
static   void TimerTuningM$ITimerB6Control$disableEvents(void);
#line 39
static   void TimerTuningM$ITimerB1Control$disableEvents(void);
#line 34
static   void TimerTuningM$ITimerB1Control$setControl(MSP430CompareControl_t arg_0x40741010);




static   void TimerTuningM$ITimerB3Control$disableEvents(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void TimerTuningM$ITimerA$setClockSource(uint16_t arg_0x40735dd8);
#line 38
static   void TimerTuningM$ITimerA$disableEvents(void);
#line 32
static   void TimerTuningM$ITimerA$clearOverflow(void);


static   void TimerTuningM$ITimerA$setMode(int arg_0x40735010);
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
static   TMilliSec LocalTime64M$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a35558);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime LocalTime64M$ILocalTime$Read(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t LocalTime64M$ITimerSymbol$Stop(void);
#line 6
static  result_t LocalTime64M$ITimerSymbol$SetPeriodic(TSysTime arg_0x4090c0c0, TUniData arg_0x4090c248);




static  bool LocalTime64M$ITimerSymbol$IsSet(void);
# 17 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
uint64_t LocalTime64M$baseTime = 0;
TSysTime LocalTime64M$firedCount;

static  uint64_t LocalTime64M$ILocalTime64$getLocalTimeAt(TSysTime at);









static inline  uint64_t LocalTime64M$ILocalTime64$getLocalTime(void);




static inline  void LocalTime64M$ILocalTime64$setLocalTimeAt(TSysTime at, uint64_t time);
#line 47
static inline  void LocalTime64M$ILocalTime64$setLocalTime(uint64_t time);




static inline  result_t LocalTime64M$ITimerSymbol$Fired(TUniData ud);
# 12 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   TSysTime TimeCastM$ITimeCast$MillisToSymbols(TMilliSec millis);



static inline   TSysTime TimeCastM$ITimeCast$JiffiesToSymbols(uint32_t jiffies);








static inline   TMilliSec TimeCastM$ITimeCast$SymbolsToMillis(TSysTime symbols);
#line 42
static inline   uint32_t TimeCastM$ITimeCast$SymbolsToJiffies(TSysTime symbols);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyRxDATA.nc"
static  result_t PhyCC2420M$IPhyRxDATA$Indication(const TPhyPoolHandle arg_0x40b26580);
# 61 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420.nc"
static   uint16_t PhyCC2420M$IChipcon$read(uint8_t arg_0x40b9b248);
#line 54
static   uint8_t PhyCC2420M$IChipcon$write(uint8_t arg_0x40b9cb78, uint16_t arg_0x40b9cd00);
#line 47
static   uint8_t PhyCC2420M$IChipcon$cmd(uint8_t arg_0x40b9c678);
# 47 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
static   result_t PhyCC2420M$IChipconRAM$write(uint16_t arg_0x40b90638, uint8_t arg_0x40b907b8, uint8_t *arg_0x40b90958);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   TSysTime PhyCC2420M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a37d70);
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static  result_t PhyCC2420M$IPhyTxDATA$Confirm(
# 9 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b4e010, 
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
TPhyPoolHandle arg_0x40b2c1d8, 
TPhyStatus arg_0x40b2c378, 
TUniData arg_0x40b2c510);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t PhyCC2420M$IPhyPool$Alloc(const uint8_t arg_0x40b5e6b8, const uint16_t arg_0x40b5e860, 
TPhyPoolHandle *const arg_0x40b5ea58);

static   TPhyFrame *PhyCC2420M$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010);
# 59 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t PhyCC2420M$IChipconFIFOSignal$disable(void);
#line 43
static   result_t PhyCC2420M$IChipconFIFOSignal$startWait(bool arg_0x40b92010);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t PhyCC2420M$HPLCC2420Control$init(void);






static  result_t PhyCC2420M$HPLCC2420Control$start(void);







static  result_t PhyCC2420M$HPLCC2420Control$stop(void);
# 59 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t PhyCC2420M$IChipconFIFOP$disable(void);
#line 43
static   result_t PhyCC2420M$IChipconFIFOP$startWait(bool arg_0x40b92010);
# 60 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
static   result_t PhyCC2420M$IChipconSFD$disable(void);
#line 43
static   result_t PhyCC2420M$IChipconSFD$enableCapture(bool arg_0x40b76d48);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   uint8_t *PhyCC2420M$IPhyFrame$GetPPDU(const TPhyFrame *const arg_0x40b41c08);
#line 25
static   result_t PhyCC2420M$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b3c3c8, TPhyTimeStamp arg_0x40b3c550);
#line 9
static   uint8_t PhyCC2420M$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b406b8);
# 29 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
static   result_t PhyCC2420M$IChipconFIFO$writeTXFIFO(uint8_t arg_0x40b98260, uint8_t *arg_0x40b98400);
#line 19
static   result_t PhyCC2420M$IChipconFIFO$readRXFIFO(uint8_t arg_0x40b99a48, uint8_t *arg_0x40b99be8);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t PhyCC2420M$IPhySET_TRX_STATE$Confirm(
# 13 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b79340, 
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
TPhyStatus arg_0x40b47df8, TUniData arg_0x40b46010);
# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
static   void PhyCC2420M$IPhyTxFIFO$WriteDone(
# 17 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b78bf8, 
# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
TPhyPoolHandle arg_0x40b29960, 
result_t arg_0x40b29af8, TUniData arg_0x40b29c80);
# 32 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static   uint16_t PhyCC2420M$overflowCount(void);
#line 60
#line 41
typedef enum PhyCC2420M$__nesc_unnamed4390 {

  PhyCC2420M$PHY_RADIO_OFF, 
  PhyCC2420M$PHY_RADIO_IDLE, 

  PhyCC2420M$PHY_RADIO_TX_IDLE, 
  PhyCC2420M$PHY_RADIO_TX_INIT, 
  PhyCC2420M$PHY_RADIO_TX_FIFO_WRITE, 
  PhyCC2420M$PHY_RADIO_TX_BEGIN, 
  PhyCC2420M$PHY_RADIO_TX_FRAME_SEND, 
  PhyCC2420M$PHY_RADIO_TX_END, 

  PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH, 
  PhyCC2420M$PHY_RADIO_RX_BEGIN, 
  PhyCC2420M$PHY_RADIO_RX_FRAME_RECV, 
  PhyCC2420M$PHY_RADIO_RX_END, 
  PhyCC2420M$PHY_RADIO_RX_FIFO_READ, 
  PhyCC2420M$PHY_RADIO_RX_INDICATION
} 
PhyCC2420M$TPhyRadioState;

PhyCC2420M$TPhyRadioState PhyCC2420M$radioState;










 
#line 64
struct PhyCC2420M$__nesc_unnamed4391 {
  uint8_t *pData;
  uint8_t dataLength;
  uint8_t context;
  TUniData uniData;
  TSysTime timeStamp;
  TPhyPoolHandle handle;
  TPhyPoolHandle txFifoHandle;
  bool txFifoSend;
} PhyCC2420M$radio;

uint64_t PhyCC2420M$g_extendedAddress;
uint16_t PhyCC2420M$g_shortAddress;
uint16_t PhyCC2420M$g_panId;
TPhyChannelsSupported PhyCC2420M$g_phyChannelsSupported;
TPhyChannel PhyCC2420M$g_channel = 11;
bool PhyCC2420M$g_autoAck;
TPhyPoolHandle PhyCC2420M$g_rxHandle;

static inline   void PhyCC2420M$FreePoolItem(TPhyPoolHandle handle);








static inline void PhyCC2420M$flushTXFIFO(void);








static result_t PhyCC2420M$WriteToTxFIFO(TPhyPoolHandle handle);
#line 116
static result_t PhyCC2420M$SendTxFIFO(void);
#line 136
static inline   
#line 135
result_t PhyCC2420M$IPhyTxFIFO$Write(uint8_t context, 
TPhyPoolHandle handle, TUniData uniData);
#line 158
static   result_t PhyCC2420M$IPhyTxDATA$Request(uint8_t context, 
const TPhyPoolHandle handle, 
const TUniData uniData);
#line 191
static void PhyCC2420M$flushRXFIFO(void);










static void PhyCC2420M$SendDone(TPhyStatus phyStatus);
#line 222
static inline  void PhyCC2420M$send_done_fail(void);
static inline  void PhyCC2420M$send_done_success(void);

static inline   result_t PhyCC2420M$IChipconFIFO$TXFIFODone(uint8_t length, uint8_t *data);
#line 258
static inline   result_t PhyCC2420M$IChipconSFD$captured(uint16_t time);
#line 274
static inline   result_t PhyCC2420M$IChipconFIFOSignal$fired(void);
#line 299
static inline  void PhyCC2420M$fifop_handler(void);
#line 327
static inline   result_t PhyCC2420M$IChipconFIFOP$fired(void);









static inline  void PhyCC2420M$TaskRxIndication(void);
#line 357
static inline   result_t PhyCC2420M$IChipconFIFO$RXFIFODone(uint8_t length, uint8_t *data);
#line 376
static inline   TPhyStatus PhyCC2420M$IPhyCCA$Request(void);
#line 407
#line 404
enum PhyCC2420M$TPhyEDState {
  PhyCC2420M$PHY_ED_FREE, 
  PhyCC2420M$PHY_ED_BUSY
} PhyCC2420M$edState;
#line 473
static inline result_t PhyCC2420M$SetDefaultRegisters(void);




#line 475
enum PhyCC2420M$TPhySetTRXState {
  PhyCC2420M$PHY_TRX_FREE, 
  PhyCC2420M$PHY_TRX_BUSY
} PhyCC2420M$trxState;




 
#line 479
struct PhyCC2420M$__nesc_unnamed4392 {
  uint8_t context;
  TUniData uniData;
  TPhyStatus phyStatus;
} PhyCC2420M$trx;


static void PhyCC2420M$turnOnChipcon(void);
#line 522
static void PhyCC2420M$turnOffChipcon(void);
#line 541
static void PhyCC2420M$turnRxOnChipcon(void);







static void PhyCC2420M$turnTxOnChipcon(void);








static inline  void PhyCC2420M$SetTRXStateDone(void);
#line 668
static   result_t PhyCC2420M$IPhySET_TRX_STATE$Request(uint8_t context, 
TPhyStatus phyStatus, 
const TUniData uniData);
#line 687
static inline  result_t PhyCC2420M$IStdControl$init(void);
#line 714
static inline result_t PhyCC2420M$SetDefaultRegisters(void);
#line 737
static inline  result_t PhyCC2420M$IStdControl$start(void);
#line 792
static  TPhyStatus PhyCC2420M$IPhySET$SetphyCurrentChannel(TPhyChannel phyCurrentChannel);
#line 851
static  result_t PhyCC2420M$IPhyAttr$SetshortAddress(uint16_t shortAddress);






static  result_t PhyCC2420M$IPhyAttr$SetpanId(uint16_t panId);






static  result_t PhyCC2420M$IPhyAttr$SetextendedAddress(uint64_t extendedAddress);
#line 880
static inline  result_t PhyCC2420M$IPhyAttr$SetAutoAck(bool autoAck);





static inline   result_t PhyCC2420M$IPhyTxDATA$default$Confirm(uint8_t context, TPhyPoolHandle handle, 
TPhyStatus phyStatus, TUniData uniData);

static inline    void PhyCC2420M$IPhyTxFIFO$default$WriteDone(uint8_t context, TPhyPoolHandle handle, 
result_t result, TUniData uniData);



static inline   result_t PhyCC2420M$IPhySET_TRX_STATE$default$Confirm(uint8_t context, TPhyStatus phyStatus, TUniData uniData);






static inline   result_t PhyCC2420M$IChipconRAM$writeDone(uint16_t addr, uint8_t _length, uint8_t *data);
# 10 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static   uint8_t *PhyFrameM$IPhyFrame$GetPPDU(const TPhyFrame *const pPhyFrame);






static inline   uint8_t PhyFrameM$IPhyFrame$GetMPDULength(const TPhyFrame *const pPhyFrame);






static   uint8_t PhyFrameM$IPhyFrame$GetPPDULength(const TPhyFrame *const pPhyFrame);
#line 38
static   result_t PhyFrameM$IPhyFrame$AppendOctet(TPhyFrame *const pPhyFrame, const uint8_t octet);










static   result_t PhyFrameM$IPhyFrame$ReadOctet(TPhyFrame *const pPhyFrame, uint8_t *const pOctet);
#line 61
static inline   result_t PhyFrameM$IPhyFrame$ResetPosition(TPhyFrame *const pPhyFrame);








static   result_t PhyFrameM$IPhyFrame$Erase(TPhyFrame *const pPhyFrame);









static inline   result_t PhyFrameM$IPhyFrame$CalcCRC(TPhyFrame *const pPhyFrame);








static inline   result_t PhyFrameM$IPhyFrame$CheckCRC(TPhyFrame *const pPhyFrame);










static   result_t PhyFrameM$IPhyFrame$SetTimeStamp(TPhyFrame *const pPhyFrame, TPhyTimeStamp stamp);








static inline   result_t PhyFrameM$IPhyFrame$GetTimeStamp(const TPhyFrame *const pPhyFrame, TPhyTimeStamp *const pStamp);
# 11 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static   void PhyPoolM$FreePoolItem(TPhyPoolHandle arg_0x40c45330);
# 19 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   result_t PhyPoolM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b3d418);
# 15 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
enum PhyPoolM$__nesc_unnamed4393 {
#line 15
  PhyPoolM$POOL_SIZE = 5
};
typedef TPhyPoolHandle PhyPoolM$TPoolIndex;





#line 19
struct PhyPoolM$__nesc_unnamed4394 {
  bool free;
  uint16_t uniData;
  TPhyFrame frame;
} PhyPoolM$pool[PhyPoolM$POOL_SIZE];

static   result_t PhyPoolM$IPhyPool$Alloc(
const uint8_t size, 
const uint16_t uniData, 
TPhyPoolHandle *const pHandle);
#line 49
static   TPhyFrame *PhyPoolM$IPhyPool$GetFrame(const TPhyPoolHandle handle);
#line 63
static inline   result_t PhyPoolM$IPhyPool$GetUniData(const TPhyPoolHandle handle, 
uint16_t *const pUniData);
#line 77
static   result_t PhyPoolM$IPhyPool$Free(const TPhyPoolHandle handle);
#line 94
static inline void PhyPoolM$ResetPool(void);







static inline  void PhyPoolM$IPhyPoolReset$reset(void);





static inline  result_t PhyPoolM$IStdControl$init(void);





static inline  result_t PhyPoolM$IStdControl$start(void);
# 50 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
static   result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t arg_0x40b97118, uint8_t *arg_0x40b972b8);
#line 39
static   result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t arg_0x40b989a8, uint8_t *arg_0x40b98b48);
# 191 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
static   result_t HPLCC2420M$USARTControl$isTxEmpty(void);
#line 115
static   void HPLCC2420M$USARTControl$disableSPI(void);
#line 172
static   result_t HPLCC2420M$USARTControl$disableRxIntr(void);
static   result_t HPLCC2420M$USARTControl$disableTxIntr(void);
#line 135
static   void HPLCC2420M$USARTControl$setModeSPI(void);
#line 180
static   result_t HPLCC2420M$USARTControl$isTxIntrPending(void);
#line 202
static   result_t HPLCC2420M$USARTControl$tx(uint8_t arg_0x40c4b760);






static   uint8_t HPLCC2420M$USARTControl$rx(void);
#line 185
static   result_t HPLCC2420M$USARTControl$isRxIntrPending(void);
# 49 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
static   result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t arg_0x40b90e80, uint8_t arg_0x40b8f030, uint8_t *arg_0x40b8f1d0);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static   result_t HPLCC2420M$BusArbitration$releaseBus(void);
#line 37
static   result_t HPLCC2420M$BusArbitration$getBus(void);
# 57 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
 uint8_t *HPLCC2420M$txbuf;
 uint8_t *HPLCC2420M$rxbuf;
 uint8_t *HPLCC2420M$rambuf;

 uint8_t HPLCC2420M$txlen;
 uint8_t HPLCC2420M$rxlen;
 uint8_t HPLCC2420M$ramlen;
 uint16_t HPLCC2420M$ramaddr;








 
#line 68
struct HPLCC2420M$__nesc_unnamed4395 {
  bool enabled : 1;
  bool busy : 1;
  bool rxbufBusy : 1;
  bool txbufBusy : 1;
} HPLCC2420M$f;





static inline uint8_t HPLCC2420M$adjustStatusByte(uint8_t status);



static inline  result_t HPLCC2420M$StdControl$init(void);
#line 96
static inline  result_t HPLCC2420M$StdControl$start(void);
#line 111
static inline  result_t HPLCC2420M$StdControl$stop(void);
#line 127
static   uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t addr);
#line 162
static   uint8_t HPLCC2420M$HPLCC2420$write(uint8_t addr, uint16_t data);
#line 205
static   uint16_t HPLCC2420M$HPLCC2420$read(uint8_t addr);
#line 288
static inline  void HPLCC2420M$signalRAMWr(void);



static   result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t addr, uint8_t _length, uint8_t *buffer);
#line 322
static inline  void HPLCC2420M$signalRXFIFO(void);
#line 335
static inline   result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t length, uint8_t *data);
#line 395
static inline  void HPLCC2420M$signalTXFIFO(void);
#line 416
static inline   result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t length, uint8_t *data);
#line 468
static inline  result_t HPLCC2420M$BusArbitration$busFree(void);
# 43 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLI2CInterrupt.nc"
static   void HPLUSART0M$HPLI2CInterrupt$fired(void);
# 53 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
static   result_t HPLUSART0M$USARTData$rxDone(uint8_t arg_0x40c8e570);
#line 46
static   result_t HPLUSART0M$USARTData$txDone(void);
# 47 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
 static volatile uint8_t HPLUSART0M$IE1 __asm ("0x0000");
 static volatile uint8_t HPLUSART0M$ME1 __asm ("0x0004");
 static volatile uint8_t HPLUSART0M$IFG1 __asm ("0x0002");
 static volatile uint8_t HPLUSART0M$U0TCTL __asm ("0x0071");
 static volatile uint8_t HPLUSART0M$U0TXBUF __asm ("0x0077");


uint16_t HPLUSART0M$l_br;

uint8_t HPLUSART0M$l_ssel;

void sig_UART0RX_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(18))) ;




void sig_UART0TX_VECTOR(void)  __attribute((wakeup)) __attribute((interrupt(16))) ;






static inline    void HPLUSART0M$HPLI2CInterrupt$default$fired(void);

static inline   bool HPLUSART0M$USARTControl$isSPI(void);








static inline   bool HPLUSART0M$USARTControl$isUART(void);










static inline   bool HPLUSART0M$USARTControl$isUARTtx(void);










static inline   bool HPLUSART0M$USARTControl$isUARTrx(void);










static inline   bool HPLUSART0M$USARTControl$isI2C(void);










static inline   msp430_usartmode_t HPLUSART0M$USARTControl$getMode(void);
#line 172
static inline   void HPLUSART0M$USARTControl$disableUART(void);
#line 205
static inline   void HPLUSART0M$USARTControl$disableSPI(void);
#line 218
static inline   void HPLUSART0M$USARTControl$disableI2C(void);






static   void HPLUSART0M$USARTControl$setModeSPI(void);
#line 424
static   result_t HPLUSART0M$USARTControl$isTxIntrPending(void);







static inline   result_t HPLUSART0M$USARTControl$isTxEmpty(void);






static   result_t HPLUSART0M$USARTControl$isRxIntrPending(void);







static inline   result_t HPLUSART0M$USARTControl$disableRxIntr(void);




static inline   result_t HPLUSART0M$USARTControl$disableTxIntr(void);
#line 473
static inline   result_t HPLUSART0M$USARTControl$tx(uint8_t data);




static   uint8_t HPLUSART0M$USARTControl$rx(void);







static inline    result_t HPLUSART0M$USARTData$default$txDone(void);

static inline    result_t HPLUSART0M$USARTData$default$rxDone(uint8_t data);
# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t HPLCC2420InterruptM$FIFO$fired(void);
#line 51
static   result_t HPLCC2420InterruptM$FIFOP$fired(void);
# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void HPLCC2420InterruptM$CCAInterrupt$clear(void);
#line 35
static   void HPLCC2420InterruptM$CCAInterrupt$disable(void);
# 36 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void HPLCC2420InterruptM$SFDControl$setControlAsCapture(bool arg_0x407417d0);

static   void HPLCC2420InterruptM$SFDControl$enableEvents(void);
static   void HPLCC2420InterruptM$SFDControl$disableEvents(void);
#line 32
static   void HPLCC2420InterruptM$SFDControl$clearPendingInterrupt(void);
# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void HPLCC2420InterruptM$FIFOInterrupt$clear(void);
#line 35
static   void HPLCC2420InterruptM$FIFOInterrupt$disable(void);
#line 54
static   void HPLCC2420InterruptM$FIFOInterrupt$edge(bool arg_0x40cd4068);
#line 30
static   void HPLCC2420InterruptM$FIFOInterrupt$enable(void);
# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t HPLCC2420InterruptM$CCA$fired(void);
# 56 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void HPLCC2420InterruptM$SFDCapture$clearOverflow(void);
#line 51
static   bool HPLCC2420InterruptM$SFDCapture$isOverflowPending(void);
# 53 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
static   result_t HPLCC2420InterruptM$SFD$captured(uint16_t arg_0x40b95360);
# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void HPLCC2420InterruptM$FIFOPInterrupt$clear(void);
#line 35
static   void HPLCC2420InterruptM$FIFOPInterrupt$disable(void);
#line 54
static   void HPLCC2420InterruptM$FIFOPInterrupt$edge(bool arg_0x40cd4068);
#line 30
static   void HPLCC2420InterruptM$FIFOPInterrupt$enable(void);
# 65 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420InterruptM.nc"
static   result_t HPLCC2420InterruptM$FIFOP$startWait(bool low_to_high);
#line 78
static   result_t HPLCC2420InterruptM$FIFOP$disable(void);










static inline   void HPLCC2420InterruptM$FIFOPInterrupt$fired(void);
#line 106
static   result_t HPLCC2420InterruptM$FIFO$startWait(bool low_to_high);
#line 119
static   result_t HPLCC2420InterruptM$FIFO$disable(void);










static inline   void HPLCC2420InterruptM$FIFOInterrupt$fired(void);
#line 171
static inline   void HPLCC2420InterruptM$CCAInterrupt$fired(void);









static inline    result_t HPLCC2420InterruptM$CCA$default$fired(void);



static   result_t HPLCC2420InterruptM$SFD$enableCapture(bool low_to_high);
#line 200
static   result_t HPLCC2420InterruptM$SFD$disable(void);








static inline   void HPLCC2420InterruptM$SFDCapture$captured(uint16_t time);
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








static inline    void MSP430InterruptM$Port11$default$fired(void);
static inline    void MSP430InterruptM$Port12$default$fired(void);


static inline    void MSP430InterruptM$Port15$default$fired(void);
static inline    void MSP430InterruptM$Port16$default$fired(void);
static inline    void MSP430InterruptM$Port17$default$fired(void);

static inline    void MSP430InterruptM$Port20$default$fired(void);

static inline    void MSP430InterruptM$Port22$default$fired(void);
static inline    void MSP430InterruptM$Port23$default$fired(void);
static inline    void MSP430InterruptM$Port24$default$fired(void);
static inline    void MSP430InterruptM$Port25$default$fired(void);
static inline    void MSP430InterruptM$Port26$default$fired(void);
static inline    void MSP430InterruptM$Port27$default$fired(void);

static inline    void MSP430InterruptM$NMI$default$fired(void);
static inline    void MSP430InterruptM$OF$default$fired(void);
static inline    void MSP430InterruptM$ACCV$default$fired(void);

static inline   void MSP430InterruptM$Port10$enable(void);


static inline   void MSP430InterruptM$Port13$enable(void);






static inline   void MSP430InterruptM$Port21$enable(void);
#line 146
static inline   void MSP430InterruptM$Port10$disable(void);


static inline   void MSP430InterruptM$Port13$disable(void);
static inline   void MSP430InterruptM$Port14$disable(void);





static inline   void MSP430InterruptM$Port21$disable(void);
#line 177
static inline   void MSP430InterruptM$Port10$clear(void);
static inline   void MSP430InterruptM$Port11$clear(void);
static inline   void MSP430InterruptM$Port12$clear(void);
static inline   void MSP430InterruptM$Port13$clear(void);
static inline   void MSP430InterruptM$Port14$clear(void);
static inline   void MSP430InterruptM$Port15$clear(void);
static inline   void MSP430InterruptM$Port16$clear(void);
static inline   void MSP430InterruptM$Port17$clear(void);

static inline   void MSP430InterruptM$Port20$clear(void);
static inline   void MSP430InterruptM$Port21$clear(void);
static inline   void MSP430InterruptM$Port22$clear(void);
static inline   void MSP430InterruptM$Port23$clear(void);
static inline   void MSP430InterruptM$Port24$clear(void);
static inline   void MSP430InterruptM$Port25$clear(void);
static inline   void MSP430InterruptM$Port26$clear(void);
static inline   void MSP430InterruptM$Port27$clear(void);

static inline   void MSP430InterruptM$NMI$clear(void);
static inline   void MSP430InterruptM$OF$clear(void);
static inline   void MSP430InterruptM$ACCV$clear(void);
#line 221
static inline   void MSP430InterruptM$Port10$edge(bool l2h);
#line 239
static inline   void MSP430InterruptM$Port13$edge(bool l2h);
#line 276
static inline   void MSP430InterruptM$Port21$edge(bool l2h);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static  result_t BusArbitrationM$BusArbitration$busFree(
# 31 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
uint8_t arg_0x40d9aa88);





uint8_t BusArbitrationM$state;
uint8_t BusArbitrationM$busid;
bool BusArbitrationM$isBusReleasedPending;
enum BusArbitrationM$__nesc_unnamed4396 {
#line 40
  BusArbitrationM$BUS_IDLE, BusArbitrationM$BUS_BUSY, BusArbitrationM$BUS_OFF
};
static inline  void BusArbitrationM$busReleased(void);
#line 54
static inline  result_t BusArbitrationM$StdControl$init(void);







static inline  result_t BusArbitrationM$StdControl$start(void);
#line 78
static inline  result_t BusArbitrationM$StdControl$stop(void);
#line 94
static   result_t BusArbitrationM$BusArbitration$getBus(uint8_t id);
#line 108
static   result_t BusArbitrationM$BusArbitration$releaseBus(uint8_t id);
#line 125
static inline   result_t BusArbitrationM$BusArbitration$default$busFree(uint8_t id);
# 8 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressM.nc"
static inline  TMacAddressMode MacAddressM$IMacAddress$GetAddressMode(const TMacAddress macAddress);




static  result_t MacAddressM$IMacAddress$GetShortAddress(TMacShortAddress *const pMacShortAddress, 
const TMacAddress macAddress);










static  result_t MacAddressM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const pMacExtendedAddress, 
const TMacAddress macAddress);










static  result_t MacAddressM$IMacAddress$SetShortAddress(TMacAddress *const pMacAddress, 
const TMacShortAddress macShortAddress);









static  result_t MacAddressM$IMacAddress$SetExtendedAddress(TMacAddress *const pMacAddress, 
const TMacExtendedAddress macExtendedAddress);









static  result_t MacAddressM$IMacAddress$SetEmptyAddress(TMacAddress *const pMacAddress);









static inline  bool MacAddressM$IMacAddress$Equal(const TMacAddress *const pMacAddress1, 
const TMacAddress *const pMacAddress2);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacFrameFormatM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409bc8b0, const TMacAddress arg_0x409bca60);
#line 8
static  result_t MacFrameFormatM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x409bc1e8, const TMacAddress arg_0x409bc398);







static  result_t MacFrameFormatM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x409bad90);
#line 14
static  result_t MacFrameFormatM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x409ba6d0, const TMacExtendedAddress arg_0x409ba890);
#line 12
static  result_t MacFrameFormatM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x409ba010, const TMacShortAddress arg_0x409ba1c8);
#line 6
static  TMacAddressMode MacFrameFormatM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409bdc98);
# 17 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   result_t MacFrameFormatM$IPhyFrame$ResetPosition(TPhyFrame *const arg_0x40b3fed8);







static   result_t MacFrameFormatM$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b3c3c8, TPhyTimeStamp arg_0x40b3c550);
#line 19
static   result_t MacFrameFormatM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b3d418);
#line 13
static   result_t MacFrameFormatM$IPhyFrame$AppendOctet(TPhyFrame *const arg_0x40b3f118, const uint8_t arg_0x40b3f2b8);

static   result_t MacFrameFormatM$IPhyFrame$ReadOctet(TPhyFrame *const arg_0x40b3f7d8, uint8_t *const arg_0x40b3f9b8);
#line 27
static   result_t MacFrameFormatM$IPhyFrame$GetTimeStamp(const TPhyFrame *const arg_0x40b3ca90, TPhyTimeStamp *const arg_0x40b3cc78);
#line 23
static   result_t MacFrameFormatM$IPhyFrame$CheckCRC(TPhyFrame *const arg_0x40b3de40);
#line 21
static   result_t MacFrameFormatM$IPhyFrame$CalcCRC(TPhyFrame *const arg_0x40b3d928);
# 15 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static  result_t MacFrameFormatM$IMacFrame$SetFrameType(TMacFrame *const pMacFrame, const TMacFrameType macFrameType);








static  result_t MacFrameFormatM$IMacFrame$GetFrameType(const TMacFrame *const pMacFrame, TMacFrameType *const pFrameType);








static  result_t MacFrameFormatM$IMacFrame$SetSecurityEnabled(TMacFrame *const pMacFrame, const bool securityEnabled);








static inline  result_t MacFrameFormatM$IMacFrame$GetSecurityEnabled(const TMacFrame *const pMacFrame, 
bool *const pSecurityEnabled);








static  result_t MacFrameFormatM$IMacFrame$SetFramePending(TMacFrame *const pMacFrame, const bool framePending);
#line 71
static  result_t MacFrameFormatM$IMacFrame$SetAckRequest(TMacFrame *const pMacFrame, const bool ackRequest);








static  result_t MacFrameFormatM$IMacFrame$GetAckRequest(const TMacFrame *const pMacFrame, bool *const pAckRequest);








static  result_t MacFrameFormatM$IMacFrame$SetIntraPAN(TMacFrame *const pMacFrame, const bool intraPAN);








static  result_t MacFrameFormatM$IMacFrame$GetIntraPAN(const TMacFrame *const pMacFrame, bool *const pIntraPAN);








static  result_t MacFrameFormatM$IMacFrame$SetSequenceNumber(TMacFrame *const pMacFrame, 
const TMacSequenceNumber sequenceNumber);








static  result_t MacFrameFormatM$IMacFrame$GetSequenceNumber(const TMacFrame *const pMacFrame, 
TMacSequenceNumber *const pSequenceNumber);








static  result_t MacFrameFormatM$IMacFrame$SetDstPANId(TMacFrame *const pMacFrame, const TMacPANId dstPANId);








static inline  result_t MacFrameFormatM$IMacFrame$GetDstPANId(const TMacFrame *const pMacFrame, TMacPANId *const pDstPANId);








static  result_t MacFrameFormatM$IMacFrame$SetDstAddress(TMacFrame *const pMacFrame, const TMacAddress dstAddress);
#line 169
static  result_t MacFrameFormatM$IMacFrame$GetDstAddress(const TMacFrame *const pMacFrame, TMacAddress *const pDstAddress);
#line 184
static  result_t MacFrameFormatM$IMacFrame$SetSrcPANId(TMacFrame *const pMacFrame, 
const TMacPANId srcPANId);








static inline  result_t MacFrameFormatM$IMacFrame$GetSrcPANId(const TMacFrame *const pMacFrame, TMacPANId *const pSrcPANId);








static  result_t MacFrameFormatM$IMacFrame$SetSrcAddress(TMacFrame *const pMacFrame, const TMacAddress srcAddress);
#line 227
static  result_t MacFrameFormatM$IMacFrame$GetSrcAddress(const TMacFrame *const pMacFrame, TMacAddress *const pSrcAddress);
#line 242
static  result_t MacFrameFormatM$IMacFrame$SetPayload(TMacFrame *const pMacFrame, 
const uint8_t *const pFramePayload, TMacPayloadLength payloadLength);
#line 258
static  result_t MacFrameFormatM$IMacFrame$GetPayloadLength(const TMacFrame *const pMacFrame, 
TMacPayloadLength *const pPayloadLength);








static  result_t MacFrameFormatM$IMacFrame$GetPayload(const TMacFrame *const pMacFrame, 
uint8_t *const pFramePayload);










static inline  result_t MacFrameFormatM$IMacFrame$SetTimeStamp(TMacFrame *const pMacFrame, TMacTimeStamp timeStamp);








static inline  result_t MacFrameFormatM$IMacFrame$GetTimeStamp(const TMacFrame *const pMacFrame, 
TMacTimeStamp *const pTimeStamp);








static inline  result_t MacFrameFormatM$IMacFrame$GetLinkQuality(const TMacFrame *const pMacFrame, 
TMacLinkQuality *const pLinkQuality);
#line 315
static  result_t MacFrameFormatM$IMacFrame$Pack(const TMacFrame *const pMacFrame, TPhyFrame *const pPhyFrame);
#line 364
static inline  result_t MacFrameFormatM$IMacFrame$UnPack(TMacFrame *const pMacFrame, TPhyFrame *const pPhyFrame);
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
static   uint16_t MacCommonAttrM$IMacRandom$Rand(void);
# 7 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
static  result_t MacCommonAttrM$IPhyAttr$SetextendedAddress(uint64_t arg_0x40b384b0);
#line 5
static  result_t MacCommonAttrM$IPhyAttr$SetpanId(uint16_t arg_0x40b38010);
#line 3
static  result_t MacCommonAttrM$IPhyAttr$SetshortAddress(uint16_t arg_0x40b39ad8);
# 23 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
#line 16
struct MacCommonAttrM$__nesc_unnamed4397 {
  TMacAckWaitDuration macAckWaitDuration;
  TMacSecurityMode macSecurityMode;
  TMacPANId macPANId;
  TMacShortAddress macShortAddress;
  TMacDSN macDSN;
  bool macRxOnWhenIdle;
} MacCommonAttrM$pib;


static  TMacDSN MacCommonAttrM$IMacCommonAttr$GetmacDSN(TMacStatus *const pMacStatus);





static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacDSN(TMacDSN macDSN);




static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacDSN(void);






static inline  TMacAckWaitDuration MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const pMacStatus);
#line 64
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void);
#line 83
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void);
#line 100
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void);






static  TMacPANId MacCommonAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const pMacStatus);





static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacPANId(TMacPANId macPANId);







static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPANId(void);





static  TMacShortAddress MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const pMacStatus);





static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacShortAddress(TMacShortAddress macShortAddress);







static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void);
#line 154
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(bool macRxOnWhenIdle);




static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void);





static inline  result_t MacCommonAttrM$IStdControl$init(void);










static inline  result_t MacCommonAttrM$IStdControl$start(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime MacRandomM$ILocalTime$Read(void);
# 11 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacRandomM.nc"
uint16_t MacRandomM$randomValue;

static inline  result_t MacRandomM$IStdControl$init(void);









static inline  result_t MacRandomM$IStdControl$start(void);


static   uint16_t MacRandomM$IMacRandom$Rand(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
static   result_t MacSuperframesM$Timer$Stop(void);
#line 7
static   result_t MacSuperframesM$Timer$SetOneShotAt(TSysTime arg_0x40a317e0, TUniData arg_0x40a31968);
# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
static   void MacSuperframesM$ParentCAP$BeginCAP(void);

static   void MacSuperframesM$ParentCAP$EndCAP(void);
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   TMilliSec MacSuperframesM$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a35558);
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacBeaconOrder MacSuperframesM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90);



static  TMacSuperframeOrder MacSuperframesM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e80a60);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC_LOSS.nc"
static  void MacSuperframesM$IMacSYNC_LOSS$Indication(
TMacStatus arg_0x40e9fb50);
# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
static   void MacSuperframesM$OwnCAP$BeginCAP(void);

static   void MacSuperframesM$OwnCAP$EndCAP(void);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t MacSuperframesM$IPhyPool$Alloc(const uint8_t arg_0x40b5e6b8, const uint16_t arg_0x40b5e860, 
TPhyPoolHandle *const arg_0x40b5ea58);

static   TPhyFrame *MacSuperframesM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010);



static   result_t MacSuperframesM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b5cbc0);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static   result_t MacSuperframesM$IPhyTxDATA$Request(const TPhyPoolHandle arg_0x40b2ab78, 
const TUniData arg_0x40b2ad30);
# 23 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static  void MacSuperframesM$packBeacon(TPhyFrame *arg_0x40e71c50, uint64_t *arg_0x40e71df8);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime MacSuperframesM$ILocalTime$Read(void);
# 35 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static  void MacSuperframesM$sleepIndication(TMilliSec arg_0x40e6d338);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
static   result_t MacSuperframesM$IPhyTxFIFO$Write(TPhyPoolHandle arg_0x40b2edd8, TUniData arg_0x40b29010);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  uint64_t MacSuperframesM$ILocalTime64$getLocalTimeAt(TSysTime arg_0x40a39010);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacSuperframesM$IPhyTrxParent$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960);
#line 6
static   result_t MacSuperframesM$IPhyTrxOwn$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960);
# 40 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
 TSysTime MacSuperframesM$sf_offset;
 TSysTime MacSuperframesM$timestamp;
 TSysTime MacSuperframesM$next_sf_start;

 TSysTime MacSuperframesM$sf_duration;
 TSysTime MacSuperframesM$beacon_interval;


 uint8_t MacSuperframesM$bad_beacons = 0;

 void (*MacSuperframesM$current_handler)(void);

 TPhyPoolHandle MacSuperframesM$beacon_handle;

 bool MacSuperframesM$accept_beacon;
bool MacSuperframesM$beacon_received;
bool MacSuperframesM$parent_active = FALSE;
bool MacSuperframesM$own_active = FALSE;
bool MacSuperframesM$beacon_allocated = FALSE;
 enum MacSuperframesM$__nesc_unnamed4398 {
#line 59
  MacSuperframesM$SF_PARENT, MacSuperframesM$SF_OWN
} 
#line 59
MacSuperframesM$current_sf;

enum MacSuperframesM$__nesc_unnamed4399 {
  MacSuperframesM$PARENT_SF_PREPARATION_TIME = 
  400
   + 250, 
  MacSuperframesM$OWN_SF_PREPARATION_TIME = 400
};


static inline void MacSuperframesM$calcIntervals(void);
static void MacSuperframesM$newCycle(void);
static inline void MacSuperframesM$ownPreparationHandler(void);
static inline void MacSuperframesM$beaconTransmitHandler(void);
static inline void MacSuperframesM$parentPreparationHandler(void);
static inline void MacSuperframesM$parentBeginCAPHandler(void);
static inline void MacSuperframesM$parentEndCAPHandler(void);
static inline void MacSuperframesM$ownBeginCAPHandler(void);
static inline void MacSuperframesM$ownEndCAPHandler(void);
static inline  void MacSuperframesM$createBeacon(void);
static inline  void MacSuperframesM$syncLoss(void);
#line 105
static inline  result_t MacSuperframesM$runOwnSF(TSysTime offset);
#line 147
static void MacSuperframesM$newCycle(void);
#line 195
static inline void MacSuperframesM$ownPreparationHandler(void);
#line 210
static inline  result_t MacSuperframesM$IPhyTrxOwn$Confirm(TPhyStatus status, TUniData unidata);









static inline   void MacSuperframesM$IPhyTxFIFO$WriteDone(TPhyPoolHandle handle, 
result_t result, TUniData uniData);





static inline void MacSuperframesM$beaconTransmitHandler(void);
#line 241
static inline  result_t MacSuperframesM$IPhyTxDATA$Confirm(const TPhyPoolHandle handle, TPhyStatus phyStatus, TUniData unidata);








static inline void MacSuperframesM$ownBeginCAPHandler(void);










static inline void MacSuperframesM$ownEndCAPHandler(void);
#line 280
static inline void MacSuperframesM$parentPreparationHandler(void);
#line 294
static inline  result_t MacSuperframesM$IPhyTrxParent$Confirm(TPhyStatus status, TUniData unidata);




static inline void MacSuperframesM$parentBeginCAPHandler(void);










static inline void MacSuperframesM$parentEndCAPHandler(void);
#line 362
static inline  void MacSuperframesM$createBeacon(void);
#line 380
static inline void MacSuperframesM$calcIntervals(void);







static inline   result_t MacSuperframesM$Timer$Fired(TUniData unidata);
#line 402
static inline  result_t MacSuperframesM$StdControl$init(void);



static inline  result_t MacSuperframesM$StdControl$start(void);



static inline  result_t MacSuperframesM$StdControl$stop(void);










static inline  void MacSuperframesM$Reset$reset(void);






static inline  void MacSuperframesM$syncLoss(void);







static inline   void MacSuperframesM$IMacSYNC_LOSS$default$Indication(TMacStatus status);



static inline    void MacSuperframesM$ParentCAP$default$BeginCAP(void);

static inline    void MacSuperframesM$ParentCAP$default$EndCAP(void);






static inline   void MacSuperframesM$default$sleepIndication(TMilliSec sleep_duration);
# 10 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static  void MacSuperframeAttrC$BeaconOrderChanged(uint8_t arg_0x40ea5778);


enum MacSuperframeAttrC$__nesc_unnamed4400 {
  MacSuperframeAttrC$MAX_BEACON_ORDER = 15, 
  MacSuperframeAttrC$MAX_SUPERFRAME_ORDER = 15
};

TMacBeaconOrder MacSuperframeAttrC$macBeaconOrder = MacSuperframeAttrC$MAX_BEACON_ORDER;
TMacSuperframeOrder MacSuperframeAttrC$macSuperframeOrder = MacSuperframeAttrC$MAX_SUPERFRAME_ORDER;

static inline  void MacSuperframeAttrC$Reset$reset(void);





static inline  TMacBeaconOrder MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(
TMacStatus *const pMacStatus);





static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(
TMacBeaconOrder beaconOrder);







static inline  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void);





static inline  TMacSuperframeOrder MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(
TMacStatus *const pMacStatus);





static inline  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetmacSuperframeOrder(
TMacSuperframeOrder superframeOrder);




static inline  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void);
# 13 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
static  uint8_t *MacBeaconPackerM$IMacBeaconAttr$GetmacBeaconPayload(
uint8_t *const arg_0x40ec53b8, 
TMacStatus *const arg_0x40ec55b8);





static  TMacBSN MacBeaconPackerM$IMacBeaconAttr$GetmacBSN(TMacStatus *const arg_0x40ec3480);
static  TMacStatus MacBeaconPackerM$IMacBeaconAttr$SetmacBSN(TMacBSN arg_0x40ec3908);
#line 11
static  uint8_t MacBeaconPackerM$IMacBeaconAttr$GetmacBeaconPayloadLength(
TMacStatus *const arg_0x40ec6e20);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacShortAddress MacBeaconPackerM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dc99a8);
#line 6
static  TMacPANId MacBeaconPackerM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dcacf0);
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacBeaconOrder MacBeaconPackerM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90);



static  TMacSuperframeOrder MacBeaconPackerM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e80a60);
# 27 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacBeaconPackerM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40da5b38, const TMacAddress arg_0x40da5ce8);
#line 17
static  result_t MacBeaconPackerM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40da80a8, const bool arg_0x40da8250);
#line 30
static  result_t MacBeaconPackerM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40da3908, const TMacPANId arg_0x40da3ab8);





static  result_t MacBeaconPackerM$IMacFrame$SetPayload(TMacFrame *const arg_0x40dd2510, 
const uint8_t *const arg_0x40dd2728, TMacPayloadLength arg_0x40dd28c0);
#line 8
static  result_t MacBeaconPackerM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40dac708, const bool arg_0x40dac8b0);
#line 44
static  TMacRawFrameLength MacBeaconPackerM$IMacFrame$Pack(const TMacFrame *const arg_0x40dd1c38, TPhyFrame *const arg_0x40dd1e28);
#line 11
static  result_t MacBeaconPackerM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40dab510, const bool arg_0x40dab6b8);
#line 33
static  result_t MacBeaconPackerM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40da2700, const TMacAddress arg_0x40da28b0);
#line 14
static  result_t MacBeaconPackerM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40da92d8, const bool arg_0x40da9480);
#line 5
static  result_t MacBeaconPackerM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40dad8e8, const TMacFrameType arg_0x40dada98);
#line 20
static  result_t MacBeaconPackerM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40da8e60, const TMacSequenceNumber arg_0x40da6030);
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacBeaconPackerM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x409bad90);
#line 14
static  result_t MacBeaconPackerM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x409ba6d0, const TMacExtendedAddress arg_0x409ba890);
#line 12
static  result_t MacBeaconPackerM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x409ba010, const TMacShortAddress arg_0x409ba1c8);
# 18 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
static  bool MacBeaconPackerM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40eec9e8);
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
static  bool MacBeaconPackerM$IMacCAPAttr$GetmacBattLifeExt(TMacStatus *const arg_0x40ebc988);
# 20 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconPackerM.nc"
static  uint8_t MacBeaconPackerM$getLastBSN(void);









static  void MacBeaconPackerM$packBeacon(TPhyFrame *phyFrame, uint64_t *ptime);
#line 217
static inline  void MacBeaconPackerM$IMacPool$FreeDone(const TMacPoolHandle macPoolHandle, const TMacStatus status);
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
static   uint16_t MacCAPM$IMacRandom$Rand(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacCAPM$IBackoff$Stop(void);
#line 7
static  result_t MacCAPM$IBackoff$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868);
# 44 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  TMacRawFrameLength MacCAPM$IMacFrame$Pack(const TMacFrame *const arg_0x40dd1c38, TPhyFrame *const arg_0x40dd1e28);
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacBeaconOrder MacCAPM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90);



static  TMacSuperframeOrder MacCAPM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e80a60);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t MacCAPM$IPhyPool$Alloc(const uint8_t arg_0x40b5e6b8, const uint16_t arg_0x40b5e860, 
TPhyPoolHandle *const arg_0x40b5ea58);

static   TPhyFrame *MacCAPM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010);



static   result_t MacCAPM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b5cbc0);
#line 10
static   result_t MacCAPM$IPhyPool$GetUniData(const TPhyPoolHandle arg_0x40b5c500, uint16_t *const arg_0x40b5c6f0);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static   result_t MacCAPM$IPhyTxDATA$Request(const TPhyPoolHandle arg_0x40b2ab78, 
const TUniData arg_0x40b2ad30);
# 50 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  void MacCAPM$SendDone(const uint8_t arg_0x40f16328, uint8_t arg_0x40f164a8, TMacStatus arg_0x40f16638, const TUniData arg_0x40f167e0);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime MacCAPM$ILocalTime$Read(void);
# 51 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$Receive(const uint8_t arg_0x40f16ca0, TMacFrame *const arg_0x40f16e90);
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   uint8_t MacCAPM$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b406b8);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacCAPM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyCCA.nc"
static   TPhyStatus MacCAPM$IPhyCCA$Request(void);
# 58 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
#line 55
typedef enum MacCAPM$__nesc_unnamed4401 {
  MacCAPM$CAP_STATE_PASSIVE, 
  MacCAPM$CAP_STATE_ACTIVE
} MacCAPM$TMacCAPState;
#line 71
#line 60
typedef enum MacCAPM$__nesc_unnamed4402 {
  MacCAPM$CAP_SEND_OFF, 
  MacCAPM$CAP_SEND_IDLE, 
  MacCAPM$CAP_SEND_PENDING, 
  MacCAPM$CAP_SEND_BACKOFF, 
  MacCAPM$CAP_SEND_CCA, 
  MacCAPM$CAP_SEND_TX_ON, 
  MacCAPM$CAP_SEND_SUCCESS, 
  MacCAPM$CAP_SEND_FAILURE, 
  MacCAPM$CAP_SEND_PHY, 
  MacCAPM$CAP_SEND_PHY_DONE
} MacCAPM$TMacCAPSendState;

enum MacCAPM$__nesc_unnamed4403 {
  MacCAPM$NUM_SUPERFRAMES = 1U
};






bool MacCAPM$macBattLifeExt = FALSE;
TMacBattLifeExtPeriods MacCAPM$macBattLifeExtPeriods = MAC_BATT_LIFE_EXT_PERIOD_6;
TMacMaxCSMABackoffs MacCAPM$macMaxCSMABackoffs = MAC_MAX_CSMA_BACKOFFS_4;
TMacBackoffExponent MacCAPM$macMinBE = MAC_BACKOFF_EXPONENT_3;
#line 97
 
#line 87
struct MacCAPM$__nesc_unnamed4404 {
  TSysTime beginCAPTime;
  TPhyPoolHandle handle;
  TMacBackoffExponent backoffExponent;
  TMacCAPType foo1;
  MacCAPM$TMacCAPState state;
  uint8_t backoffNumber;
  uint8_t backoffRemain;
  uint8_t context;
  MacCAPM$TMacCAPSendState sendState;
} MacCAPM$cap[MacCAPM$NUM_SUPERFRAMES];

TMacSuperframeMode MacCAPM$sfMode;
static inline TSysTime MacCAPM$calcSendTime(uint8_t sfIndex);
static result_t MacCAPM$StartSending(uint8_t sfIndex);
static result_t MacCAPM$SetBackoff(uint8_t sfIndex);
static inline result_t MacCAPM$PutToPhyPool(const TMacFrame *const pMacFrame, const TUniData uniData, TPhyPoolHandle *const pPhyPoolHandle);
static inline TSysTime MacCAPM$calcBackoff(uint8_t capIndex);
static void MacCAPM$_SendDone(const uint8_t sfIndex, const TMacStatus macStatus);
static result_t MacCAPM$TurnOffIfInactive(TPhyStatus state);
static bool MacCAPM$enoughTime(uint8_t sfIndex);

uint8_t MacCAPM$currentSf;

bool MacCAPM$constantOn;






static inline  result_t MacCAPM$ICAPControl$init(uint8_t sfIndex);








static  result_t MacCAPM$ICAPControl$start(uint8_t sfIndex);
#line 141
static inline  void MacCAPM$beginSend(void);
#line 157
static inline   void MacCAPM$IMacCAP$BeginCAP(uint8_t sfIndex);
#line 170
static inline  void MacCAPM$TurnOffTask(void);




static inline   void MacCAPM$IMacCAP$EndCAP(uint8_t sfIndex);
#line 216
static result_t MacCAPM$SendPhyFrame(const uint8_t sfIndex, 
const uint8_t context);
#line 253
static result_t MacCAPM$StartSending(uint8_t sfIndex);
#line 274
static result_t MacCAPM$nextBackoff(uint8_t sfIndex);
#line 295
static inline  result_t MacCAPM$IBackoff$Fired(TUniData uniData);
#line 353
static bool MacCAPM$enoughTime(uint8_t sfIndex);
#line 368
static inline  result_t MacCAPM$IPhySET_TRX_STATE$Confirm(TPhyStatus phyStatus, TUniData uniData);
#line 424
static result_t MacCAPM$TurnOffIfInactive(TPhyStatus state);
#line 452
static inline  result_t MacCAPM$IPhyTxDATA$Confirm(const TPhyPoolHandle handle, 
TPhyStatus phyStatus, TUniData uniData);
#line 480
static  result_t MacCAPM$Send(const uint8_t sfIndex, 
const TMacFrame *const pMacFrame, 
const uint8_t context, 
const TUniData uniData);
#line 506
static inline  result_t MacCAPM$SendFromPhyPool(const uint8_t sfIndex, 
const TPhyPoolHandle frameHandle, 
const uint8_t context);










static void MacCAPM$_SendDone(const uint8_t sfIndex, const TMacStatus macStatus);
#line 554
static inline result_t MacCAPM$PutToPhyPool(const TMacFrame *const pMacFrame, 
const TUniData uniData, 
TPhyPoolHandle *const pPhyPoolHandle);
#line 569
static inline  void MacCAPM$BeaconOrderChanged(uint8_t bo);
#line 599
static inline TSysTime MacCAPM$calcSendTime(uint8_t sfIndex);









static inline TSysTime MacCAPM$calcBackoff(uint8_t sfIndex);







static result_t MacCAPM$SetBackoff(uint8_t sfIndex);
#line 641
static inline  void MacCAPM$Reset$reset(uint8_t sfIndex);







static inline  result_t MacCAPM$ICAPControl$stop(uint8_t sfIndex);
#line 667
static inline  result_t MacCAPM$IMacReceive$Receive(TMacFrame *const pMacFrame);








static inline  bool MacCAPM$IMacCAPAttr$GetmacBattLifeExt(TMacStatus *const pMacStatus);





static inline  TMacStatus MacCAPM$IMacCAPAttr$SetmacBattLifeExt(bool battLifeExt);




static inline  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExt(void);










static inline  TMacStatus MacCAPM$IMacCAPAttr$SetmacBattLifeExtPeriods(TMacBattLifeExtPeriods battLifeExtPeriods);




static inline  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods(void);










static inline  TMacStatus MacCAPM$IMacCAPAttr$SetmacMaxCSMABackoffs(TMacMaxCSMABackoffs maxCSMABackoffs);




static inline  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs(void);










static inline  TMacStatus MacCAPM$IMacCAPAttr$SetmacMinBE(TMacBackoffExponent minBE);




static inline  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacMinBE(void);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacRxM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200);
#line 45
static  result_t MacRxM$IMacFrame$UnPack(TMacFrame *const arg_0x40dcf3a0, TPhyFrame *const arg_0x40dcf590);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   TPhyFrame *MacRxM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacRxM$DataReceive$Receive(TMacFrame *const arg_0x40ef5318);
#line 5
static  result_t MacRxM$BeaconReceive$Receive(TMacFrame *const arg_0x40ef5318);
# 21 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacRxM.nc"
static inline  result_t MacRxM$IPhyRxDATA$Indication(const TPhyPoolHandle handle);
#line 43
static inline   result_t MacRxM$BeaconReceive$default$Receive(TMacFrame *const pMacFrame);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendDone(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f88168, 
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
TMacStatus arg_0x40ef9b08, const TUniData arg_0x40ef9cb0);
# 16 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Send(const uint8_t arg_0x40f86c88, 
const TMacFrame *const arg_0x40f86ea8, 
const uint8_t arg_0x40f85088, 
const TUniData arg_0x40f85240);

static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendFromPhyPool(const uint8_t arg_0x40f85708, 
const TPhyPoolHandle arg_0x40f858c8, 
const uint8_t arg_0x40f85a78);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacReceive$Receive(TMacFrame *const arg_0x40ef5318);
# 30 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static inline  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(uint8_t context, const TMacFrame *const pMacFrame, 
const TUniData uniData);





static inline  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendFromPhyPool(uint8_t context, const TPhyPoolHandle frameHandle);




static inline  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendDone(const uint8_t _sfIndex, uint8_t context, 
TMacStatus macStatus, const TUniData uniData);





static inline  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Receive(const uint8_t _sfIndex, TMacFrame *const pMacFrame);






static inline   void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$default$SendDone(uint8_t context, TMacStatus macStatus, TUniData unidata);
# 9 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  void MacPoolM$IMacPool$FreeDone(const TMacPoolHandle arg_0x40ee5030, const TMacStatus arg_0x40ee51d8);
# 28 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacPoolM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40da3218, TMacAddress *const arg_0x40da3408);
# 18 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  bool MacPoolM$IMacAddress$Equal(const TMacAddress *const arg_0x409b92b0, const TMacAddress *const arg_0x409b94c0);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacPoolM$IExpiredTimer$Stop(void);
#line 7
static  result_t MacPoolM$IExpiredTimer$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868);



static  bool MacPoolM$IExpiredTimer$IsSet(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime MacPoolM$ILocalTime$Read(void);
# 25 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
enum MacPoolM$__nesc_unnamed4405 {
#line 25
  MacPoolM$MAC_POOL_SIZE = 3
};
typedef uint16_t MacPoolM$TMacPoolIndex;





#line 29
typedef enum MacPoolM$__nesc_unnamed4406 {
  MacPoolM$MAC_POOL_ITEM_FREE, 
  MacPoolM$MAC_POOL_ITEM_FREE_DONE, 
  MacPoolM$MAC_POOL_ITEM_ALLOC
} MacPoolM$TMacPoolItemState;







#line 35
struct MacPoolM$__nesc_unnamed4407 {
  MacPoolM$TMacPoolItemState state;
  TUniData uniData;
  TMacFrame macFrame;
  TSysTime expiredTime;
  TMacStatus status;
} MacPoolM$pool[MacPoolM$MAC_POOL_SIZE];

TMacTransactionPersistenceTime MacPoolM$macTransactionPersistenceTime;
#line 57
static inline  TMacStatus MacPoolM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void);






static inline MacPoolM$TMacPoolIndex MacPoolM$ToPoolIndex(const TMacPoolHandle handle);




static inline TMacPoolHandle MacPoolM$ToPoolHandle(MacPoolM$TMacPoolIndex poolIndex);




static inline bool MacPoolM$IsValidHandle(const TMacPoolHandle macPoolHandle);




static inline void MacPoolM$SearchExpiredItem(void);
#line 115
static inline  result_t MacPoolM$IExpiredTimer$Fired(TUniData uniData);
#line 144
static inline  void MacPoolM$FreeDone(void);










static  result_t MacPoolM$IMacPool$Free(const TMacPoolHandle macPoolHandle, const TMacStatus status);
#line 168
static inline  TMacFrame *MacPoolM$IMacPool$GetFrame(const TMacPoolHandle macPoolHandle);
#line 192
static inline  bool MacPoolM$SearchDstAddress(const TMacAddress macAddress, 
TMacPoolHandle *const pMacPoolHandle);
#line 257
static void MacPoolM$ResetPool(void);










static inline  result_t MacPoolM$IStdControl$init(void);





static inline  result_t MacPoolM$IStdControl$start(void);
#line 286
static inline  void MacPoolM$Reset$reset(void);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacPendingM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200);
#line 42
static  result_t MacPendingM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40dd1530, uint8_t *const arg_0x40dd1718);
#line 39
static  result_t MacPendingM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40dd2df0, 
TMacPayloadLength *const arg_0x40dd1010);
#line 15
static  result_t MacPendingM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40da99a0, bool *const arg_0x40da9b88);
#line 34
static  result_t MacPendingM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40da2dd0, TMacAddress *const arg_0x40dd2010);
# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  TMacFrame *MacPendingM$IMacPool$GetFrame(const TMacPoolHandle arg_0x40ee5698);
#line 8
static  result_t MacPendingM$IMacPool$Free(const TMacPoolHandle arg_0x40ee69b8, const TMacStatus arg_0x40ee6b60);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacPendingM$IMacCSMA$Send(const TMacFrame *const arg_0x40ef9010, 
const TUniData arg_0x40ef91c8);
# 14 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPendingM.nc"
static  bool MacPendingM$SearchDstAddress(const TMacAddress arg_0x40fec5f0, 
TMacPoolHandle *const arg_0x40fec7f0);



static inline  result_t MacPendingM$IMacReceive$Receive(TMacFrame *const pMacFrame);
#line 57
static inline  void MacPendingM$IMacCSMA$SendDone(TMacStatus macStatus, const TUniData uniData);










static inline  void MacPendingM$IMacPool$FreeDone(const TMacPoolHandle macPoolHandle, 
const TMacStatus status);
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
static   uint16_t MacBeaconAttrCoordM$IMacRandom$Rand(void);
# 12 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
TMacBeaconTxTime MacBeaconAttrCoordM$beacon_tx_time;
#line 26
static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconTxTime(void);









uint8_t MacBeaconAttrCoordM$beaconPayload[MAC_AMAX_BEACON_PAYLOAD_LENGTH];
uint8_t MacBeaconAttrCoordM$beaconPayloadLength;

static inline  uint8_t MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayloadLength(
TMacStatus *const pMacStatus);





static inline  uint8_t *MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayload(
uint8_t *const placeForBeaconPayload, 
TMacStatus *const pMacStatus);
#line 63
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBeaconPayload(
uint8_t *payload, 
uint8_t length);
#line 80
static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconPayload(void);






bool MacBeaconAttrCoordM$macAutoRequest = TRUE;
#line 101
static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacAutoRequest(void);







TMacBSN MacBeaconAttrCoordM$macBSN;

static inline  TMacBSN MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBSN(TMacStatus *const pMacStatus);





static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBSN(TMacBSN _macBSN);




static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBSN(void);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacAssocCoordM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409bc8b0, const TMacAddress arg_0x409bca60);



static  result_t MacAssocCoordM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x409ba6d0, const TMacExtendedAddress arg_0x409ba890);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacAssocCoordM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200);
#line 27
static  result_t MacAssocCoordM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40da5b38, const TMacAddress arg_0x40da5ce8);
#line 17
static  result_t MacAssocCoordM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40da80a8, const bool arg_0x40da8250);
#line 42
static  result_t MacAssocCoordM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40dd1530, uint8_t *const arg_0x40dd1718);
#line 30
static  result_t MacAssocCoordM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40da3908, const TMacPANId arg_0x40da3ab8);








static  result_t MacAssocCoordM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40dd2df0, 
TMacPayloadLength *const arg_0x40dd1010);
#line 36
static  result_t MacAssocCoordM$IMacFrame$SetPayload(TMacFrame *const arg_0x40dd2510, 
const uint8_t *const arg_0x40dd2728, TMacPayloadLength arg_0x40dd28c0);
#line 8
static  result_t MacAssocCoordM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40dac708, const bool arg_0x40dac8b0);


static  result_t MacAssocCoordM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40dab510, const bool arg_0x40dab6b8);
#line 33
static  result_t MacAssocCoordM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40da2700, const TMacAddress arg_0x40da28b0);
#line 24
static  result_t MacAssocCoordM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40da6c60, const TMacPANId arg_0x40da6e10);
#line 9
static  result_t MacAssocCoordM$IMacFrame$GetSecurityEnabled(const TMacFrame *const arg_0x40dacdd8, bool *const arg_0x40dab010);




static  result_t MacAssocCoordM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40da92d8, const bool arg_0x40da9480);
#line 5
static  result_t MacAssocCoordM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40dad8e8, const TMacFrameType arg_0x40dada98);
#line 34
static  result_t MacAssocCoordM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40da2dd0, TMacAddress *const arg_0x40dd2010);
#line 20
static  result_t MacAssocCoordM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40da8e60, const TMacSequenceNumber arg_0x40da6030);
# 38 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t MacAssocCoordM$IMacASSOCIATE$Indication(TMacExtendedAddress arg_0x408ef610, 
TCapabilityInformation arg_0x408ef7c0, 
bool arg_0x408ef958, 
TACLEntry arg_0x408efaf8);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacPANId MacAssocCoordM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dcacf0);
#line 18
static  TMacDSN MacAssocCoordM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dc63a0);
static  TMacStatus MacAssocCoordM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dc6828);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCOMM_STATUS.nc"
static  void MacAssocCoordM$IMacCOMM_STATUS$Indication(TMacPANId arg_0x409059c0, 
TMacAddress arg_0x40905b60, TMacAddress arg_0x40905cf0, 
TMacStatus arg_0x40905e90);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacAssocCoordM$IMacSendOwn$Send(const TMacFrame *const arg_0x40ef9010, 
const TUniData arg_0x40ef91c8);
# 31 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
enum MacAssocCoordM$__nesc_unnamed4408 {
  MacAssocCoordM$ASSOC_REQUEST_PAYLOAD_LEN = 2, 
  MacAssocCoordM$ASSOC_RESPONSE_PAYLOAD_LEN = 4, 
  MacAssocCoordM$DISASSOC_NOTIF_PAYLOAD_LEN = 2
};

static inline result_t MacAssocCoordM$MakeAssocResponse(const TMacExtendedAddress deviceAddress, 
const TMacShortAddress assocShortAddress, 
const TMacStatus status, 
const bool securityEnable, 
TMacFrame *const pMacFrame);
#line 81
static inline void MacAssocCoordM$FailureResponse(TMacStatus status, TMacExtendedAddress deviceAddress);










TMacExtendedAddress MacAssocCoordM$dstExtAddress;

static  result_t MacAssocCoordM$IMacASSOCIATE$Response(TMacExtendedAddress deviceAddress, 
TMacShortAddress assocShortAddress, 
TMacStatus status, 
bool securityEnable);
#line 119
static inline  void MacAssocCoordM$IMacSendOwn$SendDone(TMacStatus status, const TUniData unidata);
#line 131
bool MacAssocCoordM$macAssociationPermit = FALSE;

static inline  bool MacAssocCoordM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const pMacStatus);





static inline  TMacStatus MacAssocCoordM$IMacAssocAttr$SetmacAssociationPermit(bool associationPermit);




static inline  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacAssociationPermit(void);





static inline  result_t MacAssocCoordM$IAssocRequest$Receive(TMacFrame *const pMacFrame);
#line 199
#line 194
typedef enum MacAssocCoordM$__nesc_unnamed4409 {
  MacAssocCoordM$MAC_DISASSOCIATE_OFF, 
  MacAssocCoordM$MAC_DISASSOCIATE_IDLE, 
  MacAssocCoordM$MAC_DISASSOCIATE_REQUEST, 
  MacAssocCoordM$MAC_DISASSOCIATE_WAIT_ACK
} MacAssocCoordM$TMacDisassocState;




#line 201
struct MacAssocCoordM$__nesc_unnamed4410 {
  MacAssocCoordM$TMacDisassocState state;
  TMacSequenceNumber seqNum;
} MacAssocCoordM$disassoc;
#line 382
static inline  result_t MacAssocCoordM$IStdControl$init(void);







static inline  result_t MacAssocCoordM$IStdControl$start(void);










static inline  void MacAssocCoordM$Reset$reset(void);
#line 419
static inline  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void);
#line 436
static inline  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void);
# 8 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconCommonM.nc"
uint8_t MacBeaconCommonM$lastParentBSN = 0;






static inline  uint8_t MacBeaconCommonM$getLastBSN(void);
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacStatus MacStartM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dc9198);


static  TMacShortAddress MacStartM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dc99a8);
# 11 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacStatus MacStartM$IMacSuperframeAttr$SetmacSuperframeOrder(TMacSuperframeOrder arg_0x40e80f10);
#line 7
static  TMacStatus MacStartM$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e80248);
# 11 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
static  result_t MacStartM$IPhyAttr$SetAutoAck(bool arg_0x40b38dd0);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
static  TPhyStatus MacStartM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b4a180);
# 20 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacStartM.nc"
static  result_t MacStartM$runOwnSF(TSysTime arg_0x41056ce8);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacStartM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960);
# 22 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
static  result_t MacStartM$IMacSTART$Confirm(TMacStatus arg_0x40928820);
# 29 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacStartM.nc"
static inline  result_t MacStartM$IMacSTART$Request(
TMacPANId panId, 
TMacLogicalChannel logicalChannel, 
TMacBeaconOrder beaconOrder, 
TMacSuperframeOrder superframeOrder, 
bool panCoordinator, 
bool batteryLifeExtension, 
bool coordRealignment, 
bool securityEnable, 
TSysTime startTime);
#line 97
static inline  result_t MacStartM$IPhySET_TRX_STATE$Confirm(TPhyStatus status, TUniData unidata);
# 18 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacDSN MacDataM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dc63a0);
#line 14
static  TMacAckWaitDuration MacDataM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dc86c8);




static  TMacStatus MacDataM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dc6828);
# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t MacDataM$IMacDataOwn$Confirm(uint8_t arg_0x409e1a00, TMacStatus arg_0x409e1b90);
#line 16
static  result_t MacDataM$IMacDataOwn$Indication(TMacPANId arg_0x409e6810, 
TMacAddress arg_0x409e69a8, 
TMacPANId arg_0x409e6b48, 
TMacAddress arg_0x409e6ce0, 
uint8_t arg_0x409e6e78, 
uint8_t *arg_0x409e1068, 
TMacLinkQuality arg_0x409e1208, 
bool arg_0x409e13a0, 
uint8_t arg_0x409e1538);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacDataM$AckWaitTimer$Stop(void);
#line 7
static  result_t MacDataM$AckWaitTimer$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacDataM$IMacSendParent$Send(const TMacFrame *const arg_0x40ef9010, 
const TUniData arg_0x40ef91c8);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacDataM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200);
#line 27
static  result_t MacDataM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40da5b38, const TMacAddress arg_0x40da5ce8);
#line 17
static  result_t MacDataM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40da80a8, const bool arg_0x40da8250);
#line 42
static  result_t MacDataM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40dd1530, uint8_t *const arg_0x40dd1718);
#line 30
static  result_t MacDataM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40da3908, const TMacPANId arg_0x40da3ab8);








static  result_t MacDataM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40dd2df0, 
TMacPayloadLength *const arg_0x40dd1010);
#line 36
static  result_t MacDataM$IMacFrame$SetPayload(TMacFrame *const arg_0x40dd2510, 
const uint8_t *const arg_0x40dd2728, TMacPayloadLength arg_0x40dd28c0);
#line 15
static  result_t MacDataM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40da99a0, bool *const arg_0x40da9b88);
#line 8
static  result_t MacDataM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40dac708, const bool arg_0x40dac8b0);
#line 31
static  result_t MacDataM$IMacFrame$GetSrcPANId(const TMacFrame *const arg_0x40da2010, TMacPANId *const arg_0x40da2200);
#line 11
static  result_t MacDataM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40dab510, const bool arg_0x40dab6b8);









static  result_t MacDataM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40da6558, 
TMacSequenceNumber *const arg_0x40da6760);










static  result_t MacDataM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40da2700, const TMacAddress arg_0x40da28b0);
#line 24
static  result_t MacDataM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40da6c60, const TMacPANId arg_0x40da6e10);



static  result_t MacDataM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40da3218, TMacAddress *const arg_0x40da3408);
#line 14
static  result_t MacDataM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40da92d8, const bool arg_0x40da9480);
#line 5
static  result_t MacDataM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40dad8e8, const TMacFrameType arg_0x40dada98);
#line 50
static  result_t MacDataM$IMacFrame$GetLinkQuality(const TMacFrame *const arg_0x40dce970, TMacLinkQuality *const arg_0x40dceb60);
#line 34
static  result_t MacDataM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40da2dd0, TMacAddress *const arg_0x40dd2010);
#line 20
static  result_t MacDataM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40da8e60, const TMacSequenceNumber arg_0x40da6030);




static  result_t MacDataM$IMacFrame$GetDstPANId(const TMacFrame *const arg_0x40da5448, TMacPANId *const arg_0x40da5638);
# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t MacDataM$IMacDataParent$Confirm(uint8_t arg_0x409e1a00, TMacStatus arg_0x409e1b90);
#line 16
static  result_t MacDataM$IMacDataParent$Indication(TMacPANId arg_0x409e6810, 
TMacAddress arg_0x409e69a8, 
TMacPANId arg_0x409e6b48, 
TMacAddress arg_0x409e6ce0, 
uint8_t arg_0x409e6e78, 
uint8_t *arg_0x409e1068, 
TMacLinkQuality arg_0x409e1208, 
bool arg_0x409e13a0, 
uint8_t arg_0x409e1538);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacDataM$IMacSendOwn$Send(const TMacFrame *const arg_0x40ef9010, 
const TUniData arg_0x40ef91c8);
# 34 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
bool MacDataM$st_waitAck = FALSE;

enum MacDataM$__nesc_unnamed4411 {
#line 36
  MacDataM$IDLE, MacDataM$WAIT_SEND_DONE, MacDataM$WAIT_ACK, MacDataM$OFF
} 
#line 36
MacDataM$state = MacDataM$IDLE;

typedef enum MacDataM$__nesc_unnamed4412 {
#line 38
  MacDataM$OWN_SF, MacDataM$PARENT_SF
} 
#line 38
MacDataM$SF;

MacDataM$SF MacDataM$currentSf;



TMacDSN MacDataM$st_ackDSN;
uint8_t MacDataM$st_msduHandle;
uint8_t MacDataM$st_TimesRetrLeft;

TMacFrame MacDataM$st_msg;



static result_t MacDataM$Confirm(uint8_t sf, uint8_t msduHandle, TMacStatus status);
static result_t MacDataM$Send(const TMacFrame *const frame, const TUniData unidata);



static inline result_t MacDataM$DATA_Request(MacDataM$SF sf, 
TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
const uint8_t *const pmsduFrame, 
uint8_t msduHandle, 
TxOptions bmTxOpt);
#line 120
static inline void MacDataM$SendDone(TMacStatus status, const TUniData unidata);
#line 148
static inline  result_t MacDataM$AckWaitTimer$Fired(TUniData uniData);
#line 180
static inline result_t MacDataM$Receive(MacDataM$SF sf, TMacFrame *const pmacFrame);
#line 267
static result_t MacDataM$Confirm(uint8_t sf, uint8_t msduHandle, TMacStatus status);








static result_t MacDataM$Send(const TMacFrame *const frame, const TUniData unidata);
#line 308
static inline  result_t MacDataM$IMacDataOwn$Request(
TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
const uint8_t *const pmsduFrame, 
uint8_t msduHandle, 
TxOptions bmTxOpt);
#line 337
static inline  void MacDataM$IMacSendOwn$SendDone(TMacStatus status, const TUniData unidata);
#line 349
static inline  result_t MacDataM$IMacReceiveOwn$Receive(TMacFrame *const pmacFrame);










static inline  result_t MacDataM$StdControl$start(void);










static inline  result_t MacDataM$StdControl$stop(void);






static inline  void MacDataM$Reset$reset(void);









static inline   result_t MacDataM$IMacDataParent$default$Indication(TMacPANId SrcPANid, TMacAddress SrcAddr, 
TMacPANId DstPANid, TMacAddress DstAddr, uint8_t msduLength, uint8_t *msduFrame, 
TMacLinkQuality linkQuality, bool bSecurityUse, uint8_t ACLEntry);


static inline   result_t MacDataM$IMacDataParent$default$Confirm(uint8_t msdu_L_Handle, TMacStatus MacStatus);
#line 405
static inline   result_t MacDataM$IMacSendParent$default$Send(const TMacFrame *const frame, const TUniData unidata);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacPANId MacScanBeaconM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dcacf0);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacScanBeaconM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200);
#line 42
static  result_t MacScanBeaconM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40dd1530, uint8_t *const arg_0x40dd1718);
#line 39
static  result_t MacScanBeaconM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40dd2df0, 
TMacPayloadLength *const arg_0x40dd1010);
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacBeaconOrder MacScanBeaconM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t MacScanBeaconM$IPhyPool$Alloc(const uint8_t arg_0x40b5e6b8, const uint16_t arg_0x40b5e860, 
TPhyPoolHandle *const arg_0x40b5ea58);

static   TPhyFrame *MacScanBeaconM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010);



static   result_t MacScanBeaconM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b5cbc0);
# 17 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanBeaconM.nc"
static  void MacScanBeaconM$packBeacon(TPhyFrame *arg_0x410b7698, uint64_t *arg_0x410b7840);
# 10 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacScanBeaconM$CSMA$SendFromPhyPool(const TPhyPoolHandle arg_0x40ef9680);
# 26 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanBeaconM.nc"
static inline  void MacScanBeaconM$sendBcn(void);
TPhyPoolHandle MacScanBeaconM$beaconHandle;
bool MacScanBeaconM$beaconAllocated = FALSE;

static inline  result_t MacScanBeaconM$IMacReceive$Receive(TMacFrame *const pmacFrame);
#line 58
static inline  void MacScanBeaconM$sendBcn(void);
#line 73
static inline  void MacScanBeaconM$CSMA$SendDone(TMacStatus status, const TUniData unidata);







static inline  void MacScanBeaconM$Reset$reset(void);
# 16 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetmacBeaconPayload(
uint8_t *arg_0x40ec5ab0, 
uint8_t arg_0x40ec5c50);
static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetDefaultmacBeaconPayload(void);









static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetDefaultmacBeaconTxTime(void);
#line 23
static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetDefaultmacBSN(void);
#line 9
static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetDefaultmacAutoRequest(void);
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void);
#line 8
static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacPANId(void);
#line 7
static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dc9198);


static  TMacShortAddress MacCoordAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dc99a8);

static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void);
#line 24
static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void);



static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void);


static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(bool arg_0x40dc4f00);
#line 6
static  TMacPANId MacCoordAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dcacf0);




static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetmacShortAddress(TMacShortAddress arg_0x40dc9e50);








static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacDSN(void);
#line 32
static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void);
# 12 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacStatus MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void);
#line 8
static  TMacStatus MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void);
#line 6
static  TMacBeaconOrder MacCoordAttrM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90);
# 10 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPoolAttr.nc"
static  TMacStatus MacCoordAttrM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void);
# 10 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void);







static  bool MacCoordAttrM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40eec9e8);
static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetmacAssociationPermit(bool arg_0x40eece88);
static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacAssociationPermit(void);
#line 16
static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacCoordAttrM$IMacReset$reset(void);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetmacBattLifeExt(bool arg_0x40ebce20);
#line 20
static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs(void);
#line 8
static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacBattLifeExt(void);





static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods(void);
#line 26
static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacMinBE(void);
# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t MacCoordAttrM$IMacRESET$Confirm(TMacStatus arg_0x409c24b8);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacCoordAttrM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960);
# 2 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacPANId MacCoordAttrM$IMacGET$GetmacPANId(TMacStatus *const pMacStatus);



static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacPANId(TMacPANId macPANId);





static inline  TMacShortAddress MacCoordAttrM$IMacGET$GetmacShortAddress(TMacStatus *const pMacStatus);



static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacShortAddress(TMacShortAddress macShortAddress);
#line 80
static inline  TMacBeaconOrder MacCoordAttrM$IMacGET$GetmacBeaconOrder(
TMacStatus *const pMacStatus);
#line 108
static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacBattLifeExt(bool macBattLifeExt);
#line 155
static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacRxOnWhenIdle(
bool macRxOnWhenIdle);
#line 187
static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacBeaconPayload(
TMacBeaconPayload macBeaconPayload, 
TMacBeaconPayloadLength macBeaconPayloadLength);
#line 249
static inline  bool MacCoordAttrM$IMacGET$GetmacAssociationPermit(
TMacStatus *const pMacStatus);



static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacAssociationPermit(
bool macAssociationPermit);




static __inline void MacCoordAttrM$SetDefaultAttributes(void);
#line 291
static inline  void MacCoordAttrM$confirmReset(void);




static inline  result_t MacCoordAttrM$IMacRESET$Request(bool setDefaultPIB);









static inline  result_t MacCoordAttrM$IPhySET_TRX_STATE$Confirm(TPhyStatus phyStatus, TUniData uniData);
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   TSysTime ChildSupervisorM$ITimeCast$MillisToSymbols(TMilliSec arg_0x40a378b8);
# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
static  void ChildSupervisorM$NLME_Leave$indication(IEEEAddr arg_0x408d6548);
# 17 "../../../zigzag/ZigBee/extra/ChildSupervisorM.nc"
static  void ChildSupervisorM$updateBeacon(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime ChildSupervisorM$ILocalTime$Read(void);
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t ChildSupervisorM$ITimerSymbol$SetPeriodic(TSysTime arg_0x4090c0c0, TUniData arg_0x4090c248);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t ChildSupervisorM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x408dea50);










static  result_t ChildSupervisorM$NeighborTable$remove(NwkNeighbor *arg_0x408dd580);
# 24 "../../../zigzag/ZigBee/extra/ChildSupervisorM.nc"
TSysTime ChildSupervisorM$table_check_period;

static inline result_t  ChildSupervisorM$StdControl$start(void);
#line 43
static inline  result_t ChildSupervisorM$ITimerSymbol$Fired(TUniData u);
# 5 "../../../zigzag/PowerManagerM.nc"
static inline  result_t PowerManagerM$StdControl$init(void);
static inline  result_t PowerManagerM$StdControl$start(void);
# 23 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
static  result_t ZigNet2SpiM$IZigDATA$request(
NwkAddr arg_0x408e5938, 
uint8_t arg_0x408e5ad0, 
uint8_t *arg_0x408e5c80, 
uint8_t arg_0x408e5e18, 
uint8_t arg_0x408e4010, 
uint8_t arg_0x408e41a8, 
bool arg_0x408e4340);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress ZigNet2SpiM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408f9440);
# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void ZigNet2SpiM$INTR_2$clear(void);
#line 35
static   void ZigNet2SpiM$INTR_2$disable(void);
#line 54
static   void ZigNet2SpiM$INTR_2$edge(bool arg_0x40cd4068);
#line 30
static   void ZigNet2SpiM$INTR_2$enable(void);
# 70 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
static   void ZigNet2SpiM$USARTControl$setClockRate(uint16_t arg_0x41157ca8, uint8_t arg_0x41157e28);
#line 68
static   void ZigNet2SpiM$USARTControl$setClockSource(uint8_t arg_0x41157800);
#line 92
static   result_t ZigNet2SpiM$USARTControl$isTxEmpty(void);
#line 73
static   result_t ZigNet2SpiM$USARTControl$disableRxIntr(void);
static   result_t ZigNet2SpiM$USARTControl$disableTxIntr(void);
#line 65
static   void ZigNet2SpiM$USARTControl$setModeSPI(bool arg_0x41157348);
#line 81
static   result_t ZigNet2SpiM$USARTControl$isTxIntrPending(void);
#line 103
static   result_t ZigNet2SpiM$USARTControl$tx(uint8_t arg_0x41154ab0);






static   uint8_t ZigNet2SpiM$USARTControl$rx(void);
#line 86
static   result_t ZigNet2SpiM$USARTControl$isRxIntrPending(void);
# 8 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  uint64_t ZigNet2SpiM$ILocalTime64$getLocalTime(void);
static  void ZigNet2SpiM$ILocalTime64$setLocalTime(uint64_t arg_0x40a39da0);
# 29 "../../../zigzag/ZigNet2SpiM.nc"
bool ZigNet2SpiM$sending = FALSE;

uint8_t ZigNet2SpiM$usartBufSize = 0;
NwkAddr ZigNet2SpiM$usartDstAddr = 0;
uint8_t ZigNet2SpiM$usartBuf[MAC_AMAX_MAC_FRAME_SIZE - NWK_MIN_HEADER_OVERHEAD];

size_t serialize(void *dst, size_t max_size, const char *fmt, ...)   ;
size_t deserialize(const void *src, size_t size, const char *fmt, ...)   ;

static void ZigNet2SpiM$usartTx(NwkAddr srcAddr, IEEEAddr srcExtAddr, uint8_t nsduLength, uint8_t *nsdu);

static inline  void ZigNet2SpiM$taskSendEvents(void);
#line 66
static void ZigNet2SpiM$sendEvents(void);










static  void ZigNet2SpiM$IZigDATA$confirm(uint8_t nsduHandler, NwkStatus status);
#line 95
static IEEEAddr *ZigNet2SpiM$workaround;

static inline  void ZigNet2SpiM$IZigParentJoin$indication(NwkAddr shortAddr, IEEEAddr extendedAddr, 
NwkCapabilityInfo capabilityInformation, bool secureJoin);
#line 112
static inline  void ZigNet2SpiM$IZigLeave$indication(IEEEAddr deviceAddr);
#line 127
static void ZigNet2SpiM$usartTx(NwkAddr srcAddr, IEEEAddr extAddr, uint8_t nsduLength, uint8_t *nsdu);
#line 188
static inline  void ZigNet2SpiM$IZigNetFormation$confirm(NwkStatus status);







static inline  void ZigNet2SpiM$IZigReset$confirm(NwkStatus status);








static inline  void ZigNet2SpiM$IZigDATA$indication(NwkAddr srcAddr, IEEEAddr srcExtAddr, uint8_t nsduLength, 
uint8_t *nsdu, uint8_t linkQuality);






static inline  void ZigNet2SpiM$taskUsartRx(void);
#line 268
static inline   void ZigNet2SpiM$INTR_2$fired(void);








static inline  result_t ZigNet2SpiM$StdControl$init(void);






static inline  result_t ZigNet2SpiM$StdControl$start(void);
# 45 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
 static volatile uint8_t UniSART1M$ME2 __asm ("0x0005");
 static volatile uint8_t UniSART1M$IFG2 __asm ("0x0003");
 static volatile uint8_t UniSART1M$U1TCTL __asm ("0x0079");
 static volatile uint8_t UniSART1M$U1TXBUF __asm ("0x007F");

uint16_t UniSART1M$l_br;
uint8_t UniSART1M$l_mctl;
uint8_t UniSART1M$l_ssel;
#line 77
static inline   void UniSART1M$USARTControl$setModeSPI(bool master);
#line 127
static inline   void UniSART1M$USARTControl$setClockSource(uint8_t source);







static inline   void UniSART1M$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl);









static   result_t UniSART1M$USARTControl$isTxIntrPending(void);







static inline   result_t UniSART1M$USARTControl$isTxEmpty(void);






static   result_t UniSART1M$USARTControl$isRxIntrPending(void);







static inline   result_t UniSART1M$USARTControl$disableRxIntr(void);




static inline   result_t UniSART1M$USARTControl$disableTxIntr(void);
#line 194
static inline   result_t UniSART1M$USARTControl$tx(uint8_t data);






static   uint8_t UniSART1M$USARTControl$rx(void);
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
  union __nesc_unnamed4413 {
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
  union __nesc_unnamed4414 {
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
inline static   void TimerTuningM$ITimerB1Control$setControl(MSP430CompareControl_t arg_0x40741010){
#line 34
  MSP430TimerM$ControlB1$setControl(arg_0x40741010);
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
inline static   void TimerTuningM$ITimerB$setMode(int arg_0x40735010){
#line 35
  MSP430TimerM$TimerB$setMode(arg_0x40735010);
#line 35
}
#line 35
# 200 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerB$setInputDivider(uint16_t inputDivider)
{
  TBCTL = (TBCTL & ~((1 << 6) | (3 << 6))) | ((inputDivider << 8) & ((1 << 6) | (3 << 6)));
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerB$setInputDivider(uint16_t arg_0x40733358){
#line 40
  MSP430TimerM$TimerB$setInputDivider(arg_0x40733358);
#line 40
}
#line 40
# 190 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerB$setClockSource(uint16_t clockSource)
{
  TBCTL = (TBCTL & ~(0x0100 | 0x0200)) | ((clockSource << 8) & (0x0100 | 0x0200));
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerB$setClockSource(uint16_t arg_0x40735dd8){
#line 39
  MSP430TimerM$TimerB$setClockSource(arg_0x40735dd8);
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
inline static   void TimerTuningM$ITimerA$setClockSource(uint16_t arg_0x40735dd8){
#line 39
  MSP430TimerM$TimerA$setClockSource(arg_0x40735dd8);
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
inline static   void TimerTuningM$ITimerA$setMode(int arg_0x40735010){
#line 35
  MSP430TimerM$TimerA$setMode(arg_0x40735010);
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

  uint8_t i;

#line 25
  BCSCTL1 = (0x04 | 0x02) | 0x01;
  DCOCTL = 0;
  do {
      IFG1 &= ~(1 << 1);
      i = 0xff;
      while (--i) ;
    }
  while (
#line 31
  IFG1 & (1 << 1));
  BCSCTL2 = 0x80 | 0x08;
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
# 89 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_SET_PIN_DIRECTIONS(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 91
    {
      P1IE = 0x00;
#line 92
      P1OUT = 0x00;
#line 92
      P1SEL = 0x00;
#line 92
      P1DIR = 0x00;
#line 92
      P1IFG = 0x00;
      P2IE = 0x00;
#line 93
      P2OUT = 0x00;
#line 93
      P2SEL = 0x01;
#line 93
      P2DIR = 0x01;
#line 93
      P2IFG = 0x00;
      P3OUT = 0x00;
#line 94
      P3SEL = 0x0e;
#line 94
      P3DIR = 0x0a;
      P4OUT = 0xc4;
#line 95
      P4SEL = 0x02;
#line 95
      P4DIR = 0xe4;
      P5OUT = 0x01;
#line 96
      P5SEL = 0x0e;
#line 96
      P5DIR = 0xdb;
      P6OUT = 0x00;
#line 97
      P6SEL = 0x00;
#line 97
      P6DIR = 0x07;
    }
#line 98
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

# 277 "../../../zigzag/ZigNet2SpiM.nc"
static inline  result_t ZigNet2SpiM$StdControl$init(void)
{
  ZigNet2SpiM$sending = FALSE;

  return SUCCESS;
}

# 436 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static inline  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void)
{
  return MAC_UNSUPPORTED_ATTRIBUTE;
}

#line 419
static inline  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void)
{
  return MAC_UNSUPPORTED_ATTRIBUTE;
}

#line 144
static inline  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacAssociationPermit(void)
{
  MacAssocCoordM$macAssociationPermit = FALSE;
  return MAC_SUCCESS;
}

#line 382
static inline  result_t MacAssocCoordM$IStdControl$init(void)
{
  MacAssocCoordM$disassoc.state = MacAssocCoordM$MAC_DISASSOCIATE_OFF;
  MacAssocCoordM$IMacAssocAttr$SetDefaultmacAssociationPermit();
  MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress();
  MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordShortAddress();
  return SUCCESS;
}

# 57 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static inline  TMacStatus MacPoolM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void)
{

  MacPoolM$macTransactionPersistenceTime = 6;
  return MAC_SUCCESS;
}

#line 268
static inline  result_t MacPoolM$IStdControl$init(void)
{
  MacPoolM$ResetPool();
  MacPoolM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime();
  return SUCCESS;
}

# 118 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  result_t MacCAPM$ICAPControl$init(uint8_t sfIndex)
{
  MacCAPM$currentSf = 0;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 121
    MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_OFF;
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 402 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  result_t MacSuperframesM$StdControl$init(void)
{
  return SUCCESS;
}

# 159 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void)
{
  MacCommonAttrM$pib.macRxOnWhenIdle = FALSE;
  return MAC_SUCCESS;
}

#line 141
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void)
{
  MacCommonAttrM$pib.macShortAddress = 0xffff;
  return MAC_SUCCESS;
}

#line 121
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPANId(void)
{
  MacCommonAttrM$pib.macPANId = 0xffff;
  return MAC_SUCCESS;
}

#line 100
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void)
{
  MacCommonAttrM$pib.macSecurityMode = MAC_UNSECURED_MODE;
  return MAC_SUCCESS;
}

#line 83
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void)
{
  return MAC_UNSUPPORTED_ATTRIBUTE;
}

#line 64
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void)
{
  MacCommonAttrM$pib.macAckWaitDuration = MAC_ACK_WAIT_DURATION_54;
  return MAC_SUCCESS;
}

# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
inline static   uint16_t MacCommonAttrM$IMacRandom$Rand(void){
#line 3
  unsigned int result;
#line 3

#line 3
  result = MacRandomM$IMacRandom$Rand();
#line 3

#line 3
  return result;
#line 3
}
#line 3
# 37 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacDSN(void)
{
  MacCommonAttrM$pib.macDSN = (TMacDSN )MacCommonAttrM$IMacRandom$Rand();
  return MAC_SUCCESS;
}

#line 165
static inline  result_t MacCommonAttrM$IStdControl$init(void)
{
  MacCommonAttrM$IMacCommonAttr$SetDefaultmacDSN();
  MacCommonAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration();
  MacCommonAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode();
  MacCommonAttrM$IMacCommonAttr$SetDefaultmacSecurityMode();
  MacCommonAttrM$IMacCommonAttr$SetDefaultmacPANId();
  MacCommonAttrM$IMacCommonAttr$SetDefaultmacShortAddress();
  MacCommonAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle();
  return SUCCESS;
}

# 13 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacRandomM.nc"
static inline  result_t MacRandomM$IStdControl$init(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 15
    {
      MacRandomM$randomValue = (uint64_t )info_param.MAC_ADDRESS & 0xFFFF;
      MacRandomM$randomValue ^= ((uint64_t )info_param.MAC_ADDRESS >> 8) & 0xFFFF;
      MacRandomM$randomValue ^= ((uint64_t )info_param.MAC_ADDRESS >> 16) & 0xFFFF;
      MacRandomM$randomValue ^= ((uint64_t )info_param.MAC_ADDRESS >> 24) & 0xFFFF;
    }
#line 20
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 452 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline   result_t HPLUSART0M$USARTControl$disableTxIntr(void)
#line 452
{
  HPLUSART0M$IE1 &= ~(1 << 7);
  return SUCCESS;
}

# 173 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$disableTxIntr(void){
#line 173
  unsigned char result;
#line 173

#line 173
  result = HPLUSART0M$USARTControl$disableTxIntr();
#line 173

#line 173
  return result;
#line 173
}
#line 173
# 447 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline   result_t HPLUSART0M$USARTControl$disableRxIntr(void)
#line 447
{
  HPLUSART0M$IE1 &= ~(1 << 6);
  return SUCCESS;
}

# 172 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$disableRxIntr(void){
#line 172
  unsigned char result;
#line 172

#line 172
  result = HPLUSART0M$USARTControl$disableRxIntr();
#line 172

#line 172
  return result;
#line 172
}
#line 172
#line 135
inline static   void HPLCC2420M$USARTControl$setModeSPI(void){
#line 135
  HPLUSART0M$USARTControl$setModeSPI();
#line 135
}
#line 135
# 65 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_MAKE_RADIO_CSN_OUTPUT(void)
#line 65
{
#line 65
   static volatile uint8_t r __asm ("0x001E");

#line 65
  r |= 1 << 2;
}

#line 65
static inline void TOSH_SET_RADIO_CSN_PIN(void)
#line 65
{
#line 65
   static volatile uint8_t r __asm ("0x001D");

#line 65
  r |= 1 << 2;
}

# 83 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static inline  result_t HPLCC2420M$StdControl$init(void)
#line 83
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 84
    {
      HPLCC2420M$f.busy = HPLCC2420M$f.enabled = HPLCC2420M$f.rxbufBusy = HPLCC2420M$f.txbufBusy = FALSE;
    }
#line 86
    __nesc_atomic_end(__nesc_atomic); }

  TOSH_SET_RADIO_CSN_PIN();
  TOSH_MAKE_RADIO_CSN_OUTPUT();
  HPLCC2420M$USARTControl$setModeSPI();
  HPLCC2420M$USARTControl$disableRxIntr();
  HPLCC2420M$USARTControl$disableTxIntr();
  return SUCCESS;
}

# 54 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
static inline  result_t BusArbitrationM$StdControl$init(void)
#line 54
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 55
    {
      BusArbitrationM$state = BusArbitrationM$BUS_OFF;
      BusArbitrationM$isBusReleasedPending = FALSE;
    }
#line 58
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t PhyCC2420M$HPLCC2420Control$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = BusArbitrationM$StdControl$init();
#line 63
  result = rcombine(result, HPLCC2420M$StdControl$init());
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 70 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_MAKE_RADIO_POWER_OUTPUT(void)
#line 70
{
#line 70
   static volatile uint8_t r __asm ("0x001E");

#line 70
  r |= 1 << 7;
}

#line 70
static inline void TOSH_SEL_RADIO_POWER_IOFUNC(void)
#line 70
{
#line 70
   static volatile uint8_t r __asm ("0x001F");

#line 70
  r &= ~(1 << 7);
}

# 687 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  result_t PhyCC2420M$IStdControl$init(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 689
    PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_OFF;
#line 689
    __nesc_atomic_end(__nesc_atomic); }
  PhyCC2420M$radio.pData = NULL;
  PhyCC2420M$radio.dataLength = 0;
  PhyCC2420M$radio.timeStamp = 0;
  PhyCC2420M$g_phyChannelsSupported = 0x07fff800UL;
  PhyCC2420M$g_channel = 11;
  PhyCC2420M$g_extendedAddress = 0;
  PhyCC2420M$g_shortAddress = 0xffff;
  PhyCC2420M$g_panId = 0xffff;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 698
    PhyCC2420M$g_autoAck = FALSE;
#line 698
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 699
    PhyCC2420M$trxState = PhyCC2420M$PHY_TRX_FREE;
#line 699
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 700
    PhyCC2420M$edState = PhyCC2420M$PHY_ED_FREE;
#line 700
    __nesc_atomic_end(__nesc_atomic); }






  TOSH_SEL_RADIO_POWER_IOFUNC();
  TOSH_MAKE_RADIO_POWER_OUTPUT();

  PhyCC2420M$HPLCC2420Control$init();
  return SUCCESS;
}

# 94 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static inline void PhyPoolM$ResetPool(void)
{
  PhyPoolM$TPoolIndex i;

#line 97
  for (i = 0; i < PhyPoolM$POOL_SIZE; i++) 
    PhyPoolM$IPhyPool$Free(i);
  return;
}







static inline  result_t PhyPoolM$IStdControl$init(void)
{
  PhyPoolM$ResetPool();
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

# 70 "../../../zigzag/ZigCoordM.nc"
static inline  result_t ZigCoordM$StdControl$init(void)
{
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
  result = rcombine(result, ZigCoordM$StdControl$init());
#line 63
  result = rcombine(result, TimerSymbol2M$IStdControl$init());
#line 63
  result = rcombine(result, TimerSymbolAsyncM$IStdControl$init());
#line 63
  result = rcombine(result, PhyPoolM$IStdControl$init());
#line 63
  result = rcombine(result, PhyCC2420M$IStdControl$init());
#line 63
  result = rcombine(result, MacRandomM$IStdControl$init());
#line 63
  result = rcombine(result, MacCommonAttrM$IStdControl$init());
#line 63
  result = rcombine(result, MacSuperframesM$StdControl$init());
#line 63
  result = rcombine(result, MacCAPM$ICAPControl$init(MAC_OWN_SF));
#line 63
  result = rcombine(result, MacPoolM$IStdControl$init());
#line 63
  result = rcombine(result, MacAssocCoordM$IStdControl$init());
#line 63
  result = rcombine(result, ZigNet2SpiM$StdControl$init());
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
# 83 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline   void PhyCC2420M$FreePoolItem(TPhyPoolHandle handle)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 85
    {
      if (handle == PhyCC2420M$radio.txFifoHandle) {
        PhyCC2420M$radio.txFifoHandle = 0xff;
        }
    }
#line 89
    __nesc_atomic_end(__nesc_atomic); }
#line 89
  return;
}

# 11 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
inline static   void PhyPoolM$FreePoolItem(TPhyPoolHandle arg_0x40c45330){
#line 11
  PhyCC2420M$FreePoolItem(arg_0x40c45330);
#line 11
}
#line 11
# 114 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline   bool HPLUSART0M$USARTControl$isI2C(void)
#line 114
{
  bool _ret = FALSE;






  return _ret;
}

#line 72
static inline   bool HPLUSART0M$USARTControl$isSPI(void)
#line 72
{
  bool _ret = FALSE;

#line 74
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 74
    {
      if (HPLUSART0M$ME1 & (1 << 6)) {
        _ret = TRUE;
        }
    }
#line 78
    __nesc_atomic_end(__nesc_atomic); }
#line 78
  return _ret;
}

# 57 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline bool TOSH_IS_URXD0_IOFUNC(void)
#line 57
{
#line 57
   static volatile uint8_t r __asm ("0x001B");

#line 57
  return r | ~(1 << 5);
}

#line 56
static inline bool TOSH_IS_UTXD0_MODFUNC(void)
#line 56
{
#line 56
   static volatile uint8_t r __asm ("0x001B");

#line 56
  return r & (1 << 4);
}

# 92 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline   bool HPLUSART0M$USARTControl$isUARTtx(void)
#line 92
{
  bool _ret = FALSE;

#line 94
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 94
    {

      if (
#line 95
      HPLUSART0M$ME1 & (1 << 7) && 
      TOSH_IS_UTXD0_MODFUNC() && 
      TOSH_IS_URXD0_IOFUNC()) {
        _ret = TRUE;
        }
    }
#line 100
    __nesc_atomic_end(__nesc_atomic); }
#line 100
  return _ret;
}

# 56 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline bool TOSH_IS_UTXD0_IOFUNC(void)
#line 56
{
#line 56
   static volatile uint8_t r __asm ("0x001B");

#line 56
  return r | ~(1 << 4);
}

#line 57
static inline bool TOSH_IS_URXD0_MODFUNC(void)
#line 57
{
#line 57
   static volatile uint8_t r __asm ("0x001B");

#line 57
  return r & (1 << 5);
}

# 103 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline   bool HPLUSART0M$USARTControl$isUARTrx(void)
#line 103
{
  bool _ret = FALSE;

#line 105
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 105
    {

      if (
#line 106
      HPLUSART0M$ME1 & (1 << 6) && 
      TOSH_IS_URXD0_MODFUNC() && 
      TOSH_IS_UTXD0_IOFUNC()) {
        _ret = TRUE;
        }
    }
#line 111
    __nesc_atomic_end(__nesc_atomic); }
#line 111
  return _ret;
}

#line 81
static inline   bool HPLUSART0M$USARTControl$isUART(void)
#line 81
{
  bool _ret = FALSE;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 83
    {

      if (
#line 84
      HPLUSART0M$ME1 & (1 << 7) && HPLUSART0M$ME1 & (1 << 6) && 
      TOSH_IS_URXD0_MODFUNC() && 
      TOSH_IS_UTXD0_MODFUNC()) {
        _ret = TRUE;
        }
    }
#line 89
    __nesc_atomic_end(__nesc_atomic); }
#line 89
  return _ret;
}

#line 125
static inline   msp430_usartmode_t HPLUSART0M$USARTControl$getMode(void)
#line 125
{
  if (HPLUSART0M$USARTControl$isUART()) {
    return USART_UART;
    }
  else {
#line 128
    if (HPLUSART0M$USARTControl$isUARTrx()) {
      return USART_UART_RX;
      }
    else {
#line 130
      if (HPLUSART0M$USARTControl$isUARTtx()) {
        return USART_UART_TX;
        }
      else {
#line 132
        if (HPLUSART0M$USARTControl$isSPI()) {
          return USART_SPI;
          }
        else {
#line 134
          if (HPLUSART0M$USARTControl$isI2C()) {
            return USART_I2C;
            }
          else {
#line 137
            return USART_NONE;
            }
          }
        }
      }
    }
}

# 57 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_SEL_URXD0_IOFUNC(void)
#line 57
{
#line 57
   static volatile uint8_t r __asm ("0x001B");

#line 57
  r &= ~(1 << 5);
}

#line 56
static inline void TOSH_SEL_UTXD0_IOFUNC(void)
#line 56
{
#line 56
   static volatile uint8_t r __asm ("0x001B");

#line 56
  r &= ~(1 << 4);
}

# 172 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline   void HPLUSART0M$USARTControl$disableUART(void)
#line 172
{
  HPLUSART0M$ME1 &= ~((1 << 7) | (1 << 6));
  TOSH_SEL_UTXD0_IOFUNC();
  TOSH_SEL_URXD0_IOFUNC();
}

#line 218
static inline   void HPLUSART0M$USARTControl$disableI2C(void)
#line 218
{
}

# 53 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_SEL_SIMO0_MODFUNC(void)
#line 53
{
#line 53
   static volatile uint8_t r __asm ("0x001B");

#line 53
  r |= 1 << 1;
}

#line 54
static inline void TOSH_SEL_SOMI0_MODFUNC(void)
#line 54
{
#line 54
   static volatile uint8_t r __asm ("0x001B");

#line 54
  r |= 1 << 2;
}

#line 55
static inline void TOSH_SEL_UCLK0_MODFUNC(void)
#line 55
{
#line 55
   static volatile uint8_t r __asm ("0x001B");

#line 55
  r |= 1 << 3;
}

# 16 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   TSysTime TimeCastM$ITimeCast$JiffiesToSymbols(uint32_t jiffies)
{
  return jiffies << 1;
}

# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TSysTime TimerSymbol2M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a37d70){
#line 7
  unsigned long result;
#line 7

#line 7
  result = TimeCastM$ITimeCast$JiffiesToSymbols(arg_0x40a37d70);
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
inline static   TSysTime MacRandomM$ILocalTime$Read(void){
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
# 166 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   uint16_t MSP430TimerM$TimerB$read(void)
#line 166
{
#line 166
  return TBR;
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
# 125 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port21$enable(void)
#line 125
{
#line 125
  MSP430InterruptM$P2IE |= 1 << 1;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigNet2SpiM$INTR_2$enable(void){
#line 30
  MSP430InterruptM$Port21$enable();
#line 30
}
#line 30
# 276 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port21$edge(bool l2h)
#line 276
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 277
    {
      if (l2h) {
#line 278
        P2IES &= ~(1 << 1);
        }
      else {
#line 279
        P2IES |= 1 << 1;
        }
    }
#line 281
    __nesc_atomic_end(__nesc_atomic); }
}

# 54 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigNet2SpiM$INTR_2$edge(bool arg_0x40cd4068){
#line 54
  MSP430InterruptM$Port21$edge(arg_0x40cd4068);
#line 54
}
#line 54
# 187 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port21$clear(void)
#line 187
{
#line 187
  MSP430InterruptM$P2IFG &= ~(1 << 1);
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigNet2SpiM$INTR_2$clear(void){
#line 40
  MSP430InterruptM$Port21$clear();
#line 40
}
#line 40
# 156 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port21$disable(void)
#line 156
{
#line 156
  MSP430InterruptM$P2IE &= ~(1 << 1);
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void ZigNet2SpiM$INTR_2$disable(void){
#line 35
  MSP430InterruptM$Port21$disable();
#line 35
}
#line 35
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
inline static   result_t ZigNet2SpiM$USARTControl$disableTxIntr(void){
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
inline static   result_t ZigNet2SpiM$USARTControl$disableRxIntr(void){
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
# 76 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_SEL_UCLK1_MODFUNC(void)
#line 76
{
#line 76
   static volatile uint8_t r __asm ("0x0033");

#line 76
  r |= 1 << 3;
}

#line 75
static inline void TOSH_SEL_SOMI1_MODFUNC(void)
#line 75
{
#line 75
   static volatile uint8_t r __asm ("0x0033");

#line 75
  r |= 1 << 2;
}

#line 74
static inline void TOSH_SEL_SIMO1_MODFUNC(void)
#line 74
{
#line 74
   static volatile uint8_t r __asm ("0x0033");

#line 74
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
inline static   void ZigNet2SpiM$USARTControl$setModeSPI(bool arg_0x41157348){
#line 65
  UniSART1M$USARTControl$setModeSPI(arg_0x41157348);
#line 65
}
#line 65
# 135 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static inline   void UniSART1M$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl)
#line 135
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 136
    {
      UniSART1M$l_br = baudrate;
      UniSART1M$l_mctl = mctl;
      U1BR0 = baudrate & 0x0FF;
      U1BR1 = (baudrate >> 8) & 0x0FF;
      U1MCTL = mctl;
    }
#line 142
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   void ZigNet2SpiM$USARTControl$setClockRate(uint16_t arg_0x41157ca8, uint8_t arg_0x41157e28){
#line 70
  UniSART1M$USARTControl$setClockRate(arg_0x41157ca8, arg_0x41157e28);
#line 70
}
#line 70
# 127 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static inline   void UniSART1M$USARTControl$setClockSource(uint8_t source)
#line 127
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 128
    {
      UniSART1M$l_ssel = source | 0x80;
      UniSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
      UniSART1M$U1TCTL |= UniSART1M$l_ssel & 0x7F;
    }
#line 132
    __nesc_atomic_end(__nesc_atomic); }
}

# 68 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   void ZigNet2SpiM$USARTControl$setClockSource(uint8_t arg_0x41157800){
#line 68
  UniSART1M$USARTControl$setClockSource(arg_0x41157800);
#line 68
}
#line 68
# 79 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_CLR_SATELLITE_INTA_2_PIN(void)
#line 79
{
#line 79
   static volatile uint8_t r __asm ("0x0031");

#line 79
  r &= ~(1 << 6);
}

#line 77
static inline void TOSH_CLR_SATELLITE_INTR_PIN(void)
#line 77
{
#line 77
   static volatile uint8_t r __asm ("0x0031");

#line 77
  r &= ~(1 << 4);
}

#line 73
static inline void TOSH_MAKE_SATELLITE_CSN_OUTPUT(void)
#line 73
{
#line 73
   static volatile uint8_t r __asm ("0x0032");

#line 73
  r |= 1 << 0;
}

#line 73
static inline void TOSH_SET_SATELLITE_CSN_PIN(void)
#line 73
{
#line 73
   static volatile uint8_t r __asm ("0x0031");

#line 73
  r |= 1 << 0;
}

# 284 "../../../zigzag/ZigNet2SpiM.nc"
static inline  result_t ZigNet2SpiM$StdControl$start(void)
{

  TOSH_SET_SATELLITE_CSN_PIN();
  TOSH_MAKE_SATELLITE_CSN_OUTPUT();




  TOSH_CLR_SATELLITE_INTR_PIN();
  TOSH_CLR_SATELLITE_INTA_2_PIN();

  ZigNet2SpiM$USARTControl$setClockSource(0x20);
  ZigNet2SpiM$USARTControl$setClockRate(0x08, 0);
  ZigNet2SpiM$USARTControl$setModeSPI(TRUE);
  ZigNet2SpiM$USARTControl$disableRxIntr();
  ZigNet2SpiM$USARTControl$disableTxIntr();

  ZigNet2SpiM$INTR_2$disable();
  ZigNet2SpiM$INTR_2$clear();
  ZigNet2SpiM$INTR_2$edge(TRUE);
  ZigNet2SpiM$INTR_2$enable();

  return SUCCESS;
}

# 390 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static inline  result_t MacAssocCoordM$IStdControl$start(void)
{
  MacAssocCoordM$disassoc.state = MacAssocCoordM$MAC_DISASSOCIATE_IDLE;
  return SUCCESS;
}

# 329 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  bool TimerSymbol2M$ITimerSymbol$IsSet(uint8_t num)
{
  return TimerSymbol2M$timers[num].isSet;
}

# 11 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  bool MacPoolM$IExpiredTimer$IsSet(void){
#line 11
  unsigned char result;
#line 11

#line 11
  result = TimerSymbol2M$ITimerSymbol$IsSet(3U);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 274 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static inline  result_t MacPoolM$IStdControl$start(void)
{
  if (MacPoolM$IExpiredTimer$IsSet()) {
    MacPoolM$IExpiredTimer$Stop();
    }
#line 278
  return SUCCESS;
}

# 406 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  result_t MacSuperframesM$StdControl$start(void)
{
  return SUCCESS;
}

# 7 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
inline static  result_t MacCommonAttrM$IPhyAttr$SetextendedAddress(uint64_t arg_0x40b384b0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = PhyCC2420M$IPhyAttr$SetextendedAddress(arg_0x40b384b0);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 176 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  result_t MacCommonAttrM$IStdControl$start(void)
{
  uint64_t macExtendedAddress = info_param.MAC_ADDRESS;

#line 179
  MacCommonAttrM$IPhyAttr$SetextendedAddress(macExtendedAddress);
  return SUCCESS;
}

# 23 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacRandomM.nc"
static inline  result_t MacRandomM$IStdControl$start(void)
#line 23
{
#line 23
  return SUCCESS;
}

# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t PhyCC2420M$IPhyPool$Alloc(const uint8_t arg_0x40b5e6b8, const uint16_t arg_0x40b5e860, TPhyPoolHandle *const arg_0x40b5ea58){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyPoolM$IPhyPool$Alloc(arg_0x40b5e6b8, arg_0x40b5e860, arg_0x40b5ea58);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 737 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  result_t PhyCC2420M$IStdControl$start(void)
{
  PhyCC2420M$TPhyRadioState _radioState;

#line 740
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 740
    _radioState = PhyCC2420M$radioState;
#line 740
    __nesc_atomic_end(__nesc_atomic); }
  if (PhyCC2420M$PHY_RADIO_OFF == _radioState) {
      if (SUCCESS == PhyCC2420M$IPhyPool$Alloc(PHY_AMAX_PHY_PACKET_SIZE, 0, &PhyCC2420M$g_rxHandle)) {
          PhyCC2420M$turnOnChipcon();
          PhyCC2420M$turnRxOnChipcon();
          return SUCCESS;
        }
    }
  return FAIL;
}

# 114 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static inline  result_t PhyPoolM$IStdControl$start(void)
{

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

# 36 "../../../zigzag/ZigBee/implementation/NeighborTableM.nc"
static inline  void NeighborTableM$Reset$reset(void)
{
  int i;

#line 39
  NeighborTableM$free_idx = 0;
  for (i = 0; i < 20; i++) 
    {
      {
#line 42
        NeighborTableM$Table[i].networkAddr = 0xFFFF;
#line 42
        NeighborTableM$Table[i].extendedAddr = 0xFFFFFFFFFFFFFFFFLL;
      }
#line 42
      ;
    }
}

# 152 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
static inline  void NLME_JoinParentM$Reset$reset(void)
{
  NLME_JoinParentM$state = NLME_JoinParentM$IDLE;
}

# 33 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
inline static  void RouterM$NLDE_Data$confirm(uint8_t arg_0x408e47f8, NwkStatus arg_0x408e4990){
#line 33
  ZigNet2SpiM$IZigDATA$confirm(arg_0x408e47f8, arg_0x408e4990);
#line 33
}
#line 33
# 475 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline  void RouterM$Reset$reset(void)
{
  if (RouterM$wait_confirm && !RouterM$relaying) {
    RouterM$NLDE_Data$confirm(RouterM$current_nsduHandle, MAC_CHANNEL_ACCESS_FAILURE);
    }
#line 479
  RouterM$wait_confirm = FALSE;
  RouterM$reportErrorWhenGetFree = FALSE;
}

# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
inline static   uint16_t NIBM$Rand$Rand(void){
#line 3
  unsigned int result;
#line 3

#line 3
  result = MacRandomM$IMacRandom$Rand();
#line 3

#line 3
  return result;
#line 3
}
#line 3
# 238 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$Reset$reset(void)
{
  NIBM$nwkSequenceNumber = (uint8_t )NIBM$Rand$Rand();

  NIBM$nwkPassiveAckTimeout = NIBM$DEFAULT_nwkPassiveAckTimeout;
  NIBM$nwkMaxBroadcastRetries = NIBM$DEFAULT_nwkMaxBroadcastRetries;
  NIBM$nwkMaxChildren = NIBM$DEFAULT_nwkMaxChildren;
  NIBM$nwkMaxDepth = NIBM$DEFAULT_nwkMaxDepth;
  NIBM$nwkMaxRouters = NIBM$DEFAULT_nwkMaxRouters;
  NIBM$nwkNetworkBroadcastDeliveryTime = NIBM$DEFAULT_nwkNetworkBroadcastDeliveryTime;
  NIBM$nwkReportConstantCost = NIBM$DEFAULT_nwkReportConstantCost;
  NIBM$nwkRouteDiscoveryRetriesPermitted = NIBM$DEFAULT_nwkRouteDiscoveryRetriesPermitted;
  NIBM$nwkSymLink = NIBM$DEFAULT_nwkSymLink;

  NIBM$nwkUseTreeAddrAlloc = NIBM$DEFAULT_nwkUseTreeAddrAlloc;
  NIBM$nwkUseTreeRouting = NIBM$DEFAULT_nwkUseTreeRouting;
  NIBM$nwkNextAddress = NIBM$DEFAULT_nwkNextAddress;
  NIBM$nwkAvailableAddresses = NIBM$DEFAULT_nwkAvailableAddresses;
  NIBM$nwkAddressIncrement = NIBM$DEFAULT_nwkAddressIncrement;
  NIBM$nwkTransactionPersistenceTime = NIBM$DEFAULT_nwkTransactionPersistenceTime;

  NIBM$onlineStatus = NWK_OFFLINE;
  NIBM$depth = 0;
  NIBM$channel = 0;
}

# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
inline static  void NLME_ResetM$Reset$reset(void){
#line 46
  NIBM$Reset$reset();
#line 46
  RouterM$Reset$reset();
#line 46
  NLME_JoinParentM$Reset$reset();
#line 46
  NeighborTableM$Reset$reset();
#line 46
  NwkAddressingM$Reset$reset();
#line 46
}
#line 46
# 80 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_CLR_JOIN_PIN(void)
#line 80
{
#line 80
   static volatile uint8_t r __asm ("0x0031");

#line 80
  r &= ~(1 << 7);
}

# 196 "../../../zigzag/ZigNet2SpiM.nc"
static inline  void ZigNet2SpiM$IZigReset$confirm(NwkStatus status)
{
  if (NWK_SUCCESS == status) {
      TOSH_CLR_JOIN_PIN();
    }

  return;
}

# 880 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  result_t PhyCC2420M$IPhyAttr$SetAutoAck(bool autoAck)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 882
    PhyCC2420M$g_autoAck = autoAck;
#line 882
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 11 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
inline static  result_t MacStartM$IPhyAttr$SetAutoAck(bool arg_0x40b38dd0){
#line 11
  unsigned char result;
#line 11

#line 11
  result = PhyCC2420M$IPhyAttr$SetAutoAck(arg_0x40b38dd0);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 22 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
inline static  result_t MacStartM$IMacSTART$Confirm(TMacStatus arg_0x40928820){
#line 22
  unsigned char result;
#line 22

#line 22
  result = NLME_NetworkFormationM$IMacSTART$Confirm(arg_0x40928820);
#line 22

#line 22
  return result;
#line 22
}
#line 22
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacStartM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(3U, arg_0x40b477b8, arg_0x40b47960);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
inline static   TSysTime MacSuperframesM$ILocalTime$Read(void){
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
# 49 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static inline  TMacSuperframeOrder MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(
TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 54
  return MacSuperframeAttrC$macSuperframeOrder;
}

# 10 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacSuperframeOrder MacSuperframesM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e80a60){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(arg_0x40e80a60);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 27 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static inline  TMacBeaconOrder MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(
TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 32
  return MacSuperframeAttrC$macBeaconOrder;
}

# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacSuperframesM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e84d90);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 380 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline void MacSuperframesM$calcIntervals(void)
{
  MacSuperframesM$beacon_interval = (uint32_t )MAC_ABASE_SUPERFRAME_DURATION << MacSuperframesM$IMacSuperframeAttr$GetmacBeaconOrder(NULL);


  MacSuperframesM$sf_duration = (uint32_t )MAC_ABASE_SUPERFRAME_DURATION << MacSuperframesM$IMacSuperframeAttr$GetmacSuperframeOrder(NULL);
}

# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacSuperframesM$IPhyPool$Alloc(const uint8_t arg_0x40b5e6b8, const uint16_t arg_0x40b5e860, TPhyPoolHandle *const arg_0x40b5ea58){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyPoolM$IPhyPool$Alloc(arg_0x40b5e6b8, arg_0x40b5e860, arg_0x40b5ea58);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 105 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  result_t MacSuperframesM$runOwnSF(TSysTime offset)
{





  {
    if (MacSuperframesM$own_active) {
      return FAIL;
      }
  }
  if (!MacSuperframesM$beacon_allocated) 
    {
      {
#line 119
        MacSuperframesM$IPhyPool$Alloc(PHY_AMAX_PHY_PACKET_SIZE, 0, &MacSuperframesM$beacon_handle);
      }
#line 119
      ;
      MacSuperframesM$beacon_allocated = TRUE;
    }
  if (MacSuperframesM$parent_active) 
    {
      MacSuperframesM$sf_offset = offset;
    }
  else 
    {
      MacSuperframesM$sf_offset = MacSuperframesM$OWN_SF_PREPARATION_TIME + 400;
      MacSuperframesM$current_sf = MacSuperframesM$SF_OWN;
      MacSuperframesM$calcIntervals();
      MacSuperframesM$timestamp = MacSuperframesM$ILocalTime$Read();
    }

  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {

      MacSuperframesM$own_active = TRUE;
      if (!MacSuperframesM$parent_active) {
        MacSuperframesM$newCycle();
        }
    }
#line 142
    __nesc_atomic_end(__nesc_atomic); }
#line 142
  return SUCCESS;
}

# 20 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacStartM.nc"
inline static  result_t MacStartM$runOwnSF(TSysTime arg_0x41056ce8){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacSuperframesM$runOwnSF(arg_0x41056ce8);
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
inline static  TPhyStatus MacStartM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b4a180){
#line 5
  enum __nesc_unnamed4242 result;
#line 5

#line 5
  result = PhyCC2420M$IPhySET$SetphyCurrentChannel(arg_0x40b4a180);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacStartM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dc9198){
#line 7
  enum __nesc_unnamed4280 result;
#line 7

#line 7
  result = MacCommonAttrM$IMacCommonAttr$SetmacPANId(arg_0x40dc9198);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 56 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static inline  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetmacSuperframeOrder(
TMacSuperframeOrder superframeOrder)
{
  MacSuperframeAttrC$macSuperframeOrder = superframeOrder;
  return MAC_SUCCESS;
}

# 11 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacStatus MacStartM$IMacSuperframeAttr$SetmacSuperframeOrder(TMacSuperframeOrder arg_0x40e80f10){
#line 11
  enum __nesc_unnamed4280 result;
#line 11

#line 11
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetmacSuperframeOrder(arg_0x40e80f10);
#line 11

#line 11
  return result;
#line 11
}
#line 11
#line 7
inline static  TMacStatus MacStartM$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e80248){
#line 7
  enum __nesc_unnamed4280 result;
#line 7

#line 7
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(arg_0x40e80248);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacShortAddress MacStartM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dc99a8){
#line 10
  unsigned int result;
#line 10

#line 10
  result = MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(arg_0x40dc99a8);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 29 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacStartM.nc"
static inline  result_t MacStartM$IMacSTART$Request(
TMacPANId panId, 
TMacLogicalChannel logicalChannel, 
TMacBeaconOrder beaconOrder, 
TMacSuperframeOrder superframeOrder, 
bool panCoordinator, 
bool batteryLifeExtension, 
bool coordRealignment, 
bool securityEnable, 
TSysTime startTime)


{
  if (MacStartM$IMacCommonAttr$GetmacShortAddress(NULL) == 0xffff) 
    {
      MacStartM$IMacSTART$Confirm(MAC_NO_SHORT_ADDRESS);
      return SUCCESS;
    }

  if (MacStartM$IMacSuperframeAttr$SetmacBeaconOrder(beaconOrder) != MAC_SUCCESS) 
    {
      MacStartM$IMacSTART$Confirm(MAC_INVALID_PARAMETER);
      return SUCCESS;
    }

  if (beaconOrder == 15) {
    superframeOrder = 15;
    }
  if (MacStartM$IMacSuperframeAttr$SetmacSuperframeOrder(superframeOrder) != MAC_SUCCESS) 
    {
      MacStartM$IMacSTART$Confirm(MAC_INVALID_PARAMETER);
      return SUCCESS;
    }

  if (panCoordinator) 
    {
      MacStartM$IMacCommonAttr$SetmacPANId(panId);
      MacStartM$IPhySET$SetphyCurrentChannel(logicalChannel);
      startTime = 0;
    }



  {
    result_t result;

#line 74
    if (beaconOrder != 15) {
      result = MacStartM$runOwnSF(startTime);
      }
    else {
#line 77
      result = MacStartM$IPhySET_TRX_STATE$Request(PHY_RX_ON, 0);
      }
    if (result) {
      MacStartM$IMacSTART$Confirm(MAC_SUCCESS);
      }
    else {
#line 82
      MacStartM$IMacSTART$Confirm(MAC_INVALID_PARAMETER);
      }
  }
  MacStartM$IPhyAttr$SetAutoAck(TRUE);
  return SUCCESS;
}

# 10 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
inline static  result_t NLME_NetworkFormationM$IMacSTART$Request(TMacPANId arg_0x409455b0, TMacLogicalChannel arg_0x40945758, TMacBeaconOrder arg_0x409458f8, TMacSuperframeOrder arg_0x40945aa0, bool arg_0x40945c38, bool arg_0x40945dd8, bool arg_0x40928010, bool arg_0x409281a8, TSysTime arg_0x40928358){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacStartM$IMacSTART$Request(arg_0x409455b0, arg_0x40945758, arg_0x409458f8, arg_0x40945aa0, arg_0x40945c38, arg_0x40945dd8, arg_0x40928010, arg_0x409281a8, arg_0x40928358);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 24 "../../../zigzag/ZigBee/implementation/NLME_NetworkFormationM.nc"
inline static  void NLME_NetworkFormationM$updateBeacon(void){
#line 24
  NwkBeaconParentM$updateBeacon();
#line 24
}
#line 24
# 285 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setBeaconOffset(TSysTime offs)
#line 285
{
#line 285
  NIBM$beaconOffset = offs;
}

# 204 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NLME_NetworkFormationM$NIB$setBeaconOffset(TSysTime arg_0x408da228){
#line 204
  NIBM$NIB$setBeaconOffset(arg_0x408da228);
#line 204
}
#line 204
# 275 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setDepth(uint8_t d)
{
  NIBM$depth = d;
}

# 197 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NLME_NetworkFormationM$NIB$setDepth(uint8_t arg_0x408bd620){
#line 197
  NIBM$NIB$setDepth(arg_0x408bd620);
#line 197
}
#line 197
# 281 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setChannel(uint8_t c)
#line 281
{
#line 281
  NIBM$channel = c;
}

# 201 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NLME_NetworkFormationM$NIB$setChannel(uint8_t arg_0x408bdd90){
#line 201
  NIBM$NIB$setChannel(arg_0x408bdd90);
#line 201
}
#line 201
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dc9198){
#line 7
  enum __nesc_unnamed4280 result;
#line 7

#line 7
  result = MacCommonAttrM$IMacCommonAttr$SetmacPANId(arg_0x40dc9198);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 6 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacPANId(TMacPANId macPANId)
{
  return MacCoordAttrM$IMacCommonAttr$SetmacPANId(macPANId);
}

# 36 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_NetworkFormationM$IMacSET$SetmacPANId(TMacPANId arg_0x4092e1c0){
#line 36
  enum __nesc_unnamed4280 result;
#line 36

#line 36
  result = MacCoordAttrM$IMacSET$SetmacPANId(arg_0x4092e1c0);
#line 36

#line 36
  return result;
#line 36
}
#line 36
# 94 "../../../zigzag/ZigBee/implementation/NLME_NetworkFormationM.nc"
static inline void NLME_NetworkFormationM$start_mac(void)
{

  NLME_NetworkFormationM$IMacSET$SetmacPANId(NLME_NetworkFormationM$request_panID);





  NLME_NetworkFormationM$NIB$setChannel(NLME_NetworkFormationM$channel);

  NLME_NetworkFormationM$NIB$setDepth(0);
  NLME_NetworkFormationM$NIB$setBeaconOffset(0);
  NLME_NetworkFormationM$updateBeacon();

  NLME_NetworkFormationM$IMacSTART$Request(NLME_NetworkFormationM$request_panID, NLME_NetworkFormationM$channel, 
  NLME_NetworkFormationM$request_beaconOrder, 
  NLME_NetworkFormationM$request_superframeOrder, 
  TRUE, FALSE, FALSE, FALSE, 0);
}

# 80 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_SET_JOIN_PIN(void)
#line 80
{
#line 80
   static volatile uint8_t r __asm ("0x0031");

#line 80
  r |= 1 << 7;
}

# 188 "../../../zigzag/ZigNet2SpiM.nc"
static inline  void ZigNet2SpiM$IZigNetFormation$confirm(NwkStatus status)
{
  if (NWK_SUCCESS == status) {
      TOSH_SET_JOIN_PIN();
    }
  return;
}

# 25 "../../../zigzag/ZigBee/interface/NLME_NetworkFormation.nc"
inline static  void NLME_NetworkFormationM$NLME_NetworkFormation$confirm(NwkStatus arg_0x408b09a8){
#line 25
  ZigCoordM$IZigNetFormation$confirm(arg_0x408b09a8);
#line 25
  ZigNet2SpiM$IZigNetFormation$confirm(arg_0x408b09a8);
#line 25
}
#line 25
# 19 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
inline static  bool NLME_NetworkFormationM$NwkAddressing$hasVacantAddrs(void){
#line 19
  unsigned char result;
#line 19

#line 19
  result = NwkAddressingM$NwkAddressing$hasVacantAddrs();
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 270 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  NwkOnlineStatus NIBM$NIB$getOnlineStatus(void)
#line 270
{
#line 270
  return NIBM$onlineStatus;
}

# 192 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  NwkOnlineStatus NLME_NetworkFormationM$NIB$getOnlineStatus(void){
#line 192
  enum __nesc_unnamed4334 result;
#line 192

#line 192
  result = NIBM$NIB$getOnlineStatus();
#line 192

#line 192
  return result;
#line 192
}
#line 192
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
inline static  void NLME_NetworkFormationM$NwkAddressingReset$reset(void){
#line 46
  NwkAddressingM$Reset$reset();
#line 46
}
#line 46
# 154 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(bool macRxOnWhenIdle)
{
  MacCommonAttrM$pib.macRxOnWhenIdle = macRxOnWhenIdle;
  return MAC_SUCCESS;
}

# 31 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(bool arg_0x40dc4f00){
#line 31
  enum __nesc_unnamed4280 result;
#line 31

#line 31
  result = MacCommonAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(arg_0x40dc4f00);
#line 31

#line 31
  return result;
#line 31
}
#line 31
# 155 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacRxOnWhenIdle(
bool macRxOnWhenIdle)
{
  return MacCoordAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(macRxOnWhenIdle);
}

# 40 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_NetworkFormationM$IMacSET$SetmacRxOnWhenIdle(bool arg_0x4092eaf8){
#line 40
  enum __nesc_unnamed4280 result;
#line 40

#line 40
  result = MacCoordAttrM$IMacSET$SetmacRxOnWhenIdle(arg_0x4092eaf8);
#line 40

#line 40
  return result;
#line 40
}
#line 40
# 682 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetmacBattLifeExt(bool battLifeExt)
{
  MacCAPM$macBattLifeExt = battLifeExt;
  return MAC_SUCCESS;
}

# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetmacBattLifeExt(bool arg_0x40ebce20){
#line 7
  enum __nesc_unnamed4280 result;
#line 7

#line 7
  result = MacCAPM$IMacCAPAttr$SetmacBattLifeExt(arg_0x40ebce20);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 108 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacBattLifeExt(bool macBattLifeExt)
{
  return MacCoordAttrM$IMacCAPAttr$SetmacBattLifeExt(macBattLifeExt);
}

# 11 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_NetworkFormationM$IMacSET$SetmacBattLifeExt(bool arg_0x409346b8){
#line 11
  enum __nesc_unnamed4280 result;
#line 11

#line 11
  result = MacCoordAttrM$IMacSET$SetmacBattLifeExt(arg_0x409346b8);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 3 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
inline static  result_t MacCommonAttrM$IPhyAttr$SetshortAddress(uint16_t arg_0x40b39ad8){
#line 3
  unsigned char result;
#line 3

#line 3
  result = PhyCC2420M$IPhyAttr$SetshortAddress(arg_0x40b39ad8);
#line 3

#line 3
  return result;
#line 3
}
#line 3
# 133 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacShortAddress(TMacShortAddress macShortAddress)
{
  if (SUCCESS == MacCommonAttrM$IPhyAttr$SetshortAddress(macShortAddress)) {
      MacCommonAttrM$pib.macShortAddress = macShortAddress;
      return MAC_SUCCESS;
    }
  return MAC_INVALID_PARAMETER;
}

# 11 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetmacShortAddress(TMacShortAddress arg_0x40dc9e50){
#line 11
  enum __nesc_unnamed4280 result;
#line 11

#line 11
  result = MacCommonAttrM$IMacCommonAttr$SetmacShortAddress(arg_0x40dc9e50);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 16 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacShortAddress(TMacShortAddress macShortAddress)
{
  return MacCoordAttrM$IMacCommonAttr$SetmacShortAddress(macShortAddress);
}

# 42 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_NetworkFormationM$IMacSET$SetmacShortAddress(TMacShortAddress arg_0x4092d010){
#line 42
  enum __nesc_unnamed4280 result;
#line 42

#line 42
  result = MacCoordAttrM$IMacSET$SetmacShortAddress(arg_0x4092d010);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 39 "../../../zigzag/ZigBee/implementation/NLME_NetworkFormationM.nc"
static inline  void NLME_NetworkFormationM$NLME_NetworkFormation$request(
uint32_t scanChannels, 
uint8_t scanDuration, 
uint8_t beaconOrder, 
uint8_t superframeOrder, 

PanID_t panID, 
bool batteryLifeExtension)

{
  NLME_NetworkFormationM$request_panID = panID;
  NLME_NetworkFormationM$request_beaconOrder = beaconOrder;
  NLME_NetworkFormationM$request_superframeOrder = superframeOrder;
  NLME_NetworkFormationM$channel = 0;


  while (!((scanChannels >> NLME_NetworkFormationM$channel) & 1)) 
    NLME_NetworkFormationM$channel++;


  NLME_NetworkFormationM$NIB$setDepth(0);
  NLME_NetworkFormationM$IMacSET$SetmacShortAddress(0x0000);
  NLME_NetworkFormationM$IMacSET$SetmacBattLifeExt(batteryLifeExtension);
  NLME_NetworkFormationM$IMacSET$SetmacRxOnWhenIdle(TRUE);

  NLME_NetworkFormationM$NwkAddressingReset$reset();




  if (((
#line 67
  !NWK_COORDINATOR_CAPABLE || 
  NLME_NetworkFormationM$NIB$getOnlineStatus() == NWK_JOINED_AS_ROUTER) || 
  NLME_NetworkFormationM$NIB$getOnlineStatus() == NWK_JOINED_AS_ED) || 
  !NLME_NetworkFormationM$NwkAddressing$hasVacantAddrs()) 
    {

      NLME_NetworkFormationM$NLME_NetworkFormation$confirm(NWK_INVALID_REQUEST);
      return;
    }
  else 
    {

      NLME_NetworkFormationM$start_mac();
    }
}

# 15 "../../../zigzag/ZigBee/interface/NLME_NetworkFormation.nc"
inline static  void ZigCoordM$IZigNetFormation$request(uint32_t arg_0x408aecd0, uint8_t arg_0x408aee68, uint8_t arg_0x408b0030, uint8_t arg_0x408b01c8, PanID_t arg_0x408b0358, bool arg_0x408b0508){
#line 15
  NLME_NetworkFormationM$NLME_NetworkFormation$request(arg_0x408aecd0, arg_0x408aee68, arg_0x408b0030, arg_0x408b01c8, arg_0x408b0358, arg_0x408b0508);
#line 15
}
#line 15
# 109 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setNwkMaxDepth(uint8_t attr)
{
  NIBM$nwkMaxDepth = attr;
}

# 55 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void ZigCoordM$NIB$setNwkMaxDepth(uint8_t arg_0x408c7328){
#line 55
  NIBM$NIB$setNwkMaxDepth(arg_0x408c7328);
#line 55
}
#line 55
# 119 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setNwkMaxRouters(uint8_t attr)
{
  NIBM$nwkMaxRouters = attr;
}

# 65 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void ZigCoordM$NIB$setNwkMaxRouters(uint8_t arg_0x408c7aa0){
#line 65
  NIBM$NIB$setNwkMaxRouters(arg_0x408c7aa0);
#line 65
}
#line 65
# 99 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setNwkMaxChildren(uint8_t attr)
{
  NIBM$nwkMaxChildren = attr;
}

# 47 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void ZigCoordM$NIB$setNwkMaxChildren(uint8_t arg_0x408c8b40){
#line 47
  NIBM$NIB$setNwkMaxChildren(arg_0x408c8b40);
#line 47
}
#line 47
# 30 "../../../zigzag/ZigCoordM.nc"
static inline  void ZigCoordM$networkFormation(void)
{
  ;
  ZigCoordM$NIB$setNwkMaxChildren(info_param.Z_MAX_CHILDREN);
  ZigCoordM$NIB$setNwkMaxRouters(info_param.Z_MAX_ROUTERS);
  ZigCoordM$NIB$setNwkMaxDepth(info_param.Z_MAX_DEPTH);

  ZigCoordM$IZigNetFormation$request(
  info_param.Z_CHANNELS, 
  1, 
  info_param.Z_BEACON_ORDER, 
  info_param.Z_SUPERFRAME_ORDER, 
  info_param.Z_PAN_ID, 
  FALSE);
  return;
}


static inline  void ZigCoordM$IZigReset$confirm(NwkStatus status)
{
  TOS_post(ZigCoordM$networkFormation);
  return;
}

# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
inline static  void NLME_ResetM$NLME_Reset$confirm(NwkStatus arg_0x408d5c68){
#line 16
  ZigCoordM$IZigReset$confirm(arg_0x408d5c68);
#line 16
  ZigNet2SpiM$IZigReset$confirm(arg_0x408d5c68);
#line 16
}
#line 16
# 29 "../../../zigzag/ZigBee/implementation/NLME_ResetM.nc"
static inline  void NLME_ResetM$MacReset$confirm(NwkStatus status)
{
  NLME_ResetM$waitConfirm = FALSE;
  NLME_ResetM$NLME_Reset$confirm(status);
}

# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
inline static  void NwkResetMacSingleM$NLME_Reset$confirm(NwkStatus arg_0x408d5c68){
#line 16
  NLME_ResetM$MacReset$confirm(arg_0x408d5c68);
#line 16
}
#line 16
# 19 "../../../zigzag/ZigBee/implementation/NwkResetMacSingleM.nc"
static inline  result_t NwkResetMacSingleM$IMacRESET$Confirm(TMacStatus status)
{
  NwkResetMacSingleM$NLME_Reset$confirm(status);
  return SUCCESS;
}

# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
inline static  result_t MacCoordAttrM$IMacRESET$Confirm(TMacStatus arg_0x409c24b8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = NwkResetMacSingleM$IMacRESET$Confirm(arg_0x409c24b8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 291 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  void MacCoordAttrM$confirmReset(void)
{
  MacCoordAttrM$IMacRESET$Confirm(MAC_SUCCESS);
}

# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacCoordAttrM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(4U, arg_0x40b477b8, arg_0x40b47960);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 20 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacAssociationPermit(void){
#line 20
  enum __nesc_unnamed4280 result;
#line 20

#line 20
  result = MacAssocCoordM$IMacAssocAttr$SetDefaultmacAssociationPermit();
#line 20

#line 20
  return result;
#line 20
}
#line 20
#line 16
inline static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void){
#line 16
  enum __nesc_unnamed4280 result;
#line 16

#line 16
  result = MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordShortAddress();
#line 16

#line 16
  return result;
#line 16
}
#line 16
#line 10
inline static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void){
#line 10
  enum __nesc_unnamed4280 result;
#line 10

#line 10
  result = MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress();
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 26 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconTxTime(void)
{
  MacBeaconAttrCoordM$beacon_tx_time = 0x0;
  return MAC_SUCCESS;
}

# 29 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetDefaultmacBeaconTxTime(void){
#line 29
  enum __nesc_unnamed4280 result;
#line 29

#line 29
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconTxTime();
#line 29

#line 29
  return result;
#line 29
}
#line 29
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
inline static   uint16_t MacBeaconAttrCoordM$IMacRandom$Rand(void){
#line 3
  unsigned int result;
#line 3

#line 3
  result = MacRandomM$IMacRandom$Rand();
#line 3

#line 3
  return result;
#line 3
}
#line 3
# 122 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBSN(void)
{
  MacBeaconAttrCoordM$macBSN = (TMacBSN )MacBeaconAttrCoordM$IMacRandom$Rand();
  return MAC_SUCCESS;
}

# 23 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetDefaultmacBSN(void){
#line 23
  enum __nesc_unnamed4280 result;
#line 23

#line 23
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBSN();
#line 23

#line 23
  return result;
#line 23
}
#line 23
# 80 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconPayload(void)
{
  MacBeaconAttrCoordM$beaconPayloadLength = 0;
  return MAC_SUCCESS;
}

# 19 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetDefaultmacBeaconPayload(void){
#line 19
  enum __nesc_unnamed4280 result;
#line 19

#line 19
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconPayload();
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 101 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacAutoRequest(void)
{
  MacBeaconAttrCoordM$macAutoRequest = TRUE;
  return MAC_SUCCESS;
}

# 9 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetDefaultmacAutoRequest(void){
#line 9
  enum __nesc_unnamed4280 result;
#line 9

#line 9
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacAutoRequest();
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 730 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetmacMinBE(TMacBackoffExponent minBE)
{
  MacCAPM$macMinBE = minBE;
  return MAC_SUCCESS;
}

#line 735
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacMinBE(void)
{
  return MacCAPM$IMacCAPAttr$SetmacMinBE(MAC_BACKOFF_EXPONENT_3);
}

# 26 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacMinBE(void){
#line 26
  enum __nesc_unnamed4280 result;
#line 26

#line 26
  result = MacCAPM$IMacCAPAttr$SetDefaultmacMinBE();
#line 26

#line 26
  return result;
#line 26
}
#line 26
# 714 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetmacMaxCSMABackoffs(TMacMaxCSMABackoffs maxCSMABackoffs)
{
  MacCAPM$macMaxCSMABackoffs = maxCSMABackoffs;
  return MAC_SUCCESS;
}

#line 719
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs(void)
{
  return MacCAPM$IMacCAPAttr$SetmacMaxCSMABackoffs(MAC_MAX_CSMA_BACKOFFS_4);
}

# 20 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs(void){
#line 20
  enum __nesc_unnamed4280 result;
#line 20

#line 20
  result = MacCAPM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs();
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 698 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetmacBattLifeExtPeriods(TMacBattLifeExtPeriods battLifeExtPeriods)
{
  MacCAPM$macBattLifeExtPeriods = battLifeExtPeriods;
  return MAC_SUCCESS;
}

#line 703
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods(void)
{
  return MacCAPM$IMacCAPAttr$SetmacBattLifeExtPeriods(MAC_BATT_LIFE_EXT_PERIOD_6);
}

# 14 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods(void){
#line 14
  enum __nesc_unnamed4280 result;
#line 14

#line 14
  result = MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods();
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 687 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExt(void)
{
  return MacCAPM$IMacCAPAttr$SetmacBattLifeExt(FALSE);
}

# 8 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacBattLifeExt(void){
#line 8
  enum __nesc_unnamed4280 result;
#line 8

#line 8
  result = MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExt();
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 10 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPoolAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void){
#line 10
  enum __nesc_unnamed4280 result;
#line 10

#line 10
  result = MacPoolM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime();
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 62 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static inline  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void)
{
  MacSuperframeAttrC$macSuperframeOrder = MacSuperframeAttrC$MAX_SUPERFRAME_ORDER;
  return MAC_SUCCESS;
}

# 12 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void){
#line 12
  enum __nesc_unnamed4280 result;
#line 12

#line 12
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder();
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 43 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static inline  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void)
{
  MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(MacSuperframeAttrC$MAX_BEACON_ORDER);
  return MAC_SUCCESS;
}

# 8 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void){
#line 8
  enum __nesc_unnamed4280 result;
#line 8

#line 8
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacBeaconOrder();
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 8 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacPANId(void){
#line 8
  enum __nesc_unnamed4280 result;
#line 8

#line 8
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacPANId();
#line 8

#line 8
  return result;
#line 8
}
#line 8




inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void){
#line 12
  enum __nesc_unnamed4280 result;
#line 12

#line 12
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacShortAddress();
#line 12

#line 12
  return result;
#line 12
}
#line 12
#line 32
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void){
#line 32
  enum __nesc_unnamed4280 result;
#line 32

#line 32
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle();
#line 32

#line 32
  return result;
#line 32
}
#line 32
#line 20
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacDSN(void){
#line 20
  enum __nesc_unnamed4280 result;
#line 20

#line 20
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacDSN();
#line 20

#line 20
  return result;
#line 20
}
#line 20




inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void){
#line 24
  enum __nesc_unnamed4280 result;
#line 24

#line 24
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode();
#line 24

#line 24
  return result;
#line 24
}
#line 24




inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void){
#line 28
  enum __nesc_unnamed4280 result;
#line 28

#line 28
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacSecurityMode();
#line 28

#line 28
  return result;
#line 28
}
#line 28
#line 16
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void){
#line 16
  enum __nesc_unnamed4280 result;
#line 16

#line 16
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration();
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 260 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static __inline void MacCoordAttrM$SetDefaultAttributes(void)
{
  MacCoordAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration();
  MacCoordAttrM$IMacCommonAttr$SetDefaultmacSecurityMode();
  MacCoordAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode();
  MacCoordAttrM$IMacCommonAttr$SetDefaultmacDSN();
  MacCoordAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle();
  MacCoordAttrM$IMacCommonAttr$SetDefaultmacShortAddress();
  MacCoordAttrM$IMacCommonAttr$SetDefaultmacPANId();

  MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacBeaconOrder();
  MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacSuperframeOrder();

  MacCoordAttrM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime();

  MacCoordAttrM$IMacCAPAttr$SetDefaultmacBattLifeExt();
  MacCoordAttrM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods();
  MacCoordAttrM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs();
  MacCoordAttrM$IMacCAPAttr$SetDefaultmacMinBE();

  MacCoordAttrM$IMacBeaconAttr$SetDefaultmacAutoRequest();
  MacCoordAttrM$IMacBeaconAttr$SetDefaultmacBeaconPayload();
  MacCoordAttrM$IMacBeaconAttr$SetDefaultmacBSN();
  MacCoordAttrM$IMacBeaconAttr$SetDefaultmacBeaconTxTime();

  MacCoordAttrM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress();
  MacCoordAttrM$IMacAssocAttr$SetDefaultmacCoordShortAddress();
  MacCoordAttrM$IMacAssocAttr$SetDefaultmacAssociationPermit();
}

# 12 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacScanBeaconM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b5cbc0){
#line 12
  unsigned char result;
#line 12

#line 12
  result = PhyPoolM$IPhyPool$Free(arg_0x40b5cbc0);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 81 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanBeaconM.nc"
static inline  void MacScanBeaconM$Reset$reset(void)
{
  if (MacScanBeaconM$beaconAllocated) {
    MacScanBeaconM$IPhyPool$Free(MacScanBeaconM$beaconHandle);
    }
#line 85
  MacScanBeaconM$beaconAllocated = FALSE;
}

# 360 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline  result_t MacDataM$StdControl$start(void)
{
  MacDataM$state = MacDataM$IDLE;
  return SUCCESS;
}






static inline  result_t MacDataM$StdControl$stop(void)
{
  MacDataM$AckWaitTimer$Stop();
  MacDataM$state = MacDataM$OFF;
  return SUCCESS;
}

static inline  void MacDataM$Reset$reset(void)
{
  MacDataM$StdControl$stop();
  MacDataM$StdControl$start();
  return;
}

# 401 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static inline  void MacAssocCoordM$Reset$reset(void)
{
  MacAssocCoordM$disassoc.state = MacAssocCoordM$MAC_DISASSOCIATE_IDLE;
  return;
}

# 286 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static inline  void MacPoolM$Reset$reset(void)
{
  MacPoolM$ResetPool();
  if (MacPoolM$IExpiredTimer$IsSet()) {
    MacPoolM$IExpiredTimer$Stop();
    }
#line 291
  return;
}

# 12 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacCAPM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b5cbc0){
#line 12
  unsigned char result;
#line 12

#line 12
  result = PhyPoolM$IPhyPool$Free(arg_0x40b5cbc0);
#line 12

#line 12
  return result;
#line 12
}
#line 12
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
inline static  result_t MacCAPM$IBackoff$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(4U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 649 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  result_t MacCAPM$ICAPControl$stop(uint8_t sfIndex)
{
  MacCAPM$IBackoff$Stop();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      MacCAPM$cap[sfIndex].state = MacCAPM$CAP_STATE_PASSIVE;
      if (MacCAPM$CAP_SEND_IDLE < MacCAPM$cap[sfIndex].sendState) {
        MacCAPM$IPhyPool$Free(MacCAPM$cap[sfIndex].handle);
        }
#line 657
      MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_IDLE;
    }
#line 658
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 659
    MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_OFF;
#line 659
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 641
static inline  void MacCAPM$Reset$reset(uint8_t sfIndex)
{
  ;
  MacCAPM$ICAPControl$stop(sfIndex);
  MacCAPM$ICAPControl$start(sfIndex);
  return;
}

# 21 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static inline  void MacSuperframeAttrC$Reset$reset(void)
{
  MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacBeaconOrder();
  MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder();
}

# 12 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacSuperframesM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b5cbc0){
#line 12
  unsigned char result;
#line 12

#line 12
  result = PhyPoolM$IPhyPool$Free(arg_0x40b5cbc0);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 234 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline   result_t TimerSymbolAsyncM$ITimerSymbolAsync$Stop(uint8_t timerNumber)
{
  if (timerNumber >= TimerSymbolAsyncM$ASYNC_TIMER_NUM) {
#line 236
    return FAIL;
    }
#line 237
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 237
    {
      TimerSymbolAsyncM$timer[timerNumber].isSet = FALSE;
      TimerSymbolAsyncM$DisableEvents(timerNumber);
    }
#line 240
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
inline static   result_t MacSuperframesM$Timer$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbolAsyncM$ITimerSymbolAsync$Stop(0U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 410 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  result_t MacSuperframesM$StdControl$stop(void)
{
  MacSuperframesM$Timer$Stop();
  MacSuperframesM$parent_active = FALSE;
  MacSuperframesM$own_active = FALSE;
  if (MacSuperframesM$beacon_allocated) {
    MacSuperframesM$IPhyPool$Free(MacSuperframesM$beacon_handle);
    }
#line 417
  MacSuperframesM$beacon_allocated = FALSE;
  return SUCCESS;
}

static inline  void MacSuperframesM$Reset$reset(void)
{
  MacSuperframesM$StdControl$stop();
  MacSuperframesM$StdControl$start();
}

# 102 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static inline  void PhyPoolM$IPhyPoolReset$reset(void)
{

  return;
}

# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
inline static  void MacCoordAttrM$IMacReset$reset(void){
#line 46
  PhyPoolM$IPhyPoolReset$reset();
#line 46
  MacSuperframesM$Reset$reset();
#line 46
  MacSuperframeAttrC$Reset$reset();
#line 46
  MacCAPM$Reset$reset(MAC_OWN_SF);
#line 46
  MacPoolM$Reset$reset();
#line 46
  MacAssocCoordM$Reset$reset();
#line 46
  MacDataM$Reset$reset();
#line 46
  MacScanBeaconM$Reset$reset();
#line 46
}
#line 46
# 296 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  result_t MacCoordAttrM$IMacRESET$Request(bool setDefaultPIB)
{
  MacCoordAttrM$IMacReset$reset();
  if (setDefaultPIB) {
      MacCoordAttrM$SetDefaultAttributes();
    }
  MacCoordAttrM$IPhySET_TRX_STATE$Request(PHY_FORCE_TRX_OFF, 0);
  return TOS_post(MacCoordAttrM$confirmReset);
}

# 5 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
inline static  result_t NwkResetMacSingleM$IMacRESET$Request(bool arg_0x409c2030){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacCoordAttrM$IMacRESET$Request(arg_0x409c2030);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 14 "../../../zigzag/ZigBee/implementation/NwkResetMacSingleM.nc"
static inline  void NwkResetMacSingleM$NLME_Reset$request(void)
{
  NwkResetMacSingleM$IMacRESET$Request(TRUE);
}

# 15 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
inline static  void NLME_ResetM$MacReset$request(void){
#line 15
  NwkResetMacSingleM$NLME_Reset$request();
#line 15
}
#line 15
# 22 "../../../zigzag/ZigBee/implementation/NLME_ResetM.nc"
static inline  void NLME_ResetM$NLME_Reset$request(void)
{
  NLME_ResetM$waitConfirm = TRUE;
  NLME_ResetM$MacReset$request();
  NLME_ResetM$Reset$reset();
}

# 15 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
inline static  void ZigCoordM$IZigReset$request(void){
#line 15
  NLME_ResetM$NLME_Reset$request();
#line 15
}
#line 15
# 23 "../../../zigzag/ZigCoordM.nc"
static inline  void ZigCoordM$startDemo(void)
{
  ;
  ZigCoordM$IZigReset$request();
  return;
}

#line 74
static inline  result_t ZigCoordM$StdControl$start(void)
{
  TOS_post(ZigCoordM$startDemo);
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
  result = rcombine(result, ZigCoordM$StdControl$start());
#line 70
  result = rcombine(result, TimerSymbol2M$IStdControl$start());
#line 70
  result = rcombine(result, TimerSymbolAsyncM$IStdControl$start());
#line 70
  result = rcombine(result, PhyPoolM$IStdControl$start());
#line 70
  result = rcombine(result, PhyCC2420M$IStdControl$start());
#line 70
  result = rcombine(result, MacRandomM$IStdControl$start());
#line 70
  result = rcombine(result, MacCommonAttrM$IStdControl$start());
#line 70
  result = rcombine(result, MacSuperframesM$StdControl$start());
#line 70
  result = rcombine(result, MacCAPM$ICAPControl$start(MAC_OWN_SF));
#line 70
  result = rcombine(result, MacPoolM$IStdControl$start());
#line 70
  result = rcombine(result, MacAssocCoordM$IStdControl$start());
#line 70
  result = rcombine(result, ZigNet2SpiM$StdControl$start());
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 569 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  void MacCAPM$BeaconOrderChanged(uint8_t bo)
{
  MacCAPM$sfMode = bo == 15 ? MAC_SUPERFRAME_MODE_UNSLOTTED : MAC_SUPERFRAME_MODE_SLOTTED;
  if (bo == 15) 
    {
#line 588
      ;
    }
  else 
    {
      uint8_t i;

#line 593
      for (i = 0; i < MacCAPM$NUM_SUPERFRAMES; i++) 
        MacCAPM$cap[i].state = MacCAPM$CAP_STATE_PASSIVE;
    }
}

# 10 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
inline static  void MacSuperframeAttrC$BeaconOrderChanged(uint8_t arg_0x40ea5778){
#line 10
  MacCAPM$BeaconOrderChanged(arg_0x40ea5778);
#line 10
}
#line 10
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacCAPM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e84d90);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 136 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline   
#line 135
result_t PhyCC2420M$IPhyTxFIFO$Write(uint8_t context, 
TPhyPoolHandle handle, TUniData uniData)
{
  PhyCC2420M$TPhyRadioState _radioState;

#line 139
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 139
    {
      _radioState = PhyCC2420M$radioState;
      if (PhyCC2420M$radioState == PhyCC2420M$PHY_RADIO_TX_IDLE) {
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_INIT;
        }
    }
#line 144
    __nesc_atomic_end(__nesc_atomic); }
#line 144
  if (_radioState == PhyCC2420M$PHY_RADIO_TX_IDLE) {
      result_t result;

#line 146
      PhyCC2420M$radio.context = context, PhyCC2420M$radio.uniData = uniData, PhyCC2420M$radio.handle = handle;
      result = PhyCC2420M$WriteToTxFIFO(handle);
      if (SUCCESS == result) {
          PhyCC2420M$radio.txFifoSend = FALSE;
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 150
            PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_FIFO_WRITE;
#line 150
            __nesc_atomic_end(__nesc_atomic); }
          return SUCCESS;
        }
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 153
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_IDLE;
#line 153
        __nesc_atomic_end(__nesc_atomic); }
    }
  return FAIL;
}

# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
inline static   result_t MacSuperframesM$IPhyTxFIFO$Write(TPhyPoolHandle arg_0x40b2edd8, TUniData arg_0x40b29010){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhyTxFIFO$Write(0U, arg_0x40b2edd8, arg_0x40b29010);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 210 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  result_t MacSuperframesM$IPhyTrxOwn$Confirm(TPhyStatus status, TUniData unidata)
{
  TPhyStatus tx_status;


  tx_status = MacSuperframesM$IPhyTxFIFO$Write(MacSuperframesM$beacon_handle, 0);

  return SUCCESS;
}

#line 294
static inline  result_t MacSuperframesM$IPhyTrxParent$Confirm(TPhyStatus status, TUniData unidata)
{

  return SUCCESS;
}

# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
inline static   result_t MacCAPM$IPhyTxDATA$Request(const TPhyPoolHandle arg_0x40b2ab78, const TUniData arg_0x40b2ad30){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhyTxDATA$Request(1U, arg_0x40b2ab78, arg_0x40b2ad30);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 368 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  result_t MacCAPM$IPhySET_TRX_STATE$Confirm(TPhyStatus phyStatus, TUniData uniData)
{
  uint8_t sfIndex = (uint8_t )uniData;
  MacCAPM$TMacCAPSendState sendState;

#line 372
  sendState = MacCAPM$cap[sfIndex].sendState;



  if (sendState == MacCAPM$CAP_SEND_TX_ON) 
    {
      result_t result;

#line 379
      if (!MacCAPM$enoughTime(sfIndex)) 
        {
          MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_BACKOFF;
          MacCAPM$TurnOffIfInactive(PHY_TRX_OFF);
          return SUCCESS;
        }
      result = FAIL;
      if (PHY_SUCCESS == phyStatus || PHY_TX_ON == phyStatus) {
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
          {
            if (MacCAPM$cap[sfIndex].state != MacCAPM$CAP_STATE_ACTIVE) 

              {
                MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_BACKOFF;
                MacCAPM$TurnOffIfInactive(PHY_TRX_OFF);
                {
                  unsigned char __nesc_temp = 
#line 394
                  SUCCESS;

                  {
#line 394
                    __nesc_atomic_end(__nesc_atomic); 
#line 394
                    return __nesc_temp;
                  }
                }
              }
            else 
#line 397
              {
                MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_PHY;
                result = MacCAPM$IPhyTxDATA$Request(MacCAPM$cap[sfIndex].handle, sfIndex);
              }
          }
#line 401
          __nesc_atomic_end(__nesc_atomic); }
        }
#line 402
      if (!result) 
        {


          if (MacCAPM$nextBackoff(sfIndex)) {
            result = MacCAPM$SetBackoff(sfIndex);
            }
#line 408
          if (result) 
            {
              MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_BACKOFF;
            }
          else 
            {
              MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_FAILURE;

              MacCAPM$_SendDone(sfIndex, MAC_CHANNEL_ACCESS_FAILURE);
            }
        }
    }
  return SUCCESS;
}

# 97 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacStartM.nc"
static inline  result_t MacStartM$IPhySET_TRX_STATE$Confirm(TPhyStatus status, TUniData unidata)
{
  return SUCCESS;
}

# 306 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  result_t MacCoordAttrM$IPhySET_TRX_STATE$Confirm(TPhyStatus phyStatus, TUniData uniData)
{
  return SUCCESS;
}

# 894 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline   result_t PhyCC2420M$IPhySET_TRX_STATE$default$Confirm(uint8_t context, TPhyStatus phyStatus, TUniData uniData)
{
#line 895
  return SUCCESS;
}

# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static  result_t PhyCC2420M$IPhySET_TRX_STATE$Confirm(uint8_t arg_0x40b79340, TPhyStatus arg_0x40b47df8, TUniData arg_0x40b46010){
#line 8
  unsigned char result;
#line 8

#line 8
  switch (arg_0x40b79340) {
#line 8
    case 0U:
#line 8
      result = MacSuperframesM$IPhyTrxParent$Confirm(arg_0x40b47df8, arg_0x40b46010);
#line 8
      break;
#line 8
    case 1U:
#line 8
      result = MacSuperframesM$IPhyTrxOwn$Confirm(arg_0x40b47df8, arg_0x40b46010);
#line 8
      break;
#line 8
    case 2U:
#line 8
      result = MacCAPM$IPhySET_TRX_STATE$Confirm(arg_0x40b47df8, arg_0x40b46010);
#line 8
      break;
#line 8
    case 3U:
#line 8
      result = MacStartM$IPhySET_TRX_STATE$Confirm(arg_0x40b47df8, arg_0x40b46010);
#line 8
      break;
#line 8
    case 4U:
#line 8
      result = MacCoordAttrM$IPhySET_TRX_STATE$Confirm(arg_0x40b47df8, arg_0x40b46010);
#line 8
      break;
#line 8
    default:
#line 8
      result = PhyCC2420M$IPhySET_TRX_STATE$default$Confirm(arg_0x40b79340, arg_0x40b47df8, arg_0x40b46010);
#line 8
      break;
#line 8
    }
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 558 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  void PhyCC2420M$SetTRXStateDone(void)
{
  TPhyStatus status = PHY_INVALID_PARAMETER;
  PhyCC2420M$TPhyRadioState rs;

#line 562
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 562
    rs = PhyCC2420M$radioState;
#line 562
    __nesc_atomic_end(__nesc_atomic); }
  switch (rs) {
      case PhyCC2420M$PHY_RADIO_OFF: 
        switch (PhyCC2420M$trx.phyStatus) {
            case PHY_TRX_OFF: case PHY_FORCE_TRX_OFF: 
                status = PHY_TRX_OFF;
            break;
            case PHY_RX_ON: case PHY_TX_ON: 
                PhyCC2420M$turnOnChipcon();
            if (PHY_RX_ON == PhyCC2420M$trx.phyStatus) {
              PhyCC2420M$turnRxOnChipcon();
              }
            else {
#line 574
              PhyCC2420M$turnTxOnChipcon();
              }
#line 575
            status = PHY_SUCCESS;
            break;
            default: break;
          }
      break;
      case PhyCC2420M$PHY_RADIO_IDLE: 
        switch (PhyCC2420M$trx.phyStatus) {
            case PHY_TRX_OFF: case PHY_FORCE_TRX_OFF: 
                PhyCC2420M$turnOffChipcon();
            status = PHY_SUCCESS;
            break;
            case PHY_RX_ON: 
              PhyCC2420M$turnRxOnChipcon();
            status = PHY_SUCCESS;
            break;
            case PHY_TX_ON: 
              PhyCC2420M$turnTxOnChipcon();
            status = PHY_SUCCESS;
            break;
            default: break;
          }
      break;
      case PhyCC2420M$PHY_RADIO_TX_IDLE: 
        switch (PhyCC2420M$trx.phyStatus) {
            case PHY_TRX_OFF: case PHY_FORCE_TRX_OFF: 
                PhyCC2420M$turnOffChipcon();
            status = PHY_SUCCESS;
            break;
            case PHY_RX_ON: 
              PhyCC2420M$turnRxOnChipcon();
            status = PHY_SUCCESS;
            break;
            case PHY_TX_ON: 
              status = PHY_TX_ON;
            break;
            default: break;
          }
      break;
      case PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH: 
        switch (PhyCC2420M$trx.phyStatus) {
            case PHY_TRX_OFF: case PHY_FORCE_TRX_OFF: 
                PhyCC2420M$turnOffChipcon();
            status = PHY_SUCCESS;
            break;
            case PHY_RX_ON: 
              status = PHY_RX_ON;
            break;
            case PHY_TX_ON: 
              PhyCC2420M$turnTxOnChipcon();
            status = PHY_SUCCESS;
            break;
            default: break;
          }
      break;
      default: 
        if (PhyCC2420M$PHY_RADIO_TX_INIT <= rs && rs <= PhyCC2420M$PHY_RADIO_TX_END) {
            switch (PhyCC2420M$trx.phyStatus) {
                case PHY_FORCE_TRX_OFF: 
                  PhyCC2420M$turnOffChipcon();
                status = PHY_SUCCESS;
                break;
                case PHY_TRX_OFF: case PHY_RX_ON: 
                    status = PHY_BUSY_TX;
                break;
                case PHY_TX_ON: 
                  status = PHY_TX_ON;
                break;
                default: break;
              }
          }
        else {
#line 644
          if (PhyCC2420M$PHY_RADIO_RX_BEGIN <= rs && rs <= PhyCC2420M$PHY_RADIO_RX_INDICATION) {
              switch (PhyCC2420M$trx.phyStatus) {
                  case PHY_FORCE_TRX_OFF: 
                    PhyCC2420M$turnOffChipcon();
                  status = PHY_SUCCESS;
                  break;
                  case PHY_TRX_OFF: case PHY_TX_ON: 
                      status = PHY_BUSY_RX;
                  break;
                  case PHY_RX_ON: 
                    status = PHY_RX_ON;
                  break;
                  default: break;
                }
            }
          }
    }
#line 660
  {
#line 660
    uint8_t context = PhyCC2420M$trx.context;
    TUniData uniData = PhyCC2420M$trx.uniData;

#line 662
    { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 662
      PhyCC2420M$trxState = PhyCC2420M$PHY_TRX_FREE;
#line 662
      __nesc_atomic_end(__nesc_atomic); }
    PhyCC2420M$IPhySET_TRX_STATE$Confirm(context, status, uniData);
  }
  return;
}

# 96 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static inline  result_t HPLCC2420M$StdControl$start(void)
#line 96
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 97
    {
      if (! HPLCC2420M$f.busy) {
          TOSH_SET_RADIO_CSN_PIN();
          TOSH_MAKE_RADIO_CSN_OUTPUT();
          HPLCC2420M$USARTControl$setModeSPI();
          HPLCC2420M$USARTControl$disableRxIntr();
          HPLCC2420M$USARTControl$disableTxIntr();
          HPLCC2420M$f.busy = HPLCC2420M$f.rxbufBusy = HPLCC2420M$f.txbufBusy = FALSE;
          HPLCC2420M$f.enabled = TRUE;
        }
    }
#line 107
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 62 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
static inline  result_t BusArbitrationM$StdControl$start(void)
#line 62
{
  uint8_t _state;

#line 64
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 64
    {
      if (BusArbitrationM$state == BusArbitrationM$BUS_OFF) {
          BusArbitrationM$state = BusArbitrationM$BUS_IDLE;
          BusArbitrationM$isBusReleasedPending = FALSE;
        }
      _state = BusArbitrationM$state;
    }
#line 70
    __nesc_atomic_end(__nesc_atomic); }

  if (_state == BusArbitrationM$BUS_IDLE) {
    return SUCCESS;
    }
  return FAIL;
}

# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t PhyCC2420M$HPLCC2420Control$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = BusArbitrationM$StdControl$start();
#line 70
  result = rcombine(result, HPLCC2420M$StdControl$start());
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 64 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_SEL_CC_SFD_IOFUNC(void)
#line 64
{
#line 64
   static volatile uint8_t r __asm ("0x001F");

#line 64
  r &= ~(1 << 1);
}

# 59 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
inline static   result_t PhyCC2420M$IChipconFIFOP$disable(void){
#line 59
  unsigned char result;
#line 59

#line 59
  result = HPLCC2420InterruptM$FIFOP$disable();
#line 59

#line 59
  return result;
#line 59
}
#line 59
inline static   result_t PhyCC2420M$IChipconFIFOSignal$disable(void){
#line 59
  unsigned char result;
#line 59

#line 59
  result = HPLCC2420InterruptM$FIFO$disable();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 70 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_CLR_RADIO_POWER_PIN(void)
#line 70
{
#line 70
   static volatile uint8_t r __asm ("0x001D");

#line 70
  r &= ~(1 << 7);
}

#line 67
static inline void TOSH_SET_CC_VREN_PIN(void)
#line 67
{
#line 67
   static volatile uint8_t r __asm ("0x001D");

#line 67
  r |= 1 << 5;
}

#line 69
static inline void TOSH_CLR_CC_RSTN_PIN(void)
#line 69
{
#line 69
   static volatile uint8_t r __asm ("0x001D");

#line 69
  r &= ~(1 << 6);
}

# 161 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
static __inline void TOSH_wait(void )
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 69 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_SET_CC_RSTN_PIN(void)
#line 69
{
#line 69
   static volatile uint8_t r __asm ("0x001D");

#line 69
  r |= 1 << 6;
}

# 79 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static inline uint8_t HPLCC2420M$adjustStatusByte(uint8_t status)
#line 79
{
  return status & 0x7E;
}

#line 468
static inline  result_t HPLCC2420M$BusArbitration$busFree(void)
#line 468
{
  return SUCCESS;
}

# 125 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
static inline   result_t BusArbitrationM$BusArbitration$default$busFree(uint8_t id)
#line 125
{
  return SUCCESS;
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
inline static  result_t BusArbitrationM$BusArbitration$busFree(uint8_t arg_0x40d9aa88){
#line 39
  unsigned char result;
#line 39

#line 39
  switch (arg_0x40d9aa88) {
#line 39
    case 0U:
#line 39
      result = HPLCC2420M$BusArbitration$busFree();
#line 39
      break;
#line 39
    default:
#line 39
      result = BusArbitrationM$BusArbitration$default$busFree(arg_0x40d9aa88);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 42 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
static inline  void BusArbitrationM$busReleased(void)
#line 42
{
  uint8_t i;
  uint8_t currentstate;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    BusArbitrationM$isBusReleasedPending = FALSE;
#line 46
    __nesc_atomic_end(__nesc_atomic); }
  for (i = 0; i < 1U; i++) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 48
        currentstate = BusArbitrationM$state;
#line 48
        __nesc_atomic_end(__nesc_atomic); }
      if (currentstate == BusArbitrationM$BUS_IDLE) {
        BusArbitrationM$BusArbitration$busFree(i);
        }
    }
}

# 54 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420.nc"
inline static   uint8_t PhyCC2420M$IChipcon$write(uint8_t arg_0x40b9cb78, uint16_t arg_0x40b9cd00){
#line 54
  unsigned char result;
#line 54

#line 54
  result = HPLCC2420M$HPLCC2420$write(arg_0x40b9cb78, arg_0x40b9cd00);
#line 54

#line 54
  return result;
#line 54
}
#line 54







inline static   uint16_t PhyCC2420M$IChipcon$read(uint8_t arg_0x40b9b248){
#line 61
  unsigned int result;
#line 61

#line 61
  result = HPLCC2420M$HPLCC2420$read(arg_0x40b9b248);
#line 61

#line 61
  return result;
#line 61
}
#line 61
# 714 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline result_t PhyCC2420M$SetDefaultRegisters(void)
{
  uint16_t data;

#line 717
  PhyCC2420M$IChipcon$write(CC2420_MAIN_ADDRESS, CC2420_MAIN_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_MDMCTRL0_ADDRESS, CC2420_MDMCTRL0_DEFAULT.raw);
  data = PhyCC2420M$IChipcon$read(CC2420_MDMCTRL0_ADDRESS);
  if (data != CC2420_MDMCTRL0_DEFAULT.raw) {
    return FAIL;
    }
#line 722
  PhyCC2420M$IChipcon$write(CC2420_MDMCTRL1_ADDRESS, CC2420_MDMCTRL1_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_RSSI_ADDRESS, CC2420_RSSI_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_SYNCWORD_ADDRESS, CC2420_SYNCWORD_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_TXCTRL_ADDRESS, CC2420_TXCTRL_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_RXCTRL0_ADDRESS, CC2420_RXCTRL0_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_RXCTRL1_ADDRESS, CC2420_RXCTRL1_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_FSCTRL_ADDRESS, CC2420_FSCTRL_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_SECCTRL0_ADDRESS, CC2420_SECCTRL0_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_SECCTRL1_ADDRESS, CC2420_SECCTRL1_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_BATTMON_ADDRESS, CC2420_BATTMON_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_IOCFG0_ADDRESS, CC2420_IOCFG0_DEFAULT.raw);
  PhyCC2420M$IChipcon$write(CC2420_IOCFG1_ADDRESS, CC2420_IOCFG1_DEFAULT.raw);
  return SUCCESS;
}

# 47 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
inline static   result_t PhyCC2420M$IChipconRAM$write(uint16_t arg_0x40b90638, uint8_t arg_0x40b907b8, uint8_t *arg_0x40b90958){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLCC2420M$HPLCC2420RAM$write(arg_0x40b90638, arg_0x40b907b8, arg_0x40b90958);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 901 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline   result_t PhyCC2420M$IChipconRAM$writeDone(uint16_t addr, uint8_t _length, uint8_t *data)
{
#line 902
  return SUCCESS;
}

# 49 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
inline static   result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t arg_0x40b90e80, uint8_t arg_0x40b8f030, uint8_t *arg_0x40b8f1d0){
#line 49
  unsigned char result;
#line 49

#line 49
  result = PhyCC2420M$IChipconRAM$writeDone(arg_0x40b90e80, arg_0x40b8f030, arg_0x40b8f1d0);
#line 49

#line 49
  return result;
#line 49
}
#line 49
# 288 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static inline  void HPLCC2420M$signalRAMWr(void)
#line 288
{
  HPLCC2420M$HPLCC2420RAM$writeDone(HPLCC2420M$ramaddr, HPLCC2420M$ramlen, HPLCC2420M$rambuf);
}

# 64 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_SEL_CC_SFD_MODFUNC(void)
#line 64
{
#line 64
   static volatile uint8_t r __asm ("0x001F");

#line 64
  r |= 1 << 1;
}

# 110 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline uint16_t MSP430TimerM$captureControl(uint8_t l_cm)
{
  MSP430TimerM$CC_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 1, 
  .ccie = 0 };

  return MSP430TimerM$CC2int(x);
}

#line 388
static inline   void MSP430TimerM$ControlB1$setControlAsCapture(uint8_t cm)
#line 388
{
#line 388
  MSP430TimerM$TBCCTL1 = MSP430TimerM$captureControl(cm);
}

# 36 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void HPLCC2420InterruptM$SFDControl$setControlAsCapture(bool arg_0x407417d0){
#line 36
  MSP430TimerM$ControlB1$setControlAsCapture(arg_0x407417d0);
#line 36
}
#line 36
# 412 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB1$enableEvents(void)
#line 412
{
#line 412
  MSP430TimerM$TBCCTL1 |= 0x0010;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void HPLCC2420InterruptM$SFDControl$enableEvents(void){
#line 38
  MSP430TimerM$ControlB1$enableEvents();
#line 38
}
#line 38
# 43 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
inline static   result_t PhyCC2420M$IChipconFIFOP$startWait(bool arg_0x40b92010){
#line 43
  unsigned char result;
#line 43

#line 43
  result = HPLCC2420InterruptM$FIFOP$startWait(arg_0x40b92010);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 221 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port10$edge(bool l2h)
#line 221
{
  /* atomic removed: atomic calls only */
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
}

# 54 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$edge(bool arg_0x40cd4068){
#line 54
  MSP430InterruptM$Port10$edge(arg_0x40cd4068);
#line 54
}
#line 54
# 115 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port10$enable(void)
#line 115
{
#line 115
  MSP430InterruptM$P1IE |= 1 << 0;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$enable(void){
#line 30
  MSP430InterruptM$Port10$enable();
#line 30
}
#line 30
# 239 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port13$edge(bool l2h)
#line 239
{
  /* atomic removed: atomic calls only */
#line 240
  {
    if (l2h) {
#line 241
      P1IES &= ~(1 << 3);
      }
    else {
#line 242
      P1IES |= 1 << 3;
      }
  }
}

# 54 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOInterrupt$edge(bool arg_0x40cd4068){
#line 54
  MSP430InterruptM$Port13$edge(arg_0x40cd4068);
#line 54
}
#line 54
# 118 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port13$enable(void)
#line 118
{
#line 118
  MSP430InterruptM$P1IE |= 1 << 3;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOInterrupt$enable(void){
#line 30
  MSP430InterruptM$Port13$enable();
#line 30
}
#line 30
# 67 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_CLR_CC_VREN_PIN(void)
#line 67
{
#line 67
   static volatile uint8_t r __asm ("0x001D");

#line 67
  r &= ~(1 << 5);
}

static inline void TOSH_SET_RADIO_POWER_PIN(void)
#line 70
{
#line 70
   static volatile uint8_t r __asm ("0x001D");

#line 70
  r |= 1 << 7;
}

#line 55
static inline void TOSH_SEL_UCLK0_IOFUNC(void)
#line 55
{
#line 55
   static volatile uint8_t r __asm ("0x001B");

#line 55
  r &= ~(1 << 3);
}

#line 54
static inline void TOSH_SEL_SOMI0_IOFUNC(void)
#line 54
{
#line 54
   static volatile uint8_t r __asm ("0x001B");

#line 54
  r &= ~(1 << 2);
}

#line 53
static inline void TOSH_SEL_SIMO0_IOFUNC(void)
#line 53
{
#line 53
   static volatile uint8_t r __asm ("0x001B");

#line 53
  r &= ~(1 << 1);
}

# 205 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline   void HPLUSART0M$USARTControl$disableSPI(void)
#line 205
{
  HPLUSART0M$ME1 &= ~(1 << 6);
  TOSH_SEL_SIMO0_IOFUNC();
  TOSH_SEL_SOMI0_IOFUNC();
  TOSH_SEL_UCLK0_IOFUNC();
}

# 115 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   void HPLCC2420M$USARTControl$disableSPI(void){
#line 115
  HPLUSART0M$USARTControl$disableSPI();
#line 115
}
#line 115
# 111 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static inline  result_t HPLCC2420M$StdControl$stop(void)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    {


      if (! HPLCC2420M$f.busy) {
        HPLCC2420M$USARTControl$disableSPI();
        }
#line 117
      HPLCC2420M$f.enabled = FALSE;
    }
#line 118
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 78 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
static inline  result_t BusArbitrationM$StdControl$stop(void)
#line 78
{
  uint8_t _state;

#line 80
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 80
    {
      if (BusArbitrationM$state == BusArbitrationM$BUS_IDLE) {
          BusArbitrationM$state = BusArbitrationM$BUS_OFF;
          BusArbitrationM$isBusReleasedPending = FALSE;
        }
      _state = BusArbitrationM$state;
    }
#line 86
    __nesc_atomic_end(__nesc_atomic); }

  if (_state == BusArbitrationM$BUS_OFF) {
      return SUCCESS;
    }
  return FAIL;
}

# 78 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t PhyCC2420M$HPLCC2420Control$stop(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = BusArbitrationM$StdControl$stop();
#line 78
  result = rcombine(result, HPLCC2420M$StdControl$stop());
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 10 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacSuperframeOrder MacCAPM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e80a60){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(arg_0x40e80a60);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *MacCAPM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010){
#line 8
  struct __nesc_unnamed4275 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b5c010);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   uint8_t MacCAPM$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b406b8){
#line 9
  unsigned char result;
#line 9

#line 9
  result = PhyFrameM$IPhyFrame$GetPPDULength(arg_0x40b406b8);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 599 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline TSysTime MacCAPM$calcSendTime(uint8_t sfIndex)
{
  uint8_t len;
  TSysTime duration;

#line 603
  len = MacCAPM$IPhyFrame$GetPPDULength(MacCAPM$IPhyPool$GetFrame(MacCAPM$cap[sfIndex].handle));
  duration = len * 4 + 20;
  duration += len <= MAC_AMAX_SIFS_FRAME_SIZE ? MAC_AMIN_SIFS_PERIOD : MAC_AMIN_LIFS_PERIOD;
  return duration;
}

# 17 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static inline   uint8_t PhyFrameM$IPhyFrame$GetMPDULength(const TPhyFrame *const pPhyFrame)
{
  if (pPhyFrame != NULL) {
    return pPhyFrame->data[0];
    }
#line 21
  return 0;
}

# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   uint8_t PhyCC2420M$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b406b8){
#line 9
  unsigned char result;
#line 9

#line 9
  result = PhyFrameM$IPhyFrame$GetPPDULength(arg_0x40b406b8);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 43 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
inline static   result_t PhyCC2420M$IChipconSFD$enableCapture(bool arg_0x40b76d48){
#line 43
  unsigned char result;
#line 43

#line 43
  result = HPLCC2420InterruptM$SFD$enableCapture(arg_0x40b76d48);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 47 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420.nc"
inline static   uint8_t PhyCC2420M$IChipcon$cmd(uint8_t arg_0x40b9c678){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLCC2420M$HPLCC2420$cmd(arg_0x40b9c678);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 60 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
inline static   result_t PhyCC2420M$IChipconSFD$disable(void){
#line 60
  unsigned char result;
#line 60

#line 60
  result = HPLCC2420InterruptM$SFD$disable();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 92 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline void PhyCC2420M$flushTXFIFO(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 94
    PhyCC2420M$radio.txFifoHandle = 0xff;
#line 94
    __nesc_atomic_end(__nesc_atomic); }
  PhyCC2420M$IChipconSFD$disable();
  PhyCC2420M$IChipcon$cmd(0x09);
  PhyCC2420M$IChipcon$cmd(0x09);
  PhyCC2420M$IChipconSFD$enableCapture(FALSE);
}

# 220 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline   void MacSuperframesM$IPhyTxFIFO$WriteDone(TPhyPoolHandle handle, 
result_t result, TUniData uniData)
{
}

# 889 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline    void PhyCC2420M$IPhyTxFIFO$default$WriteDone(uint8_t context, TPhyPoolHandle handle, 
result_t result, TUniData uniData)
#line 890
{
}

# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
inline static   void PhyCC2420M$IPhyTxFIFO$WriteDone(uint8_t arg_0x40b78bf8, TPhyPoolHandle arg_0x40b29960, result_t arg_0x40b29af8, TUniData arg_0x40b29c80){
#line 10
  switch (arg_0x40b78bf8) {
#line 10
    case 0U:
#line 10
      MacSuperframesM$IPhyTxFIFO$WriteDone(arg_0x40b29960, arg_0x40b29af8, arg_0x40b29c80);
#line 10
      break;
#line 10
    default:
#line 10
      PhyCC2420M$IPhyTxFIFO$default$WriteDone(arg_0x40b78bf8, arg_0x40b29960, arg_0x40b29af8, arg_0x40b29c80);
#line 10
      break;
#line 10
    }
#line 10
}
#line 10
# 222 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  void PhyCC2420M$send_done_fail(void)
#line 222
{
#line 222
  PhyCC2420M$SendDone(PHY_TRX_OFF);
}

static inline   result_t PhyCC2420M$IChipconFIFO$TXFIFODone(uint8_t length, uint8_t *data)
{
  PhyCC2420M$TPhyRadioState _radioState;

#line 228
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 228
    {
      _radioState = PhyCC2420M$radioState;
      if (PhyCC2420M$radioState == PhyCC2420M$PHY_RADIO_TX_FIFO_WRITE) {
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_BEGIN;
        }
    }
#line 233
    __nesc_atomic_end(__nesc_atomic); }
#line 233
  if (_radioState == PhyCC2420M$PHY_RADIO_TX_FIFO_WRITE) {
      result_t writeStatus = data == PhyCC2420M$radio.pData && length == PhyCC2420M$radio.dataLength ? 
      SUCCESS : FAIL;

#line 236
      if (PhyCC2420M$radio.txFifoSend) {
          if (SUCCESS == writeStatus && SUCCESS == PhyCC2420M$SendTxFIFO()) {
              PhyCC2420M$radio.txFifoHandle = PhyCC2420M$radio.handle;
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 239
                PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_FRAME_SEND;
#line 239
                __nesc_atomic_end(__nesc_atomic); }
              return SUCCESS;
            }
          TOS_post(PhyCC2420M$send_done_fail);
          return FAIL;
        }
      else 
#line 244
        {
          TUniData uniData = PhyCC2420M$radio.uniData;
          uint8_t context = PhyCC2420M$radio.context;
          TPhyPoolHandle handle = PhyCC2420M$radio.handle;

#line 248
          if (SUCCESS == writeStatus) {
            PhyCC2420M$radio.txFifoHandle = handle;
            }
#line 250
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 250
            PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_IDLE;
#line 250
            __nesc_atomic_end(__nesc_atomic); }
          PhyCC2420M$IPhyTxFIFO$WriteDone(context, writeStatus, handle, uniData);
          return SUCCESS;
        }
    }
  return FAIL;
}

# 50 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
inline static   result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t arg_0x40b97118, uint8_t *arg_0x40b972b8){
#line 50
  unsigned char result;
#line 50

#line 50
  result = PhyCC2420M$IChipconFIFO$TXFIFODone(arg_0x40b97118, arg_0x40b972b8);
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 395 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static inline  void HPLCC2420M$signalTXFIFO(void)
#line 395
{
  uint8_t _txlen;
  uint8_t *_txbuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 399
    {
      _txlen = HPLCC2420M$txlen;
      _txbuf = HPLCC2420M$txbuf;
      HPLCC2420M$f.txbufBusy = FALSE;
    }
#line 403
    __nesc_atomic_end(__nesc_atomic); }

  HPLCC2420M$HPLCC2420FIFO$TXFIFODone(_txlen, _txbuf);
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
inline static   result_t HPLCC2420M$BusArbitration$releaseBus(void){
#line 38
  unsigned char result;
#line 38

#line 38
  result = BusArbitrationM$BusArbitration$releaseBus(0U);
#line 38

#line 38
  return result;
#line 38
}
#line 38
# 432 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline   result_t HPLUSART0M$USARTControl$isTxEmpty(void)
#line 432
{
  if (HPLUSART0M$U0TCTL & 0x01) {
      return SUCCESS;
    }
  return FAIL;
}

# 191 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$isTxEmpty(void){
#line 191
  unsigned char result;
#line 191

#line 191
  result = HPLUSART0M$USARTControl$isTxEmpty();
#line 191

#line 191
  return result;
#line 191
}
#line 191
#line 180
inline static   result_t HPLCC2420M$USARTControl$isTxIntrPending(void){
#line 180
  unsigned char result;
#line 180

#line 180
  result = HPLUSART0M$USARTControl$isTxIntrPending();
#line 180

#line 180
  return result;
#line 180
}
#line 180
# 473 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline   result_t HPLUSART0M$USARTControl$tx(uint8_t data)
#line 473
{
  HPLUSART0M$U0TXBUF = data;
  return SUCCESS;
}

# 202 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$tx(uint8_t arg_0x40c4b760){
#line 202
  unsigned char result;
#line 202

#line 202
  result = HPLUSART0M$USARTControl$tx(arg_0x40c4b760);
#line 202

#line 202
  return result;
#line 202
}
#line 202







inline static   uint8_t HPLCC2420M$USARTControl$rx(void){
#line 209
  unsigned char result;
#line 209

#line 209
  result = HPLUSART0M$USARTControl$rx();
#line 209

#line 209
  return result;
#line 209
}
#line 209
# 65 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_CLR_RADIO_CSN_PIN(void)
#line 65
{
#line 65
   static volatile uint8_t r __asm ("0x001D");

#line 65
  r &= ~(1 << 2);
}

# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
inline static   result_t HPLCC2420M$BusArbitration$getBus(void){
#line 37
  unsigned char result;
#line 37

#line 37
  result = BusArbitrationM$BusArbitration$getBus(0U);
#line 37

#line 37
  return result;
#line 37
}
#line 37
# 416 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static inline   result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t length, uint8_t *data)
#line 416
{
  uint8_t i = 0;
  bool returnFail = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 420
    {
      if (HPLCC2420M$f.txbufBusy) {
        returnFail = TRUE;
        }
      else {
#line 424
        HPLCC2420M$f.txbufBusy = TRUE;
        }
    }
#line 426
    __nesc_atomic_end(__nesc_atomic); }
  if (returnFail) {
    return FAIL;
    }
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 431
        {
          HPLCC2420M$f.busy = TRUE;
          HPLCC2420M$txlen = length;
          HPLCC2420M$txbuf = data;
        }
#line 435
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0x3E);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
      {
#line 442
        register uint8_t *ttxbuf = HPLCC2420M$txbuf;

#line 443
        for (i = HPLCC2420M$txlen; 0 < i; i--) {
            U0TXBUF = * ttxbuf++;
            while (!(IFG1 & (1 << 7))) ;
            IFG1 &= ~(1 << 7);
          }
      }
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isTxEmpty()) ;
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 452
        HPLCC2420M$f.busy = FALSE;
#line 452
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 455
        HPLCC2420M$f.txbufBusy = FALSE;
#line 455
        __nesc_atomic_end(__nesc_atomic); }

      ;

      return FAIL;
    }
  if (TOS_post(HPLCC2420M$signalTXFIFO) == FAIL) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 462
        HPLCC2420M$f.txbufBusy = FALSE;
#line 462
        __nesc_atomic_end(__nesc_atomic); }
      return FAIL;
    }
  return SUCCESS;
}

# 29 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
inline static   result_t PhyCC2420M$IChipconFIFO$writeTXFIFO(uint8_t arg_0x40b98260, uint8_t *arg_0x40b98400){
#line 29
  unsigned char result;
#line 29

#line 29
  result = HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(arg_0x40b98260, arg_0x40b98400);
#line 29

#line 29
  return result;
#line 29
}
#line 29
# 241 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  result_t MacSuperframesM$IPhyTxDATA$Confirm(const TPhyPoolHandle handle, TPhyStatus phyStatus, TUniData unidata)
{




  return SUCCESS;
}

# 452 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  result_t MacCAPM$IPhyTxDATA$Confirm(const TPhyPoolHandle handle, 
TPhyStatus phyStatus, TUniData uniData)
{
  uint8_t sfIndex = (uint8_t )uniData;

  if (MacCAPM$cap[sfIndex].sendState == MacCAPM$CAP_SEND_PHY) {
    MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_PHY_DONE;
    }
  else {
#line 460
    return FAIL;
    }
  if (PHY_SUCCESS == phyStatus) 
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 464
        MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_SUCCESS;
#line 464
        __nesc_atomic_end(__nesc_atomic); }
      MacCAPM$_SendDone(sfIndex, MAC_SUCCESS);
      return SUCCESS;
    }
  else 
    {
      MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_FAILURE;

      MacCAPM$_SendDone(sfIndex, MAC_CHANNEL_ACCESS_FAILURE);
    }
  return FAIL;
}

# 886 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline   result_t PhyCC2420M$IPhyTxDATA$default$Confirm(uint8_t context, TPhyPoolHandle handle, 
TPhyStatus phyStatus, TUniData uniData)
{
#line 888
  return SUCCESS;
}

# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
inline static  result_t PhyCC2420M$IPhyTxDATA$Confirm(uint8_t arg_0x40b4e010, TPhyPoolHandle arg_0x40b2c1d8, TPhyStatus arg_0x40b2c378, TUniData arg_0x40b2c510){
#line 9
  unsigned char result;
#line 9

#line 9
  switch (arg_0x40b4e010) {
#line 9
    case 0U:
#line 9
      result = MacSuperframesM$IPhyTxDATA$Confirm(arg_0x40b2c1d8, arg_0x40b2c378, arg_0x40b2c510);
#line 9
      break;
#line 9
    case 1U:
#line 9
      result = MacCAPM$IPhyTxDATA$Confirm(arg_0x40b2c1d8, arg_0x40b2c378, arg_0x40b2c510);
#line 9
      break;
#line 9
    default:
#line 9
      result = PhyCC2420M$IPhyTxDATA$default$Confirm(arg_0x40b4e010, arg_0x40b2c1d8, arg_0x40b2c378, arg_0x40b2c510);
#line 9
      break;
#line 9
    }
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 63 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static inline   result_t PhyPoolM$IPhyPool$GetUniData(const TPhyPoolHandle handle, 
uint16_t *const pUniData)
{
  if (handle < PhyPoolM$POOL_SIZE) {
      bool allocItem;

#line 68
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 68
        allocItem = ! PhyPoolM$pool[handle].free;
#line 68
        __nesc_atomic_end(__nesc_atomic); }
      if (allocItem && pUniData) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 70
            *pUniData = PhyPoolM$pool[handle].uniData;
#line 70
            __nesc_atomic_end(__nesc_atomic); }
          return SUCCESS;
        }
    }
  return FAIL;
}

# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacCAPM$IPhyPool$GetUniData(const TPhyPoolHandle arg_0x40b5c500, uint16_t *const arg_0x40b5c6f0){
#line 10
  unsigned char result;
#line 10

#line 10
  result = PhyPoolM$IPhyPool$GetUniData(arg_0x40b5c500, arg_0x40b5c6f0);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 42 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   uint32_t TimeCastM$ITimeCast$SymbolsToJiffies(TSysTime symbols)
{
  return symbols >> 1;
}

# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   uint32_t TimerSymbol2M$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a346a0){
#line 15
  unsigned long result;
#line 15

#line 15
  result = TimeCastM$ITimeCast$SymbolsToJiffies(arg_0x40a346a0);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 318 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  result_t TimerSymbol2M$ITimerSymbol$SetOneShot(uint8_t num, TSysTime symbols, TUniData uniData)
{
  return TimerSymbol2M$SetTimer(num, TimerSymbol2M$ITimeCast$SymbolsToJiffies(symbols), FALSE, uniData);
}

# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacCAPM$IBackoff$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(4U, arg_0x4090c6e0, arg_0x4090c868);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 467 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$CompareB0$setEventFromNow(uint16_t x)
#line 467
{
#line 467
  MSP430TimerM$TBCCR0 = TBR + x;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerSymbol2M$AlarmCompare$setEventFromNow(uint16_t arg_0x40754010){
#line 32
  MSP430TimerM$CompareB0$setEventFromNow(arg_0x40754010);
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

# 25 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   TMilliSec TimeCastM$ITimeCast$SymbolsToMillis(TSysTime symbols)
{
  return (TMilliSec )(((uint64_t )symbols << 1) / 125);
}

# 10 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TMilliSec LocalTime64M$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a35558){
#line 10
  unsigned long result;
#line 10

#line 10
  result = TimeCastM$ITimeCast$SymbolsToMillis(arg_0x40a35558);
#line 10

#line 10
  return result;
#line 10
}
#line 10
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

# 23 "../../../zigzag/ZigBee/implementation/NLME_PermitJoiningM.nc"
inline static  void NLME_PermitJoiningM$updateBeacon(void){
#line 23
  NwkBeaconParentM$updateBeacon();
#line 23
}
#line 23
# 139 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static inline  TMacStatus MacAssocCoordM$IMacAssocAttr$SetmacAssociationPermit(bool associationPermit)
{
  MacAssocCoordM$macAssociationPermit = associationPermit;
  return MAC_SUCCESS;
}

# 19 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetmacAssociationPermit(bool arg_0x40eece88){
#line 19
  enum __nesc_unnamed4280 result;
#line 19

#line 19
  result = MacAssocCoordM$IMacAssocAttr$SetmacAssociationPermit(arg_0x40eece88);
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 254 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacAssociationPermit(
bool macAssociationPermit)
{
  return MacCoordAttrM$IMacAssocAttr$SetmacAssociationPermit(macAssociationPermit);
}

# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_PermitJoiningM$IMacSET$SetmacAssociationPermit(bool arg_0x40935d78){
#line 7
  enum __nesc_unnamed4280 result;
#line 7

#line 7
  result = MacCoordAttrM$IMacSET$SetmacAssociationPermit(arg_0x40935d78);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 53 "../../../zigzag/ZigBee/implementation/NLME_PermitJoiningM.nc"
static inline  result_t NLME_PermitJoiningM$ITimerSymbol$Fired(TUniData u)
{
  NLME_PermitJoiningM$IMacSET$SetmacAssociationPermit(FALSE);
  NLME_PermitJoiningM$updateBeacon();
  return SUCCESS;
}

# 141 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
static inline  result_t NLME_JoinParentM$ITimerSymbol$Fired(TUniData u)
{
  NLME_JoinParentM$state = NLME_JoinParentM$IDLE;
  return SUCCESS;
}

# 69 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static inline TMacPoolHandle MacPoolM$ToPoolHandle(MacPoolM$TMacPoolIndex poolIndex)
{
  return (TMacPoolHandle )poolIndex;
}

# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacPoolM$IExpiredTimer$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(3U, arg_0x4090c6e0, arg_0x4090c868);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
inline static   TSysTime MacPoolM$ILocalTime$Read(void){
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
# 79 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static inline void MacPoolM$SearchExpiredItem(void)
{
  uint16_t freeCount = 0;
  TSysTime expiredPeriod = 0;
#line 82
  TSysTime itemExpiredPeriod;
  TSysTime currentTime = MacPoolM$ILocalTime$Read();
  bool setTimer = FALSE;
  MacPoolM$TMacPoolIndex itemIndex;

  for (itemIndex = 0; itemIndex < MacPoolM$MAC_POOL_SIZE; itemIndex++) 
    if (MacPoolM$MAC_POOL_ITEM_ALLOC == MacPoolM$pool[itemIndex].state) {
        if (0 <= (int32_t )(currentTime - MacPoolM$pool[itemIndex].expiredTime)) {
            MacPoolM$IMacPool$Free(MacPoolM$ToPoolHandle(itemIndex), MAC_TRANSACTION_EXPIRED);

            continue;
          }
        else 
#line 93
          {
            itemExpiredPeriod = MacPoolM$pool[itemIndex].expiredTime - currentTime;
            if (setTimer) {
                expiredPeriod = itemExpiredPeriod < expiredPeriod ? 
                itemExpiredPeriod : expiredPeriod;
              }
            else 
#line 98
              {
                setTimer = TRUE;
                expiredPeriod = itemExpiredPeriod;
              }
            continue;
          }
      }
    else {
#line 105
      ++freeCount;
      }
  if (setTimer) {
      if (MacPoolM$IExpiredTimer$IsSet()) {
        MacPoolM$IExpiredTimer$Stop();
        }
#line 110
      MacPoolM$IExpiredTimer$SetOneShot(expiredPeriod, (TUniData )MacPoolM$ToPoolHandle(itemIndex));
    }
  return;
}

static inline  result_t MacPoolM$IExpiredTimer$Fired(TUniData uniData)
{
  MacPoolM$SearchExpiredItem();
  return SUCCESS;
}

# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacCAPM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(2U, arg_0x40b477b8, arg_0x40b47960);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 47 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline uint8_t TOSH_READ_RADIO_CCA_PIN(void)
#line 47
{
#line 47
   static volatile uint8_t r __asm ("0x0020");

#line 47
  return r & (1 << 4);
}

#line 44
static inline uint8_t TOSH_READ_CC_FIFOP_PIN(void)
#line 44
{
#line 44
   static volatile uint8_t r __asm ("0x0020");

#line 44
  return r & (1 << 0);
}

#line 46
static inline uint8_t TOSH_READ_CC_FIFO_PIN(void)
#line 46
{
#line 46
   static volatile uint8_t r __asm ("0x0020");

#line 46
  return r & (1 << 3);
}

# 376 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline   TPhyStatus PhyCC2420M$IPhyCCA$Request(void)
{
  if (!TOSH_READ_CC_FIFO_PIN() && TOSH_READ_CC_FIFOP_PIN()) {
      PhyCC2420M$flushRXFIFO();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 380
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH;
#line 380
        __nesc_atomic_end(__nesc_atomic); }
      return PHY_BUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (PhyCC2420M$PHY_RADIO_OFF == PhyCC2420M$radioState || PhyCC2420M$PHY_RADIO_IDLE == PhyCC2420M$radioState) {
          {
            enum __nesc_unnamed4242 __nesc_temp = 
#line 387
            PHY_TRX_OFF;

            {
#line 387
              __nesc_atomic_end(__nesc_atomic); 
#line 387
              return __nesc_temp;
            }
          }
        }
      else {
#line 388
        if (PhyCC2420M$PHY_RADIO_TX_IDLE <= PhyCC2420M$radioState && PhyCC2420M$radioState <= PhyCC2420M$PHY_RADIO_TX_END) {
            {
              enum __nesc_unnamed4242 __nesc_temp = 
#line 389
              PHY_TX_ON;

              {
#line 389
                __nesc_atomic_end(__nesc_atomic); 
#line 389
                return __nesc_temp;
              }
            }
          }
        else {
#line 390
          if (PhyCC2420M$PHY_RADIO_RX_BEGIN <= PhyCC2420M$radioState && PhyCC2420M$radioState <= PhyCC2420M$PHY_RADIO_RX_INDICATION) {
              {
                enum __nesc_unnamed4242 __nesc_temp = 
#line 391
                PHY_BUSY_RX;

                {
#line 391
                  __nesc_atomic_end(__nesc_atomic); 
#line 391
                  return __nesc_temp;
                }
              }
            }
          else 
#line 392
            {
              uint8_t cca = TOSH_READ_RADIO_CCA_PIN();

#line 394
              if (cca > 0) 
                {
                  enum __nesc_unnamed4242 __nesc_temp = 
#line 395
                  PHY_IDLE;

                  {
#line 395
                    __nesc_atomic_end(__nesc_atomic); 
#line 395
                    return __nesc_temp;
                  }
                }
              else 
#line 397
                {
                  {
                    enum __nesc_unnamed4242 __nesc_temp = 
#line 398
                    PHY_BUSY;

                    {
#line 398
                      __nesc_atomic_end(__nesc_atomic); 
#line 398
                      return __nesc_temp;
                    }
                  }
                }
            }
          }
        }
    }
#line 405
    __nesc_atomic_end(__nesc_atomic); }
}

# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyCCA.nc"
inline static   TPhyStatus MacCAPM$IPhyCCA$Request(void){
#line 6
  enum __nesc_unnamed4242 result;
#line 6

#line 6
  result = PhyCC2420M$IPhyCCA$Request();
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 295 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  result_t MacCAPM$IBackoff$Fired(TUniData uniData)
{
  result_t result;
  TPhyStatus status;
  uint8_t sfIndex = (uint8_t )uniData;





  if (MacCAPM$CAP_SEND_BACKOFF != MacCAPM$cap[sfIndex].sendState || !MacCAPM$enoughTime(sfIndex)) {
    return FAIL;
    }

  {
    if (MacCAPM$cap[sfIndex].state != MacCAPM$CAP_STATE_ACTIVE) {
      return FAIL;
      }

    status = MacCAPM$IPhyCCA$Request();
  }




  if (status == PHY_IDLE) {
    { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
      {
        if (MacCAPM$CAP_SEND_BACKOFF != MacCAPM$cap[sfIndex].sendState || MacCAPM$cap[sfIndex].state != MacCAPM$CAP_STATE_ACTIVE) 
          {
            unsigned char __nesc_temp = 
#line 324
            FAIL;

            {
#line 324
              __nesc_atomic_end(__nesc_atomic); 
#line 324
              return __nesc_temp;
            }
          }
#line 325
        result = MacCAPM$IPhySET_TRX_STATE$Request(PHY_TX_ON, sfIndex);

        if (SUCCESS == result) 
          {
            MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_TX_ON;
            {
              unsigned char __nesc_temp = 
#line 330
              SUCCESS;

              {
#line 330
                __nesc_atomic_end(__nesc_atomic); 
#line 330
                return __nesc_temp;
              }
            }
          }
      }
#line 334
      __nesc_atomic_end(__nesc_atomic); }
    }
#line 334
  {


    result = FAIL;
    if (MacCAPM$nextBackoff(sfIndex)) 
      {
        result = MacCAPM$SetBackoff(sfIndex);
      }
    if (FAIL == result) 
      {
        MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_FAILURE;
        ;
        MacCAPM$_SendDone(sfIndex, MAC_CHANNEL_ACCESS_FAILURE);
      }
  }
  return SUCCESS;
}

# 148 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline  result_t MacDataM$AckWaitTimer$Fired(TUniData uniData)
{


  if (MacDataM$state == MacDataM$WAIT_ACK) 
    {
      if (MacDataM$st_TimesRetrLeft > 0) 
        {
          MacDataM$st_TimesRetrLeft--;

          if (MacDataM$Send(&MacDataM$st_msg, MacDataM$st_msduHandle)) 
            {
              MacDataM$state = MacDataM$WAIT_SEND_DONE;
            }
          else 
            {

              MacDataM$state = MacDataM$IDLE;
              MacDataM$Confirm(MacDataM$currentSf, MacDataM$st_msduHandle, MAC_CHANNEL_ACCESS_FAILURE);
            }
        }
      else 

        {
          MacDataM$state = MacDataM$IDLE;
          MacDataM$Confirm(MacDataM$currentSf, MacDataM$st_msduHandle, MAC_NO_ACK);
        }
    }

  return SUCCESS;
}

# 17 "../../../zigzag/ZigBee/extra/ChildSupervisorM.nc"
inline static  void ChildSupervisorM$updateBeacon(void){
#line 17
  NwkBeaconParentM$updateBeacon();
#line 17
}
#line 17
# 137 "../../../zigzag/ZigBee/implementation/NeighborTableM.nc"
static inline  result_t NeighborTableM$NeighborTable$remove(NwkNeighbor *pneighbor)
{
  {
#line 139
    pneighbor[0].networkAddr = 0xFFFF;
#line 139
    pneighbor[0].extendedAddr = 0xFFFFFFFFFFFFFFFFLL;
  }
#line 139
  ;
  return SUCCESS;
}

# 32 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t ChildSupervisorM$NeighborTable$remove(NwkNeighbor *arg_0x408dd580){
#line 32
  unsigned char result;
#line 32

#line 32
  result = NeighborTableM$NeighborTable$remove(arg_0x408dd580);
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 85 "../../../zigzag/ZigCoordM.nc"
static inline  void ZigCoordM$NLME_Leave$indication(IEEEAddr DeviceAddr)
#line 85
{
}

# 30 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static inline  uint64_t LocalTime64M$ILocalTime64$getLocalTime(void)
{
  return LocalTime64M$ILocalTime64$getLocalTimeAt(LocalTime64M$ILocalTime$Read());
}

# 8 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
inline static  uint64_t ZigNet2SpiM$ILocalTime64$getLocalTime(void){
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
# 112 "../../../zigzag/ZigNet2SpiM.nc"
static inline  void ZigNet2SpiM$IZigLeave$indication(IEEEAddr deviceAddr)
{

  uint8_t leave_msg[15];

  ZigNet2SpiM$workaround = &deviceAddr;
  serialize(leave_msg, 15, "1:1:1:1:8:1:2:", 
  (uint8_t )0x00, (uint8_t )0x00, (uint8_t )0x04, (uint8_t )(15 - 4), 
  ZigNet2SpiM$ILocalTime64$getLocalTime(), (uint8_t )0x21, 
  (uint16_t )0x1);

  ZigNet2SpiM$usartTx(0xfffe, deviceAddr, 15, leave_msg);
  return;
}

# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
inline static  void ChildSupervisorM$NLME_Leave$indication(IEEEAddr arg_0x408d6548){
#line 21
  ZigNet2SpiM$IZigLeave$indication(arg_0x408d6548);
#line 21
  ZigCoordM$NLME_Leave$indication(arg_0x408d6548);
#line 21
}
#line 21
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
inline static   TSysTime ChildSupervisorM$ILocalTime$Read(void){
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
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t ChildSupervisorM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x408dea50){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x408dea50);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 43 "../../../zigzag/ZigBee/extra/ChildSupervisorM.nc"
static inline  result_t ChildSupervisorM$ITimerSymbol$Fired(TUniData u)
{
  NwkNeighbor *pneighbor;


  ;
  pneighbor = NULL;
  while (ChildSupervisorM$NeighborTable$getNextPtr(&pneighbor) == SUCCESS) 
    {
      if (pneighbor->relationship == NWK_CHILD) 
        {

          if (ChildSupervisorM$ILocalTime$Read() - pneighbor->lastFrameTime > ChildSupervisorM$table_check_period) 
            {


              ;
              ;
              ChildSupervisorM$NLME_Leave$indication(pneighbor->extendedAddr);
              ChildSupervisorM$NeighborTable$remove(pneighbor);

              ChildSupervisorM$updateBeacon();
            }
        }
    }

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
inline static  result_t TimerSymbol2M$ITimerSymbol$Fired(uint8_t arg_0x40a667c0, TUniData arg_0x4090ac28){
#line 16
  unsigned char result;
#line 16

#line 16
  switch (arg_0x40a667c0) {
#line 16
    case 0U:
#line 16
      result = NLME_JoinParentM$ITimerSymbol$Fired(arg_0x4090ac28);
#line 16
      break;
#line 16
    case 1U:
#line 16
      result = NLME_PermitJoiningM$ITimerSymbol$Fired(arg_0x4090ac28);
#line 16
      break;
#line 16
    case 2U:
#line 16
      result = LocalTime64M$ITimerSymbol$Fired(arg_0x4090ac28);
#line 16
      break;
#line 16
    case 3U:
#line 16
      result = MacPoolM$IExpiredTimer$Fired(arg_0x4090ac28);
#line 16
      break;
#line 16
    case 4U:
#line 16
      result = MacCAPM$IBackoff$Fired(arg_0x4090ac28);
#line 16
      break;
#line 16
    case 5U:
#line 16
      result = MacDataM$AckWaitTimer$Fired(arg_0x4090ac28);
#line 16
      break;
#line 16
    case 6U:
#line 16
      result = ChildSupervisorM$ITimerSymbol$Fired(arg_0x4090ac28);
#line 16
      break;
#line 16
    default:
#line 16
      result = TimerSymbol2M$ITimerSymbol$default$Fired(arg_0x40a667c0, arg_0x4090ac28);
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
# 399 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline   result_t TimerSymbol2M$TimerMilli$default$fired(uint8_t num)
#line 399
{
#line 399
  return SUCCESS;
}

# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
inline static  result_t TimerSymbol2M$TimerMilli$fired(uint8_t arg_0x40a643d0){
#line 37
  unsigned char result;
#line 37

#line 37
    result = TimerSymbol2M$TimerMilli$default$fired(arg_0x40a643d0);
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

# 77 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_SET_SATELLITE_INTR_PIN(void)
#line 77
{
#line 77
   static volatile uint8_t r __asm ("0x0031");

#line 77
  r |= 1 << 4;
}

#line 78
static inline uint8_t TOSH_READ_SATELLITE_INTA_PIN(void)
#line 78
{
#line 78
   static volatile uint8_t r __asm ("0x0030");

#line 78
  return r & (1 << 5);
}



static inline void TOSH_TOGGLE_LED0_PIN(void)
#line 83
{
#line 83
   static volatile uint8_t r __asm ("0x0035");

#line 83
  r ^= 1 << 0;
}

# 153 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static inline   result_t UniSART1M$USARTControl$isTxEmpty(void)
#line 153
{
  if (UniSART1M$U1TCTL & 0x01) {
      return SUCCESS;
    }
  return FAIL;
}

# 92 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   result_t ZigNet2SpiM$USARTControl$isTxEmpty(void){
#line 92
  unsigned char result;
#line 92

#line 92
  result = UniSART1M$USARTControl$isTxEmpty();
#line 92

#line 92
  return result;
#line 92
}
#line 92
# 85 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_TOGGLE_LED2_PIN(void)
#line 85
{
#line 85
   static volatile uint8_t r __asm ("0x0035");

#line 85
  r ^= 1 << 2;
}

# 133 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static inline  bool MacAssocCoordM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 137
  return MacAssocCoordM$macAssociationPermit;
}

# 18 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  bool MacCoordAttrM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40eec9e8){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacAssocCoordM$IMacAssocAttr$GetmacAssociationPermit(arg_0x40eec9e8);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 249 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  bool MacCoordAttrM$IMacGET$GetmacAssociationPermit(
TMacStatus *const pMacStatus)
{
  return MacCoordAttrM$IMacAssocAttr$GetmacAssociationPermit(pMacStatus);
}

# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  bool NwkBeaconParentM$IMacGET$GetmacAssociationPermit(TMacStatus *const arg_0x409026d0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = MacCoordAttrM$IMacGET$GetmacAssociationPermit(arg_0x409026d0);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 123 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  uint8_t NIBM$NIB$getNwkMaxRouters(void)
{
  return NIBM$nwkMaxRouters;
}

# 66 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NwkAddressingM$NIB$getNwkMaxRouters(void){
#line 66
  unsigned char result;
#line 66

#line 66
  result = NIBM$NIB$getNwkMaxRouters();
#line 66

#line 66
  return result;
#line 66
}
#line 66
# 180 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static inline  bool NwkAddressingM$NwkAddressing$hasVacantRouterAddrs(void)
{
  uint8_t routers;
  uint8_t eds;
  result_t result;

#line 185
  NwkAddressingM$countChildren(&routers, &eds);
  result = NwkAddressingM$cskip != 0 && routers < NwkAddressingM$NIB$getNwkMaxRouters();
  return result;
}

# 21 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
inline static  bool NwkBeaconParentM$NwkAddressing$hasVacantRouterAddrs(void){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NwkAddressingM$NwkAddressing$hasVacantRouterAddrs();
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t NwkAddressingM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x408dea50){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x408dea50);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 103 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  uint8_t NIBM$NIB$getNwkMaxChildren(void)
{
  return NIBM$nwkMaxChildren;
}

# 48 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NwkAddressingM$NIB$getNwkMaxChildren(void){
#line 48
  unsigned char result;
#line 48

#line 48
  result = NIBM$NIB$getNwkMaxChildren();
#line 48

#line 48
  return result;
#line 48
}
#line 48
# 172 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static inline  bool NwkAddressingM$NwkAddressing$hasVacantEdAddrs(void)
{
  uint8_t routers;
  uint8_t eds;

#line 176
  NwkAddressingM$countChildren(&routers, &eds);
  return NwkAddressingM$cskip != 0 && eds < NwkAddressingM$NIB$getNwkMaxChildren() - NwkAddressingM$NIB$getNwkMaxRouters();
}

# 20 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
inline static  bool NwkBeaconParentM$NwkAddressing$hasVacantEdAddrs(void){
#line 20
  unsigned char result;
#line 20

#line 20
  result = NwkAddressingM$NwkAddressing$hasVacantEdAddrs();
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 274 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  uint8_t NIBM$NIB$getDepth(void)
#line 274
{
#line 274
  return NIBM$depth;
}

# 196 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NwkBeaconParentM$NIB$getDepth(void){
#line 196
  unsigned char result;
#line 196

#line 196
  result = NIBM$NIB$getDepth();
#line 196

#line 196
  return result;
#line 196
}
#line 196
# 286 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  TSysTime NIBM$NIB$getBeaconOffset(void)
#line 286
{
#line 286
  return NIBM$beaconOffset;
}

# 205 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  TSysTime NwkBeaconParentM$NIB$getBeaconOffset(void){
#line 205
  unsigned long result;
#line 205

#line 205
  result = NIBM$NIB$getBeaconOffset();
#line 205

#line 205
  return result;
#line 205
}
#line 205
# 16 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetmacBeaconPayload(uint8_t *arg_0x40ec5ab0, uint8_t arg_0x40ec5c50){
#line 16
  enum __nesc_unnamed4280 result;
#line 16

#line 16
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBeaconPayload(arg_0x40ec5ab0, arg_0x40ec5c50);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 187 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacCoordAttrM$IMacSET$SetmacBeaconPayload(
TMacBeaconPayload macBeaconPayload, 
TMacBeaconPayloadLength macBeaconPayloadLength)
{
  return MacCoordAttrM$IMacBeaconAttr$SetmacBeaconPayload(
  macBeaconPayload, macBeaconPayloadLength);
}

# 15 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NwkBeaconParentM$IMacSET$SetmacBeaconPayload(TMacBeaconPayload arg_0x40932030, TMacBeaconPayloadLength arg_0x409321e0){
#line 15
  enum __nesc_unnamed4280 result;
#line 15

#line 15
  result = MacCoordAttrM$IMacSET$SetmacBeaconPayload(arg_0x40932030, arg_0x409321e0);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 405 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline   result_t MacDataM$IMacSendParent$default$Send(const TMacFrame *const frame, const TUniData unidata)
{
  return FAIL;
}

# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  result_t MacDataM$IMacSendParent$Send(const TMacFrame *const arg_0x40ef9010, const TUniData arg_0x40ef91c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = MacDataM$IMacSendParent$default$Send(arg_0x40ef9010, arg_0x40ef91c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 16 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
inline static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Send(const uint8_t arg_0x40f86c88, const TMacFrame *const arg_0x40f86ea8, const uint8_t arg_0x40f85088, const TUniData arg_0x40f85240){
#line 16
  unsigned char result;
#line 16

#line 16
  result = MacCAPM$Send(arg_0x40f86c88, arg_0x40f86ea8, arg_0x40f85088, arg_0x40f85240);
#line 16

#line 16
  return result;
#line 16
}
#line 16
#line 30
static inline  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(uint8_t context, const TMacFrame *const pMacFrame, 
const TUniData uniData)
{

  return /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Send(0, pMacFrame, context, uniData);
}

# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  result_t MacDataM$IMacSendOwn$Send(const TMacFrame *const arg_0x40ef9010, const TUniData arg_0x40ef91c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(2U, arg_0x40ef9010, arg_0x40ef91c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 44 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  TMacRawFrameLength MacCAPM$IMacFrame$Pack(const TMacFrame *const arg_0x40dd1c38, TPhyFrame *const arg_0x40dd1e28){
#line 44
  unsigned char result;
#line 44

#line 44
  result = MacFrameFormatM$IMacFrame$Pack(arg_0x40dd1c38, arg_0x40dd1e28);
#line 44

#line 44
  return result;
#line 44
}
#line 44
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacCAPM$IPhyPool$Alloc(const uint8_t arg_0x40b5e6b8, const uint16_t arg_0x40b5e860, TPhyPoolHandle *const arg_0x40b5ea58){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyPoolM$IPhyPool$Alloc(arg_0x40b5e6b8, arg_0x40b5e860, arg_0x40b5ea58);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 554 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline result_t MacCAPM$PutToPhyPool(const TMacFrame *const pMacFrame, 
const TUniData uniData, 
TPhyPoolHandle *const pPhyPoolHandle)
{
  if (SUCCESS == MacCAPM$IPhyPool$Alloc(PHY_AMAX_PHY_PACKET_SIZE, uniData, pPhyPoolHandle)) {
      TPhyFrame *pPhyFrame = MacCAPM$IPhyPool$GetFrame(*pPhyPoolHandle);

#line 560
      if (NULL != pPhyFrame) {
          if (SUCCESS == MacCAPM$IMacFrame$Pack(pMacFrame, pPhyFrame)) {
            return SUCCESS;
            }
        }
#line 564
      MacCAPM$IPhyPool$Free(*pPhyPoolHandle);
    }
  return FAIL;
}

# 19 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   result_t PhyPoolM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b3d418){
#line 19
  unsigned char result;
#line 19

#line 19
  result = PhyFrameM$IPhyFrame$Erase(arg_0x40b3d418);
#line 19

#line 19
  return result;
#line 19
}
#line 19
inline static   result_t MacFrameFormatM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b3d418){
#line 19
  unsigned char result;
#line 19

#line 19
  result = PhyFrameM$IPhyFrame$Erase(arg_0x40b3d418);
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 289 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static inline  result_t MacFrameFormatM$IMacFrame$GetTimeStamp(const TMacFrame *const pMacFrame, 
TMacTimeStamp *const pTimeStamp)
{
  if (pMacFrame != NULL && pTimeStamp != NULL) {
      *pTimeStamp = pMacFrame->timeStamp;
      return SUCCESS;
    }
  return FAIL;
}

# 25 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   result_t MacFrameFormatM$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b3c3c8, TPhyTimeStamp arg_0x40b3c550){
#line 25
  unsigned char result;
#line 25

#line 25
  result = PhyFrameM$IPhyFrame$SetTimeStamp(arg_0x40b3c3c8, arg_0x40b3c550);
#line 25

#line 25
  return result;
#line 25
}
#line 25
#line 13
inline static   result_t MacFrameFormatM$IPhyFrame$AppendOctet(TPhyFrame *const arg_0x40b3f118, const uint8_t arg_0x40b3f2b8){
#line 13
  unsigned char result;
#line 13

#line 13
  result = PhyFrameM$IPhyFrame$AppendOctet(arg_0x40b3f118, arg_0x40b3f2b8);
#line 13

#line 13
  return result;
#line 13
}
#line 13
# 80 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static inline   result_t PhyFrameM$IPhyFrame$CalcCRC(TPhyFrame *const pPhyFrame)
{
  if (pPhyFrame != NULL) {
      pPhyFrame->data[0] += 2;
      return SUCCESS;
    }
  return FAIL;
}

# 21 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   result_t MacFrameFormatM$IPhyFrame$CalcCRC(TPhyFrame *const arg_0x40b3d928){
#line 21
  unsigned char result;
#line 21

#line 21
  result = PhyFrameM$IPhyFrame$CalcCRC(arg_0x40b3d928);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 393 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline   result_t MacDataM$IMacDataParent$default$Confirm(uint8_t msdu_L_Handle, TMacStatus MacStatus)
{
#line 394
  return SUCCESS;
}

# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t MacDataM$IMacDataParent$Confirm(uint8_t arg_0x409e1a00, TMacStatus arg_0x409e1b90){
#line 26
  unsigned char result;
#line 26

#line 26
  result = MacDataM$IMacDataParent$default$Confirm(arg_0x409e1a00, arg_0x409e1b90);
#line 26

#line 26
  return result;
#line 26
}
#line 26
# 223 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline result_t RouterM$IMacDATA_Confirm(uint8_t msduHandle, TMacStatus status)
{

  if (status == MAC_TRANSACTION_OVERFLOW) 
    {

      return SUCCESS;
    }

  if (!RouterM$wait_confirm) {
    return SUCCESS;
    }
  RouterM$wait_confirm = FALSE;

  if (RouterM$reportErrorWhenGetFree) 
    {
      RouterM$NLDE_Data$confirm(RouterM$reportErrorNsduHandle, MAC_TRANSACTION_OVERFLOW);
      RouterM$reportErrorWhenGetFree = FALSE;
    }

  if (msduHandle == RouterM$current_msduHandle) 
    {


      if (!RouterM$relaying) 
        {
          RouterM$NLDE_Data$confirm(RouterM$current_nsduHandle, status);
        }
      else 
        {

          if (status == MAC_SUCCESS) {
            RouterM$frames[RouterM$cur_frame].state = RouterM$NWK_FRAME_NULL;
            }
        }









      RouterM$cur_frame++;
      if (RouterM$cur_frame == RouterM$NWK_FRAME_BUFFER_SIZE) {
        RouterM$cur_frame = 0;
        }
      TOS_post(RouterM$relayNext);
    }
  else 
    {

      if (!RouterM$relaying) {
        RouterM$NLDE_Data$confirm(RouterM$current_nsduHandle, MAC_TRANSACTION_OVERFLOW);
        }
      else {
          TOS_post(RouterM$relayNext);
        }
    }



  return SUCCESS;
}

#line 494
static inline  result_t RouterM$LowerIf$Confirm(uint8_t msduHandle, TMacStatus status)
{
  return RouterM$IMacDATA_Confirm(msduHandle, status);
}

# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t MacDataM$IMacDataOwn$Confirm(uint8_t arg_0x409e1a00, TMacStatus arg_0x409e1b90){
#line 26
  unsigned char result;
#line 26

#line 26
  result = RouterM$LowerIf$Confirm(arg_0x409e1a00, arg_0x409e1b90);
#line 26

#line 26
  return result;
#line 26
}
#line 26
# 313 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  result_t TimerSymbol2M$ITimerSymbol$SetPeriodic(uint8_t num, TSysTime symbols, TUniData uniData)
{
  return TimerSymbol2M$SetTimer(num, TimerSymbol2M$ITimeCast$SymbolsToJiffies(symbols), TRUE, uniData);
}

# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t LocalTime64M$ITimerSymbol$SetPeriodic(TSysTime arg_0x4090c0c0, TUniData arg_0x4090c248){
#line 6
  unsigned char result;
#line 6

#line 6
  result = TimerSymbol2M$ITimerSymbol$SetPeriodic(2U, arg_0x4090c0c0, arg_0x4090c248);
#line 6

#line 6
  return result;
#line 6
}
#line 6



inline static  result_t LocalTime64M$ITimerSymbol$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(2U);
#line 9

#line 9
  return result;
#line 9
}
#line 9


inline static  bool LocalTime64M$ITimerSymbol$IsSet(void){
#line 11
  unsigned char result;
#line 11

#line 11
  result = TimerSymbol2M$ITimerSymbol$IsSet(2U);
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

static inline  void LocalTime64M$ILocalTime64$setLocalTime(uint64_t time)
{
  LocalTime64M$ILocalTime64$setLocalTimeAt(LocalTime64M$ILocalTime$Read(), time);
}

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
inline static  void ZigNet2SpiM$ILocalTime64$setLocalTime(uint64_t arg_0x40a39da0){
#line 9
  LocalTime64M$ILocalTime64$setLocalTime(arg_0x40a39da0);
#line 9
}
#line 9
# 69 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setNwkSequenceNumber(uint8_t attr)
{
  NIBM$nwkSequenceNumber = attr;
}

# 19 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NwkFrameM$NIB$setNwkSequenceNumber(uint8_t arg_0x408ca478){
#line 19
  NIBM$NIB$setNwkSequenceNumber(arg_0x408ca478);
#line 19
}
#line 19
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacShortAddress MacCoordAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dc99a8){
#line 10
  unsigned int result;
#line 10

#line 10
  result = MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(arg_0x40dc99a8);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 12 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacShortAddress MacCoordAttrM$IMacGET$GetmacShortAddress(TMacStatus *const pMacStatus)
{
  return MacCoordAttrM$IMacCommonAttr$GetmacShortAddress(pMacStatus);
}

# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacShortAddress NwkFrameM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408f9440){
#line 43
  unsigned int result;
#line 43

#line 43
  result = MacCoordAttrM$IMacGET$GetmacShortAddress(arg_0x408f9440);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 73 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  uint8_t NIBM$NIB$getNwkSequenceNumber(void)
{
  return NIBM$nwkSequenceNumber;
}

# 20 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NwkFrameM$NIB$getNwkSequenceNumber(void){
#line 20
  unsigned char result;
#line 20

#line 20
  result = NIBM$NIB$getNwkSequenceNumber();
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 15 "../../../zigzag/ZigBee/implementation/NwkFrameM.nc"
static inline  uint8_t NwkFrameM$NwkFrame$mkDataFrameHeader(uint8_t *msdu, 
NwkAddr dstAddr, 
uint8_t radius, 
uint8_t discoverRoute, 
bool securityEnable)

{
  uint8_t sequenceNumber = NwkFrameM$NIB$getNwkSequenceNumber();
  NwkHeader *const header = (NwkHeader *)msdu;


  msdu[0] = 0;
  msdu[1] = 0;

  header->frameType = NWK_DATA_FRAME;
  header->protocolVersion = NWK_PROTOCOL_VERSION;
  header->discoverRoute = discoverRoute;
  header->security = securityEnable ? 1 : 0;
  header->destinationAddr = dstAddr;
  header->sourceAddr = NwkFrameM$IMacGET$GetmacShortAddress(NULL);
  header->radius = radius;
  header->sequenceNumber = sequenceNumber;
  NwkFrameM$NIB$setNwkSequenceNumber(sequenceNumber + 1);
  header->srcIEEEAddr = 1;

  header->sourceExtAddr = info_param.MAC_ADDRESS;

  return sizeof(NwkHeader );
}

# 7 "../../../zigzag/ZigBee/implementation/NwkFrame.nc"
inline static  uint8_t RouterM$NwkFrame$mkDataFrameHeader(uint8_t *arg_0x409fc010, NwkAddr arg_0x409fc1a0, uint8_t arg_0x409fc330, uint8_t arg_0x409fc4c8, bool arg_0x409fc670){
#line 7
  unsigned char result;
#line 7

#line 7
  result = NwkFrameM$NwkFrame$mkDataFrameHeader(arg_0x409fc010, arg_0x409fc1a0, arg_0x409fc330, arg_0x409fc4c8, arg_0x409fc670);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 272 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  bool NIBM$NIB$isJoined(void)
#line 272
{
#line 272
  return NIBM$onlineStatus != NWK_OFFLINE;
}

# 193 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  bool RouterM$NIB$isJoined(void){
#line 193
  unsigned char result;
#line 193

#line 193
  result = NIBM$NIB$isJoined();
#line 193

#line 193
  return result;
#line 193
}
#line 193
# 205 "../../../zigzag/ZigNet2SpiM.nc"
static inline  void ZigNet2SpiM$IZigDATA$indication(NwkAddr srcAddr, IEEEAddr srcExtAddr, uint8_t nsduLength, 
uint8_t *nsdu, uint8_t linkQuality)
{
  ZigNet2SpiM$workaround = &srcExtAddr;
  ZigNet2SpiM$usartTx(srcAddr, srcExtAddr, nsduLength, nsdu);
  return;
}

# 39 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
inline static  void RouterM$NLDE_Data$indication(NwkAddr arg_0x408e4e20, IEEEAddr arg_0x408e2010, uint8_t arg_0x408e21a8, uint8_t *arg_0x408e2358, uint8_t arg_0x408e24f0){
#line 39
  ZigNet2SpiM$IZigDATA$indication(arg_0x408e4e20, arg_0x408e2010, arg_0x408e21a8, arg_0x408e2358, arg_0x408e24f0);
#line 39
}
#line 39
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacShortAddress RouterM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408f9440){
#line 43
  unsigned int result;
#line 43

#line 43
  result = MacCoordAttrM$IMacGET$GetmacShortAddress(arg_0x408f9440);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 83 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline  result_t RouterM$NLDE_Data$request(
NwkAddr dstAddr, 
uint8_t nsduLength, 
uint8_t *nsdu, 
uint8_t nsduHandle, 
uint8_t radius, 
uint8_t discoverRoute, 
bool securityEnable)

{
  RouterM$FrameEntry frame_entry;
  NwkAddr myAddr;

  myAddr = RouterM$IMacGET$GetmacShortAddress(NULL);


  if (dstAddr == myAddr) 
    {

      RouterM$NLDE_Data$indication(
      myAddr, 
      info_param.MAC_ADDRESS, 
      nsduLength, 
      nsdu, 
      0xff);
      RouterM$NLDE_Data$confirm(nsduHandle, NWK_SUCCESS);

      return SUCCESS;
    }

  if (RouterM$wait_confirm) 
    {



      if (RouterM$reportErrorWhenGetFree) 
        {

          ;
          return FAIL;
        }
      else 
        {
          RouterM$reportErrorWhenGetFree = TRUE;
          RouterM$reportErrorNsduHandle = nsduHandle;
        }
      return SUCCESS;
    }

  if (!RouterM$NIB$isJoined() || radius == 0) 
    {
      RouterM$NLDE_Data$confirm(nsduHandle, NWK_INVALID_REQUEST);
      return SUCCESS;
    }


  {
    uint8_t header_size;

    header_size = RouterM$NwkFrame$mkDataFrameHeader(frame_entry.msdu, dstAddr, radius, discoverRoute, securityEnable);

    RouterM$current_nsduHandle = nsduHandle;
    RouterM$relaying = FALSE;
    nmemcpy(frame_entry.msdu + header_size, nsdu, nsduLength);

    if (FAIL == RouterM$sendFrame(frame_entry.msdu, nsduLength + header_size)) {
      RouterM$NLDE_Data$confirm(nsduHandle, NWK_INVALID_REQUEST);
      }
#line 150
    return SUCCESS;
  }
}

# 23 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
inline static  result_t ZigNet2SpiM$IZigDATA$request(NwkAddr arg_0x408e5938, uint8_t arg_0x408e5ad0, uint8_t *arg_0x408e5c80, uint8_t arg_0x408e5e18, uint8_t arg_0x408e4010, uint8_t arg_0x408e41a8, bool arg_0x408e4340){
#line 23
  unsigned char result;
#line 23

#line 23
  result = RouterM$NLDE_Data$request(arg_0x408e5938, arg_0x408e5ad0, arg_0x408e5c80, arg_0x408e5e18, arg_0x408e4010, arg_0x408e41a8, arg_0x408e4340);
#line 23

#line 23
  return result;
#line 23
}
#line 23
# 79 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline uint8_t TOSH_READ_SATELLITE_INTA_2_PIN(void)
#line 79
{
#line 79
   static volatile uint8_t r __asm ("0x0030");

#line 79
  return r & (1 << 6);
}

#line 80
static inline uint8_t TOSH_READ_JOIN_PIN(void)
#line 80
{
#line 80
   static volatile uint8_t r __asm ("0x0030");

#line 80
  return r & (1 << 7);
}

# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacShortAddress ZigNet2SpiM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408f9440){
#line 43
  unsigned int result;
#line 43

#line 43
  result = MacCoordAttrM$IMacGET$GetmacShortAddress(arg_0x408f9440);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 40 "../../../zigzag/ZigNet2SpiM.nc"
static inline  void ZigNet2SpiM$taskSendEvents(void)
{
  TMacShortAddress shortAddress = ZigNet2SpiM$IMacGET$GetmacShortAddress(0);

#line 43
  if (0x0000 == shortAddress || TOSH_READ_JOIN_PIN()) {
      if (TOSH_READ_SATELLITE_INTA_2_PIN()) {
          if (ZigNet2SpiM$usartDstAddr != 0xfffe) {
              ZigNet2SpiM$IZigDATA$request(ZigNet2SpiM$usartDstAddr, ZigNet2SpiM$usartBufSize, ZigNet2SpiM$usartBuf, 
              0x20, 0xff, 0x0, FALSE);
              return;
            }
          else 
#line 49
            {
              uint8_t msg_type;
#line 50
              uint8_t ev_type;
              uint64_t localtime;

              deserialize(ZigNet2SpiM$usartBuf, ZigNet2SpiM$usartBufSize, "1:1:1:1:8:1:", 0, 0, &msg_type, 0, &localtime, &ev_type);
              if (0x04 == msg_type && ev_type == 0xff) {
                  ZigNet2SpiM$ILocalTime64$setLocalTime(localtime);
                }
            }
        }
    }

  TOSH_CLR_SATELLITE_INTA_2_PIN();
  ZigNet2SpiM$sending = FALSE;
  return;
}

# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacCoordAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dcacf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dcacf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 2 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacPANId MacCoordAttrM$IMacGET$GetmacPANId(TMacStatus *const pMacStatus)
{
  return MacCoordAttrM$IMacCommonAttr$GetmacPANId(pMacStatus);
}

# 37 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacPANId RouterM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408fb528){
#line 37
  unsigned int result;
#line 37

#line 37
  result = MacCoordAttrM$IMacGET$GetmacPANId(arg_0x408fb528);
#line 37

#line 37
  return result;
#line 37
}
#line 37
# 29 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t RouterM$NeighborTable$getByAddr(TMacAddress arg_0x408deee0, NwkNeighbor **arg_0x408dd0c8){
#line 29
  unsigned char result;
#line 29

#line 29
  result = NeighborTableM$NeighborTable$getByAddr(arg_0x408deee0, arg_0x408dd0c8);
#line 29

#line 29
  return result;
#line 29
}
#line 29
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t RouterM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x409ba010, const TMacShortAddress arg_0x409ba1c8){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacAddressM$IMacAddress$SetShortAddress(arg_0x409ba010, arg_0x409ba1c8);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t RouterM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x408dea50){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x408dea50);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 196 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NwkAddressingM$NIB$getDepth(void){
#line 196
  unsigned char result;
#line 196

#line 196
  result = NIBM$NIB$getDepth();
#line 196

#line 196
  return result;
#line 196
}
#line 196
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacShortAddress NwkAddressingM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408f9440){
#line 43
  unsigned int result;
#line 43

#line 43
  result = MacCoordAttrM$IMacGET$GetmacShortAddress(arg_0x408f9440);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 204 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static inline  void NwkAddressingM$NwkAddressing$getDirection(NwkAddr dest, NwkRelationship *relationship, NwkAddr *next_hop)
{
  NwkAddr myAddr = NwkAddressingM$IMacGET$GetmacShortAddress(NULL);
  uint8_t Rm = NwkAddressingM$NIB$getNwkMaxRouters();
  uint8_t my_depth = NwkAddressingM$NIB$getDepth();

  if (myAddr == 0) {
    *relationship = NWK_CHILD;
    }
  else {
      if (myAddr < dest && dest < myAddr + NwkAddressingM$countCskip(my_depth - 1)) {
        *relationship = NWK_CHILD;
        }
      else {
#line 217
        *relationship = NWK_PARENT;
        }
    }
  if (*relationship == NWK_CHILD) 
    {
      NwkAddr cskip_d = NwkAddressingM$countCskip(my_depth);

#line 223
      if (dest > myAddr + Rm * cskip_d) {
        *next_hop = dest;
        }
      else {
          *next_hop = dest - (dest - (myAddr + 1)) % cskip_d;
        }
    }
}

# 25 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
inline static  void RouterM$NwkAddressing$getDirection(NwkAddr arg_0x40913dd8, NwkRelationship *arg_0x40911010, NwkAddr *arg_0x409111b0){
#line 25
  NwkAddressingM$NwkAddressing$getDirection(arg_0x40913dd8, arg_0x40911010, arg_0x409111b0);
#line 25
}
#line 25
# 438 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline result_t RouterM$get_relaying_options(NwkAddr destinationAddr, NwkAddr *nextHop, NwkRelationship *relationship, bool *indirect)
{
  NwkNeighbor *pngbr;

  RouterM$NwkAddressing$getDirection(destinationAddr, relationship, nextHop);
  if (*relationship == NWK_PARENT) 
    {

      pngbr = NULL;
      while (RouterM$NeighborTable$getNextPtr(&pngbr)) 
        {
          if (pngbr->relationship == NWK_PARENT) 
            {
              *nextHop = pngbr->networkAddr;
              *indirect = FALSE;
              break;
            }
        }
    }
  else 

    {
      TMacAddress ma;

      RouterM$IMacAddress$SetShortAddress(&ma, *nextHop);
      if (RouterM$NeighborTable$getByAddr(ma, &pngbr)) 
        {
          *indirect = ! pngbr->rxOnWhenIdle;
        }
      else {
        return FAIL;
        }
    }
  return SUCCESS;
}

# 113 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  uint8_t NIBM$NIB$getNwkMaxDepth(void)
{
  return NIBM$nwkMaxDepth;
}

# 56 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NwkAddressingM$NIB$getNwkMaxDepth(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = NIBM$NIB$getNwkMaxDepth();
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 8 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressM.nc"
static inline  TMacAddressMode MacAddressM$IMacAddress$GetAddressMode(const TMacAddress macAddress)
{
  return ((_TMacAddress )macAddress).mode;
}

# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  TMacAddressMode NeighborTableM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409bdc98){
#line 6
  enum __nesc_unnamed4293 result;
#line 6

#line 6
  result = MacAddressM$IMacAddress$GetAddressMode(arg_0x409bdc98);
#line 6

#line 6
  return result;
#line 6
}
#line 6


inline static  result_t NeighborTableM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x409bc1e8, const TMacAddress arg_0x409bc398){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacAddressM$IMacAddress$GetShortAddress(arg_0x409bc1e8, arg_0x409bc398);
#line 8

#line 8
  return result;
#line 8
}
#line 8


inline static  result_t NeighborTableM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409bc8b0, const TMacAddress arg_0x409bca60){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacAddressM$IMacAddress$GetExtendedAddress(arg_0x409bc8b0, arg_0x409bca60);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 57 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline  uint8_t RouterM$newMsduHandle(void)
{
  static uint8_t brr_current_msduHandle = 0;

#line 60
  return ++brr_current_msduHandle;
}

#line 559
static inline   result_t RouterM$UpperIf$default$Request(TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
const uint8_t *const pmsduFrame, 
uint8_t msduHandle, 
TxOptions bmTxOpt)
{
  return FAIL;
}

# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t RouterM$UpperIf$Request(TMacPANId arg_0x409e2778, TMacAddress arg_0x409e2910, TMacPANId arg_0x409e2ab0, TMacAddress arg_0x409e2c48, uint8_t arg_0x409e2de0, const uint8_t *const arg_0x409e6010, uint8_t arg_0x409e61a8, TxOptions arg_0x409e6340){
#line 7
  unsigned char result;
#line 7

#line 7
  result = RouterM$UpperIf$default$Request(arg_0x409e2778, arg_0x409e2910, arg_0x409e2ab0, arg_0x409e2c48, arg_0x409e2de0, arg_0x409e6010, arg_0x409e61a8, arg_0x409e6340);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40da92d8, const bool arg_0x40da9480){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacFrameFormatM$IMacFrame$SetAckRequest(arg_0x40da92d8, arg_0x40da9480);
#line 14

#line 14
  return result;
#line 14
}
#line 14
#line 36
inline static  result_t MacDataM$IMacFrame$SetPayload(TMacFrame *const arg_0x40dd2510, const uint8_t *const arg_0x40dd2728, TMacPayloadLength arg_0x40dd28c0){
#line 36
  unsigned char result;
#line 36

#line 36
  result = MacFrameFormatM$IMacFrame$SetPayload(arg_0x40dd2510, arg_0x40dd2728, arg_0x40dd28c0);
#line 36

#line 36
  return result;
#line 36
}
#line 36
# 32 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacDSN(TMacDSN macDSN)
{
  MacCommonAttrM$pib.macDSN = macDSN;
  return MAC_SUCCESS;
}

# 19 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacDataM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dc6828){
#line 19
  enum __nesc_unnamed4280 result;
#line 19

#line 19
  result = MacCommonAttrM$IMacCommonAttr$SetmacDSN(arg_0x40dc6828);
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 20 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40da8e60, const TMacSequenceNumber arg_0x40da6030){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacFrameFormatM$IMacFrame$SetSequenceNumber(arg_0x40da8e60, arg_0x40da6030);
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 18 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacDSN MacDataM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dc63a0){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacCommonAttrM$IMacCommonAttr$GetmacDSN(arg_0x40dc63a0);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 27 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40da5b38, const TMacAddress arg_0x40da5ce8){
#line 27
  unsigned char result;
#line 27

#line 27
  result = MacFrameFormatM$IMacFrame$SetDstAddress(arg_0x40da5b38, arg_0x40da5ce8);
#line 27

#line 27
  return result;
#line 27
}
#line 27
#line 24
inline static  result_t MacDataM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40da6c60, const TMacPANId arg_0x40da6e10){
#line 24
  unsigned char result;
#line 24

#line 24
  result = MacFrameFormatM$IMacFrame$SetDstPANId(arg_0x40da6c60, arg_0x40da6e10);
#line 24

#line 24
  return result;
#line 24
}
#line 24









inline static  result_t MacDataM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40da2700, const TMacAddress arg_0x40da28b0){
#line 33
  unsigned char result;
#line 33

#line 33
  result = MacFrameFormatM$IMacFrame$SetSrcAddress(arg_0x40da2700, arg_0x40da28b0);
#line 33

#line 33
  return result;
#line 33
}
#line 33
#line 30
inline static  result_t MacDataM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40da3908, const TMacPANId arg_0x40da3ab8){
#line 30
  unsigned char result;
#line 30

#line 30
  result = MacFrameFormatM$IMacFrame$SetSrcPANId(arg_0x40da3908, arg_0x40da3ab8);
#line 30

#line 30
  return result;
#line 30
}
#line 30
#line 17
inline static  result_t MacDataM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40da80a8, const bool arg_0x40da8250){
#line 17
  unsigned char result;
#line 17

#line 17
  result = MacFrameFormatM$IMacFrame$SetIntraPAN(arg_0x40da80a8, arg_0x40da8250);
#line 17

#line 17
  return result;
#line 17
}
#line 17
#line 8
inline static  result_t MacDataM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40dac708, const bool arg_0x40dac8b0){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacFrameFormatM$IMacFrame$SetSecurityEnabled(arg_0x40dac708, arg_0x40dac8b0);
#line 8

#line 8
  return result;
#line 8
}
#line 8



inline static  result_t MacDataM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40dab510, const bool arg_0x40dab6b8){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacFrameFormatM$IMacFrame$SetFramePending(arg_0x40dab510, arg_0x40dab6b8);
#line 11

#line 11
  return result;
#line 11
}
#line 11
#line 5
inline static  result_t MacDataM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40dad8e8, const TMacFrameType arg_0x40dada98){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacFrameFormatM$IMacFrame$SetFrameType(arg_0x40dad8e8, arg_0x40dada98);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 57 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline result_t MacDataM$DATA_Request(MacDataM$SF sf, 
TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
const uint8_t *const pmsduFrame, 
uint8_t msduHandle, 
TxOptions bmTxOpt)
{



  if (MacDataM$state != MacDataM$IDLE) 
    {
      MacDataM$Confirm(sf, msduHandle, MAC_TRANSACTION_OVERFLOW);
      return SUCCESS;
    }

  MacDataM$currentSf = sf;

  MacDataM$st_msduHandle = msduHandle;

  MacDataM$st_TimesRetrLeft = MAC_AMAX_FRAME_RETRIES;







  MacDataM$IMacFrame$SetFrameType(&MacDataM$st_msg, MAC_FRAME_TYPE_DATA);
  MacDataM$IMacFrame$SetFramePending(&MacDataM$st_msg, FALSE);
  MacDataM$IMacFrame$SetSecurityEnabled(&MacDataM$st_msg, FALSE);

  MacDataM$IMacFrame$SetIntraPAN(&MacDataM$st_msg, FALSE);
  MacDataM$IMacFrame$SetSrcPANId(&MacDataM$st_msg, SrcPANid);
  MacDataM$IMacFrame$SetSrcAddress(&MacDataM$st_msg, SrcAddr);
  MacDataM$IMacFrame$SetDstPANId(&MacDataM$st_msg, DstPANid);
  MacDataM$IMacFrame$SetDstAddress(&MacDataM$st_msg, DstAddr);

  MacDataM$st_ackDSN = MacDataM$IMacCommonAttr$GetmacDSN(NULL);
  MacDataM$IMacFrame$SetSequenceNumber(&MacDataM$st_msg, MacDataM$st_ackDSN);
  MacDataM$IMacCommonAttr$SetmacDSN(MacDataM$st_ackDSN + 1);
  MacDataM$IMacFrame$SetPayload(&MacDataM$st_msg, pmsduFrame, msduLength);

  MacDataM$IMacFrame$SetAckRequest(&MacDataM$st_msg, bmTxOpt.acknowledged);

  if (bmTxOpt.acknowledged) {
    MacDataM$st_waitAck = TRUE;
    }

  if (MacDataM$Send(&MacDataM$st_msg, msduHandle)) {
    MacDataM$state = MacDataM$WAIT_SEND_DONE;
    }
  else {
      ;
      MacDataM$Confirm(sf, MacDataM$st_msduHandle, MAC_CHANNEL_ACCESS_FAILURE);
    }

  return SUCCESS;
}

#line 308
static inline  result_t MacDataM$IMacDataOwn$Request(
TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
const uint8_t *const pmsduFrame, 
uint8_t msduHandle, 
TxOptions bmTxOpt)
{
  return MacDataM$DATA_Request(MacDataM$OWN_SF, 
  SrcPANid, 
  SrcAddr, 
  DstPANid, 
  DstAddr, 
  msduLength, 
  pmsduFrame, 
  msduHandle, 
  bmTxOpt);
}

# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t RouterM$LowerIf$Request(TMacPANId arg_0x409e2778, TMacAddress arg_0x409e2910, TMacPANId arg_0x409e2ab0, TMacAddress arg_0x409e2c48, uint8_t arg_0x409e2de0, const uint8_t *const arg_0x409e6010, uint8_t arg_0x409e61a8, TxOptions arg_0x409e6340){
#line 7
  unsigned char result;
#line 7

#line 7
  result = MacDataM$IMacDataOwn$Request(arg_0x409e2778, arg_0x409e2910, arg_0x409e2ab0, arg_0x409e2c48, arg_0x409e2de0, arg_0x409e6010, arg_0x409e61a8, arg_0x409e6340);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  TMacAddressMode MacFrameFormatM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409bdc98){
#line 6
  enum __nesc_unnamed4293 result;
#line 6

#line 6
  result = MacAddressM$IMacAddress$GetAddressMode(arg_0x409bdc98);
#line 6

#line 6
  return result;
#line 6
}
#line 6


inline static  result_t MacFrameFormatM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x409bc1e8, const TMacAddress arg_0x409bc398){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacAddressM$IMacAddress$GetShortAddress(arg_0x409bc1e8, arg_0x409bc398);
#line 8

#line 8
  return result;
#line 8
}
#line 8


inline static  result_t MacFrameFormatM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409bc8b0, const TMacAddress arg_0x409bca60){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacAddressM$IMacAddress$GetExtendedAddress(arg_0x409bc8b0, arg_0x409bca60);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 68 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPendingM.nc"
static inline  void MacPendingM$IMacPool$FreeDone(const TMacPoolHandle macPoolHandle, 
const TMacStatus status)
#line 69
{
}

# 217 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconPackerM.nc"
static inline  void MacBeaconPackerM$IMacPool$FreeDone(const TMacPoolHandle macPoolHandle, const TMacStatus status)
{
}

# 9 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
inline static  void MacPoolM$IMacPool$FreeDone(const TMacPoolHandle arg_0x40ee5030, const TMacStatus arg_0x40ee51d8){
#line 9
  MacBeaconPackerM$IMacPool$FreeDone(arg_0x40ee5030, arg_0x40ee51d8);
#line 9
  MacPendingM$IMacPool$FreeDone(arg_0x40ee5030, arg_0x40ee51d8);
#line 9
}
#line 9
# 144 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static inline  void MacPoolM$FreeDone(void)
{
  MacPoolM$TMacPoolIndex i = 0;

#line 147
  for (; i < MacPoolM$MAC_POOL_SIZE; i++) 
    if (MacPoolM$MAC_POOL_ITEM_FREE_DONE == MacPoolM$pool[i].state) {
        MacPoolM$IMacPool$FreeDone(MacPoolM$ToPoolHandle(i), MacPoolM$pool[i].status);
        MacPoolM$pool[i].state = MacPoolM$MAC_POOL_ITEM_FREE;
        return;
      }
}

# 8 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
inline static  result_t MacPendingM$IMacPool$Free(const TMacPoolHandle arg_0x40ee69b8, const TMacStatus arg_0x40ee6b60){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacPoolM$IMacPool$Free(arg_0x40ee69b8, arg_0x40ee6b60);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 57 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPendingM.nc"
static inline  void MacPendingM$IMacCSMA$SendDone(TMacStatus macStatus, const TUniData uniData)
{

  if (MAC_SUCCESS == macStatus) {
      TMacPoolHandle macPoolHandle = (TMacPoolHandle )uniData;

#line 62
      MacPendingM$IMacPool$Free(macPoolHandle, MAC_SUCCESS);
      return;
    }
  return;
}

# 73 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanBeaconM.nc"
static inline  void MacScanBeaconM$CSMA$SendDone(TMacStatus status, const TUniData unidata)
{


  MacScanBeaconM$beaconAllocated = FALSE;
}

# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacAssocCoordM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dcacf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dcacf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCOMM_STATUS.nc"
inline static  void MacAssocCoordM$IMacCOMM_STATUS$Indication(TMacPANId arg_0x409059c0, TMacAddress arg_0x40905b60, TMacAddress arg_0x40905cf0, TMacStatus arg_0x40905e90){
#line 6
  NLME_JoinParentM$IMacCOMM_STATUS$Indication(arg_0x409059c0, arg_0x40905b60, arg_0x40905cf0, arg_0x40905e90);
#line 6
}
#line 6
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacAssocCoordM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x409ba6d0, const TMacExtendedAddress arg_0x409ba890){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacAddressM$IMacAddress$SetExtendedAddress(arg_0x409ba6d0, arg_0x409ba890);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 119 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static inline  void MacAssocCoordM$IMacSendOwn$SendDone(TMacStatus status, const TUniData unidata)
{
  TMacAddress dstAddress;
  TMacAddress srcAddress;

  MacAssocCoordM$IMacAddress$SetExtendedAddress(&dstAddress, MacAssocCoordM$dstExtAddress);
  MacAssocCoordM$IMacAddress$SetExtendedAddress(&srcAddress, info_param.MAC_ADDRESS);

  MacAssocCoordM$IMacCOMM_STATUS$Indication(MacAssocCoordM$IMacCommonAttr$GetmacPANId(NULL), srcAddress, dstAddress, status);
  return;
}

# 44 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacAckWaitDuration MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const pMacStatus)
{
  if (pMacStatus != NULL) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 48
  return MacCommonAttrM$pib.macAckWaitDuration;
}

# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacAckWaitDuration MacDataM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dc86c8){
#line 14
  enum __nesc_unnamed4282 result;
#line 14

#line 14
  result = MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(arg_0x40dc86c8);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacDataM$AckWaitTimer$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(5U, arg_0x4090c6e0, arg_0x4090c868);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 120 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline void MacDataM$SendDone(TMacStatus status, const TUniData unidata)
{



  if (MacDataM$state == MacDataM$WAIT_SEND_DONE) 
    {

      MacDataM$st_msduHandle = unidata;

      if (status == MAC_SUCCESS && MacDataM$st_waitAck) 
        {
          if (MacDataM$AckWaitTimer$SetOneShot(MacDataM$IMacCommonAttr$GetmacAckWaitDuration(NULL), NULL)) {
            MacDataM$state = MacDataM$WAIT_ACK;
            }
          else {
              MacDataM$state = MacDataM$IDLE;
              MacDataM$Confirm(MacDataM$currentSf, unidata, MAC_CHANNEL_ACCESS_FAILURE);
            }
        }
      else 
        {
          MacDataM$state = MacDataM$IDLE;
          MacDataM$Confirm(MacDataM$currentSf, unidata, status);
        }
    }
}

#line 337
static inline  void MacDataM$IMacSendOwn$SendDone(TMacStatus status, const TUniData unidata)
{

  return MacDataM$SendDone(status, unidata);
}

# 56 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static inline   void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$default$SendDone(uint8_t context, TMacStatus macStatus, TUniData unidata)
{
  ;
}

# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendDone(uint8_t arg_0x40f88168, TMacStatus arg_0x40ef9b08, const TUniData arg_0x40ef9cb0){
#line 12
  switch (arg_0x40f88168) {
#line 12
    case 0U:
#line 12
      MacPendingM$IMacCSMA$SendDone(arg_0x40ef9b08, arg_0x40ef9cb0);
#line 12
      break;
#line 12
    case 1U:
#line 12
      MacAssocCoordM$IMacSendOwn$SendDone(arg_0x40ef9b08, arg_0x40ef9cb0);
#line 12
      break;
#line 12
    case 2U:
#line 12
      MacDataM$IMacSendOwn$SendDone(arg_0x40ef9b08, arg_0x40ef9cb0);
#line 12
      break;
#line 12
    case 3U:
#line 12
      MacScanBeaconM$CSMA$SendDone(arg_0x40ef9b08, arg_0x40ef9cb0);
#line 12
      break;
#line 12
    default:
#line 12
      /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$default$SendDone(arg_0x40f88168, arg_0x40ef9b08, arg_0x40ef9cb0);
#line 12
      break;
#line 12
    }
#line 12
}
#line 12
# 42 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static inline  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendDone(const uint8_t _sfIndex, uint8_t context, 
TMacStatus macStatus, const TUniData uniData)
{
  if (_sfIndex == 0) {
    /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendDone(context, macStatus, uniData);
    }
}

# 50 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
inline static  void MacCAPM$SendDone(const uint8_t arg_0x40f16328, uint8_t arg_0x40f164a8, TMacStatus arg_0x40f16638, const TUniData arg_0x40f167e0){
#line 50
  /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendDone(arg_0x40f16328, arg_0x40f164a8, arg_0x40f16638, arg_0x40f167e0);
#line 50
}
#line 50
# 37 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacPANId NLME_JoinParentM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408fb528){
#line 37
  unsigned int result;
#line 37

#line 37
  result = MacCoordAttrM$IMacGET$GetmacPANId(arg_0x408fb528);
#line 37

#line 37
  return result;
#line 37
}
#line 37
# 196 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NLME_JoinParentM$NIB$getDepth(void){
#line 196
  unsigned char result;
#line 196

#line 196
  result = NIBM$NIB$getDepth();
#line 196

#line 196
  return result;
#line 196
}
#line 196
# 280 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  uint8_t NIBM$NIB$getChannel(void)
#line 280
{
#line 280
  return NIBM$channel;
}

# 200 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NLME_JoinParentM$NIB$getChannel(void){
#line 200
  unsigned char result;
#line 200

#line 200
  result = NIBM$NIB$getChannel();
#line 200

#line 200
  return result;
#line 200
}
#line 200
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
inline static   TSysTime NLME_JoinParentM$ILocalTime$Read(void){
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
# 49 "../../../zigzag/ZigBee/implementation/NeighborTableM.nc"
static inline  result_t NeighborTableM$NeighborTable$update(NwkNeighbor new_neighbor)
{
  int i;
  int l_free_idx = 20;


  for (i = 0; i < 20; i++) 
    {

      if ((
#line 57
      new_neighbor.extendedAddr != 0xFFFFFFFFFFFFFFFFLL && new_neighbor.extendedAddr == NeighborTableM$Table[i].extendedAddr)
       || (
      new_neighbor.networkAddr != 0xFFFF && new_neighbor.networkAddr == NeighborTableM$Table[i].networkAddr)) 
        {
#line 60
          NeighborTableM$Table[i].networkAddr = 0xFFFF;
#line 60
          NeighborTableM$Table[i].extendedAddr = 0xFFFFFFFFFFFFFFFFLL;
        }
#line 60
      ;

      if (l_free_idx == 20 && (NeighborTableM$Table[i].networkAddr == 0xFFFF && NeighborTableM$Table[i].extendedAddr == 0xFFFFFFFFFFFFFFFFLL)) {
        l_free_idx = i;
        }
    }
  if (l_free_idx != 20) {
    NeighborTableM$Table[l_free_idx] = new_neighbor;
    }
#line 81
  return FAIL;
}

# 15 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t NLME_JoinParentM$NeighborTable$update(NwkNeighbor arg_0x408de578){
#line 15
  unsigned char result;
#line 15

#line 15
  result = NeighborTableM$NeighborTable$update(arg_0x408de578);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 28 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
inline static  void NLME_JoinParentM$updateBeacon(void){
#line 28
  NwkBeaconParentM$updateBeacon();
#line 28
}
#line 28
# 97 "../../../zigzag/ZigNet2SpiM.nc"
static inline  void ZigNet2SpiM$IZigParentJoin$indication(NwkAddr shortAddr, IEEEAddr extendedAddr, 
NwkCapabilityInfo capabilityInformation, bool secureJoin)
{

  uint8_t join_msg[15];


  ZigNet2SpiM$workaround = &extendedAddr;
  serialize(join_msg, 15, "1:1:1:1:8:1:2:", 
  (uint8_t )0x00, (uint8_t )0x00, (uint8_t )0x04, (uint8_t )(15 - 4), 
  ZigNet2SpiM$ILocalTime64$getLocalTime(), (uint8_t )0x20, shortAddr);
  ZigNet2SpiM$usartTx(0xfffe, extendedAddr, 15, join_msg);
  return;
}

# 15 "../../../zigzag/ZigBee/interface/NLME_JoinParent.nc"
inline static  void NLME_JoinParentM$NLME_JoinParent$indication(NwkAddr arg_0x408e7808, IEEEAddr arg_0x408e79a8, NwkCapabilityInfo arg_0x408e7b58, bool arg_0x408e7d00){
#line 15
  ZigNet2SpiM$IZigParentJoin$indication(arg_0x408e7808, arg_0x408e79a8, arg_0x408e7b58, arg_0x408e7d00);
#line 15
}
#line 15
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacCoordAttrM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e84d90);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 80 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacBeaconOrder MacCoordAttrM$IMacGET$GetmacBeaconOrder(
TMacStatus *const pMacStatus)
{
  return MacCoordAttrM$IMacSuperframeAttr$GetmacBeaconOrder(pMacStatus);
}

# 19 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacBeaconOrder NLME_JoinParentM$IMacGET$GetmacBeaconOrder(TMacStatus *const arg_0x408ff728){
#line 19
  unsigned char result;
#line 19

#line 19
  result = MacCoordAttrM$IMacGET$GetmacBeaconOrder(arg_0x408ff728);
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t NLME_JoinParentM$ITimerSymbol$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(0U, arg_0x4090c6e0, arg_0x4090c868);
#line 7

#line 7
  return result;
#line 7
}
#line 7
inline static  result_t NLME_PermitJoiningM$ITimerSymbol$SetOneShot(TSysTime arg_0x4090c6e0, TUniData arg_0x4090c868){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(1U, arg_0x4090c6e0, arg_0x4090c868);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 19 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
inline static  bool NLME_PermitJoiningM$NwkAddressing$hasVacantAddrs(void){
#line 19
  unsigned char result;
#line 19

#line 19
  result = NwkAddressingM$NwkAddressing$hasVacantAddrs();
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 30 "../../../zigzag/ZigBee/implementation/NLME_PermitJoiningM.nc"
static inline  NwkStatus NLME_PermitJoiningM$NLME_PermitJoining$request(
uint8_t permitDuration)

{
  if (permitDuration == 0) {
    return NLME_PermitJoiningM$IMacSET$SetmacAssociationPermit(FALSE);
    }
  else {
      if (!NLME_PermitJoiningM$NwkAddressing$hasVacantAddrs()) {
        return NWK_INVALID_REQUEST;
        }
      else {
          NwkStatus status = NLME_PermitJoiningM$IMacSET$SetmacAssociationPermit(TRUE);

#line 43
          NLME_PermitJoiningM$updateBeacon();
          if (status == MAC_SUCCESS && permitDuration != 0xff) 
            {
              NLME_PermitJoiningM$ITimerSymbol$SetOneShot(65536 * permitDuration, 0);
            }
          return status;
        }
    }
}

# 16 "../../../zigzag/ZigBee/interface/NLME_PermitJoining.nc"
inline static  NwkStatus ZigCoordM$IZigPermitJoining$request(uint8_t arg_0x408cb030){
#line 16
  enum __nesc_unnamed4327 result;
#line 16

#line 16
  result = NLME_PermitJoiningM$NLME_PermitJoining$request(arg_0x408cb030);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t ChildSupervisorM$ITimerSymbol$SetPeriodic(TSysTime arg_0x4090c0c0, TUniData arg_0x4090c248){
#line 6
  unsigned char result;
#line 6

#line 6
  result = TimerSymbol2M$ITimerSymbol$SetPeriodic(6U, arg_0x4090c0c0, arg_0x4090c248);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 12 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   TSysTime TimeCastM$ITimeCast$MillisToSymbols(TMilliSec millis)
{
  return (TSysTime )((uint64_t )millis * 125 >> 1);
}

# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TSysTime ChildSupervisorM$ITimeCast$MillisToSymbols(TMilliSec arg_0x40a378b8){
#line 6
  unsigned long result;
#line 6

#line 6
  result = TimeCastM$ITimeCast$MillisToSymbols(arg_0x40a378b8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 26 "../../../zigzag/ZigBee/extra/ChildSupervisorM.nc"
static inline result_t  ChildSupervisorM$StdControl$start(void)
{
  TSysTime millis = (TSysTime )((uint16_t )info_param.ALIVE_SEND_PERIOD * (info_param.ALIVE_SEND_ATTEMPTS + 1)) << 10;

#line 29
  ChildSupervisorM$table_check_period = ChildSupervisorM$ITimeCast$MillisToSymbols(millis);
  ChildSupervisorM$ITimerSymbol$SetPeriodic(ChildSupervisorM$table_check_period, 0);

  ;

  return SUCCESS;
}

# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t ZigCoordM$ChildSupervisorControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = ChildSupervisorM$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
inline static  result_t MacCommonAttrM$IPhyAttr$SetpanId(uint16_t arg_0x40b38010){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyCC2420M$IPhyAttr$SetpanId(arg_0x40b38010);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 269 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setOnline(NwkOnlineStatus status)
#line 269
{
#line 269
  NIBM$onlineStatus = status;
}

# 191 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NLME_NetworkFormationM$NIB$setOnline(NwkOnlineStatus arg_0x408bf800){
#line 191
  NIBM$NIB$setOnline(arg_0x408bf800);
#line 191
}
#line 191
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacSuperframesM$IPhyTrxParent$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(0U, arg_0x40b477b8, arg_0x40b47960);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   uint32_t TimerSymbolAsyncM$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a346a0){
#line 15
  unsigned long result;
#line 15

#line 15
  result = TimeCastM$ITimeCast$SymbolsToJiffies(arg_0x40a346a0);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 183 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline   
#line 182
result_t TimerSymbolAsyncM$ITimerSymbolAsync$SetOneShotAt(uint8_t timerNumber, 
TSysTime time, TUniData uniData)
{
#line 197
  return TimerSymbolAsyncM$StartTimerAt(timerNumber, 
  TimerSymbolAsyncM$ITimeCast$SymbolsToJiffies(time), 
  0, uniData, FALSE);
}

# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
inline static   result_t MacSuperframesM$Timer$SetOneShotAt(TSysTime arg_0x40a317e0, TUniData arg_0x40a31968){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbolAsyncM$ITimerSymbolAsync$SetOneShotAt(0U, arg_0x40a317e0, arg_0x40a31968);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 436 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline   void MacSuperframesM$IMacSYNC_LOSS$default$Indication(TMacStatus status)
{
}

# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC_LOSS.nc"
inline static  void MacSuperframesM$IMacSYNC_LOSS$Indication(TMacStatus arg_0x40e9fb50){
#line 6
  MacSuperframesM$IMacSYNC_LOSS$default$Indication(arg_0x40e9fb50);
#line 6
}
#line 6
# 428 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  void MacSuperframesM$syncLoss(void)
{

  MacSuperframesM$IMacSYNC_LOSS$Indication(MAC_BEACON_LOSS);
}









static inline    void MacSuperframesM$ParentCAP$default$EndCAP(void)
{
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
inline static   void MacSuperframesM$ParentCAP$EndCAP(void){
#line 5
  MacSuperframesM$ParentCAP$default$EndCAP();
#line 5
}
#line 5
# 310 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline void MacSuperframesM$parentEndCAPHandler(void)
{
  MacSuperframesM$ParentCAP$EndCAP();
  MacSuperframesM$accept_beacon = FALSE;
  if (!MacSuperframesM$beacon_received) 
    {
      MacSuperframesM$timestamp += MacSuperframesM$beacon_interval;
      MacSuperframesM$bad_beacons++;
      if (MacSuperframesM$bad_beacons >= MAC_AMAX_LOST_BEACONS) 
        {
          ;
          MacSuperframesM$parent_active = FALSE;
          TOS_post(MacSuperframesM$syncLoss);
        }
    }
  MacSuperframesM$newCycle();
}

#line 440
static inline    void MacSuperframesM$ParentCAP$default$BeginCAP(void)
{
}

# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
inline static   void MacSuperframesM$ParentCAP$BeginCAP(void){
#line 3
  MacSuperframesM$ParentCAP$default$BeginCAP();
#line 3
}
#line 3
# 299 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline void MacSuperframesM$parentBeginCAPHandler(void)
{


  MacSuperframesM$ParentCAP$BeginCAP();
  MacSuperframesM$current_handler = MacSuperframesM$parentEndCAPHandler;

  MacSuperframesM$Timer$SetOneShotAt(MacSuperframesM$next_sf_start + MacSuperframesM$sf_duration, 0);
}

#line 280
static inline void MacSuperframesM$parentPreparationHandler(void)
{


  MacSuperframesM$current_handler = MacSuperframesM$parentBeginCAPHandler;
  MacSuperframesM$Timer$SetOneShotAt(MacSuperframesM$next_sf_start + 70, 0);

  MacSuperframesM$beacon_received = FALSE;
  MacSuperframesM$accept_beacon = TRUE;


  MacSuperframesM$IPhyTrxParent$Request(PHY_RX_ON, 0);
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
# 457 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$CompareB6$setEvent(uint16_t x)
#line 457
{
#line 457
  MSP430TimerM$TBCCR6 = x;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerSymbolAsyncM$AlarmCompare4$setEvent(uint16_t arg_0x4073c680){
#line 30
  MSP430TimerM$CompareB6$setEvent(arg_0x4073c680);
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
inline static   void TimerSymbolAsyncM$AlarmCompare3$setEvent(uint16_t arg_0x4073c680){
#line 30
  MSP430TimerM$CompareB5$setEvent(arg_0x4073c680);
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
inline static   void TimerSymbolAsyncM$AlarmCompare2$setEvent(uint16_t arg_0x4073c680){
#line 30
  MSP430TimerM$CompareB4$setEvent(arg_0x4073c680);
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
inline static   void TimerSymbolAsyncM$AlarmCompare1$setEvent(uint16_t arg_0x4073c680){
#line 30
  MSP430TimerM$CompareB3$setEvent(arg_0x4073c680);
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
inline static   void TimerSymbolAsyncM$AlarmCompare0$setEvent(uint16_t arg_0x4073c680){
#line 30
  MSP430TimerM$CompareB2$setEvent(arg_0x4073c680);
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

#line 28
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
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacSuperframesM$IPhyTrxOwn$Request(TPhyStatus arg_0x40b477b8, const TUniData arg_0x40b47960){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(1U, arg_0x40b477b8, arg_0x40b47960);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 170 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  void MacCAPM$TurnOffTask(void)
{
  MacCAPM$TurnOffIfInactive(PHY_TRX_OFF);
}

static inline   void MacCAPM$IMacCAP$EndCAP(uint8_t sfIndex)
{



  MacCAPM$cap[sfIndex].state = MacCAPM$CAP_STATE_PASSIVE;

  {
    uint8_t newTrxState = PHY_TRX_OFF;

#line 184
    switch (MacCAPM$cap[sfIndex].sendState) 
      {
        case MacCAPM$CAP_SEND_OFF: case MacCAPM$CAP_SEND_IDLE: 
            case MacCAPM$CAP_SEND_PHY_DONE: 
              case MacCAPM$CAP_SEND_SUCCESS: case MacCAPM$CAP_SEND_FAILURE: 
                  break;
        case MacCAPM$CAP_SEND_PHY: case MacCAPM$CAP_SEND_CCA: case MacCAPM$CAP_SEND_TX_ON: 
              MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_BACKOFF;
        newTrxState = PHY_FORCE_TRX_OFF;
        break;

        case MacCAPM$CAP_SEND_BACKOFF: 
          MacCAPM$IBackoff$Stop();
        break;
      }






    if (!MacCAPM$TurnOffIfInactive(newTrxState)) {
      TOS_post(MacCAPM$TurnOffTask);
      }
  }
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
inline static   void MacSuperframesM$OwnCAP$EndCAP(void){
#line 5
  MacCAPM$IMacCAP$EndCAP(MAC_OWN_SF);
#line 5
}
#line 5
# 261 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline void MacSuperframesM$ownEndCAPHandler(void)
{



  MacSuperframesM$OwnCAP$EndCAP();


  if (!MacSuperframesM$parent_active) {
    MacSuperframesM$timestamp += MacSuperframesM$beacon_interval;
    }

  MacSuperframesM$newCycle();
}

# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
inline static   uint16_t MacCAPM$IMacRandom$Rand(void){
#line 3
  unsigned int result;
#line 3

#line 3
  result = MacRandomM$IMacRandom$Rand();
#line 3

#line 3
  return result;
#line 3
}
#line 3
# 609 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline TSysTime MacCAPM$calcBackoff(uint8_t sfIndex)
{
  return MacCAPM$IMacRandom$Rand() % (1 << MacCAPM$cap[sfIndex].backoffExponent);
}

#line 141
static inline  void MacCAPM$beginSend(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (MacCAPM$cap[MacCAPM$currentSf].state == MacCAPM$CAP_STATE_ACTIVE) 
        {
          MacCAPM$cap[MacCAPM$currentSf].backoffRemain = MacCAPM$calcBackoff(MacCAPM$currentSf);
          if (!MacCAPM$SetBackoff(MacCAPM$currentSf)) 
            {
              MacCAPM$cap[MacCAPM$currentSf].sendState = MacCAPM$CAP_SEND_FAILURE;
              MacCAPM$_SendDone(MacCAPM$currentSf, MAC_CHANNEL_ACCESS_FAILURE);
            }
        }
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
inline static   TSysTime MacCAPM$ILocalTime$Read(void){
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
# 157 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline   void MacCAPM$IMacCAP$BeginCAP(uint8_t sfIndex)
{
  TSysTime now = MacCAPM$ILocalTime$Read();

#line 160
  MacCAPM$cap[sfIndex].beginCAPTime = now;
  MacCAPM$cap[sfIndex].state = MacCAPM$CAP_STATE_ACTIVE;
  MacCAPM$currentSf = sfIndex;



  if (MacCAPM$cap[MacCAPM$currentSf].sendState == MacCAPM$CAP_SEND_BACKOFF) {
    TOS_post(MacCAPM$beginSend);
    }
}

# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
inline static   void MacSuperframesM$OwnCAP$BeginCAP(void){
#line 3
  MacCAPM$IMacCAP$BeginCAP(MAC_OWN_SF);
#line 3
}
#line 3
# 250 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline void MacSuperframesM$ownBeginCAPHandler(void)
{

  MacSuperframesM$OwnCAP$BeginCAP();


  MacSuperframesM$current_handler = MacSuperframesM$ownEndCAPHandler;

  MacSuperframesM$Timer$SetOneShotAt(MacSuperframesM$next_sf_start + MacSuperframesM$sf_duration, 0);
}

# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
inline static   result_t MacSuperframesM$IPhyTxDATA$Request(const TPhyPoolHandle arg_0x40b2ab78, const TUniData arg_0x40b2ad30){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhyTxDATA$Request(0U, arg_0x40b2ab78, arg_0x40b2ad30);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 227 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline void MacSuperframesM$beaconTransmitHandler(void)
{





  MacSuperframesM$IPhyTxDATA$Request(MacSuperframesM$beacon_handle, 0);

  MacSuperframesM$current_handler = MacSuperframesM$ownBeginCAPHandler;

  MacSuperframesM$Timer$SetOneShotAt(MacSuperframesM$next_sf_start + 70, 0);
}

#line 195
static inline void MacSuperframesM$ownPreparationHandler(void)
{





  MacSuperframesM$current_handler = MacSuperframesM$beaconTransmitHandler;
  MacSuperframesM$Timer$SetOneShotAt(MacSuperframesM$next_sf_start - 16, 0);


  MacSuperframesM$IPhyTrxOwn$Request(PHY_TX_ON, 0);
}

#line 23
inline static  void MacSuperframesM$packBeacon(TPhyFrame *arg_0x40e71c50, uint64_t *arg_0x40e71df8){
#line 23
  MacBeaconPackerM$packBeacon(arg_0x40e71c50, arg_0x40e71df8);
#line 23
}
#line 23
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *MacSuperframesM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010){
#line 8
  struct __nesc_unnamed4275 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b5c010);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
inline static  uint64_t MacSuperframesM$ILocalTime64$getLocalTimeAt(TSysTime arg_0x40a39010){
#line 5
  unsigned long long result;
#line 5

#line 5
  result = LocalTime64M$ILocalTime64$getLocalTimeAt(arg_0x40a39010);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 362 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  void MacSuperframesM$createBeacon(void)
{
  uint64_t nextBeaconTime;
  TSysTime now_symbols;
  uint64_t now_millis;




  nextBeaconTime = MacSuperframesM$ILocalTime64$getLocalTimeAt(MacSuperframesM$sf_offset + MacSuperframesM$timestamp);

  {
    TPhyFrame *phyFrame = MacSuperframesM$IPhyPool$GetFrame(MacSuperframesM$beacon_handle);

#line 375
    MacSuperframesM$packBeacon(phyFrame, &nextBeaconTime);
  }
}

# 5 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40dad8e8, const TMacFrameType arg_0x40dada98){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacFrameFormatM$IMacFrame$SetFrameType(arg_0x40dad8e8, arg_0x40dada98);
#line 5

#line 5
  return result;
#line 5
}
#line 5



inline static  result_t MacBeaconPackerM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40dac708, const bool arg_0x40dac8b0){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacFrameFormatM$IMacFrame$SetSecurityEnabled(arg_0x40dac708, arg_0x40dac8b0);
#line 8

#line 8
  return result;
#line 8
}
#line 8



inline static  result_t MacBeaconPackerM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40dab510, const bool arg_0x40dab6b8){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacFrameFormatM$IMacFrame$SetFramePending(arg_0x40dab510, arg_0x40dab6b8);
#line 11

#line 11
  return result;
#line 11
}
#line 11



inline static  result_t MacBeaconPackerM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40da92d8, const bool arg_0x40da9480){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacFrameFormatM$IMacFrame$SetAckRequest(arg_0x40da92d8, arg_0x40da9480);
#line 14

#line 14
  return result;
#line 14
}
#line 14



inline static  result_t MacBeaconPackerM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40da80a8, const bool arg_0x40da8250){
#line 17
  unsigned char result;
#line 17

#line 17
  result = MacFrameFormatM$IMacFrame$SetIntraPAN(arg_0x40da80a8, arg_0x40da8250);
#line 17

#line 17
  return result;
#line 17
}
#line 17
# 111 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static inline  TMacBSN MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBSN(TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 115
  return MacBeaconAttrCoordM$macBSN;
}

# 21 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  TMacBSN MacBeaconPackerM$IMacBeaconAttr$GetmacBSN(TMacStatus *const arg_0x40ec3480){
#line 21
  unsigned char result;
#line 21

#line 21
  result = MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBSN(arg_0x40ec3480);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 20 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40da8e60, const TMacSequenceNumber arg_0x40da6030){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacFrameFormatM$IMacFrame$SetSequenceNumber(arg_0x40da8e60, arg_0x40da6030);
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 117 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBSN(TMacBSN _macBSN)
{
  MacBeaconAttrCoordM$macBSN = _macBSN;
  return MAC_SUCCESS;
}

# 22 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  TMacStatus MacBeaconPackerM$IMacBeaconAttr$SetmacBSN(TMacBSN arg_0x40ec3908){
#line 22
  enum __nesc_unnamed4280 result;
#line 22

#line 22
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBSN(arg_0x40ec3908);
#line 22

#line 22
  return result;
#line 22
}
#line 22
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacShortAddress MacBeaconPackerM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dc99a8){
#line 10
  unsigned int result;
#line 10

#line 10
  result = MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(arg_0x40dc99a8);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacBeaconPackerM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x409ba010, const TMacShortAddress arg_0x409ba1c8){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacAddressM$IMacAddress$SetShortAddress(arg_0x409ba010, arg_0x409ba1c8);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 33 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40da2700, const TMacAddress arg_0x40da28b0){
#line 33
  unsigned char result;
#line 33

#line 33
  result = MacFrameFormatM$IMacFrame$SetSrcAddress(arg_0x40da2700, arg_0x40da28b0);
#line 33

#line 33
  return result;
#line 33
}
#line 33
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacBeaconPackerM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x409ba6d0, const TMacExtendedAddress arg_0x409ba890){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacAddressM$IMacAddress$SetExtendedAddress(arg_0x409ba6d0, arg_0x409ba890);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 30 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40da3908, const TMacPANId arg_0x40da3ab8){
#line 30
  unsigned char result;
#line 30

#line 30
  result = MacFrameFormatM$IMacFrame$SetSrcPANId(arg_0x40da3908, arg_0x40da3ab8);
#line 30

#line 30
  return result;
#line 30
}
#line 30
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacBeaconPackerM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dcacf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dcacf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacBeaconPackerM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x409bad90){
#line 16
  unsigned char result;
#line 16

#line 16
  result = MacAddressM$IMacAddress$SetEmptyAddress(arg_0x409bad90);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 27 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40da5b38, const TMacAddress arg_0x40da5ce8){
#line 27
  unsigned char result;
#line 27

#line 27
  result = MacFrameFormatM$IMacFrame$SetDstAddress(arg_0x40da5b38, arg_0x40da5ce8);
#line 27

#line 27
  return result;
#line 27
}
#line 27
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacBeaconPackerM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e84d90);
#line 6

#line 6
  return result;
#line 6
}
#line 6




inline static  TMacSuperframeOrder MacBeaconPackerM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e80a60){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(arg_0x40e80a60);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 676 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  bool MacCAPM$IMacCAPAttr$GetmacBattLifeExt(TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 680
  return MacCAPM$macBattLifeExt;
}

# 6 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
inline static  bool MacBeaconPackerM$IMacCAPAttr$GetmacBattLifeExt(TMacStatus *const arg_0x40ebc988){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacCAPM$IMacCAPAttr$GetmacBattLifeExt(arg_0x40ebc988);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 18 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  bool MacBeaconPackerM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40eec9e8){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacAssocCoordM$IMacAssocAttr$GetmacAssociationPermit(arg_0x40eec9e8);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 4 "../../../zigzag/IEEE802_15_4/SysCommon/public/ByteOrder.h"
static __inline bool IsTargetHostLSB(void)
{
  const uint8_t n[2] = { 1, 0 };

#line 7
  return * (uint16_t *)n == 1 ? TRUE : FALSE;
}

static __inline uint16_t ToLSB16(uint16_t a)
{
  return IsTargetHostLSB() ? a : (a << 8) | (a >> 8);
}

# 46 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static inline  uint8_t *MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayload(
uint8_t *const placeForBeaconPayload, 
TMacStatus *const pMacStatus)
{
  uint8_t i;

#line 51
  if (pMacStatus != NULL) {
#line 51
    *pMacStatus = MAC_SUCCESS;
    }
  if (placeForBeaconPayload != NULL) 
    {
      for (i = 0; i < MacBeaconAttrCoordM$beaconPayloadLength; i++) 
        {
          placeForBeaconPayload[i] = MacBeaconAttrCoordM$beaconPayload[i];
        }
    }
  return MacBeaconAttrCoordM$beaconPayload;
}

# 13 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  uint8_t *MacBeaconPackerM$IMacBeaconAttr$GetmacBeaconPayload(uint8_t *const arg_0x40ec53b8, TMacStatus *const arg_0x40ec55b8){
#line 13
  unsigned char *result;
#line 13

#line 13
  result = MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayload(arg_0x40ec53b8, arg_0x40ec55b8);
#line 13

#line 13
  return result;
#line 13
}
#line 13
# 39 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static inline  uint8_t MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayloadLength(
TMacStatus *const pMacStatus)
{
  if (pMacStatus != NULL) {
#line 42
    *pMacStatus = MAC_SUCCESS;
    }
#line 43
  return MacBeaconAttrCoordM$beaconPayloadLength;
}

# 11 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  uint8_t MacBeaconPackerM$IMacBeaconAttr$GetmacBeaconPayloadLength(TMacStatus *const arg_0x40ec6e20){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayloadLength(arg_0x40ec6e20);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 15 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconCommonM.nc"
static inline  uint8_t MacBeaconCommonM$getLastBSN(void)
{
  return MacBeaconCommonM$lastParentBSN;
}

# 20 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconPackerM.nc"
inline static  uint8_t MacBeaconPackerM$getLastBSN(void){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacBeaconCommonM$getLastBSN();
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 36 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetPayload(TMacFrame *const arg_0x40dd2510, const uint8_t *const arg_0x40dd2728, TMacPayloadLength arg_0x40dd28c0){
#line 36
  unsigned char result;
#line 36

#line 36
  result = MacFrameFormatM$IMacFrame$SetPayload(arg_0x40dd2510, arg_0x40dd2728, arg_0x40dd28c0);
#line 36

#line 36
  return result;
#line 36
}
#line 36








inline static  TMacRawFrameLength MacBeaconPackerM$IMacFrame$Pack(const TMacFrame *const arg_0x40dd1c38, TPhyFrame *const arg_0x40dd1e28){
#line 44
  unsigned char result;
#line 44

#line 44
  result = MacFrameFormatM$IMacFrame$Pack(arg_0x40dd1c38, arg_0x40dd1e28);
#line 44

#line 44
  return result;
#line 44
}
#line 44
# 449 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline   void MacSuperframesM$default$sleepIndication(TMilliSec sleep_duration)
#line 449
{
}

#line 35
inline static  void MacSuperframesM$sleepIndication(TMilliSec arg_0x40e6d338){
#line 35
  MacSuperframesM$default$sleepIndication(arg_0x40e6d338);
#line 35
}
#line 35
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TMilliSec MacSuperframesM$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a35558){
#line 10
  unsigned long result;
#line 10

#line 10
  result = TimeCastM$ITimeCast$SymbolsToMillis(arg_0x40a35558);
#line 10

#line 10
  return result;
#line 10
}
#line 10
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
  union __nesc_unnamed4415 {
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

#line 160
static inline    void MSP430TimerM$CaptureA0$default$captured(uint16_t time)
#line 160
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureA0$captured(uint16_t arg_0x40750170){
#line 74
  MSP430TimerM$CaptureA0$default$captured(arg_0x40750170);
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

#line 157
static inline    void MSP430TimerM$CompareA0$default$fired(void)
#line 157
{
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA0$fired(void){
#line 34
  MSP430TimerM$CompareA0$default$fired();
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

#line 161
static inline    void MSP430TimerM$CaptureA1$default$captured(uint16_t time)
#line 161
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureA1$captured(uint16_t arg_0x40750170){
#line 74
  MSP430TimerM$CaptureA1$default$captured(arg_0x40750170);
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

#line 158
static inline    void MSP430TimerM$CompareA1$default$fired(void)
#line 158
{
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA1$fired(void){
#line 34
  MSP430TimerM$CompareA1$default$fired();
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

#line 162
static inline    void MSP430TimerM$CaptureA2$default$captured(uint16_t time)
#line 162
{
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureA2$captured(uint16_t arg_0x40750170){
#line 74
  MSP430TimerM$CaptureA2$default$captured(arg_0x40750170);
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

#line 159
static inline    void MSP430TimerM$CompareA2$default$fired(void)
#line 159
{
}

# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA2$fired(void){
#line 34
  MSP430TimerM$CompareA2$default$fired();
#line 34
}
#line 34
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

# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void MSP430TimerM$TimerA$overflow(void){
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
inline static   void MSP430TimerM$CaptureB0$captured(uint16_t arg_0x40750170){
#line 74
  MSP430TimerM$CaptureB0$default$captured(arg_0x40750170);
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
# 348 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB1$getControl(void)
#line 348
{
#line 348
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL1);
}

#line 484
static inline   void MSP430TimerM$CaptureB1$clearOverflow(void)
#line 484
{
#line 484
  MSP430TimerM$TBCCTL1 &= ~0x0002;
}

# 56 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void HPLCC2420InterruptM$SFDCapture$clearOverflow(void){
#line 56
  MSP430TimerM$CaptureB1$clearOverflow();
#line 56
}
#line 56
# 476 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   bool MSP430TimerM$CaptureB1$isOverflowPending(void)
#line 476
{
#line 476
  return MSP430TimerM$TBCCTL1 & 0x0002;
}

# 51 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   bool HPLCC2420InterruptM$SFDCapture$isOverflowPending(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = MSP430TimerM$CaptureB1$isOverflowPending();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 364 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$ControlB1$clearPendingInterrupt(void)
#line 364
{
#line 364
  MSP430TimerM$TBCCTL1 &= ~0x0001;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void HPLCC2420InterruptM$SFDControl$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB1$clearPendingInterrupt();
#line 32
}
#line 32







inline static   void HPLCC2420InterruptM$SFDControl$disableEvents(void){
#line 39
  MSP430TimerM$ControlB1$disableEvents();
#line 39
}
#line 39
# 223 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  void PhyCC2420M$send_done_success(void)
#line 223
{
#line 223
  PhyCC2420M$SendDone(PHY_SUCCESS);
}

#line 32
inline static   uint16_t PhyCC2420M$overflowCount(void){
#line 32
  unsigned int result;
#line 32

#line 32
  result = TimerSymbol2M$overflowCount();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TSysTime PhyCC2420M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a37d70){
#line 7
  unsigned long result;
#line 7

#line 7
  result = TimeCastM$ITimeCast$JiffiesToSymbols(arg_0x40a37d70);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 258 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline   result_t PhyCC2420M$IChipconSFD$captured(uint16_t time)
{
  PhyCC2420M$TPhyRadioState _radioState;

#line 261
  PhyCC2420M$radio.timeStamp = PhyCC2420M$ITimeCast$JiffiesToSymbols(((TSysTime )PhyCC2420M$overflowCount() << 16) | time);
  PhyCC2420M$radio.timeStamp -= 2 * (CC2420_MDMCTRL0_DEFAULT.value.preamble_length + 1) + 2;
  /* atomic removed: atomic calls only */
#line 263
  {
    _radioState = PhyCC2420M$radioState;
    if (PhyCC2420M$PHY_RADIO_TX_FRAME_SEND == PhyCC2420M$radioState) {
      PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_END;
      }
  }
#line 268
  if (PhyCC2420M$PHY_RADIO_TX_FRAME_SEND == _radioState) {
      TOS_post(PhyCC2420M$send_done_success);
    }
  return SUCCESS;
}

# 53 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
inline static   result_t HPLCC2420InterruptM$SFD$captured(uint16_t arg_0x40b95360){
#line 53
  unsigned char result;
#line 53

#line 53
  result = PhyCC2420M$IChipconSFD$captured(arg_0x40b95360);
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 209 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420InterruptM.nc"
static inline   void HPLCC2420InterruptM$SFDCapture$captured(uint16_t time)
#line 209
{
  result_t val = SUCCESS;

#line 211
  HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
  val = HPLCC2420InterruptM$SFD$captured(time);
  if (val == FAIL) {
      HPLCC2420InterruptM$SFDControl$disableEvents();
      HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
    }
  else {
      if (HPLCC2420InterruptM$SFDCapture$isOverflowPending()) {
        HPLCC2420InterruptM$SFDCapture$clearOverflow();
        }
    }
}

# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB1$captured(uint16_t arg_0x40750170){
#line 74
  HPLCC2420InterruptM$SFDCapture$captured(arg_0x40750170);
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
inline static   void MSP430TimerM$CaptureB2$captured(uint16_t arg_0x40750170){
#line 74
  MSP430TimerM$CaptureB2$default$captured(arg_0x40750170);
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
# 388 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline   result_t MacSuperframesM$Timer$Fired(TUniData unidata)
{
  MacSuperframesM$current_handler();






  return SUCCESS;
}

# 347 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static inline    
#line 346
result_t TimerSymbolAsyncM$ITimerSymbolAsync$default$Fired(uint8_t timerNumber, 
TUniData uniData)
{
  return SUCCESS;
}

# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
inline static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$Fired(uint8_t arg_0x40aac3d8, TUniData arg_0x40a53dd8){
#line 16
  unsigned char result;
#line 16

#line 16
  switch (arg_0x40aac3d8) {
#line 16
    case 0U:
#line 16
      result = MacSuperframesM$Timer$Fired(arg_0x40a53dd8);
#line 16
      break;
#line 16
    default:
#line 16
      result = TimerSymbolAsyncM$ITimerSymbolAsync$default$Fired(arg_0x40aac3d8, arg_0x40a53dd8);
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
inline static   void MSP430TimerM$CaptureB3$captured(uint16_t arg_0x40750170){
#line 74
  MSP430TimerM$CaptureB3$default$captured(arg_0x40750170);
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
inline static   void MSP430TimerM$CaptureB4$captured(uint16_t arg_0x40750170){
#line 74
  MSP430TimerM$CaptureB4$default$captured(arg_0x40750170);
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
inline static   void MSP430TimerM$CaptureB5$captured(uint16_t arg_0x40750170){
#line 74
  MSP430TimerM$CaptureB5$default$captured(arg_0x40750170);
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
inline static   void MSP430TimerM$CaptureB6$captured(uint16_t arg_0x40750170){
#line 74
  MSP430TimerM$CaptureB6$default$captured(arg_0x40750170);
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
# 488 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline    result_t HPLUSART0M$USARTData$default$rxDone(uint8_t data)
#line 488
{
#line 488
  return SUCCESS;
}

# 53 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static   result_t HPLUSART0M$USARTData$rxDone(uint8_t arg_0x40c8e570){
#line 53
  unsigned char result;
#line 53

#line 53
  result = HPLUSART0M$USARTData$default$rxDone(arg_0x40c8e570);
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 70 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline    void HPLUSART0M$HPLI2CInterrupt$default$fired(void)
#line 70
{
}

# 43 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLI2CInterrupt.nc"
inline static   void HPLUSART0M$HPLI2CInterrupt$fired(void){
#line 43
  HPLUSART0M$HPLI2CInterrupt$default$fired();
#line 43
}
#line 43
# 486 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static inline    result_t HPLUSART0M$USARTData$default$txDone(void)
#line 486
{
#line 486
  return SUCCESS;
}

# 46 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static   result_t HPLUSART0M$USARTData$txDone(void){
#line 46
  unsigned char result;
#line 46

#line 46
  result = HPLUSART0M$USARTData$default$txDone();
#line 46

#line 46
  return result;
#line 46
}
#line 46
# 177 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port10$clear(void)
#line 177
{
#line 177
  MSP430InterruptM$P1IFG &= ~(1 << 0);
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$clear(void){
#line 40
  MSP430InterruptM$Port10$clear();
#line 40
}
#line 40
# 146 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port10$disable(void)
#line 146
{
#line 146
  MSP430InterruptM$P1IE &= ~(1 << 0);
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$disable(void){
#line 35
  MSP430InterruptM$Port10$disable();
#line 35
}
#line 35
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  result_t MacPendingM$IMacCSMA$Send(const TMacFrame *const arg_0x40ef9010, const TUniData arg_0x40ef91c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(0U, arg_0x40ef9010, arg_0x40ef91c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 64 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static inline MacPoolM$TMacPoolIndex MacPoolM$ToPoolIndex(const TMacPoolHandle handle)
{
  return (MacPoolM$TMacPoolIndex )handle;
}






static inline bool MacPoolM$IsValidHandle(const TMacPoolHandle macPoolHandle)
{
  return MacPoolM$ToPoolIndex(macPoolHandle) < MacPoolM$MAC_POOL_SIZE ? TRUE : FALSE;
}

#line 168
static inline  TMacFrame *MacPoolM$IMacPool$GetFrame(const TMacPoolHandle macPoolHandle)
{
  MacPoolM$TMacPoolIndex itemIndex;

#line 171
  if (!MacPoolM$IsValidHandle(macPoolHandle)) {
#line 171
    return (TMacFrame *)NULL;
    }
#line 172
  itemIndex = MacPoolM$ToPoolIndex(macPoolHandle);
  if (MacPoolM$MAC_POOL_ITEM_ALLOC == MacPoolM$pool[itemIndex].state || 
  MacPoolM$MAC_POOL_ITEM_FREE_DONE == MacPoolM$pool[itemIndex].state) {
    return & MacPoolM$pool[itemIndex].macFrame;
    }
#line 176
  return (TMacFrame *)NULL;
}

# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
inline static  TMacFrame *MacPendingM$IMacPool$GetFrame(const TMacPoolHandle arg_0x40ee5698){
#line 11
  struct __nesc_unnamed4310 *result;
#line 11

#line 11
  result = MacPoolM$IMacPool$GetFrame(arg_0x40ee5698);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 69 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressM.nc"
static inline  bool MacAddressM$IMacAddress$Equal(const TMacAddress *const pMacAddress1, 
const TMacAddress *const pMacAddress2)
{
  if (pMacAddress1 == NULL || pMacAddress2 == NULL) {
    return FALSE;
    }
#line 74
  if (pMacAddress1->mode != pMacAddress2->mode) {
    return FALSE;
    }
#line 76
  if (pMacAddress1->mode == _MAC_ADDRESS_SHORT) {
    return pMacAddress1->address.shortAddress == pMacAddress2->address.shortAddress;
    }
#line 78
  if (pMacAddress1->mode == _MAC_ADDRESS_EXTENDED) {
    return pMacAddress1->address.extendedAddress == pMacAddress2->address.extendedAddress;
    }
#line 80
  return TRUE;
}

# 18 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  bool MacPoolM$IMacAddress$Equal(const TMacAddress *const arg_0x409b92b0, const TMacAddress *const arg_0x409b94c0){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacAddressM$IMacAddress$Equal(arg_0x409b92b0, arg_0x409b94c0);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 28 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacPoolM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40da3218, TMacAddress *const arg_0x40da3408){
#line 28
  unsigned char result;
#line 28

#line 28
  result = MacFrameFormatM$IMacFrame$GetDstAddress(arg_0x40da3218, arg_0x40da3408);
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 192 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static inline  bool MacPoolM$SearchDstAddress(const TMacAddress macAddress, 
TMacPoolHandle *const pMacPoolHandle)
{
  if (NULL != pMacPoolHandle) {
      result_t result;
      TMacAddress poolDstAddress;
      MacPoolM$TMacPoolIndex i;

#line 199
      for (i = 0; i < MacPoolM$MAC_POOL_SIZE; i++) 
        if (MacPoolM$MAC_POOL_ITEM_ALLOC == MacPoolM$pool[i].state) {
            result = MacPoolM$IMacFrame$GetDstAddress(& MacPoolM$pool[i].macFrame, &poolDstAddress);
            if (SUCCESS == result && MacPoolM$IMacAddress$Equal(&macAddress, &poolDstAddress)) {
                *pMacPoolHandle = MacPoolM$ToPoolHandle(i);
                return TRUE;
              }
          }
    }
  return FALSE;
}

# 14 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPendingM.nc"
inline static  bool MacPendingM$SearchDstAddress(const TMacAddress arg_0x40fec5f0, TMacPoolHandle *const arg_0x40fec7f0){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacPoolM$SearchDstAddress(arg_0x40fec5f0, arg_0x40fec7f0);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 34 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacPendingM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40da2dd0, TMacAddress *const arg_0x40dd2010){
#line 34
  unsigned char result;
#line 34

#line 34
  result = MacFrameFormatM$IMacFrame$GetSrcAddress(arg_0x40da2dd0, arg_0x40dd2010);
#line 34

#line 34
  return result;
#line 34
}
#line 34
#line 15
inline static  result_t MacPendingM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40da99a0, bool *const arg_0x40da9b88){
#line 15
  unsigned char result;
#line 15

#line 15
  result = MacFrameFormatM$IMacFrame$GetAckRequest(arg_0x40da99a0, arg_0x40da9b88);
#line 15

#line 15
  return result;
#line 15
}
#line 15
#line 42
inline static  result_t MacPendingM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40dd1530, uint8_t *const arg_0x40dd1718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40dd1530, arg_0x40dd1718);
#line 42

#line 42
  return result;
#line 42
}
#line 42
#line 39
inline static  result_t MacPendingM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40dd2df0, TMacPayloadLength *const arg_0x40dd1010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40dd2df0, arg_0x40dd1010);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 6
inline static  result_t MacPendingM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40dac010, arg_0x40dac200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 19 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPendingM.nc"
static inline  result_t MacPendingM$IMacReceive$Receive(TMacFrame *const pMacFrame)
{
  result_t result;

#line 22
  {
#line 22
    TMacFrameType frameType;

#line 23
    result = MacPendingM$IMacFrame$GetFrameType(pMacFrame, &frameType);
    if (SUCCESS != result || MAC_FRAME_TYPE_COMMAND != frameType) {
      return FAIL;
      }
  }
#line 27
  {
#line 27
    enum __nesc_unnamed4416 {
#line 27
      DATA_REQUEST_PAYLOAD_LEN = 1
    };
#line 28
    TMacPayloadLength payloadLength;
    uint8_t payload[DATA_REQUEST_PAYLOAD_LEN];

#line 30
    result = MacPendingM$IMacFrame$GetPayloadLength(pMacFrame, &payloadLength);
    result &= MacPendingM$IMacFrame$GetPayload(pMacFrame, payload);

    if ((
#line 32
    SUCCESS != result || DATA_REQUEST_PAYLOAD_LEN != payloadLength)
     || MAC_CMD_DATA_REQUEST != payload[0]) {
#line 33
      return FAIL;
      }
  }
#line 35
  {
#line 35
    bool ackRequest;

#line 36
    result = MacPendingM$IMacFrame$GetAckRequest(pMacFrame, &ackRequest);
    if (SUCCESS != result || FALSE == ackRequest) {
      return FAIL;
      }
  }
#line 40
  {
#line 40
    TMacPoolHandle macPoolHandle;
    TMacAddress srcAddress;
    TMacFrame *pFindFrame;

#line 43
    result = MacPendingM$IMacFrame$GetSrcAddress(pMacFrame, &srcAddress);
    if (SUCCESS != result || !MacPendingM$SearchDstAddress(srcAddress, &macPoolHandle)) {
      return FAIL;
      }
#line 46
    pFindFrame = MacPendingM$IMacPool$GetFrame(macPoolHandle);
    if (NULL != pFindFrame) {

        result = MacPendingM$IMacCSMA$Send(pFindFrame, (TUniData )macPoolHandle);

        return result;
      }
  }
  return SUCCESS;
}

# 21 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40da6558, TMacSequenceNumber *const arg_0x40da6760){
#line 21
  unsigned char result;
#line 21

#line 21
  result = MacFrameFormatM$IMacFrame$GetSequenceNumber(arg_0x40da6558, arg_0x40da6760);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
inline static   TSysTime RouterM$ILocalTime$Read(void){
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
# 370 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline result_t RouterM$find_place(uint8_t *idx)
{

  uint8_t i;

  uint8_t oldest_frame_idx = RouterM$cur_frame;
  TSysTime max_wait_time = 0;
  TSysTime now = RouterM$ILocalTime$Read();


  *idx = RouterM$cur_frame;

  for (i = 0; i < RouterM$NWK_FRAME_BUFFER_SIZE; i++) 
    {
      if (RouterM$frames[*idx].state == RouterM$NWK_FRAME_NULL) {
        return SUCCESS;
        }
      {

        if (now - RouterM$frames[*idx].timestamp > max_wait_time) 
          {
            oldest_frame_idx = *idx;
            max_wait_time = now - RouterM$frames[*idx].timestamp;
          }
      }

      (*idx)++;
      if (*idx == RouterM$NWK_FRAME_BUFFER_SIZE) {
        *idx = 0;
        }
    }


  *idx = oldest_frame_idx;
  return SUCCESS;
}

# 192 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  NwkOnlineStatus RouterM$NIB$getOnlineStatus(void){
#line 192
  enum __nesc_unnamed4334 result;
#line 192

#line 192
  result = NIBM$NIB$getOnlineStatus();
#line 192

#line 192
  return result;
#line 192
}
#line 192
# 294 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline result_t RouterM$IMacDATA_Indication(
TMacPANId srcPanID, 
TMacAddress src, 
TMacPANId dstPanID, 
TMacAddress dst, 
uint8_t msduLength, 
uint8_t *msdu, 
TMacLinkQuality mpduLinkQuality, 
bool sequrityUse, 
uint8_t aclEntry)

{


  TSysTime now;

#line 309
  now = RouterM$ILocalTime$Read();
  {
    NwkNeighbor *pn;

    if (RouterM$NeighborTable$getByAddr(src, &pn)) {
      pn->lastFrameTime = now;
      }
  }
  if (msduLength >= sizeof(NwkHeader )) 
    {

      const NwkHeader *header = (NwkHeader *)msdu;
      uint8_t *nsdu;
      uint8_t nsduLength;
      NwkAddr myAddr;

      if (header->protocolVersion != NWK_PROTOCOL_VERSION) {
        return SUCCESS;
        }
      if (header->frameType != NWK_DATA_FRAME) {
        return SUCCESS;
        }
      myAddr = RouterM$IMacGET$GetmacShortAddress(NULL);

      nsdu = msdu + sizeof(NwkHeader );
      nsduLength = msduLength - sizeof(NwkHeader );


      if (header->destinationAddr == myAddr || header->destinationAddr == NWK_BROADCAST) 
        {
          RouterM$NLDE_Data$indication(
          header->sourceAddr, 
          header->sourceExtAddr, 
          nsduLength, 
          nsdu, 
          mpduLinkQuality);
        }

      if (header->destinationAddr != myAddr) 
        {
          if (header->radius > 0 && RouterM$NIB$getOnlineStatus() == NWK_ROUTING) 
            {
              uint8_t idx;

#line 352
              if (RouterM$find_place(&idx)) 
                {
                  nmemcpy(RouterM$frames[idx].msdu, msdu, msduLength);
                  RouterM$frames[idx].timestamp = now;
                  RouterM$frames[idx].state = RouterM$NWK_FRAME_PENDING;
                  RouterM$frames[idx].msduLength = msduLength;
                  ((NwkHeader *)RouterM$frames[idx].msdu)->radius--;

                  TOS_post(RouterM$relayNext);
                }
            }
        }
    }

  return SUCCESS;
}

#line 499
static inline  result_t RouterM$LowerIf$Indication(
TMacPANId srcPanID, 
TMacAddress src, 
TMacPANId dstPanID, 
TMacAddress dst, 
uint8_t msduLength, 
uint8_t *msdu, 
TMacLinkQuality mpduLinkQuality, 
bool securityUse, 
uint8_t aclEntry)

{
  securityUse = ((bool *)msdu)[0];
  return RouterM$IMacDATA_Indication(srcPanID, src, dstPanID, dst, msduLength, msdu, mpduLinkQuality, securityUse, aclEntry);
}

# 16 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t MacDataM$IMacDataOwn$Indication(TMacPANId arg_0x409e6810, TMacAddress arg_0x409e69a8, TMacPANId arg_0x409e6b48, TMacAddress arg_0x409e6ce0, uint8_t arg_0x409e6e78, uint8_t *arg_0x409e1068, TMacLinkQuality arg_0x409e1208, bool arg_0x409e13a0, uint8_t arg_0x409e1538){
#line 16
  unsigned char result;
#line 16

#line 16
  result = RouterM$LowerIf$Indication(arg_0x409e6810, arg_0x409e69a8, arg_0x409e6b48, arg_0x409e6ce0, arg_0x409e6e78, arg_0x409e1068, arg_0x409e1208, arg_0x409e13a0, arg_0x409e1538);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 388 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline   result_t MacDataM$IMacDataParent$default$Indication(TMacPANId SrcPANid, TMacAddress SrcAddr, 
TMacPANId DstPANid, TMacAddress DstAddr, uint8_t msduLength, uint8_t *msduFrame, 
TMacLinkQuality linkQuality, bool bSecurityUse, uint8_t ACLEntry)
{
#line 391
  return SUCCESS;
}

# 16 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t MacDataM$IMacDataParent$Indication(TMacPANId arg_0x409e6810, TMacAddress arg_0x409e69a8, TMacPANId arg_0x409e6b48, TMacAddress arg_0x409e6ce0, uint8_t arg_0x409e6e78, uint8_t *arg_0x409e1068, TMacLinkQuality arg_0x409e1208, bool arg_0x409e13a0, uint8_t arg_0x409e1538){
#line 16
  unsigned char result;
#line 16

#line 16
  result = MacDataM$IMacDataParent$default$Indication(arg_0x409e6810, arg_0x409e69a8, arg_0x409e6b48, arg_0x409e6ce0, arg_0x409e6e78, arg_0x409e1068, arg_0x409e1208, arg_0x409e13a0, arg_0x409e1538);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 299 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static inline  result_t MacFrameFormatM$IMacFrame$GetLinkQuality(const TMacFrame *const pMacFrame, 
TMacLinkQuality *const pLinkQuality)
{
  if (pMacFrame != NULL && pLinkQuality != NULL) {
      uint8_t plen = pMacFrame->payload.length;
      int8_t rssi = pMacFrame->payload.raw[plen];
      uint8_t corr = pMacFrame->payload.raw[plen + 1] & 0x7f;

#line 306
      *pLinkQuality = rssi + 60;
      if (70 <= corr) {
#line 307
        *pLinkQuality |= 0x80;
        }
      else {
#line 308
        *pLinkQuality &= 0x7f;
        }
#line 309
      return SUCCESS;
    }
  return FAIL;
}

# 50 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$GetLinkQuality(const TMacFrame *const arg_0x40dce970, TMacLinkQuality *const arg_0x40dceb60){
#line 50
  unsigned char result;
#line 50

#line 50
  result = MacFrameFormatM$IMacFrame$GetLinkQuality(arg_0x40dce970, arg_0x40dceb60);
#line 50

#line 50
  return result;
#line 50
}
#line 50
#line 42
inline static  result_t MacDataM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40dd1530, uint8_t *const arg_0x40dd1718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40dd1530, arg_0x40dd1718);
#line 42

#line 42
  return result;
#line 42
}
#line 42
#line 28
inline static  result_t MacDataM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40da3218, TMacAddress *const arg_0x40da3408){
#line 28
  unsigned char result;
#line 28

#line 28
  result = MacFrameFormatM$IMacFrame$GetDstAddress(arg_0x40da3218, arg_0x40da3408);
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 136 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static inline  result_t MacFrameFormatM$IMacFrame$GetDstPANId(const TMacFrame *const pMacFrame, TMacPANId *const pDstPANId)
{
  if (pMacFrame != NULL && pDstPANId != NULL) {
      *pDstPANId = pMacFrame->dstPANId.value;
      return SUCCESS;
    }
  return FAIL;
}

# 25 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$GetDstPANId(const TMacFrame *const arg_0x40da5448, TMacPANId *const arg_0x40da5638){
#line 25
  unsigned char result;
#line 25

#line 25
  result = MacFrameFormatM$IMacFrame$GetDstPANId(arg_0x40da5448, arg_0x40da5638);
#line 25

#line 25
  return result;
#line 25
}
#line 25









inline static  result_t MacDataM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40da2dd0, TMacAddress *const arg_0x40dd2010){
#line 34
  unsigned char result;
#line 34

#line 34
  result = MacFrameFormatM$IMacFrame$GetSrcAddress(arg_0x40da2dd0, arg_0x40dd2010);
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 194 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static inline  result_t MacFrameFormatM$IMacFrame$GetSrcPANId(const TMacFrame *const pMacFrame, TMacPANId *const pSrcPANId)
{
  if (pMacFrame != NULL && pSrcPANId != NULL) {
      *pSrcPANId = pMacFrame->srcPANId.value;
      return SUCCESS;
    }
  return FAIL;
}

# 31 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$GetSrcPANId(const TMacFrame *const arg_0x40da2010, TMacPANId *const arg_0x40da2200){
#line 31
  unsigned char result;
#line 31

#line 31
  result = MacFrameFormatM$IMacFrame$GetSrcPANId(arg_0x40da2010, arg_0x40da2200);
#line 31

#line 31
  return result;
#line 31
}
#line 31








inline static  result_t MacDataM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40dd2df0, TMacPayloadLength *const arg_0x40dd1010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40dd2df0, arg_0x40dd1010);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 15
inline static  result_t MacDataM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40da99a0, bool *const arg_0x40da9b88){
#line 15
  unsigned char result;
#line 15

#line 15
  result = MacFrameFormatM$IMacFrame$GetAckRequest(arg_0x40da99a0, arg_0x40da9b88);
#line 15

#line 15
  return result;
#line 15
}
#line 15
#line 6
inline static  result_t MacDataM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40dac010, arg_0x40dac200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 180 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline result_t MacDataM$Receive(MacDataM$SF sf, TMacFrame *const pmacFrame)
{
  TMacFrameType FrameType;

#line 183
  MacDataM$IMacFrame$GetFrameType(pmacFrame, &FrameType);

  switch (FrameType) 
    {
      case MAC_FRAME_TYPE_DATA: 
        {
          TMacPANId SrcPANid;
          TMacDSN msnNumReceive;
          TMacAddress SrcAddr;
          TMacPANId DstPANid;
          TMacAddress DstAddr;
          TMacPayloadLength macPayloadLength;
          TMacLinkQuality lqi;
          uint8_t FramePayload[MAC_AMAX_MAC_FRAME_SIZE];
          bool AckRequest;


          MacDataM$IMacFrame$GetSequenceNumber(pmacFrame, &msnNumReceive);
          MacDataM$IMacFrame$GetAckRequest(pmacFrame, &AckRequest);

          MacDataM$IMacFrame$GetPayloadLength(pmacFrame, &macPayloadLength);

          if (macPayloadLength >= MAC_AMAX_MAC_FRAME_SIZE) 
            {

              return FAIL;
            }

          MacDataM$IMacFrame$GetSrcPANId(pmacFrame, &SrcPANid);
          MacDataM$IMacFrame$GetSrcAddress(pmacFrame, &SrcAddr);
          MacDataM$IMacFrame$GetDstPANId(pmacFrame, &DstPANid);
          MacDataM$IMacFrame$GetDstAddress(pmacFrame, &DstAddr);
          MacDataM$IMacFrame$GetPayload(pmacFrame, FramePayload);
          MacDataM$IMacFrame$GetLinkQuality(pmacFrame, &lqi);



          if (sf == MacDataM$PARENT_SF) {

              MacDataM$IMacDataParent$Indication(
              SrcPANid, 
              SrcAddr, 
              DstPANid, DstAddr, 
              macPayloadLength, FramePayload, 
              lqi, 0x0, 0x08);
            }
          else 
            {

              MacDataM$IMacDataOwn$Indication(
              SrcPANid, 
              SrcAddr, 
              DstPANid, DstAddr, 
              macPayloadLength, FramePayload, 
              lqi, 0x0, 0x08);
            }
        }
      break;
      case MAC_FRAME_TYPE_ACK: 
        if (MacDataM$state == MacDataM$WAIT_ACK) 
          {
            TMacDSN msnNumReceive;

#line 245
            MacDataM$IMacFrame$GetSequenceNumber(pmacFrame, &msnNumReceive);
            if (msnNumReceive == MacDataM$st_ackDSN) 
              {

                MacDataM$AckWaitTimer$Stop();
                MacDataM$state = MacDataM$IDLE;
                MacDataM$Confirm(MacDataM$currentSf, MacDataM$st_msduHandle, MAC_SUCCESS);
              }
          }
#line 253
      break;
      default: break;
    }

  return SUCCESS;
}

#line 349
static inline  result_t MacDataM$IMacReceiveOwn$Receive(TMacFrame *const pmacFrame)
{
  return MacDataM$Receive(MacDataM$OWN_SF, pmacFrame);
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
inline static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacReceive$Receive(TMacFrame *const arg_0x40ef5318){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacDataM$IMacReceiveOwn$Receive(arg_0x40ef5318);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 49 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static inline  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Receive(const uint8_t _sfIndex, TMacFrame *const pMacFrame)
{
  if (_sfIndex == 0) {
    return /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacReceive$Receive(pMacFrame);
    }
#line 53
  return SUCCESS;
}

# 51 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
inline static  result_t MacCAPM$Receive(const uint8_t arg_0x40f16ca0, TMacFrame *const arg_0x40f16e90){
#line 51
  unsigned char result;
#line 51

#line 51
  result = /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Receive(arg_0x40f16ca0, arg_0x40f16e90);
#line 51

#line 51
  return result;
#line 51
}
#line 51
#line 667
static inline  result_t MacCAPM$IMacReceive$Receive(TMacFrame *const pMacFrame)
{
  MacCAPM$TurnOffIfInactive(PHY_TRX_OFF);
  return MacCAPM$Receive(MacCAPM$currentSf, pMacFrame);
}

# 51 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
inline static  result_t NLME_JoinParentM$IMacASSOCIATE$Response(TMacExtendedAddress arg_0x408ee228, TMacShortAddress arg_0x408ee3d8, TMacStatus arg_0x408ee570, bool arg_0x408ee708){
#line 51
  unsigned char result;
#line 51

#line 51
  result = MacAssocCoordM$IMacASSOCIATE$Response(arg_0x408ee228, arg_0x408ee3d8, arg_0x408ee570, arg_0x408ee708);
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 69 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static inline void NwkAddressingM$calcNextEDAddr(void)
{
  uint8_t Rm = NwkAddressingM$NIB$getNwkMaxRouters();
  uint8_t Cm = NwkAddressingM$NIB$getNwkMaxChildren();

#line 73
  NwkAddressingM$edAddrNum++;
  if (NwkAddressingM$edAddrNum == Cm - Rm) 
    {
      NwkAddressingM$edAddrNum = 0;
    }
  NwkAddressingM$nextEDAddr = NwkAddressingM$IMacGET$GetmacShortAddress(NULL) + NwkAddressingM$cskip * Rm + NwkAddressingM$edAddrNum + 1;
}

#line 105
static inline  result_t NwkAddressingM$NwkAddressing$allocEdAddress(NwkAddr *newAddr)
{
  uint8_t Rm = NwkAddressingM$NIB$getNwkMaxRouters();
  uint8_t Cm = NwkAddressingM$NIB$getNwkMaxChildren();
  uint8_t i;
  bool found = FALSE;

  for (i = 0; i < Cm - Rm && !found; i++) 
    {
      if (NwkAddressingM$checkFree(NwkAddressingM$nextEDAddr)) 
        {
          *newAddr = NwkAddressingM$nextEDAddr;
          found = TRUE;
        }

      NwkAddressingM$calcNextEDAddr();
    }

  return found;
}

# 15 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
inline static  result_t NLME_JoinParentM$NwkAddressing$allocEdAddress(NwkAddr *arg_0x40914ba8){
#line 15
  unsigned char result;
#line 15

#line 15
  result = NwkAddressingM$NwkAddressing$allocEdAddress(arg_0x40914ba8);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 81 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static inline void NwkAddressingM$calcNextRouterAddr(void)
{
  uint8_t Rm = NwkAddressingM$NIB$getNwkMaxRouters();

#line 84
  NwkAddressingM$routerAddrNum++;
  if (NwkAddressingM$routerAddrNum == Rm) 
    {
      NwkAddressingM$routerAddrNum = 0;
    }
  NwkAddressingM$nextRouterAddr = NwkAddressingM$IMacGET$GetmacShortAddress(NULL) + NwkAddressingM$cskip * NwkAddressingM$routerAddrNum + 1;
}

#line 126
static inline  result_t NwkAddressingM$NwkAddressing$allocRouterAddress(NwkAddr *newAddr)
{
  uint8_t Rm = NwkAddressingM$NIB$getNwkMaxRouters();
  uint8_t i;
  bool found = FALSE;

  for (i = 0; i < Rm && !found; i++) 
    {
      if (NwkAddressingM$checkFree(NwkAddressingM$nextRouterAddr)) 
        {
          *newAddr = NwkAddressingM$nextRouterAddr;
          found = TRUE;
        }

      NwkAddressingM$calcNextRouterAddr();
    }


  return found;
}

# 17 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
inline static  result_t NLME_JoinParentM$NwkAddressing$allocRouterAddress(NwkAddr *arg_0x40913068){
#line 17
  unsigned char result;
#line 17

#line 17
  result = NwkAddressingM$NwkAddressing$allocRouterAddress(arg_0x40913068);
#line 17

#line 17
  return result;
#line 17
}
#line 17
# 45 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
static inline  result_t NLME_JoinParentM$IMacASSOCIATE$Indication(TMacExtendedAddress deviceAddr, 
TCapabilityInformation capability, 
bool securityUse, 
TACLEntry aclEntry)
{


  ;

  if (NLME_JoinParentM$state != NLME_JoinParentM$IDLE) 
    {

      ;
      return FAIL;
    }
  NLME_JoinParentM$joiningExtAddr = deviceAddr;
  NLME_JoinParentM$joiningCapability = capability;





  NLME_JoinParentM$addr_allocated = capability.deviceType == 1 ? 
  NLME_JoinParentM$NwkAddressing$allocRouterAddress(&NLME_JoinParentM$joiningNwkAddr) : 
  NLME_JoinParentM$NwkAddressing$allocEdAddress(&NLME_JoinParentM$joiningNwkAddr);




  NLME_JoinParentM$state = NLME_JoinParentM$WAIT_COMM_STATUS;
  if (NLME_JoinParentM$addr_allocated) 
    {
      {
#line 77
        NLME_JoinParentM$IMacASSOCIATE$Response(deviceAddr, NLME_JoinParentM$joiningNwkAddr, MAC_SUCCESS, FALSE) == SUCCESS;
      }
#line 77
      ;
    }
  else 
    {
      {
#line 81
        NLME_JoinParentM$IMacASSOCIATE$Response(deviceAddr, 0xFFFF, MAC_PAN_AT_CAPACITY, FALSE) == SUCCESS;
      }
#line 81
      ;
    }

  return SUCCESS;
}

# 38 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
inline static  result_t MacAssocCoordM$IMacASSOCIATE$Indication(TMacExtendedAddress arg_0x408ef610, TCapabilityInformation arg_0x408ef7c0, bool arg_0x408ef958, TACLEntry arg_0x408efaf8){
#line 38
  unsigned char result;
#line 38

#line 38
  result = NLME_JoinParentM$IMacASSOCIATE$Indication(arg_0x408ef610, arg_0x408ef7c0, arg_0x408ef958, arg_0x408efaf8);
#line 38

#line 38
  return result;
#line 38
}
#line 38
# 42 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static inline  result_t MacFrameFormatM$IMacFrame$GetSecurityEnabled(const TMacFrame *const pMacFrame, 
bool *const pSecurityEnabled)
{
  if (pMacFrame != NULL && pSecurityEnabled != NULL) {
      *pSecurityEnabled = 1 == pMacFrame->frameControl.lsb.value.securityEnabled;
      return SUCCESS;
    }
  return FAIL;
}

# 9 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocCoordM$IMacFrame$GetSecurityEnabled(const TMacFrame *const arg_0x40dacdd8, bool *const arg_0x40dab010){
#line 9
  unsigned char result;
#line 9

#line 9
  result = MacFrameFormatM$IMacFrame$GetSecurityEnabled(arg_0x40dacdd8, arg_0x40dab010);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacAssocCoordM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409bc8b0, const TMacAddress arg_0x409bca60){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacAddressM$IMacAddress$GetExtendedAddress(arg_0x409bc8b0, arg_0x409bca60);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 34 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocCoordM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40da2dd0, TMacAddress *const arg_0x40dd2010){
#line 34
  unsigned char result;
#line 34

#line 34
  result = MacFrameFormatM$IMacFrame$GetSrcAddress(arg_0x40da2dd0, arg_0x40dd2010);
#line 34

#line 34
  return result;
#line 34
}
#line 34








inline static  result_t MacAssocCoordM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40dd1530, uint8_t *const arg_0x40dd1718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40dd1530, arg_0x40dd1718);
#line 42

#line 42
  return result;
#line 42
}
#line 42
#line 39
inline static  result_t MacAssocCoordM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40dd2df0, TMacPayloadLength *const arg_0x40dd1010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40dd2df0, arg_0x40dd1010);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 6
inline static  result_t MacAssocCoordM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40dac010, arg_0x40dac200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 150 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static inline  result_t MacAssocCoordM$IAssocRequest$Receive(TMacFrame *const pMacFrame)
{
  result_t result;


  if (!MacAssocCoordM$macAssociationPermit) {
    return SUCCESS;
    }
  {
#line 158
    TMacFrameType frameType;

#line 159
    result = MacAssocCoordM$IMacFrame$GetFrameType(pMacFrame, &frameType);
    if (result != SUCCESS || frameType != MAC_FRAME_TYPE_COMMAND) {
      return FAIL;
      }
  }
  {
#line 164
    TMacPayloadLength payloadLength;

#line 165
    result = MacAssocCoordM$IMacFrame$GetPayloadLength(pMacFrame, &payloadLength);
    if (result != SUCCESS || payloadLength != MacAssocCoordM$ASSOC_REQUEST_PAYLOAD_LEN) {
      return FAIL;
      }
  }
  {
#line 170
    TCapabilityInformation capability;
    TMacExtendedAddress deviceExtAddress;
    bool securityUse;

#line 173
    {
#line 173
      uint8_t payload[MacAssocCoordM$ASSOC_REQUEST_PAYLOAD_LEN];

#line 174
      result = MacAssocCoordM$IMacFrame$GetPayload(pMacFrame, payload);
      if (result != SUCCESS || payload[0] != MAC_CMD_ASSOC_REQUEST) {
        return FAIL;
        }
#line 177
      capability = * (TCapabilityInformation *)(void *)&payload[1];
    }

    {
#line 180
      TMacAddress devAddress;

#line 181
      result = MacAssocCoordM$IMacFrame$GetSrcAddress(pMacFrame, &devAddress);
      result &= MacAssocCoordM$IMacAddress$GetExtendedAddress(&deviceExtAddress, devAddress);
      if (result != SUCCESS) {
#line 183
        return FAIL;
        }
    }
#line 185
    result = MacAssocCoordM$IMacFrame$GetSecurityEnabled(pMacFrame, &securityUse);
    if (SUCCESS != result) {
      return FAIL;
      }
    MacAssocCoordM$IMacASSOCIATE$Indication(deviceExtAddress, capability, securityUse, 0x0);
  }
  return SUCCESS;
}

# 506 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  result_t MacCAPM$SendFromPhyPool(const uint8_t sfIndex, 
const TPhyPoolHandle frameHandle, 
const uint8_t context)
{
  MacCAPM$TMacCAPSendState sendState = MacCAPM$cap[sfIndex].sendState;

#line 511
  if (MacCAPM$CAP_SEND_IDLE != sendState) {
    return FAIL;
    }
#line 513
  MacCAPM$cap[sfIndex].handle = frameHandle;
  return MacCAPM$SendPhyFrame(sfIndex, context);
}

# 21 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
inline static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendFromPhyPool(const uint8_t arg_0x40f85708, const TPhyPoolHandle arg_0x40f858c8, const uint8_t arg_0x40f85a78){
#line 21
  unsigned char result;
#line 21

#line 21
  result = MacCAPM$SendFromPhyPool(arg_0x40f85708, arg_0x40f858c8, arg_0x40f85a78);
#line 21

#line 21
  return result;
#line 21
}
#line 21
#line 37
static inline  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendFromPhyPool(uint8_t context, const TPhyPoolHandle frameHandle)
{
  return /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendFromPhyPool(0, frameHandle, context);
}

# 10 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  result_t MacScanBeaconM$CSMA$SendFromPhyPool(const TPhyPoolHandle arg_0x40ef9680){
#line 10
  unsigned char result;
#line 10

#line 10
  result = /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendFromPhyPool(3U, arg_0x40ef9680);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *MacScanBeaconM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010){
#line 8
  struct __nesc_unnamed4275 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b5c010);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 17 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanBeaconM.nc"
inline static  void MacScanBeaconM$packBeacon(TPhyFrame *arg_0x410b7698, uint64_t *arg_0x410b7840){
#line 17
  MacBeaconPackerM$packBeacon(arg_0x410b7698, arg_0x410b7840);
#line 17
}
#line 17
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacScanBeaconM$IPhyPool$Alloc(const uint8_t arg_0x40b5e6b8, const uint16_t arg_0x40b5e860, TPhyPoolHandle *const arg_0x40b5ea58){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyPoolM$IPhyPool$Alloc(arg_0x40b5e6b8, arg_0x40b5e860, arg_0x40b5ea58);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 58 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanBeaconM.nc"
static inline  void MacScanBeaconM$sendBcn(void)
{
  ;
  if (!MacScanBeaconM$beaconAllocated && MacScanBeaconM$IPhyPool$Alloc(PHY_AMAX_PHY_PACKET_SIZE, 0, &MacScanBeaconM$beaconHandle)) 
    {
      ;
      MacScanBeaconM$packBeacon(MacScanBeaconM$IPhyPool$GetFrame(MacScanBeaconM$beaconHandle), NULL);
      if (MacScanBeaconM$CSMA$SendFromPhyPool(MacScanBeaconM$beaconHandle)) {
          MacScanBeaconM$beaconAllocated = TRUE;
          ;
        }
      else {
#line 69
        MacScanBeaconM$IPhyPool$Free(MacScanBeaconM$beaconHandle);
        }
    }
}

# 42 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacScanBeaconM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40dd1530, uint8_t *const arg_0x40dd1718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40dd1530, arg_0x40dd1718);
#line 42

#line 42
  return result;
#line 42
}
#line 42
#line 39
inline static  result_t MacScanBeaconM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40dd2df0, TMacPayloadLength *const arg_0x40dd1010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40dd2df0, arg_0x40dd1010);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 6
inline static  result_t MacScanBeaconM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40dac010, arg_0x40dac200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacScanBeaconM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dcacf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dcacf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacScanBeaconM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e84d90){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e84d90);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 30 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanBeaconM.nc"
static inline  result_t MacScanBeaconM$IMacReceive$Receive(TMacFrame *const pmacFrame)
{
  if (MacScanBeaconM$IMacSuperframeAttr$GetmacBeaconOrder(NULL) == 15 && MacScanBeaconM$IMacCommonAttr$GetmacPANId(NULL) != 0xFFFF) 
    {
      TMacFrameType FrameType;

      MacScanBeaconM$IMacFrame$GetFrameType(pmacFrame, &FrameType);

      if (FrameType == MAC_FRAME_TYPE_COMMAND) 
        {
          TMacPayloadLength payloadLength;

          MacScanBeaconM$IMacFrame$GetPayloadLength(pmacFrame, &payloadLength);

          if (payloadLength == 1) 
            {
              uint8_t payload[1];

#line 47
              MacScanBeaconM$IMacFrame$GetPayload(pmacFrame, payload);

              if (payload[0] == MAC_CMD_BEACON_REQUEST) {
                TOS_post(MacScanBeaconM$sendBcn);
                }
            }
        }
    }
  return SUCCESS;
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
inline static  result_t MacRxM$DataReceive$Receive(TMacFrame *const arg_0x40ef5318){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacScanBeaconM$IMacReceive$Receive(arg_0x40ef5318);
#line 5
  result = rcombine(result, MacAssocCoordM$IAssocRequest$Receive(arg_0x40ef5318));
#line 5
  result = rcombine(result, MacCAPM$IMacReceive$Receive(arg_0x40ef5318));
#line 5
  result = rcombine(result, MacPendingM$IMacReceive$Receive(arg_0x40ef5318));
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 43 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacRxM.nc"
static inline   result_t MacRxM$BeaconReceive$default$Receive(TMacFrame *const pMacFrame)
{
  return SUCCESS;
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
inline static  result_t MacRxM$BeaconReceive$Receive(TMacFrame *const arg_0x40ef5318){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacRxM$BeaconReceive$default$Receive(arg_0x40ef5318);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacRxM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40dac010, TMacFrameType *const arg_0x40dac200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40dac010, arg_0x40dac200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 15 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   result_t MacFrameFormatM$IPhyFrame$ReadOctet(TPhyFrame *const arg_0x40b3f7d8, uint8_t *const arg_0x40b3f9b8){
#line 15
  unsigned char result;
#line 15

#line 15
  result = PhyFrameM$IPhyFrame$ReadOctet(arg_0x40b3f7d8, arg_0x40b3f9b8);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 280 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static inline  result_t MacFrameFormatM$IMacFrame$SetTimeStamp(TMacFrame *const pMacFrame, TMacTimeStamp timeStamp)
{
  if (pMacFrame != NULL) {
      pMacFrame->timeStamp = timeStamp;
      return SUCCESS;
    }
  return FAIL;
}

# 109 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static inline   result_t PhyFrameM$IPhyFrame$GetTimeStamp(const TPhyFrame *const pPhyFrame, TPhyTimeStamp *const pStamp)
{
  if (pPhyFrame != NULL && pStamp != NULL) {
      *pStamp = pPhyFrame->timeStamp;
      return SUCCESS;
    }
  return FAIL;
}

# 27 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   result_t MacFrameFormatM$IPhyFrame$GetTimeStamp(const TPhyFrame *const arg_0x40b3ca90, TPhyTimeStamp *const arg_0x40b3cc78){
#line 27
  unsigned char result;
#line 27

#line 27
  result = PhyFrameM$IPhyFrame$GetTimeStamp(arg_0x40b3ca90, arg_0x40b3cc78);
#line 27

#line 27
  return result;
#line 27
}
#line 27
# 89 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static inline   result_t PhyFrameM$IPhyFrame$CheckCRC(TPhyFrame *const pPhyFrame)
{
  if (pPhyFrame != NULL) {
      if (pPhyFrame->data[pPhyFrame->data[0]] & 0x80) {

          return SUCCESS;
        }
    }
  return FAIL;
}

# 23 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   result_t MacFrameFormatM$IPhyFrame$CheckCRC(TPhyFrame *const arg_0x40b3de40){
#line 23
  unsigned char result;
#line 23

#line 23
  result = PhyFrameM$IPhyFrame$CheckCRC(arg_0x40b3de40);
#line 23

#line 23
  return result;
#line 23
}
#line 23
# 61 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static inline   result_t PhyFrameM$IPhyFrame$ResetPosition(TPhyFrame *const pPhyFrame)
{
  if (pPhyFrame != NULL) {
      pPhyFrame->current = 0;
      return SUCCESS;
    }
  return FAIL;
}

# 17 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   result_t MacFrameFormatM$IPhyFrame$ResetPosition(TPhyFrame *const arg_0x40b3fed8){
#line 17
  unsigned char result;
#line 17

#line 17
  result = PhyFrameM$IPhyFrame$ResetPosition(arg_0x40b3fed8);
#line 17

#line 17
  return result;
#line 17
}
#line 17
# 364 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static inline  result_t MacFrameFormatM$IMacFrame$UnPack(TMacFrame *const pMacFrame, TPhyFrame *const pPhyFrame)
{
  result_t result;

#line 367
  if (NULL == pPhyFrame) {
#line 367
    return FAIL;
    }
#line 368
  result = MacFrameFormatM$IPhyFrame$ResetPosition(pPhyFrame);
  result &= MacFrameFormatM$IPhyFrame$CheckCRC(pPhyFrame);
  if (result != SUCCESS || NULL == pMacFrame) {
#line 370
    return FAIL;
    }
#line 371
  {
#line 371
    TPhyTimeStamp timeStamp;

#line 372
    result = MacFrameFormatM$IPhyFrame$GetTimeStamp(pPhyFrame, &timeStamp);
    result &= MacFrameFormatM$IMacFrame$SetTimeStamp(pMacFrame, (TMacTimeStamp )timeStamp);
  }
  result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, & pMacFrame->frameControl.lsb.raw);
  result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, & pMacFrame->frameControl.msb.raw);
  result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->sequenceNumber.raw[0]);
  if (MAC_ADDRESS_NOT_PRESENT != pMacFrame->frameControl.msb.value.dstAddressMode) {
      result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->dstPANId.raw[0]);
      result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->dstPANId.raw[1]);
      if (MAC_ADDRESS_SHORT == pMacFrame->frameControl.msb.value.dstAddressMode) {
          result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->dstAddress._short.raw[0]);
          result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->dstAddress._short.raw[1]);
        }
      else 
#line 384
        {
          uint8_t i;

#line 386
          for (i = 0; i < MAC_EXTENDED_ADDRESS_LENGTH; i++) 
            result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->dstAddress._extended.raw[i]);
        }
    }
  if (MAC_ADDRESS_NOT_PRESENT != pMacFrame->frameControl.msb.value.srcAddressMode) {
      bool intraPAN;

#line 392
      result &= MacFrameFormatM$IMacFrame$GetIntraPAN(pMacFrame, &intraPAN);
      if (!intraPAN) {
          result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->srcPANId.raw[0]);
          result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->srcPANId.raw[1]);
        }
      if (MAC_ADDRESS_SHORT == pMacFrame->frameControl.msb.value.srcAddressMode) {
          result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->srcAddress._short.raw[0]);
          result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->srcAddress._short.raw[1]);
        }
      else 
#line 400
        {
          uint8_t i;

#line 402
          for (i = 0; i < MAC_EXTENDED_ADDRESS_LENGTH; i++) 
            result &= MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->srcAddress._extended.raw[i]);
        }
    }
  if (SUCCESS != result) {
    return FAIL;
    }
#line 408
  {
#line 408
    int16_t i = -1;

#line 409
    do {
        i += 1;
        result = MacFrameFormatM$IPhyFrame$ReadOctet(pPhyFrame, &pMacFrame->payload.raw[i]);
      }
    while (
#line 412
    result == SUCCESS);
    i -= MAC_FRAME_CHECK_SEQUENCE_LENGTH;
    pMacFrame->payload.length = i > 0 ? i : 0;
  }
  return SUCCESS;
}

# 45 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacRxM$IMacFrame$UnPack(TMacFrame *const arg_0x40dcf3a0, TPhyFrame *const arg_0x40dcf590){
#line 45
  unsigned char result;
#line 45

#line 45
  result = MacFrameFormatM$IMacFrame$UnPack(arg_0x40dcf3a0, arg_0x40dcf590);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *MacRxM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010){
#line 8
  struct __nesc_unnamed4275 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b5c010);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 21 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacRxM.nc"
static inline  result_t MacRxM$IPhyRxDATA$Indication(const TPhyPoolHandle handle)
{
  TPhyFrame *pPhyFrame;
  TMacFrameType frame_type;

  pPhyFrame = MacRxM$IPhyPool$GetFrame(handle);

  if (NULL == pPhyFrame) {
#line 28
    return FAIL;
    }
#line 29
  {
#line 29
    TMacFrame macFrame;

#line 30
    if (SUCCESS != MacRxM$IMacFrame$UnPack(&macFrame, pPhyFrame)) {
        return FAIL;
      }

    MacRxM$IMacFrame$GetFrameType(&macFrame, &frame_type);
    if (frame_type == MAC_FRAME_TYPE_BEACON) {
      MacRxM$BeaconReceive$Receive(&macFrame);
      }
    else {
#line 38
      MacRxM$DataReceive$Receive(&macFrame);
      }
  }
#line 40
  return SUCCESS;
}

# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyRxDATA.nc"
inline static  result_t PhyCC2420M$IPhyRxDATA$Indication(const TPhyPoolHandle arg_0x40b26580){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacRxM$IPhyRxDATA$Indication(arg_0x40b26580);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 337 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  void PhyCC2420M$TaskRxIndication(void)
{
  PhyCC2420M$TPhyRadioState _radioState;
  TPhyPoolHandle rxHandle = 0xff;
  uint8_t *pData = NULL;

#line 342
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 342
    {
      _radioState = PhyCC2420M$radioState;
      if (PhyCC2420M$PHY_RADIO_RX_INDICATION == _radioState) {
          rxHandle = PhyCC2420M$g_rxHandle;
          pData = PhyCC2420M$radio.pData;
          PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH;
        }
    }
#line 349
    __nesc_atomic_end(__nesc_atomic); }
  if (PhyCC2420M$PHY_RADIO_RX_INDICATION == _radioState) {
      PhyCC2420M$IPhyRxDATA$Indication(rxHandle);
      return;
    }
  return;
}

static inline   result_t PhyCC2420M$IChipconFIFO$RXFIFODone(uint8_t length, uint8_t *data)
{
  PhyCC2420M$TPhyRadioState _radioState;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      _radioState = PhyCC2420M$radioState;
      if (PhyCC2420M$radioState == PhyCC2420M$PHY_RADIO_RX_FIFO_READ) {
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_INDICATION;
        }
    }
#line 365
    __nesc_atomic_end(__nesc_atomic); }
#line 365
  if (_radioState == PhyCC2420M$PHY_RADIO_RX_FIFO_READ) {
      PhyCC2420M$flushRXFIFO();
      if (PhyCC2420M$radio.pData == data && length <= PHY_AMAX_PHY_PACKET_SIZE) {
          if (TOS_post(PhyCC2420M$TaskRxIndication)) {
            return SUCCESS;
            }
        }
#line 371
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 371
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH;
#line 371
        __nesc_atomic_end(__nesc_atomic); }
    }
  return FAIL;
}

# 39 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
inline static   result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t arg_0x40b989a8, uint8_t *arg_0x40b98b48){
#line 39
  unsigned char result;
#line 39

#line 39
  result = PhyCC2420M$IChipconFIFO$RXFIFODone(arg_0x40b989a8, arg_0x40b98b48);
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 322 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static inline  void HPLCC2420M$signalRXFIFO(void)
#line 322
{
  uint8_t _rxlen;
  uint8_t *_rxbuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 326
    {
      _rxlen = HPLCC2420M$rxlen;
      _rxbuf = HPLCC2420M$rxbuf;
      HPLCC2420M$f.rxbufBusy = FALSE;
    }
#line 330
    __nesc_atomic_end(__nesc_atomic); }

  HPLCC2420M$HPLCC2420FIFO$RXFIFODone(_rxlen, _rxbuf);
}

# 185 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$isRxIntrPending(void){
#line 185
  unsigned char result;
#line 185

#line 185
  result = HPLUSART0M$USARTControl$isRxIntrPending();
#line 185

#line 185
  return result;
#line 185
}
#line 185
# 335 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static inline   result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t length, uint8_t *data)
#line 335
{
  uint8_t i;
  bool returnFail = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 339
    {
      if (HPLCC2420M$f.rxbufBusy) {
        returnFail = TRUE;
        }
      else {
#line 343
        HPLCC2420M$f.rxbufBusy = TRUE;
        }
    }
#line 345
    __nesc_atomic_end(__nesc_atomic); }
  if (returnFail) {
    return FAIL;
    }
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 350
        {
          HPLCC2420M$f.busy = TRUE;
          HPLCC2420M$rxbuf = data;
          TOSH_CLR_RADIO_CSN_PIN();

          HPLCC2420M$USARTControl$isTxIntrPending();
          HPLCC2420M$USARTControl$rx();
          HPLCC2420M$USARTControl$tx(0x3F | 0x40);
          while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
          HPLCC2420M$rxlen = HPLCC2420M$USARTControl$rx();
          HPLCC2420M$USARTControl$tx(0);
          while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;

          HPLCC2420M$rxlen = HPLCC2420M$USARTControl$rx();
        }
#line 364
        __nesc_atomic_end(__nesc_atomic); }
      if (HPLCC2420M$rxlen > 0) {
          HPLCC2420M$rxbuf[0] = HPLCC2420M$rxlen;

          HPLCC2420M$rxlen++;

          if (HPLCC2420M$rxlen > length) {
#line 370
            HPLCC2420M$rxlen = length;
            }
#line 371
          {
#line 371
            register uint8_t *trxbuf = HPLCC2420M$rxbuf + 1;

#line 372
            for (i = HPLCC2420M$rxlen - 1; 0 < i; i--) {
                U0TXBUF = 0;
                while (!(IFG1 & (1 << 6))) ;
                IFG1 &= ~(1 << 6);
                * trxbuf++ = U0RXBUF;
              }
          }
        }
      TOSH_SET_RADIO_CSN_PIN();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 381
        HPLCC2420M$f.busy = FALSE;
#line 381
        __nesc_atomic_end(__nesc_atomic); }
      HPLCC2420M$BusArbitration$releaseBus();
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 385
        HPLCC2420M$f.rxbufBusy = FALSE;
#line 385
        __nesc_atomic_end(__nesc_atomic); }
      return FAIL;
    }
  if (TOS_post(HPLCC2420M$signalRXFIFO) == FAIL) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 389
        HPLCC2420M$f.rxbufBusy = FALSE;
#line 389
        __nesc_atomic_end(__nesc_atomic); }
      return FAIL;
    }
  return SUCCESS;
}

# 19 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
inline static   result_t PhyCC2420M$IChipconFIFO$readRXFIFO(uint8_t arg_0x40b99a48, uint8_t *arg_0x40b99be8){
#line 19
  unsigned char result;
#line 19

#line 19
  result = HPLCC2420M$HPLCC2420FIFO$readRXFIFO(arg_0x40b99a48, arg_0x40b99be8);
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   uint8_t *PhyCC2420M$IPhyFrame$GetPPDU(const TPhyFrame *const arg_0x40b41c08){
#line 5
  unsigned char *result;
#line 5

#line 5
  result = PhyFrameM$IPhyFrame$GetPPDU(arg_0x40b41c08);
#line 5

#line 5
  return result;
#line 5
}
#line 5
#line 25
inline static   result_t PhyCC2420M$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b3c3c8, TPhyTimeStamp arg_0x40b3c550){
#line 25
  unsigned char result;
#line 25

#line 25
  result = PhyFrameM$IPhyFrame$SetTimeStamp(arg_0x40b3c3c8, arg_0x40b3c550);
#line 25

#line 25
  return result;
#line 25
}
#line 25
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *PhyCC2420M$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b5c010){
#line 8
  struct __nesc_unnamed4275 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b5c010);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 299 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  void PhyCC2420M$fifop_handler(void)
{
  PhyCC2420M$TPhyRadioState _radioState;

#line 302
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 302
    {
      _radioState = PhyCC2420M$radioState;
      if (PhyCC2420M$radioState == PhyCC2420M$PHY_RADIO_RX_FRAME_RECV) {
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_END;
        }
    }
#line 307
    __nesc_atomic_end(__nesc_atomic); }
#line 307
  if (PhyCC2420M$PHY_RADIO_RX_FRAME_RECV == _radioState) {
      if (TOSH_READ_CC_FIFO_PIN()) {
          TPhyFrame *pPhyFrame = PhyCC2420M$IPhyPool$GetFrame(PhyCC2420M$g_rxHandle);

#line 310
          if (NULL != pPhyFrame) {
              result_t result = PhyCC2420M$IPhyFrame$SetTimeStamp(pPhyFrame, PhyCC2420M$radio.timeStamp);

#line 312
              PhyCC2420M$radio.pData = PhyCC2420M$IPhyFrame$GetPPDU(pPhyFrame);
              if (SUCCESS == result && NULL != PhyCC2420M$radio.pData) {
                  if (SUCCESS == PhyCC2420M$IChipconFIFO$readRXFIFO(MAX_PPDU_LENGTH, PhyCC2420M$radio.pData)) {
                      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 315
                        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_FIFO_READ;
#line 315
                        __nesc_atomic_end(__nesc_atomic); }
                      return;
                    }
                }
            }
        }
      PhyCC2420M$flushRXFIFO();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 322
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH;
#line 322
        __nesc_atomic_end(__nesc_atomic); }
    }
  return;
}

static inline   result_t PhyCC2420M$IChipconFIFOP$fired(void)
{
  if (TOS_post(PhyCC2420M$fifop_handler)) {
      return SUCCESS;
    }
  PhyCC2420M$flushRXFIFO();
  /* atomic removed: atomic calls only */
#line 333
  PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH;
  return SUCCESS;
}

# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
inline static   result_t HPLCC2420InterruptM$FIFOP$fired(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = PhyCC2420M$IChipconFIFOP$fired();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 89 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420InterruptM.nc"
static inline   void HPLCC2420InterruptM$FIFOPInterrupt$fired(void)
#line 89
{
  result_t val = SUCCESS;

#line 91
  HPLCC2420InterruptM$FIFOPInterrupt$clear();
  val = HPLCC2420InterruptM$FIFOP$fired();
  if (val == FAIL) {
      HPLCC2420InterruptM$FIFOPInterrupt$disable();
      HPLCC2420InterruptM$FIFOPInterrupt$clear();
    }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port10$fired(void){
#line 59
  HPLCC2420InterruptM$FIFOPInterrupt$fired();
#line 59
}
#line 59
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacFrameFormatM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x409bad90){
#line 16
  unsigned char result;
#line 16

#line 16
  result = MacAddressM$IMacAddress$SetEmptyAddress(arg_0x409bad90);
#line 16

#line 16
  return result;
#line 16
}
#line 16
#line 12
inline static  result_t MacFrameFormatM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x409ba010, const TMacShortAddress arg_0x409ba1c8){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacAddressM$IMacAddress$SetShortAddress(arg_0x409ba010, arg_0x409ba1c8);
#line 12

#line 12
  return result;
#line 12
}
#line 12


inline static  result_t MacFrameFormatM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x409ba6d0, const TMacExtendedAddress arg_0x409ba890){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacAddressM$IMacAddress$SetExtendedAddress(arg_0x409ba6d0, arg_0x409ba890);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 36 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocCoordM$IMacFrame$SetPayload(TMacFrame *const arg_0x40dd2510, const uint8_t *const arg_0x40dd2728, TMacPayloadLength arg_0x40dd28c0){
#line 36
  unsigned char result;
#line 36

#line 36
  result = MacFrameFormatM$IMacFrame$SetPayload(arg_0x40dd2510, arg_0x40dd2728, arg_0x40dd28c0);
#line 36

#line 36
  return result;
#line 36
}
#line 36
#line 30
inline static  result_t MacAssocCoordM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40da3908, const TMacPANId arg_0x40da3ab8){
#line 30
  unsigned char result;
#line 30

#line 30
  result = MacFrameFormatM$IMacFrame$SetSrcPANId(arg_0x40da3908, arg_0x40da3ab8);
#line 30

#line 30
  return result;
#line 30
}
#line 30
#line 24
inline static  result_t MacAssocCoordM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40da6c60, const TMacPANId arg_0x40da6e10){
#line 24
  unsigned char result;
#line 24

#line 24
  result = MacFrameFormatM$IMacFrame$SetDstPANId(arg_0x40da6c60, arg_0x40da6e10);
#line 24

#line 24
  return result;
#line 24
}
#line 24



inline static  result_t MacAssocCoordM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40da5b38, const TMacAddress arg_0x40da5ce8){
#line 27
  unsigned char result;
#line 27

#line 27
  result = MacFrameFormatM$IMacFrame$SetDstAddress(arg_0x40da5b38, arg_0x40da5ce8);
#line 27

#line 27
  return result;
#line 27
}
#line 27






inline static  result_t MacAssocCoordM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40da2700, const TMacAddress arg_0x40da28b0){
#line 33
  unsigned char result;
#line 33

#line 33
  result = MacFrameFormatM$IMacFrame$SetSrcAddress(arg_0x40da2700, arg_0x40da28b0);
#line 33

#line 33
  return result;
#line 33
}
#line 33
#line 20
inline static  result_t MacAssocCoordM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40da8e60, const TMacSequenceNumber arg_0x40da6030){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacFrameFormatM$IMacFrame$SetSequenceNumber(arg_0x40da8e60, arg_0x40da6030);
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 19 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacAssocCoordM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dc6828){
#line 19
  enum __nesc_unnamed4280 result;
#line 19

#line 19
  result = MacCommonAttrM$IMacCommonAttr$SetmacDSN(arg_0x40dc6828);
#line 19

#line 19
  return result;
#line 19
}
#line 19
#line 18
inline static  TMacDSN MacAssocCoordM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dc63a0){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacCommonAttrM$IMacCommonAttr$GetmacDSN(arg_0x40dc63a0);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 17 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocCoordM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40da80a8, const bool arg_0x40da8250){
#line 17
  unsigned char result;
#line 17

#line 17
  result = MacFrameFormatM$IMacFrame$SetIntraPAN(arg_0x40da80a8, arg_0x40da8250);
#line 17

#line 17
  return result;
#line 17
}
#line 17
#line 14
inline static  result_t MacAssocCoordM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40da92d8, const bool arg_0x40da9480){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacFrameFormatM$IMacFrame$SetAckRequest(arg_0x40da92d8, arg_0x40da9480);
#line 14

#line 14
  return result;
#line 14
}
#line 14
#line 11
inline static  result_t MacAssocCoordM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40dab510, const bool arg_0x40dab6b8){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacFrameFormatM$IMacFrame$SetFramePending(arg_0x40dab510, arg_0x40dab6b8);
#line 11

#line 11
  return result;
#line 11
}
#line 11
#line 8
inline static  result_t MacAssocCoordM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40dac708, const bool arg_0x40dac8b0){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacFrameFormatM$IMacFrame$SetSecurityEnabled(arg_0x40dac708, arg_0x40dac8b0);
#line 8

#line 8
  return result;
#line 8
}
#line 8
#line 5
inline static  result_t MacAssocCoordM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40dad8e8, const TMacFrameType arg_0x40dada98){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacFrameFormatM$IMacFrame$SetFrameType(arg_0x40dad8e8, arg_0x40dada98);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 37 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static inline result_t MacAssocCoordM$MakeAssocResponse(const TMacExtendedAddress deviceAddress, 
const TMacShortAddress assocShortAddress, 
const TMacStatus status, 
const bool securityEnable, 
TMacFrame *const pMacFrame)
{
  result_t result = MacAssocCoordM$IMacFrame$SetFrameType(pMacFrame, MAC_FRAME_TYPE_COMMAND);

#line 44
  result &= MacAssocCoordM$IMacFrame$SetSecurityEnabled(pMacFrame, securityEnable);
  result &= MacAssocCoordM$IMacFrame$SetFramePending(pMacFrame, FALSE);
  result &= MacAssocCoordM$IMacFrame$SetAckRequest(pMacFrame, FALSE);
  result &= MacAssocCoordM$IMacFrame$SetIntraPAN(pMacFrame, FALSE);
  if (result != SUCCESS) {
#line 48
    return FAIL;
    }
#line 49
  {
#line 49
    TMacStatus macStatus;
    TMacSequenceNumber sequenceNumber = MacAssocCoordM$IMacCommonAttr$GetmacDSN(&macStatus);

#line 51
    if (macStatus != MAC_SUCCESS) {
#line 51
      return FAIL;
      }
#line 52
    macStatus = MacAssocCoordM$IMacCommonAttr$SetmacDSN(sequenceNumber + 1);
    result = MacAssocCoordM$IMacFrame$SetSequenceNumber(pMacFrame, sequenceNumber);
    if (macStatus != MAC_SUCCESS || result != SUCCESS) {
#line 54
      return FAIL;
      }
  }
#line 56
  {
#line 56
    TMacAddress srcAddress;
#line 56
    TMacAddress dstAddress;

#line 57
    result = MacAssocCoordM$IMacAddress$SetExtendedAddress(&srcAddress, info_param.MAC_ADDRESS);
    result &= MacAssocCoordM$IMacAddress$SetExtendedAddress(&dstAddress, deviceAddress);
    if (SUCCESS != result) {
#line 59
      return result;
      }
#line 60
    result = MacAssocCoordM$IMacFrame$SetSrcAddress(pMacFrame, srcAddress);
    result &= MacAssocCoordM$IMacFrame$SetDstAddress(pMacFrame, dstAddress);
    if (SUCCESS != result) {
#line 62
      return FAIL;
      }
  }
#line 64
  {
#line 64
    TMacStatus macStatus;
    TMacPANId macPANId = MacAssocCoordM$IMacCommonAttr$GetmacPANId(&macStatus);

#line 66
    if (MAC_SUCCESS != macStatus) {
#line 66
      return FAIL;
      }
#line 67
    result = MacAssocCoordM$IMacFrame$SetDstPANId(pMacFrame, macPANId);
    result &= MacAssocCoordM$IMacFrame$SetSrcPANId(pMacFrame, macPANId);
    if (SUCCESS != result) {
#line 69
      return result;
      }
  }
#line 71
  {
#line 71
    uint8_t payload[MacAssocCoordM$ASSOC_RESPONSE_PAYLOAD_LEN];

#line 72
    payload[0] = (uint8_t )MAC_CMD_ASSOC_RESPONSE;
    payload[1] = * (uint8_t *)&assocShortAddress;
    payload[2] = *((uint8_t *)&assocShortAddress + 1);
    payload[3] = (uint8_t )status;
    result = MacAssocCoordM$IMacFrame$SetPayload(pMacFrame, payload, MacAssocCoordM$ASSOC_RESPONSE_PAYLOAD_LEN);
  }
  return result;
}

# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  result_t MacAssocCoordM$IMacSendOwn$Send(const TMacFrame *const arg_0x40ef9010, const TUniData arg_0x40ef91c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(1U, arg_0x40ef9010, arg_0x40ef91c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 81 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static inline void MacAssocCoordM$FailureResponse(TMacStatus status, TMacExtendedAddress deviceAddress)
{
  TMacStatus macStatus;
  TMacAddress dstAddress;
#line 84
  TMacAddress srcAddress;
  TMacPANId panId = MacAssocCoordM$IMacCommonAttr$GetmacPANId(&macStatus);
  result_t result = MacAssocCoordM$IMacAddress$SetExtendedAddress(&dstAddress, deviceAddress);

#line 87
  result &= MacAssocCoordM$IMacAddress$SetExtendedAddress(&srcAddress, info_param.MAC_ADDRESS);
  if (result == SUCCESS && macStatus == MAC_SUCCESS) {
    MacAssocCoordM$IMacCOMM_STATUS$Indication(panId, srcAddress, dstAddress, status);
    }
}

# 178 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port11$clear(void)
#line 178
{
#line 178
  MSP430InterruptM$P1IFG &= ~(1 << 1);
}

#line 94
static inline    void MSP430InterruptM$Port11$default$fired(void)
#line 94
{
#line 94
  MSP430InterruptM$Port11$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port11$fired(void){
#line 59
  MSP430InterruptM$Port11$default$fired();
#line 59
}
#line 59
# 179 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port12$clear(void)
#line 179
{
#line 179
  MSP430InterruptM$P1IFG &= ~(1 << 2);
}

#line 95
static inline    void MSP430InterruptM$Port12$default$fired(void)
#line 95
{
#line 95
  MSP430InterruptM$Port12$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port12$fired(void){
#line 59
  MSP430InterruptM$Port12$default$fired();
#line 59
}
#line 59
# 180 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port13$clear(void)
#line 180
{
#line 180
  MSP430InterruptM$P1IFG &= ~(1 << 3);
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOInterrupt$clear(void){
#line 40
  MSP430InterruptM$Port13$clear();
#line 40
}
#line 40
# 149 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port13$disable(void)
#line 149
{
#line 149
  MSP430InterruptM$P1IE &= ~(1 << 3);
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOInterrupt$disable(void){
#line 35
  MSP430InterruptM$Port13$disable();
#line 35
}
#line 35
# 43 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
inline static   result_t PhyCC2420M$IChipconFIFOSignal$startWait(bool arg_0x40b92010){
#line 43
  unsigned char result;
#line 43

#line 43
  result = HPLCC2420InterruptM$FIFO$startWait(arg_0x40b92010);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 274 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline   result_t PhyCC2420M$IChipconFIFOSignal$fired(void)
{
  PhyCC2420M$TPhyRadioState _radioState;

  /* atomic removed: atomic calls only */
#line 277
  _radioState = PhyCC2420M$radioState;
  if (!TOSH_READ_CC_FIFO_PIN() && TOSH_READ_CC_FIFOP_PIN()) {
      PhyCC2420M$flushRXFIFO();
      /* atomic removed: atomic calls only */
#line 280
      PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH;
      return SUCCESS;
    }
  switch (_radioState) {
      case PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH: 
        PhyCC2420M$IChipconFIFOSignal$startWait(FALSE);
      if (TOSH_READ_CC_FIFO_PIN()) {
          /* atomic removed: atomic calls only */
#line 287
          PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_FRAME_RECV;
          break;
        }
      case PhyCC2420M$PHY_RADIO_RX_FRAME_RECV: 
        PhyCC2420M$IChipconFIFOSignal$startWait(TRUE);
      /* atomic removed: atomic calls only */
#line 292
      PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH;
      break;
      default: ;
    }
  return SUCCESS;
}

# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
inline static   result_t HPLCC2420InterruptM$FIFO$fired(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = PhyCC2420M$IChipconFIFOSignal$fired();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 130 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420InterruptM.nc"
static inline   void HPLCC2420InterruptM$FIFOInterrupt$fired(void)
#line 130
{
  result_t val = SUCCESS;

#line 132
  HPLCC2420InterruptM$FIFOInterrupt$clear();
  val = HPLCC2420InterruptM$FIFO$fired();
  if (val == FAIL) {
      HPLCC2420InterruptM$FIFOInterrupt$disable();
      HPLCC2420InterruptM$FIFOInterrupt$clear();
    }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port13$fired(void){
#line 59
  HPLCC2420InterruptM$FIFOInterrupt$fired();
#line 59
}
#line 59
# 181 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port14$clear(void)
#line 181
{
#line 181
  MSP430InterruptM$P1IFG &= ~(1 << 4);
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$CCAInterrupt$clear(void){
#line 40
  MSP430InterruptM$Port14$clear();
#line 40
}
#line 40
# 150 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port14$disable(void)
#line 150
{
#line 150
  MSP430InterruptM$P1IE &= ~(1 << 4);
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$CCAInterrupt$disable(void){
#line 35
  MSP430InterruptM$Port14$disable();
#line 35
}
#line 35
# 181 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420InterruptM.nc"
static inline    result_t HPLCC2420InterruptM$CCA$default$fired(void)
#line 181
{
#line 181
  return FAIL;
}

# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
inline static   result_t HPLCC2420InterruptM$CCA$fired(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = HPLCC2420InterruptM$CCA$default$fired();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 171 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420InterruptM.nc"
static inline   void HPLCC2420InterruptM$CCAInterrupt$fired(void)
#line 171
{
  result_t val = SUCCESS;

#line 173
  HPLCC2420InterruptM$CCAInterrupt$clear();
  val = HPLCC2420InterruptM$CCA$fired();
  if (val == FAIL) {
      HPLCC2420InterruptM$CCAInterrupt$disable();
      HPLCC2420InterruptM$CCAInterrupt$clear();
    }
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port14$fired(void){
#line 59
  HPLCC2420InterruptM$CCAInterrupt$fired();
#line 59
}
#line 59
# 182 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port15$clear(void)
#line 182
{
#line 182
  MSP430InterruptM$P1IFG &= ~(1 << 5);
}

#line 98
static inline    void MSP430InterruptM$Port15$default$fired(void)
#line 98
{
#line 98
  MSP430InterruptM$Port15$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port15$fired(void){
#line 59
  MSP430InterruptM$Port15$default$fired();
#line 59
}
#line 59
# 183 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port16$clear(void)
#line 183
{
#line 183
  MSP430InterruptM$P1IFG &= ~(1 << 6);
}

#line 99
static inline    void MSP430InterruptM$Port16$default$fired(void)
#line 99
{
#line 99
  MSP430InterruptM$Port16$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port16$fired(void){
#line 59
  MSP430InterruptM$Port16$default$fired();
#line 59
}
#line 59
# 184 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port17$clear(void)
#line 184
{
#line 184
  MSP430InterruptM$P1IFG &= ~(1 << 7);
}

#line 100
static inline    void MSP430InterruptM$Port17$default$fired(void)
#line 100
{
#line 100
  MSP430InterruptM$Port17$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port17$fired(void){
#line 59
  MSP430InterruptM$Port17$default$fired();
#line 59
}
#line 59
# 186 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port20$clear(void)
#line 186
{
#line 186
  MSP430InterruptM$P2IFG &= ~(1 << 0);
}

#line 102
static inline    void MSP430InterruptM$Port20$default$fired(void)
#line 102
{
#line 102
  MSP430InterruptM$Port20$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port20$fired(void){
#line 59
  MSP430InterruptM$Port20$default$fired();
#line 59
}
#line 59
# 110 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   uint8_t ZigNet2SpiM$USARTControl$rx(void){
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
inline static   result_t ZigNet2SpiM$USARTControl$isRxIntrPending(void){
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
# 194 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static inline   result_t UniSART1M$USARTControl$tx(uint8_t data)
#line 194
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 195
    {
      UniSART1M$U1TXBUF = data;
    }
#line 197
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 103 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSARTControl.nc"
inline static   result_t ZigNet2SpiM$USARTControl$tx(uint8_t arg_0x41154ab0){
#line 103
  unsigned char result;
#line 103

#line 103
  result = UniSART1M$USARTControl$tx(arg_0x41154ab0);
#line 103

#line 103
  return result;
#line 103
}
#line 103
#line 81
inline static   result_t ZigNet2SpiM$USARTControl$isTxIntrPending(void){
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
# 73 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline void TOSH_CLR_SATELLITE_CSN_PIN(void)
#line 73
{
#line 73
   static volatile uint8_t r __asm ("0x0031");

#line 73
  r &= ~(1 << 0);
}

# 174 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
static __inline void TOSH_uwait(uint16_t u)
{
  uint16_t i;

#line 177
  if (u < 500) {
    for (i = 2; i < u; i++) {
         __asm volatile ("nop\n\t"
        "nop\n\t"
        "nop\n\t"
        "nop\n\t");}
    }
  else {

    for (i = 0; i < u; i++) {
         __asm volatile ("nop\n\t"
        "nop\n\t"
        "nop\n\t"
        "nop\n\t");}
    }
}

# 50 "/home/max/tinyos/tinyos-1.x/tos/platform/pult_net/hardware.h"
static inline uint8_t TOSH_READ_SATELLITE_INTR_2_PIN(void)
#line 50
{
#line 50
   static volatile uint8_t r __asm ("0x0028");

#line 50
  return r & (1 << 1);
}

#line 79
static inline void TOSH_SET_SATELLITE_INTA_2_PIN(void)
#line 79
{
#line 79
   static volatile uint8_t r __asm ("0x0031");

#line 79
  r |= 1 << 6;
}

# 213 "../../../zigzag/ZigNet2SpiM.nc"
static inline  void ZigNet2SpiM$taskUsartRx(void)
{
  register uint8_t *data;
  uint8_t i = 100;

  TOSH_SET_SATELLITE_INTA_2_PIN();
  while (TOSH_READ_SATELLITE_INTR_2_PIN()) {
      if (--i == 0) {
          TOSH_CLR_SATELLITE_INTA_2_PIN();
          return;
        }
      TOSH_uwait(5);
    }

  TOSH_CLR_SATELLITE_CSN_PIN();

  ZigNet2SpiM$USARTControl$isTxIntrPending();
  ZigNet2SpiM$USARTControl$rx();


  data = (uint8_t *)&ZigNet2SpiM$usartDstAddr;
  ZigNet2SpiM$USARTControl$tx(0);
  while (!ZigNet2SpiM$USARTControl$isRxIntrPending()) ;
  * data++ = ZigNet2SpiM$USARTControl$rx();

  ZigNet2SpiM$USARTControl$tx(0);
  while (!ZigNet2SpiM$USARTControl$isRxIntrPending()) ;
  *data = ZigNet2SpiM$USARTControl$rx();


  ZigNet2SpiM$USARTControl$tx(0);
  while (!ZigNet2SpiM$USARTControl$isRxIntrPending()) ;
  ZigNet2SpiM$usartBufSize = ZigNet2SpiM$USARTControl$rx();

  ZigNet2SpiM$USARTControl$tx(0);
  while (!ZigNet2SpiM$USARTControl$isRxIntrPending()) ;
  ZigNet2SpiM$USARTControl$rx();


  if (0 < ZigNet2SpiM$usartBufSize && ZigNet2SpiM$usartBufSize <= MAC_AMAX_MAC_FRAME_SIZE - NWK_MIN_HEADER_OVERHEAD) {
      uint8_t i;

#line 254
      data = ZigNet2SpiM$usartBuf;
      for (i = 0; i < ZigNet2SpiM$usartBufSize; i++) {
          ZigNet2SpiM$USARTControl$tx(0);
          while (!ZigNet2SpiM$USARTControl$isRxIntrPending()) ;
          * data++ = ZigNet2SpiM$USARTControl$rx();
        }
      ZigNet2SpiM$sendEvents();
    }
  else {
#line 262
    TOSH_CLR_SATELLITE_INTA_2_PIN();
    }
  TOSH_SET_SATELLITE_CSN_PIN();
  return;
}

static inline   void ZigNet2SpiM$INTR_2$fired(void)
{
  if (!TOSH_READ_SATELLITE_INTA_2_PIN()) {
    if (!TOS_post(ZigNet2SpiM$taskUsartRx)) {
      TOSH_CLR_SATELLITE_INTA_2_PIN();
      }
    }
#line 273
  ZigNet2SpiM$INTR_2$clear();
  return;
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port21$fired(void){
#line 59
  ZigNet2SpiM$INTR_2$fired();
#line 59
}
#line 59
# 188 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port22$clear(void)
#line 188
{
#line 188
  MSP430InterruptM$P2IFG &= ~(1 << 2);
}

#line 104
static inline    void MSP430InterruptM$Port22$default$fired(void)
#line 104
{
#line 104
  MSP430InterruptM$Port22$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port22$fired(void){
#line 59
  MSP430InterruptM$Port22$default$fired();
#line 59
}
#line 59
# 189 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port23$clear(void)
#line 189
{
#line 189
  MSP430InterruptM$P2IFG &= ~(1 << 3);
}

#line 105
static inline    void MSP430InterruptM$Port23$default$fired(void)
#line 105
{
#line 105
  MSP430InterruptM$Port23$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port23$fired(void){
#line 59
  MSP430InterruptM$Port23$default$fired();
#line 59
}
#line 59
# 190 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port24$clear(void)
#line 190
{
#line 190
  MSP430InterruptM$P2IFG &= ~(1 << 4);
}

#line 106
static inline    void MSP430InterruptM$Port24$default$fired(void)
#line 106
{
#line 106
  MSP430InterruptM$Port24$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port24$fired(void){
#line 59
  MSP430InterruptM$Port24$default$fired();
#line 59
}
#line 59
# 191 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port25$clear(void)
#line 191
{
#line 191
  MSP430InterruptM$P2IFG &= ~(1 << 5);
}

#line 107
static inline    void MSP430InterruptM$Port25$default$fired(void)
#line 107
{
#line 107
  MSP430InterruptM$Port25$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port25$fired(void){
#line 59
  MSP430InterruptM$Port25$default$fired();
#line 59
}
#line 59
# 192 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port26$clear(void)
#line 192
{
#line 192
  MSP430InterruptM$P2IFG &= ~(1 << 6);
}

#line 108
static inline    void MSP430InterruptM$Port26$default$fired(void)
#line 108
{
#line 108
  MSP430InterruptM$Port26$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port26$fired(void){
#line 59
  MSP430InterruptM$Port26$default$fired();
#line 59
}
#line 59
# 193 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port27$clear(void)
#line 193
{
#line 193
  MSP430InterruptM$P2IFG &= ~(1 << 7);
}

#line 109
static inline    void MSP430InterruptM$Port27$default$fired(void)
#line 109
{
#line 109
  MSP430InterruptM$Port27$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port27$fired(void){
#line 59
  MSP430InterruptM$Port27$default$fired();
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

# 77 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static   result_t PhyPoolM$IPhyPool$Free(const TPhyPoolHandle handle)
{
  if (handle < PhyPoolM$POOL_SIZE) {
      bool isFree;

#line 81
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 81
        {
          isFree = PhyPoolM$pool[handle].free;
          if (FALSE == isFree) {
            PhyPoolM$pool[handle].free = TRUE;
            }
        }
#line 86
        __nesc_atomic_end(__nesc_atomic); }
#line 86
      if (FALSE == isFree) {
          PhyPoolM$FreePoolItem(handle);
          return SUCCESS;
        }
    }
  return FAIL;
}

# 225 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static   void HPLUSART0M$USARTControl$setModeSPI(void)
#line 225
{

  if (HPLUSART0M$USARTControl$getMode() == USART_SPI) {
    return;
    }
  HPLUSART0M$USARTControl$disableUART();
  HPLUSART0M$USARTControl$disableI2C();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 233
    {
      TOSH_SEL_SIMO0_MODFUNC();
      TOSH_SEL_SOMI0_MODFUNC();
      TOSH_SEL_UCLK0_MODFUNC();

      HPLUSART0M$IE1 &= ~((1 << 7) | (1 << 6));

      U0CTL = 0x01;
      U0CTL |= (0x10 | 0x04) | 0x02;
      U0CTL &= ~0x20;

      HPLUSART0M$U0TCTL = 0x02;
      HPLUSART0M$U0TCTL |= 0x80;

      if (HPLUSART0M$l_ssel & 0x80) {
          HPLUSART0M$U0TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
          HPLUSART0M$U0TCTL |= HPLUSART0M$l_ssel & 0x7F;
        }
      else {
          HPLUSART0M$U0TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
          HPLUSART0M$U0TCTL |= 0x20;
        }

      if (HPLUSART0M$l_br != 0) {
          U0BR0 = HPLUSART0M$l_br & 0x0FF;
          U0BR1 = (HPLUSART0M$l_br >> 8) & 0x0FF;
        }
      else {
          U0BR0 = 0x02;
          U0BR1 = 0x00;
        }
      U0MCTL = 0;

      HPLUSART0M$ME1 &= ~((1 << 7) | (1 << 6));
      HPLUSART0M$ME1 |= 1 << 6;
      U0CTL &= ~0x01;

      HPLUSART0M$IFG1 &= ~((1 << 7) | (1 << 6));
      HPLUSART0M$IE1 &= ~((1 << 7) | (1 << 6));
    }
#line 272
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 26 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacRandomM.nc"
static   uint16_t MacRandomM$IMacRandom$Rand(void)
{
  uint16_t r;
  uint32_t localTime = (uint32_t )MacRandomM$ILocalTime$Read();

#line 30
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 30
    r = MacRandomM$randomValue;
#line 30
    __nesc_atomic_end(__nesc_atomic); }
  r *= (uint16_t )(localTime & 0xFFFF);
  r += (localTime >> 8) & 0xFFFF;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 33
    MacRandomM$randomValue = r;
#line 33
    __nesc_atomic_end(__nesc_atomic); }
  return r;
}

# 74 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
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

# 257 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static void MacPoolM$ResetPool(void)
{
  MacPoolM$TMacPoolIndex i = 0;

#line 260
  for (; i < MacPoolM$MAC_POOL_SIZE; i++) {
      MacPoolM$pool[i].state = MacPoolM$MAC_POOL_ITEM_FREE;
      MacPoolM$pool[i].uniData = 0x0;
      MacPoolM$pool[i].expiredTime = 0x0;
      MacPoolM$pool[i].status = MAC_TRANSACTION_EXPIRED;
    }
}

# 34 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(
TMacBeaconOrder beaconOrder)
{
  bool changed = MacSuperframeAttrC$macBeaconOrder != beaconOrder;

#line 38
  MacSuperframeAttrC$macBeaconOrder = beaconOrder;
  if (changed) {
    MacSuperframeAttrC$BeaconOrderChanged(beaconOrder);
    }
#line 41
  return MAC_SUCCESS;
}

# 127 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$ICAPControl$start(uint8_t sfIndex)
{
  MacCAPM$constantOn = FALSE;
  MacCAPM$sfMode = MacCAPM$IMacSuperframeAttr$GetmacBeaconOrder(NULL) == 15 ? MAC_SUPERFRAME_MODE_UNSLOTTED : MAC_SUPERFRAME_MODE_SLOTTED;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    MacCAPM$cap[sfIndex].state = MacCAPM$sfMode == MAC_SUPERFRAME_MODE_UNSLOTTED ? MacCAPM$CAP_STATE_ACTIVE : MacCAPM$CAP_STATE_PASSIVE;
#line 131
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 132
    MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_IDLE;
#line 132
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacPoolM$IExpiredTimer$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(3U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
static  result_t MacDataM$AckWaitTimer$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(5U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 668 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static   result_t PhyCC2420M$IPhySET_TRX_STATE$Request(uint8_t context, 
TPhyStatus phyStatus, 
const TUniData uniData)
{
  enum PhyCC2420M$TPhySetTRXState state;

#line 673
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 673
    {
      state = PhyCC2420M$trxState;
      if (PhyCC2420M$trxState == PhyCC2420M$PHY_TRX_FREE) {
        PhyCC2420M$trxState = PhyCC2420M$PHY_TRX_BUSY;
        }
    }
#line 678
    __nesc_atomic_end(__nesc_atomic); }
#line 678
  if (PhyCC2420M$PHY_TRX_BUSY == state) {
    return FAIL;
    }
#line 680
  PhyCC2420M$trx.context = context, PhyCC2420M$trx.uniData = uniData, PhyCC2420M$trx.phyStatus = phyStatus;
  if (TOS_post(PhyCC2420M$SetTRXStateDone)) {
    return SUCCESS;
    }
#line 683
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 683
    PhyCC2420M$trxState = PhyCC2420M$PHY_TRX_FREE;
#line 683
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

#line 486
static void PhyCC2420M$turnOnChipcon(void)
{
  TCC2420_STATUS status;
  PhyCC2420M$TPhyRadioState _radioState;

#line 490
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 490
    _radioState = PhyCC2420M$radioState;
#line 490
    __nesc_atomic_end(__nesc_atomic); }
  if (PhyCC2420M$PHY_RADIO_OFF != _radioState) {
    return;
    }
  PhyCC2420M$HPLCC2420Control$start();
  PhyCC2420M$IChipconSFD$disable();
  PhyCC2420M$IChipconFIFOP$disable();
  PhyCC2420M$IChipconFIFOSignal$disable();


  TOSH_CLR_RADIO_POWER_PIN();


  TOSH_SET_CC_VREN_PIN();
#line 503
  TOSH_uwait(600);
  TOSH_CLR_CC_RSTN_PIN();
#line 504
  TOSH_wait();
  TOSH_SET_CC_RSTN_PIN();
#line 505
  TOSH_wait();
  status.raw = PhyCC2420M$IChipcon$cmd(0x01);
  do {
      status.raw = PhyCC2420M$IChipcon$cmd(0x00);
    }
  while (
#line 509
  status.value.xosc16m_stable != 1);
  PhyCC2420M$SetDefaultRegisters();
  PhyCC2420M$IPhySET$SetphyCurrentChannel(PhyCC2420M$g_channel);
  PhyCC2420M$IPhyAttr$SetextendedAddress(PhyCC2420M$g_extendedAddress);
  PhyCC2420M$IPhyAttr$SetpanId(PhyCC2420M$g_panId);
  PhyCC2420M$IPhyAttr$SetshortAddress(PhyCC2420M$g_shortAddress);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 515
    PhyCC2420M$radio.txFifoHandle = 0xff;
#line 515
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 516
    PhyCC2420M$trxState = PhyCC2420M$PHY_TRX_FREE;
#line 516
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 517
    PhyCC2420M$edState = PhyCC2420M$PHY_ED_FREE;
#line 517
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 518
    PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_IDLE;
#line 518
    __nesc_atomic_end(__nesc_atomic); }
  PhyCC2420M$IChipcon$cmd(0x0B);
}

# 200 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420InterruptM.nc"
static   result_t HPLCC2420InterruptM$SFD$disable(void)
#line 200
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 201
    {
      HPLCC2420InterruptM$SFDControl$disableEvents();
      HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
      TOSH_SEL_CC_SFD_IOFUNC();
    }
#line 205
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 78
static   result_t HPLCC2420InterruptM$FIFOP$disable(void)
#line 78
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      HPLCC2420InterruptM$FIFOPInterrupt$disable();
      HPLCC2420InterruptM$FIFOPInterrupt$clear();
    }
#line 82
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 119
static   result_t HPLCC2420InterruptM$FIFO$disable(void)
#line 119
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 120
    {
      HPLCC2420InterruptM$FIFOInterrupt$disable();
      HPLCC2420InterruptM$FIFOInterrupt$clear();
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 127 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static   uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t addr)
#line 127
{
  uint8_t status = 0;

#line 129
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
        HPLCC2420M$f.busy = TRUE;
#line 131
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      status = HPLCC2420M$adjustStatusByte(HPLCC2420M$USARTControl$rx());
      TOSH_SET_RADIO_CSN_PIN();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 140
        HPLCC2420M$f.busy = FALSE;
#line 140
        __nesc_atomic_end(__nesc_atomic); }
#line 152
      HPLCC2420M$BusArbitration$releaseBus();
    }
  return status;
}

# 94 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
static   result_t BusArbitrationM$BusArbitration$getBus(uint8_t id)
#line 94
{
  bool gotbus = FALSE;

#line 96
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 96
    {
      if (BusArbitrationM$state == BusArbitrationM$BUS_IDLE) {
          BusArbitrationM$state = BusArbitrationM$BUS_BUSY;
          gotbus = TRUE;
          BusArbitrationM$busid = id;
        }
    }
#line 102
    __nesc_atomic_end(__nesc_atomic); }
  if (gotbus) {
    return SUCCESS;
    }
#line 105
  return FAIL;
}

# 424 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static   result_t HPLUSART0M$USARTControl$isTxIntrPending(void)
#line 424
{
  if (HPLUSART0M$IFG1 & (1 << 7)) {
      HPLUSART0M$IFG1 &= ~(1 << 7);
      return SUCCESS;
    }
  return FAIL;
}

#line 478
static   uint8_t HPLUSART0M$USARTControl$rx(void)
#line 478
{
  uint8_t value;

#line 480
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 480
    {
      value = U0RXBUF;
    }
#line 482
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

#line 439
static   result_t HPLUSART0M$USARTControl$isRxIntrPending(void)
#line 439
{
  if (HPLUSART0M$IFG1 & (1 << 6)) {
      HPLUSART0M$IFG1 &= ~(1 << 6);
      return SUCCESS;
    }
  return FAIL;
}

# 108 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
static   result_t BusArbitrationM$BusArbitration$releaseBus(uint8_t id)
#line 108
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (BusArbitrationM$state == BusArbitrationM$BUS_BUSY && BusArbitrationM$busid == id) {
          BusArbitrationM$state = BusArbitrationM$BUS_IDLE;





          if (BusArbitrationM$isBusReleasedPending == FALSE && TOS_post(BusArbitrationM$busReleased) == TRUE) {
            BusArbitrationM$isBusReleasedPending = TRUE;
            }
        }
    }
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 162 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static   uint8_t HPLCC2420M$HPLCC2420$write(uint8_t addr, uint16_t data)
#line 162
{
  uint8_t status = 0;

#line 164
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 166
        HPLCC2420M$f.busy = TRUE;
#line 166
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      status = HPLCC2420M$adjustStatusByte(HPLCC2420M$USARTControl$rx());
      HPLCC2420M$USARTControl$tx((data >> 8) & 0x0FF);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
      HPLCC2420M$USARTControl$tx(data & 0x0FF);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isTxEmpty()) ;
      TOSH_SET_RADIO_CSN_PIN();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 179
        HPLCC2420M$f.busy = FALSE;
#line 179
        __nesc_atomic_end(__nesc_atomic); }
#line 195
      HPLCC2420M$BusArbitration$releaseBus();
    }
  return status;
}






static   uint16_t HPLCC2420M$HPLCC2420$read(uint8_t addr)
#line 205
{
  uint16_t data = 0;

#line 207
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 209
        HPLCC2420M$f.busy = TRUE;
#line 209
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr | 0x40);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      data = (HPLCC2420M$USARTControl$rx() << 8) & 0xFF00;
      HPLCC2420M$USARTControl$tx(0);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isRxIntrPending()) ;
      data = data | (HPLCC2420M$USARTControl$rx() & 0x0FF);
      TOSH_SET_RADIO_CSN_PIN();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 224
        HPLCC2420M$f.busy = FALSE;
#line 224
        __nesc_atomic_end(__nesc_atomic); }
#line 242
      HPLCC2420M$BusArbitration$releaseBus();
    }
  return data;
}

# 792 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static  TPhyStatus PhyCC2420M$IPhySET$SetphyCurrentChannel(TPhyChannel phyCurrentChannel)
{


  if (
#line 795
  phyCurrentChannel <= 26
   && PhyCC2420M$g_phyChannelsSupported & ((uint32_t )1 << phyCurrentChannel)) {
      TCC2420_STATUS status;
      TCC2420_FSCTRL fsctrl;

#line 799
      fsctrl.raw = PhyCC2420M$IChipcon$read(CC2420_FSCTRL_ADDRESS);
      fsctrl.value.freq = 357 + 5 * (phyCurrentChannel - 11);
      status.raw = PhyCC2420M$IChipcon$write(CC2420_FSCTRL_ADDRESS, fsctrl.raw);
      if (status.value.xosc16m_stable == 1) {
        PhyCC2420M$IChipcon$cmd(0x03);
        }
#line 804
      PhyCC2420M$g_channel = phyCurrentChannel;
      return PHY_SUCCESS;
    }
  return PHY_INVALID_PARAMETER;
}

#line 865
static  result_t PhyCC2420M$IPhyAttr$SetextendedAddress(uint64_t extendedAddress)
{
  PhyCC2420M$g_extendedAddress = extendedAddress;
  PhyCC2420M$IChipconRAM$write(CC2420_EXTADDR_ADDRESS, sizeof extendedAddress, (uint8_t *)&extendedAddress);
  return SUCCESS;
}

# 292 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420M.nc"
static   result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t addr, uint8_t _length, uint8_t *buffer)
#line 292
{
  uint8_t i = 0;

#line 294
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 295
        {
          HPLCC2420M$f.busy = TRUE;
          HPLCC2420M$ramaddr = addr;
          HPLCC2420M$ramlen = _length;
          HPLCC2420M$rambuf = buffer;
        }
#line 300
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx((HPLCC2420M$ramaddr & 0x7F) | 0x80);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
      HPLCC2420M$USARTControl$tx((HPLCC2420M$ramaddr >> 1) & 0xC0);
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
      for (i = 0; i < HPLCC2420M$ramlen; i++) {
          HPLCC2420M$USARTControl$tx(HPLCC2420M$rambuf[i]);
          while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isTxIntrPending()) ;
        }
      while (HPLCC2420M$f.enabled && !HPLCC2420M$USARTControl$isTxEmpty()) ;
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 316
        HPLCC2420M$f.busy = FALSE;
#line 316
        __nesc_atomic_end(__nesc_atomic); }
      return TOS_post(HPLCC2420M$signalRAMWr);
    }
  return FAIL;
}

# 858 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static  result_t PhyCC2420M$IPhyAttr$SetpanId(uint16_t panId)
{
  PhyCC2420M$g_panId = panId;
  PhyCC2420M$IChipconRAM$write(CC2420_PANID_ADDRESS, sizeof panId, (uint8_t *)&panId);
  return SUCCESS;
}

#line 851
static  result_t PhyCC2420M$IPhyAttr$SetshortAddress(uint16_t shortAddress)
{
  PhyCC2420M$g_shortAddress = shortAddress;
  PhyCC2420M$IChipconRAM$write(CC2420_SHORTADDR_ADDRESS, sizeof shortAddress, (uint8_t *)&shortAddress);
  return SUCCESS;
}

#line 541
static void PhyCC2420M$turnRxOnChipcon(void)
{
  PhyCC2420M$IChipconSFD$enableCapture(TRUE);
  PhyCC2420M$flushRXFIFO();
  PhyCC2420M$IChipcon$cmd(0x03);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 546
    PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH;
#line 546
    __nesc_atomic_end(__nesc_atomic); }
}

# 185 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420InterruptM.nc"
static   result_t HPLCC2420InterruptM$SFD$enableCapture(bool low_to_high)
#line 185
{
  uint8_t _direction;

#line 187
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 187
    {
      TOSH_SEL_CC_SFD_MODFUNC();
      HPLCC2420InterruptM$SFDControl$disableEvents();
      if (low_to_high) {
#line 190
        _direction = MSP430TIMER_CM_RISING;
        }
      else {
#line 191
        _direction = MSP430TIMER_CM_FALLING;
        }
#line 192
      HPLCC2420InterruptM$SFDControl$setControlAsCapture(_direction);
      HPLCC2420InterruptM$SFDCapture$clearOverflow();
      HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
      HPLCC2420InterruptM$SFDControl$enableEvents();
    }
#line 196
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 191 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static void PhyCC2420M$flushRXFIFO(void)
{
  PhyCC2420M$IChipconFIFOSignal$disable();
  PhyCC2420M$IChipconFIFOP$disable();
  PhyCC2420M$IChipcon$read(0x3F);
  PhyCC2420M$IChipcon$cmd(0x08);
  PhyCC2420M$IChipcon$cmd(0x08);
  PhyCC2420M$IChipconFIFOP$startWait(TRUE);
  PhyCC2420M$IChipconFIFOSignal$startWait(TRUE);
}

# 65 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/HPLCC2420InterruptM.nc"
static   result_t HPLCC2420InterruptM$FIFOP$startWait(bool low_to_high)
#line 65
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 66
    {
      HPLCC2420InterruptM$FIFOPInterrupt$disable();
      HPLCC2420InterruptM$FIFOPInterrupt$clear();
      HPLCC2420InterruptM$FIFOPInterrupt$edge(low_to_high);
      HPLCC2420InterruptM$FIFOPInterrupt$enable();
    }
#line 71
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 106
static   result_t HPLCC2420InterruptM$FIFO$startWait(bool low_to_high)
#line 106
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    {
      HPLCC2420InterruptM$FIFOInterrupt$disable();
      HPLCC2420InterruptM$FIFOInterrupt$clear();
      HPLCC2420InterruptM$FIFOInterrupt$edge(low_to_high);
      HPLCC2420InterruptM$FIFOInterrupt$enable();
    }
#line 112
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 549 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static void PhyCC2420M$turnTxOnChipcon(void)
{
  PhyCC2420M$IChipconFIFOP$disable();
  PhyCC2420M$IChipconFIFOSignal$disable();
  PhyCC2420M$IChipcon$cmd(0x06);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 555
    PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_IDLE;
#line 555
    __nesc_atomic_end(__nesc_atomic); }
}

#line 522
static void PhyCC2420M$turnOffChipcon(void)
{
  PhyCC2420M$IChipcon$cmd(0x07);
  TOSH_CLR_RADIO_CSN_PIN();
  TOSH_CLR_CC_RSTN_PIN();
  TOSH_CLR_CC_VREN_PIN();



  TOSH_SET_RADIO_POWER_PIN();


  PhyCC2420M$HPLCC2420Control$stop();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 536
    PhyCC2420M$radio.txFifoHandle = 0xff;
#line 536
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 537
    PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_OFF;
#line 537
    __nesc_atomic_end(__nesc_atomic); }
}

# 353 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static bool MacCAPM$enoughTime(uint8_t sfIndex)
{
  if (MacCAPM$sfMode == MAC_SUPERFRAME_MODE_UNSLOTTED) {
    return TRUE;
    }
  else {
      TSysTime now = MacCAPM$ILocalTime$Read();
      TSysTime sf_duration = (TSysTime )MAC_ABASE_SUPERFRAME_DURATION << MacCAPM$IMacSuperframeAttr$GetmacSuperframeOrder(NULL);

#line 361
      return sf_duration > now - MacCAPM$cap[sfIndex].beginCAPTime + MacCAPM$calcSendTime(sfIndex);
    }
}

# 24 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static   uint8_t PhyFrameM$IPhyFrame$GetPPDULength(const TPhyFrame *const pPhyFrame)
{
  if (pPhyFrame != NULL) {
    return PhyFrameM$IPhyFrame$GetMPDULength(pPhyFrame) + PHY_SIZE_OF_FRAME_LENGTH;
    }
#line 28
  return 0;
}

# 49 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static   TPhyFrame *PhyPoolM$IPhyPool$GetFrame(const TPhyPoolHandle handle)
{
  if (handle < PhyPoolM$POOL_SIZE) {
      bool allocItem;

#line 53
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 53
        allocItem = ! PhyPoolM$pool[handle].free;
#line 53
        __nesc_atomic_end(__nesc_atomic); }
      if (allocItem) {


          return & PhyPoolM$pool[(uint16_t )handle].frame;
        }
    }
  return (TPhyFrame *)NULL;
}

# 424 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static result_t MacCAPM$TurnOffIfInactive(TPhyStatus state)
{
  uint8_t i;

  if (MacCAPM$constantOn == TRUE) {
    return SUCCESS;
    }



  if (MacCAPM$IMacSuperframeAttr$GetmacSuperframeOrder(NULL) != 15) 
    {
      bool active = FALSE;

#line 437
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          for (i = 0; i < MacCAPM$NUM_SUPERFRAMES && !active; i++) {
              active = MacCAPM$cap[i].state == MacCAPM$CAP_STATE_ACTIVE;
            }
          if (!active) 
            {
              {
                unsigned char __nesc_temp = 
#line 444
                MacCAPM$IPhySET_TRX_STATE$Request(state, 0);

                {
#line 444
                  __nesc_atomic_end(__nesc_atomic); 
#line 444
                  return __nesc_temp;
                }
              }
            }
        }
#line 448
        __nesc_atomic_end(__nesc_atomic); }
    }
#line 449
  return SUCCESS;
}

# 158 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static   result_t PhyCC2420M$IPhyTxDATA$Request(uint8_t context, 
const TPhyPoolHandle handle, 
const TUniData uniData)
{
  PhyCC2420M$TPhyRadioState _radioState;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 164
    {
      _radioState = PhyCC2420M$radioState;
      if (PhyCC2420M$radioState == PhyCC2420M$PHY_RADIO_TX_IDLE) {
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_INIT;
        }
    }
#line 169
    __nesc_atomic_end(__nesc_atomic); }
#line 169
  if (_radioState == PhyCC2420M$PHY_RADIO_TX_IDLE) {
      result_t result;

#line 171
      PhyCC2420M$radio.context = context, PhyCC2420M$radio.uniData = uniData, PhyCC2420M$radio.handle = handle;
      if (handle == PhyCC2420M$radio.txFifoHandle) {
          result = PhyCC2420M$SendTxFIFO();
          if (SUCCESS == result) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 175
                PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_FRAME_SEND;
#line 175
                __nesc_atomic_end(__nesc_atomic); }
              return SUCCESS;
            }
        }
      else 
#line 178
        {
          result = PhyCC2420M$WriteToTxFIFO(handle);
          if (SUCCESS == result) {
              PhyCC2420M$radio.txFifoSend = TRUE;
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 182
                PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_FIFO_WRITE;
#line 182
                __nesc_atomic_end(__nesc_atomic); }
              return SUCCESS;
            }
        }
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 186
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_IDLE;
#line 186
        __nesc_atomic_end(__nesc_atomic); }
    }
  return FAIL;
}

#line 116
static result_t PhyCC2420M$SendTxFIFO(void)
{
  TCC2420_STATUS status;



  PhyCC2420M$IChipcon$cmd(0x04);
  status.raw = PhyCC2420M$IChipcon$cmd(0x00);
  return status.value.tx_active == 1 ? SUCCESS : FAIL;
}

#line 101
static result_t PhyCC2420M$WriteToTxFIFO(TPhyPoolHandle handle)
{
  TPhyFrame *pPhyFrame = PhyCC2420M$IPhyPool$GetFrame(handle);

#line 104
  if (NULL != pPhyFrame) {
      PhyCC2420M$radio.pData = PhyCC2420M$IPhyFrame$GetPPDU(pPhyFrame);
      PhyCC2420M$radio.dataLength = PhyCC2420M$IPhyFrame$GetPPDULength(pPhyFrame);
      if (NULL != PhyCC2420M$radio.pData && 0 < PhyCC2420M$radio.dataLength) {
          PhyCC2420M$flushTXFIFO();
          if (SUCCESS == PhyCC2420M$IChipconFIFO$writeTXFIFO(PhyCC2420M$radio.dataLength, PhyCC2420M$radio.pData)) {
            return SUCCESS;
            }
        }
    }
#line 113
  return FAIL;
}

# 10 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static   uint8_t *PhyFrameM$IPhyFrame$GetPPDU(const TPhyFrame *const pPhyFrame)
{
  if (pPhyFrame != NULL) {
    return (uint8_t *)&pPhyFrame->data[0];
    }
#line 14
  return NULL;
}

# 202 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static void PhyCC2420M$SendDone(TPhyStatus phyStatus)
{
  uint8_t context = PhyCC2420M$radio.context;
  TUniData uniData = PhyCC2420M$radio.uniData;
  TPhyPoolHandle handle = PhyCC2420M$radio.handle;



  if (!TOSH_READ_CC_FIFO_PIN() && TOSH_READ_CC_FIFOP_PIN()) {
    PhyCC2420M$flushRXFIFO();
    }
#line 212
  if (PHY_SUCCESS == phyStatus) {
      PhyCC2420M$IChipconSFD$enableCapture(TRUE);
      PhyCC2420M$IChipconFIFOP$startWait(TRUE);
      PhyCC2420M$IChipconFIFOSignal$startWait(TRUE);
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 216
        PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_RX_SFD_SEARCH;
#line 216
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
#line 218
    { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 218
      PhyCC2420M$radioState = PhyCC2420M$PHY_RADIO_TX_IDLE;
#line 218
      __nesc_atomic_end(__nesc_atomic); }
    }
#line 219
  PhyCC2420M$IPhyTxDATA$Confirm(context, handle, phyStatus, uniData);
}

# 519 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static void MacCAPM$_SendDone(const uint8_t sfIndex, const TMacStatus macStatus)
{
  TUniData uniData;
  TPhyPoolHandle handle = MacCAPM$cap[sfIndex].handle;
  uint8_t context = MacCAPM$cap[sfIndex].context;

  MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_IDLE;
  MacCAPM$IPhyPool$GetUniData(handle, &uniData);
  MacCAPM$IPhyPool$Free(handle);


  if (MacCAPM$sfMode == MAC_SUPERFRAME_MODE_UNSLOTTED) 
    {
      uint8_t i;

#line 533
      for (i = 0; i < MacCAPM$NUM_SUPERFRAMES; i++) {
          if (MacCAPM$cap[i].sendState == MacCAPM$CAP_SEND_PENDING) 
            {
              ;
              ;
              if (!MacCAPM$StartSending(i)) {
                MacCAPM$_SendDone(i, MAC_CHANNEL_ACCESS_FAILURE);
                }
#line 540
              break;
            }
        }
    }


  MacCAPM$SendDone(sfIndex, context, macStatus, uniData);

  return;
}

#line 253
static result_t MacCAPM$StartSending(uint8_t sfIndex)
{
  result_t result;

#line 256
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (MacCAPM$cap[sfIndex].state == MacCAPM$CAP_STATE_ACTIVE) 

        {
          result = MacCAPM$SetBackoff(sfIndex);
        }
      else 

        {
          result = SUCCESS;
        }

      MacCAPM$cap[sfIndex].sendState = result ? MacCAPM$CAP_SEND_BACKOFF : MacCAPM$CAP_SEND_IDLE;
    }
#line 270
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

#line 617
static result_t MacCAPM$SetBackoff(uint8_t sfIndex)
{
  TSysTime remain = MAC_AUNIT_BACKOFF_PERIOD;
  uint8_t intrvl_remain = MacCAPM$cap[sfIndex].backoffRemain;

#line 621
  if (0 != intrvl_remain) {
    remain = intrvl_remain * MAC_AUNIT_BACKOFF_PERIOD;
    }




  return MacCAPM$IBackoff$SetOneShot(remain, sfIndex);
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

#line 88
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

#line 189
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

# 84 "../../../zigzag/ZigBee/implementation/NeighborTableM.nc"
static  result_t NeighborTableM$NeighborTable$getNextPtr(NwkNeighbor **neighbor)
{
  if (*neighbor == NULL) {
    *neighbor = NeighborTableM$Table;
    }
  else {
#line 89
    (*neighbor)++;
    }
#line 90
  while (*neighbor - NeighborTableM$Table < 20) 
    {
      if (!((*neighbor)->networkAddr == 0xFFFF && (*neighbor)->extendedAddr == 0xFFFFFFFFFFFFFFFFLL)) 
        {
          return SUCCESS;
        }
      (*neighbor)++;
    }

  return FAIL;
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

#line 8
static void SerializeM$memcopy(void *const dst, const void *const src, const uint8_t len)
{
  uint8_t i;

#line 11
  for (i = 0; i < len; i++) (
    (uint8_t *)dst)[i] = ((uint8_t *)src)[i];
}

# 20 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static  uint64_t LocalTime64M$ILocalTime64$getLocalTimeAt(TSysTime at)
{
  TSysTime curCount = at;
  TMilliSec t = LocalTime64M$ITimeCast$SymbolsToMillis(curCount - LocalTime64M$firedCount);
  uint64_t localTime = LocalTime64M$baseTime + t;


  return localTime;
}

# 127 "../../../zigzag/ZigNet2SpiM.nc"
static void ZigNet2SpiM$usartTx(NwkAddr srcAddr, IEEEAddr extAddr, uint8_t nsduLength, uint8_t *nsdu)
{
  register uint8_t *data;
  uint8_t i = 80;

  ZigNet2SpiM$workaround = &extAddr;

  TOSH_SET_SATELLITE_INTR_PIN();
  while (!TOSH_READ_SATELLITE_INTA_PIN()) {
      if (--i == 0) {
          TOSH_CLR_SATELLITE_INTR_PIN();
          TOSH_TOGGLE_LED0_PIN();
          return;
        }
      TOSH_uwait(10);
    }
  TOSH_CLR_SATELLITE_CSN_PIN();

  ZigNet2SpiM$USARTControl$isTxIntrPending();
  ZigNet2SpiM$USARTControl$rx();


  data = (uint8_t *)&srcAddr;
  ZigNet2SpiM$USARTControl$tx(* data++);

  while (!ZigNet2SpiM$USARTControl$isTxIntrPending()) ;
  ZigNet2SpiM$USARTControl$tx(*data);
  while (!ZigNet2SpiM$USARTControl$isTxIntrPending()) ;




  data = (uint8_t *)&extAddr;
  for (i = 0; i < sizeof(IEEEAddr ); i++) {
      ZigNet2SpiM$USARTControl$tx(* data++);
      while (!ZigNet2SpiM$USARTControl$isTxIntrPending()) ;
    }


  ZigNet2SpiM$USARTControl$tx(nsduLength);
  while (!ZigNet2SpiM$USARTControl$isTxIntrPending()) ;
  ZigNet2SpiM$USARTControl$tx(0x00);
  while (!ZigNet2SpiM$USARTControl$isTxIntrPending()) ;


  data = nsdu;
  for (i = 0; i < nsduLength; i++) {
      ZigNet2SpiM$USARTControl$tx(* data++);
      while (!ZigNet2SpiM$USARTControl$isTxIntrPending()) ;
    }

  while (!ZigNet2SpiM$USARTControl$isTxEmpty()) ;

  TOSH_SET_SATELLITE_CSN_PIN();
  TOSH_CLR_SATELLITE_INTR_PIN();

  TOSH_TOGGLE_LED2_PIN();

  return;
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

#line 201
static   uint8_t UniSART1M$USARTControl$rx(void)
#line 201
{
  uint8_t value;

#line 203
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 203
    {
      value = U1RXBUF;
    }
#line 205
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 27 "../../../zigzag/ZigBee/implementation/NwkBeaconParentM.nc"
static  void NwkBeaconParentM$updateBeacon(void)
{
  if (NwkBeaconParentM$IMacGET$GetmacAssociationPermit(NULL)) 
    {
      NwkBeaconPayload beacon;
      TSysTime b_offs;
      IEEEAddr my_ext_addr = info_param.MAC_ADDRESS;

#line 34
      beacon.protocolID = 0;
      beacon.stkProf = 0;
      beacon.prVer = NWK_PROTOCOL_VERSION;

      beacon.rtrCap = NwkBeaconParentM$NwkAddressing$hasVacantRouterAddrs() & 1;
      beacon.endCap = NwkBeaconParentM$NwkAddressing$hasVacantEdAddrs() & 1;
      beacon.devDep = NwkBeaconParentM$NIB$getDepth() & 0xf;

      b_offs = NwkBeaconParentM$NIB$getBeaconOffset();
      nmemcpy(beacon.txOffset, &b_offs, 3);

      nmemcpy(& beacon.extendedPANID, &my_ext_addr, 8);

      NwkBeaconParentM$IMacSET$SetmacBeaconPayload((uint8_t *)&beacon, sizeof(NwkBeaconPayload ));
    }
  else 

    {
      NwkBeaconParentM$IMacSET$SetmacBeaconPayload(NULL, 0);
    }
}

# 148 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static void NwkAddressingM$countChildren(uint8_t *routers, uint8_t *eds)
{
  NwkNeighbor *pn;

#line 151
  *routers = 0;
  *eds = 0;
  pn = NULL;
  while (NwkAddressingM$NeighborTable$getNextPtr(&pn)) 
    if (pn->relationship == NWK_CHILD) 
      {
        if (pn->deviceType == ZIGBEE_ROUTER) {
          (*routers)++;
          }
        else {
#line 159
          if (pn->deviceType == ZIGBEE_END_DEVICE) {
            (*eds)++;
            }
          }
      }
}

# 149 "/home/max/tinyos/tinyos-1.x/tos/system/tos.h"
static void *nmemcpy(void *to, const void *from, size_t n)
{
  char *cto = to;
  const char *cfrom = from;

  while (n--) * cto++ = * cfrom++;

  return to;
}

# 63 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBeaconPayload(
uint8_t *payload, 
uint8_t length)
{
  uint8_t i;

  if (length > MAC_AMAX_BEACON_PAYLOAD_LENGTH) {
    return FAIL;
    }
  MacBeaconAttrCoordM$beaconPayloadLength = length;
  for (i = 0; i < length; i++) 
    {
      MacBeaconAttrCoordM$beaconPayload[i] = payload[i];
    }
  return MAC_SUCCESS;
}

# 276 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static result_t MacDataM$Send(const TMacFrame *const frame, const TUniData unidata)
{
  if (MacDataM$currentSf == MacDataM$PARENT_SF) {
    return MacDataM$IMacSendParent$Send(frame, unidata);
    }
  else {
#line 281
    return MacDataM$IMacSendOwn$Send(frame, unidata);
    }
}

# 480 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$Send(const uint8_t sfIndex, 
const TMacFrame *const pMacFrame, 
const uint8_t context, 
const TUniData uniData)
{
  result_t result;
  MacCAPM$TMacCAPSendState sendState = MacCAPM$cap[sfIndex].sendState;

  ;
  if (MacCAPM$CAP_SEND_IDLE != sendState) 
    {
      return FAIL;
    }

  result = MacCAPM$PutToPhyPool(pMacFrame, uniData, & MacCAPM$cap[sfIndex].handle);
  if (SUCCESS != result) {
      return FAIL;
    }

  result = MacCAPM$SendPhyFrame(sfIndex, context);
  if (FAIL == result) {
    MacCAPM$IPhyPool$Free(MacCAPM$cap[sfIndex].handle);
    }
#line 502
  return result;
}

# 25 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static   result_t PhyPoolM$IPhyPool$Alloc(
const uint8_t size, 
const uint16_t uniData, 
TPhyPoolHandle *const pHandle)
{
  if (pHandle != NULL) {
      PhyPoolM$TPoolIndex i;

#line 32
      for (i = 0; i < PhyPoolM$POOL_SIZE; i++) {
          bool isFree;

#line 34
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 34
            {
              isFree = PhyPoolM$pool[i].free;
              if (TRUE == PhyPoolM$pool[i].free) {
                PhyPoolM$pool[i].free = FALSE;
                }
            }
#line 39
            __nesc_atomic_end(__nesc_atomic); }
#line 39
          if (FALSE == isFree) {
#line 39
            continue;
            }
#line 40
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 40
            PhyPoolM$pool[i].uniData = uniData;
#line 40
            __nesc_atomic_end(__nesc_atomic); }
          *pHandle = (TPhyPoolHandle )i;
          PhyPoolM$IPhyFrame$Erase(& PhyPoolM$pool[i].frame);
          return SUCCESS;
        }
    }
  return FAIL;
}

# 70 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static   result_t PhyFrameM$IPhyFrame$Erase(TPhyFrame *const pPhyFrame)
{
  if (pPhyFrame != NULL) {
      pPhyFrame->data[0] = 0;
      pPhyFrame->current = 0;
      return SUCCESS;
    }
  return FAIL;
}

# 315 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static  result_t MacFrameFormatM$IMacFrame$Pack(const TMacFrame *const pMacFrame, TPhyFrame *const pPhyFrame)
{
  result_t result;

#line 318
  if (NULL == pMacFrame || NULL == pPhyFrame) {
    return FAIL;
    }
#line 320
  result = MacFrameFormatM$IPhyFrame$Erase(pPhyFrame);
  {
#line 321
    TMacTimeStamp timeStamp;

#line 322
    result &= MacFrameFormatM$IMacFrame$GetTimeStamp(pMacFrame, &timeStamp);
    result &= MacFrameFormatM$IPhyFrame$SetTimeStamp(pPhyFrame, (TPhyTimeStamp )timeStamp);
  }
  result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->frameControl.lsb.raw);
  result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->frameControl.msb.raw);
  result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->sequenceNumber.raw[0]);
  if (MAC_ADDRESS_NOT_PRESENT != pMacFrame->frameControl.msb.value.dstAddressMode) {
      result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->dstPANId.raw[0]);
      result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->dstPANId.raw[1]);
      if (MAC_ADDRESS_SHORT == pMacFrame->frameControl.msb.value.dstAddressMode) {
          result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->dstAddress._short.raw[0]);
          result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->dstAddress._short.raw[1]);
        }
      else 
#line 334
        {
          uint8_t i;

#line 336
          for (i = 0; i < MAC_EXTENDED_ADDRESS_LENGTH; i++) 
            result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->dstAddress._extended.raw[i]);
        }
    }
  if (MAC_ADDRESS_NOT_PRESENT != pMacFrame->frameControl.msb.value.srcAddressMode) {
      bool intraPAN;

#line 342
      result &= MacFrameFormatM$IMacFrame$GetIntraPAN(pMacFrame, &intraPAN);
      if (!intraPAN) {
          result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->srcPANId.raw[0]);
          result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->srcPANId.raw[1]);
        }
      if (MAC_ADDRESS_SHORT == pMacFrame->frameControl.msb.value.srcAddressMode) {
          result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->srcAddress._short.raw[0]);
          result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->srcAddress._short.raw[1]);
        }
      else 
#line 350
        {
          uint8_t i;

#line 352
          for (i = 0; i < MAC_EXTENDED_ADDRESS_LENGTH; i++) 
            result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->srcAddress._extended.raw[i]);
        }
    }
  {
#line 356
    uint8_t i;

#line 357
    for (i = 0; i < pMacFrame->payload.length; i++) 
      result &= MacFrameFormatM$IPhyFrame$AppendOctet(pPhyFrame, pMacFrame->payload.raw[i]);
  }
  result &= MacFrameFormatM$IPhyFrame$CalcCRC(pPhyFrame);
  return SUCCESS == result ? SUCCESS : FALSE;
}

# 100 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static   result_t PhyFrameM$IPhyFrame$SetTimeStamp(TPhyFrame *const pPhyFrame, TPhyTimeStamp stamp)
{
  if (pPhyFrame != NULL) {
      pPhyFrame->timeStamp = stamp;
      return SUCCESS;
    }
  return FAIL;
}

#line 38
static   result_t PhyFrameM$IPhyFrame$AppendOctet(TPhyFrame *const pPhyFrame, const uint8_t octet)
{
  if (pPhyFrame != NULL) {
    if (pPhyFrame->data[0] <= PHY_AMAX_PHY_PACKET_SIZE - CRC16_LENGTH) {
        pPhyFrame->data[PHY_SIZE_OF_FRAME_LENGTH + pPhyFrame->data[0]] = octet;
        pPhyFrame->data[0] += 1;
        return SUCCESS;
      }
    }
#line 46
  return FAIL;
}

# 98 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static  result_t MacFrameFormatM$IMacFrame$GetIntraPAN(const TMacFrame *const pMacFrame, bool *const pIntraPAN)
{
  if (pMacFrame != NULL && pIntraPAN != NULL) {
      *pIntraPAN = 1 == pMacFrame->frameControl.lsb.value.intraPAN;
      return SUCCESS;
    }
  return FAIL;
}

# 216 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static result_t MacCAPM$SendPhyFrame(const uint8_t sfIndex, 
const uint8_t context)
{

  MacCAPM$cap[sfIndex].context = context;


  MacCAPM$cap[sfIndex].backoffExponent = MacCAPM$macMinBE;
  MacCAPM$cap[sfIndex].backoffNumber = 0;
  MacCAPM$cap[sfIndex].backoffRemain = MacCAPM$calcBackoff(sfIndex);



  if (MacCAPM$sfMode == MAC_SUPERFRAME_MODE_UNSLOTTED) 
    {
      uint8_t i;
      bool tx_busy = FALSE;

#line 233
      for (i = 0; i < MacCAPM$NUM_SUPERFRAMES && !tx_busy; i++) {
          tx_busy = MacCAPM$cap[i].sendState != MacCAPM$CAP_SEND_IDLE && MacCAPM$cap[i].sendState != MacCAPM$CAP_SEND_OFF;
        }
      if (tx_busy) 
        {
          ;
          MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_PENDING;
          return SUCCESS;
        }
    }

  return MacCAPM$StartSending(sfIndex);
}

# 267 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static result_t MacDataM$Confirm(uint8_t sf, uint8_t msduHandle, TMacStatus status)
{
  if (sf == MacDataM$PARENT_SF) {
    return MacDataM$IMacDataParent$Confirm(msduHandle, status);
    }
  else {
#line 272
    return MacDataM$IMacDataOwn$Confirm(msduHandle, status);
    }
}

# 77 "../../../zigzag/ZigNet2SpiM.nc"
static  void ZigNet2SpiM$IZigDATA$confirm(uint8_t nsduHandler, NwkStatus status)
{
  if (!ZigNet2SpiM$sending) {
      TOSH_CLR_SATELLITE_INTA_2_PIN();
      return;
    }
  else {
#line 83
    ZigNet2SpiM$sending = FALSE;
    }
  TOSH_CLR_SATELLITE_INTA_2_PIN();
  return;
  if (0x20 == nsduHandler) {
      if (NWK_SUCCESS == status) {
        TOSH_CLR_SATELLITE_INTA_2_PIN();
        }
    }
#line 91
  ZigNet2SpiM$sendEvents();
  return;
}

#line 66
static void ZigNet2SpiM$sendEvents(void)
{
  if (FALSE == ZigNet2SpiM$sending) {
      if (TOS_post(ZigNet2SpiM$taskSendEvents)) {
          ZigNet2SpiM$sending = TRUE;
        }
      else 
#line 71
        {
          TOSH_CLR_SATELLITE_INTA_2_PIN();
        }
    }
}

# 127 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static  TMacShortAddress MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 131
  return MacCommonAttrM$pib.macShortAddress;
}

# 156 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static bool RouterM$sendFrame(uint8_t *frame, uint8_t length)
{
  PanID_t panid = RouterM$IMacGET$GetmacPANId(NULL);
  TMacAddress myAddr_mac;
  TMacAddress nextHop_mac;
  NwkRelationship direction;
  TxOptions tx_opts;

  {
    NwkAddr myAddr = RouterM$IMacGET$GetmacShortAddress(NULL);
    NwkAddr nextHop;
    bool indirect;

    if (((NwkHeader *)frame)->destinationAddr == NWK_BROADCAST) 
      {
        nextHop = NWK_BROADCAST;
        direction = NWK_CHILD;
        tx_opts.indirect = 0;
        tx_opts.acknowledged = 0;
      }
    else 
      {
        if (FAIL == RouterM$get_relaying_options(((NwkHeader *)frame)->destinationAddr, &nextHop, &direction, &indirect)) {
          return FAIL;
          }

        tx_opts.indirect = indirect;
        tx_opts.acknowledged = 1;
      }
    tx_opts.secure = 0;
    tx_opts.gts = 0;

    RouterM$IMacAddress$SetShortAddress(&myAddr_mac, myAddr);
    RouterM$IMacAddress$SetShortAddress(&nextHop_mac, nextHop);
  }

  RouterM$current_msduHandle = RouterM$newMsduHandle();
  RouterM$wait_confirm = TRUE;

  if (direction == NWK_PARENT) 
    {

      RouterM$UpperIf$Request(
      panid, myAddr_mac, 
      panid, nextHop_mac, 
      length, frame, 
      RouterM$current_msduHandle, 
      tx_opts);
    }
  else 

    {

      RouterM$LowerIf$Request(
      panid, myAddr_mac, 
      panid, nextHop_mac, 
      length, frame, 
      RouterM$current_msduHandle, 
      tx_opts);
    }

  return SUCCESS;
}

# 107 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static  TMacPANId MacCommonAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 111
  return MacCommonAttrM$pib.macPANId;
}

# 31 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static NwkAddr NwkAddressingM$countCskip(uint8_t depth)
{
  uint8_t Rm = NwkAddressingM$NIB$getNwkMaxRouters();
  uint8_t Lm = NwkAddressingM$NIB$getNwkMaxDepth();
  uint8_t Cm = NwkAddressingM$NIB$getNwkMaxChildren();
  NwkAddr csk;


  if (Lm == depth) {
    csk = 0;
    }
  else {
#line 41
    if (Rm == 1) {
      csk = 1 + (uint16_t )Cm * (Lm - depth - 1);
      }
    else {
        uint8_t i;
        uint16_t t = 1;

#line 47
        for (i = 0; i < Lm - depth - 1; i++) 
          t *= Rm;
        csk = (int16_t )(1 + Cm - Rm - Cm * t) / (int16_t )(1 - Rm);
      }
    }
  return csk;
}

# 37 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressM.nc"
static  result_t MacAddressM$IMacAddress$SetShortAddress(TMacAddress *const pMacAddress, 
const TMacShortAddress macShortAddress)
{
  if (pMacAddress != NULL) {
      ((_TMacAddress *)pMacAddress)->mode = _MAC_ADDRESS_SHORT;
      ((_TMacAddress *)pMacAddress)->address.shortAddress = macShortAddress;
      return SUCCESS;
    }
  return FAIL;
}

# 102 "../../../zigzag/ZigBee/implementation/NeighborTableM.nc"
static  result_t NeighborTableM$NeighborTable$getByAddr(TMacAddress addr, NwkNeighbor **pneighbor)
{

  uint8_t i;

  if (NeighborTableM$IMacAddress$GetAddressMode(addr) == MAC_ADDRESS_SHORT) 
    {
      TMacShortAddress a;

#line 110
      NeighborTableM$IMacAddress$GetShortAddress(&a, addr);
      for (i = 0; i < 20; i++) 
        {
          if (a == NeighborTableM$Table[i].networkAddr) 
            {
              *pneighbor = NeighborTableM$Table + i;
              return SUCCESS;
            }
        }
    }
  if (NeighborTableM$IMacAddress$GetAddressMode(addr) == MAC_ADDRESS_EXTENDED) 
    {
      TMacExtendedAddress a;

#line 123
      NeighborTableM$IMacAddress$GetExtendedAddress(&a, addr);
      for (i = 0; i < 20; i++) 
        {
          if (a == NeighborTableM$Table[i].extendedAddr) 
            {
              *pneighbor = NeighborTableM$Table + i;
              return SUCCESS;
            }
        }
    }

  return FAIL;
}

# 13 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressM.nc"
static  result_t MacAddressM$IMacAddress$GetShortAddress(TMacShortAddress *const pMacShortAddress, 
const TMacAddress macAddress)
{
  TMacAddressMode mode = MacAddressM$IMacAddress$GetAddressMode(macAddress);

#line 17
  if (mode == MAC_ADDRESS_SHORT) {
    if (pMacShortAddress != NULL) {
        *pMacShortAddress = ((_TMacAddress )macAddress).address.shortAddress;
        return SUCCESS;
      }
    }
#line 22
  return FAIL;
}

static  result_t MacAddressM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const pMacExtendedAddress, 
const TMacAddress macAddress)
{
  TMacAddressMode mode = MacAddressM$IMacAddress$GetAddressMode(macAddress);

#line 29
  if (mode == MAC_ADDRESS_EXTENDED) {
    if (pMacExtendedAddress != NULL) {
        *pMacExtendedAddress = ((_TMacAddress )macAddress).address.extendedAddress;
        return SUCCESS;
      }
    }
#line 34
  return FAIL;
}

# 15 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static  result_t MacFrameFormatM$IMacFrame$SetFrameType(TMacFrame *const pMacFrame, const TMacFrameType macFrameType)
{
  if (pMacFrame != NULL) {
      pMacFrame->frameControl.lsb.value.frameType = macFrameType;
      return SUCCESS;
    }
  return FAIL;
}

#line 52
static  result_t MacFrameFormatM$IMacFrame$SetFramePending(TMacFrame *const pMacFrame, const bool framePending)
{
  if (pMacFrame != NULL) {
      pMacFrame->frameControl.lsb.value.framePending = framePending ? 1 : 0;
      return SUCCESS;
    }
  return FAIL;
}

#line 33
static  result_t MacFrameFormatM$IMacFrame$SetSecurityEnabled(TMacFrame *const pMacFrame, const bool securityEnabled)
{
  if (pMacFrame != NULL) {
      pMacFrame->frameControl.lsb.value.securityEnabled = securityEnabled ? 1 : 0;
      return SUCCESS;
    }
  return FAIL;
}

#line 89
static  result_t MacFrameFormatM$IMacFrame$SetIntraPAN(TMacFrame *const pMacFrame, const bool intraPAN)
{
  if (pMacFrame != NULL) {
      pMacFrame->frameControl.lsb.value.intraPAN = intraPAN ? 1 : 0;
      return SUCCESS;
    }
  return FAIL;
}

#line 184
static  result_t MacFrameFormatM$IMacFrame$SetSrcPANId(TMacFrame *const pMacFrame, 
const TMacPANId srcPANId)
{
  if (pMacFrame != NULL) {
      pMacFrame->srcPANId.value = srcPANId;
      return SUCCESS;
    }
  return FAIL;
}










static  result_t MacFrameFormatM$IMacFrame$SetSrcAddress(TMacFrame *const pMacFrame, const TMacAddress srcAddress)
{
  if (pMacFrame != NULL) {
      result_t result;
      TMacAddressMode srcAddressMode = MacFrameFormatM$IMacAddress$GetAddressMode(srcAddress);

#line 208
      switch (srcAddressMode) {
          case MAC_ADDRESS_NOT_PRESENT: 
            break;
          case MAC_ADDRESS_SHORT: 
            result = MacFrameFormatM$IMacAddress$GetShortAddress(& pMacFrame->srcAddress._short.value, srcAddress);
          if (FAIL == result) {
#line 213
            return FAIL;
            }
#line 214
          break;
          case MAC_ADDRESS_EXTENDED: 
            result = MacFrameFormatM$IMacAddress$GetExtendedAddress(& pMacFrame->srcAddress._extended.value, srcAddress);
          if (FAIL == result) {
#line 217
            return FAIL;
            }
#line 218
          break;
          default: return FAIL;
        }
      pMacFrame->frameControl.msb.value.srcAddressMode = srcAddressMode;
      return SUCCESS;
    }
  return FAIL;
}

#line 127
static  result_t MacFrameFormatM$IMacFrame$SetDstPANId(TMacFrame *const pMacFrame, const TMacPANId dstPANId)
{
  if (pMacFrame != NULL) {
      pMacFrame->dstPANId.value = dstPANId;
      return SUCCESS;
    }
  return FAIL;
}










static  result_t MacFrameFormatM$IMacFrame$SetDstAddress(TMacFrame *const pMacFrame, const TMacAddress dstAddress)
{
  if (pMacFrame != NULL) {
      result_t result;
      TMacAddressMode dstAddressMode = MacFrameFormatM$IMacAddress$GetAddressMode(dstAddress);

#line 150
      switch (dstAddressMode) {
          case MAC_ADDRESS_NOT_PRESENT: 
            break;
          case MAC_ADDRESS_SHORT: 
            result = MacFrameFormatM$IMacAddress$GetShortAddress(& pMacFrame->dstAddress._short.value, dstAddress);
          if (FAIL == result) {
#line 155
            return FAIL;
            }
#line 156
          break;
          case MAC_ADDRESS_EXTENDED: 
            result = MacFrameFormatM$IMacAddress$GetExtendedAddress(& pMacFrame->dstAddress._extended.value, dstAddress);
          if (FAIL == result) {
#line 159
            return FAIL;
            }
#line 160
          break;
          default: return FAIL;
        }
      pMacFrame->frameControl.msb.value.dstAddressMode = dstAddressMode;
      return SUCCESS;
    }
  return FAIL;
}

# 26 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static  TMacDSN MacCommonAttrM$IMacCommonAttr$GetmacDSN(TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 30
  return MacCommonAttrM$pib.macDSN;
}

# 107 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static  result_t MacFrameFormatM$IMacFrame$SetSequenceNumber(TMacFrame *const pMacFrame, 
const TMacSequenceNumber sequenceNumber)
{
  if (pMacFrame != NULL) {
      pMacFrame->sequenceNumber.value = sequenceNumber;
      return SUCCESS;
    }
  return FAIL;
}

#line 242
static  result_t MacFrameFormatM$IMacFrame$SetPayload(TMacFrame *const pMacFrame, 
const uint8_t *const pFramePayload, TMacPayloadLength payloadLength)
{
  if (pMacFrame != NULL) {
      if (pFramePayload != NULL && 0 < payloadLength) {
          TMacPayloadLength octetIndex;

#line 248
          for (octetIndex = 0; octetIndex < payloadLength; octetIndex += 1) 
            pMacFrame->payload.raw[octetIndex] = pFramePayload[octetIndex];
          pMacFrame->payload.length = payloadLength;
        }
      else {
#line 252
        pMacFrame->payload.length = 0;
        }
#line 253
      return SUCCESS;
    }
  return FAIL;
}

#line 71
static  result_t MacFrameFormatM$IMacFrame$SetAckRequest(TMacFrame *const pMacFrame, const bool ackRequest)
{
  if (pMacFrame != NULL) {
      pMacFrame->frameControl.lsb.value.ackRequest = ackRequest ? 1 : 0;
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

# 410 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static  void RouterM$relayNext(void)

{
  if (!RouterM$wait_confirm && RouterM$NIB$isJoined()) 
    {
      uint8_t cur;
      uint8_t i;

#line 417
      cur = RouterM$cur_frame;

      for (i = 0; i < RouterM$NWK_FRAME_BUFFER_SIZE; i++) 
        {
          if (RouterM$frames[cur].state == RouterM$NWK_FRAME_PENDING) 
            {
              break;
            }
          cur++;
          if (cur == RouterM$NWK_FRAME_BUFFER_SIZE) {
            cur = 0;
            }
        }
      if (i != RouterM$NWK_FRAME_BUFFER_SIZE) 
        {
          RouterM$relaying = TRUE;
          RouterM$sendFrame(RouterM$frames[cur].msdu, RouterM$frames[cur].msduLength);
          RouterM$cur_frame = cur;
        }
    }
}

# 274 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static result_t MacCAPM$nextBackoff(uint8_t sfIndex)
{
  if (++ MacCAPM$cap[sfIndex].backoffExponent > MAC_AMAX_BE) {
    MacCAPM$cap[sfIndex].backoffExponent = MAC_AMAX_BE;
    }
#line 278
  if (++ MacCAPM$cap[sfIndex].backoffNumber <= MacCAPM$macMaxCSMABackoffs) {
      MacCAPM$cap[sfIndex].backoffRemain = MacCAPM$calcBackoff(sfIndex);
      return SUCCESS;
    }
  else 
    {
      ;
      return FAIL;
    }
}

# 155 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static  result_t MacPoolM$IMacPool$Free(const TMacPoolHandle macPoolHandle, const TMacStatus status)
{
  MacPoolM$TMacPoolIndex itemIndex;

#line 158
  if (!MacPoolM$IsValidHandle(macPoolHandle)) {
#line 158
    return FAIL;
    }
#line 159
  itemIndex = MacPoolM$ToPoolIndex(macPoolHandle);
  if (MacPoolM$MAC_POOL_ITEM_ALLOC != MacPoolM$pool[itemIndex].state) {
#line 160
    return FAIL;
    }
#line 161
  MacPoolM$pool[itemIndex].status = status;
  MacPoolM$pool[itemIndex].state = MacPoolM$MAC_POOL_ITEM_FREE_DONE;
  if (TOS_post(MacPoolM$FreeDone)) {
    return SUCCESS;
    }
#line 165
  return FAIL;
}

# 48 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressM.nc"
static  result_t MacAddressM$IMacAddress$SetExtendedAddress(TMacAddress *const pMacAddress, 
const TMacExtendedAddress macExtendedAddress)
{
  if (pMacAddress != NULL) {
      ((_TMacAddress *)pMacAddress)->mode = _MAC_ADDRESS_EXTENDED;
      ((_TMacAddress *)pMacAddress)->address.extendedAddress = macExtendedAddress;
      return SUCCESS;
    }
  return FAIL;
}

# 87 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
static  void NLME_JoinParentM$IMacCOMM_STATUS$Indication(TMacPANId panID, TMacAddress src, TMacAddress dst, TMacStatus status)
{
  ;
  if (NLME_JoinParentM$state != NLME_JoinParentM$WAIT_COMM_STATUS) {
    return;
    }
  if (status == MAC_SUCCESS && NLME_JoinParentM$addr_allocated) 
    {
      NwkNeighbor neighbor;








      neighbor.panID = NLME_JoinParentM$IMacGET$GetmacPANId(NULL);
      neighbor.networkAddr = NLME_JoinParentM$joiningNwkAddr;
      neighbor.extendedAddr = NLME_JoinParentM$joiningExtAddr;
      neighbor.deviceType = NLME_JoinParentM$joiningCapability.deviceType == 1 ? ZIGBEE_ROUTER : ZIGBEE_END_DEVICE;
      neighbor.rxOnWhenIdle = NLME_JoinParentM$joiningCapability.receiverOnWhenIdle;
      neighbor.relationship = NWK_CHILD;
      neighbor.depth = NLME_JoinParentM$NIB$getDepth() + 1;

      neighbor.permitJoining = NLME_JoinParentM$joiningCapability.deviceType == 1;
      neighbor.transmitFailure = 0;
      neighbor.potentialParent = NLME_JoinParentM$joiningCapability.deviceType == 1;

      neighbor.logicalChannel = NLME_JoinParentM$NIB$getChannel();
      neighbor.lastFrameTime = NLME_JoinParentM$ILocalTime$Read();



      NLME_JoinParentM$NeighborTable$update(neighbor);

      NLME_JoinParentM$updateBeacon();

      NLME_JoinParentM$NLME_JoinParent$indication(NLME_JoinParentM$joiningNwkAddr, NLME_JoinParentM$joiningExtAddr, NLME_JoinParentM$joiningCapability, FALSE);

      if (NLME_JoinParentM$joiningCapability.deviceType == 1) 
        {
          uint8_t bo = NLME_JoinParentM$IMacGET$GetmacBeaconOrder(NULL);



          if (bo < 15 && NLME_JoinParentM$ITimerSymbol$SetOneShot(NLME_JoinParentM$WAIT_BEACONS * ((uint32_t )MAC_ABASE_SUPERFRAME_DURATION << bo), 0)) {
            return;
            }
        }
    }
#line 137
  NLME_JoinParentM$state = NLME_JoinParentM$IDLE;
}

# 55 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static  void NwkAddressingM$Reset$reset(void)
{
  uint8_t Rm = NwkAddressingM$NIB$getNwkMaxRouters();
  uint8_t my_depth = NwkAddressingM$NIB$getDepth();

#line 59
  NwkAddressingM$cskip = NwkAddressingM$countCskip(my_depth);
  NwkAddressingM$edAddrNum = 0;
  NwkAddressingM$routerAddrNum = 0;

  NwkAddressingM$nextRouterAddr = NwkAddressingM$IMacGET$GetmacShortAddress(NULL) + 1;
  NwkAddressingM$nextEDAddr = NwkAddressingM$IMacGET$GetmacShortAddress(NULL) + NwkAddressingM$cskip * Rm + 1;
  ;
}

#line 164
static  bool NwkAddressingM$NwkAddressing$hasVacantAddrs(void)
{
  uint8_t routers;
  uint8_t eds;

#line 168
  NwkAddressingM$countChildren(&routers, &eds);
  return NwkAddressingM$cskip != 0 && eds + routers < NwkAddressingM$NIB$getNwkMaxChildren();
}

# 55 "../../../zigzag/ZigCoordM.nc"
static  void ZigCoordM$IZigNetFormation$confirm(NwkStatus status)
{
  if (NWK_SUCCESS == status) {
      ;
      {
#line 59
        ZigCoordM$IZigPermitJoining$request(0xff) == NWK_SUCCESS;
      }
#line 59
      ;
      ZigCoordM$ChildSupervisorControl$start();
      return;
    }
  else 
#line 62
    {
      ;

      return;
    }
}

# 113 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacPANId(TMacPANId macPANId)
{
  if (SUCCESS == MacCommonAttrM$IPhyAttr$SetpanId(macPANId)) {
      MacCommonAttrM$pib.macPANId = macPANId;
      return MAC_SUCCESS;
    }
  return MAC_INVALID_PARAMETER;
}

# 136 "../../../zigzag/ZigBee/implementation/NLME_NetworkFormationM.nc"
static  result_t NLME_NetworkFormationM$IMacSTART$Confirm(TMacStatus status)
{
  if (status == MAC_SUCCESS) 
    {
      NLME_NetworkFormationM$NIB$setOnline(NWK_ROUTING);
      NLME_NetworkFormationM$NIB$setChannel(NLME_NetworkFormationM$channel);
    }
  NLME_NetworkFormationM$NLME_NetworkFormation$confirm(status);
  return SUCCESS;
}

# 147 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static void MacSuperframesM$newCycle(void)
{


  if (!MacSuperframesM$parent_active && !MacSuperframesM$own_active) {
    return;
    }

  if (MacSuperframesM$parent_active && MacSuperframesM$own_active) {
    MacSuperframesM$current_sf = !MacSuperframesM$current_sf;
    }
  else 
    {
      if (MacSuperframesM$parent_active) {
        MacSuperframesM$current_sf = MacSuperframesM$SF_PARENT;
        }
      else {
#line 163
        MacSuperframesM$current_sf = MacSuperframesM$SF_OWN;
        }
    }

  if (MacSuperframesM$current_sf == MacSuperframesM$SF_PARENT) 
    {
      MacSuperframesM$current_handler = MacSuperframesM$parentPreparationHandler;
      MacSuperframesM$next_sf_start = MacSuperframesM$timestamp + MacSuperframesM$beacon_interval;

      MacSuperframesM$Timer$SetOneShotAt(MacSuperframesM$next_sf_start - MacSuperframesM$PARENT_SF_PREPARATION_TIME, 0);
    }
  else 


    {
      MacSuperframesM$current_handler = MacSuperframesM$ownPreparationHandler;
      MacSuperframesM$next_sf_start = MacSuperframesM$timestamp + MacSuperframesM$sf_offset;
      TOS_post(MacSuperframesM$createBeacon);
      MacSuperframesM$Timer$SetOneShotAt(MacSuperframesM$next_sf_start - MacSuperframesM$OWN_SF_PREPARATION_TIME, 0);
    }





  MacSuperframesM$sleepIndication(MacSuperframesM$ITimeCast$SymbolsToMillis(MacSuperframesM$next_sf_start - MacSuperframesM$ILocalTime$Read()));
}

# 142 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static result_t TimerSymbolAsyncM$StartTimerAt(const uint8_t timerNumber, 
const uint32_t time, const uint32_t period, 
const TUniData uniData, const bool isPeriodic)
{
  result_t result = FAIL;

#line 147
  if (timerNumber < TimerSymbolAsyncM$ASYNC_TIMER_NUM) {
#line 147
    { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 147
      {
        if (FALSE == TimerSymbolAsyncM$timer[timerNumber].isSet) {
            TimerSymbolAsyncM$SetTimerAt(timerNumber, time);
            TimerSymbolAsyncM$timer[timerNumber].shotTime = time;
            TimerSymbolAsyncM$timer[timerNumber].period = period;
            TimerSymbolAsyncM$timer[timerNumber].isPeriodic = isPeriodic;
            TimerSymbolAsyncM$timer[timerNumber].isSet = TRUE;
            TimerSymbolAsyncM$timer[timerNumber].uniData = uniData;
            result = SUCCESS;
          }
      }
#line 157
      __nesc_atomic_end(__nesc_atomic); }
    }
#line 158
  return result;
}

#line 86
static void TimerSymbolAsyncM$SetTimerAt(const uint8_t timerNumber, const uint32_t time)
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

# 65 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static   uint16_t TimerSymbol2M$overflowCount(void)
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

# 296 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
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

# 30 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconPackerM.nc"
static  void MacBeaconPackerM$packBeacon(TPhyFrame *phyFrame, uint64_t *ptime)
{

  TMacFrame macFrame;


  MacBeaconPackerM$IMacFrame$SetFrameType(&macFrame, MAC_FRAME_TYPE_BEACON);
  MacBeaconPackerM$IMacFrame$SetSecurityEnabled(&macFrame, FALSE);
  MacBeaconPackerM$IMacFrame$SetFramePending(&macFrame, FALSE);
  MacBeaconPackerM$IMacFrame$SetAckRequest(&macFrame, FALSE);
  MacBeaconPackerM$IMacFrame$SetIntraPAN(&macFrame, FALSE);



  {
    uint8_t bsn;

#line 46
    bsn = MacBeaconPackerM$IMacBeaconAttr$GetmacBSN(NULL);
    MacBeaconPackerM$IMacFrame$SetSequenceNumber(&macFrame, bsn);
    MacBeaconPackerM$IMacBeaconAttr$SetmacBSN(bsn + 1);
  }



  {
    TMacAddress ma;
    TMacShortAddress sa = MacBeaconPackerM$IMacCommonAttr$GetmacShortAddress(NULL);

#line 56
    if (sa < 0xfffe) 
      {
        MacBeaconPackerM$IMacAddress$SetShortAddress(&ma, sa);
        MacBeaconPackerM$IMacFrame$SetSrcAddress(&macFrame, ma);
      }
    else 
      {
        MacBeaconPackerM$IMacAddress$SetExtendedAddress(&ma, info_param.MAC_ADDRESS);
        MacBeaconPackerM$IMacFrame$SetSrcAddress(&macFrame, ma);
      }
    MacBeaconPackerM$IMacFrame$SetSrcPANId(&macFrame, MacBeaconPackerM$IMacCommonAttr$GetmacPANId(NULL));


    MacBeaconPackerM$IMacAddress$SetEmptyAddress(&ma);
    MacBeaconPackerM$IMacFrame$SetDstAddress(&macFrame, ma);
  }


  {
    uint8_t finalCap;
    uint8_t framePayload[MAC_AMAX_MAC_FRAME_SIZE];
    uint8_t beaconOffset = 0;








    finalCap = MAC_ANUM_SUPERFRAME_SLOTS - 1;


    {
      TMacSuperframeSpec *sf = (TMacSuperframeSpec *)framePayload;

#line 91
      sf->beaconOrder = MacBeaconPackerM$IMacSuperframeAttr$GetmacBeaconOrder(NULL);
      sf->superframeOrder = MacBeaconPackerM$IMacSuperframeAttr$GetmacSuperframeOrder(NULL);
      sf->finalCapSlot = finalCap;
      sf->battLifeExtension = MacBeaconPackerM$IMacCAPAttr$GetmacBattLifeExt(NULL) ? 1 : 0;
      sf->panCoordinator = MacBeaconPackerM$IMacCommonAttr$GetmacShortAddress(NULL) ? 0 : 1;
      sf->associationPermit = MacBeaconPackerM$IMacAssocAttr$GetmacAssociationPermit(NULL) ? 1 : 0;
      (

      (uint16_t *)framePayload)[0] = ToLSB16(* (uint16_t *)sf);
    }

    beaconOffset = 2;

    framePayload[beaconOffset] = 0;
    beaconOffset++;
#line 171
    framePayload[beaconOffset++] = 0;



    {
      uint8_t i;
      uint8_t payloadLength;


      uint8_t *beaconPayload = MacBeaconPackerM$IMacBeaconAttr$GetmacBeaconPayload(NULL, NULL);

#line 181
      payloadLength = MacBeaconPackerM$IMacBeaconAttr$GetmacBeaconPayloadLength(NULL);


      for (i = 0; i < payloadLength; i++, beaconOffset++) {
          framePayload[beaconOffset] = beaconPayload[i];
        }



      if (payloadLength > 0) 
        {
          uint64_t time = ptime ? *ptime : 0;

#line 193
          for (i = 0; i < 8; i++) 
            framePayload[beaconOffset++] = ((uint8_t *)&time)[i];
          framePayload[beaconOffset++] = MacBeaconPackerM$getLastBSN();
        }


      MacBeaconPackerM$IMacFrame$SetPayload(&macFrame, framePayload, beaconOffset);
    }
  }



  MacBeaconPackerM$IMacFrame$Pack(&macFrame, phyFrame);
}

# 59 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressM.nc"
static  result_t MacAddressM$IMacAddress$SetEmptyAddress(TMacAddress *const pMacAddress)
{
  if (pMacAddress != NULL) {
      ((_TMacAddress *)pMacAddress)->mode = _MAC_ADDRESS_NOT_PRESENT;
      return SUCCESS;
    }
  return FAIL;
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

#line 285
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

# 246 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static void TimerSymbol2M$CheckLongTimers(void)
{
  uint8_t head = TimerSymbol2M$longTimersHead;

#line 249
  TimerSymbol2M$longTimersHead = TimerSymbol2M$EMPTY_LIST;
  TimerSymbol2M$ExecuteTimers(head);
  TimerSymbol2M$SetNextShortEvent();
}

# 58 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
 __attribute((wakeup)) __attribute((interrupt(18))) void sig_UART0RX_VECTOR(void)
#line 58
{
  uint8_t temp = U0RXBUF;

#line 60
  HPLUSART0M$USARTData$rxDone(temp);
}

 __attribute((wakeup)) __attribute((interrupt(16))) void sig_UART0TX_VECTOR(void)
#line 63
{
  if (HPLUSART0M$USARTControl$isI2C()) {
    HPLUSART0M$HPLI2CInterrupt$fired();
    }
  else {
#line 67
    HPLUSART0M$USARTData$txDone();
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

# 49 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static   result_t PhyFrameM$IPhyFrame$ReadOctet(TPhyFrame *const pPhyFrame, uint8_t *const pOctet)
{
  if (pPhyFrame != NULL && pOctet != NULL) {
    if (pPhyFrame->current < PHY_AMAX_PHY_PACKET_SIZE) {
      if (pPhyFrame->current < pPhyFrame->data[0]) {
          *pOctet = pPhyFrame->data[PHY_SIZE_OF_FRAME_LENGTH + pPhyFrame->current];
          pPhyFrame->current += 1;
          return SUCCESS;
        }
      }
    }
#line 58
  return FAIL;
}

# 24 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static  result_t MacFrameFormatM$IMacFrame$GetFrameType(const TMacFrame *const pMacFrame, TMacFrameType *const pFrameType)
{
  if (pMacFrame != NULL && pFrameType != NULL) {
      *pFrameType = pMacFrame->frameControl.lsb.value.frameType;
      return SUCCESS;
    }
  return FAIL;
}

#line 258
static  result_t MacFrameFormatM$IMacFrame$GetPayloadLength(const TMacFrame *const pMacFrame, 
TMacPayloadLength *const pPayloadLength)
{
  if (pMacFrame != NULL && pPayloadLength != NULL) {
      *pPayloadLength = pMacFrame->payload.length;
      return SUCCESS;
    }
  return FAIL;
}

static  result_t MacFrameFormatM$IMacFrame$GetPayload(const TMacFrame *const pMacFrame, 
uint8_t *const pFramePayload)
{
  if (pMacFrame != NULL && pFramePayload != NULL) {
      TMacPayloadLength octetIndex;

#line 273
      for (octetIndex = 0; octetIndex < pMacFrame->payload.length; octetIndex += 1) 
        pFramePayload[octetIndex] = pMacFrame->payload.raw[octetIndex];
      return SUCCESS;
    }
  return FAIL;
}

#line 227
static  result_t MacFrameFormatM$IMacFrame$GetSrcAddress(const TMacFrame *const pMacFrame, TMacAddress *const pSrcAddress)
{
  if (pMacFrame != NULL && pSrcAddress != NULL) {
      switch (pMacFrame->frameControl.msb.value.srcAddressMode) {
          case MAC_ADDRESS_NOT_PRESENT: 
            return MacFrameFormatM$IMacAddress$SetEmptyAddress(pSrcAddress);
          case MAC_ADDRESS_SHORT: 
            return MacFrameFormatM$IMacAddress$SetShortAddress(pSrcAddress, pMacFrame->srcAddress._short.value);
          case MAC_ADDRESS_EXTENDED: 
            return MacFrameFormatM$IMacAddress$SetExtendedAddress(pSrcAddress, pMacFrame->srcAddress._extended.value);
        }
    }
  return FAIL;
}

# 93 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static bool NwkAddressingM$checkFree(NwkAddr addr)
{
  NwkNeighbor *pn;

#line 96
  pn = NULL;
  while (NwkAddressingM$NeighborTable$getNextPtr(&pn)) 


    if (pn->networkAddr == addr && pn->relationship == NWK_CHILD) {
      return FALSE;
      }
#line 102
  return TRUE;
}

# 94 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
static  result_t MacAssocCoordM$IMacASSOCIATE$Response(TMacExtendedAddress deviceAddress, 
TMacShortAddress assocShortAddress, 
TMacStatus status, 
bool securityEnable)
{

  TMacFrame AssocResponse;

  result_t result;

#line 103
  result = MacAssocCoordM$MakeAssocResponse(deviceAddress, assocShortAddress, 
  status, securityEnable, &AssocResponse);
  if (result == SUCCESS) {

      result = MacAssocCoordM$IMacSendOwn$Send(&AssocResponse, 0);
      if (result == SUCCESS) 
        {
          MacAssocCoordM$dstExtAddress = deviceAddress;
          return SUCCESS;
        }
    }

  MacAssocCoordM$FailureResponse(MAC_TRANSACTION_OVERFLOW, deviceAddress);
  return FAIL;
}

# 117 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static  result_t MacFrameFormatM$IMacFrame$GetSequenceNumber(const TMacFrame *const pMacFrame, 
TMacSequenceNumber *const pSequenceNumber)
{
  if (pMacFrame != NULL && pSequenceNumber != NULL) {
      *pSequenceNumber = pMacFrame->sequenceNumber.value;
      return SUCCESS;
    }
  return FAIL;
}

#line 80
static  result_t MacFrameFormatM$IMacFrame$GetAckRequest(const TMacFrame *const pMacFrame, bool *const pAckRequest)
{
  if (pMacFrame != NULL && pAckRequest != NULL) {
      *pAckRequest = 1 == pMacFrame->frameControl.lsb.value.ackRequest;
      return SUCCESS;
    }
  return FAIL;
}

#line 169
static  result_t MacFrameFormatM$IMacFrame$GetDstAddress(const TMacFrame *const pMacFrame, TMacAddress *const pDstAddress)
{
  if (pMacFrame != NULL && pDstAddress != NULL) {
      switch (pMacFrame->frameControl.msb.value.dstAddressMode) {
          case MAC_ADDRESS_NOT_PRESENT: 
            return MacFrameFormatM$IMacAddress$SetEmptyAddress(pDstAddress);
          case MAC_ADDRESS_SHORT: 
            return MacFrameFormatM$IMacAddress$SetShortAddress(pDstAddress, pMacFrame->dstAddress._short.value);
          case MAC_ADDRESS_EXTENDED: 
            return MacFrameFormatM$IMacAddress$SetExtendedAddress(pDstAddress, pMacFrame->dstAddress._extended.value);
        }
    }
  return FAIL;
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

# 160 "/home/max/tinyos/tinyos-1.x/tos/platform/pult/UniSART1M.nc"
static   result_t UniSART1M$USARTControl$isRxIntrPending(void)
#line 160
{
  if (UniSART1M$IFG2 & (1 << 4)) {
      UniSART1M$IFG2 &= ~(1 << 4);
      return SUCCESS;
    }
  return FAIL;
}

# 85 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
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

