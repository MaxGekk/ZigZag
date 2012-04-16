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
#line 137
volatile unsigned char P2IFG __asm ("0x002B");



volatile unsigned char P2IE __asm ("0x002D");
#line 154
volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");
# 85 "/opt/msp430-3.3.6/msp430/include/msp430/usart.h"
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");

volatile unsigned char U0TXBUF __asm ("0x0077");
#line 258
volatile unsigned char U1TCTL __asm ("0x0079");
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
#line 83
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
# 116 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430ADC12.h"
#line 105
typedef struct __nesc_unnamed4259 {

  unsigned int refVolt2_5 : 1;
  unsigned int clockSourceSHT : 2;
  unsigned int clockSourceSAMPCON : 2;
  unsigned int clockDivSAMPCON : 2;
  unsigned int referenceVoltage : 3;
  unsigned int clockDivSHT : 3;
  unsigned int inputChannel : 4;
  unsigned int sampleHoldTime : 4;
  unsigned int  : 0;
} MSP430ADC12Settings_t;






#line 118
typedef enum __nesc_unnamed4260 {

  MSP430ADC12_FAIL = 0, 
  MSP430ADC12_SUCCESS = 1, 
  MSP430ADC12_DELAYED = 2
} msp430ADCresult_t;

enum refVolt2_5_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1
};

enum clockDivSHT_enum {

  SHT_CLOCK_DIV_1 = 0, 
  SHT_CLOCK_DIV_2 = 1, 
  SHT_CLOCK_DIV_3 = 2, 
  SHT_CLOCK_DIV_4 = 3, 
  SHT_CLOCK_DIV_5 = 4, 
  SHT_CLOCK_DIV_6 = 5, 
  SHT_CLOCK_DIV_7 = 6, 
  SHT_CLOCK_DIV_8 = 7
};

enum clockDivSAMPCON_enum {

  SAMPCON_CLOCK_DIV_1 = 0, 
  SAMPCON_CLOCK_DIV_2 = 1, 
  SAMPCON_CLOCK_DIV_3 = 2, 
  SAMPCON_CLOCK_DIV_4 = 3
};

enum clockSourceSAMPCON_enum {

  SAMPCON_SOURCE_TACLK = 0, 
  SAMPCON_SOURCE_ACLK = 1, 
  SAMPCON_SOURCE_SMCLK = 2, 
  SAMPCON_SOURCE_INCLK = 3
};

enum inputChannel_enum {


  INPUT_CHANNEL_A0 = 0, 
  INPUT_CHANNEL_A1 = 1, 
  INPUT_CHANNEL_A2 = 2, 
  INPUT_CHANNEL_A3 = 3, 
  INPUT_CHANNEL_A4 = 4, 
  INPUT_CHANNEL_A5 = 5, 
  INPUT_CHANNEL_A6 = 6, 
  INPUT_CHANNEL_A7 = 7, 
  EXTERNAL_REFERENCE_VOLTAGE = 8, 
  REFERENCE_VOLTAGE_NEGATIVE_TERMINAL = 9, 
  INTERNAL_TEMPERATURE = 10, 
  INTERNAL_VOLTAGE = 11
};

enum referenceVoltage_enum {

  REFERENCE_AVcc_AVss = 0, 
  REFERENCE_VREFplus_AVss = 1, 
  REFERENCE_VeREFplus_AVss = 2, 
  REFERENCE_AVcc_VREFnegterm = 4, 
  REFERENCE_VREFplus_VREFnegterm = 5, 
  REFERENCE_VeREFplus_VREFnegterm = 6
};

enum clockSourceSHT_enum {

  SHT_SOURCE_ADC12OSC = 0, 
  SHT_SOURCE_ACLK = 1, 
  SHT_SOURCE_MCLK = 2, 
  SHT_SOURCE_SMCLK = 3
};

enum sampleHold_enum {

  SAMPLE_HOLD_4_CYCLES = 0, 
  SAMPLE_HOLD_8_CYCLES = 1, 
  SAMPLE_HOLD_16_CYCLES = 2, 
  SAMPLE_HOLD_32_CYCLES = 3, 
  SAMPLE_HOLD_64_CYCLES = 4, 
  SAMPLE_HOLD_96_CYCLES = 5, 
  SAMPLE_HOLD_123_CYCLES = 6, 
  SAMPLE_HOLD_192_CYCLES = 7, 
  SAMPLE_HOLD_256_CYCLES = 8, 
  SAMPLE_HOLD_384_CYCLES = 9, 
  SAMPLE_HOLD_512_CYCLES = 10, 
  SAMPLE_HOLD_768_CYCLES = 11, 
  SAMPLE_HOLD_1024_CYCLES = 12
};









#line 216
typedef union __nesc_unnamed4261 {
  uint32_t i;
  MSP430ADC12Settings_t s;
} MSP430ADC12Settings_ut;








enum __nesc_unnamed4262 {

  ADC_IDLE = 0, 
  SINGLE_CHANNEL = 1, 
  REPEAT_SINGLE_CHANNEL = 2, 
  SEQUENCE_OF_CHANNELS = 4, 
  REPEAT_SEQUENCE_OF_CHANNELS = 8, 
  TIMER_USED = 16, 
  RESERVED = 32, 
  VREF_WAIT = 64
};
#line 261
#line 255
typedef struct __nesc_unnamed4263 {

  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute((packed))  adc12memctl_t;
#line 274
#line 263
typedef struct __nesc_unnamed4264 {

  unsigned int refVolt2_5 : 1;
  unsigned int gotRefVolt : 1;
  unsigned int result_16bit : 1;
  unsigned int clockSourceSHT : 2;
  unsigned int clockSourceSAMPCON : 2;
  unsigned int clockDivSAMPCON : 2;
  unsigned int clockDivSHT : 3;
  unsigned int sampleHoldTime : 4;
  adc12memctl_t memctl;
} __attribute((packed))  adc12settings_t;
# 58 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/CC2420Const.h"
enum __nesc_unnamed4265 {
  CC2420_TIME_BIT = 4, 
  CC2420_TIME_BYTE = CC2420_TIME_BIT << 3, 
  CC2420_TIME_SYMBOL = 16
};
#line 76
enum __nesc_unnamed4266 {
  CC2420_MIN_CHANNEL = 11, 
  CC2420_MAX_CHANNEL = 26
};
#line 261
enum __nesc_unnamed4267 {
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
# 49 "/home/max/tinyos/tinyos-1.x/tos/types/AM.h"
enum __nesc_unnamed4268 {
  TOS_BCAST_ADDR = 0xffff, 
  TOS_UART_ADDR = 0x007e
};





enum __nesc_unnamed4269 {
  TOS_DEFAULT_AM_GROUP = 0x7d
};
#line 105
#line 84
typedef struct TOS_Msg {


  uint16_t addr;
  uint8_t type;
  uint8_t group;
  uint8_t length;
  int8_t data[29];
  uint16_t crc;







  uint16_t strength;
  uint8_t ack;
  uint16_t time;
  uint8_t sendSecurityMode;
  uint8_t receiveSecurityMode;
} TOS_Msg;
#line 129
#line 107
typedef struct TOS_Msg_TinySecCompat {


  uint16_t addr;
  uint8_t type;

  uint8_t length;
  uint8_t group;
  int8_t data[29];
  uint16_t crc;







  uint16_t strength;
  uint8_t ack;
  uint16_t time;
  uint8_t sendSecurityMode;
  uint8_t receiveSecurityMode;
} TOS_Msg_TinySecCompat;
#line 150
#line 131
typedef struct TinySec_Msg {

  uint16_t addr;
  uint8_t type;
  uint8_t length;

  uint8_t iv[4];

  uint8_t enc[29];

  uint8_t mac[4];


  uint8_t calc_mac[4];
  uint8_t ack_byte;
  bool cryptoDone;
  bool receiveDone;

  bool MACcomputed;
} __attribute((packed))  TinySec_Msg;



enum __nesc_unnamed4270 {
  MSG_DATA_SIZE = (size_t )& ((struct TOS_Msg *)0)->crc + sizeof(uint16_t ), 
  TINYSEC_MSG_DATA_SIZE = (size_t )& ((struct TinySec_Msg *)0)->mac + 4, 
  DATA_LENGTH = 29, 
  LENGTH_BYTE_NUMBER = (size_t )& ((struct TOS_Msg *)0)->length + 1, 
  TINYSEC_NODE_ID_SIZE = sizeof(uint16_t )
};

enum __nesc_unnamed4271 {
  TINYSEC_AUTH_ONLY = 1, 
  TINYSEC_ENCRYPT_AND_AUTH = 2, 
  TINYSEC_DISABLED = 3, 
  TINYSEC_RECEIVE_AUTHENTICATED = 4, 
  TINYSEC_RECEIVE_CRC = 5, 
  TINYSEC_RECEIVE_ANY = 6, 
  TINYSEC_ENABLED_BIT = 128, 
  TINYSEC_ENCRYPT_ENABLED_BIT = 64
} __attribute((packed)) ;


typedef TOS_Msg *TOS_MsgPtr;
# 29 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline uint8_t TOSH_READ_CC_FIFO_PIN(void);

static inline uint8_t TOSH_READ_CC_FIFOP_PIN(void);
static inline uint8_t TOSH_READ_RADIO_CCA_PIN(void);


static inline void TOSH_SET_CC_RSTN_PIN(void);
#line 35
static inline void TOSH_CLR_CC_RSTN_PIN(void);

static inline void TOSH_SET_CC_VREN_PIN(void);
#line 37
static inline void TOSH_CLR_CC_VREN_PIN(void);
static inline void TOSH_SET_RADIO_POWER_PIN(void);
#line 38
static inline void TOSH_CLR_RADIO_POWER_PIN(void);
#line 38
static inline void TOSH_MAKE_RADIO_POWER_OUTPUT(void);
#line 38
static inline void TOSH_SEL_RADIO_POWER_IOFUNC(void);



static inline void TOSH_SET_RADIO_CSN_PIN(void);
#line 42
static inline void TOSH_CLR_RADIO_CSN_PIN(void);
#line 42
static inline void TOSH_MAKE_RADIO_CSN_OUTPUT(void);
static inline void TOSH_SEL_SIMO0_MODFUNC(void);
#line 43
static inline void TOSH_SEL_SIMO0_IOFUNC(void);
static inline void TOSH_SEL_SOMI0_MODFUNC(void);
#line 44
static inline void TOSH_SEL_SOMI0_IOFUNC(void);
static inline void TOSH_SEL_UCLK0_MODFUNC(void);
#line 45
static inline void TOSH_SEL_UCLK0_IOFUNC(void);
static inline void TOSH_SEL_UTXD0_IOFUNC(void);
#line 46
static inline bool TOSH_IS_UTXD0_MODFUNC(void);
#line 46
static inline bool TOSH_IS_UTXD0_IOFUNC(void);
static inline void TOSH_SEL_URXD0_IOFUNC(void);
#line 47
static inline bool TOSH_IS_URXD0_MODFUNC(void);
#line 47
static inline bool TOSH_IS_URXD0_IOFUNC(void);





static inline void TOSH_SEL_CC_SFD_MODFUNC(void);
#line 53
static inline void TOSH_SEL_CC_SFD_IOFUNC(void);


static inline void TOSH_CLR_DBG1_PIN(void);
#line 56
static inline void TOSH_MAKE_DBG1_OUTPUT(void);
#line 56
static inline void TOSH_SEL_DBG1_IOFUNC(void);
static inline void TOSH_CLR_DBG2_PIN(void);
#line 57
static inline void TOSH_MAKE_DBG2_OUTPUT(void);
#line 57
static inline void TOSH_SEL_DBG2_IOFUNC(void);



static inline void TOSH_SET_PIN_DIRECTIONS(void );
# 54 "/home/max/tinyos/tinyos-1.x/tos/types/dbg_modes.h"
typedef long long TOS_dbg_mode;



enum __nesc_unnamed4272 {
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
typedef struct __nesc_unnamed4273 {
  void (*tp)(void);
} TOSH_sched_entry_T;

enum __nesc_unnamed4274 {




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
enum __nesc_unnamed4275 {

  IDENT_MAX_PROGRAM_NAME_LENGTH = 16
};






#line 33
typedef struct __nesc_unnamed4276 {

  uint32_t unix_time;
  uint32_t user_hash;
  char program_name[IDENT_MAX_PROGRAM_NAME_LENGTH];
} Ident_t;
# 4 "../../../zigzag/IEEE802_15_4/SysCommon/public/ByteOrder.h"
static __inline bool IsTargetHostLSB(void);





static __inline uint16_t ToLSB16(uint16_t a);
# 6 "../../../zigzag/IEEE802_15_4/SysCommon/public/SysCommon.h"
typedef uint16_t TUniData;
# 4 "../../../zigzag/IEEE802_15_4/SysTimer/public/SysTime.h"
typedef uint32_t TSysTime;

typedef uint32_t TMilliSec;
# 4 "../../../zigzag/IEEE802_15_4/Phy/public/PhyConst.h"
enum __nesc_unnamed4277 {

  PHY_AMAX_PHY_PACKET_SIZE = 127, 


  PHY_ATURNAROUND_TIME = 12
};
# 6 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPrivate.h"
enum __nesc_unnamed4278 {
#line 6
  PHY_SIZE_OF_FRAME_LENGTH = 1
};
#line 7
enum __nesc_unnamed4279 {
#line 7
  MAX_PPDU_LENGTH = PHY_SIZE_OF_FRAME_LENGTH + PHY_AMAX_PHY_PACKET_SIZE
};
enum __nesc_unnamed4280 {
#line 9
  CRC16_LENGTH = 2
};
typedef uint8_t _TPhyFrameLength;

typedef uint32_t _TPhyTimeStamp;





#line 15
typedef struct __nesc_unnamed4281 {
  _TPhyTimeStamp timeStamp;
  uint8_t current;
  uint8_t data[MAX_PPDU_LENGTH];
} _TPhyFrame;

typedef uint8_t _TPhyLinkQuality;

typedef uint8_t _TPhyChannel;

typedef uint32_t _TPhyChannelsSupported;




#line 27
typedef struct __nesc_unnamed4282 {
  unsigned tolerance : 2;
  signed txPower : 6;
} _TPhyTransmitPower;







#line 32
typedef enum __nesc_unnamed4283 {
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
typedef enum __nesc_unnamed4284 {
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
enum __nesc_unnamed4285 {

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
typedef enum __nesc_unnamed4286 {

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
typedef enum __nesc_unnamed4287 {
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
typedef enum __nesc_unnamed4288 {
  MAC_ACK_WAIT_DURATION_54 = 54 * 4, 
  MAC_ACK_WAIT_DURATION_120 = 120
} TMacAckWaitDuration;





#line 161
typedef enum __nesc_unnamed4289 {
  MAC_UNSECURED_MODE = 0x00, 
  MAC_ACL_MODE = 0x01, 
  MAC_SECURED_MODE = 0x02
} TMacSecurityMode;
# 4 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressPrv.h"
enum __nesc_unnamed4290 {
#line 4
  _MAC_PANID_LENGTH = 2
};
enum __nesc_unnamed4291 {
#line 6
  _MAC_SHORT_ADDRESS_LENGTH = 2
};
enum __nesc_unnamed4292 {
#line 8
  _MAC_EXTENDED_ADDRESS_LENGTH = 8
};
typedef uint16_t _TMacPANId;





#line 12
typedef enum __nesc_unnamed4293 {
  _MAC_ADDRESS_NOT_PRESENT = 0x0, 
  _MAC_ADDRESS_SHORT = 0x2, 
  _MAC_ADDRESS_EXTENDED = 0x3
} _TMacAddressMode;

typedef uint16_t _TMacShortAddress;

typedef uint64_t _TMacExtendedAddress;







#line 22
typedef struct __nesc_unnamed4294 {
  _TMacAddressMode mode;
  union __nesc_unnamed4295 {
    _TMacShortAddress shortAddress;
    _TMacExtendedAddress extendedAddress;
  } address;
} _TMacAddress;
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/MacAddress.h"
enum __nesc_unnamed4296 {
#line 6
  MAC_SHORT_ADDRESS_LENGTH = _MAC_SHORT_ADDRESS_LENGTH
};
enum __nesc_unnamed4297 {
#line 8
  MAC_EXTENDED_ADDRESS_LENGTH = _MAC_EXTENDED_ADDRESS_LENGTH
};
enum __nesc_unnamed4298 {
#line 10
  MAC_PANID_LENGTH = _MAC_PANID_LENGTH
};




#line 12
typedef enum __nesc_unnamed4299 {
  MAC_ADDRESS_NOT_PRESENT = _MAC_ADDRESS_NOT_PRESENT, 
  MAC_ADDRESS_SHORT = _MAC_ADDRESS_SHORT, 
  MAC_ADDRESS_EXTENDED = _MAC_ADDRESS_EXTENDED
} TMacAddressMode;

typedef _TMacPANId TMacPANId;

typedef _TMacShortAddress TMacShortAddress;

typedef _TMacExtendedAddress TMacExtendedAddress;

typedef _TMacAddress TMacAddress;
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormat.h"
enum __nesc_unnamed4300 {
#line 7
  MAC_FRAME_CONTROL_LENGTH = 2
};






#line 8
typedef struct __nesc_unnamed4301 {
  unsigned frameType : 3;
  unsigned securityEnabled : 1;
  unsigned framePending : 1;
  unsigned ackRequest : 1;
  unsigned intraPAN : 1;
  unsigned  : 1;
} TMacLSBFrameControl;






#line 17
typedef struct __nesc_unnamed4302 {
  unsigned  : 2;
  unsigned dstAddressMode : 2;
  unsigned  : 2;
  unsigned srcAddressMode : 2;
} TMacMSBFrameControl;










#line 24
typedef struct __nesc_unnamed4303 {
  union __nesc_unnamed4304 {
    TMacLSBFrameControl value;
    uint8_t raw;
  } lsb;
  union __nesc_unnamed4305 {
    TMacMSBFrameControl value;
    uint8_t raw;
  } msb;
} TMacFrameControl;
typedef TMacFrameControl TMacFrameControlField;

enum __nesc_unnamed4306 {
#line 36
  MAC_SEQUENCE_NUMBER_LENGTH = 1
};
#line 37
typedef uint8_t _TMacSequenceNumber;



#line 38
typedef union __nesc_unnamed4307 {
  uint8_t raw[MAC_SEQUENCE_NUMBER_LENGTH];
  _TMacSequenceNumber value;
} TMacSequenceNumberField;




#line 43
typedef union __nesc_unnamed4308 {
  uint8_t raw[MAC_PANID_LENGTH];
  TMacPANId value;
} TMacPANIdField;




#line 48
typedef union __nesc_unnamed4309 {
  uint8_t raw[MAC_SHORT_ADDRESS_LENGTH];
  TMacShortAddress value;
} TMacShortAddressField;




#line 53
typedef union __nesc_unnamed4310 {
  uint8_t raw[MAC_EXTENDED_ADDRESS_LENGTH];
  TMacExtendedAddress value;
} TMacExtendedAddressField;




#line 58
typedef union __nesc_unnamed4311 {
  TMacShortAddressField _short;
  TMacExtendedAddressField _extended;
} TMacAddressField;

typedef uint8_t _TMacPayloadLength;
enum __nesc_unnamed4312 {
#line 64
  MAC_PAYLOAD_LENGTH = MAC_AMAX_MAC_FRAME_SIZE
};


#line 65
typedef struct __nesc_unnamed4313 {
  _TMacPayloadLength length;
  uint8_t raw[MAC_PAYLOAD_LENGTH];
} TMacPayloadField;

enum __nesc_unnamed4314 {
#line 70
  MAC_FRAME_CHECK_SEQUENCE_LENGTH = 2
};
#line 71
typedef uint16_t _TMacFrameCheckSequence;



#line 72
typedef union __nesc_unnamed4315 {
  uint8_t raw[MAC_FRAME_CHECK_SEQUENCE_LENGTH];
  _TMacFrameCheckSequence value;
} TMacFrameCheckSequenceField;

typedef uint8_t _TMacRawFrameLength;

typedef uint32_t _TMacTimeStamp;

typedef uint8_t _TMacLinkQuality;
#line 93
#line 83
typedef struct __nesc_unnamed4316 {
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
typedef enum __nesc_unnamed4317 {
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
typedef enum __nesc_unnamed4318 {
  MAC_SEND_MODE_DIRECT, 
  MAC_SEND_MODE_CSMA
} TMacSendMode;
# 4 "../../../zigzag/IEEE802_15_4/MacCommon/public/MacSecurity.h"
typedef uint8_t TACLEntry;
# 4 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/MacSuperframe.h"
typedef uint8_t TMacBeaconOrder;

typedef uint8_t TMacSuperframeOrder;







#line 8
typedef enum __nesc_unnamed4319 {
  MAC_SLOT_TYPE_UNKNOWN = 0, 
  MAC_SLOT_TYPE_FREE = 1, 
  MAC_SLOT_TYPE_BEACON = 2, 
  MAC_SLOT_TYPE_CAP = 3, 
  MAC_SLOT_TYPE_CFP = 4
} TMacSlotType;
enum __nesc_unnamed4320 {
#line 15
  SLOT_TYPE_NUMBER = 5
};
#line 31
#line 26
typedef enum __nesc_unnamed4321 {
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
typedef enum __nesc_unnamed4322 {
  MAC_BATT_LIFE_EXT_PERIOD_6 = 6, 
  MAC_BATT_LIFE_EXT_PERIOD_8 = 8
} TMacBattLifeExtPeriods;










#line 9
typedef enum __nesc_unnamed4323 {
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
typedef enum __nesc_unnamed4324 {
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
enum __nesc_unnamed4325 {
  MAC_DEVICE_TYPE_RFD = 0, 
  MAC_DEVICE_TYPE_FFD = 1
};









#line 9
typedef struct __nesc_unnamed4326 {
  unsigned int alternatePANCoordinator : 1;
  unsigned int deviceType : 1;
  unsigned int powerSource : 1;
  unsigned int receiverOnWhenIdle : 1;
  unsigned int  : 2;
  unsigned int securityCapability : 1;
  unsigned int allocateAddress : 1;
} TCapabilityInformation;




#line 19
typedef enum __nesc_unnamed4327 {
  MAC_DISASSOCIATE_BY_COORD = 0x01, 
  MAC_DISASSOCIATE_BY_DEVICE = 0x02
} TDisassociateReason;
#line 54
#line 24
typedef enum __nesc_unnamed4328 {
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
typedef struct __nesc_unnamed4329 {

  unsigned shortAddrNum : 3;
  unsigned  : 1;
  unsigned extAddrNum : 3;
} 
TMacPendAddrSpec;


typedef uint8_t *TMacAddrList;










#line 19
typedef struct __nesc_unnamed4330 {

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
typedef struct __nesc_unnamed4331 {

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
typedef enum __nesc_unnamed4332 {

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
typedef enum __nesc_unnamed4333 {
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
typedef struct __nesc_unnamed4334 {


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
typedef enum __nesc_unnamed4335 {

  ZIGBEE_COORDINATOR = 0, 
  ZIGBEE_ROUTER = 1, 
  ZIGBEE_END_DEVICE = 2
} NwkDeviceType;









#line 74
typedef enum __nesc_unnamed4336 {

  NWK_PARENT = 0, 
  NWK_CHILD = 1, 
  NWK_SIBLING = 2, 
  NWK_OTHER = 3
} 
NwkRelationship;
#line 106
#line 87
typedef struct __nesc_unnamed4337 {

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
typedef enum __nesc_unnamed4338 {

  ROUTE_ACTIVE = 0, 
  ROUTE_DISCOVERY_UNDERWAY = 1, 
  ROUTE_DISCOVERY_FAILED = 2, 
  ROUTE_INACTIVE = 3
} 
NwkRouteStatus;









#line 120
typedef struct __nesc_unnamed4339 {

  NwkAddr destinationAddr;
  NwkAddr nextHopAddr;
  NwkRouteStatus status;
} NwkRoute;


typedef TCapabilityInformation NwkCapabilityInfo;









#line 131
typedef enum __nesc_unnamed4340 {

  NWK_OFFLINE, 
  NWK_JOINED_AS_ED, 
  NWK_JOINED_AS_ROUTER, 
  NWK_ROUTING
} 
NwkOnlineStatus;


enum __nesc_unnamed4341 {

  NWK_DATA_FRAME = 0, 
  NWK_COMMAND_FRAME = 1
};


enum __nesc_unnamed4342 {

  SUPPRESS_DISCOVERY = 0, 
  ENABLE_DISCOVERY = 1, 
  FORCE_DISCOVERY = 2
};

enum __nesc_unnamed4343 {

  NWK_BROADCAST = 0xffff
};
#line 178
#line 160
typedef struct __nesc_unnamed4344 {

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
typedef struct __nesc_unnamed4345 {

  uint8_t routeRequestID;
  NwkAddr sourceAddr;
  NwkAddr senderAddr;
  uint8_t forwardCost;
  uint8_t residualCost;
  uint16_t expirationTime;
} NwkRouteDiscovery;
# 16 "../../../zigzag/ZigBee/implementation/constants.h"
enum __nesc_unnamed4346 {
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
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.h"
enum __nesc_unnamed4347 {
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
typedef struct __nesc_unnamed4348 {

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
typedef struct __nesc_unnamed4349 {

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
typedef struct __nesc_unnamed4350 {

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
#line 89
int16_t __net_sleep(uint32_t duration);
# 14 "../../../include/port.h"
enum __nesc_unnamed4351 {
  OP_WRITE = 0, 
  OP_READ, 
  OP_SET_ATTR, 
  OP_GET_ATTR, 
  OP_GET_IFLAG, 
  OP_RESET_IFLAG
};
# 18 "../../../include/syszig.h"
typedef uint16_t faddr_t;


extern uint16_t __module_load;
# 17 "../../../zigzag/ZigBee/implementation/NwkBeacon.h"
#line 6
typedef struct __nesc_unnamed4352 {

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
typedef struct __nesc_unnamed4353 {

  unsigned  : 4;
  unsigned secure : 1;
  unsigned indirect : 1;
  unsigned gts : 1;
  unsigned acknowledged : 1;
} TxOptions;
# 4 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/SysTimer.h"
enum __nesc_unnamed4354 {
  MAX_SYS_JIFFY_HI = 0x7fffUL, 
  MAX_SYS_JIFFY_LO = 0xffffUL, 
  MAX_SYS_JIFFY = (uint32_t )(MAX_SYS_JIFFY_HI << 16) | MAX_SYS_JIFFY_LO
};
# 4 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420.h"
const uint16_t CC2420_EXTADDR_ADDRESS = 0x160;
const uint16_t CC2420_PANID_ADDRESS = 0x168;
const uint16_t CC2420_SHORTADDR_ADDRESS = 0x16a;
#line 21
const signed RSSI_OFFSET = -45;
const signed RSSI_MIN = -85;
#line 36
#line 24
typedef union __nesc_unnamed4355 {
  struct __nesc_unnamed4356 {
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
typedef union __nesc_unnamed4357 {
  struct __nesc_unnamed4358 {
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
typedef union __nesc_unnamed4359 {
  struct __nesc_unnamed4360 {
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
typedef union __nesc_unnamed4361 {
  struct __nesc_unnamed4362 {
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
typedef union __nesc_unnamed4363 {
  struct __nesc_unnamed4364 {
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
typedef union __nesc_unnamed4365 {
  struct __nesc_unnamed4366 {
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
typedef union __nesc_unnamed4367 {
  struct __nesc_unnamed4368 {
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
typedef union __nesc_unnamed4369 {
  struct __nesc_unnamed4370 {
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
typedef union __nesc_unnamed4371 {
  struct __nesc_unnamed4372 {
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
typedef union __nesc_unnamed4373 {
  struct __nesc_unnamed4374 {
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
typedef union __nesc_unnamed4375 {
  struct __nesc_unnamed4376 {
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
typedef union __nesc_unnamed4377 {
  struct __nesc_unnamed4378 {
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
typedef union __nesc_unnamed4379 {
  struct __nesc_unnamed4380 {
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
typedef union __nesc_unnamed4381 {
  struct __nesc_unnamed4382 {
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
typedef union __nesc_unnamed4383 {
  struct __nesc_unnamed4384 {
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
typedef union __nesc_unnamed4385 {
  struct __nesc_unnamed4386 {
    unsigned int fsm_cur_state : 6;
    unsigned int  : 10;
  } value;
  uint16_t raw;
} TCC2420_FSMSTATE;
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430usart.h"
#line 31
typedef enum __nesc_unnamed4387 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacSlottedCAP.h"
#line 4
typedef enum __nesc_unnamed4388 {
  FFMAC_PARENT_CAP = 0, 
  FFMAC_OWN_CAP = 2
} TMacCAPType;
# 4 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacOwnCAP.h"
enum __nesc_unnamed4389 {

  MAC_OWN_SF = 0U
};
# 4 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacParentCAP.h"
enum __nesc_unnamed4390 {

  MAC_PARENT_SF = 1U
};
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
static   void MSP430TimerM$CaptureA1$default$captured(uint16_t arg_0x40746d38);
#line 32
static   uint16_t MSP430TimerM$CaptureB3$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB3$default$captured(uint16_t arg_0x40746d38);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA1$default$fired(void);
#line 30
static   void MSP430TimerM$CompareB3$setEvent(uint16_t arg_0x407331d8);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB6$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB6$default$captured(uint16_t arg_0x40746d38);
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
static   void MSP430TimerM$ControlB1$setControlAsCapture(bool arg_0x40737358);
#line 30
static   MSP430CompareControl_t MSP430TimerM$ControlB1$getControl(void);







static   void MSP430TimerM$ControlB1$enableEvents(void);
static   void MSP430TimerM$ControlB1$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB1$clearPendingInterrupt(void);

static   void MSP430TimerM$ControlB1$setControl(MSP430CompareControl_t arg_0x40739b58);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureA2$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureA2$default$captured(uint16_t arg_0x40746d38);
#line 32
static   uint16_t MSP430TimerM$CaptureB4$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB4$default$captured(uint16_t arg_0x40746d38);
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
static   void MSP430TimerM$TimerA$setClockSource(uint16_t arg_0x4072b970);
#line 38
static   void MSP430TimerM$TimerA$disableEvents(void);
#line 32
static   void MSP430TimerM$TimerA$clearOverflow(void);


static   void MSP430TimerM$TimerA$setMode(int arg_0x4072db50);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB4$setEvent(uint16_t arg_0x407331d8);
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
static   void MSP430TimerM$CaptureA0$default$captured(uint16_t arg_0x40746d38);
#line 32
static   uint16_t MSP430TimerM$CaptureB2$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB2$default$captured(uint16_t arg_0x40746d38);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA0$default$fired(void);
#line 30
static   void MSP430TimerM$CompareB2$setEvent(uint16_t arg_0x407331d8);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB5$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB5$default$captured(uint16_t arg_0x40746d38);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB3$getControl(void);







static   void MSP430TimerM$ControlB3$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB3$setControlAsCompare(void);



static   void MSP430TimerM$ControlB3$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB3$clearPendingInterrupt(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerB$setClockSource(uint16_t arg_0x4072b970);
#line 30
static   uint16_t MSP430TimerM$TimerB$read(void);

static   void MSP430TimerM$TimerB$clearOverflow(void);
#line 31
static   bool MSP430TimerM$TimerB$isOverflowPending(void);



static   void MSP430TimerM$TimerB$setMode(int arg_0x4072db50);




static   void MSP430TimerM$TimerB$setInputDivider(uint16_t arg_0x4072be18);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB5$setEvent(uint16_t arg_0x407331d8);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   uint16_t MSP430TimerM$CaptureB0$getEvent(void);
#line 74
static   void MSP430TimerM$CaptureB0$default$captured(uint16_t arg_0x40746d38);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB6$setEvent(uint16_t arg_0x407331d8);

static   void MSP430TimerM$CompareB0$setEventFromNow(uint16_t arg_0x40733b20);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   MSP430CompareControl_t MSP430TimerM$ControlB6$getControl(void);







static   void MSP430TimerM$ControlB6$enableEvents(void);
#line 35
static   void MSP430TimerM$ControlB6$setControlAsCompare(void);



static   void MSP430TimerM$ControlB6$disableEvents(void);
#line 32
static   void MSP430TimerM$ControlB6$clearPendingInterrupt(void);
# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void ZigRouterM$NLME_Reset$confirm(NwkStatus arg_0x40867458);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t ZigRouterM$TimerMilli$fired(void);
# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
static  void ZigRouterM$NLME_Leave$indication(IEEEAddr arg_0x4088a300);
# 19 "../../../zigzag/ZigBee/interface/NLME_NetworkDiscovery.nc"
static  void ZigRouterM$NLME_NetworkDiscovery$confirm(
uint8_t arg_0x4086d990, 
NwkNetworkDescriptor *arg_0x4086db60, 
NwkStatus arg_0x4086dd08);
# 27 "../../../zigzag/ZigBee/interface/NLME_JoinChild.nc"
static  void ZigRouterM$NLME_JoinChild$confirm(
PanID_t arg_0x40870010, 
NwkStatus arg_0x408701b8);
# 17 "../../../zigzag/ZigBee/interface/NLME_Sync.nc"
static  void ZigRouterM$NLME_Sync$indication(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ZigRouterM$StdControl$init(void);






static  result_t ZigRouterM$StdControl$start(void);
# 22 "../../../zigzag/ZigBee/interface/NLME_StartRouter.nc"
static  void ZigRouterM$NLME_StartRouter$confirm(NwkStatus arg_0x4086fd88);
# 38 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t NLME_JoinParentM$IMacASSOCIATE$Indication(TMacExtendedAddress arg_0x408cb068, 
TCapabilityInformation arg_0x408cb218, 
bool arg_0x408cb3b0, 
TACLEntry arg_0x408cb550);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCOMM_STATUS.nc"
static  void NLME_JoinParentM$IMacCOMM_STATUS$Indication(TMacPANId arg_0x408c8400, 
TMacAddress arg_0x408c85a0, TMacAddress arg_0x408c8730, 
TMacStatus arg_0x408c88d0);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NLME_JoinParentM$Reset$reset(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t NLME_JoinParentM$ITimerSymbol$Fired(TUniData arg_0x408cc858);
# 63 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t NLME_JoinChildM$IMacASSOCIATE$Confirm(TMacShortAddress arg_0x408ca888, 
TMacStatus arg_0x408caa20);
# 15 "../../../zigzag/ZigBee/interface/NLME_JoinChild.nc"
static  void NLME_JoinChildM$NLME_JoinChild$request(
PanID_t arg_0x4085bd90, 
bool arg_0x4085bf28, 
bool arg_0x408730d8, 
IEEEAddr *arg_0x408732a0, 
uint32_t arg_0x40873440, 
uint8_t arg_0x408735d8, 
uint8_t arg_0x40873770, 
bool arg_0x40873908, 
bool arg_0x40873ab0);
# 11 "../../../zigzag/IEEE802_15_4/MacScan/public/IMacSCAN.nc"
static  result_t NLME_NetworkDiscoveryM$IMacSCAN$Confirm(TMacStatus arg_0x4091f968, 
TMacScanType arg_0x4091fb08, 
uint32_t arg_0x4091fcb0, 
uint8_t arg_0x4091fe48, 
uint8_t *arg_0x4091d030, 
TMacPanDescriptor *arg_0x4091d200);
# 15 "../../../zigzag/ZigBee/interface/NLME_NetworkDiscovery.nc"
static  void NLME_NetworkDiscoveryM$NLME_NetworkDiscovery$request(
uint32_t arg_0x4086d358, 
uint8_t arg_0x4086d500);
# 66 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  uint8_t NIBM$NIB$getNwkMaxRouters(void);
#line 117
static  NwkCapabilityInfo NIBM$NIB$getNwkCapabilityInformation(void);
#line 191
static  void NIBM$NIB$setOnline(NwkOnlineStatus arg_0x40875ae0);
#line 20
static  uint8_t NIBM$NIB$getNwkSequenceNumber(void);
#line 55
static  void NIBM$NIB$setNwkMaxDepth(uint8_t arg_0x4087d550);
#line 204
static  void NIBM$NIB$setBeaconOffset(TSysTime arg_0x4088e530);
#line 116
static  void NIBM$NIB$setNwkCapabilityInformation(NwkCapabilityInfo arg_0x40879398);
#line 192
static  NwkOnlineStatus NIBM$NIB$getOnlineStatus(void);



static  uint8_t NIBM$NIB$getDepth(void);
#line 65
static  void NIBM$NIB$setNwkMaxRouters(uint8_t arg_0x4087dcc8);
#line 197
static  void NIBM$NIB$setDepth(uint8_t arg_0x4088f918);
#line 47
static  void NIBM$NIB$setNwkMaxChildren(uint8_t arg_0x4087edc0);
#line 201
static  void NIBM$NIB$setChannel(uint8_t arg_0x4088e0b0);



static  TSysTime NIBM$NIB$getBeaconOffset(void);
#line 19
static  void NIBM$NIB$setNwkSequenceNumber(uint8_t arg_0x4087f6c8);
#line 200
static  uint8_t NIBM$NIB$getChannel(void);
#line 48
static  uint8_t NIBM$NIB$getNwkMaxChildren(void);







static  uint8_t NIBM$NIB$getNwkMaxDepth(void);
#line 193
static  bool NIBM$NIB$isJoined(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NIBM$Reset$reset(void);
# 27 "../../../zigzag/ZigBee/interface/NLME_JoinChild.nc"
static  void NwkAddressingM$NLME_JoinChild$confirm(
PanID_t arg_0x40870010, 
NwkStatus arg_0x408701b8);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NwkAddressingM$Reset$reset(void);
# 15 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  result_t NwkAddressingM$NwkAddressing$allocEdAddress(NwkAddr *arg_0x408d66e0);



static  bool NwkAddressingM$NwkAddressing$hasVacantAddrs(void);
static  bool NwkAddressingM$NwkAddressing$hasVacantEdAddrs(void);




static  void NwkAddressingM$NwkAddressing$getDirection(NwkAddr arg_0x408d4928, NwkRelationship *arg_0x408d4ad8, NwkAddr *arg_0x408d4c78);
#line 17
static  result_t NwkAddressingM$NwkAddressing$allocRouterAddress(NwkAddr *arg_0x408d6b90);



static  bool NwkAddressingM$NwkAddressing$hasVacantRouterAddrs(void);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC_LOSS.nc"
static  void NLME_SyncM$IMacSYNC_LOSS$Indication(
TMacStatus arg_0x4097b5b0);
# 21 "../../../zigzag/ZigBee/implementation/NwkBeaconParentM.nc"
static  void NwkBeaconParentM$updateBeacon(void);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBEACON_NOTIFY.nc"
static  void NwkBeaconChildM$IMacBEACON_NOTIFY$Indication(
TMacBSN arg_0x4099e708, 
TMacPanDescriptor arg_0x4099e8b0, 
TMacPendAddrSpec arg_0x4099ea58, 
uint8_t *arg_0x4099ec10, 
uint8_t arg_0x4099eda8, 
uint8_t *arg_0x4099d010);
# 16 "../../../zigzag/ZigBee/interface/NLME_PermitJoining.nc"
static  NwkStatus NLME_PermitJoiningM$NLME_PermitJoining$request(
uint8_t arg_0x408728c0);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t NLME_PermitJoiningM$ITimerSymbol$Fired(TUniData arg_0x408cc858);
# 15 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void NLME_ResetM$NLME_Reset$request(void);
static  void NLME_ResetM$MacReset$confirm(NwkStatus arg_0x40867458);
#line 15
static  void NwkResetMacM$NLME_Reset$request(void);
# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t NwkResetMacM$IMacRESET1$Confirm(TMacStatus arg_0x409c6ac8);
#line 7
static  result_t NwkResetMacM$IMacRESET2$Confirm(TMacStatus arg_0x409c6ac8);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NeighborTableM$Reset$reset(void);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NeighborTableM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790);










static  result_t NeighborTableM$NeighborTable$remove(NwkNeighbor *arg_0x40869360);

static  result_t NeighborTableM$NeighborTable$getFreePtr(NwkNeighbor **arg_0x40869838);
#line 15
static  result_t NeighborTableM$NeighborTable$update(NwkNeighbor arg_0x4086b2b8);
#line 29
static  result_t NeighborTableM$NeighborTable$getByAddr(TMacAddress arg_0x4086bc20, NwkNeighbor **arg_0x4086bdf0);
# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t RouterM$LowerIf$Confirm(uint8_t arg_0x409e8348, TMacStatus arg_0x409e84d8);
#line 16
static  result_t RouterM$LowerIf$Indication(TMacPANId arg_0x409e9178, 
TMacAddress arg_0x409e9310, 
TMacPANId arg_0x409e94b0, 
TMacAddress arg_0x409e9648, 
uint8_t arg_0x409e97e0, 
uint8_t *arg_0x409e9998, 
TMacLinkQuality arg_0x409e9b38, 
bool arg_0x409e9cd0, 
uint8_t arg_0x409e9e68);
# 15 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static  uint8_t RouterM$newMsduHandle(void);
# 23 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
static  result_t RouterM$NLDE_Data$request(
NwkAddr arg_0x408a4490, 
uint8_t arg_0x408a4628, 
uint8_t *arg_0x408a47d8, 
uint8_t arg_0x408a4970, 
uint8_t arg_0x408a4b00, 
uint8_t arg_0x408a4c98, 
bool arg_0x408a4e30);
# 27 "../../../zigzag/ZigBee/interface/NLME_JoinChild.nc"
static  void RouterM$NLME_JoinChild$confirm(
PanID_t arg_0x40870010, 
NwkStatus arg_0x408701b8);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void RouterM$Reset$reset(void);
# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t RouterM$UpperIf$Confirm(uint8_t arg_0x409e8348, TMacStatus arg_0x409e84d8);
#line 16
static  result_t RouterM$UpperIf$Indication(TMacPANId arg_0x409e9178, 
TMacAddress arg_0x409e9310, 
TMacPANId arg_0x409e94b0, 
TMacAddress arg_0x409e9648, 
uint8_t arg_0x409e97e0, 
uint8_t *arg_0x409e9998, 
TMacLinkQuality arg_0x409e9b38, 
bool arg_0x409e9cd0, 
uint8_t arg_0x409e9e68);
# 7 "../../../zigzag/ZigBee/implementation/NwkFrame.nc"
static  uint8_t NwkFrameM$NwkFrame$mkDataFrameHeader(uint8_t *arg_0x409e2938, 
NwkAddr arg_0x409e2ac8, 
uint8_t arg_0x409e2c58, 
uint8_t arg_0x409e2df0, 
bool arg_0x409e0010);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t TimerSymbol2M$IStdControl$init(void);






static  result_t TimerSymbol2M$IStdControl$start(void);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t TimerSymbol2M$TimerMilli$default$fired(
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a4c810);
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t TimerSymbol2M$TimerMilli$setOneShot(
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a4c810, 
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
int32_t arg_0x40887668);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbol2M$AlarmCompare$fired(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime TimerSymbol2M$ILocalTime$Read(void);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void TimerSymbol2M$AlarmTimer$overflow(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t TimerSymbol2M$ITimerSymbol$Stop(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a4dc70);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t TimerSymbol2M$ITimerSymbol$SetOneShot(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a4dc70, 
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
#line 6
static  result_t TimerSymbol2M$ITimerSymbol$SetPeriodic(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a4dc70, 
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TSysTime arg_0x408cdca8, TUniData arg_0x408cde30);









static  result_t TimerSymbol2M$ITimerSymbol$default$Fired(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a4dc70, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TUniData arg_0x408cc858);
#line 11
static  bool TimerSymbol2M$ITimerSymbol$IsSet(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a4dc70);

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
uint8_t arg_0x40a93770);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$default$Fired(
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
uint8_t arg_0x40a93770, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
TUniData arg_0x40a37b28);
#line 7
static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$SetOneShotAt(
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
uint8_t arg_0x40a93770, 
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
TSysTime arg_0x40a38550, TUniData arg_0x40a386d8);
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
static  result_t LocalTime64M$ITimerSymbol$Fired(TUniData arg_0x408cc858);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  uint64_t LocalTime64M$ILocalTime64$getLocalTimeAt(TSysTime arg_0x40a42e70);
static  void LocalTime64M$ILocalTime64$setLocalTimeAt(TSysTime arg_0x40a41330, uint64_t arg_0x40a414b8);

static  uint64_t LocalTime64M$ILocalTime64$getLocalTime(void);
static  void LocalTime64M$ILocalTime64$setLocalTime(uint64_t arg_0x40a41c40);
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   uint32_t TimeCastM$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a3c4c8);
#line 10
static   TMilliSec TimeCastM$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a3d378);



static   uint32_t TimeCastM$ITimeCast$MillisToJiffies(TMilliSec arg_0x40a3c010);
#line 7
static   TSysTime TimeCastM$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a3eb98);
#line 6
static   TSysTime TimeCastM$ITimeCast$MillisToSymbols(TMilliSec arg_0x40a3e6e0);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t PhyCC2420M$IStdControl$init(void);






static  result_t PhyCC2420M$IStdControl$start(void);
# 49 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
static   result_t PhyCC2420M$IChipconRAM$writeDone(uint16_t arg_0x40b98e80, uint8_t arg_0x40b97030, uint8_t *arg_0x40b971d0);
# 11 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
static  result_t PhyCC2420M$IPhyAttr$SetAutoAck(bool arg_0x40b40ec0);
#line 7
static  result_t PhyCC2420M$IPhyAttr$SetextendedAddress(uint64_t arg_0x40b405a0);
#line 5
static  result_t PhyCC2420M$IPhyAttr$SetpanId(uint16_t arg_0x40b40100);
#line 3
static  result_t PhyCC2420M$IPhyAttr$SetshortAddress(uint16_t arg_0x40b24c58);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static   result_t PhyCC2420M$IPhyTxDATA$Request(
# 9 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b57010, 
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
const TPhyPoolHandle arg_0x40b32f18, 
const TUniData arg_0x40b340f0);

static  result_t PhyCC2420M$IPhyTxDATA$default$Confirm(
# 9 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b57010, 
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
TPhyPoolHandle arg_0x40b34580, 
TPhyStatus arg_0x40b34720, 
TUniData arg_0x40b348b8);
# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t PhyCC2420M$IChipconFIFOSignal$fired(void);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyED.nc"
static   result_t PhyCC2420M$IPhyED$Request(
# 11 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b57998, 
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyED.nc"
const TUniData arg_0x40b2dd08);

static  result_t PhyCC2420M$IPhyED$default$Confirm(
# 11 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b57998, 
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyED.nc"
TPhyStatus arg_0x40b4b1a8, 
TPhyEnergyLevel arg_0x40b4b348, TUniData arg_0x40b4b4d0);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
static  TPhyStatus PhyCC2420M$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b2a360);
# 18 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static   void PhyCC2420M$FreePoolItem(TPhyPoolHandle arg_0x40b7f408);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyGET.nc"
static  TPhyChannel PhyCC2420M$IPhyGET$GetphyCurrentChannel(TPhyStatus *const arg_0x40b33d28);
# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t PhyCC2420M$IChipconFIFOP$fired(void);
# 53 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
static   result_t PhyCC2420M$IChipconSFD$captured(uint16_t arg_0x40b9d360);
# 50 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
static   result_t PhyCC2420M$IChipconFIFO$TXFIFODone(uint8_t arg_0x40b77118, uint8_t *arg_0x40b772b8);
#line 39
static   result_t PhyCC2420M$IChipconFIFO$RXFIFODone(uint8_t arg_0x40b789a8, uint8_t *arg_0x40b78b48);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t PhyCC2420M$IPhySET_TRX_STATE$Request(
# 13 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b81340, 
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8);

static  result_t PhyCC2420M$IPhySET_TRX_STATE$default$Confirm(
# 13 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b81340, 
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
static   result_t PhyCC2420M$IPhyTxFIFO$Write(
# 17 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b80bf8, 
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
TPhyPoolHandle arg_0x40b31130, TUniData arg_0x40b312b8);



static   void PhyCC2420M$IPhyTxFIFO$default$WriteDone(
# 17 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b80bf8, 
# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
TPhyPoolHandle arg_0x40b31c08, 
result_t arg_0x40b31da0, TUniData arg_0x40b31f28);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyCCA.nc"
static   TPhyStatus PhyCC2420M$IPhyCCA$Request(void);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   uint8_t *PhyFrameM$IPhyFrame$GetPPDU(const TPhyFrame *const arg_0x40b2bd60);
#line 17
static   result_t PhyFrameM$IPhyFrame$ResetPosition(TPhyFrame *const arg_0x40b45068);







static   result_t PhyFrameM$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b44528, TPhyTimeStamp arg_0x40b446b0);
#line 9
static   uint8_t PhyFrameM$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b48808);









static   result_t PhyFrameM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b45578);
#line 13
static   result_t PhyFrameM$IPhyFrame$AppendOctet(TPhyFrame *const arg_0x40b47270, const uint8_t arg_0x40b47410);

static   result_t PhyFrameM$IPhyFrame$ReadOctet(TPhyFrame *const arg_0x40b47930, uint8_t *const arg_0x40b47b10);
#line 27
static   result_t PhyFrameM$IPhyFrame$GetTimeStamp(const TPhyFrame *const arg_0x40b44bf0, TPhyTimeStamp *const arg_0x40b44dd8);
#line 7
static   uint8_t PhyFrameM$IPhyFrame$GetMPDULength(const TPhyFrame *const arg_0x40b482d8);
#line 23
static   result_t PhyFrameM$IPhyFrame$CheckCRC(TPhyFrame *const arg_0x40b44010);
#line 21
static   result_t PhyFrameM$IPhyFrame$CalcCRC(TPhyFrame *const arg_0x40b45a88);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t PhyPoolM$IStdControl$init(void);






static  result_t PhyPoolM$IStdControl$start(void);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t PhyPoolM$IPhyPool$Alloc(const uint8_t arg_0x40b3e6b8, const uint16_t arg_0x40b3e860, 
TPhyPoolHandle *const arg_0x40b3ea58);

static   TPhyFrame *PhyPoolM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010);



static   result_t PhyPoolM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b3dbc0);
#line 10
static   result_t PhyPoolM$IPhyPool$GetUniData(const TPhyPoolHandle arg_0x40b3d500, uint16_t *const arg_0x40b3d6f0);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void PhyPoolM$IPhyPoolReset$reset(void);
# 61 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420.nc"
static   uint16_t HPLCC2420M$HPLCC2420$read(uint8_t arg_0x40b7b248);
#line 54
static   uint8_t HPLCC2420M$HPLCC2420$write(uint8_t arg_0x40b7cb78, uint16_t arg_0x40b7cd00);
#line 47
static   uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t arg_0x40b7c678);
# 29 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
static   result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t arg_0x40b78260, uint8_t *arg_0x40b78400);
#line 19
static   result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t arg_0x40b7aa48, uint8_t *arg_0x40b7abe8);
# 47 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
static   result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t arg_0x40b98638, uint8_t arg_0x40b987b8, uint8_t *arg_0x40b98958);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t HPLCC2420M$StdControl$init(void);






static  result_t HPLCC2420M$StdControl$start(void);







static  result_t HPLCC2420M$StdControl$stop(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static  result_t HPLCC2420M$BusArbitration$busFree(void);
# 43 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLI2CInterrupt.nc"
static   void HPLUSART0M$HPLI2CInterrupt$default$fired(void);
# 53 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
static   result_t HPLUSART0M$USARTData$default$rxDone(uint8_t arg_0x40c97b88);
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
static   result_t HPLUSART0M$USARTControl$tx(uint8_t arg_0x40c54e18);






static   uint8_t HPLUSART0M$USARTControl$rx(void);
#line 185
static   result_t HPLUSART0M$USARTControl$isRxIntrPending(void);
#line 70
static   bool HPLUSART0M$USARTControl$isUARTrx(void);
# 59 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t HPLCC2420InterruptM$FIFO$disable(void);
#line 43
static   result_t HPLCC2420InterruptM$FIFO$startWait(bool arg_0x40b9b010);
#line 59
static   result_t HPLCC2420InterruptM$FIFOP$disable(void);
#line 43
static   result_t HPLCC2420InterruptM$FIFOP$startWait(bool arg_0x40b9b010);
# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void HPLCC2420InterruptM$CCAInterrupt$fired(void);
#line 59
static   void HPLCC2420InterruptM$FIFOInterrupt$fired(void);
# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t HPLCC2420InterruptM$CCA$default$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void HPLCC2420InterruptM$SFDCapture$captured(uint16_t arg_0x40746d38);
# 60 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
static   result_t HPLCC2420InterruptM$SFD$disable(void);
#line 43
static   result_t HPLCC2420InterruptM$SFD$enableCapture(bool arg_0x40b7ed48);
# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void HPLCC2420InterruptM$FIFOPInterrupt$fired(void);
#line 40
static   void MSP430InterruptM$Port14$clear(void);
#line 59
static   void MSP430InterruptM$Port14$default$fired(void);
#line 40
static   void MSP430InterruptM$Port26$clear(void);
#line 59
static   void MSP430InterruptM$Port26$default$fired(void);
#line 40
static   void MSP430InterruptM$Port17$clear(void);
#line 59
static   void MSP430InterruptM$Port17$default$fired(void);
#line 40
static   void MSP430InterruptM$Port21$clear(void);
#line 59
static   void MSP430InterruptM$Port21$default$fired(void);
#line 40
static   void MSP430InterruptM$Port12$clear(void);
#line 35
static   void MSP430InterruptM$Port12$disable(void);




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
static   void MSP430InterruptM$Port10$edge(bool arg_0x40cdd418);
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
#line 59
static   void MSP430InterruptM$Port13$default$fired(void);
#line 40
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
#line 35
static   void MSP430InterruptM$Port11$disable(void);
#line 54
static   void MSP430InterruptM$Port11$edge(bool arg_0x40cdd418);
#line 30
static   void MSP430InterruptM$Port11$enable(void);









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
uint8_t arg_0x40d73a88);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static   result_t BusArbitrationM$BusArbitration$releaseBus(
# 31 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
uint8_t arg_0x40d73a88);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static   result_t BusArbitrationM$BusArbitration$getBus(
# 31 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
uint8_t arg_0x40d73a88);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacAddressM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570);
#line 8
static  result_t MacAddressM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90);







static  result_t MacAddressM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x408ff828);

static  bool MacAddressM$IMacAddress$Equal(const TMacAddress *const arg_0x408ffd30, const TMacAddress *const arg_0x408fd010);
#line 14
static  result_t MacAddressM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328);
#line 12
static  result_t MacAddressM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28);
#line 6
static  TMacAddressMode MacAddressM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacFrameFormatM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200);
#line 27
static  result_t MacFrameFormatM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8);
#line 17
static  result_t MacFrameFormatM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250);
#line 42
static  result_t MacFrameFormatM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718);




static  result_t MacFrameFormatM$IMacFrame$SetTimeStamp(TMacFrame *const arg_0x40da6b90, TMacTimeStamp arg_0x40da6d20);
#line 30
static  result_t MacFrameFormatM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40dac908, const TMacPANId arg_0x40dacab8);








static  result_t MacFrameFormatM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, 
TMacPayloadLength *const arg_0x40da8010);
#line 36
static  result_t MacFrameFormatM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, 
const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0);
#line 15
static  result_t MacFrameFormatM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40db29a0, bool *const arg_0x40db2b88);
#line 8
static  result_t MacFrameFormatM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0);
#line 31
static  result_t MacFrameFormatM$IMacFrame$GetSrcPANId(const TMacFrame *const arg_0x40dab010, TMacPANId *const arg_0x40dab200);
#line 48
static  result_t MacFrameFormatM$IMacFrame$GetTimeStamp(const TMacFrame *const arg_0x40da5260, TMacTimeStamp *const arg_0x40da5450);
#line 44
static  TMacRawFrameLength MacFrameFormatM$IMacFrame$Pack(const TMacFrame *const arg_0x40da8c38, TPhyFrame *const arg_0x40da8e28);
#line 18
static  result_t MacFrameFormatM$IMacFrame$GetIntraPAN(const TMacFrame *const arg_0x40db1770, bool *const arg_0x40db1958);
#line 11
static  result_t MacFrameFormatM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8);
#line 45
static  result_t MacFrameFormatM$IMacFrame$UnPack(TMacFrame *const arg_0x40da63a0, TPhyFrame *const arg_0x40da6590);
#line 21
static  result_t MacFrameFormatM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40daf558, 
TMacSequenceNumber *const arg_0x40daf760);










static  result_t MacFrameFormatM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0);
#line 24
static  result_t MacFrameFormatM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40dafc60, const TMacPANId arg_0x40dafe10);



static  result_t MacFrameFormatM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40dac218, TMacAddress *const arg_0x40dac408);
#line 9
static  result_t MacFrameFormatM$IMacFrame$GetSecurityEnabled(const TMacFrame *const arg_0x40db5dd8, bool *const arg_0x40db4010);




static  result_t MacFrameFormatM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480);
#line 12
static  result_t MacFrameFormatM$IMacFrame$GetFramePending(const TMacFrame *const arg_0x40db4bd8, bool *const arg_0x40db4dc0);
#line 5
static  result_t MacFrameFormatM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98);
#line 50
static  result_t MacFrameFormatM$IMacFrame$GetLinkQuality(const TMacFrame *const arg_0x40da5970, TMacLinkQuality *const arg_0x40da5b60);
#line 34
static  result_t MacFrameFormatM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010);
#line 20
static  result_t MacFrameFormatM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030);




static  result_t MacFrameFormatM$IMacFrame$GetDstPANId(const TMacFrame *const arg_0x40dae448, TMacPANId *const arg_0x40dae638);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacCommonAttrM$IStdControl$init(void);






static  result_t MacCommonAttrM$IStdControl$start(void);
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void);
#line 8
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPANId(void);
#line 7
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dd2198);


static  TMacShortAddress MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dd29a8);

static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void);
#line 24
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void);



static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void);


static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(bool arg_0x40dcdf00);
#line 6
static  TMacPANId MacCommonAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0);
#line 18
static  TMacDSN MacCommonAttrM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dcf3a0);
#line 11
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacShortAddress(TMacShortAddress arg_0x40dd2e50);


static  TMacAckWaitDuration MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dd16c8);





static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacDSN(void);
#line 19
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dcf828);
#line 32
static  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacRandomM$IStdControl$init(void);






static  result_t MacRandomM$IStdControl$start(void);
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
static   uint16_t MacRandomM$IMacRandom$Rand(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
static   result_t MacSuperframesM$Timer$Fired(TUniData arg_0x40a37b28);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBEACON_NOTIFY.nc"
static  void MacSuperframesM$CoordBeacons$Indication(
TMacBSN arg_0x4099e708, 
TMacPanDescriptor arg_0x4099e8b0, 
TMacPendAddrSpec arg_0x4099ea58, 
uint8_t *arg_0x4099ec10, 
uint8_t arg_0x4099eda8, 
uint8_t *arg_0x4099d010);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacSuperframesM$StdControl$init(void);






static  result_t MacSuperframesM$StdControl$start(void);







static  result_t MacSuperframesM$StdControl$stop(void);
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static  result_t MacSuperframesM$IPhyTxDATA$Confirm(TPhyPoolHandle arg_0x40b34580, 
TPhyStatus arg_0x40b34720, 
TUniData arg_0x40b348b8);
# 28 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static  result_t MacSuperframesM$syncParentSF(TSysTime arg_0x40e7a010);
static  result_t MacSuperframesM$runOwnSF(TSysTime arg_0x40e7a4c0);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacSuperframesM$Reset$reset(void);
# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
static   void MacSuperframesM$IPhyTxFIFO$WriteDone(TPhyPoolHandle arg_0x40b31c08, 
result_t arg_0x40b31da0, TUniData arg_0x40b31f28);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacSuperframesM$IPhyTrxParent$Confirm(TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
#line 8
static  result_t MacSuperframesM$IPhyTrxOwn$Confirm(TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
# 11 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetmacSuperframeOrder(TMacSuperframeOrder arg_0x40e55cd8);
static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void);
#line 8
static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void);
#line 6
static  TMacBeaconOrder MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8);



static  TMacSuperframeOrder MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e55828);
#line 7
static  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e55010);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacSuperframeAttrC$Reset$reset(void);
# 9 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  void MacBeaconPackerM$IMacPool$FreeDone(const TMacPoolHandle arg_0x40ef1358, const TMacStatus arg_0x40ef1500);
# 23 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconPackerM.nc"
static  void MacBeaconPackerM$packBeacon(TPhyFrame *arg_0x40ebe260, uint64_t *arg_0x40ebe408);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacCAPM$IBackoff$Fired(TUniData arg_0x408cc858);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacCAPM$ResetAll$reset(void);
# 21 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$Send(const uint8_t arg_0x40ef5de0, 
const TMacFrame *const arg_0x40ef4030, 
const uint8_t arg_0x40ef41e0, 
const TUniData arg_0x40ef4398);
# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
static   void MacCAPM$IMacCAP$BeginCAP(
# 13 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40ef6168);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
static   void MacCAPM$IMacCAP$EndCAP(
# 13 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40ef6168);
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static  result_t MacCAPM$IPhyTxDATA$Confirm(TPhyPoolHandle arg_0x40b34580, 
TPhyStatus arg_0x40b34720, 
TUniData arg_0x40b348b8);
# 26 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$SendFromPhyPool(const uint8_t arg_0x40ef4858, 
const TPhyPoolHandle arg_0x40ef4a18, 
const uint8_t arg_0x40ef4bc8);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
static  TMacStatus MacCAPM$IMacCAPAttr$SetmacBattLifeExt(bool arg_0x40ecb0b8);
#line 20
static  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs(void);



static  TMacStatus MacCAPM$IMacCAPAttr$SetmacMinBE(
TMacBackoffExponent arg_0x40ec7798);
#line 8
static  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExt(void);





static  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods(void);



static  TMacStatus MacCAPM$IMacCAPAttr$SetmacMaxCSMABackoffs(
TMacMaxCSMABackoffs arg_0x40ec8ab8);
#line 6
static  bool MacCAPM$IMacCAPAttr$GetmacBattLifeExt(TMacStatus *const arg_0x40ec9c08);





static  TMacStatus MacCAPM$IMacCAPAttr$SetmacBattLifeExtPeriods(
TMacBattLifeExtPeriods arg_0x40ecbdb0);
#line 26
static  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacMinBE(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacCAPM$Reset$reset(
# 17 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40ef54c8);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacCAPM$ICAPControl$init(
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40ef79c8);
# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacCAPM$ICAPControl$start(
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40ef79c8);
# 78 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacCAPM$ICAPControl$stop(
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
uint8_t arg_0x40ef79c8);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacCAPM$IPhySET_TRX_STATE$Confirm(TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
# 33 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  void MacCAPM$BeaconOrderChanged(uint8_t arg_0x40f2a508);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacCAPM$IMacReceive$Receive(TMacFrame *const arg_0x40f024f8);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyRxDATA.nc"
static  result_t MacRxM$IPhyRxDATA$Indication(const TPhyPoolHandle arg_0x40b2e820);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f94168, 
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
const TMacFrame *const arg_0x40f051f8, 
const TUniData arg_0x40f053b0);



static  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$default$SendDone(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f94168, 
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
#line 10
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendFromPhyPool(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f94168, 
# 10 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
const TPhyPoolHandle arg_0x40f05868);
# 8 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendDone(const uint8_t arg_0x40f948f0, uint8_t arg_0x40f94a70, TMacStatus arg_0x40f94c00, const TUniData arg_0x40f94da8);
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Receive(const uint8_t arg_0x40f93280, TMacFrame *const arg_0x40f93470);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacPoolM$IStdControl$init(void);






static  result_t MacPoolM$IStdControl$start(void);
# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  TMacFrame *MacPoolM$IMacPool$GetFrame(const TMacPoolHandle arg_0x40ef19c0);
#line 8
static  result_t MacPoolM$IMacPool$Free(const TMacPoolHandle arg_0x40ef2c68, const TMacStatus arg_0x40ef2e10);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacPoolM$IExpiredTimer$Fired(TUniData arg_0x408cc858);
# 10 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPoolAttr.nc"
static  TMacStatus MacPoolM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void);
# 13 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
static  bool MacPoolM$SearchDstAddress(const TMacAddress arg_0x40f84ea8, 
TMacPoolHandle *const arg_0x40f830c8);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacPoolM$Reset$reset(void);
# 9 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  void MacPendingM$IMacPool$FreeDone(const TMacPoolHandle arg_0x40ef1358, const TMacStatus arg_0x40ef1500);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacPendingM$IMacCSMA$SendDone(TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacPendingM$IMacReceive$Receive(TMacFrame *const arg_0x40f024f8);
# 7 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
static  bool MacBeaconAttrCoordM$IMacBeaconAttr$GetmacAutoRequest(TMacStatus *const arg_0x40ed34a8);





static  uint8_t *MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayload(
uint8_t *const arg_0x40ed1680, 
TMacStatus *const arg_0x40ed1880);
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBeaconPayload(
uint8_t *arg_0x40ed1d78, 
uint8_t arg_0x40ed1f18);


static  TMacBSN MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBSN(TMacStatus *const arg_0x40ed0740);
#line 19
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconPayload(void);









static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconTxTime(void);
#line 23
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBSN(void);
#line 22
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBSN(TMacBSN arg_0x40ed0bc8);
#line 11
static  uint8_t MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayloadLength(
TMacStatus *const arg_0x40ed1178);
#line 9
static  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacAutoRequest(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacAssocCoordM$IStdControl$init(void);






static  result_t MacAssocCoordM$IStdControl$start(void);
# 51 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t MacAssocCoordM$IMacASSOCIATE$Response(TMacExtendedAddress arg_0x408cbc68, 
TMacShortAddress arg_0x408cbe18, 
TMacStatus arg_0x408ca010, 
bool arg_0x408ca1a8);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacAssocCoordM$IAssocRequest$Receive(TMacFrame *const arg_0x40f024f8);
# 10 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
static  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void);







static  bool MacAssocCoordM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40ec2c00);
static  TMacStatus MacAssocCoordM$IMacAssocAttr$SetmacAssociationPermit(bool arg_0x40ec10c0);
static  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacAssociationPermit(void);
#line 16
static  TMacStatus MacAssocCoordM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacAssocCoordM$Reset$reset(void);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacAssocCoordM$IMacSendOwn$SendDone(TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 3 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconCommonM.nc"
static  void MacBeaconCommonM$setLastBSN(uint8_t arg_0x410382f0);
static  uint8_t MacBeaconCommonM$getLastBSN(void);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacStartM$IPhySET_TRX_STATE$Confirm(TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
# 10 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
static  result_t MacStartM$IMacSTART$Request(
TMacPANId arg_0x40b14110, 
TMacLogicalChannel arg_0x40b142b8, 
TMacBeaconOrder arg_0x40b14458, 
TMacSuperframeOrder arg_0x40b14600, 
bool arg_0x40b14798, 
bool arg_0x40b14938, 
bool arg_0x40b14ad8, 
bool arg_0x40b14c70, 
TSysTime arg_0x40b14e20);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacDataM$IMacReceiveParent$Receive(TMacFrame *const arg_0x40f024f8);
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t MacDataM$IMacDataOwn$Request(TMacPANId arg_0x409ca0b8, 
TMacAddress arg_0x409ca250, 
TMacPANId arg_0x409ca3f0, 
TMacAddress arg_0x409ca588, 
uint8_t arg_0x409ca720, 
const uint8_t *const arg_0x409ca938, 
uint8_t arg_0x409caad0, 
TxOptions arg_0x409cac68);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacDataM$AckWaitTimer$Fired(TUniData arg_0x408cc858);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacDataM$IMacReceiveOwn$Receive(TMacFrame *const arg_0x40f024f8);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacDataM$IMacSendParent$SendDone(TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t MacDataM$IMacDataParent$Request(TMacPANId arg_0x409ca0b8, 
TMacAddress arg_0x409ca250, 
TMacPANId arg_0x409ca3f0, 
TMacAddress arg_0x409ca588, 
uint8_t arg_0x409ca720, 
const uint8_t *const arg_0x409ca938, 
uint8_t arg_0x409caad0, 
TxOptions arg_0x409cac68);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacDataM$Reset$reset(void);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacDataM$StdControl$init(void);






static  result_t MacDataM$StdControl$start(void);







static  result_t MacDataM$StdControl$stop(void);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacDataM$IMacSendOwn$SendDone(TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
#line 12
static  void MacScanBeaconM$CSMA$SendDone(TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacScanBeaconM$Reset$reset(void);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacScanBeaconM$IMacReceive$Receive(TMacFrame *const arg_0x40f024f8);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress MacCoordAttrM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08);
#line 7
static  bool MacCoordAttrM$IMacGET$GetmacAssociationPermit(TMacStatus *const arg_0x408c5168);
#line 19
static  TMacBeaconOrder MacCoordAttrM$IMacGET$GetmacBeaconOrder(TMacStatus *const arg_0x408c11d8);
#line 37
static  TMacPANId MacCoordAttrM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010);







static  TMacSuperframeOrder MacCoordAttrM$IMacGET$GetmacSuperframeOrder(TMacStatus *const arg_0x408d9440);
# 15 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
static  TMacStatus MacCoordAttrM$IMacSET$SetmacBeaconPayload(TMacBeaconPayload arg_0x408ee360, 
TMacBeaconPayloadLength arg_0x408ee510);
#line 7
static  TMacStatus MacCoordAttrM$IMacSET$SetmacAssociationPermit(bool arg_0x408f0010);
# 5 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t MacCoordAttrM$IMacRESET$Request(bool arg_0x409c6640);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacCoordAttrM$IPhySET_TRX_STATE$Confirm(TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacBeaconNotifierM$BeaconReceive$Receive(TMacFrame *const arg_0x40f024f8);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$Send(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f94168, 
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
const TMacFrame *const arg_0x40f051f8, 
const TUniData arg_0x40f053b0);



static  void /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$default$SendDone(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f94168, 
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 8 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static  void /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$SendDone(const uint8_t arg_0x40f948f0, uint8_t arg_0x40f94a70, TMacStatus arg_0x40f94c00, const TUniData arg_0x40f94da8);
static  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$Receive(const uint8_t arg_0x40f93280, TMacFrame *const arg_0x40f93470);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacExtractM$IStdControl$init(void);






static  result_t MacExtractM$IStdControl$start(void);
# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacExtract.nc"
static  void MacExtractM$IMacExtract$default$End(
# 11 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
uint8_t arg_0x411427b8, 
# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacExtract.nc"
TMacStatus arg_0x41149550, bool arg_0x411496d8, TUniData arg_0x41149860);
# 10 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPoolAttr.nc"
static  TMacStatus MacExtractM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacExtractM$IMacCSMA$SendDone(TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacExtractM$IAckWaitTimer$Fired(TUniData arg_0x408cc858);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacExtractM$Reset$reset(void);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacExtractM$IMacReceive$Receive(TMacFrame *const arg_0x40f024f8);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacAssocDeviceM$IStdControl$init(void);






static  result_t MacAssocDeviceM$IStdControl$start(void);
# 25 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t MacAssocDeviceM$IMacASSOCIATE$Request(TMacLogicalChannel arg_0x408ae290, 
TMacPANId arg_0x408ae430, TMacAddress arg_0x408ae5c0, 
TCapabilityInformation arg_0x408ae770, 
bool arg_0x408ae908);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacAssocDeviceM$IAssocAck$Receive(TMacFrame *const arg_0x40f024f8);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacAssocDeviceM$IAssocRequest$SendDone(TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacAssocDeviceM$IAssocAckTimer$Fired(TUniData arg_0x408cc858);
# 6 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
static  TMacExtendedAddress MacAssocDeviceM$IMacAssocAttr$GetmacCoordExtendedAddress(
TMacStatus *const arg_0x40ec4208);


static  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void);
#line 8
static  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetmacCoordExtendedAddress(
TMacExtendedAddress arg_0x40ec46d0);


static  TMacShortAddress MacAssocDeviceM$IMacAssocAttr$GetmacCoordShortAddress(
TMacStatus *const arg_0x40ec4f00);






static  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetDefaultmacAssociationPermit(void);
#line 14
static  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetmacCoordShortAddress(
TMacShortAddress arg_0x40ec23f8);
static  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacAssocDeviceM$IAssocResponse$Receive(TMacFrame *const arg_0x40f024f8);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacAssocDeviceM$IReceiveResponseTimer$Fired(TUniData arg_0x408cc858);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacAssocDeviceM$Reset$reset(void);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBEACON_NOTIFY.nc"
static  void MacSyncM$CoordBeacons$Indication(
TMacBSN arg_0x4099e708, 
TMacPanDescriptor arg_0x4099e8b0, 
TMacPendAddrSpec arg_0x4099ea58, 
uint8_t *arg_0x4099ec10, 
uint8_t arg_0x4099eda8, 
uint8_t *arg_0x4099d010);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacSyncM$InitialTimer$Fired(TUniData arg_0x408cc858);
# 5 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC.nc"
static  result_t MacSyncM$IMacSYNC$Request(
TMacLogicalChannel arg_0x408fca00, 
bool arg_0x408fcba8);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacSyncM$Reset$reset(void);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacSyncM$IPhySET_TRX_STATE$Confirm(TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t MacSyncM$StdControl$init(void);






static  result_t MacSyncM$StdControl$start(void);







static  result_t MacSyncM$StdControl$stop(void);
# 7 "../../../zigzag/IEEE802_15_4/MacScan/public/IMacSCAN.nc"
static  result_t MacScanM$IMacSCAN$Request(TMacScanType arg_0x4091f198, 
uint32_t arg_0x4091f338, 
uint8_t arg_0x4091f4d0);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacScanM$TimerED$Fired(TUniData arg_0x408cc858);
#line 16
static  result_t MacScanM$TimerBeaconScan$Fired(TUniData arg_0x408cc858);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyED.nc"
static  result_t MacScanM$IPhyED$Confirm(TPhyStatus arg_0x40b4b1a8, 
TPhyEnergyLevel arg_0x40b4b348, TUniData arg_0x40b4b4d0);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBEACON_NOTIFY.nc"
static  void MacScanM$AllBeacons$Indication(
TMacBSN arg_0x4099e708, 
TMacPanDescriptor arg_0x4099e8b0, 
TMacPendAddrSpec arg_0x4099ea58, 
uint8_t *arg_0x4099ec10, 
uint8_t arg_0x4099eda8, 
uint8_t *arg_0x4099d010);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void MacScanM$IMacCSMA$SendDone(TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacScanM$IPhySET_TRX_STATE$Confirm(TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress MacDeviceAttrM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08);
#line 37
static  TMacPANId MacDeviceAttrM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010);
#line 27
static  TMacShortAddress MacDeviceAttrM$IMacGET$GetmacCoordShortAddress(TMacStatus *const arg_0x408c0600);
# 36 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
static  TMacStatus MacDeviceAttrM$IMacSET$SetmacPANId(TMacPANId arg_0x408ea4a8);



static  TMacStatus MacDeviceAttrM$IMacSET$SetmacRxOnWhenIdle(bool arg_0x408eade0);
#line 24
static  TMacStatus MacDeviceAttrM$IMacSET$SetmacCoordExtendedAddress(TMacExtendedAddress arg_0x408ed7f0);
#line 42
static  TMacStatus MacDeviceAttrM$IMacSET$SetmacShortAddress(TMacShortAddress arg_0x40907358);
#line 18
static  TMacStatus MacDeviceAttrM$IMacSET$SetmacBeaconOrder(TMacBeaconOrder arg_0x408ee9b8);







static  TMacStatus MacDeviceAttrM$IMacSET$SetmacCoordShortAddress(TMacShortAddress arg_0x408edca0);
# 5 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t MacDeviceAttrM$IMacRESET$Request(bool arg_0x409c6640);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t MacDeviceAttrM$IPhySET_TRX_STATE$Confirm(TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t AliveSender2M$IMacDATA$Confirm(uint8_t arg_0x409e8348, TMacStatus arg_0x409e84d8);
#line 16
static  result_t AliveSender2M$IMacDATA$Indication(TMacPANId arg_0x409e9178, 
TMacAddress arg_0x409e9310, 
TMacPANId arg_0x409e94b0, 
TMacAddress arg_0x409e9648, 
uint8_t arg_0x409e97e0, 
uint8_t *arg_0x409e9998, 
TMacLinkQuality arg_0x409e9b38, 
bool arg_0x409e9cd0, 
uint8_t arg_0x409e9e68);
# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t AliveSender2M$StdControl$start(void);







static  result_t AliveSender2M$StdControl$stop(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t AliveSender2M$Timer$Fired(TUniData arg_0x408cc858);
# 22 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
static  result_t NLME_StartRouterM$IMacSTART$Confirm(TMacStatus arg_0x40b13338);
# 15 "../../../zigzag/ZigBee/interface/NLME_StartRouter.nc"
static  void NLME_StartRouterM$NLME_StartRouter$request(


uint16_t arg_0x4086f758, 
bool arg_0x4086f908);
# 5 "../../../zigzag/ZigBee/implementation/NwkBeaconOffsetRandomM.nc"
static  result_t NwkBeaconOffsetRandomM$findOffset(uint8_t arg_0x412c4010, uint8_t arg_0x412c4198, TSysTime *arg_0x412c4340);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t ChildSupervisorM$ITimerSymbol$Fired(TUniData arg_0x408cc858);
# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ChildSupervisorM$StdControl$start(void);







static  result_t ChildSupervisorM$StdControl$stop(void);
#line 63
static  result_t PowerManagerM$StdControl$init(void);






static  result_t PowerManagerM$StdControl$start(void);
#line 63
static  result_t ZigSysM$StdControl$init(void);






static  result_t ZigSysM$StdControl$start(void);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t ZigSTimerM$Timer0$fired(void);
#line 37
static  result_t ZigSTimerM$Timer1$fired(void);
#line 37
static  result_t ZigSTimerM$Timer2$fired(void);
# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void ZigNetM$NLME_Reset$confirm(NwkStatus arg_0x40867458);
# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
static  void ZigNetM$NLME_Leave$indication(IEEEAddr arg_0x4088a300);
# 15 "../../../zigzag/ZigBee/interface/NLME_JoinParent.nc"
static  void ZigNetM$NLME_JoinParent$indication(
NwkAddr arg_0x408a8a70, 
IEEEAddr arg_0x408a8c10, 
NwkCapabilityInfo arg_0x408a8dc0, 
bool arg_0x408a6010);
# 33 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
static  void ZigNetM$NLDE_Data$confirm(
uint8_t arg_0x408a3338, 
NwkStatus arg_0x408a34d0);



static  void ZigNetM$NLDE_Data$indication(
NwkAddr arg_0x408a3960, 
IEEEAddr arg_0x408a3b00, 
uint8_t arg_0x408a3c98, 
uint8_t *arg_0x408a3e48, 
uint8_t arg_0x408a2010);
# 27 "../../../zigzag/ZigBee/interface/NLME_JoinChild.nc"
static  void ZigNetM$NLME_JoinChild$confirm(
PanID_t arg_0x40870010, 
NwkStatus arg_0x408701b8);
# 7 "../../../zigzag/ZigNetM.nc"
static  void ZigNetM$sleepIndication(TMilliSec arg_0x4133b580);
# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
static  void ZigMonitorM$NLME_Leave$indication(IEEEAddr arg_0x4088a300);
# 15 "../../../zigzag/ZigBee/interface/NLME_JoinParent.nc"
static  void ZigMonitorM$NLME_JoinParent$indication(
NwkAddr arg_0x408a8a70, 
IEEEAddr arg_0x408a8c10, 
NwkCapabilityInfo arg_0x408a8dc0, 
bool arg_0x408a6010);
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

enum MSP430ClockM$__nesc_unnamed4391 {

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

enum MSP430DCOCalibM$__nesc_unnamed4392 {

  MSP430DCOCalibM$TARGET_DELTA = 2048, 
  MSP430DCOCalibM$MAX_DEVIATION = 7
};


static inline   void MSP430DCOCalibM$TimerMicro$overflow(void);
#line 75
static inline   void MSP430DCOCalibM$Timer32khz$overflow(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureA1$captured(uint16_t arg_0x40746d38);
#line 74
static   void MSP430TimerM$CaptureB3$captured(uint16_t arg_0x40746d38);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA1$fired(void);
#line 34
static   void MSP430TimerM$CompareB3$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureB6$captured(uint16_t arg_0x40746d38);
#line 74
static   void MSP430TimerM$CaptureB1$captured(uint16_t arg_0x40746d38);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB1$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureA2$captured(uint16_t arg_0x40746d38);
#line 74
static   void MSP430TimerM$CaptureB4$captured(uint16_t arg_0x40746d38);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA2$fired(void);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerA$overflow(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB4$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureA0$captured(uint16_t arg_0x40746d38);
#line 74
static   void MSP430TimerM$CaptureB2$captured(uint16_t arg_0x40746d38);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareA0$fired(void);
#line 34
static   void MSP430TimerM$CompareB2$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureB5$captured(uint16_t arg_0x40746d38);
# 33 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void MSP430TimerM$TimerB$overflow(void);
# 34 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void MSP430TimerM$CompareB5$fired(void);
# 74 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void MSP430TimerM$CaptureB0$captured(uint16_t arg_0x40746d38);
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
# 15 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void ZigRouterM$NLME_Reset$request(void);
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t ZigRouterM$TimerMilli$setOneShot(int32_t arg_0x40887668);
# 55 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  void ZigRouterM$NIB$setNwkMaxDepth(uint8_t arg_0x4087d550);









static  void ZigRouterM$NIB$setNwkMaxRouters(uint8_t arg_0x4087dcc8);
#line 47
static  void ZigRouterM$NIB$setNwkMaxChildren(uint8_t arg_0x4087edc0);
# 15 "../../../zigzag/ZigBee/interface/NLME_NetworkDiscovery.nc"
static  void ZigRouterM$NLME_NetworkDiscovery$request(
uint32_t arg_0x4086d358, 
uint8_t arg_0x4086d500);
# 16 "../../../zigzag/ZigBee/interface/NLME_PermitJoining.nc"
static  NwkStatus ZigRouterM$NLME_PermitJoining$request(
uint8_t arg_0x408728c0);
# 15 "../../../zigzag/ZigBee/interface/NLME_JoinChild.nc"
static  void ZigRouterM$NLME_JoinChild$request(
PanID_t arg_0x4085bd90, 
bool arg_0x4085bf28, 
bool arg_0x408730d8, 
IEEEAddr *arg_0x408732a0, 
uint32_t arg_0x40873440, 
uint8_t arg_0x408735d8, 
uint8_t arg_0x40873770, 
bool arg_0x40873908, 
bool arg_0x40873ab0);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t ZigRouterM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790);
# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t ZigRouterM$AliveSenderControl$start(void);







static  result_t ZigRouterM$AliveSenderControl$stop(void);
#line 70
static  result_t ZigRouterM$ChildSupervisorControl$start(void);







static  result_t ZigRouterM$ChildSupervisorControl$stop(void);
# 15 "../../../zigzag/ZigBee/interface/NLME_StartRouter.nc"
static  void ZigRouterM$NLME_StartRouter$request(


uint16_t arg_0x4086f758, 
bool arg_0x4086f908);
# 25 "../../../zigzag/ZigRouterM.nc"
enum ZigRouterM$__nesc_unnamed4393 {
  ZigRouterM$IDLE_SCAN_INTERVAL = 20, 
  ZigRouterM$REJOIN_INTERVAL = 2
};


bool ZigRouterM$idle_scan;

static inline  void ZigRouterM$join(void);

static inline  void ZigRouterM$start_router(void);
static inline  void ZigRouterM$start_discovery(void);

static inline  void ZigRouterM$restart(void);







static inline  void ZigRouterM$NLME_Reset$confirm(NwkStatus status);






static inline  result_t ZigRouterM$StdControl$start(void);









static inline  result_t ZigRouterM$TimerMilli$fired(void);






static inline  void ZigRouterM$start_discovery(void);










static  void ZigRouterM$NLME_NetworkDiscovery$confirm(
uint8_t networkCount, 
NwkNetworkDescriptor *networkDescriptors, 
NwkStatus status);
#line 105
static inline  void ZigRouterM$join(void);
#line 122
static  void ZigRouterM$NLME_JoinChild$confirm(
PanID_t panID, 
NwkStatus status);
#line 137
static inline  void ZigRouterM$start_router(void);










static  void ZigRouterM$NLME_StartRouter$confirm(NwkStatus status);
#line 163
static  void ZigRouterM$NLME_Sync$indication(void);







static inline  void ZigRouterM$NLME_Leave$indication(IEEEAddr deviceAddr);









static inline  result_t ZigRouterM$StdControl$init(void);
# 196 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  uint8_t NLME_JoinParentM$NIB$getDepth(void);



static  uint8_t NLME_JoinParentM$NIB$getChannel(void);
# 51 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t NLME_JoinParentM$IMacASSOCIATE$Response(TMacExtendedAddress arg_0x408cbc68, 
TMacShortAddress arg_0x408cbe18, 
TMacStatus arg_0x408ca010, 
bool arg_0x408ca1a8);
# 19 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacBeaconOrder NLME_JoinParentM$IMacGET$GetmacBeaconOrder(TMacStatus *const arg_0x408c11d8);
#line 37
static  TMacPANId NLME_JoinParentM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010);
# 28 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
static  void NLME_JoinParentM$updateBeacon(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime NLME_JoinParentM$ILocalTime$Read(void);
# 15 "../../../zigzag/ZigBee/interface/NLME_JoinParent.nc"
static  void NLME_JoinParentM$NLME_JoinParent$indication(
NwkAddr arg_0x408a8a70, 
IEEEAddr arg_0x408a8c10, 
NwkCapabilityInfo arg_0x408a8dc0, 
bool arg_0x408a6010);
# 15 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NLME_JoinParentM$NeighborTable$update(NwkNeighbor arg_0x4086b2b8);
# 15 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  result_t NLME_JoinParentM$NwkAddressing$allocEdAddress(NwkAddr *arg_0x408d66e0);

static  result_t NLME_JoinParentM$NwkAddressing$allocRouterAddress(NwkAddr *arg_0x408d6b90);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t NLME_JoinParentM$ITimerSymbol$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
# 34 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
NwkAddr NLME_JoinParentM$joiningNwkAddr;
IEEEAddr NLME_JoinParentM$joiningExtAddr;
NwkCapabilityInfo NLME_JoinParentM$joiningCapability;

enum NLME_JoinParentM$__nesc_unnamed4394 {
#line 38
  NLME_JoinParentM$WAIT_BEACONS = 3
};
NwkStatus NLME_JoinParentM$addr_allocated;
enum NLME_JoinParentM$__nesc_unnamed4395 {
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
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t NLME_JoinChildM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328);
#line 12
static  result_t NLME_JoinChildM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28);
# 117 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  NwkCapabilityInfo NLME_JoinChildM$NIB$getNwkCapabilityInformation(void);
#line 191
static  void NLME_JoinChildM$NIB$setOnline(NwkOnlineStatus arg_0x40875ae0);
#line 116
static  void NLME_JoinChildM$NIB$setNwkCapabilityInformation(NwkCapabilityInfo arg_0x40879398);
#line 197
static  void NLME_JoinChildM$NIB$setDepth(uint8_t arg_0x4088f918);



static  void NLME_JoinChildM$NIB$setChannel(uint8_t arg_0x4088e0b0);
#line 193
static  bool NLME_JoinChildM$NIB$isJoined(void);
# 25 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t NLME_JoinChildM$IMacASSOCIATE$Request(TMacLogicalChannel arg_0x408ae290, 
TMacPANId arg_0x408ae430, TMacAddress arg_0x408ae5c0, 
TCapabilityInformation arg_0x408ae770, 
bool arg_0x408ae908);
# 36 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
static  TMacStatus NLME_JoinChildM$IMacSET$SetmacPANId(TMacPANId arg_0x408ea4a8);



static  TMacStatus NLME_JoinChildM$IMacSET$SetmacRxOnWhenIdle(bool arg_0x408eade0);
#line 24
static  TMacStatus NLME_JoinChildM$IMacSET$SetmacCoordExtendedAddress(TMacExtendedAddress arg_0x408ed7f0);
#line 42
static  TMacStatus NLME_JoinChildM$IMacSET$SetmacShortAddress(TMacShortAddress arg_0x40907358);
#line 18
static  TMacStatus NLME_JoinChildM$IMacSET$SetmacBeaconOrder(TMacBeaconOrder arg_0x408ee9b8);







static  TMacStatus NLME_JoinChildM$IMacSET$SetmacCoordShortAddress(TMacShortAddress arg_0x408edca0);
# 27 "../../../zigzag/ZigBee/interface/NLME_JoinChild.nc"
static  void NLME_JoinChildM$NLME_JoinChild$confirm(
PanID_t arg_0x40870010, 
NwkStatus arg_0x408701b8);
# 5 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC.nc"
static  result_t NLME_JoinChildM$IMacSYNC$Request(
TMacLogicalChannel arg_0x408fca00, 
bool arg_0x408fcba8);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NLME_JoinChildM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790);







static  result_t NLME_JoinChildM$NeighborTable$getByAddr(TMacAddress arg_0x4086bc20, NwkNeighbor **arg_0x4086bdf0);
# 34 "../../../zigzag/ZigBee/implementation/NLME_JoinChildM.nc"
NwkAddr NLME_JoinChildM$potential_parent_addr;

PanID_t NLME_JoinChildM$request_panID;
bool NLME_JoinChildM$request_joinAsRouter;
bool NLME_JoinChildM$joinToSpecified;
IEEEAddr NLME_JoinChildM$joinAddress;
NwkStatus NLME_JoinChildM$confirm_status;

static  void NLME_JoinChildM$try_to_assoc(void);



static inline  void NLME_JoinChildM$NLME_JoinChild$request(
PanID_t panID, 
bool joinAsRouter, 
bool rejoinNetwork, 
IEEEAddr *joinToSpecifiedParent, 
uint32_t scanChannels, 
uint8_t scanDuration, 
uint8_t powerSource, 
bool rxOnWhenIdle, 
bool macSecurity);
#line 102
static  void NLME_JoinChildM$try_to_assoc(void);
#line 183
static  result_t NLME_JoinChildM$IMacASSOCIATE$Confirm(NwkAddr assocAddr, TMacStatus status);
# 7 "../../../zigzag/IEEE802_15_4/MacScan/public/IMacSCAN.nc"
static  result_t NLME_NetworkDiscoveryM$IMacSCAN$Request(TMacScanType arg_0x4091f198, 
uint32_t arg_0x4091f338, 
uint8_t arg_0x4091f4d0);
# 19 "../../../zigzag/ZigBee/interface/NLME_NetworkDiscovery.nc"
static  void NLME_NetworkDiscoveryM$NLME_NetworkDiscovery$confirm(
uint8_t arg_0x4086d990, 
NwkNetworkDescriptor *arg_0x4086db60, 
NwkStatus arg_0x4086dd08);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NLME_NetworkDiscoveryM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790);
# 30 "../../../zigzag/ZigBee/implementation/NLME_NetworkDiscoveryM.nc"
static inline  void NLME_NetworkDiscoveryM$NLME_NetworkDiscovery$request(
uint32_t scanChannels, 
uint8_t scanDuration);










static  result_t NLME_NetworkDiscoveryM$IMacSCAN$Confirm(TMacStatus status, TMacScanType scanType, uint32_t unscannedChannels, uint8_t resultListSize, uint8_t *energyDetectList, TMacPanDescriptor *panDescriptorList);
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
static   uint16_t NIBM$Rand$Rand(void);
# 21 "../../../zigzag/ZigBee/implementation/NIBM.nc"
enum NIBM$__nesc_unnamed4396 {

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
NwkCapabilityInfo NIBM$nwkCapabilityInformation;
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
#line 169
static inline  void NIBM$NIB$setNwkCapabilityInformation(NwkCapabilityInfo attr);



static inline  NwkCapabilityInfo NIBM$NIB$getNwkCapabilityInformation(void);
#line 238
static  void NIBM$Reset$reset(void);
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
static  TMacShortAddress NwkAddressingM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NwkAddressingM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790);
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
static inline  bool NwkAddressingM$NwkAddressing$hasVacantAddrs(void);







static inline  bool NwkAddressingM$NwkAddressing$hasVacantEdAddrs(void);







static inline  bool NwkAddressingM$NwkAddressing$hasVacantRouterAddrs(void);
#line 204
static inline  void NwkAddressingM$NwkAddressing$getDirection(NwkAddr dest, NwkRelationship *relationship, NwkAddr *next_hop);
#line 234
static inline  void NwkAddressingM$NLME_JoinChild$confirm(TMacPANId panid, NwkStatus status);
# 17 "../../../zigzag/ZigBee/interface/NLME_Sync.nc"
static  void NLME_SyncM$NLME_Sync$indication(void);
# 54 "../../../zigzag/ZigBee/implementation/NLME_SyncM.nc"
static inline  void NLME_SyncM$IMacSYNC_LOSS$Indication(TMacStatus reason);
# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  bool NwkBeaconParentM$IMacGET$GetmacAssociationPermit(TMacStatus *const arg_0x408c5168);
# 15 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
static  TMacStatus NwkBeaconParentM$IMacSET$SetmacBeaconPayload(TMacBeaconPayload arg_0x408ee360, 
TMacBeaconPayloadLength arg_0x408ee510);
# 196 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  uint8_t NwkBeaconParentM$NIB$getDepth(void);








static  TSysTime NwkBeaconParentM$NIB$getBeaconOffset(void);
# 20 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  bool NwkBeaconParentM$NwkAddressing$hasVacantEdAddrs(void);
static  bool NwkBeaconParentM$NwkAddressing$hasVacantRouterAddrs(void);
# 27 "../../../zigzag/ZigBee/implementation/NwkBeaconParentM.nc"
static  void NwkBeaconParentM$updateBeacon(void);
# 8 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t NwkBeaconChildM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90);





static  result_t NwkBeaconChildM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328);
#line 6
static  TMacAddressMode NwkBeaconChildM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8);
# 34 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NwkBeaconChildM$NeighborTable$getFreePtr(NwkNeighbor **arg_0x40869838);
#line 29
static  result_t NwkBeaconChildM$NeighborTable$getByAddr(TMacAddress arg_0x4086bc20, NwkNeighbor **arg_0x4086bdf0);
# 25 "../../../zigzag/ZigBee/implementation/NwkBeaconChildM.nc"
static inline  void NwkBeaconChildM$IMacBEACON_NOTIFY$Indication(
TMacBSN bsn, 
TMacPanDescriptor panDescriptor, 
TMacPendAddrSpec pendAddrSpec, 
uint8_t *addrList, 
uint8_t sduLength, 
uint8_t *sdu);
# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
static  TMacStatus NLME_PermitJoiningM$IMacSET$SetmacAssociationPermit(bool arg_0x408f0010);
# 23 "../../../zigzag/ZigBee/implementation/NLME_PermitJoiningM.nc"
static  void NLME_PermitJoiningM$updateBeacon(void);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t NLME_PermitJoiningM$ITimerSymbol$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
# 19 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  bool NLME_PermitJoiningM$NwkAddressing$hasVacantAddrs(void);
# 30 "../../../zigzag/ZigBee/implementation/NLME_PermitJoiningM.nc"
static  NwkStatus NLME_PermitJoiningM$NLME_PermitJoining$request(
uint8_t permitDuration);
#line 53
static inline  result_t NLME_PermitJoiningM$ITimerSymbol$Fired(TUniData u);
# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void NLME_ResetM$NLME_Reset$confirm(NwkStatus arg_0x40867458);
#line 15
static  void NLME_ResetM$MacReset$request(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void NLME_ResetM$Reset$reset(void);
# 21 "../../../zigzag/ZigBee/implementation/NLME_ResetM.nc"
bool NLME_ResetM$waitConfirm = FALSE;
static inline  void NLME_ResetM$NLME_Reset$request(void);






static inline  void NLME_ResetM$MacReset$confirm(NwkStatus status);
# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
static  void NwkResetMacM$NLME_Reset$confirm(NwkStatus arg_0x40867458);
# 5 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t NwkResetMacM$IMacRESET1$Request(bool arg_0x409c6640);
#line 5
static  result_t NwkResetMacM$IMacRESET2$Request(bool arg_0x409c6640);
# 16 "../../../zigzag/ZigBee/implementation/NwkResetMacM.nc"
static inline  void NwkResetMacM$NLME_Reset$request(void);




static inline  result_t NwkResetMacM$IMacRESET1$Confirm(TMacStatus status);





static inline  result_t NwkResetMacM$IMacRESET2$Confirm(TMacStatus status);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t NeighborTableM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570);
#line 8
static  result_t NeighborTableM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90);
#line 6
static  TMacAddressMode NeighborTableM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8);
# 32 "../../../zigzag/ZigBee/implementation/NeighborTableM.nc"
NwkNeighbor NeighborTableM$Table[20];

int NeighborTableM$free_idx = 0;

static  void NeighborTableM$Reset$reset(void);
#line 49
static inline  result_t NeighborTableM$NeighborTable$update(NwkNeighbor new_neighbor);
#line 84
static  result_t NeighborTableM$NeighborTable$getNextPtr(NwkNeighbor **neighbor);
#line 102
static  result_t NeighborTableM$NeighborTable$getByAddr(TMacAddress addr, NwkNeighbor **pneighbor);
#line 137
static inline  result_t NeighborTableM$NeighborTable$remove(NwkNeighbor *pneighbor);





static inline  result_t NeighborTableM$NeighborTable$getFreePtr(NwkNeighbor **pneighbor);
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t RouterM$LowerIf$Request(TMacPANId arg_0x409ca0b8, 
TMacAddress arg_0x409ca250, 
TMacPANId arg_0x409ca3f0, 
TMacAddress arg_0x409ca588, 
uint8_t arg_0x409ca720, 
const uint8_t *const arg_0x409ca938, 
uint8_t arg_0x409caad0, 
TxOptions arg_0x409cac68);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress RouterM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08);
#line 37
static  TMacPANId RouterM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010);
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t RouterM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28);
# 192 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  NwkOnlineStatus RouterM$NIB$getOnlineStatus(void);
static  bool RouterM$NIB$isJoined(void);
# 7 "../../../zigzag/ZigBee/implementation/NwkFrame.nc"
static  uint8_t RouterM$NwkFrame$mkDataFrameHeader(uint8_t *arg_0x409e2938, 
NwkAddr arg_0x409e2ac8, 
uint8_t arg_0x409e2c58, 
uint8_t arg_0x409e2df0, 
bool arg_0x409e0010);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime RouterM$ILocalTime$Read(void);
# 33 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
static  void RouterM$NLDE_Data$confirm(
uint8_t arg_0x408a3338, 
NwkStatus arg_0x408a34d0);



static  void RouterM$NLDE_Data$indication(
NwkAddr arg_0x408a3960, 
IEEEAddr arg_0x408a3b00, 
uint8_t arg_0x408a3c98, 
uint8_t *arg_0x408a3e48, 
uint8_t arg_0x408a2010);
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t RouterM$UpperIf$Request(TMacPANId arg_0x409ca0b8, 
TMacAddress arg_0x409ca250, 
TMacPANId arg_0x409ca3f0, 
TMacAddress arg_0x409ca588, 
uint8_t arg_0x409ca720, 
const uint8_t *const arg_0x409ca938, 
uint8_t arg_0x409caad0, 
TxOptions arg_0x409cac68);
# 25 "../../../zigzag/ZigBee/implementation/NwkAddressing.nc"
static  void RouterM$NwkAddressing$getDirection(NwkAddr arg_0x408d4928, NwkRelationship *arg_0x408d4ad8, NwkAddr *arg_0x408d4c78);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t RouterM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790);







static  result_t RouterM$NeighborTable$getByAddr(TMacAddress arg_0x4086bc20, NwkNeighbor **arg_0x4086bdf0);
# 34 "../../../zigzag/ZigBee/implementation/RouterM.nc"
enum RouterM$__nesc_unnamed4397 {

  RouterM$NWK_FRAME_BUFFER_SIZE = 8, 
  RouterM$NWK_MAX_PAYLOAD_SIZE = MAC_AMAX_MAC_FRAME_SIZE - sizeof(NwkHeader ), 


  RouterM$NWK_FRAME_NULL = 0, 
  RouterM$NWK_FRAME_PENDING = 1
};








#line 44
typedef struct RouterM$__nesc_unnamed4398 {

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
static result_t RouterM$IMacDATA_Confirm(uint8_t msduHandle, TMacStatus status);
#line 292
static inline result_t RouterM$find_place(uint8_t *idx);

static result_t RouterM$IMacDATA_Indication(
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
static  void RouterM$Reset$reset(void);
#line 489
static inline  result_t RouterM$UpperIf$Confirm(uint8_t msduHandle, TMacStatus status);




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






static inline  result_t RouterM$UpperIf$Indication(
TMacPANId srcPanID, 
TMacAddress src, 
TMacPANId dstPanID, 
TMacAddress dst, 
uint8_t msduLength, 
uint8_t *msdu, 
TMacLinkQuality mpduLinkQuality, 
bool securityUse, 
uint8_t aclEntry);
#line 571
static inline  void RouterM$NLME_JoinChild$confirm(TMacPANId panid, NwkStatus status);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress NwkFrameM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08);
# 20 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  uint8_t NwkFrameM$NIB$getNwkSequenceNumber(void);
#line 19
static  void NwkFrameM$NIB$setNwkSequenceNumber(uint8_t arg_0x4087f6c8);
# 15 "../../../zigzag/ZigBee/implementation/NwkFrameM.nc"
static inline  uint8_t NwkFrameM$NwkFrame$mkDataFrameHeader(uint8_t *msdu, 
NwkAddr dstAddr, 
uint8_t radius, 
uint8_t discoverRoute, 
bool securityEnable);
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t TimerSymbol2M$TimerMilli$fired(
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a4c810);
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   uint32_t TimerSymbol2M$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a3c4c8);
#line 14
static   uint32_t TimerSymbol2M$ITimeCast$MillisToJiffies(TMilliSec arg_0x40a3c010);
#line 7
static   TSysTime TimerSymbol2M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a3eb98);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbol2M$AlarmControl$enableEvents(void);
#line 35
static   void TimerSymbol2M$AlarmControl$setControlAsCompare(void);



static   void TimerSymbol2M$AlarmControl$disableEvents(void);
#line 32
static   void TimerSymbol2M$AlarmControl$clearPendingInterrupt(void);
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbol2M$AlarmCompare$setEventFromNow(uint16_t arg_0x40733b20);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   uint16_t TimerSymbol2M$AlarmTimer$read(void);
static   bool TimerSymbol2M$AlarmTimer$isOverflowPending(void);
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t TimerSymbol2M$ITimerSymbol$Fired(
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
uint8_t arg_0x40a4dc70, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
TUniData arg_0x408cc858);
# 21 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
enum TimerSymbol2M$__nesc_unnamed4399 {
  TimerSymbol2M$NUM_SYMBOL_TIMERS = 14U, 
  TimerSymbol2M$NUM_MILLIS_TIMERS = 4U, 
  TimerSymbol2M$NUM_TIMERS = TimerSymbol2M$NUM_SYMBOL_TIMERS + TimerSymbol2M$NUM_MILLIS_TIMERS, 
  TimerSymbol2M$EMPTY_LIST = 255, 
  TimerSymbol2M$MAX_HW_COUNTER = 0xffffL, 
  TimerSymbol2M$MIN_HW_COUNTER = 2
};

typedef uint32_t TimerSymbol2M$TJiffy;










#line 32
typedef struct TimerSymbol2M$__nesc_unnamed4400 {
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

static  result_t TimerSymbol2M$IStdControl$init(void);










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
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   uint32_t TimerSymbolAsyncM$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a3c4c8);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare1$setEvent(uint16_t arg_0x407331d8);
#line 30
static   void TimerSymbolAsyncM$AlarmCompare4$setEvent(uint16_t arg_0x407331d8);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl3$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl3$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl3$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl3$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare2$setEvent(uint16_t arg_0x407331d8);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl0$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl0$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl0$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl0$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare0$setEvent(uint16_t arg_0x407331d8);
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerSymbolAsyncM$AlarmControl4$enableEvents(void);
#line 35
static   void TimerSymbolAsyncM$AlarmControl4$setControlAsCompare(void);



static   void TimerSymbolAsyncM$AlarmControl4$disableEvents(void);
#line 32
static   void TimerSymbolAsyncM$AlarmControl4$clearPendingInterrupt(void);
# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
static   void TimerSymbolAsyncM$AlarmCompare3$setEvent(uint16_t arg_0x407331d8);
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
uint8_t arg_0x40a93770, 
# 16 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
TUniData arg_0x40a37b28);
# 33 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
enum TimerSymbolAsyncM$__nesc_unnamed4401 {
  TimerSymbolAsyncM$ASYNC_TIMER_NUM = 5, 
  TimerSymbolAsyncM$MAX_HW_COUNTER = 0xFFFF, 
  TimerSymbolAsyncM$MIN_HW_COUNTER = 2, 
  TimerSymbolAsyncM$WARNING_INTERVAL = 10
};








#line 40
struct TimerSymbolAsyncM$__nesc_unnamed4402 {
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









static  result_t TimerSymbolAsyncM$IStdControl$init(void);









static  result_t TimerSymbolAsyncM$IStdControl$start(void);
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
static   void TimerTuningM$ITimerB$setClockSource(uint16_t arg_0x4072b970);
#line 32
static   void TimerTuningM$ITimerB$clearOverflow(void);


static   void TimerTuningM$ITimerB$setMode(int arg_0x4072db50);




static   void TimerTuningM$ITimerB$setInputDivider(uint16_t arg_0x4072be18);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
static   void TimerTuningM$ITimerB4Control$disableEvents(void);
#line 39
static   void TimerTuningM$ITimerB5Control$disableEvents(void);
#line 39
static   void TimerTuningM$ITimerB6Control$disableEvents(void);
#line 39
static   void TimerTuningM$ITimerB1Control$disableEvents(void);
#line 34
static   void TimerTuningM$ITimerB1Control$setControl(MSP430CompareControl_t arg_0x40739b58);




static   void TimerTuningM$ITimerB3Control$disableEvents(void);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
static   void TimerTuningM$ITimerA$setClockSource(uint16_t arg_0x4072b970);
#line 38
static   void TimerTuningM$ITimerA$disableEvents(void);
#line 32
static   void TimerTuningM$ITimerA$clearOverflow(void);


static   void TimerTuningM$ITimerA$setMode(int arg_0x4072db50);
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
static   TMilliSec LocalTime64M$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a3d378);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime LocalTime64M$ILocalTime$Read(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t LocalTime64M$ITimerSymbol$Stop(void);
#line 6
static  result_t LocalTime64M$ITimerSymbol$SetPeriodic(TSysTime arg_0x408cdca8, TUniData arg_0x408cde30);




static  bool LocalTime64M$ITimerSymbol$IsSet(void);
# 17 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
uint64_t LocalTime64M$baseTime = 0;
TSysTime LocalTime64M$firedCount;

static  uint64_t LocalTime64M$ILocalTime64$getLocalTimeAt(TSysTime at);









static  uint64_t LocalTime64M$ILocalTime64$getLocalTime(void);




static  void LocalTime64M$ILocalTime64$setLocalTimeAt(TSysTime at, uint64_t time);
#line 47
static inline  void LocalTime64M$ILocalTime64$setLocalTime(uint64_t time);




static inline  result_t LocalTime64M$ITimerSymbol$Fired(TUniData ud);
# 12 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   TSysTime TimeCastM$ITimeCast$MillisToSymbols(TMilliSec millis);



static inline   TSysTime TimeCastM$ITimeCast$JiffiesToSymbols(uint32_t jiffies);








static inline   TMilliSec TimeCastM$ITimeCast$SymbolsToMillis(TSysTime symbols);
#line 38
static inline   uint32_t TimeCastM$ITimeCast$MillisToJiffies(TMilliSec millis);



static inline   uint32_t TimeCastM$ITimeCast$SymbolsToJiffies(TSysTime symbols);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyRxDATA.nc"
static  result_t PhyCC2420M$IPhyRxDATA$Indication(const TPhyPoolHandle arg_0x40b2e820);
# 61 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420.nc"
static   uint16_t PhyCC2420M$IChipcon$read(uint8_t arg_0x40b7b248);
#line 54
static   uint8_t PhyCC2420M$IChipcon$write(uint8_t arg_0x40b7cb78, uint16_t arg_0x40b7cd00);
#line 47
static   uint8_t PhyCC2420M$IChipcon$cmd(uint8_t arg_0x40b7c678);
# 47 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
static   result_t PhyCC2420M$IChipconRAM$write(uint16_t arg_0x40b98638, uint8_t arg_0x40b987b8, uint8_t *arg_0x40b98958);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   TSysTime PhyCC2420M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a3eb98);
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static  result_t PhyCC2420M$IPhyTxDATA$Confirm(
# 9 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b57010, 
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
TPhyPoolHandle arg_0x40b34580, 
TPhyStatus arg_0x40b34720, 
TUniData arg_0x40b348b8);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t PhyCC2420M$IPhyPool$Alloc(const uint8_t arg_0x40b3e6b8, const uint16_t arg_0x40b3e860, 
TPhyPoolHandle *const arg_0x40b3ea58);

static   TPhyFrame *PhyCC2420M$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010);
# 59 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t PhyCC2420M$IChipconFIFOSignal$disable(void);
#line 43
static   result_t PhyCC2420M$IChipconFIFOSignal$startWait(bool arg_0x40b9b010);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyED.nc"
static  result_t PhyCC2420M$IPhyED$Confirm(
# 11 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b57998, 
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyED.nc"
TPhyStatus arg_0x40b4b1a8, 
TPhyEnergyLevel arg_0x40b4b348, TUniData arg_0x40b4b4d0);
# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
static  result_t PhyCC2420M$HPLCC2420Control$init(void);






static  result_t PhyCC2420M$HPLCC2420Control$start(void);







static  result_t PhyCC2420M$HPLCC2420Control$stop(void);
# 59 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t PhyCC2420M$IChipconFIFOP$disable(void);
#line 43
static   result_t PhyCC2420M$IChipconFIFOP$startWait(bool arg_0x40b9b010);
# 60 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
static   result_t PhyCC2420M$IChipconSFD$disable(void);
#line 43
static   result_t PhyCC2420M$IChipconSFD$enableCapture(bool arg_0x40b7ed48);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   uint8_t *PhyCC2420M$IPhyFrame$GetPPDU(const TPhyFrame *const arg_0x40b2bd60);
#line 25
static   result_t PhyCC2420M$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b44528, TPhyTimeStamp arg_0x40b446b0);
#line 9
static   uint8_t PhyCC2420M$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b48808);
# 29 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
static   result_t PhyCC2420M$IChipconFIFO$writeTXFIFO(uint8_t arg_0x40b78260, uint8_t *arg_0x40b78400);
#line 19
static   result_t PhyCC2420M$IChipconFIFO$readRXFIFO(uint8_t arg_0x40b7aa48, uint8_t *arg_0x40b7abe8);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static  result_t PhyCC2420M$IPhySET_TRX_STATE$Confirm(
# 13 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b81340, 
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198);
# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
static   void PhyCC2420M$IPhyTxFIFO$WriteDone(
# 17 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
uint8_t arg_0x40b80bf8, 
# 10 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
TPhyPoolHandle arg_0x40b31c08, 
result_t arg_0x40b31da0, TUniData arg_0x40b31f28);
# 32 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static   uint16_t PhyCC2420M$overflowCount(void);
#line 60
#line 41
typedef enum PhyCC2420M$__nesc_unnamed4403 {

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
struct PhyCC2420M$__nesc_unnamed4404 {
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
uint8_t PhyCC2420M$edContext;
TUniData PhyCC2420M$edUniData;

static inline  void PhyCC2420M$MeasureED(void);
#line 454
static   result_t PhyCC2420M$IPhyED$Request(uint8_t context, 
const TUniData uniData);
#line 473
static inline result_t PhyCC2420M$SetDefaultRegisters(void);




#line 475
enum PhyCC2420M$TPhySetTRXState {
  PhyCC2420M$PHY_TRX_FREE, 
  PhyCC2420M$PHY_TRX_BUSY
} PhyCC2420M$trxState;




 
#line 479
struct PhyCC2420M$__nesc_unnamed4405 {
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
static  result_t PhyCC2420M$IStdControl$init(void);
#line 714
static inline result_t PhyCC2420M$SetDefaultRegisters(void);
#line 737
static  result_t PhyCC2420M$IStdControl$start(void);
#line 776
static inline  TPhyChannel PhyCC2420M$IPhyGET$GetphyCurrentChannel(TPhyStatus *const pPhyStatus);
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


static inline   result_t PhyCC2420M$IPhyED$default$Confirm(uint8_t context, TPhyStatus phyStatus, 
TPhyEnergyLevel phyEnergyLevel, TUniData uniData);


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
static   void PhyPoolM$FreePoolItem(TPhyPoolHandle arg_0x40c21690);
# 19 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   result_t PhyPoolM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b45578);
# 15 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
enum PhyPoolM$__nesc_unnamed4406 {
#line 15
  PhyPoolM$POOL_SIZE = 5
};
typedef TPhyPoolHandle PhyPoolM$TPoolIndex;





#line 19
struct PhyPoolM$__nesc_unnamed4407 {
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
static void PhyPoolM$ResetPool(void);







static inline  void PhyPoolM$IPhyPoolReset$reset(void);





static inline  result_t PhyPoolM$IStdControl$init(void);





static inline  result_t PhyPoolM$IStdControl$start(void);
# 50 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420FIFO.nc"
static   result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t arg_0x40b77118, uint8_t *arg_0x40b772b8);
#line 39
static   result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t arg_0x40b789a8, uint8_t *arg_0x40b78b48);
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
static   result_t HPLCC2420M$USARTControl$tx(uint8_t arg_0x40c54e18);






static   uint8_t HPLCC2420M$USARTControl$rx(void);
#line 185
static   result_t HPLCC2420M$USARTControl$isRxIntrPending(void);
# 49 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420RAM.nc"
static   result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t arg_0x40b98e80, uint8_t arg_0x40b97030, uint8_t *arg_0x40b971d0);
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
struct HPLCC2420M$__nesc_unnamed4408 {
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
static   result_t HPLUSART0M$USARTData$rxDone(uint8_t arg_0x40c97b88);
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
static   void HPLCC2420InterruptM$SFDControl$setControlAsCapture(bool arg_0x40737358);

static   void HPLCC2420InterruptM$SFDControl$enableEvents(void);
static   void HPLCC2420InterruptM$SFDControl$disableEvents(void);
#line 32
static   void HPLCC2420InterruptM$SFDControl$clearPendingInterrupt(void);
# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void HPLCC2420InterruptM$FIFOInterrupt$clear(void);
#line 35
static   void HPLCC2420InterruptM$FIFOInterrupt$disable(void);
#line 54
static   void HPLCC2420InterruptM$FIFOInterrupt$edge(bool arg_0x40cdd418);
#line 30
static   void HPLCC2420InterruptM$FIFOInterrupt$enable(void);
# 51 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
static   result_t HPLCC2420InterruptM$CCA$fired(void);
# 56 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
static   void HPLCC2420InterruptM$SFDCapture$clearOverflow(void);
#line 51
static   bool HPLCC2420InterruptM$SFDCapture$isOverflowPending(void);
# 53 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
static   result_t HPLCC2420InterruptM$SFD$captured(uint16_t arg_0x40b9d360);
# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
static   void HPLCC2420InterruptM$FIFOPInterrupt$clear(void);
#line 35
static   void HPLCC2420InterruptM$FIFOPInterrupt$disable(void);
#line 54
static   void HPLCC2420InterruptM$FIFOPInterrupt$edge(bool arg_0x40cdd418);
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










static inline    void MSP430InterruptM$Port13$default$fired(void);
static inline    void MSP430InterruptM$Port14$default$fired(void);
static inline    void MSP430InterruptM$Port15$default$fired(void);
static inline    void MSP430InterruptM$Port16$default$fired(void);
static inline    void MSP430InterruptM$Port17$default$fired(void);

static inline    void MSP430InterruptM$Port20$default$fired(void);
static inline    void MSP430InterruptM$Port21$default$fired(void);
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
static inline   void MSP430InterruptM$Port11$enable(void);
#line 146
static inline   void MSP430InterruptM$Port10$disable(void);
static inline   void MSP430InterruptM$Port11$disable(void);
static inline   void MSP430InterruptM$Port12$disable(void);
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





static inline   void MSP430InterruptM$Port11$edge(bool l2h);
# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitration.nc"
static  result_t BusArbitrationM$BusArbitration$busFree(
# 31 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/BusArbitrationM.nc"
uint8_t arg_0x40d73a88);





uint8_t BusArbitrationM$state;
uint8_t BusArbitrationM$busid;
bool BusArbitrationM$isBusReleasedPending;
enum BusArbitrationM$__nesc_unnamed4409 {
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
static  result_t MacFrameFormatM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570);
#line 8
static  result_t MacFrameFormatM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90);







static  result_t MacFrameFormatM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x408ff828);
#line 14
static  result_t MacFrameFormatM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328);
#line 12
static  result_t MacFrameFormatM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28);
#line 6
static  TMacAddressMode MacFrameFormatM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8);
# 17 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   result_t MacFrameFormatM$IPhyFrame$ResetPosition(TPhyFrame *const arg_0x40b45068);







static   result_t MacFrameFormatM$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b44528, TPhyTimeStamp arg_0x40b446b0);
#line 19
static   result_t MacFrameFormatM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b45578);
#line 13
static   result_t MacFrameFormatM$IPhyFrame$AppendOctet(TPhyFrame *const arg_0x40b47270, const uint8_t arg_0x40b47410);

static   result_t MacFrameFormatM$IPhyFrame$ReadOctet(TPhyFrame *const arg_0x40b47930, uint8_t *const arg_0x40b47b10);
#line 27
static   result_t MacFrameFormatM$IPhyFrame$GetTimeStamp(const TPhyFrame *const arg_0x40b44bf0, TPhyTimeStamp *const arg_0x40b44dd8);
#line 23
static   result_t MacFrameFormatM$IPhyFrame$CheckCRC(TPhyFrame *const arg_0x40b44010);
#line 21
static   result_t MacFrameFormatM$IPhyFrame$CalcCRC(TPhyFrame *const arg_0x40b45a88);
# 15 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static  result_t MacFrameFormatM$IMacFrame$SetFrameType(TMacFrame *const pMacFrame, const TMacFrameType macFrameType);








static  result_t MacFrameFormatM$IMacFrame$GetFrameType(const TMacFrame *const pMacFrame, TMacFrameType *const pFrameType);








static  result_t MacFrameFormatM$IMacFrame$SetSecurityEnabled(TMacFrame *const pMacFrame, const bool securityEnabled);








static inline  result_t MacFrameFormatM$IMacFrame$GetSecurityEnabled(const TMacFrame *const pMacFrame, 
bool *const pSecurityEnabled);








static  result_t MacFrameFormatM$IMacFrame$SetFramePending(TMacFrame *const pMacFrame, const bool framePending);








static inline  result_t MacFrameFormatM$IMacFrame$GetFramePending(const TMacFrame *const pMacFrame, 
bool *const pFramePending);








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








static  result_t MacFrameFormatM$IMacFrame$GetSrcPANId(const TMacFrame *const pMacFrame, TMacPANId *const pSrcPANId);








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








static  result_t MacFrameFormatM$IMacFrame$GetTimeStamp(const TMacFrame *const pMacFrame, 
TMacTimeStamp *const pTimeStamp);








static  result_t MacFrameFormatM$IMacFrame$GetLinkQuality(const TMacFrame *const pMacFrame, 
TMacLinkQuality *const pLinkQuality);
#line 315
static  result_t MacFrameFormatM$IMacFrame$Pack(const TMacFrame *const pMacFrame, TPhyFrame *const pPhyFrame);
#line 364
static inline  result_t MacFrameFormatM$IMacFrame$UnPack(TMacFrame *const pMacFrame, TPhyFrame *const pPhyFrame);
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
static   uint16_t MacCommonAttrM$IMacRandom$Rand(void);
# 7 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
static  result_t MacCommonAttrM$IPhyAttr$SetextendedAddress(uint64_t arg_0x40b405a0);
#line 5
static  result_t MacCommonAttrM$IPhyAttr$SetpanId(uint16_t arg_0x40b40100);
#line 3
static  result_t MacCommonAttrM$IPhyAttr$SetshortAddress(uint16_t arg_0x40b24c58);
# 23 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
#line 16
struct MacCommonAttrM$__nesc_unnamed4410 {
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






static  TMacAckWaitDuration MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const pMacStatus);
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





static  result_t MacCommonAttrM$IStdControl$init(void);










static inline  result_t MacCommonAttrM$IStdControl$start(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime MacRandomM$ILocalTime$Read(void);
# 11 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacRandomM.nc"
uint16_t MacRandomM$randomValue;

static  result_t MacRandomM$IStdControl$init(void);









static inline  result_t MacRandomM$IStdControl$start(void);


static   uint16_t MacRandomM$IMacRandom$Rand(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbolAsync.nc"
static   result_t MacSuperframesM$Timer$Stop(void);
#line 7
static   result_t MacSuperframesM$Timer$SetOneShotAt(TSysTime arg_0x40a38550, TUniData arg_0x40a386d8);
# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
static   void MacSuperframesM$ParentCAP$BeginCAP(void);

static   void MacSuperframesM$ParentCAP$EndCAP(void);
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   TMilliSec MacSuperframesM$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a3d378);
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacBeaconOrder MacSuperframesM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8);



static  TMacSuperframeOrder MacSuperframesM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e55828);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC_LOSS.nc"
static  void MacSuperframesM$IMacSYNC_LOSS$Indication(
TMacStatus arg_0x4097b5b0);
# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
static   void MacSuperframesM$OwnCAP$BeginCAP(void);

static   void MacSuperframesM$OwnCAP$EndCAP(void);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t MacSuperframesM$IPhyPool$Alloc(const uint8_t arg_0x40b3e6b8, const uint16_t arg_0x40b3e860, 
TPhyPoolHandle *const arg_0x40b3ea58);

static   TPhyFrame *MacSuperframesM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010);



static   result_t MacSuperframesM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b3dbc0);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static   result_t MacSuperframesM$IPhyTxDATA$Request(const TPhyPoolHandle arg_0x40b32f18, 
const TUniData arg_0x40b340f0);
# 23 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static  void MacSuperframesM$packBeacon(TPhyFrame *arg_0x40e7b010, uint64_t *arg_0x40e7b1b8);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime MacSuperframesM$ILocalTime$Read(void);
# 35 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static  void MacSuperframesM$sleepIndication(TMilliSec arg_0x40e79508);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxFIFO.nc"
static   result_t MacSuperframesM$IPhyTxFIFO$Write(TPhyPoolHandle arg_0x40b31130, TUniData arg_0x40b312b8);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  uint64_t MacSuperframesM$ILocalTime64$getLocalTimeAt(TSysTime arg_0x40a42e70);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacSuperframesM$IPhyTrxParent$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8);
#line 6
static   result_t MacSuperframesM$IPhyTrxOwn$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8);
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
 enum MacSuperframesM$__nesc_unnamed4411 {
#line 59
  MacSuperframesM$SF_PARENT, MacSuperframesM$SF_OWN
} 
#line 59
MacSuperframesM$current_sf;

enum MacSuperframesM$__nesc_unnamed4412 {
  MacSuperframesM$PARENT_SF_PREPARATION_TIME = 
  400
   + 250, 
  MacSuperframesM$OWN_SF_PREPARATION_TIME = 400
};


static void MacSuperframesM$calcIntervals(void);
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

static inline  result_t MacSuperframesM$syncParentSF(TSysTime beacon_timestamp);
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
#line 331
static inline  void MacSuperframesM$CoordBeacons$Indication(
TMacBSN bsn, 
TMacPanDescriptor panDescriptor, 
TMacPendAddrSpec pendAddrSpec, 
TMacAddrList addrList, 
uint8_t sduLength, 
uint8_t *sdu);
#line 362
static inline  void MacSuperframesM$createBeacon(void);
#line 380
static void MacSuperframesM$calcIntervals(void);







static inline   result_t MacSuperframesM$Timer$Fired(TUniData unidata);
#line 402
static inline  result_t MacSuperframesM$StdControl$init(void);



static inline  result_t MacSuperframesM$StdControl$start(void);



static  result_t MacSuperframesM$StdControl$stop(void);










static inline  void MacSuperframesM$Reset$reset(void);






static inline  void MacSuperframesM$syncLoss(void);
# 10 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static  void MacSuperframeAttrC$BeaconOrderChanged(uint8_t arg_0x40eb1998);


enum MacSuperframeAttrC$__nesc_unnamed4413 {
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
uint8_t *const arg_0x40ed1680, 
TMacStatus *const arg_0x40ed1880);





static  TMacBSN MacBeaconPackerM$IMacBeaconAttr$GetmacBSN(TMacStatus *const arg_0x40ed0740);
static  TMacStatus MacBeaconPackerM$IMacBeaconAttr$SetmacBSN(TMacBSN arg_0x40ed0bc8);
#line 11
static  uint8_t MacBeaconPackerM$IMacBeaconAttr$GetmacBeaconPayloadLength(
TMacStatus *const arg_0x40ed1178);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacShortAddress MacBeaconPackerM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dd29a8);
#line 6
static  TMacPANId MacBeaconPackerM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0);
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacBeaconOrder MacBeaconPackerM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8);



static  TMacSuperframeOrder MacBeaconPackerM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e55828);
# 27 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacBeaconPackerM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8);
#line 17
static  result_t MacBeaconPackerM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250);
#line 30
static  result_t MacBeaconPackerM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40dac908, const TMacPANId arg_0x40dacab8);





static  result_t MacBeaconPackerM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, 
const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0);
#line 8
static  result_t MacBeaconPackerM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0);
#line 44
static  TMacRawFrameLength MacBeaconPackerM$IMacFrame$Pack(const TMacFrame *const arg_0x40da8c38, TPhyFrame *const arg_0x40da8e28);
#line 11
static  result_t MacBeaconPackerM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8);
#line 33
static  result_t MacBeaconPackerM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0);
#line 14
static  result_t MacBeaconPackerM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480);
#line 5
static  result_t MacBeaconPackerM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98);
#line 20
static  result_t MacBeaconPackerM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030);
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacBeaconPackerM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x408ff828);
#line 14
static  result_t MacBeaconPackerM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328);
#line 12
static  result_t MacBeaconPackerM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28);
# 18 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
static  bool MacBeaconPackerM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40ec2c00);
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
static  bool MacBeaconPackerM$IMacCAPAttr$GetmacBattLifeExt(TMacStatus *const arg_0x40ec9c08);
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
static  result_t MacCAPM$IBackoff$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
# 44 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  TMacRawFrameLength MacCAPM$IMacFrame$Pack(const TMacFrame *const arg_0x40da8c38, TPhyFrame *const arg_0x40da8e28);
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacBeaconOrder MacCAPM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8);



static  TMacSuperframeOrder MacCAPM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e55828);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t MacCAPM$IPhyPool$Alloc(const uint8_t arg_0x40b3e6b8, const uint16_t arg_0x40b3e860, 
TPhyPoolHandle *const arg_0x40b3ea58);

static   TPhyFrame *MacCAPM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010);



static   result_t MacCAPM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b3dbc0);
#line 10
static   result_t MacCAPM$IPhyPool$GetUniData(const TPhyPoolHandle arg_0x40b3d500, uint16_t *const arg_0x40b3d6f0);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyTxDATA.nc"
static   result_t MacCAPM$IPhyTxDATA$Request(const TPhyPoolHandle arg_0x40b32f18, 
const TUniData arg_0x40b340f0);
# 50 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  void MacCAPM$SendDone(const uint8_t arg_0x40f235b0, uint8_t arg_0x40f23730, TMacStatus arg_0x40f238c0, const TUniData arg_0x40f23a68);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime MacCAPM$ILocalTime$Read(void);
# 51 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$Receive(const uint8_t arg_0x40f23f28, TMacFrame *const arg_0x40f22130);
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
static   uint8_t MacCAPM$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b48808);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacCAPM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyCCA.nc"
static   TPhyStatus MacCAPM$IPhyCCA$Request(void);
# 58 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
#line 55
typedef enum MacCAPM$__nesc_unnamed4414 {
  MacCAPM$CAP_STATE_PASSIVE, 
  MacCAPM$CAP_STATE_ACTIVE
} MacCAPM$TMacCAPState;
#line 71
#line 60
typedef enum MacCAPM$__nesc_unnamed4415 {
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

enum MacCAPM$__nesc_unnamed4416 {
  MacCAPM$NUM_SUPERFRAMES = 2U
};






bool MacCAPM$macBattLifeExt = FALSE;
TMacBattLifeExtPeriods MacCAPM$macBattLifeExtPeriods = MAC_BATT_LIFE_EXT_PERIOD_6;
TMacMaxCSMABackoffs MacCAPM$macMaxCSMABackoffs = MAC_MAX_CSMA_BACKOFFS_4;
TMacBackoffExponent MacCAPM$macMinBE = MAC_BACKOFF_EXPONENT_3;
#line 97
 
#line 87
struct MacCAPM$__nesc_unnamed4417 {
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






static  result_t MacCAPM$ICAPControl$init(uint8_t sfIndex);








static  result_t MacCAPM$ICAPControl$start(uint8_t sfIndex);
#line 141
static inline  void MacCAPM$beginSend(void);
#line 157
static   void MacCAPM$IMacCAP$BeginCAP(uint8_t sfIndex);
#line 170
static inline  void MacCAPM$TurnOffTask(void);




static   void MacCAPM$IMacCAP$EndCAP(uint8_t sfIndex);
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
#line 635
static inline  void MacCAPM$ResetAll$reset(void);





static inline  void MacCAPM$Reset$reset(uint8_t sfIndex);







static  result_t MacCAPM$ICAPControl$stop(uint8_t sfIndex);
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
static  result_t MacRxM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200);
#line 45
static  result_t MacRxM$IMacFrame$UnPack(TMacFrame *const arg_0x40da63a0, TPhyFrame *const arg_0x40da6590);
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   TPhyFrame *MacRxM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t MacRxM$DataReceive$Receive(TMacFrame *const arg_0x40f024f8);
#line 5
static  result_t MacRxM$BeaconReceive$Receive(TMacFrame *const arg_0x40f024f8);
# 21 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacRxM.nc"
static inline  result_t MacRxM$IPhyRxDATA$Indication(const TPhyPoolHandle handle);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendDone(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f94168, 
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 16 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Send(const uint8_t arg_0x40f93c88, 
const TMacFrame *const arg_0x40f93ea8, 
const uint8_t arg_0x40f92088, 
const TUniData arg_0x40f92240);

static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendFromPhyPool(const uint8_t arg_0x40f92708, 
const TPhyPoolHandle arg_0x40f928c8, 
const uint8_t arg_0x40f92a78);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacReceive$Receive(TMacFrame *const arg_0x40f024f8);
# 30 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static inline  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(uint8_t context, const TMacFrame *const pMacFrame, 
const TUniData uniData);





static inline  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendFromPhyPool(uint8_t context, const TPhyPoolHandle frameHandle);




static inline  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendDone(const uint8_t _sfIndex, uint8_t context, 
TMacStatus macStatus, const TUniData uniData);





static inline  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Receive(const uint8_t _sfIndex, TMacFrame *const pMacFrame);






static inline   void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$default$SendDone(uint8_t context, TMacStatus macStatus, TUniData unidata);
# 9 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  void MacPoolM$IMacPool$FreeDone(const TMacPoolHandle arg_0x40ef1358, const TMacStatus arg_0x40ef1500);
# 28 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacPoolM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40dac218, TMacAddress *const arg_0x40dac408);
# 18 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  bool MacPoolM$IMacAddress$Equal(const TMacAddress *const arg_0x408ffd30, const TMacAddress *const arg_0x408fd010);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacPoolM$IExpiredTimer$Stop(void);
#line 7
static  result_t MacPoolM$IExpiredTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);



static  bool MacPoolM$IExpiredTimer$IsSet(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime MacPoolM$ILocalTime$Read(void);
# 25 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPoolM.nc"
enum MacPoolM$__nesc_unnamed4418 {
#line 25
  MacPoolM$MAC_POOL_SIZE = 3
};
typedef uint16_t MacPoolM$TMacPoolIndex;





#line 29
typedef enum MacPoolM$__nesc_unnamed4419 {
  MacPoolM$MAC_POOL_ITEM_FREE, 
  MacPoolM$MAC_POOL_ITEM_FREE_DONE, 
  MacPoolM$MAC_POOL_ITEM_ALLOC
} MacPoolM$TMacPoolItemState;







#line 35
struct MacPoolM$__nesc_unnamed4420 {
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
static  result_t MacPendingM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200);
#line 42
static  result_t MacPendingM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718);
#line 39
static  result_t MacPendingM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, 
TMacPayloadLength *const arg_0x40da8010);
#line 15
static  result_t MacPendingM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40db29a0, bool *const arg_0x40db2b88);
#line 34
static  result_t MacPendingM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010);
# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
static  TMacFrame *MacPendingM$IMacPool$GetFrame(const TMacPoolHandle arg_0x40ef19c0);
#line 8
static  result_t MacPendingM$IMacPool$Free(const TMacPoolHandle arg_0x40ef2c68, const TMacStatus arg_0x40ef2e10);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacPendingM$IMacCSMA$Send(const TMacFrame *const arg_0x40f051f8, 
const TUniData arg_0x40f053b0);
# 14 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacPendingM.nc"
static  bool MacPendingM$SearchDstAddress(const TMacAddress arg_0x40fbf5f0, 
TMacPoolHandle *const arg_0x40fbf7f0);



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

static inline  bool MacBeaconAttrCoordM$IMacBeaconAttr$GetmacAutoRequest(TMacStatus *const pMacStatus);
#line 101
static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacAutoRequest(void);







TMacBSN MacBeaconAttrCoordM$macBSN;

static inline  TMacBSN MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBSN(TMacStatus *const pMacStatus);





static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBSN(TMacBSN _macBSN);




static inline  TMacStatus MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBSN(void);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacAssocCoordM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570);



static  result_t MacAssocCoordM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacAssocCoordM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200);
#line 27
static  result_t MacAssocCoordM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8);
#line 17
static  result_t MacAssocCoordM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250);
#line 42
static  result_t MacAssocCoordM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718);
#line 30
static  result_t MacAssocCoordM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40dac908, const TMacPANId arg_0x40dacab8);








static  result_t MacAssocCoordM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, 
TMacPayloadLength *const arg_0x40da8010);
#line 36
static  result_t MacAssocCoordM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, 
const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0);
#line 8
static  result_t MacAssocCoordM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0);


static  result_t MacAssocCoordM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8);
#line 33
static  result_t MacAssocCoordM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0);
#line 24
static  result_t MacAssocCoordM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40dafc60, const TMacPANId arg_0x40dafe10);
#line 9
static  result_t MacAssocCoordM$IMacFrame$GetSecurityEnabled(const TMacFrame *const arg_0x40db5dd8, bool *const arg_0x40db4010);




static  result_t MacAssocCoordM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480);
#line 5
static  result_t MacAssocCoordM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98);
#line 34
static  result_t MacAssocCoordM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010);
#line 20
static  result_t MacAssocCoordM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030);
# 38 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t MacAssocCoordM$IMacASSOCIATE$Indication(TMacExtendedAddress arg_0x408cb068, 
TCapabilityInformation arg_0x408cb218, 
bool arg_0x408cb3b0, 
TACLEntry arg_0x408cb550);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacPANId MacAssocCoordM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0);
#line 18
static  TMacDSN MacAssocCoordM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dcf3a0);
static  TMacStatus MacAssocCoordM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dcf828);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCOMM_STATUS.nc"
static  void MacAssocCoordM$IMacCOMM_STATUS$Indication(TMacPANId arg_0x408c8400, 
TMacAddress arg_0x408c85a0, TMacAddress arg_0x408c8730, 
TMacStatus arg_0x408c88d0);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacAssocCoordM$IMacSendOwn$Send(const TMacFrame *const arg_0x40f051f8, 
const TUniData arg_0x40f053b0);
# 31 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocCoordM.nc"
enum MacAssocCoordM$__nesc_unnamed4421 {
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
typedef enum MacAssocCoordM$__nesc_unnamed4422 {
  MacAssocCoordM$MAC_DISASSOCIATE_OFF, 
  MacAssocCoordM$MAC_DISASSOCIATE_IDLE, 
  MacAssocCoordM$MAC_DISASSOCIATE_REQUEST, 
  MacAssocCoordM$MAC_DISASSOCIATE_WAIT_ACK
} MacAssocCoordM$TMacDisassocState;




#line 201
struct MacAssocCoordM$__nesc_unnamed4423 {
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

static inline  void MacBeaconCommonM$setLastBSN(uint8_t bsn);




static inline  uint8_t MacBeaconCommonM$getLastBSN(void);
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacStatus MacStartM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dd2198);


static  TMacShortAddress MacStartM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dd29a8);
# 11 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacStatus MacStartM$IMacSuperframeAttr$SetmacSuperframeOrder(TMacSuperframeOrder arg_0x40e55cd8);
#line 7
static  TMacStatus MacStartM$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e55010);
# 11 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
static  result_t MacStartM$IPhyAttr$SetAutoAck(bool arg_0x40b40ec0);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
static  TPhyStatus MacStartM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b2a360);
# 20 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacStartM.nc"
static  result_t MacStartM$runOwnSF(TSysTime arg_0x41063ce8);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacStartM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8);
# 22 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
static  result_t MacStartM$IMacSTART$Confirm(TMacStatus arg_0x40b13338);
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
static  TMacDSN MacDataM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dcf3a0);
#line 14
static  TMacAckWaitDuration MacDataM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dd16c8);




static  TMacStatus MacDataM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dcf828);
# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t MacDataM$IMacDataOwn$Confirm(uint8_t arg_0x409e8348, TMacStatus arg_0x409e84d8);
#line 16
static  result_t MacDataM$IMacDataOwn$Indication(TMacPANId arg_0x409e9178, 
TMacAddress arg_0x409e9310, 
TMacPANId arg_0x409e94b0, 
TMacAddress arg_0x409e9648, 
uint8_t arg_0x409e97e0, 
uint8_t *arg_0x409e9998, 
TMacLinkQuality arg_0x409e9b38, 
bool arg_0x409e9cd0, 
uint8_t arg_0x409e9e68);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacDataM$AckWaitTimer$Stop(void);
#line 7
static  result_t MacDataM$AckWaitTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacDataM$IMacSendParent$Send(const TMacFrame *const arg_0x40f051f8, 
const TUniData arg_0x40f053b0);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacDataM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200);
#line 27
static  result_t MacDataM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8);
#line 17
static  result_t MacDataM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250);
#line 42
static  result_t MacDataM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718);
#line 30
static  result_t MacDataM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40dac908, const TMacPANId arg_0x40dacab8);








static  result_t MacDataM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, 
TMacPayloadLength *const arg_0x40da8010);
#line 36
static  result_t MacDataM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, 
const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0);
#line 15
static  result_t MacDataM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40db29a0, bool *const arg_0x40db2b88);
#line 8
static  result_t MacDataM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0);
#line 31
static  result_t MacDataM$IMacFrame$GetSrcPANId(const TMacFrame *const arg_0x40dab010, TMacPANId *const arg_0x40dab200);
#line 11
static  result_t MacDataM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8);









static  result_t MacDataM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40daf558, 
TMacSequenceNumber *const arg_0x40daf760);










static  result_t MacDataM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0);
#line 24
static  result_t MacDataM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40dafc60, const TMacPANId arg_0x40dafe10);



static  result_t MacDataM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40dac218, TMacAddress *const arg_0x40dac408);
#line 14
static  result_t MacDataM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480);
#line 5
static  result_t MacDataM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98);
#line 50
static  result_t MacDataM$IMacFrame$GetLinkQuality(const TMacFrame *const arg_0x40da5970, TMacLinkQuality *const arg_0x40da5b60);
#line 34
static  result_t MacDataM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010);
#line 20
static  result_t MacDataM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030);




static  result_t MacDataM$IMacFrame$GetDstPANId(const TMacFrame *const arg_0x40dae448, TMacPANId *const arg_0x40dae638);
# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t MacDataM$IMacDataParent$Confirm(uint8_t arg_0x409e8348, TMacStatus arg_0x409e84d8);
#line 16
static  result_t MacDataM$IMacDataParent$Indication(TMacPANId arg_0x409e9178, 
TMacAddress arg_0x409e9310, 
TMacPANId arg_0x409e94b0, 
TMacAddress arg_0x409e9648, 
uint8_t arg_0x409e97e0, 
uint8_t *arg_0x409e9998, 
TMacLinkQuality arg_0x409e9b38, 
bool arg_0x409e9cd0, 
uint8_t arg_0x409e9e68);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacDataM$IMacSendOwn$Send(const TMacFrame *const arg_0x40f051f8, 
const TUniData arg_0x40f053b0);
# 34 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
bool MacDataM$st_waitAck = FALSE;

enum MacDataM$__nesc_unnamed4424 {
#line 36
  MacDataM$IDLE, MacDataM$WAIT_SEND_DONE, MacDataM$WAIT_ACK, MacDataM$OFF
} 
#line 36
MacDataM$state = MacDataM$IDLE;

typedef enum MacDataM$__nesc_unnamed4425 {
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



static result_t MacDataM$DATA_Request(MacDataM$SF sf, 
TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
const uint8_t *const pmsduFrame, 
uint8_t msduHandle, 
TxOptions bmTxOpt);
#line 120
static void MacDataM$SendDone(TMacStatus status, const TUniData unidata);
#line 148
static inline  result_t MacDataM$AckWaitTimer$Fired(TUniData uniData);
#line 180
static result_t MacDataM$Receive(MacDataM$SF sf, TMacFrame *const pmacFrame);
#line 267
static result_t MacDataM$Confirm(uint8_t sf, uint8_t msduHandle, TMacStatus status);








static result_t MacDataM$Send(const TMacFrame *const frame, const TUniData unidata);








static inline  result_t MacDataM$IMacDataParent$Request(
TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
const uint8_t *const pmsduFrame, 
uint8_t msduHandle, 
TxOptions bmTxOpt);
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
#line 331
static inline  void MacDataM$IMacSendParent$SendDone(TMacStatus status, const TUniData unidata);





static inline  void MacDataM$IMacSendOwn$SendDone(TMacStatus status, const TUniData unidata);






static inline  result_t MacDataM$IMacReceiveParent$Receive(TMacFrame *const pmacFrame);




static inline  result_t MacDataM$IMacReceiveOwn$Receive(TMacFrame *const pmacFrame);










static inline  result_t MacDataM$StdControl$start(void);






static inline  result_t MacDataM$StdControl$init(void);



static inline  result_t MacDataM$StdControl$stop(void);






static inline  void MacDataM$Reset$reset(void);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacPANId MacScanBeaconM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacScanBeaconM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200);
#line 42
static  result_t MacScanBeaconM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718);
#line 39
static  result_t MacScanBeaconM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, 
TMacPayloadLength *const arg_0x40da8010);
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacBeaconOrder MacScanBeaconM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
static   result_t MacScanBeaconM$IPhyPool$Alloc(const uint8_t arg_0x40b3e6b8, const uint16_t arg_0x40b3e860, 
TPhyPoolHandle *const arg_0x40b3ea58);

static   TPhyFrame *MacScanBeaconM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010);



static   result_t MacScanBeaconM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b3dbc0);
# 17 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanBeaconM.nc"
static  void MacScanBeaconM$packBeacon(TPhyFrame *arg_0x41088848, uint64_t *arg_0x410889f0);
# 10 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacScanBeaconM$CSMA$SendFromPhyPool(const TPhyPoolHandle arg_0x40f05868);
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
uint8_t *arg_0x40ed1d78, 
uint8_t arg_0x40ed1f18);
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

static  TMacShortAddress MacCoordAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dd29a8);

static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void);
#line 24
static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void);



static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void);
#line 6
static  TMacPANId MacCoordAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0);
#line 20
static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacDSN(void);
#line 32
static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void);
# 12 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacStatus MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void);
#line 8
static  TMacStatus MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void);
#line 6
static  TMacBeaconOrder MacCoordAttrM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8);



static  TMacSuperframeOrder MacCoordAttrM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e55828);
# 10 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPoolAttr.nc"
static  TMacStatus MacCoordAttrM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void);
# 10 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void);







static  bool MacCoordAttrM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40ec2c00);
static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetmacAssociationPermit(bool arg_0x40ec10c0);
static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacAssociationPermit(void);
#line 16
static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacCoordAttrM$IMacReset$reset(void);
# 20 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs(void);
#line 8
static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacBattLifeExt(void);





static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods(void);
#line 26
static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacMinBE(void);
# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t MacCoordAttrM$IMacRESET$Confirm(TMacStatus arg_0x409c6ac8);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacCoordAttrM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8);
# 2 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacPANId MacCoordAttrM$IMacGET$GetmacPANId(TMacStatus *const pMacStatus);









static inline  TMacShortAddress MacCoordAttrM$IMacGET$GetmacShortAddress(TMacStatus *const pMacStatus);
#line 80
static inline  TMacBeaconOrder MacCoordAttrM$IMacGET$GetmacBeaconOrder(
TMacStatus *const pMacStatus);










static inline  TMacSuperframeOrder MacCoordAttrM$IMacGET$GetmacSuperframeOrder(
TMacStatus *const pMacStatus);
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




static  result_t MacCoordAttrM$IMacRESET$Request(bool setDefaultPIB);









static inline  result_t MacCoordAttrM$IPhySET_TRX_STATE$Confirm(TPhyStatus phyStatus, TUniData uniData);
# 7 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
static  bool MacBeaconNotifierM$IMacBeaconAttr$GetmacAutoRequest(TMacStatus *const arg_0x40ed34a8);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacPANId MacBeaconNotifierM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacBeaconNotifierM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570);
#line 8
static  result_t MacBeaconNotifierM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90);
#line 6
static  TMacAddressMode MacBeaconNotifierM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8);
# 42 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacBeaconNotifierM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718);
#line 39
static  result_t MacBeaconNotifierM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, 
TMacPayloadLength *const arg_0x40da8010);
#line 31
static  result_t MacBeaconNotifierM$IMacFrame$GetSrcPANId(const TMacFrame *const arg_0x40dab010, TMacPANId *const arg_0x40dab200);
#line 48
static  result_t MacBeaconNotifierM$IMacFrame$GetTimeStamp(const TMacFrame *const arg_0x40da5260, TMacTimeStamp *const arg_0x40da5450);
#line 21
static  result_t MacBeaconNotifierM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40daf558, 
TMacSequenceNumber *const arg_0x40daf760);
#line 50
static  result_t MacBeaconNotifierM$IMacFrame$GetLinkQuality(const TMacFrame *const arg_0x40da5970, TMacLinkQuality *const arg_0x40da5b60);
#line 34
static  result_t MacBeaconNotifierM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBEACON_NOTIFY.nc"
static  void MacBeaconNotifierM$CoordBeacons$Indication(
TMacBSN arg_0x4099e708, 
TMacPanDescriptor arg_0x4099e8b0, 
TMacPendAddrSpec arg_0x4099ea58, 
uint8_t *arg_0x4099ec10, 
uint8_t arg_0x4099eda8, 
uint8_t *arg_0x4099d010);
# 6 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
static  TMacExtendedAddress MacBeaconNotifierM$IMacAssocAttr$GetmacCoordExtendedAddress(
TMacStatus *const arg_0x40ec4208);




static  TMacShortAddress MacBeaconNotifierM$IMacAssocAttr$GetmacCoordShortAddress(
TMacStatus *const arg_0x40ec4f00);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyGET.nc"
static  TPhyChannel MacBeaconNotifierM$IPhyGET$GetphyCurrentChannel(TPhyStatus *const arg_0x40b33d28);
# 27 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconNotifierM.nc"
static  void MacBeaconNotifierM$setLastBSN(uint8_t arg_0x4113d068);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBEACON_NOTIFY.nc"
static  void MacBeaconNotifierM$AllBeacons$Indication(
TMacBSN arg_0x4099e708, 
TMacPanDescriptor arg_0x4099e8b0, 
TMacPendAddrSpec arg_0x4099ea58, 
uint8_t *arg_0x4099ec10, 
uint8_t arg_0x4099eda8, 
uint8_t *arg_0x4099d010);
#line 6
static  void MacBeaconNotifierM$ToHigher$Indication(
TMacBSN arg_0x4099e708, 
TMacPanDescriptor arg_0x4099e8b0, 
TMacPendAddrSpec arg_0x4099ea58, 
uint8_t *arg_0x4099ec10, 
uint8_t arg_0x4099eda8, 
uint8_t *arg_0x4099d010);
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  void MacBeaconNotifierM$ILocalTime64$setLocalTimeAt(TSysTime arg_0x40a41330, uint64_t arg_0x40a414b8);
# 51 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconNotifierM.nc"
static inline bool MacBeaconNotifierM$isFromCoord(const TMacPanDescriptor *const pan_d);
#line 112
static inline  result_t MacBeaconNotifierM$BeaconReceive$Receive(TMacFrame *const pMacFrame);
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  void /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$SendDone(
# 6 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
uint8_t arg_0x40f94168, 
# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98);
# 16 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$Send(const uint8_t arg_0x40f93c88, 
const TMacFrame *const arg_0x40f93ea8, 
const uint8_t arg_0x40f92088, 
const TUniData arg_0x40f92240);
# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
static  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacReceive$Receive(TMacFrame *const arg_0x40f024f8);
# 30 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static inline  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$Send(uint8_t context, const TMacFrame *const pMacFrame, 
const TUniData uniData);










static inline  void /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$SendDone(const uint8_t _sfIndex, uint8_t context, 
TMacStatus macStatus, const TUniData uniData);





static inline  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$Receive(const uint8_t _sfIndex, TMacFrame *const pMacFrame);






static inline   void /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$default$SendDone(uint8_t context, TMacStatus macStatus, TUniData unidata);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacExtractM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200);
#line 21
static  result_t MacExtractM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40daf558, 
TMacSequenceNumber *const arg_0x40daf760);
#line 12
static  result_t MacExtractM$IMacFrame$GetFramePending(const TMacFrame *const arg_0x40db4bd8, bool *const arg_0x40db4dc0);
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacAckWaitDuration MacExtractM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dd16c8);
# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacExtract.nc"
static  void MacExtractM$IMacExtract$End(
# 11 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
uint8_t arg_0x411427b8, 
# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacExtract.nc"
TMacStatus arg_0x41149550, bool arg_0x411496d8, TUniData arg_0x41149860);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacExtractM$IAckWaitTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
# 29 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
#line 24
typedef enum MacExtractM$__nesc_unnamed4426 {
  MacExtractM$MAC_EXTRACT_OFF, 
  MacExtractM$MAC_EXTRACT_IDLE, 
  MacExtractM$MAC_EXTRACT_SEND, 
  MacExtractM$MAC_EXTRACT_WAIT_ACK
} MacExtractM$TMacExtractState;
MacExtractM$TMacExtractState MacExtractM$extractState;





#line 32
struct MacExtractM$__nesc_unnamed4427 {
  TMacSequenceNumber seqNum;
  TUniData uniData;
  uint8_t context;
} MacExtractM$extract;
#line 114
static void MacExtractM$ExtractDone(TMacStatus macStatus, bool framePending);






static inline  void MacExtractM$IMacCSMA$SendDone(TMacStatus macStatus, const TUniData uniData);
#line 143
static inline  result_t MacExtractM$IAckWaitTimer$Fired(TUniData uniData);








static inline  result_t MacExtractM$IMacReceive$Receive(TMacFrame *const pMacFrame);
#line 175
static inline  result_t MacExtractM$IStdControl$init(void);





static inline  result_t MacExtractM$IStdControl$start(void);
#line 193
static inline  void MacExtractM$Reset$reset(void);
#line 211
static inline  TMacStatus MacExtractM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void);




static inline   void MacExtractM$IMacExtract$default$End(uint8_t context, TMacStatus status, 
bool framePending, TUniData uniData);
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacStatus MacAssocDeviceM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dd2198);










static  TMacDSN MacAssocDeviceM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dcf3a0);
#line 14
static  TMacAckWaitDuration MacAssocDeviceM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dd16c8);




static  TMacStatus MacAssocDeviceM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dcf828);
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacAssocDeviceM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570);
#line 8
static  result_t MacAssocDeviceM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90);





static  result_t MacAssocDeviceM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328);
#line 6
static  TMacAddressMode MacAssocDeviceM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8);
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacBeaconOrder MacAssocDeviceM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8);
# 63 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
static  result_t MacAssocDeviceM$IMacASSOCIATE$Confirm(TMacShortAddress arg_0x408ca888, 
TMacStatus arg_0x408caa20);
# 11 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
static  result_t MacAssocDeviceM$IPhyAttr$SetAutoAck(bool arg_0x40b40ec0);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacAssocDeviceM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200);
#line 27
static  result_t MacAssocDeviceM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8);
#line 17
static  result_t MacAssocDeviceM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250);
#line 42
static  result_t MacAssocDeviceM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718);
#line 39
static  result_t MacAssocDeviceM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, 
TMacPayloadLength *const arg_0x40da8010);
#line 36
static  result_t MacAssocDeviceM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, 
const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0);
#line 8
static  result_t MacAssocDeviceM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0);


static  result_t MacAssocDeviceM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8);









static  result_t MacAssocDeviceM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40daf558, 
TMacSequenceNumber *const arg_0x40daf760);










static  result_t MacAssocDeviceM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0);
#line 24
static  result_t MacAssocDeviceM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40dafc60, const TMacPANId arg_0x40dafe10);
#line 14
static  result_t MacAssocDeviceM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480);
#line 5
static  result_t MacAssocDeviceM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98);
#line 34
static  result_t MacAssocDeviceM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010);
#line 20
static  result_t MacAssocDeviceM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacAssocDeviceM$IAssocRequest$Send(const TMacFrame *const arg_0x40f051f8, 
const TUniData arg_0x40f053b0);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacAssocDeviceM$IAssocAckTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
static  TPhyStatus MacAssocDeviceM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b2a360);
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacAssocDeviceM$IReceiveResponseTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
# 39 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
enum MacAssocDeviceM$__nesc_unnamed4428 {
  MacAssocDeviceM$ASSOC_REQUEST_PAYLOAD_LEN = 2, 
  MacAssocDeviceM$ASSOC_RESPONSE_PAYLOAD_LEN = 4, 
  MacAssocDeviceM$DISASSOC_NOTIF_PAYLOAD_LEN = 2
};
#line 56
#line 45
typedef enum MacAssocDeviceM$__nesc_unnamed4429 {
  MacAssocDeviceM$MAC_ASSOC_OFF, 
  MacAssocDeviceM$MAC_ASSOC_IDLE, 
  MacAssocDeviceM$MAC_ASSOC_REQUEST, 
  MacAssocDeviceM$MAC_ASSOC_WAIT_ACK, 
  MacAssocDeviceM$MAC_ASSOC_WAIT_RESPONSE, 
  MacAssocDeviceM$MAC_ASSOC_EXTRACT, 
  MacAssocDeviceM$MAC_ASSOC_RECEIVE_RESPONSE, 
  MacAssocDeviceM$MAC_DISASSOC_REQUEST, 
  MacAssocDeviceM$MAC_DISASSOC_WAIT_ACK, 
  MacAssocDeviceM$MAC_DISASSOC_INDICATION_ACK
} MacAssocDeviceM$TMacAssocState;
#line 69
#line 58
struct MacAssocDeviceM$__nesc_unnamed4430 {
  MacAssocDeviceM$TMacAssocState state;
  bool securityEnable;
  TMacPANId coordPANId;
  TMacAddress coordAddress;
  TMacSequenceNumber seqNum;
  TCapabilityInformation capabilityInformation;
  TDisassociateReason disassociateReason;

  TMacExtendedAddress macCoordExtendedAddress;
  TMacShortAddress macCoordShortAddress;
} MacAssocDeviceM$assoc;

static inline  TMacExtendedAddress MacAssocDeviceM$IMacAssocAttr$GetmacCoordExtendedAddress(
TMacStatus *const pMacStatus);





static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetmacCoordExtendedAddress(
TMacExtendedAddress macCoordExtendedAddress);




static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void);





static  TMacShortAddress MacAssocDeviceM$IMacAssocAttr$GetmacCoordShortAddress(
TMacStatus *const pMacStatus);





static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetmacCoordShortAddress(
TMacShortAddress macCoordShortAddress);




static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void);





static inline  result_t MacAssocDeviceM$IStdControl$init(void);








static inline  result_t MacAssocDeviceM$IStdControl$start(void);
#line 130
static inline  void MacAssocDeviceM$Reset$reset(void);






static result_t MacAssocDeviceM$MakeAssocReqFrame(const TMacPANId coordPANId, 
const TMacAddress coordAddress, 
const TCapabilityInformation capabilityInformation, 
const bool securityEnable, 
TMacFrame *const pMacFrame);
#line 177
static inline  result_t MacAssocDeviceM$IMacASSOCIATE$Request(TMacLogicalChannel logicalChannel, 
TMacPANId coordPANId, TMacAddress coordAddress, 
TCapabilityInformation capabilityInformation, 
bool securityEnable);
#line 210
static inline void MacAssocDeviceM$AssocSuccessDone(TMacShortAddress assocShortAddress, TMacStatus macStatus);






static inline void MacAssocDeviceM$AssocFailDone(TMacStatus macStatus);





static inline  void MacAssocDeviceM$IAssocRequest$SendDone(TMacStatus macStatus, const TUniData uniData);
#line 249
static inline  result_t MacAssocDeviceM$IAssocAckTimer$Fired(TUniData uniData);
#line 274
static inline  result_t MacAssocDeviceM$IAssocAck$Receive(TMacFrame *const pMacFrame);
#line 308
static inline  result_t MacAssocDeviceM$IAssocResponse$Receive(TMacFrame *const pMacFrame);
#line 353
static inline  result_t MacAssocDeviceM$IReceiveResponseTimer$Fired(TUniData uniData);
#line 567
static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetDefaultmacAssociationPermit(void);
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC_LOSS.nc"
static  void MacSyncM$IMacSYNC_LOSS$Indication(
TMacStatus arg_0x4097b5b0);
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacPANId MacSyncM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacSyncM$CAPReset$reset(void);
# 11 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacStatus MacSyncM$IMacSuperframeAttr$SetmacSuperframeOrder(TMacSuperframeOrder arg_0x40e55cd8);
#line 6
static  TMacBeaconOrder MacSyncM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8);
static  TMacStatus MacSyncM$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e55010);
# 22 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
static  result_t MacSyncM$syncParentSF(TSysTime arg_0x411fd868);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacSyncM$InitialTimer$Stop(void);
#line 7
static  result_t MacSyncM$InitialTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
static  TPhyStatus MacSyncM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b2a360);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacSyncM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacSyncM$SuperframesReset$reset(void);
# 30 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
bool MacSyncM$first_radio_on = FALSE;
bool MacSyncM$beacon_expected = FALSE;




static inline  void MacSyncM$continueSync(void);


static inline  result_t MacSyncM$IMacSYNC$Request(TMacLogicalChannel logicalChannel, 
bool trackBeacon);
#line 58
static inline  void MacSyncM$continueSync(void);
#line 71
static inline  result_t MacSyncM$IPhySET_TRX_STATE$Confirm(TPhyStatus status, TUniData unidata);
#line 85
static inline  result_t MacSyncM$InitialTimer$Fired(TUniData uniData);










static inline  void MacSyncM$CoordBeacons$Indication(
TMacBSN bsn, 
TMacPanDescriptor panDescriptor, 
TMacPendAddrSpec pendAddrSpec, 
TMacAddrList addrList, 
uint8_t sduLength, 
uint8_t *sdu);
#line 120
static inline  void MacSyncM$Reset$reset(void);





static inline  result_t MacSyncM$StdControl$start(void);




static inline  result_t MacSyncM$StdControl$init(void);




static inline  result_t MacSyncM$StdControl$stop(void);
# 11 "../../../zigzag/IEEE802_15_4/MacScan/public/IMacSCAN.nc"
static  result_t MacScanM$IMacSCAN$Confirm(TMacStatus arg_0x4091f968, 
TMacScanType arg_0x4091fb08, 
uint32_t arg_0x4091fcb0, 
uint8_t arg_0x4091fe48, 
uint8_t *arg_0x4091d030, 
TMacPanDescriptor *arg_0x4091d200);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacScanM$TimerED$Stop(void);
#line 7
static  result_t MacScanM$TimerED$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);

static  result_t MacScanM$TimerBeaconScan$Stop(void);
#line 7
static  result_t MacScanM$TimerBeaconScan$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8);
# 18 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacDSN MacScanM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dcf3a0);
static  TMacStatus MacScanM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dcf828);
# 27 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
static  result_t MacScanM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8);
#line 17
static  result_t MacScanM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250);
#line 36
static  result_t MacScanM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, 
const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0);
#line 8
static  result_t MacScanM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0);


static  result_t MacScanM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8);
#line 33
static  result_t MacScanM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0);
#line 24
static  result_t MacScanM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40dafc60, const TMacPANId arg_0x40dafe10);
#line 14
static  result_t MacScanM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480);
#line 5
static  result_t MacScanM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98);
#line 20
static  result_t MacScanM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030);
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t MacScanM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x408ff828);
#line 12
static  result_t MacScanM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyED.nc"
static   result_t MacScanM$IPhyED$Request(const TUniData arg_0x40b2dd08);
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
static  TPhyStatus MacScanM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b2a360);
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
static  result_t MacScanM$IMacCSMA$Send(const TMacFrame *const arg_0x40f051f8, 
const TUniData arg_0x40f053b0);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacScanM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8);
# 32 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanM.nc"
uint8_t MacScanM$CurBit;
#line 32
uint8_t MacScanM$BcnCount;
uint8_t MacScanM$energyDetectList[27];

TMacPanDescriptor *MacScanM$panDescriptorList;
uint32_t MacScanM$ScanTime;
uint32_t MacScanM$ScanChannels;
bool MacScanM$bTrOn;
bool MacScanM$TimIsSet;
TMacScanType MacScanM$eMode;



static void MacScanM$nextBit(void);






static inline  result_t MacScanM$IMacSCAN$Request(TMacScanType scanType, 
uint32_t scanChannels, 
uint8_t scanDuration);
#line 90
static void MacScanM$SendFrame(void);
#line 119
static inline  result_t MacScanM$IPhySET_TRX_STATE$Confirm(TPhyStatus status, TUniData uniData);
#line 157
static inline  void MacScanM$AllBeacons$Indication(TMacBSN bsn, 
TMacPanDescriptor panDescriptor, 
TMacPendAddrSpec pendAddrSpec, 
uint8_t *addrList, 
uint8_t sduLength, 
uint8_t *sdu);
#line 181
static inline  result_t MacScanM$TimerBeaconScan$Fired(TUniData uniData);
#line 215
static  result_t MacScanM$IPhyED$Confirm(TPhyStatus phyStatus, TPhyEnergyLevel phyEnergyLevel, TUniData uniData);
#line 252
static inline  result_t MacScanM$TimerED$Fired(TUniData uniData);
#line 273
static inline  void MacScanM$IMacCSMA$SendDone(TMacStatus status, const TUniData unidata);
# 19 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
static  TMacStatus MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacBeaconPayload(void);









static  TMacStatus MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacBeaconTxTime(void);
#line 23
static  TMacStatus MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacBSN(void);
#line 9
static  TMacStatus MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacAutoRequest(void);
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void);
#line 8
static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacPANId(void);
#line 7
static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dd2198);


static  TMacShortAddress MacDeviceAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dd29a8);

static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void);
#line 24
static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void);



static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void);


static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(bool arg_0x40dcdf00);
#line 6
static  TMacPANId MacDeviceAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0);




static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetmacShortAddress(TMacShortAddress arg_0x40dd2e50);








static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacDSN(void);
#line 32
static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void);
# 12 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
static  TMacStatus MacDeviceAttrM$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void);
#line 8
static  TMacStatus MacDeviceAttrM$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void);
#line 7
static  TMacStatus MacDeviceAttrM$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e55010);
# 10 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPoolAttr.nc"
static  TMacStatus MacDeviceAttrM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void);
# 10 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void);
#line 8
static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetmacCoordExtendedAddress(
TMacExtendedAddress arg_0x40ec46d0);


static  TMacShortAddress MacDeviceAttrM$IMacAssocAttr$GetmacCoordShortAddress(
TMacStatus *const arg_0x40ec4f00);






static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetDefaultmacAssociationPermit(void);
#line 14
static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetmacCoordShortAddress(
TMacShortAddress arg_0x40ec23f8);
static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void);
# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
static  void MacDeviceAttrM$IMacReset$reset(void);
# 20 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
static  TMacStatus MacDeviceAttrM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs(void);
#line 8
static  TMacStatus MacDeviceAttrM$IMacCAPAttr$SetDefaultmacBattLifeExt(void);





static  TMacStatus MacDeviceAttrM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods(void);
#line 26
static  TMacStatus MacDeviceAttrM$IMacCAPAttr$SetDefaultmacMinBE(void);
# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
static  result_t MacDeviceAttrM$IMacRESET$Confirm(TMacStatus arg_0x409c6ac8);
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
static   result_t MacDeviceAttrM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8);
# 2 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacPANId MacDeviceAttrM$IMacGET$GetmacPANId(TMacStatus *const pMacStatus);



static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacPANId(TMacPANId macPANId);





static inline  TMacShortAddress MacDeviceAttrM$IMacGET$GetmacShortAddress(TMacStatus *const pMacStatus);



static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacShortAddress(TMacShortAddress macShortAddress);
#line 85
static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacBeaconOrder(
TMacBeaconOrder macBeaconOrder);
#line 155
static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacRxOnWhenIdle(
bool macRxOnWhenIdle);
#line 230
static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacCoordExtendedAddress(
TMacExtendedAddress macCoordExtendedAddress);





static inline  TMacShortAddress MacDeviceAttrM$IMacGET$GetmacCoordShortAddress(
TMacStatus *const pMacStatus);



static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacCoordShortAddress(
TMacShortAddress macCoordShortAddress);
#line 260
static __inline void MacDeviceAttrM$SetDefaultAttributes(void);
#line 291
static inline  void MacDeviceAttrM$confirmReset(void);




static inline  result_t MacDeviceAttrM$IMacRESET$Request(bool setDefaultPIB);









static inline  result_t MacDeviceAttrM$IPhySET_TRX_STATE$Confirm(TPhyStatus phyStatus, TUniData uniData);
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   TSysTime AliveSender2M$ITimeCast$MillisToSymbols(TMilliSec arg_0x40a3e6e0);
# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
static  void AliveSender2M$NLME_Leave$indication(IEEEAddr arg_0x4088a300);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress AliveSender2M$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08);
#line 37
static  TMacPANId AliveSender2M$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010);
#line 27
static  TMacShortAddress AliveSender2M$IMacGET$GetmacCoordShortAddress(TMacStatus *const arg_0x408c0600);
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
static  result_t AliveSender2M$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28);
# 12 "../../../zigzag/ZigBee/extra/AliveSender2M.nc"
static  uint8_t AliveSender2M$newMsduHandle(void);
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
static  result_t AliveSender2M$IMacDATA$Request(TMacPANId arg_0x409ca0b8, 
TMacAddress arg_0x409ca250, 
TMacPANId arg_0x409ca3f0, 
TMacAddress arg_0x409ca588, 
uint8_t arg_0x409ca720, 
const uint8_t *const arg_0x409ca938, 
uint8_t arg_0x409caad0, 
TxOptions arg_0x409cac68);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t AliveSender2M$Timer$Stop(void);
#line 6
static  result_t AliveSender2M$Timer$SetPeriodic(TSysTime arg_0x408cdca8, TUniData arg_0x408cde30);




static  bool AliveSender2M$Timer$IsSet(void);
# 19 "../../../zigzag/ZigBee/extra/AliveSender2M.nc"
#line 16
struct AliveSender2M$__nesc_unnamed4431 {
  uint8_t failCount;
  uint8_t sending;
} AliveSender2M$f;

static inline result_t AliveSender2M$sendEmptyFrame(void);

static result_t AliveSender2M$restartTimer(void);










static inline  result_t AliveSender2M$IMacDATA$Confirm(uint8_t msdu_L_Handle, TMacStatus MacStatus);








static inline  result_t AliveSender2M$Timer$Fired(TUniData u);
#line 62
static inline  result_t AliveSender2M$StdControl$start(void);
static inline  result_t AliveSender2M$StdControl$stop(void);

static inline result_t AliveSender2M$sendEmptyFrame(void);
#line 85
static inline  result_t AliveSender2M$IMacDATA$Indication(TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
uint8_t *msduFrame, 
TMacLinkQuality linkQuality, 
bool bSecurityUse, 
uint8_t ACLEntry);
# 191 "../../../zigzag/ZigBee/implementation/NIB.nc"
static  void NLME_StartRouterM$NIB$setOnline(NwkOnlineStatus arg_0x40875ae0);
#line 204
static  void NLME_StartRouterM$NIB$setBeaconOffset(TSysTime arg_0x4088e530);
#line 192
static  NwkOnlineStatus NLME_StartRouterM$NIB$getOnlineStatus(void);



static  uint8_t NLME_StartRouterM$NIB$getDepth(void);



static  uint8_t NLME_StartRouterM$NIB$getChannel(void);
#line 56
static  uint8_t NLME_StartRouterM$NIB$getNwkMaxDepth(void);
# 19 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacBeaconOrder NLME_StartRouterM$IMacGET$GetmacBeaconOrder(TMacStatus *const arg_0x408c11d8);
#line 37
static  TMacPANId NLME_StartRouterM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010);







static  TMacSuperframeOrder NLME_StartRouterM$IMacGET$GetmacSuperframeOrder(TMacStatus *const arg_0x408d9440);
# 26 "../../../zigzag/ZigBee/implementation/NLME_StartRouterM.nc"
static  void NLME_StartRouterM$updateBeacon(void);
static  result_t NLME_StartRouterM$findOffset(uint8_t arg_0x4128f970, uint8_t arg_0x4128faf8, TSysTime *arg_0x4128fca0);
# 10 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
static  result_t NLME_StartRouterM$IMacSTART$Request(
TMacPANId arg_0x40b14110, 
TMacLogicalChannel arg_0x40b142b8, 
TMacBeaconOrder arg_0x40b14458, 
TMacSuperframeOrder arg_0x40b14600, 
bool arg_0x40b14798, 
bool arg_0x40b14938, 
bool arg_0x40b14ad8, 
bool arg_0x40b14c70, 
TSysTime arg_0x40b14e20);
# 22 "../../../zigzag/ZigBee/interface/NLME_StartRouter.nc"
static  void NLME_StartRouterM$NLME_StartRouter$confirm(NwkStatus arg_0x4086fd88);
# 34 "../../../zigzag/ZigBee/implementation/NLME_StartRouterM.nc"
static inline  void NLME_StartRouterM$NLME_StartRouter$request(
uint16_t slotNumber, 
bool batteryLifeExtension);
#line 110
static  result_t NLME_StartRouterM$IMacSTART$Confirm(TMacStatus status);
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
static   uint16_t NwkBeaconOffsetRandomM$IMacRandom$Rand(void);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t NwkBeaconOffsetRandomM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790);
# 17 "../../../zigzag/ZigBee/implementation/NwkBeaconOffsetRandomM.nc"
static inline bool NwkBeaconOffsetRandomM$overlap(uint32_t a, uint32_t b);







static uint16_t NwkBeaconOffsetRandomM$positive_residual(uint32_t dividend, uint32_t divisor);
#line 44
static inline  result_t NwkBeaconOffsetRandomM$findOffset(uint8_t beaconOrder, uint8_t superframeOrder, TSysTime *offset);
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
static   TSysTime ChildSupervisorM$ITimeCast$MillisToSymbols(TMilliSec arg_0x40a3e6e0);
# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
static  void ChildSupervisorM$NLME_Leave$indication(IEEEAddr arg_0x4088a300);
# 17 "../../../zigzag/ZigBee/extra/ChildSupervisorM.nc"
static  void ChildSupervisorM$updateBeacon(void);
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime.nc"
static   TSysTime ChildSupervisorM$ILocalTime$Read(void);
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t ChildSupervisorM$ITimerSymbol$Stop(void);
#line 6
static  result_t ChildSupervisorM$ITimerSymbol$SetPeriodic(TSysTime arg_0x408cdca8, TUniData arg_0x408cde30);
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
static  result_t ChildSupervisorM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790);










static  result_t ChildSupervisorM$NeighborTable$remove(NwkNeighbor *arg_0x40869360);
# 24 "../../../zigzag/ZigBee/extra/ChildSupervisorM.nc"
TSysTime ChildSupervisorM$table_check_period;

static inline result_t  ChildSupervisorM$StdControl$start(void);









static inline result_t  ChildSupervisorM$StdControl$stop(void);






static inline  result_t ChildSupervisorM$ITimerSymbol$Fired(TUniData u);
# 5 "../../../zigzag/PowerManagerM.nc"
static inline  result_t PowerManagerM$StdControl$init(void);
static inline  result_t PowerManagerM$StdControl$start(void);
# 8 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
static  uint64_t ZigSysM$ILocalTime64$getLocalTime(void);
static  void ZigSysM$ILocalTime64$setLocalTime(uint64_t arg_0x40a41c40);
# 22 "../../../zigzag/ZigSysM.nc"
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
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
static  result_t ZigSTimerM$Timer0$setOneShot(int32_t arg_0x40887668);
#line 28
static  result_t ZigSTimerM$Timer1$setOneShot(int32_t arg_0x40887668);
#line 28
static  result_t ZigSTimerM$Timer2$setOneShot(int32_t arg_0x40887668);
# 11 "../../../zigzag/ZigSTimerM.nc"
int16_t __stimer_set(const uint8_t timer_num, const uint32_t milli_sec)   ;
#line 26
static inline  result_t ZigSTimerM$Timer0$fired(void);






static inline  result_t ZigSTimerM$Timer1$fired(void);






static inline  result_t ZigSTimerM$Timer2$fired(void);
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
static  TMacShortAddress ZigNetM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08);
# 23 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
static  result_t ZigNetM$NLDE_Data$request(
NwkAddr arg_0x408a4490, 
uint8_t arg_0x408a4628, 
uint8_t *arg_0x408a47d8, 
uint8_t arg_0x408a4970, 
uint8_t arg_0x408a4b00, 
uint8_t arg_0x408a4c98, 
bool arg_0x408a4e30);
# 24 "../../../zigzag/ZigNetM.nc"
int16_t __net_send(const uint16_t dst_addr, const uint16_t data_size, 
const uint8_t *const data, uint8_t handle)   ;
#line 37
static  void ZigNetM$NLDE_Data$confirm(uint8_t nsduHandle, NwkStatus status);







static inline  void ZigNetM$NLDE_Data$indication(NwkAddr srcAddr, IEEEAddr srcExtAddr, uint8_t nsduLength, 
uint8_t *nsdu, uint8_t linkQuality);







uint16_t __net_addr(void)   ;








static  void ZigNetM$NLME_JoinChild$confirm(PanID_t panID, NwkStatus status);






static inline  void ZigNetM$NLME_Reset$confirm(NwkStatus status);






static inline  void ZigNetM$NLME_JoinParent$indication(NwkAddr shortAddr, IEEEAddr extendedAddr, 
NwkCapabilityInfo capabilityInformation, bool secureJoin);






static inline  void ZigNetM$NLME_Leave$indication(IEEEAddr DeviceAddr);







static inline  void ZigNetM$sleepIndication(TMilliSec sleep_duration);
# 8 "../../../zigzag/ZigMonitorM.nc"
static inline  void ZigMonitorM$NLME_JoinParent$indication(NwkAddr shortAddr, IEEEAddr extendedAddr, 
NwkCapabilityInfo capabilityInformation, bool secureJoin);




static inline  void ZigMonitorM$NLME_Leave$indication(IEEEAddr deviceAddr);
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
  union __nesc_unnamed4432 {
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
  union __nesc_unnamed4433 {
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
inline static   void TimerTuningM$ITimerB1Control$setControl(MSP430CompareControl_t arg_0x40739b58){
#line 34
  MSP430TimerM$ControlB1$setControl(arg_0x40739b58);
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
inline static   void TimerTuningM$ITimerB$setMode(int arg_0x4072db50){
#line 35
  MSP430TimerM$TimerB$setMode(arg_0x4072db50);
#line 35
}
#line 35
# 200 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerB$setInputDivider(uint16_t inputDivider)
{
  TBCTL = (TBCTL & ~((1 << 6) | (3 << 6))) | ((inputDivider << 8) & ((1 << 6) | (3 << 6)));
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerB$setInputDivider(uint16_t arg_0x4072be18){
#line 40
  MSP430TimerM$TimerB$setInputDivider(arg_0x4072be18);
#line 40
}
#line 40
# 190 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$TimerB$setClockSource(uint16_t clockSource)
{
  TBCTL = (TBCTL & ~(0x0100 | 0x0200)) | ((clockSource << 8) & (0x0100 | 0x0200));
}

# 39 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerTuningM$ITimerB$setClockSource(uint16_t arg_0x4072b970){
#line 39
  MSP430TimerM$TimerB$setClockSource(arg_0x4072b970);
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
inline static   void TimerTuningM$ITimerA$setClockSource(uint16_t arg_0x4072b970){
#line 39
  MSP430TimerM$TimerA$setClockSource(arg_0x4072b970);
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
inline static   void TimerTuningM$ITimerA$setMode(int arg_0x4072db50){
#line 35
  MSP430TimerM$TimerA$setMode(arg_0x4072db50);
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
# 57 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_CLR_DBG2_PIN(void)
#line 57
{
#line 57
   static volatile uint8_t r __asm ("0x0031");

#line 57
  r &= ~(1 << 5);
}

#line 56
static inline void TOSH_CLR_DBG1_PIN(void)
#line 56
{
#line 56
   static volatile uint8_t r __asm ("0x0031");

#line 56
  r &= ~(1 << 4);
}

#line 57
static inline void TOSH_MAKE_DBG2_OUTPUT(void)
#line 57
{
#line 57
   static volatile uint8_t r __asm ("0x0032");

#line 57
  r |= 1 << 5;
}

#line 56
static inline void TOSH_MAKE_DBG1_OUTPUT(void)
#line 56
{
#line 56
   static volatile uint8_t r __asm ("0x0032");

#line 56
  r |= 1 << 4;
}

#line 57
static inline void TOSH_SEL_DBG2_IOFUNC(void)
#line 57
{
#line 57
   static volatile uint8_t r __asm ("0x0033");

#line 57
  r &= ~(1 << 5);
}

#line 56
static inline void TOSH_SEL_DBG1_IOFUNC(void)
#line 56
{
#line 56
   static volatile uint8_t r __asm ("0x0033");

#line 56
  r &= ~(1 << 4);
}



static inline void TOSH_SET_PIN_DIRECTIONS(void )
{
  uint16_t i;

#line 64
  for (i = 0xFFFF; i > 0; i--) 
     __asm volatile ("nop");

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 67
    {

      P1IE = 0;
      P1OUT = 0xc0;
      P1SEL = 0x00;
      P1DIR = 0xf0;
      P1IFG = 0x00;

      P2IE = 0x00;
      P2IFG = 0x00;

      P3OUT = 0x01;
      P3SEL = 0x0e;
      P3DIR = 0x01;

      P4OUT = 0x00;
      P4SEL = 0x02;
      P4DIR = 0x00;


      TOSH_SEL_DBG1_IOFUNC();
      TOSH_SEL_DBG2_IOFUNC();
      TOSH_MAKE_DBG1_OUTPUT();
      TOSH_MAKE_DBG2_OUTPUT();
      TOSH_CLR_DBG1_PIN();
      TOSH_CLR_DBG2_PIN();
    }
#line 93
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

# 5 "../../../zigzag/PowerManagerM.nc"
static inline  result_t PowerManagerM$StdControl$init(void)
#line 5
{
#line 5
  return SUCCESS;
}

# 367 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline  result_t MacDataM$StdControl$init(void)
{
  return SUCCESS;
}

# 567 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetDefaultmacAssociationPermit(void)
{
  return MAC_UNSUPPORTED_ATTRIBUTE;
}

#line 103
static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void)
{
  MacAssocDeviceM$assoc.macCoordShortAddress = 0xffff;
  return MAC_SUCCESS;
}

#line 84
static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void)
{
  MacAssocDeviceM$assoc.macCoordExtendedAddress = 0x0;
  return MAC_SUCCESS;
}

#line 109
static inline  result_t MacAssocDeviceM$IStdControl$init(void)
{
  MacAssocDeviceM$assoc.state = MacAssocDeviceM$MAC_ASSOC_OFF;
  MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress();
  MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordShortAddress();
  MacAssocDeviceM$IMacAssocAttr$SetDefaultmacAssociationPermit();
  return SUCCESS;
}

# 175 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
static inline  result_t MacExtractM$IStdControl$init(void)
{
  MacExtractM$extractState = MacExtractM$MAC_EXTRACT_OFF;
  return SUCCESS;
}

# 131 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
static inline  result_t MacSyncM$StdControl$init(void)
{
  return SUCCESS;
}

# 402 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  result_t MacSuperframesM$StdControl$init(void)
{
  return SUCCESS;
}

# 108 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static inline  result_t PhyPoolM$IStdControl$init(void)
{
  PhyPoolM$ResetPool();
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

# 181 "../../../zigzag/ZigRouterM.nc"
static inline  result_t ZigRouterM$StdControl$init(void)
{
  return SUCCESS;
}

# 63 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t MainM$StdControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = ZigRouterM$StdControl$init();
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
  result = rcombine(result, MacSyncM$StdControl$init());
#line 63
  result = rcombine(result, MacCAPM$ICAPControl$init(MAC_PARENT_SF));
#line 63
  result = rcombine(result, MacExtractM$IStdControl$init());
#line 63
  result = rcombine(result, MacAssocDeviceM$IStdControl$init());
#line 63
  result = rcombine(result, MacDataM$StdControl$init());
#line 63
  result = rcombine(result, PowerManagerM$StdControl$init());
#line 63
  result = rcombine(result, ZigSysM$StdControl$init());
#line 63

#line 63
  return result;
#line 63
}
#line 63
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
inline static   void PhyPoolM$FreePoolItem(TPhyPoolHandle arg_0x40c21690){
#line 11
  PhyCC2420M$FreePoolItem(arg_0x40c21690);
#line 11
}
#line 11
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_SEL_RADIO_POWER_IOFUNC(void)
#line 38
{
#line 38
   static volatile uint8_t r __asm ("0x0026");

#line 38
  r &= ~(1 << 6);
}

#line 38
static inline void TOSH_MAKE_RADIO_POWER_OUTPUT(void)
#line 38
{
#line 38
   static volatile uint8_t r __asm ("0x0022");

#line 38
  r |= 1 << 6;
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
# 42 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_MAKE_RADIO_CSN_OUTPUT(void)
#line 42
{
#line 42
   static volatile uint8_t r __asm ("0x001A");

#line 42
  r |= 1 << 0;
}

#line 42
static inline void TOSH_SET_RADIO_CSN_PIN(void)
#line 42
{
#line 42
   static volatile uint8_t r __asm ("0x0019");

#line 42
  r |= 1 << 0;
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

# 47 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline bool TOSH_IS_URXD0_IOFUNC(void)
#line 47
{
#line 47
   static volatile uint8_t r __asm ("0x001B");

#line 47
  return r | ~(1 << 5);
}

#line 46
static inline bool TOSH_IS_UTXD0_MODFUNC(void)
#line 46
{
#line 46
   static volatile uint8_t r __asm ("0x001B");

#line 46
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

# 46 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline bool TOSH_IS_UTXD0_IOFUNC(void)
#line 46
{
#line 46
   static volatile uint8_t r __asm ("0x001B");

#line 46
  return r | ~(1 << 4);
}

#line 47
static inline bool TOSH_IS_URXD0_MODFUNC(void)
#line 47
{
#line 47
   static volatile uint8_t r __asm ("0x001B");

#line 47
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

# 47 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_SEL_URXD0_IOFUNC(void)
#line 47
{
#line 47
   static volatile uint8_t r __asm ("0x001B");

#line 47
  r &= ~(1 << 5);
}

#line 46
static inline void TOSH_SEL_UTXD0_IOFUNC(void)
#line 46
{
#line 46
   static volatile uint8_t r __asm ("0x001B");

#line 46
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

# 43 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_SEL_SIMO0_MODFUNC(void)
#line 43
{
#line 43
   static volatile uint8_t r __asm ("0x001B");

#line 43
  r |= 1 << 1;
}

#line 44
static inline void TOSH_SEL_SOMI0_MODFUNC(void)
#line 44
{
#line 44
   static volatile uint8_t r __asm ("0x001B");

#line 44
  r |= 1 << 2;
}

#line 45
static inline void TOSH_SEL_UCLK0_MODFUNC(void)
#line 45
{
#line 45
   static volatile uint8_t r __asm ("0x001B");

#line 45
  r |= 1 << 3;
}

# 16 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   TSysTime TimeCastM$ITimeCast$JiffiesToSymbols(uint32_t jiffies)
{
  return jiffies << 1;
}

# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TSysTime TimerSymbol2M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a3eb98){
#line 7
  unsigned long result;
#line 7

#line 7
  result = TimeCastM$ITimeCast$JiffiesToSymbols(arg_0x40a3eb98);
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

# 360 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline  result_t MacDataM$StdControl$start(void)
{
  MacDataM$state = MacDataM$IDLE;
  return SUCCESS;
}

# 118 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  result_t MacAssocDeviceM$IStdControl$start(void)
{
  MacAssocDeviceM$assoc.state = MacAssocDeviceM$MAC_ASSOC_IDLE;
  return SUCCESS;
}

# 181 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
static inline  result_t MacExtractM$IStdControl$start(void)
{
  MacExtractM$extractState = MacExtractM$MAC_EXTRACT_IDLE;
  return SUCCESS;
}

# 126 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
static inline  result_t MacSyncM$StdControl$start(void)
{
  return SUCCESS;
}

# 406 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  result_t MacSuperframesM$StdControl$start(void)
{
  return SUCCESS;
}

# 7 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
inline static  result_t MacCommonAttrM$IPhyAttr$SetextendedAddress(uint64_t arg_0x40b405a0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = PhyCC2420M$IPhyAttr$SetextendedAddress(arg_0x40b405a0);
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

# 114 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static inline  result_t PhyPoolM$IStdControl$start(void)
{

  return SUCCESS;
}

# 62 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  result_t TimerSymbol2M$IStdControl$start(void)
#line 62
{
#line 62
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

# 152 "../../../zigzag/ZigBee/implementation/NLME_JoinParentM.nc"
static inline  void NLME_JoinParentM$Reset$reset(void)
{
  NLME_JoinParentM$state = NLME_JoinParentM$IDLE;
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
# 5 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
inline static  result_t NwkResetMacM$IMacRESET1$Request(bool arg_0x409c6640){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacCoordAttrM$IMacRESET$Request(arg_0x409c6640);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 16 "../../../zigzag/ZigBee/implementation/NwkResetMacM.nc"
static inline  void NwkResetMacM$NLME_Reset$request(void)
{
  NwkResetMacM$IMacRESET1$Request(TRUE);
}

# 15 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
inline static  void NLME_ResetM$MacReset$request(void){
#line 15
  NwkResetMacM$NLME_Reset$request();
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
inline static  void ZigRouterM$NLME_Reset$request(void){
#line 15
  NLME_ResetM$NLME_Reset$request();
#line 15
}
#line 15
# 63 "../../../zigzag/ZigBee/extra/AliveSender2M.nc"
static inline  result_t AliveSender2M$StdControl$stop(void)
#line 63
{
#line 63
  return AliveSender2M$Timer$Stop();
}

# 78 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t ZigRouterM$AliveSenderControl$stop(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = AliveSender2M$StdControl$stop();
#line 78

#line 78
  return result;
#line 78
}
#line 78
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
inline static  result_t ChildSupervisorM$ITimerSymbol$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(13U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 36 "../../../zigzag/ZigBee/extra/ChildSupervisorM.nc"
static inline result_t  ChildSupervisorM$StdControl$stop(void)
{
  ChildSupervisorM$ITimerSymbol$Stop();

  return SUCCESS;
}

# 78 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t ZigRouterM$ChildSupervisorControl$stop(void){
#line 78
  unsigned char result;
#line 78

#line 78
  result = ChildSupervisorM$StdControl$stop();
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 38 "../../../zigzag/ZigRouterM.nc"
static inline  void ZigRouterM$restart(void)
{
  ;
  ZigRouterM$ChildSupervisorControl$stop();
  ZigRouterM$AliveSenderControl$stop();
  ZigRouterM$NLME_Reset$request();
}








static inline  result_t ZigRouterM$StdControl$start(void)
{
  ;
  ZigRouterM$idle_scan = FALSE;

  TOS_post(ZigRouterM$restart);

  return SUCCESS;
}

# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t MainM$StdControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = ZigRouterM$StdControl$start();
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
  result = rcombine(result, MacSyncM$StdControl$start());
#line 70
  result = rcombine(result, MacCAPM$ICAPControl$start(MAC_PARENT_SF));
#line 70
  result = rcombine(result, MacExtractM$IStdControl$start());
#line 70
  result = rcombine(result, MacAssocDeviceM$IStdControl$start());
#line 70
  result = rcombine(result, MacDataM$StdControl$start());
#line 70
  result = rcombine(result, PowerManagerM$StdControl$start());
#line 70
  result = rcombine(result, ZigSysM$StdControl$start());
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 12 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacScanBeaconM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b3dbc0){
#line 12
  unsigned char result;
#line 12

#line 12
  result = PhyPoolM$IPhyPool$Free(arg_0x40b3dbc0);
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

# 371 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
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

# 641 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  void MacCAPM$Reset$reset(uint8_t sfIndex)
{
  ;
  MacCAPM$ICAPControl$stop(sfIndex);
  MacCAPM$ICAPControl$start(sfIndex);
  return;
}

# 62 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframeAttrC.nc"
static inline  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void)
{
  MacSuperframeAttrC$macSuperframeOrder = MacSuperframeAttrC$MAX_SUPERFRAME_ORDER;
  return MAC_SUCCESS;
}

#line 43
static inline  TMacStatus MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void)
{
  MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(MacSuperframeAttrC$MAX_BEACON_ORDER);
  return MAC_SUCCESS;
}

#line 21
static inline  void MacSuperframeAttrC$Reset$reset(void)
{
  MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacBeaconOrder();
  MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder();
}

# 421 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
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
# 12 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacSuperframesM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b3dbc0){
#line 12
  unsigned char result;
#line 12

#line 12
  result = PhyPoolM$IPhyPool$Free(arg_0x40b3dbc0);
#line 12

#line 12
  return result;
#line 12
}
#line 12
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
inline static  void MacSuperframeAttrC$BeaconOrderChanged(uint8_t arg_0x40eb1998){
#line 10
  MacCAPM$BeaconOrderChanged(arg_0x40eb1998);
#line 10
}
#line 10
#line 27
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
inline static  TMacBeaconOrder MacCAPM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e57ab8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 20 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetDefaultmacAssociationPermit(void){
#line 20
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
#line 14

#line 14
  result = MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods();
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 682 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetmacBattLifeExt(bool battLifeExt)
{
  MacCAPM$macBattLifeExt = battLifeExt;
  return MAC_SUCCESS;
}

#line 687
static inline  TMacStatus MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExt(void)
{
  return MacCAPM$IMacCAPAttr$SetmacBattLifeExt(FALSE);
}

# 8 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCAPAttr$SetDefaultmacBattLifeExt(void){
#line 8
  enum __nesc_unnamed4286 result;
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
  enum __nesc_unnamed4286 result;
#line 10

#line 10
  result = MacPoolM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime();
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 12 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void){
#line 12
  enum __nesc_unnamed4286 result;
#line 12

#line 12
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder();
#line 12

#line 12
  return result;
#line 12
}
#line 12
#line 8
inline static  TMacStatus MacCoordAttrM$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void){
#line 8
  enum __nesc_unnamed4286 result;
#line 8

#line 8
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacBeaconOrder();
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 121 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPANId(void)
{
  MacCommonAttrM$pib.macPANId = 0xffff;
  return MAC_SUCCESS;
}

# 8 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacPANId(void){
#line 8
  enum __nesc_unnamed4286 result;
#line 8

#line 8
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacPANId();
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 141 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void)
{
  MacCommonAttrM$pib.macShortAddress = 0xffff;
  return MAC_SUCCESS;
}

# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void){
#line 12
  enum __nesc_unnamed4286 result;
#line 12

#line 12
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacShortAddress();
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 159 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void)
{
  MacCommonAttrM$pib.macRxOnWhenIdle = FALSE;
  return MAC_SUCCESS;
}

# 32 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void){
#line 32
  enum __nesc_unnamed4286 result;
#line 32

#line 32
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle();
#line 32

#line 32
  return result;
#line 32
}
#line 32
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

# 20 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacDSN(void){
#line 20
  enum __nesc_unnamed4286 result;
#line 20

#line 20
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacDSN();
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 83 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void)
{
  return MAC_UNSUPPORTED_ATTRIBUTE;
}

# 24 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void){
#line 24
  enum __nesc_unnamed4286 result;
#line 24

#line 24
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode();
#line 24

#line 24
  return result;
#line 24
}
#line 24
# 100 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void)
{
  MacCommonAttrM$pib.macSecurityMode = MAC_UNSECURED_MODE;
  return MAC_SUCCESS;
}

# 28 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void){
#line 28
  enum __nesc_unnamed4286 result;
#line 28

#line 28
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacSecurityMode();
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 64 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void)
{
  MacCommonAttrM$pib.macAckWaitDuration = MAC_ACK_WAIT_DURATION_54;
  return MAC_SUCCESS;
}

# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacCoordAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void){
#line 16
  enum __nesc_unnamed4286 result;
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

# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacCoordAttrM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(4U, arg_0x40b27940, arg_0x40b27ae8);
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
inline static   result_t MacSuperframesM$IPhyTxFIFO$Write(TPhyPoolHandle arg_0x40b31130, TUniData arg_0x40b312b8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhyTxFIFO$Write(0U, arg_0x40b31130, arg_0x40b312b8);
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
inline static   result_t MacCAPM$IPhyTxDATA$Request(const TPhyPoolHandle arg_0x40b32f18, const TUniData arg_0x40b340f0){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhyTxDATA$Request(1U, arg_0x40b32f18, arg_0x40b340f0);
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

# 17 "../../../zigzag/ZigBee/interface/NLME_Sync.nc"
inline static  void NLME_SyncM$NLME_Sync$indication(void){
#line 17
  ZigRouterM$NLME_Sync$indication();
#line 17
}
#line 17
# 54 "../../../zigzag/ZigBee/implementation/NLME_SyncM.nc"
static inline  void NLME_SyncM$IMacSYNC_LOSS$Indication(TMacStatus reason)
{

  NLME_SyncM$NLME_Sync$indication();
}

# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC_LOSS.nc"
inline static  void MacSyncM$IMacSYNC_LOSS$Indication(TMacStatus arg_0x4097b5b0){
#line 6
  NLME_SyncM$IMacSYNC_LOSS$Indication(arg_0x4097b5b0);
#line 6
}
#line 6
# 42 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   uint32_t TimeCastM$ITimeCast$SymbolsToJiffies(TSysTime symbols)
{
  return symbols >> 1;
}

# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   uint32_t TimerSymbol2M$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a3c4c8){
#line 15
  unsigned long result;
#line 15

#line 15
  result = TimeCastM$ITimeCast$SymbolsToJiffies(arg_0x40a3c4c8);
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
inline static  result_t MacSyncM$InitialTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(6U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacSyncM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e57ab8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 71 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
static inline  result_t MacSyncM$IPhySET_TRX_STATE$Confirm(TPhyStatus status, TUniData unidata)
{
  if (MacSyncM$first_radio_on) 
    {
      TSysTime scan_duration;

#line 76
      MacSyncM$first_radio_on = FALSE;
      scan_duration = (((uint32_t )MAC_ABASE_SUPERFRAME_DURATION << MacSyncM$IMacSuperframeAttr$GetmacBeaconOrder(NULL)) + MAC_ABASE_SUPERFRAME_DURATION) * MAC_AMAX_LOST_BEACONS;
      if (!MacSyncM$InitialTimer$SetOneShot(scan_duration, 0)) {
        MacSyncM$IMacSYNC_LOSS$Indication(MAC_BEACON_LOSS);
        }
    }
#line 81
  return SUCCESS;
}

# 11 "../../../zigzag/IEEE802_15_4/MacScan/public/IMacSCAN.nc"
inline static  result_t MacScanM$IMacSCAN$Confirm(TMacStatus arg_0x4091f968, TMacScanType arg_0x4091fb08, uint32_t arg_0x4091fcb0, uint8_t arg_0x4091fe48, uint8_t *arg_0x4091d030, TMacPanDescriptor *arg_0x4091d200){
#line 11
  unsigned char result;
#line 11

#line 11
  result = NLME_NetworkDiscoveryM$IMacSCAN$Confirm(arg_0x4091f968, arg_0x4091fb08, arg_0x4091fcb0, arg_0x4091fe48, arg_0x4091d030, arg_0x4091d200);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacScanM$TimerBeaconScan$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(11U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyED.nc"
inline static   result_t MacScanM$IPhyED$Request(const TUniData arg_0x40b2dd08){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhyED$Request(0U, arg_0x40b2dd08);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacScanM$TimerED$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(10U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 119 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanM.nc"
static inline  result_t MacScanM$IPhySET_TRX_STATE$Confirm(TPhyStatus status, TUniData uniData)
{
  if (MacScanM$bTrOn) 
    {
      MacScanM$bTrOn = FALSE;

      if (status == PHY_SUCCESS || status == PHY_RX_ON) 
        {
          if (MacScanM$eMode == MAC_ED_SCAN) 
            {
              if (MacScanM$TimerED$SetOneShot(MacScanM$ScanTime, 0)) 
                {
                  MacScanM$IPhyED$Request(NULL);
                  MacScanM$TimIsSet = TRUE;
                  return SUCCESS;
                }
            }
          else 
            {
              if (MacScanM$eMode == MAC_ACTIVE_SCAN) {
                MacScanM$SendFrame();
                }
              if (MacScanM$TimerBeaconScan$SetOneShot(MacScanM$ScanTime, 0)) 
                {
                  MacScanM$TimIsSet = TRUE;
                  return SUCCESS;
                }
            }
        }

      {
        return MacScanM$IMacSCAN$Confirm(MAC_INVALID_PARAMETER, NULL, MacScanM$ScanChannels, 0, NULL, NULL);
      }
    }

  return SUCCESS;
}

# 306 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  result_t MacDeviceAttrM$IPhySET_TRX_STATE$Confirm(TPhyStatus phyStatus, TUniData uniData)
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
inline static  result_t PhyCC2420M$IPhySET_TRX_STATE$Confirm(uint8_t arg_0x40b81340, TPhyStatus arg_0x40b26010, TUniData arg_0x40b26198){
#line 8
  unsigned char result;
#line 8

#line 8
  switch (arg_0x40b81340) {
#line 8
    case 0U:
#line 8
      result = MacSuperframesM$IPhyTrxParent$Confirm(arg_0x40b26010, arg_0x40b26198);
#line 8
      break;
#line 8
    case 1U:
#line 8
      result = MacSuperframesM$IPhyTrxOwn$Confirm(arg_0x40b26010, arg_0x40b26198);
#line 8
      break;
#line 8
    case 2U:
#line 8
      result = MacCAPM$IPhySET_TRX_STATE$Confirm(arg_0x40b26010, arg_0x40b26198);
#line 8
      break;
#line 8
    case 3U:
#line 8
      result = MacStartM$IPhySET_TRX_STATE$Confirm(arg_0x40b26010, arg_0x40b26198);
#line 8
      break;
#line 8
    case 4U:
#line 8
      result = MacCoordAttrM$IPhySET_TRX_STATE$Confirm(arg_0x40b26010, arg_0x40b26198);
#line 8
      break;
#line 8
    case 5U:
#line 8
      result = MacSyncM$IPhySET_TRX_STATE$Confirm(arg_0x40b26010, arg_0x40b26198);
#line 8
      break;
#line 8
    case 6U:
#line 8
      result = MacScanM$IPhySET_TRX_STATE$Confirm(arg_0x40b26010, arg_0x40b26198);
#line 8
      break;
#line 8
    case 7U:
#line 8
      result = MacDeviceAttrM$IPhySET_TRX_STATE$Confirm(arg_0x40b26010, arg_0x40b26198);
#line 8
      break;
#line 8
    default:
#line 8
      result = PhyCC2420M$IPhySET_TRX_STATE$default$Confirm(arg_0x40b81340, arg_0x40b26010, arg_0x40b26198);
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
# 53 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_SEL_CC_SFD_IOFUNC(void)
#line 53
{
#line 53
   static volatile uint8_t r __asm ("0x001F");

#line 53
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
# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_CLR_RADIO_POWER_PIN(void)
#line 38
{
#line 38
   static volatile uint8_t r __asm ("0x0021");

#line 38
  r &= ~(1 << 6);
}

#line 37
static inline void TOSH_SET_CC_VREN_PIN(void)
#line 37
{
#line 37
   static volatile uint8_t r __asm ("0x0021");

#line 37
  r |= 1 << 5;
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

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_CLR_CC_RSTN_PIN(void)
#line 35
{
#line 35
   static volatile uint8_t r __asm ("0x0021");

#line 35
  r &= ~(1 << 4);
}

# 161 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
static __inline void TOSH_wait(void )
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_SET_CC_RSTN_PIN(void)
#line 35
{
#line 35
   static volatile uint8_t r __asm ("0x0021");

#line 35
  r |= 1 << 4;
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
inline static  result_t BusArbitrationM$BusArbitration$busFree(uint8_t arg_0x40d73a88){
#line 39
  unsigned char result;
#line 39

#line 39
  switch (arg_0x40d73a88) {
#line 39
    case 0U:
#line 39
      result = HPLCC2420M$BusArbitration$busFree();
#line 39
      break;
#line 39
    default:
#line 39
      result = BusArbitrationM$BusArbitration$default$busFree(arg_0x40d73a88);
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
inline static   uint8_t PhyCC2420M$IChipcon$write(uint8_t arg_0x40b7cb78, uint16_t arg_0x40b7cd00){
#line 54
  unsigned char result;
#line 54

#line 54
  result = HPLCC2420M$HPLCC2420$write(arg_0x40b7cb78, arg_0x40b7cd00);
#line 54

#line 54
  return result;
#line 54
}
#line 54







inline static   uint16_t PhyCC2420M$IChipcon$read(uint8_t arg_0x40b7b248){
#line 61
  unsigned int result;
#line 61

#line 61
  result = HPLCC2420M$HPLCC2420$read(arg_0x40b7b248);
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
inline static   result_t PhyCC2420M$IChipconRAM$write(uint16_t arg_0x40b98638, uint8_t arg_0x40b987b8, uint8_t *arg_0x40b98958){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLCC2420M$HPLCC2420RAM$write(arg_0x40b98638, arg_0x40b987b8, arg_0x40b98958);
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
inline static   result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t arg_0x40b98e80, uint8_t arg_0x40b97030, uint8_t *arg_0x40b971d0){
#line 49
  unsigned char result;
#line 49

#line 49
  result = PhyCC2420M$IChipconRAM$writeDone(arg_0x40b98e80, arg_0x40b97030, arg_0x40b971d0);
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

# 53 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_SEL_CC_SFD_MODFUNC(void)
#line 53
{
#line 53
   static volatile uint8_t r __asm ("0x001F");

#line 53
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
inline static   void HPLCC2420InterruptM$SFDControl$setControlAsCapture(bool arg_0x40737358){
#line 36
  MSP430TimerM$ControlB1$setControlAsCapture(arg_0x40737358);
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
inline static   result_t PhyCC2420M$IChipconFIFOP$startWait(bool arg_0x40b9b010){
#line 43
  unsigned char result;
#line 43

#line 43
  result = HPLCC2420InterruptM$FIFOP$startWait(arg_0x40b9b010);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 227 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port11$edge(bool l2h)
#line 227
{
  /* atomic removed: atomic calls only */
#line 228
  {
    if (l2h) {
#line 229
      P1IES &= ~(1 << 1);
      }
    else {
#line 230
      P1IES |= 1 << 1;
      }
  }
}

# 54 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$edge(bool arg_0x40cdd418){
#line 54
  MSP430InterruptM$Port11$edge(arg_0x40cdd418);
#line 54
}
#line 54
# 116 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port11$enable(void)
#line 116
{
#line 116
  MSP430InterruptM$P1IE |= 1 << 1;
}

# 30 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$enable(void){
#line 30
  MSP430InterruptM$Port11$enable();
#line 30
}
#line 30
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
inline static   void HPLCC2420InterruptM$FIFOInterrupt$edge(bool arg_0x40cdd418){
#line 54
  MSP430InterruptM$Port10$edge(arg_0x40cdd418);
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
inline static   void HPLCC2420InterruptM$FIFOInterrupt$enable(void){
#line 30
  MSP430InterruptM$Port10$enable();
#line 30
}
#line 30
# 37 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_CLR_CC_VREN_PIN(void)
#line 37
{
#line 37
   static volatile uint8_t r __asm ("0x0021");

#line 37
  r &= ~(1 << 5);
}

#line 38
static inline void TOSH_SET_RADIO_POWER_PIN(void)
#line 38
{
#line 38
   static volatile uint8_t r __asm ("0x0021");

#line 38
  r |= 1 << 6;
}





static inline void TOSH_SEL_UCLK0_IOFUNC(void)
#line 45
{
#line 45
   static volatile uint8_t r __asm ("0x001B");

#line 45
  r &= ~(1 << 3);
}

#line 44
static inline void TOSH_SEL_SOMI0_IOFUNC(void)
#line 44
{
#line 44
   static volatile uint8_t r __asm ("0x001B");

#line 44
  r &= ~(1 << 2);
}

#line 43
static inline void TOSH_SEL_SIMO0_IOFUNC(void)
#line 43
{
#line 43
   static volatile uint8_t r __asm ("0x001B");

#line 43
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
# 467 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static inline   void MSP430TimerM$CompareB0$setEventFromNow(uint16_t x)
#line 467
{
#line 467
  MSP430TimerM$TBCCR0 = TBR + x;
}

# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerSymbol2M$AlarmCompare$setEventFromNow(uint16_t arg_0x40733b20){
#line 32
  MSP430TimerM$CompareB0$setEventFromNow(arg_0x40733b20);
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
inline static   TMilliSec LocalTime64M$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a3d378){
#line 10
  unsigned long result;
#line 10

#line 10
  result = TimeCastM$ITimeCast$SymbolsToMillis(arg_0x40a3d378);
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
inline static  TMacStatus MacCoordAttrM$IMacAssocAttr$SetmacAssociationPermit(bool arg_0x40ec10c0){
#line 19
  enum __nesc_unnamed4286 result;
#line 19

#line 19
  result = MacAssocCoordM$IMacAssocAttr$SetmacAssociationPermit(arg_0x40ec10c0);
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
inline static  TMacStatus NLME_PermitJoiningM$IMacSET$SetmacAssociationPermit(bool arg_0x408f0010){
#line 7
  enum __nesc_unnamed4286 result;
#line 7

#line 7
  result = MacCoordAttrM$IMacSET$SetmacAssociationPermit(arg_0x408f0010);
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
inline static  result_t MacPoolM$IExpiredTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(3U, arg_0x408d0340, arg_0x408d04c8);
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
inline static   result_t MacCAPM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(2U, arg_0x40b27940, arg_0x40b27ae8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 32 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline uint8_t TOSH_READ_RADIO_CCA_PIN(void)
#line 32
{
#line 32
   static volatile uint8_t r __asm ("0x0020");

#line 32
  return r & (1 << 2);
}

#line 31
static inline uint8_t TOSH_READ_CC_FIFOP_PIN(void)
#line 31
{
#line 31
   static volatile uint8_t r __asm ("0x0020");

#line 31
  return r & (1 << 1);
}

#line 29
static inline uint8_t TOSH_READ_CC_FIFO_PIN(void)
#line 29
{
#line 29
   static volatile uint8_t r __asm ("0x0020");

#line 29
  return r & (1 << 0);
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

# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacSyncM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(5U, arg_0x40b27940, arg_0x40b27ae8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 85 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
static inline  result_t MacSyncM$InitialTimer$Fired(TUniData uniData)
{

  MacSyncM$IPhySET_TRX_STATE$Request(PHY_TRX_OFF, 0);
  MacSyncM$IMacSYNC_LOSS$Indication(MAC_BEACON_LOSS);
  return SUCCESS;
}

# 63 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
inline static  result_t MacAssocDeviceM$IMacASSOCIATE$Confirm(TMacShortAddress arg_0x408ca888, TMacStatus arg_0x408caa20){
#line 63
  unsigned char result;
#line 63

#line 63
  result = NLME_JoinChildM$IMacASSOCIATE$Confirm(arg_0x408ca888, arg_0x408caa20);
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 217 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline void MacAssocDeviceM$AssocFailDone(TMacStatus macStatus)
{
  MacAssocDeviceM$assoc.state = MacAssocDeviceM$MAC_ASSOC_IDLE;
  MacAssocDeviceM$IMacASSOCIATE$Confirm(0xffff, macStatus);
}

#line 353
static inline  result_t MacAssocDeviceM$IReceiveResponseTimer$Fired(TUniData uniData)
{
  if (MacAssocDeviceM$assoc.state == MacAssocDeviceM$MAC_ASSOC_RECEIVE_RESPONSE) {
      MacAssocDeviceM$AssocFailDone(MAC_NO_DATA);
      return SUCCESS;
    }
  return FAIL;
}

# 16 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
inline static  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$Send(const uint8_t arg_0x40f93c88, const TMacFrame *const arg_0x40f93ea8, const uint8_t arg_0x40f92088, const TUniData arg_0x40f92240){
#line 16
  unsigned char result;
#line 16

#line 16
  result = MacCAPM$Send(arg_0x40f93c88, arg_0x40f93ea8, arg_0x40f92088, arg_0x40f92240);
#line 16

#line 16
  return result;
#line 16
}
#line 16
#line 30
static inline  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$Send(uint8_t context, const TMacFrame *const pMacFrame, 
const TUniData uniData)
{

  return /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$Send(1, pMacFrame, context, uniData);
}

# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  result_t MacAssocDeviceM$IAssocRequest$Send(const TMacFrame *const arg_0x40f051f8, const TUniData arg_0x40f053b0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$Send(4U, arg_0x40f051f8, arg_0x40f053b0);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 249 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  result_t MacAssocDeviceM$IAssocAckTimer$Fired(TUniData uniData)
{
  if (MacAssocDeviceM$assoc.state == MacAssocDeviceM$MAC_ASSOC_WAIT_ACK) {
      if (uniData >= MAC_AMAX_FRAME_RETRIES) {
          MacAssocDeviceM$AssocFailDone(MAC_NO_ACK);
          return SUCCESS;
        }
      else 
#line 255
        {
          TMacFrame assocRequest;
          result_t result = MacAssocDeviceM$MakeAssocReqFrame(MacAssocDeviceM$assoc.coordPANId, MacAssocDeviceM$assoc.coordAddress, 
          MacAssocDeviceM$assoc.capabilityInformation, MacAssocDeviceM$assoc.securityEnable, 
          &assocRequest);

#line 260
          if (result == SUCCESS) {
              result = MacAssocDeviceM$IAssocRequest$Send(&assocRequest, uniData + 1);
              if (result == SUCCESS) {
                  MacAssocDeviceM$assoc.state = MacAssocDeviceM$MAC_ASSOC_REQUEST;
                  return SUCCESS;
                }
            }
          MacAssocDeviceM$AssocFailDone(MAC_NO_ACK);
          return FAIL;
        }
    }
  return FAIL;
}

# 143 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
static inline  result_t MacExtractM$IAckWaitTimer$Fired(TUniData uniData)
{
  if (MacExtractM$MAC_EXTRACT_WAIT_ACK == MacExtractM$extractState) {
      MacExtractM$ExtractDone(MAC_NO_ACK, FALSE);
      return SUCCESS;
    }
  return FAIL;
}

# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
inline static  TPhyStatus MacScanM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b2a360){
#line 5
  enum __nesc_unnamed4242 result;
#line 5

#line 5
  result = PhyCC2420M$IPhySET$SetphyCurrentChannel(arg_0x40b2a360);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 181 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanM.nc"
static inline  result_t MacScanM$TimerBeaconScan$Fired(TUniData uniData)
{
  uint32_t channelBit = 0;

  MacScanM$TimIsSet = FALSE;


  channelBit = 1UL << MacScanM$CurBit;
  MacScanM$ScanChannels &= channelBit ^ 0xFFFFFFFFL;

  MacScanM$nextBit();
  if (MacScanM$CurBit < 27) 
    {

      MacScanM$IPhySET$SetphyCurrentChannel(MacScanM$CurBit);
      if (MacScanM$eMode == MAC_ACTIVE_SCAN) {
        MacScanM$SendFrame();
        }
#line 198
      if (MacScanM$TimerBeaconScan$SetOneShot(MacScanM$ScanTime, 0)) 
        {
          MacScanM$TimIsSet = TRUE;
          return SUCCESS;
        }
    }

  {


    return MacScanM$IMacSCAN$Confirm(MAC_SUCCESS, MacScanM$eMode, MacScanM$ScanChannels, MacScanM$BcnCount, 
    MacScanM$energyDetectList, MacScanM$panDescriptorList);
  }
}

#line 252
static inline  result_t MacScanM$TimerED$Fired(TUniData uniData)
{

  MacScanM$TimIsSet = FALSE;
  return NULL;
}

# 171 "../../../zigzag/ZigRouterM.nc"
static inline  void ZigRouterM$NLME_Leave$indication(IEEEAddr deviceAddr)
{
  if (deviceAddr == info_param.MAC_ADDRESS) {
      ;
      TOS_post(ZigRouterM$restart);
    }

  return;
}

# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
inline static  void AliveSender2M$NLME_Leave$indication(IEEEAddr arg_0x4088a300){
#line 21
  ZigRouterM$NLME_Leave$indication(arg_0x4088a300);
#line 21
}
#line 21
# 285 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline  result_t MacDataM$IMacDataParent$Request(
TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
const uint8_t *const pmsduFrame, 
uint8_t msduHandle, 
TxOptions bmTxOpt)
{
  return MacDataM$DATA_Request(MacDataM$PARENT_SF, 
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
inline static  result_t AliveSender2M$IMacDATA$Request(TMacPANId arg_0x409ca0b8, TMacAddress arg_0x409ca250, TMacPANId arg_0x409ca3f0, TMacAddress arg_0x409ca588, uint8_t arg_0x409ca720, const uint8_t *const arg_0x409ca938, uint8_t arg_0x409caad0, TxOptions arg_0x409cac68){
#line 7
  unsigned char result;
#line 7

#line 7
  result = MacDataM$IMacDataParent$Request(arg_0x409ca0b8, arg_0x409ca250, arg_0x409ca3f0, arg_0x409ca588, arg_0x409ca720, arg_0x409ca938, arg_0x409caad0, arg_0x409cac68);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 57 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline  uint8_t RouterM$newMsduHandle(void)
{
  static uint8_t brr_current_msduHandle = 0;

#line 60
  return ++brr_current_msduHandle;
}

# 12 "../../../zigzag/ZigBee/extra/AliveSender2M.nc"
inline static  uint8_t AliveSender2M$newMsduHandle(void){
#line 12
  unsigned char result;
#line 12

#line 12
  result = RouterM$newMsduHandle();
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 12 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  TMacShortAddress MacDeviceAttrM$IMacAssocAttr$GetmacCoordShortAddress(TMacStatus *const arg_0x40ec4f00){
#line 12
  unsigned int result;
#line 12

#line 12
  result = MacAssocDeviceM$IMacAssocAttr$GetmacCoordShortAddress(arg_0x40ec4f00);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 237 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacShortAddress MacDeviceAttrM$IMacGET$GetmacCoordShortAddress(
TMacStatus *const pMacStatus)
{
  return MacDeviceAttrM$IMacAssocAttr$GetmacCoordShortAddress(pMacStatus);
}

# 27 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacShortAddress AliveSender2M$IMacGET$GetmacCoordShortAddress(TMacStatus *const arg_0x408c0600){
#line 27
  unsigned int result;
#line 27

#line 27
  result = MacDeviceAttrM$IMacGET$GetmacCoordShortAddress(arg_0x408c0600);
#line 27

#line 27
  return result;
#line 27
}
#line 27
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t AliveSender2M$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacAddressM$IMacAddress$SetShortAddress(arg_0x40900a70, arg_0x40900c28);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacShortAddress MacDeviceAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dd29a8){
#line 10
  unsigned int result;
#line 10

#line 10
  result = MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(arg_0x40dd29a8);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 12 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacShortAddress MacDeviceAttrM$IMacGET$GetmacShortAddress(TMacStatus *const pMacStatus)
{
  return MacDeviceAttrM$IMacCommonAttr$GetmacShortAddress(pMacStatus);
}

# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacShortAddress AliveSender2M$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08){
#line 43
  unsigned int result;
#line 43

#line 43
  result = MacDeviceAttrM$IMacGET$GetmacShortAddress(arg_0x408bdf08);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacDeviceAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dd3cf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 2 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacPANId MacDeviceAttrM$IMacGET$GetmacPANId(TMacStatus *const pMacStatus)
{
  return MacDeviceAttrM$IMacCommonAttr$GetmacPANId(pMacStatus);
}

# 37 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacPANId AliveSender2M$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010){
#line 37
  unsigned int result;
#line 37

#line 37
  result = MacDeviceAttrM$IMacGET$GetmacPANId(arg_0x408bd010);
#line 37

#line 37
  return result;
#line 37
}
#line 37
# 65 "../../../zigzag/ZigBee/extra/AliveSender2M.nc"
static inline result_t AliveSender2M$sendEmptyFrame(void)
{
  uint8_t ourMsduHandle;
  PanID_t panid = AliveSender2M$IMacGET$GetmacPANId(NULL);
  TMacAddress myAddr_mac;
  TMacAddress prntAddr_mac;
  TxOptions tx_opts = { .secure = 0, .indirect = 0, .acknowledged = 1, .gts = 0 };

  AliveSender2M$IMacAddress$SetShortAddress(&myAddr_mac, AliveSender2M$IMacGET$GetmacShortAddress(NULL));
  AliveSender2M$IMacAddress$SetShortAddress(&prntAddr_mac, AliveSender2M$IMacGET$GetmacCoordShortAddress(NULL));

  ourMsduHandle = AliveSender2M$newMsduHandle();
  ;
  return AliveSender2M$IMacDATA$Request(panid, myAddr_mac, panid, prntAddr_mac, 0, NULL, ourMsduHandle, tx_opts);
}

#line 43
static inline  result_t AliveSender2M$Timer$Fired(TUniData u)
{
  if (AliveSender2M$f.failCount < info_param.ALIVE_SEND_ATTEMPTS) {
      if (FAIL == AliveSender2M$sendEmptyFrame()) 
        {
          ++ AliveSender2M$f.failCount;
        }
      else 
        {
          AliveSender2M$f.sending = 1;
        }
    }
  else {
      AliveSender2M$NLME_Leave$indication(info_param.MAC_ADDRESS);
      AliveSender2M$Timer$Stop();
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
inline static  result_t ChildSupervisorM$NeighborTable$remove(NwkNeighbor *arg_0x40869360){
#line 32
  unsigned char result;
#line 32

#line 32
  result = NeighborTableM$NeighborTable$remove(arg_0x40869360);
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 85 "../../../zigzag/ZigNetM.nc"
static inline  void ZigNetM$NLME_Leave$indication(IEEEAddr DeviceAddr)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __net_child_leave(DeviceAddr);
    }
#line 89
  return;
}

# 14 "../../../zigzag/ZigMonitorM.nc"
static inline  void ZigMonitorM$NLME_Leave$indication(IEEEAddr deviceAddr)
{
  return;
}

# 21 "../../../zigzag/ZigBee/interface/NLME_Leave.nc"
inline static  void ChildSupervisorM$NLME_Leave$indication(IEEEAddr arg_0x4088a300){
#line 21
  ZigMonitorM$NLME_Leave$indication(arg_0x4088a300);
#line 21
  ZigNetM$NLME_Leave$indication(arg_0x4088a300);
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
inline static  result_t ChildSupervisorM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x4086b790);
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
inline static  result_t TimerSymbol2M$ITimerSymbol$Fired(uint8_t arg_0x40a4dc70, TUniData arg_0x408cc858){
#line 16
  unsigned char result;
#line 16

#line 16
  switch (arg_0x40a4dc70) {
#line 16
    case 0U:
#line 16
      result = NLME_JoinParentM$ITimerSymbol$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 1U:
#line 16
      result = NLME_PermitJoiningM$ITimerSymbol$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 2U:
#line 16
      result = LocalTime64M$ITimerSymbol$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 3U:
#line 16
      result = MacPoolM$IExpiredTimer$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 4U:
#line 16
      result = MacCAPM$IBackoff$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 5U:
#line 16
      result = MacDataM$AckWaitTimer$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 6U:
#line 16
      result = MacSyncM$InitialTimer$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 7U:
#line 16
      result = MacAssocDeviceM$IAssocAckTimer$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 8U:
#line 16
      result = MacAssocDeviceM$IReceiveResponseTimer$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 9U:
#line 16
      result = MacExtractM$IAckWaitTimer$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 10U:
#line 16
      result = MacScanM$TimerED$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 11U:
#line 16
      result = MacScanM$TimerBeaconScan$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 12U:
#line 16
      result = AliveSender2M$Timer$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    case 13U:
#line 16
      result = ChildSupervisorM$ITimerSymbol$Fired(arg_0x408cc858);
#line 16
      break;
#line 16
    default:
#line 16
      result = TimerSymbol2M$ITimerSymbol$default$Fired(arg_0x40a4dc70, arg_0x408cc858);
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
# 19 "../../../zigzag/ZigBee/interface/NLME_NetworkDiscovery.nc"
inline static  void NLME_NetworkDiscoveryM$NLME_NetworkDiscovery$confirm(uint8_t arg_0x4086d990, NwkNetworkDescriptor *arg_0x4086db60, NwkStatus arg_0x4086dd08){
#line 19
  ZigRouterM$NLME_NetworkDiscovery$confirm(arg_0x4086d990, arg_0x4086db60, arg_0x4086dd08);
#line 19
}
#line 19
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacScanM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(6U, arg_0x40b27940, arg_0x40b27ae8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacScanM$TimerBeaconScan$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(11U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
inline static  result_t MacScanM$TimerED$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(10U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 51 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanM.nc"
static inline  result_t MacScanM$IMacSCAN$Request(TMacScanType scanType, 
uint32_t scanChannels, 
uint8_t scanDuration)
{
  uint8_t i;

  MacScanM$TimerED$Stop();
  MacScanM$TimerBeaconScan$Stop();

  MacScanM$ScanChannels = scanChannels;
  MacScanM$TimIsSet = FALSE;
  MacScanM$eMode = scanType;
  MacScanM$bTrOn = TRUE;

  MacScanM$CurBit = 0;
  MacScanM$BcnCount = 0;

  MacScanM$ScanTime = ((uint32_t )MAC_ABASE_SUPERFRAME_DURATION << scanDuration) + MAC_ABASE_SUPERFRAME_DURATION;

  for (i = 0; i < 27; i++) 
    MacScanM$energyDetectList[i] = 0;








  MacScanM$nextBit();

  MacScanM$IPhySET$SetphyCurrentChannel(MacScanM$CurBit);


  return MacScanM$IPhySET_TRX_STATE$Request(PHY_RX_ON, 0);
}

# 7 "../../../zigzag/IEEE802_15_4/MacScan/public/IMacSCAN.nc"
inline static  result_t NLME_NetworkDiscoveryM$IMacSCAN$Request(TMacScanType arg_0x4091f198, uint32_t arg_0x4091f338, uint8_t arg_0x4091f4d0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = MacScanM$IMacSCAN$Request(arg_0x4091f198, arg_0x4091f338, arg_0x4091f4d0);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 30 "../../../zigzag/ZigBee/implementation/NLME_NetworkDiscoveryM.nc"
static inline  void NLME_NetworkDiscoveryM$NLME_NetworkDiscovery$request(
uint32_t scanChannels, 
uint8_t scanDuration)

{


  if (!NLME_NetworkDiscoveryM$IMacSCAN$Request(MAC_ACTIVE_SCAN, scanChannels, scanDuration)) {
    NLME_NetworkDiscoveryM$NLME_NetworkDiscovery$confirm(0, NULL, MAC_CHANNEL_ACCESS_FAILURE);
    }
}

# 15 "../../../zigzag/ZigBee/interface/NLME_NetworkDiscovery.nc"
inline static  void ZigRouterM$NLME_NetworkDiscovery$request(uint32_t arg_0x4086d358, uint8_t arg_0x4086d500){
#line 15
  NLME_NetworkDiscoveryM$NLME_NetworkDiscovery$request(arg_0x4086d358, arg_0x4086d500);
#line 15
}
#line 15
# 109 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setNwkMaxDepth(uint8_t attr)
{
  NIBM$nwkMaxDepth = attr;
}

# 55 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void ZigRouterM$NIB$setNwkMaxDepth(uint8_t arg_0x4087d550){
#line 55
  NIBM$NIB$setNwkMaxDepth(arg_0x4087d550);
#line 55
}
#line 55
# 119 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setNwkMaxRouters(uint8_t attr)
{
  NIBM$nwkMaxRouters = attr;
}

# 65 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void ZigRouterM$NIB$setNwkMaxRouters(uint8_t arg_0x4087dcc8){
#line 65
  NIBM$NIB$setNwkMaxRouters(arg_0x4087dcc8);
#line 65
}
#line 65
# 99 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setNwkMaxChildren(uint8_t attr)
{
  NIBM$nwkMaxChildren = attr;
}

# 47 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void ZigRouterM$NIB$setNwkMaxChildren(uint8_t arg_0x4087edc0){
#line 47
  NIBM$NIB$setNwkMaxChildren(arg_0x4087edc0);
#line 47
}
#line 47
# 70 "../../../zigzag/ZigRouterM.nc"
static inline  void ZigRouterM$start_discovery(void)
{
  ZigRouterM$NIB$setNwkMaxChildren(info_param.Z_MAX_CHILDREN);
  ZigRouterM$NIB$setNwkMaxRouters(info_param.Z_MAX_ROUTERS);
  ZigRouterM$NIB$setNwkMaxDepth(info_param.Z_MAX_DEPTH);
  ZigRouterM$idle_scan = FALSE;
  ZigRouterM$NLME_NetworkDiscovery$request(info_param.Z_CHANNELS, info_param.Z_SCAN_ORDER);

  return;
}

#line 63
static inline  result_t ZigRouterM$TimerMilli$fired(void)
{
  TOS_post(ZigRouterM$start_discovery);

  return SUCCESS;
}

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
inline static  result_t TimerSymbol2M$TimerMilli$fired(uint8_t arg_0x40a4c810){
#line 37
  unsigned char result;
#line 37

#line 37
  switch (arg_0x40a4c810) {
#line 37
    case 0U:
#line 37
      result = ZigRouterM$TimerMilli$fired();
#line 37
      break;
#line 37
    case 1U:
#line 37
      result = ZigSTimerM$Timer0$fired();
#line 37
      break;
#line 37
    case 2U:
#line 37
      result = ZigSTimerM$Timer1$fired();
#line 37
      break;
#line 37
    case 3U:
#line 37
      result = ZigSTimerM$Timer2$fired();
#line 37
      break;
#line 37
    default:
#line 37
      result = TimerSymbol2M$TimerMilli$default$fired(arg_0x40a4c810);
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

# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t ZigRouterM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x4086b790);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 571 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline  void RouterM$NLME_JoinChild$confirm(TMacPANId panid, NwkStatus status)
{
  if (status == NWK_SUCCESS) {
    TOS_post(RouterM$relayNext);
    }
}

# 234 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static inline  void NwkAddressingM$NLME_JoinChild$confirm(TMacPANId panid, NwkStatus status)
{
  if (status == NWK_SUCCESS) 
    {
      NwkAddressingM$Reset$reset();
    }
}

# 27 "../../../zigzag/ZigBee/interface/NLME_JoinChild.nc"
inline static  void NLME_JoinChildM$NLME_JoinChild$confirm(PanID_t arg_0x40870010, NwkStatus arg_0x408701b8){
#line 27
  NwkAddressingM$NLME_JoinChild$confirm(arg_0x40870010, arg_0x408701b8);
#line 27
  RouterM$NLME_JoinChild$confirm(arg_0x40870010, arg_0x408701b8);
#line 27
  ZigRouterM$NLME_JoinChild$confirm(arg_0x40870010, arg_0x408701b8);
#line 27
  ZigNetM$NLME_JoinChild$confirm(arg_0x40870010, arg_0x408701b8);
#line 27
}
#line 27
# 169 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setNwkCapabilityInformation(NwkCapabilityInfo attr)
{
  NIBM$nwkCapabilityInformation = attr;
}

# 116 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NLME_JoinChildM$NIB$setNwkCapabilityInformation(NwkCapabilityInfo arg_0x40879398){
#line 116
  NIBM$NIB$setNwkCapabilityInformation(arg_0x40879398);
#line 116
}
#line 116
# 154 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(bool macRxOnWhenIdle)
{
  MacCommonAttrM$pib.macRxOnWhenIdle = macRxOnWhenIdle;
  return MAC_SUCCESS;
}

# 31 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(bool arg_0x40dcdf00){
#line 31
  enum __nesc_unnamed4286 result;
#line 31

#line 31
  result = MacCommonAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(arg_0x40dcdf00);
#line 31

#line 31
  return result;
#line 31
}
#line 31
# 155 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacRxOnWhenIdle(
bool macRxOnWhenIdle)
{
  return MacDeviceAttrM$IMacCommonAttr$SetmacRxOnWhenIdle(macRxOnWhenIdle);
}

# 40 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_JoinChildM$IMacSET$SetmacRxOnWhenIdle(bool arg_0x408eade0){
#line 40
  enum __nesc_unnamed4286 result;
#line 40

#line 40
  result = MacDeviceAttrM$IMacSET$SetmacRxOnWhenIdle(arg_0x408eade0);
#line 40

#line 40
  return result;
#line 40
}
#line 40
# 272 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  bool NIBM$NIB$isJoined(void)
#line 272
{
#line 272
  return NIBM$onlineStatus != NWK_OFFLINE;
}

# 193 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  bool NLME_JoinChildM$NIB$isJoined(void){
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
# 46 "../../../zigzag/ZigBee/implementation/NLME_JoinChildM.nc"
static inline  void NLME_JoinChildM$NLME_JoinChild$request(
PanID_t panID, 
bool joinAsRouter, 
bool rejoinNetwork, 
IEEEAddr *joinToSpecifiedParent, 
uint32_t scanChannels, 
uint8_t scanDuration, 
uint8_t powerSource, 
bool rxOnWhenIdle, 
bool macSecurity)

{

  NwkCapabilityInfo capability;

  NLME_JoinChildM$request_panID = panID;
  NLME_JoinChildM$request_joinAsRouter = joinAsRouter;

  if (NLME_JoinChildM$NIB$isJoined()) 
    {

      NLME_JoinChildM$NLME_JoinChild$confirm(panID, NWK_INVALID_REQUEST);
      return;
    }


  NLME_JoinChildM$IMacSET$SetmacRxOnWhenIdle(rxOnWhenIdle);

  capability.alternatePANCoordinator = 0;
  capability.deviceType = joinAsRouter ? 1 : 0;
  capability.powerSource = powerSource & 0x01;
  capability.receiverOnWhenIdle = rxOnWhenIdle ? 1 : 0;
  capability.securityCapability = macSecurity ? 1 : 0;
  capability.allocateAddress = 1;

  NLME_JoinChildM$NIB$setNwkCapabilityInformation(capability);




  if (joinToSpecifiedParent != NULL) 
    {
      NLME_JoinChildM$joinToSpecified = TRUE;
      NLME_JoinChildM$joinAddress = *joinToSpecifiedParent;
    }
  else 
    {
      NLME_JoinChildM$joinToSpecified = FALSE;
    }

  NLME_JoinChildM$confirm_status = NWK_NOT_PERMITTED;
  if (!TOS_post(NLME_JoinChildM$try_to_assoc)) {
    NLME_JoinChildM$NLME_JoinChild$confirm(NLME_JoinChildM$request_panID, NWK_INVALID_REQUEST);
    }
}

# 15 "../../../zigzag/ZigBee/interface/NLME_JoinChild.nc"
inline static  void ZigRouterM$NLME_JoinChild$request(PanID_t arg_0x4085bd90, bool arg_0x4085bf28, bool arg_0x408730d8, IEEEAddr *arg_0x408732a0, uint32_t arg_0x40873440, uint8_t arg_0x408735d8, uint8_t arg_0x40873770, bool arg_0x40873908, bool arg_0x40873ab0){
#line 15
  NLME_JoinChildM$NLME_JoinChild$request(arg_0x4085bd90, arg_0x4085bf28, arg_0x408730d8, arg_0x408732a0, arg_0x40873440, arg_0x408735d8, arg_0x40873770, arg_0x40873908, arg_0x40873ab0);
#line 15
}
#line 15
# 105 "../../../zigzag/ZigRouterM.nc"
static inline  void ZigRouterM$join(void)
{
  ZigRouterM$NLME_JoinChild$request(
  info_param.Z_PAN_ID, 
  TRUE, 
  FALSE, 
  NULL, 
  0, 
  0, 
  0, 
  TRUE, 
  FALSE);

  return;
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
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacCoordAttrM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dd3cf0);
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
inline static  TMacPANId RouterM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010){
#line 37
  unsigned int result;
#line 37

#line 37
  result = MacCoordAttrM$IMacGET$GetmacPANId(arg_0x408bd010);
#line 37

#line 37
  return result;
#line 37
}
#line 37
# 29 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t RouterM$NeighborTable$getByAddr(TMacAddress arg_0x4086bc20, NwkNeighbor **arg_0x4086bdf0){
#line 29
  unsigned char result;
#line 29

#line 29
  result = NeighborTableM$NeighborTable$getByAddr(arg_0x4086bc20, arg_0x4086bdf0);
#line 29

#line 29
  return result;
#line 29
}
#line 29
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t RouterM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacAddressM$IMacAddress$SetShortAddress(arg_0x40900a70, arg_0x40900c28);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t RouterM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x4086b790);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 274 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  uint8_t NIBM$NIB$getDepth(void)
#line 274
{
#line 274
  return NIBM$depth;
}

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
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacShortAddress MacCoordAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dd29a8){
#line 10
  unsigned int result;
#line 10

#line 10
  result = MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(arg_0x40dd29a8);
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
inline static  TMacShortAddress NwkAddressingM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08){
#line 43
  unsigned int result;
#line 43

#line 43
  result = MacCoordAttrM$IMacGET$GetmacShortAddress(arg_0x408bdf08);
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
inline static  void RouterM$NwkAddressing$getDirection(NwkAddr arg_0x408d4928, NwkRelationship *arg_0x408d4ad8, NwkAddr *arg_0x408d4c78){
#line 25
  NwkAddressingM$NwkAddressing$getDirection(arg_0x408d4928, arg_0x408d4ad8, arg_0x408d4c78);
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

# 8 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacAddressM.nc"
static inline  TMacAddressMode MacAddressM$IMacAddress$GetAddressMode(const TMacAddress macAddress)
{
  return ((_TMacAddress )macAddress).mode;
}

# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  TMacAddressMode NeighborTableM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8){
#line 6
  enum __nesc_unnamed4299 result;
#line 6

#line 6
  result = MacAddressM$IMacAddress$GetAddressMode(arg_0x409017d8);
#line 6

#line 6
  return result;
#line 6
}
#line 6


inline static  result_t NeighborTableM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacAddressM$IMacAddress$GetShortAddress(arg_0x40901ce0, arg_0x40901e90);
#line 8

#line 8
  return result;
#line 8
}
#line 8


inline static  result_t NeighborTableM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacAddressM$IMacAddress$GetExtendedAddress(arg_0x409003c0, arg_0x40900570);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 7 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t RouterM$UpperIf$Request(TMacPANId arg_0x409ca0b8, TMacAddress arg_0x409ca250, TMacPANId arg_0x409ca3f0, TMacAddress arg_0x409ca588, uint8_t arg_0x409ca720, const uint8_t *const arg_0x409ca938, uint8_t arg_0x409caad0, TxOptions arg_0x409cac68){
#line 7
  unsigned char result;
#line 7

#line 7
  result = MacDataM$IMacDataParent$Request(arg_0x409ca0b8, arg_0x409ca250, arg_0x409ca3f0, arg_0x409ca588, arg_0x409ca720, arg_0x409ca938, arg_0x409caad0, arg_0x409cac68);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 489 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline  result_t RouterM$UpperIf$Confirm(uint8_t msduHandle, TMacStatus status)
{
  return RouterM$IMacDATA_Confirm(msduHandle, status);
}

# 34 "../../../zigzag/ZigBee/extra/AliveSender2M.nc"
static inline  result_t AliveSender2M$IMacDATA$Confirm(uint8_t msdu_L_Handle, TMacStatus MacStatus)
{
  if (MAC_SUCCESS == MacStatus) {
    return AliveSender2M$restartTimer();
    }
  else {
#line 38
    if (AliveSender2M$f.sending) {
      ++ AliveSender2M$f.failCount, AliveSender2M$f.sending = 0;
      }
    }
#line 40
  return SUCCESS;
}

# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t MacDataM$IMacDataParent$Confirm(uint8_t arg_0x409e8348, TMacStatus arg_0x409e84d8){
#line 26
  unsigned char result;
#line 26

#line 26
  result = AliveSender2M$IMacDATA$Confirm(arg_0x409e8348, arg_0x409e84d8);
#line 26
  result = rcombine(result, RouterM$UpperIf$Confirm(arg_0x409e8348, arg_0x409e84d8));
#line 26

#line 26
  return result;
#line 26
}
#line 26
# 11 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  bool AliveSender2M$Timer$IsSet(void){
#line 11
  unsigned char result;
#line 11

#line 11
  result = TimerSymbol2M$ITimerSymbol$IsSet(12U);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 313 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static inline  result_t TimerSymbol2M$ITimerSymbol$SetPeriodic(uint8_t num, TSysTime symbols, TUniData uniData)
{
  return TimerSymbol2M$SetTimer(num, TimerSymbol2M$ITimeCast$SymbolsToJiffies(symbols), TRUE, uniData);
}

# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t AliveSender2M$Timer$SetPeriodic(TSysTime arg_0x408cdca8, TUniData arg_0x408cde30){
#line 6
  unsigned char result;
#line 6

#line 6
  result = TimerSymbol2M$ITimerSymbol$SetPeriodic(12U, arg_0x408cdca8, arg_0x408cde30);
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
inline static   TSysTime AliveSender2M$ITimeCast$MillisToSymbols(TMilliSec arg_0x40a3e6e0){
#line 6
  unsigned long result;
#line 6

#line 6
  result = TimeCastM$ITimeCast$MillisToSymbols(arg_0x40a3e6e0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 494 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline  result_t RouterM$LowerIf$Confirm(uint8_t msduHandle, TMacStatus status)
{
  return RouterM$IMacDATA_Confirm(msduHandle, status);
}

# 26 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t MacDataM$IMacDataOwn$Confirm(uint8_t arg_0x409e8348, TMacStatus arg_0x409e84d8){
#line 26
  unsigned char result;
#line 26

#line 26
  result = RouterM$LowerIf$Confirm(arg_0x409e8348, arg_0x409e84d8);
#line 26

#line 26
  return result;
#line 26
}
#line 26
# 5 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacFrameFormatM$IMacFrame$SetFrameType(arg_0x40db68e8, arg_0x40db6a98);
#line 5

#line 5
  return result;
#line 5
}
#line 5






inline static  result_t MacDataM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacFrameFormatM$IMacFrame$SetFramePending(arg_0x40db4510, arg_0x40db46b8);
#line 11

#line 11
  return result;
#line 11
}
#line 11
#line 8
inline static  result_t MacDataM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacFrameFormatM$IMacFrame$SetSecurityEnabled(arg_0x40db5708, arg_0x40db58b0);
#line 8

#line 8
  return result;
#line 8
}
#line 8









inline static  result_t MacDataM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250){
#line 17
  unsigned char result;
#line 17

#line 17
  result = MacFrameFormatM$IMacFrame$SetIntraPAN(arg_0x40db10a8, arg_0x40db1250);
#line 17

#line 17
  return result;
#line 17
}
#line 17
#line 30
inline static  result_t MacDataM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40dac908, const TMacPANId arg_0x40dacab8){
#line 30
  unsigned char result;
#line 30

#line 30
  result = MacFrameFormatM$IMacFrame$SetSrcPANId(arg_0x40dac908, arg_0x40dacab8);
#line 30

#line 30
  return result;
#line 30
}
#line 30



inline static  result_t MacDataM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0){
#line 33
  unsigned char result;
#line 33

#line 33
  result = MacFrameFormatM$IMacFrame$SetSrcAddress(arg_0x40dab700, arg_0x40dab8b0);
#line 33

#line 33
  return result;
#line 33
}
#line 33
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  TMacAddressMode MacFrameFormatM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8){
#line 6
  enum __nesc_unnamed4299 result;
#line 6

#line 6
  result = MacAddressM$IMacAddress$GetAddressMode(arg_0x409017d8);
#line 6

#line 6
  return result;
#line 6
}
#line 6


inline static  result_t MacFrameFormatM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacAddressM$IMacAddress$GetShortAddress(arg_0x40901ce0, arg_0x40901e90);
#line 8

#line 8
  return result;
#line 8
}
#line 8


inline static  result_t MacFrameFormatM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacAddressM$IMacAddress$GetExtendedAddress(arg_0x409003c0, arg_0x40900570);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 24 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40dafc60, const TMacPANId arg_0x40dafe10){
#line 24
  unsigned char result;
#line 24

#line 24
  result = MacFrameFormatM$IMacFrame$SetDstPANId(arg_0x40dafc60, arg_0x40dafe10);
#line 24

#line 24
  return result;
#line 24
}
#line 24



inline static  result_t MacDataM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8){
#line 27
  unsigned char result;
#line 27

#line 27
  result = MacFrameFormatM$IMacFrame$SetDstAddress(arg_0x40daeb38, arg_0x40daece8);
#line 27

#line 27
  return result;
#line 27
}
#line 27
# 18 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacDSN MacDataM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dcf3a0){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacCommonAttrM$IMacCommonAttr$GetmacDSN(arg_0x40dcf3a0);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 20 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacFrameFormatM$IMacFrame$SetSequenceNumber(arg_0x40db1e60, arg_0x40daf030);
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 32 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static inline  TMacStatus MacCommonAttrM$IMacCommonAttr$SetmacDSN(TMacDSN macDSN)
{
  MacCommonAttrM$pib.macDSN = macDSN;
  return MAC_SUCCESS;
}

# 19 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacDataM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dcf828){
#line 19
  enum __nesc_unnamed4286 result;
#line 19

#line 19
  result = MacCommonAttrM$IMacCommonAttr$SetmacDSN(arg_0x40dcf828);
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 36 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0){
#line 36
  unsigned char result;
#line 36

#line 36
  result = MacFrameFormatM$IMacFrame$SetPayload(arg_0x40da9510, arg_0x40da9728, arg_0x40da98c0);
#line 36

#line 36
  return result;
#line 36
}
#line 36
#line 14
inline static  result_t MacDataM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacFrameFormatM$IMacFrame$SetAckRequest(arg_0x40db22d8, arg_0x40db2480);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  result_t MacDataM$IMacSendParent$Send(const TMacFrame *const arg_0x40f051f8, const TUniData arg_0x40f053b0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$Send(6U, arg_0x40f051f8, arg_0x40f053b0);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 12 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacCAPM$IPhyPool$Free(const TPhyPoolHandle arg_0x40b3dbc0){
#line 12
  unsigned char result;
#line 12

#line 12
  result = PhyPoolM$IPhyPool$Free(arg_0x40b3dbc0);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 44 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  TMacRawFrameLength MacCAPM$IMacFrame$Pack(const TMacFrame *const arg_0x40da8c38, TPhyFrame *const arg_0x40da8e28){
#line 44
  unsigned char result;
#line 44

#line 44
  result = MacFrameFormatM$IMacFrame$Pack(arg_0x40da8c38, arg_0x40da8e28);
#line 44

#line 44
  return result;
#line 44
}
#line 44
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *MacCAPM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010){
#line 8
  struct __nesc_unnamed4281 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b3d010);
#line 8

#line 8
  return result;
#line 8
}
#line 8
#line 5
inline static   result_t MacCAPM$IPhyPool$Alloc(const uint8_t arg_0x40b3e6b8, const uint16_t arg_0x40b3e860, TPhyPoolHandle *const arg_0x40b3ea58){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyPoolM$IPhyPool$Alloc(arg_0x40b3e6b8, arg_0x40b3e860, arg_0x40b3ea58);
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
inline static   result_t PhyPoolM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b45578){
#line 19
  unsigned char result;
#line 19

#line 19
  result = PhyFrameM$IPhyFrame$Erase(arg_0x40b45578);
#line 19

#line 19
  return result;
#line 19
}
#line 19
inline static   result_t MacFrameFormatM$IPhyFrame$Erase(TPhyFrame *const arg_0x40b45578){
#line 19
  unsigned char result;
#line 19

#line 19
  result = PhyFrameM$IPhyFrame$Erase(arg_0x40b45578);
#line 19

#line 19
  return result;
#line 19
}
#line 19






inline static   result_t MacFrameFormatM$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b44528, TPhyTimeStamp arg_0x40b446b0){
#line 25
  unsigned char result;
#line 25

#line 25
  result = PhyFrameM$IPhyFrame$SetTimeStamp(arg_0x40b44528, arg_0x40b446b0);
#line 25

#line 25
  return result;
#line 25
}
#line 25
#line 13
inline static   result_t MacFrameFormatM$IPhyFrame$AppendOctet(TPhyFrame *const arg_0x40b47270, const uint8_t arg_0x40b47410){
#line 13
  unsigned char result;
#line 13

#line 13
  result = PhyFrameM$IPhyFrame$AppendOctet(arg_0x40b47270, arg_0x40b47410);
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
inline static   result_t MacFrameFormatM$IPhyFrame$CalcCRC(TPhyFrame *const arg_0x40b45a88){
#line 21
  unsigned char result;
#line 21

#line 21
  result = PhyFrameM$IPhyFrame$CalcCRC(arg_0x40b45a88);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacCAPM$IBackoff$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(4U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 16 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
inline static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Send(const uint8_t arg_0x40f93c88, const TMacFrame *const arg_0x40f93ea8, const uint8_t arg_0x40f92088, const TUniData arg_0x40f92240){
#line 16
  unsigned char result;
#line 16

#line 16
  result = MacCAPM$Send(arg_0x40f93c88, arg_0x40f93ea8, arg_0x40f92088, arg_0x40f92240);
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
inline static  result_t MacDataM$IMacSendOwn$Send(const TMacFrame *const arg_0x40f051f8, const TUniData arg_0x40f053b0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(2U, arg_0x40f051f8, arg_0x40f053b0);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 308 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
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
inline static  result_t RouterM$LowerIf$Request(TMacPANId arg_0x409ca0b8, TMacAddress arg_0x409ca250, TMacPANId arg_0x409ca3f0, TMacAddress arg_0x409ca588, uint8_t arg_0x409ca720, const uint8_t *const arg_0x409ca938, uint8_t arg_0x409caad0, TxOptions arg_0x409cac68){
#line 7
  unsigned char result;
#line 7

#line 7
  result = MacDataM$IMacDataOwn$Request(arg_0x409ca0b8, arg_0x409ca250, arg_0x409ca3f0, arg_0x409ca588, arg_0x409ca720, arg_0x409ca938, arg_0x409caad0, arg_0x409cac68);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 62 "../../../zigzag/ZigBee/extra/AliveSender2M.nc"
static inline  result_t AliveSender2M$StdControl$start(void)
#line 62
{
#line 62
  return AliveSender2M$restartTimer();
}

# 70 "/home/max/tinyos/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t ZigRouterM$AliveSenderControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = AliveSender2M$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 280 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  uint8_t NIBM$NIB$getChannel(void)
#line 280
{
#line 280
  return NIBM$channel;
}

# 200 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NLME_StartRouterM$NIB$getChannel(void){
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
# 37 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacPANId NLME_StartRouterM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010){
#line 37
  unsigned int result;
#line 37

#line 37
  result = MacCoordAttrM$IMacGET$GetmacPANId(arg_0x408bd010);
#line 37

#line 37
  return result;
#line 37
}
#line 37
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
inline static  result_t MacStartM$IPhyAttr$SetAutoAck(bool arg_0x40b40ec0){
#line 11
  unsigned char result;
#line 11

#line 11
  result = PhyCC2420M$IPhyAttr$SetAutoAck(arg_0x40b40ec0);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 22 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSTART.nc"
inline static  result_t MacStartM$IMacSTART$Confirm(TMacStatus arg_0x40b13338){
#line 22
  unsigned char result;
#line 22

#line 22
  result = NLME_StartRouterM$IMacSTART$Confirm(arg_0x40b13338);
#line 22

#line 22
  return result;
#line 22
}
#line 22
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacStartM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(3U, arg_0x40b27940, arg_0x40b27ae8);
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
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacSuperframesM$IPhyPool$Alloc(const uint8_t arg_0x40b3e6b8, const uint16_t arg_0x40b3e860, TPhyPoolHandle *const arg_0x40b3ea58){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyPoolM$IPhyPool$Alloc(arg_0x40b3e6b8, arg_0x40b3e860, arg_0x40b3ea58);
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
inline static  result_t MacStartM$runOwnSF(TSysTime arg_0x41063ce8){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacSuperframesM$runOwnSF(arg_0x41063ce8);
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
inline static  TPhyStatus MacStartM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b2a360){
#line 5
  enum __nesc_unnamed4242 result;
#line 5

#line 5
  result = PhyCC2420M$IPhySET$SetphyCurrentChannel(arg_0x40b2a360);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacStartM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dd2198){
#line 7
  enum __nesc_unnamed4286 result;
#line 7

#line 7
  result = MacCommonAttrM$IMacCommonAttr$SetmacPANId(arg_0x40dd2198);
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
inline static  TMacStatus MacStartM$IMacSuperframeAttr$SetmacSuperframeOrder(TMacSuperframeOrder arg_0x40e55cd8){
#line 11
  enum __nesc_unnamed4286 result;
#line 11

#line 11
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetmacSuperframeOrder(arg_0x40e55cd8);
#line 11

#line 11
  return result;
#line 11
}
#line 11
#line 7
inline static  TMacStatus MacStartM$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e55010){
#line 7
  enum __nesc_unnamed4286 result;
#line 7

#line 7
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(arg_0x40e55010);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacShortAddress MacStartM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dd29a8){
#line 10
  unsigned int result;
#line 10

#line 10
  result = MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(arg_0x40dd29a8);
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
inline static  result_t NLME_StartRouterM$IMacSTART$Request(TMacPANId arg_0x40b14110, TMacLogicalChannel arg_0x40b142b8, TMacBeaconOrder arg_0x40b14458, TMacSuperframeOrder arg_0x40b14600, bool arg_0x40b14798, bool arg_0x40b14938, bool arg_0x40b14ad8, bool arg_0x40b14c70, TSysTime arg_0x40b14e20){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacStartM$IMacSTART$Request(arg_0x40b14110, arg_0x40b142b8, arg_0x40b14458, arg_0x40b14600, arg_0x40b14798, arg_0x40b14938, arg_0x40b14ad8, arg_0x40b14c70, arg_0x40b14e20);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 26 "../../../zigzag/ZigBee/implementation/NLME_StartRouterM.nc"
inline static  void NLME_StartRouterM$updateBeacon(void){
#line 26
  NwkBeaconParentM$updateBeacon();
#line 26
}
#line 26
# 285 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setBeaconOffset(TSysTime offs)
#line 285
{
#line 285
  NIBM$beaconOffset = offs;
}

# 204 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NLME_StartRouterM$NIB$setBeaconOffset(TSysTime arg_0x4088e530){
#line 204
  NIBM$NIB$setBeaconOffset(arg_0x4088e530);
#line 204
}
#line 204
# 22 "../../../zigzag/ZigBee/interface/NLME_StartRouter.nc"
inline static  void NLME_StartRouterM$NLME_StartRouter$confirm(NwkStatus arg_0x4086fd88){
#line 22
  ZigRouterM$NLME_StartRouter$confirm(arg_0x4086fd88);
#line 22
}
#line 22
# 17 "../../../zigzag/ZigBee/implementation/NwkBeaconOffsetRandomM.nc"
static inline bool NwkBeaconOffsetRandomM$overlap(uint32_t a, uint32_t b)
{
  ;
  return b - a < 256 || a - b < 256;
}

# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t NwkBeaconOffsetRandomM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x4086b790);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 3 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacRandom.nc"
inline static   uint16_t NwkBeaconOffsetRandomM$IMacRandom$Rand(void){
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
# 44 "../../../zigzag/ZigBee/implementation/NwkBeaconOffsetRandomM.nc"
static inline  result_t NwkBeaconOffsetRandomM$findOffset(uint8_t beaconOrder, uint8_t superframeOrder, TSysTime *offset)
{
  uint32_t BeaconInterval;
  uint16_t SFDuration;
  TSysTime parentTime = 0;
  uint16_t numVariants;
  NwkNeighbor *pnbr;
  bool parentFound = FALSE;

  BeaconInterval = (uint32_t )MAC_ABASE_SUPERFRAME_DURATION << beaconOrder;
  SFDuration = (uint32_t )MAC_ABASE_SUPERFRAME_DURATION << superframeOrder;




  pnbr = NULL;
  while (NwkBeaconOffsetRandomM$NeighborTable$getNextPtr(&pnbr) == SUCCESS) 
    {

      if (!parentFound && pnbr->relationship == NWK_PARENT) 
        {
          parentFound = TRUE;
          parentTime = pnbr->incomingBeaconTimestamp;
        }
    }

  if (!parentFound) {
    return FALSE;
    }

  {
    uint8_t numTries = 4;
    uint32_t aTry;
    uint16_t rnd;
    bool tryMore = TRUE;

    numVariants = 1 << (beaconOrder - superframeOrder);

    while (numTries > 0 && tryMore) 
      {
        numTries--;
        rnd = NwkBeaconOffsetRandomM$IMacRandom$Rand() % (numVariants - 3) + 2;


        ;



        aTry = 0;
        while (rnd) 
          {
            aTry += SFDuration;
            rnd--;
          }


        ;

        tryMore = FALSE;
        pnbr = NULL;
        while (NwkBeaconOffsetRandomM$NeighborTable$getNextPtr(&pnbr) == SUCCESS) 
          {

            if (
#line 106
            NwkBeaconOffsetRandomM$overlap(aTry, NwkBeaconOffsetRandomM$positive_residual(pnbr->incomingBeaconTimestamp - parentTime, BeaconInterval))
             || NwkBeaconOffsetRandomM$overlap(aTry, NwkBeaconOffsetRandomM$positive_residual(pnbr->incomingBeaconTimestamp - pnbr->beaconTransmissionTimeOffset - parentTime, BeaconInterval))) 

              {
                tryMore = TRUE;
                break;
              }
          }
      }

    if (tryMore) {
      return FALSE;
      }
    else {
        *offset = aTry;
        return TRUE;
      }
  }
}

# 27 "../../../zigzag/ZigBee/implementation/NLME_StartRouterM.nc"
inline static  result_t NLME_StartRouterM$findOffset(uint8_t arg_0x4128f970, uint8_t arg_0x4128faf8, TSysTime *arg_0x4128fca0){
#line 27
  unsigned char result;
#line 27

#line 27
  result = NwkBeaconOffsetRandomM$findOffset(arg_0x4128f970, arg_0x4128faf8, arg_0x4128fca0);
#line 27

#line 27
  return result;
#line 27
}
#line 27
# 56 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  uint8_t NLME_StartRouterM$NIB$getNwkMaxDepth(void){
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
#line 196
inline static  uint8_t NLME_StartRouterM$NIB$getDepth(void){
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
inline static  TMacSuperframeOrder MacCoordAttrM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e55828){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(arg_0x40e55828);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 92 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacSuperframeOrder MacCoordAttrM$IMacGET$GetmacSuperframeOrder(
TMacStatus *const pMacStatus)
{
  return MacCoordAttrM$IMacSuperframeAttr$GetmacSuperframeOrder(pMacStatus);
}

# 45 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacSuperframeOrder NLME_StartRouterM$IMacGET$GetmacSuperframeOrder(TMacStatus *const arg_0x408d9440){
#line 45
  unsigned char result;
#line 45

#line 45
  result = MacCoordAttrM$IMacGET$GetmacSuperframeOrder(arg_0x408d9440);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacCoordAttrM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e57ab8);
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
inline static  TMacBeaconOrder NLME_StartRouterM$IMacGET$GetmacBeaconOrder(TMacStatus *const arg_0x408c11d8){
#line 19
  unsigned char result;
#line 19

#line 19
  result = MacCoordAttrM$IMacGET$GetmacBeaconOrder(arg_0x408c11d8);
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
inline static  NwkOnlineStatus NLME_StartRouterM$NIB$getOnlineStatus(void){
#line 192
  enum __nesc_unnamed4340 result;
#line 192

#line 192
  result = NIBM$NIB$getOnlineStatus();
#line 192

#line 192
  return result;
#line 192
}
#line 192
# 34 "../../../zigzag/ZigBee/implementation/NLME_StartRouterM.nc"
static inline  void NLME_StartRouterM$NLME_StartRouter$request(
uint16_t slotNumber, 
bool batteryLifeExtension)

{
  TSysTime beaconsOffset = 0;
  uint8_t beaconOrder;
  uint8_t superframeOrder;

#line 42
  if (NLME_StartRouterM$NIB$getOnlineStatus() != NWK_JOINED_AS_ROUTER) 
    {
      ;
      NLME_StartRouterM$NLME_StartRouter$confirm(NWK_INVALID_REQUEST);
      return;
    }

  beaconOrder = NLME_StartRouterM$IMacGET$GetmacBeaconOrder(NULL);
  superframeOrder = NLME_StartRouterM$IMacGET$GetmacSuperframeOrder(NULL);

  if (beaconOrder != 15) 
    {
      if (NLME_StartRouterM$NIB$getDepth() == NLME_StartRouterM$NIB$getNwkMaxDepth()) 
        {


          NLME_StartRouterM$NLME_StartRouter$confirm(NWK_INVALID_REQUEST);
          return;
        }

      if (slotNumber == 0) 
        {

          if (NLME_StartRouterM$findOffset(beaconOrder, superframeOrder, &beaconsOffset) == FAIL) 
            {

              NLME_StartRouterM$NLME_StartRouter$confirm(NWK_INVALID_REQUEST);
              return;
            }
        }
      else 
        {

          uint16_t i;
          TSysTime superframeDuration;

          if (slotNumber >= (uint16_t )1 << (beaconOrder - superframeOrder)) 
            {

              NLME_StartRouterM$NLME_StartRouter$confirm(NWK_INVALID_REQUEST);
              return;
            }
          superframeDuration = (TSysTime )MAC_ABASE_SUPERFRAME_DURATION << superframeOrder;
          beaconsOffset = 0;
          for (i = 0; i < slotNumber; i++) 
            beaconsOffset += superframeDuration;
        }
    }
  else {
    beaconsOffset = 0;
    }


  NLME_StartRouterM$NIB$setBeaconOffset(beaconsOffset);
  NLME_StartRouterM$updateBeacon();


  NLME_StartRouterM$IMacSTART$Request(
  NLME_StartRouterM$IMacGET$GetmacPANId(NULL), 
  NLME_StartRouterM$NIB$getChannel(), 
  beaconOrder, 
  superframeOrder, 
  FALSE, batteryLifeExtension, 
  FALSE, 
  FALSE, 
  beaconsOffset);
}

# 15 "../../../zigzag/ZigBee/interface/NLME_StartRouter.nc"
inline static  void ZigRouterM$NLME_StartRouter$request(uint16_t arg_0x4086f758, bool arg_0x4086f908){
#line 15
  NLME_StartRouterM$NLME_StartRouter$request(arg_0x4086f758, arg_0x4086f908);
#line 15
}
#line 15
# 137 "../../../zigzag/ZigRouterM.nc"
static inline  void ZigRouterM$start_router(void)
{

  ZigRouterM$NLME_StartRouter$request(
  0, 
  FALSE);


  return;
}

# 16 "../../../zigzag/ZigBee/interface/NLME_PermitJoining.nc"
inline static  NwkStatus ZigRouterM$NLME_PermitJoining$request(uint8_t arg_0x408728c0){
#line 16
  enum __nesc_unnamed4333 result;
#line 16

#line 16
  result = NLME_PermitJoiningM$NLME_PermitJoining$request(arg_0x408728c0);
#line 16

#line 16
  return result;
#line 16
}
#line 16
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
# 164 "../../../zigzag/ZigBee/implementation/NwkAddressingM.nc"
static inline  bool NwkAddressingM$NwkAddressing$hasVacantAddrs(void)
{
  uint8_t routers;
  uint8_t eds;

#line 168
  NwkAddressingM$countChildren(&routers, &eds);
  return NwkAddressingM$cskip != 0 && eds + routers < NwkAddressingM$NIB$getNwkMaxChildren();
}

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
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t NwkAddressingM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x4086b790);
#line 21

#line 21
  return result;
#line 21
}
#line 21
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
inline static  bool MacCoordAttrM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40ec2c00){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacAssocCoordM$IMacAssocAttr$GetmacAssociationPermit(arg_0x40ec2c00);
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
inline static  bool NwkBeaconParentM$IMacGET$GetmacAssociationPermit(TMacStatus *const arg_0x408c5168){
#line 7
  unsigned char result;
#line 7

#line 7
  result = MacCoordAttrM$IMacGET$GetmacAssociationPermit(arg_0x408c5168);
#line 7

#line 7
  return result;
#line 7
}
#line 7
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
inline static  TMacStatus MacCoordAttrM$IMacBeaconAttr$SetmacBeaconPayload(uint8_t *arg_0x40ed1d78, uint8_t arg_0x40ed1f18){
#line 16
  enum __nesc_unnamed4286 result;
#line 16

#line 16
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBeaconPayload(arg_0x40ed1d78, arg_0x40ed1f18);
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
inline static  TMacStatus NwkBeaconParentM$IMacSET$SetmacBeaconPayload(TMacBeaconPayload arg_0x408ee360, TMacBeaconPayloadLength arg_0x408ee510){
#line 15
  enum __nesc_unnamed4286 result;
#line 15

#line 15
  result = MacCoordAttrM$IMacSET$SetmacBeaconPayload(arg_0x408ee360, arg_0x408ee510);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t NLME_PermitJoiningM$ITimerSymbol$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(1U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
#line 6
inline static  result_t ChildSupervisorM$ITimerSymbol$SetPeriodic(TSysTime arg_0x408cdca8, TUniData arg_0x408cde30){
#line 6
  unsigned char result;
#line 6

#line 6
  result = TimerSymbol2M$ITimerSymbol$SetPeriodic(13U, arg_0x408cdca8, arg_0x408cde30);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TSysTime ChildSupervisorM$ITimeCast$MillisToSymbols(TMilliSec arg_0x40a3e6e0){
#line 6
  unsigned long result;
#line 6

#line 6
  result = TimeCastM$ITimeCast$MillisToSymbols(arg_0x40a3e6e0);
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
inline static  result_t ZigRouterM$ChildSupervisorControl$start(void){
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
# 269 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setOnline(NwkOnlineStatus status)
#line 269
{
#line 269
  NIBM$onlineStatus = status;
}

# 191 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NLME_StartRouterM$NIB$setOnline(NwkOnlineStatus arg_0x40875ae0){
#line 191
  NIBM$NIB$setOnline(arg_0x40875ae0);
#line 191
}
#line 191
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
inline static  result_t MacCommonAttrM$IPhyAttr$SetpanId(uint16_t arg_0x40b40100){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyCC2420M$IPhyAttr$SetpanId(arg_0x40b40100);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacSuperframesM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e57ab8);
#line 6

#line 6
  return result;
#line 6
}
#line 6




inline static  TMacSuperframeOrder MacSuperframesM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e55828){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(arg_0x40e55828);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacSuperframesM$IPhyTrxParent$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(0U, arg_0x40b27940, arg_0x40b27ae8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 15 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   uint32_t TimerSymbolAsyncM$ITimeCast$SymbolsToJiffies(TSysTime arg_0x40a3c4c8){
#line 15
  unsigned long result;
#line 15

#line 15
  result = TimeCastM$ITimeCast$SymbolsToJiffies(arg_0x40a3c4c8);
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
inline static   result_t MacSuperframesM$Timer$SetOneShotAt(TSysTime arg_0x40a38550, TUniData arg_0x40a386d8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbolAsyncM$ITimerSymbolAsync$SetOneShotAt(0U, arg_0x40a38550, arg_0x40a386d8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC_LOSS.nc"
inline static  void MacSuperframesM$IMacSYNC_LOSS$Indication(TMacStatus arg_0x4097b5b0){
#line 6
  NLME_SyncM$IMacSYNC_LOSS$Indication(arg_0x4097b5b0);
#line 6
}
#line 6
# 428 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  void MacSuperframesM$syncLoss(void)
{

  MacSuperframesM$IMacSYNC_LOSS$Indication(MAC_BEACON_LOSS);
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
inline static   void MacSuperframesM$ParentCAP$EndCAP(void){
#line 5
  MacCAPM$IMacCAP$EndCAP(MAC_PARENT_SF);
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

# 3 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/IMacCAP.nc"
inline static   void MacSuperframesM$ParentCAP$BeginCAP(void){
#line 3
  MacCAPM$IMacCAP$BeginCAP(MAC_PARENT_SF);
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
inline static   result_t MacCAPM$IPhyPool$GetUniData(const TPhyPoolHandle arg_0x40b3d500, uint16_t *const arg_0x40b3d6f0){
#line 10
  unsigned char result;
#line 10

#line 10
  result = PhyPoolM$IPhyPool$GetUniData(arg_0x40b3d500, arg_0x40b3d6f0);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 8 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPool.nc"
inline static  result_t MacPendingM$IMacPool$Free(const TMacPoolHandle arg_0x40ef2c68, const TMacStatus arg_0x40ef2e10){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacPoolM$IMacPool$Free(arg_0x40ef2c68, arg_0x40ef2e10);
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
inline static  TMacPANId MacAssocCoordM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dd3cf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCOMM_STATUS.nc"
inline static  void MacAssocCoordM$IMacCOMM_STATUS$Indication(TMacPANId arg_0x408c8400, TMacAddress arg_0x408c85a0, TMacAddress arg_0x408c8730, TMacStatus arg_0x408c88d0){
#line 6
  NLME_JoinParentM$IMacCOMM_STATUS$Indication(arg_0x408c8400, arg_0x408c85a0, arg_0x408c8730, arg_0x408c88d0);
#line 6
}
#line 6
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacAssocCoordM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacAddressM$IMacAddress$SetExtendedAddress(arg_0x408ff168, arg_0x408ff328);
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

# 337 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
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
inline static  void /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendDone(uint8_t arg_0x40f94168, TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98){
#line 12
  switch (arg_0x40f94168) {
#line 12
    case 0U:
#line 12
      MacPendingM$IMacCSMA$SendDone(arg_0x40f05cf0, arg_0x40f05e98);
#line 12
      break;
#line 12
    case 1U:
#line 12
      MacAssocCoordM$IMacSendOwn$SendDone(arg_0x40f05cf0, arg_0x40f05e98);
#line 12
      break;
#line 12
    case 2U:
#line 12
      MacDataM$IMacSendOwn$SendDone(arg_0x40f05cf0, arg_0x40f05e98);
#line 12
      break;
#line 12
    case 3U:
#line 12
      MacScanBeaconM$CSMA$SendDone(arg_0x40f05cf0, arg_0x40f05e98);
#line 12
      break;
#line 12
    default:
#line 12
      /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$default$SendDone(arg_0x40f94168, arg_0x40f05cf0, arg_0x40f05e98);
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

# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacAssocDeviceM$IAssocAckTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(7U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacAckWaitDuration MacAssocDeviceM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dd16c8){
#line 14
  enum __nesc_unnamed4288 result;
#line 14

#line 14
  result = MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(arg_0x40dd16c8);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 223 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  void MacAssocDeviceM$IAssocRequest$SendDone(TMacStatus macStatus, const TUniData uniData)
{
  if (MacAssocDeviceM$assoc.state == MacAssocDeviceM$MAC_ASSOC_REQUEST) {
    if (macStatus == MAC_SUCCESS) {
        TMacStatus status;
        TMacAckWaitDuration ackWaitPeriod = MacAssocDeviceM$IMacCommonAttr$GetmacAckWaitDuration(&status);

#line 229
        if (status == MAC_SUCCESS) {
            result_t result = MacAssocDeviceM$IAssocAckTimer$SetOneShot(ackWaitPeriod, uniData);

#line 231
            if (result == SUCCESS) {
                MacAssocDeviceM$assoc.state = MacAssocDeviceM$MAC_ASSOC_WAIT_ACK;
                return;
              }
            else 
#line 234
              {
                MacAssocDeviceM$AssocFailDone(MAC_INVALID_PARAMETER);
                return;
              }
          }
        else 
#line 238
          {
            MacAssocDeviceM$AssocFailDone(status);
            return;
          }
      }
    else 
#line 242
      {
        MacAssocDeviceM$AssocFailDone(macStatus);
        return;
      }
    }
#line 246
  return;
}

# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacExtractM$IAckWaitTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(9U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacAckWaitDuration MacExtractM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dd16c8){
#line 14
  enum __nesc_unnamed4288 result;
#line 14

#line 14
  result = MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(arg_0x40dd16c8);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 121 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
static inline  void MacExtractM$IMacCSMA$SendDone(TMacStatus macStatus, const TUniData uniData)
{
  if (MacExtractM$MAC_EXTRACT_SEND == MacExtractM$extractState) 
    {
      if (macStatus == MAC_SUCCESS) {
          TMacStatus status;
          TSysTime ackWaitDuration = (TSysTime )MacExtractM$IMacCommonAttr$GetmacAckWaitDuration(&status);

#line 128
          if (MAC_SUCCESS == status) {
              if (SUCCESS == MacExtractM$IAckWaitTimer$SetOneShot(ackWaitDuration, uniData)) {
                  MacExtractM$extractState = MacExtractM$MAC_EXTRACT_WAIT_ACK;
                  return;
                }
            }
          MacExtractM$ExtractDone(MAC_NO_ACK, FALSE);
          return;
        }
      else 
#line 136
        {
          MacExtractM$ExtractDone(macStatus, FALSE);
          return;
        }
    }
}

# 273 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanM.nc"
static inline  void MacScanM$IMacCSMA$SendDone(TMacStatus status, const TUniData unidata)
{
}

# 331 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline  void MacDataM$IMacSendParent$SendDone(TMacStatus status, const TUniData unidata)
{

  return MacDataM$SendDone(status, unidata);
}

# 56 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static inline   void /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$default$SendDone(uint8_t context, TMacStatus macStatus, TUniData unidata)
{
  ;
}

# 12 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  void /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$SendDone(uint8_t arg_0x40f94168, TMacStatus arg_0x40f05cf0, const TUniData arg_0x40f05e98){
#line 12
  switch (arg_0x40f94168) {
#line 12
    case 4U:
#line 12
      MacAssocDeviceM$IAssocRequest$SendDone(arg_0x40f05cf0, arg_0x40f05e98);
#line 12
      break;
#line 12
    case 5U:
#line 12
      MacExtractM$IMacCSMA$SendDone(arg_0x40f05cf0, arg_0x40f05e98);
#line 12
      break;
#line 12
    case 6U:
#line 12
      MacDataM$IMacSendParent$SendDone(arg_0x40f05cf0, arg_0x40f05e98);
#line 12
      break;
#line 12
    case 7U:
#line 12
      MacScanM$IMacCSMA$SendDone(arg_0x40f05cf0, arg_0x40f05e98);
#line 12
      break;
#line 12
    default:
#line 12
      /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$default$SendDone(arg_0x40f94168, arg_0x40f05cf0, arg_0x40f05e98);
#line 12
      break;
#line 12
    }
#line 12
}
#line 12
# 42 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static inline  void /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$SendDone(const uint8_t _sfIndex, uint8_t context, 
TMacStatus macStatus, const TUniData uniData)
{
  if (_sfIndex == 1) {
    /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$SendDone(context, macStatus, uniData);
    }
}

# 50 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
inline static  void MacCAPM$SendDone(const uint8_t arg_0x40f235b0, uint8_t arg_0x40f23730, TMacStatus arg_0x40f238c0, const TUniData arg_0x40f23a68){
#line 50
  /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$SendDone(arg_0x40f235b0, arg_0x40f23730, arg_0x40f238c0, arg_0x40f23a68);
#line 50
  /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendDone(arg_0x40f235b0, arg_0x40f23730, arg_0x40f238c0, arg_0x40f23a68);
#line 50
}
#line 50
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacDataM$AckWaitTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(5U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacAckWaitDuration MacDataM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const arg_0x40dd16c8){
#line 14
  enum __nesc_unnamed4288 result;
#line 14

#line 14
  result = MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(arg_0x40dd16c8);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 216 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
static inline   void MacExtractM$IMacExtract$default$End(uint8_t context, TMacStatus status, 
bool framePending, TUniData uniData)
#line 217
{
}

# 11 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacExtract.nc"
inline static  void MacExtractM$IMacExtract$End(uint8_t arg_0x411427b8, TMacStatus arg_0x41149550, bool arg_0x411496d8, TUniData arg_0x41149860){
#line 11
    MacExtractM$IMacExtract$default$End(arg_0x411427b8, arg_0x41149550, arg_0x411496d8, arg_0x41149860);
#line 11
}
#line 11
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t NLME_JoinChildM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacAddressM$IMacAddress$SetShortAddress(arg_0x40900a70, arg_0x40900c28);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 29 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t NLME_JoinChildM$NeighborTable$getByAddr(TMacAddress arg_0x4086bc20, NwkNeighbor **arg_0x4086bdf0){
#line 29
  unsigned char result;
#line 29

#line 29
  result = NeighborTableM$NeighborTable$getByAddr(arg_0x4086bc20, arg_0x4086bdf0);
#line 29

#line 29
  return result;
#line 29
}
#line 29
# 191 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NLME_JoinChildM$NIB$setOnline(NwkOnlineStatus arg_0x40875ae0){
#line 191
  NIBM$NIB$setOnline(arg_0x40875ae0);
#line 191
}
#line 191
# 275 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setDepth(uint8_t d)
{
  NIBM$depth = d;
}

# 197 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NLME_JoinChildM$NIB$setDepth(uint8_t arg_0x4088f918){
#line 197
  NIBM$NIB$setDepth(arg_0x4088f918);
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
inline static  void NLME_JoinChildM$NIB$setChannel(uint8_t arg_0x4088e0b0){
#line 201
  NIBM$NIB$setChannel(arg_0x4088e0b0);
#line 201
}
#line 201
# 7 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dd2198){
#line 7
  enum __nesc_unnamed4286 result;
#line 7

#line 7
  result = MacCommonAttrM$IMacCommonAttr$SetmacPANId(arg_0x40dd2198);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 6 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacPANId(TMacPANId macPANId)
{
  return MacDeviceAttrM$IMacCommonAttr$SetmacPANId(macPANId);
}

# 36 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_JoinChildM$IMacSET$SetmacPANId(TMacPANId arg_0x408ea4a8){
#line 36
  enum __nesc_unnamed4286 result;
#line 36

#line 36
  result = MacDeviceAttrM$IMacSET$SetmacPANId(arg_0x408ea4a8);
#line 36

#line 36
  return result;
#line 36
}
#line 36
# 3 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
inline static  result_t MacCommonAttrM$IPhyAttr$SetshortAddress(uint16_t arg_0x40b24c58){
#line 3
  unsigned char result;
#line 3

#line 3
  result = PhyCC2420M$IPhyAttr$SetshortAddress(arg_0x40b24c58);
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
inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetmacShortAddress(TMacShortAddress arg_0x40dd2e50){
#line 11
  enum __nesc_unnamed4286 result;
#line 11

#line 11
  result = MacCommonAttrM$IMacCommonAttr$SetmacShortAddress(arg_0x40dd2e50);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 16 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacShortAddress(TMacShortAddress macShortAddress)
{
  return MacDeviceAttrM$IMacCommonAttr$SetmacShortAddress(macShortAddress);
}

# 42 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_JoinChildM$IMacSET$SetmacShortAddress(TMacShortAddress arg_0x40907358){
#line 42
  enum __nesc_unnamed4286 result;
#line 42

#line 42
  result = MacDeviceAttrM$IMacSET$SetmacShortAddress(arg_0x40907358);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t NLME_JoinChildM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x4086b790);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t NLME_JoinChildM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacAddressM$IMacAddress$SetExtendedAddress(arg_0x408ff168, arg_0x408ff328);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 97 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetmacCoordShortAddress(
TMacShortAddress macCoordShortAddress)
{
  MacAssocDeviceM$assoc.macCoordShortAddress = macCoordShortAddress;
  return MAC_SUCCESS;
}

# 14 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetmacCoordShortAddress(TMacShortAddress arg_0x40ec23f8){
#line 14
  enum __nesc_unnamed4286 result;
#line 14

#line 14
  result = MacAssocDeviceM$IMacAssocAttr$SetmacCoordShortAddress(arg_0x40ec23f8);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 242 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacCoordShortAddress(
TMacShortAddress macCoordShortAddress)
{
  return MacDeviceAttrM$IMacAssocAttr$SetmacCoordShortAddress(macCoordShortAddress);
}

# 26 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_JoinChildM$IMacSET$SetmacCoordShortAddress(TMacShortAddress arg_0x408edca0){
#line 26
  enum __nesc_unnamed4286 result;
#line 26

#line 26
  result = MacDeviceAttrM$IMacSET$SetmacCoordShortAddress(arg_0x408edca0);
#line 26

#line 26
  return result;
#line 26
}
#line 26
# 78 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  TMacStatus MacAssocDeviceM$IMacAssocAttr$SetmacCoordExtendedAddress(
TMacExtendedAddress macCoordExtendedAddress)
{
  MacAssocDeviceM$assoc.macCoordExtendedAddress = macCoordExtendedAddress;
  return MAC_SUCCESS;
}

# 8 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetmacCoordExtendedAddress(TMacExtendedAddress arg_0x40ec46d0){
#line 8
  enum __nesc_unnamed4286 result;
#line 8

#line 8
  result = MacAssocDeviceM$IMacAssocAttr$SetmacCoordExtendedAddress(arg_0x40ec46d0);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 230 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacCoordExtendedAddress(
TMacExtendedAddress macCoordExtendedAddress)
{
  return MacDeviceAttrM$IMacAssocAttr$SetmacCoordExtendedAddress(macCoordExtendedAddress);
}

# 24 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_JoinChildM$IMacSET$SetmacCoordExtendedAddress(TMacExtendedAddress arg_0x408ed7f0){
#line 24
  enum __nesc_unnamed4286 result;
#line 24

#line 24
  result = MacDeviceAttrM$IMacSET$SetmacCoordExtendedAddress(arg_0x408ed7f0);
#line 24

#line 24
  return result;
#line 24
}
#line 24
# 7 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e55010){
#line 7
  enum __nesc_unnamed4286 result;
#line 7

#line 7
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(arg_0x40e55010);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 85 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  TMacStatus MacDeviceAttrM$IMacSET$SetmacBeaconOrder(
TMacBeaconOrder macBeaconOrder)
{
  return MacDeviceAttrM$IMacSuperframeAttr$SetmacBeaconOrder(macBeaconOrder);
}

# 18 "../../../zigzag/IEEE802_15_4/Mac/public/IMacSET.nc"
inline static  TMacStatus NLME_JoinChildM$IMacSET$SetmacBeaconOrder(TMacBeaconOrder arg_0x408ee9b8){
#line 18
  enum __nesc_unnamed4286 result;
#line 18

#line 18
  result = MacDeviceAttrM$IMacSET$SetmacBeaconOrder(arg_0x408ee9b8);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 58 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
static inline  void MacSyncM$continueSync(void)
{
  if (!MacSyncM$IPhySET_TRX_STATE$Request(PHY_RX_ON, 0)) 
    {
      MacSyncM$IMacSYNC_LOSS$Indication(MAC_BEACON_LOSS);
      return;
    }

  MacSyncM$first_radio_on = TRUE;
  MacSyncM$beacon_expected = TRUE;
}

# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
inline static  TPhyStatus MacSyncM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b2a360){
#line 5
  enum __nesc_unnamed4242 result;
#line 5

#line 5
  result = PhyCC2420M$IPhySET$SetphyCurrentChannel(arg_0x40b2a360);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 11 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacStatus MacSyncM$IMacSuperframeAttr$SetmacSuperframeOrder(TMacSuperframeOrder arg_0x40e55cd8){
#line 11
  enum __nesc_unnamed4286 result;
#line 11

#line 11
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetmacSuperframeOrder(arg_0x40e55cd8);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacSyncM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dd3cf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 635 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  void MacCAPM$ResetAll$reset(void)
{
  uint8_t i;

#line 638
  for (i = 0; i < MacCAPM$NUM_SUPERFRAMES; i++) 
    MacCAPM$Reset$reset(i);
}

# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
inline static  void MacSyncM$CAPReset$reset(void){
#line 46
  MacCAPM$ResetAll$reset();
#line 46
}
#line 46
inline static  void MacSyncM$SuperframesReset$reset(void){
#line 46
  MacSuperframesM$Reset$reset();
#line 46
}
#line 46
# 39 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
static inline  result_t MacSyncM$IMacSYNC$Request(TMacLogicalChannel logicalChannel, 
bool trackBeacon)
{


  MacSyncM$SuperframesReset$reset();
  MacSyncM$CAPReset$reset();

  if (MacSyncM$IMacCommonAttr$GetmacPANId(NULL) == 0xFFFF || MacSyncM$IMacSuperframeAttr$GetmacBeaconOrder(NULL) == 15) 
    {

      return FAIL;
    }
  MacSyncM$IMacSuperframeAttr$SetmacSuperframeOrder(15);
  MacSyncM$IPhySET$SetphyCurrentChannel(logicalChannel);

  return TOS_post(MacSyncM$continueSync);
}

# 5 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacSYNC.nc"
inline static  result_t NLME_JoinChildM$IMacSYNC$Request(TMacLogicalChannel arg_0x408fca00, bool arg_0x408fcba8){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacSyncM$IMacSYNC$Request(arg_0x408fca00, arg_0x408fcba8);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 19 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacAssocDeviceM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dcf828){
#line 19
  enum __nesc_unnamed4286 result;
#line 19

#line 19
  result = MacCommonAttrM$IMacCommonAttr$SetmacDSN(arg_0x40dcf828);
#line 19

#line 19
  return result;
#line 19
}
#line 19
#line 18
inline static  TMacDSN MacAssocDeviceM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dcf3a0){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacCommonAttrM$IMacCommonAttr$GetmacDSN(arg_0x40dcf3a0);
#line 18

#line 18
  return result;
#line 18
}
#line 18
#line 7
inline static  TMacStatus MacAssocDeviceM$IMacCommonAttr$SetmacPANId(TMacPANId arg_0x40dd2198){
#line 7
  enum __nesc_unnamed4286 result;
#line 7

#line 7
  result = MacCommonAttrM$IMacCommonAttr$SetmacPANId(arg_0x40dd2198);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET.nc"
inline static  TPhyStatus MacAssocDeviceM$IPhySET$SetphyCurrentChannel(TPhyChannel arg_0x40b2a360){
#line 5
  enum __nesc_unnamed4242 result;
#line 5

#line 5
  result = PhyCC2420M$IPhySET$SetphyCurrentChannel(arg_0x40b2a360);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 177 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  result_t MacAssocDeviceM$IMacASSOCIATE$Request(TMacLogicalChannel logicalChannel, 
TMacPANId coordPANId, TMacAddress coordAddress, 
TCapabilityInformation capabilityInformation, 
bool securityEnable)
{

  if (MacAssocDeviceM$assoc.state == MacAssocDeviceM$MAC_ASSOC_IDLE) {
      TPhyStatus phyStatus = MacAssocDeviceM$IPhySET$SetphyCurrentChannel(logicalChannel);
      TMacStatus macStatus = MacAssocDeviceM$IMacCommonAttr$SetmacPANId(coordPANId);

#line 186
      if (phyStatus == PHY_SUCCESS && macStatus == MAC_SUCCESS) {
          TMacFrame assocRequest;
          result_t result;

#line 189
          MacAssocDeviceM$assoc.seqNum = MacAssocDeviceM$IMacCommonAttr$GetmacDSN(NULL);
          MacAssocDeviceM$IMacCommonAttr$SetmacDSN(MacAssocDeviceM$assoc.seqNum + 1);
          result = MacAssocDeviceM$MakeAssocReqFrame(coordPANId, coordAddress, 
          capabilityInformation, securityEnable, &assocRequest);
          if (result == SUCCESS) {
              result = MacAssocDeviceM$IAssocRequest$Send(&assocRequest, 0);

              if (result == SUCCESS) {
                  MacAssocDeviceM$assoc.coordPANId = coordPANId;
                  MacAssocDeviceM$assoc.coordAddress = coordAddress;
                  MacAssocDeviceM$assoc.capabilityInformation = capabilityInformation;
                  MacAssocDeviceM$assoc.securityEnable = securityEnable;
                  MacAssocDeviceM$assoc.state = MacAssocDeviceM$MAC_ASSOC_REQUEST;
                  return SUCCESS;
                }
            }
        }
    }
  return FAIL;
}

# 25 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacASSOCIATE.nc"
inline static  result_t NLME_JoinChildM$IMacASSOCIATE$Request(TMacLogicalChannel arg_0x408ae290, TMacPANId arg_0x408ae430, TMacAddress arg_0x408ae5c0, TCapabilityInformation arg_0x408ae770, bool arg_0x408ae908){
#line 25
  unsigned char result;
#line 25

#line 25
  result = MacAssocDeviceM$IMacASSOCIATE$Request(arg_0x408ae290, arg_0x408ae430, arg_0x408ae5c0, arg_0x408ae770, arg_0x408ae908);
#line 25

#line 25
  return result;
#line 25
}
#line 25
# 5 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocDeviceM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacFrameFormatM$IMacFrame$SetFrameType(arg_0x40db68e8, arg_0x40db6a98);
#line 5

#line 5
  return result;
#line 5
}
#line 5



inline static  result_t MacAssocDeviceM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacFrameFormatM$IMacFrame$SetSecurityEnabled(arg_0x40db5708, arg_0x40db58b0);
#line 8

#line 8
  return result;
#line 8
}
#line 8



inline static  result_t MacAssocDeviceM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacFrameFormatM$IMacFrame$SetFramePending(arg_0x40db4510, arg_0x40db46b8);
#line 11

#line 11
  return result;
#line 11
}
#line 11



inline static  result_t MacAssocDeviceM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacFrameFormatM$IMacFrame$SetAckRequest(arg_0x40db22d8, arg_0x40db2480);
#line 14

#line 14
  return result;
#line 14
}
#line 14



inline static  result_t MacAssocDeviceM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250){
#line 17
  unsigned char result;
#line 17

#line 17
  result = MacFrameFormatM$IMacFrame$SetIntraPAN(arg_0x40db10a8, arg_0x40db1250);
#line 17

#line 17
  return result;
#line 17
}
#line 17



inline static  result_t MacAssocDeviceM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacFrameFormatM$IMacFrame$SetSequenceNumber(arg_0x40db1e60, arg_0x40daf030);
#line 20

#line 20
  return result;
#line 20
}
#line 20







inline static  result_t MacAssocDeviceM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8){
#line 27
  unsigned char result;
#line 27

#line 27
  result = MacFrameFormatM$IMacFrame$SetDstAddress(arg_0x40daeb38, arg_0x40daece8);
#line 27

#line 27
  return result;
#line 27
}
#line 27
#line 24
inline static  result_t MacAssocDeviceM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40dafc60, const TMacPANId arg_0x40dafe10){
#line 24
  unsigned char result;
#line 24

#line 24
  result = MacFrameFormatM$IMacFrame$SetDstPANId(arg_0x40dafc60, arg_0x40dafe10);
#line 24

#line 24
  return result;
#line 24
}
#line 24
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacAssocDeviceM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacAddressM$IMacAddress$SetExtendedAddress(arg_0x408ff168, arg_0x408ff328);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 33 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocDeviceM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0){
#line 33
  unsigned char result;
#line 33

#line 33
  result = MacFrameFormatM$IMacFrame$SetSrcAddress(arg_0x40dab700, arg_0x40dab8b0);
#line 33

#line 33
  return result;
#line 33
}
#line 33



inline static  result_t MacAssocDeviceM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0){
#line 36
  unsigned char result;
#line 36

#line 36
  result = MacFrameFormatM$IMacFrame$SetPayload(arg_0x40da9510, arg_0x40da9728, arg_0x40da98c0);
#line 36

#line 36
  return result;
#line 36
}
#line 36
# 173 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  NwkCapabilityInfo NIBM$NIB$getNwkCapabilityInformation(void)
{
  return NIBM$nwkCapabilityInformation;
}

# 117 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  NwkCapabilityInfo NLME_JoinChildM$NIB$getNwkCapabilityInformation(void){
#line 117
  struct __nesc_unnamed4326 result;
#line 117

#line 117
  result = NIBM$NIB$getNwkCapabilityInformation();
#line 117

#line 117
  return result;
#line 117
}
#line 117
# 37 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacPANId NLME_JoinParentM$IMacGET$GetmacPANId(TMacStatus *const arg_0x408bd010){
#line 37
  unsigned int result;
#line 37

#line 37
  result = MacCoordAttrM$IMacGET$GetmacPANId(arg_0x408bd010);
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
inline static  result_t NLME_JoinParentM$NeighborTable$update(NwkNeighbor arg_0x4086b2b8){
#line 15
  unsigned char result;
#line 15

#line 15
  result = NeighborTableM$NeighborTable$update(arg_0x4086b2b8);
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
# 8 "../../../zigzag/ZigMonitorM.nc"
static inline  void ZigMonitorM$NLME_JoinParent$indication(NwkAddr shortAddr, IEEEAddr extendedAddr, 
NwkCapabilityInfo capabilityInformation, bool secureJoin)
{
  return;
}

# 77 "../../../zigzag/ZigNetM.nc"
static inline  void ZigNetM$NLME_JoinParent$indication(NwkAddr shortAddr, IEEEAddr extendedAddr, 
NwkCapabilityInfo capabilityInformation, bool secureJoin)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __net_child_join(shortAddr, extendedAddr);
    }
#line 82
  return;
}

# 15 "../../../zigzag/ZigBee/interface/NLME_JoinParent.nc"
inline static  void NLME_JoinParentM$NLME_JoinParent$indication(NwkAddr arg_0x408a8a70, IEEEAddr arg_0x408a8c10, NwkCapabilityInfo arg_0x408a8dc0, bool arg_0x408a6010){
#line 15
  ZigNetM$NLME_JoinParent$indication(arg_0x408a8a70, arg_0x408a8c10, arg_0x408a8dc0, arg_0x408a6010);
#line 15
  ZigMonitorM$NLME_JoinParent$indication(arg_0x408a8a70, arg_0x408a8c10, arg_0x408a8dc0, arg_0x408a6010);
#line 15
}
#line 15
# 19 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacBeaconOrder NLME_JoinParentM$IMacGET$GetmacBeaconOrder(TMacStatus *const arg_0x408c11d8){
#line 19
  unsigned char result;
#line 19

#line 19
  result = MacCoordAttrM$IMacGET$GetmacBeaconOrder(arg_0x408c11d8);
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t NLME_JoinParentM$ITimerSymbol$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(0U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
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
inline static  void MacPoolM$IMacPool$FreeDone(const TMacPoolHandle arg_0x40ef1358, const TMacStatus arg_0x40ef1500){
#line 9
  MacBeaconPackerM$IMacPool$FreeDone(arg_0x40ef1358, arg_0x40ef1500);
#line 9
  MacPendingM$IMacPool$FreeDone(arg_0x40ef1358, arg_0x40ef1500);
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

# 10 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacSuperframeOrder MacCAPM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e55828){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(arg_0x40e55828);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 170 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static inline  void MacCAPM$TurnOffTask(void)
{
  MacCAPM$TurnOffIfInactive(PHY_TRX_OFF);
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
inline static   void TimerSymbolAsyncM$AlarmCompare4$setEvent(uint16_t arg_0x407331d8){
#line 30
  MSP430TimerM$CompareB6$setEvent(arg_0x407331d8);
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
inline static   void TimerSymbolAsyncM$AlarmCompare3$setEvent(uint16_t arg_0x407331d8){
#line 30
  MSP430TimerM$CompareB5$setEvent(arg_0x407331d8);
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
inline static   void TimerSymbolAsyncM$AlarmCompare2$setEvent(uint16_t arg_0x407331d8){
#line 30
  MSP430TimerM$CompareB4$setEvent(arg_0x407331d8);
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
inline static   void TimerSymbolAsyncM$AlarmCompare1$setEvent(uint16_t arg_0x407331d8){
#line 30
  MSP430TimerM$CompareB3$setEvent(arg_0x407331d8);
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
inline static   void TimerSymbolAsyncM$AlarmCompare0$setEvent(uint16_t arg_0x407331d8){
#line 30
  MSP430TimerM$CompareB2$setEvent(arg_0x407331d8);
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
inline static   result_t MacSuperframesM$IPhyTrxOwn$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(1U, arg_0x40b27940, arg_0x40b27ae8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
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
inline static   result_t MacSuperframesM$IPhyTxDATA$Request(const TPhyPoolHandle arg_0x40b32f18, const TUniData arg_0x40b340f0){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhyTxDATA$Request(0U, arg_0x40b32f18, arg_0x40b340f0);
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

# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   uint8_t PhyCC2420M$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b48808){
#line 9
  unsigned char result;
#line 9

#line 9
  result = PhyFrameM$IPhyFrame$GetPPDULength(arg_0x40b48808);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 17 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyFrameM.nc"
static inline   uint8_t PhyFrameM$IPhyFrame$GetMPDULength(const TPhyFrame *const pPhyFrame)
{
  if (pPhyFrame != NULL) {
    return pPhyFrame->data[0];
    }
#line 21
  return 0;
}

# 43 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Capture.nc"
inline static   result_t PhyCC2420M$IChipconSFD$enableCapture(bool arg_0x40b7ed48){
#line 43
  unsigned char result;
#line 43

#line 43
  result = HPLCC2420InterruptM$SFD$enableCapture(arg_0x40b7ed48);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 47 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420.nc"
inline static   uint8_t PhyCC2420M$IChipcon$cmd(uint8_t arg_0x40b7c678){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLCC2420M$HPLCC2420$cmd(arg_0x40b7c678);
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
inline static   void PhyCC2420M$IPhyTxFIFO$WriteDone(uint8_t arg_0x40b80bf8, TPhyPoolHandle arg_0x40b31c08, result_t arg_0x40b31da0, TUniData arg_0x40b31f28){
#line 10
  switch (arg_0x40b80bf8) {
#line 10
    case 0U:
#line 10
      MacSuperframesM$IPhyTxFIFO$WriteDone(arg_0x40b31c08, arg_0x40b31da0, arg_0x40b31f28);
#line 10
      break;
#line 10
    default:
#line 10
      PhyCC2420M$IPhyTxFIFO$default$WriteDone(arg_0x40b80bf8, arg_0x40b31c08, arg_0x40b31da0, arg_0x40b31f28);
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
inline static   result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t arg_0x40b77118, uint8_t *arg_0x40b772b8){
#line 50
  unsigned char result;
#line 50

#line 50
  result = PhyCC2420M$IChipconFIFO$TXFIFODone(arg_0x40b77118, arg_0x40b772b8);
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
inline static   result_t HPLCC2420M$USARTControl$tx(uint8_t arg_0x40c54e18){
#line 202
  unsigned char result;
#line 202

#line 202
  result = HPLUSART0M$USARTControl$tx(arg_0x40c54e18);
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
# 42 "/home/max/tinyos/tinyos-1.x/tos/platform/smoke/hardware.h"
static inline void TOSH_CLR_RADIO_CSN_PIN(void)
#line 42
{
#line 42
   static volatile uint8_t r __asm ("0x0019");

#line 42
  r &= ~(1 << 0);
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
inline static   result_t PhyCC2420M$IChipconFIFO$writeTXFIFO(uint8_t arg_0x40b78260, uint8_t *arg_0x40b78400){
#line 29
  unsigned char result;
#line 29

#line 29
  result = HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(arg_0x40b78260, arg_0x40b78400);
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
inline static  result_t PhyCC2420M$IPhyTxDATA$Confirm(uint8_t arg_0x40b57010, TPhyPoolHandle arg_0x40b34580, TPhyStatus arg_0x40b34720, TUniData arg_0x40b348b8){
#line 9
  unsigned char result;
#line 9

#line 9
  switch (arg_0x40b57010) {
#line 9
    case 0U:
#line 9
      result = MacSuperframesM$IPhyTxDATA$Confirm(arg_0x40b34580, arg_0x40b34720, arg_0x40b348b8);
#line 9
      break;
#line 9
    case 1U:
#line 9
      result = MacCAPM$IPhyTxDATA$Confirm(arg_0x40b34580, arg_0x40b34720, arg_0x40b348b8);
#line 9
      break;
#line 9
    default:
#line 9
      result = PhyCC2420M$IPhyTxDATA$default$Confirm(arg_0x40b57010, arg_0x40b34580, arg_0x40b34720, arg_0x40b348b8);
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
# 23 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
inline static  void MacSuperframesM$packBeacon(TPhyFrame *arg_0x40e7b010, uint64_t *arg_0x40e7b1b8){
#line 23
  MacBeaconPackerM$packBeacon(arg_0x40e7b010, arg_0x40e7b1b8);
#line 23
}
#line 23
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *MacSuperframesM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010){
#line 8
  struct __nesc_unnamed4281 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b3d010);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 5 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
inline static  uint64_t MacSuperframesM$ILocalTime64$getLocalTimeAt(TSysTime arg_0x40a42e70){
#line 5
  unsigned long long result;
#line 5

#line 5
  result = LocalTime64M$ILocalTime64$getLocalTimeAt(arg_0x40a42e70);
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
inline static  result_t MacBeaconPackerM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacFrameFormatM$IMacFrame$SetFrameType(arg_0x40db68e8, arg_0x40db6a98);
#line 5

#line 5
  return result;
#line 5
}
#line 5



inline static  result_t MacBeaconPackerM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacFrameFormatM$IMacFrame$SetSecurityEnabled(arg_0x40db5708, arg_0x40db58b0);
#line 8

#line 8
  return result;
#line 8
}
#line 8



inline static  result_t MacBeaconPackerM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacFrameFormatM$IMacFrame$SetFramePending(arg_0x40db4510, arg_0x40db46b8);
#line 11

#line 11
  return result;
#line 11
}
#line 11



inline static  result_t MacBeaconPackerM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacFrameFormatM$IMacFrame$SetAckRequest(arg_0x40db22d8, arg_0x40db2480);
#line 14

#line 14
  return result;
#line 14
}
#line 14



inline static  result_t MacBeaconPackerM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250){
#line 17
  unsigned char result;
#line 17

#line 17
  result = MacFrameFormatM$IMacFrame$SetIntraPAN(arg_0x40db10a8, arg_0x40db1250);
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
inline static  TMacBSN MacBeaconPackerM$IMacBeaconAttr$GetmacBSN(TMacStatus *const arg_0x40ed0740){
#line 21
  unsigned char result;
#line 21

#line 21
  result = MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBSN(arg_0x40ed0740);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 20 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacFrameFormatM$IMacFrame$SetSequenceNumber(arg_0x40db1e60, arg_0x40daf030);
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
inline static  TMacStatus MacBeaconPackerM$IMacBeaconAttr$SetmacBSN(TMacBSN arg_0x40ed0bc8){
#line 22
  enum __nesc_unnamed4286 result;
#line 22

#line 22
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetmacBSN(arg_0x40ed0bc8);
#line 22

#line 22
  return result;
#line 22
}
#line 22
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacShortAddress MacBeaconPackerM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const arg_0x40dd29a8){
#line 10
  unsigned int result;
#line 10

#line 10
  result = MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(arg_0x40dd29a8);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacBeaconPackerM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacAddressM$IMacAddress$SetShortAddress(arg_0x40900a70, arg_0x40900c28);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 33 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0){
#line 33
  unsigned char result;
#line 33

#line 33
  result = MacFrameFormatM$IMacFrame$SetSrcAddress(arg_0x40dab700, arg_0x40dab8b0);
#line 33

#line 33
  return result;
#line 33
}
#line 33
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacBeaconPackerM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacAddressM$IMacAddress$SetExtendedAddress(arg_0x408ff168, arg_0x408ff328);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 30 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40dac908, const TMacPANId arg_0x40dacab8){
#line 30
  unsigned char result;
#line 30

#line 30
  result = MacFrameFormatM$IMacFrame$SetSrcPANId(arg_0x40dac908, arg_0x40dacab8);
#line 30

#line 30
  return result;
#line 30
}
#line 30
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacBeaconPackerM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dd3cf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacBeaconPackerM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x408ff828){
#line 16
  unsigned char result;
#line 16

#line 16
  result = MacAddressM$IMacAddress$SetEmptyAddress(arg_0x408ff828);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 27 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconPackerM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8){
#line 27
  unsigned char result;
#line 27

#line 27
  result = MacFrameFormatM$IMacFrame$SetDstAddress(arg_0x40daeb38, arg_0x40daece8);
#line 27

#line 27
  return result;
#line 27
}
#line 27
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacBeaconPackerM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e57ab8);
#line 6

#line 6
  return result;
#line 6
}
#line 6




inline static  TMacSuperframeOrder MacBeaconPackerM$IMacSuperframeAttr$GetmacSuperframeOrder(TMacStatus *const arg_0x40e55828){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacSuperframeOrder(arg_0x40e55828);
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
inline static  bool MacBeaconPackerM$IMacCAPAttr$GetmacBattLifeExt(TMacStatus *const arg_0x40ec9c08){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacCAPM$IMacCAPAttr$GetmacBattLifeExt(arg_0x40ec9c08);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 18 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  bool MacBeaconPackerM$IMacAssocAttr$GetmacAssociationPermit(TMacStatus *const arg_0x40ec2c00){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacAssocCoordM$IMacAssocAttr$GetmacAssociationPermit(arg_0x40ec2c00);
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
inline static  uint8_t *MacBeaconPackerM$IMacBeaconAttr$GetmacBeaconPayload(uint8_t *const arg_0x40ed1680, TMacStatus *const arg_0x40ed1880){
#line 13
  unsigned char *result;
#line 13

#line 13
  result = MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayload(arg_0x40ed1680, arg_0x40ed1880);
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
inline static  uint8_t MacBeaconPackerM$IMacBeaconAttr$GetmacBeaconPayloadLength(TMacStatus *const arg_0x40ed1178){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacBeaconAttrCoordM$IMacBeaconAttr$GetmacBeaconPayloadLength(arg_0x40ed1178);
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
inline static  result_t MacBeaconPackerM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0){
#line 36
  unsigned char result;
#line 36

#line 36
  result = MacFrameFormatM$IMacFrame$SetPayload(arg_0x40da9510, arg_0x40da9728, arg_0x40da98c0);
#line 36

#line 36
  return result;
#line 36
}
#line 36








inline static  TMacRawFrameLength MacBeaconPackerM$IMacFrame$Pack(const TMacFrame *const arg_0x40da8c38, TPhyFrame *const arg_0x40da8e28){
#line 44
  unsigned char result;
#line 44

#line 44
  result = MacFrameFormatM$IMacFrame$Pack(arg_0x40da8c38, arg_0x40da8e28);
#line 44

#line 44
  return result;
#line 44
}
#line 44
# 93 "../../../zigzag/ZigNetM.nc"
static inline  void ZigNetM$sleepIndication(TMilliSec sleep_duration)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __net_sleep(sleep_duration);
    }
}

# 35 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
inline static  void MacSuperframesM$sleepIndication(TMilliSec arg_0x40e79508){
#line 35
  ZigNetM$sleepIndication(arg_0x40e79508);
#line 35
}
#line 35
# 10 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   TMilliSec MacSuperframesM$ITimeCast$SymbolsToMillis(TSysTime arg_0x40a3d378){
#line 10
  unsigned long result;
#line 10

#line 10
  result = TimeCastM$ITimeCast$SymbolsToMillis(arg_0x40a3d378);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 5 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacScanM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacFrameFormatM$IMacFrame$SetFrameType(arg_0x40db68e8, arg_0x40db6a98);
#line 5

#line 5
  return result;
#line 5
}
#line 5






inline static  result_t MacScanM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacFrameFormatM$IMacFrame$SetFramePending(arg_0x40db4510, arg_0x40db46b8);
#line 11

#line 11
  return result;
#line 11
}
#line 11
#line 8
inline static  result_t MacScanM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacFrameFormatM$IMacFrame$SetSecurityEnabled(arg_0x40db5708, arg_0x40db58b0);
#line 8

#line 8
  return result;
#line 8
}
#line 8






inline static  result_t MacScanM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacFrameFormatM$IMacFrame$SetAckRequest(arg_0x40db22d8, arg_0x40db2480);
#line 14

#line 14
  return result;
#line 14
}
#line 14



inline static  result_t MacScanM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250){
#line 17
  unsigned char result;
#line 17

#line 17
  result = MacFrameFormatM$IMacFrame$SetIntraPAN(arg_0x40db10a8, arg_0x40db1250);
#line 17

#line 17
  return result;
#line 17
}
#line 17
# 18 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacDSN MacScanM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dcf3a0){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacCommonAttrM$IMacCommonAttr$GetmacDSN(arg_0x40dcf3a0);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 20 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacScanM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacFrameFormatM$IMacFrame$SetSequenceNumber(arg_0x40db1e60, arg_0x40daf030);
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 19 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacScanM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dcf828){
#line 19
  enum __nesc_unnamed4286 result;
#line 19

#line 19
  result = MacCommonAttrM$IMacCommonAttr$SetmacDSN(arg_0x40dcf828);
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 24 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacScanM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40dafc60, const TMacPANId arg_0x40dafe10){
#line 24
  unsigned char result;
#line 24

#line 24
  result = MacFrameFormatM$IMacFrame$SetDstPANId(arg_0x40dafc60, arg_0x40dafe10);
#line 24

#line 24
  return result;
#line 24
}
#line 24
# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacScanM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacAddressM$IMacAddress$SetShortAddress(arg_0x40900a70, arg_0x40900c28);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 27 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacScanM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8){
#line 27
  unsigned char result;
#line 27

#line 27
  result = MacFrameFormatM$IMacFrame$SetDstAddress(arg_0x40daeb38, arg_0x40daece8);
#line 27

#line 27
  return result;
#line 27
}
#line 27
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacScanM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x408ff828){
#line 16
  unsigned char result;
#line 16

#line 16
  result = MacAddressM$IMacAddress$SetEmptyAddress(arg_0x408ff828);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 33 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacScanM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0){
#line 33
  unsigned char result;
#line 33

#line 33
  result = MacFrameFormatM$IMacFrame$SetSrcAddress(arg_0x40dab700, arg_0x40dab8b0);
#line 33

#line 33
  return result;
#line 33
}
#line 33



inline static  result_t MacScanM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0){
#line 36
  unsigned char result;
#line 36

#line 36
  result = MacFrameFormatM$IMacFrame$SetPayload(arg_0x40da9510, arg_0x40da9728, arg_0x40da98c0);
#line 36

#line 36
  return result;
#line 36
}
#line 36
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  result_t MacScanM$IMacCSMA$Send(const TMacFrame *const arg_0x40f051f8, const TUniData arg_0x40f053b0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacSend$Send(7U, arg_0x40f051f8, arg_0x40f053b0);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 21 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t NLME_NetworkDiscoveryM$NeighborTable$getNextPtr(NwkNeighbor **arg_0x4086b790){
#line 21
  unsigned char result;
#line 21

#line 21
  result = NeighborTableM$NeighborTable$getNextPtr(arg_0x4086b790);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 9 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   uint8_t MacCAPM$IPhyFrame$GetPPDULength(const TPhyFrame *const arg_0x40b48808){
#line 9
  unsigned char result;
#line 9

#line 9
  result = PhyFrameM$IPhyFrame$GetPPDULength(arg_0x40b48808);
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

# 897 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline   result_t PhyCC2420M$IPhyED$default$Confirm(uint8_t context, TPhyStatus phyStatus, 
TPhyEnergyLevel phyEnergyLevel, TUniData uniData)
{
#line 899
  return SUCCESS;
}

# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyED.nc"
inline static  result_t PhyCC2420M$IPhyED$Confirm(uint8_t arg_0x40b57998, TPhyStatus arg_0x40b4b1a8, TPhyEnergyLevel arg_0x40b4b348, TUniData arg_0x40b4b4d0){
#line 8
  unsigned char result;
#line 8

#line 8
  switch (arg_0x40b57998) {
#line 8
    case 0U:
#line 8
      result = MacScanM$IPhyED$Confirm(arg_0x40b4b1a8, arg_0x40b4b348, arg_0x40b4b4d0);
#line 8
      break;
#line 8
    default:
#line 8
      result = PhyCC2420M$IPhyED$default$Confirm(arg_0x40b57998, arg_0x40b4b1a8, arg_0x40b4b348, arg_0x40b4b4d0);
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
# 411 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  void PhyCC2420M$MeasureED(void)
{
  TCC2420_STATUS status;
  PhyCC2420M$TPhyRadioState _radioState;
  uint8_t context;
  TUniData uniData;

#line 417
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 417
    {
      _radioState = PhyCC2420M$radioState;
      PhyCC2420M$edState = PhyCC2420M$PHY_ED_FREE;
      context = PhyCC2420M$edContext;
      uniData = PhyCC2420M$edUniData;
    }
#line 422
    __nesc_atomic_end(__nesc_atomic); }
  switch (_radioState) {
      case PhyCC2420M$PHY_RADIO_OFF: case PhyCC2420M$PHY_RADIO_IDLE: 
          PhyCC2420M$IPhyED$Confirm(context, PHY_TRX_OFF, 0x0, uniData);
      return;
      case PhyCC2420M$PHY_RADIO_TX_IDLE: case PhyCC2420M$PHY_RADIO_TX_INIT: 
          case PhyCC2420M$PHY_RADIO_TX_FIFO_WRITE: case PhyCC2420M$PHY_RADIO_TX_BEGIN: 
              case PhyCC2420M$PHY_RADIO_TX_FRAME_SEND: case PhyCC2420M$PHY_RADIO_TX_END: 
                  PhyCC2420M$IPhyED$Confirm(context, PHY_TX_ON, 0x0, uniData);
      return;
      default: 
        break;
    }
  status.raw = PhyCC2420M$IChipcon$cmd(0x00);
  if (status.value.rssi_valid) {
      TCC2420_RSSI rssi;
      signed relativePower;
      TPhyEnergyLevel energyLevel;

#line 440
      rssi.raw = PhyCC2420M$IChipcon$read(CC2420_RSSI_ADDRESS);
      relativePower = rssi.value.rssi_val + RSSI_OFFSET - RSSI_MIN;
      if (relativePower < 10) {
          energyLevel = 0;
        }
      else {
#line 444
        if (relativePower < 40 + 10) {
            energyLevel = 0xff / 40 * relativePower - 0xff / 4;
          }
        else {
#line 447
          energyLevel = 0xff;
          }
        }
#line 448
      PhyCC2420M$IPhyED$Confirm(context, PHY_SUCCESS, energyLevel, uniData);
      return;
    }
  PhyCC2420M$IPhyED$Confirm(context, PHY_TRX_OFF, 0x0, uniData);
}

# 70 "../../../zigzag/ZigNetM.nc"
static inline  void ZigNetM$NLME_Reset$confirm(NwkStatus status)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __net_exit();
    }
#line 74
  return;
}

# 38 "/home/max/tinyos/tinyos-1.x/tos/platform/bbox/TimeCastM.nc"
static inline   uint32_t TimeCastM$ITimeCast$MillisToJiffies(TMilliSec millis)
{
  return (uint32_t )((uint64_t )millis * 125) >> 2;
}

# 14 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimeCast.nc"
inline static   uint32_t TimerSymbol2M$ITimeCast$MillisToJiffies(TMilliSec arg_0x40a3c010){
#line 14
  unsigned long result;
#line 14

#line 14
  result = TimeCastM$ITimeCast$MillisToJiffies(arg_0x40a3c010);
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
inline static  result_t ZigRouterM$TimerMilli$setOneShot(int32_t arg_0x40887668){
#line 28
  unsigned char result;
#line 28

#line 28
  result = TimerSymbol2M$TimerMilli$setOneShot(0U, arg_0x40887668);
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 46 "../../../zigzag/ZigRouterM.nc"
static inline  void ZigRouterM$NLME_Reset$confirm(NwkStatus status)
{
  ZigRouterM$TimerMilli$setOneShot((ZigRouterM$idle_scan ? ZigRouterM$IDLE_SCAN_INTERVAL : ZigRouterM$REJOIN_INTERVAL) * 1000);

  return;
}

# 16 "../../../zigzag/ZigBee/interface/NLME_Reset.nc"
inline static  void NLME_ResetM$NLME_Reset$confirm(NwkStatus arg_0x40867458){
#line 16
  ZigRouterM$NLME_Reset$confirm(arg_0x40867458);
#line 16
  ZigNetM$NLME_Reset$confirm(arg_0x40867458);
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
inline static  void NwkResetMacM$NLME_Reset$confirm(NwkStatus arg_0x40867458){
#line 16
  NLME_ResetM$MacReset$confirm(arg_0x40867458);
#line 16
}
#line 16
# 27 "../../../zigzag/ZigBee/implementation/NwkResetMacM.nc"
static inline  result_t NwkResetMacM$IMacRESET2$Confirm(TMacStatus status)
{
  NwkResetMacM$NLME_Reset$confirm(status);
  return SUCCESS;
}

# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
inline static  result_t MacDeviceAttrM$IMacRESET$Confirm(TMacStatus arg_0x409c6ac8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = NwkResetMacM$IMacRESET2$Confirm(arg_0x409c6ac8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 291 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  void MacDeviceAttrM$confirmReset(void)
{
  MacDeviceAttrM$IMacRESET$Confirm(MAC_SUCCESS);
}

# 6 "../../../zigzag/IEEE802_15_4/Phy/public/IPhySET_TRX_STATE.nc"
inline static   result_t MacDeviceAttrM$IPhySET_TRX_STATE$Request(TPhyStatus arg_0x40b27940, const TUniData arg_0x40b27ae8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = PhyCC2420M$IPhySET_TRX_STATE$Request(7U, arg_0x40b27940, arg_0x40b27ae8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 20 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetDefaultmacAssociationPermit(void){
#line 20
  enum __nesc_unnamed4286 result;
#line 20

#line 20
  result = MacAssocDeviceM$IMacAssocAttr$SetDefaultmacAssociationPermit();
#line 20

#line 20
  return result;
#line 20
}
#line 20
#line 16
inline static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetDefaultmacCoordShortAddress(void){
#line 16
  enum __nesc_unnamed4286 result;
#line 16

#line 16
  result = MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordShortAddress();
#line 16

#line 16
  return result;
#line 16
}
#line 16
#line 10
inline static  TMacStatus MacDeviceAttrM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress(void){
#line 10
  enum __nesc_unnamed4286 result;
#line 10

#line 10
  result = MacAssocDeviceM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress();
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 29 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacBeaconTxTime(void){
#line 29
  enum __nesc_unnamed4286 result;
#line 29

#line 29
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconTxTime();
#line 29

#line 29
  return result;
#line 29
}
#line 29
#line 23
inline static  TMacStatus MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacBSN(void){
#line 23
  enum __nesc_unnamed4286 result;
#line 23

#line 23
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBSN();
#line 23

#line 23
  return result;
#line 23
}
#line 23
#line 19
inline static  TMacStatus MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacBeaconPayload(void){
#line 19
  enum __nesc_unnamed4286 result;
#line 19

#line 19
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacBeaconPayload();
#line 19

#line 19
  return result;
#line 19
}
#line 19
#line 9
inline static  TMacStatus MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacAutoRequest(void){
#line 9
  enum __nesc_unnamed4286 result;
#line 9

#line 9
  result = MacBeaconAttrCoordM$IMacBeaconAttr$SetDefaultmacAutoRequest();
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 26 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacCAPAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacCAPAttr$SetDefaultmacMinBE(void){
#line 26
  enum __nesc_unnamed4286 result;
#line 26

#line 26
  result = MacCAPM$IMacCAPAttr$SetDefaultmacMinBE();
#line 26

#line 26
  return result;
#line 26
}
#line 26
#line 20
inline static  TMacStatus MacDeviceAttrM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs(void){
#line 20
  enum __nesc_unnamed4286 result;
#line 20

#line 20
  result = MacCAPM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs();
#line 20

#line 20
  return result;
#line 20
}
#line 20
#line 14
inline static  TMacStatus MacDeviceAttrM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods(void){
#line 14
  enum __nesc_unnamed4286 result;
#line 14

#line 14
  result = MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods();
#line 14

#line 14
  return result;
#line 14
}
#line 14
#line 8
inline static  TMacStatus MacDeviceAttrM$IMacCAPAttr$SetDefaultmacBattLifeExt(void){
#line 8
  enum __nesc_unnamed4286 result;
#line 8

#line 8
  result = MacCAPM$IMacCAPAttr$SetDefaultmacBattLifeExt();
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 211 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
static inline  TMacStatus MacExtractM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void)
{
  return MAC_UNSUPPORTED_ATTRIBUTE;
}

# 10 "../../../zigzag/IEEE802_15_4/MacPool/public/IMacPoolAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime(void){
#line 10
  enum __nesc_unnamed4286 result;
#line 10

#line 10
  result = MacExtractM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime();
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 12 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacStatus MacDeviceAttrM$IMacSuperframeAttr$SetDefaultmacSuperframeOrder(void){
#line 12
  enum __nesc_unnamed4286 result;
#line 12

#line 12
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetDefaultmacSuperframeOrder();
#line 12

#line 12
  return result;
#line 12
}
#line 12
#line 8
inline static  TMacStatus MacDeviceAttrM$IMacSuperframeAttr$SetDefaultmacBeaconOrder(void){
#line 8
  enum __nesc_unnamed4286 result;
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
inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacPANId(void){
#line 8
  enum __nesc_unnamed4286 result;
#line 8

#line 8
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacPANId();
#line 8

#line 8
  return result;
#line 8
}
#line 8




inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacShortAddress(void){
#line 12
  enum __nesc_unnamed4286 result;
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
inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle(void){
#line 32
  enum __nesc_unnamed4286 result;
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
inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacDSN(void){
#line 20
  enum __nesc_unnamed4286 result;
#line 20

#line 20
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacDSN();
#line 20

#line 20
  return result;
#line 20
}
#line 20




inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode(void){
#line 24
  enum __nesc_unnamed4286 result;
#line 24

#line 24
  result = MacCommonAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode();
#line 24

#line 24
  return result;
#line 24
}
#line 24




inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacSecurityMode(void){
#line 28
  enum __nesc_unnamed4286 result;
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
inline static  TMacStatus MacDeviceAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration(void){
#line 16
  enum __nesc_unnamed4286 result;
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
static __inline void MacDeviceAttrM$SetDefaultAttributes(void)
{
  MacDeviceAttrM$IMacCommonAttr$SetDefaultmacAckWaitDuration();
  MacDeviceAttrM$IMacCommonAttr$SetDefaultmacSecurityMode();
  MacDeviceAttrM$IMacCommonAttr$SetDefaultmacPromiscuousMode();
  MacDeviceAttrM$IMacCommonAttr$SetDefaultmacDSN();
  MacDeviceAttrM$IMacCommonAttr$SetDefaultmacRxOnWhenIdle();
  MacDeviceAttrM$IMacCommonAttr$SetDefaultmacShortAddress();
  MacDeviceAttrM$IMacCommonAttr$SetDefaultmacPANId();

  MacDeviceAttrM$IMacSuperframeAttr$SetDefaultmacBeaconOrder();
  MacDeviceAttrM$IMacSuperframeAttr$SetDefaultmacSuperframeOrder();

  MacDeviceAttrM$IMacPoolAttr$SetDefaultmacTransactionPersistenceTime();

  MacDeviceAttrM$IMacCAPAttr$SetDefaultmacBattLifeExt();
  MacDeviceAttrM$IMacCAPAttr$SetDefaultmacBattLifeExtPeriods();
  MacDeviceAttrM$IMacCAPAttr$SetDefaultmaxMaxCSMABackoffs();
  MacDeviceAttrM$IMacCAPAttr$SetDefaultmacMinBE();

  MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacAutoRequest();
  MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacBeaconPayload();
  MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacBSN();
  MacDeviceAttrM$IMacBeaconAttr$SetDefaultmacBeaconTxTime();

  MacDeviceAttrM$IMacAssocAttr$SetDefaultmacCoordExtendedAddress();
  MacDeviceAttrM$IMacAssocAttr$SetDefaultmacCoordShortAddress();
  MacDeviceAttrM$IMacAssocAttr$SetDefaultmacAssociationPermit();
}

# 11 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyAttr.nc"
inline static  result_t MacAssocDeviceM$IPhyAttr$SetAutoAck(bool arg_0x40b40ec0){
#line 11
  unsigned char result;
#line 11

#line 11
  result = PhyCC2420M$IPhyAttr$SetAutoAck(arg_0x40b40ec0);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 130 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  void MacAssocDeviceM$Reset$reset(void)
{
  MacAssocDeviceM$IPhyAttr$SetAutoAck(FALSE);
  MacAssocDeviceM$assoc.state = MacAssocDeviceM$MAC_ASSOC_IDLE;
  return;
}

# 193 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
static inline  void MacExtractM$Reset$reset(void)
{
  MacExtractM$extractState = MacExtractM$MAC_EXTRACT_IDLE;
  return;
}

# 136 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
static inline  result_t MacSyncM$StdControl$stop(void)
{
  MacSyncM$InitialTimer$Stop();
  MacSyncM$first_radio_on = FALSE;
  MacSyncM$beacon_expected = FALSE;
  return SUCCESS;
}

#line 120
static inline  void MacSyncM$Reset$reset(void)
{
  MacSyncM$StdControl$stop();
  MacSyncM$StdControl$start();
}

# 46 "/home/max/tinyos/tinyos-1.x/tos/interfaces/Reset.nc"
inline static  void MacDeviceAttrM$IMacReset$reset(void){
#line 46
  PhyPoolM$IPhyPoolReset$reset();
#line 46
  MacSuperframesM$Reset$reset();
#line 46
  MacSyncM$Reset$reset();
#line 46
  MacSuperframeAttrC$Reset$reset();
#line 46
  MacCAPM$Reset$reset(MAC_PARENT_SF);
#line 46
  MacExtractM$Reset$reset();
#line 46
  MacAssocDeviceM$Reset$reset();
#line 46
  MacDataM$Reset$reset();
#line 46
}
#line 46
# 296 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static inline  result_t MacDeviceAttrM$IMacRESET$Request(bool setDefaultPIB)
{
  MacDeviceAttrM$IMacReset$reset();
  if (setDefaultPIB) {
      MacDeviceAttrM$SetDefaultAttributes();
    }
  MacDeviceAttrM$IPhySET_TRX_STATE$Request(PHY_FORCE_TRX_OFF, 0);
  return TOS_post(MacDeviceAttrM$confirmReset);
}

# 5 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
inline static  result_t NwkResetMacM$IMacRESET2$Request(bool arg_0x409c6640){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacDeviceAttrM$IMacRESET$Request(arg_0x409c6640);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 21 "../../../zigzag/ZigBee/implementation/NwkResetMacM.nc"
static inline  result_t NwkResetMacM$IMacRESET1$Confirm(TMacStatus status)
{
  NwkResetMacM$IMacRESET2$Request(TRUE);
  return SUCCESS;
}

# 7 "../../../zigzag/IEEE802_15_4/Mac/public/IMacRESET.nc"
inline static  result_t MacCoordAttrM$IMacRESET$Confirm(TMacStatus arg_0x409c6ac8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = NwkResetMacM$IMacRESET1$Confirm(arg_0x409c6ac8);
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
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t PhyCC2420M$IPhyPool$Alloc(const uint8_t arg_0x40b3e6b8, const uint16_t arg_0x40b3e860, TPhyPoolHandle *const arg_0x40b3ea58){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyPoolM$IPhyPool$Alloc(arg_0x40b3e6b8, arg_0x40b3e860, arg_0x40b3ea58);
#line 5

#line 5
  return result;
#line 5
}
#line 5
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
  union __nesc_unnamed4434 {
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
inline static   void MSP430TimerM$CaptureA0$captured(uint16_t arg_0x40746d38){
#line 74
  MSP430TimerM$CaptureA0$default$captured(arg_0x40746d38);
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
inline static   void MSP430TimerM$CaptureA1$captured(uint16_t arg_0x40746d38){
#line 74
  MSP430TimerM$CaptureA1$default$captured(arg_0x40746d38);
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
inline static   void MSP430TimerM$CaptureA2$captured(uint16_t arg_0x40746d38){
#line 74
  MSP430TimerM$CaptureA2$default$captured(arg_0x40746d38);
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
inline static   void MSP430TimerM$CaptureB0$captured(uint16_t arg_0x40746d38){
#line 74
  MSP430TimerM$CaptureB0$default$captured(arg_0x40746d38);
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
inline static   TSysTime PhyCC2420M$ITimeCast$JiffiesToSymbols(uint32_t arg_0x40a3eb98){
#line 7
  unsigned long result;
#line 7

#line 7
  result = TimeCastM$ITimeCast$JiffiesToSymbols(arg_0x40a3eb98);
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
inline static   result_t HPLCC2420InterruptM$SFD$captured(uint16_t arg_0x40b9d360){
#line 53
  unsigned char result;
#line 53

#line 53
  result = PhyCC2420M$IChipconSFD$captured(arg_0x40b9d360);
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
inline static   void MSP430TimerM$CaptureB1$captured(uint16_t arg_0x40746d38){
#line 74
  HPLCC2420InterruptM$SFDCapture$captured(arg_0x40746d38);
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
inline static   void MSP430TimerM$CaptureB2$captured(uint16_t arg_0x40746d38){
#line 74
  MSP430TimerM$CaptureB2$default$captured(arg_0x40746d38);
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
inline static   result_t TimerSymbolAsyncM$ITimerSymbolAsync$Fired(uint8_t arg_0x40a93770, TUniData arg_0x40a37b28){
#line 16
  unsigned char result;
#line 16

#line 16
  switch (arg_0x40a93770) {
#line 16
    case 0U:
#line 16
      result = MacSuperframesM$Timer$Fired(arg_0x40a37b28);
#line 16
      break;
#line 16
    default:
#line 16
      result = TimerSymbolAsyncM$ITimerSymbolAsync$default$Fired(arg_0x40a93770, arg_0x40a37b28);
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
inline static   void MSP430TimerM$CaptureB3$captured(uint16_t arg_0x40746d38){
#line 74
  MSP430TimerM$CaptureB3$default$captured(arg_0x40746d38);
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
inline static   void MSP430TimerM$CaptureB4$captured(uint16_t arg_0x40746d38){
#line 74
  MSP430TimerM$CaptureB4$default$captured(arg_0x40746d38);
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
inline static   void MSP430TimerM$CaptureB5$captured(uint16_t arg_0x40746d38){
#line 74
  MSP430TimerM$CaptureB5$default$captured(arg_0x40746d38);
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
inline static   void MSP430TimerM$CaptureB6$captured(uint16_t arg_0x40746d38){
#line 74
  MSP430TimerM$CaptureB6$default$captured(arg_0x40746d38);
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
inline static   result_t HPLUSART0M$USARTData$rxDone(uint8_t arg_0x40c97b88){
#line 53
  unsigned char result;
#line 53

#line 53
  result = HPLUSART0M$USARTData$default$rxDone(arg_0x40c97b88);
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
inline static   void HPLCC2420InterruptM$FIFOInterrupt$clear(void){
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
inline static   void HPLCC2420InterruptM$FIFOInterrupt$disable(void){
#line 35
  MSP430InterruptM$Port10$disable();
#line 35
}
#line 35
# 43 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/HPLCC2420Interrupt.nc"
inline static   result_t PhyCC2420M$IChipconFIFOSignal$startWait(bool arg_0x40b9b010){
#line 43
  unsigned char result;
#line 43

#line 43
  result = HPLCC2420InterruptM$FIFO$startWait(arg_0x40b9b010);
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
inline static   void MSP430InterruptM$Port10$fired(void){
#line 59
  HPLCC2420InterruptM$FIFOInterrupt$fired();
#line 59
}
#line 59
# 178 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port11$clear(void)
#line 178
{
#line 178
  MSP430InterruptM$P1IFG &= ~(1 << 1);
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$clear(void){
#line 40
  MSP430InterruptM$Port11$clear();
#line 40
}
#line 40
# 147 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port11$disable(void)
#line 147
{
#line 147
  MSP430InterruptM$P1IE &= ~(1 << 1);
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$disable(void){
#line 35
  MSP430InterruptM$Port11$disable();
#line 35
}
#line 35
# 7 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacSend.nc"
inline static  result_t MacPendingM$IMacCSMA$Send(const TMacFrame *const arg_0x40f051f8, const TUniData arg_0x40f053b0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(0U, arg_0x40f051f8, arg_0x40f053b0);
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
inline static  TMacFrame *MacPendingM$IMacPool$GetFrame(const TMacPoolHandle arg_0x40ef19c0){
#line 11
  struct __nesc_unnamed4316 *result;
#line 11

#line 11
  result = MacPoolM$IMacPool$GetFrame(arg_0x40ef19c0);
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
inline static  bool MacPoolM$IMacAddress$Equal(const TMacAddress *const arg_0x408ffd30, const TMacAddress *const arg_0x408fd010){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacAddressM$IMacAddress$Equal(arg_0x408ffd30, arg_0x408fd010);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 28 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacPoolM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40dac218, TMacAddress *const arg_0x40dac408){
#line 28
  unsigned char result;
#line 28

#line 28
  result = MacFrameFormatM$IMacFrame$GetDstAddress(arg_0x40dac218, arg_0x40dac408);
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
inline static  bool MacPendingM$SearchDstAddress(const TMacAddress arg_0x40fbf5f0, TMacPoolHandle *const arg_0x40fbf7f0){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacPoolM$SearchDstAddress(arg_0x40fbf5f0, arg_0x40fbf7f0);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 34 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacPendingM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010){
#line 34
  unsigned char result;
#line 34

#line 34
  result = MacFrameFormatM$IMacFrame$GetSrcAddress(arg_0x40dabdd0, arg_0x40da9010);
#line 34

#line 34
  return result;
#line 34
}
#line 34
#line 15
inline static  result_t MacPendingM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40db29a0, bool *const arg_0x40db2b88){
#line 15
  unsigned char result;
#line 15

#line 15
  result = MacFrameFormatM$IMacFrame$GetAckRequest(arg_0x40db29a0, arg_0x40db2b88);
#line 15

#line 15
  return result;
#line 15
}
#line 15
#line 42
inline static  result_t MacPendingM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40da8530, arg_0x40da8718);
#line 42

#line 42
  return result;
#line 42
}
#line 42
#line 39
inline static  result_t MacPendingM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, TMacPayloadLength *const arg_0x40da8010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40da9df0, arg_0x40da8010);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 6
inline static  result_t MacPendingM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40db5010, arg_0x40db5200);
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
    enum __nesc_unnamed4435 {
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

# 349 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline  result_t MacDataM$IMacReceiveOwn$Receive(TMacFrame *const pmacFrame)
{
  return MacDataM$Receive(MacDataM$OWN_SF, pmacFrame);
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
inline static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacReceive$Receive(TMacFrame *const arg_0x40f024f8){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacDataM$IMacReceiveOwn$Receive(arg_0x40f024f8);
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

# 344 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static inline  result_t MacDataM$IMacReceiveParent$Receive(TMacFrame *const pmacFrame)
{
  return MacDataM$Receive(MacDataM$PARENT_SF, pmacFrame);
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
inline static  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacReceive$Receive(TMacFrame *const arg_0x40f024f8){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacDataM$IMacReceiveParent$Receive(arg_0x40f024f8);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 49 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPInterfaceSwitchM.nc"
static inline  result_t /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$Receive(const uint8_t _sfIndex, TMacFrame *const pMacFrame)
{
  if (_sfIndex == 1) {
    return /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$IMacReceive$Receive(pMacFrame);
    }
#line 53
  return SUCCESS;
}

# 51 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
inline static  result_t MacCAPM$Receive(const uint8_t arg_0x40f23f28, TMacFrame *const arg_0x40f22130){
#line 51
  unsigned char result;
#line 51

#line 51
  result = /*MacParentCAPC.MacCAPParentSwitch*/MacCAPInterfaceSwitchM$1$Receive(arg_0x40f23f28, arg_0x40f22130);
#line 51
  result = rcombine(result, /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$Receive(arg_0x40f23f28, arg_0x40f22130));
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
inline static  result_t NLME_JoinParentM$IMacASSOCIATE$Response(TMacExtendedAddress arg_0x408cbc68, TMacShortAddress arg_0x408cbe18, TMacStatus arg_0x408ca010, bool arg_0x408ca1a8){
#line 51
  unsigned char result;
#line 51

#line 51
  result = MacAssocCoordM$IMacASSOCIATE$Response(arg_0x408cbc68, arg_0x408cbe18, arg_0x408ca010, arg_0x408ca1a8);
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
inline static  result_t NLME_JoinParentM$NwkAddressing$allocEdAddress(NwkAddr *arg_0x408d66e0){
#line 15
  unsigned char result;
#line 15

#line 15
  result = NwkAddressingM$NwkAddressing$allocEdAddress(arg_0x408d66e0);
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
inline static  result_t NLME_JoinParentM$NwkAddressing$allocRouterAddress(NwkAddr *arg_0x408d6b90){
#line 17
  unsigned char result;
#line 17

#line 17
  result = NwkAddressingM$NwkAddressing$allocRouterAddress(arg_0x408d6b90);
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
inline static  result_t MacAssocCoordM$IMacASSOCIATE$Indication(TMacExtendedAddress arg_0x408cb068, TCapabilityInformation arg_0x408cb218, bool arg_0x408cb3b0, TACLEntry arg_0x408cb550){
#line 38
  unsigned char result;
#line 38

#line 38
  result = NLME_JoinParentM$IMacASSOCIATE$Indication(arg_0x408cb068, arg_0x408cb218, arg_0x408cb3b0, arg_0x408cb550);
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
inline static  result_t MacAssocCoordM$IMacFrame$GetSecurityEnabled(const TMacFrame *const arg_0x40db5dd8, bool *const arg_0x40db4010){
#line 9
  unsigned char result;
#line 9

#line 9
  result = MacFrameFormatM$IMacFrame$GetSecurityEnabled(arg_0x40db5dd8, arg_0x40db4010);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacAssocCoordM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacAddressM$IMacAddress$GetExtendedAddress(arg_0x409003c0, arg_0x40900570);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 34 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocCoordM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010){
#line 34
  unsigned char result;
#line 34

#line 34
  result = MacFrameFormatM$IMacFrame$GetSrcAddress(arg_0x40dabdd0, arg_0x40da9010);
#line 34

#line 34
  return result;
#line 34
}
#line 34








inline static  result_t MacAssocCoordM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40da8530, arg_0x40da8718);
#line 42

#line 42
  return result;
#line 42
}
#line 42
#line 39
inline static  result_t MacAssocCoordM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, TMacPayloadLength *const arg_0x40da8010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40da9df0, arg_0x40da8010);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 6
inline static  result_t MacAssocCoordM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40db5010, arg_0x40db5200);
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
inline static  result_t /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$SendFromPhyPool(const uint8_t arg_0x40f92708, const TPhyPoolHandle arg_0x40f928c8, const uint8_t arg_0x40f92a78){
#line 21
  unsigned char result;
#line 21

#line 21
  result = MacCAPM$SendFromPhyPool(arg_0x40f92708, arg_0x40f928c8, arg_0x40f92a78);
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
inline static  result_t MacScanBeaconM$CSMA$SendFromPhyPool(const TPhyPoolHandle arg_0x40f05868){
#line 10
  unsigned char result;
#line 10

#line 10
  result = /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$SendFromPhyPool(3U, arg_0x40f05868);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *MacScanBeaconM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010){
#line 8
  struct __nesc_unnamed4281 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b3d010);
#line 8

#line 8
  return result;
#line 8
}
#line 8
# 17 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanBeaconM.nc"
inline static  void MacScanBeaconM$packBeacon(TPhyFrame *arg_0x41088848, uint64_t *arg_0x410889f0){
#line 17
  MacBeaconPackerM$packBeacon(arg_0x41088848, arg_0x410889f0);
#line 17
}
#line 17
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   result_t MacScanBeaconM$IPhyPool$Alloc(const uint8_t arg_0x40b3e6b8, const uint16_t arg_0x40b3e860, TPhyPoolHandle *const arg_0x40b3ea58){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyPoolM$IPhyPool$Alloc(arg_0x40b3e6b8, arg_0x40b3e860, arg_0x40b3ea58);
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
inline static  result_t MacScanBeaconM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40da8530, arg_0x40da8718);
#line 42

#line 42
  return result;
#line 42
}
#line 42
#line 39
inline static  result_t MacScanBeaconM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, TMacPayloadLength *const arg_0x40da8010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40da9df0, arg_0x40da8010);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 6
inline static  result_t MacScanBeaconM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40db5010, arg_0x40db5200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacScanBeaconM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dd3cf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacScanBeaconM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e57ab8);
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

# 210 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline void MacAssocDeviceM$AssocSuccessDone(TMacShortAddress assocShortAddress, TMacStatus macStatus)
{
  MacAssocDeviceM$IPhyAttr$SetAutoAck(TRUE);
  MacAssocDeviceM$assoc.state = MacAssocDeviceM$MAC_ASSOC_IDLE;
  MacAssocDeviceM$IMacASSOCIATE$Confirm(assocShortAddress, macStatus);
}

# 8 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacAssocDeviceM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacAddressM$IMacAddress$GetShortAddress(arg_0x40901ce0, arg_0x40901e90);
#line 8

#line 8
  return result;
#line 8
}
#line 8
#line 6
inline static  TMacAddressMode MacAssocDeviceM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8){
#line 6
  enum __nesc_unnamed4299 result;
#line 6

#line 6
  result = MacAddressM$IMacAddress$GetAddressMode(arg_0x409017d8);
#line 6

#line 6
  return result;
#line 6
}
#line 6




inline static  result_t MacAssocDeviceM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacAddressM$IMacAddress$GetExtendedAddress(arg_0x409003c0, arg_0x40900570);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 34 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocDeviceM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010){
#line 34
  unsigned char result;
#line 34

#line 34
  result = MacFrameFormatM$IMacFrame$GetSrcAddress(arg_0x40dabdd0, arg_0x40da9010);
#line 34

#line 34
  return result;
#line 34
}
#line 34








inline static  result_t MacAssocDeviceM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40da8530, arg_0x40da8718);
#line 42

#line 42
  return result;
#line 42
}
#line 42
#line 39
inline static  result_t MacAssocDeviceM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, TMacPayloadLength *const arg_0x40da8010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40da9df0, arg_0x40da8010);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 6
inline static  result_t MacAssocDeviceM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40db5010, arg_0x40db5200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 308 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  result_t MacAssocDeviceM$IAssocResponse$Receive(TMacFrame *const pMacFrame)
{
  result_t result;

#line 311
  if (MacAssocDeviceM$assoc.state != MacAssocDeviceM$MAC_ASSOC_RECEIVE_RESPONSE) {
    return FAIL;
    }
#line 313
  {
#line 313
    TMacFrameType frameType;

#line 314
    result = MacAssocDeviceM$IMacFrame$GetFrameType(pMacFrame, &frameType);
    if (result != SUCCESS || frameType != MAC_FRAME_TYPE_COMMAND) {
      return FAIL;
      }
  }
#line 318
  {
#line 318
    TMacPayloadLength payloadLength;

#line 319
    result = MacAssocDeviceM$IMacFrame$GetPayloadLength(pMacFrame, &payloadLength);
    if (result != SUCCESS || payloadLength != MacAssocDeviceM$ASSOC_RESPONSE_PAYLOAD_LEN) {
      return FAIL;
      }
  }
#line 323
  {
#line 323
    uint8_t payload[MacAssocDeviceM$ASSOC_RESPONSE_PAYLOAD_LEN];

#line 324
    result = MacAssocDeviceM$IMacFrame$GetPayload(pMacFrame, payload);
    if (result != SUCCESS || payload[0] != MAC_CMD_ASSOC_RESPONSE) {
      return FAIL;
      }
#line 327
    {
#line 327
      TMacStatus status;
      TMacShortAddress shortAddress;

#line 329
      * (uint8_t *)&shortAddress = payload[1];
      *((uint8_t *)&shortAddress + 1) = payload[2];
      * (uint8_t *)&status = payload[3];
      status &= 0xFF;

      if (status == MAC_SUCCESS) {
          TMacExtendedAddress ea;
          TMacAddress ma;

#line 337
          {
#line 337
            MacAssocDeviceM$IMacFrame$GetSrcAddress(pMacFrame, &ma);
          }
#line 337
          ;
          {
#line 338
            MacAssocDeviceM$IMacAddress$GetExtendedAddress(&ea, ma);
          }
#line 338
          ;
          MacAssocDeviceM$assoc.macCoordExtendedAddress = ea;
          if (MacAssocDeviceM$IMacAddress$GetAddressMode(MacAssocDeviceM$assoc.coordAddress) == MAC_ADDRESS_SHORT) {
              TMacShortAddress sa;

#line 342
              MacAssocDeviceM$IMacAddress$GetShortAddress(&sa, MacAssocDeviceM$assoc.coordAddress);
              MacAssocDeviceM$assoc.macCoordShortAddress = sa;
            }
        }
      MacAssocDeviceM$AssocSuccessDone(shortAddress, status);
      return SUCCESS;
    }
  }
  return FAIL;
}

# 7 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
inline static  result_t MacAssocDeviceM$IReceiveResponseTimer$SetOneShot(TSysTime arg_0x408d0340, TUniData arg_0x408d04c8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = TimerSymbol2M$ITimerSymbol$SetOneShot(8U, arg_0x408d0340, arg_0x408d04c8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 6 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacBeaconOrder MacAssocDeviceM$IMacSuperframeAttr$GetmacBeaconOrder(TMacStatus *const arg_0x40e57ab8){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacSuperframeAttrC$IMacSuperframeAttr$GetmacBeaconOrder(arg_0x40e57ab8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 21 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocDeviceM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40daf558, TMacSequenceNumber *const arg_0x40daf760){
#line 21
  unsigned char result;
#line 21

#line 21
  result = MacFrameFormatM$IMacFrame$GetSequenceNumber(arg_0x40daf558, arg_0x40daf760);
#line 21

#line 21
  return result;
#line 21
}
#line 21
# 274 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  result_t MacAssocDeviceM$IAssocAck$Receive(TMacFrame *const pMacFrame)
{
  if (MacAssocDeviceM$assoc.state == MacAssocDeviceM$MAC_ASSOC_WAIT_ACK) {
      TMacFrameType frameType;
      result_t result = MacAssocDeviceM$IMacFrame$GetFrameType(pMacFrame, &frameType);

#line 279
      if (result != SUCCESS || frameType != MAC_FRAME_TYPE_ACK) {
        return FAIL;
        }
#line 281
      {
#line 281
        TMacSequenceNumber seqNum;

#line 282
        result = MacAssocDeviceM$IMacFrame$GetSequenceNumber(pMacFrame, &seqNum);
        if (result != SUCCESS || seqNum != MacAssocDeviceM$assoc.seqNum) {
          return FAIL;
          }
      }
#line 286
      {
        enum __nesc_unnamed4436 {
#line 287
          MAX_ORDER = 15
        };
#line 288
        TMacBeaconOrder bo = MacAssocDeviceM$IMacSuperframeAttr$GetmacBeaconOrder(NULL);
        TSysTime frameResponseTime = (uint32_t )MAC_ARESPONSE_WAIT_TIME
         + (uint32_t )MAC_AMAX_FRAME_RESPONSE_TIME
         + (bo < MAX_ORDER ? (uint32_t )MAC_ABASE_SUPERFRAME_DURATION << bo : 0);

#line 292
        result = MacAssocDeviceM$IReceiveResponseTimer$SetOneShot(frameResponseTime, 0);
        if (result == SUCCESS) {
            MacAssocDeviceM$assoc.state = MacAssocDeviceM$MAC_ASSOC_RECEIVE_RESPONSE;
            return SUCCESS;
          }
      }
    }

  return FAIL;
}

# 61 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
static inline  result_t MacFrameFormatM$IMacFrame$GetFramePending(const TMacFrame *const pMacFrame, 
bool *const pFramePending)
{
  if (pMacFrame != NULL && pFramePending != NULL) {
      *pFramePending = 1 == pMacFrame->frameControl.lsb.value.framePending;
      return SUCCESS;
    }
  return FAIL;
}

# 12 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacExtractM$IMacFrame$GetFramePending(const TMacFrame *const arg_0x40db4bd8, bool *const arg_0x40db4dc0){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacFrameFormatM$IMacFrame$GetFramePending(arg_0x40db4bd8, arg_0x40db4dc0);
#line 12

#line 12
  return result;
#line 12
}
#line 12









inline static  result_t MacExtractM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40daf558, TMacSequenceNumber *const arg_0x40daf760){
#line 21
  unsigned char result;
#line 21

#line 21
  result = MacFrameFormatM$IMacFrame$GetSequenceNumber(arg_0x40daf558, arg_0x40daf760);
#line 21

#line 21
  return result;
#line 21
}
#line 21
#line 6
inline static  result_t MacExtractM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40db5010, arg_0x40db5200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 152 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
static inline  result_t MacExtractM$IMacReceive$Receive(TMacFrame *const pMacFrame)
{
  if (MacExtractM$MAC_EXTRACT_WAIT_ACK != MacExtractM$extractState) {
#line 154
    return FAIL;
    }
#line 155
  {
#line 155
    TMacFrameType frameType;
    result_t result = MacExtractM$IMacFrame$GetFrameType(pMacFrame, &frameType);

#line 157
    if (SUCCESS != result || MAC_FRAME_TYPE_ACK != frameType) {
      return FAIL;
      }
  }
#line 160
  {
#line 160
    TMacSequenceNumber recvSeqNum;
    result_t result = MacExtractM$IMacFrame$GetSequenceNumber(pMacFrame, &recvSeqNum);

#line 162
    if (SUCCESS != result || recvSeqNum != MacExtractM$extract.seqNum) {
      return FAIL;
      }
  }
#line 165
  {
#line 165
    bool framePending;
    result_t result = MacExtractM$IMacFrame$GetFramePending(pMacFrame, &framePending);

#line 167
    if (SUCCESS == result) {
        MacExtractM$ExtractDone(MAC_SUCCESS, framePending);
        return SUCCESS;
      }
  }
  return FAIL;
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
inline static  result_t MacRxM$DataReceive$Receive(TMacFrame *const arg_0x40f024f8){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacExtractM$IMacReceive$Receive(arg_0x40f024f8);
#line 5
  result = rcombine(result, MacAssocDeviceM$IAssocAck$Receive(arg_0x40f024f8));
#line 5
  result = rcombine(result, MacAssocDeviceM$IAssocResponse$Receive(arg_0x40f024f8));
#line 5
  result = rcombine(result, MacScanBeaconM$IMacReceive$Receive(arg_0x40f024f8));
#line 5
  result = rcombine(result, MacAssocCoordM$IAssocRequest$Receive(arg_0x40f024f8));
#line 5
  result = rcombine(result, MacCAPM$IMacReceive$Receive(arg_0x40f024f8));
#line 5
  result = rcombine(result, MacPendingM$IMacReceive$Receive(arg_0x40f024f8));
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 8 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t NwkBeaconChildM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacAddressM$IMacAddress$GetShortAddress(arg_0x40901ce0, arg_0x40901e90);
#line 8

#line 8
  return result;
#line 8
}
#line 8
#line 6
inline static  TMacAddressMode NwkBeaconChildM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8){
#line 6
  enum __nesc_unnamed4299 result;
#line 6

#line 6
  result = MacAddressM$IMacAddress$GetAddressMode(arg_0x409017d8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 143 "../../../zigzag/ZigBee/implementation/NeighborTableM.nc"
static inline  result_t NeighborTableM$NeighborTable$getFreePtr(NwkNeighbor **pneighbor)
{
  if (NeighborTableM$free_idx < 20 && (NeighborTableM$Table[NeighborTableM$free_idx].networkAddr == 0xFFFF && NeighborTableM$Table[NeighborTableM$free_idx].extendedAddr == 0xFFFFFFFFFFFFFFFFLL)) 
    {
      *pneighbor = NeighborTableM$Table + NeighborTableM$free_idx;
      NeighborTableM$free_idx++;
      return SUCCESS;
    }
  else 
    {
      int i;

#line 154
      for (i = 0; i < 20; i++) 
        {
          if (NeighborTableM$Table[i].networkAddr == 0xFFFF && NeighborTableM$Table[i].extendedAddr == 0xFFFFFFFFFFFFFFFFLL) 
            {
              *pneighbor = NeighborTableM$Table + i;
              return SUCCESS;
            }
        }
    }
  return FAIL;
}

# 34 "../../../zigzag/ZigBee/implementation/NeighborTable.nc"
inline static  result_t NwkBeaconChildM$NeighborTable$getFreePtr(NwkNeighbor **arg_0x40869838){
#line 34
  unsigned char result;
#line 34

#line 34
  result = NeighborTableM$NeighborTable$getFreePtr(arg_0x40869838);
#line 34

#line 34
  return result;
#line 34
}
#line 34
#line 29
inline static  result_t NwkBeaconChildM$NeighborTable$getByAddr(TMacAddress arg_0x4086bc20, NwkNeighbor **arg_0x4086bdf0){
#line 29
  unsigned char result;
#line 29

#line 29
  result = NeighborTableM$NeighborTable$getByAddr(arg_0x4086bc20, arg_0x4086bdf0);
#line 29

#line 29
  return result;
#line 29
}
#line 29
# 14 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t NwkBeaconChildM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacAddressM$IMacAddress$SetExtendedAddress(arg_0x408ff168, arg_0x408ff328);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 25 "../../../zigzag/ZigBee/implementation/NwkBeaconChildM.nc"
static inline  void NwkBeaconChildM$IMacBEACON_NOTIFY$Indication(
TMacBSN bsn, 
TMacPanDescriptor panDescriptor, 
TMacPendAddrSpec pendAddrSpec, 
uint8_t *addrList, 
uint8_t sduLength, 
uint8_t *sdu)

{
  NwkBeaconPayload *beacon;
  NwkNeighbor *pneighbor;

  if (sduLength < sizeof(NwkBeaconPayload )) {
    return;
    }
  beacon = (NwkBeaconPayload *)sdu;
  if (beacon->prVer != NWK_PROTOCOL_VERSION) {
    return;
    }

  {
    result_t res;
    TMacAddress payload_addr;
    IEEEAddr ext_addr;




    nmemcpy(&ext_addr, & beacon->extendedPANID, 8);
    ;
    NwkBeaconChildM$IMacAddress$SetExtendedAddress(&payload_addr, ext_addr);

    res = NwkBeaconChildM$NeighborTable$getByAddr(payload_addr, &pneighbor);
    if (res != SUCCESS) 
      {
        if (!NwkBeaconChildM$NeighborTable$getFreePtr(&pneighbor)) {
          return;
          }
#line 62
        pneighbor->extendedAddr = ext_addr;

        if (NwkBeaconChildM$IMacAddress$GetAddressMode(panDescriptor.coordAddr) == MAC_ADDRESS_SHORT) 
          {
            NwkBeaconChildM$IMacAddress$GetShortAddress(& pneighbor->networkAddr, panDescriptor.coordAddr);
          }
        else 
          {
            pneighbor->networkAddr = 0xFFFF;
          }
        pneighbor->relationship = NWK_OTHER;
      }


    pneighbor->panID = panDescriptor.coordPANId;
    pneighbor->deviceType = ZIGBEE_ROUTER;

    pneighbor->depth = beacon->devDep;
    pneighbor->beaconOrder = panDescriptor.superframeSpec.beaconOrder;
    {
      bool rtrCap = beacon->rtrCap;
      bool endCap = beacon->endCap;

#line 84
      pneighbor->permitJoining = rtrCap || endCap;
    }
    pneighbor->transmitFailure = 0;
    pneighbor->lqi = panDescriptor.linkQuality;
    pneighbor->logicalChannel = panDescriptor.logicalChannel;
    pneighbor->incomingBeaconTimestamp = panDescriptor.timeStamp;
    pneighbor->lastFrameTime = panDescriptor.timeStamp;
    pneighbor->beaconTransmissionTimeOffset = ((uint32_t )beacon->txOffset[0] | ((uint32_t )beacon->txOffset[1] << 8)) | ((uint32_t )beacon->txOffset[2] << 16);
    ;
    ;
    ;


    return;
  }
}

# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBEACON_NOTIFY.nc"
inline static  void MacBeaconNotifierM$ToHigher$Indication(TMacBSN arg_0x4099e708, TMacPanDescriptor arg_0x4099e8b0, TMacPendAddrSpec arg_0x4099ea58, uint8_t *arg_0x4099ec10, uint8_t arg_0x4099eda8, uint8_t *arg_0x4099d010){
#line 6
  NwkBeaconChildM$IMacBEACON_NOTIFY$Indication(arg_0x4099e708, arg_0x4099e8b0, arg_0x4099ea58, arg_0x4099ec10, arg_0x4099eda8, arg_0x4099d010);
#line 6
}
#line 6
# 89 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconAttrCoordM.nc"
static inline  bool MacBeaconAttrCoordM$IMacBeaconAttr$GetmacAutoRequest(TMacStatus *const pMacStatus)
{
  if (pMacStatus != NULL) {
#line 91
    *pMacStatus = MAC_SUCCESS;
    }
#line 92
  return MacBeaconAttrCoordM$macAutoRequest;
}

# 7 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBeaconAttr.nc"
inline static  bool MacBeaconNotifierM$IMacBeaconAttr$GetmacAutoRequest(TMacStatus *const arg_0x40ed34a8){
#line 7
  unsigned char result;
#line 7

#line 7
  result = MacBeaconAttrCoordM$IMacBeaconAttr$GetmacAutoRequest(arg_0x40ed34a8);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 157 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanM.nc"
static inline  void MacScanM$AllBeacons$Indication(TMacBSN bsn, 
TMacPanDescriptor panDescriptor, 
TMacPendAddrSpec pendAddrSpec, 
uint8_t *addrList, 
uint8_t sduLength, 
uint8_t *sdu)



{
}

# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBEACON_NOTIFY.nc"
inline static  void MacBeaconNotifierM$AllBeacons$Indication(TMacBSN arg_0x4099e708, TMacPanDescriptor arg_0x4099e8b0, TMacPendAddrSpec arg_0x4099ea58, uint8_t *arg_0x4099ec10, uint8_t arg_0x4099eda8, uint8_t *arg_0x4099d010){
#line 6
  MacScanM$AllBeacons$Indication(arg_0x4099e708, arg_0x4099e8b0, arg_0x4099ea58, arg_0x4099ec10, arg_0x4099eda8, arg_0x4099d010);
#line 6
}
#line 6
# 81 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  result_t MacSuperframesM$syncParentSF(TSysTime beacon_timestamp)
{

  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {

      if (MacSuperframesM$own_active) 
        {
          unsigned char __nesc_temp = 
#line 89
          FAIL;

          {
#line 89
            __nesc_atomic_end(__nesc_atomic); 
#line 89
            return __nesc_temp;
          }
        }
      MacSuperframesM$Reset$reset();
    }
#line 93
    __nesc_atomic_end(__nesc_atomic); }


  MacSuperframesM$parent_active = TRUE;
  MacSuperframesM$timestamp = beacon_timestamp;
  MacSuperframesM$current_sf = MacSuperframesM$SF_PARENT;
  MacSuperframesM$bad_beacons = 0;
  MacSuperframesM$calcIntervals();
  MacSuperframesM$newCycle();
  return SUCCESS;
}

# 22 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
inline static  result_t MacSyncM$syncParentSF(TSysTime arg_0x411fd868){
#line 22
  unsigned char result;
#line 22

#line 22
  result = MacSuperframesM$syncParentSF(arg_0x411fd868);
#line 22

#line 22
  return result;
#line 22
}
#line 22
# 7 "../../../zigzag/IEEE802_15_4/MacSuperframe/public/IMacSuperframeAttr.nc"
inline static  TMacStatus MacSyncM$IMacSuperframeAttr$SetmacBeaconOrder(TMacBeaconOrder arg_0x40e55010){
#line 7
  enum __nesc_unnamed4286 result;
#line 7

#line 7
  result = MacSuperframeAttrC$IMacSuperframeAttr$SetmacBeaconOrder(arg_0x40e55010);
#line 7

#line 7
  return result;
#line 7
}
#line 7
# 96 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSyncM.nc"
static inline  void MacSyncM$CoordBeacons$Indication(
TMacBSN bsn, 
TMacPanDescriptor panDescriptor, 
TMacPendAddrSpec pendAddrSpec, 
TMacAddrList addrList, 
uint8_t sduLength, 
uint8_t *sdu)

{

  if (!MacSyncM$beacon_expected) {
#line 106
    return;
    }
  MacSyncM$InitialTimer$Stop();
  MacSyncM$beacon_expected = FALSE;

  MacSyncM$IMacSuperframeAttr$SetmacBeaconOrder(panDescriptor.superframeSpec.beaconOrder);
  MacSyncM$IMacSuperframeAttr$SetmacSuperframeOrder(panDescriptor.superframeSpec.superframeOrder);

  MacSyncM$IPhySET_TRX_STATE$Request(PHY_TRX_OFF, 0);

  MacSyncM$syncParentSF(panDescriptor.timeStamp);
}

# 331 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static inline  void MacSuperframesM$CoordBeacons$Indication(
TMacBSN bsn, 
TMacPanDescriptor panDescriptor, 
TMacPendAddrSpec pendAddrSpec, 
TMacAddrList addrList, 
uint8_t sduLength, 
uint8_t *sdu)

{


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (MacSuperframesM$parent_active && MacSuperframesM$accept_beacon) 
        {
          MacSuperframesM$timestamp = panDescriptor.timeStamp;
          MacSuperframesM$beacon_received = TRUE;
          MacSuperframesM$bad_beacons = 0;
        }
    }
#line 350
    __nesc_atomic_end(__nesc_atomic); }
}

# 6 "../../../zigzag/IEEE802_15_4/MacBeacon/public/IMacBEACON_NOTIFY.nc"
inline static  void MacBeaconNotifierM$CoordBeacons$Indication(TMacBSN arg_0x4099e708, TMacPanDescriptor arg_0x4099e8b0, TMacPendAddrSpec arg_0x4099ea58, uint8_t *arg_0x4099ec10, uint8_t arg_0x4099eda8, uint8_t *arg_0x4099d010){
#line 6
  MacSuperframesM$CoordBeacons$Indication(arg_0x4099e708, arg_0x4099e8b0, arg_0x4099ea58, arg_0x4099ec10, arg_0x4099eda8, arg_0x4099d010);
#line 6
  MacSyncM$CoordBeacons$Indication(arg_0x4099e708, arg_0x4099e8b0, arg_0x4099ea58, arg_0x4099ec10, arg_0x4099eda8, arg_0x4099d010);
#line 6
}
#line 6
# 10 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconCommonM.nc"
static inline  void MacBeaconCommonM$setLastBSN(uint8_t bsn)
{
  MacBeaconCommonM$lastParentBSN = bsn;
}

# 27 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconNotifierM.nc"
inline static  void MacBeaconNotifierM$setLastBSN(uint8_t arg_0x4113d068){
#line 27
  MacBeaconCommonM$setLastBSN(arg_0x4113d068);
#line 27
}
#line 27
# 6 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
inline static  void MacBeaconNotifierM$ILocalTime64$setLocalTimeAt(TSysTime arg_0x40a41330, uint64_t arg_0x40a414b8){
#line 6
  LocalTime64M$ILocalTime64$setLocalTimeAt(arg_0x40a41330, arg_0x40a414b8);
#line 6
}
#line 6
# 71 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static inline  TMacExtendedAddress MacAssocDeviceM$IMacAssocAttr$GetmacCoordExtendedAddress(
TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 76
  return MacAssocDeviceM$assoc.macCoordExtendedAddress;
}

# 6 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  TMacExtendedAddress MacBeaconNotifierM$IMacAssocAttr$GetmacCoordExtendedAddress(TMacStatus *const arg_0x40ec4208){
#line 6
  unsigned long long result;
#line 6

#line 6
  result = MacAssocDeviceM$IMacAssocAttr$GetmacCoordExtendedAddress(arg_0x40ec4208);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 10 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacBeaconNotifierM$IMacAddress$GetExtendedAddress(TMacExtendedAddress *const arg_0x409003c0, const TMacAddress arg_0x40900570){
#line 10
  unsigned char result;
#line 10

#line 10
  result = MacAddressM$IMacAddress$GetExtendedAddress(arg_0x409003c0, arg_0x40900570);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 12 "../../../zigzag/IEEE802_15_4/MacAssoc/public/IMacAssocAttr.nc"
inline static  TMacShortAddress MacBeaconNotifierM$IMacAssocAttr$GetmacCoordShortAddress(TMacStatus *const arg_0x40ec4f00){
#line 12
  unsigned int result;
#line 12

#line 12
  result = MacAssocDeviceM$IMacAssocAttr$GetmacCoordShortAddress(arg_0x40ec4f00);
#line 12

#line 12
  return result;
#line 12
}
#line 12
# 8 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacBeaconNotifierM$IMacAddress$GetShortAddress(TMacShortAddress *const arg_0x40901ce0, const TMacAddress arg_0x40901e90){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacAddressM$IMacAddress$GetShortAddress(arg_0x40901ce0, arg_0x40901e90);
#line 8

#line 8
  return result;
#line 8
}
#line 8
#line 6
inline static  TMacAddressMode MacBeaconNotifierM$IMacAddress$GetAddressMode(const TMacAddress arg_0x409017d8){
#line 6
  enum __nesc_unnamed4299 result;
#line 6

#line 6
  result = MacAddressM$IMacAddress$GetAddressMode(arg_0x409017d8);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacPANId MacBeaconNotifierM$IMacCommonAttr$GetmacPANId(TMacStatus *const arg_0x40dd3cf0){
#line 6
  unsigned int result;
#line 6

#line 6
  result = MacCommonAttrM$IMacCommonAttr$GetmacPANId(arg_0x40dd3cf0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 51 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconNotifierM.nc"
static inline bool MacBeaconNotifierM$isFromCoord(const TMacPanDescriptor *const pan_d)
{


  if (pan_d->coordPANId != MacBeaconNotifierM$IMacCommonAttr$GetmacPANId(NULL)) {
    return FALSE;
    }
  if (MacBeaconNotifierM$IMacAddress$GetAddressMode(pan_d->coordAddr) == MAC_ADDRESS_SHORT) 
    {
      TMacShortAddress sa;

#line 61
      MacBeaconNotifierM$IMacAddress$GetShortAddress(&sa, pan_d->coordAddr);
      if (MacBeaconNotifierM$IMacAssocAttr$GetmacCoordShortAddress(NULL) >= 0xfffe || sa != MacBeaconNotifierM$IMacAssocAttr$GetmacCoordShortAddress(NULL)) {
        return FALSE;
        }
    }
  else {
      TMacExtendedAddress ea;

#line 68
      MacBeaconNotifierM$IMacAddress$GetExtendedAddress(&ea, pan_d->coordAddr);
      if (ea != MacBeaconNotifierM$IMacAssocAttr$GetmacCoordExtendedAddress(NULL)) 
        {
          return FALSE;
        }
    }
  return TRUE;
}

# 50 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconNotifierM$IMacFrame$GetLinkQuality(const TMacFrame *const arg_0x40da5970, TMacLinkQuality *const arg_0x40da5b60){
#line 50
  unsigned char result;
#line 50

#line 50
  result = MacFrameFormatM$IMacFrame$GetLinkQuality(arg_0x40da5970, arg_0x40da5b60);
#line 50

#line 50
  return result;
#line 50
}
#line 50
#line 21
inline static  result_t MacBeaconNotifierM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40daf558, TMacSequenceNumber *const arg_0x40daf760){
#line 21
  unsigned char result;
#line 21

#line 21
  result = MacFrameFormatM$IMacFrame$GetSequenceNumber(arg_0x40daf558, arg_0x40daf760);
#line 21

#line 21
  return result;
#line 21
}
#line 21
#line 42
inline static  result_t MacBeaconNotifierM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40da8530, arg_0x40da8718);
#line 42

#line 42
  return result;
#line 42
}
#line 42
#line 39
inline static  result_t MacBeaconNotifierM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, TMacPayloadLength *const arg_0x40da8010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40da9df0, arg_0x40da8010);
#line 39

#line 39
  return result;
#line 39
}
#line 39









inline static  result_t MacBeaconNotifierM$IMacFrame$GetTimeStamp(const TMacFrame *const arg_0x40da5260, TMacTimeStamp *const arg_0x40da5450){
#line 48
  unsigned char result;
#line 48

#line 48
  result = MacFrameFormatM$IMacFrame$GetTimeStamp(arg_0x40da5260, arg_0x40da5450);
#line 48

#line 48
  return result;
#line 48
}
#line 48
# 776 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static inline  TPhyChannel PhyCC2420M$IPhyGET$GetphyCurrentChannel(TPhyStatus *const pPhyStatus)
{









  if (pPhyStatus != NULL) {
    *pPhyStatus = PHY_SUCCESS;
    }
#line 789
  return PhyCC2420M$g_channel;
}

# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyGET.nc"
inline static  TPhyChannel MacBeaconNotifierM$IPhyGET$GetphyCurrentChannel(TPhyStatus *const arg_0x40b33d28){
#line 5
  unsigned char result;
#line 5

#line 5
  result = PhyCC2420M$IPhyGET$GetphyCurrentChannel(arg_0x40b33d28);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 34 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacBeaconNotifierM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010){
#line 34
  unsigned char result;
#line 34

#line 34
  result = MacFrameFormatM$IMacFrame$GetSrcAddress(arg_0x40dabdd0, arg_0x40da9010);
#line 34

#line 34
  return result;
#line 34
}
#line 34
#line 31
inline static  result_t MacBeaconNotifierM$IMacFrame$GetSrcPANId(const TMacFrame *const arg_0x40dab010, TMacPANId *const arg_0x40dab200){
#line 31
  unsigned char result;
#line 31

#line 31
  result = MacFrameFormatM$IMacFrame$GetSrcPANId(arg_0x40dab010, arg_0x40dab200);
#line 31

#line 31
  return result;
#line 31
}
#line 31
# 112 "../../../zigzag/IEEE802_15_4/MacBeacon/private/MacBeaconNotifierM.nc"
static inline  result_t MacBeaconNotifierM$BeaconReceive$Receive(TMacFrame *const pMacFrame)
{
  uint8_t sdu[MAC_AMAX_MAC_FRAME_SIZE];
  TMacBSN bsn;
  TMacPendAddrSpec pendAddrSpec;
  uint8_t *addrList;
  uint8_t sduLength;
  uint8_t beaconPayloadLength;
  uint8_t *beaconPayload;
  uint8_t i;
  TMacPanDescriptor panDescriptor;



  MacBeaconNotifierM$IMacFrame$GetSrcPANId(pMacFrame, & panDescriptor.coordPANId);
  MacBeaconNotifierM$IMacFrame$GetSrcAddress(pMacFrame, & panDescriptor.coordAddr);
  panDescriptor.logicalChannel = MacBeaconNotifierM$IPhyGET$GetphyCurrentChannel(NULL);


  MacBeaconNotifierM$IMacFrame$GetTimeStamp(pMacFrame, & panDescriptor.timeStamp);




  MacBeaconNotifierM$IMacFrame$GetPayloadLength(pMacFrame, &sduLength);

  MacBeaconNotifierM$IMacFrame$GetPayload(pMacFrame, sdu);
  MacBeaconNotifierM$IMacFrame$GetSequenceNumber(pMacFrame, &bsn);
  panDescriptor.superframeSpec = ((TMacSuperframeSpec *)sdu)[0];

  panDescriptor.gtsPermit = sdu[2] & 0x80;
  i = (sdu[2] & 0x7) * 3 + 3;
  pendAddrSpec = * (TMacPendAddrSpec *)(sdu + i);
  i++;
  addrList = sdu + i;
  i += pendAddrSpec.shortAddrNum * sizeof(TMacShortAddress ) + pendAddrSpec.extAddrNum * sizeof(TMacExtendedAddress );
  beaconPayload = sdu + i;
  beaconPayloadLength = sduLength - i;
  beaconPayloadLength = beaconPayloadLength == 0 ? 0 : beaconPayloadLength - 9;

  MacBeaconNotifierM$IMacFrame$GetLinkQuality(pMacFrame, & panDescriptor.linkQuality);





  if (MacBeaconNotifierM$isFromCoord(&panDescriptor)) 
    {


      if (beaconPayloadLength != 0) 
        {
          uint64_t time;

#line 165
          for (i = 0; i < 8; i++) (
            (uint8_t *)&time)[i] = beaconPayload[beaconPayloadLength + i];

          {



            MacBeaconNotifierM$ILocalTime64$setLocalTimeAt(panDescriptor.timeStamp, time);
          }
        }

      MacBeaconNotifierM$setLastBSN(bsn);
      MacBeaconNotifierM$CoordBeacons$Indication(bsn, panDescriptor, pendAddrSpec, 
      addrList, beaconPayloadLength, beaconPayload);
    }


  MacBeaconNotifierM$AllBeacons$Indication(bsn, panDescriptor, pendAddrSpec, 
  addrList, beaconPayloadLength, beaconPayload);



  if (beaconPayloadLength != 0 || MacBeaconNotifierM$IMacBeaconAttr$GetmacAutoRequest(NULL) == FALSE) {
    MacBeaconNotifierM$ToHigher$Indication(bsn, panDescriptor, pendAddrSpec, 
    addrList, beaconPayloadLength, beaconPayload);
    }
#line 190
  return SUCCESS;
}

# 5 "../../../zigzag/IEEE802_15_4/MacCAP/public/IMacReceive.nc"
inline static  result_t MacRxM$BeaconReceive$Receive(TMacFrame *const arg_0x40f024f8){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacBeaconNotifierM$BeaconReceive$Receive(arg_0x40f024f8);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacRxM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40db5010, arg_0x40db5200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 15 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   result_t MacFrameFormatM$IPhyFrame$ReadOctet(TPhyFrame *const arg_0x40b47930, uint8_t *const arg_0x40b47b10){
#line 15
  unsigned char result;
#line 15

#line 15
  result = PhyFrameM$IPhyFrame$ReadOctet(arg_0x40b47930, arg_0x40b47b10);
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
inline static   result_t MacFrameFormatM$IPhyFrame$GetTimeStamp(const TPhyFrame *const arg_0x40b44bf0, TPhyTimeStamp *const arg_0x40b44dd8){
#line 27
  unsigned char result;
#line 27

#line 27
  result = PhyFrameM$IPhyFrame$GetTimeStamp(arg_0x40b44bf0, arg_0x40b44dd8);
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
inline static   result_t MacFrameFormatM$IPhyFrame$CheckCRC(TPhyFrame *const arg_0x40b44010){
#line 23
  unsigned char result;
#line 23

#line 23
  result = PhyFrameM$IPhyFrame$CheckCRC(arg_0x40b44010);
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
inline static   result_t MacFrameFormatM$IPhyFrame$ResetPosition(TPhyFrame *const arg_0x40b45068){
#line 17
  unsigned char result;
#line 17

#line 17
  result = PhyFrameM$IPhyFrame$ResetPosition(arg_0x40b45068);
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
inline static  result_t MacRxM$IMacFrame$UnPack(TMacFrame *const arg_0x40da63a0, TPhyFrame *const arg_0x40da6590){
#line 45
  unsigned char result;
#line 45

#line 45
  result = MacFrameFormatM$IMacFrame$UnPack(arg_0x40da63a0, arg_0x40da6590);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *MacRxM$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010){
#line 8
  struct __nesc_unnamed4281 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b3d010);
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
inline static  result_t PhyCC2420M$IPhyRxDATA$Indication(const TPhyPoolHandle arg_0x40b2e820){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacRxM$IPhyRxDATA$Indication(arg_0x40b2e820);
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
inline static   result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t arg_0x40b789a8, uint8_t *arg_0x40b78b48){
#line 39
  unsigned char result;
#line 39

#line 39
  result = PhyCC2420M$IChipconFIFO$RXFIFODone(arg_0x40b789a8, arg_0x40b78b48);
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
inline static   result_t PhyCC2420M$IChipconFIFO$readRXFIFO(uint8_t arg_0x40b7aa48, uint8_t *arg_0x40b7abe8){
#line 19
  unsigned char result;
#line 19

#line 19
  result = HPLCC2420M$HPLCC2420FIFO$readRXFIFO(arg_0x40b7aa48, arg_0x40b7abe8);
#line 19

#line 19
  return result;
#line 19
}
#line 19
# 5 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyFrame.nc"
inline static   uint8_t *PhyCC2420M$IPhyFrame$GetPPDU(const TPhyFrame *const arg_0x40b2bd60){
#line 5
  unsigned char *result;
#line 5

#line 5
  result = PhyFrameM$IPhyFrame$GetPPDU(arg_0x40b2bd60);
#line 5

#line 5
  return result;
#line 5
}
#line 5
#line 25
inline static   result_t PhyCC2420M$IPhyFrame$SetTimeStamp(TPhyFrame *const arg_0x40b44528, TPhyTimeStamp arg_0x40b446b0){
#line 25
  unsigned char result;
#line 25

#line 25
  result = PhyFrameM$IPhyFrame$SetTimeStamp(arg_0x40b44528, arg_0x40b446b0);
#line 25

#line 25
  return result;
#line 25
}
#line 25
# 8 "../../../zigzag/IEEE802_15_4/Phy/public/IPhyPool.nc"
inline static   TPhyFrame *PhyCC2420M$IPhyPool$GetFrame(const TPhyPoolHandle arg_0x40b3d010){
#line 8
  struct __nesc_unnamed4281 *result;
#line 8

#line 8
  result = PhyPoolM$IPhyPool$GetFrame(arg_0x40b3d010);
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
inline static   void MSP430InterruptM$Port11$fired(void){
#line 59
  HPLCC2420InterruptM$FIFOPInterrupt$fired();
#line 59
}
#line 59
# 16 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacAddress.nc"
inline static  result_t MacFrameFormatM$IMacAddress$SetEmptyAddress(TMacAddress *const arg_0x408ff828){
#line 16
  unsigned char result;
#line 16

#line 16
  result = MacAddressM$IMacAddress$SetEmptyAddress(arg_0x408ff828);
#line 16

#line 16
  return result;
#line 16
}
#line 16
#line 12
inline static  result_t MacFrameFormatM$IMacAddress$SetShortAddress(TMacAddress *const arg_0x40900a70, const TMacShortAddress arg_0x40900c28){
#line 12
  unsigned char result;
#line 12

#line 12
  result = MacAddressM$IMacAddress$SetShortAddress(arg_0x40900a70, arg_0x40900c28);
#line 12

#line 12
  return result;
#line 12
}
#line 12


inline static  result_t MacFrameFormatM$IMacAddress$SetExtendedAddress(TMacAddress *const arg_0x408ff168, const TMacExtendedAddress arg_0x408ff328){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacAddressM$IMacAddress$SetExtendedAddress(arg_0x408ff168, arg_0x408ff328);
#line 14

#line 14
  return result;
#line 14
}
#line 14
# 11 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
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
#line 9
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
#line 6
inline static  result_t LocalTime64M$ITimerSymbol$SetPeriodic(TSysTime arg_0x408cdca8, TUniData arg_0x408cde30){
#line 6
  unsigned char result;
#line 6

#line 6
  result = TimerSymbol2M$ITimerSymbol$SetPeriodic(2U, arg_0x408cdca8, arg_0x408cde30);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 36 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocCoordM$IMacFrame$SetPayload(TMacFrame *const arg_0x40da9510, const uint8_t *const arg_0x40da9728, TMacPayloadLength arg_0x40da98c0){
#line 36
  unsigned char result;
#line 36

#line 36
  result = MacFrameFormatM$IMacFrame$SetPayload(arg_0x40da9510, arg_0x40da9728, arg_0x40da98c0);
#line 36

#line 36
  return result;
#line 36
}
#line 36
#line 30
inline static  result_t MacAssocCoordM$IMacFrame$SetSrcPANId(TMacFrame *const arg_0x40dac908, const TMacPANId arg_0x40dacab8){
#line 30
  unsigned char result;
#line 30

#line 30
  result = MacFrameFormatM$IMacFrame$SetSrcPANId(arg_0x40dac908, arg_0x40dacab8);
#line 30

#line 30
  return result;
#line 30
}
#line 30
#line 24
inline static  result_t MacAssocCoordM$IMacFrame$SetDstPANId(TMacFrame *const arg_0x40dafc60, const TMacPANId arg_0x40dafe10){
#line 24
  unsigned char result;
#line 24

#line 24
  result = MacFrameFormatM$IMacFrame$SetDstPANId(arg_0x40dafc60, arg_0x40dafe10);
#line 24

#line 24
  return result;
#line 24
}
#line 24



inline static  result_t MacAssocCoordM$IMacFrame$SetDstAddress(TMacFrame *const arg_0x40daeb38, const TMacAddress arg_0x40daece8){
#line 27
  unsigned char result;
#line 27

#line 27
  result = MacFrameFormatM$IMacFrame$SetDstAddress(arg_0x40daeb38, arg_0x40daece8);
#line 27

#line 27
  return result;
#line 27
}
#line 27






inline static  result_t MacAssocCoordM$IMacFrame$SetSrcAddress(TMacFrame *const arg_0x40dab700, const TMacAddress arg_0x40dab8b0){
#line 33
  unsigned char result;
#line 33

#line 33
  result = MacFrameFormatM$IMacFrame$SetSrcAddress(arg_0x40dab700, arg_0x40dab8b0);
#line 33

#line 33
  return result;
#line 33
}
#line 33
#line 20
inline static  result_t MacAssocCoordM$IMacFrame$SetSequenceNumber(TMacFrame *const arg_0x40db1e60, const TMacSequenceNumber arg_0x40daf030){
#line 20
  unsigned char result;
#line 20

#line 20
  result = MacFrameFormatM$IMacFrame$SetSequenceNumber(arg_0x40db1e60, arg_0x40daf030);
#line 20

#line 20
  return result;
#line 20
}
#line 20
# 19 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacCommonAttr.nc"
inline static  TMacStatus MacAssocCoordM$IMacCommonAttr$SetmacDSN(TMacDSN arg_0x40dcf828){
#line 19
  enum __nesc_unnamed4286 result;
#line 19

#line 19
  result = MacCommonAttrM$IMacCommonAttr$SetmacDSN(arg_0x40dcf828);
#line 19

#line 19
  return result;
#line 19
}
#line 19
#line 18
inline static  TMacDSN MacAssocCoordM$IMacCommonAttr$GetmacDSN(TMacStatus *const arg_0x40dcf3a0){
#line 18
  unsigned char result;
#line 18

#line 18
  result = MacCommonAttrM$IMacCommonAttr$GetmacDSN(arg_0x40dcf3a0);
#line 18

#line 18
  return result;
#line 18
}
#line 18
# 17 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacAssocCoordM$IMacFrame$SetIntraPAN(TMacFrame *const arg_0x40db10a8, const bool arg_0x40db1250){
#line 17
  unsigned char result;
#line 17

#line 17
  result = MacFrameFormatM$IMacFrame$SetIntraPAN(arg_0x40db10a8, arg_0x40db1250);
#line 17

#line 17
  return result;
#line 17
}
#line 17
#line 14
inline static  result_t MacAssocCoordM$IMacFrame$SetAckRequest(TMacFrame *const arg_0x40db22d8, const bool arg_0x40db2480){
#line 14
  unsigned char result;
#line 14

#line 14
  result = MacFrameFormatM$IMacFrame$SetAckRequest(arg_0x40db22d8, arg_0x40db2480);
#line 14

#line 14
  return result;
#line 14
}
#line 14
#line 11
inline static  result_t MacAssocCoordM$IMacFrame$SetFramePending(TMacFrame *const arg_0x40db4510, const bool arg_0x40db46b8){
#line 11
  unsigned char result;
#line 11

#line 11
  result = MacFrameFormatM$IMacFrame$SetFramePending(arg_0x40db4510, arg_0x40db46b8);
#line 11

#line 11
  return result;
#line 11
}
#line 11
#line 8
inline static  result_t MacAssocCoordM$IMacFrame$SetSecurityEnabled(TMacFrame *const arg_0x40db5708, const bool arg_0x40db58b0){
#line 8
  unsigned char result;
#line 8

#line 8
  result = MacFrameFormatM$IMacFrame$SetSecurityEnabled(arg_0x40db5708, arg_0x40db58b0);
#line 8

#line 8
  return result;
#line 8
}
#line 8
#line 5
inline static  result_t MacAssocCoordM$IMacFrame$SetFrameType(TMacFrame *const arg_0x40db68e8, const TMacFrameType arg_0x40db6a98){
#line 5
  unsigned char result;
#line 5

#line 5
  result = MacFrameFormatM$IMacFrame$SetFrameType(arg_0x40db68e8, arg_0x40db6a98);
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
inline static  result_t MacAssocCoordM$IMacSendOwn$Send(const TMacFrame *const arg_0x40f051f8, const TUniData arg_0x40f053b0){
#line 7
  unsigned char result;
#line 7

#line 7
  result = /*MacOwnCAPC.MacCAPOwnSwitch*/MacCAPInterfaceSwitchM$0$IMacSend$Send(1U, arg_0x40f051f8, arg_0x40f053b0);
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

# 6 "../../../zigzag/IEEE802_15_4/MacCommon/public/IMacFrame.nc"
inline static  result_t MacDataM$IMacFrame$GetFrameType(const TMacFrame *const arg_0x40db5010, TMacFrameType *const arg_0x40db5200){
#line 6
  unsigned char result;
#line 6

#line 6
  result = MacFrameFormatM$IMacFrame$GetFrameType(arg_0x40db5010, arg_0x40db5200);
#line 6

#line 6
  return result;
#line 6
}
#line 6
#line 21
inline static  result_t MacDataM$IMacFrame$GetSequenceNumber(const TMacFrame *const arg_0x40daf558, TMacSequenceNumber *const arg_0x40daf760){
#line 21
  unsigned char result;
#line 21

#line 21
  result = MacFrameFormatM$IMacFrame$GetSequenceNumber(arg_0x40daf558, arg_0x40daf760);
#line 21

#line 21
  return result;
#line 21
}
#line 21
#line 15
inline static  result_t MacDataM$IMacFrame$GetAckRequest(const TMacFrame *const arg_0x40db29a0, bool *const arg_0x40db2b88){
#line 15
  unsigned char result;
#line 15

#line 15
  result = MacFrameFormatM$IMacFrame$GetAckRequest(arg_0x40db29a0, arg_0x40db2b88);
#line 15

#line 15
  return result;
#line 15
}
#line 15
#line 39
inline static  result_t MacDataM$IMacFrame$GetPayloadLength(const TMacFrame *const arg_0x40da9df0, TMacPayloadLength *const arg_0x40da8010){
#line 39
  unsigned char result;
#line 39

#line 39
  result = MacFrameFormatM$IMacFrame$GetPayloadLength(arg_0x40da9df0, arg_0x40da8010);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 31
inline static  result_t MacDataM$IMacFrame$GetSrcPANId(const TMacFrame *const arg_0x40dab010, TMacPANId *const arg_0x40dab200){
#line 31
  unsigned char result;
#line 31

#line 31
  result = MacFrameFormatM$IMacFrame$GetSrcPANId(arg_0x40dab010, arg_0x40dab200);
#line 31

#line 31
  return result;
#line 31
}
#line 31



inline static  result_t MacDataM$IMacFrame$GetSrcAddress(const TMacFrame *const arg_0x40dabdd0, TMacAddress *const arg_0x40da9010){
#line 34
  unsigned char result;
#line 34

#line 34
  result = MacFrameFormatM$IMacFrame$GetSrcAddress(arg_0x40dabdd0, arg_0x40da9010);
#line 34

#line 34
  return result;
#line 34
}
#line 34
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
inline static  result_t MacDataM$IMacFrame$GetDstPANId(const TMacFrame *const arg_0x40dae448, TMacPANId *const arg_0x40dae638){
#line 25
  unsigned char result;
#line 25

#line 25
  result = MacFrameFormatM$IMacFrame$GetDstPANId(arg_0x40dae448, arg_0x40dae638);
#line 25

#line 25
  return result;
#line 25
}
#line 25



inline static  result_t MacDataM$IMacFrame$GetDstAddress(const TMacFrame *const arg_0x40dac218, TMacAddress *const arg_0x40dac408){
#line 28
  unsigned char result;
#line 28

#line 28
  result = MacFrameFormatM$IMacFrame$GetDstAddress(arg_0x40dac218, arg_0x40dac408);
#line 28

#line 28
  return result;
#line 28
}
#line 28
#line 42
inline static  result_t MacDataM$IMacFrame$GetPayload(const TMacFrame *const arg_0x40da8530, uint8_t *const arg_0x40da8718){
#line 42
  unsigned char result;
#line 42

#line 42
  result = MacFrameFormatM$IMacFrame$GetPayload(arg_0x40da8530, arg_0x40da8718);
#line 42

#line 42
  return result;
#line 42
}
#line 42








inline static  result_t MacDataM$IMacFrame$GetLinkQuality(const TMacFrame *const arg_0x40da5970, TMacLinkQuality *const arg_0x40da5b60){
#line 50
  unsigned char result;
#line 50

#line 50
  result = MacFrameFormatM$IMacFrame$GetLinkQuality(arg_0x40da5970, arg_0x40da5b60);
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 515 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static inline  result_t RouterM$UpperIf$Indication(
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

# 85 "../../../zigzag/ZigBee/extra/AliveSender2M.nc"
static inline  result_t AliveSender2M$IMacDATA$Indication(TMacPANId SrcPANid, 
TMacAddress SrcAddr, 
TMacPANId DstPANid, 
TMacAddress DstAddr, 
uint8_t msduLength, 
uint8_t *msduFrame, 
TMacLinkQuality linkQuality, 
bool bSecurityUse, 
uint8_t ACLEntry)
{

  return SUCCESS;
}

# 16 "../../../zigzag/IEEE802_15_4/MacData/public/IMacDATA.nc"
inline static  result_t MacDataM$IMacDataParent$Indication(TMacPANId arg_0x409e9178, TMacAddress arg_0x409e9310, TMacPANId arg_0x409e94b0, TMacAddress arg_0x409e9648, uint8_t arg_0x409e97e0, uint8_t *arg_0x409e9998, TMacLinkQuality arg_0x409e9b38, bool arg_0x409e9cd0, uint8_t arg_0x409e9e68){
#line 16
  unsigned char result;
#line 16

#line 16
  result = AliveSender2M$IMacDATA$Indication(arg_0x409e9178, arg_0x409e9310, arg_0x409e94b0, arg_0x409e9648, arg_0x409e97e0, arg_0x409e9998, arg_0x409e9b38, arg_0x409e9cd0, arg_0x409e9e68);
#line 16
  result = rcombine(result, RouterM$UpperIf$Indication(arg_0x409e9178, arg_0x409e9310, arg_0x409e94b0, arg_0x409e9648, arg_0x409e97e0, arg_0x409e9998, arg_0x409e9b38, arg_0x409e9cd0, arg_0x409e9e68));
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 192 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  NwkOnlineStatus RouterM$NIB$getOnlineStatus(void){
#line 192
  enum __nesc_unnamed4340 result;
#line 192

#line 192
  result = NIBM$NIB$getOnlineStatus();
#line 192

#line 192
  return result;
#line 192
}
#line 192
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
inline static  result_t MacDataM$IMacDataOwn$Indication(TMacPANId arg_0x409e9178, TMacAddress arg_0x409e9310, TMacPANId arg_0x409e94b0, TMacAddress arg_0x409e9648, uint8_t arg_0x409e97e0, uint8_t *arg_0x409e9998, TMacLinkQuality arg_0x409e9b38, bool arg_0x409e9cd0, uint8_t arg_0x409e9e68){
#line 16
  unsigned char result;
#line 16

#line 16
  result = RouterM$LowerIf$Indication(arg_0x409e9178, arg_0x409e9310, arg_0x409e94b0, arg_0x409e9648, arg_0x409e97e0, arg_0x409e9998, arg_0x409e9b38, arg_0x409e9cd0, arg_0x409e9e68);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 179 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port12$clear(void)
#line 179
{
#line 179
  MSP430InterruptM$P1IFG &= ~(1 << 2);
}

# 40 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$CCAInterrupt$clear(void){
#line 40
  MSP430InterruptM$Port12$clear();
#line 40
}
#line 40
# 148 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port12$disable(void)
#line 148
{
#line 148
  MSP430InterruptM$P1IE &= ~(1 << 2);
}

# 35 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$CCAInterrupt$disable(void){
#line 35
  MSP430InterruptM$Port12$disable();
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
inline static   void MSP430InterruptM$Port12$fired(void){
#line 59
  HPLCC2420InterruptM$CCAInterrupt$fired();
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

#line 96
static inline    void MSP430InterruptM$Port13$default$fired(void)
#line 96
{
#line 96
  MSP430InterruptM$Port13$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port13$fired(void){
#line 59
  MSP430InterruptM$Port13$default$fired();
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

#line 97
static inline    void MSP430InterruptM$Port14$default$fired(void)
#line 97
{
#line 97
  MSP430InterruptM$Port14$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port14$fired(void){
#line 59
  MSP430InterruptM$Port14$default$fired();
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
# 187 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static inline   void MSP430InterruptM$Port21$clear(void)
#line 187
{
#line 187
  MSP430InterruptM$P2IFG &= ~(1 << 1);
}

#line 103
static inline    void MSP430InterruptM$Port21$default$fired(void)
#line 103
{
#line 103
  MSP430InterruptM$Port21$clear();
}

# 59 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port21$fired(void){
#line 59
  MSP430InterruptM$Port21$default$fired();
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
# 47 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static inline  void LocalTime64M$ILocalTime64$setLocalTime(uint64_t time)
{
  LocalTime64M$ILocalTime64$setLocalTimeAt(LocalTime64M$ILocalTime$Read(), time);
}

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ILocalTime64.nc"
inline static  void ZigSysM$ILocalTime64$setLocalTime(uint64_t arg_0x40a41c40){
#line 9
  LocalTime64M$ILocalTime64$setLocalTime(arg_0x40a41c40);
#line 9
}
#line 9
# 28 "/home/max/tinyos/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
inline static  result_t ZigSTimerM$Timer0$setOneShot(int32_t arg_0x40887668){
#line 28
  unsigned char result;
#line 28

#line 28
  result = TimerSymbol2M$TimerMilli$setOneShot(1U, arg_0x40887668);
#line 28

#line 28
  return result;
#line 28
}
#line 28
inline static  result_t ZigSTimerM$Timer1$setOneShot(int32_t arg_0x40887668){
#line 28
  unsigned char result;
#line 28

#line 28
  result = TimerSymbol2M$TimerMilli$setOneShot(2U, arg_0x40887668);
#line 28

#line 28
  return result;
#line 28
}
#line 28
inline static  result_t ZigSTimerM$Timer2$setOneShot(int32_t arg_0x40887668){
#line 28
  unsigned char result;
#line 28

#line 28
  result = TimerSymbol2M$TimerMilli$setOneShot(3U, arg_0x40887668);
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 33 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
inline static  void RouterM$NLDE_Data$confirm(uint8_t arg_0x408a3338, NwkStatus arg_0x408a34d0){
#line 33
  ZigNetM$NLDE_Data$confirm(arg_0x408a3338, arg_0x408a34d0);
#line 33
}
#line 33
# 69 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static inline  void NIBM$NIB$setNwkSequenceNumber(uint8_t attr)
{
  NIBM$nwkSequenceNumber = attr;
}

# 19 "../../../zigzag/ZigBee/implementation/NIB.nc"
inline static  void NwkFrameM$NIB$setNwkSequenceNumber(uint8_t arg_0x4087f6c8){
#line 19
  NIBM$NIB$setNwkSequenceNumber(arg_0x4087f6c8);
#line 19
}
#line 19
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacShortAddress NwkFrameM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08){
#line 43
  unsigned int result;
#line 43

#line 43
  result = MacCoordAttrM$IMacGET$GetmacShortAddress(arg_0x408bdf08);
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
inline static  uint8_t RouterM$NwkFrame$mkDataFrameHeader(uint8_t *arg_0x409e2938, NwkAddr arg_0x409e2ac8, uint8_t arg_0x409e2c58, uint8_t arg_0x409e2df0, bool arg_0x409e0010){
#line 7
  unsigned char result;
#line 7

#line 7
  result = NwkFrameM$NwkFrame$mkDataFrameHeader(arg_0x409e2938, arg_0x409e2ac8, arg_0x409e2c58, arg_0x409e2df0, arg_0x409e0010);
#line 7

#line 7
  return result;
#line 7
}
#line 7
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
# 45 "../../../zigzag/ZigNetM.nc"
static inline  void ZigNetM$NLDE_Data$indication(NwkAddr srcAddr, IEEEAddr srcExtAddr, uint8_t nsduLength, 
uint8_t *nsdu, uint8_t linkQuality)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __net_recv((uint16_t )srcAddr, srcExtAddr, (uint16_t )nsduLength, nsdu, linkQuality);
    }
  return;
}

# 39 "../../../zigzag/ZigBee/interface/NLDE_Data.nc"
inline static  void RouterM$NLDE_Data$indication(NwkAddr arg_0x408a3960, IEEEAddr arg_0x408a3b00, uint8_t arg_0x408a3c98, uint8_t *arg_0x408a3e48, uint8_t arg_0x408a2010){
#line 39
  ZigNetM$NLDE_Data$indication(arg_0x408a3960, arg_0x408a3b00, arg_0x408a3c98, arg_0x408a3e48, arg_0x408a2010);
#line 39
}
#line 39
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacShortAddress RouterM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08){
#line 43
  unsigned int result;
#line 43

#line 43
  result = MacCoordAttrM$IMacGET$GetmacShortAddress(arg_0x408bdf08);
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
inline static  result_t ZigNetM$NLDE_Data$request(NwkAddr arg_0x408a4490, uint8_t arg_0x408a4628, uint8_t *arg_0x408a47d8, uint8_t arg_0x408a4970, uint8_t arg_0x408a4b00, uint8_t arg_0x408a4c98, bool arg_0x408a4e30){
#line 23
  unsigned char result;
#line 23

#line 23
  result = RouterM$NLDE_Data$request(arg_0x408a4490, arg_0x408a4628, arg_0x408a47d8, arg_0x408a4970, arg_0x408a4b00, arg_0x408a4c98, arg_0x408a4e30);
#line 23

#line 23
  return result;
#line 23
}
#line 23
# 43 "../../../zigzag/IEEE802_15_4/Mac/public/IMacGET.nc"
inline static  TMacShortAddress ZigNetM$IMacGET$GetmacShortAddress(TMacStatus *const arg_0x408bdf08){
#line 43
  unsigned int result;
#line 43

#line 43
  result = MacDeviceAttrM$IMacGET$GetmacShortAddress(arg_0x408bdf08);
#line 43

#line 43
  return result;
#line 43
}
#line 43
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

# 51 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbol2M.nc"
static  result_t TimerSymbol2M$IStdControl$init(void)
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

# 244 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static  result_t TimerSymbolAsyncM$IStdControl$init(void)
{
  uint8_t i;

#line 247
  for (i = 0; i < TimerSymbolAsyncM$ASYNC_TIMER_NUM; i++) {
      TimerSymbolAsyncM$SetControlAsCompare(i);
      TimerSymbolAsyncM$DisableEvents(i);
    }
  return SUCCESS;
}

#line 285
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

# 94 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyPoolM.nc"
static void PhyPoolM$ResetPool(void)
{
  PhyPoolM$TPoolIndex i;

#line 97
  for (i = 0; i < PhyPoolM$POOL_SIZE; i++) 
    PhyPoolM$IPhyPool$Free(i);
  return;
}

#line 77
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

# 687 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static  result_t PhyCC2420M$IStdControl$init(void)
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

# 13 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacRandomM.nc"
static  result_t MacRandomM$IStdControl$init(void)
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

# 165 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static  result_t MacCommonAttrM$IStdControl$init(void)
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

# 118 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$ICAPControl$init(uint8_t sfIndex)
{
  MacCAPM$currentSf = 0;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 121
    MacCAPM$cap[sfIndex].sendState = MacCAPM$CAP_SEND_OFF;
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
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

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t AliveSender2M$Timer$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(12U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 296 "../../../zigzag/IEEE802_15_4/Mac/private/v2/MacAttr.h"
static  result_t MacCoordAttrM$IMacRESET$Request(bool setDefaultPIB)
{
  MacCoordAttrM$IMacReset$reset();
  if (setDefaultPIB) {
      MacCoordAttrM$SetDefaultAttributes();
    }
  MacCoordAttrM$IPhySET_TRX_STATE$Request(PHY_FORCE_TRX_OFF, 0);
  return TOS_post(MacCoordAttrM$confirmReset);
}

# 410 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static  result_t MacSuperframesM$StdControl$stop(void)
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

# 649 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static  result_t MacCAPM$ICAPControl$stop(uint8_t sfIndex)
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

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacCAPM$IBackoff$Stop(void){
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

# 44 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanM.nc"
static void MacScanM$nextBit(void)
{
  while (!((MacScanM$ScanChannels >> MacScanM$CurBit) & 1) && MacScanM$CurBit < 27) 
    MacScanM$CurBit++;
}

# 81 "../../../zigzag/ZigRouterM.nc"
static  void ZigRouterM$NLME_NetworkDiscovery$confirm(
uint8_t networkCount, 
NwkNetworkDescriptor *networkDescriptors, 
NwkStatus status)

{
  ;
  if (status == NWK_SUCCESS) {
      NwkNeighbor *pnbr;

      while (ZigRouterM$NeighborTable$getNextPtr(&pnbr) == SUCCESS) 
        {
          if (pnbr->panID == info_param.Z_PAN_ID && pnbr->permitJoining) 
            {
              TOS_post(ZigRouterM$join);
              return;
            }
        }
    }
  ZigRouterM$idle_scan = TRUE;
  TOS_post(ZigRouterM$restart);
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

#line 31
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

# 127 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static  TMacShortAddress MacCommonAttrM$IMacCommonAttr$GetmacShortAddress(TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 131
  return MacCommonAttrM$pib.macShortAddress;
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

#line 156
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

# 57 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static result_t MacDataM$DATA_Request(MacDataM$SF sf, 
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

#line 267
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

# 23 "../../../zigzag/ZigBee/extra/AliveSender2M.nc"
static result_t AliveSender2M$restartTimer(void)
{
  TSysTime millis = (TSysTime )info_param.ALIVE_SEND_PERIOD << 10;

#line 26
  AliveSender2M$f.failCount = 0;
  AliveSender2M$f.sending = 0;

  if (AliveSender2M$Timer$IsSet()) {
#line 29
    AliveSender2M$Timer$Stop();
    }
  return AliveSender2M$Timer$SetPeriodic(AliveSender2M$ITimeCast$MillisToSymbols(millis), 0);
}

# 223 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static result_t RouterM$IMacDATA_Confirm(uint8_t msduHandle, TMacStatus status)
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

# 37 "../../../zigzag/ZigNetM.nc"
static  void ZigNetM$NLDE_Data$confirm(uint8_t nsduHandle, NwkStatus status)
{
  if ((faddr_t )__module_load != (faddr_t )-1) {
    __net_send_done(nsduHandle, NWK_SUCCESS == status ? 0 : -1);
    }
  return;
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

#line 289
static  result_t MacFrameFormatM$IMacFrame$GetTimeStamp(const TMacFrame *const pMacFrame, 
TMacTimeStamp *const pTimeStamp)
{
  if (pMacFrame != NULL && pTimeStamp != NULL) {
      *pTimeStamp = pMacFrame->timeStamp;
      return SUCCESS;
    }
  return FAIL;
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

# 122 "../../../zigzag/ZigRouterM.nc"
static  void ZigRouterM$NLME_JoinChild$confirm(
PanID_t panID, 
NwkStatus status)

{
  ;
  if (status != NWK_SUCCESS) {
    TOS_post(ZigRouterM$restart);
    }
  else 
#line 130
    {
      ZigRouterM$AliveSenderControl$start();
      TOS_post(ZigRouterM$start_router);
    }
  return;
}

#line 148
static  void ZigRouterM$NLME_StartRouter$confirm(NwkStatus status)
{
  ;

  if (status == NWK_SUCCESS) {
      ZigRouterM$NLME_PermitJoining$request(0xff);
      ZigRouterM$ChildSupervisorControl$start();
    }
  else {
#line 155
    if (status == NWK_INVALID_REQUEST) {
        ZigRouterM$NLME_PermitJoining$request(0);
      }
    else {
#line 158
      TOS_post(ZigRouterM$restart);
      }
    }
#line 160
  return;
}

# 30 "../../../zigzag/ZigBee/implementation/NLME_PermitJoiningM.nc"
static  NwkStatus NLME_PermitJoiningM$NLME_PermitJoining$request(
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

# 25 "../../../zigzag/ZigBee/implementation/NwkBeaconOffsetRandomM.nc"
static uint16_t NwkBeaconOffsetRandomM$positive_residual(uint32_t dividend, uint32_t divisor)
{
  uint32_t result;




  if (dividend >= 1L << 31) 
    {
      result = divisor - -dividend % divisor;
    }
  else {
    result = dividend % divisor;
    }
  ;
  return result;
}

# 110 "../../../zigzag/ZigBee/implementation/NLME_StartRouterM.nc"
static  result_t NLME_StartRouterM$IMacSTART$Confirm(TMacStatus status)
{
  if (status == MAC_SUCCESS) 
    {
      NLME_StartRouterM$NIB$setOnline(NWK_ROUTING);
    }
  NLME_StartRouterM$NLME_StartRouter$confirm(status);
  return SUCCESS;
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

# 380 "../../../zigzag/IEEE802_15_4/MacSuperframe/private/v4/MacSuperframesM.nc"
static void MacSuperframesM$calcIntervals(void)
{
  MacSuperframesM$beacon_interval = (uint32_t )MAC_ABASE_SUPERFRAME_DURATION << MacSuperframesM$IMacSuperframeAttr$GetmacBeaconOrder(NULL);


  MacSuperframesM$sf_duration = (uint32_t )MAC_ABASE_SUPERFRAME_DURATION << MacSuperframesM$IMacSuperframeAttr$GetmacSuperframeOrder(NULL);
}

#line 147
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

# 157 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static   void MacCAPM$IMacCAP$BeginCAP(uint8_t sfIndex)
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

#line 519
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

# 120 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static void MacDataM$SendDone(TMacStatus status, const TUniData unidata)
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

# 44 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacCommonAttrM.nc"
static  TMacAckWaitDuration MacCommonAttrM$IMacCommonAttr$GetmacAckWaitDuration(TMacStatus *const pMacStatus)
{
  if (pMacStatus != NULL) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 48
  return MacCommonAttrM$pib.macAckWaitDuration;
}

# 114 "../../../zigzag/IEEE802_15_4/MacPool/private/small/MacExtractM.nc"
static void MacExtractM$ExtractDone(TMacStatus macStatus, bool framePending)
{
  MacExtractM$extractState = MacExtractM$MAC_EXTRACT_IDLE;
  MacExtractM$IMacExtract$End(MacExtractM$extract.context, macStatus, framePending, MacExtractM$extract.uniData);
  return;
}

# 183 "../../../zigzag/ZigBee/implementation/NLME_JoinChildM.nc"
static  result_t NLME_JoinChildM$IMacASSOCIATE$Confirm(NwkAddr assocAddr, TMacStatus status)
{
  if (status == MAC_SUCCESS) 
    {
      NwkNeighbor *potential_parent;
      TMacAddress addr;

#line 189
      ;
      NLME_JoinChildM$IMacAddress$SetShortAddress(&addr, NLME_JoinChildM$potential_parent_addr);
      if (NLME_JoinChildM$NeighborTable$getByAddr(addr, &potential_parent)) 
        {
          ;
          potential_parent->relationship = NWK_PARENT;
          NLME_JoinChildM$NIB$setOnline(NLME_JoinChildM$request_joinAsRouter ? NWK_JOINED_AS_ROUTER : NWK_JOINED_AS_ED);
          NLME_JoinChildM$NIB$setDepth(potential_parent->depth + 1);
          NLME_JoinChildM$NIB$setChannel(potential_parent->logicalChannel);
          NLME_JoinChildM$IMacSET$SetmacPANId(potential_parent->panID);
          NLME_JoinChildM$IMacSET$SetmacShortAddress(assocAddr);




          NLME_JoinChildM$NLME_JoinChild$confirm(potential_parent->panID, NWK_SUCCESS);
          return SUCCESS;
        }
      else {
        status = NWK_NOT_PERMITTED;
        }
    }
  {
    ;

    if (status == MAC_PAN_AT_CAPACITY) {
      status = NWK_NOT_PERMITTED;
      }

    NLME_JoinChildM$confirm_status = status;
    if (!TOS_post(NLME_JoinChildM$try_to_assoc)) {
      NLME_JoinChildM$NLME_JoinChild$confirm(NLME_JoinChildM$request_panID, NWK_INVALID_REQUEST);
      }
  }
#line 222
  return SUCCESS;
}

#line 102
static  void NLME_JoinChildM$try_to_assoc(void)
{
  NwkNeighbor *pneighbor;
  NwkNeighbor *potential_parent;
  uint8_t best_depth = 0xff;
  bool found = FALSE;

  ;

  if (!NLME_JoinChildM$joinToSpecified) 
    {

      pneighbor = NULL;
      while (NLME_JoinChildM$NeighborTable$getNextPtr(&pneighbor) == SUCCESS) 
        {

          if (
#line 117
          pneighbor->panID == NLME_JoinChildM$request_panID && 
          pneighbor->potentialParent && pneighbor->permitJoining && pneighbor->lqi >= 0x80) 
            {

              if (pneighbor->depth < best_depth) 

                {
                  potential_parent = pneighbor;

                  best_depth = pneighbor->depth;
                  found = TRUE;
                }
            }
        }
    }
  else 
    {

      TMacAddress coord_addr;

      found = FALSE;
      NLME_JoinChildM$IMacAddress$SetExtendedAddress(&coord_addr, NLME_JoinChildM$joinAddress);
      if (NLME_JoinChildM$NeighborTable$getByAddr(coord_addr, &potential_parent)) 
        {
          found = TRUE;
        }
    }

  if (!found) 
    {

      NLME_JoinChildM$NLME_JoinChild$confirm(NLME_JoinChildM$request_panID, NLME_JoinChildM$confirm_status);
      return;
    }

  NLME_JoinChildM$potential_parent_addr = potential_parent->networkAddr;
  NLME_JoinChildM$IMacSET$SetmacCoordShortAddress(NLME_JoinChildM$potential_parent_addr);
  NLME_JoinChildM$IMacSET$SetmacCoordExtendedAddress(potential_parent->extendedAddr);
  NLME_JoinChildM$IMacSET$SetmacPANId(potential_parent->panID);
  NLME_JoinChildM$IMacSET$SetmacBeaconOrder(potential_parent->beaconOrder);

  NLME_JoinChildM$IMacSYNC$Request(potential_parent->logicalChannel, TRUE);

  potential_parent->potentialParent = FALSE;
  {
    TMacAddress dst;

#line 163
    NLME_JoinChildM$IMacAddress$SetShortAddress(&dst, potential_parent->networkAddr);





    if (!NLME_JoinChildM$IMacASSOCIATE$Request(potential_parent->logicalChannel, 
    potential_parent->panID, 
    dst, 
    NLME_JoinChildM$NIB$getNwkCapabilityInformation(), 
    FALSE)) 
      {
        ;
        NLME_JoinChildM$NLME_JoinChild$confirm(0xffff, MAC_CHANNEL_ACCESS_FAILURE);
      }

    ;
  }
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

# 163 "../../../zigzag/ZigRouterM.nc"
static  void ZigRouterM$NLME_Sync$indication(void)
{
  ;
  TOS_post(ZigRouterM$restart);

  return;
}

# 137 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static result_t MacAssocDeviceM$MakeAssocReqFrame(const TMacPANId coordPANId, 
const TMacAddress coordAddress, 
const TCapabilityInformation capabilityInformation, 
const bool securityEnable, 
TMacFrame *const pMacFrame)

{
  result_t result;

#line 145
  if (pMacFrame == NULL) {
#line 145
    return FAIL;
    }
#line 146
  result = MacAssocDeviceM$IMacFrame$SetFrameType(pMacFrame, MAC_FRAME_TYPE_COMMAND);
  result &= MacAssocDeviceM$IMacFrame$SetSecurityEnabled(pMacFrame, securityEnable);
  result &= MacAssocDeviceM$IMacFrame$SetFramePending(pMacFrame, FALSE);
  result &= MacAssocDeviceM$IMacFrame$SetAckRequest(pMacFrame, TRUE);
  result &= MacAssocDeviceM$IMacFrame$SetIntraPAN(pMacFrame, TRUE);
  if (result != SUCCESS) {
#line 151
    return FAIL;
    }
#line 152
  {
    result = MacAssocDeviceM$IMacFrame$SetSequenceNumber(pMacFrame, MacAssocDeviceM$assoc.seqNum);
    if (result != SUCCESS) {
#line 154
      return FAIL;
      }
  }
#line 156
  {
#line 156
    TMacAddressMode coordAddressMode = MacAssocDeviceM$IMacAddress$GetAddressMode(coordAddress);

#line 157
    if (coordAddressMode != MAC_ADDRESS_SHORT && coordAddressMode != MAC_ADDRESS_EXTENDED) {
      return FAIL;
      }
#line 159
    result = MacAssocDeviceM$IMacFrame$SetDstAddress(pMacFrame, coordAddress);
    result &= MacAssocDeviceM$IMacFrame$SetDstPANId(pMacFrame, coordPANId);
    if (result != SUCCESS) {
#line 161
      return FAIL;
      }
  }
#line 163
  {
#line 163
    TMacAddress srcAddress;

#line 164
    result = MacAssocDeviceM$IMacAddress$SetExtendedAddress(&srcAddress, info_param.MAC_ADDRESS);
    result &= MacAssocDeviceM$IMacFrame$SetSrcAddress(pMacFrame, srcAddress);
    if (result != SUCCESS) {
#line 166
      return FAIL;
      }
  }
#line 168
  {
#line 168
    uint8_t payload[MacAssocDeviceM$ASSOC_REQUEST_PAYLOAD_LEN];

#line 169
    payload[0] = MAC_CMD_ASSOC_REQUEST;
    payload[1] = * (uint8_t *)(void *)&capabilityInformation;
    result = MacAssocDeviceM$IMacFrame$SetPayload(pMacFrame, payload, MacAssocDeviceM$ASSOC_REQUEST_PAYLOAD_LEN);
  }
  return result;
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

# 175 "../../../zigzag/IEEE802_15_4/MacCAP/private/sync/MacCAPM.nc"
static   void MacCAPM$IMacCAP$EndCAP(uint8_t sfIndex)
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

#line 424
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








static   uint8_t PhyFrameM$IPhyFrame$GetPPDULength(const TPhyFrame *const pPhyFrame)
{
  if (pPhyFrame != NULL) {
    return PhyFrameM$IPhyFrame$GetMPDULength(pPhyFrame) + PHY_SIZE_OF_FRAME_LENGTH;
    }
#line 28
  return 0;
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

# 20 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static  uint64_t LocalTime64M$ILocalTime64$getLocalTimeAt(TSysTime at)
{
  TSysTime curCount = at;
  TMilliSec t = LocalTime64M$ITimeCast$SymbolsToMillis(curCount - LocalTime64M$firedCount);
  uint64_t localTime = LocalTime64M$baseTime + t;


  return localTime;
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

# 63 "../../../zigzag/ZigNetM.nc"
static  void ZigNetM$NLME_JoinChild$confirm(PanID_t panID, NwkStatus status)
{
  if (NWK_SUCCESS == status && (faddr_t )__module_load != (faddr_t )-1) {
    __net_enter((uint16_t )panID);
    }
#line 67
  return;
}

# 90 "../../../zigzag/IEEE802_15_4/MacAssoc/private/direct/MacAssocDeviceM.nc"
static  TMacShortAddress MacAssocDeviceM$IMacAssocAttr$GetmacCoordShortAddress(
TMacStatus *const pMacStatus)
{
  if (NULL != pMacStatus) {
    *pMacStatus = MAC_SUCCESS;
    }
#line 95
  return MacAssocDeviceM$assoc.macCoordShortAddress;
}

# 90 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanM.nc"
static void MacScanM$SendFrame(void)
{
  TMacDSN msnNum;
  TMacAddress macAddr;
  TMacFrame msg;

  MacScanM$IMacFrame$SetFrameType(&msg, MAC_FRAME_TYPE_COMMAND);
  MacScanM$IMacFrame$SetFramePending(&msg, FALSE);
  MacScanM$IMacFrame$SetSecurityEnabled(&msg, FALSE);
  MacScanM$IMacFrame$SetAckRequest(&msg, FALSE);
  MacScanM$IMacFrame$SetIntraPAN(&msg, TRUE);
  msnNum = MacScanM$IMacCommonAttr$GetmacDSN(NULL);
  MacScanM$IMacFrame$SetSequenceNumber(&msg, msnNum);
  MacScanM$IMacCommonAttr$SetmacDSN(msnNum + 1);
  MacScanM$IMacFrame$SetDstPANId(&msg, 0xFFFF);
  MacScanM$IMacAddress$SetShortAddress(&macAddr, 0xFFFF);
  MacScanM$IMacFrame$SetDstAddress(&msg, macAddr);

  MacScanM$IMacAddress$SetEmptyAddress(&macAddr);
  MacScanM$IMacFrame$SetSrcAddress(&msg, macAddr);

  {
    uint8_t payload[1] = { MAC_CMD_BEACON_REQUEST };

#line 113
    MacScanM$IMacFrame$SetPayload(&msg, payload, 1);
  }
  {
#line 115
    MacScanM$IMacCSMA$Send(&msg, 0);
  }
#line 115
  ;
}

# 43 "../../../zigzag/ZigBee/implementation/NLME_NetworkDiscoveryM.nc"
static  result_t NLME_NetworkDiscoveryM$IMacSCAN$Confirm(TMacStatus status, TMacScanType scanType, uint32_t unscannedChannels, uint8_t resultListSize, uint8_t *energyDetectList, TMacPanDescriptor *panDescriptorList)
{
  NwkNeighbor *neighbor = NULL;

#line 93
  neighbor = NULL;
  while (NLME_NetworkDiscoveryM$NeighborTable$getNextPtr(&neighbor) == SUCCESS) 
    {
      neighbor->potentialParent = 1;
    }


  NLME_NetworkDiscoveryM$NLME_NetworkDiscovery$confirm(0, NULL, status);

  return SUCCESS;
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

#line 274
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

# 454 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static   result_t PhyCC2420M$IPhyED$Request(uint8_t context, 
const TUniData uniData)
{
  enum PhyCC2420M$TPhyEDState state;

#line 458
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 458
    {
      state = PhyCC2420M$edState;
      if (PhyCC2420M$edState == PhyCC2420M$PHY_ED_FREE) {
        PhyCC2420M$edState = PhyCC2420M$PHY_ED_BUSY;
        }
    }
#line 463
    __nesc_atomic_end(__nesc_atomic); }
#line 463
  if (state == PhyCC2420M$PHY_ED_FREE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 464
        PhyCC2420M$edContext = context;
#line 464
        __nesc_atomic_end(__nesc_atomic); }
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 465
        PhyCC2420M$edUniData = uniData;
#line 465
        __nesc_atomic_end(__nesc_atomic); }
      if (TOS_post(PhyCC2420M$MeasureED)) {
        return SUCCESS;
        }
#line 468
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 468
        PhyCC2420M$edState = PhyCC2420M$PHY_ED_FREE;
#line 468
        __nesc_atomic_end(__nesc_atomic); }
    }
  return FAIL;
}

# 215 "../../../zigzag/IEEE802_15_4/MacScan/private/MacScanM.nc"
static  result_t MacScanM$IPhyED$Confirm(TPhyStatus phyStatus, TPhyEnergyLevel phyEnergyLevel, TUniData uniData)
{


  if (phyStatus == SUCCESS) 
    {
      if (MacScanM$energyDetectList[MacScanM$CurBit] < phyEnergyLevel) {
        MacScanM$energyDetectList[MacScanM$CurBit] = phyEnergyLevel;
        }
    }
  if (MacScanM$TimIsSet) 
    {
      MacScanM$IPhyED$Request(NULL);
    }
  else 
    {
      MacScanM$nextBit();



      if (MacScanM$CurBit < 27) 
        {

          MacScanM$IPhySET$SetphyCurrentChannel(MacScanM$CurBit);
          MacScanM$TimIsSet = TRUE;
          MacScanM$TimerED$SetOneShot(MacScanM$ScanTime, NULL);
          MacScanM$IPhyED$Request(NULL);
        }
      else {
        MacScanM$IMacSCAN$Confirm(MAC_SUCCESS, MAC_ED_SCAN, NULL, 8 * 27, 
        MacScanM$energyDetectList, NULL);
        }
    }

  return SUCCESS;
}

# 9 "../../../zigzag/IEEE802_15_4/SysTimer/public/ITimerSymbol.nc"
static  result_t MacSyncM$InitialTimer$Stop(void){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TimerSymbol2M$ITimerSymbol$Stop(6U);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 238 "../../../zigzag/ZigBee/implementation/NIBM.nc"
static  void NIBM$Reset$reset(void)
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

# 475 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static  void RouterM$Reset$reset(void)
{
  if (RouterM$wait_confirm && !RouterM$relaying) {
    RouterM$NLDE_Data$confirm(RouterM$current_nsduHandle, MAC_CHANNEL_ACCESS_FAILURE);
    }
#line 479
  RouterM$wait_confirm = FALSE;
  RouterM$reportErrorWhenGetFree = FALSE;
}

# 36 "../../../zigzag/ZigBee/implementation/NeighborTableM.nc"
static  void NeighborTableM$Reset$reset(void)
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

# 254 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/TimerSymbolAsyncM.nc"
static  result_t TimerSymbolAsyncM$IStdControl$start(void)
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

# 737 "../../../zigzag/IEEE802_15_4/Phy/private/lowpower_cc2420/PhyCC2420M.nc"
static  result_t PhyCC2420M$IStdControl$start(void)
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

#line 194
static  result_t MacFrameFormatM$IMacFrame$GetSrcPANId(const TMacFrame *const pMacFrame, TMacPANId *const pSrcPANId)
{
  if (pMacFrame != NULL && pSrcPANId != NULL) {
      *pSrcPANId = pMacFrame->srcPANId.value;
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

#line 117
static  result_t MacFrameFormatM$IMacFrame$GetSequenceNumber(const TMacFrame *const pMacFrame, 
TMacSequenceNumber *const pSequenceNumber)
{
  if (pMacFrame != NULL && pSequenceNumber != NULL) {
      *pSequenceNumber = pMacFrame->sequenceNumber.value;
      return SUCCESS;
    }
  return FAIL;
}

#line 299
static  result_t MacFrameFormatM$IMacFrame$GetLinkQuality(const TMacFrame *const pMacFrame, 
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

# 35 "../../../zigzag/IEEE802_15_4/SysTimer/private/telosb/LocalTime64M.nc"
static  void LocalTime64M$ILocalTime64$setLocalTimeAt(TSysTime at, uint64_t time)
{
  if (TRUE == LocalTime64M$ITimerSymbol$IsSet()) {
    LocalTime64M$ITimerSymbol$Stop();
    }
  LocalTime64M$baseTime = time;
  LocalTime64M$firedCount = at;

  LocalTime64M$ITimerSymbol$SetPeriodic(225000000L, 0);
  return;
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

# 180 "../../../zigzag/IEEE802_15_4/MacData/private/MacDataM.nc"
static result_t MacDataM$Receive(MacDataM$SF sf, TMacFrame *const pmacFrame)
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

# 80 "../../../zigzag/IEEE802_15_4/MacCommon/private/small/MacFrameFormatM.nc"
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

# 294 "../../../zigzag/ZigBee/implementation/RouterM.nc"
static result_t RouterM$IMacDATA_Indication(
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

# 22 "../../../zigzag/ZigSysM.nc"
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

# 24 "../../../zigzag/ZigNetM.nc"
  int16_t __net_send(const uint16_t dst_addr, const uint16_t data_size, 
const uint8_t *const data, uint8_t handle)
{
  result_t res;

#line 28
  res = ZigNetM$NLDE_Data$request(
  dst_addr, 
  data_size, (uint8_t *)data, handle, 
  0xff, 0x0, FALSE);


  return res == SUCCESS ? 0 : -5;
}

#line 54
  uint16_t __net_addr(void)
{
  TMacStatus status;
  TMacShortAddress shortAddress = ZigNetM$IMacGET$GetmacShortAddress(&status);

#line 58
  if (MAC_SUCCESS == status) {
    return (uint16_t )shortAddress;
    }
#line 60
  return 0xffff;
}

