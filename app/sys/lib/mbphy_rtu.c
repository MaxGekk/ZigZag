/*! @file   mbphy_rtu.c
 *  @brief  Реализация ModBus RTU
 *  @author Max Gekk
 *  @date   декабрь 2007
 *  @version 1
 */   

#include    <syszig.h>
#include    <_zigzag.h>
#include    <modbus.h>
#include    <msp430.h>
#include    <msp430dma.h>
#include    <msp430timera.h>
#include    <platform.h>

#ifndef MODBUS_USART
#define     MODBUS_USART    1
#endif

#if MODBUS_USART == 0
#include    <msp430uart0.h>
#else
#include    <msp430uart1.h>
#endif


#if MODBUS_USART == 0
#define     LED_PORT     2
#define     RED_PIN     0x10
#define     BLUE_PIN    0x10
#define     GREEN_PIN   0x80
#else
#define     LED_PORT     6
#define     RED_PIN     0x1
#define     BLUE_PIN    0x4
#define     GREEN_PIN   0x8
#endif
/*
#define     LED_PORT     4       
#define     RED_PIN     0x1
#define     BLUE_PIN    0x10
#define     GREEN_PIN   0x8
*/

/* Скорость приёма и передачи по UART*/
#define     BAUDRATE    115200UL
//#define     BAUDRATE    2400UL
/* Частота основного источника тактирования. Тактирование производится только от него. */
//#define     CLOCK       8000000UL
#define     CLOCK       31250UL

/* Интервал тишины */
#define     SILENCE_INTERVAL   (((CLOCK*70)/(BAUDRATE*2)) + 10)

/* Настройки параметров UART */
#define     UCTL_VALUE  (CHAR | SPB)      /* 8-бит, без проверки чётности, 2 стоповых*/
//#define     UCTL_VALUE  (CHAR)      /* 8-бит, без проверки чётности*/
//#define     UCTL_VALUE  (PENA)    /* 7-бит, проверка чётности */
/* Настройки скорости приёмапередачи */
#if BAUDRATE == 115200UL
#define     UBR_SMCLK   70U
#define     UMCTL_SMCLK 0x10U
#elif   BAUDRATE == 300UL
#define     UMCTL_SMCLK     0x682aU 
#define     UBR_SMCLK       0x3fU
#elif   BAUDRATE == 2400UL
#define     UMCTL_SMCLK     7U 
#define     UBR_SMCLK       3333U 
#endif


#define TX_UART_OUT 0
#define TX_IO       1
/* Значение, записываемое в регистр TACTL после приёма очередного байта.
 * Тактирование от SMCLK, Режим вверх ( счёт до TACCR0 ). TACLR - сбрасывает TAR.
 * Прерывания разрешены.
 * XXX: При изменении CLOCK не забыть поменять TASSEL_xx
 * */
static const   uint16_t  tactl = TASSEL_01 | MC_01 | TACLR | TAIE;

static  struct {
    uint8_t     buf[ MB_BUF_SIZE ];    /* Буфер отправки */
    result_t    result;
} tx;


static void config_tx_pin(uint8_t dir)
{
   if(dir == TX_UART_OUT)
   {
#ifdef MB_CONVERTER
      MB_CTRL_PORT_OUT |=  (1<<MB_DE);
      MB_CTRL_PORT_DIR |=  (1<<MB_DE);
#endif
      P3SEL |= (1 << UTXDx);
      P3DIR |= (1 << UTXDx);
   }
   else
   {
#ifdef MB_CONVERTER
      MB_CTRL_PORT_DIR &= ~(1<<MB_DE);
#endif
      P3DIR &= ~(1 << UTXDx);
      P3SEL &= ~(1 << UTXDx);
   }
}

/* Получение указателя на буфер отправки */
uint8_t*    mb_get_txbuf()
{
    return tx.buf;
}

/* Передача данных из буфера отправки */
result_t    mb_tx( size_t  size )
{

    if( MB_BUF_SIZE < size ) {
        //port_write( LED_PORT, GREEN_PIN, PIN_LO );
        return EINVAL;
    }
    config_tx_pin(TX_UART_OUT);

    /* останавливаем DMA приема */
    DMA1CTL &= ~(DMAEN|DMAIE); DMA1CTL &= ~DMAIFG;
    DMA2CTL &= ~(DMAEN|DMAIE); DMA2CTL &= ~DMAIFG;


    // turn off rx
    P3SEL &= ~(1 << URXDx);

/*
    #ifdef MB_CONVERTER
    // XXX: turn on converter tx
    MB_CTRL_PORT_OUT |= (1<<MB_DE);
    //MB_CTRL_PORT_OUT |= ((1<<MB_DE) | (1<<MB_RE));
    #endif
*/
    /* Запуск канала 0 на передачу по USART */
    DMA0SZ = size;
    DMA0CTL |= DMAEN | DMAIE;
    
    return ENOERR;
}

static void    _mb_tx_done()
{
    mb_tx_done( tx.result );
    return;
}

__attribute__((weak)) void    mb_tx_done( result_t    result ) {}

static void     reinit_dma_timer()
{
    TACTL = 0;
    DMA2CTL &= ~DMAIFG;
    DMA2SA  = (uint16_t)&tactl;
    DMA2DA  = (uint16_t)&TACTL;
    DMA2SZ  = 65535;  
    DMA2CTL = DMAEN | DMAIE;
}

static  struct {
    uint8_t     buf[ MB_BUF_SIZE ];    /* Буфер приёма */
    uint8_t     size;   /* Текущий размер буфера приёма */
} rx;

static void    _mb_rx_done()
{
//    port_write( LED_PORT, RED_PIN, PIN_LO );
    mb_rx_notify( rx.buf, rx.size );
    return;
}

__attribute__((weak)) void    mb_rx_notify( uint8_t   *buf, uint8_t   size ) {}

static  volatile uint16_t t;

static void reinit_dma_rx()
{
    /* Инициализация канала 1 */
    t = URXBUF;
    IFGx &= ~URXIFGx;
    DMA1CTL &= ~DMAIFG;
    DMA1SA  = (uint16_t)&URXBUF;
    DMA1DA  = (uint16_t)(rx.buf);
    DMA1SZ  = MB_BUF_SIZE;
    DMA1CTL = DMASRCBYTE | DMADSTBYTE | DMADSTINCR_11 | DMALEVEL | DMAEN | DMAIE;
   
    reinit_dma_timer();
}

static  void    rx_done_irq( int buf_overflow  )
{
    /* Остановить таймер A */
    TACTL = 0;
    /* Истёк интервал тишины. Конец фрейма. */   

    if( DMA1SZ < MB_BUF_SIZE ) rx.size = MB_BUF_SIZE - DMA1SZ;
    else { 
      if( buf_overflow ) rx.size = MB_BUF_SIZE;
      else rx.size = 0;
   }
   // else rx.size = MB_BUF_SIZE;
    //if( DMA1SZ == MB_BUF_SIZE )
    //    port_write( LED_PORT, RED_PIN, PIN_LO );
    //port_write( LED_PORT, RED_PIN, PIN_HI );

    
    if( IS_OK(__post_task(_mb_rx_done)) ) {
        /* Останавливаем 1 и 2 каналы DMA */
        DMA1CTL &= ~(DMAEN|DMAIE); DMA1CTL &= ~DMAIFG;
        DMA2CTL &= ~(DMAEN|DMAIE); DMA2CTL &= ~DMAIFG;
        return;
    }
    reinit_dma_rx();
    return;    
}

static void    tune_timer( int for_rx )
{
    if( for_rx ) {
        TACTL = 0;
        TACCR0 = SILENCE_INTERVAL;
        TACCTL0 = CCIE; 
        TACCTL1 = 0;
    } else {
        TACCTL0 = 0;
        TACCTL1 = CCIE; 
        TACCTL2 = 0;
        TACCR1 = SILENCE_INTERVAL;
        TAR = 0;
        TACTL = TASSEL_01 | MC_10 | TAIE;
    }
}

IRQ_HANDLER( DACDMA_IRQ )
{
    /* Флаги DMAIFG не сбрасываются автоматически. 
     * Поэтому их нужно сбрасывать программно */ 
    if (DMA0CTL & DMAIFG) {
        DMA0CTL &= ~DMAIFG;
        if (DMA0CTL & DMAABORT){
            DMA0CTL &= ~DMAABORT;
            tx.result = EABORT;
        } else 
            tx.result = ENOERR;
        tune_timer(0);
        return;
    }
    if (DMA1CTL & DMAIFG) {
        DMA1CTL &= ~DMAIFG;
        if (DMA1CTL & DMAABORT)
            DMA1CTL &= ~DMAABORT;
        //reinit_dma_rx();
        rx_done_irq(1);
        //port_write( LED_PORT, GREEN_PIN, PIN_LO );
        return;
    }
    if (DMA2CTL& DMAIFG) {  
        DMA2CTL &= ~DMAIFG;
        if (DMA2CTL & DMAABORT)
            DMA2CTL &= ~DMAABORT;
        reinit_dma_timer();
        //port_write( LED_PORT, BLUE_PIN, PIN_LO );
    }
    return;
}


void    mb_rx_enable( uint16_t  wait_duration )
{
    // turn on rx pin

/*
        #ifdef MB_CONVERTER
        // XXX: turn on converter rx
        MB_CTRL_PORT_OUT &= ~((1<<MB_DE));
        //MB_CTRL_PORT_OUT &= ~((1<<MB_DE) | (1<<MB_RE));
        #endif
*/

    P3SEL |= (1 << URXDx);
//            port_write( LED_PORT, RED_PIN, PIN_LO );

    rx.size = 0;
    reinit_dma_rx();
    if( wait_duration != 0 ) {
        if( 2048 <= wait_duration )
            TACCR0 = 65535;
        else {
            TACCR0 = wait_duration<<5;
            //port_write( LED_PORT, RED_PIN, PIN_LO );
        }
        TACTL = tactl;
    }
    return;
}

IRQ_HANDLER( TIMERA0_IRQ )
{

    //port_write( LED_PORT, GREEN_PIN, PIN_LO );
    rx_done_irq(0);
}

IRQ_HANDLER( TIMERA1_IRQ )
{
    config_tx_pin(TX_IO);
    //__post_task( _mb_tx_done );
    tune_timer(1);
            //port_write( LED_PORT, RED_PIN, PIN_HI );
    _mb_tx_done();
}

void    init_modbus()
{

    // LEDS
    /*
    port_attr_t   port_attr;

    PIN_SET( port_attr.dir, (BLUE_PIN|GREEN_PIN|RED_PIN), PIN_HI );  
    PIN_CLEAR( port_attr.sel, BLUE_PIN|GREEN_PIN|RED_PIN );          
    port_set_attr( LED_PORT, BLUE_PIN|GREEN_PIN|RED_PIN, &port_attr );
    port_write( LED_PORT, BLUE_PIN|RED_PIN|GREEN_PIN, PIN_LO );
    */

    // XXX: other LEDS
    //P6SEL &= ~3;
    //P6DIR |= 3;
    //P6OUT &= ~3;

    if( ISZIGLOAD ) __critical_enter();
    else    asm("dint");
    /* Настройка UART1 */
    UCTL = SWRST | UCTL_VALUE;
    
    UTCTL &= ~(SSEL_0 | SSEL_1 | SSEL_2 | SSEL_3);
    UTCTL |= SSEL_SMCLK;
    UBR0 = UBR_SMCLK & 0x0FF;
    UBR1 = (UBR_SMCLK >> 8) & 0x0FF;
    UMCTL = UMCTL_SMCLK;
    MEx &= ~USPIEx;             // SPI module disable
    MEx |= (UTXEx | URXEx);     // UART module enable

    UCTL &= ~SWRST;

    //IFGx &= ~(UTXIFGx | URXIFGx);
    //IEx &= ~(UTXIEx | URXIEx);   // interrupt enable


#ifdef MB_CONVERTER
    // init rs485 converter
    MB_CTRL_PORT_SEL &= ~((1<<MB_DE) | (1<<MB_RE));
    MB_CTRL_PORT_DIR &= ~(1<<MB_DE) | (1<<MB_RE);
    //MB_CTRL_PORT_DIR |= (1<<MB_DE) | (1<<MB_RE);

    // set the converter to tx mode
    //MB_CTRL_PORT_OUT |= (1<<MB_RE) | (1<<MB_DE);
    
    // XXX: turn on rx
    MB_CTRL_PORT_OUT &= ~((1<<MB_RE) | (1<<MB_DE));
    MB_CTRL_PORT_DIR |= (1 << MB_RE);
#endif

    P3SEL |= (1 << UTXDx);
    P3DIR |= (1 << UTXDx);
    P3SEL |= (1 << URXDx);

    config_tx_pin(TX_IO);

    // чтобы не дергалось при выключении функции UART
    P3DIR &= ~(1 << URXDx);

    /* Настройка таймера А */
    tune_timer(1);

    /* Выбор условий запуска каналов DMA */
#if MODBUS_USART == 0
    DMACTL0 = ( DMA0TSEL_0100    /* Канал 0 работает на передачу по USART0 */
        | DMA1TSEL_0011     /* Канал 1 работает на приём по USART0 */
        | DMA2TSEL_1111 );  /* Канал 2 запускает таймер после изменения уровня на вводе */    
#else
    DMACTL0 = ( DMA0TSEL_1010    /* Канал 0 работает на передачу по USART1 */
        | DMA1TSEL_1001     /* Канал 1 работает на приём по USART1 */
        | DMA2TSEL_1111 );  /* Канал 2 запускает таймер после изменения уровня на вводе */    
#endif

    /* Инициализация канала 0 */
    DMA0SA  = (uint16_t)(tx.buf);
    DMA0DA  = (uint16_t)&UTXBUF;
    DMA0CTL = DMASRCBYTE | DMADSTBYTE | DMASRCINCR_11 | DMALEVEL;
   
    /* Инициализация канала 2 */
    P2DIR   &= ~(1<<6);
    P2SEL   |= (1<<6);
    P2IE    &= ~(1<<6);
    P2IFG   &= ~(1<<6);

    reinit_dma_rx();

    if( ISZIGLOAD ) __critical_exit();
    else    asm("eint");

    return;
}

INIT( init_modbus );

