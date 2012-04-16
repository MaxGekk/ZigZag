#include    "ZigParam.h"
#include    <syszig.h>

module  ZigSpi2AppM {
    provides {
        interface StdControl;
    }
    uses {
        interface UniSARTControl  as USARTControl;
        interface MSP430Interrupt as INTR;
        interface MSP430Interrupt as INTA_2;
        interface MSP430Interrupt as INTJOIN;
        interface ILocalTime64;
    }
} implementation {

size_t deserialize( const void *src, size_t size, const char *fmt, ... ) @spontaneous() @C();
size_t serialize( void *dst, size_t max_size, const char *fmt, ... )  @spontaneous() @C();

norace bool    spi_busy = FALSE;

norace static struct {
    uint16_t    dst_addr;
    uint16_t    data_size;
    uint8_t     *data;
    uint8_t     handle;
    bool        serv;
} tx;

static void tx_request()
{
    TOSH_CLR_SATELLITE_INTR_2_PIN();
    /* Если сетевой процессор находится в состоянии передачи
       ( передаёт не наши данные ), то выйти.
     */
    call INTA_2.disable(); 
    call INTA_2.edge( FALSE );
    call INTA_2.clear();
    if( call INTA_2.getValue() == TRUE ) {
        call INTA_2.enable();
        return;
    }
    /* Инициируем запрос на передачу */
    call INTA_2.edge( TRUE );
    call INTA_2.clear();
    TOSH_SET_SATELLITE_INTR_2_PIN();
    atomic {
        P5SEL = 0x0e;
        U1CTL = SWRST | CHAR | SYNC;
        U1TCTL = STC | CKPH;
        U1MCTL = 0;
        ME2 |= USPIE1;
        U1CTL &= ~SWRST;  
    }
    call INTA_2.enable();
    return;
}

norace static struct {
    uint16_t    src_addr;
    uint64_t    ext_addr;
    uint16_t    data_size;
    uint8_t     data[ MAX_ZIG_PACKET_SIZE ];
} rx;

task void   loop_tx()
{
    if( ISMODLOAD ) {
        __net_send_done( tx.handle, 0 );
        __net_recv( rx.src_addr, rx.ext_addr, rx.data_size, rx.data, 0 );
    }
    spi_busy = FALSE;
    return;
}

int16_t __net_send( const uint16_t dst_addr, const uint16_t data_size,
        const uint8_t *const data, uint8_t handle ) @spontaneous() @C()
{
    atomic {
        if( spi_busy == TRUE )
            return -1;
        spi_busy = TRUE;
    }
    tx.dst_addr = dst_addr;
    tx.data_size = data_size;
    tx.data = (uint8_t *)data;
    tx.handle = handle;
    tx.serv = FALSE;
    if( dst_addr == __net_addr() ) {
        uint16_t i;
        //TOSH_TOGGLE_LED_RED_PIN();

        for( i=0; i<data_size; i++ )
            rx.data[i] = data[i];
        rx.src_addr = __net_addr();
        rx.ext_addr = 0;
        rx.data_size = data_size;
        if( !post loop_tx() ) {
            spi_busy = FALSE;
            return -1;
        }
    } else {
        /* Если присоединены, то инициируем запрос на передачу */
        if( call INTJOIN.getValue() == TRUE  )
            tx_request();
    }
    return 0;
}

#define     SET_TIME_MSG_SIZE   12
static uint8_t  set_time_msg[ SET_TIME_MSG_SIZE ];
volatile uint64_t* __workaround;
int16_t __set_sys_time2( uint64_t time )@spontaneous() @C()
{
    __workaround = &time;

    atomic {
        if( spi_busy == TRUE )
            return -1;
        spi_busy = TRUE;
    }
    serialize( set_time_msg, SET_TIME_MSG_SIZE, "1:1:1:1:8:1:",
        (uint8_t)0, (uint8_t)0, (uint8_t)0x04, (uint8_t)9,time, (uint8_t)0xff );
    call ILocalTime64.setLocalTime( time );
    tx.dst_addr = 0xfffe;
    tx.data_size = SET_TIME_MSG_SIZE;
    tx.data = set_time_msg;
    tx.serv = TRUE;
    if( call INTJOIN.getValue() == TRUE  )
        tx_request();
    return 0;
}

task void   task_tx_done()
{
    spi_busy = FALSE;
    if( (tx.serv == FALSE) && ISMODLOAD )
        __net_send_done( tx.handle, 0 );
    return;
}

static inline void tx_data()
{
    register uint8_t *data;
    uint8_t i;

    /* Разрешена передача */
    call USARTControl.isTxIntrPending();
    call USARTControl.rx(); //isRxIntrPending();
    /* transmit network address */
    data = (uint8_t *)&(tx.dst_addr);
    call USARTControl.tx( *data++ );
    TOSH_CLR_SATELLITE_INTR_2_PIN();

    while( !(call USARTControl.isTxIntrPending()) );

    call USARTControl.tx( *data );
    while( !(call USARTControl.isTxIntrPending()) );

    /* transmit data size */
    //tx.data_size = 0x00;
    data = (uint8_t *)&(tx.data_size);
    call USARTControl.tx( *data++ );
    while( !(call USARTControl.isTxIntrPending()) );
    call USARTControl.tx( *data );
    while( !(call USARTControl.isTxIntrPending()) );

    /* transmit data */
    data = tx.data;

    for( i=0; i<tx.data_size; i++ ) {
        call USARTControl.tx( *data++ );
        while( !(call USARTControl.isTxIntrPending()) );
    }

    if( !post task_tx_done() ) 
        spi_busy = FALSE;

//    TOSH_TOGGLE_LED_WHITE_PIN();
    return;
}

async event void INTA_2.fired() 
{

    tx_data();    
    call INTA_2.clear();
    call INTA_2.disable();
    return;
}

task void task_net_enter()
{
    if(  ISMODLOAD )
        __net_enter( 0x0000 );
    return;
}

task void task_net_exit()
{
    if(  ISMODLOAD )
        __net_exit();
    return;
}

static void process_join()
{
    call INTJOIN.edge( TRUE );
    call INTJOIN.clear();
    if( call INTJOIN.getValue() == FALSE ) {
        post task_net_exit();
        return;
    }
    call INTJOIN.edge( FALSE );
    call INTJOIN.clear();
    post task_net_enter();
    return;
}

async event void INTJOIN.fired() 
{
    process_join();
    return;
}

#define     APP_MSG_HEADER_SIZE     4
#define     EVENT_HEADER_SIZE       9

task void task_rx_done()
{
    /* Разбор пакета с целью определения служебный это пакет или нет */
    /* Если пакет служебный то адрес источника 0xfffe */
    if( rx.src_addr == 0xfffe ) {
        uint8_t     msg_type;
        /* Разбираем заголовок */

        deserialize( rx.data, APP_MSG_HEADER_SIZE, "1:1:1:1:",
            (void *)0,(void *)0, &msg_type, (void *)0 ); 
        if( msg_type == 0x04 ) {
            uint8_t     event_type;
            deserialize( rx.data+APP_MSG_HEADER_SIZE, EVENT_HEADER_SIZE, "8:1:",
                (void *)0, &event_type );
            if( event_type == 0x20 ) {
                uint16_t saddr;
                deserialize( rx.data+APP_MSG_HEADER_SIZE+EVENT_HEADER_SIZE, 2+8, "2:",
                    &saddr );
                spi_busy = FALSE;
                if( ISMODLOAD ) {
                    __net_child_join( saddr, rx.ext_addr );
                    //TOSH_TOGGLE_LED_WHITE_PIN();
                }
                return;
            } else if( event_type == 0x21 ) {
                spi_busy = FALSE;
                if( ISMODLOAD )
                    __net_child_leave( rx.ext_addr );            
                return;
            }
        }
        spi_busy = FALSE;
        return;
    }
    if( ISMODLOAD )
        __net_recv( rx.src_addr, rx.ext_addr, rx.data_size, rx.data, 0 );
    spi_busy = FALSE;
    return;
}

async event void INTR.fired() 
{
    register uint8_t *data;   

    if( spi_busy == TRUE ) {
        call INTR.clear();
        return;
    }
    atomic {
        P5SEL = 0x0e;
        U1CTL = SWRST | CHAR | SYNC;
        U1TCTL = STC | CKPH;
        U1MCTL = 0;
        ME2 |= USPIE1;
        U1CTL &= ~SWRST;  
    }
	call USARTControl.isTxIntrPending();
    call USARTControl.rx();

    TOSH_SET_SATELLITE_INTA_PIN();

    /* Receive network address */
    data = (uint8_t *)&(rx.src_addr);
	while(!(call USARTControl.isRxIntrPending()));

	*data++ = call USARTControl.rx();
	while(!(call USARTControl.isRxIntrPending()));
	*data = call USARTControl.rx();

    /* Receive extended address */
    {
        uint8_t i;
        data = &rx.ext_addr;
        for( i=0; i<sizeof(IEEEAddr); i++ ) {
            while(!(call USARTControl.isRxIntrPending()));
            *data++ = call USARTControl.rx();
        }
    }

    /* Receive nsduLength */
    data = (uint8_t *)&(rx.data_size);
	while(!(call USARTControl.isRxIntrPending()));
	*data++ = call USARTControl.rx();
	while(!(call USARTControl.isRxIntrPending()));
	*data = call USARTControl.rx();

    /* Receive nsdu*/
    if( 0 < rx.data_size ) {
        uint8_t i;
        data = rx.data;
        for( i=0; i<rx.data_size; i++ ) {
            while(!(call USARTControl.isRxIntrPending())) ;
            *data++ = call USARTControl.rx();
        }
        if( post task_rx_done() )
            spi_busy = TRUE;
    }
//    TOSH_TOGGLE_LED_RED_PIN();
    call INTR.clear();
    TOSH_CLR_SATELLITE_INTA_PIN();
    return;
}

uint16_t    __net_addr() @spontaneous() @C()
{
    return 0x0000; /* Координатор */
}

command result_t StdControl.init() 
   { 
    return SUCCESS; 
   }

command result_t StdControl.start() 
{
    spi_busy = FALSE;

    TOSH_MAKE_SATELLITE_INTR_2_OUTPUT();
    TOSH_CLR_SATELLITE_INTR_2_PIN();

    call INTR.edge( TRUE );
    call INTR.clear();
    call INTR.enable();

    call INTA_2.disable();

    process_join();

    call USARTControl.setModeSPI( FALSE ); // slave
    call USARTControl.disableRxIntr();
    call USARTControl.disableTxIntr();  
    
//    TOSH_TOGGLE_LED_WHITE_PIN();

    return SUCCESS;
}

command result_t StdControl.stop() 
{ 
    spi_busy = TRUE;
    TOSH_CLR_SATELLITE_INTR_2_PIN();
    call INTR.disable();
    call INTA_2.disable();
    call INTJOIN.disable();

    call USARTControl.disableSPI();

    return SUCCESS;
}

}

