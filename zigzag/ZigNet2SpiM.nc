#include    "ZigParam.h"

module ZigNet2SpiM {
    provides {
        interface StdControl;
    }
    uses {

        interface NLDE_Data as IZigDATA;
        interface ILocalTime64;
        interface NLME_Leave as IZigLeave;
        interface NLME_JoinParent as IZigParentJoin;
        interface NLME_NetworkFormation as IZigNetFormation;
        interface NLME_Reset as IZigReset;
        interface UniSARTControl as USARTControl;
        interface MSP430Interrupt as INTR_2;
        interface IMacGET;

    }
} implementation {

#define     ROOT_ADDRESS    0x0000
#define     USART_NSDU_HANDLER     0x20       
#define     MAX_HOPS   0xff
#define     DISCOVER_ROUTE   0x0
#define     SECURITY_ENABLE   FALSE
#define     MSG_HEADER_SIZE   4

bool    sending = FALSE;
uint8_t seqNum = 0;
uint8_t usartBufSize = 0;
NwkAddr usartDstAddr = 0;
uint8_t usartBuf[ MAX_ZIG_PACKET_SIZE ];

size_t serialize( void *dst, size_t max_size, const char *fmt, ... )  @spontaneous() @C();
size_t deserialize( const void *src, size_t size, const char *fmt, ... ) @spontaneous() @C(); 

void    usartTx( NwkAddr srcAddr, IEEEAddr srcExtAddr, uint8_t   nsduLength, uint8_t *nsdu );

task void taskSendEvents()
   {
    TMacShortAddress   shortAddress = call IMacGET.GetmacShortAddress(0);
    if( ( ROOT_ADDRESS == shortAddress )|| TOSH_READ_JOIN_PIN() ) {
        if( TOSH_READ_SATELLITE_INTA_2_PIN() ) {
            if( usartDstAddr != 0xfffe ) {
                call IZigDATA.request( usartDstAddr, usartBufSize, usartBuf, 
                    USART_NSDU_HANDLER, MAX_HOPS, DISCOVER_ROUTE, SECURITY_ENABLE );
                return;
            } else {
                uint8_t msg_type, ev_type;
                uint64_t  localtime;
                #define     SET_TIME_MSG_SIZE   12
                deserialize( usartBuf, usartBufSize, "1:1:1:1:8:1:", 0,0, &msg_type, 0,&localtime, &ev_type );
                if( (0x04 == msg_type)&&(ev_type == 0xff)) {
                    call ILocalTime64.setLocalTime(localtime);
                    //TOSH_TOGGLE_LED2_PIN();
                }
            }
        }
    }
    TOSH_CLR_SATELLITE_INTA_2_PIN();
    sending = FALSE;
    return;
   }

void sendEvents()
   {
    if( FALSE == sending ) {
        if( post taskSendEvents() ) {
            sending = TRUE;
        } else {
            TOSH_CLR_SATELLITE_INTA_2_PIN();
        }
    }
   }

event void IZigDATA.confirm( uint8_t nsduHandler, NwkStatus status) 
   {
    if( !sending ) {
        TOSH_CLR_SATELLITE_INTA_2_PIN();
        return;
    } else
        sending = FALSE;
    //TOSH_TOGGLE_LED2_PIN();
    TOSH_CLR_SATELLITE_INTA_2_PIN();
    return;
    if( USART_NSDU_HANDLER == nsduHandler ) {
        if( NWK_SUCCESS == status )
            TOSH_CLR_SATELLITE_INTA_2_PIN();
    }
    sendEvents();
    return;
   }

static IEEEAddr* workaround;

event void IZigParentJoin.indication( NwkAddr   shortAddr, IEEEAddr   extendedAddr,
   NwkCapabilityInfo   capabilityInformation, bool   secureJoin ) 
{
    #define     JOIN_MSG_SIZE   15 
    uint8_t     join_msg[ JOIN_MSG_SIZE ];


    workaround = &extendedAddr;
    serialize( join_msg, JOIN_MSG_SIZE, "1:1:1:1:8:1:2:", 
         (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x04, (uint8_t)(JOIN_MSG_SIZE-MSG_HEADER_SIZE),
         call ILocalTime64.getLocalTime(), (uint8_t)0x20,  shortAddr);
    usartTx( 0xfffe, extendedAddr, JOIN_MSG_SIZE, join_msg );
    return;
}

event void IZigLeave.indication( IEEEAddr deviceAddr )
{
    #define     LEAVE_MSG_SIZE  15 
    uint8_t     leave_msg[ LEAVE_MSG_SIZE ];    

    workaround = &deviceAddr;
    serialize( leave_msg, LEAVE_MSG_SIZE, "1:1:1:1:8:1:2:",
        (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x04, (uint8_t)(LEAVE_MSG_SIZE-MSG_HEADER_SIZE),
         call ILocalTime64.getLocalTime(), (uint8_t)0x21,
         (uint16_t)0x1);

    usartTx( 0xfffe, deviceAddr, LEAVE_MSG_SIZE, leave_msg );
    return;
   }

void    usartTx( NwkAddr srcAddr, IEEEAddr extAddr, uint8_t   nsduLength, uint8_t *nsdu )
{
    register uint8_t *data;
    uint8_t i = 80;

    workaround = &extAddr;

    TOSH_SET_SATELLITE_INTR_PIN();
    while( !TOSH_READ_SATELLITE_INTA_PIN() ) {
        if( --i == 0 ) {
            TOSH_CLR_SATELLITE_INTR_PIN();
            //TOSH_TOGGLE_LED0_PIN();
            return;
        }
        TOSH_uwait(10);
    }
    TOSH_CLR_SATELLITE_CSN_PIN();

    call USARTControl.isTxIntrPending();
    call USARTControl.rx(); //isRxIntrPending();

    /* transmit network address */
    data = (uint8_t *)&srcAddr;
    call USARTControl.tx( *data++ );

    while( !(call USARTControl.isTxIntrPending()) );
    call USARTControl.tx( *data );
    while( !(call USARTControl.isTxIntrPending()) );


    //    TOSH_TOGGLE_LED1_PIN();
    /* transmit extended address */
    data = (uint8_t*)&extAddr;
    for( i=0; i<sizeof(IEEEAddr); i++ ) {
        call USARTControl.tx( *data++ );
        while( !(call USARTControl.isTxIntrPending()) );
    }

    /* transmit data size */
    call USARTControl.tx( nsduLength );
    while( !(call USARTControl.isTxIntrPending()) );
    call USARTControl.tx( 0x00 );
    while( !(call USARTControl.isTxIntrPending()) );
    /* transmit data */

    data = nsdu;
    for( i=0; i<nsduLength; i++ ) {
        call USARTControl.tx( *data++ );
        while( !(call USARTControl.isTxIntrPending()) );
    }
        
    while( !(call USARTControl.isTxEmpty()) ) ;

    TOSH_SET_SATELLITE_CSN_PIN();
    TOSH_CLR_SATELLITE_INTR_PIN();

    //TOSH_TOGGLE_LED2_PIN();

    return;
}

event void IZigNetFormation.confirm( NwkStatus status )
{
    if( NWK_SUCCESS == status ) {
        TOSH_SET_JOIN_PIN();
    }
    return;    
}

event void IZigReset.confirm(NwkStatus status)
{
    if( NWK_SUCCESS == status ) {
        TOSH_CLR_JOIN_PIN();
        //TOSH_CLR_LED0_PIN();
    }
    return;
}

event void IZigDATA.indication( NwkAddr srcAddr, IEEEAddr srcExtAddr, uint8_t nsduLength,
    uint8_t* nsdu, uint8_t linkQuality)
{
    workaround = &srcExtAddr;
    usartTx( srcAddr, srcExtAddr, nsduLength, nsdu );
    return;
}

task void    taskUsartRx()
{
    register uint8_t *data;    
    uint8_t i = 100;

    TOSH_SET_SATELLITE_INTA_2_PIN();
    while( TOSH_READ_SATELLITE_INTR_2_PIN() ) {
        if( --i == 0 ) {
            TOSH_CLR_SATELLITE_INTA_2_PIN();
            return;
        }
        TOSH_uwait(5);
    }

	TOSH_CLR_SATELLITE_CSN_PIN();

	call USARTControl.isTxIntrPending();
    call USARTControl.rx();

    /* Receive network address */
    data = (uint8_t *)&usartDstAddr;
	call USARTControl.tx(0);
	while(!(call USARTControl.isRxIntrPending()));
	*data++ = call USARTControl.rx();

	call USARTControl.tx(0);
	while(!(call USARTControl.isRxIntrPending()));
	*data = call USARTControl.rx();

    /* Receive nsduLength */
	call USARTControl.tx(0);
	while(!(call USARTControl.isRxIntrPending()));
	usartBufSize = call USARTControl.rx();

  	call USARTControl.tx(0);
	while(!(call USARTControl.isRxIntrPending()));
	call USARTControl.rx();

    /* Receive nsdu*/
    if( (0<usartBufSize)&&(usartBufSize <= MAX_ZIG_PACKET_SIZE) ) {
        uint8_t i;
        data = usartBuf;
        for( i=0; i<usartBufSize; i++ ) {
            call USARTControl.tx(0);
            while(!(call USARTControl.isRxIntrPending())) ;
            *data++ = call USARTControl.rx();
        }
        sendEvents();
    } else 
        TOSH_CLR_SATELLITE_INTA_2_PIN();

    TOSH_SET_SATELLITE_CSN_PIN();    
    return;
}

async event void INTR_2.fired() 
{
    if( !TOSH_READ_SATELLITE_INTA_2_PIN() )
        if( !post taskUsartRx() )
            TOSH_CLR_SATELLITE_INTA_2_PIN();
    call INTR_2.clear();
    return;
}

command result_t StdControl.init() 
   { 
    sending = FALSE;

    return SUCCESS; 
   }

command result_t StdControl.start() 
{

    TOSH_SET_SATELLITE_CSN_PIN();
    TOSH_MAKE_SATELLITE_CSN_OUTPUT();

    //TOSH_MAKE_JOIN_OUTPUT();
    //TOSH_CLR_JOIN_PIN();

    TOSH_CLR_SATELLITE_INTR_PIN();
    TOSH_CLR_SATELLITE_INTA_2_PIN();

    call USARTControl.setClockSource( SSEL_SMCLK );
    call USARTControl.setClockRate( 0x08, 0);
    call USARTControl.setModeSPI( TRUE ); /* master */
    call USARTControl.disableRxIntr();
    call USARTControl.disableTxIntr();

    call INTR_2.disable();
    call INTR_2.clear();
    call INTR_2.edge( TRUE );
    call INTR_2.enable();

    return SUCCESS;
}

command result_t StdControl.stop() 
{ 
    call USARTControl.disableSPI();

    return SUCCESS;
}

}

