#include    <_zigzag.h>
#include    <syszig.h>

//#define     DBGLOG
#include    <zzDebug.h>
#include    <msp430.h>

#define     TOTAL_PACKET    4
#define     TOTAL_MESSAGE   8

typedef struct {
    net_addr_t  dst_addr;
    net_addr_t  src_addr;
	ieee_addr_t	src_ext_addr;
    size_t      size;
    uint8_t     flags;
    uint8_t     msg_count;
    uint8_t     lock_count;
    uint8_t     __align;
    uint8_t     payload[ MAX_PAYLOAD_SIZE ];
} packet_t;

uint8_t     busy_pkt_count;
packet_t    packet[ TOTAL_PACKET ];

typedef struct {
    uint8_t     flags;
    uint16_t    obj_offset;
    void        *data_ptr;
    packet_t    *pkt_ptr;
} message_t;

#define DST_PORT( msg_i )    ((msg_header_t *)message[msg_i].data_ptr)->dst_port 
#define SRC_PORT( msg_i )    ((msg_header_t *)message[msg_i].data_ptr)->src_port
#define MSG_TYPE( msg_i )    ((msg_header_t *)message[msg_i].data_ptr)->type
#define MSG_BODY_SIZE( msg_i )    ((msg_header_t *)message[msg_i].data_ptr)->body_size
#define BODY_PTR( msg_i )    (void *)(((uint8_t *)message[msg_i].data_ptr)+MSG_HEADER_SIZE)

uint8_t     busy_msg_count;
message_t   message[ TOTAL_MESSAGE ];

#define     SYSTEM_MASK 0x0f
#define     BUSY_MASK   0x01
#define     IS_BUSY( entity ) ( (entity).flags & BUSY_MASK )
#define     IS_FREE( entity ) (!IS_BUSY( entity ))
#define     SET_BUSY( entity ) (entity).flags |= BUSY_MASK
#define     CLEAR_BUSY( entity ) (entity).flags &= ~BUSY_MASK

#define     LOCK_MASK   0x02
#define     IS_LOCK( entity )   ( (entity).flags & LOCK_MASK )
#define     LOCK( entity )  (entity).flags |= LOCK_MASK
#define     IS_UNLOCK( entity ) (!IS_LOCK( entity ))
#define     UNLOCK( entity ) (entity).flags &= ~LOCK_MASK

#define     DIRECT_MASK 0x04
#define     IS_OUTPUT( entity ) ( (entity).flags & DIRECT_MASK )
#define     SET_OUTPUT( entity ) (entity).flags |= DIRECT_MASK
#define     IS_INPUT( entity ) (!IS_OUTPUT( entity ))
#define     SET_INPUT( entity ) (entity).flags &= ~DIRECT_MASK


static void create_message( uint8_t msg_index )
{
    SET_BUSY( message[msg_index] );
    busy_msg_count++;
    return;
}

static void destroy_message( message_t *msg_ptr )
{
    CLEAR_BUSY( *msg_ptr );
    busy_msg_count--;
    return;
}

static void create_packet( uint8_t pkt_index )
{
    SET_BUSY( packet[pkt_index] );
    busy_pkt_count++;
    return;
}

static void destroy_packet( packet_t *pkt_ptr )
{
    CLEAR_BUSY( *pkt_ptr );
    pkt_ptr->size = 0;
    pkt_ptr->dst_addr = 0xFFFE;
    busy_pkt_count--;
    return;
}

static void link( uint8_t pkt_index, uint8_t msg_index )
{
    message[ msg_index ].pkt_ptr = &packet[ pkt_index ];
    packet[ pkt_index ].msg_count++;
    return;
}

static void unlink( uint8_t   msg_index )
{
    packet_t *pkt_ptr = message[ msg_index ].pkt_ptr;
    pkt_ptr->msg_count--;
    if( 0 == pkt_ptr->msg_count )
        destroy_packet( pkt_ptr );
    message[ msg_index ].pkt_ptr = 0;
    destroy_message( &message[ msg_index ] );        
}

static uint16_t net_addr()
{
    if( ISZIGLOAD ) {
        return __net_addr();
    }
    return BAD_ADDRESS;
}

msg_t   _msg_new( const uint16_t    obj_offset, net_addr_t dst_addr, app_port_t    dst_port, 
        app_port_t  src_port, uint8_t msg_type, size_t  body_size, uint8_t flags )
{
    register uint16_t msg_i, pkt_i;
    result_t  res = ENOERR;

    //DBG("_msg_new");
    if( MAX_PAYLOAD_SIZE < body_size + MSG_HEADER_SIZE ) {
        res = EINVAL;
        goto exit;
    }
    if( TOTAL_MESSAGE == busy_msg_count ) {
        res = ENOMEM;
        goto exit;
    }

    for( msg_i = 0; msg_i < TOTAL_MESSAGE; msg_i++ )
        if( IS_FREE(message[ msg_i ]) ) {
            create_message( msg_i );
            UNLOCK( message[ msg_i] ); SET_OUTPUT( message[ msg_i] );
            break;
        }

    if( TOTAL_MESSAGE <= msg_i ) {
        res = ENOMEM;
        goto exit;
    }

    for( pkt_i =0; pkt_i < TOTAL_PACKET; pkt_i++ ) {
        if( !( IS_BUSY( packet[pkt_i]) && IS_UNLOCK( packet[pkt_i]) && IS_OUTPUT( packet[pkt_i]) ) )
            continue;
        if( dst_addr != packet[pkt_i].dst_addr )
            continue;
        if( ( MAX_PAYLOAD_SIZE - packet[pkt_i].size ) < body_size + MSG_HEADER_SIZE )
            continue;
        break;
    }

    if( TOTAL_PACKET <= pkt_i ) {
        if( TOTAL_PACKET == busy_pkt_count ) {
            res = ENOMEM;
            goto destroy_msg;
        }
        for( pkt_i = 0; pkt_i < TOTAL_PACKET; pkt_i++ )
            if( IS_FREE( packet[pkt_i] ) ) {
                create_packet( pkt_i );
                UNLOCK( packet[pkt_i] ); SET_OUTPUT( packet[pkt_i]);
                packet[pkt_i].dst_addr = dst_addr;  packet[pkt_i].src_addr = net_addr();
                packet[pkt_i].msg_count = 0; packet[pkt_i].lock_count = 0;
                packet[pkt_i].size = 0;
                break;
            }
        if( TOTAL_PACKET <= pkt_i ) {
            res = ENOMEM;
            goto destroy_msg;
        }
    }

    message[msg_i].flags &= SYSTEM_MASK; 
    message[msg_i].flags |= (flags & ~SYSTEM_MASK);
    message[msg_i].obj_offset = obj_offset;
    message[msg_i].data_ptr = &packet[pkt_i].payload[ packet[pkt_i].size ];

    link(  pkt_i, msg_i );
    if( _MFLAG_RAW == (message[ msg_i ].flags & _MFLAG_RAW) ) {
        packet[pkt_i].size += body_size;
    } else {
        packet[pkt_i].size += body_size + MSG_HEADER_SIZE;

        DST_PORT( msg_i ) = dst_port;
        SRC_PORT( msg_i ) = src_port;
        MSG_TYPE( msg_i ) = msg_type;
        MSG_BODY_SIZE( msg_i ) = body_size;
    }
    return (msg_t)msg_i;

destroy_msg:
    destroy_message( &message[msg_i] );
exit:
    return (msg_t)res;
}

result_t   _msg_info( const uint16_t obj_offset, msg_t   msg,  struct  msginfo *info )
{
    register uint16_t msg_i = msg;
    if( ( 0 == info )||( TOTAL_MESSAGE <= msg_i ) )
        return EINVAL;
    if( IS_FREE( message[msg_i] )
            ||( message[msg_i].obj_offset != obj_offset )||IS_LOCK(message[msg_i])) 
        return EACCESS;

    info->dst_addr = message[msg_i].pkt_ptr->dst_addr;
    info->dst_port = DST_PORT( msg_i );

    info->src_addr = message[msg_i].pkt_ptr->src_addr;
	info->src_ext_addr = message[msg_i].pkt_ptr->src_ext_addr;
    info->src_port = SRC_PORT( msg_i );

    info->msg_type = MSG_TYPE( msg_i );
    info->body_size = MSG_BODY_SIZE( msg_i );
    info->body_ptr = BODY_PTR( msg_i );

    return ENOERR;
}

void __recv_handler();

__attribute__((used)) void    __net_recv( uint16_t src_addr, uint64_t src_ext_addr, uint16_t data_size,  uint8_t *data, uint8_t lqi )
{
    register uint16_t pkt_i;


    if( TOTAL_PACKET == busy_pkt_count )
        return;
    /* Поиск свободного пакета */
    for( pkt_i = 0; pkt_i < TOTAL_PACKET; pkt_i++ )
        if( IS_FREE( packet[pkt_i] ) )
            break;
    if( TOTAL_PACKET <= pkt_i )
        return;

    create_packet( pkt_i );
    LOCK( packet[pkt_i] );
    SET_INPUT( packet[pkt_i] );

    packet[pkt_i].dst_addr = net_addr(); 
    packet[pkt_i].src_addr = src_addr;
	packet[pkt_i].src_ext_addr = src_ext_addr;
    packet[pkt_i].msg_count = 0;
    packet[pkt_i].lock_count = 0;
    packet[pkt_i].size = data_size;

    { /* Копируем принятый пакет */
        uint8_t i;
        for( i=0; i<data_size; i++ ) {
            packet[pkt_i].payload[i] = data[i];
            //DBG2("payload[%hhx]=%hhx",i,packet[pkt_i].payload[i]);
        }
    }

    if( IS_ERROR( __post_task( __recv_handler ) ))
        destroy_packet( &packet[pkt_i] );
    return;
}

__attribute__((used)) void __recv_handler()
{
    register uint16_t pkt_i, msg_i;
    int32_t     obj_offset;
    size_t      msg_size;
    size_t      pkt_position = 0;
    size_t      pkt_size;

    //DBG("start __recv_handler");
    for( pkt_i = 0; pkt_i <TOTAL_PACKET; pkt_i++ )
        if( IS_BUSY( packet[pkt_i]) && IS_LOCK( packet[pkt_i]) && IS_INPUT( packet[pkt_i]) )
            break;

    if( TOTAL_PACKET <= pkt_i )    
        return;

    UNLOCK( packet[pkt_i] );

    msg_i = 0;

    //DBG1("pkt_size = %x", (uint16_t)packet[pkt_i].size);

    pkt_size = packet[pkt_i].size; // сохраняем размер в локальной переменной, так как пакет может быть удален в процессе разбора и создан новый в этом же месте.
    while( MSG_HEADER_SIZE + pkt_position <= pkt_size) {
        //DBG1("pkt_position = %x", (uint16_t)pkt_position);
        if( TOTAL_MESSAGE == busy_msg_count )
        {
            break;
        }
        /* Выделяем место под сообщение */
        for(; msg_i < TOTAL_MESSAGE; msg_i++ )
            if( IS_FREE(message[ msg_i ]) ) {
                create_message( msg_i );
                UNLOCK( message[ msg_i] ); SET_INPUT( message[ msg_i] );
                break;
            }
        //DBG1("alloc message msg_i = %x", msg_i);
        if( TOTAL_MESSAGE <= msg_i )
            break;

        // KILL IT message[msg_i].body_size =  packet[pkt_i].size - (pkt_position + MSG_HEADER_SIZE);
        link( pkt_i, msg_i );
        message[msg_i].data_ptr = &(packet[pkt_i].payload[ pkt_position ]);

        msg_size = MSG_BODY_SIZE(msg_i) + MSG_HEADER_SIZE;

        if ( msg_size > packet[pkt_i].size - pkt_position ){
            // указанный в сообщении размер больше чем остаток пакета
            unlink( msg_i );
            break;
        }

        if( IS_ERROR( obj_offset = port2offset( DST_PORT(msg_i) ) ) ) {
            //DBG("not found objoffset");
            unlink( msg_i );
            break;
        }

        message[msg_i].obj_offset = (uint16_t)obj_offset;
        //DBG1("MSG_TYPE= %x", (uint16_t)MSG_TYPE(msg_i));

        /* Если сообщение связано с атрибутами, то передаём его модулю атрибутов */
        if( ( MSG_ATTR_GET == MSG_TYPE(msg_i) )||( MSG_MULTI_ATTR_GET == MSG_TYPE(msg_i))
         ||( MSG_ATTR_SET == MSG_TYPE(msg_i) )||( MSG_MULTI_ATTR_SET == MSG_TYPE(msg_i) ) ) {
            _msg_attr_recv( obj_offset, msg_i );
            //DBG2("call _msg_attr_recv msg_size = %d MSG_TYPE= %x", (int16_t)msg_size, (uint16_t)MSG_TYPE(msg_i));
        } else { /* Вызываем объект, которому предназначено сообщение */    
            const msg_recv_ft *mr =  &_begin_msg_recv;
            mr += obj_offset;
            (*mr)( (msg_t)msg_i );         
            //DBG1("call app msg_size = %d", (int16_t)msg_size);
        }
        pkt_position += msg_size;

    }
    //DBG1("2 busy_pkt %d", (uint16_t)busy_pkt_count);

    //DBG("return from __recv_handler");
    return;
}

bool_t  sending;
bool_t  joined;

static void send_packet();

result_t   _msg_send( const uint16_t obj_offset, msg_t   msg )
{
    register uint16_t msg_i;
    msg_i = msg;
    packet_t    *pkt_ptr;

    if( ( TOTAL_MESSAGE <= msg_i )|| IS_FREE( message[msg_i] ) 
            || IS_INPUT( message[msg_i]) )
        return EINVAL;

    if( message[msg_i].obj_offset != obj_offset )
        return EACCESS;

    if( IS_LOCK( message[msg_i] ) )
        return EBUSY;

    LOCK( message[msg_i] );

    pkt_ptr = message[msg_i].pkt_ptr;
    pkt_ptr->lock_count++;

    if( pkt_ptr->lock_count < pkt_ptr->msg_count )
       return ENOERR;

    /* Все сообщения готовы к отправке. Помечаем пакет как готовый к отправке. */
    LOCK( *pkt_ptr );

    send_packet();

    return ENOERR;
}

__attribute__((used)) void __send_packet()
{
    uint16_t my_addr;

    my_addr = net_addr();

    if( BAD_ADDRESS == my_addr ) {
        sending = FALSE;
        return;
    }

    if( ( ROOT_ADDRESS == my_addr )||joined ) {
        register uint16_t pkt_i;

        for( pkt_i = 0; pkt_i < TOTAL_PACKET; pkt_i++ )
            if( IS_BUSY( packet[pkt_i] ) && IS_LOCK( packet[pkt_i]) && IS_OUTPUT( packet[pkt_i]) )
                break;

        if( ( pkt_i < TOTAL_PACKET )&& ISZIGLOAD) {
            int16_t result;
            //DBG("__net_send request");
            result = __net_send( 
                    packet[pkt_i].dst_addr ,
                    packet[pkt_i].size,
                    packet[pkt_i].payload,
                    (uint8_t)pkt_i
                    );
            if( IS_OK( result ) )
                return;
        }
    }
    sending = FALSE;
    return;
}

static void send_packet()
{
    if( ( FALSE == sending ) && ( ISZIGLOAD ) ) {
        if( IS_OK( __post_task( &__send_packet ) ) ) 
            sending = TRUE;
    }
}


__attribute__((used)) void __net_send_done( uint8_t handle, int8_t status )
{
    register uint16_t msg_i, pkt_i;

    if( !sending ) {
        /*XXX Пришёл __net_send_done, хотя мы не вызывали __net_send */
        goto exit;
    }

    sending = FALSE;
    pkt_i = handle;

    if( TOTAL_PACKET <= pkt_i) {
        /*XXX Такого не может быть!!!*/
        goto send_next_packet;
    }
  
    /* Проверим: предназначен ли этот пакет для отпраки */
    if( !(IS_BUSY(packet[pkt_i])&&IS_LOCK(packet[pkt_i])&&IS_OUTPUT(packet[pkt_i]) ) )
        goto send_next_packet;
    
    UNLOCK( packet[pkt_i] );

    /* Ищем все сообщения пакета и оповещаем объекты об окончании процедуры отправки */
    for( msg_i = 0; msg_i < TOTAL_MESSAGE; msg_i++ ) {
        if( !(IS_BUSY(message[msg_i])&&IS_LOCK(message[msg_i])&&IS_OUTPUT(message[msg_i]) ) )
            continue;
        if( &packet[pkt_i] != message[msg_i].pkt_ptr )
            continue;
             
        /* Сообщение из данного пакета */       
        UNLOCK( message[msg_i] );
        packet[pkt_i].lock_count--;
        /* Если не требуется подтверждения, то удаляем пакет*/
        if( _MFLAG_NO_CONFIRM == (message[ msg_i ].flags & _MFLAG_NO_CONFIRM) ) {
            unlink( msg_i );        
        } else {/* Извещаем прикладной объект */
            status_t s = STATUS_SUCCESS;
            const msg_send_done_ft  *msd = &_begin_msg_send_done;
            msd += message[ msg_i ].obj_offset;
            if( status < 0 )
                s = STATUS_MAX_ATTEMPTS;
            (*msd)( (msg_t)msg_i, s);
        }
    }
    if( packet[pkt_i].lock_count != 0 ) {
        /*XXX Очень странно. Какие то, сообщения мы не нашли.
         * Потенциальная утечка памяти. На всякий случай сбросим lock_count. Продлим агонию. */
        packet[pkt_i].lock_count = 0;
    }
send_next_packet:
    send_packet();
exit:
    return;
}

result_t _msg_destroy( uint16_t obj_offset, msg_t   msg )
{
    register uint16_t   msg_i;
    packet_t    *pkt_ptr;

    msg_i = (uint16_t)msg;

    if( (TOTAL_MESSAGE <= msg_i)||( IS_FREE(message[msg_i]) ) )
        return EINVAL;

    if( message[msg_i].obj_offset != obj_offset )
        return EACCESS;

    if( IS_LOCK( message[msg_i] ) )
        return EBUSY;

    if( IS_OUTPUT( message[msg_i] ) )
        return ENOSYS;

    unlink(msg_i);

    return ENOERR;
}

__attribute__((used)) void    __net_enter( uint16_t pan_id )
{
    joined = TRUE;
    send_packet();
    return;
}

__attribute__((used)) void    __net_exit()
{
    sending = FALSE;
    joined = FALSE;
    return;
}

void network_init()
{
    uint16_t i;
    for( i = 0; i < TOTAL_PACKET; i++ ) {
        CLEAR_BUSY( packet[i] ); UNLOCK( packet[i] );
        packet[i].dst_addr = BAD_ADDRESS; packet[i].src_addr = BAD_ADDRESS;
        packet[i].msg_count = packet[i].lock_count = 0;
        packet[i].size = 0;
    }
    busy_pkt_count = 0;
    for( i = 0; i < TOTAL_MESSAGE; i++ ) {
        CLEAR_BUSY( message[i] ); UNLOCK( packet[i] );
        message[i].obj_offset = 0xffff; message[i].data_ptr = 0;
        message[i].pkt_ptr = 0;
    }
    busy_msg_count = 0;

    sending = FALSE; joined = FALSE;

    return;
}

INIT( network_init );

__attribute__((weak)) void    msg_send_done( msg_t   msg, status_t    status ) {}
__attribute__((weak)) size_t    msg_recv( msg_t   msg ) { return 0; }

