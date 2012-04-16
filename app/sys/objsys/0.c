#include    <_zigzag.h>
#include    <syszig.h>

#define     OBJ   0
#define     PORT    0

size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num );
size_t  ATTR_FUNC( attr_set )( const uint8_t attr_num, const void *const from );
size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to );

#include    <zigzag.h> // Делаем системную и прикладную часть в одном файле.

#define     OBJOFFSET    onum2offset(OBJ)


#define     MSG_JOIN    0x20
#define     MSG_JOIN_BODY_SIZE  (19+1)

#define     MSG_LEAVE   0x21
#define     MSG_LEAVE_BODY_SIZE 19

/* Номера атрибутов */
/*** Специальные атрибуты */
#define     ATTR_ROLE           0x20        /* Роль устройства в сети */
#define     ATTR_TIME           0x21        /* Текущее время */
#define     ATTR_EXT_ADDR       0x22        /* Длинный адрес узла */
#define     ATTR_SHORT_ADDR     0x23        /* Короткий адрес узла */
#define     ATTR_CHANNEL        0x24        /* Номер канала */ 
#define     ATTR_PAN_ID         0x25        /* Номер сети */
#define     ATTR_MAX_CHILD      0x26        /* Максимальное число дочерних узлов */
#define     ATTR_MAX_DEPTH      0x27        /* Максимальная глубина сети */
#define     ATTR_CHILD          0x28        /* Текущее число дочерних узлов */
#define     ATTR_CHILD_EADDR_FIRST      0x30    /* Длинные адреса дочерних узлов */
#define     ATTR_CHILD_EADDR_LAST       0x3f
#define     ATTR_CHILD_SADDR_FIRST      0x40    /* Короткие адреса дочерних узлов */
#define     ATTR_CHILD_SADDR_LAST       0x4f
#define     ATTR_NODE_KIND              0x60    /* Идентификатор вида устройства */

/* Размеры атрибутов */
#define     ATTR_ROLE_SIZE          1
#define     ATTR_TIME_SIZE          8
#define     ATTR_EXT_ADDR_SIZE      8
#define     ATTR_SHORT_ADDR_SIZE    2
#define     ATTR_CHANNEL_SIZE       1
#define     ATTR_PAN_ID_SIZE        2
#define     ATTR_MAX_CHILD_SIZE     1
#define     ATTR_MAX_DEPTH_SIZE     1
#define     ATTR_CHILD_SIZE         1
#define     ATTR_CHILD_EADDR_SIZE       8
#define     ATTR_CHILD_SADDR_SIZE       2
#define     ATTR_NODE_KIND_SIZE         2

#define     MAX_CHILD   4

static struct {
    uint16_t    short_addr;
    uint64_t    ext_addr;
} child[ MAX_CHILD ] = { 
    {BAD_ADDRESS,0}, {BAD_ADDRESS,0},
    {BAD_ADDRESS,0}, {BAD_ADDRESS,0}
};

uint8_t child_count;    /* Число дочерних узлов */

common_attr_t common_attrs;


size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num )
{
    // сначала обработка общих атрибутов
    size_t size = common_attr_size(&common_attrs, attr_num);
    if (size != 0)
        return size;

    // иначе обработка атрибутов данного профиля
    switch( attr_num ) {
        case ATTR_ROLE: return ATTR_ROLE_SIZE;     
        case ATTR_TIME: return ATTR_TIME_SIZE;
        case ATTR_EXT_ADDR: return ATTR_EXT_ADDR_SIZE;   
        case ATTR_SHORT_ADDR: return ATTR_SHORT_ADDR_SIZE;    
        case ATTR_CHANNEL: return ATTR_CHANNEL_SIZE;    
        case ATTR_PAN_ID: return ATTR_PAN_ID_SIZE;    
        case ATTR_MAX_CHILD: return ATTR_MAX_CHILD_SIZE;    
        case ATTR_MAX_DEPTH: return ATTR_MAX_DEPTH_SIZE;    
        case ATTR_CHILD: return ATTR_CHILD_SIZE;    
        case ATTR_CHILD_EADDR_FIRST...ATTR_CHILD_EADDR_LAST: 
                         return ATTR_CHILD_EADDR_SIZE;  
        case ATTR_CHILD_SADDR_FIRST...ATTR_CHILD_SADDR_LAST: 
                         return ATTR_CHILD_SADDR_SIZE;  
        case ATTR_NODE_KIND:
                         return ATTR_NODE_KIND_SIZE;
        default: break;
    }
    return 0;
}
REG_ATTR_FUNC( attr_size );

size_t  ATTR_FUNC( attr_set )( const uint8_t attr_num, const void *const from )
{
    // сначала обработка общих атрибутов
    size_t size = common_attr_set(&common_attrs, attr_num, from);
    if (size != 0)
        return size;

    // иначе обработка атрибутов данного профиля
    switch( attr_num ) {
        case ATTR_ROLE...ATTR_CHILD: break;    
        case ATTR_CHILD_EADDR_FIRST...ATTR_CHILD_EADDR_LAST: break;
        case ATTR_CHILD_SADDR_FIRST...ATTR_CHILD_SADDR_LAST: break;
        default: return (size_t)0;
    }
    return ATTR_FUNC(attr_size)(attr_num);
}
REG_ATTR_FUNC( attr_set );

size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to )
{
    // сначала обработка общих атрибутов
    size_t size = common_attr_get(&common_attrs, attr_num, to);
    if (size != 0)
        return size;

    // иначе обработка атрибутов данного профиля
    switch( attr_num ) {
        case ATTR_ROLE: return ATTR_ROLE_SIZE;     
        case ATTR_TIME: {
            const uint64_t  local_time = __sys_time();
            memcopy( to, &local_time, ATTR_TIME_SIZE );
            }
            return ATTR_TIME_SIZE;
        case ATTR_EXT_ADDR: return ATTR_EXT_ADDR_SIZE;   
        case ATTR_SHORT_ADDR: {
            const uint16_t  saddr = __net_addr();
            memcopy( to, &saddr, ATTR_SHORT_ADDR_SIZE );
            }
            return ATTR_SHORT_ADDR_SIZE;    
        case ATTR_CHANNEL: return ATTR_CHANNEL_SIZE;    
        case ATTR_PAN_ID: return ATTR_PAN_ID_SIZE;    
        case ATTR_MAX_CHILD: return ATTR_MAX_CHILD_SIZE;    
        case ATTR_MAX_DEPTH: return ATTR_MAX_DEPTH_SIZE;    
        case ATTR_CHILD: 
            memcopy( to, &child_count, ATTR_CHILD_SIZE );
            return ATTR_CHILD_SIZE;    
        case ATTR_CHILD_EADDR_FIRST...ATTR_CHILD_EADDR_LAST: 
            memcopy( to, &child[ attr_num - ATTR_CHILD_EADDR_FIRST ].ext_addr, ATTR_CHILD_EADDR_SIZE);
            return ATTR_CHILD_EADDR_SIZE;  
        case ATTR_CHILD_SADDR_FIRST...ATTR_CHILD_SADDR_LAST: 
            memcopy( to, &child[ attr_num - ATTR_CHILD_SADDR_FIRST ].short_addr, ATTR_CHILD_SADDR_SIZE);
            return ATTR_CHILD_SADDR_SIZE;  
        case ATTR_NODE_KIND:
            {
                uint16_t kind = ZZ_NODE_KIND;
                memcopy(to, &kind, ATTR_NODE_KIND_SIZE);
                return ATTR_NODE_KIND_SIZE;
            }
        default: return (size_t)0;
    }
    return ATTR_FUNC(attr_size)(attr_num);
}
REG_ATTR_FUNC( attr_get );

void    __net_child_join( uint16_t short_addr, uint64_t ext_addr, uint8_t lqi )
{
    uint8_t i;
    size_t len;
    uint8_t buf[MSG_JOIN_BODY_SIZE];
    for( i = 0; i < MAX_CHILD; i++ )
        if( child[i].short_addr == BAD_ADDRESS ) {
            child[i].short_addr = short_addr;
            child[i].ext_addr = ext_addr;
            child_count++;
            break;
        }
    /* Формируем тело сообщения */
    len = serialize( buf, MSG_JOIN_BODY_SIZE, 
            "8:1:2:8:1:", __sys_time(), (uint8_t)MSG_JOIN, short_addr, ext_addr,lqi  );
    
    if( MSG_JOIN_BODY_SIZE == len )
        send_to_interested( OBJOFFSET, &common_attrs, PORT, MSG_EVENT, buf, len );
}

void    __net_child_leave( uint64_t ext_addr )
{
    uint8_t i;
    size_t len;
    uint8_t buf[MSG_LEAVE_BODY_SIZE];
    for( i = 0; i < MAX_CHILD; i++ )
        if( child[i].ext_addr == ext_addr ) {
            child[i].short_addr = BAD_ADDRESS;
            child_count--;
            break;
        }
    if( MAX_CHILD <= i )
        return;
    /* Формируем сообщение-событие об отсоединении и 
     * отправляем его заинтересованным сторонам */
            /* Формируем тело сообщения */
    len = serialize( buf, MSG_LEAVE_BODY_SIZE, 
            "8:1:2:8:", __sys_time(), (uint8_t)MSG_LEAVE, (uint16_t)0x1, ext_addr  );
    
    if(  MSG_LEAVE_BODY_SIZE == len  )
        send_to_interested( OBJOFFSET, &common_attrs, PORT, MSG_EVENT, buf, len );    
}

/*
#define     LED_PORT     2     
#define     RED_LED     0x80 
*/

void    msg_recv( msg_t msg )
{
    __label__   destroy_omsg, destroy_imsg;
    struct  msginfo minfo;

    if( IS_ERROR(msg_info(msg, &minfo)) )
        goto destroy_imsg;

    if( minfo.msg_type == MSG_REQ_BINDS ) {
        msg_t   omsg_id;
        uint16_t    ombody_size;

        //port_write( LED_PORT, RED_LED, PIN_HI );
        ombody_size = (&_end_bind - &_begin_bind)*sizeof(bind_t);
        omsg_id = msg_new( minfo.src_addr, minfo.src_port,
            MSG_RESP_BINDS, ombody_size, _MFLAG_NO_CONFIRM );        
        if( IS_ERROR(omsg_id) )
            goto destroy_imsg;
        if( IS_ERROR(msg_info(omsg_id, &minfo)) )
            goto destroy_omsg;
        memcopy( minfo.body_ptr, &_begin_bind, ombody_size );
        if( IS_OK(msg_send(omsg_id)) ) {
            goto destroy_imsg;
        }
destroy_omsg:
        msg_destroy( omsg_id );        
    }
destroy_imsg:
    msg_destroy( msg );
    return;    
}

void init_obj_0()
{
    /*
    port_attr_t   port_attr;
    PIN_SET( port_attr.dir, RED_LED, PIN_HI );  
    PIN_CLEAR( port_attr.sel, RED_LED);         
    PIN_CLEAR( port_attr.ie, RED_LED);          
    port_set_attr( LED_PORT, RED_LED, &port_attr );
    port_write( LED_PORT, RED_LED, PIN_LO );
    */
    common_attrs_init(&common_attrs);
}

INIT(init_obj_0);

