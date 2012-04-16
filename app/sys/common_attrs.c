//#define DBGLOG
//#include <zzDebug.h>

#include <_zzCommonAttrs.h>
#include <_zzMsg.h>
#include <_zigzag.h>
#include <config.h>

void common_attrs_init(common_attr_t *ca)
{
    uint8_t i;
#ifdef DEFAULT_DST_PORT
    ca->interested[0].port = DEFAULT_DST_PORT;  // порт доставки сообщений по умолчанию
#else
    ca->interested[0].port = 1;  // порт доставки сообщений по умолчанию
#endif

#ifdef DEFAULT_DST_ADDR
    ca->interested[0].addr = DEFAULT_DST_ADDR;
#else
    ca->interested[0].addr = ROOT_ADDRESS;
#endif

    for (i=1; i<MAX_INTERESTED; i++)
    {
        ca->interested[i].addr = BAD_ADDRESS;
        ca->interested[i].port = 0;
    }
}


size_t common_attr_size(common_attr_t *ca, const uint8_t attr_num)
{
    switch( attr_num )
    {
        case ATTR_STATE:    return ATTR_STATE_SIZE;
        case ATTR_OBJ:  return ATTR_OBJ_SIZE;
        case ATTR_INTEREST_ADDR_FIRST...ATTR_INTEREST_ADDR_LAST:
                        return ATTR_INTEREST_ADDR_SIZE;
        case ATTR_INTEREST_PORT_FIRST...ATTR_INTEREST_PORT_LAST:
                        return ATTR_INTEREST_PORT_SIZE;
        case ATTR_THRESHOLD_REPORT_MODE:
                        return ATTR_THRESHOLD_REPORT_MODE_SIZE;
        default: return 0;
    }
}

size_t common_attr_set(common_attr_t *ca, const uint8_t attr_num, const void *const from)
{
    switch( attr_num )
    {
        case ATTR_STATE:    break;
        case ATTR_OBJ:      break;
        case ATTR_INTEREST_ADDR_FIRST...ATTR_INTEREST_ADDR_LAST:
            memcopy( &(ca->interested[ attr_num - ATTR_INTEREST_ADDR_FIRST ].addr), from, ATTR_INTEREST_ADDR_SIZE );
            break;
        case ATTR_INTEREST_PORT_FIRST...ATTR_INTEREST_PORT_LAST:
            memcopy( &(ca->interested[ attr_num - ATTR_INTEREST_PORT_FIRST ].port), from, ATTR_INTEREST_PORT_SIZE );
            break;
        case ATTR_THRESHOLD_REPORT_MODE:
            memcopy( &(ca->threshold_report_mode), from, ATTR_THRESHOLD_REPORT_MODE_SIZE );
            break;
        default:
            return 0;
    }
    return common_attr_size(ca, attr_num);
}

size_t common_attr_get(common_attr_t *ca, const uint8_t attr_num, void *const to)
{
    switch( attr_num )
    {
        case ATTR_STATE: {
            const uint8_t state = 0x1;  /* активное состояние */
            memcopy( to, &state, ATTR_STATE_SIZE );
            }
            return ATTR_STATE_SIZE;
        case ATTR_OBJ: {
            const uint16_t obj_num = ca->obj;
            memcopy( to, &obj_num, ATTR_OBJ_SIZE );
            }
            return ATTR_OBJ_SIZE;
        case ATTR_INTEREST_ADDR_FIRST...ATTR_INTEREST_ADDR_LAST:
            memcopy( to, &(ca->interested[ attr_num - ATTR_INTEREST_ADDR_FIRST ].addr),ATTR_INTEREST_ADDR_SIZE );
            return ATTR_INTEREST_ADDR_SIZE;
        case ATTR_INTEREST_PORT_FIRST...ATTR_INTEREST_PORT_LAST:
            memcopy( to, &(ca->interested[ attr_num - ATTR_INTEREST_PORT_FIRST ].port), ATTR_INTEREST_PORT_SIZE );
            return ATTR_INTEREST_PORT_SIZE;
        case ATTR_THRESHOLD_REPORT_MODE:
            memcopy( to, &(ca->threshold_report_mode), ATTR_THRESHOLD_REPORT_MODE_SIZE );
            return ATTR_THRESHOLD_REPORT_MODE_SIZE;
        default:
            break;
    }
    return 0;
}

result_t send_to_interested(const uint16_t objoffset, common_attr_t *ca, const uint8_t src_port, const uint8_t msg_type, const uint8_t *const buf, size_t length)
{
    uint8_t i;

    for( i = 0; i < MAX_INTERESTED; i++ )
        if( BAD_ADDRESS != ca->interested[i].addr ) {
            __label__   destroy_message;
            msg_t msg_id;
            struct msginfo minfo;

            msg_id = _msg_new( objoffset,
                    ca->interested[i].addr, ca->interested[i].port, src_port,
                    msg_type, length, _MFLAG_NO_CONFIRM
                   );
            if( IS_ERROR( msg_id ) ) goto destroy_message;

            if( IS_ERROR( _msg_info( objoffset, msg_id, &minfo ) ) )
                goto destroy_message;

            /* Формируем тело сообщения */
            memcopy(minfo.body_ptr, buf, length);
            
            //if (ca->interested[i].addr == 0 && ca->interested[i].port == 0)
            //    P2OUT |= 0x40;
            

            if( IS_ERROR( _msg_send( objoffset, msg_id )))
                goto destroy_message;
            continue;
    destroy_message:
            _msg_destroy( objoffset, msg_id );
            break;
        }

}

