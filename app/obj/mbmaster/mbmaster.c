/*! @file   mbmaster.c
 *  @brief  Реализация мастера ModBus
 *  @author Max Gekk
 *  @date   январь 2008
 *  @version 1
 *      Мастер циклически опрашивает все подчинённые узлы.
 */   

#define OBJ     3
#define PORT    16
#include    <zigzag.h>
#include    "../../../include/syszig.h"
#include    <modbus.h>
#include    <serialize.h>

#define     ZIGSWARM_HEAD_SIZE  3
#define     NET_ADDR_SIZE   2
#define     EXT_ADDR_SIZE   8
#define     SUM_ADDR_SIZE   (NET_ADDR_SIZE+EXT_ADDR_SIZE)
#define     MIN_ANSWER_SIZE    ( SUM_ADDR_SIZE+ZIGSWARM_HEAD_SIZE )

static const uint8_t  slave_addr[] = {1,2,3,4,5,6};
#define SLAVE_TOTAL   sizeof(slave_addr)

static uint8_t  cur_slave = 0;
uint8_t  ans_fcode;
uint8_t  ans_dptr[ MB_BUF_SIZE ];
uint8_t  ans_dsize = sizeof(ans_dptr);

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
    if( EV_MBREQ_DONE == event_type ) {
        result_t    res;
        if( (result_t)unidata == ENOERR ) {
            if( ( ans_fcode == MB_FCODE_ANSWER )&&( MIN_ANSWER_SIZE <= ans_dsize ) ) {
                uint16_t    net_addr;
                uint64_t    ext_addr;
                deserialize( ans_dptr, ans_dsize, "2:8:", &net_addr, &ext_addr);
                __net_recv( net_addr, ext_addr, 
                        ans_dsize-SUM_ADDR_SIZE, ans_dptr+SUM_ADDR_SIZE, 0xff );    
            }        
        }
        res = mb_master_request( slave_addr[cur_slave], MB_FCODE_REQUEST, 0,0, 
                &ans_fcode,ans_dptr,&ans_dsize );
        if( IS_OK(res) ) {
            cur_slave++;
            if( cur_slave >= SLAVE_TOTAL )
                cur_slave = 0;
            return;
        }
        event_emit( PRIORITY_HIGH, EV_MBREQ_DONE, (unidata_t)ENOERR );
    }
}

void    sys_init()
{
    event_emit( PRIORITY_HIGH, EV_MBREQ_DONE, (unidata_t)ENOERR );
    return;
}

