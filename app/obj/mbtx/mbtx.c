/*! @file   mbtx.c
 *  @brief  Реализация мастера ModBus
 *  @author Max Gekk
 *  @date   январь 2008
 *  @version 1
 *      Мастер циклически опрашивает все подчинённые узлы.
 */   

#define OBJ     3
#define PORT    17
#include    <zigzag.h>
#include    <modbus.h>

static const uint8_t  slave_addr[] = {1,1,2,3,5,8};
#define SLAVE_TOTAL   sizeof(slave_addr)
static uint8_t  cur_slave = 0;
uint8_t  ans_fcode;
uint8_t  ans_dptr[10];
uint8_t  ans_dsize = sizeof(ans_dptr);

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
    if( EV_MBREQ_DONE == event_type ) {
        result_t    res;
        res = mb_master_request( slave_addr[cur_slave],
                100, 0,0, &ans_fcode,ans_dptr,&ans_dsize );
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
