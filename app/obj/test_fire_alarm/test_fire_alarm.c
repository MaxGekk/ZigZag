/*! @file   test_file_alarm.c
 *  @brief  Тестовый прикладной объект пожарного извещателя
 *  @author Истомин Тимофей
 *  @date   декабрь 2007
 *  @version 1
 *  */

//#define DBGLOG
#include <zzDebug.h>
#include <8001.h>

//#define OBJ 4
#define PORT    10

#include    <zigzag.h>
#include    <msp430.h>
#include    <_zzCommonAttrs.h>

#define INIT_EVENT 0

uint16_t isGT(uint8_t attr_num1, uint8_t attr_num2)
{
    int16_t val1, val2;
    attr_read(attr_num1, &val1);
    attr_read(attr_num2, &val2);

    return val1 > val2;
}


/* Инициализация прикладного объекта */
void    sys_init()
{

    event_emit(0, INIT_EVENT, 0);
    return;
}

void    stimer_fired( const uint8_t tnum )
{
    event_emit(0, INIT_EVENT, 0);
    return;
}

static uint16_t val = 0;


void    event_handler( event_type_t event_type, unidata_t   unidata )
{
    if (event_type == INIT_EVENT)
    {

        attr_write(DAY_TIME, &val);
        val += 1;
        stimer_set( 1 , 10000 );
    }
}

