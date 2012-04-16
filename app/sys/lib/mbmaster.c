/*! @file   mbmaster.c
 *  @brief  Реализация мастера ModBus.
 *  @author Max Gekk
 *  @date   январь 2008
 *  @version 1
 *  */

#include    <syszig.h>
#include    <_zigzag.h>
#include    <modbus.h>
#include    <serialize.h>
#include    <platform.h>

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

#define BROADCAST_ADDR  0   /* Широковещательный адрес */
#define TURNAROUND_DELAY    100     /* миллисекунд. Таймаут между широковещательными фреймами */
#define ANSWER_TIMEOUT      1000    /* миллисекунд. Время подчинённому устройству на ответ мастеру. */

#define SLAVE_ADDR_SIZE     1       /* Размер поля адреса */
#define FUNC_CODE_SIZE      1       /* Размер поля с функциональным кодом */
#define CSUM_SIZE           2       /* Размер поля контрольной суммы */
/* Суммарный размер всех сервисных полей канального уровня ModBus */
#define MODBUS_DLFIELD_SIZE (SLAVE_ADDR_SIZE+FUNC_CODE_SIZE+CSUM_SIZE)

/* Флаги состояния модуля */
static  struct  {
    unsigned    busy        :1;     /* Флаг занятости */
    unsigned    wait_ans    :1;     /* 1 - ожидать ответ от подчинённого устройства, 0 - не ожидать */    
    unsigned    reserv      :6;
    uint8_t     slave_addr;         /* Адрес подчинённого узла, которому адресован запрос */
    uint8_t     *ans_fcode;         /* Функциональный код ответа */
    void        *ans_dptr;          /* Указатель на данные ответа */
    uint8_t     *ans_dsize;         /* Размер данных ответа */
} master;

/* Инициирование мастером обмена данными с подчинённым устройством */
result_t    mb_master_request( uint8_t slave_addr, 
    uint8_t req_fcode, void    *req_dptr,  uint8_t req_dsize,
    uint8_t *ans_fcode,  void   *ans_dptr,   uint8_t *ans_dsize )
{
    uint8_t     *txbuf, frame_size = 0;

    if( MB_BUF_SIZE<(req_dsize+MODBUS_DLFIELD_SIZE) )
        return  EINVAL;
    if( master.busy )
        return  EBUSY;  /* Мастер уже занят обменом */
    if( (uint8_t *)0 == (txbuf = mb_get_txbuf()) )
        return  EINVAL; /* Не удалось получить доступ к буферу отправки */
    if( req_dptr == 0 )
        req_dsize = 0;
    if( (ans_fcode == 0)||(ans_dptr == 0)||(ans_dsize == 0) )
        master.wait_ans = 0;    /* Не ждать ответа от подчинённого */
    else
        master.wait_ans = 1;
    /* Копирование в буфер отправки адреса подчинённого узла и функционального кода */
    frame_size = SLAVE_ADDR_SIZE+FUNC_CODE_SIZE;
    serialize( txbuf, frame_size, "1:1:", slave_addr, req_fcode );
    /* Копирование в буфер отправки тела запроса */
    {   uint16_t    i;
        for( i=0; i<req_dsize; i++ )
            txbuf[frame_size+i] = ((uint8_t *)req_dptr)[i];
        frame_size += req_dsize;
    }
    {/* Подсчёт контрольной суммы */
        uint16_t    csum;
        csum = mb_crc16( txbuf, frame_size );
        serialize( txbuf+frame_size, CSUM_SIZE, "2:", csum );
        frame_size += CSUM_SIZE;
    }
    master.slave_addr = slave_addr;
    master.ans_fcode = ans_fcode;
    master.ans_dptr = ans_dptr;
    master.ans_dsize = ans_dsize;
    {   /* Отправка данных */
        result_t    res = mb_tx( frame_size );
        if( IS_ERROR(res) )
            return res;
    }
    master.busy = 1;
    return  ENOERR;
}

/* Завершена процедура отправки данных подчинённому устройству */
void    mb_tx_done( result_t    result )
{
    if( !master.busy ) 
        return; /* Мастер не был занят отправкой */
    if( IS_ERROR(result) || (!master.wait_ans && (master.slave_addr != 0)) ) {
        /* Если возникла ошибка передачи или мастер не ожидает ответа от конкретного устройства, 
         * то известить об окончании процедуры обмена данными с подчинённым узлом */
        master.busy = 0;
        _event_emit( ALL_OBJOFFSET, PRIORITY_HIGH-1, EV_MBREQ_DONE, (unidata_t)result );
        return;    
    }
    if( master.slave_addr == 0 ) {
        /* Была широковещательная передача. Делаем намеренную паузу.*/
        //port_write( LED_PORT, BLUE_PIN, PIN_LO );
        mb_rx_enable( TURNAROUND_DELAY );
    } else {
        /* Переходим в режим приёма и ожидаем ответа */    
        //port_write( LED_PORT, GREEN_PIN, PIN_LO );
        mb_rx_enable( ANSWER_TIMEOUT );
    }
    return;
}

static uint16_t get_uint(uint8_t * buf)
{
   return ( (((uint16_t)*(buf + 1)) << 8) | *buf );
}


/* Обработка ответа от подчинённого узла */
void    mb_rx_notify( uint8_t   *buf, uint8_t   size )
{
    __label__ exit;
    result_t    res = ENOERR;
    uint8_t     slave_addr;
    static int i = 0;
//    port_write( LED_PORT, RED_PIN, ++i & 0x01 ? PIN_HI : PIN_LO);

    if( size == 0 ) {
        /* Истёк таймаут на ответ */
        res = ENOTFOUND;
        goto exit;
    }
    
    
    if( ( size<MODBUS_DLFIELD_SIZE )||((*master.ans_dsize+MODBUS_DLFIELD_SIZE) < size) ) {
        /* Размер принятых данных некорректен */
        res = EINVAL;
        goto exit; 
    }
    port_write( LED_PORT, RED_PIN, PIN_HI);
    /* Проверка целостности принятого фрейма */
    { uint16_t  crc16 = mb_crc16( buf, size-CSUM_SIZE );
      /* Предполагаем, что порядок байт на узле и во фрейме - LSB */
      //if( *(uint16_t *)&buf[size-CSUM_SIZE] != crc16 ) {
      if( get_uint(&buf[size-CSUM_SIZE]) != crc16 ) {
          res = EINVAL;
          res = size;
          //res = buf[1];
          *(master.ans_dsize) = size;
          for( i=0; i<size; i++ )
            ((uint8_t *)master.ans_dptr)[i] = buf[i];
          goto exit;
      }
    }
    
    port_write( LED_PORT, GREEN_PIN, PIN_HI);
    
    /* Распаковка адреса подчинённого устройства и функционального кода ответа */
    deserialize( buf, SLAVE_ADDR_SIZE+FUNC_CODE_SIZE, "1:1:", &slave_addr, master.ans_fcode );
    if( (slave_addr == 0)||(slave_addr != master.slave_addr) ) {
        /* Неправильный адрес источника. Можно рассматривать как попытку взлома */
        res = EACCESS;
        goto exit; 
    }
    *master.ans_dsize = size - MODBUS_DLFIELD_SIZE; /* Записываем реальный размер принятых данных */
    { /* Копируем из буфера приёма полезную нагрузку фрейма */  
        uint8_t i;
        for( i=0; i<*master.ans_dsize; i++ )
            ((uint8_t *)master.ans_dptr)[i] = buf[SLAVE_ADDR_SIZE+FUNC_CODE_SIZE+i];
    }
exit:   
    master.busy = 0;
    _event_emit( ALL_OBJOFFSET, PRIORITY_HIGH-1, EV_MBREQ_DONE, (unidata_t)res );
    return;
}

