/*! @file   mbslave.c
 *  @brief  Мост между сенсорной сетью и RS-485/ModBus
 *  @author Max Gekk
 *  @date   январь 2008
 *  @version 1
 *  */

/*
#define     LED_PORT     4       
#define     RED_PIN     0x1
#define     BLUE_PIN    0x10
#define     GREEN_PIN   0x8
*/
#define OBJ     3
#define PORT    1

#include    <zigzag.h>
#include    <msp430.h>
#include    <modbus.h>

#define     LED_PORT     6
#define     RED_PIN     0x1
#define     BLUE_PIN    0x4
#define     GREEN_PIN   0x8

#define INIT_EVT            1


#define     MODBUS_ADDR     0x4         /* Алрес на шине ModBus */
#define     MB_FCODE_REQUEST    100     /* Функциональный код запроса мастером имеющихся на слейве данных */
#define     MB_FCODE_ANSWER     101     /* Функциональный код ответа на запрос мастером данных */

/* Реализация кольцевого буфера для хранения дескрипторов, принятых из
 * сенсорной сети сообщений.
 * */
#define     RX_RINGBUF_SIZE     16  /* Размер кольцевого буфера */
static  msg_t   rx_rb[ RX_RINGBUF_SIZE ];
static  uint8_t getindex = 0;   /* Индекс самого старого элемента в буфере */
static  uint8_t putindex = 0;   /* Индекс свободной позиции для вставки */
static  uint8_t buffersize = 0; /* Число элементов в буфере */

/* Вставка элемента в кольцевой буфер */
static result_t    rb_put( msg_t m )
{
    if ( buffersize >= RX_RINGBUF_SIZE )
        return ENOMEM;  /* Буфер полон */
    rx_rb[ putindex ] = m;  /* Вставка элемента */
    /* Коррекция индекса свободной позиции */
    putindex++;
    if ( putindex >= RX_RINGBUF_SIZE )
        putindex = 0;
    /* Увеличение числа элементов в буфере */
    buffersize++;
    return ENOERR;
}

/* Извлечение элемента из кольцевого буфера */
static result_t    rb_get( msg_t   *mptr )
{
    if( 0 == buffersize )
        return ENOTFOUND;   /* Буфер пуст, извлекать нечего */
    if( (msg_t *)0 == mptr )
        return EINVAL;      /* Некорретный параметр */
    buffersize--;   /* Уменьшение числа элементов в буфере */
    *mptr = rx_rb[ getindex ];  /* Извлечение элемента */
    /* Корректировка индекса самого старого элемента в буфере */
    getindex++;
    if( getindex >= RX_RINGBUF_SIZE )
        getindex = 0;
    return ENOERR;
}

/* Обработка сообщения из сенсорной сети */
void    msg_recv( msg_t   msg )
{
    struct  msginfo minfo;

    if( IS_ERROR(msg_info(msg, &minfo)) )
        return 0;
    /* Помещаем дескриптор принятого сообщения в кольцевой буфер */
    if( IS_OK(rb_put( msg )) ) {
        //port_write( LED_PORT, BLUE_PIN, PIN_LO );
    }
}

/* Размер поля адреса на шине ModBus */
#define     MB_ADDR_SIZE  1
/* Размер поля с функциональным кодом */
#define     MB_FUNC_CODE_SIZE   1
/* Размер поля "контрольная сумма" */
#define     MB_CSUM_SIZE  2
/* Суммарный размер всех полей канального уровня ModBus */
#define     MB_DL_FIELD_SIZE    (MB_ADDR_SIZE+MB_FUNC_CODE_SIZE+MB_CSUM_SIZE)
/* Максимальный размер фрейма ModBus. Длина полей канального уровня ModBus +
 * размер сетевого адреса + макс. размер пакета ZigBee  */
#define     MB_MAX_FRAME_SIZE   (MB_DL_FIELD_SIZE+2+95)

static void set_uint(uint16_t val, uint8_t * buf)
{
   *buf       = (uint8_t)(val);
   *(buf + 1) = (uint8_t)(val >> 8);
}

void    mb_rx_notify( uint8_t   *buf, uint8_t   size )
{
    __label__   exit;


        //port_write( LED_PORT, GREEN_PIN, PIN_LO );

    /* Проверка допустимости длины фрейма */
    if( (size < MB_DL_FIELD_SIZE)||( MB_MAX_FRAME_SIZE < size ) )
        goto exit;
    /* Проверка целостности принятого фрейма */
    { uint16_t  crc16;
      uint16_t  packet_crc16;
      crc16 = mb_crc16( buf, size-MB_CSUM_SIZE );
      /* Предполагаем, что порядок байт на узле и во фрейме - LSB */
      packet_crc16 = (buf[size-1] << 8) | buf[size-2];
      if( packet_crc16 != crc16 ) 
          goto exit;
    }
    /* Проверяем, что пакет адресован нам */
    if( buf[0] != MODBUS_ADDR )
        goto exit;

    if( buf[1] == MB_FCODE_REQUEST ) {
        /* Обработка запроса имеющихся данных */
        __label__   rmmsg;
        /* Суммарный размер основных полей заголовка */
        enum { REQ_ANSWER_HEAD_SIZE = 16 };
        msg_t   msg;
        struct  msginfo minfo;
        uint8_t *txbuf,i;

        /* Получение указателя на буфер отправки */
        if( (uint8_t *)0 == (txbuf = mb_get_txbuf()) )
            goto exit;
        //port_write( LED_PORT, RED_PIN, PIN_LO );
        /* Извлечение самого старого дескриптора сообщения */
        if( IS_ERROR(rb_get(&msg)) )
            goto exit;
        //port_write( LED_PORT, GREEN_PIN, PIN_LO );

        if( IS_ERROR(msg_info(msg, &minfo)) )
            goto rmmsg;
        /* Упаковка основного заголовка ответа */
        i = serialize( txbuf, REQ_ANSWER_HEAD_SIZE, "1:1:2:8:1:1:1:1:", 
                (uint8_t)MODBUS_ADDR, (uint8_t)MB_FCODE_ANSWER, 
                minfo.src_addr, minfo.src_ext_addr, minfo.dst_port, minfo.src_port, minfo.msg_type, minfo.body_size);
        if( i != REQ_ANSWER_HEAD_SIZE ) goto rmmsg; /* Упаковка заголовка не удалась */
        /* Копируем запрошенные данные в буфер отправки */
        for( i=0; i<minfo.body_size; i++ )
            txbuf[i+REQ_ANSWER_HEAD_SIZE] = *((uint8_t *)minfo.body_ptr+i);
        /* Подсчёт контрольной суммы для отправляемого фрейма */
        { uint16_t  crc16;
          crc16 = mb_crc16( txbuf, REQ_ANSWER_HEAD_SIZE+minfo.body_size );
          /* Предполагаем, что порядок байт на узле и во фрейме - LSB */
          set_uint (crc16, &txbuf[ REQ_ANSWER_HEAD_SIZE+minfo.body_size ]);
          //*(uint16_t *)&txbuf[ REQ_ANSWER_HEAD_SIZE+minfo.body_size ] = crc16;
        }
        /* Отправка ответа по ModBus */
         mb_tx( REQ_ANSWER_HEAD_SIZE+minfo.body_size+MB_CSUM_SIZE );
rmmsg:
        msg_destroy(msg);      
    } else {
        msg_t msg;
        struct msginfo minfo;
        net_addr_t  dst_addr;
        app_port_t  dst_port;
        uint8_t     msg_body_size;
        uint8_t     msg_type, i;
        
        //port_write( LED_PORT, RED_PIN, PIN_LO );
        if( size < (MB_DL_FIELD_SIZE+6) )
            goto exit;
        deserialize( buf+MB_ADDR_SIZE+MB_FUNC_CODE_SIZE, 6, "2:1:1:1:1:", 
                &dst_addr, &dst_port,(void *)0, &msg_type, &msg_body_size);
        if (msg_body_size > size-MB_DL_FIELD_SIZE-6)
            goto exit;
        msg = msg_new(dst_addr, dst_port, msg_type, msg_body_size, MFLAG_NO_CONFIRM); 
        if( IS_ERROR(msg))
            goto exit;
        msg_info(msg, &minfo); /* не проверяем результат, так как сообщение было только что создано */
        for (i=0; i<size-MB_DL_FIELD_SIZE-5; i++)
            ((uint8_t*)minfo.body_ptr)[i] = buf[i+7];
        msg_send(msg);
    }
exit:
    mb_rx_enable(0);    
    return;
}

void    sys_init()
{
/*
    port_attr_t   port_attr;

    PIN_SET( port_attr.dir, (BLUE_PIN|GREEN_PIN|RED_PIN), PIN_HI );  
    PIN_CLEAR( port_attr.sel, BLUE_PIN|GREEN_PIN|RED_PIN );          
    port_set_attr( LED_PORT, BLUE_PIN|GREEN_PIN|RED_PIN, &port_attr );
    port_write( LED_PORT, BLUE_PIN|RED_PIN|GREEN_PIN, PIN_LO );
    port_write( LED_PORT, RED_PIN, PIN_HI );
    */

    getindex = 0;
    putindex = 0;
    buffersize = 0;
    //stimer_set(0, 1000);

    return;
}

void    stimer_fired( const uint8_t tnum )
{
    static int i = 0;
    stimer_set(0, 1000);
    /*    port_t p;
     *        port_read(LED_PORT, GREEN_PIN, &p);
     *        */
    port_write( LED_PORT, BLUE_PIN, ++i & 0x01 ? PIN_HI : PIN_LO);
    //    port_write( LED_PORT, GREEN_PIN, PIN_HI );
}

