//#define DBGLOG
#include <zzDebug.h>

#include    <_zigzag.h>


/* Получение размера атрибута.*/
size_t  _attr_size( uint16_t obj_offset, const uint8_t attr_num )
{
    const   _attr_size_ft  *as = &_begin_attr_size;
    as += obj_offset;
    DBG1("get attribute num=%hhx",attr_num);
    return (* as)( attr_num );
}

/* Установка атрибута. Функция возвращает размер атрибута  */
size_t  _attr_set( uint16_t obj_offset, const uint8_t attr_num, const void  *const from )
{
    const _attr_set_ft  *as = &_begin_attr_set;
    as += obj_offset;
    DBG1("set attribute num=%hhx",attr_num);
    return (* as)( attr_num, from );
}

/* Запроса атрибута. Функция возвращает размер атрибута. */
size_t  _attr_get( uint16_t obj_offset, const uint8_t attr_num, void  *const to )
{
    const _attr_get_ft  *ag = &_begin_attr_get;
    ag += obj_offset;
    return (* ag)( attr_num, to );
}

/* Обработка сообщений установки и запроса атрибутов */
size_t  _msg_attr_recv( uint16_t obj_offset, msg_t  imsg_id )
{
    size_t  imsg_size   =   0;      /* Размер обрабатываемого входящего сообщения */
    uint8_t itype;                  /* Тип входящего сообщения */
    uint8_t *ibody_begin;           /* Указатель на начала тела входящего сообщения */
    net_addr_t  dst_addr;           /* Адрес назначения для исходящего сообщения */
    app_port_t  dst_port;           /* Порт для исходящего сообщения */
    uint8_t *ibody_ptr;             /* Указатель тела входящего сообщения */
    uint8_t attr_total;             /* Общее число запрашиваемых или устанавливаемых атрибутов */
    uint8_t ombody_size;            /* Размер тела исходящего сообщения */
    msg_t   omsg_id;                /* Идентификатор исходящего сообщения */
    uint8_t *obody_ptr;             /* Указатель тела исходящего сообщения */

    {struct msginfo     iminfo;
     if( IS_ERROR( _msg_info( obj_offset, imsg_id, &iminfo ) ) )
         goto   destroy_imessage;
     itype = iminfo.msg_type;
     ibody_ptr = ibody_begin = (uint8_t *)iminfo.body_ptr;
     dst_addr = iminfo.src_addr, dst_port = iminfo.src_port;   
    }
    switch( itype ) {
        case  MSG_ATTR_GET: case MSG_ATTR_SET:
            attr_total = 1;
            break;
        case MSG_MULTI_ATTR_GET: case MSG_MULTI_ATTR_SET:
            attr_total = *ibody_ptr++;    /* Первый байт тела сообщения содержит общее число атрибутов */
            if( 0 == attr_total ) {
                imsg_size = MSG_HEADER_SIZE;
                goto    destroy_imessage;
            }
            break;
        default: goto destroy_imessage;
    }

    ombody_size = 9;    /* Атрибуты будем возвращать в сообщении типа MSG_MULTI_ATTR_RET, 
                           первое поле -- метка времени (8 байт)
                           второе поле -- количество передаваемых атрибутов (1 байт) */

    {uint8_t    attr_size, i;
     for( i = 0; i < attr_total; i++ )   {
        if( ( MSG_ATTR_SET == itype )||( MSG_MULTI_ATTR_SET == itype ) ) {
            attr_size = _attr_set( obj_offset, *ibody_ptr, ibody_ptr + 1 );
            ibody_ptr += 1 + attr_size;
        } else {
            attr_size = _attr_size( obj_offset, *ibody_ptr );
            ++ibody_ptr;
        }
        if( 0 == attr_size )      
            goto    destroy_imessage;
        ombody_size += 1 + attr_size;
     }
    }
    DBG("new msg");
    omsg_id = _msg_new( obj_offset, dst_addr, dst_port, offset2port( obj_offset),
            MSG_MULTI_ATTR_RET, ombody_size, _MFLAG_NO_CONFIRM );
    if( IS_ERROR( omsg_id ) )
        goto    destroy_imessage;
    {struct msginfo     ominfo;
     if( IS_ERROR( _msg_info( obj_offset, omsg_id, &ominfo ) ) )   
         goto   destroy_omessage;
     obody_ptr = (uint8_t *)ominfo.body_ptr;
    }

    serialize(obody_ptr, 8, "8:", (uint64_t)_sys_time());
    obody_ptr += 8;

    // запись кол-ва атрибутов
    *obody_ptr++ = attr_total;

    ibody_ptr = ibody_begin;
    if( ( MSG_MULTI_ATTR_GET == itype )||( MSG_MULTI_ATTR_SET == itype ) )
        ibody_ptr++;
    {uint8_t    attr_size, i;
     for( i = 0; i< attr_total; i++ ) {
         *obody_ptr = *ibody_ptr;   /* Копируем номер атрибута */
         attr_size = _attr_get( obj_offset, *obody_ptr, obody_ptr + 1 );
         obody_ptr += 1 + attr_size;
         ibody_ptr++;
         if( ( MSG_ATTR_SET == itype )||( MSG_MULTI_ATTR_SET == itype ) )
             ibody_ptr += attr_size;
     }
    }
    imsg_size = MSG_HEADER_SIZE + ( ibody_ptr - ibody_begin );
    if( IS_OK( _msg_send( obj_offset, omsg_id ) ) )
        goto    destroy_imessage;
destroy_omessage:
    _msg_destroy( obj_offset, omsg_id );
destroy_imessage:
    _msg_destroy( obj_offset, imsg_id );

    return imsg_size;
}

