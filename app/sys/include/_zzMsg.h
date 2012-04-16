#ifndef     __ZZ_MSG_H
#define     __ZZ_MSG_H

#include    <zzTypes.h>
#include    <_zzMacro.h>

#define     ROOT_ADDRESS    0x0000
#define     BAD_ADDRESS     0xFFFF

#define     MAX_PAYLOAD_SIZE    95

enum {

    MSG_ATTR_GET = 0x00,
    MSG_MULTI_ATTR_GET = 0x10,

    MSG_ATTR_SET = 0x01,
    MSG_MULTI_ATTR_SET = 0x11,

    MSG_ATTR_RET = 0x02,
    MSG_MULTI_ATTR_RET = 0x12,

    MSG_EVENT = 0x04,
    MSG_STORE_EVENT = 0x05,

    MSG_REQ_BINDS    = 0x30,
    MSG_RESP_BINDS   = 0x31    
};

#define _MFLAG_DEFAULT      0x00      /*!< Все флаги по умолчанию сброшены */
#define _MFLAG_NO_CONFIRM   0x10      /*!< Не извещать об окончании процедуры отправки сообщения */
#define _MFLAG_RAW          0x20      /*!< Сообщение уже сформировано. Заголовок уже есть в теле.*/

typedef struct {
    app_port_t  dst_port;
    app_port_t  src_port;
    uint8_t     type;
    uint8_t     body_size;
} __attribute__((packed)) msg_header_t;

#define MSG_HEADER_SIZE     sizeof(msg_header_t)

typedef void    (* msg_recv_ft )( msg_t     msg );

extern const msg_recv_ft    _begin_msg_recv;
extern const msg_recv_ft    _end_msg_recv;

typedef void    (* msg_send_done_ft)( msg_t, status_t );

extern const msg_send_done_ft   _begin_msg_send_done;
extern const msg_send_done_ft   _end_msg_send_done;

msg_t   _msg_new( const uint16_t    obj_offset, net_addr_t dst_addr, app_port_t    dst_port, 
        app_port_t  src_port, uint8_t msg_type, size_t  body_size, uint8_t flags );   

result_t   _msg_info( const uint16_t obj_offset, msg_t   msg,  struct  msginfo *info );    

result_t   _msg_destroy( const uint16_t obj_offset, msg_t   msg );

result_t   _msg_send( const uint16_t obj_offset, msg_t   msg );

#endif  /*  __ZZ_MSG_H    */

