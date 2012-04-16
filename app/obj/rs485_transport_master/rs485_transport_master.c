
#define PORT 11
#include <8004.h>
#include <modbus.h>

#define RS_REQUEST_MSG_TYPE 0x20
#define RS_RESPONSE_MSG_TYPE 0x21
#define RS_MAX_PACKET_SIZE 90

void    sys_init()
{
    // тут ничего нет
}


size_t    msg_recv( msg_t   msg )
{
    uint8_t msg_size;
    struct msginfo req_info;
    uint8_t * req_data;
    struct msginfo resp_info;
    msg_t resp_msg;
    uint8_t resp_data[MAX_PACKET_SIZE];
    uint8_t resp_size;


    if(msg_info(msg, &req_info) != ENOERR)
        return 0;
    if (req_info.msg_type != RS_REQUEST_MSG_TYPE)
        return 0;
    
    req_data = req_info.body_ptr;

    resp_data[0] = req_data[0]; // slave addr

    mb_master_request( req_data[0], req_data[1], req_data + 3, req_data[2], resp_data+1, resp_data+2, &resp_size);


    // wait for event

    resp_msg = msg_new(req_info.src_addr, req_info.src_port, PORT, RS_RESPONSE_MSG_TYPE, /*Черти!!! мы еще не знаем длину ответа*/ MFLAG_NO_CONFIRM);
        
    }
    
    return 0;
}

