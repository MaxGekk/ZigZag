#include<zigzag.h>


#define MAX_ZIGSWARM_PAYLOAD 80
#define MAX_CHUNK_SIZE (MAX_ZIGSWARM_PAYLOAD - 2)

#define DATA_MSG_TYPE 0x20
#define ACK_MSG_TYPE 0x21
/* 
 * Data msg format
 *
 * +-------------+--------------+------------+------+
 * | msg_seq_num | cur_chunk_no | last chunk | data |
 * +-------------+--------------+------------+------+
 * |      8      |       4      |      1     |  var |
 * +-------------+--------------+------------+------+
 */

/* 
 * ACK msg format
 *
 * +-------------+--------------+
 * | msg_seq_num | cur_chunk_no |
 * +-------------+--------------+
 * |      8      |       4      |
 * +-------------+--------------+
 */

uint16_t data_size;
uint16_t current_position;
uint8_t current_chunk;
uint8_t msg_seq_num;


enum {IDLE, AWAITING_ACK, AWAITING_RESPONSE}
state;


result_t tr_send(net_addr_t dst_addr, app_port_t dst_port, uint8_t* data, uint16_t length)
{
	if (state != IDLE)
		// ignoring all new requests unless have finished the last one.
		return EBUSY;

	data_size = length
	current_position = 0;
	current_chunk = 0;
	msg_seq_num++;

	send_next_chunk();
}

void send_next_chunk()
{
	msg_t msg;
	struct msginfo msg_inf;
	uint8_t size_to_send;
	uint8_t last_chunk;
	
	
	if (data_size - current_position <= MAX_CHUNK_SIZE)
	{
		last_chunk = 1;
		size_to_send = data_size - current_position;
		state = AWAITING_RESPONSE;
	}
	else
	{
		last_chunk = 0;
		size_to_send = MAX_CHUNK_SIZE;
		state = AWAITING_ACK;
	}

    msg = msg_new(DEST_ADDR, DEST_PORT, PORT, DATA_MSG_TYPE, size_to_send + 2, MFLAG_NO_CONFIRM);
	msg_info(msg, &msg_inf);
	
	// header
	msg_inf.body_ptr[0] = msg_seq_num;
	msg_inf.body_ptr[1] = (current_chunk << 4) | last_chunk;
	// data
	memcpy(msg_inf.body_ptr + 2, data + current_position, size_to_send);
	msg_send(msg);
	current_position += size_to_send;
}

size_t    msg_recv( msg_t   msg )
{
    uint8_t msg_size;
    struct msginfo info;

    if(msg_info(msg, &info) != ENOERR)
	{
		msg_destroy(msg);
        return 0;
	}

	if (state == IDLE)
	{
		msg_destroy(msg);
		return 0;
	}

    if (req_info.msg_type != DATA_MSG_TYPE)
	{
		msg_destroy(msg);
        return 0;
	}
    
    req_data = req_info.body_ptr;

    resp_msg = msg_new(req_info.src_addr, req_info.src_port, PORT, RS_RESPONSE_MSG_TYPE, /*Черти!!! мы еще не знаем длину ответа*/ MFLAG_NO_CONFIRM);

    mb_master_request( req_data[0], req_data[1], req_data + 3, req_data[2], )

    }
    
    return 0;
}
