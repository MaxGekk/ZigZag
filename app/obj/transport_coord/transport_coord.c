
#define PORT 11
#include <8006.h>
#include <modbus.h>


#define DEST_ADDR 0x0001
#define DEST_PORT 0x11

#define MAX_RS_PACKET_LENGTH 256
#define MAX_ZIGSWARM_PAYLOAD 80
#define MAX_CHUNK_SIZE (MAX_ZIGSWARM_PAYLOAD - 2)




void    sys_init()
{
    // тут ничего нет
}

uint8_t data[MAX_RS_PACKET_LENGTH];

void    mb_rx_notify( uint8_t   *buf, uint8_t   size )
{
	
	if (state != IDLE)
		// ignoring all new requests unless have finished the last one.
		return;

	if (size > MAX_RS_PACKET_LENGTH)
		return;

	memcpy(data, buf, size);
	tr_send(DEST_ADDR, DEST_PORT, data, size);
}


