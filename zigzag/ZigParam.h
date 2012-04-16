#ifndef   _ZIG_PARAM_H
#define   _ZIG_PARAM_H
#include    <types.h>
#include    <constants.h>

#define     MAX_ZIG_PACKET_SIZE ( MAC_AMAX_MAC_FRAME_SIZE - NWK_MIN_HEADER_OVERHEAD )

struct info_param_t{
    uint64_t    MAC_ADDRESS;
    uint32_t    Z_CHANNELS;
    uint16_t    Z_PAN_ID;
    uint8_t     Z_MAX_CHILDREN;
    uint8_t     Z_MAX_ROUTERS;
    uint8_t     Z_MAX_DEPTH;
    uint8_t     Z_BEACON_ORDER;
    uint8_t     Z_SUPERFRAME_ORDER;
    uint8_t     Z_SCAN_ORDER;
    uint8_t     ALIVE_SEND_PERIOD;
    uint8_t     ALIVE_SEND_ATTEMPTS;
} __attribute__((packed));

static struct info_param_t info_param __attribute__((section(".infomem"))) = {
    .MAC_ADDRESS = 1003,
    .Z_CHANNELS  = ( 1 << 15 ),
    .Z_PAN_ID    = 0xf3,
    .Z_MAX_CHILDREN = 4,
    .Z_MAX_ROUTERS = 4,
    .Z_MAX_DEPTH    = 7,
    .Z_BEACON_ORDER = 6,
    .Z_SUPERFRAME_ORDER = 1,
    .Z_SCAN_ORDER       = 6,
    .ALIVE_SEND_PERIOD  = 10,
    .ALIVE_SEND_ATTEMPTS = 3
};

#define MAC_AEXTENDED_ADDRESS   info_param.MAC_ADDRESS
#define Z_CHANNELS              info_param.Z_CHANNELS
#define Z_PAN_ID                info_param.Z_PAN_ID
#define Z_MAX_CHILDREN          info_param.Z_MAX_CHILDREN
#define Z_MAX_ROUTERS           info_param.Z_MAX_ROUTERS
#define Z_MAX_DEPTH             info_param.Z_MAX_DEPTH
#define Z_BEACON_ORDER          info_param.Z_BEACON_ORDER
#define Z_SUPERFRAME_ORDER      info_param.Z_SUPERFRAME_ORDER
#define Z_SCAN_ORDER            info_param.Z_SCAN_ORDER
#define ALIVE_SEND_PERIOD       info_param.ALIVE_SEND_PERIOD
#define ALIVE_SEND_ATTEMPTS     info_param.ALIVE_SEND_ATTEMPTS

#define DBG(str, ... )
#define ASSERT( condition, str, ... ) {condition;}
#define TEST_SUCCESS(condition, str, ...) ASSERT(((condition) == SUCCESS), str, ##__VA_ARGS__)
#define TEST_FAIL(condition, str, ...) ASSERT(((condition) == FAIL), str, ##__VA_ARGS__)

#define TEST_TRUE(condition, str, ...) ASSERT((condition), str, ##__VA_ARGS__)
#define TEST_FALSE(condition, str, ...) ASSERT(!(condition), str, ##__VA_ARGS__)

#endif   /* _ZIG_PARAM_H */

