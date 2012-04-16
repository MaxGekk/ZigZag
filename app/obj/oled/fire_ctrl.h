#ifndef _FIRE_CTRL_H_
#define _FIRE_CTRL_H_

#define MAX_CHILDREN       40

#define ZONE_STATE_ON      0x01
#define IS_ZONE_ON(zone)   (!(zone & ZONE_STATE_ON))
#define ZONE_SET_ON(zone)  (zone & ~ZONE_STATE_ON)
#define ZONE_SET_OFF(zone) (zone |  ZONE_STATE_ON)

#define SYS_STATE_POWER    0x10
#define SYS_STATE_FIRE     0x20
#define SYS_STATE_ALARM    0x40
#define SYS_STATE_FAILURE  0x80

#define SYS_STATE_MASK     (SYS_STATE_POWER | SYS_STATE_FIRE | SYS_STATE_ALARM | SYS_STATE_FAILURE)

#define NODE_MEASUREMENT   0x01
#define NODE_STATE_MON     0x02

// Thresholds definitions
// For measurement
#define NODE_FIRE_DANGER_DAY       0x01
#define NODE_NO_FIRE_DANGER_DAY    0x02
#define NODE_FIRE_DAY              0x04
#define NODE_NO_FIRE_DAY           0x08
#define NODE_FIRE_DANGER_NIGHT     0x10
#define NODE_NO_FIRE_DANGER_NIGHT  0x20
#define NODE_FIRE_NIGHT            0x40
#define NODE_NO_FIRE_NIGHT         0x80
// For sensor state
#define NODE_PRECLEANING           0x01
#define NODE_NO_PRECLEANING        0x0a
#define NODE_CLEANING              0x04
#define NODE_PREFAILURE            0x10
#define NODE_OK                    0xa0
#define NODE_FAILURE               0x40

#define NODE_STATE_FIRE            0x01
#define NODE_STATE_PREFIRE         0x02
#define NODE_STATE_CLEANING        0x04
#define NODE_STATE_PRECLEANING     0x08
#define NODE_STATE_FAILURE         0x10
#define NODE_STATE_PREFAILURE      0x20
#define NODE_STATE_ALARM           0x40
#define NODE_STATE_OK              0x00
#define NODE_STATE_NOT_OK          (NODE_STATE_FIRE | NODE_STATE_PREFIRE | NODE_STATE_CLEANING | NODE_STATE_PRECLEANING)

struct EmergencyMessage
{
   struct LogEntry msg_data;
   char   msg[128];
};

extern struct EmergencyMessage lastMsg;

void setNormalState(void);
void setFireState(void);
void clearFireState(void);

void out_dump(unsigned int x, unsigned int y, unsigned int len, unsigned char * data, char * buf);
unsigned int getFirstChildPosition(void);
unsigned int getLastChildPosition(void);
unsigned long long getNextChildExtAddress(unsigned int * pos);
unsigned long long getPrevChildExtAddress(unsigned int * pos);
unsigned int systemState(unsigned int mask);
unsigned int zoneState(unsigned int zone, unsigned int mask);
int zoneSet(unsigned int zone, unsigned int state, unsigned int mask);
unsigned int compileSysState(void);
int compileZoneState(const unsigned int zone_idx, const unsigned int zone);

#endif // _FIRE_CTRL_H_
