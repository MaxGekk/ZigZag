//#define DBGLOG
#define OBJ     10 
#define PORT    1
#include    <zigzag.h>


size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num )
{
       return 0;
}
REG_ATTR_FUNC( attr_size );

size_t  ATTR_FUNC( attr_set )( const uint8_t attr_num, const void *const from )
{
       return 0;
}
REG_ATTR_FUNC( attr_set );

size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to )
{
       return 0;
}
REG_ATTR_FUNC( attr_get );

#include <modbus.h>

#include "oled.h"
#include "font.h"
#include "kbrd.h"
#include "oled_msgs.h"
#include "fire_msgs.h"
#include "storage.h"
#include "mmc.h"
#include "menu.h"
#include "fat16.h"
#include "fire_ctrl.h"
#include "fmt.h"

//#include "ipmce.h"
//
//
extern volatile uint8_t P2IFG asm("0x002B");  /* UART Receive Buffer */
extern volatile uint8_t P2IE  asm("0x002d");
//
#define LOW_PRIORITY        0
#define KBRD_EVT_PRIORITY   1

#define KBRD_EVT            0
#define INIT_EVT            1
#define DISPLAY_EVT         2
#define INIT_SEND_MSGS_EVT  3
#define INIT_MMC_EVT        4
#define INIT_OLED_EVT       5
#define INIT_STORAGE_EVT    6
#define INIT_USER_IO_EVT    7


#define MSG_TABLE_SIZE      10
#define MSG_VIEW_SIZE       5

#define MSG_JOIN            0x20
#define MSG_JOIN_MSG_SIZE   22
#define MSG_LEAVE           0x21
#define MSG_LEAVE_MSG_SIZE  22
#define MSG_THRESHOLDS      0x02
#define MSG_THRESHOLDS_SIZE 11


//#define SINGLE_LED_PORT

#ifndef SINGLE_LED_PORT
#define LED_PORT           2
#define LED_WHITE          0x10
#define LED_GREEN          0x40
//#define LED_GREEN          0x10
#define LED_BLUE           0x80

#define LED_PORT1          4
#define LED_RED            0x08

#define LED_POWER          0x01
#define LED_FIRE           0x02
#define LED_ALARM          0x04
#define LED_FAILURE        0x08
#endif // SINGLE_LED_PORT

#define FCTRL_MB_REQ_TIMER  1 
#define FCTRL_MB_REQ_PERIOD 1000


unsigned long total_msg_count;

void out_dump(unsigned int x, unsigned int y, unsigned int len, unsigned char * data, char * buf);
void init_LEDs(void);
void switchOnLED(unsigned int leds);
void switchOffLED(unsigned int leds);
void toggleLED(unsigned int leds);
void addEmergencyMsg(struct LogEntry * entry, char * msg);

//unsigned int msg_count;
//unsigned int last_msg_pos;
//struct msginfo msg_table[MSG_TABLE_SIZE];
/*
char message_table[MSG_TABLE_SIZE][64];
unsigned char scroll_pos;
*/
static struct child
{
   net_addr_t   net_addr;
   uint64_t     ext_addr;
   unsigned int node_state;
   unsigned int zone_num;
   unsigned int node_num;
}children[MAX_CHILDREN];// = {{BAD_ADDRESS,0}, {BAD_ADDRESS,0}, {BAD_ADDRESS,0}, {BAD_ADDRESS,0}, {BAD_ADDRESS,0}};

unsigned int zone_state[STRG_MAX_ZONE_NUM];
//unsigned int node_state[MAX_CHILDREN];

unsigned int sys_state;
static unsigned int sound_state;

port_attr_t LED_port_attr;
port_attr_t LED_port_attr1;

//struct LogEntry lastMsg;
//char lastMsgBuf[128];
struct EmergencyMessage lastMsg;

#define MINFO_LOCAL   0xffffffff
static struct msginfo minfo;

static size_t parse_message(msg_t msg)
{
   struct LogEntry entry;
   struct NodeCfg ncfg;
   struct ZoneCfg zcfg;
   char str[64];
   uint16_t i;
   size_t msg_size = 0;
   int err;

//   last_msg_pos = msg_table_get_next_idx(last_msg_pos);
   if((msg != MINFO_LOCAL) && (msg_info(msg, &minfo) != ENOERR))
   {
      //memcpy(&message_table[last_msg_pos], "Error retrieving msg info", 26);
      //memcopy(&message_table[last_msg_pos], "Error retrieving msg info", 26);
      //return;
      memcopy(str, "Error retrieving msg info", 26);
      goto finish_parse_msg;
   }
   str[0] = 0;
   switch(minfo.msg_type)
   {
   case 0x02:
      {
         unsigned int  thresholds;
         int           zone_idx, node_idx;
         unsigned char attr_num;
         unsigned char addMsg = 0;

         msg_size = deserialize(minfo.body_ptr, MSG_THRESHOLDS_SIZE, "8:1:2:", &entry.timestamp, &attr_num, &thresholds);
         msg_size = MSG_THRESHOLDS_SIZE + 3;
         entry.entry_type = 0;
         if((node_idx = strgFindNode(minfo.src_ext_addr, &ncfg)) < 0)
         {
            strcpy(str, FSTR_NODE_NOCFG); // "Node is not configured.");
            break;
         }
         if((zone_idx = strgFindZone(ncfg.zone, &zcfg)) < 0)
         {
            strcpy(str, FSTR_NODE_NOZONE); //"Node doesn't belong to any zone.");
            break;
         }
         if(zoneState(zone_idx, ZONE_STATE_ON) != ZONE_STATE_ON)
            break;

         //strcpy(str, "Zone: ");
         //strgGetZoneName(zone_idx, str + sizeof("Zone: ") - 1, 64 - sizeof("Zone: ") - 1);
         //itoa(zcfg.zone, str + sizeof("Zone: ") - 1, 10);
         /*
         itoa(zcfg.zone, str, 10);
         strcat(str, ", ");
         itoa(ncfg.node_num, str + strlen(str), 10);
         strcat(str, ", ");
         */
         format_str(str, 64, FSTR_ZONE_NODE_FMT, zcfg.zone, ncfg.node_num, minfo.src_ext_addr);
         //format_str(str, 64, "Z%u, N%u ", zcfg.zone, ncfg.node_num, minfo.src_ext_addr);

         /*
         itoa(attr_num, str, 16);
         strcat(str, ", ");
         itoa(thresholds, str + strlen(str), 16);

         strcat(str, ", ");
         */
         i = strlen(str);
         //if(attr_num == 0xe2)//0x26) // Thresholds for measured value
         if(attr_num == 0x26) // Thresholds for measured value
         {
            //setNodeState(node_idx, thresholds, NODE_MEASUREMENT);
            if(thresholds & (NODE_NO_FIRE_DANGER_DAY | NODE_NO_FIRE_DANGER_NIGHT))
            {
               setNodeState(node_idx, ~NODE_STATE_PREFIRE, NODE_STATE_PREFIRE);
               strcpy(str + i, FSTR_NODANGER); //"No danger." );
            }
            if(thresholds & (NODE_NO_FIRE_DAY | NODE_NO_FIRE_NIGHT))
            {
               setNodeState(node_idx, ~NODE_STATE_FIRE, NODE_STATE_FIRE);
               strcpy(str + i, FSTR_NOFIRE); //"No fire.");
            }
            if(thresholds & (NODE_FIRE_DANGER_DAY | NODE_FIRE_DANGER_NIGHT))
            {
               //setNodeState(node_idx, NODE_STATE_PREFIRE, NODE_STATE_PREFIRE);
               setNodeState(node_idx, ~NODE_STATE_PREFIRE, NODE_STATE_PREFIRE);
               strcpy(str + i, FSTR_FIREDANGER); //"Fire danger!");
               /*
               if(!systemState(SYS_STATE_MASK))
                  addMsg = 1;
               */
            }
            if(thresholds & (NODE_FIRE_DAY | NODE_FIRE_NIGHT))
            {
               setNodeState(node_idx, NODE_STATE_FIRE, NODE_STATE_FIRE);
               strcpy(str + i, FSTR_FIRE); //"Fire!");
               zoneSet(zone_idx, SYS_STATE_FIRE, SYS_STATE_FIRE);
               addMsg = 1;
            }
            compileZoneState(zone_idx, zcfg.zone);
            /*
            if(thresholds & NODE_FIRE_DANGER_NIGHT)
            {
               setNodeState(node_idx, NODE_STATE_PREFIRE, NODE_STATE_PREFIRE);
               strcpy(str + i, "Fire danger! (night)");
               if(!systemState(SYS_STATE_MASK))
                  addMsg = 1;
            }
            if(thresholds & NODE_NO_FIRE_DANGER_NIGHT)
            {
               setNodeState(node_idx, ~NODE_STATE_PREFIRE, NODE_STATE_PREFIRE);
               strcpy(str + i, "No danger. (night)" );
            }
            if(thresholds & NODE_FIRE_NIGHT)
            {
               setNodeState(node_idx, NODE_STATE_FIRE, NODE_STATE_FIRE);
               strcpy(str + i, "Fire! (night)");
               zoneSet(zone_idx, SYS_STATE_FIRE, SYS_STATE_FIRE);
               addMsg = 1;
            }
            if(thresholds & NODE_NO_FIRE_NIGHT)
            {
               setNodeState(node_idx, ~NODE_STATE_FIRE, NODE_STATE_FIRE);
               strcpy(str + i, "No fire. (night)");
            }
            */
         }
         //else if(attr_num == 0xe3)//0x4c)
         else if(attr_num == 0x4c)
         {
            if(thresholds & NODE_NO_PRECLEANING)
               strcpy(str + i, FSTR_NOPRECLEAN); //"Sensor doesn't require cleaning");
            if(thresholds & NODE_PRECLEANING)
               strcpy(str + i, FSTR_PRECLEAN); //"Sensor will require cleaning soon.");
            if(thresholds & NODE_OK)
               strcpy(str + i, FSTR_SENSOROK); //"Sensor OK");
            if(thresholds & NODE_PREFAILURE)
               strcpy(str + i, FSTR_PREFAILURE); //"Sensor may be inoperational");
            if(thresholds & NODE_CLEANING)
            {
               strcpy(str + i, FSTR_CLEAN_SENSOR); //"Clean the sensor!");
               zoneSet(zone_idx, SYS_STATE_FAILURE, SYS_STATE_FAILURE);
               if(!systemState(SYS_STATE_FIRE | SYS_STATE_ALARM))
                  addMsg = 1;
            }
            if(thresholds & NODE_FAILURE)
            {
               strcpy(str + i, FSTR_FAILURE); //"Sensor failure!");
               zoneSet(zone_idx, SYS_STATE_FAILURE, SYS_STATE_FAILURE);
               if(!systemState(SYS_STATE_FIRE | SYS_STATE_ALARM))
                  addMsg = 1;
            }
            if(!thresholds)
            {
               strcpy(str + i, FSTR_NOTHRESHOLDS); // "No thresholds");
               //addMsg = 1;
            }
         }
         else if(attr_num == 0xe2)
         {
            //if(minfo.src_ext_addr == 0x67ULL) // patch for testing; something has to be done to distiguish between different profile attributes
            // First attempt to distiguish between different profiles
            if(((minfo.src_ext_addr >= FBUT_FIRST_ADDR) && (minfo.src_ext_addr <= FBUT_FIRST_ADDR + FBUT_NUM)) || (minfo.src_ext_addr == 0x1010))
            {
               //format_str(str, 64, "Button: 0x%04X", thresholds);
               if(thresholds & 0x20)
               {
                  setNodeState(node_idx, ~NODE_STATE_FIRE, NODE_STATE_FIRE);
                  strcpy(str + i, FSTR_NOFIRE); //"No fire.");
                  //setNodeState(node_idx, ~NODE_STATE_PREFIRE, NODE_STATE_PREFIRE);
                  //strcpy(str + i, FSTR_NODANGER); //"No danger." );
               }
               else
               {
                  setNodeState(node_idx, NODE_STATE_FIRE, NODE_STATE_FIRE);
                  strcpy(str + i, FSTR_FIRE); //"Fire!");
                  zoneSet(zone_idx, SYS_STATE_FIRE, SYS_STATE_FIRE);
                  addMsg = 1;
               }
            }
            else if((minfo.src_ext_addr >= GERK_FIRST_ADDR) && (minfo.src_ext_addr <= GERK_FIRST_ADDR + GERK_SENSOR_NUM))
            {
               if(!(thresholds & 0x01))
               {
                  strcpy(str + i, FSTR_ALARM);
                  setNodeState(node_idx, NODE_STATE_ALARM, NODE_STATE_ALARM);
                  zoneSet(zone_idx, SYS_STATE_ALARM, SYS_STATE_ALARM);
                  if(!systemState(SYS_STATE_FIRE))
                     addMsg = 1;
               }
               else if(thresholds & 0x01)
               {
                  strcpy(str + i, FSTR_NOALARM);
                  setNodeState(node_idx, ~NODE_STATE_ALARM, NODE_STATE_ALARM);
                  compileZoneState(zone_idx, zcfg.zone);
               }
            }
            else if((minfo.src_ext_addr >= ABUT_FIRST_ADDR) && (minfo.src_ext_addr <= ABUT_FIRST_ADDR + ABUT_NUM))
            {
               if(!(thresholds & 0x20))
               {
                  strcpy(str + i, FSTR_ALARM);
                  setNodeState(node_idx, NODE_STATE_ALARM, NODE_STATE_ALARM);
                  zoneSet(zone_idx, SYS_STATE_ALARM, SYS_STATE_ALARM);
                  if(!systemState(SYS_STATE_FIRE))
                     addMsg = 1;
               }
               else if(thresholds & 0x20)
               {
                  strcpy(str + i, FSTR_NOALARM);
                  setNodeState(node_idx, ~NODE_STATE_ALARM, NODE_STATE_ALARM);
                  compileZoneState(zone_idx, zcfg.zone);
               }
            }
            else if((minfo.src_ext_addr >= USVD_FIRST_ADDR) && (minfo.src_ext_addr <= USVD_FIRST_ADDR + USVD_NUM))
            {
               format_str(str, 64, "USVD: 0x%04x", thresholds);
            }
            else
               format_str(str, 64, "?: 0x%04x", thresholds);
            compileZoneState(zone_idx, zcfg.zone);
         }
         else
         {
            //strcpy(str, "Attr num: ");
            //itoa(attr_num, str + strlen(str), 16);
            format_str(str, 64, "Attr num: 0x%x, val: 0x%x", attr_num, thresholds);
            //addMsg = 1;
         }
         compileSysState();
         entry.msg_len = strlen(str);
         if(addMsg)
            addEmergencyMsg(&entry, str);
      }
      break;

   case MSG_EVENT:
      {
         net_addr_t    node_addr;
         //uint64_t      timestamp;
         uint64_t      node_extaddr;
         unsigned char type;
         
         // Both messages (JOIN and LEAVE are supposed to have same overall size and field sizes and sequence (though they may
         // have different meaning). If it's not true anymore or another event with
         // different body size is supported correct the next line as required
         msg_size = deserialize(minfo.body_ptr, MSG_JOIN_MSG_SIZE, "8:1:2:8:", &entry.timestamp, &type, &node_addr, &node_extaddr);
         
         //if(((uint8_t *)minfo.body_ptr)[8] == MSG_JOIN)
         if(type == MSG_JOIN)
         {
            /*
            msg_size = MSG_JOIN_MSG_SIZE;
            node_addr   = ((uint8_t *)minfo.body_ptr)[10];
            node_addr <<= 8;
            node_addr  |= ((uint8_t *)minfo.body_ptr)[9];
            //sprintf(str, MSG_STR_ADD_NODE, node_addr);
            //strcpy(str, "+Node(0x");
            //itoa(node_addr, str + strlen(str), 16);
            //strcat(str, ", 0x");
            for(i = 10 + 8; i > 10; i--)
            {
               //sprintf(str + strlen(str), "%02X", (uint16_t)(((uint8_t *)minfo.body_ptr)[i]));
               //itoa((uint16_t)(((uint8_t *)minfo.body_ptr)[i]), str + strlen(str), 16);
               node_extaddr <<= 8;
               node_extaddr  |= ((uint8_t *)minfo.body_ptr)[i];
            }
            */
            //sprintf(str + strlen(str), ")");
            //strcat(str, ")");
            //format_str(str, 64, "Join: 0x%llx 0x%04x", node_extaddr, node_addr);
            //OLED_puts(0, 50, 0xff, font6x9, str);
            if((err = strgFindNode(node_extaddr, &ncfg)) >= 0)
            {
               //format_str(str, 64, "+Node(0x%x, 0x%llx) (N%u)", node_addr, node_extaddr, ncfg.node_num);
               if(err < MAX_CHILDREN)
               {
                  //format_str(str, 64, FSTR_NODE_ADD_FMT, node_addr, node_extaddr, ncfg.node_num);
                  format_str(str, 64, FSTR_NODE_ADD1_FMT, ncfg.zone, ncfg.node_num);
                  children[err].net_addr   = node_addr;
                  children[err].ext_addr   = node_extaddr;
                  children[err].node_state = 0;
                  children[err].zone_num   = ncfg.zone;
                  children[err].node_num   = ncfg.node_num;
               }
               else
                  format_str(str, 64, FSTR_NODE_ATCAP_FMT, node_addr, node_extaddr);
                  //format_str(str, 64, "Fail: +Node(0x%x, 0x%llx) - at capacity", node_addr, node_extaddr);
               //i = strlen(str);
               //format_str(str + i, sizeof(str) - i, " (n%u)", ncfg.node_num);
               //strcat(str, " (n");
               //itoa(ncfg.node_num, str + strlen(str), 10);
               //strcat(str, ")");
            }
            else
               format_str(str, 64, FSTR_NODE_NOTFOUND_FMT, node_addr, node_extaddr);
            update_disp_nwk();
            update_disp_zones_cfg();
               //format_str(str, 64, "Fail: +Node(0x%x, 0x%llx) - not found", node_addr, node_extaddr);
            /*
            for(i = 0; i < MAX_CHILDREN; i++)
            {
               if((children[i].net_addr == BAD_ADDRESS) || (children[i].ext_addr == node_extaddr))
               {
                  children[i].net_addr = node_addr;
                  children[i].ext_addr = node_extaddr;
                  break;
               }
            }
            if(i == MAX_CHILDREN)
            {
               //sprintf(str, "+Node failed: at capacity");
               strcpy(str, "+Node failed: at capacity");
            }
            else
            {
               if(strgFindNode(node_extaddr, &ncfg) >= 0)
               {
                     //sprintf(str + strlen(str), " (n%u)", zones[i].node_num);
                  strcat(str, " (n");
                  itoa(ncfg.node_num, str + strlen(str), 10);
                  strcat(str, ")");
               }
            }
            */
            //OLED_puts(0, 50, 0xff, font6x9, str); // for debugging

         }
         else if(type == MSG_LEAVE)
         {
            /*
            net_addr_t node_addr;
            uint64_t   node_extaddr = 0;
            
            msg_size = MSG_LEAVE_MSG_SIZE;
            node_addr   = ((uint8_t *)minfo.body_ptr)[10];
            node_addr <<= 8;
            node_addr  |= ((uint8_t *)minfo.body_ptr)[9];
            //sprintf(str, MSG_STR_DEL_NODE, node_addr);
            //strcpy(str, "-Node(0x");
            // Now the reason of disconnection is placed instead of short address
            // So we'll not print the short address at all
            // itoa(node_addr, str + strlen(str), 16);
            // strcat(str, ", 0x");
            for(i = 10 + 8; i > 10; i--)
            {
               //sprintf(str + strlen(str), "%02X", (uint16_t)(((uint8_t *)minfo.body_ptr)[i]));
               //itoa((uint16_t)(((uint8_t *)minfo.body_ptr)[i]), str + strlen(str), 16);
               node_extaddr <<= 8;
               node_extaddr  |= ((uint8_t *)minfo.body_ptr)[i];
            }
            */
            if((err = strgFindNode(node_extaddr, &ncfg)) >= 0)
            {
               if(err < MAX_CHILDREN)
               {
                  //format_str(str, 64, "-Node(0x%x, 0x%llx) (N%u)", children[err].net_addr, node_extaddr, ncfg.node_num);
                  //format_str(str, 64,FSTR_NODE_DEL_FMT, children[err].net_addr, node_extaddr, ncfg.node_num);
                  format_str(str, 64,FSTR_NODE_DEL1_FMT, ncfg.zone, ncfg.node_num);
                  children[err].net_addr = BAD_ADDRESS;
               }
               else
                  format_str(str, 64, FSTR_NODE_DEL_FMT1, node_extaddr);
                  //format_str(str, 64, "-Node(0x%llx)", node_extaddr);
               //strcat(str, " (n");
               //ultoa((unsigned long)ncfg.node_num, str + strlen(str), 10);
               //strcat(str, ")");
            }
            update_disp_nwk();
            update_disp_zones_cfg();
            //sprintf(str + strlen(str), ")");
            //strcat(str, ")");
            /*
            for(i = 0; i < MAX_CHILDREN; i++)
            {
               if(children[i].ext_addr == node_extaddr)
                  children[i].net_addr = BAD_ADDRESS;
            }
            
            if(strgFindNode(node_extaddr, &ncfg) >= 0)
            {
               strcat(str, " (n");
               ultoa((unsigned long)ncfg.node_num, str + strlen(str), 10);
               strcat(str, ")");
            }
            */
         }
         else
         {
            format_str(str, 64, "MsgEvt %u", type);
            //strcpy(str, "Msg evt ");
            //itoa(((uint8_t *)minfo.body_ptr)[8], str + 8, 10);
         }
      }
      break;
   default:
      //sprintf(str, "Msg type %u", minfo.msg_type);
      strcpy(str, "Msg type ");
      itoa(minfo.msg_type, str + 9, 10);
   }

finish_parse_msg:
   if(*str)
   {
      logAddEntry(&entry, str);
      on_msg_add();
      //memcpy(&message_table[last_msg_pos], str, strlen(str) + 1);
      //memcopy(&message_table[last_msg_pos], str, strlen(str) + 1);
      //if(msg_count < MSG_TABLE_SIZE)
      //   msg_count++;
   }

   return msg_size;
}

static unsigned char mb_slave_ans[MB_BUF_SIZE];
static unsigned char mb_ans_code;
static unsigned char mb_ans_size = MB_BUF_SIZE;

int requestSlaves(void)
{
   int res;
   static unsigned int cur_slave = 0;
   char str[32];
   static int err_cnt = 0;
   static int success_cnt = 0;

   if(((res = strgMBGetSlaveAddr(cur_slave)) == STRG_INV_IDX))
      res = strgMBGetSlaveAddr((cur_slave = 0));

//   format_str(str, 32, "Slave: 0x%02x", res);
//   OLED_puts(0, 41 - 9, 0xff, font6x9, str);

   mb_ans_size = MB_BUF_SIZE;
   res = mb_master_request(res, MB_FCODE_REQUEST, NULL, 0, &mb_ans_code, mb_slave_ans, &mb_ans_size);
   if(res == ENOERR)
   {
     cur_slave++;
     //count = 0;
     format_str(str, 32, "rS: %d", ++success_cnt);
     OLED_puts(256 - strlen(str) * 6, 50, 0xff, font6x9, str);
   }
   else
   {
      format_str(str, 32, "rS err: %d (%d)", res, ++err_cnt);
      OLED_puts(0, 50, 0xff, font6x9, str);
   }

   return res;
}

result_t set_stimer(uint8_t tnum, uint16_t period)
{
/* uint32_t tval;
   tval = atimer_counter() + period;
   return atimer_set(tnum, tval);
   */
   return stimer_set(tnum, period);
}

uint32_t get_atimer_counter(void)
{
   return atimer_counter();
}

result_t set_atimer( const uint8_t tnum, const uint32_t tpoint)
{
   //return atimer_set(tnum, tpoint);
   return atimer_set(0, atimer_counter());
}

extern struct Config syscfg;
void    event_handler( event_type_t event_type, unidata_t   unidata ) 
{
   char str[32];
   unsigned char ch;
   static int err;
   
   switch(event_type)
   {
   case EV_MBREQ_DONE:
      if((result_t)unidata == ENOERR)
      {
         //format_str(str, 32, "MB answer: %3u", (unsigned int)mb_ans_code);
         //OLED_puts(0, 41 - 9, 0xff, font6x9, str);
         //if(mb_ans_code == 101)
         if(mb_ans_code == MB_FCODE_ANSWER)
         {
            //OLED_puts(14 * 6, 41 - 9, 0xff, font6x9, " Modbus answer");
            err = deserialize(mb_slave_ans, mb_ans_size, "2:8:1:1:1:1:", &minfo.src_addr, &minfo.src_ext_addr, &minfo.dst_port, 
               &minfo.src_port, &minfo.msg_type, &minfo.body_size);
            //format_str(str, 32, "; Deserialize err: %d %d", err, (unsigned int)mb_ans_size);
            //OLED_puts(14 * 6, 41 - 9, 0xff, font6x9, str);
            if(err == (mb_ans_size - 14))
            {
               minfo.body_ptr = &mb_slave_ans[14]; // ZigBee packet encapsulated in Modbus packet
               parse_message(MINFO_LOCAL);
            }
         }
      }
      else
      {
         //format_str(str, 32, "MB Error: %5d", (result_t)unidata );
         //OLED_puts(0, 41, 0xff, font6x9, str);
         if((result_t)unidata > 0)
         {
            OLED_clrLine(41 - 9 * 2, 41);
            format_str(str, 32, "Size: %d", (result_t)mb_ans_size );
            OLED_puts(0, 41 - 9 * 2, 0xff, font6x9, str);

            for(err = 0; err < mb_ans_size; err++)
            {
               format_str(str, 32, "%02X", mb_slave_ans[err]);
               //sprintf(str, "%02X", mb_slave_ans[err]);
               OLED_puts(err * 12, 41 - 9, 0xff, font6x9, str);
            }
         }
      }
      break;
   case KBRD_EVT:
     
      if(kbrd_pressed)
      {
         kbrd_pressed = 0;
         ch = kbrd_getch();
         if(!(ch & 0x80))
         {
            ch = active.menu->action(ch);
            active.menu->display(ch);
            update_statusbar();
         }
      }
      /*
      else
         OLED_puts(0, 41, 0xff, font6x9, "Key not pressed");
         */
      break;

   case INIT_EVT:
      //OLED_init();
      //init_kbrd();
      SPI_init();
      event_emit(LOW_PRIORITY, INIT_MMC_EVT, 0);
      break;

   case INIT_MMC_EVT:
      err = initMMC();
      err = initStorage();
      event_emit(LOW_PRIORITY, INIT_OLED_EVT, 0);
      break;

   case INIT_OLED_EVT:
      OLED_init();
      OLED_puts(0, 0 , 0xff, font6x9, "System initialization...");
      OLED_puts(0, 1 * 9 , 0xff, font6x9, "SPI     init'ed.");
      OLED_puts(0, 2 * 9 , 0xff, font6x9, "MMC     init'ed.");
      OLED_puts(0, 3 * 9 , 0xff, font6x9, "OLED    init'ed.");
      event_emit(LOW_PRIORITY, INIT_STORAGE_EVT, 0);
      break;

   case INIT_STORAGE_EVT:
      //err = initStorage();
      //OLED_puts(0, 4 * 9 , 0xff, font6x9, err ? "Storage init failed!" : "Storage init'ed.");
      format_str(str, 32, "Storage err: %d", err);
      OLED_puts(0, 4 * 9 , 0xff, font6x9, err ? str : "Storage init'ed.");
      /*
      format_str(str, 32, "ZoneCfg pos: %u", syscfg.zone_cfg_addr);
      OLED_puts(0, 5 * 9 , 0xff, font6x9, str);
      err = 1;
      */
      event_emit(LOW_PRIORITY, INIT_USER_IO_EVT, 0);
      break;

   case INIT_USER_IO_EVT:
      init_kbrd();
      menu_init(err ? MNU_INIT | MNU_DO_NOT_SHOW : MNU_INIT);
      //set_sys_time((uint64_t)20000);
      stimer_set(FCTRL_MB_REQ_TIMER, FCTRL_MB_REQ_PERIOD);
      break;
   }
   /*
   else
   {
      //sprintf(str, "Evt: %u", (uint16_t)event_type);
      //OLED_puts(0, 0, 0xff, font6x9, str);
   }
   */
}

void    msg_send_done( msg_t   msg, status_t    status )
{
//   char str[64];
//   sprintf(str, "Msg sent: %u, %u", msg, status);
//   OLED_puts(0, 0, 0xff, font6x9, str);
   msg_destroy(msg);
}

//size_t    msg_recv( msg_t   msg ) // Return type has changed in new version
void    msg_recv( msg_t   msg )
{
   size_t msg_size;
   
//   OLED_puts(0, 41, 0xff, font6x9, "Message");
   msg_size = parse_message(msg);
   total_msg_count++;
   event_emit(LOW_PRIORITY, DISPLAY_EVT, 2);
 //  toggleLED(LED_RED);
   msg_destroy(msg);
//   toggleLED(LED_GREEN);
      
//   return msg_size;
//   return (sizeof(struct msginfo) + minfo.body_size);
}

port_t p4_val;
extern const struct OLED_bitmap ipmce_bmp;
extern unsigned char mmc_error;
extern struct LogState logstat;

void    sys_init()
{
//   char str[32];
//   unsigned long ul;
//   struct LogEntry entry;
   struct NodeCfg ncfg;
   unsigned long long _ull;
   
   FILEDESCR_t fd;
   int err;
   static struct child c = {BAD_ADDRESS, 0};
   
//   msg_count       = 0;
//   last_msg_pos    = 0;
   total_msg_count = 0;

   for(err = MAX_CHILDREN; err--; )
      memcopy(&children[err], &c, sizeof(struct child));
   //memset(zone_state, 0, sizeof(zone_state));
   for(err = STRG_MAX_ZONE_NUM; err--;)
      zone_state[err] = ZONE_STATE_ON;

   init_LEDs();
   setNormalState();
   //switchOnLED(LED_RED);
//   scroll_pos      = 0;
//   port_attr_t p4;

//   event_emit(0, INIT_EVT, 0);

   /*
   PIN_SET(p4.dir, 0x19, PIN_HI);
   PIN_CLEAR(p4.sel, 0x19);
   port_set_attr(4, 0x19, &p4);
   port_write(4, 0x19, PIN_HI);
   p4_val = 0x01;
   */
  
//   SPI_init();
//   err = initStorage();
//   OLED_init();
//   init_kbrd();
//   menu_init(0);
//   OLED_clr(68, 3*9, 2 * 6, 9);
//   err = mmcReadBlock(23, 3, &ul);

   //err = logEntriesNum();
   //err = logAddEntry(&entry, "Test message3");
   //err = strgFindNode(0x66ULL, &ncfg);
   //itoa(err, fd.filename, 10);
   //OLED_puts(0, 0, 0xff, font6x9, fd.filename);
   //out_dump (0, 50, sizeof(struct NodeCfg), &ncfg);
   /*
   strcpy(str, "Pos: ");
   ul = logstat.last_entry_pos;
   //ul = (unsigned long)syscfg.log_state_pos << 9;
   ultoa(ul, str + strlen(str), 10);
   str[strlen(str) + 1] = 0;
   str[strlen(str)]     = ' ';
   ul = syscfg.log_state_pos;
   ultoa(ul, str + strlen(str), 10);

   OLED_puts(0, 0, 0xff, font6x9, str);
   */

//   init_FAT();
//   Read_PBR();
//   ultoa(fat.root_dir, fd.filename, 16);
//   OLED_puts(17 * 6, 50, 0xff, font6x9, fd.filename);
/*
   strcpy(fd.filename, "LOG     ");
   if(FindFile(fat.root_dir, &fd) != FAT_SUCCESS)
      OLED_puts(0, 50, 0xff, font6x9, "FindFile failed");
   else
   {
      itoa(fd.clust, fd.filename, 16);
      OLED_puts(0, 50, 0xff, font6x9, fd.filename);
      ultoa(fd.size, fd.filename, 16);
      OLED_puts(10 * 6, 50, 0xff, font6x9, fd.filename);
   }
*/

   event_emit(LOW_PRIORITY, INIT_EVT, 0);

//   stimer_set(1, 1000);
//
               
    return;
}

void    atimer_fired( uint8_t    tnum )
{
   atimer_stop(tnum);
   //if(tnum == KBRD_SOUND_TIMER)
   //if(tnum == 0)
   {
      port_write(KBRD_SOUND_PORT, KBRD_SOUND_PIN, PIN_LO);
   }
}

void    irq_handler( irq_t  irq )
{
   if(irq == IRQ_PORT2)
   {
      if(P2IFG & 0x0f)
      {
         kbrd_service();
         //event_emit(KBRD_EVT_PRIORITY + 1, KBRD_EVT, 0);
//         port_write(4, 0x01, ++p4_val);
      }
   }
}

void    stimer_fired( const uint8_t tnum ) 
{
   static unsigned int i = 0;
   if(tnum == KBRD_ANTIRING_TIMER)
   {
      kbrd_clear_ring_buf();
      P2IE  &= ~0x0f; // Disable interrupts
//      port_write(4, 0x01, PIN_LO);
      kbrd_service();
//      port_write(4, 0x01, PIN_HI);
      P2IE  |=  0x0f; // Enable interrupts
      setSound(SYS_SOUND_OFF);
      if(kbrd_pressed)
         event_emit(KBRD_EVT_PRIORITY, KBRD_EVT, 0);
   }
   else if(tnum == KBRD_SOUND_TIMER)
   {
      if(sound_state == SYS_SOUND_ALARM)
      {
         stimer_set(KBRD_SOUND_TIMER, SOUND_ALARM_PERIOD);
         if(isSoundOn())
            soundOff();
         else
            soundOn();
      }
   }
   else
   {
      stimer_set(FCTRL_MB_REQ_TIMER, FCTRL_MB_REQ_PERIOD);
//      requestSlaves();
      update_time();
      //toggleLED(LED_FIRE | LED_POWER);
   }
}


int isNodeConnected(unsigned int node)
{
   /*
   unsigned int i;
   struct NodeCfg ncfg;
   if(strgGetNode(node, &ncfg) != STRG_SUCCESS)
      return 0;

   for(i = 0; i < MAX_CHILDREN; i++)
   {
      if(children[i].ext_addr == ncfg.ext_addr)
      {
         if(children[i].net_addr == BAD_ADDRESS)
            return 0;
         else
            return 1;
      }
   }
   */
   if(node < MAX_CHILDREN)
   {
      if(children[node].net_addr != BAD_ADDRESS)
         return 1;
   }
   return 0;
}

void out_dump(unsigned int x, unsigned int y, unsigned int len, unsigned char * data, char * buf)
{
   char str[4];
   unsigned int i;
   char * ptr = buf ? buf : str;
   
   i = 0;
   while(len--)
   {
      ptr[i++] = bin2sym(*data >> 4);
      ptr[i++] = bin2sym(*data & 0x0f);
      ptr[i] = 0;
      if(!buf)
      {
         OLED_puts(x, y, 0xff, font6x9, ptr);
         x += 2 * 6;
         i = 0;
      }
      else
         buf += 2;
      data++;
   }
}

unsigned int getFirstChildPosition(void)
{
   unsigned int i;
   for(i = 0; i < MAX_CHILDREN; i++)
      if(children[i].net_addr != BAD_ADDRESS)
         return i;
   return -1;
}

unsigned int getLastChildPosition(void)
{
   unsigned int i, pos;
   for(i = MAX_CHILDREN, pos = -1; i--; )
   {
      if(children[i].net_addr != BAD_ADDRESS)
      {
         pos = i;
         break;
      }
   }
   return pos;
}

unsigned long long getNextChildExtAddress(unsigned int * pos)
{
   unsigned int found = 0;
   unsigned long long res = -1;
   /*
   if(*pos >= MAX_CHILDREN)
      return res;
      */
//   do
   while((*pos) < MAX_CHILDREN)
   {
      if(children[*pos].net_addr != BAD_ADDRESS)
      {
         if(!found)
         {
            res = children[*pos].ext_addr;
            found = 1;
         }
         else
            return res;
      }
      (*pos)++;
   }
//   }while(++(*pos) < MAX_CHILDREN);
   *pos = -1;
   return res;
}

unsigned long long getPrevChildExtAddress(unsigned int * pos)
{
   unsigned int found = 0;
   unsigned long long res = -1;
   if(*pos >= MAX_CHILDREN)
      return res;
   do
   {
      if(children[*pos].net_addr != BAD_ADDRESS)
      {
         if(!found)
         {
            res = children[*pos].ext_addr;
            found = 1;
         }
         else
            return res;
      }
   }while((*pos)--);
   return res;
}

void init_LEDs(void)
{
#ifndef SINGLE_LED_PORT
   PIN_SET(LED_port_attr1.dir, LED_RED, PIN_HI);
   PIN_CLEAR(LED_port_attr1.sel, LED_RED);
   port_set_attr(LED_PORT1, LED_RED, &LED_port_attr1);
   port_write(LED_PORT1, LED_RED, PIN_LO);
   
   PIN_SET(LED_port_attr.dir, LED_GREEN | LED_BLUE | LED_WHITE, PIN_HI);
   PIN_CLEAR(LED_port_attr.sel, LED_GREEN | LED_BLUE | LED_WHITE);
   port_set_attr(LED_PORT, LED_GREEN | LED_BLUE | LED_WHITE, &LED_port_attr);
   port_write(LED_PORT, LED_GREEN | LED_WHITE, PIN_LO);
   port_write(LED_PORT, LED_BLUE, PIN_HI);
   //port_write(LED_PORT, LED_BLUE, PIN_LO);
#else // SINGLE_LED_PORT
   PIN_SET(LED_port_attr.dir, LED_POWER | LED_FIRE | LED_ALARM | LED_FAILURE, PIN_HI);
   PIN_CLEAR(LED_port_attr.sel, LED_POWER | LED_FIRE | LED_ALARM | LED_FAILURE);
   port_set_attr(LED_PORT,LED_POWER | LED_FIRE | LED_ALARM | LED_FAILURE, &LED_port_attr);
   port_write(LED_PORT, LED_FIRE | LED_ALARM | LED_FAILURE, PIN_LO);
   port_write(LED_PORT, LED_POWER, PIN_HI);
#endif // SINGLE_LED_PORT
}

void switchOnLED(unsigned int leds)
{
#ifndef SINGLE_LED_PORT
   if(leds & LED_POWER)
      port_write(LED_PORT, LED_BLUE, PIN_HI);
   if(leds & LED_FIRE)
      port_write(LED_PORT1, LED_RED, PIN_HI);
   if(leds & LED_ALARM)
      port_write(LED_PORT, LED_WHITE, PIN_HI);
   if(leds & LED_FAILURE)
      port_write(LED_PORT, LED_GREEN, PIN_HI);

#else // SINGLE_LED_PORT
   port_write(LED_PORT, (unsigned char)leds, PIN_HI);
#endif // SINGLE_LED_PORT
}

void switchOffLED(unsigned int leds)
{
#ifndef SINGLE_LED_PORT
   if(leds & LED_POWER)
      port_write(LED_PORT, LED_BLUE, PIN_LO);
   if(leds & LED_FIRE)
      port_write(LED_PORT1, LED_RED, PIN_LO);
   if(leds & LED_ALARM)
      port_write(LED_PORT, LED_WHITE, PIN_LO);
   if(leds & LED_FAILURE)
      port_write(LED_PORT, LED_GREEN, PIN_LO);
#else // SINGLE_LED_PORT
   port_write(LED_PORT, (unsigned char)leds, PIN_LO);
#endif // SINGLE_LED_PORT
}

static void soundOn(void)
{
   port_write(KBRD_SOUND_PORT, KBRD_SOUND_PIN, PIN_HI);
}

static void soundOff(void)
{
   port_write(KBRD_SOUND_PORT, KBRD_SOUND_PIN, PIN_LO);
}

int isSoundOn(void)
{
   port_t sound_pin = 0;
   port_read(KBRD_SOUND_PORT, KBRD_SOUND_PIN, &sound_pin);
   return (sound_pin ? 1 : 0);
}

void setSound(unsigned int type)
{
   sound_state = type;
   if(type == SYS_SOUND_OFF)
      soundOff();
   else
   {
      if(type == SYS_SOUND_ALARM)
         stimer_set(KBRD_SOUND_TIMER, SOUND_ALARM_PERIOD);
      soundOn();
   }
}

void toggleLED(unsigned int leds)
{
   port_t LEDport;
   port_read(LED_PORT, leds, &LEDport);
   LEDport ^= (unsigned char)leds;
   port_write(LED_PORT, (unsigned char)leds &   LEDport,  PIN_HI);
   port_write(LED_PORT, (unsigned char)leds & (~LEDport), PIN_LO);
}

void setNormalState(void)
{
   sys_state &= ~SYS_STATE_MASK;
   switchOffLED(LED_FIRE | LED_ALARM | LED_FAILURE);
   setSound(SYS_SOUND_OFF);
}

void setFireState(void)
{
   sys_state |= SYS_STATE_FIRE;
   switchOnLED(LED_FIRE);
   setSound(SYS_SOUND_FIRE);
}

void clearFireState(void)
{
   sys_state &= ~SYS_STATE_FIRE;
   switchOffLED(LED_FIRE);
   if(sys_state & SYS_STATE_ALARM)
      setSound(SYS_SOUND_ALARM);
   else
      setSound(SYS_SOUND_OFF);
}

void setFailureState(void)
{
   sys_state |= SYS_STATE_FAILURE;
   switchOnLED(LED_FAILURE);
}

void clearFailureState(void)
{
   sys_state &= ~SYS_STATE_FAILURE;
   switchOffLED(LED_FAILURE);
}

void setAlarmState(void)
{
   sys_state |= SYS_STATE_ALARM;
   switchOnLED(LED_ALARM);
   if(!(sys_state & SYS_STATE_FIRE))
      setSound(SYS_SOUND_ALARM);
}

void clearAlarmState(void)
{
   sys_state &= ~SYS_STATE_ALARM;
   switchOffLED(LED_ALARM);
   if(!(sys_state & SYS_STATE_FIRE))
      setSound(SYS_SOUND_OFF);
}

unsigned int systemState(unsigned int mask)
{
   return sys_state & mask;
}

unsigned int zoneState(unsigned int zone, unsigned int mask)
{
   return zone_state[zone] & mask;
}

int zoneSet(unsigned int zone, unsigned int state, unsigned int mask)
{
   if(zone > STRG_MAX_ZONE_NUM)
      return STRG_INV_IDX;
   zone_state[zone] |= state &  mask;
   zone_state[zone] &= state | ~mask;
   return 0;
}

//int setNodeState(unsigned int node, unsigned int state, unsigned int what)
int setNodeState(unsigned int node, unsigned int state, unsigned int mask)
{
   if(node > MAX_CHILDREN)
      return STRG_INV_IDX;
   children[node].node_state |= state &  mask;
   children[node].node_state &= state | ~mask;
   /*
   if(what == NODE_MEASUREMENT)
      children[node].node_state = (children[node].node_state & 0xf0) | (state & 0x0f);
   else if(what == NODE_STATE_MON)
      children[node].node_state = (children[node].node_state & 0x0f) | (state << 8);
   */
   return 0;
}

int isNodeInFire(unsigned int node)
{
   return (children[node].node_state & NODE_STATE_FIRE);
}

int isNodeAlarmed(unsigned int node)
{
   return (children[node].node_state & NODE_STATE_ALARM);
}

int compileZoneState(const unsigned int zone_idx, const unsigned int zone)
{
   unsigned int i, state = 0;
   for(i = MAX_CHILDREN; i--;)
   {
      if(children[i].zone_num == zone)
      {
         if(isNodeInFire(i))
            state |= SYS_STATE_FIRE;
         if(isNodeAlarmed(i))
            state |= SYS_STATE_ALARM;
      }
   }
   zoneSet(zone_idx, state, SYS_STATE_FIRE | SYS_STATE_ALARM);
   return 0;
}

unsigned int compileSysState(void)
{
   unsigned int i, state = 0;

   for(i = 0; i < STRG_MAX_ZONE_NUM; i++)
   {
      if(!(zone_state[i] & ZONE_STATE_ON))
         continue;
      if(zone_state[i] & SYS_STATE_MASK)
         state |= zone_state[i] & SYS_STATE_MASK;
   }

   if(state & SYS_STATE_FIRE)
      setFireState();
   else
      clearFireState();
   if(state & SYS_STATE_ALARM)
      setAlarmState();
   else
      clearAlarmState();
   if(state & SYS_STATE_FAILURE)
      setFailureState();
   else
      clearFailureState();
}

void addEmergencyMsg(struct LogEntry * entry, char * msg)
{
   memcpy(&lastMsg.msg_data, entry, sizeof(struct LogEntry));
   strcpy(lastMsg.msg, msg);
   show_last_msg();
}

