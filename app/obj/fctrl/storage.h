#ifndef _STORAGE_H_
#define _STORAGE_H_


#define STRG_CONFIG_POS     0
#define STRG_LOG_STATE_POS  2
#define STRG_PICT_START     3
#define STRG_NODE_CFG_POS   4096
#define STRG_MAX_NODE_NUM   1024
#define STRG_ZONE_CFG_POS   STRG_NODE_CFG_POS + STRG_MAX_NODE_NUM
#define STRG_MAX_ZONE_NUM   128

#define STRG_LOG_START      32768 // 16MB offset (in 512 byte sectors)

#define STRG_SUCCESS           0
#define STRG_MMC_ERROR        -1
#define STRG_CFG_CORRUPTED    -2
#define STRG_LOG_INIT_FAIL    -3
#define STRG_WRONG_ENTRY_TYPE -4
#define STRG_OUT_OF_STORAGE   -5
#define STRG_INV_IDX          -6
#define STRG_FAT_NOT_FOUND    -7
#define STRG_NOT_FOUND        -8

struct Config
{
   unsigned short crc16;
   unsigned short size;
   
   unsigned short log_state_pos;
   unsigned short pict_pos;

   unsigned short node_cfg_addr;
   unsigned short max_node_num;
   unsigned short node_num;
   unsigned short zone_cfg_addr;
   unsigned short max_zone_num;
   unsigned short zone_num;

   unsigned long  log_start_pos;
};

struct NodeCfg
{
   unsigned short     sign; // Signature
   unsigned long long ext_addr;
   unsigned short     zone;
   unsigned short     node_num;
   unsigned char      node_name[0];
};

struct ZoneCfg
{
   unsigned short     sign;
   unsigned short     zone;
   unsigned char      zone_name[0];
};

struct LogEntry
{
   unsigned long long timestamp;
   unsigned short     entry_type;
   unsigned short     msg_len;
   char               msg[0];
};

struct LogState
{
   unsigned long last_entry_pos;
   unsigned long entries_num;
};

int initStorage(void);
int initConfig(void);
int logAddEntry(struct LogEntry * pEntry, char * msg);
int logAddNwkMsgEntry(struct LogEntry * pEntry, struct msginfo * minfo, unsigned long long extaddr);
unsigned long logGetLastEntryPos(void);
unsigned long logGetPrevEntryPos(unsigned long pos);
unsigned long logGetNextEntryPos(unsigned long pos);
unsigned long logGetEntryPos(unsigned long num);
int logGetEntry(unsigned long pos, struct LogEntry * pEntry);
unsigned long long logGetEntryTimestamp(unsigned long pos);
int logGetEntryMsg(unsigned long pos, char * buf, unsigned short size);

int addNode(struct NodeCfg * node);
int changeNode(struct NodeCfg * node);
int getNode(unsigned long long addr, struct NodeCfg * cfg);
int getNodeByNum(unsigned short num, struct NodeCfg * cfg);
unsigned long logEntriesNum(void);
int strgGetZone(unsigned int zone, struct ZoneCfg * zcfg);
int strgGetZoneName(unsigned int zone, char * name, unsigned int size);
int strgGetNode(unsigned int node, struct NodeCfg * ncfg);
int strgGetNodeName(unsigned int node, char * name, unsigned int size);
unsigned int strgGetZonesNum(void);
unsigned int strgGetNodesNum(void);
int strgFindNode(unsigned long long ext_addr, struct NodeCfg * ncfg);
int strgFindZone(unsigned int zone, struct ZoneCfg * zcfg);
unsigned int strgMBGetSlavesNum(void);
int strgMBGetSlaveAddr(unsigned int pos);


#endif  // _STORAGE_H_

