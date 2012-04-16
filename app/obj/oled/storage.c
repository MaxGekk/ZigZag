#include <zzTypes.h>
#include "storage.h"
#include "mmc.h"
#include "fat16.h"

struct Config syscfg;
struct LogState logstat;
static unsigned long mmcSize;
static unsigned long mmcSectors;
static unsigned long strgMaxLogEntries;
static unsigned long strgCurEntryPos;
static unsigned long strgEntriesNum;

#ifdef _FAT16_H_
static const char *fnames[] = {"FIRECFG ", "LOGSTATE", "NODECFG ", "ZONECFG ", "RESERVED", "LOG     ",  0};
enum SysCfgFNames
{
   firecfgName  = 0,
   logstateName = 1,
   nodecfgName  = 2,
   zonecfgName  = 3,
   reservedName = 4,
   logName      = 5
};
#endif // _FAT16_H_


int logInit(void);

#ifdef _FAT16_H_

static unsigned long dataCluster2Sector(const unsigned int cluster)
{
   return fat.data_area + (unsigned long)(cluster - 2) * fat.sectors_per_cluster;
   //return fat.data_area + (unsigned long)(cluster - 2) * 4;
}

int initStorage(void)
{
   int err;
   FILEDESCR_t fd;
   unsigned long sector;
   
   mmcSize           = 0;
   mmcSectors        = 0;
   strgCurEntryPos   = 0;
   strgMaxLogEntries = 0;
   strgEntriesNum    = 0;

   memset(&syscfg, 0, sizeof(struct Config));
   //if((err = initMMC()) != MMC_SUCCESS)
   //   return err;
   if((err = initMMC()) != MMC_SUCCESS)
      return err; //STRG_MMC_ERROR;
   mmcSize = MMC_ReadCardSize();
   mmcSectors = mmcSize >> 9;
   init_FAT();
   if(!fat.root_dir)
   {
      memset(&syscfg, 0, sizeof(struct Config));
      return STRG_FAT_NOT_FOUND;
   }

   strcpy(fd.filename, fnames[firecfgName]);
   if(FindFile(fat.root_dir, &fd) == FAT_SUCCESS)
   {
      //sector = fat.data_area + (unsigned long)(fd.clust - 2) * 4; //fat.sectors_per_fat;
      sector = dataCluster2Sector(fd.clust);;
      err = mmcReadBlock(sector << 9, sizeof(struct Config), &syscfg);
      if(err != MMC_SUCCESS)
         return STRG_MMC_ERROR;
   }
   else
      return STRG_CFG_CORRUPTED;

   strcpy(fd.filename, fnames[logstateName]);
   if(FindFile(fat.root_dir, &fd) == FAT_SUCCESS)
      //syscfg.log_state_pos = fat.data_area + (unsigned long)(fd.clust - 2) * 4; //fat.sectors_per_fat;
      syscfg.log_state_pos = dataCluster2Sector(fd.clust);
   else
      return STRG_CFG_CORRUPTED;
   
   strcpy(fd.filename, fnames[nodecfgName]);
   if(FindFile(fat.root_dir, &fd) == FAT_SUCCESS)
   {
      //syscfg.node_cfg_addr = fat.data_area + (unsigned long)(fd.clust - 2) * 4; //fat.sectors_per_fat;
      syscfg.node_cfg_addr = dataCluster2Sector(fd.clust);
      //syscfg.max_node_num  = fd.size / sizeof(struct NodeCfg);
   }
   else
      return STRG_CFG_CORRUPTED;

   strcpy(fd.filename, fnames[zonecfgName]);
   if(FindFile(fat.root_dir, &fd) == FAT_SUCCESS)
   {
      //syscfg.zone_cfg_addr = fat.data_area + (unsigned long)(fd.clust - 2) * 4; //fat.sectors_per_fat;
      syscfg.zone_cfg_addr = dataCluster2Sector(fd.clust);
      //syscfg.max_zone_num  = fd.size / sizeof(struct ZoneCfg);
   }
   else
      return STRG_CFG_CORRUPTED;
   err = logInit();
   if(err != STRG_SUCCESS)
      mmcSize = 0;
   return err;
}

static int logInit(void)
{
   int err;
   unsigned long ul;
   FILEDESCR_t fd;

   /*
   strcpy(fd.filename, fnames[logstateName]);
   if(FindFile(fat.root_dir, &fd) != FAT_SUCCESS)
      return STRG_CFG_CORRUPTED;
   syscfg.log_state_pos = dataCluster2Sector(fd.clust);
   */

   err = mmcReadBlock((unsigned long)syscfg.log_state_pos << 9, sizeof(struct LogState), &logstat);
   if(err != MMC_SUCCESS)
      return STRG_MMC_ERROR;
   
   strcpy(fd.filename, fnames[logName]);
   if(FindFile(fat.root_dir, &fd) == FAT_SUCCESS)
   {
      //syscfg.log_start_pos = fat.data_area + (unsigned long)(fd.clust - 2) * 4; //fat.sectors_per_fat;
      syscfg.log_start_pos = dataCluster2Sector(fd.clust);
      strgMaxLogEntries    = fd.size >> 9;
      ul = (unsigned long)syscfg.log_start_pos + strgMaxLogEntries;
      if((logstat.last_entry_pos < syscfg.log_start_pos) || (logstat.last_entry_pos >= ul)) //((unsigned long)syscfg.log_start_pos + strgMaxLogEntries)))
      {
         logstat.last_entry_pos = syscfg.log_start_pos;
         logstat.entries_num    = 0;
      }
   }
   else
      return STRG_CFG_CORRUPTED;
   strgCurEntryPos = logstat.last_entry_pos;
   return STRG_SUCCESS;
}

int logAddEntry(struct LogEntry * pEntry, char * msg)
{
   int err;
   unsigned long entry_pos = strgCurEntryPos;
   if(logstat.entries_num)
   {
      if(entry_pos < (syscfg.log_start_pos + strgMaxLogEntries))
         entry_pos++;
      else
         entry_pos = syscfg.log_start_pos;
   }

   pEntry->entry_type = 0;
   pEntry->msg_len    = strlen(msg) + 1;
   if(pEntry->msg_len > 512)
      pEntry->msg_len = 512;
   mmc_buf_cpy(pEntry, sizeof(struct LogEntry), 0);
   mmc_buf_cpy(msg, pEntry->msg_len, sizeof(struct LogEntry));
   err = mmcWriteBuf(entry_pos);
   if(err != MMC_SUCCESS)
      return err;
   logstat.last_entry_pos = entry_pos;
   if(logstat.entries_num < strgMaxLogEntries)
      logstat.entries_num++;
//   logstat.entries_num %= strgMaxLogEntries;
   err = mmcWriteBlock((unsigned long)syscfg.log_state_pos << 9, 512, &logstat);
   if(err != MMC_SUCCESS)
      return err;
   strgCurEntryPos = entry_pos;
   return MMC_SUCCESS;
}

unsigned long logGetPrevEntryPos(unsigned long pos)
{
   /*
   if(pos == strgCurEntryPos)
      return pos;
   */
   if(pos <= syscfg.log_start_pos)
      return (syscfg.log_start_pos + strgMaxLogEntries - 1);
   return --pos;
}

unsigned long logGetNextEntryPos(unsigned long pos)
{
   if(pos >= (syscfg.log_start_pos + strgMaxLogEntries - 1))
      return syscfg.log_start_pos;
   if(pos == strgCurEntryPos)
      return pos;
   return pos++;
}

unsigned long logGetEntryPos(unsigned long num)
{
   if(num > strgMaxLogEntries)
      num %= strgMaxLogEntries;
   num = strgCurEntryPos - num;
   if(num < syscfg.log_start_pos)
      return ((strgMaxLogEntries - 1) + num);
   /*
   if(num < STRG_LOG_START)
      return mmcSectors - (STRG_LOG_START - num);
   */
   return num;
}

#else // _FAT16_H_

int initStorage(void)
{
   int err;
   mmcSize           = 0;
   mmcSectors        = 0;
   strgCurEntryPos   = 0;
   strgMaxLogEntries = 0;
   strgEntriesNum    = 0;

   //if((err = initMMC()) != MMC_SUCCESS)
   //   return err;
   if(initMMC() != MMC_SUCCESS)
      return STRG_MMC_ERROR;
   mmcSize = MMC_ReadCardSize();
   mmcSectors = mmcSize >> 9;
   err = mmcReadBlock(STRG_CONFIG_POS * 512, sizeof(struct Config), &syscfg);
   if(err != MMC_SUCCESS)
   {
      memset(&syscfg, 0, sizeof(struct Config));
      return STRG_MMC_ERROR;
   }

   if((syscfg.size < sizeof(struct Config)) || (syscfg.size > 1024))
   {
      memset(&syscfg, 0, sizeof(struct Config));
      return STRG_CFG_CORRUPTED;
   }

   err = logInit();
   if(err != STRG_SUCCESS)
   {
      mmcSize = 0;
      return err;
   }
   return STRG_SUCCESS;
}

static int logInit(void)
{
   int err;

   err = mmcReadBlock(STRG_LOG_STATE_POS * 512, sizeof(struct LogState), &logstat);
   if(err != MMC_SUCCESS)
      return STRG_MMC_ERROR;
   strgCurEntryPos = logstat.last_entry_pos;
   return STRG_SUCCESS;
}

int initConfig(void)
{
   int err;
   syscfg.size           = sizeof(struct Config);
   syscfg.log_state_pos  = STRG_LOG_STATE_POS;
   syscfg.pict_pos       = STRG_PICT_START;
   syscfg.node_cfg_addr  = STRG_NODE_CFG_POS;
   syscfg.max_node_num   = STRG_MAX_NODE_NUM;
   syscfg.node_num       = 0;
   syscfg.zone_cfg_addr  = STRG_ZONE_CFG_POS;
   syscfg.max_zone_num   = STRG_MAX_ZONE_NUM;
   syscfg.zone_num       = 0;
   syscfg.log_start_pos  = STRG_LOG_START;

   err = mmcWriteBlock(STRG_CONFIG_POS * 512, 512, &syscfg);
   if(err != MMC_SUCCESS)
      return err;
   logstat.last_entry_pos = strgCurEntryPos = STRG_LOG_START;
   logstat.entries_num    = 0;
   return mmcWriteBlock(STRG_LOG_STATE_POS * 512, 512, &logstat);
}

int logAddEntry(struct LogEntry * pEntry, char * msg)
{
   int err;
   unsigned long entry_pos = strgCurEntryPos;
   if(entry_pos < (/*mmcSize >> 9*/ mmcSectors - 1))
      entry_pos++;
   else
      entry_pos = STRG_LOG_START;

   pEntry->entry_type = 0;
   pEntry->msg_len    = strlen(msg) + 1;
   if(pEntry->msg_len > 512)
      pEntry->msg_len = 512;
   mmc_buf_cpy(pEntry, sizeof(struct LogEntry), 0);
   mmc_buf_cpy(msg, pEntry->msg_len, sizeof(struct LogEntry));
   err = mmcWriteBuf(entry_pos);
   if(err != MMC_SUCCESS)
      return err;
   logstat.last_entry_pos = entry_pos;
   logstat.entries_num++;
   logstat.entries_num %= /*mmcSize >> 9*/ mmcSectors - syscfg.log_start_pos - 1;
   err = mmcWriteBlock(syscfg.log_state_pos * 512, 512, &logstat);
   if(err != MMC_SUCCESS)
      return err;
   strgCurEntryPos = entry_pos;
   return MMC_SUCCESS;
}

unsigned long logGetPrevEntryPos(unsigned long pos)
{
   if(pos == strgCurEntryPos)
      return pos;
   if(pos <= STRG_LOG_START)
      return /*mmcSize >> 9*/mmcSectors - 1;
   return --pos;
}

unsigned long logGetNextEntryPos(unsigned long pos)
{
   if(pos >= (/*mmcSize >> 9*/mmcSectors - 1))
      return STRG_LOG_START;
   if(pos == strgCurEntryPos)
      return pos;
   return pos++;
}

unsigned long logGetEntryPos(unsigned long num)
{
   if(num > strgMaxLogEntries)
      num %= strgMaxLogEntries;
   num = strgCurEntryPos - num;
   if(num < STRG_LOG_START)
      return /*mmcSize >> 9*/mmcSectors - (STRG_LOG_START - num);
   return num;
}
#endif // _FAT16_H_

/*
int logScan(void)
{
   unsigned long long timestamp, prevtime;
   unsigned long entry_pos, sectors;
   int err;
   
   sectors = mmcSize >> 9;
   strgMaxLogEntries = sectors - syscfg.log_start_pos - 1;
   if(strgMaxLogEntries > sectors)
   {
      strgMaxLogEntries = 0;
      return STRG_LOG_INIT_FAIL;
   }
   prevtime = 0;
   strgCurEntryPos = syscfg.log_start_pos;
   for(entry_pos = strgCurEntryPos; entry_pos < sectors; entry_pos++)
   {
      err = mmcReadBlock(entry_pos << 9, sizeof(unsigned long long), &timestamp);
      if(err != MMC_SUCCESS)
         return STRG_LOG_INIT_FAIL;
      if(timestamp == 0xffffffffffffffffull)
         continue;
      strgEntriesNum++;
      if(timestamp < prevtime)
         continue;
      strgCurEntryPos = entry_pos;
   }
   return STRG_SUCCESS;
}
*/


/*
int logAddNwkMsgEntry(struct LogEntry * pEntry, struct msginfo * minfo, unsigned long long extaddr)
{
   int err;
   unsigned long entry_pos = strgCurEntryPos;
   if(entry_pos < (mmcSectors - 1))
      entry_pos++;
   else
      entry_pos = STRG_LOG_START;

   pEntry->entry_type = 1;
   pEntry->msg_len    = sizeof(struct msginfo) + minfo->body_size + sizeof(extaddr);
   if(pEntry->msg_len > 512)
      pEntry->msg_len = 512;
   mmc_buf_cpy(pEntry, sizeof(struct LogEntry), 0);
   mmc_buf_cpy(minfo, sizeof(struct msginfo), sizeof(struct LogEntry));
   mmc_buf_cpy(minfo->body_ptr, minfo->body_size, sizeof(struct LogEntry) + sizeof(struct LogEntry));
   mmc_buf_cpy(&extaddr, sizeof(extaddr), sizeof(struct LogEntry) + sizeof(struct LogEntry) + 2);
   err = mmcWriteBuf(entry_pos);
   if(err != MMC_SUCCESS)
      return err;
   logstat.last_entry_pos = entry_pos;
   logstat.entries_num++;
   logstat.entries_num %= mmcSectors - syscfg.log_start_pos - 1;
   err = mmcWriteBlock(syscfg.log_state_pos * 512, 512, &logstat);
   if(err != MMC_SUCCESS)
      return err;
   strgCurEntryPos = entry_pos;
   return MMC_SUCCESS;
}
*/

unsigned long logGetLastEntryPos(void)
{
   return strgCurEntryPos;
}

int logGetEntry(unsigned long pos, struct LogEntry * pEntry)
{
   return mmcReadBlock(pos << 9, sizeof(struct LogEntry), pEntry);
}

unsigned long long logGetEntryTimestamp(unsigned long pos)
{
   unsigned long long res;

   mmcReadBlock(pos << 9, sizeof(unsigned long long), &res);
   return res;
}

int logGetEntryMsg(unsigned long pos, char * buf, unsigned short size)
{
   struct LogEntry entry;
   int err, set_zero = 0;

   pos <<= 9; // Calculate address
   err = mmcReadBlock(pos, sizeof(struct LogEntry), &entry);
   if(err != MMC_SUCCESS)
      return STRG_MMC_ERROR;
   if(entry.entry_type)
      return STRG_WRONG_ENTRY_TYPE;
   if(!size)
      return (entry.msg_len > 512 - sizeof(struct LogEntry) ? 512 - sizeof(struct LogEntry) : entry.msg_len);
   if(size > entry.msg_len)
      size = entry.msg_len;
//   err = mmcReadBlock(pos + sizeof(struct LogEntry), size, buf);
   err = mmcReadBlock(pos + sizeof(struct LogEntry), size, buf);
   if(err != MMC_SUCCESS)
      return STRG_MMC_ERROR;
   buf[size - 1] = 0;
   return STRG_SUCCESS;
}

/*
int addNode(struct NodeCfg * node)
{
   unsigned short pos;
   unsigned short sign;
   int err;

   for(pos = syscfg.node_cfg_addr; pos < syscfg.node_cfg_addr + syscfg.max_node_num; pos++)
   {
      err = mmcReadBlock(pos << 9, sizeof(unsigned short), &sign);
      if(err != MMC_SUCCESS)
         return STRG_MMC_ERROR;
      if(sign != 0x444E)
         break;
   }
   if(pos == (syscfg.node_cfg_addr + syscfg.max_node_num))
      return STRG_OUT_OF_STORAGE;
   err = mmcWriteBlock(pos << 9, sizeof(struct NodeCfg), node);
   if(err != MMC_SUCCESS)
      return STRG_MMC_ERROR;
   return STRG_SUCCESS;
}
*/

unsigned long logEntriesNum(void)
{
   return logstat.entries_num;
}

int strgGetZone(unsigned int zone, struct ZoneCfg * zcfg)
{
   if(zone >= syscfg.zone_num)
      return STRG_INV_IDX;
   if(mmcReadBlock((unsigned long)(syscfg.zone_cfg_addr + zone) << 9, sizeof(struct ZoneCfg), zcfg) !=MMC_SUCCESS)
      return STRG_MMC_ERROR;
   return STRG_SUCCESS;
}

//int strgGetZoneName(unsigned int zone, char * name, unsigned int size)
int strgGetZoneName(unsigned int zone, char * name, unsigned int size, struct ZoneCfg * zcfg)
{
   unsigned int i;
   unsigned long addr;
   struct ZoneCfg z;
   
   *name = 0;
   if(zone >= syscfg.zone_num)
      return STRG_INV_IDX;
   if(!zcfg)
   {
      zcfg = &z;
      if(mmcReadBlock((unsigned long)(syscfg.zone_cfg_addr + zone) << 9, sizeof(struct ZoneCfg), zcfg) !=MMC_SUCCESS)
         return STRG_MMC_ERROR;
   }
   addr = ((unsigned long)(syscfg.zone_cfg_addr + zone) << 9) + sizeof(struct ZoneCfg);
   if(zcfg->name_len == -1)
   {
      size--;
      for(i = 0; i < size; i++, name++)
      {
         if((mmcReadBlock(addr, 1,  name) != MMC_SUCCESS) || (!(*name)))
            break;
         addr++;
      }
   }
   else
   {
      if(size > zcfg->name_len)
         size = zcfg->name_len;
      else
         size--;
      if(mmcReadBlock(addr, size, name) == MMC_SUCCESS)
      {
         name += size;
         i = size;
      }
      else
         i = 0;
   }
   *name = 0;
   return i;
}

int strgGetNode(unsigned int node, struct NodeCfg * ncfg)
{
   if(node >= syscfg.node_num)
      return STRG_INV_IDX;
   if(mmcReadBlock((unsigned long)(syscfg.node_cfg_addr + node) << 9, sizeof(struct NodeCfg), ncfg) !=MMC_SUCCESS)
      return STRG_MMC_ERROR;
   return STRG_SUCCESS;
}

//int strgGetNodeName(unsigned int node, char * name, unsigned int size)
int strgGetNodeName(unsigned int node, char * name, unsigned int size, struct NodeCfg * ncfg)
{
   unsigned int i;
   unsigned long addr;
   struct NodeCfg * n;
   
   *name = 0;
   if(node >= syscfg.node_num)
      return STRG_INV_IDX;
   if(!ncfg)
   {
      ncfg = &n;
      if(mmcReadBlock((unsigned long)(syscfg.node_cfg_addr + node) << 9, sizeof(struct NodeCfg), ncfg) !=MMC_SUCCESS)
         return STRG_MMC_ERROR;
   }
   addr = ((unsigned long)(syscfg.node_cfg_addr + node) << 9) + sizeof(struct NodeCfg);
   if(ncfg->name_len == -1)
   {
      size--;
      for(i = 0; i < size; i++, name++)
      {
         if((mmcReadBlock(addr, 1,  name) != MMC_SUCCESS) || (!(*name)))
            break;
         addr++;
      }
   }
   else
   {
      if(size > ncfg->name_len)
         size = ncfg->name_len;
      else
         size--;
      if(mmcReadBlock(addr, size,  name) == MMC_SUCCESS)
      {
         name += size;
         i = size;
      }
      else
         i = 0;
   }
   *name = 0;
   return i;
}

unsigned int strgGetZonesNum(void)
{
   return syscfg.zone_num;
}

unsigned int strgGetNodesNum(void)
{
   return syscfg.node_num;
}

int strgFindNode(unsigned long long ext_addr, struct NodeCfg * ncfg)
{
   unsigned int i;
   for(i = 0; i < strgGetNodesNum(); i++)
   {
      if(strgGetNode(i, ncfg) != STRG_SUCCESS)
         return STRG_MMC_ERROR;
      if(ncfg->ext_addr == ext_addr)
         return i;
   }
   return STRG_NOT_FOUND;
}

int strgFindZone(unsigned int zone, struct ZoneCfg * zcfg)
{
   unsigned int i;
   for(i = 0; i < strgGetZonesNum(); i++)
   {
      if(strgGetZone(i, zcfg) != STRG_SUCCESS)
         return STRG_MMC_ERROR;
      if(zcfg->zone == zone)
         return i;
   }
   return STRG_NOT_FOUND;
}

