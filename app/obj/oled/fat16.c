#include "fat16.h"
#include "mmc.h"

extern unsigned char mmc_error;

FAT_t fat;

void init_FAT(void)
{
   memset(&fat, 0, sizeof(FAT_t));
   Read_PBR();
}

void Read_PBR (void)
{
      unsigned int reserved_sectors;
      unsigned char num_copies_of_fat;
      unsigned char * buffer;
//      unsigned char tmp_buf[512];
//      unsigned long res = 0xff;
      
//      mmcReadSector (fat.partition_start, buffer);

//      res = mmcReadBuf(0);

//      mmc_error = 100;
      if(mmcReadBuf(0) != MMC_SUCCESS)
         return;
//      mmc_error = 101;
      buffer = mmcGetBuffer();
/*
      if(mmcReadBlock(0, 512, tmp_buf) != MMC_SUCCESS)
         return;
      buffer = tmp_buf;
      */
      if((buffer[510] == 0x55) && (buffer[511] == 0xAA));
      {
         union {unsigned int word; char byte[2];} WB;
         /*
         int i;

      mmc_error = 102;
     
         for(i = 0, mmc_error = 0; i <512; i++, mmc_error++)
         {
            if(buffer[i])
               break;
         }
      */
         WB.byte[0] = buffer[11];
         WB.byte[1] = buffer[12];
         fat.bytes_per_sector = WB.word;
         fat.sectors_per_cluster = buffer[13];
         reserved_sectors = *((unsigned int *) &buffer[14]);
         num_copies_of_fat = buffer[16];
         WB.byte[0] = buffer[17];
         WB.byte[1] = buffer[18];
         fat.max_root_dir_entries = WB.word;;
         fat.sectors_per_fat = *((unsigned int *) &buffer[22]);
         //fat.sectors_in_partition = *((unsigned long *) &buffer[32]);
         
         fat.fat_table = fat.partition_start + reserved_sectors;
         fat.root_dir = fat.fat_table + (fat.sectors_per_fat * num_copies_of_fat);
         fat.data_area = fat.root_dir + fat.max_root_dir_entries * 32 /fat.bytes_per_sector;
      }

//      fat.root_dir = fat.data_area;
      
      return;
}

int FindFile(unsigned long sector, FILEDESCR_t * file)
{
   unsigned char * buffer;
   unsigned char i, f;
   unsigned char init_i = 0;
   unsigned int attribute;
   unsigned int first_char;
   
   do
   {
      mmcReadBuf(sector);
      buffer = mmcGetBuffer();
      for(i = init_i; i < 16; i++)
      {
         attribute = 32 * i + 11;
         first_char = 32 * i;
         if ((buffer[attribute] != 0x0f) && (buffer[attribute] != 0x00) && 
             (buffer[first_char] != 0xe5) && (buffer[first_char] != 0x00) && (buffer[first_char] != 0x05))
         {
            if (((buffer[attribute] & 0x10) != 0x10) && ((buffer[attribute] & 0x08) != 0x08))
            {
               union {unsigned int   word; char byte[2];}  WB;
               union {unsigned long dword; char byte[4];} DWB;
               for(f = 8; f--; )
               {
                  if(buffer[first_char + f] != file->filename[f])
                     break;
               }
               if(f != 0xff)
                  continue; // Next file
               attribute = 32 * i + 26;
               WB.byte[0]  = buffer[attribute++];
               WB.byte[1]  = buffer[attribute++];
               file->clust = WB.word;
               DWB.byte[0] = buffer[attribute++];
               DWB.byte[1] = buffer[attribute++];
               DWB.byte[2] = buffer[attribute++];
               DWB.byte[3] = buffer[attribute];
               file->size  = DWB.dword;
               return FAT_SUCCESS;
            }
         }
      }
   }while(++sector != (fat.root_dir + 31));
   return FAT_FILE_NOTFOUND;
}
   
