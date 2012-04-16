#ifndef _FAT16_H_
#define _FAT16_H_

#define FAT_SUCCESS        0x00
#define FAT_FILE_NOTFOUND  -1
       
typedef struct 
{
//   unsigned long card_size;
   unsigned long partition_start;
//   char file_system;
   unsigned int bytes_per_sector;
   unsigned char sectors_per_cluster;  
   unsigned int max_root_dir_entries;
   unsigned int sectors_per_fat;
//   unsigned long sectors_in_partition;
   unsigned long fat_table;
   unsigned long root_dir;
   unsigned long data_area;
//   unsigned int files_on_disk;
//   unsigned int current_file_num;
//   unsigned long file_entry_sector[MAX_FILES];
//   unsigned int file_entry_number[MAX_FILES];
//   unsigned char file_type[MAX_FILES];
}FAT_t;

typedef struct
{
   char           filename[10];
   char           ext[4];
   unsigned short clust;
   unsigned long  size;
}FILEDESCR_t;


extern FAT_t fat;


void init_FAT(void);
void Read_MBR (void);
void Read_PBR (void);
int FindFile(unsigned long sector, FILEDESCR_t * file);

#endif // _FAT16_H_

