//#include "zzPort.h"
#include "zzSys.h"
#include "oled.h"
#include "font.h"
#include "ipmce.h"
#include "storage.h"
#include "fire_time.h"
#include "fire_ctrl.h"
#include "menu.h"
#include "fmt.h"
#include "kbrd.h"
#include "fire_msgs.h"


//#include <stdlib.h>

struct menu_status active, // Menu state
                   mstack[MENU_DEPTH],
                   *mstack_ptr = mstack; // Menu stack pointer

extern int isNodeConnected(unsigned long long ext_addr);

extern unsigned char zone_state[STRG_MAX_ZONE_NUM];

void nwk_msgs_init(void);
void disp_zones_init(void);
void disp_nwk_init(void);
void disp_datetime_init(void);

void display_label(unsigned char token);
unsigned char menu_navigate(unsigned char key);
unsigned char nav_ESC(unsigned char key);
unsigned char nav_nwk_msg(unsigned char key);
unsigned char nav_nwk(unsigned char key);
unsigned char nav_zones(unsigned char key);
unsigned char nav_datetime(unsigned char key);

void display_menu_list(unsigned char key);
void disp_root(unsigned char key);
void disp_info_ver(unsigned char key);
void disp_info_sernum(unsigned char key);
void disp_msgs(unsigned char key);
void disp_nwk(unsigned char key);
void disp_zones_cfg(unsigned char key);
void disp_datetime(unsigned char key);


void display_last_msg(unsigned char key);

void tm_edit_init(void);
unsigned char tm_edit_on_key(unsigned char key);
void disp_tm_edit(void * edit_data);



void stat_draw_text(unsigned int x, unsigned int y, struct statusbar_item * item);
void stat_draw_textEx(unsigned int x, unsigned int y, struct statusbar_item * item);
void stat_draw_time(unsigned int x, unsigned int y, struct statusbar_item * item);

const char Yes_Msg[]        = {FSTR_YES}; //"Yes"};
const char No_Msg[]         = {FSTR_NO};  //"No"};
const char SetZone_Msg[]    = {FSTR_SET}; //"Set"};
const char ClrZone_Msg[]    = {FSTR_CLEAR}; //"Clear"};
const char Left_Right_17[]  = {"<-             ->"};

static char * Yes_ptr = Yes_Msg; //"Yes";
static char * No_ptr  = No_Msg;

const struct statusbar_item empty_item1  = {MNU_STATUS_TYPE_EMPTY,  3 * 6, NULL, NULL};
const struct statusbar_item empty_item2  = {MNU_STATUS_TYPE_EMPTY, 10 * 6, NULL, NULL};
const struct statusbar_item empty_item3  = {MNU_STATUS_TYPE_EMPTY,  5 * 6, NULL, NULL};
//const struct statusbar_item yes_item     = {MNU_STATUS_TYPE_TEXT,   5 * 6, " Yes ", NULL};
const struct statusbar_item yes_item     = {MNU_STATUS_TYPE_TEXT,  13 * 6, FSTR_STAT_YES, stat_draw_text};
const struct statusbar_item time_item    = {MNU_STATUS_TYPE_USR,   17 * 6, NULL, stat_draw_time};
//const struct statusbar_item no_item      = {MNU_STATUS_TYPE_TEXT,   4 * 6, FSTR_STAT_NO, NULL};
const struct statusbar_item no_item      = {MNU_STATUS_TYPE_TEXT,  17 * 6, FSTR_STAT_NO, stat_draw_text};
const struct statusbar_item yes_itemEx   = {MNU_STATUS_TYPE_USR,   13 * 6, &Yes_ptr, stat_draw_textEx};
const struct statusbar_item lr_nav_item  = {MNU_STATUS_TYPE_TEXT,  17 * 6, Left_Right_17, NULL};


//struct statusbar mnu_statusbar = {1, {&empty_item1, &yes_item, &empty_item3, &time_item, &empty_item3, &no_item, NULL, NULL}};
//struct statusbar mnu_statusbar = {1, {&yes_itemEx, &time_item, &empty_item3, &no_item, NULL, NULL, NULL, NULL}};
struct statusbar mnu_statusbar = {1, {&yes_itemEx, &time_item, &no_item, NULL, NULL, NULL, NULL, NULL}};

static struct MsgDisplayState
{
   unsigned long firstMsgIdx;
}msgdispstat;

extern struct edit_ctrl tm_edit;

extern const struct mnu_node
                             root_menu,
                                info_menu,
                                   info_zones,
                                   info_datetime,
                                   info_ver,
                                   info_sernum,
                                net_menu,
                                   net_msg,
                                   net_nwk;
                          
/*--------------------------------------------------
 * Root menu
 *--------------------------------------------------*/
const struct mnu_node * root_menu_jump[] = {
                                           &net_menu,
                                           &info_menu,
                                           NULL
                                           },

                        root_menu        = { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE, "Root", NULL,
                                             root_menu_jump, menu_navigate, disp_root };

/*--------------------------------------------------
 * Info menu
 *--------------------------------------------------*/
const struct mnu_node * info_menu_jump[] =  {
                                            &info_zones,
                                            &info_datetime,
                                            &info_ver,
                                            &info_sernum,
                                            NULL
                                            },

                        info_menu        =  { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE,  FSTR_MNU_DEVDATA, NULL, //"Device Data", NULL,
                                              info_menu_jump, menu_navigate, display_menu_list, NULL };

const struct mnu_node   info_zones       = { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE, FSTR_MNU_ZONES, NULL, //"Zones", NULL,
                                             NULL, nav_zones, disp_zones_cfg, disp_zones_init};
const struct mnu_node   info_datetime    = { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE, FSTR_MNU_DATETIME, &tm_edit, //"Date/Time", &tm_edit,
                                             NULL, nav_datetime, disp_datetime, disp_datetime_init};
const struct mnu_node   info_ver         = { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE, FSTR_MNU_FIRMWARE, NULL, //"Firmware Version", NULL,
                                             NULL, nav_ESC, disp_info_ver, NULL};
const struct mnu_node   info_sernum      = { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE, FSTR_MNU_SERIAL, NULL, //"Serial number", NULL,
                                             NULL, nav_ESC, disp_info_sernum, NULL};

/*--------------------------------------------------
 * Net menu
 *--------------------------------------------------*/
const struct mnu_node * net_menu_jump[]  = {
                                           &net_msg,
                                           &net_nwk,
                                           NULL
                                           },
                        net_menu         = { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE, FSTR_MNU_NETWORK, NULL, //"Network", NULL,
                                             net_menu_jump, menu_navigate, display_menu_list, NULL};
const struct mnu_node   net_msg          = { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE, FSTR_MNU_MSGS, NULL, //"Messages", NULL,
                                             NULL, nav_nwk_msg, disp_msgs, nwk_msgs_init};
const struct mnu_node   net_nwk          = { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE, FSTR_MNU_NODELIST, NULL, //"Node List", NULL,
                                             NULL, nav_nwk, disp_nwk, disp_nwk_init};

const struct mnu_node   last_msg_screen  = { NODE_SIGNATURE, 0, 0, OLED_X_SIZE, OLED_Y_SIZE, FSTR_MNU_LASTMSG, NULL, //"Last Message", NULL,
                                             NULL, nav_ESC, display_last_msg, NULL};
unsigned char mnu_init_state;
unsigned char mnu_show_last_msg;

static struct tm tm4edit;
static struct edit_ctrl tm_edit = {EDIT_SIGNATURE, 0, 0, 102, 18, font6x9, (char *)&tm4edit, sizeof(struct tm), 0,
                                   tm_edit_on_key, disp_tm_edit, tm_edit_init};

void tm_edit_init(void)
{
   unsigned long cur_time = sys_time() / 1000;
   parse_time2(cur_time, &tm4edit);
   
   if(tm4edit.tm_year > 99)
      tm4edit.tm_year = 0;
   if(tm4edit.tm_mon > 11)
      tm4edit.tm_mon = 0;
   if(tm4edit.tm_mday > 31)
      tm4edit.tm_mday = 1;
   if(tm4edit.tm_hour > 23)
      tm4edit.tm_hour = 0;
   if(tm4edit.tm_min > 59)
      tm4edit.tm_min = 0;
   if(tm4edit.tm_sec > 59)
      tm4edit.tm_sec = 0;

   tm_edit.buf_pos = 0;
}

int make_value(int orig, unsigned char dig, unsigned char pos)
{
   int val;
   dig -= '0';
   if(pos)
      val = (int)dig * 10 + orig % 10;
   else
      val = (orig / 10) * 10 + dig;
   return val;
}

unsigned char tm_edit_on_key(unsigned char key)
{
   if(isdigit(key))
   {
      int val;
      char hi_pos;
      if(tm_edit.buf_pos & 0x01)
         hi_pos = 0;
      else
         hi_pos = 1;
      switch(tm_edit.buf_pos / 2)
      {
      case 0:
         val = make_value(((struct tm *)tm_edit.buf)->tm_mday, key, hi_pos);
         ((struct tm *)tm_edit.buf)->tm_mday = val;
         break;
      case 1:
         val = make_value(((struct tm *)tm_edit.buf)->tm_mon + 1, key, hi_pos);
         ((struct tm *)tm_edit.buf)->tm_mon = val ? val - 1 : 1;
         break;
      case 2:
         val = make_value(getyear2(((struct tm *)tm_edit.buf)->tm_year), key, hi_pos);
         ((struct tm *)tm_edit.buf)->tm_year = setyear2(val);
         break;
      case 3:
         val = make_value(((struct tm *)tm_edit.buf)->tm_hour, key, hi_pos);
         ((struct tm *)tm_edit.buf)->tm_hour = val;
         break;
      case 4:
         val = make_value(((struct tm *)tm_edit.buf)->tm_min, key, hi_pos);
         ((struct tm *)tm_edit.buf)->tm_min = val;
         break;
      default:
         tm_edit.buf_pos = -1;
      }
      tm_edit.buf_pos++;
   }
   else if(key == LEFT)
   {
      if(tm_edit.buf_pos)
         tm_edit.buf_pos--;
   }
   else if(key == RIGHT)
   {
      if(tm_edit.buf_pos < 10)
         tm_edit.buf_pos++;
   }
   return 0;
}

static unsigned int tm_get_pointer_pos(unsigned int pos)
{
   if(pos > 9)
      pos = 9 + 4;
   else if(pos > 7)
      pos += 4;
   else if(pos > 5)
      pos += 3;
   else if(pos > 3)
      pos += 2;
   else if(pos > 1)
      pos++;
   return pos;
}

void disp_tm_edit(void * edit_data)
{
   struct edit_ctrl * edit_ptr;
   struct tm * tm_buf;
   char str[32];
   int x, y;
   int fwidth, fheight;
   static int prev_pos = 0;
   time_t t;
   
   edit_ptr = (struct edit_ctrl *)edit_data;
   tm_buf   = (struct tm *)edit_ptr->buf;
   
   format_str(str, 32, "%02u/%02u/%02u %02u:%02u", tm_buf->tm_mday, tm_buf->tm_mon + 1, getyear2(tm_buf->tm_year), tm_buf->tm_hour, tm_buf->tm_min);

   x = edit_ptr->x;
   y = edit_ptr->y;
   fwidth  = fh[edit_ptr->font]->width;
   fheight = fh[edit_ptr->font]->height;
   
   OLED_puts(x , y, 0xff, edit_ptr->font, str);
   x += tm_get_pointer_pos(prev_pos) * fwidth;
   y += fheight;
   OLED_clr(x, y, fwidth, fheight);
   prev_pos = edit_ptr->buf_pos;
   x  = edit_ptr->x + tm_get_pointer_pos(prev_pos) * fwidth;
   OLED_puts(x , y, 0xff, edit_ptr->font, "^");
   /*
   t = mktime2(&tm4edit);
   format_str(str, 32, "%ld, %llx", t, (unsigned long long)t * 1000);
   OLED_puts(0, 5 * 9,  0xff, edit_ptr->font, str);
   */
}

//static volatile unsigned long long tl, t1;
//
static struct statusbar_item * datime_prev_item;

unsigned char nav_datetime(unsigned char key)
{
   time_t t;
      switch (key)
      {
      case ENTER:
         t = mktime2(&tm4edit);
         /*
         if(t != -1)
         {
            tl = (unsigned long long)t * 1000;
            if(set_sys_time((uint64_t)tl) == 0)
            {
               char str[64];
               static struct tm tm1;
               t1 = sys_time();
               format_str(str, 64, "%ld, %llx, %llx", t, t1, tl);
               OLED_clrLine(4 * 9, 6 * 9);
               OLED_puts(0, 5 * 9, 0xff, font6x9, str);
               parse_time2(t, &tm1);
               format_str(str, 64, "%02u/%02u/%02u %02u:%02u:%02u", tm1.tm_mday, tm1.tm_mon + 1, getyear2(tm1.tm_year), tm1.tm_hour, tm1.tm_min,
                     tm1.tm_sec);
               OLED_puts(0, 4 * 9, 0xff, font6x9, str);
               //OLED_puts(0, 5 * 9, 0xff, font6x9, "set_sys_time failed");
               return 0;
            }
         }
         */
         set_sys_time((unsigned long long)t * 1000);
      case ESC:
         if (active.menu != &root_menu)
            active = *(--mstack_ptr);                // POP_MENU;
         set_statusbar_item(1, datime_prev_item, NULL);
         return MNU_CLR_ALL;
      default:
         (tm_edit.action)(get_digit(key));
         break;
      }
      return 0;
}

void disp_datetime(unsigned char key)
{
   if(key == MNU_CLR_ALL)
      OLED_cls();
   //OLED_puts(0, 0, 0xff, font6x9, "Date and time setup:");
   OLED_puts(0, 0, 0xff, font6x9,FSTR_MNU_DATETIME_SET);
   (((struct edit_ctrl *)info_datetime.data)->display)(info_datetime.data);
}

void disp_datetime_init(void)
{
   ((struct edit_ctrl *)info_datetime.data)->x = ((256 / 6 + 1) - 14) / 2 * 6;
   ((struct edit_ctrl *)info_datetime.data)->y = 3 * 9;
   (((struct edit_ctrl *)info_datetime.data)->init_fnc)();
   set_statusbar_item(1, &lr_nav_item, &datime_prev_item);
}

void menu_init(unsigned char reason)
{
   mstack_ptr     = &mstack[0];
   active.menu    = &root_menu;
   active.alt     = 0;
   if(reason & MNU_INIT)
   {
      mnu_init_state    = 1;
      mnu_show_last_msg = 0;
   }
   else if(reason & MNU_SHOW_LAST_MSG)
      mnu_show_last_msg = 1;

   if(reason &= MNU_DO_NOT_SHOW)
      return;

   (active.menu->display)(0);
}

void nwk_msgs_init(void)
{
   msgdispstat.firstMsgIdx = 0;
}

void display_label(unsigned char token)
{
   if(token == MNU_CLR_ALL)
      OLED_cls();
   OLED_puts(0, 0, 0xff, font6x9, active.menu->label);
}

unsigned char menu_navigate(unsigned char key)
{
   if(mnu_init_state)
   {
      mnu_init_state = 0;
      return MNU_CLR_ALL;
   }
   /*
   {
      char str[16];
         OLED_clrLine(50, 59);
         OLED_puts(0, 50, 0xff, font6x9, "Key pressed");
         itoa(key, str, 16);
         OLED_puts(sizeof("Key pressed") * 6, 50, 0xff, font6x9, str);
   }
   */
      switch (key)
      {
      case ESC:
         if (active.menu != &root_menu)
         {
            active = *(--mstack_ptr);                // POP_MENU;
            return MNU_CLR_ALL;
         }
         else
            mnu_init_state = 1;
         return 0;
      case DOWN:
         if ( active.menu->jump[++active.alt] == NULL)
            active.alt = 0;
         return MNU_UPDATE;
      case UP:
         if (active.alt)
            --active.alt;
         else
         {
            while(active.menu->jump[active.alt+1])
               active.alt++;
         }
//         OLED_puts(0, 50, 0xff, font6x9, "Right");
         return MNU_UPDATE;
      case ENTER:
         if (active.menu->jump[active.alt] != NULL)
         {
            *mstack_ptr++ = active;                // PUSH_MENU;
            active.menu = active.menu->jump[active.alt]; // Choosen menu
            active.alt = 0;
            if(active.menu->init_fnc)
               (active.menu->init_fnc)();
         }
         return MNU_CLR_ALL;
      default:
         return 0;
      }
}

unsigned char nav_ESC(unsigned char key)
{
      switch (key)
      {
      case ESC:
         if (active.menu != &root_menu)
            active = *(--mstack_ptr);                // POP_MENU;
         return MNU_CLR_ALL;
      default:
         break;
      }
      return 0;
}


unsigned char nav_nwk_msg(unsigned char key)
{
      switch (key)
      {
      case ESC:
         if (active.menu != &root_menu)
            active = *(--mstack_ptr);                // POP_MENU;
         return MNU_CLR_ALL;
      case UP:
         if(msgdispstat.firstMsgIdx)
            msgdispstat.firstMsgIdx--;
         else
            msgdispstat.firstMsgIdx = logEntriesNum() - 1;
         return MNU_UPDATE;
      case DOWN:
         if(++msgdispstat.firstMsgIdx >= logEntriesNum())
            msgdispstat.firstMsgIdx = 0;
         return MNU_UPDATE;
      default:
         break;
      }
      return 0;
}

void on_msg_add(void)
{
   if(active.menu != &net_msg)
      return;
   disp_list(NULL, 5, -1, 1);
}

static void disp_list(char ** list, unsigned int size, unsigned int selection, unsigned char clearRemaining)
{
   unsigned int i, Y_pos, font_Vsize, lines2disp;
   unsigned char noSelectionColor;
   int res;
   font_Vsize = fh[font6x9]->height;
   lines2disp = (OLED_Y_SIZE - 2 * font_Vsize) / font_Vsize;
   Y_pos = font_Vsize;
   noSelectionColor = selection == -1 ? 0xff : 0xaf;
      
   if(list)
   {
      for(i = 0; *list[i]; i++)
      {
         if(i == size)
            break;
         OLED_clrLine(Y_pos, Y_pos + font_Vsize);
         OLED_puts(30, Y_pos, i == selection ? 0xff : noSelectionColor , font6x9, list[i]);
         Y_pos += font_Vsize;
      }
      if(clearRemaining)
      {
         while(i < 5)
         {
            OLED_clrLine(Y_pos, Y_pos + font_Vsize);
            Y_pos += font_Vsize;
            i++;
         }
      }
   }
   else
   {
      unsigned long timestamp;
      struct tm fmt_time;
      unsigned long pos;
      char str[128];
      if(logEntriesNum() < msgdispstat.firstMsgIdx)
      {
         msgdispstat.firstMsgIdx = logEntriesNum();
         if(msgdispstat.firstMsgIdx < lines2disp)
         {
            lines2disp = msgdispstat.firstMsgIdx;
            msgdispstat.firstMsgIdx = 0;
         }
         else
            msgdispstat.firstMsgIdx -= lines2disp;
      }
      pos = logGetEntryPos(msgdispstat.firstMsgIdx);
      while(lines2disp--)
      {
         timestamp = logGetEntryTimestamp(pos) / 1000;
         parse_time2(timestamp, &fmt_time);
         format_str(str, 128, "%02u/%02u/%02u %02u:%02u ", fmt_time.tm_mday, fmt_time.tm_mon + 1, getyear2(fmt_time.tm_year),
               fmt_time.tm_hour, fmt_time.tm_min);
         timestamp = strlen(str);
         OLED_clrLine(Y_pos, Y_pos + font_Vsize);
         if(logGetEntryMsg(pos, str + timestamp, 128 - timestamp) == STRG_SUCCESS)
            OLED_puts(6, Y_pos, 0xff , font6x9, str);
         /*
         if((res = logGetEntryMsg(pos, str, 128)) != STRG_SUCCESS)
         {
            strcpy(str, "Error retrieving msg: ");
            itoa(res, str + strlen(str), 10);
            strcpy(str + strlen(str), ", pos = ");
            ultoa(pos, str + strlen(str), 10);
         }
         OLED_puts(6, Y_pos, 0xff , font6x9, str);
         */
         Y_pos += font_Vsize;
         pos = logGetPrevEntryPos(pos);
      }
   }
}

static void disp_mnu_node_list(struct mnu_node ** nodes, unsigned char start_idx, unsigned char active)
{
   unsigned int i, Y_pos, font_Vsize, nodes2disp;
   font_Vsize = fh[font6x9]->height;
   nodes2disp = (OLED_Y_SIZE - 2 * font_Vsize) / font_Vsize;
   Y_pos = font_Vsize;
   for(i = 0; *nodes; nodes++, i++)
   {
      if(i < start_idx)
         continue;
      if((*nodes)->signature != NODE_SIGNATURE)
         break;
      OLED_clrLine(Y_pos, Y_pos + font_Vsize);
      OLED_puts(30, Y_pos, (i == active ? 0xff : 0xaf), font6x9, (*nodes)->label);
      Y_pos += font_Vsize;
   }
}

void disp_root(unsigned char key)
{
   char str[8];
//      OLED_putBitmap(0, 0, &ipmce_bmp);

   if(mnu_show_last_msg)
   {
      //OLED_cls();
      display_last_msg(0);
      return;
   }
   
   if(mnu_init_state)
   {
      OLED_cls();

      OLED_putBitmap(0, 0, &ipmce_bmp);
         
      OLED_puts(68, 05, 0xff, font6x9, "\xC8\xED\xF1\xF2\xE8\xF2\xF3\xF2 \xD2\xEE\xF7\xED\xEE\xE9 \xCC\xE5\xF5\xE0\xED\xE8\xEA\xE8 \xE8");
      OLED_puts(68, 14, 0xff, font6x9, "\xC2\xFB\xF7\xE8\xF1\xEB\xE8\xF2\xE5\xEB\xFC\xED\xEE\xE9 \xD2\xE5\xF5\xED\xE8\xEA\xE8");
      OLED_puts(68, 23, 0xff, font6x9, "\xE8\xEC\x2E \xD1\x2E \xC0\x2E \xCB\xE5\xE1\xE5\xE4\xE5\xE2\xE0 \xD0\xC0\xCD");
      OLED_puts(68, 32, 0xff, font6x9, "http://www.ipmce.ru");
      /*
      if(systemState(SYS_STATE_FIRE))
         OLED_puts(68, 41, 0xff, font6x9, "Fire!");
      else
         OLED_puts(68, 41, 0xff, font6x9, "No fire");
      */

      return;
   }

   if((key == MNU_UPDATE) || (key == MNU_CLR_ALL))
   {
      if(key == MNU_CLR_ALL)
         OLED_cls();
      //display_label(1);
      disp_mnu_node_list(active.menu->jump, 0, active.alt);
   }
}

void display_menu_list(unsigned char key)
{
   if((key == MNU_UPDATE) || (key == MNU_CLR_ALL))
   {
      if(key == MNU_CLR_ALL)
         OLED_cls();
      display_label(1);
      disp_mnu_node_list(active.menu->jump, 0, active.alt);
   }
}

void disp_info_ver(unsigned char key)
{
   if(key == MNU_CLR_ALL)
      OLED_cls();
   display_label(1);
   OLED_puts(0, 18, 0xff, font6x9, FSTR_MNU_FIRMWARE_VER);
}

void disp_info_sernum(unsigned char key)
{
   if(key == MNU_CLR_ALL)
      OLED_cls();
   display_label(1);
   OLED_puts(0, 18, 0xff, font6x9, FSTR_MNU_SERIAL_NUM);
}

void disp_msgs(unsigned char key)
{
   if((key == MNU_CLR_ALL) || (key == MNU_UPDATE))
   {
      if(key == MNU_CLR_ALL)
         OLED_cls();
      display_label(1);
      disp_list(NULL, 0, -1, 1);
   }
//   OLED_puts(0, 18, 0xff, font6x9, "Messages");
}

static struct ConnectedNodesDispCfg
{
   unsigned int upperNode2Disp;
}cnodeDispCfg;

unsigned char nav_nwk(unsigned char key)
{
      switch (key)
      {
      case ESC:
         if (active.menu != &root_menu)
         {
            active = *(--mstack_ptr);                // POP_MENU;
            return MNU_CLR_ALL;
         }
         return 0;
      case DOWN:
         getNextChildExtAddress(&cnodeDispCfg.upperNode2Disp);
         if(cnodeDispCfg.upperNode2Disp == -1)
            cnodeDispCfg.upperNode2Disp = getFirstChildPosition();
         return MNU_UPDATE;
      case UP:
         getPrevChildExtAddress(&cnodeDispCfg.upperNode2Disp);
         if(cnodeDispCfg.upperNode2Disp == -1)
            cnodeDispCfg.upperNode2Disp = getLastChildPosition();
         return MNU_UPDATE;
      default:
         return 0;
      }
}

void disp_nwk_init(void)
{
   cnodeDispCfg.upperNode2Disp = getFirstChildPosition();
}

void update_disp_nwk(void)
{
   unsigned int pos;
   if(active.menu != &net_nwk)
      return;
   if(cnodeDispCfg.upperNode2Disp == -1)
      cnodeDispCfg.upperNode2Disp = getFirstChildPosition();
   else if(!isNodeConnected(cnodeDispCfg.upperNode2Disp))
   {
      pos = cnodeDispCfg.upperNode2Disp;
      if(getPrevChildExtAddress(&cnodeDispCfg.upperNode2Disp) == -1)
      {
         cnodeDispCfg.upperNode2Disp = pos;
         getNextChildExtAddress(&cnodeDispCfg.upperNode2Disp);
      }
   }
   disp_nwk(MNU_UPDATE);
}

void disp_nwk(unsigned char key)
{
   unsigned int i, pos, empty;
   int nodepos;
   struct NodeCfg ncfg;
   static char nodelist[5][32];
   static char * nodes[5] = {&nodelist[0][0], &nodelist[1][0],&nodelist[2][0], &nodelist[3][0], &nodelist[4][0]};

   if(key == MNU_CLR_ALL)
      OLED_cls();
   display_label(1);
   pos = cnodeDispCfg.upperNode2Disp;
//   OLED_puts(0, 18, 0xff, font6x9, "Node list");
   for(i = 0, empty = 1; i < 5; i++)
   {
      if(pos != -1)
      {
         empty = 0;
         if((nodepos = strgFindNode(getNextChildExtAddress(&pos), &ncfg)) >= 0)
         {
            if((nodepos = strgGetNodeName(nodepos, nodes[i], 32)) < 0)
               strcpy(nodes[i], FSTR_MNU_CONFIG_ERR);
         }
         else
            strcpy(nodes[i], FSTR_MNU_NODE_NOTCONF);
      }
      else
         nodes[i][0] = 0;
   }
   if(empty)
      strcpy(nodes[0], FSTR_MNU_NONE);
   disp_list(nodes, 5, -1, 1);
}

static struct ZonesDispCfg
{
   unsigned int upperZone2Disp;
   unsigned int curSelIdx;
   unsigned int showCurSelection;
   
   unsigned int firstNodeInZone;
   unsigned int lastNodeInZone;
   unsigned int nodesInZone;
   unsigned int upperNode2Disp;
   unsigned int displayedNodes[5];
   unsigned int selectedNodeIdx;
   unsigned int displayedNodesNum;
   unsigned int mostLeftDispPos;
   char *       prevStatYesMsg;
}zdispcfg;

extern struct Config syscfg;

static void findNodes(unsigned int zone)
{
   unsigned int   max_node, node_idx;
   struct NodeCfg node;
   struct ZoneCfg zone_cfg;
   
   zdispcfg.displayedNodesNum = 0;
   zdispcfg.nodesInZone = 0;
   if(strgGetZone(zone, &zone_cfg) != STRG_SUCCESS)
      return;
   max_node = strgGetNodesNum();
   for(zdispcfg.firstNodeInZone = 0; zdispcfg.firstNodeInZone < max_node; zdispcfg.firstNodeInZone++)
   {
      if(strgGetNode(zdispcfg.firstNodeInZone, &node) != STRG_SUCCESS)
         continue;
      if(node.zone == zone_cfg.zone)
         break;
   }
   if(zdispcfg.firstNodeInZone == max_node)
      return;
 

   for(node_idx = zdispcfg.lastNodeInZone = zdispcfg.firstNodeInZone; node_idx < max_node; node_idx++)
   {
      if(strgGetNode(node_idx, &node) != STRG_SUCCESS)
         continue;
      if(node.zone == zone_cfg.zone)
      {
         zdispcfg.lastNodeInZone = node_idx;
         if(zdispcfg.displayedNodesNum < 5)
            zdispcfg.displayedNodes[zdispcfg.displayedNodesNum++] = zdispcfg.lastNodeInZone;
         zdispcfg.nodesInZone++;
      }
   }
}

static unsigned int fillNodeList(unsigned int node, unsigned int * list, unsigned int list_size)
{
   struct NodeCfg ncfg;
   int i = 0;

   while(list_size)
   {
      if(node == zdispcfg.lastNodeInZone)
         break;
      if(strgGetNode(node, &ncfg) != STRG_SUCCESS)
         break;
      if(ncfg.zone == zdispcfg.curSelIdx)
      {
         *list++ = node;
         i++;
         list_size--;
      }
      node++;
   }
   return i;
}

static unsigned int menuNextNode(unsigned int node)
{
   unsigned int   next_node;
   struct NodeCfg node_cfg;
   struct ZoneCfg zone_cfg;
   
   if(strgGetZone(zdispcfg.curSelIdx, &zone_cfg) != STRG_SUCCESS)
      return node;
   next_node = (node == zdispcfg.lastNodeInZone) ? zdispcfg.firstNodeInZone : node + 1;
   while(next_node != node)
   {
      if(strgGetNode(next_node, &node_cfg) != STRG_SUCCESS)
         return node;
      if(node_cfg.zone == zone_cfg.zone)
         break;
      if(next_node == zdispcfg.lastNodeInZone)
         next_node = zdispcfg.firstNodeInZone;
      else
         next_node++;
   }
   return next_node;
}

static unsigned int menuPrevNode(unsigned int node)
{
   unsigned int   next_node;
   struct NodeCfg node_cfg;
   struct ZoneCfg zone_cfg;
   char str[32];
   
   if(strgGetZone(zdispcfg.curSelIdx, &zone_cfg) != STRG_SUCCESS)
      return node;
   next_node = (node == zdispcfg.firstNodeInZone) ? zdispcfg.lastNodeInZone : node - 1;
   while(next_node != node)
   {
      if(strgGetNode(next_node, &node_cfg) != STRG_SUCCESS)
         return node;
      if(node_cfg.zone == zone_cfg.zone)
         break;
      if(next_node == zdispcfg.firstNodeInZone)
         next_node = zdispcfg.lastNodeInZone;
      else
         next_node--;
   }
   return next_node;
}

static inline void menu_inc_node_cnt(void)
{
   unsigned int node_idx;
   if(!zdispcfg.displayedNodesNum)
      return;
   if(++zdispcfg.selectedNodeIdx >= zdispcfg.displayedNodesNum)
   {
      if(zdispcfg.nodesInZone < 5)
      {
        zdispcfg.selectedNodeIdx = 0;
      }
      else if(zdispcfg.upperNode2Disp == zdispcfg.lastNodeInZone)
      {
         zdispcfg.selectedNodeIdx = 0;
         zdispcfg.upperNode2Disp  = zdispcfg.firstNodeInZone;
         zdispcfg.displayedNodesNum = fillNodeList(zdispcfg.firstNodeInZone, zdispcfg.displayedNodes, 5);
      }
      else
      {
         zdispcfg.selectedNodeIdx--;
         for(node_idx = 0; node_idx < zdispcfg.selectedNodeIdx; node_idx++)
            zdispcfg.displayedNodes[node_idx] = zdispcfg.displayedNodes[node_idx + 1];
         zdispcfg.upperNode2Disp = zdispcfg.displayedNodes[0];
         if(zdispcfg.displayedNodes[node_idx] != zdispcfg.lastNodeInZone)
            zdispcfg.displayedNodes[node_idx] = menuNextNode(zdispcfg.displayedNodes[node_idx]);
         else
            zdispcfg.displayedNodesNum = zdispcfg.selectedNodeIdx--;
      }
   }
}


static inline void menu_dec_node_cnt(void)
{
   unsigned int node_idx;
   if(!zdispcfg.displayedNodesNum)
      return;
   if(!zdispcfg.selectedNodeIdx)
   {
      if(zdispcfg.nodesInZone < 5)
      {
         zdispcfg.selectedNodeIdx = zdispcfg.displayedNodesNum - 1;
      }
      else if(zdispcfg.upperNode2Disp != zdispcfg.firstNodeInZone)
      {
         if(zdispcfg.displayedNodesNum < 5)
            zdispcfg.displayedNodesNum++;
         for(node_idx = zdispcfg.displayedNodesNum - 1; node_idx--;)
            zdispcfg.displayedNodes[node_idx + 1] = zdispcfg.displayedNodes[node_idx];
         zdispcfg.displayedNodes[0] = menuPrevNode(zdispcfg.upperNode2Disp);
         zdispcfg.upperNode2Disp = zdispcfg.displayedNodes[0];
      }
      else
      {
         zdispcfg.upperNode2Disp = zdispcfg.displayedNodes[0] = zdispcfg.lastNodeInZone;
         zdispcfg.displayedNodesNum = 1;
      }
   }
   else
      zdispcfg.selectedNodeIdx--;
}

static char * zoneState2Str(unsigned int zone)
{
   //return zoneState(zone, ZONE_STATE_ON) ? " (on)" : " (off)";
   return zoneState(zone, ZONE_STATE_ON) ? FSTR_MNU_ZONE_ON : FSTR_MNU_ZONE_OFF;
}

void update_disp_zones_cfg(void)
{
   if((active.menu != &info_zones) || (!zdispcfg.showCurSelection))
      return;
   disp_zones_cfg(MNU_UPDATE);
}

void disp_zones_cfg(unsigned char key)
{
   unsigned int i;
   unsigned int selection;
   int err;
   static char namelist[5][32];
   static char * names[5] = {&namelist[0][0], &namelist[1][0],&namelist[2][0], &namelist[3][0], &namelist[4][0]};

   if((key == MNU_CLR_ALL) || (key == MNU_UPDATE))
   {
      if(key == MNU_CLR_ALL)
         OLED_cls();
      if(!zdispcfg.showCurSelection)
      {
         display_label(1);
         for(i = 0; i < 5; i++)
         {
            //if(strgGetZoneName(zdispcfg.upperZone2Disp + i, &namelist[i][0], 32) <= 0)
            if((err = strgGetZoneName(zdispcfg.upperZone2Disp + i, &namelist[i][0], 32)) < 0)
            {
               //format_str(&namelist[i][0], 32, "MMC err: %d", (err == -6) ? i : err);
               //i++;
               break;
            }
            strcat(&namelist[i][0], zoneState2Str(zdispcfg.upperZone2Disp + i));
         }
         selection = zdispcfg.curSelIdx - zdispcfg.upperZone2Disp;
         /*
         if(!i)
         {
            strcpy(&namelist[0][0], "<none>");
            i++;
         }
         disp_list(names, i, zdispcfg.curSelIdx - zdispcfg.upperZone2Disp);
         */
      }
      else
      {
//         struct NodeCfg ncfg;
         struct ZoneCfg zcfg;
         int name_len;
         
         if(strgGetZone(zdispcfg.curSelIdx, &zcfg) != STRG_SUCCESS)
            return;
         if(strgGetZoneName(zdispcfg.curSelIdx, &namelist[0][0], 32 * 5) < 0)
            return;
         if(key == MNU_UPDATE)
            OLED_clrLine(0, 9);
         OLED_puts(0, 0, 0xff, font6x9, FSTR_MNU_ZONE);
         OLED_puts(6 *  sizeof(FSTR_MNU_ZONE), 0, 0xff, font6x9, &namelist[0][0]);
         OLED_puts(6 * (sizeof(FSTR_MNU_ZONE) + strlen(&namelist[0][0])), 0, 0xff, font6x9, zoneState2Str(zdispcfg.curSelIdx));
         //format_str(&namelist[0][0], 32, " %u z%u", zdispcfg.nodesInZone, zcfg.zone);
         //OLED_puts(6 * (sizeof(FSTR_MNU_ZONE) + strlen(&namelist[0][0])), 0, 0xff, font6x9, &namelist[0][0]);
       
         name_len = 0;
         for(i = 0; i < zdispcfg.displayedNodesNum; i++)
         {
            if((name_len = strgGetNodeName(zdispcfg.displayedNodes[i], &namelist[i][0], 32)) < 0)
               break;
         /*
            if((name_len = strgGetNode(zdispcfg.displayedNodes[i], &ncfg)) != STRG_SUCCESS)
               break;
            format_str(&namelist[i][0], 32, "%04u z%u", zdispcfg.displayedNodes[i], ncfg.zone);
         */

            //strcat(&namelist[i][0], isNodeConnected(zdispcfg.displayedNodes[i]) ? " ("FSTR_MNU_CONNECTED")" : " ("FSTR_MNU_NOTCONNECTED")");
            //name_len = strlen(&namelist[i][0]) * 6;
            name_len *= 6;
            if(name_len > zdispcfg.mostLeftDispPos)
               zdispcfg.mostLeftDispPos = name_len;
         }
         
         //selection = -1;
         selection = zdispcfg.selectedNodeIdx;
         if((selection >= 0) && (selection < 5))
            strcat(&namelist[selection][0], isNodeConnected(zdispcfg.displayedNodes[selection]) ? "("FSTR_MNU_CONNECTED")" : "("FSTR_MNU_NOTCONNECTED")");
         /*
         if(isNodeConnected(zdispcfg.displayedNodes[zdispcfg.selectedNodeIdx]))
            OLED_puts(zdispcfg.mostLeftDispPos + 6 * 9, 9, 0xff, font6x9, FSTR_MNU_CONNECTED);
         else
            OLED_puts(zdispcfg.mostLeftDispPos + 6 * 9, 9, 0xff, font6x9, FSTR_MNU_NOTCONNECTED);
         */
      }

      if(!i)
      {
         strcpy(&namelist[i][0], FSTR_MNU_NONE);
         i++;
      }
      disp_list(names, i, selection, 1);
   }
}

void disp_zones_init(void)
{
   memset(&zdispcfg, 0, sizeof(struct ZonesDispCfg));
}

unsigned char nav_zones(unsigned char key)
{
   int i;
   if(!zdispcfg.showCurSelection)
   {
      switch (key)
      {
      case ESC:
         if (active.menu != &root_menu)
            active = *(--mstack_ptr);                // POP_MENU;
         return MNU_CLR_ALL;
      case UP:
         if(zdispcfg.curSelIdx)
         {
            if(zdispcfg.curSelIdx == zdispcfg.upperZone2Disp)
               zdispcfg.upperZone2Disp--;
            zdispcfg.curSelIdx--;
         }
         else
         {
            zdispcfg.curSelIdx = strgGetZonesNum() - 1;
            zdispcfg.upperZone2Disp = (zdispcfg.curSelIdx > 4) ? zdispcfg.curSelIdx - 5 : 0;
         }
         return MNU_UPDATE;
      case DOWN:
         if(++zdispcfg.curSelIdx >= strgGetZonesNum())
         {
            zdispcfg.curSelIdx = 0;
            zdispcfg.upperZone2Disp = 0;
         }
         else if((zdispcfg.curSelIdx - zdispcfg.upperZone2Disp) > 4)
            zdispcfg.upperZone2Disp = zdispcfg.curSelIdx;
         return MNU_UPDATE;
      case ENTER:
         zdispcfg.showCurSelection = 1;
         zdispcfg.selectedNodeIdx  = 0;
         findNodes(zdispcfg.curSelIdx);
         i = /*zdispcfg.upperZone2Disp +*/ zdispcfg.curSelIdx;
//         zdispcfg.prevStatYesMsg   = *(char **)(yes_itemEx.data);
//         *(char **)(yes_itemEx.data) = zoneState(i, ZONE_STATE_ON) ? SetZone_Msg : ClrZone_Msg;
         zdispcfg.prevStatYesMsg   = set_statusbar_usrtext(&yes_itemEx, zoneState(i, ZONE_STATE_ON) ? ClrZone_Msg : SetZone_Msg);
         return MNU_CLR_ALL;
      default:
         break;
      }
   }
   else
   {
      switch (key)
      {
      case ESC:
         zdispcfg.showCurSelection = 0;
         *(char **)(yes_itemEx.data) = zdispcfg.prevStatYesMsg;
         return MNU_CLR_ALL;
      case UP:
         menu_dec_node_cnt();
         return MNU_UPDATE;
      case DOWN:
         menu_inc_node_cnt();
         return MNU_UPDATE;
      case ENTER:
         i = /*ispcfg.upperZone2Disp +*/ zdispcfg.curSelIdx;
//         zoneSet(i, zoneState(i, ZONE_STATE_ON) ? ZONE_STATE_ON : ~ZONE_STATE_ON, ZONE_STATE_ON);
         if(zoneState(i, ZONE_STATE_ON))
         {
            zoneSet(i, ~ZONE_STATE_ON, ZONE_STATE_ON);
            set_statusbar_usrtext(&yes_itemEx, SetZone_Msg);
         }
         else
         {
            zoneSet(i,  ZONE_STATE_ON, ZONE_STATE_ON);
            set_statusbar_usrtext(&yes_itemEx, ClrZone_Msg);
         }
         compileSysState();
//         if(IS_ZONE_ON(zone_state[i]))
//            ZONE_STATE
         return MNU_UPDATE;
      default:
         break;
      }
   }
   return 0;
}

static void display_last_msg(unsigned char key)
{
   unsigned int len;
   char str[32];
   struct tm fmt_time;
   time_t cur_time;

   OLED_clrLine(2 * 9, 3 * 9);
   //OLED_puts((OLED_X_SIZE - sizeof("Attention!") * 6)/ 2, 2 * 9, 0xff, font6x9, "Attention!");
   OLED_puts((OLED_X_SIZE - sizeof(FSTR_MNU_ATTENTION) * 6)/ 2, 2 * 9, 0xff, font6x9, FSTR_MNU_ATTENTION);
   if(lastMsg.msg_data.entry_type == 0)
   {
      len = lastMsg.msg_data.msg_len * 6;
      if(len)
      {
         if(len > OLED_X_SIZE)
            len = OLED_X_SIZE;
         OLED_clrLine(3 * 9, 5 * 9);
         OLED_puts((OLED_X_SIZE - len)/ 2, 3 * 9, 0xff, font6x9, lastMsg.msg);
         //parse_time2(lastMsg.msg_data.timestamp / 1000, &fmt_time);
         //memset(&fmt_time, 0, sizeof(struct tm));
         cur_time = lastMsg.msg_data.timestamp / 1000;
         parse_time2(cur_time, &fmt_time);
         format_str(str, 32, "%02u/%02u/%02u %02u:%02u:%02u", fmt_time.tm_mday, fmt_time.tm_mon + 1, getyear2(fmt_time.tm_year),
               fmt_time.tm_hour, fmt_time.tm_min, fmt_time.tm_sec);
         //tm2str(&fmt_time, str);
         OLED_puts((OLED_X_SIZE - 17 * 6) / 2, 4 * 9, 0xff, font6x9, str);
      }
   }
/*   
   char str[64];

   if(logEntriesNum())
   {
      if(logGetEntryMsg(logGetLastEntryPos(), str, 64)!= STRG_SUCCESS)
         strcpy(str, "Error retrieveing msg");
   }
   else
      strcpy(str, "No messages");
   OLED_puts(30, 18, 0xff, font6x9, str);
*/
}

int show_last_msg(void)
{
   if(active.menu != &last_msg_screen)
   {
      if(mstack_ptr == (mstack + MENU_DEPTH))
         return MNU_OUT_OF_MEMORY;
      *mstack_ptr++ = active;
      active.menu = &last_msg_screen;
      OLED_cls();
   }
   active.menu->display(0);
   return MNU_SUCCESS;
}

int set_statusbar_item(unsigned int idx, struct statusbar_item * item, struct statusbar_item ** prev)
{
   if(idx >= MNU_STATUS_ITEM_NUM)
      return MNU_INV_IDX;
   if(prev)
      *prev = mnu_statusbar.list[idx];
   mnu_statusbar.list[idx] = item;
   return MNU_SUCCESS;
}

void update_statusbar(void)
{
   unsigned int i;
   struct statusbar_item * item;
   unsigned int x_pos = 0;
//   OLED_clrLine(OLED_Y_SIZE - 9, OLED_Y_SIZE - 1);
   for(i = 0; i < MNU_STATUS_ITEM_NUM; i++)
   {
      item = mnu_statusbar.list[i];
      if(!item)
         break;
      OLED_clr(x_pos, OLED_Y_SIZE - 9, item->len, 9);
      if(item->draw_item)
         (item->draw_item)(x_pos, OLED_Y_SIZE - 9, item);
      else if(item->type == MNU_STATUS_TYPE_TEXT)
         OLED_puts(x_pos, OLED_Y_SIZE - 9, 0xff, font6x9, (char *)item->data);
      x_pos += item->len;
   }
}

int update_statusbar_item(const struct statusbar_item * item, unsigned char clr)
{
   unsigned int i;
   unsigned int x_pos = 0;
   for(i = 0; i < MNU_STATUS_ITEM_NUM; i++)
   {
      if(item == mnu_statusbar.list[i])
      {
         if(clr)
            OLED_clr(x_pos, OLED_Y_SIZE - 9, item->len, 9);
         if(item->draw_item)
            (item->draw_item)(x_pos, OLED_Y_SIZE - 9, item);
         else if(item->type == MNU_STATUS_TYPE_TEXT)
            OLED_puts(x_pos, OLED_Y_SIZE - 9, 0xff, font6x9, (char *)item->data);
         break;
      }
      x_pos += mnu_statusbar.list[i]->len;
   }
   if(i == MNU_STATUS_ITEM_NUM)
      return MNU_ITEM_NOT_FOUND;
   return i;
}

char * set_statusbar_usrtext(const struct statusbar_item * item, char * new_text)
{
   char * prev;

   prev = *(char **)(item->data);
   *(char **)(item->data) = new_text;
   return prev;
}

void stat_draw_text(unsigned int x, unsigned int y, struct statusbar_item * item)
{
   unsigned int text_len = strlen((char *)item->data) * 9;

   if(text_len > item->len)
      text_len = item->len;
   x += (item->len - text_len) / 2;
   OLED_puts(x, y, 0xff, font6x9, (char *)item->data);
}

void stat_draw_textEx(unsigned int x, unsigned int y, struct statusbar_item * item)
{
   unsigned int text_len;
   char * ptr;
   
   if(item->data == NULL)
      return;
   if(item->type == MNU_STATUS_TYPE_USR)
   {
      ptr = *((char **)(item->data));
      if(ptr == NULL)
         return;
   }
   else if(item->type == MNU_STATUS_TYPE_TEXT)
      ptr = item->data;
   else
      return;
   text_len = strlen(ptr) * 9;
   if(text_len > item->len)
      text_len = item->len;
   x += (item->len - text_len) / 2;
   OLED_puts(x, y, 0xff, font6x9, ptr);
}

void stat_draw_time(unsigned int x, unsigned int y, struct statusbar_item * item)
{
   
   time_t cur_time;
   struct tm fmt_time;
   unsigned int i;
   char str[32];
   char * str_ptr;
   

   cur_time = sys_time() / 1000;
   parse_time2(cur_time, &fmt_time);
   //tm2str(&fmt_time, str);
   
   format_str(str, 32, "%02u/%02u/%02u %02u:%02u:%02u", fmt_time.tm_mday, fmt_time.tm_mon + 1, getyear2(fmt_time.tm_year),
         fmt_time.tm_hour, fmt_time.tm_min, fmt_time.tm_sec);
   
   //format_str(str, 32, "%llx", sys_time());
   OLED_puts(x, y, 0xff, font6x9, str);
}

void update_time(void)
{
   update_statusbar_item(&time_item, 0);
}

