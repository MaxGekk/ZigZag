/****************************************************************************
 * File:   menu.h
 * **************************************************************************/
#ifndef _MENU_H_
#define _MENU_H_

#define NODE_SIGNATURE        0x4e44
#define EDIT_SIGNATURE        0x4544

#define display_node(keys) (active.menu->display)((active.menu->action)(keys))
#define PUSH_MENU          {*mstack_ptr++ = active;  }
#define POP_MENU           { active = *(--mstack_ptr);}

#define MENU_DEPTH  5 /* макс глубина cтека меню */


#ifndef NULL
#define  NULL  ((void *)0)
#endif  // NULL

#define LINE1  1   /* передача режима регенерации дисплея */
#define LINE2  2

#ifndef ESC
#define ESC       0x04
#endif

#ifndef ENTER
#define ENTER     0x01
#endif

#ifndef DOWN
#define DOWN      0x0e
#endif

#ifndef UP
#define UP        0x0d
#endif

#ifndef LEFT
#define LEFT      0x02
#endif

#ifndef RIGHT
#define RIGHT     0x03
#endif

#define MNU_INIT               1
#define MNU_SHOW_LAST_MSG      2
#define MNU_DO_NOT_SHOW        8
#define MNU_UPDATE             0xfe
#define MNU_CLR_ALL            0xff


#define MNU_STATUS_TYPE_TEXT   0
#define MNU_STATUS_TYPE_EMPTY  1
#define MNU_STATUS_TYPE_USR    0x1000

#define MNU_SUCCESS            0
#define MNU_MMC_ERROR          -1
#define MNU_OUT_OF_MEMORY      -5
#define MNU_INV_IDX            -6
#define MNU_ITEM_NOT_FOUND     -8

#define MNU_STATUS_ITEM_NUM    8

struct statusbar_item
{
   unsigned int type;
   unsigned int len;
   void         *data;
   void (*draw_item)(unsigned int, unsigned int, struct statusbar_item *);
};

struct statusbar
{
   unsigned int          lines;
   struct statusbar_item *list[MNU_STATUS_ITEM_NUM];
};

struct mnu_node {
   unsigned int    signature           ; // Signature
   unsigned int    x, y;               ; // Upper left corner
   unsigned int    x_size,
                   y_size              ; // Sizes
   char            *label              ; // Node name
   void            *data;              ; // Node associated data
   struct mnu_node **jump              ; // Pointer to lower level menu list
   unsigned char (*action)(unsigned char)     ; // event processing
   void (*display)(unsigned char)             ; // 
   void (*init_fnc)(void)                     ; // Menu init when entering
};

struct edit_ctrl
{
   unsigned int    signature;
   unsigned int    x, y;
   unsigned int    x_size,
                   y_size;
   unsigned int    font;
   unsigned char   *buf;
   unsigned int    buf_size;
   unsigned int    buf_pos;
   unsigned char (*action)(unsigned char);
   void (*display)(void *);
   void (*init_fnc)(void);
};

struct menu_status {
       struct mnu_node * menu; // Pointer to the active node
       unsigned char      alt; // 
};

extern struct menu_status active  ,      // Menu state
                          mstack[],
                          *mstack_ptr;   // Menu stack pointer

void menu_init(unsigned char reason);
int show_last_msg(void);
char * tm2str(struct tm * tm, char * str_ptr);
void update_statusbar(void);
int update_statusbar_item(const struct statusbar_item * item, unsigned char clr);
void update_time(void);
void update_disp_nwk(void);
void update_disp_zones_cfg(void);
void on_msg_add(void);

#endif // _MENU_H_
/* end of file menu.h */


