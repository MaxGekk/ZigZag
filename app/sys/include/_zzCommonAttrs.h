#ifndef COMMON_ATTRS_H
#define COMMON_ATTRS_H

#include<zzTypes.h>


/* Номера атрибутов */
/*** Общие атрибуты */
#define     ATTR_STATE      0x00    /* Состояние объекта */
#define     ATTR_OBJ        0x01    /* Код прикладного объекта */
#define     ATTR_INTEREST_ADDR_FIRST    0x02  /* Короткие адреса заинтересованных в событиях сторон */
#define     ATTR_INTEREST_ADDR_LAST     0x06
#define     ATTR_INTEREST_PORT_FIRST    0x07  /* Номера портов заинтересованных в событиях сторон */
#define     ATTR_INTEREST_PORT_LAST     0x0b
#define     ATTR_THRESHOLD_REPORT_MODE  0x20 /* Режим генерации сообщений о пересечении порогов*/

/* Размеры атрибутов */
#define     ATTR_STATE_SIZE         1
#define     ATTR_OBJ_SIZE           2
#define     ATTR_INTEREST_ADDR_SIZE      2
#define     ATTR_INTEREST_PORT_SIZE      1
#define     ATTR_THRESHOLD_REPORT_MODE_SIZE  2 

#ifndef MAX_INTERESTED
    #define MAX_INTERESTED 5
#endif

/* Короткие адреса и порты заинтересованных сторон */


struct addr_port {
    net_addr_t  addr;
    app_port_t  port;
};

typedef struct 
{
    uint8_t state;
    uint8_t obj;
    uint16_t threshold_report_mode;
    struct addr_port interested[ MAX_INTERESTED ];
} common_attr_t;


size_t common_attr_size(common_attr_t *ca, const uint8_t attr_num);

size_t common_attr_set(common_attr_t *ca, const uint8_t attr_num, const void *const from);

size_t common_attr_get(common_attr_t *ca, const uint8_t attr_num, void *const to);

result_t send_to_interested(const uint16_t objoffset, common_attr_t *ca, const uint8_t src_port, const uint8_t msg_type, const uint8_t *const buf, size_t length);

void common_attrs_init(common_attr_t *ca);

#endif //COMMON_ATTRS_H
