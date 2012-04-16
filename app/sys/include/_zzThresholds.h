#ifndef __ZZ_THRESHOLDS_H
#define __ZZ_THRESHOLDS_H

#include <zzTypes.h>
#include <_zzCommonAttrs.h>

void _attr_on_change(uint16_t objoffset, common_attr_t *ca, uint8_t attr_num);

typedef uint16_t (* isGT_ft )(uint8_t attr_num1, uint8_t attr_num2);

const extern isGT_ft _begin_isGT;

#endif // __ZZ_THRESHOLDS_H
