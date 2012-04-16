#ifndef _FMT_H_
#define _FMT_H_

char bin2sym(unsigned char val);
//char * _i64toa(unsigned long long value, char * string, int radix);
int format_str(char * buffer, int buf_size, const char * format, ...);

#endif // _FMT_H_
