#include "fire_time.h"

#define  SECS_PER_HOUR  (60 * 60)
#define  SECS_PER_DAY   86400UL
//(SECS_PER_HOUR * 24)

static const unsigned char __mon0[12] =
   { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static const unsigned char __mon1[12] =
   { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

int parse_time2 (time_t t, struct tm *tp)
{
   unsigned int days,rem,m,y;
   const unsigned char *mon;
   
   days = t / SECS_PER_DAY;
   t  %= SECS_PER_DAY;
   tp->tm_hour = t / SECS_PER_HOUR;
   rem = t % SECS_PER_HOUR;
   
   tp->tm_min  = rem / 60;
   tp->tm_sec  = rem % 60;
   tp->tm_wday = (4 + days) % 7;
   
//   tp->tm_year = 70 + days / (365*4+1) * 4;
   y   = 70 + days / (365 * 4 + 1) * 4;
   rem = days % (365 * 4 + 1);
   m   = 0;
   if (rem >= 365) m++, rem -= 365;
   if (rem >= 365) m++, rem -= 365;
   if (rem >= 366) m++, rem -= 366;
   
   tp->tm_yday  = rem;
//   tp->tm_year += m;
   tp->tm_year  = y + m;
   if (m == 2)
      mon =  __mon1;
   else
      mon = __mon0;
   
   m = 0;
   while(rem >= *mon)
   {
      rem -= *mon++;
      m++;
   }
   
   tp->tm_mday = rem + 1;
   tp->tm_mon  = m;
   
   return 1;
}

int getyear2(int tm_year)
{
   if(tm_year > 99)
      tm_year -= 100;
   return tm_year;
}

int setyear2(int year)
{
   if(year < 70)
      year += 100;
   return year;
}

time_t mktime2(struct tm *timeptr)
{
   unsigned int rem, i, days;
   time_t res;
   unsigned char * day_num = __mon0;

   if(timeptr->tm_year > 138)
      return -1;
   if(timeptr->tm_mon > 11)
      return -1;
   if(timeptr->tm_hour > 23)
      return -1;
   if(timeptr->tm_min > 59)
      return -1;
   //if(timeptr->tm_sec > 59)
      timeptr->tm_sec = 0;
   
   rem  = timeptr->tm_year - 70;
   res  = rem / 4;
   rem %= 4;

   res *= 365 * 3 + 366;
//   res = mul_32x16(res, 365 * 3 + 366);
   if(rem == 1) res += 365;
   else if(rem == 2){res += 365 * 2; day_num = __mon1;}
   else if(rem == 3) res += 365 * 2 + 366;

   if((!timeptr->tm_mday) || ( timeptr->tm_mday > day_num[timeptr->tm_mon]))
      return -1;
   
   for(i = 0, days = 0; i < 12; i++)
   {
      if(i == timeptr->tm_mon)
         break;
      days += day_num[i];
   }

   days += timeptr->tm_mday - 1;

   res += days;
   res *= 24 * 60;
//   res  = mul_32x16(res, 24 * 60);
   res += timeptr->tm_hour * 60 + timeptr->tm_min;
   res = res * 60 + timeptr->tm_sec;
//   res  = mul_32x16(res, 60);
//   res += timeptr->tm_sec;

   return res;
}

