/****************************************************************************
 *        File:   fmt.c
 *     Version:
 * Created by :
 *   Reference:
 *     Purpose:
 *        Note:   
 *       To Do:
 ****************************************************************************/
/*--------------------------------------------------------------------------
 * Section: Includes
 *--------------------------------------------------------------------------*/
#include <stdarg.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "fmt.h"

/*--------------------------------------------------------------------------
 * Section: Defines
 *--------------------------------------------------------------------------*/
#define LL_SIZE_SPEC (((unsigned int)'l' << 8) | (unsigned int)'l')
#define _ui64toa      _i64toa

/*--------------------------------------------------------------------------
 * Section: Type Definitions
 *--------------------------------------------------------------------------*/
enum FormatFields
{
	fmt_flags = 0,
	fmt_width,
	fmt_precis,
	fmt_size_prefix,
	fmt_type,
	fmt_unknown
};

union Numbers
{
	unsigned char      uc;
	  signed char      sc;
	unsigned short     us;
	  signed short     ss;
	unsigned int       ui;
	  signed int       si;
	unsigned long      ul;
	  signed long      sl;
	unsigned long long ull;
	  signed long long sll;
	char               *str;
};

struct ValueFormat
{
	unsigned int  fmt;         // Format type (c, d, u, x, s)
	int           width;       // Minimum field width
	int           flag;        // Flag (+, -, 0, ' ', #)
	int           precis;      // Conversion precision (not used)
	int           size_prefix; // Value size specifier
	union Numbers value;       // Value itself
	int           len;         // Full length of the buffer
	int           pos;         // Current position
	char          *buf;        // Address of current position
};

/*--------------------------------------------------------------------------
 * Section: Globals
 *--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------
 * Section: Local Globals
 *--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------
 * Section: Function Prototypes
 *--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------
 * Section: Local Function Prototypes.
 *--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
 * Section: Function Definitions
 *--------------------------------------------------------------------------*/

char bin2sym(unsigned char val)
{
   if(val < 10)
      return '0' + val;
   else
      return 'A' + val - 0x0A;
}

char * _i64toa(unsigned long long value, char * string, int radix)
{
   unsigned char * ptr;
   char * res, * behindLastNonZero;
   int i, zeroFlag = 1;
   
   res = string;
   behindLastNonZero = string;
   if(radix == 16)
   {
      ptr = (unsigned char *)&value;
      i = 8;
      while(i--)
      {
         if(zeroFlag && (!ptr[i]))
            continue;
         zeroFlag = 0;
         *string++ = bin2sym((ptr[i] & 0xf0) >> 4);
         *string++ = bin2sym( ptr[i] & 0x0f);
         behindLastNonZero = string;
      }
   }
   *behindLastNonZero = 0;
   return res;
}

/*--------------------------------------------------------------------------
 * Function name: static const char * parse_format(const char * format, struct ValueFormat * val_fmt)
 *    returns		: address of current position in format-control string (after processing current format specifier)
 *    format		: address of current position in format-control string
 *    val_fmt		: Address of structure that will accept foramt details
 * Created by		: Maxim Mikhailov
 * Date created : 06.01.2008
 * Description  : Function parses given format specifier (pointed at by 'format') and stores the details
 *                in 'val_fmt' structure
 * Notes        : 
 *-------------------------------------------------------------------------*/
static const char * parse_format(const char * format, struct ValueFormat * val_fmt)
{
	char str[8];
	int i;
	int stage = fmt_flags;

	val_fmt->fmt = 0;
	while(*format)
	{
		// Find mandatory and optional fields if any
		switch(stage)
		{
		case fmt_flags:
			// First check for flags
			switch(*format)
			{
			case '-':
			case '+':
			case '0':
			case ' ':
			case '#':
				val_fmt->flag = *format++; // Store flag found and let next character to be checked at next stage
				break;
			case '%': // Printable percent sign is requested
				val_fmt->fmt = 0; // Show that format specifier isn't recognized
				return format;    // And let calling routine to copy that sign into result buffer
			default:
				val_fmt->flag = 0;
			}
			val_fmt->width = 0; // Prepare field width for next stage
			i = 0;              // And character counter too
			stage++; // Next stage
			break;
		case fmt_width: // Try to get field width if exists
			if(i > 7)
				i = 7; // Temporal buffer accept only 7 characters
			// Field width subfield may consist only from decimal didgits
			if(!isdigit(*format))
			{
				// This is the end of width field
				str[i] = 0; // Prepare buffer for conversion
				i = 0; // Reinitialize temporal variable
				val_fmt->width = atoi(str); // Get width value
				val_fmt->size_prefix = 0;   // Initialize value of size prefix
				val_fmt->precis = 0;
				if(*format == '.')
				{
					// If current character is '.' the next subfield means 
					// precision value
					stage =  fmt_precis;
					format++;
				}
				else
					stage = fmt_size_prefix; // No precision is given. Try to find size prefix
			}
			else
				str[i++] = *format++; // Add another digit into temporal buffer
			break;
		case fmt_precis: // Try to get precision (if exists)
			if(i > 7)
				i = 7;
			if(!isdigit(*format))
			{
				str[i] = 0;
				i = 0;
				val_fmt->precis = atoi(str);
				stage++;
			}
			else
				str[i++] = *format++;
			break;
		case fmt_size_prefix: // Try to get size prefix if exists
			if((*format == 'l') || (*format == 'L'))
			{
				format++;
				if(val_fmt->size_prefix == 'l')
				{
					val_fmt->size_prefix = (val_fmt->size_prefix << 8) | (unsigned int)'l';
					stage++;
				}
				else
					val_fmt->size_prefix = (unsigned int)'l';
			}
			else
				stage++;
			break;
		case fmt_type: // Mandatory subfield - format type
			switch(*format)
			{
			case 'c':
			case 'd':
			case 'u':
			case 'x':
			case 'X':
			case 's':
				val_fmt->fmt = *format;
				break;
			default:
				val_fmt->fmt = 0;
			}
			return ++format;
		default:
			return format;
		}
	}

	return format;
}
/* end of function parse_format() */

/*--------------------------------------------------------------------------
 * Function name: static char * i2str(struct ValueFormat * val_fmt)
 *    returns		: current position in buffer
 *    val_fmt		: structure that contains format parameters, value to convert and output buffer parameters
 * Created by		: Maxim Mikhailov
 * Date created : 06.01.2008
 * Description  : Converts given value into string according to format parameters given by 'val_fmt'
 * Notes        : 
 *-------------------------------------------------------------------------*/
static char * i2str(struct ValueFormat * val_fmt)
{
	static char cvtbuf[32];
	size_t len;
	char padSymbol;
	char sign = 0;
	int radix = 16;
	char * ptr = cvtbuf;

	// Choose symbol for padding
	if(val_fmt->flag == '0')
		padSymbol = '0';
	else
		padSymbol = ' ';

	// Check format type field
	switch(val_fmt->fmt)
	{
	case 'd': // Signed integer
		if(val_fmt->flag != '0')
		{
			// If we don't have to pad field with zeroes just convert value.
			// Later we'll do the rest of the job by stabdard algorithm
			if(val_fmt->size_prefix == LL_SIZE_SPEC)
				_i64toa(val_fmt->value.sll, cvtbuf, 10);
			else if(val_fmt->size_prefix == 'l')
				ltoa(val_fmt->value.sl, cvtbuf, 10);
			else
				itoa(val_fmt->value.si, cvtbuf, 10);
			break;
		}

		// If coversion result must be padded with zeroes ad value is negative
		// it's necessary to set sign flag and invert the value
		if(val_fmt->size_prefix == LL_SIZE_SPEC)
		{
			if(val_fmt->value.sll < 0)
			{
				sign = 1;
				val_fmt->value.sll = -val_fmt->value.sll;
			}
		}
		else if(val_fmt->size_prefix == 'l')
		{
			if(val_fmt->value.sl < 0)
			{
				sign = 1;
				val_fmt->value.sl = -val_fmt->value.sl;
			}
		}
		else
		{
			if(val_fmt->value.si < 0)
			{
				sign = 1;
				val_fmt->value.si = -val_fmt->value.si;
			}
		}

		if(sign)
		{
			// Add '-' sign in the beginning of the field
			if(val_fmt->pos >= val_fmt->len)
				break;
			*val_fmt->buf++ = '-';
			val_fmt->pos++;
			// Adjust width of the field
			if(val_fmt->width)
				val_fmt->width--;
		}
		// Now processing is the same as with unsigned integers
	case 'u':
		/*
		if(val_fmt->size_prefix == LL_SIZE_SPEC)
			_ui64toa(val_fmt->value.ull, cvtbuf, 10);
		else if(val_fmt->size_prefix == 'l')
			ultoa(val_fmt->value.ul, cvtbuf, 10);
		else
			ultoa((unsigned long)val_fmt->value.ui, cvtbuf, 10);
		break;
		*/
		// Set radix to decimal
		radix = 10;
	case 'x':
	case 'X':
		// Convert unsigned integer according to its size and requested radix
		if(val_fmt->size_prefix == LL_SIZE_SPEC)
			_ui64toa(val_fmt->value.ull, cvtbuf, radix);
		else if(val_fmt->size_prefix == 'l')
			ultoa(val_fmt->value.ul, cvtbuf, radix);
		else
			ultoa((unsigned long)val_fmt->value.ui, cvtbuf, radix);
		break;
	default: // Unsupported format. Conversion is impossible
		cvtbuf[0] = 0;
	}

	len = strlen(cvtbuf);
	if(len < (size_t)val_fmt->width)
	{
		// If length of value representation is less than field width
		// pad remaining positions with requested pad symbol
		len = val_fmt->width - len;
		while(len--)
		{
			if(val_fmt->pos >= val_fmt->len)
				break;
			*val_fmt->buf++ = padSymbol;
			val_fmt->pos++;
		}
	}

	// Copy conversion result into the buffer
	while(*ptr)
	{
		if(val_fmt->pos >= val_fmt->len)
			break;
		*val_fmt->buf++ = *ptr++;
		val_fmt->pos++;
	}

	return val_fmt->buf;
}
/* end of function i2str() */

/*--------------------------------------------------------------------------
 * Function name: int format_str(char * buffer, int buf_size, const char * format [, argument]...)
 *    returns		: Zero if succeed negative value otherwise
 *    buffer		: address of buffer that will accept formatted string
 *    buf_size		: its size
 *    format		: Format-control string
 *    argument		: Optional arguments
 * Created by		: Maxim Mikhailov
 * Date created : 06.01.2008
 * Description  : Function is similar to sprintf, but support only few formats and 
 *                takes into account buffer size
 * Notes        : 
 *-------------------------------------------------------------------------*/
int format_str(char * buffer, int buf_size, const char * format, ...)
{
	va_list arglist;
	struct ValueFormat vfmt;
	int pos = 0;

	if(buf_size <= 0)
		return -1;
	buf_size--; // For more convinient comparision
	va_start(arglist, format); // Get address of variable argument list
	while(*format) // Let's proceed with the format-control string
	{
		if(pos >= buf_size) // Check buffer bounds
			break;
		if(*format == '%') // Is the current character the format specifier sign?
		{
			// If so, parse format specifier we've just met
			format = parse_format(++format, &vfmt);
			if(vfmt.fmt) // Format specifier was successfully recognized if nonzero
			{
				// Retrieve the next argument of the type corresponds to the recognized
				// format specifier from the stack, convert it into string representation
				// and place into the buffer if fit
				switch(vfmt.fmt)
				{
				case 'c': // Just character
					vfmt.value.sc = va_arg(arglist, signed char);
					*buffer++ = vfmt.value.sc;
					pos++;
					break;
				case 'd': // Signed integer
					if(vfmt.size_prefix == LL_SIZE_SPEC)
						vfmt.value.sll = va_arg(arglist, unsigned long long);
					else if(vfmt.size_prefix == 'l')
						vfmt.value.sl = va_arg(arglist, signed long);
					else
						vfmt.value.si = va_arg(arglist, int);
					// Call conversion routine
					vfmt.buf = buffer;   // It requiers destination buffer address,
					vfmt.len = buf_size; // its size
					vfmt.pos = pos;      // and position there
					buffer = i2str(&vfmt);
					pos = vfmt.pos;      // update current position
					break;
				case 'u': // Unsigned integer
				case 'x': // Hexadecimal integer
				case 'X':
					if(vfmt.size_prefix == LL_SIZE_SPEC)
						vfmt.value.ull = va_arg(arglist, unsigned long long int);
					else if(vfmt.size_prefix == 'l')
						vfmt.value.ul = va_arg(arglist, unsigned long);
					else
						vfmt.value.ui = va_arg(arglist, unsigned int);
					// Same parameters as for signed integer
					vfmt.buf = buffer;
					vfmt.len = buf_size;
					vfmt.pos = pos;
					buffer = i2str(&vfmt);
					pos = vfmt.pos;
					break;
				case 's': // Zero-terminated string
					vfmt.value.str = va_arg(arglist, char *); // Get string address
					{
						size_t len = strlen(vfmt.value.str);  // Get its length
						if(len < (size_t)vfmt.width)
						{
							// If its length is shorter than the field width
							// we have to insert spaces befor the string into the buffer
							len = vfmt.width - len; // Number of spaces to insert
							while(len--)
							{
								if(pos >= buf_size) // Check buffer bounds
									break;
								*buffer++ = ' ';
								pos++;
							}
						}
					}
					// Copy the string into the buffer
					while(*vfmt.value.str)
					{
						if(pos >= buf_size)
							break;
						*buffer++ = *vfmt.value.str++;
						pos++;
					}
					break;
				default: // Unsupported format
					vfmt.value.ull = 0;
				}
				continue; // Check next character
			}
			// If format isn't recognized or it was double percent sign ("%%")
			// place the current symbol into the buffer
		}
		*buffer++ = *format++; // Place current character into buffer
		pos++; // Update position
	}
	va_end(arglist);
	*buffer = 0;
	return 0;
}
/* end of function format_str() */

/* end of file fmt.c */
