/*
 * platform_timestamp.c
 *
 *  Created on: Jan 19, 2015
 *      Author: Ian Smith
 */
 
#include "platform.h"
#include "platform_print.h"


#ifdef __KERNEL__
/******************* Kernel space version ****************/


#include <linux/module.h>


int
itson_printf(const char *format, ...)
{
	va_list ap;
	int stat;
	
    va_start(ap, format);
    stat = vprintk(format, ap);
    va_end(ap);

	return stat;
}
EXPORT_SYMBOL(itson_printf);


long
itson_strtol(const char *cp, char **endp, unsigned int base)
{
	return simple_strtol(cp, endp, base);
}
EXPORT_SYMBOL(itson_strtol);


#endif	// __KERNEL__


#ifdef __LINUX_UNITTESTS__
/*******************Linux User space version ****************/


int
itson_printf(const char *format, ...)
{
	va_list ap;
	int stat;
	
    va_start(ap, format);
    stat = vprintf(format, ap);
    va_end(ap);

	return stat;
}


long
itson_strtol(const char *cp, char **endp, unsigned int base)
{
	return strtol(cp, endp, base);
}


#endif	// __LINUX_UNITTESTS__

