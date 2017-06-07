/*
 * platform_timestamp.c
 *
 *  Created on: Jan 19, 2015
 *      Author: Ian Smith
 */
 
#include "platform.h"
#include "platform_timestamp.h"


#ifdef __KERNEL__
/******************* Kernel space version ****************/

#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/module.h>


/**
 * Get the current time in a itson_ts_t.  This time is in ns;
 * however, the returned time will be accurate only to a
 * relatively low resolution (HZ).
 */
itson_ts_t
itson_get_timestamp(void)
{
	uint64_t jiffs = get_jiffies_64();
	return (1000000000U / HZ) * jiffs;
}
EXPORT_SYMBOL(itson_get_timestamp);


/**
 * Get the current time in a itson_ts_t.  This time is in ns;
 * it will be accurate to a higher resolution, but not necessarily
 * nanoseconds.
 */
itson_ts_t
itson_get_hi_res_ts(void)
{
	struct timespec ts;
	getnstimeofday(&ts);
	return (ts.tv_sec * NSEC_PER_SEC) + ts.tv_nsec;
}
EXPORT_SYMBOL(itson_get_hi_res_ts);


#endif	// __KERNEL__


#ifdef __LINUX_UNITTESTS__
/*******************Linux User space version ****************/

#include <time.h>


/**
 * Get the current time in a itson_ts_t.  This time is in ns;
 * however, the returned time will be accurate only to a
 * relatively low resolution (HZ).
 */
itson_ts_t
itson_get_timestamp(void)
{
	return itson_get_hi_res_ts();
}


/**
 * Get the current time in a itson_ts_t.  This time is in ns;
 * it will be accurate to a higher resolution, but not necessarily
 * nanoseconds.
 */
itson_ts_t
itson_get_hi_res_ts(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (ts.tv_sec * 1000000000) + ts.tv_nsec;
}


#endif	// __LINUX_UNITTESTS__


/**
 * Divide a given timestamp by a 64-bit value, and return the
 * 64-bit result.  Using this function avoids a 64-bit divide, which
 * is not supported on some ARM architectures at least, and not provided
 * in the kernel.
 * As a special case, divide by zero returns zero.
 */
uint64_t
itson_ts_divide_by(itson_ts_t ts, uint64_t div)
{
	// Initialize quotient and remainder to zero.
	uint64_t Q = 0;
	uint64_t R = 0;                     
	int i;
	
	if (div == 0)
		return 0;
		
	// Basic long division algorithm.
	for (i = 63; i >= 0; --i) {
		R <<= 1;
		R = (R & ~1UL) | ((ts >> i) & 0x01);
		if (R >= div) {
			R -= div;
			Q |= 1 << i;
		}
	}
	
	return Q;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(itson_ts_divide_by);
#endif	// __KERNEL__

