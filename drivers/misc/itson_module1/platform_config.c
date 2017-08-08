/*
 * platform_config.c
 *
 *  Created on: Jan 27, 2015
 *      Author: Ian Smith
 */
 
#include "platform.h"
#include "platform_config.h"
#ifdef __KERNEL__
#include <linux/module.h>
#include <linux/mm.h>
#else
#define	EXPORT_SYMBOL(x)
#endif


// Static arrays of uids which we grant various access levels to
#ifndef ITSON_SECURE_LEVEL1_UID
#error You must define the ITSON_LEVEL1_UID array
#endif
#ifndef ITSON_SECURE_LEVEL2_UID
#error You must define the ITSON_LEVEL2_UID array
#endif

static unsigned int level1_uids[] = ITSON_SECURE_LEVEL1_UID;
static unsigned int level2_uids[] = ITSON_SECURE_LEVEL2_UID;


int
itson_get_ram_mb(void)
{
    int megs;
    
#ifdef __KERNEL__
    /*
     * Find out how much RAM the device has.  Bear in mind, though,
     * that the answer may be less than the full physical RAM; some RAM
     * might be reserved for e.g. video or DRM.  For example, Nexus 5
     * reports 1855 MB -- 193MB short of the true amount.
     */
    struct sysinfo i;
    
    si_meminfo(&i);
    
    megs = (i.totalram << (PAGE_SHIFT - 10)) / 1024;
#else
    // Unit tests etc -- just assume large mem.
    megs = 1024;
#endif
    
    return megs;
}
EXPORT_SYMBOL(itson_get_ram_mb);


int
itson_conf_get_uids(unsigned int *l1[], int *l1_len, unsigned int *l2[], int *l2_len)
{
    *l1 = level1_uids;
    *l1_len = sizeof(level1_uids) / sizeof(level1_uids[0]);
    *l2 = level2_uids;
    *l2_len = sizeof(level2_uids) / sizeof(level2_uids[0]);
    return 0;
}
EXPORT_SYMBOL(itson_conf_get_uids);

