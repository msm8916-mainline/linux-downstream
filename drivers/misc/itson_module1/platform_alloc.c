/*
 * platform_alloc.c
 *
 *  Created on: Oct 11, 2011
 *      Author: pnguyen
 */
#include "platform.h"
#include "platform_alloc.h"
#include "platform_procfile.h"
#ifdef __KERNEL__
#include <linux/module.h>
#include <linux/slab.h>
#endif

uint32_t alloc_total = 0;
uint32_t alloc_cnt = 0;

uint32_t free_total = 0;
uint32_t free_cnt = 0;

/* Use for faking memory limitation */
#define TEST_MEM_DEFAULT_LIMIT 10000000
uint32_t mem_limit = TEST_MEM_DEFAULT_LIMIT;


#define MAX_ALLOC_SIZE (0x1000000)

void
itson_alloc_set_mem_limit(uint32_t new_limit)
{
	mem_limit = new_limit;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(itson_alloc_set_mem_limit);
#endif

void *
itson_alloc(uint32_t size, int can_sleep, const char *file, int line, const char *func)
{
	uint8_t *mem;
    uint32_t total_size = size;
    
	if (size > MAX_ALLOC_SIZE) {
		/* We know that our system do not allocate very large memory block, if
		 * this block is called with big number, something has gone wrong, fail it
		 */
		ERR("Trying to allocate a very large segment %u\n", size);
		return NULL;
	}

#ifdef __LINUX_UNITTESTS__
#ifdef  MEM_COUNT
    // With MEM_COUNT, we can track the amount of allocated memory.
    // However this needs space stolen from the returned buffer to
    // hold the size, and this upsets valgrind.
	if ((alloc_total + size - free_total) > mem_limit) {
		ERR("Reached memory limit allocd %d requested %d limit %d\n",
								alloc_total - free_total, size, mem_limit);
		return NULL;
	}
#else
    // Can't do as smart a check, but the limit is only used to simulate
    // out of memory conditions.
	if ((alloc_total + size) > mem_limit) {
		ERR("Reached memory limit allocd %d requested %d limit %d\n",
								alloc_total, size, mem_limit);
		return NULL;
	}
#endif
#endif

#ifdef  MEM_COUNT
    // Steal some space for the size, so free() knows how much is freed.
    // Valgrind can't handle this.
    total_size += 4;
#endif

#ifdef __KERNEL__
    if (can_sleep) {
    	mem = kzalloc(total_size, GFP_KERNEL);
    } else {
    	mem = kzalloc(total_size, GFP_ATOMIC);
    }
#endif
#ifdef __LINUX_UNITTESTS__
	mem = (uint8_t *)malloc(total_size);
#endif

	if (NULL != mem) {
		alloc_total += size;
		alloc_cnt ++;

#ifdef  MEM_COUNT
		*((uint32_t*)mem) = size;
		// DBG("allocated %s:%s:%d size %d ptr %p %p (alloc %d/%d tot %d/%d)\n", file,func, line, size, mem, (void*)(mem + 4), alloc_cnt, alloc_total, alloc_cnt-free_cnt, alloc_total-free_total);
		return (void*)(mem + 4);
#else
		return (void*)(mem);
#endif
	} else {
		ERR("Alloc failed segment %u\n", size);
	}

	return NULL;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(itson_alloc);
#endif


void
itson_free(void *mem, const char *file, int line, const char *func)
{
#ifdef  MEM_COUNT
    void *free_ptr = (((uint8_t*)mem) - 4);
	uint32_t free_sz = *(uint32_t*)free_ptr;
#else
	void *free_ptr = ((uint8_t*)mem);
	uint32_t free_sz = 16;
#endif
	free_cnt ++;
	free_total += free_sz;

#ifdef __KERNEL__
	kfree(free_ptr);
#endif
#ifdef __LINUX_UNITTESTS__
	free(free_ptr);
#endif
	// DBG("free %s:%s:%d size %d\n", file,func, line, free_total);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(itson_free);
#endif


void
itson_alloc_dump(void *ksf_handle)
{
#ifndef PURGE_LOG
#ifdef  MEM_COUNT
	itson_proc_output(ksf_handle,
		"alloc_cnt %u\nalloc_total %u\nfree_cnt %u\nfree_total %u\nmem_inuse %u\n",
		alloc_cnt, alloc_total, free_cnt, free_total, alloc_total-free_total);
#endif
#endif
}
#ifdef __KERNEL__
EXPORT_SYMBOL(itson_alloc_dump);
#endif


