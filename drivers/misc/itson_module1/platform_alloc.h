/*
 * platform_alloc.h
 *
 *  Created on: Oct 12, 2011
 *      Author: pnguyen
 */

#ifndef PLATFORM_ALLOC_H_
#define PLATFORM_ALLOC_H_

#ifdef __cplusplus
extern "C" {
#endif

extern void
itson_alloc_set_mem_limit(unsigned int new_limit);

extern void *
itson_alloc(unsigned int size, int can_sleep, const char *file, int line, const char *func);

extern void
itson_free(void *mem, const char *file, int line, const char *func);

extern void
itson_alloc_dump(void *ksf_handle);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_ALLOC_H_ */
