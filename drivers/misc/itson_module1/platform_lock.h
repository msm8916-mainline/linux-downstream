/*
 * platform_lock.h
 *
 *  Created on: May 24, 2011
 *      Author: pnguyen
 */

#ifndef PLATFORM_LOCK_H_
#define PLATFORM_LOCK_H_


#ifdef __cplusplus
extern "C" {
#endif

struct itson_lock;

typedef struct itson_lock *itson_lock_t;


struct itson_atomic;

typedef struct itson_atomic *itson_atomic_t;


extern itson_lock_t itson_lock_create(void);

extern void itson_rd_lock(itson_lock_t lk);

extern void itson_rd_unlock(itson_lock_t lk);

extern void itson_wr_lock(itson_lock_t lk);

extern void itson_wr_unlock(itson_lock_t lk);

extern int itson_is_locked(itson_lock_t lk);

extern void itson_lock_delete(itson_lock_t lk);


#ifdef __KERNEL__

extern itson_atomic_t itson_atomic_create(int initial);

extern void itson_atomic_set(itson_atomic_t at, int val);

extern int itson_atomic_read(itson_atomic_t at);

extern void itson_atomic_inc(itson_atomic_t at);

extern void itson_atomic_dec(itson_atomic_t at);

extern void itson_atomic_delete(itson_atomic_t at);

#endif	// __KERNEL__


#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_LOCK_H_ */
