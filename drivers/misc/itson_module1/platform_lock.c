/*
 * platform_lock.c
 *
 *  Created on: Jan 12, 2015
 *      Author: Ian Smith
 */
 
#include "platform.h"
#include "platform_lock.h"


#ifdef __KERNEL__
/******************* Kernel space version ****************/

#include <linux/ctype.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <asm/atomic.h>


struct itson_lock {
	spinlock_t linux_lock;
};


struct itson_atomic {
	atomic_t linux_atomic;
};


itson_lock_t
itson_lock_create()
{
	/* Try to allocate the lock... if it fails, return 0. */
	itson_lock_t lock = (itson_lock_t) MALLOC(sizeof(struct itson_lock));
	if (!lock)
		return 0;
		
	/* Ask Linux to initialise the lock. */
	spin_lock_init(&lock->linux_lock);
	
	return lock;
}
EXPORT_SYMBOL(itson_lock_create);


void
itson_rd_lock(itson_lock_t lk)
{
	if (lk != NULL)
		spin_lock_bh(&lk->linux_lock);
}
EXPORT_SYMBOL(itson_rd_lock);


void
itson_rd_unlock(itson_lock_t lk)
{
	if (lk != NULL)
		spin_unlock_bh(&lk->linux_lock);
}
EXPORT_SYMBOL(itson_rd_unlock);


void
itson_wr_lock(itson_lock_t lk)
{
	if (lk != NULL)
		spin_lock_bh(&lk->linux_lock);
}
EXPORT_SYMBOL(itson_wr_lock);


void
itson_wr_unlock(itson_lock_t lk)
{
	if (lk != NULL)
		spin_unlock_bh(&lk->linux_lock);
}
EXPORT_SYMBOL(itson_wr_unlock);

int
itson_is_locked(itson_lock_t lk)
{
    return(lk != NULL ? spin_is_locked(&lk->linux_lock) : 0);
}
EXPORT_SYMBOL(itson_is_locked);


void
itson_lock_delete(itson_lock_t lk)
{
	if (lk != NULL)
		FREE(lk);
}
EXPORT_SYMBOL(itson_lock_delete);


itson_atomic_t
itson_atomic_create(int initial)
{
	/* Try to allocate the atomic... if it fails, return 0. */
	itson_atomic_t atomic = (itson_atomic_t) MALLOC(sizeof(struct itson_atomic));
	if (!atomic)
		return 0;
		
	/* Ask Linux to initialise the atomic. */
	atomic_set(&atomic->linux_atomic, initial);
	
	return atomic;
}
EXPORT_SYMBOL(itson_atomic_create);


void
itson_atomic_set(itson_atomic_t at, int val)
{
	atomic_set(&at->linux_atomic, val);
}
EXPORT_SYMBOL(itson_atomic_set);


int
itson_atomic_read(itson_atomic_t at)
{
	return atomic_read(&at->linux_atomic);
}
EXPORT_SYMBOL(itson_atomic_read);


void
itson_atomic_inc(itson_atomic_t at)
{
	atomic_inc(&at->linux_atomic);
}
EXPORT_SYMBOL(itson_atomic_inc);


void
itson_atomic_dec(itson_atomic_t at)
{
	atomic_dec(&at->linux_atomic);
}
EXPORT_SYMBOL(itson_atomic_dec);


void
itson_atomic_delete(itson_atomic_t at)
{
	if (at != NULL)
		FREE(at);
}
EXPORT_SYMBOL(itson_atomic_delete);


#endif	// __KERNEL__


#ifdef __LINUX_UNITTESTS__
/*******************Linux User space version ****************/

#include <pthread.h>


struct itson_lock {
	pthread_spinlock_t pthread_lock;
};


itson_lock_t
itson_lock_create()
{
	/* Try to allocate the lock... if it fails, return 0. */
	itson_lock_t lock = (itson_lock_t) MALLOC(sizeof(struct itson_lock));
	if (!lock)
		return 0;
		
	/* Ask Linux to initialise the lock. */
	pthread_spin_init(&lock->pthread_lock, PTHREAD_PROCESS_PRIVATE);
	
	return lock;
}


void
itson_rd_lock(itson_lock_t lk)
{
	if (lk != NULL)
		pthread_spin_lock(&lk->pthread_lock);
}


void
itson_rd_unlock(itson_lock_t lk)
{
	if (lk != NULL)
		pthread_spin_unlock(&lk->pthread_lock);
}


void
itson_wr_lock(itson_lock_t lk)
{
	if (lk != NULL)
		pthread_spin_lock(&lk->pthread_lock);
}


void
itson_wr_unlock(itson_lock_t lk)
{
	if (lk != NULL)
		pthread_spin_unlock(&lk->pthread_lock);
}

int
itson_is_locked(itson_lock_t lk)
{
    if (lk == NULL)
        return(0);

    // This should fail with EBUSY
    if (pthread_spin_trylock(&lk->pthread_lock) == EBUSY) 
        return(1);

    pthread_spin_unlock(&lk->pthread_lock);
    return(0);
}


void
itson_lock_delete(itson_lock_t lk)
{
	if (lk != NULL)
		FREE(lk);
}


#endif	// __LINUX_UNITTESTS__

