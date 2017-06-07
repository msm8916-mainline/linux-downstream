/*
 * platform_timer.c
 *
 *  Created on: Jan 9, 2015
 *      Author: Ian Smith
 */
 
#include "platform.h"
#include "platform_timer.h"


#ifdef __KERNEL__
/******************* Kernel space version ****************/

#include <linux/module.h>
#include <linux/timer.h>


struct itson_timer {
	struct timer_list linux_timer;
};


itson_timer_t
itson_timer_create(void (*callback)(unsigned long), unsigned long data)
{
	/* Try to allocate the timer... if it fails, return 0. */
	itson_timer_t timer = (itson_timer_t) MALLOC(sizeof(struct itson_timer));
	if (!timer)
		return 0;
		
	/* Ask Linux to initialise the timer. */
	setup_timer(&timer->linux_timer, callback, data);
	
	return timer;
}
EXPORT_SYMBOL(itson_timer_create);


/**
 * Modify the given timer to go off in the given number of milliseconds
 * from now.
 *
 * @param	timer		The timer to modify.
 * @param	msec		The time in ms from now at which it should fire.
 * @return				0 on success; -1 on error.
 */
int
itson_timer_set(itson_timer_t timer, unsigned long msec)
{
	unsigned long jif_time = jiffies + msecs_to_jiffies(msec);

	/*
	 * The function returns whether it has modified a pending timer
	 * or not. (ie. mod_timer of an inactive timer returns 0, mod_timer
	 * of an active timer returns 1.)
	 */
	mod_timer(&timer->linux_timer, jif_time);
	
	return 0;
}
EXPORT_SYMBOL(itson_timer_set);


void
itson_timer_stop(itson_timer_t timer)
{
	del_timer(&timer->linux_timer);
}
EXPORT_SYMBOL(itson_timer_stop);


int
itson_timer_pending(itson_timer_t timer)
{
	return timer_pending(&timer->linux_timer);
}
EXPORT_SYMBOL(itson_timer_pending);


void
itson_timer_delete(itson_timer_t timer)
{
	FREE(timer);
}
EXPORT_SYMBOL(itson_timer_delete);


#else //not Kernel
/******************* User space version ****************/
/* Configure this differently depending on environment */

#include <signal.h>
#include <time.h>


struct itson_timer {
	timer_t timer;
	struct sigevent sigev;
	struct itimerspec itval;
	struct sigaction sa;
	void (*callback)(unsigned long);
	unsigned long data;
};


static void
sigactionHandler(int signum, siginfo_t *info, void *context)
{
	itson_timer_t timer = (itson_timer_t) info->si_value.sival_ptr;
	(*timer->callback)(timer->data);
}


// The signal we use for the timer in linux unit tests.  Note that
// SIGRTMAX clashes with valgrind.
#define TIMER_SIGNAL    (SIGRTMAX - 1)

itson_timer_t
itson_timer_create(void (*callback)(unsigned long), unsigned long data)
{
	/* Try to allocate the timer... if it fails, return 0. */
	itson_timer_t timer = (itson_timer_t) MALLOC(sizeof(struct itson_timer));
	if (!timer) {
        DBG("Timer memory allo failed.\n");
		return 0;
	}
		
	/*
	 * Set up a handler for the TIMER_SIGNAL signal.  The SA_SIGINFO flag
	 * specifies that the 3-argument signal handler will be called, and
	 * the second argument will be a siginfo_t pointer.
	 */
    sigemptyset(&timer->sa.sa_mask);
    timer->sa.sa_sigaction = &sigactionHandler;
    timer->sa.sa_flags = SA_SIGINFO;
    if (sigaction(TIMER_SIGNAL, &timer->sa, NULL) < 0) {
        DBG("sigaction error");
    	FREE(timer);
        return 0;
    }
    
    /*
     * Set up a timer, and tell it to notify us via a TIMER_SIGNAL signal.
     * The sigev_value we set here will be passed back in the siginfo_t
     * structure that is passed as the second argument of the handler.
     */
    memset(&timer->sigev, 0, sizeof(timer->sigev));
    timer->sigev.sigev_notify          = SIGEV_SIGNAL;
    timer->sigev.sigev_signo           = TIMER_SIGNAL;
    timer->sigev.sigev_value.sival_ptr = timer;
    if (timer_create(CLOCK_REALTIME, &timer->sigev, &timer->timer) == 0) {
        DBG("Timer created\n");
        timer->itval.it_value.tv_sec = 0;
        timer->itval.it_value.tv_nsec = 0;
        timer->itval.it_interval.tv_sec = 0; 
        timer->itval.it_interval.tv_nsec = 0;
    } else {
    	DBG("Timer_create failed.  SEVERE ERROR");
    	FREE(timer);
		return 0;
	}
	
	timer->callback = callback;
	timer->data = data;
	return timer;
}


/**
 * Modify the given timer to go off in the given number of milliseconds
 * from now.
 *
 * @param	timer		The timer to modify.
 * @param	msec		The time in ms from now at which it should fire.
 * @return				0 on success; -1 on error.
 */
int
itson_timer_set(itson_timer_t timer, unsigned long msec)
{
    if (NULL == timer) {
    	DBG("NULL timer passed to __FUNC__.  SEVERE ERROR");
		return -1;
	}

    timer->itval.it_value.tv_sec = msec / 1000000000;
    timer->itval.it_value.tv_nsec = msec % 1000000000;
    timer->itval.it_interval.tv_sec = 0;
    timer->itval.it_interval.tv_nsec = 0;
    
    /*
     * On success, timer_settime() returns 0.  On error,
     * -1 is returned, and errno is set to indicate the error.
	 */
    return timer_settime(timer->timer, 0, &timer->itval, NULL);
}


void
itson_timer_stop(itson_timer_t timer)
{
    if (NULL == timer) {
    	DBG("NULL timer passed to __FUNC__.  SEVERE ERROR");
		return;
	}

	timer->itval.it_value.tv_sec = 0;
	timer->itval.it_value.tv_nsec = 0;
	timer->itval.it_interval.tv_sec = 0;
	timer->itval.it_interval.tv_nsec = 0;
}


int
itson_timer_pending(itson_timer_t timer)
{
    if (NULL == timer) {
    	DBG("NULL timer passed to __FUNC__.  SEVERE ERROR");
		return 0;
	}

	timer_gettime(&timer->timer, &timer->itval);

	return timer->itval.it_value.tv_sec != 0L || timer->itval.it_value.tv_nsec != 0L;
}


void
itson_timer_delete(itson_timer_t timer)
{
    if (NULL == timer) {
    	DBG("NULL timer passed to __FUNC__.  SEVERE ERROR");
		return;
	}

	FREE(timer);
}


#endif //not Kernel
