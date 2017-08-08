/*
 * platform_timer.h
 *
 *  Created on: Jan 9, 2015
 *      Author: Ian Smith
 */

#ifndef _PLATFORM_TIMER_H
#define _PLATFORM_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif


struct itson_timer;

typedef struct itson_timer *itson_timer_t;


extern itson_timer_t
itson_timer_create(void (*callback)(unsigned long), unsigned long data);

extern int
itson_timer_set(itson_timer_t timer, unsigned long msec);

extern void
itson_timer_stop(itson_timer_t timer);

extern int
itson_timer_pending(itson_timer_t timer);

extern void
itson_timer_delete(itson_timer_t timer);


#ifdef __cplusplus
}
#endif

#endif	/* _PLATFORM_TIMER_H */

