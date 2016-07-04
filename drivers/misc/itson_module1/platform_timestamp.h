/*
 * platform_lock.h
 *
 *  Created on: Jan 19, 2015
 *      Author: Ian Smith
 */

#ifndef PLATFORM_TIMESTAMP_H_
#define PLATFORM_TIMESTAMP_H_


#ifdef __cplusplus
extern "C" {
#endif

// Timestamp, in nanoseconds.  Not necessarily fetched with
// nanosecond resolution.
typedef uint64_t itson_ts_t;

#define TS_TICK_PER_SEC 1000000000UL


extern itson_ts_t itson_get_timestamp(void);

extern itson_ts_t itson_get_hi_res_ts(void);

extern uint64_t itson_ts_divide_by(itson_ts_t ts, uint64_t div);


#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_TIMESTAMP_H_ */
