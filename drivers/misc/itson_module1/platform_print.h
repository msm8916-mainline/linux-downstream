/*
 * platform_lock.h
 *
 *  Created on: Jan 19, 2015
 *      Author: Ian Smith
 */

#ifndef PLATFORM_PRINT_H_
#define PLATFORM_PRINT_H_


#ifdef __cplusplus
extern "C" {
#endif


extern int itson_printf(const char *format, ...);

extern long itson_strtol(const char *cp, char **endp, unsigned int base);


#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_PRINT_H_ */
