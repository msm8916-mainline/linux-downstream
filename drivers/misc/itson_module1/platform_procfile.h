/*
 * platform_lock.h
 *
 *  Created on: Jan 19, 2015
 *      Author: Ian Smith
 */

#ifndef PLATFORM_PROCFILE_H_
#define PLATFORM_PROCFILE_H_


#ifdef __cplusplus
extern "C" {
#endif


extern int
itson_proc_output(void *ksf_handle, const char *format, ...);


#ifdef __KERNEL__


struct file;


extern int
ItsOnCreateKernelFolder(char *name);

extern int
ItsOnCreateKernelFile(char *name, void *readFunc, void *writeFunc, unsigned short perm);

extern int
ItsOnCreateKernelFileSimple(char *name, int (*showFunc)(void *, void *), unsigned int (*writeFunc)(struct file *, const char *, size_t, void *), unsigned short perm);

extern void
ItsOnDeleteKernelFile(char *name);


#endif	// __KERNEL__


#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_PROCFILE_H_ */
