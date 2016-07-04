/*
 * net_platform.h
 *
 *  Created on: Sep 3, 2010
 *      Author: Phuong Nguyen
 *
 *  Keep macro that differentiate platforms here
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "platform_lock.h"
#include "platform_alloc.h"


#ifdef __KERNEL__
/******************* Kernel space version ****************/

// In the kernel version, we need to include these headers, since
// the normal user-space headers (stdint.h, etc) don't exist.
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/in.h>

#define DUMP_STACK dump_stack()
#define BOOL bool
#define UCHAR unsigned char
#define UINT16 unsigned short
#define UINT32 unsigned int
#define ULONG unsigned int
#define INT32 int
#define UINT64 unsigned long long
#define NTSTATUS unsigned int
#define LARGE_INTEGER long long
#define NET_BUFFER_LIST void
#define ExAllocatePoolWithTag MALLOC
#define PVOID void*
#define VOID void
#define InitializeListHead LIST_HEAD
#define PITSON_DEVICE_EXT void*
#define IN
#define OUT
#define PItsOnDriverConnectionData void *
#define Direction unsigned int
#define STATUS_SUCCESS 0
#define DbgPrint DBG
#define size_t long
//#define PNetBufferListHolder void*
#define FWPS_STREAM_CALLOUT_IO_PACKET0 void
#define FWPS_STREAM_DATA0 void

#ifndef IPPROTO_TCP
#define IPPROTO_TCP (6)
#define IPPROTO_UDP (17)
#define	IPPROTO_ICMP (1)
#endif

#define ETH_P_IP	0x0800
#define ETH_P_IPV6	0x86DD


struct iphdr {
	//big endian, network byte order
	//uint8_t		version:4, ihl:4;

	//small endian
	uint8_t		ihl:4, version:4;

	uint8_t		tos;
	uint16_t	tot_len;
	uint16_t	id;
	uint16_t	frag_off;
	uint8_t		ttl;
	uint8_t		protocol;
	uint16_t	check;
	uint32_t	saddr;
	uint32_t	daddr;
};


struct icmphdr {
	uint8_t          type;
	uint8_t          code;
	uint16_t       checksum;
	union {
        struct {
                uint16_t  id;
                uint16_t  sequence;
        } echo;
        uint32_t  gateway;
        struct {
                uint16_t  __unused;
                uint16_t  mtu;
        } frag;
	} un;
};

#define ICMP_DEST_UNREACH       3       /* Destination Unreachable      */

/* Codes for UNREACH. */
#define ICMP_NET_UNREACH        0       /* Network Unreachable          */
#define ICMP_HOST_UNREACH       1       /* Host Unreachable             */
#define ICMP_PROT_UNREACH       2       /* Protocol Unreachable         */
#define ICMP_PORT_UNREACH       3       /* Port Unreachable             */
#define ICMP_FRAG_NEEDED        4       /* Fragmentation Needed/DF set  */
#define ICMP_SR_FAILED          5       /* Source Route failed          */


struct tcphdr
{
	uint16_t source;
	uint16_t dest;
	uint32_t seq;
	uint32_t ack_seq;


	//big endian, network byte order
#if 0
	uint16_t doff:4,
			res1:4,
			res2:2,
			urg:1,
			ack:1,
			psh:1,
			rst:1,
			syn:1,
			fin:1;
#endif
#if 1
	//small endian, network byte order
	uint16_t res1:4,
		doff:4,
		fin:1,
		syn:1,
		rst:1,
		psh:1,
		ack:1,
		urg:1,
		res2:2;
#endif

    uint16_t window;
    uint16_t check;
    uint16_t urg_ptr;
};

struct udphdr {
	uint16_t	source;
	uint16_t	dest;
	uint16_t	len;
	uint16_t	check;
};


#else

/******************* User space version ****************/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <ctype.h>

//#include <linux/types.h>

#include <asm/byteorder.h>
#include <signal.h>

#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <netinet/ip_icmp.h>

#include <netinet/if_ether.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define PItsOnDriverConnectionData void*

#ifndef __cplusplus
typedef uint8_t bool;
#define true 1
#define false 0
#endif //c++

#define u32 uint32_t
#define u8 uint8_t

#define simple_strtol strtol

#if 0
#define AF_INET		2	/* Internet IP Protocol 	*/
/* Standard well-defined IP protocols.  */
enum  {
    IPPROTO_TCP = 6,	   /* Transmission Control Protocol.  */
    IPPROTO_UDP = 17,	   /* User Datagram Protocol.  */
  };
#endif

#define DUMP_STACK  raise(SIGSEGV) /* Crash unit test so we can force a fix */


#endif	// __LINUX_UNITTESTS__


/***********************  Common Defines ********************/

#define MALLOC(X) itson_alloc(X, 0, __FILE__, __LINE__, __func__)
/* Only call this from a context that can sleep */
#define MALLOC_SLEEP(X) itson_alloc(X, 1, __FILE__, __LINE__, __func__)
#define FREE(X) itson_free(X, __FILE__, __LINE__, __func__)


#include "platform_print.h"

#define DBG(X...)  itson_printf(X)
#define INFO(X...) itson_printf(X)
#define ERR(X...) itson_printf(X)


#ifdef __cplusplus
}
#endif

#endif /* _PLATFORM_H_ */
