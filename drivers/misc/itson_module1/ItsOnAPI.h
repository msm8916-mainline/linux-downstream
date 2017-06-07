#ifndef ITSON_PUBLIC_H
#define ITSON_PUBLIC_H

#ifdef __cplusplus
extern "C" {
#endif


/* Do not modify these values, it has to match the one
 * in uid_cache.h
 */
#define UID_UNKNOWN ((~0U) -1)
#define UID_RX ((~0U) -2)

struct packetInfo
{
	uint32_t uid;
	uint32_t sock_uid;
	uint8_t *data;
	uint16_t len;
	uint8_t *l3_data;
	uint16_t l3_proto;
	const uint8_t *if_in;
	const uint8_t *if_out;

	// Netfilter linkage
	const void * packet_ptr; // sk_buff
	const void * okfn; // int (*okfn)(struct sk_buff *)
	const void * if_in_net;  // struct net_device *
	const void * if_out_net; // struct net_device *
	uint32_t     hooknum;
};

struct msgInfo
{
	uint32_t pid;
	uint32_t uid;
	uint32_t gid;
};

#define MAX_WORKERS 10


typedef void (* WorkFunc_t)(void *work);

int ItsOnInitCEA(
	int (* packetHandler)(void *packet, int hookPoint));

void 
ItsOnExitCEA(void);

int
ItsOnSendMsg(int msgType, int length, void *msg, int pid);

int
ItsOnRegisterMsgHandler(int msgType, int length,
	int (* msgHandler)(void *msg, int msgLength, struct msgInfo *info));

int
ItsOnCreateWorkerThread(WorkFunc_t worker);

int
ItsOnDoWork(int index, int delay);

void
ItsOnKillWorkerThread(int index);

int
ItsOnUserCopy(char *to, char *from, int size);

int ItsOnMarkQueued( struct packetInfo *pi );

int ItsOnReinject( struct packetInfo *pi, int verdict );

void
ItsOnGoToSleep(void);

void
ItsOnWakeUp(void);

/*
* We start out our cmd at 10 because generic netlink use the lower number
* for control
*/

enum {
	ITSON_ATTR_0 = 10,
	ITSON_ATTR_1,
	ITSON_ATTR_2,
	ITSON_ATTR_3,
	ITSON_ATTR_4,
	ITSON_ATTR_5,
    __ITSON_ATTRS_MAX,
};

#define ITSON_ATTR_MAX (__ITSON_ATTRS_MAX - 1)

/*
 * Commands to set in netlink messages for various
 * CEA-CEAA events.
 *
 * We start out our cmd at 10 because generic netlink use the lower number
 * for control
 */
enum {
	ITSON_CMD_0 = 10,
	ITSON_CMD_1,
	ITSON_CMD_2,
	ITSON_CMD_3,
	ITSON_CMD_4,
	ITSON_CMD_5,
	__ITSON_CMDS_MAX,
};
#define ITSON_CMD_MAX (__ITSON_CMDS_MAX - 1)
#define VERSION_GNL 1

#ifdef __cplusplus
}
#endif

#endif // ITSON_PUBLIC_H

