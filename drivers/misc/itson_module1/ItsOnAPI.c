/*
 * ItsOnAPI.c
 *
 *  Created on: Nov 26, 2010
 *      Author: Lisa Stark-Berryman
 *
 *      Linux platform-specific code for CEA port to Linux platform(s)
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <net/genetlink.h>
#include <linux/cred.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/icmp.h>
#include <linux/version.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <net/netfilter/nf_queue.h>

#ifdef CONFIG_BRIDGE_NETFILTER
#include <linux/netfilter_bridge.h>
#endif


#include "ItsOnAPI.h"

#define ITSON_HOOK_RECEIVE  0x01
#define ITSON_HOOK_TRANSMIT 0x02
#define ITSON_HOOK_FORWARD  0x04
#define ITSON_TX_QUEUE_LIMIT 256
#define ITSON_RX_QUEUE_LIMIT 256
#define ITSON_MAX_INDEX 256
	
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Author");
MODULE_DESCRIPTION("Description");


#undef ERR
#define ERR(X...) printk(KERN_ERR X)
#undef INFO
#define INFO(X...) printk(KERN_INFO X)
#undef DBG
#define DBG(X...)

static unsigned int itson_rx(unsigned int hooknum,  
                  struct sk_buff *skb,
                  const struct net_device *in,
                  const struct net_device *out,
                  int (*okfn)(struct sk_buff*));

static unsigned int itson_tx(unsigned int hooknum,  
                  struct sk_buff *skb,
                  const struct net_device *in,
                  const struct net_device *out,
                  int (*okfn)(struct sk_buff*));
#if 0
static unsigned int itson_fwd(unsigned int hooknum,
                  struct sk_buff *skb,
                  const struct net_device *in,
                  const struct net_device *out,
                  int (*okfn)(struct sk_buff*));
#endif
static struct nf_hook_ops itson_ops[] = {
        {
                .hook           = itson_rx,
                .owner          = THIS_MODULE,
                .pf             = PF_INET,
                .hooknum        = NF_INET_PRE_ROUTING,
                .priority       = NF_IP_PRI_FIRST,
        },
        /* Don't activate this hook, but leave it for future tethering investigation
        {
                .hook           = itson_fwd,
                .owner          = THIS_MODULE,
                .pf             = NFPROTO_IPV4,
                .hooknum        = NF_INET_FORWARD,
                .priority       = NF_IP_PRI_FIRST,
        },
        */        {
                .hook           = itson_tx,
                .owner          = THIS_MODULE,
                .pf             = PF_INET,
                .hooknum        = NF_INET_POST_ROUTING,
                .priority       = NF_IP_PRI_FIRST,
        },
};


static int
itson_rcv_msg(struct sk_buff *skb_2, struct genl_info *info);

static struct nla_policy itson_genl_policy[ITSON_ATTR_MAX + 1] =
{
	[ITSON_ATTR_1] = { .type = NLA_BINARY, .len = 100 },
	[ITSON_ATTR_2] = { .type = NLA_BINARY, .len = 100 },
	[ITSON_ATTR_3] = { .type = NLA_BINARY, .len = 100 },
	[ITSON_ATTR_4] = { .type = NLA_BINARY, .len = 100 },
	[ITSON_ATTR_5] = { .type = NLA_BINARY, .len = 100 },
};

/* family definition */
static struct genl_family itson_gnl_family =
{
	.id = GENL_ID_GENERATE,         //genetlink should generate an id
	.hdrsize = 0,
	.name = "ITSON_COMM",        //the name of this family, used by userspace application
	.version = VERSION_GNL,                   //version number
	.maxattr = ITSON_ATTR_MAX,
};

/* commands: mapping between the command enumeration and the actual function*/
struct genl_ops itson_gnl_ops_cmds[] = {
	{
		.cmd = CTRL_CMD_GETFAMILY,
		.flags = 0,
		.policy = itson_genl_policy,
		.doit = itson_rcv_msg,
		.dumpit = NULL,
	},
	{
		.cmd = ITSON_CMD_1,
		.flags = 0,
		.policy = itson_genl_policy,
		.doit = itson_rcv_msg,
		.dumpit = NULL,
	},
	{
		.cmd = ITSON_CMD_2,
		.flags = 0,
		.policy = itson_genl_policy,
		.doit = itson_rcv_msg,
		.dumpit = NULL,
	},
	{
		.cmd = ITSON_CMD_3,
		.flags = 0,
		.policy = itson_genl_policy,
		.doit = itson_rcv_msg,
		.dumpit = NULL,
	},
	{
		.cmd = ITSON_CMD_4,
		.flags = 0,
		.policy = itson_genl_policy,
		.doit = itson_rcv_msg,
		.dumpit = NULL,
	}
};

typedef int (* pMsgHandler)(void *msg, int msgLength, struct msgInfo *info);
pMsgHandler msgHandlerList[ITSON_CMD_MAX];

struct delayed_work workers[MAX_WORKERS];
int worker_count = 0;


static int
itson_rcv_msg(struct sk_buff *skb_2, struct genl_info *info)
{
	struct nlattr *na;
	struct msgInfo minfo;
	int rc = 0;
	void *msg;
	int index;

	if (info == NULL) {
		rc = -EINVAL;
		goto rcv_msg_err;
	}
	index = info->genlhdr->cmd;

	/* Ignore control command */
	if (index < CTRL_CMD_MAX) {
		INFO("Received control msg %d, skipping\n", index);
		return 0;
	}

	// Ignore out-of-range commands (not ours, so print an error).
	if (index < ITSON_CMD_0 || index > ITSON_CMD_MAX) {
		ERR("Received out-of-range netlink message %d, skipping\n", index);
		return 0;
	}


	//INFO("%s cmd %d", __func__, index);
	na = info->attrs[index];
	if (na == NULL) {
		printk("no info->attrs %i\n", index);
		rc = -EINVAL;
		goto rcv_msg_err;
	}

	msg = nla_data(na);
	if (msg == NULL) {
		printk("error while receiving data\n");
		rc = -EINVAL;
		goto rcv_msg_err;

	}

	//INFO("cmd %d len %d\n", index, na->nla_len);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
	minfo.pid = info->snd_portid;
#else
	minfo.pid = info->snd_pid;
#endif
	minfo.uid = NETLINK_CB(skb_2).creds.uid;
	minfo.gid = NETLINK_CB(skb_2).creds.gid;
	if (msgHandlerList[index]) {
		rc = msgHandlerList[index](msg, nla_len(na), &minfo);
	} else {
		ERR("No handler for found cmd %d", index);
		rc = -EINVAL;
		goto rcv_msg_err;
	}


rcv_msg_err:
	return rc;
}

int (* packetCallout)(void *packet, int hookPoint);

static void
init_pak_ext(struct sk_buff *skb, const struct net_device *if_in, 
		const struct net_device *if_out, 
		int (*okfn)(struct sk_buff *), unsigned int hooknum,
		struct packetInfo *pak, int uid, int sock_uid)
{
	*pak = (struct packetInfo) {
		.packet_ptr = skb,
		.okfn       = okfn,
		.if_in_net  = if_in,
		.if_out_net = if_out,
		.hooknum    = hooknum,

		.data       = skb->data,
		.len        = skb->len,
		.l3_data    = skb_network_header(skb),
		.l3_proto   = skb->protocol,
		.if_in      = if_in->name,
		.if_out     = if_out->name,
		.uid        = uid,
		.sock_uid   = sock_uid,
	};
}


#if KERNELVERSION==3

/**
 * Send a reset.
 */
static void reset_connection(int hooknum, struct sk_buff *skb_orig)
{
	struct tcphdr tcp_hdr_buf;
	const struct iphdr *ip_hdr_orig;
	const struct tcphdr *tcp_hdr_orig;
	struct sk_buff *skb_reset;
	struct iphdr *ip_hdr_rst;
	struct tcphdr *tcp_hdr_rst;
//	printk("[%x] start len %d\n", skb_orig, skb_orig->len);

	if (ip_hdr(skb_orig)->frag_off & htons(IP_OFFSET))
		return;

//	printk("[%x] frag ", skb_orig);
	tcp_hdr_orig = skb_header_pointer(skb_orig, ip_hdrlen(skb_orig),
				 					  sizeof(tcp_hdr_buf), &tcp_hdr_buf);
//	printk("\n[%x] tcp ", skb_orig);
	if (tcp_hdr_orig == NULL) {
		DBG("[%x] -> skip no hdr \n", skb_orig);
		return;
	}
	
	/* If the original was a reset, don't respond in kind. */
	if (tcp_hdr_orig->rst) {
//		printk("[%x] -> skip rst \n", skb_orig);
		return;
	}
	
//	printk("[%x] rst %x", skb_orig, (struct rtable *)
	if ((skb_orig->_skb_refdst & SKB_DST_PTRMASK) == 0) {
		DBG("[%x] -> skip bad pointer \n", skb_orig);
		return;
	}

	if (skb_rtable(skb_orig)->rt_flags & (RTCF_BROADCAST | RTCF_MULTICAST)) {
		DBG("[%x] -> skip broadcast \n", skb_orig);
		return;
	}

//	printk("[%x] broadcast \n", skb_orig);
	/* Validate the checksum */
	if (nf_ip_checksum(skb_orig, hooknum, ip_hdrlen(skb_orig), IPPROTO_TCP)) {
		DBG("[%x] -> skip checksum \n", skb_orig);
		return;
	}

//	printk("[%x] checksum ", skb_orig);
	ip_hdr_orig = ip_hdr(skb_orig);

	/* Create an SKB for the reset. */
	skb_reset = alloc_skb(sizeof(struct iphdr) + sizeof(struct tcphdr) +
			 			  LL_MAX_HEADER, GFP_ATOMIC);
//	printk("[%x] new skb -> %x ", skb_orig, skb_reset);
	if (!skb_reset)
		return;

	skb_reserve(skb_reset, LL_MAX_HEADER);

	/* Set up the IP header. */
	skb_reset_network_header(skb_reset);
	ip_hdr_rst = (struct iphdr *) skb_put(skb_reset, sizeof(struct iphdr));
	ip_hdr_rst->version = 4;
	ip_hdr_rst->ihl = sizeof(struct iphdr) / 4;
	ip_hdr_rst->tos = 0;
	ip_hdr_rst->id = 0;
	ip_hdr_rst->frag_off = htons(IP_DF);
	ip_hdr_rst->protocol = IPPROTO_TCP;
	ip_hdr_rst->check = 0;
	ip_hdr_rst->saddr = ip_hdr_orig->daddr;
	ip_hdr_rst->daddr = ip_hdr_orig->saddr;

	/* Set up the TCP header. */
	tcp_hdr_rst = (struct tcphdr *) skb_put(skb_reset, sizeof(struct tcphdr));
//	INFO("here 2 ---");
	memset(tcp_hdr_rst, 0, sizeof(struct tcphdr));
	tcp_hdr_rst->source = tcp_hdr_orig->dest;
	tcp_hdr_rst->dest = tcp_hdr_orig->source;
	tcp_hdr_rst->doff = sizeof(struct tcphdr) / 4;
	tcp_hdr_rst->seq = tcp_hdr_orig->ack_seq;
	tcp_hdr_rst->ack_seq = htonl(ntohl(tcp_hdr_orig->seq) +
								 tcp_hdr_orig->syn + 
								 tcp_hdr_orig->fin +
								 skb_orig->len -
								 ip_hdrlen(skb_orig) -
								 (tcp_hdr_orig->doff << 2));
	tcp_hdr_rst->ack = 1;
	tcp_hdr_rst->rst = 1;
	tcp_hdr_rst->check = ~tcp_v4_check(sizeof(struct tcphdr), ip_hdr_rst->saddr,
				    				   ip_hdr_rst->daddr, 0);

	//	INFO("here 3 ---");
	skb_reset->ip_summed = CHECKSUM_PARTIAL;
	skb_reset->csum_start = (unsigned char *) tcp_hdr_rst - skb_reset->head;
	skb_reset->csum_offset = offsetof(struct tcphdr, check);

	/* We need this for the benefit of ip_route_me_harder. */
	skb_dst_set_noref(skb_reset, skb_dst(skb_orig));

	skb_reset->protocol = htons(ETH_P_IP);
	
	/* OK, now try to send it. */
	if (ip_route_me_harder(skb_reset, RTN_UNSPEC) == 0) {
		//	INFO("here 4 ---");
		ip_hdr_rst->ttl	= ip4_dst_hoplimit(skb_dst(skb_reset));

		/* This really shouldn't happen, but just in case... */
		if (skb_reset->len <= dst_mtu(skb_dst(skb_reset))) {
			nf_ct_attach(skb_reset, skb_orig);
			//	INFO("here 5 ---");
			ip_local_out(skb_reset);
			//	printk("[%x] -> end \n", skb_orig);
			return;
		}
	}
	
	/* We failed.  Free the data. */
	kfree_skb(skb_reset);
}

#endif


#if KERNELVERSION==2

/**
 * Send a reset.
 */
static void
reset_connection(int hooknum, struct sk_buff *skb_orig)
{
	struct tcphdr tcp_hdr_buf;
	const struct iphdr *ip_hdr_orig;
	const struct tcphdr *tcp_hdr_orig;
	struct sk_buff *skb_reset;
	struct iphdr *ip_hdr_rst;
	struct tcphdr *tcp_hdr_rst;
	unsigned int addr_type;
	unsigned int ok;
	
	if (ip_hdr(skb_orig)->frag_off & htons(IP_OFFSET))
		return;

	tcp_hdr_orig = skb_header_pointer(skb_orig, ip_hdrlen(skb_orig),
				 					  sizeof(tcp_hdr_buf), &tcp_hdr_buf);
	if (tcp_hdr_orig == NULL)
		return;
		
	/* If the original was a reset, don't respond in kind. */
	if (tcp_hdr_orig->rst)
		return;

	/* Validate the checksum */
	if (nf_ip_checksum(skb_orig, hooknum, ip_hdrlen(skb_orig), IPPROTO_TCP))
		return;
	ip_hdr_orig = ip_hdr(skb_orig);

	skb_reset = alloc_skb(sizeof(struct iphdr) + sizeof(struct tcphdr) +
			 			  LL_MAX_HEADER, GFP_ATOMIC);
	if (skb_reset == NULL)
		return;

	skb_reserve(skb_reset, LL_MAX_HEADER);

	/* Set up the IP header. */
	skb_reset_network_header(skb_reset);
	ip_hdr_rst = (struct iphdr *) skb_put(skb_reset, sizeof(struct iphdr));
	ip_hdr_rst->version = 4;
	ip_hdr_rst->ihl = sizeof(struct iphdr) / 4;
	ip_hdr_rst->tos = 0;
	ip_hdr_rst->id = 0;
	ip_hdr_rst->frag_off = htons(IP_DF);
	ip_hdr_rst->protocol = IPPROTO_TCP;
	ip_hdr_rst->check = 0;
	ip_hdr_rst->saddr = ip_hdr_orig->daddr;
	ip_hdr_rst->daddr = ip_hdr_orig->saddr;

	/* Set up the TCP header. */
	tcp_hdr_rst = (struct tcphdr *) skb_put(skb_reset, sizeof(struct tcphdr));
	memset(tcp_hdr_rst, 0, sizeof(struct tcphdr));
	tcp_hdr_rst->source = tcp_hdr_orig->dest;
	tcp_hdr_rst->dest = tcp_hdr_orig->source;
	tcp_hdr_rst->doff = sizeof(struct tcphdr) / 4;

	if (tcp_hdr_orig->ack)
		tcp_hdr_rst->seq = tcp_hdr_orig->ack_seq;
	else {
		tcp_hdr_rst->ack_seq = htonl(ntohl(tcp_hdr_orig->seq) +
								     tcp_hdr_orig->syn +
								     tcp_hdr_orig->fin +
				      				 skb_orig->len -
				      				 ip_hdrlen(skb_orig) -
				      				 (tcp_hdr_orig->doff << 2));
		tcp_hdr_rst->ack = 1;
	}

	tcp_hdr_rst->rst = 1;
	tcp_hdr_rst->check = tcp_v4_check(sizeof(struct tcphdr),
				       				  ip_hdr_rst->saddr, ip_hdr_rst->daddr,
				       				  csum_partial(tcp_hdr_rst, sizeof(struct tcphdr), 0));

	addr_type = RTN_UNSPEC;
	if (hooknum != NF_INET_FORWARD)
		addr_type = RTN_LOCAL;
#ifdef CONFIG_BRIDGE_NETFILTER
	if (skb_reset->nf_bridge != 0 && (skb_reset->nf_bridge->mask & BRNF_BRIDGED) != 0)
		addr_type = RTN_LOCAL;
#endif

	/* OK, try to send it. */
	ok = 1;
#ifdef EMULATOR_CODE
	if (skb_orig->dst)
		dst_hold(skb_orig->dst);
	skb_reset->dst = skb_orig->dst;
	if (ip_route_me_harder(skb_reset, addr_type))
		ok = 0;
    else if (skb_reset->dst)
    	ip_hdr_rst->ttl = dst_metric(skb_reset->dst, RTAX_HOPLIMIT);
#else
	skb_dst_set(skb_reset, dst_clone(skb_dst(skb_orig)));

	if (ip_route_me_harder(skb_reset, addr_type))
		ok = 0;
	else
		ip_hdr_rst->ttl	= dst_metric(skb_dst(skb_reset), RTAX_HOPLIMIT);
#endif

	if (ok) {
		skb_reset->ip_summed = CHECKSUM_NONE;
		
		/* This really shouldn't happen, but just in case... */
#ifdef EMULATOR_CODE
		if (skb_reset->dst)
			if (skb_reset->len > dst_mtu(skb_reset->dst))
				ok = 0;
#else
		if (skb_reset->len > dst_mtu(skb_dst(skb_reset)))
			ok = 0;
#endif
	}
	
	if (ok) {
		nf_ct_attach(skb_reset, skb_orig);
		ip_local_out(skb_reset);
		return;
	}

	/* We failed.  Free the data. */
	kfree_skb(skb_reset);
}

#endif


static unsigned int itson_rx(unsigned int hooknum,
                  struct sk_buff *skb,
                  const struct net_device *in,
                  const struct net_device *out,
                  int (*okfn)(struct sk_buff*))
{

	int verdict;
	struct packetInfo pak;
	const struct iphdr *ip_hdr_orig;

	init_pak_ext(skb, in, out, okfn, hooknum, &pak, UID_RX, UID_RX);
	verdict = packetCallout((void *)&pak, ITSON_HOOK_RECEIVE);

	ip_hdr_orig = ip_hdr(skb);
	if (verdict == NF_DROP) {
		if(ip_hdr_orig->protocol == IPPROTO_TCP) {
			reset_connection(hooknum, skb);
		} else {
			DBG("No rst for UDP %x", skb);
		}
	}

	// A verdict < 0 means drop siliently (no reset)
	if (verdict < 0)
		verdict = NF_DROP;
	return verdict;
}
#if 0
static unsigned int itson_fwd(unsigned int hooknum,
                  struct sk_buff *skb,
                  const struct net_device *in,
                  const struct net_device *out,
                  int (*okfn)(struct sk_buff*))
{

	int verdict;
	struct packetInfo pak;

	init_pak(skb, in->name, out->name, &pak, UID_RX, UID_RX);
	verdict = packetCallout((void *)&pak, ITSON_HOOK_FORWARD);

	return verdict;
}
#endif
static unsigned int itson_tx(unsigned int hooknum,
                  struct sk_buff *skb,
                  const struct net_device *in,
                  const struct net_device *out,
                  int (*okfn)(struct sk_buff*))
{
	unsigned int uid;
	unsigned int sock_uid = UID_UNKNOWN;
	int verdict;
	struct packetInfo pak;
	const struct iphdr *ip_hdr_orig;

	uid = current_uid();
	if (skb->sk != NULL) {
		if (skb->sk->sk_socket != NULL) {
			sock_uid = SOCK_INODE(skb->sk->sk_socket)->i_uid;
		}
	}
	init_pak_ext(skb, in, out, okfn, hooknum, &pak, uid, sock_uid);

	verdict = packetCallout((void *)&pak, ITSON_HOOK_TRANSMIT);


	ip_hdr_orig = ip_hdr(skb);

	if (verdict == NF_DROP) {
		if(ip_hdr_orig->protocol == IPPROTO_TCP) {
			reset_connection(hooknum, skb);
		} else {
			DBG("No rst for UDP %x", skb);
		}
	}
	// A verdict < 0 means drop siliently (no reset)
	if (verdict < 0)
		verdict = NF_DROP;
	return verdict;
}

int ItsOnInitCEA(int (* packetHandler)(void *packet, int hookPoint))
{
	int ret = 0;

	//struct itsonBufferQueues bufferQueues;
	printk(KERN_INFO "ItsOnInitCEA\n");
	packetCallout = packetHandler;

	ret = nf_register_hooks(itson_ops, ARRAY_SIZE(itson_ops));
	printk(KERN_INFO "return from nf_register_hooks: %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(ItsOnInitCEA);


void ItsOnExitCEA(void)
{
	nf_unregister_hooks(itson_ops, ARRAY_SIZE(itson_ops));
}
EXPORT_SYMBOL(ItsOnExitCEA);

int
ModuleInit(void)
{

	int rc = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,13,0)
	rc = genl_register_family_with_ops(&itson_gnl_family, itson_gnl_ops_cmds);
#else
	int i;
	//printk(KERN_INFO "Registering genl family\n");
    /*register new family*/
	if ((rc = genl_register_family(&itson_gnl_family)) != 0)
	{
		printk(KERN_ERR "genl_register_family: %d\n", rc);
		return rc;
	}
    /*register functions (commands) of the new family*/
	for (i = 0; i < ARRAY_SIZE(itson_gnl_ops_cmds); i++)
	{

		//INFO("Registering cmd %d\n", itson_gnl_ops_cmds[i].cmd);
		if ((rc = genl_register_ops(&itson_gnl_family, &itson_gnl_ops_cmds[i])) != 0)
		{
			printk(KERN_ERR "register ops: %i\n",rc);
       	 	genl_unregister_family(&itson_gnl_family);
			return rc;
		}
	}
#endif
	return rc;
}

void ModuleShutdown(void)
{
	int ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,13,0)
	// NOOP
#else
	int i;

    /*unregister the functions*/
	for (i = 0; i < ARRAY_SIZE(itson_gnl_ops_cmds); i++)
	{
		ret = genl_unregister_ops(&itson_gnl_family, &itson_gnl_ops_cmds[i]);
		if (ret != 0)
       	  printk("unregister ops: %i\n", ret);
	}
#endif
    /*unregister the family*/
	ret = genl_unregister_family(&itson_gnl_family);
	if (ret != 0)
    	printk("unregister family %i\n", ret);
}

/**
 * Register an operation and its call back
 */
int
ItsOnRegisterMsgHandler(
		int msgType,
		int length,
		int (* msgHandler)(void *msg, int msgLength, struct msgInfo *info)
		)
{
	printk(KERN_INFO "%s type: %d , length: %d\n", __func__, msgType, length);
	msgHandlerList[msgType] = msgHandler;
	itson_genl_policy[msgType].len = length;
	itson_genl_policy[msgType].type = NLA_BINARY;

	return 0;
}
EXPORT_SYMBOL(ItsOnRegisterMsgHandler);

int
ItsOnSendMsg(
		int msgType,
		int length,
		void *msg,
		int pid
	)
{
    struct sk_buff *skb;
    int rc;
	void *msg_head;
	static int msg_seq = 0;
	int total;

	total = nla_total_size(length);

    /* Allocate message of appropriate size */
    skb = genlmsg_new(total, GFP_ATOMIC);
	if (skb == NULL) {
		ERR("Could not allocate sbk");
		return -ENOMEM;
	}

   	msg_head = genlmsg_put(skb, pid, msg_seq, &itson_gnl_family, 0, msgType);
	if (msg_head == NULL) {
		nlmsg_free(skb);
		ERR("Could not put message in skb");
		return -ENOMEM;
	}
	/* Add the data struct to message */
	rc = nla_put(skb, msgType, length, msg);
	if (rc != 0)
	{
		ERR("Could not prepare msg, len %d", length);
		nlmsg_free(skb);
		return rc;
	}

    /* finalize the message */
	rc = genlmsg_end(skb, msg_head);
	if (rc < 0)
	{
		nlmsg_free(skb);
		return rc;
	}

    /* send the message */
#ifdef EMULATOR_CODE
	rc = genlmsg_unicast(skb, pid);
#else
	rc = genlmsg_unicast(&init_net, skb, pid);
#endif
	msg_seq++;

	//printk(KERN_INFO "done");
	return rc;
}
EXPORT_SYMBOL(ItsOnSendMsg);

int
ItsOnCreateWorkerThread(WorkFunc_t worker)
{
	int rc = worker_count;

	if (worker_count >= (MAX_WORKERS-1))
		return -1;

	INIT_DELAYED_WORK(&workers[worker_count], (work_func_t)worker);
	worker_count++;

	return rc;
}
EXPORT_SYMBOL(ItsOnCreateWorkerThread);

int
ItsOnDoWork(int index, int delay)
{
	if ((index < 0) || (index >= worker_count)) {
		printk("ITSON (ItsOnDoWork): Invalid work queue index: %d (delay: %d)\n", index, delay);
		panic("ITSON: Houston, we have a problem. Work queue index out of bounds!\n");
	}
	if (index >= 0)
		schedule_delayed_work(&workers[index], delay*HZ);
	return(0);
}
EXPORT_SYMBOL(ItsOnDoWork);

void
ItsOnKillWorkerThread(int index)
{
	if ((index < 0) || (index >= worker_count)) {
		printk("ITSON (ItsOnKillWorkerThread): Invalid work queue index: %d\n", index);
		panic("ITSON: Houston, we have a problem. Work queue index out of bounds!\n");
	}
	if (index >= 0)
		cancel_delayed_work_sync(&workers[index]);
}
EXPORT_SYMBOL(ItsOnKillWorkerThread);


int
ItsOnUserCopy(char *to, char *from, int size)
{
	return copy_from_user(to, from, size);
}
EXPORT_SYMBOL(ItsOnUserCopy);


int ItsOnMarkQueued( struct packetInfo *pi ) 
{
	if (pi->if_in_net)
		dev_hold((struct net_device *)pi->if_in_net);
	if (pi->if_out_net)
		dev_hold((struct net_device *)pi->if_out_net);
	skb_dst_force((struct sk_buff *)pi->packet_ptr);

	return 0;
}
EXPORT_SYMBOL(ItsOnMarkQueued);

int ItsOnReinject( struct packetInfo *pi, int verdict )
{
	// Let's dummy up the nf_queue_entry to allow us to
	// just call nf_reinject
	const struct nf_afinfo *afinfo;
	struct nf_queue_entry *entry = NULL;
	struct nf_hook_ops *elem = NULL;

	DBG("ItsOnReinject attempt\n");
	rcu_read_lock();
	list_for_each_entry(elem, &nf_hooks[PF_INET][pi->hooknum], list) {
		if (elem->hook == itson_rx || elem->hook == itson_tx)
			break;
	}
	rcu_read_unlock();
	if (elem == NULL) {
		ERR("ItsOnReinject: Could not find hook\n");
		kfree_skb((struct sk_buff *)pi->packet_ptr);
		return(-1);
	}

	afinfo = nf_get_afinfo(PF_INET);
	entry = kmalloc(sizeof(*entry) + afinfo->route_key_size, GFP_ATOMIC);
	if (entry == NULL) {
		ERR("ItsOnReinject: Out of memory\n");
		kfree_skb((struct sk_buff *)pi->packet_ptr);
		return(-1);
	}

	*entry = (struct nf_queue_entry) {
		.skb = (struct sk_buff *)pi->packet_ptr,
		.elem = elem,
		.pf = PF_INET,
		.hook = pi->hooknum,
		.indev = (struct net_device *)pi->if_in_net,
		.outdev = (struct net_device *)pi->if_out_net,
		.okfn = pi->okfn,
	};
	afinfo->saveroute(pi->packet_ptr, entry);

	DBG("ItsOnReinject calling kernel nf_reinject\n");
	// nf_reinject frees entry and the skb
	nf_reinject(entry, verdict);
	return(0);

}
EXPORT_SYMBOL(ItsOnReinject);



module_init(ModuleInit);
module_exit(ModuleShutdown);

