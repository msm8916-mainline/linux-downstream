/*
 *  linux/drivers/video/prox_notify.c
 *
 *  Copyright (C) 2006 Antonino Daplas <adaplas@pol.net>
 *
 *	2001 - Documented with DocBook
 *	- Brad Douglas <brad@neruo.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */
#include <linux/notifier.h>
#include <linux/export.h>

static BLOCKING_NOTIFIER_HEAD(prox_notifier_list);

/**
 *	prox_register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int prox_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&prox_notifier_list, nb);
}
EXPORT_SYMBOL(prox_register_client);

/**
 *	prox_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int prox_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&prox_notifier_list, nb);
}
EXPORT_SYMBOL(prox_unregister_client);

/**
 * prox_notifier_call_chain - notify clients of prox_events
 *
 */
 
int prox_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&prox_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(prox_notifier_call_chain);
