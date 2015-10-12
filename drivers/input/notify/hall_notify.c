/*
 *  linux/drivers/video/hall_notify.c
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

static BLOCKING_NOTIFIER_HEAD(hall_notifier_list);

/**
 *	hall_register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int hall_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hall_notifier_list, nb);
}
EXPORT_SYMBOL(hall_register_client);

/**
 *	hall_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int hall_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hall_notifier_list, nb);
}
EXPORT_SYMBOL(hall_unregister_client);

/**
 * hall_notifier_call_chain - notify clients of hall_events
 *
 */
 
int hall_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&hall_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(hall_notifier_call_chain);
