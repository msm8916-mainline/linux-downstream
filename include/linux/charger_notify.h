#ifndef _LINUX_CHARGER_H
#define _LINUX_CHARGER_H

#include <linux/kgdb.h>


#include <linux/fs.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/list.h>
#include <linux/backlight.h>
#include <linux/slab.h>
#include <asm/io.h>

#define CHARGER_EVENT_REPORT                  0xf

/*	NEAR BY */ 
#define CHARGER_EVENT_NEAR_BY		0x00
/*FAR AWAY*/
#define CHARGER_EVENT_FAR_AWAY		0x01

struct charger_event {
	void *data;
};

extern int charger_register_client(struct notifier_block *nb);
extern int charger_unregister_client(struct notifier_block *nb);
extern int charger_notifier_call_chain(unsigned long val, void *v);

#endif /* _LINUX_CHARGER_H */
