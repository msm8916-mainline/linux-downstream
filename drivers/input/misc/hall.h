#ifndef __HALL_H
#define __HALL_H

struct hall_data {
	int irq_gpio;
	int tp_is_suspend;
	struct workqueue_struct *hall_wq;
	struct work_struct hall_work;
	void (*tp_set_sensitivity)(int);
};

#endif

