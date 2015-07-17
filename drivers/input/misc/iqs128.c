
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/switch.h>
#include <linux/regulator/consumer.h>
#include "psensor.h"
/*******add begin by lizhi.wu@tcl.com 2014-10-10***************/
#include <linux/interrupt.h>
/*******add end by lizhi.wu@tcl.com 2014-10-10***************/

extern struct psensor_data *psensor_data;
extern void send_psensor_uevent(enum PSENSOR_STATUS val);

static irqreturn_t iqs128_int_handler(int irq, void *dev)
{
	printk("<2>""---%s,gpio_get_value(irq_gpio) = %d\n",__func__,gpio_get_value(psensor_data->irq));
	psensor_data->p_sensor_value = gpio_get_value(psensor_data->irq);

	if (!work_pending(&psensor_data->psensor_work)) {
		queue_work(psensor_data->psensor_wq, &psensor_data->psensor_work);
	}

	return IRQ_HANDLED;
}

static void do_psensor_work(struct work_struct *work)
{
	send_psensor_uevent(psensor_data->p_sensor_value);
}

int iqs128_init(void)
{
	int ret = 0;

	INIT_WORK(&psensor_data->psensor_work, do_psensor_work);
	psensor_data->psensor_wq = create_singlethread_workqueue("psensor_wq");
	if (!psensor_data->psensor_wq) {
		pr_err("create thread error, line: %d\n", __LINE__);
		return -ENOMEM;
	}

	ret = request_irq(gpio_to_irq(psensor_data->irq), iqs128_int_handler, IRQ_TYPE_EDGE_BOTH , "iqs128_work", NULL);
	if(ret < 0)
	{
		pr_err("request_irq failed. irq = %d\n",gpio_to_irq(psensor_data->irq));
		goto exit_free_irq;
	}

	return 0;

exit_free_irq:
	free_irq(gpio_to_irq(psensor_data->irq),NULL);

	return ret;
}

