/*
*
*drivers/input/misc/hall.c
*Created by TCTSZ-WH,2014-6-17. Hall supported.
*
*/

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
#include "hall.h"

//#define WINDOW_COVER	/*ADD by TCTSZ-WH,2014-8-27. Window cover not supported on POP10.*/
#include <linux/interrupt.h>
struct input_dev *hall_input = NULL;
struct hall_data *hall = NULL;
static struct class *hall_class = NULL;
static struct device *hall_dev = NULL;

static void do_hall_work(struct work_struct *work)
{
	unsigned int gpio_code = 0;

/*MOD Begin by TCTSZ-WH,2014-8-27. Report different key with window cover and normal cover.*/
	printk("cover = %s,is_suspend = %s\n",gpio_get_value(hall->irq_gpio) ? "open" : "close", hall->tp_is_suspend ? "yes" : "no");
#ifdef WINDOW_COVER
	gpio_code = gpio_get_value(hall->irq_gpio)? KEY_UNLOCK_COVER : KEY_LOCK_LED_COVER;	/*Report different keys when cover closed and open.*/
#else
	gpio_code = KEY_POWER;
	if(((hall->tp_is_suspend) && (gpio_get_value(hall->irq_gpio))) || ((!hall->tp_is_suspend) && (!gpio_get_value(hall->irq_gpio))))
#endif
/*MOD End by TCTSZ-WH,2014-8-27. Report different key with window cover and normal cover.*/
	{
		input_report_key(hall_input, gpio_code, 1);
		input_sync(hall_input);
		input_report_key(hall_input, gpio_code, 0);
		input_sync(hall_input);
	}

#ifdef WINDOW_COVER
	if(!hall->tp_is_suspend)	/*Only reset tp when tp is not suspended.*/
	{
		if (hall->tp_set_sensitivity )
		{
			hall->tp_set_sensitivity(gpio_get_value(hall->irq_gpio));
		}
	}
#endif
}

irqreturn_t interrupt_hall_irq(int irq, void *dev)
{
	printk("<2>""-----%s,%d,irq_value = %d\n",__func__,__LINE__,gpio_get_value(hall->irq_gpio));
	//gpio_get_value(hall->irq_gpio): 0-hall closed, 1-hall open

	queue_work(hall->hall_wq, &hall->hall_work);

	return IRQ_HANDLED;
}

int hall_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device_node *node = pdev->dev.of_node;

	hall = devm_kzalloc(&pdev->dev, sizeof(struct hall_data),
				 GFP_KERNEL);
	if (hall == NULL)
	{
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	/*gpio request, interrupt request, power */
	hall->irq_gpio = of_get_named_gpio(node, "interrupt-gpios", 0);

	if (hall->irq_gpio < 0)
	{
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"interrupt-gpios", node->full_name, hall->irq_gpio);
		goto error;
	}
	else
	{
		rc = gpio_request(hall->irq_gpio, "HALL_INTERRUPT");
		if (rc)
		{
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, hall->irq_gpio, rc);

			goto error;
		}
		rc = gpio_direction_input(hall->irq_gpio);
		if (rc)
		{
			dev_err(&pdev->dev, "Unable to set direction for irq gpio [%d]\n",
				hall->irq_gpio);
			gpio_free(hall->irq_gpio);
			goto error;
		}
	}

	rc = request_irq(gpio_to_irq(hall->irq_gpio), interrupt_hall_irq, IRQ_TYPE_EDGE_BOTH , "hall_work", &pdev->dev);
	if(rc < 0)
	{
		goto exit_free_irq;
	}

	hall_input = input_allocate_device();
	if (!hall_input)
	{
		dev_err(&pdev->dev, "hall.c: Not enough memory\n");
		return -ENOMEM;
	}

	hall_input->name = "hall";

#ifdef WINDOW_COVER
	input_set_capability(hall_input, EV_KEY, KEY_UNLOCK_COVER);
	input_set_capability(hall_input, EV_KEY, KEY_LOCK_LED_COVER);
#else
	input_set_capability(hall_input, EV_KEY, KEY_POWER);
#endif

	rc = input_register_device(hall_input);
	if (rc)
	{
		dev_err(&pdev->dev, "hall.c: Failed to register device\n");
		return rc;
	}

	hall->hall_wq = create_singlethread_workqueue("hall_wq");
	INIT_WORK(&hall->hall_work, do_hall_work);
	enable_irq_wake(gpio_to_irq(hall->irq_gpio));

	hall_class= class_create(THIS_MODULE, "hall");
	hall_dev = device_create(hall_class, NULL, 0, NULL, "window_cover");
	    if (IS_ERR(hall_dev))
        dev_err(&pdev->dev, "Failed to create device(hall_dev)!\n");

	return 0;
exit_free_irq:
	free_irq(gpio_to_irq(hall->irq_gpio),NULL);

error:
	devm_kfree(&pdev->dev, hall);

	return rc;
}

int hall_remove(struct platform_device *pdev)
{
	hall = platform_get_drvdata(pdev);

	free_irq(gpio_to_irq(hall->irq_gpio),NULL);

	input_unregister_device(hall_input);
	if (hall_input)
	{
		input_free_device(hall_input);
		hall_input = NULL;
	}

	if (gpio_is_valid(hall->irq_gpio))
		gpio_free(hall->irq_gpio);

	return 0;
}

static struct of_device_id hall_of_match[] = {
	{.compatible = "qcom,hall",},
	{},
};

static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.remove = hall_remove,
	.driver = {
		   .name = "qcom,hall",
		   .owner = THIS_MODULE,
		   .of_match_table = hall_of_match,
	}
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_driver);
}

static void __init hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);
MODULE_LICENSE("GPL");

