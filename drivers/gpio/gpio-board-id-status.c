#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
//#include <linux/device.h>


//board_id_gpio		902+10

/*  add board id gpio in dtsi file and use function "of_get_named_gpio" get the gpio number, can't use gpio index directly ,
because in kener vision 3.10 gpio_number=base(8916 8936 8939 is 902)+gpio_index , the base value we can see in "gpio_lib.conf" file .  TCTNB  ZXZ add in 2015/01/07 */



static int board_id_gpio;

static char board_id_status[4];

int board_id_status_detect(void)
{
	int status;
	int rc = 0;

	pr_err("zxz=======board_id_status_detect======board_id_gpio=%d=======\n",board_id_gpio);

	rc = gpio_request(board_id_gpio, "board id status");

		if (rc) {
			pr_err("board id status request gpio=%d failed, rc=%d\n", board_id_gpio,rc);
			goto err_gpio;
		}

		rc = gpio_direction_input(board_id_gpio);
		if (rc) {
			pr_err("set_direction for gpio=%d failed, rc=%d\n",board_id_gpio,rc);
			goto err_gpio;
		}

		status=gpio_get_value(board_id_gpio);
		
pr_err("zxz=======board_id_status_detect======status=%d=======\n",status);

		gpio_free(board_id_gpio);

		if(status)
		strcpy(board_id_status,"1");
		else
		strcpy(board_id_status,"0");

		return 0;

	err_gpio:
			gpio_free(board_id_gpio);
		return  -ENODEV;

}


static ssize_t board_id_status_read(struct class *class,
				struct class_attribute *attr, char *buf)
{
	int err;

	err=board_id_status_detect();
pr_err("zxz=======board_id_status_read======board_id_status=%s=======\n",board_id_status);
	if(err == -ENODEV) {
		pr_err("board id status read error!\n");
	}
	return sprintf(buf, "%s\n", board_id_status);
}


/* board_id_status value attribute (/<sysfs>/class/board_id_status/status) */
static struct class_attribute board_id_status_value =
	__ATTR(status, 0444, board_id_status_read, NULL);


static int board_id_status_creat_file(void)
{
	int ret;
	struct class *board_id_class;

	/* board_id_status create (/<sysfs>/class/board_id_status) */
	board_id_class = class_create(THIS_MODULE, "board_id_status");
	if (IS_ERR(board_id_class)) {
		ret = PTR_ERR(board_id_class);
		printk(KERN_ERR "board_id_class: couldn't create board_id_status\n");
	}
	ret = class_create_file(board_id_class, &board_id_status_value);
	if (ret) {
		printk(KERN_ERR "board_id_status: couldn't create board_id_status_value\n");
	}

	return 0;

}


static int board_id_status_probe(struct platform_device *pdev)
{
	int rc = 0;

	board_id_gpio=of_get_named_gpio(pdev->dev.of_node,
		"qcom,board-id-gpio", 0);
	
	rc=board_id_status_creat_file();

	return rc;
}


static int board_id_status_remove(struct platform_device *pdev)
{

	gpio_free(board_id_gpio);

	return 0;
}


static const struct of_device_id board_id_dt_match[] = {
	{.compatible = "qcom,board-id-status"},
	{}
};

MODULE_DEVICE_TABLE(of, board_id_dt_match);

static struct platform_driver board_id_status_driver = {
	.probe = board_id_status_probe,
	.remove = board_id_status_remove,
	.shutdown = NULL,
	.driver = {
		.name = "board_id_status",
		.of_match_table = board_id_dt_match,
	},
};

static int board_id_register_driver(void)
{
	return platform_driver_register(&board_id_status_driver);
}



static int __init board_id_status_init(void)
{

	int ret;

	ret = board_id_register_driver();
	if (ret) {
		pr_err("board_id_register_driver() failed!\n");
		return ret;
	}

	return ret;

}

module_init(board_id_status_init);

static void __exit board_id_status_exit(void)
{
}
module_exit(board_id_status_exit);

MODULE_DESCRIPTION("Get board id status");
MODULE_LICENSE("GPL v2");

