/*

 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.


*/

#include "sim_slot.h"

#ifndef CONFIG_CHECK_SIMSLOT_COUNT_DT
#ifndef SIM_SLOT_PIN
	#error SIM_SLOT_PIN should be have a value. but not defined.
#endif
#endif
static int check_simslot_count(struct seq_file *m, void *v)
{
#ifdef CONFIG_CHECK_SIMSLOT_COUNT_DT
	struct device_node *np;
	int simslot_pin, retval, support_number_of_simslot;

	for_each_compatible_node(np, NULL, "simslot")
	simslot_pin = of_get_named_gpio_flags(np, "samsung,sim-slot", 0, NULL);
	if (simslot_pin < 0)
	{
		pr_err("***** Make a forced kernel panic because can't get pin number from sim-slot node ******\n");
		panic("kernel panic");
		return -EINVAL;
	}

	printk("\n simslot_pin : %d\n", simslot_pin);  //temp log for checking GPIO Setting correctly applyed or not
	retval = gpio_request(simslot_pin,"sim-slot ");
#else

	int retval, support_number_of_simslot;


	printk("\n SIM_SLOT_PIN : %d\n", SIM_SLOT_PIN);  //temp log for checking GPIO Setting correctly applyed or not

	retval = gpio_request(SIM_SLOT_PIN,"SIM_SLOT_PIN");
#endif
	if (retval) {
			pr_err("%s:Failed to reqeust GPIO, code = %d.\n",
				__func__, retval);
			support_number_of_simslot = retval;
	}
	else
	{
#ifdef CONFIG_CHECK_SIMSLOT_COUNT_DT
		retval = gpio_direction_input(simslot_pin);
#else
		retval = gpio_direction_input(SIM_SLOT_PIN);
#endif

		if (retval){
			pr_err("%s:Failed to set direction of GPIO, code = %d.\n",
				__func__, retval);
			support_number_of_simslot = retval;
		}
		else
		{
#ifdef CONFIG_CHECK_SIMSLOT_COUNT_DT
			retval = gpio_get_value(simslot_pin);
#else
			retval = gpio_get_value(SIM_SLOT_PIN);
#endif

			/* This codes are implemented assumption that count of GPIO about simslot is only one on H/W schematic
                           You may change this codes if count of GPIO about simslot has change */
			printk(" SIM Check : ");
			switch(retval)
			{
				case SINGLE_SIM_VALUE:
					printk("SINGLE_SIM [%d]\n",retval);
					support_number_of_simslot = SINGLE_SIM;
					break;
				case DUAL_SIM_VALUE :
					printk("DUAL_SIM [%d]\n",retval);
					support_number_of_simslot = DUAL_SIM;
					break;
				default :
					support_number_of_simslot = -1;
					break;
			}
		}
#ifdef CONFIG_CHECK_SIMSLOT_COUNT_DT
		gpio_free(simslot_pin);
#else
		gpio_free(SIM_SLOT_PIN);
#endif
	}

	if(support_number_of_simslot < 0)
	{
		pr_err("******* Make a forced kernel panic because can't check simslot count******\n");
		panic("kernel panic");
	}

	seq_printf(m, "%u\n", support_number_of_simslot);

	return 0;

}

static int check_simslot_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, check_simslot_count, NULL);
}

static const struct file_operations check_simslot_count_fops = {
	.open	= check_simslot_count_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release= single_release,
};

static int __init simslot_count_init(void)
{

	if(!proc_create("simslot_count",0,NULL,&check_simslot_count_fops))
	{
		pr_err("***** Make a forced kernel panic because can't make a simslot_count file node ******\n");
		panic("kernel panic");
		return -ENOMEM;
	}
	else return 0;
}

late_initcall(simslot_count_init);
