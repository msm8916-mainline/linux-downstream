
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>
#include <linux/list.h> 
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>

#include <linux/clk.h>
#include <linux/of_gpio.h>

#define FEATURE_DMB_USE_PINCTRL
#ifdef FEATURE_DMB_USE_PINCTRL
#include <linux/pinctrl/consumer.h>
#endif
#if defined (CONFIG_MACH_MSM8916_YG_SKT_KR)
#include <mach/board_lge.h>
#endif

#include "../../broadcast_tdmb_drv_ifdef.h"

#include "broadcast_mtv319.h"
#include "tdmb_tunerbbdrv_mtv319def.h"
#include "mtv319.h"
#include "mtv319_internal.h"

/* SPI Data read using workqueue */
//#define FEATURE_DMB_USE_WORKQUEUE

/************************************************************************/
/* LINUX Driver Setting                                                 */
/************************************************************************/
static uint32 user_stop_flg = 0;
struct tdmb_mtv319_ctrl_blk
{
	boolean									TdmbPowerOnState;
	struct spi_device*						spi_ptr;
#ifdef FEATURE_DMB_USE_WORKQUEUE
	struct work_struct						spi_work;
	struct workqueue_struct*				spi_wq;
#endif
	struct wake_lock						wake_lock;	/* wake_lock,wake_unlock */
	boolean									spi_irq_status;
	spinlock_t								spin_lock;
#ifdef FEATURE_DMB_USE_XO
	struct clk								*clk;
#endif
	struct platform_device		*pdev;
	uint32				dmb_en;
	uint32				dmb_irq;
#ifdef CONFIG_MACH_MSM8916_YG_SKT_KR
	uint32				dmb_ldo;
	uint32				dmb_ant;
#endif
};

static struct tdmb_mtv319_ctrl_blk mtv319_ctrl_info;

static Device_drv device_mtv319 = {
	&broadcast_mtv319_drv_if_power_on,
	&broadcast_mtv319_drv_if_power_off,
	&broadcast_mtv319_drv_if_init,
	&broadcast_mtv319_drv_if_stop,
	&broadcast_mtv319_drv_if_set_channel,
	&broadcast_mtv319_drv_if_detect_sync,
	&broadcast_mtv319_drv_if_get_sig_info,
	&broadcast_mtv319_drv_if_get_fic,
	&broadcast_mtv319_drv_if_get_msc,
	&broadcast_mtv319_drv_if_reset_ch,
	&broadcast_mtv319_drv_if_user_stop,
	&broadcast_mtv319_drv_if_select_antenna
};	

struct spi_device *tdmb_mtv319_get_spi_device(void)
{
	return mtv319_ctrl_info.spi_ptr;
}


void tdmb_mtv319_set_userstop(int mode)
{
	user_stop_flg = mode;
	printk("tdmb_mtv319_set_userstop, user_stop_flg = %d \n", user_stop_flg);
}


int tdmb_mtv319_tdmb_is_on(void)
{
	return (int)mtv319_ctrl_info.TdmbPowerOnState;
}

int tdmb_mtv319_power_on(void)
{
#ifdef FEATURE_DMB_USE_XO
	int rc = FALSE;
#endif
	printk("tdmb_mtv319_power_on \n");
	if ( mtv319_ctrl_info.TdmbPowerOnState == FALSE )
	{
#ifdef FEATURE_DMB_USE_XO
		if(mtv319_ctrl_info.clk != NULL) {
			rc = clk_prepare_enable(mtv319_ctrl_info.clk);
			if (rc) {
				dev_err(&mtv319_ctrl_info.spi_ptr->dev, "could not enable clock\n");
				return rc;
			}
		}
#endif
		wake_lock(&mtv319_ctrl_info.wake_lock);

#ifdef CONFIG_MACH_MSM8916_YG_SKT_KR
		gpio_set_value(mtv319_ctrl_info.dmb_ldo, 1);
		gpio_set_value(mtv319_ctrl_info.dmb_ant, 0);
#endif
		gpio_set_value(mtv319_ctrl_info.dmb_en, 0);
		gpio_set_value(mtv319_ctrl_info.dmb_en, 1);
		//gpio_set_value(MTV319_DMB_RESET_N, 1);

		//tdmb_mtv319_interrupt_free();
		mtv319_ctrl_info.TdmbPowerOnState = TRUE;

		printk("tdmb_mtv319_power_on OK\n");
	}
	else
	{
		printk("tdmb_mtv319_power_on the power already turn on \n");
	}

	printk("tdmb_mtv319_power_on completed \n");

	return TRUE;
}

int tdmb_mtv319_power_off(void)
{
	if ( mtv319_ctrl_info.TdmbPowerOnState == TRUE )
	{
		tdmb_mtv319_interrupt_lock();

#ifdef FEATURE_DMB_USE_XO
		if(mtv319_ctrl_info.clk != NULL) {
			clk_disable_unprepare(mtv319_ctrl_info.clk);
		}
#endif

		mtv319_ctrl_info.TdmbPowerOnState = FALSE;

		//gpio_set_value(MTV319_DMB_RESET_N, 0);
		gpio_set_value(mtv319_ctrl_info.dmb_en, 0);
#ifdef CONFIG_MACH_MSM8916_YG_SKT_KR
		gpio_set_value(mtv319_ctrl_info.dmb_ldo, 0);
		gpio_set_value(mtv319_ctrl_info.dmb_ant, 1);
#endif

		wake_unlock(&mtv319_ctrl_info.wake_lock);

//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 1);	// for ESD TEST
//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 0);
	}
	else
	{
		printk("tdmb_mtv319_power_on the power already turn off \n");
	}

	printk("tdmb_mtv319_power_off completed \n");
	return TRUE;
}

int tdmb_mtv319_select_antenna(unsigned int sel)
{
	return FALSE;
}

void tdmb_mtv319_interrupt_lock(void)
{
	if (mtv319_ctrl_info.spi_ptr == NULL)
	{
		printk("tdmb_mtv319_interrupt_lock fail\n");
	}
	else
	{
		disable_irq(mtv319_ctrl_info.spi_ptr->irq);
	}
}

void tdmb_mtv319_interrupt_free(void)
{
	if (mtv319_ctrl_info.spi_ptr == NULL)
	{
		printk("tdmb_mtv319_interrupt_free fail\n");
	}
	else
	{
		enable_irq(mtv319_ctrl_info.spi_ptr->irq);
	}
}

#ifdef FEATURE_DMB_USE_WORKQUEUE
static irqreturn_t broadcast_tdmb_spi_isr(int irq, void *handle)
{
	struct tdmb_mtv319_ctrl_blk* mtv319_info_p;
	unsigned long flag;

	mtv319_info_p = (struct tdmb_mtv319_ctrl_blk *)handle;
	if ( mtv319_info_p && mtv319_info_p->TdmbPowerOnState )
	{
		if (mtv319_info_p->spi_irq_status)
		{
			printk("######### spi read function is so late skip #########\n");
			return IRQ_HANDLED;
		}
//		printk("***** broadcast_tdmb_spi_isr coming *******\n");
		spin_lock_irqsave(&mtv319_info_p->spin_lock, flag);
		queue_work(mtv319_info_p->spi_wq, &mtv319_info_p->spi_work);
		spin_unlock_irqrestore(&mtv319_info_p->spin_lock, flag);
	}
	else
	{
		printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
	}

	return IRQ_HANDLED;
}

static void broacast_tdmb_spi_work(struct work_struct *tdmb_work)
{
	struct tdmb_mtv319_ctrl_blk *pTdmbWorkData;

	pTdmbWorkData = container_of(tdmb_work, struct tdmb_mtv319_ctrl_blk, spi_work);
	if ( pTdmbWorkData )
	{
		pTdmbWorkData->spi_irq_status = TRUE;
		broadcast_mtv319_drv_if_isr();
		pTdmbWorkData->spi_irq_status = FALSE;
	}
	else
	{
		printk("~~~~~~~broadcast_tdmb_spi_work call but pTdmbworkData is NULL ~~~~~~~\n");
	}
}
#else
static irqreturn_t broadcast_tdmb_spi_event_handler(int irq, void *handle)
{
	struct tdmb_mtv319_ctrl_blk* mtv319_info_p;

	mtv319_info_p = (struct tdmb_mtv319_ctrl_blk *)handle;
	if ( mtv319_info_p && mtv319_info_p->TdmbPowerOnState )
	{
		if (mtv319_info_p->spi_irq_status)
		{
			printk("######### spi read function is so late skip ignore #########\n");
			return IRQ_HANDLED;
		}

		mtv319_info_p->spi_irq_status = TRUE;
		broadcast_mtv319_drv_if_isr();
		mtv319_info_p->spi_irq_status = FALSE;
	}
	else
	{
		printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
	}

	return IRQ_HANDLED;
}
#endif
#ifdef FEATURE_DMB_USE_PINCTRL
static int tdmb_pinctrl_init(void)
{
    struct pinctrl *tdmb_pinctrl;
    struct pinctrl_state *gpio_state_suspend;

    tdmb_pinctrl = devm_pinctrl_get(&(mtv319_ctrl_info.pdev->dev));


    if(IS_ERR_OR_NULL(tdmb_pinctrl)) {
        pr_err("%s: Getting pinctrl handle failed\n", __func__);
        return -EINVAL;
    }
    gpio_state_suspend
     = pinctrl_lookup_state(tdmb_pinctrl, "gpio_tdmb_suspend");

     if(IS_ERR_OR_NULL(gpio_state_suspend)) {
         pr_err("%s: Failed to get the suspend state pinctrl handle\n", __func__);
         return -EINVAL;
    }

    if(pinctrl_select_state(tdmb_pinctrl, gpio_state_suspend)) {
        pr_err("%s: error on pinctrl_select_state for tdmb enable and irq pin\n", __func__);
        return -EINVAL;
    }
    else {
        printk("%s: success to set pinctrl_select_state for tdmb enable and irq pin\n", __func__);
    }

    return 0;
}
#endif
static int tdmb_configure_gpios(void)
{
	int rc = OK;
	int err_count = 0;

	mtv319_ctrl_info.dmb_en = of_get_named_gpio(mtv319_ctrl_info.pdev->dev.of_node,"tdmb-mtv319,en-gpio",0);
	rc = gpio_request(mtv319_ctrl_info.dmb_en, "DMB_EN");
	if (rc < 0) {
		err_count++;
		printk("%s:Failed GPIO DMB_EN request!!!\n",__func__);
	}
	gpio_direction_output(mtv319_ctrl_info.dmb_en, 0);  /* output and low */

	mtv319_ctrl_info.dmb_irq = of_get_named_gpio(mtv319_ctrl_info.pdev->dev.of_node,"tdmb-mtv319,irq-gpio",0);
	rc = gpio_request(mtv319_ctrl_info.dmb_irq, "DMB_INT_N");
	if (rc < 0) {
		err_count++;
		printk("%s:Failed GPIO DMB_INT_N request!!!\n",__func__);
	}
	gpio_direction_input(mtv319_ctrl_info.dmb_irq);

#ifdef CONFIG_MACH_MSM8916_YG_SKT_KR
	mtv319_ctrl_info.dmb_ldo = of_get_named_gpio(mtv319_ctrl_info.pdev->dev.of_node,"tdmb-mtv319,ldo-gpio",0);
	rc = gpio_request(mtv319_ctrl_info.dmb_ldo, "DMB_LDO");
	if (rc < 0) {
		err_count++;
		printk("%s:Failed GPIO DMB_LDO request!!!\n",__func__);
	}
	gpio_direction_output(mtv319_ctrl_info.dmb_ldo, 0);  /* output and low */

	mtv319_ctrl_info.dmb_ant = of_get_named_gpio(mtv319_ctrl_info.pdev->dev.of_node,"tdmb-mtv319,ant-gpio",0);
	rc = gpio_request(mtv319_ctrl_info.dmb_ant, "DMB_ANT");
	if(rc < 0) {
		err_count++;
		printk("%s:Failed GPIO DMB_ANT request!!!\n",__func__);
	}
	gpio_direction_output(mtv319_ctrl_info.dmb_ant,0);
#endif

	if(err_count > 0) rc = -EINVAL;

	return rc;
}

unsigned char mtv319_spi_read(unsigned char page, unsigned char reg)
{
	int ret;
	static u8 out_buf[4] __attribute__((aligned(8)));
	static u8 in_buf[4] __attribute__((aligned(8)));
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = out_buf,
		.rx_buf = in_buf,
		.len = 4,
		.cs_change = 0,
		.delay_usecs = 0
	};

	out_buf[0] = 0x90 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* Read size */

	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(mtv319_ctrl_info.spi_ptr, &msg);
	if (ret) {
		DMBERR("error: %d\n", ret);
		return 0xFF;
	}

#if 0
	DMBMSG("0x%02X 0x%02X 0x%02X 0x%02X\n",
			in_buf[0], in_buf[1], in_buf[2], in_buf[3]);
#endif

	return in_buf[MTV319_SPI_CMD_SIZE];
}

#if 0
void mtv319_spi_read_burst(unsigned char page, unsigned char reg,
			unsigned char *buf, int size)
{
	int ret;
	static u8 out_buf[MTV319_SPI_CMD_SIZE] __attribute__((aligned(8)));
	struct spi_message msg;
	struct spi_transfer xfer0 = {
		.tx_buf = out_buf,
		.rx_buf = buf,
		.len = MTV319_SPI_CMD_SIZE,
		.cs_change = 0,
		.delay_usecs = 0,
	};
	struct spi_transfer xfer1 = {
		.tx_buf = buf,
		.rx_buf = buf,
		.len = size,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	out_buf[0] = 0xA0; /* Memory read */
	out_buf[1] = 0x00;
	out_buf[2] = 188; /* Fix */

	spi_message_init(&msg);
	spi_message_add_tail(&xfer0, &msg);
	spi_message_add_tail(&xfer1, &msg);

	ret = spi_sync(mtv319_ctrl_info.spi_ptr, &msg);
	if (ret) {
		DMBERR("error: %d\n", ret);
		return;
	}
}
#else
void mtv319_spi_read_burst(unsigned char page, unsigned char reg,
			unsigned char *buf, int size)
{
	int ret;
	static unsigned char temp_buf[MTV319_SPI_CMD_SIZE + (16 * 188)] __attribute__((aligned(8)));
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = temp_buf,
		.rx_buf = temp_buf,
		.len = MTV319_SPI_CMD_SIZE + size,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	spi_message_init(&msg);
	temp_buf[0] = 0xA0; /* Memory read */
	temp_buf[1] = 0x00;
	temp_buf[2] = 188; /* Fix */

	spi_message_add_tail(&msg_xfer, &msg);
	ret = spi_sync(mtv319_ctrl_info.spi_ptr, &msg);
	if (ret) {
		DMBERR("0 error: %d\n", ret);
		return;
	}

	memcpy(buf, &temp_buf[MTV319_SPI_CMD_SIZE], size);
}
#endif

void mtv319_spi_write(unsigned char page, unsigned char reg, unsigned char val)
{
	static u8 out_buf[4] __attribute__((aligned(8)));
	static u8 in_buf[4] __attribute__((aligned(8)));
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = out_buf,
		.rx_buf = in_buf,
		.len = 4,
		.cs_change = 0,
		.delay_usecs = 0
	};
	int ret;

	out_buf[0] = 0x80 | page;
	out_buf[1] = reg;
	out_buf[2] = 1; /* size */
	out_buf[3] = val;

	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(mtv319_ctrl_info.spi_ptr, &msg);
	if (ret)
		DMBERR("error: %d\n", ret);
}

void mtv319_spi_recover(unsigned char *buf, unsigned int size)
{
	int ret;
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.tx_buf = buf,
		.rx_buf = buf,
		.len = size,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	memset(buf, 0xFF, size);

	spi_message_init(&msg);
	spi_message_add_tail(&msg_xfer, &msg);

	ret = spi_sync(mtv319_ctrl_info.spi_ptr, &msg);
	if (ret)
		DMBERR("error: %d\n", ret);
}

static int broadcast_tdmb_mtv319_probe(struct spi_device *spi)
{
	int rc;

	mtv319_ctrl_info.spi_ptr 					= spi;
	mtv319_ctrl_info.spi_ptr->mode 			= SPI_MODE_0;
	mtv319_ctrl_info.spi_ptr->bits_per_word 	= 8;
	mtv319_ctrl_info.spi_ptr->max_speed_hz 	= (10000*1000);
	mtv319_ctrl_info.pdev					= to_platform_device(&spi->dev);

	rc = spi_setup(spi);
	printk("%s is called and spi_setup\n", "broadcast_tdmb_mtv319_probe");

#ifdef FEATURE_DMB_USE_WORKQUEUE
	INIT_WORK(&mtv319_ctrl_info.spi_work, broacast_tdmb_spi_work);

	mtv319_ctrl_info.spi_wq = create_singlethread_workqueue("tdmb_spi_wq");
	// t3900_ctrl_info.spi_wq = create_rt_workqueue("tdmb_spi_wq");
	if(mtv319_ctrl_info.spi_wq == NULL){
		printk("Failed to setup tdmb spi workqueue \n");
		return -ENOMEM;
	}
#endif
	tdmb_configure_gpios( );

#ifdef FEATURE_DMB_USE_PINCTRL
	tdmb_pinctrl_init();
#endif

#ifdef FEATURE_DMB_USE_WORKQUEUE
	rc = request_irq(spi->irq, broadcast_tdmb_spi_isr, IRQF_DISABLED | IRQF_TRIGGER_FALLING, spi->dev.driver->name, &mtv319_ctrl_info);
#else
	rc = request_threaded_irq(spi->irq, NULL, broadcast_tdmb_spi_event_handler, IRQF_ONESHOT | IRQF_DISABLED | IRQF_TRIGGER_FALLING,
	                   spi->dev.driver->name, &mtv319_ctrl_info);
#endif
	printk("broadcast_tdmb_mtv319_probe request_irq=%d\n", rc);

	tdmb_mtv319_interrupt_lock();

	wake_lock_init(&mtv319_ctrl_info.wake_lock,  WAKE_LOCK_SUSPEND, dev_name(&spi->dev));

	spin_lock_init(&mtv319_ctrl_info.spin_lock);

	printk("broadcast_tdmb_mtv319_probe End. \n");

	return rc;
}

static int broadcast_tdmb_mtv319_remove(struct spi_device *spi)
{
	printk("broadcast_tdmb_mtv319_remove \n");

#ifdef FEATURE_DMB_USE_WORKQUEUE
	if (mtv319_ctrl_info.spi_wq)
	{
		flush_workqueue(mtv319_ctrl_info.spi_wq);
		destroy_workqueue(mtv319_ctrl_info.spi_wq);
	}
#endif

	free_irq(spi->irq, &mtv319_ctrl_info);

	wake_lock_destroy(&mtv319_ctrl_info.wake_lock);

	memset((unsigned char*)&mtv319_ctrl_info, 0x0, sizeof(struct tdmb_mtv319_ctrl_blk));
	return 0;
}

static int broadcast_tdmb_mtv319_suspend(struct spi_device *spi, pm_message_t mesg)
{
	printk("broadcast_tdmb_mtv319_suspend \n");
	return 0;
}

static int broadcast_tdmb_mtv319_resume(struct spi_device *spi)
{
	printk("broadcast_tdmb_mtv319_resume \n");
	return 0;
}

static struct of_device_id tdmb_spi_table[] = {
	{
		.compatible = "lge,tdmb",
	},
	{}
};

static struct spi_driver broadcast_tdmb_driver = {
	.driver = {
		.name = "tdmb",
		.of_match_table = tdmb_spi_table,
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},

	.probe = broadcast_tdmb_mtv319_probe,
	.suspend = broadcast_tdmb_mtv319_suspend,
	.resume	= broadcast_tdmb_mtv319_resume,
	.remove	= __broadcast_dev_exit_p(broadcast_tdmb_mtv319_remove),
};
#if 0
static int broadcast_tdmb_mtv319_check_chip_id(void)
{
	int ret = ERROR;

	ret = tdmb_mtv319_power_on();

	// add reading chip id

	tdmb_mtv319_power_off();

	return ret;
}
#endif
static int __broadcast_dev_init broadcast_tdmb_mtv319_drv_init(void)
{
	int ret;

#if defined (CONFIG_MACH_MSM8916_YG_SKT_KR)
	if(lge_get_board_revno() >= HW_REV_B) {
		printk("broadcast_tdmb_mtv319_drv_init rev no(%d)\n",lge_get_board_revno());
		return ERROR;
	}
#endif
	if(broadcast_tdmb_drv_check_module_init() != OK) {
		ret = ERROR;
		return ret;
	}

	ret = spi_register_driver(&broadcast_tdmb_driver);

#if 0
	if(broadcast_tdmb_mtv319_check_chip_id() != OK) {
		spi_unregister_driver(&broadcast_tdmb_driver);
		return ret;
	}
#endif

	if (ret < 0)
		DMBERR("MTV319 SPI driver register failed\n");

	ret = broadcast_tdmb_drv_start(&device_mtv319);
	if (ret) {
		DMBERR("Failed to load (%d)\n", ret);
		return ret;
	}

	return ret;
}

static void __exit broadcast_tdmb_mtv319_drv_exit(void)
{
	spi_unregister_driver(&broadcast_tdmb_driver);
}



module_init(broadcast_tdmb_mtv319_drv_init);
module_exit(broadcast_tdmb_mtv319_drv_exit);
MODULE_DESCRIPTION("MTV319 tdmb device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("RAONTECH");
