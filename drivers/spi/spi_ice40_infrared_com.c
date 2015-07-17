/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>

//#define COMPATIBLE_IR //mockup2 donot compatible
#define ICE40_FIRMWARE_DL
#define ICE40_FIRMWARE_DL_TEST

#ifdef ICE40_FIRMWARE_DL
#include "spi_ice40_infrared.h"
int i;
#endif
/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define	PEELIR_MAJOR		176	/* assigned */
#define PEELIR_MINORS		32	/* ... up to 256 */
#define	ICE40_INFRARED_MAJOR		176	/* assigned */
#define ICE40_INFRARED_MINORS		32	/* ... up to 256 */

//#define INFRARED_CHARDEV_NAME		"infrared_dev"
//#define INFRARED_SYSCLS_NAME		"infrared"
//#define INFRARED_SYSDEV_NAME		"spidev0.0"

#define LATTICE_CHARDEV_NAME		"spi"
#define LATTICE_SYSCLS_NAME			"spidev"
#define LATTICE_SYSDEV_NAME			"spidev0.0"

#define PEELIR_CHARDEV_NAME			"peelspi"
#define PEELIR_SYSCLS_NAME			"peelir"
#define PEELIR_SYSDEV_NAME			"peel_ir"


static int prev_tx_status; /* Status of previous transaction */
static struct class *peelir_class;


static DECLARE_BITMAP(minors, ICE40_INFRARED_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

struct infrared_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*buffer;
	u8			*bufferrx;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_sleep;

	int reset_gpio;
	int slave_select_gpio;
	int config_codne_gpio;
	int vdd_en_gpio;
	int clk_en_gpio;

	struct clk *infrared_clk;
	struct regulator *infrared_iovcc;
	struct regulator *infrared_vpp;

	bool infrared_mode;
	bool powered_vpp;
	bool powered_vcc;

};

#define INFRARED_GP0_CLK_FREQ			(19200000)

#ifdef ICE40_FIRMWARE_DL

static struct infrared_data *ice40_g_spidev = NULL;
static u8 *frameware_bin ;
int img_size = ARRAY_SIZE(ice40_bin);

#endif

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*
 * This can be used for testing the controller, given the busnum and the
 * cs required to use. If those parameters are used, spidev is
 * dynamically added as device on the busnum, and messages can be sent
 * via this interface.
 */
static int busnum = 0;
module_param(busnum, int, S_IRUGO);
MODULE_PARM_DESC(busnum, "bus num of the controller");

static int chipselect = 0;
module_param(chipselect, int, S_IRUGO);
MODULE_PARM_DESC(chipselect, "chip select of the desired device");

static int maxspeed = 1520000;
module_param(maxspeed, int, S_IRUGO);
MODULE_PARM_DESC(maxspeed, "max_speed of the desired device");

static int spimode = SPI_MODE_3;
module_param(spimode, int, S_IRUGO);
MODULE_PARM_DESC(spimode, "mode of the desired device");


static ssize_t ice40_infrared_sync(struct infrared_data *spidev, struct spi_message *message);

static int ice40_infrared_parse_dtsi(struct infrared_data	*spidev)
{
	struct device_node *node = spidev->spi->dev.of_node;
	int ret = 0;

	if (!node) {
		pr_err("device specific info missing\n");
		ret = -ENODEV;
		goto init_parse_dtsi_failed;
	}

	spidev->reset_gpio = of_get_named_gpio(node, "infrared,reset-gpio", 0);
	if (spidev->reset_gpio < 0) {
		pr_err("reset gpio is missing\n");
		ret = spidev->reset_gpio;
		goto init_parse_dtsi_failed;
	}

	spidev->slave_select_gpio = of_get_named_gpio(node,
				"infrared,slave-select-gpio", 0);
	if (spidev->slave_select_gpio < 0) {
		pr_err("slave select gpio is missing\n");
		ret = spidev->slave_select_gpio;
		goto init_parse_dtsi_failed;
	}

	spidev->config_codne_gpio = of_get_named_gpio(node,
				"infrared,config-codne-gpio", 0);
	if (spidev->config_codne_gpio < 0) {
		pr_err("config done gpio is missing\n");
		ret = spidev->config_codne_gpio;
		goto init_parse_dtsi_failed;
	}

	spidev->vdd_en_gpio = of_get_named_gpio(node, "infrared,vdd-en-gpio", 0);
	if (spidev->vdd_en_gpio < 0) {
		pr_err("vcc enable gpio is missing\n");
		ret = spidev->vdd_en_gpio;
		goto init_parse_dtsi_failed;
	}

init_parse_dtsi_failed:
	return ret;
}

static int ice40_infrared_req_gpios(struct infrared_data *spidev)
{
	int ret;

	ret = devm_gpio_request(&spidev->spi->dev, spidev->reset_gpio,
				"ice40_reset");
	if (ret < 0) {
		pr_err("fail to request reset gpio\n");
		goto req_gpios_failed;
	}

	ret = devm_gpio_request(&spidev->spi->dev, spidev->config_codne_gpio,
				"ice40_config_done");
	if (ret < 0) {
		pr_err("fail to request config_done gpio\n");
		goto req_gpios_failed;
	}

	ret = devm_gpio_request(&spidev->spi->dev, spidev->vdd_en_gpio,
				"ice40_vcc_en");
	if (ret < 0) {
		pr_err("fail to request vcc_en gpio\n");
		goto req_gpios_failed;
	}

req_gpios_failed:
	return ret;
}


static int ice40_infrared_pinctrl_init(struct infrared_data	*spidev)
{
	int ret = 0;

	spidev->pinctrl = devm_pinctrl_get(&spidev->spi->dev);
	if (IS_ERR_OR_NULL(spidev->pinctrl)) {
		pr_err("ice40 Failed to get pin ctrl\n");
		return PTR_ERR(spidev->pinctrl);
	}

	spidev->pins_active = pinctrl_lookup_state(spidev->pinctrl,
				PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(spidev->pins_active)) {
		pr_err("ice40 Failed to lookup pinctrl default state\n");
		return PTR_ERR(spidev->pins_active);
	}

	spidev->pins_sleep = pinctrl_lookup_state(spidev->pinctrl,
				PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(spidev->pins_sleep)) {
		pr_err("ice40 Failed to lookup pinctrl sleep state\n");
		return PTR_ERR(spidev->pins_sleep);
	}

	ret = pinctrl_select_state(spidev->pinctrl, spidev->pins_sleep);
		if (ret) {
			pr_err("%s: Can not set %s pins\n", __func__, PINCTRL_STATE_SLEEP);
		}

	return ret;

}

static int ice40_infrared_clock_function(struct infrared_data *spidev, int enable)
{
	int ret;

	if(enable){
		ret = pinctrl_select_state(spidev->pinctrl, spidev->pins_active);
		if (ret) {
			pr_err("%s: Can not set %s pins\n", __func__, PINCTRL_STATE_DEFAULT);
		}
	}
	else{
		ret = pinctrl_select_state(spidev->pinctrl, spidev->pins_sleep);
		if (ret) {
			pr_err("%s: Can not set %s pins\n", __func__, PINCTRL_STATE_SLEEP);
		}
	}

	return ret;
}


static int ice40_infrared_init_regulators(struct infrared_data *spidev)
{
	int ret;

	spidev->infrared_vpp = devm_regulator_get(&spidev->spi->dev, "infrared-vpp");
	if (IS_ERR(spidev->infrared_vpp)) {
		ret = PTR_ERR(spidev->infrared_vpp);
		if (ret != -EPROBE_DEFER)
			pr_err("fail to get infrared_vpp %d\n", ret);
		goto init_regulators_failed;
	}

	ret = regulator_set_voltage(spidev->infrared_vpp, 2850000, 2850000);
	if (ret < 0) {
		pr_err("fail to set infrared_vpp %d\n", ret);
		goto init_regulators_failed;
	}

	spidev->infrared_iovcc = devm_regulator_get(&spidev->spi->dev, "infrared-iovcc");
	if (IS_ERR(spidev->infrared_iovcc)) {
		ret = PTR_ERR(spidev->infrared_iovcc);
		if (ret != -EPROBE_DEFER)
			pr_err("fail to get infrared_iovcc %d\n", ret);
		goto init_regulators_failed;
	}

	ret = regulator_set_voltage(spidev->infrared_iovcc, 1800000, 1800000);
	if (ret < 0) {
		pr_err("fail to set infrared_vpp %d\n", ret);
		goto init_regulators_failed;
	}

init_regulators_failed:
	return ret;
}

static int ice40_infrared_enable_vpp(struct infrared_data *spidev)
{
	int ret = 0;
	printk("chenhui ice40_infrared_enable_vpp1\n");
	if (spidev->powered_vpp)
		goto enable_vpp_failed;
printk("chenhui ice40_infrared_enable_vpp2\n");
	if (!IS_ERR(spidev->infrared_vpp))
	{
	ret = regulator_enable(spidev->infrared_vpp); /* 2.8 V */
	if (ret < 0) {
		pr_err("fail to enable infrared_vpp\n");
		goto enable_vpp_failed;
	}
	}
	spidev->powered_vpp = true;

enable_vpp_failed:
	return ret;
}


static int ice40_infrared_disable_vpp(struct infrared_data *spidev)
{
	int ret = 0;

//	return ret;
	printk("chenhui ice40_infrared_disable_vpp1\n");
	if (!spidev->powered_vpp)
		goto disable_vpp_failed;
	printk("chenhui ice40_infrared_disable_vpp2\n");
	if (!IS_ERR(spidev->infrared_vpp))
	{
	
	ret = regulator_disable(spidev->infrared_vpp); /* 2.8 V */
	if (ret < 0) {
		pr_err("fail to disable infrared_vpp\n");
		goto disable_vpp_failed;
	}
    else
		printk("disable infrared_vpp ok \n");

	}
	spidev->powered_vpp = false;

disable_vpp_failed:
	return ret;
}

static int ice40_infrared_enable_iovcc(struct infrared_data *spidev)
{
	int ret = 0;
	if (spidev->powered_vcc)
		goto enable_vcc_failed;
	if (!IS_ERR(spidev->infrared_iovcc))
	{
	ret = regulator_enable(spidev->infrared_iovcc); /* 1.8 V */
	if (ret < 0) {
		pr_err("fail to enable infrared_iovcc\n");
	}
		}
	spidev->powered_vcc = true;
enable_vcc_failed:
	return ret;
}

static int ice40_infrared_disable_iovcc(struct infrared_data *spidev)
{
	int ret = 0;
	if (!spidev->powered_vcc)
		goto disable_vcc_failed;

//	return ret;
	if (!IS_ERR(spidev->infrared_iovcc))
	{
	ret = regulator_disable(spidev->infrared_iovcc); /* 1.8 V */
	if (ret < 0) {
		pr_err("fail to enable infrared_iovcc\n");
	}
	}
	spidev->powered_vcc = false;
disable_vcc_failed:
	return ret;
}


static int ice40_infrared_enable_vdd(struct infrared_data *spidev, int enable)
{
	int ret;

	ret = gpio_direction_output(spidev->vdd_en_gpio, enable);
	if (ret  < 0) {
		pr_err("fail to assert vdd_en_gpio %d\n", ret);
	}

	return ret;
}

static int ice40_infrared_set_clocks(struct infrared_data	*spidev)
{
	int ret = 0;

	if (!of_get_property(spidev->spi->dev.of_node, "clock-names", NULL))
		return 0;

	spidev->infrared_clk = devm_clk_get(&spidev->spi->dev, "infrared_clk");
	if (IS_ERR(spidev->infrared_clk)) {
		ret = PTR_ERR(spidev->infrared_clk);
		if (ret != -EPROBE_DEFER){
			pr_err("fail to get infrared clk %d\n", ret);
			goto set_clocks_failed;
		}
	}

	if (spidev->infrared_clk) {
		ret = clk_set_rate(spidev->infrared_clk, INFRARED_GP0_CLK_FREQ);
		if (ret < 0) {
			pr_err("fail to setting infrared clk %d\n", ret);
			goto set_clocks_failed;
		}
	}

set_clocks_failed:
	return ret;
}

static int ice40_infrared_enable_clocks(struct infrared_data *spidev)
{
	int ret = 0;

	if (spidev->infrared_clk) {
		ret = clk_prepare_enable(spidev->infrared_clk);
		if (ret < 0) {
			pr_err("fail to enable infrared clk %d\n", ret);
		}
	}

	return ret;
}

static int ice40_infrared_disable_clocks(struct infrared_data *spidev)
{
	int ret = 0;

	if (spidev->infrared_clk) {
		clk_disable_unprepare(spidev->infrared_clk);
	}

	return ret;
}


#ifdef ICE40_FIRMWARE_DL
static int ice40_infrared_firmware_transfer(void)
{
	struct infrared_data *spidev = ice40_g_spidev;
	struct spi_message ice40_msg;
	struct spi_transfer ice40_transfer;
	int ret = 0;
	memset(&ice40_transfer, 0, sizeof(ice40_transfer));
	spi_message_init(&ice40_msg);

	ice40_transfer.tx_buf = frameware_bin;
	ice40_transfer.rx_buf = NULL;
	ice40_transfer.len = img_size;

	spi_message_add_tail(&ice40_transfer, &ice40_msg);

	ice40_msg.spi = spidev->spi;
	ret = ice40_infrared_sync(spidev, &ice40_msg);
	if (ret < 0)
	{
		return -1;
	}
	return 0;
}

static int ice40_infrared_download_firmware(struct infrared_data *spidev)
{
	int ret = 0;

	printk("allan-----------down firmware-------\n");

	gpio_request(spidev->slave_select_gpio,"cs down");
//	gpio_tlmm_config(GPIO_CFG(spidev->slave_select_gpio, 0, GPIO_CFG_OUTPUT,GPIO_CFG_PULL_DOWN,
//						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_direction_output(spidev->slave_select_gpio, 0);

	udelay(100);
	gpio_direction_output(spidev->reset_gpio, 0);
	udelay(1000);//200 ns at least
	gpio_direction_output(spidev->reset_gpio, 1);
    udelay(1000);//800 ns at least

//	gpio_tlmm_config(GPIO_CFG(spidev->slave_select_gpio, 1, GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL,
//						GPIO_CFG_6MA), GPIO_CFG_ENABLE);
	gpio_free(spidev->slave_select_gpio);

	ret = ice40_infrared_firmware_transfer();
	if (ret < 0)
	{
		printk("[allan] ~~ firmware download fail\n");
	}
	return ret;
}
#endif

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void ice40_infrared_complete(void *arg)
{
	complete(arg);
}

static ssize_t
ice40_infrared_sync(struct infrared_data *spidev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = ice40_infrared_complete;
	message->context = &done;

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spidev->spi, message);
	spin_unlock_irq(&spidev->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
ice40_infrared_sync_write(struct infrared_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spidev->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return ice40_infrared_sync(spidev, &m);
}

static inline ssize_t
ice40_infrared_sync_read(struct infrared_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= spidev->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return ice40_infrared_sync(spidev, &m);
}

/* Read-only message with current device setup */
static ssize_t
ice40_infrared_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct infrared_data *spidev;
	ssize_t	status = 0;

	pr_info("%s : allan add \n", __func__);

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	status = ice40_infrared_sync_read(spidev, count);
	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, spidev->buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spidev->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t
ice40_infrared_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct infrared_data *spidev;
	ssize_t	status = 0;
	unsigned long missing;
	pr_info("%s : allan add \n", __func__);

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	missing = copy_from_user(spidev->buffer, buf, count);
	if (missing == 0) {
		status = ice40_infrared_sync_write(spidev, count);
	} else
		status = -EFAULT;
	mutex_unlock(&spidev->buf_lock);

	return status;
}

static int ice40_infrared_message(struct infrared_data *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned n, total;
	u8 *buf, *bufrx;
	int	status = -EFAULT;

	pr_info("%s : 1 \n", __func__);
	
	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = spidev->buffer;
	bufrx = spidev->bufferrx;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		pr_info("%s : 3 \n", __func__);
		
		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = bufrx;
/*			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;*/
		}

		if (u_tmp->tx_buf) {

			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		buf += k_tmp->len;
		bufrx += k_tmp->len;
		
		u_tmp->speed_hz = 1520000;
		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
//#ifdef VERBOSE
		dev_err(&spidev->spi->dev,
			"  xfer len %d %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
//#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	pr_info("%s : 2 \n", __func__);
	
	status = ice40_infrared_sync(spidev, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = spidev->bufferrx;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

static long
ice40_infrared_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int	err = 0;
	int	retval = 0;
	struct infrared_data *spidev;
	struct spi_device *spi;
	u32	tmp;
	unsigned n_ioc;
	struct spi_ioc_transfer	*ioc;

	printk("allan:%s\n",__func__);
	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;

	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spidev->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
              printk("[allan] --- SPI_IOC_RD_MODE = %d\n", cmd);
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
              printk("[allan] --- SPI_IOC_RD_LSB_FIRST = %d\n", cmd);
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
              printk("[allan] --- SPI_IOC_RD_BITS_PER_WORD = %d\n", cmd);
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
              printk("[allan] --- SPI_IOC_RD_MAX_SPEED_HZ = %d\n", cmd);
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
              printk("[allan] --- SPI_IOC_WR_MODE = %d\n", cmd);
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			//spi->mode = (u8)tmp;            when testing ,disable mode from APK  by allan
			spi->mode = 1;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
              printk("[allan] --- SPI_IOC_WR_LSB_FIRST = %d\n", cmd);
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
              printk("[allan] --- SPI_IOC_WR_BITS_PER_WORD = %d\n", cmd);
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
              printk("[allan] --- SPI_IOC_WR_MAX_SPEED_HZ = %d\n", cmd);
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = 1520000;         // when testing . disable 10K set from apk  by allan
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;

	default:
              printk("[allan] --- default = %d\n", cmd);
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		printk("[allan] --- ice40_infrared_message \n");
		retval = ice40_infrared_message(spidev, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
ice40_infrared_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return ice40_infrared_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define ice40_infrared_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int ice40_infrared_open(struct inode *inode, struct file *filp)
{
	struct infrared_data *spidev;
	int status = -ENXIO;
	//int	ret = 0;

	pr_info("%s : allan add \n", __func__);

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		/*
		if (!spidev->buffer) {
			spidev->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spidev->buffer) {
				printk("kmalloc fail\n");
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (!spidev->bufferrx) {
			spidev->bufferrx = kmalloc(bufsiz, GFP_KERNEL);
			if (!spidev->bufferrx) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				kfree(spidev->buffer);
				spidev->buffer = NULL;
				status = -ENOMEM;
			}
		}
		*/
		if (status == 0) {
			spidev->users++;
			filp->private_data = spidev;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
/*
#ifndef ICE40_FIRMWARE_DL_TEST
	ret = ice40_infrared_clock_function(spidev, 1);
	if (ret) {
		pr_err("fail to clock_fuc \n");
		goto infrared_open_failed;
	}

	ret = ice40_infrared_enable_clocks(spidev);
	if (ret) {
		pr_err("fail to set clock \n");
		goto infrared_open_failed;
	}

	ret = ice40_infrared_enable_vdd(spidev, 1);
	if (ret) {
		pr_err("fail to enable_vdd \n");
		goto infrared_open_failed;
	}
	mdelay(50);
#endif

#ifndef ICE40_FIRMWARE_DL_TEST
#ifdef ICE40_FIRMWARE_DL
	int i;
	ice40_g_spidev = spidev;
	spidev->spi->mode = SPI_MODE_3;
	spi_setup(spidev->spi);
	mutex_lock(&spidev->buf_lock);
	ret = ice40_infrared_download_firmware(spidev);
	if (ret == 0)
	{
		for (i = 0; i < 1000; i++) {
			ret = gpio_get_value(spidev->config_codne_gpio);
			if (ret) {
				printk("allan------down FW ok------------\n");
				break;
			}
			udelay(1);
		}
	}
	mutex_unlock(&spidev->buf_lock);
#endif
#endif
*/
//infrared_open_failed:
	return status;
}

static int ice40_infrared_release(struct inode *inode, struct file *filp)
{
	struct infrared_data *spidev;
	int	status = 0;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	printk("allan-------powerDown L5 1.2v-------------\n");
/*
#ifdef ICE40_FIRMWARE_DL_TEST
	ice40_infrared_disable_vpp(spidev);
	ice40_infrared_disable_iovcc(spidev);
	//ice40_infrared_enable_vdd(spidev, 0);
	//ice40_infrared_disable_clocks(spidev);
	//ice40_infrared_clock_function(spidev, 0);
#endif
*/
	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int	dofree;

/*		kfree(spidev->buffer);
		spidev->buffer = NULL;
		kfree(spidev->bufferrx);
		spidev->bufferrx = NULL;
*/
		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);

		if (dofree)
			kfree(spidev);
	}
	mutex_unlock(&device_list_lock);
	return status;
}

/*
 * sysfs layer
 */

static ssize_t ir_tx_status(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
        return sprintf( buf, "%d\n", prev_tx_status );
}

static DEVICE_ATTR(txstat, S_IRUGO, ir_tx_status, NULL);

static struct attribute *tx_status_attributes = {
        &dev_attr_txstat.attr,
};
static const struct file_operations ice40_infrared_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	ice40_infrared_write,
	.read =		ice40_infrared_read,
	.unlocked_ioctl = ice40_infrared_ioctl,
	.compat_ioctl = ice40_infrared_compat_ioctl,
	.open =		ice40_infrared_open,
	.release =	ice40_infrared_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *ice40_infrared_class;

/*-------------------------------------------------------------------------*/

static int infrared_creat_lattice_device(void) ;
static int infrared_creat_peel_device(void) ;


static int ice40_infrared_probe(struct spi_device *spi)
{
	struct infrared_data *spidev;
	int	status;
	unsigned long minor;
	int ret;

	pr_info("%s : allan add \n", __func__);

	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	spidev->spi = spi;

#ifdef COMPATIBLE_IR
	if(1){
#else
   if(0){
#endif
	ret = ice40_infrared_parse_dtsi(spidev);
	if (ret) {
		pr_err("fail to parse dt node\n");
		goto infrared_probe_failed;
	}

	ret = ice40_infrared_pinctrl_init(spidev);
	if (ret) {
		pr_err("fail to init pinctrl \n");
		goto infrared_probe_failed;
	}

	ret = ice40_infrared_req_gpios(spidev);
	if (ret) {
		pr_err("request gpio fail\n");
		goto infrared_probe_failed;
	}

	ret = ice40_infrared_set_clocks(spidev);
	if (ret) {
		pr_err("fail to set clock \n");
		goto infrared_probe_failed;
	}

//	gpio_tlmm_config(GPIO_CFG(spidev->vdd_en_gpio, 0, GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL,
//						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
//	gpio_tlmm_config(GPIO_CFG(spidev->config_codne_gpio, 0, GPIO_CFG_INPUT,GPIO_CFG_NO_PULL,
//						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
//	gpio_tlmm_config(GPIO_CFG(spidev->reset_gpio, 0, GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL,
//						GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	ret = ice40_infrared_init_regulators(spidev);
	if (ret) {
		pr_err("fail to init_regulators \n");
		goto infrared_probe_failed;
	}

	ret = ice40_infrared_enable_vpp(spidev);
	if (ret) {
		pr_err("fail to enable_vpp \n");
		goto infrared_probe_failed;
	}

	ret = ice40_infrared_enable_iovcc(spidev);
	if (ret) {
		pr_err("fail to enable_iovcc \n");
		goto infrared_probe_failed;
	}


#ifdef ICE40_FIRMWARE_DL_TEST
	ret = ice40_infrared_clock_function(spidev, 1);
	if (ret) {
		pr_err("fail to clock_fuc \n");
		goto infrared_probe_failed;
	}

	ret = ice40_infrared_enable_clocks(spidev);
	if (ret) {
		pr_err("fail to set clock \n");
		goto infrared_probe_failed;
	}

	ret = ice40_infrared_enable_vdd(spidev, 1);
	if (ret) {
		pr_err("fail to enable_vdd \n");
		goto infrared_probe_failed;
	}
	mdelay(50);

#endif
}

#ifdef COMPATIBLE_IR
	if (gpio_get_value(spidev->config_codne_gpio) == 0){
#else
  if(0){
#endif
		printk("%s : lattice exist \n", __func__);
		spidev->infrared_mode = 1;
		bufsiz = 4096;

		if (infrared_creat_lattice_device() != 0){
			goto infrared_probe_failed;
		}
	}
	else {
		printk("%s : no lattice \n", __func__);
		spidev->infrared_mode = 0;
		bufsiz = 380 * 1024;
#ifdef COMPATIBLE_IR
		ice40_infrared_disable_clocks(spidev);
		ice40_infrared_clock_function(spidev, 0);
		ice40_infrared_enable_vdd(spidev, 0);
#endif

//	if(spidev->infrared_mode == 1){
		if (infrared_creat_peel_device() != 0){
			goto infrared_probe_failed;
		}
//	}		
	}
	if (!spidev->buffer) {
		spidev->buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->buffer) {
			printk("kmalloc fail\n");
			dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto infrared_probe_failed;
		}
	}
	if (!spidev->bufferrx) {
		spidev->bufferrx = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->bufferrx) {
			dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
			kfree(spidev->buffer);
			spidev->buffer = NULL;
			status = -ENOMEM;
			goto infrared_probe_failed;
		}
	}



	
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);

	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, PEELIR_MINORS);
	if (minor < PEELIR_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(PEELIR_MAJOR, minor);

		if(spidev->infrared_mode == 1) {
			dev = device_create(ice40_infrared_class, &spi->dev, spidev->devt,
					spidev, LATTICE_SYSDEV_NAME);
		}
		else {
			dev = device_create(peelir_class, &spi->dev, spidev->devt,
					spidev, PEELIR_SYSDEV_NAME);
		}	

		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;

	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, spidev);
	else
		kfree(spidev);

	if(spidev->infrared_mode == 1) {
	#ifdef ICE40_FIRMWARE_DL
		frameware_bin = kmalloc(img_size, GFP_KERNEL);
		if (!frameware_bin){
			printk("[allan]---kmalloc for fw fail -----\n");
			status = -ENOMEM;
			goto infrared_probe_failed;
		}

		memcpy(frameware_bin,&(ice40_bin[0]),img_size);
	#endif

	#ifdef ICE40_FIRMWARE_DL_TEST
	#ifdef ICE40_FIRMWARE_DL
		ice40_g_spidev = spidev;
		spidev->spi->mode = SPI_MODE_3;
		spi_setup(spidev->spi);
		mutex_lock(&spidev->buf_lock);
		ret = ice40_infrared_download_firmware(spidev);
		if (ret == 0)
		{
			udelay(1);
			for (i = 0; i < 5; i++) {
				ret = gpio_get_value(spidev->config_codne_gpio);
				if (ret) {
					printk("allan------down FW ok------------\n");
					break;
				}
				else {
					printk("allan------down FW failed, second down ------------\n");
					ice40_infrared_download_firmware(spidev);
				}	
				udelay(1);
			}
		}
		mutex_unlock(&spidev->buf_lock);
	#endif

	//ice40_infrared_disable_clocks(spidev);
	//ice40_infrared_clock_function(spidev, 0);
	#endif
}

		/* sysfs entry */
	status = sysfs_create_file(&spi->dev.kobj, tx_status_attributes);
	if( status )
		dev_dbg(&spi->dev, " Error creating sysfs entry " );
		
	return status;

infrared_probe_failed:
	pr_info("%s : ice40_infrared register failed \n", __func__);
	return status;
}

static int  ice40_infrared_remove(struct spi_device *spi)
{
	struct infrared_data *spidev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spidev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);

	
	//chenhui add start

	if(spidev->buffer)
	kfree(spidev->buffer);
//	spidev->buffer = NULL;
	if(spidev->bufferrx)
	kfree(spidev->bufferrx);
//	spidev->bufferrx = NULL;

	//chenhui add end



	list_del(&spidev->device_entry);
	device_destroy(ice40_infrared_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static int  ice40_infrared_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct infrared_data *spidev = spi_get_drvdata(spi);
	printk("allan-------ice40_infrared_suspend-------------\n");
	mutex_lock(&device_list_lock);
if(spidev->infrared_mode == 1)
{
#ifdef ICE40_FIRMWARE_DL_TEST
	ice40_infrared_disable_clocks(spidev);
	ice40_infrared_clock_function(spidev, 0);
	ice40_infrared_enable_vdd(spidev, 0);
#endif
}
//	printk("allan-------ice40_infrared_suspend2-------------\n");
if(0)
{
	ice40_infrared_disable_vpp(spidev);
	ice40_infrared_disable_iovcc(spidev);
}
//chenhui add
	mutex_unlock(&device_list_lock);

	return 0;
}

static int  ice40_infrared_resume(struct spi_device *spi)
{
	struct infrared_data *spidev = spi_get_drvdata(spi);
	int ret = 0;
	int i;

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	if(0)
		{
	ice40_infrared_enable_vpp(spidev);
	ice40_infrared_enable_iovcc(spidev);
		}
	//chenhui add
if(spidev->infrared_mode == 1)
{
#ifdef ICE40_FIRMWARE_DL_TEST
	ice40_infrared_enable_vdd(spidev, 1);
	ice40_infrared_clock_function(spidev, 1);
	ice40_infrared_enable_clocks(spidev);
	mdelay(50);

#ifdef ICE40_FIRMWARE_DL
	ice40_g_spidev = spidev;
	spidev->spi->mode = SPI_MODE_3;
	spi_setup(spidev->spi);
	mutex_lock(&spidev->buf_lock);
	ret = ice40_infrared_download_firmware(spidev);
	if (ret == 0)
	{
		for (i = 0; i < 1000; i++) {
			ret = gpio_get_value(spidev->config_codne_gpio);
			if (ret) {
				printk("allan------down FW ok------------\n");
				break;
			}
			udelay(1);
		}
	}
	mutex_unlock(&spidev->buf_lock);
#endif
#endif
}
	mutex_unlock(&device_list_lock);

	return 0;
}


static struct of_device_id peelir_of_match_table[] = {
	{ .compatible = "infrared,ice40-spi-infrared", },
	{},
};

static struct spi_driver peelir_spi_driver = {
	.driver = {
		.name =		"ice40_spi_infrared",
		.owner =	THIS_MODULE,
		.of_match_table = peelir_of_match_table,
	},
	.probe =	ice40_infrared_probe,
	.remove =	ice40_infrared_remove,
	.suspend =  ice40_infrared_suspend,
	.resume =   ice40_infrared_resume,
};

static int infrared_creat_lattice_device(void) 
{
	int status;
	
	pr_info("%s : allan add \n", __func__);

	BUILD_BUG_ON(ICE40_INFRARED_MINORS > 256);
	status = register_chrdev(ICE40_INFRARED_MAJOR, LATTICE_CHARDEV_NAME, &ice40_infrared_fops);
	if (status < 0)
		return status;

	ice40_infrared_class = class_create(THIS_MODULE, LATTICE_SYSCLS_NAME);
	if (IS_ERR(ice40_infrared_class)) {
		status = PTR_ERR(ice40_infrared_class);
		goto lattice_error_class;
	}

	return 0;

lattice_error_class:
	pr_info("%s : class_create failed \n", __func__);
	unregister_chrdev(ICE40_INFRARED_MAJOR, peelir_spi_driver.driver.name);
	
	return status;
}

static int infrared_creat_peel_device(void) 
{
	int status;
	
	pr_info("%s : allan add \n", __func__);

	BUILD_BUG_ON(ICE40_INFRARED_MINORS > 256);
	status = register_chrdev(ICE40_INFRARED_MAJOR, PEELIR_CHARDEV_NAME, &ice40_infrared_fops);
	if (status < 0)
		return status;

	peelir_class = class_create(THIS_MODULE, PEELIR_SYSCLS_NAME);
	if (IS_ERR(peelir_class)) {
		status = PTR_ERR(peelir_class);
		goto peelir_error_class;
	}

	return 0;

peelir_error_class:
	pr_info("%s : class_create failed \n", __func__);
	unregister_chrdev(ICE40_INFRARED_MAJOR, peelir_spi_driver.driver.name);
	
	return status;
}	

/*-------------------------------------------------------------------------*/
static int __init ice40_infrared_init(void)
{
	int status = 0;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
#if 0	 
	BUILD_BUG_ON(ICE40_INFRARED_MINORS > 256);
	status = register_chrdev(ICE40_INFRARED_MAJOR, INFRARED_CHARDEV_NAME, &ice40_infrared_fops);
	if (status < 0)
		return status;

	ice40_infrared_class = class_create(THIS_MODULE, INFRARED_SYSCLS_NAME);
	if (IS_ERR(ice40_infrared_class)) {
		status = PTR_ERR(ice40_infrared_class);
		goto error_class;
	}

	return 0;

error_class:
	pr_info("%s : class_create failed \n", __func__);
	unregister_chrdev(ICE40_INFRARED_MAJOR, ice40_infrared_driver.driver.name);
	
	return status;

#else
/*	BUILD_BUG_ON(PEELIR_MINORS > 256);
	status = register_chrdev(PEELIR_MAJOR, PEELIR_CHARDEV_NAME, &ice40_infrared_fops);
	if (status < 0)
		return status;
*/
/*
	peelir_class = class_create(THIS_MODULE, PEELIR_SYSCLS_NAME);
	if (IS_ERR(peelir_class)) {
		status = PTR_ERR(peelir_class);
		goto error_class;
	}
*/
	return status;

//error_class:
//	pr_info("%s : class_create failed \n", __func__);
//	unregister_chrdev(PEELIR_MAJOR, peelir_spi_driver.driver.name);
//	return status;
#endif

//	return status;

}
core_initcall(ice40_infrared_init);


static void __exit ice40_infrared_exit(void)
{
	spi_unregister_driver(&peelir_spi_driver);
	class_destroy(peelir_class);
	unregister_chrdev(PEELIR_MAJOR, peelir_spi_driver.driver.name);
}
module_exit(ice40_infrared_exit);

module_spi_driver(peelir_spi_driver);


MODULE_AUTHOR("Allan, <pingao.yang@tcl.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spi_ice40_ir");


