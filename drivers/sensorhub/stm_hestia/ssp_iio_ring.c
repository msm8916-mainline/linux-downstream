/*
 *  Copyright (C) 2011, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>

#include "ssp.h"

//#include "inv_mpu_iio.h"

void ssp_iio_unconfigure_ring(struct iio_dev *indio_dev)
{
	iio_kfifo_free(indio_dev->buffer);
};

static int ssp_predisable(struct iio_dev *indio_dev)
{
	/* sch0317 need to implement
	struct inv_mpu_state  *st = iio_priv(indio_dev);
	int result;

	if (st->chip_config.enable) {
		result = set_inv_enable(indio_dev, false);
		if (result)
			return result;
		result = st->set_power_state(st, false);
		if (result)
			return result;
	}
*/
	return 0;
}

#if 0
static int inv_check_conflict_sysfs(struct iio_dev *indio_dev)
{
	struct inv_mpu_state  *st = iio_priv(indio_dev);

	if (st->chip_config.lpa_mode) {
		/* dmp cannot run with low power mode on */
		st->chip_config.dmp_on = 0;
		st->chip_config.gyro_enable = false;
		st->sensor[SENSOR_GYRO].on = false;
		st->sensor[SENSOR_COMPASS].on = false;
	}
	if (st->sensor[SENSOR_GYRO].on &&
		(!st->chip_config.gyro_enable)) {
		st->chip_config.gyro_enable = true;
	}
	if (st->sensor[SENSOR_ACCEL].on &&
		(!st->chip_config.accel_enable)) {
		st->chip_config.accel_enable = true;
	}

	return 0;
}
#endif

static int ssp_preenable(struct iio_dev *indio_dev)
{
	int result;

	//result = inv_check_conflict_sysfs(indio_dev);
	//if (result)
	//	return result;
	result = iio_sw_buffer_preenable(indio_dev);

	return result;
}

#if 0
void inv_init_sensor_struct(struct inv_mpu_state *st)
{
	int i;

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (i < SENSOR_SIXQ)
			st->sensor[i].sample_size =
					HEADERED_NORMAL_BYTES;
		else
			st->sensor[i].sample_size = HEADERED_Q_BYTES;
		if (i == SENSOR_STEP) {
			st->sensor[i].rate = 1;
			st->sensor[i].dur = NSEC_PER_SEC;
		} else {
			st->sensor[i].rate = INIT_DMP_OUTPUT_RATE;
			st->sensor[i].dur  = NSEC_PER_SEC /
						INIT_DMP_OUTPUT_RATE;
		}
	}

	st->sensor[SENSOR_ACCEL].send_data     = inv_send_accel_data;
	st->sensor[SENSOR_GYRO].send_data      = inv_send_gyro_data;
	st->sensor[SENSOR_COMPASS].send_data   = inv_send_compass_dmp_data;
	st->sensor[SENSOR_PRESSURE].send_data  = inv_send_pressure_dmp_data;
	st->sensor[SENSOR_STEP].send_data      = inv_send_step_detector;
	st->sensor[SENSOR_PEDQ].send_data      = inv_send_ped_q_data;
	st->sensor[SENSOR_SIXQ].send_data      = inv_send_six_q_data;
	st->sensor[SENSOR_LPQ].send_data       = inv_send_three_q_data;

	st->sensor[SENSOR_ACCEL].set_rate     = inv_set_accel_rate;
	st->sensor[SENSOR_GYRO].set_rate      = inv_set_gyro_rate;
	st->sensor[SENSOR_COMPASS].set_rate   = inv_set_compass_rate;
	st->sensor[SENSOR_PRESSURE].set_rate  = inv_set_pressure_rate;
	st->sensor[SENSOR_STEP].set_rate      = inv_set_step_detector;
	st->sensor[SENSOR_PEDQ].set_rate      = inv_set_pedq_rate;
	st->sensor[SENSOR_SIXQ].set_rate      = inv_set_sixq_rate;
	st->sensor[SENSOR_LPQ].set_rate       = inv_set_lpq_rate;
}

void batch_step_only_work(struct work_struct *work)
{
	struct inv_mpu_state *st = container_of((struct delayed_work *)work,
						struct inv_mpu_state, work);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	u32 delay = msecs_to_jiffies(st->batch.timeout);
	u8 data[FIFO_COUNT_BYTE];
	int result;

	mutex_lock(&indio_dev->mlock);
	if (!(iio_buffer_enabled(indio_dev)) || (!st->chip_config.enable))
		goto error_ret;
	schedule_delayed_work(&st->work, delay);

	if (st->batch.post_isr_run) {
		st->batch.post_isr_run = false;
	} else {
		result = inv_i2c_read(st, st->reg.fifo_count_h,
						FIFO_COUNT_BYTE, data);
		if (result)
			goto error_ret;
		st->fifo_count = be16_to_cpup((__be16 *)(data));
		process_step_only_batch(st);
	}

error_ret:
	mutex_unlock(&indio_dev->mlock);

}

int inv_flush_batch_data(struct iio_dev *indio_dev, bool *has_data)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct inv_reg_map_s *reg;
	u8 data[2];
	int result;

#define EMPTY_MARKER 0x0020

	reg = &st->reg;
	if (!(iio_buffer_enabled(indio_dev)) || (!st->chip_config.enable))
		return -EINVAL;

	if (st->batch.on) {
		result = inv_read_time_and_ticks(st, false);
		if (result)
			return result;
		result = inv_i2c_read(st, reg->fifo_count_h,
					FIFO_COUNT_BYTE, data);
		if (result)
			return result;
		st->fifo_count = be16_to_cpup((__be16 *)(data));
		if (st->fifo_count) {
			result = inv_process_batchmode(st, true);
			if (result)
				return result;
			*has_data = !!st->fifo_count;

			return 0;
		}
	}

	inv_push_marker_to_buffer(st, EMPTY_MARKER);

	return 0;
}
#endif

static const struct iio_buffer_setup_ops ssp_iio_ring_setup_ops = {
	.preenable = &ssp_preenable,
	.predisable = &ssp_predisable,
};

int ssp_iio_configure_ring(struct iio_dev *indio_dev)
{
	struct iio_buffer *ring;

	ring = iio_kfifo_allocate(indio_dev);
	if (!ring)
		return -ENOMEM;

	ring->bytes_per_datum = 8;
	indio_dev->buffer = ring;
	/* setup ring buffer */
	ring->scan_timestamp = true;
	indio_dev->setup_ops = &ssp_iio_ring_setup_ops;
	/*scan count double count timestamp. should subtract 1. but
	number of channels still includes timestamp*/

	indio_dev->modes |= INDIO_BUFFER_HARDWARE; // INDIO_BUFFER_TRIGGERED

	return 0;
}


