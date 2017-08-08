#
# Makefile for the industrial I/O core.
#

obj-$(CONFIG_IIO_SIMPLE_DUMMY) += iio_dummy.o
iio_dummy-y := iio_simple_dummy.o
iio_dummy-$(CONFIG_IIO_SIMPLE_DUMMY_EVENTS) += iio_simple_dummy_events.o
iio_dummy-$(CONFIG_IIO_SIMPLE_DUMMY_BUFFER) += iio_simple_dummy_buffer.o

obj-$(CONFIG_IIO_DUMMY_EVGEN) += iio_dummy_evgen.o

obj-y += accel/
obj-y += adc/
obj-y += addac/
obj-y += cdc/
obj-y += frequency/
obj-y += gyro/
obj-y += impedance-analyzer/
obj-y += light/
obj-y += magnetometer/
obj-y += meter/
obj-y += resolver/
obj-y += trigger/
obj-$(CONFIG_INV_MPU_IIO)       += inv_mpu/
# LGE_CHANGE_S
# Comment : Invensense MPU6515 driver
# 2014-08-05, chaesun.leem@lge.com
#obj-$(CONFIG_INV_MPU_IIO) += inv_mpu/
# LGE_CHANGE_E
