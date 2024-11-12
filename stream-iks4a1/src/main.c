/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/sensor.h>

#define ACCEL_ALIAS(i) DT_ALIAS(_CONCAT(accel, i))
#define ACCELEROMETER_DEVICE(i, _)                                                           \
	IF_ENABLED(DT_NODE_EXISTS(ACCEL_ALIAS(i)), (DEVICE_DT_GET(ACCEL_ALIAS(i)),))
#define NUM_SENSORS 1

/* support up to 10 accelerometer sensors */
static const struct device *const sensors[] = {LISTIFY(10, ACCELEROMETER_DEVICE, ())};

#ifdef CONFIG_SENSOR_ASYNC_API

#define ACCEL_IODEV_SYM(id) CONCAT(accel_iodev, id)
#define ACCEL_IODEV_PTR(id, _) &ACCEL_IODEV_SYM(id)

#define ACCEL_TRIGGERS                                   \
	{SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_INCLUDE}, \
	{SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_DROP}

#define ACCEL_DEFINE_IODEV(id, _)         \
	SENSOR_DT_STREAM_IODEV(               \
		ACCEL_IODEV_SYM(id),              \
		ACCEL_ALIAS(id),                  \
		ACCEL_TRIGGERS);

LISTIFY(NUM_SENSORS, ACCEL_DEFINE_IODEV, (;));

struct rtio_iodev *iodevs[NUM_SENSORS] = { LISTIFY(NUM_SENSORS, ACCEL_IODEV_PTR, (,)) };

RTIO_DEFINE_WITH_MEMPOOL(accel_ctx, NUM_SENSORS, NUM_SENSORS, NUM_SENSORS*20, 256, sizeof(void *));

static int print_accels_stream(const struct device *dev, struct rtio_iodev *iodev)
{
	int rc = 0;
	struct sensor_three_axis_data accel_data = {0};
	struct sensor_three_axis_data gyro_data = {0};
	struct sensor_q31_data temp_data = {0};
	const struct sensor_decoder_api *decoder;
	struct rtio_cqe *cqe;
	uint8_t *buf;
	uint32_t buf_len;
	struct rtio_sqe *handles[NUM_SENSORS];

	/* Start the streams */
	for (int i = 0; i < NUM_SENSORS; i++) {
		printk("sensor_stream\n");
		sensor_stream(iodevs[i], &accel_ctx, NULL, &handles[i]);
	}

	while (1) {
		cqe = rtio_cqe_consume_block(&accel_ctx);

		if (cqe->result != 0) {
			printk("async read failed %d\n", cqe->result);
			return cqe->result;
		}

		rc = rtio_cqe_get_mempool_buffer(&accel_ctx, cqe, &buf, &buf_len);

		if (rc != 0) {
			printk("get mempool buffer failed %d\n", rc);
			return rc;
		}

		const struct device *sensor = dev;

		rtio_cqe_release(&accel_ctx, cqe);

		rc = sensor_get_decoder(sensor, &decoder);

		if (rc != 0) {
			printk("sensor_get_decoder failed %d\n", rc);
			return rc;
		}

		/* Frame iterator values when data comes from a FIFO */
		uint32_t accel_fit = 0, gyro_fit = 0;
		uint32_t temp_fit = 0;


		/* Number of accelerometer data frames */
		uint16_t frame_count;

		rc = decoder->get_frame_count(buf,
				(struct sensor_chan_spec) {SENSOR_CHAN_ACCEL_XYZ, 0}, &frame_count);

		if (rc != 0) {
			printk("sensor_get_decoder failed %d\n", rc);
			return rc;
		}

		/* If a tap has occurred lets print it out */
		if (decoder->has_trigger(buf, SENSOR_TRIG_TAP)) {
			printk("Tap! Sensor %s\n", dev->name);
		}

		/* Decode all available accelerometer sample frames */
		printk("FIFO count - %d\n", frame_count);
		int i = 0;
		while (i < frame_count) {
		        int8_t c = 0;

			c = decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_ACCEL_XYZ, 0},
					&accel_fit, 1, &accel_data);

			for (int k = 0; k < c; k++) {
			    printk("XL data for %s %lluns (%" PRIq(6) ", %" PRIq(6)
					", %" PRIq(6) ") \n", dev->name,
			    PRIsensor_three_axis_data_arg(accel_data, k));
			}
			i += c;

			c = decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_GYRO_XYZ, 0},
					&gyro_fit, 1, &gyro_data);

			for (int k = 0; k < c; k++) {
			    printk("GY data for %s %lluns (%" PRIq(6) ", %" PRIq(6)
					", %" PRIq(6) ") \n", dev->name,
			    PRIsensor_three_axis_data_arg(gyro_data, k));
			}
			i += c;

			c = decoder->decode(buf,
					(struct sensor_chan_spec) {SENSOR_CHAN_DIE_TEMP, 0},
					&temp_fit, 1, &temp_data);

			for (int k = 0; k < c; k++) {
			    printk("TP data for %s %lluns %s%d.%d °C \n", dev->name,
				PRIsensor_q31_data_arg(temp_data, k));
			}
			i += c;

		}

		rtio_release_buffer(&accel_ctx, buf, buf_len);
	}

	return rc;
}
#else

static const enum sensor_channel channels[] = {
	SENSOR_CHAN_ACCEL_X,
	SENSOR_CHAN_ACCEL_Y,
	SENSOR_CHAN_ACCEL_Z,
};

static int print_accels(const struct device *dev)
{
	int ret;
	struct sensor_value accel[3];

	ret = sensor_sample_fetch(dev);
	if (ret < 0) {
		printk("%s: sensor_sample_fetch() failed: %d\n", dev->name, ret);
		return ret;
	}

	for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
		ret = sensor_channel_get(dev, channels[i], &accel[i]);
		if (ret < 0) {
			printk("%s: sensor_channel_get(%c) failed: %d\n", dev->name, 'X' + i, ret);
			return ret;
		}
	}

	printk("%16s [m/s^2]:    (%12.6f, %12.6f, %12.6f)\n", dev->name,
	       sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
	       sensor_value_to_double(&accel[2]));

	return 0;
}
#endif /*CONFIG_SENSOR_ASYNC_API*/

static int set_sampling_freq(const struct device *dev)
{
	int ret;
	struct sensor_value odr;

	ret = sensor_attr_get(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	/* If we don't get a frequency > 0, we set one */
	if (ret != 0 || (odr.val1 == 0 && odr.val2 == 0)) {
		odr.val1 = 100;
		odr.val2 = 0;

		ret = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
				      &odr);

		if (ret != 0) {
			printk("%s : failed to set sampling frequency\n", dev->name);
		}
	}

	return 0;
}

int main(void)
{
	int ret;

	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		if (!device_is_ready(sensors[i])) {
			printk("sensor: device %s not ready.\n", sensors[i]->name);
			return 0;
		}
		set_sampling_freq(sensors[i]);
	}

#ifndef CONFIG_COVERAGE
	while (1) {
#else
	for (int i = 0; i < 5; i++) {
#endif
		for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
#ifdef CONFIG_SENSOR_ASYNC_API
			ret = print_accels_stream(sensors[i], iodevs[i]);
#else
			ret = print_accels(sensors[i]);
#endif /*CONFIG_SENSOR_ASYNC_API*/
			if (ret < 0) {
				return 0;
			}
		}
		k_msleep(1000);
	}
	return 0;
}
