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

#define ACCEL_IODEV_SYM(id) CONCAT(accel_iodev, id)
#define ACCEL_IODEV_PTR(id, _) &ACCEL_IODEV_SYM(id)

#define ACCEL_DEFINE_IODEV(id, _)         \
	SENSOR_DT_READ_IODEV(               \
		ACCEL_IODEV_SYM(id),              \
		ACCEL_ALIAS(id),                  \
		{SENSOR_CHAN_ACCEL_XYZ, 0});

LISTIFY(NUM_SENSORS, ACCEL_DEFINE_IODEV, (;));

struct rtio_iodev *iodevs[NUM_SENSORS] = { LISTIFY(NUM_SENSORS, ACCEL_IODEV_PTR, (,)) };

RTIO_DEFINE_WITH_MEMPOOL(accel_ctx, NUM_SENSORS, NUM_SENSORS, NUM_SENSORS*20, 256, sizeof(void *));

int main(void)
{
	int rc;

	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		if (!device_is_ready(sensors[i])) {
			printk("sensor: device %s not ready.\n", sensors[i]->name);
			return 0;
		}
	}

	while (1) {
		for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
			struct device *dev = (struct device *) sensors[i];

			uint8_t buf[128];

			rc = sensor_read(iodevs[i], &accel_ctx, buf, 128);

			if (rc != 0) {
				printk("%s: sensor_read() failed: %d\n", dev->name, rc);
				return rc;
			}

			const struct sensor_decoder_api *decoder;

			rc = sensor_get_decoder(dev, &decoder);

			if (rc != 0) {
				printk("%s: sensor_get_decode() failed: %d\n", dev->name, rc);
				return rc;
			}

			uint32_t accel_fit = 0;
			struct sensor_three_axis_data accel_data = {0};

			/* decode and print Accelerometer FIFO frames */
			decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_ACCEL_XYZ, 0},
				&accel_fit, 8, &accel_data);

			printk("XL data for %s %lluns (%" PRIq(6) ", %" PRIq(6)
			       ", %" PRIq(6) ")\n", dev->name,
			       PRIsensor_three_axis_data_arg(accel_data, 0));
		}
		k_msleep(500);
	}
	return 0;
}
