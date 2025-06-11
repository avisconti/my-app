/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

#include <stdio.h>

static void lps27dewo_config(const struct device *lps27dewo)
{
	struct sensor_value odr_attr;

	/* set ILPS22QS sampling frequency to 50 Hz */
	odr_attr.val1 = 50;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lps27dewo, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for ILPS22QS\n");
		return;
	}
}

int main(void)
{
	int cnt = 1;

	const struct device *const lps27dewo = DEVICE_DT_GET_ONE(st_lps27dewo);

	if (!device_is_ready(lps27dewo)) {
		printk("%s: device not ready.\n", lps27dewo->name);
		return 0;
	}

	lps27dewo_config(lps27dewo);

	while (1) {
		struct sensor_value lps27dewo_press, lps27dewo_temp;

		if (sensor_sample_fetch(lps27dewo) < 0) {
			printf("ILPS22QS Sensor sample update error\n");
			return 0;
		}

		sensor_channel_get(lps27dewo, SENSOR_CHAN_AMBIENT_TEMP, &lps27dewo_temp);
		sensor_channel_get(lps27dewo, SENSOR_CHAN_PRESS, &lps27dewo_press);

		/* Display sensor data */

		/*  Clear terminal (ANSI ESC-C) */
		printf("\0033\014");

		printf("STWIN.box dashboard\n\n");

		/* temperature */
		printf("ILPS22QS: Temperature: %.1f C\n",
		       sensor_value_to_double(&lps27dewo_temp));

		/* pressure */
		printf("ILPS22QS: Pressure: %.3f kpa\n",
		       sensor_value_to_double(&lps27dewo_press));

		k_msleep(2000);
	}
}
