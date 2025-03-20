/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include "common_utils.h"
#include "otd.h"

#ifdef CONFIG_LIS2DUX12_TRIGGER
static struct k_sem lis2dux12_acc_drdy;
static int lis2dux12_acc_trig_cnt;

static void lis2dux12_acc_trig_handler(const struct device *dev,
				       const struct sensor_trigger *trig)
{
	lis2dux12_acc_trig_cnt++;
	k_sem_give(&lis2dux12_acc_drdy);
}
#endif

static void lis2dux12_config(const struct device *lis2dux12)
{
	struct sensor_value odr_attr, fs_attr;

	/* set LSM6DSV16X accel sampling frequency to 408 Hz */
	odr_attr.val1 = 100;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lis2dux12, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LSM6DSV16X accel\n");
		return;
	}

	sensor_g_to_ms2(8, &fs_attr);

	if (sensor_attr_set(lis2dux12, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set fs for LSM6DSV16X accel\n");
		return;
	}

#ifdef CONFIG_LIS2DUX12_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lis2dux12, &trig, lis2dux12_acc_trig_handler);
#endif
}

static const struct gpio_dt_spec green_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec blue_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec red_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

static int led_pattern(void)
{
	int i;

	/* led 0 */
	if (!gpio_is_ready_dt(&green_gpio)) {
		printk("%s: device not ready.\n", green_gpio.port->name);
		return -1;
	}
	gpio_pin_configure_dt(&green_gpio, GPIO_OUTPUT_INACTIVE);

	/* led 1 */
	if (!gpio_is_ready_dt(&blue_gpio)) {
		printk("%s: device not ready.\n", blue_gpio.port->name);
		return -1;
	}
	gpio_pin_configure_dt(&blue_gpio, GPIO_OUTPUT_INACTIVE);

	/* led 2 */
	if (!gpio_is_ready_dt(&red_gpio)) {
		printk("%s: device not ready.\n", red_gpio.port->name);
		return -1;
	}
	gpio_pin_configure_dt(&red_gpio, GPIO_OUTPUT_INACTIVE);

	/* output led alternate pattern */
	for (i = 0; i < 9; i++) {
		gpio_pin_set_dt(&green_gpio, ((i % 3) == 0) ? 1 : 0);
		gpio_pin_set_dt(&blue_gpio, ((i % 3) == 1) ? 1 : 0);
		gpio_pin_set_dt(&red_gpio, ((i % 3) == 2) ? 1 : 0);
		k_msleep(200);
	}

	/* turn all leds off but blue */
	gpio_pin_set_dt(&green_gpio, 0);
	gpio_pin_set_dt(&blue_gpio, 1);
	gpio_pin_set_dt(&red_gpio, 0);

	return 0;
}

int main(void)
{
	struct sensor_value lis2dux12_xl[3];
	int16_t acc_data[3];
#ifdef CONFIG_LIS2DUX12_ENABLE_TEMP
	struct sensor_value lis2dux12_temp;
#endif
	const struct device *const lis2dux12 = DEVICE_DT_GET_ONE(st_lis2dux12);

	led_pattern();

#ifdef CONFIG_LIS2DUX12_TRIGGER
	k_sem_init(&lis2dux12_acc_drdy, 0, K_SEM_MAX_LIMIT);
#endif

	if (!device_is_ready(lis2dux12)) {
		printk("%s: device not ready.\n", lis2dux12->name);
		return 0;
	}
	printk("%s: device is ready\n", lis2dux12->name);

	lis2dux12_config(lis2dux12);

	// Get Utils library version
	char common_utils_ver[12];

	common_utils_get_version(common_utils_ver, 12);
	printf("[LIB] Common utils library version: %s\n", common_utils_ver);

	// Initialize On-Table Detection library
	otd_init_status_t otd_init_status;
	otd_state_t *otd_state;
	uint8_t otd_decimator;

	char otd_ver[12];
	otd_get_version(otd_ver, 12);
	printf("[LIB] On-Table Detection version: %s\n", otd_ver);

	otd_state = otd_get_instance(0);
	if (otd_state == NULL)
	{
		printf("[LIB] On-Table Detection null algorithm instance\n");
		return -1;
	}

	otd_model_t otd_model;
	otd_config_t otd_config;
	otd_model = OTD_MODEL_2;
	otd_config = otd_get_default_config();
	otd_config.hyst_ontable_ths = 20.0f;
	otd_config.hyst_onlap_ths = 5.0f;
	otd_config.hyst_other_ths = 5.0f;

	otd_init_status = otd_init(otd_state, &otd_config, OTD_META_DECREMENT, otd_model);
	if (otd_init_status != OTD_INIT_SUCCESS)
	{
		printf("[LIB] On-Table Detection init failed with error %d\n", otd_init_status);
		return -1;
	}
	otd_decimator = 4;

	while (1) {
		//gpio_pin_toggle_dt(&green_gpio);
		//gpio_pin_toggle_dt(&green_gpio);
		/* Get sensor samples */

#ifdef CONFIG_LIS2DUX12_TRIGGER
		k_sem_take(&lis2dux12_acc_drdy, K_FOREVER);
#else
		k_sleep(K_MSEC(10));
#endif
		if (sensor_sample_fetch(lis2dux12) < 0) {
			printf("LIS2DUX12 XL Sensor sample update error\n");
			return 0;
		}

		/* Get sensor data */
		sensor_channel_get(lis2dux12, SENSOR_CHAN_ACCEL_XYZ, lis2dux12_xl);

		otd_output_t otd_out_raw;
		otd_output_t otd_out_meta;
		otd_input_t otd_in;

		for (uint8_t i = 0; i < 3; i++) {
			acc_data[i] = sensor_ms2_to_mg(&lis2dux12_xl[i]);
			otd_in.acc[i] = acc_data[i];
		}

		if (otd_run(otd_state, &otd_out_raw, &otd_out_meta, &otd_in))
		{
			printf("[LIB] Current On-Table Detection output:\t%d\n", otd_out_meta);
		}

#ifdef CONFIG_PRINT_ACCEL_DATA
		/* Display sensor data */
		printf("LIS2DUX12: Accel (m.s-2): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&lis2dux12_xl[0]),
			sensor_value_to_double(&lis2dux12_xl[1]),
			sensor_value_to_double(&lis2dux12_xl[2]));
#endif

#ifdef CONFIG_LIS2DUX12_TRIGGER
		if (lis2dux12_acc_trig_cnt % 100 == 99)
			printf("lis2dux12 acc trig %d\n", lis2dux12_acc_trig_cnt);
#endif
	}

	return 0;
}
