// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Driver for Goodix Touchscreens
 *
 *  Copyright (c) 2014 Red Hat Inc.
 *  Copyright (c) 2015 K. Merker <merker@debian.org>
 *
 *  This code is based on goodix.c authored by andrew@goodix.com:
 *	
 *  2010 - 2012 Goodix Technology.
 */


#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <asm/unaligned.h>

#define GOODIX_GPIO_INT_NAME		"irq"
#define GOODIX_GPIO_RST_NAME		"reset"

#define GOODIX_MAX_HEIGHT		4096
#define GOODIX_MAX_WIDTH		4096
#define GOODIX_INT_TRIGGER		1
#define GOODIX_CONTACT_SIZE		8
#define GOODIX_MAX_CONTACT_SIZE		5
#define GOODIX_MAX_CONTACTS		5
#define GOODIX_MAX_KEYS			7

#define GOODIX_CONFIG_MAX_LENGTH	240

/* Register defines */
#define GOODIX_REG_COMMAND		0x8040
#define GOODIX_CMD_SCREEN_OFF		0x05

#define GOODIX_READ_COOR_ADDR   0x0f40
#define GOODIX_REG_ID			0x0f7d

#define GOODIX_BUFFER_STATUS_READY	BIT(7)
#define GOODIX_HAVE_KEY			0x0f
#define GOODIX_BUFFER_STATUS_TIMEOUT	20

#define RESOLUTION_LOC		1
#define MAX_CONTACTS_LOC	5
#define TRIGGER_LOC		6

struct goodix_ts_data;

enum goodix_irq_pin_access_method {
	IRQ_PIN_ACCESS_NONE,
	IRQ_PIN_ACCESS_GPIO,
	IRQ_PIN_ACCESS_ACPI_GPIO,
	IRQ_PIN_ACCESS_ACPI_METHOD,
};

struct goodix_chip_data {
	u16 config_addr;
	int config_len;
	int (*check_config)(struct goodix_ts_data *ts, const u8 *cfg, int len);
	void (*calc_config_checksum)(struct goodix_ts_data *ts);
};

struct goodix_chip_id {
	const char *id;
	const struct goodix_chip_data *data;
};

#define GOODIX_ID_MAX_LEN	4

struct goodix_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct goodix_chip_data *chip;
	struct touchscreen_properties prop;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
	struct regulator *avdd28;
	struct regulator *vddio;
	struct gpio_desc *gpiod_int;
	struct gpio_desc *gpiod_rst;
	int gpio_count;
	int gpio_int_idx;
	char id[GOODIX_ID_MAX_LEN + 1];
	u16 version;
	const char *cfg_name;
	bool reset_controller_at_probe;
	bool load_cfg_from_disk;
	struct completion firmware_loading_complete;
	unsigned long irq_flags;
	enum goodix_irq_pin_access_method irq_pin_access_method;
	unsigned int contact_size;
	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	unsigned short keymap[GOODIX_MAX_KEYS];
};

static const unsigned long goodix_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};

//firmware for ployer momo7
static unsigned char firmware[] = {
	0x02, 0x11, 0x03, 0x12, 0x04, 0x13, 0x05, 0x14, 0x06, 0x15, 0x07, 0x16,
	0x08, 0x17, 0x09, 0x18, 0x0A, 0x19, 0x0B, 0x1A, 0xFF, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x03, 0x0D, 0x04, 0x0E, 0x05, 0x0F,
	0x06, 0x10, 0x07, 0x11, 0x08, 0x12, 0xFF, 0x13, 0xFF, 0x0F, 0x10, 0x11,
	0x12, 0x13, 0x1F, 0x03, 0x88, 0x00, 0x00, 0x16, 0x00, 0x00, 0x03, 0x00,
	0x00, 0x02, 0x43, 0x34, 0x30, 0x03, 0x00, 0x05, 0x00, 0x03, 0x20, 0x02,
	0x58, 0x3B, 0x49, 0x37, 0x44, 0x05, 0x00, 0x03, 0x19, 0x05, 0x14, 0x10,
	0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01
};

/**
 * goodix_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int goodix_i2c_read(struct i2c_client *client,
			   u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	__be16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *)&wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

/**
 * goodix_i2c_write - write data to a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to write to.
 * @buf: raw data buffer to write.
 * @len: length of the buffer to write
 */
static int goodix_i2c_write(struct i2c_client *client, u16 reg, const u8 *buf,
			    unsigned len)
{
	u8 *addr_buf;
	struct i2c_msg msg;
	int ret;

	addr_buf = kmalloc(len + 2, GFP_KERNEL);
	if (!addr_buf)
		return -ENOMEM;

	addr_buf[0] = reg >> 8;
	addr_buf[1] = reg & 0xFF;
	memcpy(&addr_buf[2], buf, len);

	msg.flags = 0;
	msg.addr = client->addr;
	msg.buf = addr_buf;
	msg.len = len + 2;

	ret = i2c_transfer(client->adapter, &msg, 1);
	kfree(addr_buf);
	return ret < 0 ? ret : (ret != 1 ? -EIO : 0);
}

static int goodix_i2c_write_u8(struct i2c_client *client, u16 reg, u8 value)
{
	return goodix_i2c_write(client, reg, &value, sizeof(value));
}

static int goodix_ts_read_input_report(struct goodix_ts_data *ts, u8 *data)
{
	unsigned long max_timeout;
	int touch_num = 0;
	int error;

	/*
	 * The 'buffer status' bit, which indicates that the data is valid, is
	 * not set as soon as the interrupt is raised, but slightly after.
	 * This takes around 10 ms to happen, so we poll for 20 ms.
	 */
	max_timeout = jiffies + msecs_to_jiffies(GOODIX_BUFFER_STATUS_TIMEOUT);
	do {
		error = goodix_i2c_read(ts->client, GOODIX_READ_COOR_ADDR, data,
					28);

		goodix_i2c_write(ts->client, 0x8000, data, 0);

		if (error) {
			dev_err(&ts->client->dev, "I2C transfer error: %d\n",
				error);
			return error;
		}

		if(data[0]&0x01)touch_num++;
		if(data[0]&0x02)touch_num++;
		if(data[0]&0x04)touch_num++;
		if(data[0]&0x08)touch_num++;
		if(data[0]&0x10)touch_num++;
		return touch_num;
		

		usleep_range(1000, 2000); /* Poll every 1 - 2 ms */
	} while (time_before(jiffies, max_timeout));

	/*
	 * The Goodix panel will send spurious interrupts after a
	 * 'finger up' event, which will always cause a timeout.
	 */
	return -ENOMSG;
}

static void goodix_ts_report_touch_8b(struct goodix_ts_data *ts, u8 *coor_data, int id)
{
	int input_x = coor_data[2]<<8|coor_data[3];
	int input_y = coor_data[0]<<8|coor_data[1];
	int input_w = coor_data[4];

	//ployer momo7的触摸屏很奇葩，需要特殊处理翻转xy
	input_x=800-(input_x*800)/600;
	input_y=(input_y*600)/800;

	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	touchscreen_report_pos(ts->input_dev, &ts->prop,
			       input_x, input_y, true);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
}


/**
 * goodix_process_events - Process incoming events
 *
 * @ts: our goodix_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void goodix_process_events(struct goodix_ts_data *ts)
{
	u8  point_data[2 + 26];
	int touch_num;
	int i;

	touch_num = goodix_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
		return;

	for (i = 0; i < touch_num; i++)
		goodix_ts_report_touch_8b(ts,
				&point_data[2 + 5 * i],i);

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

/**
 * goodix_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	goodix_process_events(ts);


	return IRQ_HANDLED;
}

static void goodix_free_irq(struct goodix_ts_data *ts)
{
	devm_free_irq(&ts->client->dev, ts->client->irq, ts);
}

static int goodix_request_irq(struct goodix_ts_data *ts)
{
	return devm_request_threaded_irq(&ts->client->dev, ts->client->irq,
					 NULL, goodix_ts_irq_handler,
					 ts->irq_flags, ts->client->name, ts);
}

/**
 * goodix_send_cfg - Write fw config to device
 *
 * @ts: goodix_ts_data pointer
 * @cfg: config firmware to write to device
 * @len: config data length
 */
static int goodix_send_cfg(struct goodix_ts_data *ts, const u8 *cfg, int len)
{
	int error;
	error = goodix_i2c_write(ts->client, 0x0f80, cfg, len);
	goodix_i2c_write(ts->client, 0x8000, cfg, 0);
	/* Let the firmware reconfigure itself, so sleep for 10ms */
	usleep_range(10000, 11000);

	return 0;
}

static int goodix_pin_acpi_direction_input(struct goodix_ts_data *ts)
{
	dev_err(&ts->client->dev,
		"%s called on device without ACPI support\n", __func__);
	return -EINVAL;
}

static int goodix_pin_acpi_output_method(struct goodix_ts_data *ts, int value)
{
	dev_err(&ts->client->dev,
		"%s called on device without ACPI support\n", __func__);
	return -EINVAL;
}


static int goodix_irq_direction_output(struct goodix_ts_data *ts, int value)
{
	switch (ts->irq_pin_access_method) {
	case IRQ_PIN_ACCESS_NONE:
		dev_err(&ts->client->dev,
			"%s called without an irq_pin_access_method set\n",
			__func__);
		return -EINVAL;
	case IRQ_PIN_ACCESS_GPIO:
		return gpiod_direction_output(ts->gpiod_int, value);
	case IRQ_PIN_ACCESS_ACPI_GPIO:
		/*
		 * The IRQ pin triggers on a falling edge, so its gets marked
		 * as active-low, use output_raw to avoid the value inversion.
		 */
		return gpiod_direction_output_raw(ts->gpiod_int, value);
	case IRQ_PIN_ACCESS_ACPI_METHOD:
		return goodix_pin_acpi_output_method(ts, value);
	}

	return -EINVAL; /* Never reached */
}

static int goodix_irq_direction_input(struct goodix_ts_data *ts)
{
	switch (ts->irq_pin_access_method) {
	case IRQ_PIN_ACCESS_NONE:
		dev_err(&ts->client->dev,
			"%s called without an irq_pin_access_method set\n",
			__func__);
		return -EINVAL;
	case IRQ_PIN_ACCESS_GPIO:
		return gpiod_direction_input(ts->gpiod_int);
	case IRQ_PIN_ACCESS_ACPI_GPIO:
		return gpiod_direction_input(ts->gpiod_int);
	case IRQ_PIN_ACCESS_ACPI_METHOD:
		return goodix_pin_acpi_direction_input(ts);
	}

	return -EINVAL; /* Never reached */
}

static int goodix_int_sync(struct goodix_ts_data *ts)
{
	int error;

	error = goodix_irq_direction_output(ts, 0);
	if (error)
		return error;

	msleep(50);				/* T5: 50ms */

	error = goodix_irq_direction_input(ts);
	if (error)
		return error;

	return 0;
}

/**
 * goodix_reset - Reset device during power on
 *
 * @ts: goodix_ts_data pointer
 */
static int goodix_reset(struct goodix_ts_data *ts)
{
	int error;

	/* begin select I2C slave addr */
	error = gpiod_direction_output(ts->gpiod_rst, 0);
	if (error)
		return error;

	msleep(20);				/* T2: > 10ms */

	error = gpiod_direction_output(ts->gpiod_rst, 1);
	if (error)
		return error;

	usleep_range(6000, 10000);		/* T4: > 5ms */

	/* end select I2C slave addr */
	error = gpiod_direction_input(ts->gpiod_rst);
	if (error)
		return error;

	error = goodix_int_sync(ts);
	if (error)
		return error;

	return 0;
}

static int goodix_add_acpi_gpio_mappings(struct goodix_ts_data *ts)
{
	return -EINVAL;
}

/**
 * goodix_get_gpio_config - Get GPIO config from ACPI/DT
 *
 * @ts: goodix_ts_data pointer
 */
static int goodix_get_gpio_config(struct goodix_ts_data *ts)
{
	int error;
	struct device *dev;
	struct gpio_desc *gpiod;
	bool added_acpi_mappings = false;

	if (!ts->client)
		return -EINVAL;
	dev = &ts->client->dev;

	ts->avdd28 = devm_regulator_get(dev, "AVDD28");
	if (IS_ERR(ts->avdd28)) {
		error = PTR_ERR(ts->avdd28);
		if (error != -EPROBE_DEFER)
			dev_err(dev,
				"Failed to get AVDD28 regulator: %d\n", error);
		return error;
	}

	ts->vddio = devm_regulator_get(dev, "VDDIO");
	if (IS_ERR(ts->vddio)) {
		error = PTR_ERR(ts->vddio);
		if (error != -EPROBE_DEFER)
			dev_err(dev,
				"Failed to get VDDIO regulator: %d\n", error);
		return error;
	}

retry_get_irq_gpio:
	/* Get the interrupt GPIO pin number */
	gpiod = devm_gpiod_get_optional(dev, GOODIX_GPIO_INT_NAME, GPIOD_IN);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		if (error != -EPROBE_DEFER)
			dev_dbg(dev, "Failed to get %s GPIO: %d\n",
				GOODIX_GPIO_INT_NAME, error);
		return error;
	}
	if (!gpiod && has_acpi_companion(dev) && !added_acpi_mappings) {
		added_acpi_mappings = true;
		if (goodix_add_acpi_gpio_mappings(ts) == 0)
			goto retry_get_irq_gpio;
	}

	ts->gpiod_int = gpiod;

	/* Get the reset line GPIO pin number */
	gpiod = devm_gpiod_get_optional(dev, GOODIX_GPIO_RST_NAME, GPIOD_IN);
	if (IS_ERR(gpiod)) {
		error = PTR_ERR(gpiod);
		if (error != -EPROBE_DEFER)
			dev_dbg(dev, "Failed to get %s GPIO: %d\n",
				GOODIX_GPIO_RST_NAME, error);
		return error;
	}

	ts->gpiod_rst = gpiod;

	switch (ts->irq_pin_access_method) {
	case IRQ_PIN_ACCESS_ACPI_GPIO:
		/*
		 * We end up here if goodix_add_acpi_gpio_mappings() has
		 * called devm_acpi_dev_add_driver_gpios() because the ACPI
		 * tables did not contain name to index mappings.
		 * Check that we successfully got both GPIOs after we've
		 * added our own acpi_gpio_mapping and if we did not get both
		 * GPIOs reset irq_pin_access_method to IRQ_PIN_ACCESS_NONE.
		 */
		if (!ts->gpiod_int || !ts->gpiod_rst)
			ts->irq_pin_access_method = IRQ_PIN_ACCESS_NONE;
		break;
	case IRQ_PIN_ACCESS_ACPI_METHOD:
		if (!ts->gpiod_rst)
			ts->irq_pin_access_method = IRQ_PIN_ACCESS_NONE;
		break;
	default:
		if (ts->gpiod_int && ts->gpiod_rst) {
			ts->reset_controller_at_probe = true;
			ts->load_cfg_from_disk = true;
			ts->irq_pin_access_method = IRQ_PIN_ACCESS_GPIO;
		}
	}

	return 0;
}

/**
 * goodix_read_version - Read goodix touchscreen version
 *
 * @ts: our goodix_ts_data pointer
 */
static int goodix_read_version(struct goodix_ts_data *ts)
{
	int error;
	u8 buf[3];

	error = goodix_i2c_read(ts->client, GOODIX_REG_ID, buf, sizeof(buf));
	if (error) {
		dev_err(&ts->client->dev, "read version failed: %d\n", error);
		return error;
	}

	ts->version = get_unaligned_le16(&buf[1]);

	dev_info(&ts->client->dev, "ID %s, version: %04x\n", ts->id,
		 ts->version);

	return 0;
}

/**
 * goodix_configure_dev - Finish device initialization
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called from probe to finish initialization of the device.
 * Contains the common initialization code for both devices that
 * declare gpio pins and devices that do not. It is either called
 * directly from probe or from request_firmware_wait callback.
 */
static int goodix_configure_dev(struct goodix_ts_data *ts)
{
	int error;
	int i;

	ts->int_trigger_type = GOODIX_INT_TRIGGER;
	ts->max_touch_num = GOODIX_MAX_CONTACTS;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->name = "Goodix Capacitive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	if (kstrtou16(ts->id, 10, &ts->input_dev->id.product))
		ts->input_dev->id.product = 0x1001;
	ts->input_dev->id.version = ts->version;

	ts->input_dev->keycode = ts->keymap;
	ts->input_dev->keycodesize = sizeof(ts->keymap[0]);
	ts->input_dev->keycodemax = GOODIX_MAX_KEYS;

	/* Capacitive Windows/Home button on some devices */
	for (i = 0; i < GOODIX_MAX_KEYS; ++i) {
		if (i == 0)
			ts->keymap[i] = KEY_LEFTMETA;
		else
			ts->keymap[i] = KEY_F1 + (i - 1);

		input_set_capability(ts->input_dev, EV_KEY, ts->keymap[i]);
	}

	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);


	/* Try overriding touchscreen parameters via device properties */
	touchscreen_parse_properties(ts->input_dev, true, &ts->prop);

	ts->prop.max_x = 800 - 1;
	ts->prop.max_y = 600 - 1;
	ts->max_touch_num = GOODIX_MAX_CONTACTS;
	input_abs_set_max(ts->input_dev,
				ABS_MT_POSITION_X, ts->prop.max_x);
	input_abs_set_max(ts->input_dev,
				ABS_MT_POSITION_Y, ts->prop.max_y);


	error = input_mt_init_slots(ts->input_dev, ts->max_touch_num,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to initialize MT slots: %d", error);
		return error;
	}

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d", error);
		return error;
	}

	ts->irq_flags = goodix_irq_flags[ts->int_trigger_type] | IRQF_ONESHOT;
	error = goodix_request_irq(ts);
	if (error) {
		dev_err(&ts->client->dev, "request IRQ failed: %d\n", error);
		return error;
	}

	return 0;
}

static void goodix_disable_regulators(void *arg)
{
	struct goodix_ts_data *ts = arg;

	regulator_disable(ts->vddio);
	regulator_disable(ts->avdd28);
}

static int goodix_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct goodix_ts_data *ts;
	int error;

	dev_dbg(&client->dev, "I2C Address: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);
	init_completion(&ts->firmware_loading_complete);
	ts->contact_size = GOODIX_CONTACT_SIZE;

	error = goodix_get_gpio_config(ts);
	if (error)
		return error;

	/* power up the controller */
	error = regulator_enable(ts->avdd28);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable AVDD28 regulator: %d\n",
			error);
		return error;
	}

	error = regulator_enable(ts->vddio);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable VDDIO regulator: %d\n",
			error);
		regulator_disable(ts->avdd28);
		return error;
	}

	error = devm_add_action_or_reset(&client->dev,
					 goodix_disable_regulators, ts);
	if (error)
		return error;
	
	goodix_reset(ts);

	goodix_read_version(ts);

	goodix_send_cfg(ts, firmware, sizeof(firmware));

	error = goodix_configure_dev(ts);
	if (error)
		return error;
	

	return 0;
}

static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->load_cfg_from_disk)
		wait_for_completion(&ts->firmware_loading_complete);

	return 0;
}

static int __maybe_unused goodix_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	int error;

	if (ts->load_cfg_from_disk)
		wait_for_completion(&ts->firmware_loading_complete);

	/* We need gpio pins to suspend/resume */
	if (ts->irq_pin_access_method == IRQ_PIN_ACCESS_NONE) {
		disable_irq(client->irq);
		return 0;
	}

	/* Free IRQ as IRQ pin is used as output in the suspend sequence */
	goodix_free_irq(ts);

	/* Output LOW on the INT pin for 5 ms */
	error = goodix_irq_direction_output(ts, 0);
	if (error) {
		goodix_request_irq(ts);
		return error;
	}

	usleep_range(5000, 6000);

	error = goodix_i2c_write_u8(ts->client, GOODIX_REG_COMMAND,
				    GOODIX_CMD_SCREEN_OFF);
	if (error) {
		dev_err(&ts->client->dev, "Screen off command failed\n");
		goodix_irq_direction_input(ts);
		goodix_request_irq(ts);
		return -EAGAIN;
	}

	/*
	 * The datasheet specifies that the interval between sending screen-off
	 * command and wake-up should be longer than 58 ms. To avoid waking up
	 * sooner, delay 58ms here.
	 */
	msleep(58);
	return 0;
}

static int __maybe_unused goodix_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	u8 config_ver;
	int error;

	if (ts->irq_pin_access_method == IRQ_PIN_ACCESS_NONE) {
		enable_irq(client->irq);
		return 0;
	}

	/*
	 * Exit sleep mode by outputting HIGH level to INT pin
	 * for 2ms~5ms.
	 */
	error = goodix_irq_direction_output(ts, 1);
	if (error)
		return error;

	usleep_range(2000, 5000);

	error = goodix_int_sync(ts);
	if (error)
		return error;

	error = goodix_i2c_read(ts->client, ts->chip->config_addr,
				&config_ver, 1);
	
	error = goodix_reset(ts);
	if (error) {
		dev_err(dev, "Controller reset failed.\n");
		return error;
	}

	error = goodix_send_cfg(ts, ts->config, ts->chip->config_len);
	if (error)
		return error;


	error = goodix_request_irq(ts);
	if (error)
		return error;

	return 0;
}

static SIMPLE_DEV_PM_OPS(goodix_pm_ops, goodix_suspend, goodix_resume);

static const struct i2c_device_id goodix_ts_id[] = {
	{ "GDIX1001:00", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, goodix_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id goodix_of_match[] = {
	{ .compatible = "goodix,gt813" },
	{}
};
MODULE_DEVICE_TABLE(of, goodix_of_match);
#endif

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = goodix_ts_id,
	.driver = {
		.name = "Goodix-TS",
		.acpi_match_table = ACPI_PTR(goodix_acpi_match),
		.of_match_table = of_match_ptr(goodix_of_match),
		.pm = &goodix_pm_ops,
	},
};
module_i2c_driver(goodix_ts_driver);

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_AUTHOR("Bastien Nocera <hadess@hadess.net>");
MODULE_DESCRIPTION("Goodix GT813 touchscreen driver");
MODULE_LICENSE("GPL v2");
