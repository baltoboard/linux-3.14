/* Source for:
 * Evervision VGG804808 touchscreen driver.
 * drivers/input/touchscreen/vgg804808_ts.c
 *
 * Copyright (C) 2014, Ketron Srl. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */
#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/input/mt.h>
#include <linux/input/vgg804808_ts.h>

#define VGG804808_TS_MAX_FINGERS	(5)
#define VGG804808_TS_SCANTIME_MS	(25)
#define VGG804808_TS_STATUS_REG		(2)
#define VGG804808_TS_INVALID_DATA	(0xff)

#define VGG804808_TS_DUMMY_PRESSURE	(0x7f)

#define VGG804808_TS_PEN_DOWN		(0x00)
#define VGG804808_TS_PEN_UP		(0x01)
#define VGG804808_TS_PEN_CONTACT	(0x02)
#define VGG804808_TS_NO_EVENT		(0x03)

#define VGG804808_MAX_VALUE		(0x07ff)

#define BB2W(x) (x[0] << 8 | x[1])

#undef USE_SMBUS

struct vgg804808_ts_touch_record {
	u8 x[2];
	u8 y[2];
	u8 dummy[2];
};

struct vgg804808_ts_touch_data {
	u8 status;	
	struct vgg804808_ts_touch_record record[VGG804808_TS_MAX_FINGERS];
};

struct vgg804808_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	struct workqueue_struct *wq;
	struct vgg804808_ts_platform_data *platform_data;
	struct vgg804808_ts_touch_data touch_data;
	u8 previous_touches;
	u16 max_x;
	u16 max_y;
	u8 invert_x;
	u8 invert_y;
	u8 swap_xy;
	u32 scantime_jiffies;
};

#ifdef NEEDED
static s32 vgg804808_ts_write_reg_u8(struct i2c_client *client, u8 reg, u8 val)
{
#ifdef USE_SMBUS
	s32 data;

	data = i2c_smbus_write_byte_data(client, reg, val);
	if (data < 0)
		dev_err(&client->dev, "error %d in writing reg 0x%x\n",
						 data, reg);

	return data;
#else
	struct i2c_msg msg;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;

	msg.addr = client->addr;
	msg.len = 2;
	msg.flags = 0;
	msg.buf = buf;

	return i2c_transfer(client->adapter, &msg, 1);

#endif
}

static s32 vgg804808_ts_read_reg_u8(struct i2c_client *client, u8 reg)
{
	s32 data;

	data = i2c_smbus_read_byte_data(client, reg);
	if (data < 0)
		dev_err(&client->dev, "error %d in reading reg 0x%x\n",
						 data, reg);

	return data;
}
#endif /* NEEDED */

static int vgg804808_ts_read(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = &reg;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = len;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	return i2c_transfer(client->adapter, xfer_msg, 2);
}

static void report_data(struct vgg804808_ts *ts, u16 x, u16 y, u8 id, u8 counter)
{
	static int delta = 0;

	if (delta > 4)
		delta = -4;
		
	/* handle inverting coordinates */
	if (ts->invert_x)
		x = ts->max_x - x;
	if (ts->invert_y)
		y = ts->max_y - y;

	if (ts->swap_xy)
		swap(x, y);

	input_report_abs(ts->input, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input, ABS_MT_PRESSURE, VGG804808_TS_DUMMY_PRESSURE + delta);
	input_mt_sync(ts->input);

	if (counter == 0) {
		/* funny world: if the pressure is not reported as changed,
		 * nothing happen at application layer
		 */
		input_report_abs(ts->input, ABS_X, x);
		input_report_abs(ts->input, ABS_Y, y);
		input_report_abs(ts->input, ABS_PRESSURE, VGG804808_TS_DUMMY_PRESSURE + delta);
	}

	delta++;
}

static void process_data(struct vgg804808_ts *ts)
{
	u8 id, event, touches, valid_touches, i;
	u16 x, y;

	touches = ts->touch_data.status & 0x0f;
	valid_touches = 0;

	for (i = 0; i < touches; i++) {
#if 0
		dev_dbg(&ts->client->dev, "Touch %d/%d: 0x%02x%02x 0x%02x%02x\n",
				i, ts->touch_data.status,
				ts->touch_data.record[i].x[0],
				ts->touch_data.record[i].x[1],
				ts->touch_data.record[i].y[0],
				ts->touch_data.record[i].y[1]);
#endif
		x = BB2W(ts->touch_data.record[i].x);
		y = BB2W(ts->touch_data.record[i].y);
		event = x >> 14;
		id = y >> 12;
		x &= VGG804808_MAX_VALUE;
		y &= VGG804808_MAX_VALUE;

		if ((event == VGG804808_TS_PEN_DOWN) || (event == VGG804808_TS_PEN_CONTACT)) {
			report_data(ts, x, y, id, valid_touches);
			valid_touches++;
		}
	}

	if (valid_touches && !(ts->previous_touches))
		input_report_key(ts->input, BTN_TOUCH, 1);
	else if (!valid_touches && (ts->previous_touches))
		input_report_key(ts->input, BTN_TOUCH, 0);

	/* touch firmware reports dummy sequence of zero touches,
	 * just report first one and filter the others
	 */
	if (valid_touches || ts->previous_touches)
		input_sync(ts->input);

	ts->previous_touches = valid_touches;
}

static void vgg804808_ts_xy_worker(struct work_struct *work)
{
	int rc;
	struct vgg804808_ts *ts = container_of(work, struct vgg804808_ts,
				 work.work);

	/* read touch data */
	rc = vgg804808_ts_read(ts->client, VGG804808_TS_STATUS_REG,
			(u8*)&(ts->touch_data), sizeof(ts->touch_data));
	if (rc < 0) {
		dev_err(&ts->client->dev, "touch data read failed\n");
		goto schedule;
	}

	if (ts->touch_data.status == VGG804808_TS_INVALID_DATA)
		dev_warn(&ts->client->dev, "touch data read failed\n");
	else
		process_data(ts);

schedule:
	if (ts->client->irq)
		enable_irq(ts->client->irq);
	else
		queue_delayed_work(ts->wq, &ts->work, ts->scantime_jiffies);
}

static irqreturn_t vgg804808_ts_irq(int irq, void *dev_id)
{
	struct vgg804808_ts *ts = dev_id;

	disable_irq_nosync(irq);

	queue_delayed_work(ts->wq, &ts->work, 0);

	return IRQ_HANDLED;
}

static int vgg804808_ts_open(struct input_dev *dev)
{
	struct vgg804808_ts *ts = input_get_drvdata(dev);

	if (!ts->client->irq) {
		queue_delayed_work(ts->wq, &ts->work, ts->scantime_jiffies);
	}

	return 0;
}

static void vgg804808_ts_close(struct input_dev *dev)
{
	struct vgg804808_ts *ts = input_get_drvdata(dev);

	cancel_delayed_work_sync(&ts->work);
}

static int vgg804808_ts_reset(struct i2c_client *client,
					 int reset_pin)
{
	int error;

	if (gpio_is_valid(reset_pin)) {
		/* this pulls reset down, enabling the low active reset */
		error = devm_gpio_request_one(&client->dev, reset_pin,
					      GPIOF_OUT_INIT_LOW,
					      "vgg804808 reset");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as reset pin, error %d\n",
				reset_pin, error);
			return error;
		}

		mdelay(50);
		gpio_set_value(reset_pin, 1);
		mdelay(100);
	}

	return 0;
}

static int vgg804808_ts_init(struct i2c_client *client, struct vgg804808_ts *ts)
{
	struct input_dev *input_device;
	int rc = 0;
	unsigned char infobuf[8];

	rc = vgg804808_ts_read(client, 0xa1, infobuf, sizeof(infobuf));
	if (rc < 0)
		return rc;

	dev_dbg(&client->dev, "Lib version 0x%02x%02x\n", infobuf[0], infobuf[1]);
	dev_dbg(&client->dev, "Cypher 0x%02x\n", infobuf[2]);
	switch (infobuf[3]) {
		case 0: dev_dbg(&client->dev, "Interrupt Mode polling\n"); break;
		case 1: dev_dbg(&client->dev, "Interrupt Mode trigger\n"); break;
		default:dev_dbg(&client->dev, "Interrupt Mode %d\n", infobuf[3]);
	}
	switch (infobuf[4]) {
		case 0: dev_dbg(&client->dev, "Power Mode active\n"); break;
		case 1: dev_dbg(&client->dev, "Power Mode monitor\n"); break;
		case 2: dev_dbg(&client->dev, "Power Mode hibernate\n"); break;
		default: dev_dbg(&client->dev, "Power Mode %d (unknown)\n", infobuf[4]);
	}
	dev_dbg(&client->dev, "Firmware ID 0x%02x\n", infobuf[5]);
	switch (infobuf[6]) {
		case 0: dev_dbg(&client->dev, "State configure\n"); break;
		case 1: dev_dbg(&client->dev, "State work\n"); break;
		case 2: dev_dbg(&client->dev, "State calibration\n"); break;
		case 3: dev_dbg(&client->dev, "State factory\n"); break;
		case 4: dev_dbg(&client->dev, "State autocalibration\n"); break;
		default: dev_dbg(&client->dev, "State %d (unknown)\n", infobuf[6]);
	}
	dev_dbg(&client->dev, "Vendor ID 0x%02x\n", infobuf[7]);

	if (ts->platform_data) {
		dev_dbg(&client->dev, "platform data available\n");
		ts->swap_xy = ts->platform_data->flags & VGG804808_TS_SWAP_XY;
		ts->invert_x = ts->platform_data->flags & VGG804808_TS_INVERT_X;
		ts->invert_y = ts->platform_data->flags & VGG804808_TS_INVERT_Y;
		vgg804808_ts_reset(client, ts->platform_data->reset_pin);
	} else {
		dev_dbg(&client->dev, "platform data unavailable\n");
		ts->swap_xy = 1;
		ts->invert_x = 0;
		ts->invert_y = 0;
	}

	dev_dbg(&client->dev, "swap_xy %d\n", ts->swap_xy);
	dev_dbg(&client->dev, "invert_x %d\n", ts->invert_x);
	dev_dbg(&client->dev, "invert_y %d\n", ts->invert_y);

	ts->scantime_jiffies = msecs_to_jiffies(VGG804808_TS_SCANTIME_MS);

	ts->previous_touches = 0;

	input_device = input_allocate_device();
	if (!input_device) {
		rc = -ENOMEM;
		goto error_alloc_dev;
	}

	ts->input = input_device;
	input_device->name = "Evervision VGG804808 Touchscreen";
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;
	input_set_drvdata(input_device, ts);

	__set_bit(EV_ABS, input_device->evbit);

	input_set_abs_params(input_device, ABS_X, 0, VGG804808_MAX_VALUE, 0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, VGG804808_MAX_VALUE, 0, 0);
	input_set_abs_params(input_device, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_device, ABS_MT_POSITION_X, 0, VGG804808_MAX_VALUE, 0, 0);
	input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0, VGG804808_MAX_VALUE, 0, 0);
	input_set_abs_params(input_device, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_device, ABS_MT_TRACKING_ID, 0, VGG804808_TS_MAX_FINGERS - 1, 0, 0);

	input_set_capability(input_device, EV_KEY, BTN_TOUCH);

	rc = request_irq(ts->client->irq, vgg804808_ts_irq, 0,
				ts->client->dev.driver->name, ts);
	if (rc) {
		dev_err(&ts->client->dev, "could not request irq\n");
		goto error_alloc_dev;
	}

	ts->wq = create_singlethread_workqueue("vgg804808_kworkqueue");
	if (!ts->wq) {
		dev_err(&client->dev, "Could not create workqueue\n");
		goto error_wq_create;
	}

	INIT_DELAYED_WORK(&ts->work, vgg804808_ts_xy_worker);

	input_device->open = vgg804808_ts_open;
	input_device->close = vgg804808_ts_close;

	rc = input_register_device(input_device);
	if (rc)
		goto error_unreg_device;

	return 0;

error_unreg_device:
	destroy_workqueue(ts->wq);
error_wq_create:
	input_free_device(input_device);
error_alloc_dev:
	return rc;
}

#ifdef CONFIG_PM
static int vgg804808_ts_suspend(struct device *dev)
{
	struct vgg804808_ts *ts = dev_get_drvdata(dev);

	if (ts->client->irq)
		disable_irq_nosync(ts->client->irq);

	cancel_delayed_work_sync(&ts->work);

	return 0;
}

static int vgg804808_ts_resume(struct device *dev)
{
	struct vgg804808_ts *ts = dev_get_drvdata(dev);

	if (ts->client->irq)
		enable_irq(ts->client->irq);
	else
		queue_delayed_work(ts->wq, &ts->work, ts->scantime_jiffies);

	return 0;
}

static struct dev_pm_ops vgg804808_ts_pm_ops = {
	.suspend	= vgg804808_ts_suspend,
	.resume		= vgg804808_ts_resume,
};
#endif

static int vgg804808_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct vgg804808_ts *ts;
	struct vgg804808_ts_platform_data *pdata = client->dev.platform_data;
	int rc;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -EIO;
	}


	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	ts->platform_data = pdata;
	i2c_set_clientdata(client, ts);

	rc = vgg804808_ts_init(client, ts);
	if (rc < 0) {
		dev_err(&client->dev, "vgg804808 init failed\n");
		goto error_touch_data_alloc;
	}

	return 0;

error_touch_data_alloc:
	kfree(ts);
	return rc;
}

static int vgg804808_ts_remove(struct i2c_client *client)
{
	struct vgg804808_ts *ts = i2c_get_clientdata(client);
	int rc;

	rc = cancel_delayed_work_sync(&ts->work);

	if (rc && ts->client->irq)
		disable_irq_nosync(ts->client->irq);

	if (ts->client->irq)
		free_irq(ts->client->irq, ts);

	destroy_workqueue(ts->wq);

	input_unregister_device(ts->input);

	kfree(ts);

	return 0;
}

static const struct i2c_device_id vgg804808_ts_id[] = {
	{"vgg804808_ts", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, vgg804808_ts_id);


static struct i2c_driver vgg804808_ts_driver = {
	.driver = {
		.name = "vgg804808_ts",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &vgg804808_ts_pm_ops,
#endif
	},
	.probe		= vgg804808_ts_probe,
	.remove		= vgg804808_ts_remove,
	.id_table	= vgg804808_ts_id,
};

static int __init vgg804808_ts_module_init(void)
{
	return i2c_add_driver(&vgg804808_ts_driver);
}
/* Making this as late init to avoid power fluctuations
 * during LCD initialization.
 */
late_initcall(vgg804808_ts_module_init);

static void __exit vgg804808_ts_module_exit(void)
{
	return i2c_del_driver(&vgg804808_ts_driver);
}
module_exit(vgg804808_ts_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Evervision VGG804808 touchscreen controller driver");
MODULE_AUTHOR("Pierluigi Passaro <info@phoenixsoftware.it>");
MODULE_ALIAS("platform:vgg804808_ts");
