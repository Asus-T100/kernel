//<asus-wy20131101>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/acpi_gpio.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/switch.h>

#include <linux/hid.h>
#include <linux/hiddev.h>
#include <linux/hid-debug.h>
#include <linux/hidraw.h>

static int debug = 0;

#define pen_err(fmt, arg...) pr_err(fmt, ##arg)

#define pen_dbg(fmt, arg...)					  \
do {									  \
	if (debug)							  \
		pr_err(fmt, ##arg); \
} while (0)

#define I2C_HID_RESET_PENDING	(1 << 1)

#define WACOM_INPUT_REPORT_ID    0x02
#define WACOM_FEATURE_REPORT_ID  0x03

#pragma pack(1)
struct i2c_hid_desc {
        u16  hid_desc_len;
        u16  bcd_version;
        u16  report_desc_len;
        u16  report_desc_reg;
        u16  input_reg;
        u16  input_len;
        u16  output_reg;
        u16  output_len;
        u16  command_reg;
        u16  data_reg;
        u16  vendor_id;
        u16  product_id;
        u16  version_id;
        u32  reserved;
};

struct i2c_hid_command {
	u16	command_reg;
	u8	report_type_id;
	u8	opcode;
};

struct pen_hid_input {
	//HID over I2C spec.
	u16  	input_len;
	u8	report_id;
	//Wacom pen format
        u8      tip_switch;
        u8      x_low;
        u8      x_high;
        u8      y_low;
        u8      y_high;
        u8      p_low;    //pressure
        u8      p_high;
};

struct pen_hid_feature {
	//HID over I2C spec.
	u16  	length;
	u8	report_id;
	//Wacom pen format
        u16     logical_x_max;
        u16     logical_y_max;
        u16     physical_x_max;
        u16     physical_y_max;
        u16     pressure_max;
        u16     fw_version;
        u32     reserved;
};
#pragma pack()

//private data of HID I/O driver
struct i2c_pen_data {
	struct i2c_client       *client;
	int                     gpio_num;
	int                     irq;
	struct hid_device       *hid;
	unsigned long flags;
	wait_queue_head_t	wait;/* For waiting the interrupt */

	struct i2c_hid_desc	hid_desc;
	u16		        hid_desc_addr;

	struct 	pen_hid_input	*input_buffer;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend    early_suspend;
#endif
};

//private data of HID Device driver
struct hid_pen_data {
	struct switch_dev       pen_sdev;
        //feature report
        u16               logical_x_max;
        u16               logical_y_max;
        u16               physical_x_max;
        u16               physical_y_max;
        u16               pressure_max;
        u16               fw_version;
};

//HID Device driver implementation
static ssize_t asuspen_switch_name(struct switch_dev *sdev, char *buf)
{
	struct hid_pen_data *data = container_of (sdev, struct hid_pen_data, pen_sdev);

	return sprintf(buf, "%x\n", data->fw_version);
}
static int wacom_pen_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct hid_pen_data *data;
	int ret;
        struct pen_hid_feature feature;

	data = kzalloc(sizeof(struct hid_pen_data), GFP_KERNEL);
	if (!data) {
	        pen_err("<asus-wy> cannot allocate hid pen data\n");
		return -ENOMEM;
	}

	hid_set_drvdata(hdev, data);

        //get feature report
        memset (&feature, 0, sizeof (struct pen_hid_feature));
	ret = hdev->hid_get_raw_report(hdev,
                                       WACOM_FEATURE_REPORT_ID,
                                       (__u8 *)&feature,
                                       sizeof (struct pen_hid_feature),
                                       (HID_FEATURE_REPORT+1)   //warning: HID_FEATURE_REPORT+1
                                       );

	if (ret != sizeof (struct pen_hid_feature)) {
	        pen_err("<asus-wy> get feature report error(%d)\n", ret);
		goto fail;
	}

	pen_dbg("<asus-wy> length = %x\n", feature.length);
	pen_dbg("<asus-wy> report_id = %x\n", feature.report_id);
	pen_dbg("<asus-wy> logical_x_max = %x\n", feature.logical_x_max);
	pen_dbg("<asus-wy> logical_y_max = %x\n", feature.logical_y_max);
	pen_dbg("<asus-wy> physical_x_max = %x\n", feature.physical_x_max);
	pen_dbg("<asus-wy> physical_y_max = %x\n", feature.physical_y_max);
	pen_dbg("<asus-wy> pressure_max = %x\n", feature.pressure_max);
	pen_dbg("<asus-wy> fw_version = %x\n", feature.fw_version);

        data->logical_x_max = feature.logical_x_max;
        data->logical_y_max = feature.logical_y_max;
        data->physical_x_max = feature.physical_x_max;
        data->physical_y_max = feature.physical_y_max;
        data->pressure_max = feature.pressure_max;
        data->fw_version = feature.fw_version;

	ret = hid_parse(hdev);      //ll_driver->parse(): get hid report
	if (ret) {
	        pen_err("<asus-wy> hid_parse error(%d)\n", ret);
                goto fail;
        }

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
	        pen_err("<asus-wy> hid_hw_start error(%d)\n", ret);
                goto fail;
        }

        //switch dev to show fw version
        data->pen_sdev.name = "digitizer";
        data->pen_sdev.print_name = asuspen_switch_name;
        if(switch_dev_register(&data->pen_sdev) < 0) {
	        pen_err("<asus-wy> switch_dev_register for digitizer failed\n");
        }

        return 0;

fail:
        pen_err("<asus-wy> wacom-pen probe failed\n");
	kfree(data);
	hid_set_drvdata(hdev, NULL);
	return ret;
}

static void wacom_pen_remove(struct hid_device *hdev)
{
	struct hid_pen_data *data = hid_get_drvdata(hdev);

	hid_hw_stop(hdev);
	kfree(data);
	hid_set_drvdata(hdev, NULL);
}

static int wacom_pen_input_mapping(struct hid_device *hdev, struct hid_input *hi, struct hid_field *field, struct hid_usage *usage, unsigned long **bit, int *max)
{
        if (field->application == HID_GD_MOUSE) {
	        pen_dbg("<asus-wy> don't map usage->hid = %x\n", usage->hid);
                return 1;
        }

        return 0;
}

static int wacom_pen_input_mapped(struct hid_device *hdev, struct hid_input *hi, struct hid_field *field, struct hid_usage *usage, unsigned long **bit, int *max)
{
        if (field->application == HID_GD_MOUSE) {
	        pen_dbg("<asus-wy> don't map usage->hid = %x\n", usage->hid);
                return -1;
        }
        return 0;
}

static const struct hid_device_id wacom_pen_devices[] = {
        { .driver_data = 0,
	  .bus = 0x0018,    //BUS_I2C
	  .vendor = 0x056A,
          .product = 0x0104,
	},

	{ }
};
MODULE_DEVICE_TABLE(hid, wacom_pen_devices);

static const struct hid_usage_id wacom_pen_usages[] = {
	{ HID_ANY_ID, HID_ANY_ID, HID_ANY_ID },
	{ HID_ANY_ID - 1, HID_ANY_ID - 1, HID_ANY_ID - 1}
};

static struct hid_driver wacom_pen_driver = {
	.name = "hid-wacom-pen",
	.id_table = wacom_pen_devices,
	.probe = wacom_pen_probe,
	.remove = wacom_pen_remove,
	.input_mapping = wacom_pen_input_mapping,
	.input_mapped = wacom_pen_input_mapped,
	.usage_table = wacom_pen_usages,
};

//HID I/O driver implementation
static int pen_write_read(struct i2c_client *client, u8 *write_buf, u16 write_len, u8 *read_buf, u16 read_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =2;

        memset(msg ,0, sizeof(struct i2c_msg)*2);

        num = 2;
        msg[0].addr = client->addr;
        msg[0].flags = !I2C_M_RD;
        msg[0].len = write_len;
        msg[0].buf = write_buf;
        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = read_len;
        msg[1].buf = read_buf;

        ret = i2c_transfer(client->adapter, msg, num);

        if(ret <0) {
	        pen_err("<asus-wy> i2c transfer error(%d)\n", ret);
                return -EIO;
        } else
                return 0;

}

static int pen_write(struct i2c_client *client, u8 *write_buf, u16 write_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =1;

        memset(msg, 0, sizeof(struct i2c_msg)*2);

        num = 1;
        msg[0].addr = client->addr;
        msg[0].flags = !I2C_M_RD;
        msg[0].len = write_len;
        msg[0].buf = write_buf;

        ret = i2c_transfer(client->adapter, msg, num);

        if(ret <0) {
                pen_err("<asus-wy> i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static int pen_read(struct i2c_client *client, u8 *read_buf, u16 read_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =2;

        memset(msg, 0, sizeof(struct i2c_msg)*2);

        num = 1;

        msg[0].addr = client->addr;
        msg[0].flags = I2C_M_RD;
        msg[0].len = read_len;
        msg[0].buf = read_buf;

        ret = i2c_transfer(client->adapter, msg, num);

        if(ret <0) {
                pen_err("<asus-wy> i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

//hid-over-i2c spec: class specific request (SET_POWER)
static void hid_set_power(struct hid_device *hid, int power)
{

	struct i2c_client *client = to_i2c_client(hid->dev.parent);
	struct i2c_pen_data *data = i2c_get_clientdata(client);
        struct i2c_hid_command cmd;

        pen_dbg("<asus-wy> hid set power %x\n", power);
        cmd.command_reg = data->hid_desc.command_reg;
        cmd.report_type_id = power;
        cmd.opcode = 0x08;

        pen_write(client, (u8 *)&cmd, sizeof(struct i2c_hid_command));

}

//hid-over-i2c spec: class specific request (RESET)
static int hid_reset(struct hid_device *hid)
{
	struct i2c_client *client = to_i2c_client(hid->dev.parent);
	struct i2c_pen_data *data = i2c_get_clientdata(client);
	struct i2c_hid_command cmd;
	int ret;

        pen_dbg("<asus-wy> hid reset\n");
        cmd.command_reg = data->hid_desc.command_reg;
        cmd.report_type_id = 0x00;
        cmd.opcode = 0x01;

	pen_write(client, (u8 *)&cmd, sizeof(struct i2c_hid_command));
	set_bit(I2C_HID_RESET_PENDING, &data->flags);

	ret = wait_event_timeout(data->wait, !test_bit(I2C_HID_RESET_PENDING, &data->flags), msecs_to_jiffies(5000));
	if(!ret) {
		pen_err("<asus-wy> hid reset error\n");
		return -ENODATA;
	}

        pen_dbg("<asus-wy> hid reset finished\n");

	return 0;
}

//hid-over-i2c spec: class specific request (GET_REPORT)
static int hid_get_report(struct hid_device *hid, u8 report_type, u8 report_id, u8** buf)
{
	struct i2c_client *client = to_i2c_client(hid->dev.parent);
	struct i2c_pen_data *data = i2c_get_clientdata(client);
	struct i2c_hid_command cmd;
	int ret;
        u16 length;

        pen_dbg("<asus-wy> hid get report: report type = %02x, report id = %02x\n", report_type, report_id);
        cmd.command_reg = data->hid_desc.command_reg;
        cmd.report_type_id |= (report_type << 4);
        cmd.report_type_id &= 0x30;
        cmd.report_type_id |= (report_id & 0x0f);
        cmd.report_type_id &= 0x3f;
        cmd.opcode = 0x02;    //GET_REPORT
        pen_dbg("<asus-wy> cmd.report_type_id = %02x, cmd.opcode = %02x\n", cmd.report_type_id, cmd.opcode);

	pen_write(client, (u8 *)&cmd, sizeof(struct i2c_hid_command));
        //get report from DATA register
	pen_write_read(client, (u8 *)&data->hid_desc.data_reg, 2, (u8 *)&length, 2);
        *buf = kzalloc (length, GFP_KERNEL);    //free by caller
	pen_write_read(client, (u8 *)&data->hid_desc.data_reg, 2, (u8 *)*buf, length);

	return (int)length;
}

static int pen_i2chid_parse(struct hid_device *hid)
{
	struct i2c_client *client = to_i2c_client(hid->dev.parent);
	struct i2c_pen_data *data = i2c_get_clientdata(client);
	char *rdesc;
	int ret;
        struct pen_hid_feature *fptr;
        u8 *buf = NULL;

	pen_dbg("<asus-wy> wacom-pen i2c hid parse\n");
        hid_set_power(hid, 0);

	ret = hid_reset(hid);

	if(ret)
                return ret;

	rdesc = kzalloc(data->hid_desc.report_desc_len, GFP_KERNEL);

	pen_write_read(client, (u8 *)&data->hid_desc.report_desc_reg, 2, (u8 *)rdesc,data->hid_desc.report_desc_len);	

	ret = hid_parse_report(hid, rdesc, data->hid_desc.report_desc_len);
	kfree(rdesc);
	if (ret) {
		pen_err("<asus-wy> parsing report descriptor failed\n");
		return ret;
	}

	return 0;
}

static int pen_i2chid_start(struct hid_device *hid)
{
	pen_dbg("<asus-wy> wacom-pen start\n");
	return 0;
}

static void pen_i2chid_stop(struct hid_device *hid)
{
	pen_dbg("<asus-wy> wacom-pen stop\n");
}

static int pen_i2chid_open(struct hid_device *hid)
{
	pen_dbg("<asus-wy> wacom-pen open\n");
	return 0;
}

void pen_i2chid_close(struct hid_device *hid)
{
	pen_dbg("<asus-wy> wacom-pen close\n");
}

static int pen_i2chid_power(struct hid_device *hid, int lvl)
{
	pen_dbg("<asus-wy> wacom-pen power lvl = %x\n",lvl);
	return 0;
}

static int i2c_hidinput_input_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	pen_dbg("<asus-wy> wacom-pen input event\n");
	return 0;
}

static int i2chid_get_raw_report(struct hid_device *hid,
		unsigned char report_number, __u8 *buf, size_t count,
		unsigned char report_type)
{
        u8 *tempbuf = NULL;
        int ret;

        ret = hid_get_report (hid, report_type, report_number, &tempbuf);
        if (ret > count) {
	        pen_err("<asus-wy> %s: error input buf length(%d < %d)\n", __func__, count, ret);
        }
        memcpy (buf, tempbuf, ret <= count ? ret : count);    //copy less one
        kfree (tempbuf);    //free by caller

        return (ret <= count ? ret : count);
}

static int i2chid_output_raw_report(struct hid_device *hid, __u8 *buf, size_t count,
		unsigned char report_type)
{
	pen_dbg("<asus-wy> i2chid_output_raw_report\n");
	return 0;
}


static struct hid_ll_driver pen_i2c_hid_driver = {
	.parse = pen_i2chid_parse,
	.start = pen_i2chid_start,
	.stop = pen_i2chid_stop,
	.open = pen_i2chid_open,
	.close = pen_i2chid_close,
	.power = pen_i2chid_power,
	.hidinput_input_event = i2c_hidinput_input_event,
};

static void pen_report_handle(struct i2c_pen_data *data)
{
        struct i2c_client *client = data->client;
	struct pen_hid_input *input_buffer = data->input_buffer;
	int ret;
        u16 length = 0xffff;

        //read from input_reg
        pen_write_read(client, (u8 *)&data->hid_desc.input_reg, 2, (u8 *)&length, 2);
        //reset is finished
	if(!length) {
		if (test_and_clear_bit(I2C_HID_RESET_PENDING, &data->flags))
			wake_up(&data->wait);
		return;
	}

        //get the whole input report
        pen_write_read(client, (u8 *)&data->hid_desc.input_reg, 2, (u8 *)input_buffer, length);
        pen_dbg("<asus-wy> input report: len = %04x, id = %02x, data = %02x %02x %02x %02x %02x %02x %02x\n", input_buffer->input_len, input_buffer->report_id, input_buffer->tip_switch, input_buffer->x_low, input_buffer->x_high, input_buffer->y_low, input_buffer->y_high, input_buffer->p_low, input_buffer->p_high);

	if(input_buffer->report_id != WACOM_INPUT_REPORT_ID) {
                pen_dbg("<asus-wy> wrong report id(%d)\n", input_buffer->report_id);
		return;
	}

	ret = hid_input_report(data->hid, HID_INPUT_REPORT, &input_buffer->report_id, data->hid_desc.input_len-2, 1);
        pen_dbg("<asus-wy> hid_input_report ret= %d\n",ret);

}

static irqreturn_t pen_thread_handler(int id, void *dev)
{
	struct i2c_pen_data *data = (struct i2c_pen_data *)dev;

	pen_dbg("<asus-wy> pen_thread_handler\n");

	pen_report_handle(data);

	return IRQ_HANDLED;

}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void pen_early_suspend(struct early_suspend *es)
{
	struct i2c_pen_data *data;
	data = container_of(es, struct i2c_pen_data, early_suspend);

	disable_irq(data->irq);
	hid_set_power(data->hid, 1);
}

static void pen_late_resume(struct early_suspend *es)
{
	struct i2c_pen_data *data;
	data = container_of(es, struct i2c_pen_data, early_suspend);

	hid_set_power(data->hid, 0);
	enable_irq(data->irq);
}
#endif

static int __devinit pen_probe(struct i2c_client *client,
                               const struct i2c_device_id *id)
{
	struct i2c_pen_data *data;
	int ret = 0;
	struct hid_device *hid;

	data = kzalloc(sizeof(struct i2c_pen_data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	pen_dbg("<asus-wy> pen_probe\n");

	init_waitqueue_head(&data->wait);
	data->client = client;
	data->gpio_num = 98;
	i2c_set_clientdata(client, data);
	data->irq = gpio_to_irq(data->gpio_num);

        memset (&data->hid_desc, 0, sizeof (struct i2c_hid_desc));
	data->hid_desc_addr = 0x01;

        //get hid descriptor
	ret = pen_write_read(client, (u8 *)&data->hid_desc_addr, 2, (u8 *)&data->hid_desc, 2);
        if (ret < 0) {
                pen_err("<asus-wy> get HID descriptor failed\n");
                return ret;
        }
	ret = pen_write_read(client, (u8 *)&data->hid_desc_addr, 2, (u8 *)&data->hid_desc, data->hid_desc.hid_desc_len);
        if (ret < 0) {
                pen_err("<asus-wy> get HID descriptor failed\n");
                return ret;
        }

	pen_dbg("<asus-wy> HIDDescLen = %x\n", data->hid_desc.hid_desc_len);
	pen_dbg("<asus-wy> bcdVersion = %x\n", data->hid_desc.bcd_version);
	pen_dbg("<asus-wy> ReportDescLen = %x\n", data->hid_desc.report_desc_len);
	pen_dbg("<asus-wy> ReportDescReg = %x\n", data->hid_desc.report_desc_reg);
	pen_dbg("<asus-wy> InputReg = %x\n", data->hid_desc.input_reg);
	pen_dbg("<asus-wy> InputLen = %x\n", data->hid_desc.input_len);
	pen_dbg("<asus-wy> OutputReg = %x\n", data->hid_desc.output_reg);
	pen_dbg("<asus-wy> OutputLen = %x\n", data->hid_desc.output_len);
	pen_dbg("<asus-wy> CommandReg = %x\n", data->hid_desc.command_reg);
	pen_dbg("<asus-wy> DataReg = %x\n", data->hid_desc.data_reg);
        pen_dbg("<asus-wy> VendorID = %x\n", data->hid_desc.vendor_id);
        pen_dbg("<asus-wy> ProductID = %x\n", data->hid_desc.product_id);
        pen_dbg("<asus-wy> VersionID = %x\n", data->hid_desc.version_id);

        data->input_buffer = kzalloc(data->hid_desc.input_len, GFP_KERNEL);
        /* register interrupt */
        ret = request_threaded_irq(data->irq,
                                   NULL,
                                   pen_thread_handler,
                                   IRQF_ONESHOT,
                                   "wacom-pen",
                                   data
                                   );
        if (ret) {
                pen_err("<asus-wy> cannot get IRQ:%d\n", data->irq);
                data->irq = -1;
		goto err_free_data;
        } else {
                pen_dbg("<asus-wy> IRQ No:%d\n", data->irq);
        }

	irq_set_irq_type(data->irq, IRQ_TYPE_LEVEL_LOW);

        //setup hid device
	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		ret =  PTR_ERR(hid);
		goto err_free_data;
	}
	data->hid = hid;

	hid->ll_driver = &pen_i2c_hid_driver;
	hid->hid_get_raw_report = i2chid_get_raw_report;
	hid->hid_output_raw_report = i2chid_output_raw_report;

	hid->dev.parent = &client->dev;
	hid->bus = BUS_I2C;
	hid->vendor = le16_to_cpu(data->hid_desc.vendor_id);
	hid->product = le16_to_cpu(data->hid_desc.product_id);
	hid->version = le16_to_cpu(data->hid_desc.version_id);
	strlcpy(hid->name, client->name, sizeof(hid->name));
        pen_dbg("<asus-wy> vendor = %08x, product = %08x, name = %s\n", hid->vendor, hid->product, hid->name);

	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			pen_err("<asus-wy> can't add hid device: %d\n", ret);
		goto err_free_hid;
	}

	pen_dbg("<asus-wy> add HID %s\n", hid->name);

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = pen_early_suspend;
	data->early_suspend.resume = pen_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
	return 0;

err_free_hid:
	hid_destroy_device(data->hid);
err_free_data:
	kfree(data);

	return ret;
}

static int __devexit pen_remove(struct i2c_client *client)
{

	pen_dbg("<asus-wy> pen_remove\n");

        return 0;
}

static const struct i2c_device_id pen_id[] = {
	{ "wacom_pen", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pen_id);

static struct acpi_device_id pen_acpi_match[] = {
        { "WACM0001", 0 },
        { },
};
MODULE_DEVICE_TABLE(acpi, pen_acpi_match);


static struct i2c_driver pen_driver = {
        .driver = {
                .name   = "wacom_pen",
                .owner  = THIS_MODULE,
#if 0
		.pm	= &mxt_pm,
#endif
                .acpi_match_table = ACPI_PTR(pen_acpi_match),
        },
        .probe          = pen_probe,
        .remove         = __devexit_p(pen_remove),
        .id_table       = pen_id,
};

static int __init pen_init(void)
{
        int ret = 0;

	pen_dbg("<asus-wy> add wacom-pen driver\n");
        gpio_set_value(101, 1);   //RESET pin: set to high(SOC pulls to low, low active)
        msleep (200);   //200ms to be operable

	ret = hid_register_driver(&wacom_pen_driver);
	if (ret) {
	        pen_dbg("<asus-wy> hid_register_driver failed(%d)\n", ret);
                return ret;
        }

        ret = i2c_add_driver(&pen_driver);
	if (ret) {
	        pen_dbg("<asus-wy> i2c_add_driver failed(%d)\n", ret);
                return ret;
        }

        return 0;
}

static void __exit pen_exit(void)
{
        i2c_del_driver(&pen_driver);
}

late_initcall(pen_init);
module_exit(pen_exit);


MODULE_AUTHOR("Wanya Chen");
MODULE_DESCRIPTION("Wacom Digitizer HID over I2C driver");
MODULE_LICENSE("GPL");


