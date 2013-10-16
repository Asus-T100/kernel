
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/input/mt.h>
#include "usbhid/usbhid.h"

MODULE_LICENSE("GPL");

//elan click-pad, vendor defined
#define HID_ELAN_APPLICATION_USAGE    0xff0000c5
#define ELAN_REPORT_ID                0x5d

//elan
#define ETP_MAX_FINGERS		5
#define ETP_FINGER_DATA_LEN	5
#define ETP_DEF_MAX_X		2690
#define ETP_DEF_MAX_Y		1628
#define ETP_DEF_TRACENUM_X	21
#define ETP_DEF_TRACENUM_Y	12
#define ETP_DEF_RES_X		3
#define	ETP_DEF_RES_Y		6

struct elan_pad_data {
	struct input_dev  *input;
	unsigned int      max_x;
	unsigned int      max_y;
	unsigned int      width_x;
	unsigned int      width_y;
};

static int elan_pad_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
	struct elan_pad_data *priv_data = hid_get_drvdata(hdev);
	struct input_dev *input = report->field[0]->hidinput->input;
	u8 *finger_data = &data[2];
	bool finger_on;
	int pos_x, pos_y;
	int area_x, area_y, pressure;
	int i;

	switch (data[0]) {                                                                                //byte1: report id
	        case ELAN_REPORT_ID:
                        break;
	        default:
                        dev_err(&hdev->dev, "unknown report id (%02x)\n", data[0]);
                        return 0;
	}

	for (i = 0 ; i < ETP_MAX_FINGERS ; i++) {
		finger_on = (data[1] >> (3 + i)) & 0x01;                                                  //byte2: f5|f4|f3|f2|f1|mb|rb|lb

		if (finger_on) {
			pos_x = ((finger_data[0] & 0xf0) << 4) | finger_data[1];                          //byte3: Xhigh|Yhigh
			pos_y = priv_data->max_y - (((finger_data[0] & 0x0f) << 8) | finger_data[2]);     //byte4: Xlow, byte5: Ylow

			area_x = (finger_data[3] & 0x0f) * priv_data->width_x;                            //byte6: MKy|MKx
//			area_y = (finger_data[3] >> 4) * priv_data->width_y;                              //JH Wang, area_y seems incorrect
			area_y = area_x;
			pressure = finger_data[4];                                                        //byte7: H (pressure)

			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, true);

			input_report_abs(input, ABS_MT_TRACKING_ID, i);                                   //driver tracking
			input_report_abs(input, ABS_MT_POSITION_X, pos_x);
			input_report_abs(input, ABS_MT_POSITION_Y, pos_y);
			input_report_abs(input, ABS_MT_PRESSURE, pressure);
			/* use x-axis value as TOOL_WIDTH */
			input_report_abs(input, ABS_TOOL_WIDTH, finger_data[3] & 0x0f);
//			input_report_abs(input, ABS_MT_TOUCH_MAJOR, max(area_x, area_y));
//			input_report_abs(input, ABS_MT_TOUCH_MINOR, min(area_x, area_y));
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, area_x);
			input_report_abs(input, ABS_MT_TOUCH_MINOR, area_y);
			finger_data += ETP_FINGER_DATA_LEN;
		} else {
			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
		}
	}

	input_mt_report_pointer_emulation(input, true);
	input_report_key(input, BTN_LEFT, ((data[1] & 0x01) == 1));
	input_sync(input);

        return 1;
}

/*
 * (value from firmware) * 10 + 790 = dpi
 * we also have to convert dpi to dots/mm (*10/254 to avoid floating point)
 */
static unsigned int elan_i2c_convert_res(unsigned int val)
{
	return (val * 10 + 790) * 10 / 254;
}

static int elan_pad_input_mapping(struct hid_device *hdev, struct hid_input *hi, struct hid_field *field, struct hid_usage *usage, unsigned long **bit, int *max)
{
	struct elan_pad_data *priv_data = hid_get_drvdata(hdev);
	struct input_dev *input = hi->input;
	unsigned int x_res, y_res;

	if (field->application != HID_ELAN_APPLICATION_USAGE)
                return 0;

        //set bit, etc
	__set_bit(INPUT_PROP_POINTER, input->propbit);
	__set_bit(INPUT_PROP_BUTTONPAD, input->propbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);

	__set_bit(BTN_LEFT, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_TOOL_FINGER, input->keybit);
	__set_bit(BTN_TOOL_DOUBLETAP, input->keybit);
	__set_bit(BTN_TOOL_TRIPLETAP, input->keybit);
	__set_bit(BTN_TOOL_QUADTAP, input->keybit);
	__set_bit(BTN_TOOL_QUINTTAP, input->keybit);

	__set_bit(ABS_MT_TRACKING_ID, input->absbit);   //driver tracking
	__set_bit(ABS_MT_TOUCH_MAJOR, input->absbit);
	__set_bit(ABS_MT_TOUCH_MINOR, input->absbit);
	__set_bit(ABS_MT_POSITION_X, input->absbit);
	__set_bit(ABS_MT_POSITION_Y, input->absbit);

        priv_data->max_x = ETP_DEF_MAX_X;
        priv_data->max_y = ETP_DEF_MAX_Y;
        priv_data->width_x = priv_data->max_x / (ETP_DEF_TRACENUM_X - 1);
        priv_data->width_y = priv_data->max_y / (ETP_DEF_TRACENUM_Y - 1);
        x_res = elan_i2c_convert_res(ETP_DEF_RES_X);
        y_res = elan_i2c_convert_res(ETP_DEF_RES_Y);

	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, ETP_MAX_FINGERS, 0, 0);
	input_set_abs_params(input, ABS_X, 0, priv_data->max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, priv_data->max_y, 0, 0);
	input_abs_set_res(input, ABS_X, x_res);
	input_abs_set_res(input, ABS_Y, y_res);
	input_set_abs_params(input, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	input_mt_init_slots(input, ETP_MAX_FINGERS);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, priv_data->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, priv_data->max_y, 0, 0);
	input_abs_set_res(input, ABS_MT_POSITION_X, x_res);
	input_abs_set_res(input, ABS_MT_POSITION_Y, y_res);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 15 * max(priv_data->width_x, priv_data->width_y), 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 15 * min(priv_data->width_x, priv_data->width_y), 0, 0);

        return 1;
}

static int elan_pad_input_mapped(struct hid_device *hdev, struct hid_input *hi, struct hid_field *field, struct hid_usage *usage, unsigned long **bit, int *max)
{
	if (usage->type == EV_KEY || usage->type == EV_ABS)
		set_bit(usage->type, hi->input->evbit);

	return -1;
}

static int elan_pad_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	__u8 feature[] = { 0x0d, 0x00, 0x03, 0x01, 0x00 };
	struct elan_pad_data *priv_data;
	struct hid_report_enum *report_enum = hdev->report_enum + HID_INPUT_REPORT;  //match mouse interface only
	int ret;

	priv_data = kzalloc(sizeof(struct elan_pad_data), GFP_KERNEL);
	if (!priv_data) {
		dev_err(&hdev->dev, "cannot allocate multitouch data\n");
		return -ENOMEM;
	}

	hid_set_drvdata(hdev, priv_data);

	ret = hid_parse(hdev);
	if (ret != 0)
		goto fail;

        //match mouse interface only
        if (!report_enum->report_id_hash[ELAN_REPORT_ID]) {
                ret = -1;                                       //don't match hid-elan-pad driver
                goto fail;
        }

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret)
		goto fail;

        //set to multi-touch mode
	ret = hdev->hid_output_raw_report(hdev, feature, sizeof(feature), HID_FEATURE_REPORT);
	if (ret != sizeof(feature)) {
		hid_err(hdev, "unable to request touch data (%d)\n", ret);
		goto fail;
	}

        return 0;
fail:
        dev_err(&hdev->dev, "probe failed\n");
	kfree(priv_data);
	hid_set_drvdata(hdev, NULL);
	return ret;
}

static void elan_pad_remove(struct hid_device *hdev)
{
	struct elan_pad_data *priv_data = hid_get_drvdata(hdev);

	hid_hw_stop(hdev);
	kfree(priv_data);
	hid_set_drvdata(hdev, NULL);
}

static const struct hid_device_id elan_pad_devices[] = {
        { .driver_data = 0,
	  .bus = 0x0003,
	  .vendor = 0x0b05,
          .product = 0x17e0,
	},

	{ }
};
MODULE_DEVICE_TABLE(hid, elan_pad_devices);

static const struct hid_usage_id elan_pad_usages[] = {
	{ HID_ANY_ID, HID_ANY_ID, HID_ANY_ID },
	{ HID_ANY_ID - 1, HID_ANY_ID - 1, HID_ANY_ID - 1}
};

static struct hid_driver elan_pad_driver = {
	.name = "hid-elan-pad",
	.id_table = elan_pad_devices,
	.probe = elan_pad_probe,
	.remove = elan_pad_remove,
	.input_mapping = elan_pad_input_mapping,
	.input_mapped = elan_pad_input_mapped,
	.usage_table = elan_pad_usages,
	.raw_event = elan_pad_raw_event,
};

static int __init elan_pad_init(void)
{
	return hid_register_driver(&elan_pad_driver);
}

static void __exit elan_pad_exit(void)
{
	hid_unregister_driver(&elan_pad_driver);
}

module_init(elan_pad_init);
module_exit(elan_pad_exit);

