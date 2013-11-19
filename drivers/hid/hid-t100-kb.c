//<asus-wanya20131018>

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/kernel.h>
#include <linux/hid-debug.h>
#include <linux/switch.h>
#include "usbhid/usbhid.h"

MODULE_LICENSE("GPL");

//elan click-pad, vendor defined
#define T100_REPORT_ID                0x5d

#define t100_map_key(c)  hid_map_usage(hi, usage, bit, max, EV_KEY, (c))
#define t100_map_key_clear(c)	hid_map_usage_clear(hi, usage, bit, max, \
					EV_KEY, (c))

struct t100_kb_data {
	int               winkeyoff;
	int               waitrel;
        //for fw version showing
        u16               bcdDevice;
	struct switch_dev kb_sdev;
};

static int t100_kb_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
	unsigned int i;
        int meta = 0;
	struct input_dev *input = report->field[0]->hidinput->input;
	struct t100_kb_data *priv_data = hid_get_drvdata(hdev);

        //is the winkey + any key ?
        if (priv_data->winkeyoff) {
            //ignore every report behind, until release report
            //if (data[0] != 0x08) {
                for (i = 0; i < 8; i++) {
                    //may be the code of any key
                    if (data[i] != 0)
                        break;
                }
                if (i >= 8) {
                    //restore the state until the release key
                    priv_data->winkeyoff = 0;
                }
            //}
            return 1;
        }

        //is the next report release ?
        if (priv_data->waitrel) {
            meta = 0;
            for (i = 0; i < size; i++) {
                if (data[i] != 0) {
                    meta = 1;
                    break;
                }
            }
            if (!meta) {
                //winkey release !
                priv_data->waitrel = 0;
                input_report_key (input, KEY_HOMEPAGE, 1);
                input_sync(input);
                input_report_key (input, KEY_HOMEPAGE, 0);
                input_sync(input);
                return 1;
            }
            priv_data->waitrel = 0;
        }

        //handle the case winkey only
        if (data[0] == 0x08) {
            meta = 0;
            for (i = 1; i < size; i++) {    //8 bytes
                if (data[i] != 0) {
                    meta = 1;
                    break;
                }
            }

            if (meta) {
                //winkey + any key
                priv_data->winkeyoff = 1;
                if (priv_data->waitrel)
                        priv_data->waitrel = 0;    //next report is not release report
            } else {
                //winkey only
                priv_data->waitrel = 1;      //wait for release report
            }
            //don't send the code
            return 1;
        }

        return 0;
}

static int t100_kb_input_mapping(struct hid_device *hdev, struct hid_input *hi, struct hid_field *field, struct hid_usage *usage, unsigned long **bit, int *max)
{
	struct input_dev *input = hi->input;

	switch (usage->hid & HID_USAGE_PAGE) {

	case HID_UP_KEYBOARD:
		set_bit(EV_REP, input->evbit);

		if ((usage->hid & HID_USAGE) < 256) {
                        if ((usage->hid & HID_USAGE) == 0x65) {
                            //menu key
                            t100_map_key_clear(KEY_MENU);
                            return 1;
                        } else if ((usage->hid & HID_USAGE) == 0xe3) {
                            //home key
                            t100_map_key_clear(KEY_HOMEPAGE);
                            return 1;
                        }
		}
		break;

	case HID_UP_ASUSVENDOR:
		switch (usage->hid & HID_USAGE) {
		case 0x06C:
                    //Fn+F1 (sleep)
                    t100_map_key_clear(KEY_SLEEP);
                    return 1;
		case 0x088:
                    //Fn+F2 (wl and bt)
                    t100_map_key_clear(KEY_WLAN);
                    return 1;
		case 0x010:
                    //Fn+F5 (brightness down)
                    t100_map_key_clear(KEY_BRIGHTNESSDOWN);
                    return 1;
		case 0x020:
                    //Fn+F6 (brightness up)
                    t100_map_key_clear(KEY_BRIGHTNESSUP);
                    return 1;
		case 0x06B:
                    //Fn+F9 (touchpad)
                    t100_map_key_clear(KEY_F24);
                    return 1;
		}
		break;
        }

        return 0;
}

static ssize_t asuskb_switch_name(struct switch_dev *sdev, char *buf)
{
	struct t100_kb_data *priv_data = container_of (sdev, struct t100_kb_data, kb_sdev);

	return sprintf(buf, "%2x%02x\n", (priv_data->bcdDevice >> 8), (priv_data->bcdDevice & 0xff));
}

static int t100_kb_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct t100_kb_data *priv_data;
	struct hid_report_enum *report_enum = hdev->report_enum + HID_INPUT_REPORT;  //match mouse interface only
	int ret;
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	struct usb_device *usbdev = interface_to_usbdev(intf);
        int registered;

	priv_data = kzalloc(sizeof(struct t100_kb_data), GFP_KERNEL);
	if (!priv_data) {
		dev_err(&hdev->dev, "cannot allocate private data\n");
		return -ENOMEM;
	}

	hid_set_drvdata(hdev, priv_data);

	ret = hid_parse(hdev);
	if (ret != 0)
		goto fail;

        //don't match mouse interface
        if (report_enum->report_id_hash[T100_REPORT_ID]) {
                ret = -1;                                       //match hid-elan-pad driver
                goto fail;
        }

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret)
		goto fail;

        priv_data->bcdDevice =  le16_to_cpu(usbdev->descriptor.bcdDevice);
        //switch dev to show fw version
        registered = *((int *)id->driver_data);
        if (!registered) {
                dev_err(&hdev->dev, "<asus-wy> add switch dev\n");
                priv_data->kb_sdev.name = "keyboard";
                priv_data->kb_sdev.print_name = asuskb_switch_name;
                if(switch_dev_register(&priv_data->kb_sdev) < 0) {
                        dev_err(&hdev->dev, "<asus-wy> switch_dev_register for keyboard failed!\n");
                }
                *((int *)id->driver_data) = 1;
        }

        return 0;
fail:
        dev_err(&hdev->dev, "probe failed\n");
	kfree(priv_data);
	hid_set_drvdata(hdev, NULL);
	return ret;
}

static void t100_kb_remove(struct hid_device *hdev)
{
	struct t100_kb_data *priv_data = hid_get_drvdata(hdev);

        if (priv_data->kb_sdev.dev) {
                switch_dev_unregister(&priv_data->kb_sdev);
                *(int *)(hdev->driver->id_table[0].driver_data) = 0;    //reset
        }
	hid_hw_stop(hdev);
	kfree(priv_data);
	hid_set_drvdata(hdev, NULL);
}

static int kb_sdev_registered = 0;

static const struct hid_device_id t100_kb_devices[] = {
        { .driver_data = (unsigned long)&kb_sdev_registered,
	  .bus = 0x0003,
	  .vendor = 0x0b05,
          .product = 0x17e0,
	},

	{ }
};
MODULE_DEVICE_TABLE(hid, t100_kb_devices);

static const struct hid_usage_id t100_kb_usages[] = {
	{ HID_ANY_ID, HID_ANY_ID, HID_ANY_ID },
	{ HID_ANY_ID - 1, HID_ANY_ID - 1, HID_ANY_ID - 1}
};

static struct hid_driver t100_kb_driver = {
	.name = "hid-t100-kb",
	.id_table = t100_kb_devices,
	.probe = t100_kb_probe,
	.remove = t100_kb_remove,
	.input_mapping = t100_kb_input_mapping,
	.usage_table = t100_kb_usages,
	.raw_event = t100_kb_raw_event,
};

static int __init t100_kb_init(void)
{
	return hid_register_driver(&t100_kb_driver);
}

static void __exit t100_kb_exit(void)
{
	hid_unregister_driver(&t100_kb_driver);
}

module_init(t100_kb_init);
module_exit(t100_kb_exit);

