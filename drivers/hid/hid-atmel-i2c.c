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

#include <linux/hid.h>
#include <linux/hiddev.h>
#include <linux/hid-debug.h>
#include <linux/hidraw.h>

static int debug = 0;

#define mxt_dbg(fmt, arg...)					  \
do {									  \
	if (debug)							  \
		pr_err(fmt, ##arg); \
} while (0)

#define I2C_HID_RESET_PENDING	(1 << 1)

#pragma pack(1)
typedef struct _HIDDesc{
        u16  HIDDescLen;
        u16  bcdVersion;
        u16  ReportDescLen;
        u16  ReportDescReg;
        u16  InputReg;
        u16  InputLen;
        u16  OutputReg;
        u16  OutputLen;
        u16  CommandReg;
        u16  DataReg;
        u16  VendorID;
        u16  ProductID;
        u16  VersionID;
        u32  Reserved;
}HIDDesc;


typedef struct HIDCommand
{
	u16	CommandReg;
	u8	ReportTypeID;
	u8	OpCode;
};

typedef struct _MxtHIDOutput {
	//HID over I2C spec.
	u16	OutputReg;
	u16	OutputLen;
	u8	ReportID;
	//Mxt format
	u8	CommandID;
	u8	NumWx;
	u8	NumRx;
	u16	Addr;
	u8	Data;	
}MxtHIDOutput; 

typedef struct _MxtHIDInput {
	//HID over I2C spec.
	u16  	InputLen;
	u8	ReportID;
	//Mxt format
	u8   	Status;
	u8  	NumRx;
}MxtHIDInput;

typedef struct _IDInfo {
        u8   FamilyID;
        u8   VariantID;
        u8   Version;
        u8   Build;
        u8   MatrixXSize;
        u8   MatriXYSize;
        u8   NumOfObject;
}IDInfo;

typedef struct _ObjectType {
        UINT8   Type;
        UINT8   LSB;
        UINT8   MSB;
        UINT8   Size_dec1;
        UINT8   Inst_dec1;
        UINT8   NumOfReportID;
}ObjectType;


typedef struct _T6_Message
{
	UINT8	Status;
	UINT8	ConfigCheckSum[3];
}T6_Message;

#pragma pack()

struct mxt_data {
	struct i2c_client *client;
	int gpio_num;
	int irq;
	struct hid_device *hid;
	unsigned long flags;
	wait_queue_head_t	wait;/* For waiting the interrupt */
};

static int mxt_write_read(struct i2c_client *client,u8 *write_buf, u16 write_len,u8 *read_buf, u16 read_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =2;

        memset(&msg,0,sizeof(struct i2c_msg)*2);

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
                pr_err("i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static int mxt_write(struct i2c_client *client,u8 *write_buf, u16 write_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =1;

        memset(&msg,0,sizeof(struct i2c_msg)*2);

        num = 1;
        msg[0].addr = client->addr;
        msg[0].flags = !I2C_M_RD;
        msg[0].len = write_len;
        msg[0].buf = write_buf;

        ret = i2c_transfer(client->adapter, msg, num);

        if(ret <0) {
                pr_err("i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static int mxt_read(struct i2c_client *client, u8 *read_buf, u16 read_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =2;

        memset(&msg,0,sizeof(struct i2c_msg)*2);

        num = 1;

        msg[0].addr = client->addr;
        msg[0].flags = I2C_M_RD;
        msg[0].len = read_len;
        msg[0].buf = read_buf;

        ret = i2c_transfer(client->adapter, msg, num);

        if(ret <0) {
                pr_err("i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static HIDDesc HIDDS;
static MxtHIDOutput *MxtHIDOut;
static MxtHIDInput  *MxtHIDIn;

static int mxt_hid_read_config(struct i2c_client *client,u16 addr,u8 *read_buf,u8 read_len)
{
	int TimeOut = 1000;

	memset(MxtHIDOut,0,HIDDS.OutputLen+2);
	
        MxtHIDOut->OutputReg = HIDDS.OutputReg;
        MxtHIDOut->OutputLen = HIDDS.OutputLen;
        MxtHIDOut->ReportID    = 0x06;
        MxtHIDOut->CommandID = 0x51;
        MxtHIDOut->NumWx     = 0x02;
        MxtHIDOut->NumRx    = read_len;
	MxtHIDOut->Addr	  = addr;

	mxt_write(client,MxtHIDOut,HIDDS.OutputLen+2);

	do
	{
		mxt_read(client,MxtHIDIn,HIDDS.InputLen);
		if((MxtHIDIn->ReportID == MxtHIDOut->ReportID) && (MxtHIDIn->Status ==0x00))
			break;
		msleep_interruptible(1);
	}while(TimeOut--);
	
	if(!TimeOut) return -1;

	memcpy(read_buf,MxtHIDIn+1,read_len);

	return 0;
}

static int mxt_hid_write_config(struct i2c_client *client,u16 addr,u8 *write_buf,u8 write_len)
{
        int TimeOut = 1000;

        memset(MxtHIDOut,0,HIDDS.OutputLen+2);

        MxtHIDOut->OutputReg = HIDDS.OutputReg;
        MxtHIDOut->OutputLen = HIDDS.OutputLen;
        MxtHIDOut->ReportID    = 0x06;
        MxtHIDOut->CommandID = 0x51;
        MxtHIDOut->NumWx     = write_len+0x02;
        MxtHIDOut->NumRx    = 0x0;
        MxtHIDOut->Addr   = addr;

        mxt_write(client,MxtHIDOut,HIDDS.OutputLen+2);

        do
        {
                mxt_read(client,MxtHIDIn,HIDDS.InputLen);
                if((MxtHIDIn->ReportID == MxtHIDOut->ReportID) && (MxtHIDIn->Status ==0x04))
                        break;
                msleep_interruptible(1);
        }while(TimeOut--);

        if(!TimeOut) return -1;

        return 0;
}

static void mxt_report_handle(struct mxt_data *data)
{
        struct i2c_client *client = data->client;
        u8  Buf[32];
        int i;
	int length;
	int ret;

        mxt_read(client,Buf,HIDDS.InputLen);

	length = Buf[0] | Buf[1] << 8;
        mxt_dbg("%x %x %x %x %x \n",Buf[0],Buf[1],Buf[2],Buf[3],Buf[4]);

	if(!length) {
		if (test_and_clear_bit(I2C_HID_RESET_PENDING, &data->flags))
			wake_up(&data->wait);
		return;
	}
	ret = hid_input_report(data->hid, HID_INPUT_REPORT,&Buf[2],HIDDS.InputLen-2,1);
        mxt_dbg("input report = %x  %x ret = %x\n",Buf[3],Buf[4],ret);

	
}

static irqreturn_t mxt_thread_handler(int id, void *dev)
{
	struct mxt_data *data = (struct mxt_data *)dev;
	struct i2c_client *client = data->client;
	u8  Buf[32];
	int i;

	mxt_dbg("mxt_thread_handler\n");

	do
	{
		mxt_report_handle(data);
	}while(gpio_get_value(data->gpio_num) == 0x00);

	return IRQ_HANDLED;

}

static void hid_set_power(struct hid_device *hid, int power)
{
	struct i2c_client *client = to_i2c_client(hid->dev.parent);
        struct HIDCommand Cmd;

        mxt_dbg("Set Power %x\n",power);
        Cmd.CommandReg = HIDDS.CommandReg;
        Cmd.ReportTypeID = power;
        Cmd.OpCode = 0x08;

        mxt_write(client,&Cmd,sizeof(struct HIDCommand));

}

static int hid_reset(struct hid_device *hid)
{
	struct i2c_client *client = to_i2c_client(hid->dev.parent);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct HIDCommand Cmd;
	int ret;

        mxt_dbg("Reset\n");
        Cmd.CommandReg = HIDDS.CommandReg;
        Cmd.ReportTypeID = 0x00;
        Cmd.OpCode = 0x01;

	mxt_write(client,&Cmd,sizeof(struct HIDCommand));
	set_bit(I2C_HID_RESET_PENDING, &data->flags);

	mxt_dbg("wait\n");
	ret = wait_event_timeout(data->wait,!test_bit(I2C_HID_RESET_PENDING, &data->flags),msecs_to_jiffies(5000));
	mxt_dbg("end %x\n",ret);
	if(!ret) return -ENODATA;

	return 0;
}


static int i2chid_parse(struct hid_device *hid)
{
	struct i2c_client *client = to_i2c_client(hid->dev.parent);
	char *rdesc;	
	unsigned int rsize = 0;
	int ret;

	mxt_dbg("parse\n");

	hid_set_power(hid,0);

	ret = hid_reset(hid);

	if(ret) return ret;

	rdesc = kmalloc(HIDDS.ReportDescLen, GFP_KERNEL);

	mxt_write_read(client,&HIDDS.ReportDescReg,2,rdesc,HIDDS.ReportDescLen);	

	ret = hid_parse_report(hid, rdesc, HIDDS.ReportDescLen);
	kfree(rdesc);
	if (ret) {
		mxt_dbg("parsing report descriptor failed\n");
	}
	
	return 0;
}

static int i2chid_start(struct hid_device *hid)
{
	mxt_dbg("start\n");
	return 0;
}


static void i2chid_stop(struct hid_device *hid)
{
	mxt_dbg("stop\n");
}
static int i2chid_open(struct hid_device *hid)
{
	mxt_dbg("open\n");
	return 0;
}
void i2chid_close(struct hid_device *hid)
{
	mxt_dbg("close\n");
}

static int i2chid_power(struct hid_device *hid, int lvl)
{
	mxt_dbg("power\n");
	return 0;
}
static int i2c_hidinput_input_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	mxt_dbg("input event\n");
	return 0;
}


static int i2chid_get_raw_report(struct hid_device *hid,
		unsigned char report_number, __u8 *buf, size_t count,
		unsigned char report_type)
{
	mxt_dbg("i2chid_get_raw_report\n");	
	return 0;
}

static int i2chid_output_raw_report(struct hid_device *hid, __u8 *buf, size_t count,
		unsigned char report_type)
{
	mxt_dbg("i2chid_output_raw_report\n");
	return 0;
}

static struct hid_ll_driver i2c_hid_driver = {
	.parse = i2chid_parse,
	.start = i2chid_start,
	.stop = i2chid_stop,
	.open = i2chid_open,
	.close = i2chid_close,
	.power = i2chid_power,
	.hidinput_input_event = i2c_hidinput_input_event,
};

#define USB_VENDOR_ID_ATMEL		0x03eb
#define USB_DEVICE_ID_ATMEL_MULTITOUCH	0x211c

static int __devinit mxt_probe(struct i2c_client *client,
                               const struct i2c_device_id *id)
{
	int ret;
	u16 HIDStartAddr = 0x00;
	u8  Buf[32];	
	struct mxt_data *data;
	struct hid_device *hid;


	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	mxt_dbg("mxt probe\n");

	init_waitqueue_head(&data->wait);
	data->client = client;
	data->gpio_num = 142;//acpi_get_gpio("\\_SB.GPO2", 12);
	i2c_set_clientdata(client, data);

	data->irq = gpio_to_irq(data->gpio_num);

        /* register interrupt */
        ret = request_threaded_irq(data->irq, NULL,
                                        mxt_thread_handler,
                                        IRQF_TRIGGER_LOW|IRQF_TRIGGER_FALLING,
                                        "mxt-touch", data);
        if (ret) {
                pr_err("cannot get IRQ:%d\n", data->irq);
                data->irq = -1;
        } else {
                pr_err("IRQ No:%d\n", data->irq);
        }

	mxt_write_read(client,&HIDStartAddr,2,&HIDDS,2);

	mxt_write_read(client,&HIDStartAddr,2,&HIDDS,HIDDS.HIDDescLen);

	mxt_dbg("HIDDescLen = %x\n",HIDDS.HIDDescLen);
	mxt_dbg("bcdVersion = %x\n",HIDDS.bcdVersion);
	mxt_dbg("ReportDescLen = %x\n",HIDDS.ReportDescLen);
	mxt_dbg("ReportDescReg = %x\n",HIDDS.ReportDescReg);
	mxt_dbg("InputReg = %x\n",HIDDS.InputReg);
	mxt_dbg("InputLen = %x\n",HIDDS.InputLen);
	mxt_dbg("OutputReg = %x\n",HIDDS.OutputReg);
	mxt_dbg("OutputLen = %x\n",HIDDS.OutputLen);
	mxt_dbg("CommandReg = %x\n",HIDDS.CommandReg);
	mxt_dbg("DataReg = %x\n",HIDDS.DataReg);
        mxt_dbg("VendorID = %x\n",HIDDS.VendorID);
        mxt_dbg("ProductID = %x\n",HIDDS.ProductID);
        mxt_dbg("VersionID = %x\n",HIDDS.VersionID);

	hid = hid_allocate_device();
	if (IS_ERR(hid))
		return PTR_ERR(hid);

	data->hid = hid;

	hid->ll_driver = &i2c_hid_driver;
	hid->hid_get_raw_report = i2chid_get_raw_report;
	hid->hid_output_raw_report = i2chid_output_raw_report;

	hid->dev.parent = &client->dev;
	hid->bus = BUS_I2C;
	hid->vendor = le16_to_cpu(HIDDS.VendorID);
	hid->product = le16_to_cpu(HIDDS.ProductID);
	hid->version = le16_to_cpu(HIDDS.VendorID);
	strlcpy(hid->name,client->name,sizeof(hid->name));

	ret = hid_add_device(hid);
	if (ret) {
		
		if (ret != -ENODEV)
			pr_err("can't add hid device: %d\n", ret);
	}
	pr_err("add HID %x\n",ret);

	return 0;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	mxt_dbg("mxt remove\n");

	hid_destroy_device(data->hid);
	free_irq(data->irq, data);
	
	kfree(data);	
	return 0;
}

static const struct i2c_device_id mxt_id[] = {
	{ "atmel_mxt_mxt1664S", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct acpi_device_id mxt_acpi_match[] = {
        { "ATML1000", 0 },
        { },
};
MODULE_DEVICE_TABLE(acpi, mxt_acpi_match);


static struct i2c_driver mxt_driver = {
        .driver = {
                .name   = "atmel_mxt_ts",
                .owner  = THIS_MODULE,
                .acpi_match_table = ACPI_PTR(mxt_acpi_match),
        },
        .probe          = mxt_probe,
        .remove         = __devexit_p(mxt_remove),
        .id_table       = mxt_id,
};

static int __init mxt_init(void)
{
	pr_err("add mxt driver\n");
        return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
        i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);


MODULE_AUTHOR("ChengAn Chiu");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen HID over I2C driver");
MODULE_LICENSE("GPL");


