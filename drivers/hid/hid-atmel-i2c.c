//<asus-ych20130904>
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
#include <linux/switch.h>

static int debug = 0;

#define MXT_TOUCH_FIRMWARE "atmel/mXT1664TC2U_Production_V1.0.AC.ENC"
#define MXT_TOUCH_CONFIG "atmel/ASUS_T100TA_0820_OI.raw"

#define mxt_err(fmt, arg...) pr_err(fmt, ##arg)

#define mxt_dbg(fmt, arg...)					  \
do {									  \
	if (debug)							  \
		pr_err(fmt, ##arg); \
} while (0)

#define I2C_HID_RESET_PENDING	(1 << 1)
#define I2C_HID_RWMM_PENDING   (1 << 10)


#pragma pack(1)
struct i2c_hid_desc{
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


struct i2c_hid_command
{
	u16	command_reg;
	u8	report_type_id;
	u8	opcode;
};

struct mxt_hid_output {
	//HID over I2C spec.
	u16	output_reg;
	u16	output_len;
	u8	report_id;
	//Mxt format
	u8	command_id;
	u8	num_wx;
	u8	num_rx;
	u16	addr;
	u8	data;	
}; 

struct mxt_hid_input {
	//HID over I2C spec.
	u16  	input_len;
	u8	report_id;
	//Mxt format
	union {
		u8 data[0];
		struct {	
			u8   	status;
			u8  	num_rx;
		}mxt;
	}report;
};

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
};

#pragma pack()

struct mxt_data {
	struct i2c_client *client;
	int gpio_num;
	int irq;
	struct hid_device *hid;
	unsigned long flags;
	wait_queue_head_t	wait;/* For waiting the interrupt */

	struct i2c_hid_desc	hid_desc;
	u16		hid_desc_addr;

	struct 	mxt_hid_output	*output_buffer;
	struct 	mxt_hid_input	*input_buffer;
	struct	mxt_info 	info;
	struct 	mxt_object *object_table;

	int	T5_Index;
	int 	T6_Index;
	int 	T19_Index;
	int 	T38_Index;
	u8	T6_ReportID;	
	u8	T19_ReportID;
	u32	ConfigCheckSum;
	u32	ConfigVersion;
	u8	touch_sel;

	u8	bootloader_addr;
	const struct firmware *fw; 

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct switch_dev 	pad_sdev;

};


static int mxt_write_read(struct i2c_client *client,u8 *write_buf, u16 write_len,u8 *read_buf, u16 read_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =2;

        memset(msg,0,sizeof(struct i2c_msg)*2);

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
                mxt_err("<asus-cca> i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static int mxt_write(struct i2c_client *client,u8 *write_buf, u16 write_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =1;

        memset(msg,0,sizeof(struct i2c_msg)*2);

        num = 1;
        msg[0].addr = client->addr;
        msg[0].flags = !I2C_M_RD;
        msg[0].len = write_len;
        msg[0].buf = write_buf;

        ret = i2c_transfer(client->adapter, msg, num);

        if(ret <0) {
                mxt_err("<asus-cca> i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static int mxt_read(struct i2c_client *client, u8 *read_buf, u16 read_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =2;

        memset(msg,0,sizeof(struct i2c_msg)*2);

        num = 1;

        msg[0].addr = client->addr;
        msg[0].flags = I2C_M_RD;
        msg[0].len = read_len;
        msg[0].buf = read_buf;

        ret = i2c_transfer(client->adapter, msg, num);

        if(ret <0) {
                mxt_err("<asus-cca> i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static int mxt_bootloader_read(struct mxt_data *data, u8 *read_buf, u16 read_len)
{
	int ret;
	struct i2c_msg msg;

	memset(&msg,0,sizeof(struct i2c_msg));
	msg.addr = data->bootloader_addr;
	msg.flags = I2C_M_RD;
	msg.len = read_len;
	msg.buf = read_buf;

	ret = i2c_transfer(data->client->adapter, &msg, 1);

        if(ret <0) {
                mxt_err("<asus-cca> i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static int mxt_bootloader_write(struct mxt_data *data,u8 *write_buf, u16 write_len)
{
	int ret;
	struct i2c_msg msg;

	memset(&msg,0,sizeof(struct i2c_msg));
	msg.addr = data->bootloader_addr;
	msg.flags = !I2C_M_RD;
	msg.len = write_len;
	msg.buf = write_buf;

	ret = i2c_transfer(data->client->adapter, &msg, 1);

        if(ret <0) {
                mxt_err("<asus-cca> i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

void parse_line (char  **buf_line,int	*psize, int  *argc,char  **argv)
{
	int 	arg;
  	char	*pchar;
  	bool	looking_for_arg;
  	int	index;

	*argc = 0;
  	looking_for_arg = true;
  	for (pchar = *buf_line, arg = 0,index = 0; (*pchar != '\0') && (*pchar != 0x0A) ; pchar++,index++) {
    		// Perform any text coversion here
    		if (looking_for_arg) {
      		// Look for the beging of an Argv[] entry
        		if (*pchar != ' ') {
        			argv[arg++] = pchar;
        			looking_for_arg = false;
      			}
    		} else {
      		// Looking for the terminator of an Argv[] entry
      			if ((*pchar == ' ') || (*pchar == 0x0D)) {
        			*pchar = '\0';
        			looking_for_arg = true;
      			}
    		}
    		if ((arg >= 1024)) {
      		// Error check buffer and exit since it does not look valid
      			break;
    		}
  	}
  	*argc = arg;
  	while(*pchar == 0x0A ||*pchar == 0x0D) {
		pchar++;
		index++;
  	}
  	*buf_line = pchar;	
  	*psize = index;
  	return;
}

static int mxt_hid_read_config(struct mxt_data *data,u16 addr,u8 *read_buf,u8 read_len)
{
	struct mxt_hid_output *output_buffer = data->output_buffer;
	struct mxt_hid_input *input_buffer = data->input_buffer;
	struct i2c_client *client = data->client;
	int time_out = 1000;

	memset(output_buffer,0,data->hid_desc.output_len+2);
	
        output_buffer->output_reg = data->hid_desc.output_reg;
        output_buffer->output_len = data->hid_desc.output_len;
        output_buffer->report_id    = 0x06;
        output_buffer->command_id = 0x51;
        output_buffer->num_wx     = 0x02;
        output_buffer->num_rx    = read_len;
	output_buffer->addr	  = addr;

	mxt_write(client,(u8 *)output_buffer,data->hid_desc.output_len+2);

	do
	{
		mxt_read(client,(u8 *)input_buffer,data->hid_desc.input_len);
		if((input_buffer->report_id == output_buffer->report_id) && (input_buffer->report.mxt.status ==0x00))
			break;
		msleep(1);
	}while(--time_out);
	
	if(!time_out) {
		mxt_err("<asus-cca> hid read config time out\n");
		return -EINVAL;
	}
	memcpy(read_buf,input_buffer+1,read_len);

	return 0;
}

static int mxt_hid_write(struct mxt_data *data,u16 addr,u8 *write_buf,u8 write_len)
{
        struct mxt_hid_output *output_buffer = data->output_buffer;
	struct i2c_client *client = data->client;

        memset(output_buffer,0,data->hid_desc.output_len+2);

        output_buffer->output_reg = data->hid_desc.output_reg;
        output_buffer->output_len = data->hid_desc.output_len;
        output_buffer->report_id    = 0x06;
        output_buffer->command_id = 0x51;
        output_buffer->num_wx     = write_len+0x02;
        output_buffer->num_rx    = 0x0;
        output_buffer->addr   = addr;

        memcpy(&(output_buffer->data),write_buf,write_len);
        mxt_write(client,(u8 *)output_buffer,data->hid_desc.output_len+2);

        return 0;
}



static int mxt_hid_write_config(struct mxt_data *data,u16 addr,u8 *write_buf,u8 write_len)
{
        struct mxt_hid_output *output_buffer = data->output_buffer;
        struct mxt_hid_input *input_buffer = data->input_buffer;
	struct i2c_client *client = data->client;
        int time_out = 1000;

	mxt_hid_write(data,addr,write_buf,write_len);

        do
        {
                mxt_read(client,(u8 *)input_buffer,data->hid_desc.input_len);
                if((input_buffer->report_id == output_buffer->report_id) && (input_buffer->report.mxt.status ==0x04))
                        break;
                msleep(1);
        }while(--time_out);

        if(!time_out) {
                mxt_err("<asus-cca> hid write config time out\n");
                return -EINVAL;
        }

        return 0;
}

static int mxt_init_object(struct mxt_data *data)
{
	u16 	index = 0x00;
	int 	object_num;
        u8   	cmd;
	int	time_out = 1000;	
        u8      report_id;
	int 	ret;
	u8  	buf[32];
	int	i;
	struct 	mxt_object	*object_table;


        data->output_buffer = kzalloc(data->hid_desc.output_len+2,GFP_KERNEL);
        data->input_buffer = kzalloc(data->hid_desc.input_len,GFP_KERNEL);

	ret = mxt_hid_read_config(data,index,(u8 *)&data->info,sizeof(struct mxt_info));
	if(ret != 0 ) {
		mxt_err("<asus-cca> read ID err %x\n",ret);
		return 0;
	}
	mxt_err("<asus-cca> Touch Firmware Version = %x Build = %x\n",data->info.version,data->info.build);

	object_num = data->info.object_num;

	data->object_table = kzalloc(sizeof(struct mxt_object)*object_num, GFP_KERNEL);

	object_table = data->object_table;

	index = sizeof(struct mxt_info);

	report_id = 1;//0 resever for atmel
	i=0;
	while(object_num >0) {
		ret = mxt_hid_read_config(data,index,(u8 *)&object_table[i],sizeof(struct mxt_object));
	        if(ret != 0 ) {
        	        mxt_err("<asus-cca> read Object err %x\n",ret);
      			return 0;
	  	}

		if(object_table[i].type == 0x05) {
			data->T5_Index = i;
		}
                if(object_table[i].type == 0x06) {
                        data->T6_Index = i;
			data->T6_ReportID = report_id;
                }
                if(object_table[i].type == 0x13) {
                        data->T19_Index = i;
			data->T19_ReportID = report_id;
                }
		if(object_table[i].type == 0x26) {
                        data->T38_Index = i;
                }
		report_id += (object_table[i].instances_minus_one+1)*object_table[i].num_report_ids;
                index += sizeof(struct mxt_object);
                object_num--;
		i++;
	}

	mxt_dbg("T5 addr = %x T6 addr = %x T6 report ID = %x\n",object_table[data->T5_Index].start_address,object_table[data->T6_Index].start_address,data->T6_ReportID);

	mxt_hid_read_config(data,object_table[data->T38_Index].start_address,(u8 *)&data->ConfigVersion,3);
        if(ret != 0 ) {
                mxt_err("<asus-cca> read config version err %x\n",ret);
                return 0;
        }

	mxt_err("<asus-cca> config ver = %x\n",data->ConfigVersion);

	//Use REPORTALL  to generate checksum
	cmd = 0x01;
	
	ret = mxt_hid_write_config(data,object_table[data->T6_Index].start_address+3,&cmd,1);
		
	if(ret <0) {
		mxt_err("<asus-cca> write REPORTALL err\n");
	}
	
	time_out = 1000;		
        do {
                ret = mxt_hid_read_config(data,object_table[data->T5_Index].start_address,buf,object_table[data->T5_Index].size_minus_one+1);
                if(ret <0) {
			mxt_err("<asus-cca> read REPORTALL err\n");
			break;
		}
                if(buf[0] == data->T6_ReportID) {
                        break;
		}
		msleep(100);
        }while(--time_out);
        if(!time_out) {
		mxt_err("<asus-cca> read REPORTALL time out\n");
	}
	
{
	int j =0 ;
	
	mxt_dbg("config checksum:");
	for(j=0;j<object_table[data->T5_Index].size_minus_one+1;j++)
		mxt_dbg("%x= %x ",j,buf[j]);
	mxt_dbg("\n");
}
	
	memcpy(&data->ConfigCheckSum,&buf[2],3);

	mxt_err("<asus-cca> Config checksum = %x\n",data->ConfigCheckSum);

	return 0;
}

//Update frame setting
#define WAITING_MASK      0xC0
#define WAITING_BOOTLOAD_CMD 0xC0
#define WAITING_FRAME_DATA 0x80
#define FRAME_CRC_CHECK 0x02
#define FRAME_CRC_FAIL  0x03
#define FRAME_CRC_PASS  0x04

static int mxt_update_frame(struct mxt_data *data,u8 *buf,int size) {
	u8	ack = 0;
	int	time_out;
	int	ret;
	
	time_out = 1000;
	do {
		mxt_bootloader_read(data,&ack,1);
		if((ack&WAITING_MASK)== WAITING_FRAME_DATA) 
			break;	
		msleep(1);
	}while(--time_out);

        if(!time_out) { 
		mxt_err("<asus-cca> waiting frame data time out\n");
               	return -EINVAL;
	}

	ret = mxt_bootloader_write(data,buf,size);

	time_out = 1000;
        do {
		mxt_bootloader_read(data,&ack,1);
                if(ack == FRAME_CRC_CHECK)
                        break;
                msleep(1);
        }while(--time_out);

        if(!time_out) {  
		mxt_err("<asus-cca> waiting frame crc time out\n");
                return -EINVAL;
	}

	time_out = 1000;
	ack = FRAME_CRC_FAIL;
	do {
		mxt_bootloader_read(data,&ack,1);
                if(ack == FRAME_CRC_PASS || ack == FRAME_CRC_FAIL)
                        break;
		msleep(1);
        }while(--time_out);

        if(!time_out) {
                mxt_err("<asus-cca> waiting frame crc  pass time out\n");
                return -EINVAL;
        }

        if(ack == FRAME_CRC_FAIL) {
                mxt_err("<asus-cca> CRC failed\n");
                return -EINVAL;
        }

	return 0;
}

static int mxt_update_frames(struct mxt_data *data,u8 *flash_data,int total_flash_size) {
	u16 	frame_size;	
	u8	ack = 0;
        int   	time_out;
	u16	unlock_device = 0xAADC;
	int 	flash_size = total_flash_size;
	int 	ret;       
 
	time_out = 1000;
        do {
		mxt_bootloader_read(data,&ack,1);
                if((ack&WAITING_MASK) == WAITING_BOOTLOAD_CMD)
                        break;
		msleep(1);	
        }while(--time_out);

        if(!time_out) {
		mxt_err("<asus-cca> wait bootloader cmd timeout\n");
                return -EINVAL;
	}

	mxt_dbg("unlock device\n");

	mxt_bootloader_write(data,(u8 *)&unlock_device,2);

	while(flash_size > 0 ) {
		frame_size = (int)(*flash_data<<8)|(int)(*(flash_data+1)+2);
		mxt_dbg("FlashSize = %x FrameSize= %x data = %x\n",flash_size,frame_size,flash_data[2]);
		ret = mxt_update_frame(data,flash_data,frame_size);
		if(ret) {
			mxt_err("<asus-cca> update frame error FlashSize = %x FrameSize= %x data = %x\n",flash_size,frame_size,flash_data[2]);
			return -EINVAL;
		}
		flash_data +=frame_size;
		flash_size -= frame_size;
	}
	mxt_dbg("\n");

	return 0;
}


static int mxt_check_bootloader(struct mxt_data *data)
{
	int ret;
	u8 tmp;

	ret = mxt_bootloader_read(data,&tmp,1);
	
	return ret;
} 

static int mxt_load_firmware(struct mxt_data *data,u8 **FlashData,int *FlashSize)
{
	const u8 *encode_data;
	int encode_size;
	u8 *str_data;
	int str_size;
	char str[3];
	int i;
	char *after;
        int ret ;
        const char *fn = MXT_TOUCH_FIRMWARE;
        struct i2c_client *client = data->client;

        ret = request_firmware(&data->fw, fn, &client->dev);
        if (ret < 0) {
                mxt_err("<asus-cca> Unable to open firmware %s\n", fn);
                return ret;
        }
        encode_data = data->fw->data;
        encode_size = data->fw->size;

	str_size = encode_size / 2;
	str_data = kzalloc(str_size, GFP_KERNEL);	
        str[2] = '\0';
        for(i=0;i<str_size;i++) {
		memcpy(str,encode_data,2);
		encode_data +=2;
		str_data[i] = (u8)simple_strtoul(str,&after,16);
	}
	
	*FlashData = str_data;
	*FlashSize = str_size;

	return 0;
}

static void mxt_unload_firmware(struct mxt_data *data,u8 *FlashData)
{
	kfree(FlashData);
	release_firmware(data->fw);
}
/*
static int mxt_recovery_firmware(struct mxt_data *data)
{
        u8      *flash_data;
        int     flash_size;
	int ret;

	ret = mxt_load_firmware(data,&flash_data,&flash_size);
	if(ret) {
		mxt_err("load fw error\n");
		return ret;
	}

	pr_err("Recovery firmware,please wait\n");

        ret = mxt_update_frames(data,flash_data,flash_size);
        if(ret) {
                mxt_err("update frames error\n");
		mxt_unload_firmware(data,flash_data);
                return -EINVAL;
        }

        msleep(1000);

	mxt_unload_firmware(data,flash_data);

        ret = mxt_check_bootloader(data);
        if(ret) {
                pr_err("flash firmware success\\n");
                return 0;
        }else {
                pr_err("exit boot mode failed\n");
                return -EINVAL;
        }
	
	return ret;
}
*/

static int mxt_update_firmware(struct mxt_data *data)
{
	u8	*flash_data;
	int	flash_size;
	struct mxt_object *object_table = data->object_table;
	u8	cmd;
	int ret;
	int time_out;

	ret = mxt_load_firmware(data,&flash_data,&flash_size);
        if(ret) {
                mxt_err("<asus-cca> load fw error\n");
                return ret;
        }

	mxt_err("<asus-cca> Flash firmware,please wait\n");	

        mxt_dbg("enter boot mode\n");
       	//enter bootloader mode
	cmd = 0xA5;
       	mxt_hid_write(data,object_table[data->T6_Index].start_address,&cmd,1);

	time_out = 100;
	do{
		ret = mxt_check_bootloader(data);
		if(!ret)
			break;
		msleep(200);
	}while(--time_out);

	if(!time_out) {
		mxt_err("<asus-cca> enter boot mode time out\n");
		mxt_unload_firmware(data,flash_data);
		return -EINVAL;
	}

	mxt_update_frames(data,flash_data,flash_size);
	
	msleep(1000);

	mxt_unload_firmware(data,flash_data);

        ret = mxt_check_bootloader(data);
        if(ret) {
                mxt_err("<asus-cca> flash firmware success\n");
                return 0;
        }else {
		mxt_err("<asus-cca> exit boot mode failed\n");
		return -EINVAL;
	}
}


static int mxt_update_config(struct mxt_data *data,char *config_data,int config_size)
{
	int 		i,j;
	int 		argc;
	char		**argv;
	int 		size;
	u8		type,inst,cd_size;	
	u8		*cd_data,*pcd_data;
	u8		config_object_found;	
	int		config_write_len;
	u16		config_base;
	u32		remaining;
	u8		cmd;
	u32		config_check_sum;
	char		*after;
	struct mxt_object       *object_table;
	u8      buf[32];
	int ret;
	int time_out;

	object_table = data->object_table;

	mxt_dbg("config size = %x\n",config_size);
	config_write_len = (data->hid_desc.output_len+2)-sizeof(struct mxt_hid_output);

	argv = kzalloc(sizeof(char*)*1024, GFP_KERNEL);

	parse_line(&config_data,&size,&argc,argv); //Title
	config_size -= size;
	parse_line(&config_data,&size,&argc,argv);//Version
	config_size -= size;
	parse_line(&config_data,&size,&argc,argv);//Firmware checksum
	config_size -= size;
	parse_line(&config_data,&size,&argc,argv);//Configure checksum
	config_size -= size;
	config_check_sum  = (u32)simple_strtoul(argv[0],&after,16);
	mxt_err("<asus-cca> current checksum = %x ,config checksum %x\n",data->ConfigCheckSum,config_check_sum); 

#if 0
	if(data->ConfigCheckSum == config_check_sum) {
		kfree(argv);
		return 0;
	}
#endif

	while(config_size) {
		parse_line(&config_data,&size,&argc,argv);
		type = (u8)simple_strtoul(argv[0],&after,16);
		inst = (u8)simple_strtoul(argv[1],&after,16);
		cd_size = (u8)simple_strtoul(argv[2],&after,16);
		mxt_dbg("type = %x CDSize = %x argc = %x\n",type,cd_size,argc-3);
		if(cd_size != (argc-3)) {
			mxt_err("<asus-cca> Type = %x Error CFG Data Size not match\n",type);
			break;
		}
		cd_data = kzalloc(sizeof(u8)*cd_size, GFP_KERNEL); 
		for(i=0;i<cd_size;i++)
			cd_data[i] = (u8)simple_strtoul(argv[3+i],&after,16);		
		
		config_object_found = 0;
		for(i=0;i<data->info.object_num;i++) {
			if(type == object_table[i].type) {
				//Update Configure
				pcd_data = cd_data;
				config_base = object_table[i].start_address+(object_table[i].size_minus_one+1)*inst;
				for(j=0;j<cd_size/config_write_len;j++) {
					mxt_hid_write_config(data,config_base,pcd_data,config_write_len);
					pcd_data += config_write_len;
					config_base += config_write_len;
				}
				remaining = cd_size%config_write_len;
				if(remaining)
					mxt_hid_write_config(data,config_base,pcd_data,remaining); 
				//MxtHIDWriteBlock()
				config_object_found = 1;
				break;
			}
		}
		if(!config_object_found)
			mxt_err("<asus-cca> Warring T%d object not match\n",type);	
		config_size -= size;
		kfree(cd_data);
	}

	kfree(argv);

	mxt_dbg("Backup config setting\n");
        //backup setting
        cmd = 0x55;
	mxt_hid_write_config(data,object_table[data->T6_Index].start_address+1,&cmd,1);

	
	mxt_dbg("reset mxt device\n");
	cmd = 0x01;
	mxt_hid_write_config(data,object_table[data->T6_Index].start_address,&cmd,1);
	msleep(200);

	mxt_err("Config udpate end\n");


        //Use REPORTALL  to generate checksum
        cmd = 0x01;

        ret = mxt_hid_write_config(data,object_table[data->T6_Index].start_address+3,&cmd,1);

        if(ret <0) {
                mxt_err("<asus-cca> write REPORTALL err\n");
        }

        time_out = 1000;
        do {
                ret = mxt_hid_read_config(data,object_table[data->T5_Index].start_address,buf,object_table[data->T5_Index].size_minus_one+1);
                if(ret <0) {
                        mxt_err("<asus-cca> read REPORTALL err\n");
                        break;
                }
                if(buf[0] == data->T6_ReportID) {
                        break;
                }
                msleep(100);
        }while(--time_out);
        if(!time_out) {
                mxt_err("<asus-cca> read REPORTALL time out\n");
        }

{
        int j =0 ;

        mxt_dbg("config checksum:");
        for(j=0;j<object_table[data->T5_Index].size_minus_one+1;j++)
                mxt_dbg("%x= %x ",j,buf[j]);
        mxt_dbg("\n");
}

        memcpy(&data->ConfigCheckSum,&buf[2],3);

        mxt_err("<asus-cca> Config checksum = %x\n",data->ConfigCheckSum);

	return 0;
}

static void mxt_report_handle(struct mxt_data *data)
{
        struct i2c_client *client = data->client;
	struct mxt_hid_input *input_buffer = data->input_buffer;
	int ret;

        mxt_read(client,(u8 *)input_buffer,data->hid_desc.input_len);

        mxt_dbg("len = %x id = %x %x %x %x %x %x %x %x %x\n",input_buffer->input_len,input_buffer->report_id,input_buffer->report.data[0],input_buffer->report.data[1],input_buffer->report.data[2],input_buffer->report.data[3],input_buffer->report.data[4],input_buffer->report.data[5],input_buffer->report.data[6],input_buffer->report.data[7]);

	if(!input_buffer->input_len) {
		if (test_and_clear_bit(I2C_HID_RESET_PENDING, &data->flags))
			wake_up(&data->wait);
		return;
	}
	if(input_buffer->report_id == 0x06) {
		return;
	}

	ret = hid_input_report(data->hid, HID_INPUT_REPORT,&input_buffer->report_id,data->hid_desc.input_len-2,1);
        mxt_dbg("input report ret= %x\n",ret);

	
}

static irqreturn_t mxt_thread_handler(int id, void *dev)
{
	struct mxt_data *data = (struct mxt_data *)dev;

	mxt_dbg("mxt_thread_handler\n");

	mxt_report_handle(data);

	return IRQ_HANDLED;

}

static void hid_set_power(struct hid_device *hid, int power)
{
	
	struct i2c_client *client = to_i2c_client(hid->dev.parent);
	struct mxt_data *data = i2c_get_clientdata(client);
        struct i2c_hid_command cmd;

        mxt_dbg("hid set power %x\n",power);
        cmd.command_reg = data->hid_desc.command_reg;
        cmd.report_type_id = power;
        cmd.opcode = 0x08;

        mxt_write(client,(u8 *)&cmd,sizeof(struct i2c_hid_command));

}

static int hid_reset(struct hid_device *hid)
{
	struct i2c_client *client = to_i2c_client(hid->dev.parent);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct i2c_hid_command cmd;
	int ret;

        mxt_dbg("hid reset\n");
        cmd.command_reg = data->hid_desc.command_reg;
        cmd.report_type_id = 0x00;
        cmd.opcode = 0x01;

	mxt_write(client,(u8 *)&cmd,sizeof(struct i2c_hid_command));
	set_bit(I2C_HID_RESET_PENDING, &data->flags);

	ret = wait_event_timeout(data->wait,!test_bit(I2C_HID_RESET_PENDING, &data->flags),msecs_to_jiffies(5000));
	if(!ret) {
		mxt_err("<asus-cca> hid reset error\n");
		return -ENODATA;
	}
	return 0;
}


static int i2chid_parse(struct hid_device *hid)
{
	struct i2c_client *client = to_i2c_client(hid->dev.parent);
	struct mxt_data *data = i2c_get_clientdata(client);
	char *rdesc;	
	int ret;

	mxt_dbg("i2c hid parse\n");
	hid_set_power(hid,0);

	ret = hid_reset(hid);

	if(ret) return ret;

	rdesc = kzalloc(data->hid_desc.report_desc_len, GFP_KERNEL);

	mxt_write_read(client,(u8 *)&data->hid_desc.report_desc_reg,2,(u8 *)rdesc,data->hid_desc.report_desc_len);	

	ret = hid_parse_report(hid, rdesc, data->hid_desc.report_desc_len);
	kfree(rdesc);
	if (ret) {
		mxt_err("<asus-cca> parsing report descriptor failed\n");
		return ret;
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
	mxt_dbg("power lvl = %x\n",lvl);
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

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *es)
{
	struct mxt_data *data;
	data = container_of(es, struct mxt_data, early_suspend);

	hid_set_power(data->hid,1);

}

static void mxt_late_resume(struct early_suspend *es)
{
	struct mxt_data *data;
	data = container_of(es, struct mxt_data, early_suspend);

	hid_set_power(data->hid,0);
}
#endif

static ssize_t touch_config_write(struct file *filp, struct kobject *kobj,
                struct bin_attribute *bin_attr, char *buf, loff_t off,
                size_t count)
{
        struct device *dev = container_of(kobj, struct device, kobj);
        struct mxt_data *data = dev_get_drvdata(dev);
	int ret;

	free_irq(data->irq, data);
	
	mxt_update_config(data,buf,count);

        ret = request_threaded_irq(data->irq, NULL,
                                   mxt_thread_handler,
                                   IRQF_ONESHOT,
                                   "mxt`-touch", data);
        irq_set_irq_type(data->irq, IRQ_TYPE_LEVEL_LOW);
        return count;
}


static struct bin_attribute touch_config_attr = {
        .attr.name = "config",
        .attr.mode = S_IRUGO | S_IWUSR, 
        .size = 0,
        .read = NULL,
        .write = touch_config_write
};

static ssize_t firmware_version_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return sprintf(buf,"Version = %x Build = %x\n",data->info.version,data->info.build);
}

static ssize_t config_checksum_show(struct device *dev,struct device_attribute *attr,char *buf)
{
        struct mxt_data *data = dev_get_drvdata(dev);

        return sprintf(buf,"Config checksum %x\n",data->ConfigCheckSum);
}

static ssize_t touch_sel_show(struct device *dev,struct device_attribute *attr,char *buf)
{
        struct mxt_data *data = dev_get_drvdata(dev);
	char tp[4][64] = {			
		"WTK VK",
		"WTK",
		"Cando",
		"Laibao" 	
	};
	int     time_out = 1000;
	struct  mxt_object      *object_table;
	u8 	cmd;
	int 	ret;
	u8	T5buf[32];	

	object_table = data->object_table;

        mxt_hid_read_config(data,object_table[data->T19_Index].start_address,&cmd,1);

        //set FORCEPT
        cmd |= 0x04;

        ret = mxt_hid_write_config(data,object_table[data->T19_Index].start_address,&cmd,1);

        if(ret <0) {
                mxt_err("<asus-cca> write FORCEPT err\n");
		return sprintf(buf,"Access touch gpio error, T19\n");
        }

        time_out = 1000;
        do {
                ret = mxt_hid_read_config(data,object_table[data->T5_Index].start_address,T5buf,object_table[data->T5_Index].size_minus_one+1);
                if(ret <0) {
                        mxt_err("<asus-cca> read REPORTALL err\n");
                        break;
                }
                if(T5buf[0] == data->T19_ReportID) {
                        break;
                }
                msleep(100);
        }while(--time_out);
        if(!time_out) {
                mxt_err("<asus-cca> read REPORTALL time out\n");
		return sprintf(buf,"Access touch gpio error, T5\n");
		
        }
        //buf 0 Report ID
        //Buf 1 Status
        data->touch_sel = T5buf[1];
        mxt_dbg("touch sel = %x\n",data->touch_sel);

        return sprintf(buf,"TOUCH_SEL0 = %x TOUCH_SEL1 = %x Touch Panel = %s\n",data->touch_sel&0x01,(data->touch_sel>>1)&0x01,tp[data->touch_sel&0x3]);
}


static DEVICE_ATTR(firmware_version, S_IRUGO | S_IWUSR,
                firmware_version_show, NULL);
static DEVICE_ATTR(config_checksum, S_IRUGO | S_IWUSR,
                config_checksum_show, NULL);
static DEVICE_ATTR(touch_sel, S_IRUGO | S_IWUSR,
                touch_sel_show,NULL);


static struct attribute *mxt_touch_attributes[] = {
        &dev_attr_firmware_version.attr,
        &dev_attr_config_checksum.attr,
        &dev_attr_touch_sel.attr,
        NULL
};

static const struct attribute_group mxt_attr_group = {
        .attrs = mxt_touch_attributes,
};

static ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	struct mxt_data	*data = container_of(sdev, struct mxt_data, pad_sdev);

	return sprintf(buf, "Version %2x Build %2x CheckSum = %4x\n",data->info.version,data->info.build,data->ConfigCheckSum);
}

static int __devinit mxt_probe(struct i2c_client *client,
                               const struct i2c_device_id *id)
{
	int ret;
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

	data->bootloader_addr = 0x26;

/*
	if(mxt_check_bootloader(data) ==0) {
		ret = mxt_recovery_firmware(data);
		if(ret)
			goto err_free_data;
	}
*/
	data->hid_desc_addr = 0x00;

	mxt_write_read(client,(u8 *)&data->hid_desc_addr,2,(u8 *)&data->hid_desc,2);

	mxt_write_read(client,(u8 *)&data->hid_desc_addr,2,(u8 *)&data->hid_desc,data->hid_desc.hid_desc_len);

	mxt_dbg("HIDDescLen = %x\n",data->hid_desc.hid_desc_len);
	mxt_dbg("bcdVersion = %x\n",data->hid_desc.bcd_version);
	mxt_dbg("ReportDescLen = %x\n",data->hid_desc.report_desc_len);
	mxt_dbg("ReportDescReg = %x\n",data->hid_desc.report_desc_reg);
	mxt_dbg("InputReg = %x\n",data->hid_desc.input_reg);
	mxt_dbg("InputLen = %x\n",data->hid_desc.input_len);
	mxt_dbg("OutputReg = %x\n",data->hid_desc.output_reg);
	mxt_dbg("OutputLen = %x\n",data->hid_desc.output_len);
	mxt_dbg("CommandReg = %x\n",data->hid_desc.command_reg);
	mxt_dbg("DataReg = %x\n",data->hid_desc.data_reg);
        mxt_dbg("VendorID = %x\n",data->hid_desc.vendor_id);
        mxt_dbg("ProductID = %x\n",data->hid_desc.product_id);
        mxt_dbg("VersionID = %x\n",data->hid_desc.version_id);


        ret = mxt_init_object(data);
	if(ret)
		goto err_free_data;

        if (sysfs_create_bin_file(&client->dev.kobj,&touch_config_attr) < 0) {
                mxt_err("Failed to create %s\n",touch_config_attr.attr.name);
        }

        if ( sysfs_create_group(&client->dev.kobj, &mxt_attr_group)) {
                mxt_err("failed to create sysfs group\n");
        }

#if 0
	//Auto update config and firmware
	if(data->info.version != 0x10 || data->info.build != 0xAC) {
		ret = mxt_update_firmware(data);
		if(ret)
			goto err_free_data_object;
	}

        ret = mxt_update_config(data);
        if(ret)
        	goto err_free_data_object;
#endif

        /* register interrupt */
        ret = request_threaded_irq(data->irq, NULL,
                                        mxt_thread_handler,
                                        IRQF_ONESHOT,
                                        "mxt-touch", data);
        if (ret) {
                mxt_err("<asus-cca> cannot get IRQ:%d\n", data->irq);
                data->irq = -1;
		goto err_free_data_object;
        } else {
                mxt_err("<asus-cca> IRQ No:%d\n", data->irq);
        }
    irq_set_irq_type(data->irq, IRQ_TYPE_LEVEL_LOW);

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		ret =  PTR_ERR(hid);
		goto err_free_irq;
	}
	data->hid = hid;

	hid->ll_driver = &i2c_hid_driver;
	hid->hid_get_raw_report = i2chid_get_raw_report;
	hid->hid_output_raw_report = i2chid_output_raw_report;

	hid->dev.parent = &client->dev;
	hid->bus = BUS_I2C;
	hid->vendor = le16_to_cpu(data->hid_desc.vendor_id);
	hid->product = le16_to_cpu(data->hid_desc.product_id);
	hid->version = le16_to_cpu(data->hid_desc.vendor_id);
	strlcpy(hid->name,client->name,sizeof(hid->name));

	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			mxt_err("<asus-cca> can't add hid device: %d\n", ret);
		goto err_free_hid;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
	data->pad_sdev.name = "touch";
	data->pad_sdev.print_name = touch_switch_name;
	if(switch_dev_register(&data->pad_sdev) < 0){
		mxt_err("switch_dev_register for pad failed!\n");
	}

	mxt_err("<asus-cca> add HID %s\n",hid->name);

	return 0;

err_free_hid:
	hid_destroy_device(data->hid);
err_free_irq:
	free_irq(data->irq, data);
err_free_data_object:
	kfree(data->output_buffer);
	kfree(data->input_buffer);
err_free_data:
	kfree(data);

	return ret;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	mxt_dbg("mxt remove\n");

	sysfs_remove_bin_file(&data->client->dev.kobj, &touch_config_attr);

	sysfs_remove_group(&data->client->dev.kobj, &mxt_attr_group);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	hid_destroy_device(data->hid);
	free_irq(data->irq, data);

	kfree(data->output_buffer);
	kfree(data->input_buffer);
	kfree(data);	

	return 0;
}
#if 0
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

	hid_set_power(data->hid,1);

	return 0;
}

static int mxt_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct mxt_data *data = i2c_get_clientdata(client);

        hid_set_power(data->hid,0);

	return 0;
}
#endif

#if 0
static SIMPLE_DEV_PM_OPS(mxt_pm, mxt_suspend, mxt_resume);
#endif

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
#if 0
		.pm	= &mxt_pm,
#endif
                .acpi_match_table = ACPI_PTR(mxt_acpi_match),
        },
        .probe          = mxt_probe,
        .remove         = __devexit_p(mxt_remove),
        .id_table       = mxt_id,
};

static int __init mxt_init(void)
{
	mxt_err("<asus-cca> add mxt driver\n");
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


