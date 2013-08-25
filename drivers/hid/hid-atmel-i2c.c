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
#include "atmel-config.h"
#include "atmel-firmware.h"

static int debug = 0;

#define mxt_dbg(fmt, arg...)					  \
do {									  \
	if (debug)							  \
		pr_err(fmt, ##arg); \
} while (0)

#define I2C_HID_RESET_PENDING	(1 << 1)
#define I2C_HID_RWMM_PENDING   (1 << 10)


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


struct HIDCommand
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

	MxtHIDOutput *MxtHIDOut;
	MxtHIDInput  *MxtHIDIn;
	IDInfo 		ID;
	ObjectType      *MxtCFGObjs;
	int	T5_Index;
	int 	T6_Index;
	int 	T38_Index;
	u8	T6_ReportID;	
	u32	ConfigCheckSum;
	u32	ConfigVersion;

	u8	bootloader_addr;
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

        memset(msg,0,sizeof(struct i2c_msg)*2);

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

        memset(msg,0,sizeof(struct i2c_msg)*2);

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
                pr_err("i2c transfer error\n");
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
                pr_err("i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static HIDDesc HIDDS;
static MxtHIDOutput *MxtHIDOut;
static MxtHIDInput  *MxtHIDIn;


void ParseLine (char  **BufLine,int	*PSize, int  *Argc,char  **Argv)
{
	int 	Arg;
  	char	*Char;
  	bool	LookingForArg;
  	int	Index;

	*Argc = 0;
  	LookingForArg = TRUE;
  	for (Char = *BufLine, Arg = 0,Index = 0; (*Char != '\0') && (*Char != 0x0A) ; Char++,Index++) {
    		// Perform any text coversion here
    		if (LookingForArg) {
      		// Look for the beging of an Argv[] entry
        		if (*Char != ' ') {
        			Argv[Arg++] = Char;
        			LookingForArg = FALSE;
      			}
    		} else {
      		// Looking for the terminator of an Argv[] entry
      			if ((*Char == ' ') || (*Char == 0x0D)) {
        			*Char = '\0';
        			LookingForArg = TRUE;
      			}
    		}
    		if ((Arg >= 1024)) {
      		// Error check buffer and exit since it does not look valid
      			break;
    		}
  	}
  	*Argc = Arg;
  	while(*Char == 0x0A ||*Char == 0x0D) {
		Char++;
		Index++;
  	}
  	*BufLine = Char;	
  	*PSize = Index;
  	return;
}

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

	mxt_write(client,(u8 *)MxtHIDOut,HIDDS.OutputLen+2);

	do
	{
		mxt_read(client,(u8 *)MxtHIDIn,HIDDS.InputLen);
		if((MxtHIDIn->ReportID == MxtHIDOut->ReportID) && (MxtHIDIn->Status ==0x00))
			break;
		msleep(1);
	}while(--TimeOut);
	
	if(!TimeOut) return -1;

	memcpy(read_buf,MxtHIDIn+1,read_len);

	return 0;
}

static int mxt_hid_write(struct i2c_client *client,u16 addr,u8 *write_buf,u8 write_len)
{
        memset(MxtHIDOut,0,HIDDS.OutputLen+2);

        MxtHIDOut->OutputReg = HIDDS.OutputReg;
        MxtHIDOut->OutputLen = HIDDS.OutputLen;
        MxtHIDOut->ReportID    = 0x06;
        MxtHIDOut->CommandID = 0x51;
        MxtHIDOut->NumWx     = write_len+0x02;
        MxtHIDOut->NumRx    = 0x0;
        MxtHIDOut->Addr   = addr;

        memcpy(&(MxtHIDOut->Data),write_buf,write_len);
        mxt_write(client,(u8 *)MxtHIDOut,HIDDS.OutputLen+2);

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

	memcpy(&(MxtHIDOut->Data),write_buf,write_len);
        mxt_write(client,(u8 *)MxtHIDOut,HIDDS.OutputLen+2);

        do
        {
                mxt_read(client,(u8 *)MxtHIDIn,HIDDS.InputLen);
                if((MxtHIDIn->ReportID == MxtHIDOut->ReportID) && (MxtHIDIn->Status ==0x04))
                        break;
                msleep(1);
        }while(--TimeOut);

        if(!TimeOut) return -1;
        return 0;
}

static int mxt_init_object(struct mxt_data *data)
{
	u16 	index = 0x00;
	int 	ObjNum;
        u8   	Cmd;
	int	TimeOut = 1000;	
        u8      ReportID;
	u32	CheckSum;
	int 	ret;
	u8  	Buf[32];
	int	i;
	ObjectType      *MxtCFGObjs;
	struct i2c_client *client = data->client;

	MxtHIDOut = kzalloc(HIDDS.OutputLen+2,GFP_KERNEL);
	MxtHIDIn = kzalloc(HIDDS.InputLen,GFP_KERNEL);

        data->MxtHIDOut = kzalloc(HIDDS.OutputLen+2,GFP_KERNEL);
        data->MxtHIDIn = kzalloc(HIDDS.InputLen,GFP_KERNEL);

	//static int mxt_hid_read_config(struct i2c_client *client,u16 addr,u8 *read_buf,u8 read_len)
	ret = mxt_hid_read_config(client,index,(u8 *)&data->ID,sizeof(IDInfo));
	if(ret != 0 ) {
		pr_err("read ID err %x\n",ret);
		return 0;
	}
	pr_err("Touch Firmware Version = %x Build = %x\n",data->ID.Version,data->ID.Build);

	ObjNum = data->ID.NumOfObject;
	data->MxtCFGObjs = kzalloc(sizeof(ObjectType)*ObjNum, GFP_KERNEL);
	MxtCFGObjs = data->MxtCFGObjs;
	index = sizeof(IDInfo);
	ReportID = 1;//0 resever for atmel
	i=0;
	while(ObjNum >0) {
		ret = mxt_hid_read_config(client,index,(u8 *)&MxtCFGObjs[i],sizeof(ObjectType));
	        if(ret != 0 ) {
        	        pr_err("read Object err %x\n",ret);
      			return 0;
	  	}

		if(MxtCFGObjs[i].Type == 0x05) {
			data->T5_Index = i;
		}
                if(MxtCFGObjs[i].Type == 0x06) {
                        data->T6_Index = i;
			data->T6_ReportID = ReportID;
                }
		if(MxtCFGObjs[i].Type == 0x26) {
                        data->T38_Index = i;
                }
		ReportID += (MxtCFGObjs[i].Inst_dec1+1)*MxtCFGObjs[i].NumOfReportID;
                index += sizeof(ObjectType);
                ObjNum--;
		i++;
	}

	mxt_dbg("T5 addr = %x T6 addr = %x T6 report ID = %x\n",*((UINT16*)&MxtCFGObjs[data->T5_Index].LSB),*((u16*)&MxtCFGObjs[data->T6_Index].LSB),data->T6_ReportID);

	mxt_hid_read_config(client,*((UINT16*)&MxtCFGObjs[data->T38_Index].LSB),(u8 *)&data->ConfigVersion,3);
        if(ret != 0 ) {
                pr_err("read config version err %x\n",ret);
                return 0;
        }

	pr_err("config ver = %x\n",data->ConfigVersion);

	//Use REPORTALL  to generate checksum
	CheckSum = 0x0;

	Cmd = 0x01;
	
	ret = mxt_hid_write_config(client,*((u16*)&MxtCFGObjs[data->T6_Index].LSB)+3,&Cmd,1);
		
	if(ret <0) {
		pr_err("write REPORTALL err\n");
	}
			
        do {
                ret = mxt_hid_read_config(client,*((UINT16*)&MxtCFGObjs[data->T5_Index].LSB),Buf,MxtCFGObjs[data->T5_Index].Size_dec1+1);
                if(ret <0) {
			pr_err("read REPORTALL err\n");
			break;
		}
                if(Buf[0] == data->T6_ReportID) {
                        break;
		}
		msleep(100);
        }while(--TimeOut);
        if(!TimeOut) {
		pr_err("read REPORTALL time out\n");
	}
	
{
	int j =0 ;
	
	mxt_dbg("config checksum:");
	for(j=0;j<MxtCFGObjs[data->T5_Index].Size_dec1+1;j++)
		mxt_dbg("%x= %x ",j,Buf[j]);
	mxt_dbg("\n");
}
	
	memcpy(&data->ConfigCheckSum,&Buf[2],3);
	return 0;
}

//Update frame setting
#define WAITING_MASK      0xC0
#define WAITING_BOOTLOAD_CMD 0xC0
#define WAITING_FRAME_DATA 0x80
#define FRAME_CRC_CHECK 0x02
#define FRAME_CRC_FAIL  0x03
#define FRAME_CRC_PASS  0x04

int UpdateFrame(struct mxt_data *data,u8 *Buf,int Size) {
	u8	Ack = 0;
	int	TimeOut;
	int	ret;
	
	TimeOut = 1000;
	do {
		mxt_bootloader_read(data,&Ack,1);
		if((Ack&WAITING_MASK)== WAITING_FRAME_DATA) 
			break;	
		msleep(1);
	}while(--TimeOut);

        if(!TimeOut) { 
		pr_err("waiting frame data time out\n");
               	return -EINVAL;
	}

	ret = mxt_bootloader_write(data,Buf,Size);

	TimeOut = 1000;
        do {
		mxt_bootloader_read(data,&Ack,1);
                if(Ack == FRAME_CRC_CHECK)
                        break;
                msleep(1);
        }while(--TimeOut);

        if(!TimeOut) {  
		pr_err("waiting frame crc time out\n");
                return -EINVAL;
	}

	TimeOut = 1000;
	Ack = FRAME_CRC_FAIL;
	do {
		mxt_bootloader_read(data,&Ack,1);
                if(Ack == FRAME_CRC_PASS || Ack == FRAME_CRC_FAIL)
                        break;
		msleep(1);
        }while(--TimeOut);

        if(!TimeOut) {
                pr_err("waiting frame crc  pass time out\n");
                return -EINVAL;
        }

        if(Ack == FRAME_CRC_FAIL) {
                pr_err("CRC failed\n");
                return -EINVAL;
        }

	return 0;
}

int UpdateataFrames(struct mxt_data *data,u8 *FlashData,int TotalFlashSize) {
	u16 FrameSize;	
	u8	Ack = 0;
        int   TimeOut;
	u16	UnLockDevice = 0xAADC;
	int 	FlashSize = TotalFlashSize;
	int ret;       
 
	TimeOut = 1000;
        do {
		mxt_bootloader_read(data,&Ack,1);
                if((Ack&WAITING_MASK) == WAITING_BOOTLOAD_CMD)
                        break;
		msleep(1);	
        }while(--TimeOut);

        if(!TimeOut) {
		pr_err("wait bootloader cmd timeout\n");
                return -EINVAL;
	}

	mxt_dbg("unlock device\n");

	mxt_bootloader_write(data,(u8 *)&UnLockDevice,2);

	while(FlashSize > 0 ) {
		FrameSize = (int)(*FlashData<<8)|(int)(*(FlashData+1)+2);
		mxt_dbg("FlashSize = %x FrameSize= %x data = %x\n",FlashSize,FrameSize,FlashData[0]);
		ret = UpdateFrame(data,FlashData,FrameSize);
		if(ret) return -1;

		FlashData +=FrameSize;
		FlashSize -= FrameSize;
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

static void mxt_load_firmware(u8 **FlashData,int *FlashSize)
{
	u8 *encode_data = atmel_firmware;
	int encode_size = strlen(atmel_firmware);
	u8 *str_data;
	int str_size;
	char str[3];
	int i;
	char *after;

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
}

static int mxt_recovery_firmware(struct mxt_data *data)
{
        u8      *FlashData;
        int     FlashSize;
	int ret;

	mxt_load_firmware(&FlashData,&FlashSize);

	pr_err("Recovery firmware,please wait\n");

        UpdateataFrames(data,FlashData,FlashSize);

        msleep(1000);

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


static int mxt_update_firmware(struct mxt_data *data)
{
	u8	*FlashData;
	int	FlashSize;
        struct i2c_client *client = data->client;
        ObjectType      *MxtCFGObjs = data->MxtCFGObjs;
	u8	Cmd;
	int ret;

	mxt_load_firmware(&FlashData,&FlashSize);

	pr_err("Flash firmware,please wait\n");	

        mxt_dbg("enter boot mode\n");
       	//enter bootloader mode
	Cmd = 0xA5;
       	mxt_hid_write(client,*((UINT16*)&MxtCFGObjs[data->T6_Index].LSB),&Cmd,1);

	msleep(6000);

	mxt_dbg("check bootloader mode\n");
	ret = mxt_check_bootloader(data);
	if(ret) {
		pr_err("enter boot mode failed\n");
		return ret;
	}


	UpdateataFrames(data,FlashData,FlashSize);
	
	msleep(1000);

        ret = mxt_check_bootloader(data);
        if(ret) {
                pr_err("flash firmware success\n");
                return 0;
        }else {
		pr_err("exit boot mode failed\n");
		return -EINVAL;
	}
}


static int mxt_update_config(struct mxt_data *data)
{
	int 	i,j;
	int 	Argc;
	char	**Argv;
	char	*CFGData;
	int 		CFGSize,Size;
	u8		Type,Inst,CDSize;	
	u8		*CData,*PData;
	u8		CFGObjFound;	
	int		CFGWriteLen;
	u16		CFGBase;
	u32		Remaining;
	u8		Cmd;
	u32		ConfigCheckSum;
	char		*after;
	struct i2c_client *client = data->client;
	ObjectType      *MxtCFGObjs;


	MxtCFGObjs = data->MxtCFGObjs;
	CFGData = atmel_config;
  	CFGSize = strlen(atmel_config);

	mxt_dbg("config size = %x\n",CFGSize);
	CFGWriteLen = (HIDDS.OutputLen+2)-sizeof(MxtHIDOutput);

	Argv = kzalloc(sizeof(char*)*1024, GFP_KERNEL);

	ParseLine(&CFGData,&Size,&Argc,Argv); //Title
	CFGSize -= Size;
	ParseLine(&CFGData,&Size,&Argc,Argv);//Version
	CFGSize -= Size;
	ParseLine(&CFGData,&Size,&Argc,Argv);//Firmware checksum
	CFGSize -= Size;
	ParseLine(&CFGData,&Size,&Argc,Argv);//Configure checksum
	CFGSize -= Size;
	ConfigCheckSum  = (u32)simple_strtoul(Argv[0],&after,16);
	pr_err("current checksum = %x ,config checksum %x\n",data->ConfigCheckSum,ConfigCheckSum); 

	if(data->ConfigCheckSum == ConfigCheckSum) {
		kfree(Argv);
		return 0;
	}

	while(CFGSize) {
		ParseLine(&CFGData,&Size,&Argc,Argv);
		Type = (u8)simple_strtoul(Argv[0],&after,16);
		Inst = (u8)simple_strtoul(Argv[1],&after,16);
		CDSize = (u8)simple_strtoul(Argv[2],&after,16);
		mxt_dbg("type = %x CDSize = %x argc = %x\n",Type,CDSize,Argc-3);
		if(CDSize != (Argc-3)) {
			pr_err("Type = %x Error CFG Data Size not match\n",Type);
			break;
		}
		CData = kzalloc(sizeof(u8)*CDSize, GFP_KERNEL); 
		for(i=0;i<CDSize;i++)
			CData[i] = (u8)simple_strtoul(Argv[3+i],&after,16);		
		
		CFGObjFound = 0;
		for(i=0;i<data->ID.NumOfObject;i++) {
			if(Type == data->MxtCFGObjs[i].Type) {
				//Update Configure
				PData = CData;
				CFGBase = *((UINT16*)&MxtCFGObjs[i].LSB)+(MxtCFGObjs[i].Size_dec1+1)*Inst;
				for(j=0;j<CDSize/CFGWriteLen;j++) {
					mxt_hid_write_config(client,CFGBase,PData,CFGWriteLen);
					PData += CFGWriteLen;
					CFGBase += CFGWriteLen;
				}
				Remaining = CDSize%CFGWriteLen;
				if(Remaining)
					mxt_hid_write_config(client,CFGBase,PData,Remaining); 
				//MxtHIDWriteBlock()
				CFGObjFound = 1;
				break;
			}
		}
		if(!CFGObjFound)
			pr_err("Warring T%d object not match\n",Type);	
		CFGSize -= Size;
		kfree(CData);
	}

	kfree(Argv);

	mxt_dbg("Backup config setting\n");
        //backup setting
        Cmd = 0x55;
	mxt_hid_write_config(client,*((UINT16*)&MxtCFGObjs[data->T6_Index].LSB)+1,&Cmd,1);

	
	mxt_dbg("reset mxt device\n");
	Cmd = 0x01;
	mxt_hid_write_config(client,*((UINT16*)&MxtCFGObjs[data->T6_Index].LSB),&Cmd,1);
	msleep(200);

	return 0;
}

static void mxt_report_handle(struct mxt_data *data)
{
        struct i2c_client *client = data->client;
        u8  Buf[32];
	int length;
	int ret;

        mxt_read(client,Buf,HIDDS.InputLen);

	length = Buf[0] | Buf[1] << 8;
        mxt_dbg("%x %x %x %x %x %x %x %x\n",Buf[0],Buf[1],Buf[2],Buf[3],Buf[4],Buf[5],Buf[6],Buf[7]);

	if(!length) {
		if (test_and_clear_bit(I2C_HID_RESET_PENDING, &data->flags))
			wake_up(&data->wait);
		return;
	}
	if(Buf[2] == 0x06) {
		return;
	}

	ret = hid_input_report(data->hid, HID_INPUT_REPORT,&Buf[2],HIDDS.InputLen-2,1);
        mxt_dbg("input report = %x  %x ret = %x\n",Buf[3],Buf[4],ret);

	
}

static irqreturn_t mxt_thread_handler(int id, void *dev)
{
	struct mxt_data *data = (struct mxt_data *)dev;

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

        mxt_write(client,(u8 *)&Cmd,sizeof(struct HIDCommand));

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

	mxt_write(client,(u8 *)&Cmd,sizeof(struct HIDCommand));
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
	int ret;

	mxt_dbg("parse\n");
	hid_set_power(hid,0);

	ret = hid_reset(hid);

	if(ret) return ret;

	rdesc = kzalloc(HIDDS.ReportDescLen, GFP_KERNEL);

	mxt_write_read(client,(u8 *)&HIDDS.ReportDescReg,2,(u8 *)rdesc,HIDDS.ReportDescLen);	

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

static int __devinit mxt_probe(struct i2c_client *client,
                               const struct i2c_device_id *id)
{
	int ret;
	u16 HIDStartAddr = 0x00;
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

	if(mxt_check_bootloader(data) ==0) {
		ret = mxt_recovery_firmware(data);
		if(ret)
			goto err_free_data;
	}


	mxt_write_read(client,(u8 *)&HIDStartAddr,2,(u8 *)&HIDDS,2);

	mxt_write_read(client,(u8 *)&HIDStartAddr,2,(u8 *)&HIDDS,HIDDS.HIDDescLen);

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


        ret = mxt_init_object(data);
	if(ret)
		goto err_free_data;

	if(data->ID.Version != 0x10 || data->ID.Build != 0xAC) {
		ret = mxt_update_firmware(data);
		if(ret)
			goto err_free_data_object;
	}

        ret = mxt_update_config(data);
        if(ret)
        	goto err_free_data_object;


        /* register interrupt */
        ret = request_threaded_irq(data->irq, NULL,
                                        mxt_thread_handler,
                                        IRQF_TRIGGER_LOW|IRQF_TRIGGER_FALLING,
                                        "mxt-touch", data);
        if (ret) {
                pr_err("cannot get IRQ:%d\n", data->irq);
                data->irq = -1;
		goto err_free_data_object;
        } else {
                pr_err("IRQ No:%d\n", data->irq);
        }

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
	hid->vendor = le16_to_cpu(HIDDS.VendorID);
	hid->product = le16_to_cpu(HIDDS.ProductID);
	hid->version = le16_to_cpu(HIDDS.VendorID);
	strlcpy(hid->name,client->name,sizeof(hid->name));

	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			pr_err("can't add hid device: %d\n", ret);
		goto err_free_hid;
	}
	pr_err("add HID %s\n",hid->name);

	return 0;

err_free_hid:
	hid_destroy_device(data->hid);
err_free_irq:
	free_irq(data->irq, data);
err_free_data_object:
	kfree(data->MxtHIDOut);
	kfree(data->MxtHIDIn);
err_free_data:
	kfree(data);

	return ret;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	mxt_dbg("mxt remove\n");

	hid_destroy_device(data->hid);
	free_irq(data->irq, data);

	kfree(data->MxtHIDOut);
	kfree(data->MxtHIDIn);
	kfree(data);	

        if(MxtHIDOut)
                kfree(MxtHIDOut);
        if(MxtHIDIn)
                kfree(MxtHIDIn);

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


