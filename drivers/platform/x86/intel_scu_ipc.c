/*
 * intel_scu_ipc.c: Driver for the Intel SCU IPC mechanism
 *
 * (C) Copyright 2008-2010 Intel Corporation
 * Author: Sreedhara DS (sreedhara.ds@intel.com)
 * (C) Copyright 2010 Intel Corporation
 * Author: Sudha Krishnakumar (sudha.krishnakumar@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * SCU running in ARC processor communicates with other entity running in IA
 * core through IPC mechanism which in turn messaging between IA core ad SCU.
 * SCU has two IPC mechanism IPC-1 and IPC-2. IPC-1 is used between IA32 and
 * SCU where IPC-2 is used between P-Unit and SCU. This driver delas with
 * IPC-1 Driver provides an API for power control unit registers (e.g. MSIC)
 * along with other APIs.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/pm.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/sfi.h>
#include <asm/mrst.h>
#include <asm/intel_scu_ipc.h>

/* IPC defines the following message types */
#define IPCMSG_WATCHDOG_TIMER 0xF8 /* Set Kernel Watchdog Threshold */
#define IPCMSG_BATTERY        0xEF /* Coulomb Counter Accumulator */
#define IPCMSG_FW_UPDATE      0xFE /* Firmware update */
#define IPCMSG_PCNTRL         0xFF /* Power controller unit read/write */
#define IPCMSG_FW_REVISION    0xF4 /* Get firmware revision */

/* Command id associated with message IPCMSG_PCNTRL */
#define IPC_CMD_PCNTRL_W      0 /* Register write */
#define IPC_CMD_PCNTRL_R      1 /* Register read */
#define IPC_CMD_PCNTRL_M      2 /* Register read-modify-write */

/*
 * IPC register summary
 *
 * IPC register blocks are memory mapped at fixed address of 0xFF11C000
 * To read or write information to the SCU, driver writes to IPC-1 memory
 * mapped registers (base address 0xFF11C000). The following is the IPC
 * mechanism
 *
 * 1. IA core cDMI interface claims this transaction and converts it to a
 *    Transaction Layer Packet (TLP) message which is sent across the cDMI.
 *
 * 2. South Complex cDMI block receives this message and writes it to
 *    the IPC-1 register block, causing an interrupt to the SCU
 *
 * 3. SCU firmware decodes this interrupt and IPC message and the appropriate
 *    message handler is called within firmware.
 */

#define IPC_BASE_ADDR     0xFF11C000	/* IPC1 base register address */
#define IPC_MAX_ADDR      0x100		/* Maximum IPC regisers */
#define IPC_WWBUF_SIZE    20		/* IPC Write buffer Size */
#define IPC_RWBUF_SIZE    20		/* IPC Read buffer Size */
#define IPC_I2C_BASE      0xFF12B000	/* I2C control register base address */
#define IPC_I2C_MAX_ADDR  0x10		/* Maximum I2C regisers */

static int ipc_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void ipc_remove(struct pci_dev *pdev);

struct intel_scu_ipc_dev {
	struct pci_dev *pdev;
	void __iomem *ipc_base;
	void __iomem *i2c_base;
};

static struct intel_scu_ipc_dev  ipcdev; /* Only one for now */

static int platform;		/* Platform type */

/*
 * IPC Read Buffer (Read Only):
 * 16 byte buffer for receiving data from SCU, if IPC command
 * processing results in response data
 */
#define IPC_READ_BUFFER		0x90

#define IPC_I2C_CNTRL_ADDR	0
#define I2C_DATA_ADDR		0x04

static DEFINE_MUTEX(ipclock); /* lock used to prevent multiple call to SCU */

/*
 * Command Register (Write Only):
 * A write to this register results in an interrupt to the SCU core processor
 * Format:
 * |rfu2(8) | size(8) | command id(4) | rfu1(3) | ioc(1) | command(8)|
 */
static inline void ipc_command(u32 cmd) /* Send ipc command */
{
	writel(cmd, ipcdev.ipc_base);
}

/*
 * IPC Write Buffer (Write Only):
 * 16-byte buffer for sending data associated with IPC command to
 * SCU. Size of the data is specified in the IPC_COMMAND_REG register
 */
static inline void ipc_data_writel(u32 data, u32 offset) /* Write ipc data */
{
	writel(data, ipcdev.ipc_base + 0x80 + offset);
}

/*
 * Status Register (Read Only):
 * Driver will read this register to get the ready/busy status of the IPC
 * block and error status of the IPC command that was just processed by SCU
 * Format:
 * |rfu3(8)|error code(8)|initiator id(8)|cmd id(4)|rfu1(2)|error(1)|busy(1)|
 */

static inline u8 ipc_read_status(void)
{
	return __raw_readl(ipcdev.ipc_base + 0x04);
}

static inline u8 ipc_data_readb(u32 offset) /* Read ipc byte data */
{
	return readb(ipcdev.ipc_base + IPC_READ_BUFFER + offset);
}

static inline u32 ipc_data_readl(u32 offset) /* Read ipc u32 data */
{
	return readl(ipcdev.ipc_base + IPC_READ_BUFFER + offset);
}

static inline int busy_loop(void) /* Wait till scu status is busy */
{
	u32 status = 0;
	u32 loop_count = 0;

	status = ipc_read_status();
	while (status & 1) {
		udelay(1); /* scu processing time is in few u secods */
		status = ipc_read_status();
		loop_count++;
		/* break if scu doesn't reset busy bit after huge retry */
		if (loop_count > 100000) {
			dev_err(&ipcdev.pdev->dev, "IPC timed out");
			return -ETIMEDOUT;
		}
	}
	if ((status >> 1) & 1)
		return -EIO;

	return 0;
}

/* Read/Write power control(PMIC in Langwell, MSIC in PenWell) registers */
static int pwr_reg_rdwr(u16 *addr, u8 *data, u32 count, u32 op, u32 id)
{
	int i, nc, bytes, d;
	u32 offset = 0;
	int err;
	u8 cbuf[IPC_WWBUF_SIZE] = { };
	u32 *wbuf = (u32 *)&cbuf;

	mutex_lock(&ipclock);

	memset(cbuf, 0, sizeof(cbuf));

	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}

	if (platform != MRST_CPU_CHIP_PENWELL) {
		bytes = 0;
		d = 0;
		for (i = 0; i < count; i++) {
			cbuf[bytes++] = addr[i];
			cbuf[bytes++] = addr[i] >> 8;
			if (id != IPC_CMD_PCNTRL_R)
				cbuf[bytes++] = data[d++];
			if (id == IPC_CMD_PCNTRL_M)
				cbuf[bytes++] = data[d++];
		}
		for (i = 0; i < bytes; i += 4)
			ipc_data_writel(wbuf[i/4], i);
		ipc_command(bytes << 16 |  id << 12 | 0 << 8 | op);
	} else {
		for (nc = 0; nc < count; nc++, offset += 2) {
			cbuf[offset] = addr[nc];
			cbuf[offset + 1] = addr[nc] >> 8;
		}

		if (id == IPC_CMD_PCNTRL_R) {
			for (nc = 0, offset = 0; nc < count; nc++, offset += 4)
				ipc_data_writel(wbuf[nc], offset);
			ipc_command((count*2) << 16 |  id << 12 | 0 << 8 | op);
		} else if (id == IPC_CMD_PCNTRL_W) {
			for (nc = 0; nc < count; nc++, offset += 1)
				cbuf[offset] = data[nc];
			for (nc = 0, offset = 0; nc < count; nc++, offset += 4)
				ipc_data_writel(wbuf[nc], offset);
			ipc_command((count*3) << 16 |  id << 12 | 0 << 8 | op);
		} else if (id == IPC_CMD_PCNTRL_M) {
			cbuf[offset] = data[0];
			cbuf[offset + 1] = data[1];
			ipc_data_writel(wbuf[0], 0); /* Write wbuff */
			ipc_command(4 << 16 |  id << 12 | 0 << 8 | op);
		}
	}

	err = busy_loop();
	if (id == IPC_CMD_PCNTRL_R) { /* Read rbuf */
		/* Workaround: values are read as 0 without memcpy_fromio */
		memcpy_fromio(cbuf, ipcdev.ipc_base + 0x90, 16);
		if (platform != MRST_CPU_CHIP_PENWELL) {
			for (nc = 0, offset = 2; nc < count; nc++, offset += 3)
				data[nc] = ipc_data_readb(offset);
		} else {
			for (nc = 0; nc < count; nc++)
				data[nc] = ipc_data_readb(nc);
		}
	}
	mutex_unlock(&ipclock);
	return err;
}

/**
 *	intel_scu_ipc_ioread8		-	read a word via the SCU
 *	@addr: register on SCU
 *	@data: return pointer for read byte
 *
 *	Read a single register. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_ioread8(u16 addr, u8 *data)
{
	return pwr_reg_rdwr(&addr, data, 1, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_ioread8);

/**
 *	intel_scu_ipc_ioread16		-	read a word via the SCU
 *	@addr: register on SCU
 *	@data: return pointer for read word
 *
 *	Read a register pair. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_ioread16(u16 addr, u16 *data)
{
	u16 x[2] = {addr, addr + 1 };
	return pwr_reg_rdwr(x, (u8 *)data, 2, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_ioread16);

/**
 *	intel_scu_ipc_ioread32		-	read a dword via the SCU
 *	@addr: register on SCU
 *	@data: return pointer for read dword
 *
 *	Read four registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_ioread32(u16 addr, u32 *data)
{
	u16 x[4] = {addr, addr + 1, addr + 2, addr + 3};
	return pwr_reg_rdwr(x, (u8 *)data, 4, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_ioread32);

/**
 *	intel_scu_ipc_iowrite8		-	write a byte via the SCU
 *	@addr: register on SCU
 *	@data: byte to write
 *
 *	Write a single register. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_iowrite8(u16 addr, u8 data)
{
	return pwr_reg_rdwr(&addr, &data, 1, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite8);

/**
 *	intel_scu_ipc_iowrite16		-	write a word via the SCU
 *	@addr: register on SCU
 *	@data: word to write
 *
 *	Write two registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_iowrite16(u16 addr, u16 data)
{
	u16 x[2] = {addr, addr + 1 };
	return pwr_reg_rdwr(x, (u8 *)&data, 2, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite16);

/**
 *	intel_scu_ipc_iowrite32		-	write a dword via the SCU
 *	@addr: register on SCU
 *	@data: dword to write
 *
 *	Write four registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_iowrite32(u16 addr, u32 data)
{
	u16 x[4] = {addr, addr + 1, addr + 2, addr + 3};
	return pwr_reg_rdwr(x, (u8 *)&data, 4, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite32);

/**
 *	intel_scu_ipc_readvv		-	read a set of registers
 *	@addr: register list
 *	@data: bytes to return
 *	@len: length of array
 *
 *	Read registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	The largest array length permitted by the hardware is 5 items.
 *
 *	This function may sleep.
 */
int intel_scu_ipc_readv(u16 *addr, u8 *data, int len)
{
	return pwr_reg_rdwr(addr, data, len, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_R);
}
EXPORT_SYMBOL(intel_scu_ipc_readv);

/**
 *	intel_scu_ipc_writev		-	write a set of registers
 *	@addr: register list
 *	@data: bytes to write
 *	@len: length of array
 *
 *	Write registers. Returns 0 on success or an error code. All
 *	locking between SCU accesses is handled for the caller.
 *
 *	The largest array length permitted by the hardware is 5 items.
 *
 *	This function may sleep.
 *
 */
int intel_scu_ipc_writev(u16 *addr, u8 *data, int len)
{
	return pwr_reg_rdwr(addr, data, len, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_W);
}
EXPORT_SYMBOL(intel_scu_ipc_writev);


/**
 *	intel_scu_ipc_update_register	-	r/m/w a register
 *	@addr: register address
 *	@bits: bits to update
 *	@mask: mask of bits to update
 *
 *	Read-modify-write power control unit register. The first data argument
 *	must be register value and second is mask value
 *	mask is a bitmap that indicates which bits to update.
 *	0 = masked. Don't modify this bit, 1 = modify this bit.
 *	returns 0 on success or an error code.
 *
 *	This function may sleep. Locking between SCU accesses is handled
 *	for the caller.
 */
int intel_scu_ipc_update_register(u16 addr, u8 bits, u8 mask)
{
	u8 data[2] = { bits, mask };
	return pwr_reg_rdwr(&addr, data, 1, IPCMSG_PCNTRL, IPC_CMD_PCNTRL_M);
}
EXPORT_SYMBOL(intel_scu_ipc_update_register);

/**
 *	intel_scu_ipc_simple_command	-	send a simple command
 *	@cmd: command
 *	@sub: sub type
 *
 *	Issue a simple command to the SCU. Do not use this interface if
 *	you must then access data as any data values may be overwritten
 *	by another SCU access by the time this function returns.
 *
 *	This function may sleep. Locking for SCU accesses is handled for
 *	the caller.
 */
int intel_scu_ipc_simple_command(int cmd, int sub)
{
	int err;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}
	ipc_command(sub << 12 | cmd);
	err = busy_loop();
	mutex_unlock(&ipclock);
	return err;
}
EXPORT_SYMBOL(intel_scu_ipc_simple_command);

/**
 *	intel_scu_ipc_command	-	command with data
 *	@cmd: command
 *	@sub: sub type
 *	@in: input data
 *	@inlen: input length in dwords
 *	@out: output data
 *	@outlein: output length in dwords
 *
 *	Issue a command to the SCU which involves data transfers. Do the
 *	data copies under the lock but leave it for the caller to interpret
 */

int intel_scu_ipc_command(int cmd, int sub, u32 *in, int inlen,
							u32 *out, int outlen)
{
	int i, err;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}

	for (i = 0; i < inlen; i++)
		ipc_data_writel(*in++, 4 * i);

	ipc_command((inlen << 16) | (sub << 12) | cmd);
	err = busy_loop();

	for (i = 0; i < outlen; i++)
		*out++ = ipc_data_readl(4 * i);

	mutex_unlock(&ipclock);
	return err;
}
EXPORT_SYMBOL(intel_scu_ipc_command);

/*I2C commands */
#define IPC_I2C_WRITE 1 /* I2C Write command */
#define IPC_I2C_READ  2 /* I2C Read command */

/**
 *	intel_scu_ipc_i2c_cntrl		-	I2C read/write operations
 *	@addr: I2C address + command bits
 *	@data: data to read/write
 *
 *	Perform an an I2C read/write operation via the SCU. All locking is
 *	handled for the caller. This function may sleep.
 *
 *	Returns an error code or 0 on success.
 *
 *	This has to be in the IPC driver for the locking.
 */
int intel_scu_ipc_i2c_cntrl(u32 addr, u32 *data)
{
	u32 cmd = 0;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}
	cmd = (addr >> 24) & 0xFF;
	if (cmd == IPC_I2C_READ) {
		writel(addr, ipcdev.i2c_base + IPC_I2C_CNTRL_ADDR);
		/* Write not getting updated without delay */
		mdelay(1);
		*data = readl(ipcdev.i2c_base + I2C_DATA_ADDR);
	} else if (cmd == IPC_I2C_WRITE) {
		writel(*data, ipcdev.i2c_base + I2C_DATA_ADDR);
		mdelay(1);
		writel(addr, ipcdev.i2c_base + IPC_I2C_CNTRL_ADDR);
	} else {
		dev_err(&ipcdev.pdev->dev,
			"intel_scu_ipc: I2C INVALID_CMD = 0x%x\n", cmd);

		mutex_unlock(&ipclock);
		return -EIO;
	}
	mutex_unlock(&ipclock);
	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_i2c_cntrl);

#define IPC_FW_LOAD_ADDR 0xFFFC0000 /* Storage location for FW image */
#define IPC_FW_UPDATE_MBOX_ADDR 0xFFFFDFF4 /* Mailbox between ipc and scu */
#define IPC_MAX_FW_SIZE 262144 /* 256K storage size for loading the FW image */
#define IPC_FW_MIP_HEADER_SIZE 2048 /* Firmware MIP header size */
/* IPC inform SCU to get ready for update process */
#define IPC_CMD_FW_UPDATE_READY  0x10FE
/* IPC inform SCU to go for update process */
#define IPC_CMD_FW_UPDATE_GO     0x20FE
#define IPC_CMD_FW_UPDATE_GO_MHUPD     0x2030FE
/* Status code for fw update */
#define IPC_FW_UPDATE_SUCCESS	0x444f4e45 /* Status code 'DONE' */
#define IPC_FW_UPDATE_BADN	0x4241444E /* Status code 'BADN' */
#define IPC_FW_TXHIGH		0x54784849 /* Status code 'IPC_FW_TXHIGH' */
#define IPC_FW_TXLOW		0x54784c4f /* Status code 'IPC_FW_TXLOW' */

struct fw_update_mailbox {
	u32    status;
	u32    scu_flag;
	u32    driver_flag;
};


/**
 *	intel_scu_ipc_fw_update	-	 Firmware update utility
 *	@buffer: firmware buffer
 *	@length: size of firmware buffer
 *
 *	This function provides an interface to load the firmware into
 *	the SCU. Returns 0 on success or -1 on failure
 */
int intel_scu_ipc_fw_update(u8 *buffer, u32 length)
{
	void __iomem *fw_update_base;
	struct fw_update_mailbox __iomem *mailbox = NULL;
	int retry_cnt = 0;
	u32 status;

	if (platform != MRST_CPU_CHIP_LINCROFT)
		return -EINVAL;

	mutex_lock(&ipclock);
	fw_update_base = ioremap_nocache(IPC_FW_LOAD_ADDR, (128*1024));
	if (fw_update_base == NULL) {
		mutex_unlock(&ipclock);
		return -ENOMEM;
	}
	mailbox = ioremap_nocache(IPC_FW_UPDATE_MBOX_ADDR,
					sizeof(struct fw_update_mailbox));
	if (mailbox == NULL) {
		iounmap(fw_update_base);
		mutex_unlock(&ipclock);
		return -ENOMEM;
	}

	ipc_command(IPC_CMD_FW_UPDATE_READY);

	/* Intitialize mailbox */
	writel(0, &mailbox->status);
	writel(0, &mailbox->scu_flag);
	writel(0, &mailbox->driver_flag);

	/* Driver copies the 2KB MIP header to SRAM at 0xFFFC0000*/
	memcpy_toio(fw_update_base, buffer, 0x800);

	/* Driver sends "FW Update" IPC command (CMD_ID 0xFE; MSG_ID 0x02).
	* Upon receiving this command, SCU will write the 2K MIP header
	* from 0xFFFC0000 into NAND.
	* SCU will write a status code into the Mailbox, and then set scu_flag.
	*/

	ipc_command(IPC_CMD_FW_UPDATE_GO);

	/*Driver stalls until scu_flag is set */
	while (readl(&mailbox->scu_flag) != 1) {
		rmb();
		mdelay(1);
	}

	/* Driver checks Mailbox status.
	 * If the status is 'BADN', then abort (bad NAND).
	 * If the status is 'IPC_FW_TXLOW', then continue.
	 */
	while (readl(&mailbox->status) != IPC_FW_TXLOW) {
		rmb();
		mdelay(10);
	}
	mdelay(10);

update_retry:
	if (retry_cnt > 5)
		goto update_end;

	if (readl(&mailbox->status) != IPC_FW_TXLOW)
		goto update_end;
	buffer = buffer + 0x800;
	memcpy_toio(fw_update_base, buffer, 0x20000);
	writel(1, &mailbox->driver_flag);
	while (readl(&mailbox->scu_flag) == 1) {
		rmb();
		mdelay(1);
	}

	/* check for 'BADN' */
	if (readl(&mailbox->status) == IPC_FW_UPDATE_BADN)
		goto update_end;

	while (readl(&mailbox->status) != IPC_FW_TXHIGH) {
		rmb();
		mdelay(10);
	}
	mdelay(10);

	if (readl(&mailbox->status) != IPC_FW_TXHIGH)
		goto update_end;

	buffer = buffer + 0x20000;
	memcpy_toio(fw_update_base, buffer, 0x20000);
	writel(0, &mailbox->driver_flag);

	while (mailbox->scu_flag == 0) {
		rmb();
		mdelay(1);
	}

	/* check for 'BADN' */
	if (readl(&mailbox->status) == IPC_FW_UPDATE_BADN)
		goto update_end;

	if (readl(&mailbox->status) == IPC_FW_TXLOW) {
		++retry_cnt;
		goto update_retry;
	}

update_end:
	status = readl(&mailbox->status);

	iounmap(fw_update_base);
	iounmap(mailbox);
	mutex_unlock(&ipclock);

	if (status == IPC_FW_UPDATE_SUCCESS)
		return 0;
	return -EIO;
}
EXPORT_SYMBOL(intel_scu_ipc_fw_update);

/* Medfield firmware update.
 * The flow and communication between IA and SCU has changed for
 * Medfield firmware update. For more details, please refer to
 * Firmware Arch Spec.
 * Below macros and structs apply for medfield firmware update
 */

#define MAX_FW_CHUNK (128*1024)
#define SRAM_ADDR 0xFFFC0000
#define MAILBOX_ADDR   0xFFFE0000

#define SCU_FLAG_OFFSET 8
#define IA_FLAG_OFFSET 12
#define MIP_HEADER_LEN 2048 /* For A0, will change for B0,also bug in FUPH
			     * header we should just be able to read it off
			     * correct value from FUPH..ideally.
			     */
#define MIP_HEADER_OFFSET 0
#define LOWER_128K_OFFSET (MIP_HEADER_OFFSET+MIP_HEADER_LEN)
#define UPPER_128K_OFFSET (LOWER_128K_OFFSET+MAX_FW_CHUNK)

#define DNX_HDR_LEN  24
#define FUPH_HDR_LEN 28

#define DNX_IMAGE        "DXBL"
#define FUPH_HDR_SIZE    "RUPHS"
#define FUPH		 "RUPH"
#define MIP              "DMIP"
#define LOWER_128K       "LOFW"
#define UPPER_128K       "HIFW"
#define UPDATE_DONE      "DONE"

/* Modified IA-SCU mailbox for medfield firmware update. */
struct ia_scu_mailbox {
	char mail[8];
	u32 scu_flag;
	u32 ia_flag;
};

/* Structure to parse input from firmware-update application. */
struct fw_ud {
	u8 *fw_file_data;
	u32 fsize;
	u8 *dnx_hdr;
	u8 *dnx_file_data;
	u32 dnx_size;
};

struct mfld_fw_update {
	void __iomem *sram;
	void __iomem *mailbox;
	u32 wscu;
	u32 wia;
	char mb_status[8];
};

/*
 * IA will wait in busy-state, and poll mailbox, to check
 * if SCU is done processing.
 * If it has to wait for more than a second, it will exit with
 * error code.
 */

static int busy_wait(struct mfld_fw_update *mfld_fw_upd)
{
	u32 count = 0;
	u32 flag;

	flag = mfld_fw_upd->wscu;

	while (ioread32(mfld_fw_upd->mailbox + SCU_FLAG_OFFSET) != flag
		&& count < 120) {
		/* There are synchronization issues between IA and SCU */
		mb();
		/* FIXME: we must use mdelay currently */
		mdelay(10);
		count++;
	}

	if (ioread32(mfld_fw_upd->mailbox + SCU_FLAG_OFFSET) != flag) {
		dev_err(&ipcdev.pdev->dev, "IA-wait >2 secs,quitting\n");
		return -ETIMEDOUT;
	}

	return 0;
}

/* This function will
 * 1)copy firmware chunk from user-space to kernel-space.
 * 2) Copy from kernel-space to shared SRAM.
 * 3) Write to mailbox.
 * 4) And wait for SCU to process that firmware chunk.
 * Returns 0 on success, and < 0 for failure.
 */
static int process_fw_chunk(u8 *fws, u8 __user *userptr, u32 chunklen,
					struct mfld_fw_update *mfld_fw_upd)
{
	if (copy_from_user(fws, userptr, chunklen))
		return -EFAULT;

	/* IA copy to sram */
	memcpy_toio(mfld_fw_upd->sram, fws, chunklen);

	/* There are synchronization issues between IA and SCU */
	mb();
	mfld_fw_upd->wia = !(mfld_fw_upd->wia);
	iowrite32(mfld_fw_upd->wia, mfld_fw_upd->mailbox + IA_FLAG_OFFSET);

	mb();
	dev_dbg(&ipcdev.pdev->dev, "wrote ia_flag=%d\n",
		 ioread32(mfld_fw_upd->mailbox + IA_FLAG_OFFSET));

	mfld_fw_upd->wscu = !mfld_fw_upd->wscu;
	return busy_wait(mfld_fw_upd);
}

/*
 * This function will check if mailbox status flag, indicates any errors.
 */
static int check_mb_status(char *mb_status, struct mfld_fw_update *mfld_fw_upd)
{

	/* There are synchronization issues between IA and SCU */
	smp_mb();

	memcpy_fromio(mfld_fw_upd->mb_status, mfld_fw_upd->mailbox, 8);

	if ((strncmp(mfld_fw_upd->mb_status, "ER", 2) == 0) ||
		(strncmp(mfld_fw_upd->mb_status, "HLT0", 4) == 0)) {
		dev_dbg(&ipcdev.pdev->dev,
			"mailbox error=%s\n", mfld_fw_upd->mb_status);
		return -EINVAL;
	} else {
		if (strncmp(mfld_fw_upd->mb_status, mb_status,
				strlen(mfld_fw_upd->mb_status)) != 0) {
			dev_dbg(&ipcdev.pdev->dev,
				"unexpected state=%s", mfld_fw_upd->mb_status);
			return -EINVAL;
		} else {
			dev_dbg(&ipcdev.pdev->dev,
				"mailbox pass=%s\n", mfld_fw_upd->mb_status);
		}
	}

	return 0;
}


/**
 *	intel_scu_ipc_medfw_upgrade	- Medfield Firmware update utility
 *	@arg: firmware buffer in user-space.
 *
 * The flow and communication between IA and SCU has changed for
 * Medfield firmware update. So we have a different api below
 * to support Medfield firmware update.
 *
 * On success returns 0, for failure , returns < 0.
 */
int intel_scu_ipc_medfw_upgrade(void __user *arg)
{
	struct fw_ud fw_ud_param;
	struct mfld_fw_update	mfld_fw_upd;
	u8 *fw_file_data = NULL;
	u8 *fws = NULL;
	u8 *fuph_ptr = NULL;
	int ret_val = 0;

	if (platform != MRST_CPU_CHIP_PENWELL)
		return -EINVAL;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		ret_val = -ENODEV;
		goto out_unlock;
	}

	mfld_fw_upd.wscu = 0;
	mfld_fw_upd.wia = 0;
	memset(mfld_fw_upd.mb_status, 0, sizeof(char) * 8);

	if (copy_from_user(&fw_ud_param, arg, sizeof(fw_ud_param)))
		return -EFAULT;

	fw_file_data = fw_ud_param.fw_file_data;
	if (fw_file_data == NULL || fw_ud_param.fsize == 0 ||
	    fw_ud_param.dnx_size == 0 || fw_ud_param.dnx_hdr == NULL ||
	    fw_ud_param.dnx_file_data == NULL) {
		dev_err(&ipcdev.pdev->dev,
			"ipc_medfw_upgrade, invalid args!!\n");
		ret_val = -EINVAL;
		goto out_unlock;
	}

	mfld_fw_upd.sram = ioremap_nocache(SRAM_ADDR, MAX_FW_CHUNK);
	if (mfld_fw_upd.sram == NULL) {
		dev_err(&ipcdev.pdev->dev, "unable to map sram\n");
		ret_val = -ENOMEM;
		goto out_unlock;
	}

	mfld_fw_upd.mailbox = ioremap_nocache(MAILBOX_ADDR,
					sizeof(struct ia_scu_mailbox));

	if (mfld_fw_upd.mailbox == NULL) {
		dev_err(&ipcdev.pdev->dev, "unable to map the mailbox\n");
		ret_val = -ENOMEM;
		goto unmap_sram;
	}

	/*IA initializes both IAFlag and SCUFlag to zero */
	iowrite32(0, mfld_fw_upd.mailbox + SCU_FLAG_OFFSET);
	iowrite32(0, mfld_fw_upd.mailbox + IA_FLAG_OFFSET);

	fws = kmalloc(MAX_FW_CHUNK, GFP_KERNEL);
	if (fws == NULL) {
		ret_val = -ENOMEM;
		goto unmap_mb;
	}

	/* TODO_SK::There is just
	 *  1 write required from IA side for DFU.
	 *  So commenting this-out, until it gets confirmed */
	/*ipc_command(IPC_CMD_FW_UPDATE_READY); */

	/*1. DNX SIZE HEADER   */
	ret_val = copy_from_user(fws, fw_ud_param.dnx_hdr, DNX_HDR_LEN);
	if (ret_val < 0) {
		ret_val = -EFAULT;
		goto term;
	}

	memcpy_toio(mfld_fw_upd.sram, fws, DNX_HDR_LEN);

	/* There are synchronization issues between IA and SCU */
	mb();

	/* Write cmd to trigger an interrupt to SCU for firmware update*/
	ipc_command(IPC_CMD_FW_UPDATE_GO);

	mfld_fw_upd.wscu = !mfld_fw_upd.wscu;

	ret_val = busy_wait(&mfld_fw_upd);
	if (ret_val < 0)
		goto term;

	ret_val = check_mb_status(DNX_IMAGE, &mfld_fw_upd);
	if (ret_val < 0)
		goto term;

	/*2. DNX FILE   */
	ret_val = process_fw_chunk(fws, fw_ud_param.dnx_file_data,
				      fw_ud_param.dnx_size, &mfld_fw_upd);
	if (ret_val < 0) {
		dev_err(&ipcdev.pdev->dev, "Error fw chunk=%s\n", DNX_IMAGE);
		goto term;
	}

	ret_val = check_mb_status(FUPH_HDR_SIZE, &mfld_fw_upd);
	if (ret_val < 0)
		goto term;

	iowrite32(FUPH_HDR_LEN, mfld_fw_upd.sram);

	/* There are synchronization issues between IA and SCU */
	mb();
	dev_dbg(&ipcdev.pdev->dev,
		"copied fuph hdr size=%d\n", ioread32(mfld_fw_upd.sram));

	mfld_fw_upd.wia = !(mfld_fw_upd.wia);
	iowrite32(mfld_fw_upd.wia, mfld_fw_upd.mailbox + IA_FLAG_OFFSET);

	dev_dbg(&ipcdev.pdev->dev, "ia_flag=%d\n",
		ioread32(mfld_fw_upd.mailbox + IA_FLAG_OFFSET));

	mb();

	mfld_fw_upd.wscu = !mfld_fw_upd.wscu;
	
	ret_val = busy_wait(&mfld_fw_upd);
	if (ret_val < 0)
		goto term;

	ret_val = check_mb_status(FUPH, &mfld_fw_upd);
	if (ret_val < 0) {
		dev_err(&ipcdev.pdev->dev,
			"error after FUPH hdrsize, IA quitting\n");
		goto term;
	}

	/* 3. FUPH HEADER   */
	fuph_ptr = fw_file_data + (fw_ud_param.fsize - 1) - (FUPH_HDR_LEN - 1);
	
	ret_val = process_fw_chunk(fws, fuph_ptr, FUPH_HDR_LEN, &mfld_fw_upd);
	if (ret_val < 0) {
		dev_err(&ipcdev.pdev->dev, "Error fw chunk=%s\n", FUPH);
		goto term;
	}

	/* TODO: Fix this function, to make it handle more than 1 string
	 * At this point, if MIP size is 0 in FUPH, then SCU can jump to LOFW.
	 */
	ret_val = check_mb_status(MIP, &mfld_fw_upd);
	if (ret_val < 0)
		goto term;

	ret_val = check_mb_status(LOWER_128K, &mfld_fw_upd);
	if (ret_val < 0)
		goto term;

	if ((strncmp(mfld_fw_upd.mb_status, MIP,
					strlen(mfld_fw_upd.mb_status)) == 0)) {
		/* 4. MIP HEADER */
		ret_val = process_fw_chunk(fws, 
				fw_file_data + MIP_HEADER_OFFSET,
				MIP_HEADER_LEN, &mfld_fw_upd);
		if (ret_val < 0) {
			dev_err(&ipcdev.pdev->dev, "Error fw chunk=%s\n", MIP);
			goto term;
		}

		ret_val = check_mb_status(LOWER_128K, &mfld_fw_upd);
		if (ret_val < 0)
			goto term;
	}

	/* 5. LOWER 128K */
	ret_val = process_fw_chunk(fws, fw_file_data + LOWER_128K_OFFSET,
			      MAX_FW_CHUNK, &mfld_fw_upd);
	if (ret_val < 0) {
		dev_err(&ipcdev.pdev->dev, "Error fw chunk=%s\n", LOWER_128K);
		goto term;
	}

	ret_val = check_mb_status(UPPER_128K, &mfld_fw_upd);
	if (ret_val < 0)
		goto term;

	/* 6. UPPER 128K */
	ret_val = process_fw_chunk(fws, fw_file_data + UPPER_128K_OFFSET,
					      MAX_FW_CHUNK, &mfld_fw_upd);
	if (ret_val < 0) {
		dev_err(&ipcdev.pdev->dev, "Error fw chunk=%s\n", UPPER_128K);
		goto term;
	}
	ret_val = check_mb_status(UPDATE_DONE, &mfld_fw_upd);
term:
	kfree(fws);
unmap_mb:
	iounmap(mfld_fw_upd.mailbox);
unmap_sram:
	iounmap(mfld_fw_upd.sram);
out_unlock:
	mutex_unlock(&ipclock);
	return ret_val;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_medfw_upgrade);

/*
 * Interrupt handler gets called when ioc bit of IPC_COMMAND_REG set to 1
 * When ioc bit is set to 1, caller api must wait for interrupt handler called
 * which in turn unlocks the caller api. Currently this is not used
 *
 * This is edge triggered so we need take no action to clear anything
 */
static irqreturn_t ioc(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

/**
 *	ipc_probe	-	probe an Intel SCU IPC
 *	@dev: the PCI device matching
 *	@id: entry in the match table
 *
 *	Enable and install an intel SCU IPC. This appears in the PCI space
 *	but uses some hard coded addresses as well.
 */
static int ipc_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int err;
	resource_size_t pci_resource;

	if (ipcdev.pdev)		/* We support only one SCU */
		return -EBUSY;

	ipcdev.pdev = pci_dev_get(dev);

	err = pci_enable_device(dev);
	if (err)
		return err;

	err = pci_request_regions(dev, "intel_scu_ipc");
	if (err)
		return err;

	pci_resource = pci_resource_start(dev, 0);
	if (!pci_resource)
		return -ENOMEM;

	if (request_irq(dev->irq, ioc, 0, "intel_scu_ipc", &ipcdev))
		return -EBUSY;

	ipcdev.ipc_base = ioremap_nocache(IPC_BASE_ADDR, IPC_MAX_ADDR);
	if (!ipcdev.ipc_base)
		return -ENOMEM;

	ipcdev.i2c_base = ioremap_nocache(IPC_I2C_BASE, IPC_I2C_MAX_ADDR);
	if (!ipcdev.i2c_base) {
		iounmap(ipcdev.ipc_base);
		return -ENOMEM;
	}

	intel_scu_devices_create();

	return 0;
}

/**
 *	ipc_remove	-	remove a bound IPC device
 *	@pdev: PCI device
 *
 *	In practice the SCU is not removable but this function is also
 *	called for each device on a module unload or cleanup which is the
 *	path that will get used.
 *
 *	Free up the mappings and release the PCI resources
 */
static void ipc_remove(struct pci_dev *pdev)
{
	free_irq(pdev->irq, &ipcdev);
	pci_release_regions(pdev);
	pci_dev_put(ipcdev.pdev);
	iounmap(ipcdev.ipc_base);
	iounmap(ipcdev.i2c_base);
	ipcdev.pdev = NULL;
	intel_scu_devices_destroy();
}

static const struct pci_device_id pci_ids[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x080e)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x082a)},
	{ 0,}
};
MODULE_DEVICE_TABLE(pci, pci_ids);

static struct pci_driver ipc_driver = {
	.name = "intel_scu_ipc",
	.id_table = pci_ids,
	.probe = ipc_probe,
	.remove = ipc_remove,
};


static int __init intel_scu_ipc_init(void)
{
	platform = mrst_identify_cpu();
	if (platform == 0)
		return -ENODEV;
	return  pci_register_driver(&ipc_driver);
}

static void __exit intel_scu_ipc_exit(void)
{
	pci_unregister_driver(&ipc_driver);
}

MODULE_AUTHOR("Sreedhara DS <sreedhara.ds@intel.com>");
MODULE_DESCRIPTION("Intel SCU IPC driver");
MODULE_LICENSE("GPL");

module_init(intel_scu_ipc_init);
module_exit(intel_scu_ipc_exit);
