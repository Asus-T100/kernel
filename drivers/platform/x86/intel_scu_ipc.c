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
#include <asm/mrst.h>
#include <asm/intel_scu_ipc.h>
#include <linux/pm_qos_params.h>

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

#define IPC_SPTR_ADDR     0x08          /* IPC source pointer regiser*/
#define IPC_DPTR_ADDR     0x0c          /* IPC destination pointer regiser*/
#define IPC_MIP_BASE	   0xFFFD8000   /* sram base address for mip accessing*/
#define IPC_MIP_MAX_ADDR  0x1000

#define IPC_IOC		  0x100

static int ipc_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void ipc_remove(struct pci_dev *pdev);

struct intel_scu_ipc_dev {
	struct pci_dev *pdev;
	void __iomem *ipc_base;
	void __iomem *i2c_base;
	void __iomem *mip_base;
	int ioc;
	struct completion cmd_complete;
	int status;
	struct fw_ud *fwud_pending;
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

/* PM Qos struct */
static struct pm_qos_request_list *qos;

/*
 * Command Register (Write Only):
 * A write to this register results in an interrupt to the SCU core processor
 * Format:
 * |rfu2(8) | size(8) | command id(4) | rfu1(3) | ioc(1) | command(8)|
 */
static inline void ipc_command(u32 cmd) /* Send ipc command */
{
	INIT_COMPLETION(ipcdev.cmd_complete);
	if (system_state == SYSTEM_RUNNING) {
		ipcdev.ioc = 1;
		writel(cmd | IPC_IOC, ipcdev.ipc_base);
	} else {
		ipcdev.ioc = 0;
		writel(cmd, ipcdev.ipc_base);
	}

	/* Prevent C-states beyond C6 */
	pm_qos_update_request(qos, 150);
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

static inline int ipc_wait_interrupt(void)
{
	int ret = 0;
	if (ipcdev.ioc) {
		if (0 == wait_for_completion_timeout(
				&ipcdev.cmd_complete, 3 * HZ)) {
			dev_err(&ipcdev.pdev->dev, "IPC timed out, IPC_STS=0x%x",
					ipc_read_status());
			ret = -ETIMEDOUT;
			goto leave;
		} else {
			if ((ipcdev.status >> 1) & 1) {
				dev_err(&ipcdev.pdev->dev, "IPC failed, IPC_STS=0x%x",
						ipc_read_status());
				ret = -EIO;
				goto leave;
			}
		}
	} else {
		u32 status = 0;
		u32 loop_count = 0;

		status = ipc_read_status();
		while (status & 1) {
			udelay(1);
			status = ipc_read_status();
			loop_count++;
			if (loop_count > 3000000) {
				dev_err(&ipcdev.pdev->dev, "IPC timed out");
				ret = -ETIMEDOUT;
				goto leave;
			}
		}
		if ((status >> 1) & 1)
			ret = -EIO;
	}
leave:
	/* Re-enable Deeper C-states beyond C6 */
	pm_qos_update_request(qos, PM_QOS_DEFAULT_VALUE);
	return ret;
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

	err = ipc_wait_interrupt();
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
	err = ipc_wait_interrupt();
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
	err = ipc_wait_interrupt();

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
int intel_scu_ipc_mrstfw_update(u8 *buffer, u32 length)
{
	void __iomem *fw_update_base;
	struct fw_update_mailbox __iomem *mailbox = NULL;
	int retry_cnt = 0;
	u32 status;

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

	ipc_command(IPC_CMD_FW_UPDATE_GO_MHUPD);

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
EXPORT_SYMBOL(intel_scu_ipc_mrstfw_update);

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
#define SUCP_OFFSET	0x1D8000

#define DNX_HDR_LEN  24
#define FUPH_HDR_LEN 32

#define DNX_IMAGE        "DXBL"
#define FUPH_HDR_SIZE    "RUPHS"
#define FUPH		 "RUPH"
#define MIP              "DMIP"
#define LOWER_128K       "LOFW"
#define UPPER_128K       "HIFW"
#define UPDATE_DONE      "HLT$"
#define PSFW1		 "PSFW1"
#define PSFW2		 "PSFW2"
#define SSFW		 "SSFW"
#define SUCP		 "SuCP"

#define MAX_LEN_PSFW     7
#define MAX_LEN_SSFW     6
#define MAX_LEN_SUCP     6

#define C0_STEPPING	8 /* PCI Rev for C0 stepping */

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

/* Structure to hold firmware update header */
struct fuph_hdr {
	u32 sig;
	u32 mip_size;
	u32 ifwi_size;
	u32 psfw1_size;
	u32 psfw2_size;
	u32 ssfw_size;
	u32 sucp_size;
	u32 checksum;
};

enum mailbox_status {
	MB_DONE,
	MB_CONTINUE,
	MB_ERROR
};

/* Misc. firmware components that are part of integrated firmware */
struct misc_fw {
	const char *fw_type;
	u8 str_len;
};

static struct misc_fw misc_fw_table[] = {
	{ .fw_type = "PSFW1", .str_len  = MAX_LEN_PSFW },
	{ .fw_type = "SSFW", .str_len  = MAX_LEN_SSFW  },
	{ .fw_type = "PSFW2", .str_len  = MAX_LEN_PSFW  },
	{ .fw_type = "SuCP", .str_len  = MAX_LEN_SUCP  }
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
		&& count < 500) {
		/* There are synchronization issues between IA and SCU */
		mb();
		/* FIXME: we must use mdelay currently */
		mdelay(10);
		count++;
	}

	if (ioread32(mfld_fw_upd->mailbox + SCU_FLAG_OFFSET) != flag) {
		dev_err(&ipcdev.pdev->dev, "IA-waited and quitting\n");
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
static int process_fw_chunk(u8 *fws, u8 *userptr, u32 chunklen,
					struct mfld_fw_update *mfld_fw_upd)
{
	memcpy(fws, userptr, chunklen);

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
 * This function will check mailbox status flag, and return state of mailbox.
 */
static enum mailbox_status check_mb_status(struct mfld_fw_update *mfld_fw_upd)
{

	enum mailbox_status mb_state;

	/* There are synchronization issues between IA and SCU */
	mb();

	memcpy_fromio(mfld_fw_upd->mb_status, mfld_fw_upd->mailbox, 8);

	if (!strncmp(mfld_fw_upd->mb_status, "ER", 2) ||
		!strncmp(mfld_fw_upd->mb_status, "HLT0", 4)) {
		dev_dbg(&ipcdev.pdev->dev,
			"mailbox error=%s\n", mfld_fw_upd->mb_status);
		return MB_ERROR;
	} else {
		mb_state = (!strncmp(mfld_fw_upd->mb_status, UPDATE_DONE,
				sizeof(UPDATE_DONE))) ? MB_DONE : MB_CONTINUE;
		dev_dbg(&ipcdev.pdev->dev,
			"mailbox pass=%s, mb_state=%d\n",
			mfld_fw_upd->mb_status, mb_state);
	}

	return mb_state;

}

/* Helper function used to calculate length and offset.  */
int helper_for_calc_offset_length(struct fw_ud *fw_ud_ptr, char *scu_req,
				void **offset, u32 *len, struct fuph_hdr *fuph,
				const char *fw_type)
{

	unsigned long chunk_no;
	u32 chunk_rem;
	u32 max_chunk_cnt;
	u32 fw_size;
	u32 fw_offset;

	if (!strncmp(fw_type, PSFW1, strlen(PSFW1))) {

		if (strict_strtoul(scu_req + strlen(PSFW1), 10,
						&chunk_no) < 0)
			return -EINVAL;

		fw_size = fuph->psfw1_size;
		fw_offset = fuph->mip_size + fuph->ifwi_size;
	} else if (!strncmp(fw_type, PSFW2, strlen(PSFW2))) {

		if (strict_strtoul(scu_req + strlen(PSFW2), 10,
						&chunk_no) < 0)
			return -EINVAL;

		fw_size = fuph->psfw2_size;
		fw_offset = fuph->mip_size + fuph->ifwi_size +
				fuph->psfw1_size + fuph->ssfw_size;
	} else if (!strncmp(fw_type, SSFW, strlen(SSFW))) {

		if (strict_strtoul(scu_req + strlen(SSFW), 10,
						&chunk_no) < 0)
			return -EINVAL;

		fw_size = fuph->ssfw_size;
		fw_offset = fuph->mip_size + fuph->ifwi_size +
				fuph->psfw1_size;
	} else if (!strncmp(fw_type, SUCP, strlen(SUCP))) {

		if (strict_strtoul(scu_req + strlen(SUCP), 10,
						&chunk_no) < 0)
			return -EINVAL;

		fw_size = fuph->sucp_size;
		fw_offset = SUCP_OFFSET;
	} else
		return -EINVAL;

	chunk_rem = fw_size % MAX_FW_CHUNK;
	max_chunk_cnt = (fw_size/MAX_FW_CHUNK) + (chunk_rem ? 1 : 0);

	dev_dbg(&ipcdev.pdev->dev,
		"str=%s,chunk_no=%lx, chunk_rem=%d,max_chunk_cnt=%d\n",
		fw_type, chunk_no, chunk_rem, max_chunk_cnt);

	if ((chunk_no + 1) > max_chunk_cnt)
		return -EINVAL;

	/* Note::Logic below will make sure, that we get right length if input
	 is 128K or multiple. */
	*len = (chunk_no == (max_chunk_cnt - 1)) ?
		(chunk_rem ? chunk_rem : MAX_FW_CHUNK) : MAX_FW_CHUNK;

	*offset = fw_ud_ptr->fw_file_data + fw_offset +
		(fw_size/((max_chunk_cnt - chunk_no)
		* MAX_FW_CHUNK))*MAX_FW_CHUNK;

	return 0;

}

/*
 * This api calculates offset and length depending on type of firmware chunk
 * requested by SCU. Note: Intent is to follow the architecture such that,
 * SCU controls the flow, and IA simply hands out, what is requested by SCU.
 * IA will simply follow SCU's commands, unless SCU requests for something
 * IA cannot give. TODO:That will be a special error case, need to figure out
 * how to handle that.
 */
int calc_offset_and_length(struct fw_ud *fw_ud_ptr, char *scu_req,
			void **offset, u32 *len, struct fuph_hdr *fuph)
{

	u8 cnt;

	if (!strncmp(DNX_IMAGE, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->dnx_file_data;
		*len = fw_ud_ptr->dnx_size;
		return 0;
	} else if (!strncmp(FUPH, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->fw_file_data + fw_ud_ptr->fsize
				- FUPH_HDR_LEN;
		*len = FUPH_HDR_LEN;
		return 0;
	} else if (!strncmp(MIP, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->fw_file_data + MIP_HEADER_OFFSET;
		*len = fuph->mip_size;
		return 0;
	} else if (!strncmp(LOWER_128K, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->fw_file_data + fuph->mip_size;
		*len = MAX_FW_CHUNK;
		return 0;
	} else if (!strncmp(UPPER_128K, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->fw_file_data
				+ fuph->mip_size + MAX_FW_CHUNK;
		*len = MAX_FW_CHUNK;
		return 0;
	} else {
		for (cnt = 0; cnt < ARRAY_SIZE(misc_fw_table) ; cnt++) {

			if (!strncmp(misc_fw_table[cnt].fw_type, scu_req,
					strlen(misc_fw_table[cnt].fw_type))) {

				if (strlen(scu_req) ==
						misc_fw_table[cnt].str_len) {

					if (helper_for_calc_offset_length
						(fw_ud_ptr, scu_req,
						offset, len, fuph,
						misc_fw_table[cnt].fw_type) < 0)
						goto error_case;

					dev_dbg(&ipcdev.pdev->dev,
					"\nmisc fw type=%s, len=%d,offset=%d",
					misc_fw_table[cnt].fw_type, *len,
					(int)*offset);

					return 0;

				} else
					goto error_case;
			}
		}
	}

	dev_dbg(&ipcdev.pdev->dev,
			"Unexpected mailbox request from scu\n");

error_case:
	/* TODO::Need to test this error case..and see how SCU reacts
	* and how IA handles
	* subsequent error response and whether exit is graceful...
	*/

	dev_dbg(&ipcdev.pdev->dev, "error case,respond back to SCU..\n");
	dev_dbg(&ipcdev.pdev->dev, "scu_req=%s\n", scu_req);
	*len = 0;
	*offset = 0;

	return -EINVAL;

}

int intel_scu_ipc_medfw_prepare(void __user *arg)
{
	int ret;
	struct fw_ud param;

	if (platform != MRST_CPU_CHIP_PENWELL)
		return -EINVAL;

	ret = copy_from_user(&param, arg, sizeof(struct fw_ud));
	if (ret)
		return ret;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL) {
		ret = -ENODEV;
		goto fail;
	} else {
		/* Update NOT supported for older silicon stepping */
		if (ipcdev.pdev->revision != C0_STEPPING) {
			dev_err(&ipcdev.pdev->dev,
				"Update NOT supported for stepping=%d\n",
				ipcdev.pdev->revision);
			ret = -EINVAL;
			goto fail;
		}
	}

	if (param.fw_file_data == NULL || param.fsize == 0
		|| param.dnx_size == 0 || param.dnx_hdr == NULL
		|| param.dnx_file_data == NULL) {
		dev_err(&ipcdev.pdev->dev,
			"ipc_medfw_upgrade, invalid args!!\n");
		ret = -EINVAL;
		goto fail;
	}

	ipcdev.fwud_pending = kzalloc(sizeof(struct fw_ud), GFP_KERNEL);
	if (NULL == ipcdev.fwud_pending) {
		ret = -ENOMEM;
		goto fail;
	}
	memcpy(ipcdev.fwud_pending, &param, sizeof(struct fw_ud));

	ipcdev.fwud_pending->fw_file_data = kmalloc(param.fsize, GFP_KERNEL);
	if (NULL == ipcdev.fwud_pending) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = copy_from_user(ipcdev.fwud_pending->fw_file_data,
			param.fw_file_data, param.fsize);
	if (ret)
		goto fail;

	ipcdev.fwud_pending->dnx_hdr = kmalloc(DNX_HDR_LEN, GFP_KERNEL);
	if (NULL == ipcdev.fwud_pending->dnx_hdr) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = copy_from_user(ipcdev.fwud_pending->dnx_hdr,
			param.dnx_hdr, DNX_HDR_LEN);
	if (ret)
		goto fail;

	ipcdev.fwud_pending->dnx_file_data = kmalloc(param.dnx_size,
			GFP_KERNEL);
	if (NULL == ipcdev.fwud_pending->dnx_file_data) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = copy_from_user(ipcdev.fwud_pending->dnx_file_data,
			param.dnx_file_data, param.dnx_size);
	if (ret)
		goto fail;

	mutex_unlock(&ipclock);
	return 0;
fail:
	if (ipcdev.fwud_pending) {
		kfree(ipcdev.fwud_pending->fw_file_data);
		kfree(ipcdev.fwud_pending->dnx_hdr);
		kfree(ipcdev.fwud_pending->dnx_file_data);
	}
	kfree(ipcdev.fwud_pending);
	mutex_unlock(&ipclock);
	return ret;
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
int intel_scu_ipc_medfw_upgrade(void)
{
	struct fw_ud *fw_ud_param = ipcdev.fwud_pending;
	struct mfld_fw_update	mfld_fw_upd;
	u8 *fw_file_data = NULL;
	u8 *fws = NULL;
	int ret_val = 0;
	struct fuph_hdr fuph;
	u32 length = 0;
	void *offset;
	enum mailbox_status mb_state;

	if (!fw_ud_param)
		return 0;

	mutex_lock(&ipclock);
	mfld_fw_upd.wscu = 0;
	mfld_fw_upd.wia = 0;
	memset(mfld_fw_upd.mb_status, 0, sizeof(char) * 8);

	fw_file_data = fw_ud_param->fw_file_data;
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
	memset_io(mfld_fw_upd.mailbox, 0, 8);

	fws = kmalloc(MAX_FW_CHUNK, GFP_KERNEL);
	if (fws == NULL) {
		ret_val = -ENOMEM;
		goto unmap_mb;
	}
	/* Copy fuph header to kernel space */
	memcpy(&fuph, (fw_ud_param->fw_file_data + (fw_ud_param->fsize - 1)
			 - (FUPH_HDR_LEN - 1)), FUPH_HDR_LEN);

	/* Convert sizes in DWORDS to number of bytes. */
	fuph.mip_size = fuph.mip_size * 4;
	fuph.ifwi_size = fuph.ifwi_size * 4;
	fuph.psfw1_size = fuph.psfw1_size * 4;
	fuph.psfw2_size = fuph.psfw2_size * 4;
	fuph.ssfw_size = fuph.ssfw_size * 4;
	fuph.sucp_size = fuph.sucp_size * 4;

	dev_dbg(&ipcdev.pdev->dev,
		"mip=%d, ifwi=%d, ps1=%d, ps2=%d, sfw=%d, sucp=%d\n",
		fuph.mip_size, fuph.ifwi_size, fuph.psfw1_size,
		fuph.psfw2_size, fuph.ssfw_size, fuph.sucp_size);

	/* TODO_SK::There is just
	 *  1 write required from IA side for DFU.
	 *  So commenting this-out, until it gets confirmed */
	/*ipc_command(IPC_CMD_FW_UPDATE_READY); */

	/*1. DNX SIZE HEADER   */
	memcpy(fws, fw_ud_param->dnx_hdr, DNX_HDR_LEN);

	memcpy_toio(mfld_fw_upd.sram, fws, DNX_HDR_LEN);

	/* There are synchronization issues between IA and SCU */
	mb();

	/* Write cmd to trigger an interrupt to SCU for firmware update*/
	ipc_command(IPC_CMD_FW_UPDATE_GO);

	mfld_fw_upd.wscu = !mfld_fw_upd.wscu;

	ret_val = busy_wait(&mfld_fw_upd);
	if (ret_val < 0)
		goto term;

	/* TODO:Add a count for iteration, based on sizes of security firmware,
	 * so that we determine finite number of iterations to loop thro.
	 * That way at the very least, we can atleast control the number
	 * of iterations, and prevent infinite looping if there are any bugs.
	 * The only catch being for B0, SCU will request twice for each firmware
	 * chunk, since its writing to 2 partitions.
	 * TODO::Investigate if we need to increase timeout for busy_wait,
	 * since SCU is now writing to 2 partitions.
	 */

	while ((mb_state = check_mb_status(&mfld_fw_upd)) != MB_DONE) {

		if (mb_state == MB_ERROR) {
			dev_dbg(&ipcdev.pdev->dev, "check_mb_status,error\n");
			ret_val = -1;
			goto term;
		}

		if (!strncmp(mfld_fw_upd.mb_status, FUPH_HDR_SIZE,
				strlen(FUPH_HDR_SIZE))) {
			iowrite32(FUPH_HDR_LEN, mfld_fw_upd.sram);
			/* There are synchronization issues between IA-SCU */
			mb();
			dev_dbg(&ipcdev.pdev->dev,
				"copied fuph hdr size=%d\n",
				ioread32(mfld_fw_upd.sram));
			mfld_fw_upd.wia = !mfld_fw_upd.wia;
			iowrite32(mfld_fw_upd.wia, mfld_fw_upd.mailbox +
				IA_FLAG_OFFSET);
			dev_dbg(&ipcdev.pdev->dev, "ia_flag=%d\n",
				ioread32(mfld_fw_upd.mailbox + IA_FLAG_OFFSET));
			mb();
			mfld_fw_upd.wscu = !mfld_fw_upd.wscu;

			if (busy_wait(&mfld_fw_upd) < 0) {
				ret_val = -1;
				goto term;
			}

			continue;
		}

		if (calc_offset_and_length(fw_ud_param, mfld_fw_upd.mb_status,
					&offset, &length, &fuph) < 0) {
			dev_err(&ipcdev.pdev->dev,
			"calc_offset_and_length_error,error\n");
			ret_val = -1;
			goto term;
		}

		if ((process_fw_chunk(fws, offset, length,
				      &mfld_fw_upd)) != 0) {
			dev_err(&ipcdev.pdev->dev,
			"Error processing fw chunk=%s\n",
			mfld_fw_upd.mb_status);
			ret_val = -1;
			goto term;
		} else
			dev_dbg(&ipcdev.pdev->dev,
				"PASS processing fw chunk=%s\n",
				mfld_fw_upd.mb_status);
	}
	ret_val = ipc_wait_interrupt();

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

int intel_scu_ipc_read_mip(u8 *data, int len, int offset, int issigned)
{
	int ret;
	u32 cmdid;
	u32 data_off;

	if (platform != MRST_CPU_CHIP_PENWELL)
		return -EINVAL;

	if (offset + len > IPC_MIP_MAX_ADDR)
		return -EINVAL;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL || ipcdev.mip_base == NULL) {
		mutex_unlock(&ipclock);
		return -ENODEV;
	}

	do {
		writel(offset, ipcdev.ipc_base + IPC_DPTR_ADDR);
		writel((len + 3) / 4, ipcdev.ipc_base + IPC_SPTR_ADDR);

		cmdid = issigned ? IPC_CMD_SMIP_RD : IPC_CMD_UMIP_RD;
		ipc_command(4 << 16 | cmdid << 12 | IPCMSG_MIP_ACCESS);
		ret = ipc_wait_interrupt();
	} while (ret == -EIO);
	if (!ret) {
		data_off = ipc_data_readl(0);
		memcpy(data, ipcdev.mip_base + data_off, len);
	}
	mutex_unlock(&ipclock);

	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_read_mip);

int intel_scu_ipc_write_umip(u8 *data, int len, int offset)
{
	int ret, wlen = 0;
	u32 data_off;
	u8 *buf = NULL;

	if (platform != MRST_CPU_CHIP_PENWELL)
		return -EINVAL;
	if (offset + len > IPC_MIP_MAX_ADDR)
		return -EINVAL;

	mutex_lock(&ipclock);
	if (ipcdev.pdev == NULL || ipcdev.mip_base == NULL) {
		ret = -ENODEV;
		goto fail;
	}
	wlen = (len + 3) & (~0x3);
	if (wlen != len) {
		buf = kzalloc(wlen, GFP_KERNEL);
		if (!buf) {
			dev_err(&ipcdev.pdev->dev, "Alloc memory failed\n");
			ret = -ENOMEM;
			goto fail;
		}
		do {
			writel(offset+wlen-4, ipcdev.ipc_base + IPC_DPTR_ADDR);
			writel(1, ipcdev.ipc_base + IPC_SPTR_ADDR);
			ipc_command(4 << 16
				| IPC_CMD_UMIP_RD << 12 | IPCMSG_MIP_ACCESS);
			ret = ipc_wait_interrupt();
		} while (ret == -EIO);
		if (ret)
			goto fail;
		data_off = ipc_data_readl(0);
		memcpy(buf+wlen-4, ipcdev.mip_base + data_off, 4);
		memcpy(buf, data, len);
	} else {
		buf = data;
	}
	do {
		writel(offset, ipcdev.ipc_base + IPC_DPTR_ADDR);
		writel(wlen / 4, ipcdev.ipc_base + IPC_SPTR_ADDR);
		memcpy(ipcdev.mip_base, buf, wlen);
		ipc_command(IPC_CMD_UMIP_WR << 12 | IPCMSG_MIP_ACCESS);
		ret = ipc_wait_interrupt();
	} while (ret == -EIO);
fail:
	if (buf && wlen != len)
		kfree(buf);
	mutex_unlock(&ipclock);

	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_write_umip);

#define OSHOB_SIZE		60
#define OSNIB_SIZE		32
#define IPCMSG_GET_HOBADDR	0xE5

int intel_scu_ipc_read_oshob(u8 *data, int len, int offset)
{
	int ret, i;
	u32 oshob_base;
	void __iomem *oshob_addr;
	pr_info("Get osHOB address------->-------------->");
	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR, 0,
				NULL, 0, &oshob_base, 1);
	if (ret < 0) {
		pr_err("ipc_read_osnib failed!!\n");
		goto exit;
	}
	pr_info("OSHOB addr values is %x\n", oshob_base);
	oshob_addr = ioremap_nocache(oshob_base, OSHOB_SIZE);
	if (!oshob_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	} else {
		u8 *ptr = data;
		for (i = 0; i < len; i = i+1) {
			*ptr = readb(oshob_addr + offset + i);
			pr_info("addr=%x, offset=%x, value=%x\n",
				(u32)(oshob_addr+i), offset+i, *ptr);
			pr_info("--------------------------------------\n");
			ptr++;
		}
	}
	iounmap(oshob_addr);
exit:
	return 0;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_oshob);

#define IPCMSG_WRITE_OSNIB	0xE4
#define POSNIBW_OFFSET		0x34
#define IPCREG_IPC_SPTR		0xFF11C008

int intel_scu_ipc_write_osnib(u8 *data, int len, int offset, u32 mask)
{
	int ret = 0;
	int i;
	u32 posnibw;
	u32 oshob_base;
	void __iomem *oshob_addr;
	void __iomem *osnibw_addr;
	void __iomem *sptr_addr;

	pr_info("Get osHOB address------->-------------->");
	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR, 0,
				NULL, 0, &oshob_base, 1);
	if (ret < 0) {
		pr_err("ipc_get_hobaddr failed!!\n");
		goto exit;
	}
	pr_info("OSHOB addr values is %x\n", oshob_base);
	oshob_addr = ioremap_nocache(oshob_base, OSHOB_SIZE);
	if (!oshob_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}
	posnibw = readl(oshob_addr + POSNIBW_OFFSET);
	if (posnibw == 0) { /* workaround here for BZ 2914 */
		posnibw = 0xFFFF3400;
		pr_err("ERR: posnibw from oshob is 0, manually set it here\n");
	}
	pr_info("POSNIB: %x\n", posnibw);
	osnibw_addr = ioremap_nocache(posnibw, OSNIB_SIZE);
	if (!osnibw_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}
	for (i = 0; i < len; i++)
		writeb(*(data + i), (osnibw_addr + offset + i));
	sptr_addr = ioremap_nocache(IPCREG_IPC_SPTR, sizeof(u32));
	if (!sptr_addr) {
		pr_err("ioremap failed\n");
		ret = -ENOMEM;
		goto unmap_osnibw_addr;
	};
	writel(mask, sptr_addr);
	ret = intel_scu_ipc_command(IPCMSG_WRITE_OSNIB, 0,
				NULL, 0 , NULL, 0);
	if (ret < 0)
		pr_err("ipc_write_osnib failed!!\n");
	iounmap(sptr_addr);
unmap_osnibw_addr:
	iounmap(osnibw_addr);
unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_osnib);

/*
 * Interrupt handler gets called when ioc bit of IPC_COMMAND_REG set to 1
 * When ioc bit is set to 1, caller api must wait for interrupt handler called
 * which in turn unlocks the caller api. Currently this is not used
 *
 * This is edge triggered so we need take no action to clear anything
 */
static irqreturn_t ioc(int irq, void *dev_id)
{
	ipcdev.status = ipc_read_status();
	complete(&ipcdev.cmd_complete);
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

	init_completion(&ipcdev.cmd_complete);

	if (request_irq(dev->irq, ioc, IRQF_NO_SUSPEND, "intel_scu_ipc",
		&ipcdev))
		return -EBUSY;

	ipcdev.ipc_base = ioremap_nocache(IPC_BASE_ADDR, IPC_MAX_ADDR);
	if (!ipcdev.ipc_base)
		return -ENOMEM;

	ipcdev.i2c_base = ioremap_nocache(IPC_I2C_BASE, IPC_I2C_MAX_ADDR);
	if (!ipcdev.i2c_base) {
		iounmap(ipcdev.ipc_base);
		return -ENOMEM;
	}

	ipcdev.mip_base = ioremap_nocache(IPC_MIP_BASE, IPC_MIP_MAX_ADDR);
	if (!ipcdev.mip_base) {
		iounmap(ipcdev.i2c_base);
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
	iounmap(ipcdev.mip_base);
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

	qos = kzalloc(sizeof(struct pm_qos_request_list), GFP_KERNEL);
	if (!qos)
		return -ENOMEM;

	pm_qos_add_request(qos, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);

	return  pci_register_driver(&ipc_driver);

}

static void __exit intel_scu_ipc_exit(void)
{
	pm_qos_remove_request(qos);

	pci_unregister_driver(&ipc_driver);
}

MODULE_AUTHOR("Sreedhara DS <sreedhara.ds@intel.com>");
MODULE_DESCRIPTION("Intel SCU IPC driver");
MODULE_LICENSE("GPL");

fs_initcall(intel_scu_ipc_init);
module_exit(intel_scu_ipc_exit);
