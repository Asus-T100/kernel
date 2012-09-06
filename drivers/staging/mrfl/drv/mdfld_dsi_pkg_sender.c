/*
 * Copyright © 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Jackie Li<yaodong.li@intel.com>
 */

#include <linux/freezer.h>

#include "mdfld_dsi_output.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dpi.h"

#define MDFLD_DSI_DBI_FIFO_TIMEOUT		100
#define MDFLD_DSI_MAX_RETURN_PACKET_SIZE	512
#define MDFLD_DSI_READ_MAX_COUNT		10000

const char *dsi_errors[] = {
	"RX SOT Error",
	"RX SOT Sync Error",
	"RX EOT Sync Error",
	"RX Escape Mode Entry Error",
	"RX LP TX Sync Error",
	"RX HS Receive Timeout Error",
	"RX False Control Error",
	"RX ECC Single Bit Error",
	"RX ECC Multibit Error",
	"RX Checksum Error",
	"RX DSI Data Type Not Recognised",
	"RX DSI VC ID Invalid",
	"TX False Control Error",
	"TX ECC Single Bit Error",
	"TX ECC Multibit Error",
	"TX Checksum Error",
	"TX DSI Data Type Not Recognised",
	"TX DSI VC ID invalid",
	"High Contention",
	"Low contention",
	"DPI FIFO Under run",
	"HS TX Timeout",
	"LP RX Timeout",
	"Turn Around ACK Timeout",
	"ACK With No Error",
	"RX Invalid TX Length",
	"RX Prot Violation",
	"HS Generic Write FIFO Full",
	"LP Generic Write FIFO Full",
	"Generic Read Data Avail" "Special Packet Sent",
	"Tearing Effect",
};

static inline int wait_for_gen_fifo_empty(struct mdfld_dsi_pkg_sender *sender,
					  u32 mask)
{
	struct drm_device *dev = sender->dev;
	u32 gen_fifo_stat_reg = sender->mipi_gen_fifo_stat_reg;
	int retry = 10000;

#if 1				/* FIXME MRFLD */
	return 0;
#endif				/* FIXME MRFLD */
	while (retry--) {
		if ((mask & REG_READ(gen_fifo_stat_reg)) == mask)
			return 0;
		udelay(3);
	}

	DRM_ERROR("fifo is NOT empty 0x%08x\n", REG_READ(gen_fifo_stat_reg));
	return -EIO;
}

static int wait_for_all_fifos_empty(struct mdfld_dsi_pkg_sender *sender)
{
	return wait_for_gen_fifo_empty(sender,
				       (BIT2 | BIT10 | BIT18 | BIT26 | BIT27 |
					BIT28));
}

static int wait_for_lp_fifos_empty(struct mdfld_dsi_pkg_sender *sender)
{
	return wait_for_gen_fifo_empty(sender, (BIT10 | BIT26));
}

static int wait_for_hs_fifos_empty(struct mdfld_dsi_pkg_sender *sender)
{
	return wait_for_gen_fifo_empty(sender, (BIT2 | BIT18));
}

static int wait_for_dbi_fifo_empty(struct mdfld_dsi_pkg_sender *sender)
{
	return wait_for_gen_fifo_empty(sender, (BIT27));
}

static int wait_for_dpi_fifo_empty(struct mdfld_dsi_pkg_sender *sender)
{
	return wait_for_gen_fifo_empty(sender, (BIT28));
}

static int handle_dsi_error(struct mdfld_dsi_pkg_sender *sender, u32 mask)
{
	u32 intr_stat_reg = sender->mipi_intr_stat_reg;
	struct drm_device *dev = sender->dev;

	PSB_DEBUG_ENTRY("Handling error 0x%08x\n", mask);

	switch (mask) {
	case BIT0:
	case BIT1:
	case BIT2:
	case BIT3:
	case BIT4:
	case BIT5:
	case BIT6:
	case BIT7:
	case BIT8:
	case BIT9:
	case BIT10:
	case BIT11:
	case BIT12:
	case BIT13:
		PSB_DEBUG_ENTRY("No Action required\n");
		break;
	case BIT14:
		/*wait for all fifo empty */
		/*wait_for_all_fifos_empty(sender) */ ;
		break;
	case BIT15:
		PSB_DEBUG_ENTRY("No Action required\n");
		break;
	case BIT16:
		break;
	case BIT17:
		break;
	case BIT18:
	case BIT19:
		PSB_DEBUG_ENTRY("High/Low contention detected\n");
		/*wait for contention recovery time */
		/*mdelay(10); */
		/*wait for all fifo empty */
		if (0)
			wait_for_all_fifos_empty(sender);
		break;
	case BIT20:
		PSB_DEBUG_ENTRY("No Action required\n");
		break;
	case BIT21:
		/*wait for all fifo empty */
		/*wait_for_all_fifos_empty(sender); */
		break;
	case BIT22:
		break;
	case BIT23:
	case BIT24:
	case BIT25:
	case BIT26:
	case BIT27:
		PSB_DEBUG_ENTRY("HS Gen fifo full\n");
		REG_WRITE(intr_stat_reg, mask);
		wait_for_hs_fifos_empty(sender);
		break;
	case BIT28:
		PSB_DEBUG_ENTRY("LP Gen fifo full\n");
		REG_WRITE(intr_stat_reg, mask);
		wait_for_lp_fifos_empty(sender);
		break;
	case BIT29:
	case BIT30:
	case BIT31:
		PSB_DEBUG_ENTRY("No Action required\n");
		break;
	}

	if (mask & REG_READ(intr_stat_reg))
		PSB_DEBUG_ENTRY("Cannot clean interrupt 0x%08x\n", mask);

	return 0;
}

static int dsi_error_handler(struct mdfld_dsi_pkg_sender *sender)
{
	struct drm_device *dev = sender->dev;
	u32 intr_stat_reg = sender->mipi_intr_stat_reg;
	u32 mask;
	u32 intr_stat;
	int i;
	int err = 0;

	intr_stat = REG_READ(intr_stat_reg);

	for (i = 0; i < 32; i++) {
		mask = (0x00000001UL) << i;
		if (intr_stat & mask) {
			PSB_DEBUG_ENTRY("[DSI]: %s\n", dsi_errors[i]);
			err = handle_dsi_error(sender, mask);
			if (err)
				DRM_ERROR("Cannot handle error\n");
		}
	}

	return err;
}

static inline int dbi_cmd_sent(struct mdfld_dsi_pkg_sender *sender)
{
	struct drm_device *dev = sender->dev;
	u32 retry = 0xffff;
	u32 dbi_cmd_addr_reg = sender->mipi_cmd_addr_reg;
	int ret = 0;

	/*query the command execution status */
	while (retry--) {
		if (!(REG_READ(dbi_cmd_addr_reg) & BIT0))
			break;
	}

	if (!retry) {
		DRM_ERROR("Timeout waiting for DBI Command status\n");
		ret = -EAGAIN;
	}

	return ret;
}

/**
 * NOTE: this interface is abandoned expect for write_mem_start DCS
 * other DCS are sent via generic pkg interfaces
 */
static int send_dcs_pkg(struct mdfld_dsi_pkg_sender *sender,
			struct mdfld_dsi_pkg *pkg)
{
	struct drm_device *dev = sender->dev;
	struct mdfld_dsi_dcs_pkg *dcs_pkg = &pkg->pkg.dcs_pkg;
	u32 dbi_cmd_len_reg = sender->mipi_cmd_len_reg;
	u32 dbi_cmd_addr_reg = sender->mipi_cmd_addr_reg;
	u32 cb_phy = sender->dbi_cb_phy;
	u32 index = 0;
	u8 *cb = (u8 *) sender->dbi_cb_addr;
	int i;
	int ret;

	if (!sender->dbi_pkg_support) {
		DRM_ERROR("Trying to send DCS on a non DBI output, abort!\n");
		return -ENOTSUPP;
	}

	PSB_DEBUG_ENTRY("Sending DCS pkg 0x%x...\n", dcs_pkg->cmd);

	/*wait for DBI fifo empty */
	wait_for_dbi_fifo_empty(sender);

	*(cb + (index++)) = dcs_pkg->cmd;
	if (dcs_pkg->param_num) {
		for (i = 0; i < dcs_pkg->param_num; i++)
			*(cb + (index++)) = *(dcs_pkg->param + i);
	}

	REG_WRITE(dbi_cmd_len_reg, (1 + dcs_pkg->param_num));
	REG_WRITE(dbi_cmd_addr_reg, (cb_phy << CMD_MEM_ADDR_OFFSET)
		  | BIT0
		  | ((dcs_pkg->data_src == CMD_DATA_SRC_PIPE) ? BIT1 : 0));

	ret = dbi_cmd_sent(sender);
	if (ret) {
		DRM_ERROR("command 0x%x not complete\n", dcs_pkg->cmd);
		return -EAGAIN;
	}

	PSB_DEBUG_ENTRY("sent DCS pkg 0x%x...\n", dcs_pkg->cmd);

	return 0;
}

static int __send_short_pkg(struct mdfld_dsi_pkg_sender *sender,
			    struct mdfld_dsi_pkg *pkg)
{
	struct drm_device *dev = sender->dev;
	u32 hs_gen_ctrl_reg = sender->mipi_hs_gen_ctrl_reg;
	u32 lp_gen_ctrl_reg = sender->mipi_lp_gen_ctrl_reg;
	u32 gen_ctrl_val = 0;
	struct mdfld_dsi_gen_short_pkg *short_pkg = &pkg->pkg.short_pkg;

	gen_ctrl_val |= short_pkg->cmd << MCS_COMMANDS_POS;
	gen_ctrl_val |= 0 << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= pkg->pkg_type;
	gen_ctrl_val |= short_pkg->param << MCS_PARAMETER_POS;

	if (pkg->transmission_type == MDFLD_DSI_HS_TRANSMISSION) {
		/*wait for hs fifo empty */
		wait_for_hs_fifos_empty(sender);

		/*send pkg */
		REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);
	} else if (pkg->transmission_type == MDFLD_DSI_LP_TRANSMISSION) {
		wait_for_lp_fifos_empty(sender);

		/*send pkg */
		REG_WRITE(lp_gen_ctrl_reg, gen_ctrl_val);
	} else {
		DRM_ERROR("Unknown transmission type %d\n",
			  pkg->transmission_type);
		return -EINVAL;
	}

	return 0;
}

static int __send_long_pkg(struct mdfld_dsi_pkg_sender *sender,
			   struct mdfld_dsi_pkg *pkg)
{
	struct drm_device *dev = sender->dev;
	u32 hs_gen_ctrl_reg = sender->mipi_hs_gen_ctrl_reg;
	u32 hs_gen_data_reg = sender->mipi_hs_gen_data_reg;
	u32 lp_gen_ctrl_reg = sender->mipi_lp_gen_ctrl_reg;
	u32 lp_gen_data_reg = sender->mipi_lp_gen_data_reg;
	u32 gen_ctrl_val = 0;
	u32 *dp;
	int i;
	struct mdfld_dsi_gen_long_pkg *long_pkg = &pkg->pkg.long_pkg;

	dp = long_pkg->data;

	/**
	 * Set up word count for long pkg
	 * FIXME: double check word count field.
	 * currently, using the byte counts of the payload as the word count.
	 * ------------------------------------------------------------
	 * | DI |   WC   | ECC|         PAYLOAD              |CHECKSUM|
	 * ------------------------------------------------------------
	 */
	gen_ctrl_val |= (long_pkg->len << 2) << WORD_COUNTS_POS;
	gen_ctrl_val |= 0 << DCS_CHANNEL_NUMBER_POS;
	gen_ctrl_val |= pkg->pkg_type;

	if (pkg->transmission_type == MDFLD_DSI_HS_TRANSMISSION) {
		/*wait for hs ctrl and data fifos to be empty */
		wait_for_hs_fifos_empty(sender);

		for (i = 0; i < long_pkg->len; i++) {
			PSB_DEBUG_ENTRY("HS Sending data 0x%08x\n", *(dp + i));

			REG_WRITE(hs_gen_data_reg, *(dp + i));
		}

		REG_WRITE(hs_gen_ctrl_reg, gen_ctrl_val);

	} else if (pkg->transmission_type == MDFLD_DSI_LP_TRANSMISSION) {
		wait_for_lp_fifos_empty(sender);

		for (i = 0; i < long_pkg->len; i++) {
			PSB_DEBUG_ENTRY("LP Sending data 0x%08x\n", *(dp + i));

			REG_WRITE(lp_gen_data_reg, *(dp + i));
		}

		REG_WRITE(lp_gen_ctrl_reg, gen_ctrl_val);
	} else {
		DRM_ERROR("Unknown transmission type %d\n",
			  pkg->transmission_type);
		return -EINVAL;
	}

	return 0;

}

static int send_mcs_short_pkg(struct mdfld_dsi_pkg_sender *sender,
			      struct mdfld_dsi_pkg *pkg)
{
	PSB_DEBUG_ENTRY("Sending MCS short pkg...\n");

	return __send_short_pkg(sender, pkg);
}

static int send_mcs_long_pkg(struct mdfld_dsi_pkg_sender *sender,
			     struct mdfld_dsi_pkg *pkg)
{
	PSB_DEBUG_ENTRY("Sending MCS long pkg...\n");

	return __send_long_pkg(sender, pkg);
}

static int send_gen_short_pkg(struct mdfld_dsi_pkg_sender *sender,
			      struct mdfld_dsi_pkg *pkg)
{
	PSB_DEBUG_ENTRY("Sending GEN short pkg...\n");

	return __send_short_pkg(sender, pkg);
}

static int send_gen_long_pkg(struct mdfld_dsi_pkg_sender *sender,
			     struct mdfld_dsi_pkg *pkg)
{
	PSB_DEBUG_ENTRY("Sending GEN long pkg...\n");

	return __send_long_pkg(sender, pkg);
}

static int send_dpi_spk_pkg(struct mdfld_dsi_pkg_sender *sender,
			    struct mdfld_dsi_pkg *pkg)
{
	struct drm_device *dev = sender->dev;
	u32 dpi_control_reg = sender->mipi_dpi_control_reg;
	u32 intr_stat_reg = sender->mipi_intr_stat_reg;
	u32 dpi_control_val = 0;
	struct mdfld_dsi_dpi_spk_pkg *spk_pkg = &pkg->pkg.spk_pkg;
	int retry = 10000;

	dpi_control_val = spk_pkg->cmd;

	if (pkg->transmission_type == MDFLD_DSI_LP_TRANSMISSION)
		dpi_control_val |= BIT6;

	/*Wait for DPI fifo empty */
	wait_for_dpi_fifo_empty(sender);

	/*clean spk packet sent interrupt */
	REG_WRITE(intr_stat_reg, BIT30);

	/*send out spk packet */
	REG_WRITE(dpi_control_reg, dpi_control_val);

#if 1				/* FIXME MRFLD */
	return 0;
#endif				/* FIXME MRFLD */
	/*wait for spk packet sent interrupt */
	while (--retry && !(REG_READ(intr_stat_reg) & BIT30))
		udelay(3);

	if (!retry) {
		DRM_ERROR("Fail to send SPK Packet 0x%x\n", spk_pkg->cmd);
		return -EINVAL;
	}

	return 0;
}

static int send_pkg_prepare(struct mdfld_dsi_pkg_sender *sender,
			    struct mdfld_dsi_pkg *pkg)
{
	u8 cmd;
	u8 *data;

	PSB_DEBUG_ENTRY("Prepare to Send type 0x%x pkg\n", pkg->pkg_type);

	switch (pkg->pkg_type) {
	case MDFLD_DSI_PKG_DCS:
		cmd = pkg->pkg.dcs_pkg.cmd;
		break;
	case MDFLD_DSI_PKG_MCS_SHORT_WRITE_0:
	case MDFLD_DSI_PKG_MCS_SHORT_WRITE_1:
		cmd = pkg->pkg.short_pkg.cmd;
		break;
	case MDFLD_DSI_PKG_MCS_LONG_WRITE:
		data = (u8 *) pkg->pkg.long_pkg.data;
		cmd = *data;
		break;
	default:
		return 0;
	}

	/*this prevents other package sending while doing msleep */
	sender->status = MDFLD_DSI_PKG_SENDER_BUSY;

	/*wait for 120 milliseconds in case exit_sleep_mode just be sent */
	if (unlikely(cmd == enter_sleep_mode)) {
		/*TODO: replace it with msleep later */
		mdelay(120);
	}

	if (unlikely(cmd == exit_sleep_mode)) {
		/*TODO: replace it with msleep later */
		mdelay(120);
	}

	return 0;
}

static int send_pkg_done(struct mdfld_dsi_pkg_sender *sender,
			 struct mdfld_dsi_pkg *pkg)
{
	u8 cmd;
	u8 *data;

	PSB_DEBUG_ENTRY("Sent type 0x%x pkg\n", pkg->pkg_type);

	switch (pkg->pkg_type) {
	case MDFLD_DSI_PKG_DCS:
		cmd = pkg->pkg.dcs_pkg.cmd;
		break;
	case MDFLD_DSI_PKG_MCS_SHORT_WRITE_0:
	case MDFLD_DSI_PKG_MCS_SHORT_WRITE_1:
		cmd = pkg->pkg.short_pkg.cmd;
		break;
	case MDFLD_DSI_PKG_MCS_LONG_WRITE:
		data = (u8 *) pkg->pkg.long_pkg.data;
		cmd = *data;
		break;
	default:
		return 0;
	}

	/*update panel status */
	if (unlikely(cmd == enter_sleep_mode)) {
		sender->panel_mode |= MDFLD_DSI_PANEL_MODE_SLEEP;
		/*TODO: replace it with msleep later */
		mdelay(120);
	} else if (unlikely(cmd == exit_sleep_mode)) {
		sender->panel_mode &= ~MDFLD_DSI_PANEL_MODE_SLEEP;
		/*TODO: replace it with msleep later */
		mdelay(120);
	}

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	return 0;

}

static int do_send_pkg(struct mdfld_dsi_pkg_sender *sender,
		       struct mdfld_dsi_pkg *pkg)
{
	int ret = 0;

	PSB_DEBUG_ENTRY("Sending type 0x%x pkg\n", pkg->pkg_type);

	if (sender->status == MDFLD_DSI_PKG_SENDER_BUSY) {
		DRM_ERROR("sender is busy\n");
		return -EAGAIN;
	}

	ret = send_pkg_prepare(sender, pkg);
	if (ret) {
		DRM_ERROR("send_pkg_prepare error\n");
		return ret;
	}

	switch (pkg->pkg_type) {
	case MDFLD_DSI_PKG_DCS:
		ret = send_dcs_pkg(sender, pkg);
		break;
	case MDFLD_DSI_PKG_GEN_SHORT_WRITE_0:
	case MDFLD_DSI_PKG_GEN_SHORT_WRITE_1:
	case MDFLD_DSI_PKG_GEN_SHORT_WRITE_2:
	case MDFLD_DSI_PKG_GEN_READ_0:
	case MDFLD_DSI_PKG_GEN_READ_1:
	case MDFLD_DSI_PKG_GEN_READ_2:
		ret = send_gen_short_pkg(sender, pkg);
		break;
	case MDFLD_DSI_PKG_GEN_LONG_WRITE:
		ret = send_gen_long_pkg(sender, pkg);
		break;
	case MDFLD_DSI_PKG_MCS_SHORT_WRITE_0:
	case MDFLD_DSI_PKG_MCS_SHORT_WRITE_1:
	case MDFLD_DSI_PKG_MCS_READ:
		ret = send_mcs_short_pkg(sender, pkg);
		break;
	case MDFLD_DSI_PKG_MCS_LONG_WRITE:
		ret = send_mcs_long_pkg(sender, pkg);
		break;
	case MDFLD_DSI_DPI_SPK:
		ret = send_dpi_spk_pkg(sender, pkg);
		break;
	default:
		DRM_ERROR("Invalid pkg type 0x%x\n", pkg->pkg_type);
		ret = -EINVAL;
	}

	send_pkg_done(sender, pkg);

	return ret;
}

static int send_pkg(struct mdfld_dsi_pkg_sender *sender,
		    struct mdfld_dsi_pkg *pkg)
{
	int err = 0;

	/*handle DSI error */
	err = dsi_error_handler(sender);
	if (err) {
		DRM_ERROR("Error handling failed\n");
		err = -EAGAIN;
		goto send_pkg_err;
	}

	/*send pkg */
	err = do_send_pkg(sender, pkg);
	if (err) {
		DRM_ERROR("sent pkg failed\n");
		err = -EAGAIN;
		goto send_pkg_err;
	}

	/*FIXME: should I query complete and fifo empty here? */
 send_pkg_err:
	return err;
}

static struct mdfld_dsi_pkg *pkg_sender_get_pkg_locked(struct
						       mdfld_dsi_pkg_sender
						       *sender)
{
	struct mdfld_dsi_pkg *pkg;

	if (list_empty(&sender->free_list)) {
		DRM_ERROR("No free pkg left\n");
		return NULL;
	}

	pkg = list_first_entry(&sender->free_list, struct mdfld_dsi_pkg, entry);

	/*detach from free list */
	list_del_init(&pkg->entry);

	return pkg;
}

static void pkg_sender_put_pkg_locked(struct mdfld_dsi_pkg_sender *sender,
				      struct mdfld_dsi_pkg *pkg)
{
	memset(pkg, 0, sizeof(struct mdfld_dsi_pkg));

	INIT_LIST_HEAD(&pkg->entry);

	list_add_tail(&pkg->entry, &sender->free_list);
}

static int mdfld_dbi_cb_init(struct mdfld_dsi_pkg_sender *sender,
			     struct psb_gtt *pg, int pipe)
{
	uint32_t phy;
	void *virt_addr = NULL;

	switch (pipe) {
	case 0:
		phy = pg->gtt_phys_start - 0x1000;
		break;
	case 2:
		phy = pg->gtt_phys_start - 0x800;
		break;
	default:
		DRM_ERROR("Unsupported channel\n");
		return -EINVAL;
	}

	/*mapping */
	virt_addr = ioremap_nocache(phy, 0x800);
	if (!virt_addr) {
		DRM_ERROR("Map DBI command buffer error\n");
		return -ENOMEM;
	}

	sender->dbi_cb_phy = phy;
	sender->dbi_cb_addr = virt_addr;

	PSB_DEBUG_ENTRY("DBI command buffer initailized. phy %x, addr %p\n",
			phy, virt_addr);

	return 0;
}

static void mdfld_dbi_cb_destroy(struct mdfld_dsi_pkg_sender *sender)
{
	PSB_DEBUG_ENTRY("\n");

	if (sender && sender->dbi_cb_addr)
		iounmap(sender->dbi_cb_addr);
}

static inline void pkg_sender_queue_pkg(struct mdfld_dsi_pkg_sender *sender,
					struct mdfld_dsi_pkg *pkg, int delay)
{
	unsigned long flags;

	spin_lock_irqsave(&sender->lock, flags);

	if (!delay) {
		send_pkg(sender, pkg);

		pkg_sender_put_pkg_locked(sender, pkg);
	} else {
		/*queue it */
		list_add_tail(&pkg->entry, &sender->pkg_list);
	}

	spin_unlock_irqrestore(&sender->lock, flags);
}

static inline void process_pkg_list(struct mdfld_dsi_pkg_sender *sender)
{
	struct mdfld_dsi_pkg *pkg;
	unsigned long flags;

	spin_lock_irqsave(&sender->lock, flags);

	while (!list_empty(&sender->pkg_list)) {
		pkg =
		    list_first_entry(&sender->pkg_list, struct mdfld_dsi_pkg,
				     entry);

		send_pkg(sender, pkg);

		list_del_init(&pkg->entry);

		pkg_sender_put_pkg_locked(sender, pkg);
	}

	spin_unlock_irqrestore(&sender->lock, flags);
}

static int mdfld_dsi_send_mcs_long(struct mdfld_dsi_pkg_sender *sender,
				   u32 *data,
				   u32 len, u8 transmission, int delay)
{
	struct mdfld_dsi_pkg *pkg;
	unsigned long flags;

	spin_lock_irqsave(&sender->lock, flags);

	pkg = pkg_sender_get_pkg_locked(sender);

	spin_unlock_irqrestore(&sender->lock, flags);

	if (!pkg) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	pkg->pkg_type = MDFLD_DSI_PKG_MCS_LONG_WRITE;
	pkg->transmission_type = transmission;
	pkg->pkg.long_pkg.data = data;
	pkg->pkg.long_pkg.len = len;

	INIT_LIST_HEAD(&pkg->entry);

	pkg_sender_queue_pkg(sender, pkg, delay);

	return 0;
}

static int mdfld_dsi_send_mcs_short(struct mdfld_dsi_pkg_sender *sender,
				    u8 cmd, u8 param, u8 param_num,
				    u8 transmission, int delay)
{
	struct mdfld_dsi_pkg *pkg;
	unsigned long flags;

	spin_lock_irqsave(&sender->lock, flags);

	pkg = pkg_sender_get_pkg_locked(sender);

	spin_unlock_irqrestore(&sender->lock, flags);

	if (!pkg) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	if (param_num) {
		pkg->pkg_type = MDFLD_DSI_PKG_MCS_SHORT_WRITE_1;
		pkg->pkg.short_pkg.param = param;
	} else {
		pkg->pkg_type = MDFLD_DSI_PKG_MCS_SHORT_WRITE_0;
		pkg->pkg.short_pkg.param = 0;
	}
	pkg->transmission_type = transmission;
	pkg->pkg.short_pkg.cmd = cmd;

	INIT_LIST_HEAD(&pkg->entry);

	pkg_sender_queue_pkg(sender, pkg, delay);

	return 0;
}

static int mdfld_dsi_send_gen_short(struct mdfld_dsi_pkg_sender *sender,
				    u8 param0, u8 param1, u8 param_num,
				    u8 transmission, int delay)
{
	struct mdfld_dsi_pkg *pkg;
	unsigned long flags;

	spin_lock_irqsave(&sender->lock, flags);

	pkg = pkg_sender_get_pkg_locked(sender);

	spin_unlock_irqrestore(&sender->lock, flags);

	if (!pkg) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	switch (param_num) {
	case 0:
		pkg->pkg_type = MDFLD_DSI_PKG_GEN_SHORT_WRITE_0;
		pkg->pkg.short_pkg.cmd = 0;
		pkg->pkg.short_pkg.param = 0;
		break;
	case 1:
		pkg->pkg_type = MDFLD_DSI_PKG_GEN_SHORT_WRITE_1;
		pkg->pkg.short_pkg.cmd = param0;
		pkg->pkg.short_pkg.param = 0;
		break;
	case 2:
		pkg->pkg_type = MDFLD_DSI_PKG_GEN_SHORT_WRITE_2;
		pkg->pkg.short_pkg.cmd = param0;
		pkg->pkg.short_pkg.param = param1;
		break;
	}

	pkg->transmission_type = transmission;

	INIT_LIST_HEAD(&pkg->entry);

	pkg_sender_queue_pkg(sender, pkg, delay);

	return 0;
}

static int mdfld_dsi_send_gen_long(struct mdfld_dsi_pkg_sender *sender,
				   u32 *data,
				   u32 len, u8 transmission, int delay)
{
	struct mdfld_dsi_pkg *pkg;
	unsigned long flags;

	spin_lock_irqsave(&sender->lock, flags);

	pkg = pkg_sender_get_pkg_locked(sender);

	spin_unlock_irqrestore(&sender->lock, flags);

	if (!pkg) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	pkg->pkg_type = MDFLD_DSI_PKG_GEN_LONG_WRITE;
	pkg->transmission_type = transmission;
	pkg->pkg.long_pkg.data = data;
	pkg->pkg.long_pkg.len = len;

	INIT_LIST_HEAD(&pkg->entry);

	pkg_sender_queue_pkg(sender, pkg, delay);

	return 0;
}

static int __read_panel_data(struct mdfld_dsi_pkg_sender *sender,
			     struct mdfld_dsi_pkg *pkg, u32 * data, u16 len)
{
	unsigned long flags;
	struct drm_device *dev = sender->dev;
	int i;
	u32 gen_data_reg;
	int retry = MDFLD_DSI_READ_MAX_COUNT;
	u8 transmission = pkg->transmission_type;

	/**
	 * do reading.
	 * 0) send out generic read request
	 * 1) polling read data avail interrupt
	 * 2) read data
	 */
	spin_lock_irqsave(&sender->lock, flags);

	REG_WRITE(sender->mipi_intr_stat_reg, BIT29);

	if ((REG_READ(sender->mipi_intr_stat_reg) & BIT29))
		DRM_ERROR("Can NOT clean read data valid interrupt\n");

	/*send out read request */
	send_pkg(sender, pkg);

	pkg_sender_put_pkg_locked(sender, pkg);

#if 0				/* FIXME MRFLD */
	/*polling read data avail interrupt */
	while (--retry && !(REG_READ(sender->mipi_intr_stat_reg) & BIT29))
		udelay(3);

	if (!retry) {
		spin_unlock_irqrestore(&sender->lock, flags);
		return -ETIMEDOUT;
	}
#endif				/* FIXME MRFLD */

	REG_WRITE(sender->mipi_intr_stat_reg, BIT29);

	/*read data */
	if (transmission == MDFLD_DSI_HS_TRANSMISSION)
		gen_data_reg = sender->mipi_hs_gen_data_reg;
	else if (transmission == MDFLD_DSI_LP_TRANSMISSION)
		gen_data_reg = sender->mipi_lp_gen_data_reg;
	else {
		DRM_ERROR("Unknown transmission");
		spin_unlock_irqrestore(&sender->lock, flags);
		return -EINVAL;
	}

	for (i = 0; i < len; i++)
		*(data + i) = REG_READ(gen_data_reg);

	spin_unlock_irqrestore(&sender->lock, flags);

	return 0;
}

static int mdfld_dsi_read_gen(struct mdfld_dsi_pkg_sender *sender,
			      u8 param0,
			      u8 param1,
			      u8 param_num,
			      u32 *data, u16 len, u8 transmission)
{
	struct mdfld_dsi_pkg *pkg;
	unsigned long flags;

	spin_lock_irqsave(&sender->lock, flags);

	pkg = pkg_sender_get_pkg_locked(sender);

	spin_unlock_irqrestore(&sender->lock, flags);

	if (!pkg) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	switch (param_num) {
	case 0:
		pkg->pkg_type = MDFLD_DSI_PKG_GEN_READ_0;
		pkg->pkg.short_pkg.cmd = 0;
		pkg->pkg.short_pkg.param = 0;
		break;
	case 1:
		pkg->pkg_type = MDFLD_DSI_PKG_GEN_READ_1;
		pkg->pkg.short_pkg.cmd = param0;
		pkg->pkg.short_pkg.param = 0;
		break;
	case 2:
		pkg->pkg_type = MDFLD_DSI_PKG_GEN_READ_2;
		pkg->pkg.short_pkg.cmd = param0;
		pkg->pkg.short_pkg.param = param1;
		break;
	}

	pkg->transmission_type = transmission;

	INIT_LIST_HEAD(&pkg->entry);

	return __read_panel_data(sender, pkg, data, len);
}

static int mdfld_dsi_read_mcs(struct mdfld_dsi_pkg_sender *sender,
			      u8 cmd, u32 *data, u16 len, u8 transmission)
{
	struct mdfld_dsi_pkg *pkg;
	unsigned long flags;

	spin_lock_irqsave(&sender->lock, flags);

	pkg = pkg_sender_get_pkg_locked(sender);

	spin_unlock_irqrestore(&sender->lock, flags);

	if (!pkg) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	pkg->pkg_type = MDFLD_DSI_PKG_MCS_READ;
	pkg->pkg.short_pkg.cmd = cmd;
	pkg->pkg.short_pkg.param = 0;

	pkg->transmission_type = transmission;

	INIT_LIST_HEAD(&pkg->entry);

	return __read_panel_data(sender, pkg, data, len);
}

static int mdfld_dsi_send_dpi_spk_pkg(struct mdfld_dsi_pkg_sender *sender,
				      u32 spk_pkg, u8 transmission)
{
	struct mdfld_dsi_pkg *pkg;
	unsigned long flags;

	spin_lock_irqsave(&sender->lock, flags);

	pkg = pkg_sender_get_pkg_locked(sender);

	spin_unlock_irqrestore(&sender->lock, flags);

	if (!pkg) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	pkg->pkg_type = MDFLD_DSI_DPI_SPK;
	pkg->transmission_type = transmission;
	pkg->pkg.spk_pkg.cmd = spk_pkg;

	INIT_LIST_HEAD(&pkg->entry);

	pkg_sender_queue_pkg(sender, pkg, 0);

	return 0;
}

void dsi_controller_dbi_init(struct mdfld_dsi_config *dsi_config, int pipe)
{
	struct drm_device *dev = dsi_config->dev;
	u32 reg_offset = pipe ? MIPIC_REG_OFFSET : 0;
	int lane_count = dsi_config->lane_count;
	u32 val = 0;

	PSB_DEBUG_ENTRY("Init DBI interface on pipe %d...\n", pipe);

	/*un-ready device */
	REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000000);

	/*init dsi adapter before kicking off */
	REG_WRITE((MIPIA_CONTROL_REG + reg_offset), 0x00000018);

	/*TODO: figure out how to setup these registers */
	REG_WRITE((MIPIA_DPHY_PARAM_REG + reg_offset), 0x150c3408);
	REG_WRITE((MIPIA_CLK_LANE_SWITCH_TIME_CNT_REG + reg_offset),
		  0x000a0014);
	REG_WRITE((MIPIA_DBI_BW_CTRL_REG + reg_offset), 0x00000400);
	REG_WRITE((MIPIA_DBI_FIFO_THROTTLE_REG + reg_offset), 0x00000001);
	REG_WRITE((MIPIA_HS_LS_DBI_ENABLE_REG + reg_offset), 0x00000000);

	/*enable all interrupts */
	REG_WRITE((MIPIA_INTR_EN_REG + reg_offset), 0xffffffff);
	/*max value: 20 clock cycles of txclkesc */
	REG_WRITE((MIPIA_TURN_AROUND_TIMEOUT_REG + reg_offset), 0x0000001f);
	/*min 21 txclkesc, max: ffffh */
	REG_WRITE((MIPIA_DEVICE_RESET_TIMER_REG + reg_offset), 0x0000ffff);
	/*min: 7d0 max: 4e20 */
	REG_WRITE((MIPIA_INIT_COUNT_REG + reg_offset), 0x00000fa0);

	/*set up max return packet size */
	REG_WRITE((MIPIA_MAX_RETURN_PACK_SIZE_REG + reg_offset),
		  MDFLD_DSI_MAX_RETURN_PACKET_SIZE);

	/*set up func_prg */
	val |= lane_count;
	val |= (dsi_config->channel_num << DSI_DBI_VIRT_CHANNEL_OFFSET);
	val |= DSI_DBI_COLOR_FORMAT_OPTION2;
	REG_WRITE((MIPIA_DSI_FUNC_PRG_REG + reg_offset), val);

	REG_WRITE((MIPIA_HS_TX_TIMEOUT_REG + reg_offset), 0x3fffff);
	REG_WRITE((MIPIA_LP_RX_TIMEOUT_REG + reg_offset), 0xffff);

	REG_WRITE((MIPIA_HIGH_LOW_SWITCH_COUNT_REG + reg_offset), 0x46);
	REG_WRITE((MIPIA_EOT_DISABLE_REG + reg_offset), 0x00000000);
	REG_WRITE((MIPIA_LP_BYTECLK_REG + reg_offset), 0x00000004);
	REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000001);
}

void dsi_controller_dpi_init(struct mdfld_dsi_config *dsi_config, int pipe)
{
	struct drm_device *dev = dsi_config->dev;
	u32 reg_offset = pipe ? MIPIC_REG_OFFSET : 0;
	int lane_count = dsi_config->lane_count;
	struct mdfld_dsi_dpi_timing dpi_timing;
	struct drm_display_mode *mode = dsi_config->mode;
	u32 val = 0;

	PSB_DEBUG_ENTRY("Init DPI interface on pipe %d...\n", pipe);

	/*un-ready device */
	REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000000);

	/*init dsi adapter before kicking off */
	REG_WRITE((MIPIA_CONTROL_REG + reg_offset), 0x00000018);

	/*enable all interrupts */
	REG_WRITE((MIPIA_INTR_EN_REG + reg_offset), 0xffffffff);

	/*set up func_prg */
	val |= lane_count;
	val |= dsi_config->channel_num << DSI_DPI_VIRT_CHANNEL_OFFSET;

	switch (dsi_config->bpp) {
	case 16:
		val |= DSI_DPI_COLOR_FORMAT_RGB565;
		break;
	case 18:
		val |= DSI_DPI_COLOR_FORMAT_RGB666;
		break;
	case 24:
		val |= DSI_DPI_COLOR_FORMAT_RGB888;
		break;
	default:
		DRM_ERROR("unsupported color format, bpp = %d\n",
			  dsi_config->bpp);
	}

	REG_WRITE((MIPIA_DSI_FUNC_PRG_REG + reg_offset), val);

	REG_WRITE((MIPIA_HS_TX_TIMEOUT_REG + reg_offset),
		  (mode->vtotal * mode->htotal * dsi_config->bpp /
		   (8 * lane_count)) & DSI_HS_TX_TIMEOUT_MASK);
	REG_WRITE((MIPIA_LP_RX_TIMEOUT_REG + reg_offset),
		  0xffff & DSI_LP_RX_TIMEOUT_MASK);

	/*max value: 20 clock cycles of txclkesc */
	REG_WRITE((MIPIA_TURN_AROUND_TIMEOUT_REG + reg_offset),
		  0x14 & DSI_TURN_AROUND_TIMEOUT_MASK);

	/*min 21 txclkesc, max: ffffh */
	REG_WRITE((MIPIA_DEVICE_RESET_TIMER_REG + reg_offset),
		  0xffff & DSI_RESET_TIMER_MASK);

	REG_WRITE((MIPIA_DPI_RESOLUTION_REG + reg_offset),
		  mode->vdisplay << 16 | mode->hdisplay);

	/*set DPI timing registers */
	mdfld_dsi_dpi_timing_calculation(mode, &dpi_timing,
					 dsi_config->lane_count,
					 dsi_config->bpp);

	REG_WRITE((MIPIA_HSYNC_COUNT_REG + reg_offset),
		  dpi_timing.hsync_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HBP_COUNT_REG + reg_offset),
		  dpi_timing.hbp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HFP_COUNT_REG + reg_offset),
		  dpi_timing.hfp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_HACTIVE_COUNT_REG + reg_offset),
		  dpi_timing.hactive_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VSYNC_COUNT_REG + reg_offset),
		  dpi_timing.vsync_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VBP_COUNT_REG + reg_offset),
		  dpi_timing.vbp_count & DSI_DPI_TIMING_MASK);
	REG_WRITE((MIPIA_VFP_COUNT_REG + reg_offset),
		  dpi_timing.vfp_count & DSI_DPI_TIMING_MASK);

	REG_WRITE((MIPIA_HIGH_LOW_SWITCH_COUNT_REG + reg_offset), 0x46);

	/*min: 7d0 max: 4e20 */
	REG_WRITE((MIPIA_INIT_COUNT_REG + reg_offset), 0x000007d0);

	/*set up video mode */
	val = 0;
	val = dsi_config->video_mode | DSI_DPI_COMPLETE_LAST_LINE;
	REG_WRITE((MIPIA_VIDEO_MODE_FORMAT_REG + reg_offset), val);

	REG_WRITE((MIPIA_EOT_DISABLE_REG + reg_offset), 0x00000000);

	REG_WRITE((MIPIA_LP_BYTECLK_REG + reg_offset), 0x00000004);

	/*TODO: figure out how to setup these registers */
	REG_WRITE((MIPIA_DPHY_PARAM_REG + reg_offset), 0x150c3408);

	REG_WRITE((MIPIA_CLK_LANE_SWITCH_TIME_CNT_REG + reg_offset),
		  (0xa << 16) | 0x14);

	/*set device ready */
	REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000001);
}

void mdfld_dsi_cmds_kick_out(struct mdfld_dsi_pkg_sender *sender)
{
	process_pkg_list(sender);
}

int mdfld_dsi_send_dcs(struct mdfld_dsi_pkg_sender *sender,
		       u8 dcs, u8 *param, u32 param_num, u8 data_src,
		       int delay)
{
	struct mdfld_dsi_pkg *pkg;
	u32 cb_phy = sender->dbi_cb_phy;
	struct drm_device *dev = sender->dev;
	u32 index = 0;
	u8 *cb = (u8 *) sender->dbi_cb_addr;
	unsigned long flags;
	int retry;
	u8 *dst = NULL;
	u32 len;
	int err = 0;

	if (!sender) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	if (!sender->dbi_pkg_support) {
		DRM_ERROR("No DBI pkg sending on this sender\n");
		return -ENOTSUPP;
	}

	if (param_num > MDFLD_MAX_DCS_PARAM) {
		DRM_ERROR("Sender only support up to %d DCS params\n",
			  MDFLD_MAX_DCS_PARAM);
		return -EINVAL;
	}

	/*if dcs is write_mem_start, send it directly
	 * using DSI adapter interface */
	if (dcs == write_mem_start) {
		if (!spin_trylock(&sender->lock))
			return -EAGAIN;

#if 0				/* FIXME MRFLD */
		/**
		 * query whether DBI FIFO is empty,
		 * if not wait it becoming empty
		 */
		retry = MDFLD_DSI_DBI_FIFO_TIMEOUT;
		while (retry
		       && !(REG_READ(sender->mipi_gen_fifo_stat_reg) & BIT27)) {
			udelay(500);
			retry--;
		}

		/*if DBI FIFO timeout, drop this frame */
		if (!retry) {
			spin_unlock(&sender->lock);
			return 0;
		}
#endif				/* FIXME MRFLD */

		*(cb + (index++)) = write_mem_start;

		REG_WRITE(sender->mipi_cmd_len_reg, 1);
		REG_WRITE(sender->mipi_cmd_addr_reg, cb_phy | BIT0 | BIT1);

#if 0				/* FIXME MRFLD */
		retry = MDFLD_DSI_DBI_FIFO_TIMEOUT;
		while (retry && (REG_READ(sender->mipi_cmd_addr_reg) & BIT0)) {
			udelay(1);
			retry--;
		}
#endif				/* FIXME MRFLD */

		spin_unlock(&sender->lock);
		return 0;
	}

	/*get a free pkg */
	spin_lock_irqsave(&sender->lock, flags);

	pkg = pkg_sender_get_pkg_locked(sender);

	spin_unlock_irqrestore(&sender->lock, flags);

	if (!pkg) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	dst = pkg->pkg.dcs_pkg.param;
	memcpy(dst, param, param_num);

	pkg->pkg_type = MDFLD_DSI_PKG_DCS;
	pkg->transmission_type = MDFLD_DSI_DCS;
	pkg->pkg.dcs_pkg.cmd = dcs;
	pkg->pkg.dcs_pkg.param_num = param_num;
	pkg->pkg.dcs_pkg.data_src = data_src;

	INIT_LIST_HEAD(&pkg->entry);

	if (param_num == 0)
		return mdfld_dsi_send_mcs_short_hs(sender, dcs, 0, 0, delay);
	else if (param_num == 1)
		return mdfld_dsi_send_mcs_short_hs(sender, dcs, param[0], 1,
						   delay);
	else if (param_num > 1) {
		len = (param_num + 1) / 4;
		if ((param_num + 1) % 4)
			len++;

		return mdfld_dsi_send_mcs_long_hs(sender,
						  (u32 *) &pkg->pkg.dcs_pkg,
						  len, delay);

	}

	return err;
}

int mdfld_dsi_send_mcs_short_hs(struct mdfld_dsi_pkg_sender *sender,
				u8 cmd, u8 param, u8 param_num, int delay)
{
	if (!sender) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_mcs_short(sender, cmd, param, param_num,
					MDFLD_DSI_HS_TRANSMISSION, delay);
}

int mdfld_dsi_send_mcs_short_lp(struct mdfld_dsi_pkg_sender *sender,
				u8 cmd, u8 param, u8 param_num, int delay)
{
	if (!sender) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_mcs_short(sender, cmd, param, param_num,
					MDFLD_DSI_LP_TRANSMISSION, delay);
}

int mdfld_dsi_send_mcs_long_hs(struct mdfld_dsi_pkg_sender *sender,
			       u32 *data, u32 len, int delay)
{
	if (!sender || !data || !len) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_mcs_long(sender, data, len,
				       MDFLD_DSI_HS_TRANSMISSION, delay);
}

int mdfld_dsi_send_mcs_long_lp(struct mdfld_dsi_pkg_sender *sender,
			       u32 *data, u32 len, int delay)
{
	if (!sender || !data || !len) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_mcs_long(sender, data, len,
				       MDFLD_DSI_LP_TRANSMISSION, delay);
}

int mdfld_dsi_send_gen_short_hs(struct mdfld_dsi_pkg_sender *sender,
				u8 param0, u8 param1, u8 param_num, int delay)
{
	if (!sender) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_gen_short(sender, param0, param1, param_num,
					MDFLD_DSI_HS_TRANSMISSION, delay);
}

int mdfld_dsi_send_gen_short_lp(struct mdfld_dsi_pkg_sender *sender,
				u8 param0, u8 param1, u8 param_num, int delay)
{
	if (!sender || param_num < 0 || param_num > 2) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_gen_short(sender, param0, param1, param_num,
					MDFLD_DSI_LP_TRANSMISSION, delay);
}

int mdfld_dsi_send_gen_long_hs(struct mdfld_dsi_pkg_sender *sender,
			       u32 *data, u32 len, int delay)
{
	if (!sender || !data || !len) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_gen_long(sender, data, len,
				       MDFLD_DSI_HS_TRANSMISSION, delay);
}

int mdfld_dsi_send_gen_long_lp(struct mdfld_dsi_pkg_sender *sender,
			       u32 *data, u32 len, int delay)
{
	if (!sender || !data || !len) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_gen_long(sender, data, len,
				       MDFLD_DSI_LP_TRANSMISSION, delay);
}

int mdfld_dsi_read_gen_hs(struct mdfld_dsi_pkg_sender *sender,
			  u8 param0,
			  u8 param1, u8 param_num, u32 *data, u16 len)
{
	if (!sender || !data || param_num < 0 ||
	    param_num > 2 || !data || !len) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_read_gen(sender, param0, param1, param_num,
				  data, len, MDFLD_DSI_HS_TRANSMISSION);

}

int mdfld_dsi_read_gen_lp(struct mdfld_dsi_pkg_sender *sender,
			  u8 param0,
			  u8 param1, u8 param_num, u32 *data, u16 len)
{
	if (!sender || !data || param_num < 0 ||
	    param_num > 2 || !data || !len) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_read_gen(sender, param0, param1, param_num,
				  data, len, MDFLD_DSI_LP_TRANSMISSION);
}

int mdfld_dsi_read_mcs_hs(struct mdfld_dsi_pkg_sender *sender,
			  u8 cmd, u32 *data, u16 len)
{
	if (!sender || !data || !len) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_read_mcs(sender, cmd, data, len,
				  MDFLD_DSI_HS_TRANSMISSION);
}

int mdfld_dsi_read_mcs_lp(struct mdfld_dsi_pkg_sender *sender,
			  u8 cmd, u32 *data, u16 len)
{
	if (!sender || !data || !len) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_read_mcs(sender, cmd, data, len,
				  MDFLD_DSI_LP_TRANSMISSION);
}

int mdfld_dsi_send_dpi_spk_pkg_hs(struct mdfld_dsi_pkg_sender *sender,
				  u32 spk_pkg)
{
	if (!sender) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_dpi_spk_pkg(sender, spk_pkg,
					  MDFLD_DSI_HS_TRANSMISSION);
}

int mdfld_dsi_send_dpi_spk_pkg_lp(struct mdfld_dsi_pkg_sender *sender,
				  u32 spk_pkg)
{
	if (!sender) {
		DRM_ERROR("Invalid parameters\n");
		return -EINVAL;
	}

	return mdfld_dsi_send_dpi_spk_pkg(sender, spk_pkg,
					  MDFLD_DSI_LP_TRANSMISSION);
}

int mdfld_dsi_pkg_sender_init(struct mdfld_dsi_connector *dsi_connector,
			      int pipe)
{
	int ret;
	struct mdfld_dsi_pkg_sender *pkg_sender;
	struct mdfld_dsi_config *dsi_config =
	    mdfld_dsi_get_config(dsi_connector);
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_gtt *pg = dev_priv->pg;
	int i;
	struct mdfld_dsi_pkg *pkg, *tmp;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_connector) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	pkg_sender = dsi_connector->pkg_sender;

	if (!pkg_sender || IS_ERR(pkg_sender)) {
		pkg_sender =
		    kzalloc(sizeof(struct mdfld_dsi_pkg_sender), GFP_KERNEL);
		if (!pkg_sender) {
			DRM_ERROR("Create DSI pkg sender failed\n");
			return -ENOMEM;
		}

		dsi_connector->pkg_sender = (void *)pkg_sender;
	}

	pkg_sender->dev = dev;
	pkg_sender->dsi_connector = dsi_connector;
	pkg_sender->pipe = pipe;
	pkg_sender->pkg_num = 0;
	pkg_sender->panel_mode = 0;
	pkg_sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	/*int dbi command buffer */
	if (dsi_config->type == MDFLD_DSI_ENCODER_DBI) {
		pkg_sender->dbi_pkg_support = 1;
		ret = mdfld_dbi_cb_init(pkg_sender, pg, pipe);
		if (ret) {
			DRM_ERROR("DBI command buffer map failed\n");
			goto mapping_err;
		}
	}

	/*init regs */
	if (pipe == 0) {
		pkg_sender->dpll_reg = MRST_DPLL_A;
		pkg_sender->dspcntr_reg = DSPACNTR;
		pkg_sender->pipeconf_reg = PIPEACONF;
		pkg_sender->dsplinoff_reg = DSPALINOFF;
		pkg_sender->dspsurf_reg = DSPASURF;
		pkg_sender->pipestat_reg = PIPEASTAT;

		pkg_sender->mipi_intr_stat_reg = MIPIA_INTR_STAT_REG;
		pkg_sender->mipi_lp_gen_data_reg = MIPIA_LP_GEN_DATA_REG;
		pkg_sender->mipi_hs_gen_data_reg = MIPIA_HS_GEN_DATA_REG;
		pkg_sender->mipi_lp_gen_ctrl_reg = MIPIA_LP_GEN_CTRL_REG;
		pkg_sender->mipi_hs_gen_ctrl_reg = MIPIA_HS_GEN_CTRL_REG;
		pkg_sender->mipi_gen_fifo_stat_reg = MIPIA_GEN_FIFO_STAT_REG;
		pkg_sender->mipi_data_addr_reg = MIPIA_DATA_ADD_REG;
		pkg_sender->mipi_data_len_reg = MIPIA_DATA_LEN_REG;
		pkg_sender->mipi_cmd_addr_reg = MIPIA_CMD_ADD_REG;
		pkg_sender->mipi_cmd_len_reg = MIPIA_CMD_LEN_REG;
		pkg_sender->mipi_dpi_control_reg = MIPIA_DPI_CONTROL_REG;
	} else if (pipe == 2) {
		pkg_sender->dpll_reg = MRST_DPLL_A;
		pkg_sender->dspcntr_reg = DSPCCNTR;
		pkg_sender->pipeconf_reg = PIPECCONF;
		pkg_sender->dsplinoff_reg = DSPCLINOFF;
		pkg_sender->dspsurf_reg = DSPCSURF;
		pkg_sender->pipestat_reg = 72024;

		pkg_sender->mipi_intr_stat_reg =
		    MIPIA_INTR_STAT_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_lp_gen_data_reg =
		    MIPIA_LP_GEN_DATA_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_hs_gen_data_reg =
		    MIPIA_HS_GEN_DATA_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_lp_gen_ctrl_reg =
		    MIPIA_LP_GEN_CTRL_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_hs_gen_ctrl_reg =
		    MIPIA_HS_GEN_CTRL_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_gen_fifo_stat_reg =
		    MIPIA_GEN_FIFO_STAT_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_data_addr_reg =
		    MIPIA_DATA_ADD_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_data_len_reg =
		    MIPIA_DATA_LEN_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_cmd_addr_reg =
		    MIPIA_CMD_ADD_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_cmd_len_reg =
		    MIPIA_CMD_LEN_REG + MIPIC_REG_OFFSET;
		pkg_sender->mipi_dpi_control_reg =
		    MIPIA_DPI_CONTROL_REG + MIPIC_REG_OFFSET;
	}

	/*init pkg list */
	INIT_LIST_HEAD(&pkg_sender->pkg_list);
	INIT_LIST_HEAD(&pkg_sender->free_list);

	/*init lock */
	spin_lock_init(&pkg_sender->lock);

	/*allocate free pkg pool */
	for (i = 0; i < MDFLD_MAX_PKG_NUM; i++) {
		pkg = kzalloc(sizeof(struct mdfld_dsi_pkg), GFP_KERNEL);
		if (!pkg) {
			ret = -ENOMEM;
			goto pkg_alloc_err;
		}

		INIT_LIST_HEAD(&pkg->entry);

		/*append to free list */
		list_add_tail(&pkg->entry, &pkg_sender->free_list);
	}

	PSB_DEBUG_ENTRY("initialized\n");

	return 0;

 pkg_alloc_err:
	list_for_each_entry_safe(pkg, tmp, &pkg_sender->free_list, entry) {
		list_del(&pkg->entry);
		kfree(pkg);
	}

	/*free mapped command buffer */
	mdfld_dbi_cb_destroy(pkg_sender);
 mapping_err:
	kfree(pkg_sender);
	dsi_connector->pkg_sender = NULL;

	return ret;
}

void mdfld_dsi_pkg_sender_destroy(struct mdfld_dsi_pkg_sender *sender)
{
	struct mdfld_dsi_pkg *pkg, *tmp;

	if (!sender || IS_ERR(sender))
		return;

	/*free pkg pool */
	list_for_each_entry_safe(pkg, tmp, &sender->free_list, entry) {
		list_del(&pkg->entry);
		kfree(pkg);
	}

	/*free pkg list */
	list_for_each_entry_safe(pkg, tmp, &sender->pkg_list, entry) {
		list_del(&pkg->entry);
		kfree(pkg);
	}

	/*free mapped command buffer */
	mdfld_dbi_cb_destroy(sender);

	/*free */
	kfree(sender);

	PSB_DEBUG_ENTRY("destroyed\n");
}
