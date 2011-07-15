#ifndef DW_SPI_HEADER_H
#define DW_SPI_HEADER_H

#include <linux/io.h>
#include <linux/scatterlist.h>

/* Bit fields in CTRLR0 */
#define SPI_DFS_OFFSET			0
#define SPI_DFS_MASK			0xf

#define SPI_FRF_OFFSET			4
#define SPI_FRF_SPI			0x0
#define SPI_FRF_SSP			0x1
#define SPI_FRF_MICROWIRE		0x2
#define SPI_FRF_RESV			0x3

#define SPI_MODE_OFFSET			6
#define SPI_SCPH_OFFSET			6
#define SPI_SCOL_OFFSET			7
#define SPI_MODE_MASK			(0x3 << SPI_MODE_OFFSET)

#define SPI_TMOD_OFFSET			8
#define SPI_TMOD_MASK			(0x3 << SPI_TMOD_OFFSET)
#define	SPI_TMOD_TR			0x0		/* xmit & recv */
#define SPI_TMOD_TO			0x1		/* xmit only */
#define SPI_TMOD_RO			0x2		/* recv only */
#define SPI_TMOD_EPROMREAD		0x3		/* eeprom read mode */

#define SPI_SLVOE_OFFSET		10
#define SPI_SRL_OFFSET			11
#define SPI_CFS_OFFSET			12

/* Bit fields in SR, 7 bits */
#define SR_MASK				0x7f		/* cover 7 bits */
#define SR_BUSY				(1 << 0)
#define SR_TF_NOT_FULL			(1 << 1)
#define SR_TF_EMPT			(1 << 2)
#define SR_RF_NOT_EMPT			(1 << 3)
#define SR_RF_FULL			(1 << 4)
#define SR_TX_ERR			(1 << 5)
#define SR_DCOL				(1 << 6)

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define SPI_INT_TXEI			(1 << 0)
#define SPI_INT_TXOI			(1 << 1)
#define SPI_INT_RXUI			(1 << 2)
#define SPI_INT_RXOI			(1 << 3)
#define SPI_INT_RXFI			(1 << 4)
#define SPI_INT_MSTI			(1 << 5)
#define SPI_INT_ALL  0x3f

struct spi_dw_reg {
	u32	ctrl0;
	u32	ctrl1;
	u32	ssienr;
	u32	mwcr;
	u32	ser;
	u32	baudr;
	u32	txfltr;
	u32	rxfltr;
	u32	txflr;
	u32	rxflr;
	u32	sr;
	u32	imr;
	u32	isr;
	u32	risr;
	u32	txoicr;
	u32	rxoicr;
	u32	rxuicr;
	u32	msticr;
	u32	icr;
	u32	dmacr;
	u32	dmatdlr;
	u32	dmardlr;
	u32	idr;
	u32	version;
	u32	dr;		/* Currently oper as 32 bits,
				though only low 16 bits matters */
} __packed;

struct spi_dw;
struct spi_dw_dma_ops {
	int (*dma_init)(struct spi_dw *dws);
	void (*dma_exit)(struct spi_dw *dws);
	int (*dma_transfer)(struct spi_dw *dws);
};

enum xfer_type {
	PIO_XFER,
	INT_XFER,
	DMA_XFER,
};

struct xfer_state {
	const u8 *tx_buf;
	dma_addr_t tx_dma;
	u8 *rx_buf;
	dma_addr_t rx_dma;
	struct spi_message *msg;
	u32 n_bytes;
	u32 len;
	u32 sent;
	u32 rcvd;
	u32 err;
	u32 type;
	u32  tx_threshold;
	u32  rx_threshold;
	u32 irq_status;
	struct completion complete;
};

struct spi_dw {
	struct spi_master	*master;
	struct device		*parent_dev;

	void __iomem		*regs;
	unsigned long		paddr;
	u32			iolen;
	u32			irq;
	u32			fifo_len;	/* depth of the FIFO buffer */
	u32			max_freq;	/* max bus freq supported */

	u16			bus_num;
	u16			num_cs;		/* supported slave numbers */

	/* Driver message queue */
	struct workqueue_struct	*workqueue;
	struct work_struct	pump_messages;
	spinlock_t		lock;
	struct list_head	queue;
	u32			run;

	/* Current message transfer state info */
	struct xfer_state       xfer;

	/* Dma info */
	u32			dma_inited;
	struct dma_chan		*txchan;
	struct scatterlist	tx_sgl;
	struct dma_chan		*rxchan;
	struct scatterlist	rx_sgl;
	u32			dma_chan_done;
	struct device		*dma_dev;
	dma_addr_t		dma_addr; /* phy address of the Data register */
	struct spi_dw_dma_ops	*dma_ops;
	void			*dma_priv; /* platform relate info */
	struct pci_dev		*dmac;

	/* Bus interface info */
	void			*priv;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
#endif
};

#define dw_readl(dw, name) \
	__raw_readl(&(((struct spi_dw_reg *)dw->regs)->name))
#define dw_writel(dw, name, val) \
	__raw_writel((val), &(((struct spi_dw_reg *)dw->regs)->name))
#define dw_readw(dw, name) \
	__raw_readw(&(((struct spi_dw_reg *)dw->regs)->name))
#define dw_writew(dw, name, val) \
	__raw_writew((val), &(((struct spi_dw_reg *)dw->regs)->name))
#define dw_readb(dw, name) \
	__raw_readb(&(((struct spi_dw_reg *)dw->regs)->name))
#define dw_writeb(dw, name, val) \
	__raw_writeb((val), &(((struct spi_dw_reg *)dw->regs)->name))

static inline void spi_dw_disable(struct spi_dw *dws)
{
	dw_writel(dws, ssienr, 0);
}

static inline void spi_dw_enable(struct spi_dw *dws)
{
	dw_writel(dws, ssienr, 1);
}

static inline void spi_dw_set_clk(struct spi_dw *dws, u16 div)
{
	dw_writel(dws, baudr, div);
}

static inline void spi_dw_chip_sel(struct spi_dw *dws, u16 cs)
{
	if (cs > dws->master->num_chipselect)
		return;

	dw_writel(dws, ser, 1 << cs);
}

/* Disable IRQ bits */
static inline void spi_dw_mask_intr(struct spi_dw *dws, u32 mask)
{
	u32 new_mask;

	new_mask = dw_readl(dws, imr) & ~mask;
	dw_writel(dws, imr, new_mask);
}

/* Enable IRQ bits */
static inline void spi_dw_umask_intr(struct spi_dw *dws, u32 mask)
{
	u32 new_mask;

	new_mask = dw_readl(dws, imr) | mask;
	dw_writel(dws, imr, new_mask);
}

extern int spi_dw_add_host(struct spi_dw *dws);
extern void spi_dw_remove_host(struct spi_dw *dws);
extern int spi_dw_suspend_host(struct spi_dw *dws);
extern int spi_dw_resume_host(struct spi_dw *dws);
extern void spi_dw_xfer_done(struct spi_dw *dws);
extern int spi_dw_stop_queue(struct spi_dw *dws);

/* platform related setup */
extern int spi_dw_mid_init(struct spi_dw *dws); /* Intel MID platforms */
#endif /* DW_SPI_HEADER_H */
