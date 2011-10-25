/*
  * <Driver for I2S protocol on SSP (Moorestown and Medfield hardware)>
  * Copyright (c) 2010, Intel Corporation.
  * Louis LE GALL <louis.le.gall intel.com>
  *
  * This program is free software; you can redistribute it and/or modify it
  * under the terms and conditions of the GNU General Public License,
  * version 2, as published by the Free Software Foundation.
  *
  * This program is distributed in the hope it will be useful, but WITHOUT
  * ANY WARRANTY; without evenp the implied warranty of MERCHANTABILITY or
  * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  * more details.
  *
  * You should have received a copy of the GNU General Public License along with
  * this program; if not, write to the Free Software Foundation, Inc.,
  * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  */
#ifndef MID_I2S_H_
#define MID_I2S_H_

#include <linux/intel_mid_i2s_common.h>
#include <linux/intel_mid_i2s_if.h>


/*
 * Defines
 */
#define DRIVER_NAME "I2S SSP Driver"

#define MRST_SSP0_DEVICE_ID	0x0815	/* FOR MRST */
#define MFLD_SSP0_DEVICE_ID	0x0832	/* FOR MFLD */
#define MFLD_SSP1_DEVICE_ID	0x0825	/* FOR MFLD */
#define CLV_SSP0_DEVICE_ID	0x08F1	/* For Cloverview support */
#define CLV_SSP1_DEVICE_ID	0x08E8	/* For Cloverview support */

#define MRST_LPE_DMA_DEVICE_ID	0x0814
#define MFLD_LPE_DMA_DEVICE_ID	0x0830
#define CLV_LPE_DMA_DEVICE_ID	0x08F0

/* SSP1 PCI device Base Address Register */
#define MRST_SSP_BAR	0
#define MRST_LPE_BAR	1
#define DMA1C_DEVICE_INSTANCE_SSP0 0
#define DMA1C_DEVICE_INSTANCE_SSP1 1
#define OFFSET_SSCR0	0x00
#define OFFSET_SSCR1	0x04
#define OFFSET_SSSR		0x08
#define OFFSET_SSITR	0x0c
#define OFFSET_SSDR		0x10
#define OFFSET_SSTO		0x28
#define OFFSET_SSPSP	0x2c
#define OFFSET_SSTSA	0x30	/* SSP Tx Timeslot Active */
#define OFFSET_SSRSA	0x34	/* SSP Rx Timeslot Active */
/* SST register map */
#define OFFSET_LPE_CSR			0x00
#define OFFSET_LPE_PISR			0x08
#define OFFSET_LPE_PIMR			0x10
#define OFFSET_LPE_ISRX			0x18
#define OFFSET_LPE_IMRX			0x28
#define OFFSET_LPE_IPCX			0x38	/* IPC IA-SST */
#define OFFSET_LPE_IPCD			0x40	/* IPC SST-IA */
#define OFFSET_LPE_ISRD			0x20	/* dummy register for*/
						/* shim workaround   */
#define OFFSET_LPE_SHIM_SIZE	0X44

#define SSP_IN_MASTER_MODE		0x0
#define SSP_IN_SLAVE_MODE		0x1

/*
 *	Macros
 */
#define DEFINE_SSP_REG(reg, off) \
static inline u32 read_##reg(void *p) { return __raw_readl(p + (off)); } \
static inline void write_##reg(u32 v, void *p) { __raw_writel(v, p + (off)); }
DEFINE_SSP_REG(SSCR0, 0x00)
DEFINE_SSP_REG(SSCR1, 0x04)
DEFINE_SSP_REG(SSSR, 0x08)
DEFINE_SSP_REG(SSITR, 0x0c)
DEFINE_SSP_REG(SSDR, 0x10)
DEFINE_SSP_REG(SSTO, 0x28)
DEFINE_SSP_REG(SSPSP, 0x2c)
DEFINE_SSP_REG(SSTSA, 0x30)
DEFINE_SSP_REG(SSRSA, 0x34)
DEFINE_SSP_REG(SSTSS, 0x38)
DEFINE_SSP_REG(SSACD, 0x3C)
DEFINE_SSP_REG(I2CCTRL, 0x00);
DEFINE_SSP_REG(I2CDATA, 0x04);
/*
 * Langwell SSP serial port register definitions
 */
#define SSCR0_DSS_MASK   0x0F	/* Data Size Select [4..16] */
#define SSCR0_DSS_SHIFT  0
#define SSCR0_FRF_MASK   0x03	/* FRame Format */
#define SSCR0_FRF_SHIFT  4
#define SSCR0_ECS_MASK   0x01	/* External clock select */
#define SSCR0_ECS_SHIFT  6
#define SSCR0_SSE_MASK   0x01	/* Synchronous Serial Port Enable */
#define SSCR0_SSE_SHIFT  7
#define SSCR0_SCR_MASK   0xFFF	/* Not implemented */
#define SSCR0_SCR_SHIFT  8
#define SSCR0_EDSS_MASK  0x1	/* Extended data size select */
#define SSCR0_EDSS_SHIFT 20
#define SSCR0_NCS_MASK   0x1	/* Network clock select */
#define SSCR0_NCS_SHIFT  21
#define SSCR0_RIM_MASK   0x1	/* Receive FIFO overrrun int mask */
#define SSCR0_RIM_SHIFT  22
#define SSCR0_TIM_MASK   0x1	/* Transmit FIFO underrun int mask */
#define SSCR0_TIM_SHIFT  23
#define SSCR0_FRDC_MASK  0x7	/* Frame Rate Divider Control */
#define SSCR0_FRDC_SHIFT 24
#define SSCR0_ACS_MASK   0x1	/* Audio clock select */
#define SSCR0_ACS_SHIFT  30
#define SSCR0_MOD_MASK   0x1	/* Mode (normal or network) */
#define SSCR0_MOD_SHIFT  31

#define SSCR0_DataSize(x)     ((x) - 1)	/* Data Size Select [4..16] */
#define SSCR0_SlotsPerFrm(x)  ((x) - 1)	/* Time slots per frame */
#define SSCR0_SerClkDiv(x)    ((x) - 1)	/* Divisor [1..4096],... */
					 /*...not implemented on Langwell */
#define SSCR1_TTELP_MASK     0x1	/* TXD Tristate Enable on Last Phase */
#define SSCR1_TTELP_SHIFT    31
#define SSCR1_TTE_MASK	     0x1	/* TXD Tristate Enable */
#define SSCR1_TTE_SHIFT      30
#define SSCR1_EBCEI_MASK     0x1	/* Enable Bit Count Error Interrupt */
#define SSCR1_EBCEI_SHIFT    29
#define SSCR1_SCFR_MASK      0x1	/* Slave Clock Running */
#define SSCR1_SCFR_SHIFT     28
#define SSCR1_ECRA_MASK      0x1	/* Enable Clock Request A */
#define SSCR1_ECRA_SHIFT     27
#define SSCR1_ECRB_MASK      0x1	/* Enable Clock Request B */
#define SSCR1_ECRB_SHIFT     26
#define SSCR1_SCLKDIR_MASK   0x1	/* SSPCLK Direction */
#define SSCR1_SCLKDIR_SHIFT  25
#define SSCR1_SFRMDIR_MASK   0x1	/* SSPFRM Direction */
#define SSCR1_SFRMDIR_SHIFT  24
#define SSCR1_RWOT_MASK      0x1	/* Receive without Transmit */
#define SSCR1_RWOT_SHIFT     23
#define SSCR1_TRAIL_MASK     0x1	/* Trailing Byte */
#define SSCR1_TRAIL_SHIFT    22
#define SSCR1_TSRE_MASK      0x1	/* DMA Transmit Service Request Enable*/
#define SSCR1_TSRE_SHIFT     21
#define SSCR1_RSRE_MASK      0x1	/* DMA Receive Service Request Enable */
#define SSCR1_RSRE_SHIFT     20
#define SSCR1_TINTE_MASK     0x1	/* Receiver Time-out Interrupt Enable */
#define SSCR1_TINTE_SHIFT    19
#define SSCR1_PINTE_MASK     0x1	/* Periph. Trailing Byte Int. Enable */
#define SSCR1_PINTE_SHIFT    18
#define SSCR1_IFS_MASK       0x1	/* Invert Frame Signal */
#define SSCR1_IFS_SHIFT      16
#define SSCR1_STFR_MASK      0x1	/* Select FIFO for EFWR: test mode */
#define SSCR1_STFR_SHIFT     15
#define SSCR1_EFWR_MASK      0x1	/* Enable FIFO Write/Read: test mode */
#define SSCR1_EFWR_SHIFT     14
#define SSCR1_RFT_MASK       0xF	/* Receive FIFO Trigger Threshold */
#define SSCR1_RFT_SHIFT      10
#define SSCR1_TFT_MASK       0xF	/* Transmit FIFO Trigger Threshold */
#define SSCR1_TFT_SHIFT      6
#define SSCR1_MWDS_MASK      0x1	/* Microwire Transmit Data Size */
#define SSCR1_MWDS_SHIFT     5
#define SSCR1_SPH_MASK       0x1	/* Motorola SPI SSPSCLK phase setting */
#define SSCR1_SPH_SHIFT      4
#define SSCR1_SPO_MASK       0x1	/* Motorola SPI SSPSCLK polarity */
#define SSCR1_SPO_SHIFT      3
#define SSCR1_LBM_MASK       0x1	/* Loopback mode: test mode */
#define SSCR1_LBM_SHIFT      2
#define SSCR1_TIE_MASK       0x1	/* Transmit FIFO Interrupt Enable */
#define SSCR1_TIE_SHIFT      1
#define SSCR1_RIE_MASK       0x1	/* Receive FIFO Interrupt Enable */
#define SSCR1_RIE_SHIFT      0

#define SSCR1_RxTresh(x) ((x) - 1)	/* level [1..16] */
#define SSCR1_TxTresh(x) ((x) - 1)	/* level [1..16] */

#define SSPSP_FSRT_MASK      0x1	/* Frame Sync Relative Timing Bit */
#define SSPSP_FSRT_SHIFT     25
#define SSPSP_DMYSTOP_MASK   0x3	/* Dummy Stop in Number of SSPSCLKs:T4*/
#define SSPSP_DMYSTOP_SHIFT  23
#define SSPSP_SFRMWDTH_MASK  0x3F	/* Serial Frame width : T6 */
#define SSPSP_SFRMWDTH_SHIFT 16
#define SSPSP_SFRMDLY_MASK   0x7F	/* Serial Fr. Delay in 1/2SSPSCLKs:T5 */
#define SSPSP_SFRMDLY_SHIFT  9
#define SSPSP_DMYSTRT_MASK   0x3	/* Dummy Start in Number of SSPSCLKs..*/
#define SSPSP_DMYSTRT_SHIFT  7	    /*...after STRTDLY, T2 (master mode only) */
#define SSPSP_STRTDLY_MASK   0x7	/* Start Delay, T1 (master mode only) */
#define SSPSP_STRTDLY_SHIFT  4
#define SSPSP_ETDS_MASK      0x1	/* End of Transfer Data State */
#define SSPSP_ETDS_SHIFT     3
#define SSPSP_SFRMP_MASK     0x1	/* Serial Frame Polarity */
#define SSPSP_SFRMP_SHIFT    2
#define SSPSP_SCMODE_MASK    0x3	/* Serial bit-rate Clock Mode */
#define SSPSP_SCMODE_SHIFT   0

#define SSTSA_TTSA_MASK      0xFF
#define SSTSA_TTSA_SHIFT     0

#define SSRSA_RTSA_MASK      0xFF
#define SSRSA_RTSA_SHIFT     0

#define SSSR_BCE_MASK   0x1	/* Bit Count Error: Read/Write 1 to Clear */
#define SSSR_BCE_SHIFT  23
#define SSSR_CSS_MASK   0x1	/* Clock Synchronization Status */
#define SSSR_CSS_SHIFT  22
#define SSSR_TUR_MASK   0x1	/* Transmit FIFO UnderRun: Rd/Wr 1 to Clear */
#define SSSR_TUR_SHIFT  21
#define SSSR_EOC_MASK   0x1	/* End Of Chain: Read/Write 1 to Clear */
#define SSSR_EOC_SHIFT  20
#define SSSR_TINT_MASK  0x1	/* Receiver Time-out Interrupt:... */
#define SSSR_TINT_SHIFT 19	/* ...Read/Write 1 to Clear */
#define SSSR_PINT_MASK  0x1	/* Peripheral Trailing Byte Interrupt:... */
#define SSSR_PINT_SHIFT 18	/* ...Read/Write 1 to Clear */
#define SSSR_RFL_MASK   0xF	/* Receive FIFO Level */
#define SSSR_RFL_SHIFT  12
#define SSSR_TFL_MASK   0xF	/* Transmit FIFO Level */
#define SSSR_TFL_SHIFT  8
#define SSSR_ROR_MASK   0x1	/* Receive FIFO Overrun: Read/Write 1 to Clear*/
#define SSSR_ROR_SHIFT  7
#define SSSR_RFS_MASK   0x1	/* Receive FIFO Service Request */
#define SSSR_RFS_SHIFT  6
#define SSSR_TFS_MASK   0x1	/* Transmit FIFO Service Request */
#define SSSR_TFS_SHIFT  5
#define SSSR_BSY_MASK   0x1	/* SSP Busy */
#define SSSR_BSY_SHIFT  4
#define SSSR_RNE_MASK   0x1	/* Receive FIFO not empty */
#define SSSR_RNE_SHIFT  3
#define SSSR_TNF_MASK   0x1	/* Transmit FIFO not Full */
#define SSSR_TNF_SHIFT  2


#define SSP_OFF 0
#define SSP_ON  1


/*
 *	list of differents types of SSP, value depends of adid entry of
 *	capability ID of the PCI
 */

/*
 *
 * The PCI header associated to SSP devices now includes a configuration
 * register. It provides information to a driver which is probed for the
 * SSP, specifying in which way the SSP is supposed to be used. Here is
 * the format of this byte register:
 *
 *	bits 1..0: Mode
 *		00=0x0 : Invalid, the register should be ignored
 *		01=0x1 : SSP to be used as SPI controller
 *		10=0x2: SSP to be used in I2S/ISS mode
 *		other: Reserved
 *
 *	bits 4..2: Configuration
 *	In I2S/ISS mode:
 *		000=0x0: Invalid
 *		001=0x1: Bluetooth
 *		010=0x2: Modem
 *		other: Reserved
 *	In SPI mode:
 *		Value is the SPI bus number connected to the SSP.
 *		To be used for registration to the Linux SPI
 *		framework.
 *	bit 5: SPI slave
 *	Relevant in SPI mode only. If set, indicates the SPI clock
 *	is not provided by the SSP: SPI slave mode.
 *
 *	bit 6..13: SSP_FS pin GPIO Mapping
 *	bit 14..15: SSP_FS pin Mode
 *
 * This configuration register is implemented in the adid field of the
 * Vendor Specific PCI capability associated to the SSP.
 *
 */

#define PCI_ADID_SSP_MODE_SPI  (1)
#define PCI_ADID_SSP_MODE_I2S  (2)

#define PCI_ADID_SSP_CONF_BT_FM  (1<<2)
#define PCI_ADID_SSP_CONF_MODEM  (2<<2)

#define PCI_ADID_SSP_FS_GPIO_MAPPING_SHIFT 6
#define PCI_ADID_SSP_FS_GPIO_MAPPING_MASK 0xFF

#define PCI_ADID_SSP_FS_GPIO_MODE_SHIFT 14
#define PCI_ADID_SSP_FS_GPIO_MODE_MASK 0x3



#define PCI_CAP_ADID_I2S_BT_FM  ((PCI_ADID_SSP_CONF_BT_FM) | \
					 (PCI_ADID_SSP_MODE_I2S))
#define PCI_CAP_ADID_I2S_MODEM  ((PCI_ADID_SSP_CONF_MODEM) | \
					 (PCI_ADID_SSP_MODE_I2S))

/* bit I2S_PORT_OPENED lock for open/close
 * bit I2S_PORT_READ_BUSY lock for read requests (serialized)
 * bit I2S_PORT_WRITE_BUSY lock for write requests (serialized)
 * bit I2S_PORT_CLOSING means close on going, waiting for pending callbacks.
 * bit I2S_PORT_COMPLETE_WRITE means deferred irq process is required
 *			to complete the Tx request when not using DMA
 * bit I2S_PORT_COMPLETE_READ means deferred irq process is required
 *			to complete the Rx request when not using DMA
 */
enum i2s_flags {
	I2S_PORT_OPENED,
	I2S_PORT_WRITE_BUSY,
	I2S_PORT_READ_BUSY,
	I2S_PORT_CLOSING,
	I2S_PORT_COMPLETE_WRITE,
	I2S_PORT_COMPLETE_READ
};

/*
 * variable "modem_found_and_i2s_setup_ok"
 * bit to set if modem is found on platform
 */
#define MODEM_FND 0

#define FIFO_SIZE 16
/*
 *	Structures Definition
 */

/**
 * struct intel_mid_i2s_data - context struct to keep SSP I2S data
 * @pdev: pci dev pointer corresponding to context
 * @paddr:
 * @ioaddr:
 * @iolen:
 * @irq:
 * @clear_sr:
 * @mask_sr:
 * @dmac1:
 * @dmas_tx: dma slave structure for transmit
 * @dmas_rx: dma slave structure for receive
 * @txchan: Dma channel for transmit
 * @rxchan: Dma channel for receive
 *
 * @read_done:
 * @read_dst:
 * @read_len:
 *
 * @write_done:
 * @write_src:
 * @write_len:
 *
 * @mutex:  a mutex to make sure we have once-at-time critical functions.
 *
 * Longer description
 */

/* Locking rules:
 *
 * All the fields, not listed below, are set during probe, and then read only
 * So they do not require locking
 *
 * The fields that require locking are related to the I2S read and write
 * requests.
 *
 * We allow only 1 read at a time, and 1 write at a time.
 * We allow read in parallel of write but use separate variables.
 * We allow only 1 user per SSP/I2S port.
 * Typically this user will be a dedicated PulseAudio RT thread communicating
 * with cmt-speech driver which in turns communicates with intel_mid_ssp
 * driver.
 * PCM mixing is done before access to kernel drivers;typically within
 * PulseAudio or after; typically within the modem.
 * So no concurrent users, per I2S channel, to this driver are allowed
 * The read & write are triggered from a USER context
 * The read & write callbacks are called from a BH context
 * You should have not callback pending before calling close, close will wait
 * for remaining callback calls.
 * It is not allowed to call close function from read/write callback threads.
 *
 * Locking is handled via drv_data->flags & atomic bitwise operations
 *
 * I2S0 is dedicated for PCM transfer to/from the modem module
 * I2S1 is dedicated for PCM transfer to/from the Bluetooth or FM module
 *
 * read_done:
 * read_len:
 * read_dst:
 *
 * write_done:
 * write_src:
 * write_len:
 *
 * mutex:  a mutex to make sure we have once-at-time critical functions.
 *		once-at-a-time actions functions are:
 *			-intel_mid_i2s_open
 *			-intel_mid_i2s_close
 *			-intel_mid_i2s_rd_req
 *			-intel_mid_i2s_wr_req
 *			-intel_mid_i2s_set_rd_cb
 *			-intel_mid_i2s_set_wr_cb
 * These functions should not be called during a lock() neither in interrupt.
 */

struct intel_mid_i2s_hdl {
	/* Driver model hookup */
	struct pci_dev *pdev;
	/* register addresses */
	dma_addr_t paddr;
	void __iomem *ioaddr;
	u32 iolen;
	int irq;

	/* SSP masks */
	u32 clear_sr;
	u32 mask_sr;

	/* SSP Configuration */
	/* DMA info */
	struct pci_dev *dmac1;

	struct intel_mid_dma_slave dmas_tx;
	struct intel_mid_dma_slave dmas_rx;
	struct dma_chan *txchan;
	struct dma_chan *rxchan;

	struct scatterlist *rxsgl;
	struct scatterlist *txsgl;

	unsigned int device_instance;

	/* Call back functions */
	int	(*read_callback)(void *param);
	int	(*write_callback)(void *param);
	size_t	read_len;	/* read_len > 0 <=> read_dma_running */
	size_t	write_len;	/* write_len > 0 <=> read_dma_running */
	void	*read_param;	/* context param for callback */
	void	*write_param;	/* context param for callback */
	union {
		dma_addr_t	dma;
		u16		*cpu;	/* DO_NOT_USE_DMA mode */
	} read_ptr;
	union {
		dma_addr_t	dma;
		u16		*cpu;	/* DO_NOT_USE_DMA mode */
	} write_ptr;

	unsigned long flags;
	struct mutex mutex;
	enum intel_mid_i2s_ssp_usage usage;

	struct intel_mid_i2s_settings current_settings;
};

struct intel_mid_ssp_gpio {
	u16 ssp_fs_gpio_mapping;
	u8 ssp_fs_mode;
};

static int wr_req_cpu(struct intel_mid_i2s_hdl *drv_data,
		      u32			*source,
		      size_t			len,
		      void			*param);
static int wr_req_dma(struct intel_mid_i2s_hdl *drv_data,
		      u32			*source,
		      size_t			len,
		      void			*param);
static int rd_req_cpu(struct intel_mid_i2s_hdl *drv_data,
		      u32			*destination,
		      size_t			len,
		      void			*param);
static int rd_req_dma(struct intel_mid_i2s_hdl *drv_data,
		      u32			*destination,
		      size_t			len,
		      void			*param);

static void i2s_read_done(void *arg);
static void i2s_write_done(void *arg);
static void i2s_lli_read_done(void *arg);
static void i2s_lli_write_done(void *arg);

static bool chan_filter(struct dma_chan *chan, void *param);
static void ssp1_dump_registers(struct intel_mid_i2s_hdl *);

static
irqreturn_t i2s_irq(int irq, void *dev_id);
static inline
irqreturn_t i2s_irq_handle_RFS(struct intel_mid_i2s_hdl *drv_data, u32 sssr);
static inline
irqreturn_t i2s_irq_handle_TFS(struct intel_mid_i2s_hdl *drv_data, u32 sssr);
static
irqreturn_t i2s_irq_deferred(int irq, void *dev_id);

static int check_device(struct device *device_ptr, void *data);
static void set_ssp_i2s_hw(struct intel_mid_i2s_hdl *drv_data,
			const struct intel_mid_i2s_settings *ps_settings);


#ifdef CONFIG_PM
static int intel_mid_i2s_runtime_resume(struct device *device_ptr);
static int intel_mid_i2s_runtime_suspend(struct device *device_ptr);
#endif
static int intel_mid_i2s_probe(struct pci_dev *pdev,
		const struct pci_device_id *ent);
static void intel_mid_i2s_remove(struct pci_dev *pdev);

/*static int bt_pcm_dma_init(struct intel_mid_i2s_hdl *drv_data);*/

void intel_mid_i2s_set_modem_probe_cb(void(*probe_cb)(void));
void intel_mid_i2s_set_modem_remove_cb(void(*remove_cb)(void));

#ifdef CONFIG_PM
static int  intel_mid_i2s_suspend(struct device *dev);
static int intel_mid_i2s_resume(struct device *dev);
#endif

/*
 * These define will clarify source code when accessing SSCRx registers
 */

#define SSCR0_reg(regbit, value)					\
	(((value) & SSCR0_##regbit##_MASK) << SSCR0_##regbit##_SHIFT)

#define SSCR1_reg(regbit, value)					\
	(((value) & SSCR1_##regbit##_MASK) << SSCR1_##regbit##_SHIFT)

#define SSPSP_reg(regbit, value)					\
	(((value) & SSPSP_##regbit##_MASK) << SSPSP_##regbit##_SHIFT)

#define SSRSA_reg(regbit, value)					\
	(((value) & SSRSA_##regbit##_MASK) << SSRSA_##regbit##_SHIFT)
#define SSTSA_reg(regbit, value)					\
	(((value) & SSTSA_##regbit##_MASK) << SSTSA_##regbit##_SHIFT)


#define change_SSCR0_reg(reg_pointer, regbit, value)			  \
	write_SSCR0((read_SSCR0(reg_pointer)				  \
	& (~((SSCR0_##regbit##_MASK << SSCR0_##regbit##_SHIFT))))	  \
	| (((value) & SSCR0_##regbit##_MASK) << SSCR0_##regbit##_SHIFT),  \
	reg_pointer);

#define set_SSCR0_reg(reg_pointer, regbit)				  \
	write_SSCR0(read_SSCR0(reg_pointer)				  \
	| (SSCR0_##regbit##_MASK << SSCR0_##regbit##_SHIFT),		\
	reg_pointer);

#define clear_SSCR0_reg(reg_pointer, regbit)				  \
	write_SSCR0((read_SSCR0(reg_pointer)				  \
	& (~((SSCR0_##regbit##_MASK << SSCR0_##regbit##_SHIFT)))),	  \
	reg_pointer);

#define change_SSCR1_reg(reg_pointer, regbit, value)			  \
	write_SSCR1((read_SSCR1(reg_pointer)				  \
	& (~((SSCR1_##regbit##_MASK << SSCR1_##regbit##_SHIFT))))	  \
	| (((value) & SSCR1_##regbit##_MASK) << SSCR1_##regbit##_SHIFT),  \
	reg_pointer);

#define set_SSCR1_reg(reg_pointer, regbit)				  \
	write_SSCR1(read_SSCR1(reg_pointer)				  \
	| (SSCR1_##regbit##_MASK << SSCR1_##regbit##_SHIFT),		  \
	reg_pointer);

#define clear_SSCR1_reg(reg_pointer, regbit)				  \
	write_SSCR1((read_SSCR1(reg_pointer)				  \
	& (~((SSCR1_##regbit##_MASK << SSCR1_##regbit##_SHIFT)))),	  \
	reg_pointer);

/* RX FIFO level */
#define GET_SSSR_val(x, regb)						  \
	((x & (SSSR_##regb##_MASK<<SSSR_##regb##_SHIFT))>>SSSR_##regb##_SHIFT)


/*
 * SSP hardware can be configured as I2S, PCM, SPI...
 * In order to allow flexibility without modifying the software driver, the
 * PCI header uses the configuration register 'adid':
 *
 * The PCI header associated to SSP devices includes a configuration register.
 * It provides information to a driver which is probed for the SSP, specifying
 * in which way the SSP is supposed to be used.
 * Here is the format of this configuration register (8 bits):
 *
 *   bits 2..0: Mode
 *       000: Invalid, the register should be ignored
 *       001: SSP to be used as SPI controller
 *       010: SSP to be used in I2S/ISS mode
 *       other: Reserved
 *
 *   bits 5..3: Configuration
 *       In I2S/ISS mode:
 *               000: Invalid
 *               001: Bluetooth
 *               010: Modem
 *               other: Reserved
 *       In SPI mode:
 *               Value is the SPI bus number connected to the SSP.
 *               To be used for registration to the Linux SPI
 *               framework.
 *
 *   bit 6: SPI slave
 *       Relevant in SPI mode only. If set, indicates the SPI clock
 *       is not provided by the SSP: SPI slave mode.
 *
 *   bit 7: Reserved (0)
 *
 *   This configuration register is implemented in the adid field of the
 *   Vendor Specific PCI capability associated to the SSP. The format of
 *   this capability is:
 *
 *   uint8_t     capId;              < Capability ID (vendor-specific)
 *   uint8_t     nextCap;            < Next Item Ptr
 *   uint8_t     length;             < Size of this capability (7)
 *   uint8_t     version;            < Version of this capability (1)
 *   uint8_t     lss;                < Logical subsystem info
 *                                         Bit 7 = PMU (0 = NC, 1 = SC)
 *                                         Bits 6:0 = LSS ID
 *   uint8_t     apmc;               < Additional PM capabilities
 *                                         Bit 7 = Rsvd
 *                                         Bit 6 = Wake capable
 *                                         Bit 5 = D3 support
 *                                         Bit 4 = D2 support
 *                                         Bit 3 = D1 support
 *                                         Bit 2 = D0i3 support
 *                                         Bit 1 = D0i2 support
 *                                         Bit 0 = D0i1 support
 *   uint16_t     adid;              < Additional device ID (dev-specific)
 *
 *   The capability data are in the PCI configuration space and the
 *   adid field can be modified using BMP tool.
 */
/* ADDID = Additional Device ID */
#define PCI_CAP_OFFSET_ADID 6



#define SSP_PLL_FREQ_05_622 (0<<4)
#define SSP_PLL_FREQ_11_345 (1<<4)
#define SSP_PLL_FREQ_12_235 (2<<4)
#define SSP_PLL_FREQ_14_847 (3<<4)
#define SSP_PLL_FREQ_32_842 (4<<4)
#define SSP_PLL_FREQ_48_000 (5<<4)

#define SSP_SYSCLK_DIV4_BYPASS (1<<3)

#define SSP_SYSCLK_DIV_1 (0<<0)
#define SSP_SYSCLK_DIV_2 (1<<0)
#define SSP_SYSCLK_DIV_4 (2<<0)
#define SSP_SYSCLK_DIV_8 (3<<0)
#define SSP_SYSCLK_DIV_16 (4<<0)
#define SSP_SYSCLK_DIV_32 (5<<0)

#define SSP_SSACD_NOT_AVAILABLE 0xff
#define SSP_CLK_SSCR0_SCR_NOT_AVAILABLE 0


/*
 * Following enums are for frequency calculation in master mode...
 */

enum mrst_ssp_bit_per_sample {
	SSP_BIT_PER_SAMPLE_8 = 0,
	SSP_BIT_PER_SAMPLE_16,
	SSP_BIT_PER_SAMPLE_32,
	SSP_BIT_PER_SAMPLE_SIZE
};

enum mrst_ssp_timeslot {
	SSP_TIMESLOT_1 = 0,
	SSP_TIMESLOT_2,
	SSP_TIMESLOT_4,
	SSP_TIMESLOT_8,
	SSP_TIMESLOT_SIZE
};

static u8
  ssp_ssacd[SSP_FRM_FREQ_SIZE][SSP_BIT_PER_SAMPLE_SIZE][SSP_TIMESLOT_SIZE] = {

	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_SSACD_NOT_AVAILABLE,

	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_SSACD_NOT_AVAILABLE,

	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_UNDEFINED][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_SSACD_NOT_AVAILABLE,


	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_1,

	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,

	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,
	[SSP_FRM_FREQ_48_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_12_235 | SSP_SYSCLK_DIV_1 | SSP_SYSCLK_DIV4_BYPASS,


	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1,

	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,

	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,
	[SSP_FRM_FREQ_44_100][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1 | SSP_SYSCLK_DIV4_BYPASS,


	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,

	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,

	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2 | SSP_SYSCLK_DIV4_BYPASS,
	[SSP_FRM_FREQ_22_050][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1 | SSP_SYSCLK_DIV4_BYPASS,


	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,

	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_4,

	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_16_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_2,


	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,

	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,

	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_4,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_2,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_05_622 | SSP_SYSCLK_DIV_1,
	[SSP_FRM_FREQ_11_025][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_11_345 | SSP_SYSCLK_DIV_1,


	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_2] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_8][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,

	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_1] = SSP_SSACD_NOT_AVAILABLE,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_16][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,

	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_1] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_32,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_2] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_16,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_4] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_8,
	[SSP_FRM_FREQ_8_000][SSP_BIT_PER_SAMPLE_32][SSP_TIMESLOT_8] = SSP_PLL_FREQ_32_842 | SSP_SYSCLK_DIV_4,
};

#endif /* MID_I2S_H_*/
