#ifndef _MATRIXIO_H_
#define _MATRIXIO_H_
#define MATRIX_IO_FILE "/dev/matrix"

/*enumerate operations to be done in an IOCTL scan(init, poll & term) */
enum IOCtlType {
	READ_OP = 0x00000001,
	WRITE_OP = 0x00000002,
	ENABLE_OP = 0x00000004,
	POLL_OS = 0x00000008,
	POLL_DRV = 0x00000010,
	POLL_TRACE = 0x00000020,
	SET_BITS_OP = 0x00000040,
	RESET_BITS_OP = 0x00000080,
};

#define MAX_GMCH_CTRL_REGS 4
#define MAX_GMCH_DATA_REGS 8
#define DATA_ENABLE			0x00000001
#define MTX_GMCH_PMON_GLOBAL_CTRL		0x0005F1F0
#define MTX_GMCH_PMON_GLOBAL_CTRL_ENABLE	0x0001000F
#define MTX_GMCH_PMON_GLOBAL_CTRL_DISABLE	0x00000000
#define MTX_GMCH_PMON_FIXED_CTR0		0x0005E8F0
#define MTX_GMCH_PMON_GP_CTR0_L			0x0005F8F0
#define MTX_GMCH_PMON_GP_CTR0_H			0x0005FCF0
#define MTX_GMCH_PMON_GP_CTR1_L			0x0005F9F0
#define MTX_GMCH_PMON_GP_CTR1_H			0x0005FDF0
#define MTX_GMCH_PMON_GP_CTR2_L			0x0005FAF0
#define MTX_GMCH_PMON_GP_CTR2_H			0x0005FEF0
#define MTX_GMCH_PMON_GP_CTR3_L			0x0005FBF0
#define MTX_GMCH_PMON_GP_CTR3_H			0x0005FFF0
#define MTX_GMCH_PMON_FIXED_CTR_CTRL	0x0005F4F0

#define MTX_PCI_MSG_CTRL_REG  0x000000D0
#define MTX_PCI_MSG_DATA_REG  0x000000D4

#define PWR_MGMT_BASE_ADDR_MASK      0xFFFF
#define PWR_STS_NORTH_CMPLX_LOWER    0x4
#define PWR_STS_NORTH_CMPLX_UPPER    0x30

struct mtx_msr {
	unsigned long eax_LSB;
	unsigned long edx_MSB;
	unsigned long ecx_address;
	unsigned long ebx_value;
	unsigned long n_cpu;
	unsigned long operation;
};

struct memory_map {
	unsigned long ctrl_addr;
	void *ctrl_remap_address;
	unsigned long ctrl_data;
	unsigned long data_addr;
	void *data_remap_address;
	char *ptr_data_usr;
	unsigned long data_size;
	unsigned long operation;
};

struct mtx_pci_ops {
	unsigned long port;
	unsigned long data;
	unsigned long io_type;
	unsigned long port_island;
};

struct scu_config {
	unsigned long *address;
	unsigned char *usr_data;
	unsigned char *drv_data;
	unsigned long length;
};

struct lookup_table {
	/*Init Data */
	struct mtx_msr *msrs_init;
	unsigned long msr_init_length;
	unsigned long msr_init_wb;

	struct memory_map *mmap_init;
	unsigned long mem_init_length;
	unsigned long mem_init_wb;

	struct mtx_pci_ops *pci_ops_init;
	unsigned long pci_ops_init_length;
	unsigned long pci_ops_init_wb;

	unsigned long *cfg_db_init;
	unsigned long cfg_db_init_length;
	unsigned long cfg_db_init_wb;

	/*Poll Data */
	struct mtx_msr *msrs_poll;
	unsigned long msr_poll_length;
	unsigned long msr_poll_wb;

	struct memory_map *mmap_poll;
	unsigned long mem_poll_length;
	unsigned long mem_poll_wb;
	unsigned long records;

	struct mtx_pci_ops *pci_ops_poll;
	unsigned long pci_ops_poll_length;
	unsigned long pci_ops_poll_wb;
	unsigned long pci_ops_records;

	unsigned long *cfg_db_poll;
	unsigned long cfg_db_poll_length;
	unsigned long cfg_db_poll_wb;

	struct scu_config scu_poll;
	unsigned long scu_poll_length;

	/*Term Data */
	struct mtx_msr *msrs_term;
	unsigned long msr_term_length;
	unsigned long msr_term_wb;

	struct memory_map *mmap_term;
	unsigned long mem_term_length;
	unsigned long mem_term_wb;

	struct mtx_pci_ops *pci_ops_term;
	unsigned long pci_ops_term_length;
	unsigned long pci_ops_term_wb;

	unsigned long *cfg_db_term;
	unsigned long cfg_db_term_length;
	unsigned long cfg_db_term_wb;
};

struct msr_buffer {
	unsigned long eax_LSB;
	unsigned long edx_MSB;
};

struct xchange_buffer {
	struct msr_buffer *ptr_msr_buff;
	unsigned long msr_length;
	unsigned long *ptr_mem_buff;
	unsigned long mem_length;
	unsigned long *ptr_pci_ops_buff;
	unsigned long pci_ops_length;
	unsigned long *ptr_cfg_db_buff;
	unsigned long cfg_db_length;
};

struct xchange_buffer_all {
	unsigned long long init_time_stamp;
	unsigned long long *poll_time_stamp;
	unsigned long long term_time_stamp;
	struct xchange_buffer xhg_buf_init;
	struct xchange_buffer xhg_buf_poll;
	struct xchange_buffer xhg_buf_term;
	unsigned long status;
};

struct mtx_msr_container {
	unsigned long *buffer;
	unsigned long length;
	struct mtx_msr msrType1;
};

struct gmch_container {
	unsigned long long time_stamp;
	unsigned long read_mask;
	unsigned long write_mask;
	unsigned long mcr1[MAX_GMCH_CTRL_REGS];
	unsigned long mcr2[MAX_GMCH_CTRL_REGS];
	unsigned long mcr3[MAX_GMCH_CTRL_REGS];
	unsigned long data[MAX_GMCH_DATA_REGS];
	unsigned long event[MAX_GMCH_CTRL_REGS];
	unsigned long core_clks;
};

struct mtx_size_info {
	unsigned int init_msr_size;
	unsigned int term_msr_size;
	unsigned int poll_msr_size;
	unsigned int init_mem_size;
	unsigned int term_mem_size;
	unsigned int poll_mem_size;
	unsigned int init_pci_ops_size;
	unsigned int term_pci_ops_size;
	unsigned int poll_pci_ops_size;
	unsigned int init_cfg_db_size;
	unsigned int term_cfg_db_size;
	unsigned int poll_cfg_db_size;
	unsigned int poll_scu_drv_size;
	unsigned int total_mem_bytes_req;
};

#define IOCTL_INIT_SCAN _IOR(0xF8, 0x00000001, unsigned long)
#define IOCTL_TERM_SCAN _IOR(0xF8, 0x00000002, unsigned long)
#define IOCTL_POLL_SCAN _IOR(0xF8, 0x00000004, unsigned long)

#define IOCTL_INIT_MEMORY _IOR(0xF8, 0x00000010, struct xchange_buffer_all *)
#define IOCTL_FREE_MEMORY _IO(0xF8, 0x00000020)

#define IOCTL_VERSION_INFO _IOW(0xF8, 0x00000100, char *)
#define IOCTL_COPY_TO_USER _IOW(0xF8, 0x00000200, struct xchange_buffer_all *)
#define IOCTL_READ_CONFIG_DB _IOW(0xF8, 0x00000400, unsigned long *)
#define IOCTL_OPERATE_ON_MSR _IOW(0xF8, 0x00100000, struct mtx_msr *)

#define IOCTL_MSR _IOW(0xF8, 0x10000000, struct mtx_msr_container *)
#define IOCTL_SRAM _IOW(0xF8, 0x20000000, struct memory_map *)
#define IOCTL_GMCH_RESET _IOW(0xF8, 0x40000000, struct gmch_container *)
#define IOCTL_GMCH _IOW(0xF8, 0x80000000, struct gmch_container *)

#define platform_pci_read32	intel_mid_msgbus_read32_raw
#define platform_pci_write32	intel_mid_msgbus_write32_raw

#endif
