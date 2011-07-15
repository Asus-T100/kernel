#ifndef _SPI_DW_H_
#define _SPI_DW_H_

#define SPI_DW_DEASSERT	0
#define SPI_DW_ASSERT	1

/*
 * If the platform does not use on of the chip select lines provided
 * by the SOC it may register a chip select control function. The
 * address of initialized spi_dw_chip stucture in the controller_data
 * member of spi_board_info structure registered with the subsystem.
 */
struct spi_dw_chip {
	void (*cs_control)(u32 command);
};

#endif /* _SPI_DW_H_ */
