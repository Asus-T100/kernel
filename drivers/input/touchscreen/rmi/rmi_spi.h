/**
 *
 * Register Mapped Interface SPI Physical Layer Driver Header File.
 * Copyright (C) 2008-2011, Synaptics Incorporated
 *
 */
/*
 * This file is licensed under the GPL2 license.
 *
 *#############################################################################
 * GPL
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 *#############################################################################
 */

#if !defined(_RMI_SPI_H)
#define _RMI_SPI_H

#include <linux/rmi_platformdata.h>

#define RMI4_SPI_DRIVER_NAME "rmi4_ts"
#define RMI4_SPI_DEVICE_NAME "rmi4_ts"

/* Some RMI4 SPI devices require a delay between writing the address and
 * starting the read.  A subset of those required a delay between each
 * byte transferred during the read.
 */

/* microseconds between header and start of read operation. */
#define RMI_DEFAULT_BLOCK_DELAY_US	65

/* microseconds inter-byte delay between bytes during read. */
#define RMI_DEFAULT_BYTE_DELAY_US	65

/* Use this to specify SPI interface dependent parameters on a per device basis.
 *
 * Interface independent data is given in the sensor_data field of this struct.
 */
struct rmi_spi_platformdata {
	/* RMI4 devices implement two different ways of talking to the
	 * device over SPI.  These are called SPIv1 and SPIv2.  Due to
	 * resource constraints on some ASICs, delays may be required when
	 * reading data from the chip.
	 *
	 * The block delay specifies the number of microseconds the
	 * driver should delay between sending the read request and
	 * the start of reading data from the ASIC.  If you don't know
	 * what value to use here, you should specify
	 * RMI_DEFAULT_BLOCK_DELAY_US.
	 *
	 * The byte delay specifies the number of microseconds the driver should
	 * delay between each byte of a read request.  If don't know what value
	 * to use here, you should specify RMI_DEFAULT_BLOCK_DELAY_US.
	 *
	 * Usually these two values should be the same, but in some cases
	 * it may be desirable to use different values.
	 */
	unsigned int block_delay_us;
	unsigned int byte_delay_us;

	/* SPIv2 supports a special "split read" operation, which can permit the
	 * SPI interface to run at full speed (subject to product specific
	 * limitations) with no delay between blocks and bytes.  In almost all
	 * cases, it is permissible to default these values to zero.
	 */
	unsigned int split_read_block_delay_us;
	unsigned int split_read_byte_delay_us;

	/* Some SPI hardware and/or drivers do not manage the SSB/CS line in a
	 * reasonable way.  In particular, the problem is that SSB/CS will be
	 * deasserted in between every spi_transfer in an spi_message (despite
	 * whatever you might have set the spi_transfer.cs_change flag to),
	 * rather than asserting it at the start of the spi_message and leaving
	 * it asserted until all transfers are completed.  In this case, we
	 * have to manage the SSB/CS line manually, and you need to provide
	 * the cs_assert callback here.
	 *
	 * If the cs_assert function is non-null, it will be called before
	 * the driver submits an spi_message in order to assert the line (the
	 * assert parameter will be TRUE), and afterwards to clear it (the
	 * assert parameter will be FALSE).  cs_assert should return 0 for
	 * success, or a negative error code if it fails.
	 *
	 * You can provide any needed context data in the cs_assert_data
	 * variable, which will be passed into all cs_assert calls.
	 */
	void *cs_assert_data;
	int (*cs_assert) (const void *cs_assert_data, const bool assert);


	/* Use this to specify platformdata that is not SPI specific. */
	struct rmi_sensordata *sensordata;
};

#endif
