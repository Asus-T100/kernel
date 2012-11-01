/****************************************************************
  Zoro Software.
  D-Trig Digitizer modules files
  Copyright (C) 2010, Dmitry Kuzminov

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ****************************************************************/
/* linux/spi/ntrig_spi */
#ifndef _NTRIG_SPI_H
#define _NTRIG_SPI_H

/**
 * platform data for ntrig sensor driver over SPI
 */
struct ntrig_spi_platform_data {
	/** the	gpio line used as "output enable". We must set this
	 *  line in	order to activate the SPI link. set to -1 if
	 *  there is no GPIO line */
	int oe_gpio;
	/** if 1, the output enable	line is	connected to an
	 *  "inverter" - set it	to reverse value (0	for	1, 1 for 0) */
	int oe_inverted;
	/** the gpio line used for power, -1 if not used */
	int pwr_gpio;
	/** the flags to use when requesting an interrupt handler in
	 *  the driver. They can be different for some chipsets. if 0
	 *  driver will use default flags */
	int irq_flags;
};

#endif
