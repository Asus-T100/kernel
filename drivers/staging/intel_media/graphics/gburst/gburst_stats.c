/**************************************************************************
 * Copyright (c) 2012, Intel Corporation.
 * All Rights Reserved.

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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Hitesh K. Patel <hitesh.k.patel@intel.com>
 *    Dale B. Stimson <dale.b.stimson@intel.com>
 *
 */

#if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE)

#include <stddef.h>
#include <linux/ctype.h>

#include <utilf.h>

#include "gburst.h"
#include <gburst_hw.h>

#define MAX_NUM_CORES 2
#define MAX_NUM_COUNTERS 8

/* Performance counter collection for gpu burst load information. */

/**
 * HSTLEN - "history array size" - number of consecutive counter utilization
 * values stored/
 */
#define HSTLEN 16

#if ((HSTLEN & (HSTLEN - 1)) == HSTLEN)
/* HSTLEN is a power of two. */
#define HSTLEN_MASK (HSTLEN - 1)
#else
#define HSTLEN_MASK 0
#endif

/* Count values (unsigned) above this limit are ignored as an error */
#define GFX_PERF_COUNTER_SANITY_LIMIT  0x80000000U


/**
 * struct pcel_s -- Performance counter element.
 * Data for each counter.
 */
struct pcel_s {
	/**
	 * full_util_ref - The counter delta that represents maximum
	 * utilization for one timestamp unit, obtained by
	 * characterization.
	 */
	uint32_t pce_full_util_ref;
	uint32_t average;
	uint32_t last_value;		/* for calculating delta */
	uint32_t utilization;
	/* pce_value_index - index into pce_values for next time. */
	uint32_t pce_value_index;
	/* pce_time_stamp - gpu clock divided by 16. */
	uint32_t pce_time_stamp;
	/**
	 * max_util - The highest value of any of the utilization percentages
	 * in "values".
	 */
	uint32_t max_util;
	/* max_c_per_us -- max counts per microsecond. */
	uint32_t max_c_per_us;
	/* counter values.  Current plus previous.  Circular buffer. */
	uint32_t pce_values[HSTLEN];
};

struct gburst_stats_data_s {
	/**
	 * gsd_stats_initialized - Set to !0 when initialization has been completed.
	 */
	int           gsd_stats_initialized;

	int           gsd_gpu_freq_mhz;
	unsigned int  gsd_num_cores;
	unsigned int  gsd_num_counters_hw;
	unsigned int  gsd_num_counters;

	/* gsd_active counters used for utilization calculation, default all. */
	uint32_t      gsd_active_counters;

	/* gsd_pcd - performance counter data */
	struct pcel_s gsd_pcd[MAX_NUM_CORES][MAX_NUM_COUNTERS];
};


/* Variables visible at file scope */

struct gburst_stats_data_s gsdat;


/* Forward references */
static int gburst_stats_init(void);


/**
 * gburst_stats_initialization_complete() - Attempt initialization,
 * if not already done.
 *
 * Function return value:  1 if initialized, 0 if not.
 */
static inline int gburst_stats_initialization_complete(void)
{
	if (!gsdat.gsd_stats_initialized)
		return gburst_stats_init();
	return 1;
}


/**
 * gpu_init_perf_counters() - Initialize local data that describes the
 * performance counters.  Causes old history to be forgotten.
 */
static void gpu_init_perf_counters(void)
{
	int ix_core;
	int ndx;
	int val;

	/* All zero, except for counter limit max. */
	memset(&gsdat.gsd_pcd, 0, sizeof(gsdat.gsd_pcd));

	for (ndx = 0; ndx < gsdat.gsd_num_counters; ndx++) {
		for (ix_core = 0; ix_core < gsdat.gsd_num_cores; ix_core++) {
			val = gburst_hw_inq_counter_max_value(ndx);
			if (val < 0)
				val = 0;
			gsdat.gsd_pcd[ix_core][ndx].pce_full_util_ref = val;
		}
	}
}


/**
 * next_index_for_buffer() - increment the circular buffer index.
 * @ndx: Input index
 *
 * Function return value: Updated index into circular buffer.
 */
static inline uint32_t next_index_for_buffer(uint32_t ndx)
{
#if HSTLEN_MASK
	return (ndx + 1) & HSTLEN_MASK;
#else
	ndx++;
	if (ndx >= HSTLEN)
		ndx = 0;

	return ndx;
#endif
}


/**
 * compute_utilization() - Compute a utilization metric.
 *
 * Although a call to this function is/was commented elsewhere as
 * "compute the moving average", in reality it finds the maximum
 * value in the history.
 */
static void compute_utilization(struct pcel_s *pce)
{
	int i;
	uint32_t *pValues;
	int ndx;

	/* Find highest utilization */
	pValues = pce->pce_values;
	pce->average = 0;
	ndx = pce->pce_value_index;
	for (i = 0; i < HSTLEN; i++) {
		if (pce->average < pValues[ndx])
			pce->average = pValues[ndx];
		ndx = next_index_for_buffer(ndx);
	}

	if (pce->max_util < pce->average)
		pce->max_util = pce->average;
}


/**
 * update_perf_counter_value() - Store one hw counter into database.
 * @ix_core - gpu core index
 * @ndx     - our counter index
 * @value   - raw value of counter, as read from device.
 * @time_stamp - clocks divided by 16
 * @counters_storable - non-zero to store and use counter value.
 *                  Ancilliary actions may be taken regardless.
 * Called from function gburst_stats_gfx_hw_perf_record.
 */
static void update_perf_counter_value(int ix_core, int ndx,
	uint32_t value, uint32_t time_stamp, uint32_t counters_storable)
{
	struct pcel_s *pce;
	int value_index;
	int counter_delta;
	int time_delta;
	uint32_t full_util_ref;
	uint32_t utilization_value;
	unsigned int counts_per_us = 0;

	pce = &gsdat.gsd_pcd[ix_core][ndx];
	value_index = pce->pce_value_index;
	pce->utilization = 0;
	pce->pce_values[value_index] = 0;
	full_util_ref = pce->pce_full_util_ref;

	if (counters_storable
		&& (time_stamp > pce->pce_time_stamp)
		&& (full_util_ref > 0)
		&& (value > pce->last_value)) {

		time_delta = time_stamp - pce->pce_time_stamp;
		counter_delta = value - pce->last_value;

		{
			/* 25 ticks per usec */
			/* 400MHz / 16 --> 25 ticks per microsecond */
			counts_per_us = (counter_delta*25) / time_delta;

			/* Scale full util for 400MHz */
			full_util_ref = (full_util_ref * 400) / 533;
		}

		/**
		 * Sanity check.  Do not accept unrealistic values (should
		 * be avoided by above code).
		 */
		if (counts_per_us > GFX_PERF_COUNTER_SANITY_LIMIT)
			counts_per_us = 0;

		/* Calculate counter utilization percentage w.r.t max value. */
		utilization_value = (counts_per_us*100) / full_util_ref;
		pce->utilization = utilization_value;

		pce->pce_values[value_index] = utilization_value;
	}

	compute_utilization(pce);

	pce->pce_time_stamp = time_stamp;
	pce->last_value = value;
	pce->pce_value_index = next_index_for_buffer(pce->pce_value_index);

	/* Just for tracing and tracking purpose */
	if (pce->max_c_per_us < counts_per_us)
		pce->max_c_per_us = counts_per_us;

	if (gburst_debug_msg_on)
		printk(KERN_ALERT "%d,%d\n", ndx, pce->utilization);
}


/**
 * gburst_stats_gfx_hw_perf_max_values_clear() -- Clear "maximum" values.
 *
 * Clears counter members:
 * --  max_util
 * --  max_c_per_us
 */
void gburst_stats_gfx_hw_perf_max_values_clear(void)
{
	struct pcel_s *pce;
	int ndx;
	unsigned int ix_core;

	for (ix_core = 0; ix_core < gsdat.gsd_num_cores; ix_core++) {
		for (ndx = 0; ndx < gsdat.gsd_num_counters; ndx++) {
			pce = &gsdat.gsd_pcd[ix_core][ndx];
			pce->max_util = 0;
			pce->max_c_per_us = 0;
		}
	}
}


/**
 * gburst_stats_gfx_hw_perf_counters_set() -- Specify counter visibility.
 * @buf: A null terminated string that specifies a configuration of counters
 * to be used for utilization computations.
 *
 * Function return value: < 0 for error, otherwise 0.
 *
 * This function allows changing which counters are visible.
 *
 * This function scans counter specifications from a null-terminated string
 * which is expected to be from write to a /proc file (with explicit null
 * termination added by this function's caller.
 *
 * The input string specifies which counters are to be visible as
 * whitespace-separated (e.g., space, tab, newline) groups of: %u:%u:%u:%u
 * which correspond to counter:group:bit:full_util_ref .
 * then assigns the values into data structures for all cores.
 * These per-counter values are:
 * 1.  counter - An index into this module's counter data arrays.
 * 2.  group -- The hardware "group" from which this counter is taken.
 * 3.  bit   -- The hardware bit (for this group) that selects this counter.
 * 4.  full_util_ref -- The delta for this counter (per time_stamp unit)
 *     that is nominally equal to 100% utilization.
 * Example input string: "1:0:1:400   6:0:24:32"
 */
int gburst_stats_gfx_hw_perf_counters_set(const char *buf)
{
	struct pcel_s *pce;
	int i;
	int sts;
	uint32_t counter;
	uint32_t group;
	uint32_t bit;
	uint32_t full_util_ref;
	unsigned int ix_core;
	const int svix_counter = 0;
	const int svix_group = 1;
	const int svix_bit = 2;
	const int svix_counter_max = 3;
	const int nitems = 4;
	int sval[nitems];
	const char *pstr;

	if (!gburst_stats_initialization_complete())
		return -EINVAL;

	pstr = buf;
	for (;;) {
		int nchrs = 0;

		while (isspace(*pstr))
			pstr++;

		/**
		 * The "%n" transfer will affect the return value, and will
		 * only be done if there are enough characters in the input
		 * string (e.g., terminating newline) to reach it.
		 */
		i = sscanf(pstr, " %u:%u:%u:%u%n",
			sval+0, sval+1, sval+2, sval+3, &nchrs);
		pstr += nchrs;
		if ((*pstr != '\0') && !isspace(*pstr))
			return -EINVAL;

		if (i < 4)
			return -EINVAL;

		counter = sval[svix_counter];
		group = sval[svix_group];
		bit = sval[svix_bit];
		full_util_ref = sval[svix_counter_max];

		printk(KERN_INFO "c:%u, g:%u, b:%u, max:%u\n",
			counter, group, bit, full_util_ref);

		/* Must call to gburst_hw_set_perf_status_periodic, below. */
		sts = gburst_hw_set_counter_id(counter, group, bit);
		if (sts < 0)
			return sts;

		for (ix_core = 0; ix_core < gsdat.gsd_num_cores; ix_core++) {
			pce = &gsdat.gsd_pcd[ix_core][counter];
			pce->pce_full_util_ref = full_util_ref;
		}
	}

	return gburst_hw_set_perf_status_periodic(1);
}


/**
 * gburst_stats_gfx_hw_perf_record() - Record performance info.
 *
 * Function return value: Utilization value (0-100) or
 * < 0 if error (indicating not inited).
 */
int gburst_stats_gfx_hw_perf_record(void)
{
	int icx;            /* counter index */
	int i;
	int ndx;
	int sts;
	struct pcel_s *pce;
	unsigned int ix_core;
	uint32_t sgx_util;
	int ix_roff;
	int ix_woff;

	if (!gburst_stats_initialization_complete())
		return -EINVAL;

	sgx_util = 0;

	/**
	 * The incoming data is in a circular buffer, with ix_roff the
	 * index for reading (which this function does) and ix_woff the index
	 * for writing.
	 * Index ix_roff will be updated as processing is done.  The value
	 * for index ix_woff captured here is used for determination of
	 * buffer empty, as it would be undesirable to use the updated write
	 * index and thereby possibly continue processing around the buffer
	 * without end.  The buffer is empty when ix_roff == ix_woff.
	 */
	sts = gburst_hw_perf_data_get_indices(&ix_roff, &ix_woff);
	if (sts < 0)
		return sts;

	if (ix_woff == ix_roff) {
		/* Buffer is empty, so clear utilization calculation table */
		for (ix_core = 0; ix_core < gsdat.gsd_num_cores; ix_core++) {
			for (ndx = 0; ndx < gsdat.gsd_num_counters; ndx++) {
				pce = &gsdat.gsd_pcd[ix_core][ndx];

				pce->average = 0;
				for (i = 0; i < HSTLEN; i++)
					pce->pce_values[i] = 0;
			}
		}
		return 0; /* return with zero utilization */
	}

	while (ix_woff != ix_roff) {
		/* Time stamp is gpu clock divided by 16. */
		uint32_t time_stamp;
		uint32_t counters_storable;
		uint32_t *pdat_base;
		uint32_t *pdat;

		/**
		 * Get information a single entry in the circular buffer, which
		 * includes information such as timestamp and all counter
		 * values.
		 */
		sts = gburst_hw_perf_data_get_data(&time_stamp,
			&counters_storable, &pdat_base);
		if (sts < 0)
			return sts;

		/**
		 * For each active counter, store its value in the database
		 * along with the current timestamp.
		 */
		pdat = pdat_base;
		for (ix_core = 0; ix_core < gsdat.gsd_num_cores;
			ix_core++, pdat += gsdat.gsd_num_counters_hw) {
			/* For each core */
			for (icx = 0; icx < gsdat.gsd_num_counters; icx++) {
				if (gsdat.gsd_active_counters & (1 << icx)) {
					update_perf_counter_value(
						ix_core, icx, pdat[icx],
						time_stamp, counters_storable);
				}
			}
		}
		sts = gburst_hw_perf_data_read_index_incr(&ix_roff);
		if (sts < 0)
			return sts;
	}

	for (ix_core = 0; ix_core < gsdat.gsd_num_cores; ix_core++) {
		for (i = 0; i < gsdat.gsd_num_counters; i++) {

			if ((gsdat.gsd_active_counters & (1 << i))
				&& (gsdat.gsd_pcd[ix_core][i].average >
				sgx_util))
				sgx_util = gsdat.gsd_pcd[ix_core][i].average;
		}
	}

	return sgx_util;
}


/**
 * gburst_stats_gfx_hw_perf_max_values_to_string() -- output values to string.
 * @ix_in: Initial index with buf at which to store string.
 * @buf: A buffer to hold the output string.
 * @buflen: Length of buf.
 *
 * Output string is guaranteed to be null-terminated.
 * Function return value:
 * negative error code or number of characters in output string (even
 * if only part of the string would fit in buffer).
 */
int gburst_stats_gfx_hw_perf_max_values_to_string(int ix_in, char *buf,
	size_t buflen)
{
	struct pcel_s *pce;
	int ix;
	int ndx;
	unsigned int ix_core;

	ix = ut_isnprintf(ix_in, buf, buflen,
		"c:   MaxUtil:   Max_count/us:\n");

	for (ix_core = 0; ix_core < gsdat.gsd_num_cores; ix_core++) {
		ix = ut_isnprintf(ix, buf, buflen, "Core%u:\n", ix_core);

		for (ndx = 0; ndx < gsdat.gsd_num_counters; ndx++) {
			pce = &gsdat.gsd_pcd[ix_core][ndx];
			ix = ut_isnprintf(ix, buf, buflen,
				"%d:   %8u     %8u\n",	ndx,
				pce->max_util, pce->max_c_per_us);
		}
	}

	return ix;
}


/**
 * gburst_stats_active_counters_to_string() - Output active counters to string.
 * @buf: Buffer into which to store output string.
 * @breq: Length of buf
 */
int gburst_stats_active_counters_to_string(char *buf, int breq)
{
	int i;
	int ncounters;

	if (!gburst_stats_initialization_complete())
		return -EINVAL;

	ncounters = gsdat.gsd_num_counters;

	if (ncounters > (breq - 3))
		ncounters = (breq - 3);
	if (ncounters >= (sizeof(gsdat.gsd_active_counters) * 8))
		ncounters = (sizeof(gsdat.gsd_active_counters) * 8);

	for (i = 0; i < ncounters; i++)
		buf[i] = ((gsdat.gsd_active_counters >> i) & 1) + '0';

	buf[i++] = '\n';
	buf[i] = '\0';

	return i;
}


/**
 * gburst_stats_active_counters_from_string() - Read active counters from str.
 * @buf: Buffer containing string specifying active counters.
 * @nbytes: Number of characters in buf.
 */
int gburst_stats_active_counters_from_string(const char *buf, int nbytes)
{
	int i;
	int ncounters;
	uint32_t val;

	if (!gburst_stats_initialization_complete())
		return -EINVAL;

	ncounters = gsdat.gsd_num_counters;

	if (ncounters >= (sizeof(gsdat.gsd_active_counters) * 8))
		ncounters = (sizeof(gsdat.gsd_active_counters) * 8);
	if (ncounters > (nbytes - 1))
		ncounters = (nbytes - 1);

	val = 0;
	for (i = 0; i < ncounters; i++) {
		if (buf[i] == '1')
			val |= 1 << i;
		else if (buf[i] != '0')
			return -EINVAL;
	}

	if (buf[i] != '\n')
		return -EINVAL;

	gsdat.gsd_active_counters = val;

	return 0;
}


/**
 * gburst_stats_gfx_hw_perf_counters_to_string() -- output values to string.
 * @ix_in: Initial index with buf at which to store string.
 * @buf: A buffer to hold the output string.
 * @buflen: Length of buf.
 *
 * Output string is guaranteed to be null-terminated.
 *
 * Function return value:
 * negative error code or number of characters in output string (even
 * if only part of the string would fit in buffer).
 */
int gburst_stats_gfx_hw_perf_counters_to_string(int ix_in, char *buf,
	size_t buflen)
{
	int ix;
	int ndx;
	int sts;
	int ix_core;
	int ctr_grp;
	int ctr_bit;

	if (!gburst_stats_initialization_complete())
		return -EINVAL;

	ix = ix_in;

	ix = ut_isnprintf(ix, buf, buflen, "cix   grp    bit   full\n");
	ix = ut_isnprintf(ix, buf, buflen, "=======================\n");

	for (ix_core = 0; ix_core < gsdat.gsd_num_cores; ix_core++) {
		ix = ut_isnprintf(ix, buf, buflen, "Core %d:\n", ix_core);
		for (ndx = 0; ndx < gsdat.gsd_num_counters; ndx++) {
			sts = gburst_hw_inq_counter_id(ndx, &ctr_grp, &ctr_bit);
			if (sts < 0)
				return sts;

			ix = ut_isnprintf(ix, buf, buflen,
				"%u:  %3u     %2u    %5u\n", ndx,
				ctr_grp, ctr_bit,
				gsdat.gsd_pcd[ix_core][ndx].pce_full_util_ref);
		}
	}

	return ix;
}


/**
 * gburst_stats_gpu_freq_mhz_info() - Set gpu frequency.
 * @freq_mhz: gpu frequency in MHz.
 *
 * The specified gpu frequency is used to update local data and potentially
 * the graphics driver.
 */
int gburst_stats_gpu_freq_mhz_info(int freq_mhz)
{
	gsdat.gsd_gpu_freq_mhz = freq_mhz;

	return gburst_hw_gpu_freq_mhz_info(freq_mhz);
}


/**
 * gburst_stats_init() -- Attempt initialization.
 * Function return value:  1 if initialized, 0 if not.
 */
static int gburst_stats_init(void)
{
	int ncores;
	int sts;

	sts = gburst_hw_init();
	if (sts <= 0)
		return 0;

	gsdat.gsd_active_counters = 0xffffffff;

	ncores = gburst_hw_inq_num_cores();

	if (ncores > MAX_NUM_CORES) {
		printk(KERN_ALERT
			"%s: warning: %u cores present, limiting to %u\n",
			__FILE__, gsdat.gsd_num_cores, MAX_NUM_CORES);
		gsdat.gsd_num_cores = MAX_NUM_CORES;
	} else {
		gsdat.gsd_num_cores = ncores;
	}

	gsdat.gsd_num_counters_hw = gburst_hw_inq_num_counters();

	if (gsdat.gsd_num_counters_hw > MAX_NUM_COUNTERS) {
		printk(KERN_ALERT
			"%s: warning: %u counters present, limiting to %u\n",
			__FILE__, gsdat.gsd_num_counters, MAX_NUM_COUNTERS);
		gsdat.gsd_num_counters = MAX_NUM_COUNTERS;
	} else {
		gsdat.gsd_num_counters = gsdat.gsd_num_counters_hw;
	}

	gpu_init_perf_counters();

	sts = gburst_hw_set_perf_status_periodic(1);
	if (sts < 0)
		return 0;

	gsdat.gsd_stats_initialized = 1;

	return 1;
}
#endif /* if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE) */
