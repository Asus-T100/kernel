/*
 * Support for Medfield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#ifdef CONFIG_X86_MRFLD
#define SYSTEM_hive_isp_css_2400_system
#endif

#include "sh_css_hw.h"
#include "sh_css_hrt.h"
#include "sh_css_internal.h"
#include "sh_css_metrics.h"
#if defined(SYSTEM_hive_isp_css_2400_system)
#include <scalar_processor_2400_params.h>    /* SP_PMEM_DEPTH */
#endif

#define ISP_RUN_BIT 0x3
#define MULTIPLE_PCS 0
#define SUSPEND      0
#define NOF_PCS      1
#define RESUME_MASK  0x8
#define STOP_MASK    0x0

static bool pc_histogram_enabled;
static struct sh_css_pc_histogram *isp_histogram;
static struct sh_css_pc_histogram *sp_histogram;

struct sh_css_metrics sh_css_metrics;

void
sh_css_metrics_start_frame(void)
{
	sh_css_metrics.frame_metrics.num_frames++;
}

static void
clear_histogram(struct sh_css_pc_histogram *histogram)
{
	unsigned i;
	for (i = 0; i < histogram->length; i++) {
		histogram->run[i] = 0;
		histogram->stall[i] = 0;
		histogram->msink[i] = 0xFFFF;
	}
}

void
sh_css_metrics_enable_pc_histogram(bool enable)
{
	pc_histogram_enabled = enable;
}

static void
make_histogram(struct sh_css_pc_histogram *histogram, unsigned length)
{
	if (histogram->length)
		return;
	if (histogram->run)
		return;
	histogram->run = sh_css_malloc(length * sizeof(*histogram->run));
	if (!histogram->run)
		return;
	histogram->stall = sh_css_malloc(length * sizeof(*histogram->stall));
	if (!histogram->stall)
		return;
	histogram->msink = sh_css_malloc(length * sizeof(*histogram->msink));
	if (!histogram->msink)
		return;

	histogram->length = length;
	clear_histogram(histogram);
}

static void
insert_binary_metrics(struct sh_css_binary_metrics **l,
			struct sh_css_binary_metrics *metrics)
{
	for (; *l; l = &(*l)->next)
		if (*l == metrics)
			return;

	*l = metrics;
	metrics->next = NULL;
}

void
sh_css_metrics_start_binary(struct sh_css_binary_metrics *metrics)
{
	if (!pc_histogram_enabled)
		return;

	isp_histogram = &metrics->isp_histogram;
	sp_histogram = &metrics->sp_histogram;
	make_histogram(isp_histogram, ISP_PMEM_DEPTH);
	make_histogram(sp_histogram, SP_PMEM_DEPTH);
	insert_binary_metrics(&sh_css_metrics.binary_metrics, metrics);
}

void
sh_css_metrics_sample_pcs(void)
{
	bool stall;
	unsigned int pc;
	unsigned int msink;

#if SUSPEND
	unsigned int sc = 0;
	unsigned int stopped_sc = 0;
	unsigned int resume_sc = 0;
#endif


#if MULTIPLE_PCS
	int i;
	unsigned int pc_tab[NOF_PCS] ;

	for (i = 0; i < NOF_PCS; i++)
		pc_tab[i] = 0;
#endif

	if (!pc_histogram_enabled)
		return;

	if (isp_histogram) {
#if SUSPEND
		/* STOP the ISP */
		sh_css_isp_ctrl_store(ISP_SC_REG, STOP_MASK);
#endif
		msink = sh_css_hrt_isp_current_msink();
#if MULTIPLE_PCS
		for (i = 0; i < NOF_PCS; i++)
			pc_tab[i] = sh_css_hrt_isp_current_pc();
#else
		pc = sh_css_hrt_isp_current_pc();
#endif

#if SUSPEND
		/* RESUME the ISP */
		sh_css_isp_ctrl_store(ISP_SC_REG, RESUME_MASK);
#endif
		isp_histogram->msink[pc] &= msink;
		stall = (msink != 0x7FF);

		if (stall)
			isp_histogram->stall[pc]++;
		else
			isp_histogram->run[pc]++;

#if MULTIPLE_PCS
		printk(KERN_INFO "msink = 0%X\n", msink);
		for (i = 0; i < NOF_PCS; i++)
			printk(KERN_INFO "PC = %d  ", pc_tab[i]);
		printk(KERN_INFO "\n");
#endif
	}

	if (sp_histogram && 0) {
		msink = sh_css_hrt_sp_current_msink();
		pc = sh_css_hrt_sp_current_pc();
		sp_histogram->msink[pc] &= msink;
		stall = (msink != 0x7FF);
		if (stall)
			sp_histogram->stall[pc]++;
		else
			sp_histogram->run[pc]++;
	}
}
