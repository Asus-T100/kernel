/*
 * intel_soc_mrfld.c - This driver provides utility api's for merrifield
 * platform
 * Copyright (c) 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include "intel_soc_pmu.h"

static int mrfld_pmu_init(void)
{
	mid_pmu_cxt->s3_hint = MRFLD_S3_HINT;
	return PMU_SUCCESS;
}

void platform_update_all_lss_states(struct pmu_ss_states *pmu_config,
					int *PCIALLDEV_CFG)
{
	pmu_config->pmu2_states[0] &= ~IGNORE_SSS0;
	pmu_config->pmu2_states[1] &= ~IGNORE_SSS1;
	pmu_config->pmu2_states[2] &= ~IGNORE_SSS2;
	pmu_config->pmu2_states[3] &= ~IGNORE_SSS3;
}

/*
 * In MDFLD and CLV this callback is used to issue
 * PM_CMD which is not required in MRFLD
 */
static bool mrfld_pmu_enter(int s0ix_state)
{
	return true;
}

struct platform_pmu_ops mrfld_pmu_ops = {
	.init	 = mrfld_pmu_init,
	.enter	 = mrfld_pmu_enter,
};

/**
 *      platform_set_pmu_ops - Set the global pmu method table.
 *      @ops:   Pointer to ops structure.
 */
void platform_set_pmu_ops(void)
{
	pmu_ops = &mrfld_pmu_ops;
}

/*
 * As of now since there is no sequential mapping between
 * LSS abd WKS bits the following two calls are dummy
 */

bool mid_pmu_is_wake_source(u32 lss_number)
{
	return false;
}

/* return the last wake source id, and make statistics about wake sources */
int pmu_get_wake_source(void)
{
	return INVALID_WAKE_SRC;
}


int set_extended_cstate_mode(const char *val, struct kernel_param *kp)
{
	return 0;
}

int get_extended_cstate_mode(char *buffer, struct kernel_param *kp)
{
	const char *default_string = "not supported";
	strcpy(buffer, default_string);
	return strlen(default_string);
}
