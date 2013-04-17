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

u32 __iomem *residency[SYS_STATE_MAX];
u32 __iomem *s0ix_counter[SYS_STATE_MAX];

static int mrfld_pmu_init(void)
{
	mid_pmu_cxt->s3_hint = MRFLD_S3_HINT;

	/* Put all unused LSS in D0i3 */
	mid_pmu_cxt->os_sss[0] = (SSMSK(D0I3_MASK, PMU_PSH_LSS_00)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_03)	|
				SSMSK(D0I3_MASK, PMU_HSI_LSS_05)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_07)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_12)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_13)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_14)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_15));

	mid_pmu_cxt->os_sss[1] = (SSMSK(D0I3_MASK, PMU_RESERVED_LSS_16-16)|
				SSMSK(D0I3_MASK, PMU_SSP3_LSS_17-16)|
				SSMSK(D0I3_MASK, PMU_SSP6_LSS_19-16)|
				SSMSK(D0I3_MASK, PMU_USB_OTG_LSS_28-16)|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_29-16)|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_30-16));

	/* Excpet for LSS 35 keep all in D0i3 */
	mid_pmu_cxt->os_sss[2] = 0xFFFFFFFF;
	mid_pmu_cxt->os_sss[3] = 0xFFFFFFFF;

	mid_pmu_cxt->os_sss[2] &= ~SSMSK(D0I3_MASK, PMU_SSP4_LSS_35-32);

	/* Map S0ix residency counters */
	residency[SYS_STATE_S0I1] = ioremap_nocache(S0I1_RES_ADDR, 4);
	if (residency[SYS_STATE_S0I1] == NULL)
		goto err1;
	residency[SYS_STATE_S0I2] = ioremap_nocache(S0I2_RES_ADDR, 4);
	if (residency[SYS_STATE_S0I2] == NULL)
		goto err1;
	residency[SYS_STATE_S0I3] = ioremap_nocache(S0I3_RES_ADDR, 4);
	if (residency[SYS_STATE_S0I3] == NULL)
		goto err1;

	/* Map S0ix iteration counters */
	s0ix_counter[SYS_STATE_S0I1] = ioremap_nocache(S0I1_COUNT_ADDR, 4);
	if (s0ix_counter[SYS_STATE_S0I1] == NULL)
		goto err2;
	s0ix_counter[SYS_STATE_S0I2] = ioremap_nocache(S0I2_COUNT_ADDR, 4);
	if (s0ix_counter[SYS_STATE_S0I2] == NULL)
		goto err2;
	s0ix_counter[SYS_STATE_S0I3] = ioremap_nocache(S0I3_COUNT_ADDR, 4);
	if (s0ix_counter[SYS_STATE_S0I3] == NULL)
		goto err2;

	return PMU_SUCCESS;

err1:
	pr_err("Cannot map memory to read S0ix residency\n");
err2:
	pr_err("Cannot map memory to read S0ix count\n");
	return PMU_FAILED;
}

/* FIXME: Need to start the counter only if debug is
 * needed. This will save SCU cycles if debug is
 * disabled
 */
static int __init start_scu_s0ix_res_counters(void)
{
	int ret;

	ret = intel_scu_ipc_simple_command(START_RES_COUNTER, 0);
	if (ret) {
		pr_err("IPC command to start res counter failed\n");
		BUG();
		return ret;
	}
	return 0;
}
late_initcall(start_scu_s0ix_res_counters);

void platform_update_all_lss_states(struct pmu_ss_states *pmu_config,
					int *PCIALLDEV_CFG)
{
	/* Overwrite the pmu_config values that we get */
	pmu_config->pmu2_states[0] =
				(SSMSK(D0I3_MASK, PMU_PSH_LSS_00)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_03)	|
				SSMSK(D0I3_MASK, PMU_HSI_LSS_05)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_07)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_12)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_13)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_14)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_15));

	pmu_config->pmu2_states[1] =
				(SSMSK(D0I3_MASK, PMU_RESERVED_LSS_16-16)|
				SSMSK(D0I3_MASK, PMU_SSP3_LSS_17-16)|
				SSMSK(D0I3_MASK, PMU_SSP6_LSS_19-16)|
				SSMSK(D0I3_MASK, PMU_USB_OTG_LSS_28-16)	|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_29-16)|
				SSMSK(D0I3_MASK, PMU_RESERVED_LSS_30-16));

	pmu_config->pmu2_states[0] &= ~IGNORE_SSS0;
	pmu_config->pmu2_states[1] &= ~IGNORE_SSS1;
	pmu_config->pmu2_states[2] = ~IGNORE_SSS2;
	pmu_config->pmu2_states[3] = ~IGNORE_SSS3;

	/* Excpet for LSS 35 keep all in D0i3 */
	pmu_config->pmu2_states[2] &= ~SSMSK(D0I3_MASK, PMU_SSP4_LSS_35-32);
}

/*
 * In MDFLD and CLV this callback is used to issue
 * PM_CMD which is not required in MRFLD
 */
static bool mrfld_pmu_enter(int s0ix_state)
{
	return true;
}

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

static char *nc_devices[] = {
	"GFXSLC",
	"GSDKCK",
	"GRSCD",
	"VED",
	"VEC",
	"DPA",
	"DPB",
	"DPC",
	"VSP",
	"ISP",
	"MIO",
	"HDMIO",
	"GFXSLCLDO"
};

static int wait_for_nc_pmcmd_complete(int verify_mask,
				int status_mask, int state_type , int reg)
{
	int pwr_sts;
	int count = 0;

	while (true) {
		pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
		pwr_sts = pwr_sts >> SSS_SHIFT;
		if (state_type == OSPM_ISLAND_DOWN ||
					state_type == OSPM_ISLAND_SR) {
			if ((pwr_sts & status_mask) ==
						(verify_mask & status_mask))
				break;
			else
				udelay(10);
		} else if (state_type == OSPM_ISLAND_UP) {
			if ((~pwr_sts & status_mask)  ==
						(~verify_mask & status_mask))
				break;
			else
				udelay(10);
		}

		count++;
		if (WARN_ONCE(count > 500000, "Timed out waiting for P-Unit"))
			return -EBUSY;
	}
	return 0;
}

static int mrfld_nc_set_power_state(int islands, int state_type,
							int reg, int *change)
{
	u32 pwr_sts = 0;
	u32 pwr_mask = 0;
	int i, lss, mask;
	int ret = 0;
	int status_mask = 0;

	*change = 0;
	pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
	pwr_mask = pwr_sts;

	for (i = 0; i < OSPM_MAX_POWER_ISLANDS; i++) {
		lss = islands & (0x1 << i);
		if (lss) {
			mask = D0I3_MASK << (BITS_PER_LSS * i);
			status_mask = status_mask | mask;
			if (state_type == OSPM_ISLAND_DOWN)
				pwr_mask |= mask;
			else if (state_type == OSPM_ISLAND_UP)
				pwr_mask &= ~mask;
			/* Soft reset case */
			else if (state_type == OSPM_ISLAND_SR) {
				pwr_mask &= ~mask;
				mask = SR_MASK << (BITS_PER_LSS * i);
				pwr_mask |= mask;
			}
		}
	}

	if (pwr_mask != pwr_sts) {
		intel_mid_msgbus_write32(PUNIT_PORT, reg, pwr_mask);
		ret = wait_for_nc_pmcmd_complete(pwr_mask,
					status_mask, state_type, reg);
		if (!ret)
			*change = 1;
	}

	return ret;
}

void pmu_set_s0ix_possible(int state)
{
	bool north_good;
	int no_of_nc_devices = sizeof(nc_devices)/sizeof(nc_devices[0]);

	/* assume S0ix not possible */
	mid_pmu_cxt->s0ix_possible = 0;

	if (state != PCI_D0) {
		struct pmu_ss_states cur_pmsss;

		pmu_read_sss(&cur_pmsss);

		if ((((cur_pmsss.pmu2_states[1] & S0IX_TARGET_SSS1_MASK) ==
						S0IX_TARGET_SSS1) &&
			((cur_pmsss.pmu2_states[2] & S0IX_TARGET_SSS2_MASK) ==
						S0IX_TARGET_SSS2) &&
			((cur_pmsss.pmu2_states[3] & S0IX_TARGET_SSS3_MASK) ==
						S0IX_TARGET_SSS3))) {
			u32 val, nc_pwr_sts;
			int i;

			/* assume north good */
			north_good = true;
			nc_pwr_sts =
				intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS);
			for (i = 0; i < no_of_nc_devices; i++) {
				val = nc_pwr_sts & 3;
				if (val != 3) {
					north_good = false;
					break;
				}
				nc_pwr_sts >>= BITS_PER_LSS;
			}

			/* If S0iX is not possible check
			 * LPMP3 is possible
			 */
			if (north_good) {
				if ((cur_pmsss.pmu2_states[0] &
				S0IX_TARGET_SSS0_MASK) == S0IX_TARGET_SSS0)
					mid_pmu_cxt->s0ix_possible = 1;
				else if ((cur_pmsss.pmu2_states[0] &
				LPMP3_TARGET_SSS0_MASK) == LPMP3_TARGET_SSS0)
					mid_pmu_cxt->s0ix_possible = 2;
			}
		}
	}
}

struct platform_pmu_ops mrfld_pmu_ops = {
	.init	 = mrfld_pmu_init,
	.enter	 = mrfld_pmu_enter,
	.nc_set_power_state = mrfld_nc_set_power_state,
	.set_power_state_ops = pmu_set_s0ix_possible,
};
