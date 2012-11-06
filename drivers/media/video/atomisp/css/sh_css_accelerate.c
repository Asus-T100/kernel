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

#include "sh_css_hrt.h"
#include "sh_css_sp_start.h"
#include "sh_css_sp.h"
#include "sh_css_internal.h"
#include "sh_css_accelerate.h"

#if !defined(C_RUN) && !defined(HRT_UNSCHED)
static const unsigned char *
sh_css_acc_upload_isp_code(struct sh_css_acc_fw *firmware)
{
	const unsigned char *blob = SH_CSS_ACC_ISP_CODE(firmware);
	unsigned size		  = SH_CSS_ACC_ISP_SIZE(firmware);
	const unsigned char *binary = firmware->header.isp_code;

	if (!binary) {
		binary = sh_css_load_blob(blob, size);
		firmware->header.isp_code = binary;
	}

	if (!binary)
		return NULL;
	sh_css_sp_dmem_store((unsigned int)firmware->header.sp.isp_code,
			     &binary, sizeof(binary));
	return binary;
}
#endif

static void
upload_frame(struct sh_css_frame *sp_address, struct sh_css_frame *frame)
{
	if (!frame || !sp_address)
		return;
	sh_css_sp_dmem_store((unsigned int)sp_address, frame, sizeof(*frame));
}

static void
upload_var(void *sp_address, void *val, size_t size)
{
	if (!sp_address)
		return;
	sh_css_sp_dmem_store((unsigned int)sp_address, val, size);
}

static void
upload_int(unsigned *sp_address, unsigned *val)
{
	upload_var(sp_address, val, sizeof(unsigned));
}

/* Load the firmware into xmem */
enum sh_css_err
sh_css_acc_load(const struct sh_css_acc_fw *firmware)
{
	struct sh_css_acc_fw_hdr *header
		= (struct sh_css_acc_fw_hdr *)&firmware->header;
	header->sp_args =
		sh_css_malloc(sizeof(*header->sp_args) *
			      header->sp.args_cnt);
	if (!header->sp_args)
		return sh_css_err_cannot_allocate_memory;
	header->loaded = true;
	return sh_css_success;
}

void
sh_css_acc_unload(const struct sh_css_acc_fw *firmware)
{
	struct sh_css_acc_fw_hdr *header
		= (struct sh_css_acc_fw_hdr *)&firmware->header;
	sh_css_free(header->sp_args);
	if (header->sp_code)
		hrt_isp_css_mm_free((void *)header->sp_code);
	if (header->isp_code)
		hrt_isp_css_mm_free((void *)header->isp_code);
	header->sp_args  = NULL;
	header->sp_code  = NULL;
	header->isp_code = NULL;
	header->loaded   = false;
}

/* Set argument <num> of size <size> to value <val> */
enum sh_css_err
sh_css_acc_set_argument(struct sh_css_acc_fw *firmware,
			unsigned num, void *val, size_t size)
{
	if (num >= firmware->header.sp.args_cnt)
		return sh_css_err_invalid_arguments;

	if (!firmware->header.sp_args)
		return sh_css_err_invalid_arguments;
	firmware->header.sp_args[num].type  = sh_css_argument_type(firmware,
								   num);
	firmware->header.sp_args[num].value = val;
	firmware->header.sp_args[num].size  = size;
	return sh_css_success;
}

/* Set host private data for argument <num> */
enum sh_css_err
sh_css_argument_set_host(struct sh_css_acc_fw *firmware,
			 unsigned num, void *host)
{
	if (!firmware->header.sp_args)
		return sh_css_err_invalid_arguments;
	if (num >= firmware->header.sp.args_cnt)
		return sh_css_err_invalid_arguments;
	firmware->header.sp_args[num].host = host;
	return sh_css_success;
}

/* Get host private data for argument <num> */
void *
sh_css_argument_get_host(struct sh_css_acc_fw *firmware, unsigned num)
{
	if (!firmware->header.sp_args)
		return NULL;
	if (num >= firmware->header.sp.args_cnt)
		return NULL;
	return firmware->header.sp_args[num].host;
}

/* Get type for argument <num> */
enum sh_css_acc_arg_type
sh_css_argument_type(struct sh_css_acc_fw *firmware, unsigned num)
{
	return SH_CSS_ACC_SP_ARGS(firmware)[num];
}

size_t
sh_css_argument_get_size(struct sh_css_acc_fw *firmware, unsigned num)
{
	return firmware->header.sp_args[num].size;
}

unsigned
sh_css_num_accelerator_args(struct sh_css_acc_fw *firmware)
{
	return firmware->header.sp.args_cnt;
}

void
sh_css_acc_stabilize(struct sh_css_acc_fw *firmware, unsigned num, bool stable)
{
	firmware->header.sp_args[num].stable = stable;
}

bool
sh_css_acc_is_stable(struct sh_css_acc_fw *firmware, unsigned num)
{
	return firmware->header.sp_args[num].stable;
}

static void
copy_sp_arguments(struct sh_css_acc_fw *firmware, bool to_sp)
{
	unsigned sp_address = (unsigned)firmware->header.sp.args;
	unsigned i;
	for (i = 0; i < firmware->header.sp.args_cnt; i++) {
		enum sh_css_acc_arg_type type =
			firmware->header.sp_args[i].type;
		void *value = firmware->header.sp_args[i].value;
		unsigned size = firmware->header.sp_args[i].size;
		bool copy = to_sp;
		switch (type) {
		case SH_CSS_ACC_ARG_SCALAR_IN:
			break;
		case SH_CSS_ACC_ARG_SCALAR_IO:
			copy = true;
			break;
		case SH_CSS_ACC_ARG_SCALAR_OUT:
			copy = !to_sp;
			break;
		case SH_CSS_ACC_ARG_PTR_IN:
		case SH_CSS_ACC_ARG_PTR_OUT:
		case SH_CSS_ACC_ARG_PTR_IO:
		case SH_CSS_ACC_ARG_PTR_NOFLUSH:
		case SH_CSS_ACC_ARG_PTR_STABLE:
			value = &firmware->header.sp_args[i].value;
			size = sizeof(void *);
			break;
		case SH_CSS_ACC_ARG_FRAME:
			size = sizeof(struct sh_css_frame);
			break;
		}
		if (copy && value) {
			if (to_sp)
				sh_css_sp_dmem_store(sp_address, value, size);
			else
				sh_css_sp_dmem_load(sp_address, value, size);
		}
		sp_address += size;
	}
}

#if 0
static struct sh_css_acc_fw *current_firmware;

static void
init_dmem(struct sh_css_sp_init_dmem_cfg *init_dmem_cfg)
{
#ifdef C_RUN
	(void) init_dmem_cfg;
#else
	struct sh_css_acc_fw_hdr *header
		= (struct sh_css_acc_fw_hdr *)&current_firmware->header;
	upload_var(header->sp.fw.dmem_init_data,
			init_dmem_cfg, sizeof(*init_dmem_cfg));

	_hrt_cell_start(SP, header->sp.dmem_init);
	sh_css_hrt_sp_wait();
#endif
}
#endif

/* Start the sp, which will start the isp.
*/
enum sh_css_err
sh_css_acc_start(struct sh_css_acc_fw *firmware,
		 struct sh_css_binary_args *args)
{
	struct sh_css_acc_fw_hdr *header
		= (struct sh_css_acc_fw_hdr *)&firmware->header;
	bool has_extension_args = (args != NULL);
	bool is_extension = (header->type != SH_CSS_ACC_STANDALONE);
	const struct sh_css_sp_fw *sp_fw = &header->sp.fw;
	const unsigned char *sp_program;
#if !defined(C_RUN) && !defined(HRT_UNSCHED)
	const unsigned char *isp_program;
#endif

	*(const void **)&sp_fw->text = SH_CSS_ACC_SP_CODE(firmware);
	*(const void **)&sp_fw->data = SH_CSS_ACC_SP_DATA(firmware);

	if (!header->loaded)
		return sh_css_err_invalid_arguments;
	if (has_extension_args != is_extension)
		return sh_css_err_invalid_arguments;

	/* NOTE: standalone accelerators have their (shared buffer pointer)
	 * arguments flushed in "atomisp_acc_start()"
	 */
	if (is_extension)
		sh_css_flush(firmware);

	sp_program = sh_css_sp_load_program(sp_fw,
					    SH_CSS_ACC_PROG_NAME(firmware),
					    (void *)firmware->header.sp_code,
					    false);
	if (!sp_program)
		return sh_css_err_cannot_allocate_memory;
	firmware->header.sp_code = sp_program;
#if !defined(C_RUN) && !defined(HRT_UNSCHED)
	isp_program = sh_css_acc_upload_isp_code(firmware);
	if (!isp_program)
		return sh_css_err_cannot_allocate_memory;
#endif

#ifdef C_RUN
	header->sp.init(firmware);
#endif

	if (args) {
		upload_frame(header->sp.input,  args->in_frame);
		upload_frame(header->sp.output, args->out_frame);
		upload_frame(header->sp.out_vf, args->out_vf_frame);
		upload_frame(header->sp.extra,  args->extra_frame);
		upload_int  (header->sp.vf_downscale_bits,
			     &args->vf_downscale_log2);
	}
	copy_sp_arguments(firmware, true);

	/* Start the firmware on the sp, which will start the isp */
#ifdef C_RUN
	/* No need to run the dmem_init in crun */
	sh_css_sp_do_invalidate_mmu();
	csim_processor_set_crun_func(SP, header->sp.entry);
	hrt_ctl_run(SP, 1);
	hrt_ctl_start(SP);
	hrt_sleep();
#elif defined(HRT_CSIM)
	sh_css_sp_do_invalidate_mmu();
	_hrt_cell_start(SP, header->sp.entry);
#else
	sh_css_sp_start((unsigned int)header->sp.entry);
#endif
	return sh_css_success;
}

/* To be called when acceleration has terminated.
*/
void
sh_css_acc_done(struct sh_css_acc_fw *firmware)
{
	copy_sp_arguments(firmware, false);
}

void
sh_css_acc_wait(void)
{
	sh_css_hrt_sp_wait();
}

/* Flag abortion of acceleration */
void sh_css_acc_abort(struct sh_css_acc_fw *firmware)
{
	unsigned int t = true;
	upload_int(firmware->header.sp.css_abort, &t);
}
