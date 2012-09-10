/*
 * linux/arch/x86/lib/sse.c
 *
 * Copyright 2004 Jens Maurer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Send feedback to <Jens.Maurer@gmx.net>
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/preempt.h>
#include <asm/page.h>
#include <asm/system.h>


/*
 *	SSE library helper functions
 */

#define SSE_START(cr0) do { \
	preempt_disable(); \
	cr0 = read_cr0(); \
	clts(); \
	} while (0)


#define SSE_END(cr0) do { \
	write_cr0(cr0); \
	preempt_enable(); \
	} while (0)

void sse_clear_page(void *page)
{
	unsigned char xmm_save[16];
	unsigned int cr0;
	int i;

	SSE_START(cr0);
	asm volatile ("movups %%xmm0, (%0)\n\t"
		     "xorps %%xmm0, %%xmm0"
		     : : "r" (xmm_save));
	for (i = 0; i < PAGE_SIZE/16/4; i++) {
		asm volatile("movntps %%xmm0,   (%0)\n\t"
			     "movntps %%xmm0, 16(%0)\n\t"
			     "movntps %%xmm0, 32(%0)\n\t"
			     "movntps %%xmm0, 48(%0)"
			     : : "r"(page) : "memory");
		page += 16*4;
	}
	asm volatile ("sfence\n\t"
		     "movups (%0), %%xmm0"
		     : : "r" (xmm_save) : "memory");
	SSE_END(cr0);
}

void sse_copy_page(void *to, void *from)
{
	unsigned char xmm_save[16*4+15] __attribute__((aligned(16)));
	unsigned int cr0;
	int i;

	SSE_START(cr0);
	asm volatile ("movaps %%xmm0,   (%0)\n\t"
		     "movaps %%xmm1, 16(%0)\n\t"
		     "movaps %%xmm2, 32(%0)\n\t"
		     "movaps %%xmm3, 48(%0)"
		     : : "r" (xmm_save));
	for (i = 0; i < 4096/16/4; i++) {
		asm volatile("movaps   (%0), %%xmm0\n\t"
			     "movaps 16(%0), %%xmm1\n\t"
			     "movaps 32(%0), %%xmm2\n\t"
			     "movaps 48(%0), %%xmm3\n\t"
			     "movntps %%xmm0,   (%1)\n\t"
			     "movntps %%xmm1, 16(%1)\n\t"
			     "movntps %%xmm2, 32(%1)\n\t"
			     "movntps %%xmm3, 48(%1)"
			     : : "r" (from), "r" (to) : "memory");
		from += 16*4;
		to += 16*4;
	}
	asm volatile ("sfence\n"
		     "movaps   (%0), %%xmm0\n\t"
		     "movaps 16(%0), %%xmm1\n\t"
		     "movaps 32(%0), %%xmm2\n\t"
		     "movaps 48(%0), %%xmm3"
		     : : "r" (xmm_save) : "memory");
	SSE_END(cr0);
}

void activate_sse_replacements(void)
{
	if (cpu_has_xmm && (mmu_cr4_features & X86_CR4_OSFXSR)) {
		__sse_clear_page = &sse_clear_page;
		__sse_copy_page = &sse_copy_page;
	}
}
