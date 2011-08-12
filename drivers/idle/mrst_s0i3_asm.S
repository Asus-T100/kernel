/*
 * mrst_s0i3_asm.S - super-deep sleep state for the Moorestown MID platform
 *
 * Copyright (c) 2011, Intel Corporation.
 * H. Peter Anvin <hpa@linux.intel.com>
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
 */

#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/asm.h>
#include <asm/msr-index.h>
#include <asm/page.h>
#include <asm/segment.h>

#define MRST_C6_HINTS_EAX	0x52
#define MRST_C6_HINTS_ECX	1

/* Physical address */
#define pa(X) ((X) - __PAGE_OFFSET)

	.code32		/* MRST boots up in flat 32-bit mode */
	.text
ENTRY(mrst_s0i3_entry)
	pushl	%edi
	pushl	%esi
	pushl	%ebp
	pushl	%ebx

	movl	%esp, %ebp
	movl	%eax, %esi	/* Command code to write to PM_CMD  */
	movl	%edx, %edi	/* Linear address of the PM_CMD register */

	sldt	%eax
	pushl	%eax

	subl	$8, %esp
	sidtl	2(%esp)

	pushl	%gs
	pushl	%fs
	pushl	%es
	pushl	%ds

	movl	%cr3, %eax
	pushl	%eax

	subl	$8, %esp
	sgdtl	2(%esp)

	movl	%cr0, %eax
	pushl	%eax

	movl	%cr4, %eax
	pushl	%eax

	movl	$(MSR_EFER), %ecx
	rdmsr
	pushl	%edx
	pushl	%eax

	movl	$(MSR_IA32_MISC_ENABLE), %ecx
	rdmsr
	pushl	%edx
	pushl	%eax

	movl	28(%esp), %eax	/* GDT base */
	subl	$(__PAGE_OFFSET), %eax	/* Convert to a physical address */
	pushl	%eax
	pushl	28(%esp)		/* GDT length, pad */

	pushfl

	leal	(-__PAGE_OFFSET)(%esp), %eax
	movl	%eax, mrst_s0i3_resume_stack

	movl	%esi, (%edi)		/* Set the PM_CMD register for S0i3 */
	call	mrst_s0i3_wait_for_msi_disabled

	movl	%esp, %eax		/* As good as anything... */
	xorl	%edx, %edx
	xorl	%ecx, %ecx
	monitor

	movl	$MRST_C6_HINTS_EAX, %eax
	movl	$MRST_C6_HINTS_ECX, %ecx
	mwait

	/* If MWAIT wakes us up, assume something happened... */
	movl	%ebp, %esp

	xorl	%eax, %eax		/* Not really S0i3 */
	popl	%ebx
	popl	%ebp
	popl	%esi
	popl	%edi
	ret
ENDPROC(mrst_s0i3_entry)

/*
 * After S0i3 the MRST firmare will put us back in 32-bit flat mode
 */
ENTRY(mrst_s0i3_resume)
	cli

	movl	pa(mrst_s0i3_resume_stack), %esp
	popfl
	lgdtl	2(%esp)			/* Physical GDT pointer */

	addl	$8, %esp
	movl	$(__KERNEL_DS), %eax
	movl	%eax, %ss
	movl	%eax, %ds

	popl	%eax
	popl	%edx
	movl	$(MSR_IA32_MISC_ENABLE), %ecx
	wrmsr

	popl	%eax
	popl	%edx
	movl	$(MSR_EFER), %ecx
	wrmsr

	popl	%eax
	movl	%eax, %cr4

	movl	$pa(initial_page_table), %eax
	movl	%eax, %cr3

	popl	%eax
	movl	%eax, %cr0		/* Enables paging! */
	ljmpl	$(__KERNEL_CS), $1f
1:

	addl	$__PAGE_OFFSET, %esp
	lgdtl	2(%esp)			/* Linear GDT pointer */
	popl	%eax			/* GDT length + junk */
	popl	%ebx			/* Linear address of GDT */

	popl	%eax
	movl	%eax, %cr3

	movl	$(GDT_ENTRY_TSS*8), %eax

	/* Clear the TSS busy bit: %ebx == GDT.base */
	andb	$~0x02, (GDT_ENTRY_TSS*8+5)(%ebx)
	ltr	%ax			/* Set the TSS */

	popl	%ds
	popl	%es
	popl	%fs
	popl	%gs

	/* x86-64 would need to restore MSR_GS_BASE too */

	lidtl	2(%esp)
	addl	$8, %esp

	popl	%eax
	lldt	%ax

	movl	$1, %eax		/* Resume from actual S0i3 */
	popl	%ebx
	popl	%ebp
	popl	%esi
	popl	%edi

	ret
ENDPROC(mrst_s0i3_resume)

/*
 * Simple loop in C6 MWAIT
 */
	__INITRODATA
	.code16
	.balign	64
ENTRY(s0i3_trampoline_data)
r_base = .
	cli
	movw	%cs, %ax
	movw	%ax, %ss
	movw	%ax, %ds
	movw	%ax, %es
	movw	%ax, %fs
	movw	%ax, %gs

	/* Disable autodemote */
	movl	$MSR_NHM_SNB_PKG_CST_CFG_CTL, %ecx
	rdmsr
	andl	$(~(1 << 25)), %eax
	wrmsr

1:
	movl	$(dummy_mem - r_base), %eax
	clflush	(%eax)

	xorl	%edx, %edx
	xorl	%ecx, %ecx
	monitor

	movl	$MRST_C6_HINTS_EAX, %eax
	xorl	%ecx, %ecx	/* No wakeups necessary or wanted */
	mwait
	jmp	1b

	.balign	64
	/* One cache line of otherwise unused memory */
dummy_mem:
	.space 64
END(s0i3_trampoline_data)
ENTRY(s0i3_trampoline_data_end)

	.bss
	.balign	4
mrst_s0i3_resume_stack:
	.space	4
END(mrst_s0i3_resume_stack)
