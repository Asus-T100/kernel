/*
 * ssse3_memcpy6.S: ssse3 based optimized memcpy
 *
 * (C) Copyright 2011 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef MEMCPY
# define MEMCPY         ssse3_memcpy6
#endif

#ifndef L
# define L(label)	.L##label
#endif

#ifndef ALIGN
# define ALIGN(n)	.p2align n
#endif

#ifndef cfi_startproc
# define cfi_startproc			.cfi_startproc
#endif

#ifndef cfi_endproc
# define cfi_endproc			.cfi_endproc
#endif

#ifndef cfi_rel_offset
# define cfi_rel_offset(reg, off)	.cfi_rel_offset reg, off
#endif

#ifndef cfi_restore
# define cfi_restore(reg)		.cfi_restore reg
#endif

#ifndef cfi_adjust_cfa_offset
# define cfi_adjust_cfa_offset(off)	.cfi_adjust_cfa_offset off
#endif

#ifndef cfi_remember_state
# define cfi_remember_state		.cfi_remember_state
#endif

#ifndef cfi_restore_state
# define cfi_restore_state		.cfi_restore_state
#endif

#ifndef ENTRY
# define ENTRY(name)			\
	.type name,  @function; 	\
	.globl name;			\
	.p2align 4;			\
name:					\
	cfi_startproc
#endif

#ifndef END
# define END(name)			\
	cfi_endproc;			\
	.size name, .-name
#endif

#ifdef USE_AS_BCOPY
# define SRC		PARMS
# define DEST		SRC+4
# define LEN		DEST+4
#else
# define DEST		PARMS
# define SRC		DEST+4
# define LEN		SRC+4
#endif

#define CFI_PUSH(REG)						\
  cfi_adjust_cfa_offset (4);					\
  cfi_rel_offset (REG, 0)

#define CFI_POP(REG)						\
  cfi_adjust_cfa_offset (-4);					\
  cfi_restore (REG)

#define PUSH(REG)	pushl REG; CFI_PUSH (REG)
#define POP(REG)	popl REG; CFI_POP (REG)

#ifdef SHARED
# define PARMS		8		/* Preserve EBX.  */
# define ENTRANCE	PUSH (%ebx);
# define RETURN_END	POP (%ebx); ret
# define RETURN		RETURN_END; CFI_PUSH (%ebx)
# define JMPTBL(I, B)	I - B

/* Load an entry in a jump table into EBX and branch to it.  TABLE is a
   jump table with relative offsets.  INDEX is a register contains the
   index into the jump table.   SCALE is the scale of INDEX. */
# define BRANCH_TO_JMPTBL_ENTRY(TABLE, INDEX, SCALE)		\
    /* We first load PC into EBX.  */				\
    call	__i686.get_pc_thunk.bx;				\
    /* Get the address of the jump table.  */			\
    addl	$(TABLE - .), %ebx;				\
    /* Get the entry and convert the relative offset to the	\
       absolute address.  */					\
    addl	(%ebx,INDEX,SCALE), %ebx;			\
    /* We loaded the jump table.  Go.  */			\
    jmp		*%ebx

	.section	.gnu.linkonce.t.__i686.get_pc_thunk.bx,"ax",@progbits
	.globl	__i686.get_pc_thunk.bx
	.hidden	__i686.get_pc_thunk.bx
	ALIGN (4)
	.type	__i686.get_pc_thunk.bx,@function
__i686.get_pc_thunk.bx:
	mov	(%esp), %ebx
	ret
#else
# define PARMS		4
# define ENTRANCE
# define RETURN_END	ret
# define RETURN		RETURN_END
# define JMPTBL(I, B)	I

/* Branch to an entry in a jump table.  TABLE is a jump table with
   absolute offsets.  INDEX is a register contains the index into the
   jump table.  SCALE is the scale of INDEX. */
# define BRANCH_TO_JMPTBL_ENTRY(TABLE, INDEX, SCALE)		\
    jmp		*TABLE(,INDEX,SCALE);				\
    ud2

#endif

	.section .text.ssse3,"ax",@progbits
ENTRY (MEMCPY)
	ENTRANCE
	mov	LEN(%esp), %ecx
	mov	SRC(%esp), %eax
	mov	DEST(%esp), %edx
	cfi_remember_state

#ifdef USE_AS_MEMMOVE
	cmp	%eax, %edx
	jb	L(copy_forward)
	je	L(fwd_write_0bytes)
	cmp	$79, %ecx
	jbe	L(bk_write)
	jmp	L(copy_backward)
L(copy_forward):
#endif
	cmp	$79, %ecx
	ja	L(80bytesormore)

#ifndef USE_AS_MEMMOVE
	cmp	%dl, %al
	jb	L(bk_write)
#endif
	add	%ecx, %edx
	add	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)
	ALIGN (4)
L(bk_write):
	BRANCH_TO_JMPTBL_ENTRY (L(table_48_bytes_bwd), %ecx, 4)

	ALIGN (4)
/* ECX > 32 and EDX is 4 byte aligned.  */
L(80bytesormore):
	PUSH (%edi)
#ifdef SHARED_CACHE_SIZE_HALF
	cmp	$SHARED_CACHE_SIZE_HALF, %ecx
	ja	L(large_page)
	cmp	$DATA_CACHE_SIZE_HALF, %ecx
	mov	$0, %edi
#else
# ifdef SHARED
	call	__i686.get_pc_thunk.bx
L(copy_fwd_offset_1):
	lea	_GLOBAL_OFFSET_TABLE_(%ebx), %edi
	cmp	__x86_shared_cache_size_half@GOTOFF(%edi), %ecx
L(copy_fwd_offset_2):
	lea	((L(copy_fwd_offset_2) - L(copy_fwd_offset_1)) \
				+ L(shl_table) - .)(%ebx), %ebx
	ja	L(large_page)
	cmp	__x86_data_cache_size_half@GOTOFF(%edi), %ecx
	mov	$0, %edi
# else
	cmp	__x86_shared_cache_size_half, %ecx
	ja	L(large_page)
	cmp	__x86_data_cache_size_half, %ecx
	mov	$0, %edi
# endif
#endif

#ifndef USE_AS_MEMMOVE
	ja	L(memcpy_in_L2)
#else
	ja	L(memmove_in_L2)
	mov	$7, %edi
L(memmove_in_L2):
#endif
	PUSH (%esi)
	mov	%edx, %esi
	and	$-16, %edx
	movdqu	(%eax), %xmm0
	add	$16, %edx

/*edi offset is 7 if copy size is within L1 data cache */
	add	%edi, %edx
	mov	%esi, %edi
#ifdef USE_AS_MEMMOVE
	test	$0xf, %edx
	jz	L(copy_fwd_in_L2)
	add	$7, %edi
L(copy_fwd_in_L2):
#endif
	sub	%edx, %edi
	add	%edi, %ecx
	sub	%edi, %eax

	mov	%eax, %edi
	and	$0xf, %edi
	jz	L(shl_0)
	sub	%edi, %eax
	sub	$0x40, %ecx
	movdqa	(%eax), %xmm1
	movdqu	%xmm0, (%esi)
#ifdef SHARED
	addl	(%ebx,%edi ,4), %ebx
	mov	%ebx, %edi
#else
	mov	L(shl_table)(,%edi, 4),	%edi;
#endif
#ifdef USE_AS_MEMMOVE
	mov	%edx, %esi
	and	$0xf, %esi
	and	$-0x10, %edx
	add	%esi, %edi
#else
	lea	7(%edi), %edi
#endif
	POP	(%esi)
	jmp	*%edi
	ud2

	cfi_restore_state
	cfi_remember_state
	ALIGN (4)
L(copy_backward):
	PUSH	(%edi)
L(memcpy_in_L2):
	movdqu	-16(%eax, %ecx), %xmm0
	add	%ecx, %eax
	PUSH	(%esi)
	lea	-16(%edx, %ecx), %esi
	add	%ecx, %edx

	mov	%edx, %edi
	and	$0xf, %edi
	xor	%edi, %edx
	sub	%edi, %eax
	sub	%edi, %ecx

#ifdef SHARED_CACHE_SIZE_HALF
	cmp	$SHARED_CACHE_SIZE_HALF, %ecx
	mov	$DATA_CACHE_SIZE_HALF, %edi
	ja	L(large_page_bwd)
#else
# ifdef SHARED
	call	__i686.get_pc_thunk.bx
L(copy_bwd_offset_1):
	lea	_GLOBAL_OFFSET_TABLE_(%ebx), %edi
	cmp	__x86_shared_cache_size_half@GOTOFF(%edi), %ecx
L(copy_bwd_offset_2):
	lea	((L(copy_bwd_offset_2) - L(copy_bwd_offset_1)) \
				+ L(shl_bwd_table) - .)(%ebx), %ebx
	ja	L(large_page_bwd)
	mov	__x86_data_cache_size_half@GOTOFF(%edi), %edi
# else
	cmp	__x86_shared_cache_size_half, %ecx
	mov	__x86_data_cache_size_half, %edi
	ja	L(large_page_bwd)
# endif
#endif
	cmp	%edi, %ecx
	mov	%eax, %edi
	ja	L(copy_bwd_in_L2)
	lea	7(%edx), %edx
L(copy_bwd_in_L2):
	and	$0xf, %edi
	lea	-64(%ecx), %ecx
	jz	L(shl_0_bwd)
	sub	%edi, %eax
	movdqa	(%eax), %xmm1
	movdqu	%xmm0, (%esi)
	mov	%edx, %esi
#ifdef SHARED
	addl	(%ebx,%edi ,4), %ebx
	mov	%ebx, %edi
#else
	mov	L(shl_bwd_table)(,%edi, 4), %edi;
#endif
	and	$0xf, %esi
	and	$-0x10, %edx
	add	%esi, %edi
	POP	(%esi)
	jmp	*%edi
	ud2

	cfi_restore_state
	cfi_remember_state
	CFI_PUSH (%edi)
	CFI_PUSH (%esi)
	ALIGN (4)
L(shl_0):
	movdqa	(%eax), %xmm1
	cmp	$144, %ecx
	movdqu	%xmm0, (%esi)
	ja	L(shl_0_gobble)
	movaps	0x10(%eax), %xmm2
#ifdef USE_AS_MEMMOVE
	lea	-7(%edx), %edx
#endif
	movaps	0x20(%eax), %xmm3
	sub	$0x40, %ecx
	movaps	0x30(%eax), %xmm4
	lea	0x40(%eax), %eax
	movdqa	%xmm1, (%edx)
	POP (%esi)
	movaps	%xmm2, 0x10(%edx)
	movaps	%xmm3, 0x20(%edx)
	POP (%edi)
	movaps	%xmm4, 0x30(%edx)
	cmp	$0x20, %ecx
	lea	0x40(%edx), %edx
	jb	L(shl_0_less_32bytes)
	lea	0x20(%edx), %edx
	movdqa	(%eax), %xmm1
	sub	$0x20, %ecx
	movaps	0x10(%eax), %xmm2
	lea	0x20(%eax), %eax
	movaps	%xmm1, -0x20(%edx)
	movaps	%xmm2, -0x10 (%edx)
L(shl_0_less_32bytes):
	cmp	$0x10, %ecx
	jb	L(shl_0_less_16bytes)
	movdqa	(%eax), %xmm1
	lea	0x10(%eax), %eax
	lea	-0x10(%ecx), %ecx
	movaps	%xmm1, (%edx)
	lea	0x10(%edx), %edx
L(shl_0_less_16bytes):
	add	%ecx, %edx
	add	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	cfi_restore_state
	cfi_remember_state
	CFI_PUSH (%edi)
	CFI_PUSH (%esi)
L(shl_0_gobble):
	sub	$0x90, %ecx
#ifdef USE_AS_MEMMOVE
	test	$0xf, %edx
	jz	L(shl_0_fwd_gobble_mem_start)
#endif
	lea	0x10(%eax), %eax
#ifdef USE_AS_MEMMOVE
	movaps	%xmm1, -0x7(%edx)
	lea	0x09(%edx), %edx
#else
	movaps	%xmm1, (%edx)
	lea	0x10(%edx), %edx
#endif
	ALIGN (4)
L(shl_0_fwd_gobble_loop):
	movdqa	(%eax), %xmm0
	sub	$0x80, %ecx
	movaps	0x10(%eax), %xmm1
	movaps	0x20(%eax), %xmm2
	movaps	0x30(%eax), %xmm3
	movaps	0x40(%eax), %xmm4
	lea	0x80(%edx), %edx
	movaps	0x50(%eax), %xmm5
	movaps	0x60(%eax), %xmm6
	movaps	0x70(%eax), %xmm7
	lea	0x80(%eax), %eax
	movaps	%xmm0, -0x80(%edx)
	movaps	%xmm1, -0x70(%edx)
	movaps	%xmm2, -0x60(%edx)
	movaps	%xmm3, -0x50(%edx)
	movaps	%xmm4, -0x40(%edx)
	movaps	%xmm5, -0x30(%edx)
	movaps	%xmm6, -0x20(%edx)
	movaps	%xmm7, -0x10(%edx)
	jae	L(shl_0_fwd_gobble_loop)
	cmp	$-0x40, %ecx
	lea	0x80(%ecx), %ecx
	jl	L(shl_0_less_64bytes)

	movdqa	(%eax), %xmm0
	sub	$0x40, %ecx
	movaps	0x10(%eax), %xmm1
	lea	0x40(%edx), %edx
	movaps	0x20(%eax), %xmm2
	movaps	0x30(%eax), %xmm3

	movaps	%xmm0, -0x40(%edx)
	movaps	%xmm1, -0x30(%edx)

	lea	0x40(%eax), %eax

	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_0_less_64bytes):
	cmp	$0x20, %ecx
	jb	L(shl_0_gobble_less_32bytes)
	movdqa	(%eax), %xmm0
	sub	$0x20, %ecx
	movaps	0x10(%eax), %xmm1
	lea	0x20(%eax), %eax
	movdqa	%xmm0, (%edx)
	movaps	%xmm1, 0x10(%edx)
	lea	0x20(%edx), %edx
L(shl_0_gobble_less_32bytes):
	cmp	$0x10, %ecx
	jb	L(shl_0_gobble_less_16bytes)
	sub	$0x10, %ecx
	movdqa	(%eax), %xmm0
	lea	0x10(%eax), %eax
	movdqa	%xmm0, (%edx)
	lea	0x10(%edx), %edx
L(shl_0_gobble_less_16bytes):
	add	%ecx, %edx
	add	%ecx, %eax
	POP	(%esi)
	POP	(%edi)
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	CFI_PUSH (%esi)
	ALIGN (4)
L(shl_0_fwd_gobble_mem_start):
	lea	0x10(%eax), %eax
	movaps	%xmm1, (%edx)
	lea	0x10(%edx), %edx
L(shl_0_fwd_gobble_mem_loop):
	prefetchnta 0x1c0(%eax)
	prefetchnta 0x280(%eax)
	sub	$0x80, %ecx
	movdqa	(%eax), %xmm0
	movaps	0x10(%eax), %xmm1
	movaps	0x20(%eax), %xmm2
	movaps	0x30(%eax), %xmm3
	lea	0x80(%edx), %edx
	movaps	0x40(%eax), %xmm4
	movaps	0x50(%eax), %xmm5
	movaps	0x60(%eax), %xmm6
	movaps	0x70(%eax), %xmm7
	lea	0x80(%eax), %eax
	movaps	%xmm0, -0x80(%edx)
	movaps	%xmm1, -0x70(%edx)
	movaps	%xmm2, -0x60(%edx)
	movaps	%xmm3, -0x50(%edx)
	movaps	%xmm4, -0x40(%edx)
	movaps	%xmm5, -0x30(%edx)
	movaps	%xmm6, -0x20(%edx)
	movaps	%xmm7, -0x10(%edx)
	jae	L(shl_0_fwd_gobble_mem_loop)
	cmp	$-0x40, %ecx
	lea	0x80(%ecx), %ecx
	jl	L(shl_0_mem_less_64bytes)

	movdqa	(%eax), %xmm0
	sub	$0x40, %ecx
	movaps	0x10(%eax), %xmm1
	add	$0x40, %edx
	movaps	0x20(%eax), %xmm2
	movaps	0x30(%eax), %xmm3

	movaps	%xmm0, -0x40(%edx)
	movaps	%xmm1, -0x30(%edx)

	add	$0x40, %eax

	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_0_mem_less_64bytes):
	cmp	$0x20, %ecx
	jb	L(shl_0_mem_less_32bytes)
	movdqa	(%eax), %xmm0
	sub	$0x20, %ecx
	movaps	0x10(%eax), %xmm1
	add	$0x20, %eax
	movdqa	%xmm0, (%edx)
	movaps	%xmm1, 0x10(%edx)
	add	$0x20, %edx
L(shl_0_mem_less_32bytes):
	cmp	$0x10, %ecx
	jb	L(shl_0_mem_less_16bytes)
	sub	$0x10, %ecx
	movdqa	(%eax), %xmm0
	add	$0x10, %eax
	movdqa	%xmm0, (%edx)
	add	$0x10, %edx
L(shl_0_mem_less_16bytes):
	add	%ecx, %edx
	add	%ecx, %eax
	POP	(%esi)
	POP	(%edi)
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	CFI_PUSH (%esi)
	ALIGN (4)
L(shl_0_bwd):
	cmp	$80, %ecx
	movaps	-0x10(%eax), %xmm4
	jae	L(shl_0_bwd_gobble)
	movaps	-0x10(%eax), %xmm4
	lea	-0x47(%edx), %edx
	movaps	-0x20(%eax), %xmm1
	movaps	-0x30(%eax), %xmm2
	movaps	-0x40(%eax), %xmm3
	lea	-0x40(%eax), %eax
	movaps	%xmm4, 0x30(%edx)
	movaps	%xmm1, 0x20(%edx)
	cmp	$0x20, %ecx
	movaps	%xmm2, 0x10(%edx)
	movdqa	%xmm3, (%edx)
	jb	L(shl_0_bwd_less_32bytes)
	lea	-0x20(%edx), %edx
	movaps	-0x10(%eax), %xmm4
	sub	$0x20, %ecx
	movaps	-0x20(%eax), %xmm1
	lea	-0x20(%eax), %eax
	movaps	%xmm4, 0x10(%edx)
	movdqa	%xmm1, (%edx)
L(shl_0_bwd_less_32bytes):
	movdqu	%xmm0, (%esi)
	sub	%ecx, %edx
	POP (%esi)
	POP (%edi)
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY (L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	CFI_PUSH (%esi)
L(shl_0_bwd_gobble):
	test	$0xf, %edx
	movdqu	%xmm0, (%esi)
	lea	-0x50(%ecx), %ecx
	je	L(shl_0_bwd_gobble_mem_start)
	lea	-0x10(%eax), %eax
	movaps	%xmm4, -0x17(%edx)
	lea	-0x17(%edx), %edx
	ALIGN (4)
L(shl_0_bwd_gobble_loop):
	sub	$0x80, %ecx
	movaps	-0x10(%eax), %xmm0
	movaps	-0x20(%eax), %xmm1
	movaps	-0x30(%eax), %xmm2
	movaps	-0x40(%eax), %xmm3
	lea	-0x80(%edx), %edx
	movaps	-0x50(%eax), %xmm4
	movaps	-0x60(%eax), %xmm5
	movaps	-0x70(%eax), %xmm6
	movaps	-0x80(%eax), %xmm7
	lea	-0x80(%eax), %eax
	movaps	%xmm0, 0x70(%edx)
	movaps	%xmm1, 0x60(%edx)
	movaps	%xmm2, 0x50(%edx)
	movaps	%xmm3, 0x40(%edx)
	movaps	%xmm4, 0x30(%edx)
	movaps	%xmm5, 0x20(%edx)
	movaps	%xmm6, 0x10(%edx)
	movdqa	%xmm7, (%edx)
	jae	L(shl_0_bwd_gobble_loop)
	cmp	$-0x40, %ecx
	lea	0x80(%ecx), %ecx
	jl	L(shl_0_bwd_gobble_less_64bytes)
	sub	$0x40, %edx
	movaps	-0x10(%eax), %xmm0
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm1

	movaps	-0x30(%eax), %xmm2
	movaps	-0x40(%eax), %xmm3
	sub	$0x40, %eax
	movaps	%xmm0, 0x30(%edx)
	movaps	%xmm1, 0x20(%edx)

	movaps	%xmm2, 0x10(%edx)
	movdqa	%xmm3, (%edx)
L(shl_0_bwd_gobble_less_64bytes):
	cmp	$0x20, %ecx
	POP	(%esi)
	jb	L(shl_0_bwd_gobble_less_32bytes)
	sub	$0x20, %edx
	movaps	-0x10(%eax), %xmm0
	sub	$0x20, %ecx
	movaps	-0x20(%eax), %xmm1
	sub	$0x20, %eax
	movaps	%xmm0, 0x10(%edx)
	movdqa	%xmm1, (%edx)
L(shl_0_bwd_gobble_less_32bytes):
	cmp	$0x10, %ecx
	POP	(%edi)
	jb	L(shl_0_bwd_gobble_less_16bytes)
	sub	$0x10, %ecx
	movaps	-0x10(%eax), %xmm0
	sub	$0x10, %eax
	movaps	%xmm0, -0x10(%edx)
	sub	$0x10, %edx
L(shl_0_bwd_gobble_less_16bytes):
	sub	%ecx, %eax
	sub	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	CFI_PUSH (%esi)
	ALIGN (4)
L(shl_0_bwd_gobble_mem_start):
	lea	-0x10(%eax), %eax
	movaps	%xmm4, -0x10(%edx)
	lea	-0x10(%edx), %edx
L(shl_0_bwd_gobble_mem_loop):
	prefetchnta -0x1c0(%eax)
	prefetchnta -0x280(%eax)
	movaps	-0x10(%eax), %xmm0
	movaps	-0x20(%eax), %xmm1
	sub	$0x80, %ecx
	movaps	-0x30(%eax), %xmm2
	movaps	-0x40(%eax), %xmm3
	lea	-0x80(%edx), %edx
	movaps	-0x50(%eax), %xmm4
	movaps	-0x60(%eax), %xmm5
	movaps	-0x70(%eax), %xmm6
	movaps	-0x80(%eax), %xmm7
	lea	-0x80(%eax), %eax
	movaps	%xmm0, 0x70(%edx)
	movaps	%xmm1, 0x60(%edx)
	movaps	%xmm2, 0x50(%edx)
	movaps	%xmm3, 0x40(%edx)
	movaps	%xmm4, 0x30(%edx)
	movaps	%xmm5, 0x20(%edx)
	movaps	%xmm6, 0x10(%edx)
	movdqa	%xmm7, (%edx)
	jae	L(shl_0_bwd_gobble_mem_loop)
	cmp	$-0x40, %ecx
	lea	0x80(%ecx), %ecx
	jl	L(shl_0_bwd_gobble_mem_less_64bytes)

	movaps	-0x10(%eax), %xmm0
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm1

	movaps	%xmm0, -0x10(%edx)
	movaps	%xmm1, -0x20(%edx)

	movaps	-0x30(%eax), %xmm0
	movaps	-0x40(%eax), %xmm1
	sub	$0x40, %eax

	movaps	%xmm0, -0x30(%edx)
	movaps	%xmm1, -0x40(%edx)
	sub	$0x40, %edx
L(shl_0_bwd_gobble_mem_less_64bytes):
	cmp	$0x20, %ecx
	jb	L(shl_0_bwd_gobble_mem_less_32bytes)
	movaps	-0x10(%eax), %xmm0
	sub	$0x20, %ecx
	movaps	-0x20(%eax), %xmm1
	sub	$0x20, %eax
	movaps	%xmm0, -0x10(%edx)
	movaps	%xmm1, -0x20(%edx)
	sub	$0x20, %edx
L(shl_0_bwd_gobble_mem_less_32bytes):
	cmp	$0x10, %ecx
	jb	L(shl_0_bwd_gobble_mem_less_16bytes)
	sub	$0x10, %ecx
	movdqa	-0x10(%eax), %xmm0
	sub	$0x10, %eax
	movdqa	%xmm0, -0x10(%edx)
	sub	$0x10, %edx
L(shl_0_bwd_gobble_mem_less_16bytes):
	sub	%ecx, %eax
	sub	%ecx, %edx
	POP	(%esi)
	POP	(%edi)
	BRANCH_TO_JMPTBL_ENTRY (L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_1):
	prefetchnta 0x1c0(%eax)
L(shl_1_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movdqa	0x40(%eax), %xmm5
	movdqa	%xmm5, %xmm6
	palignr	$1, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$1, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$1, %xmm2, %xmm3
	palignr	$1, %xmm1, %xmm2
	movdqa	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_1_end)
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
	jmp	*%edi
	ud2
L(shl_1_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
	jae	L(shl_1_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jb	L(shl_1_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$1, %xmm2, %xmm3
	palignr	$1, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_1_end_less_32):
	POP	(%edi)
	lea	1(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_1_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$1, %xmm2, %xmm1
	palignr	$1, %xmm3, %xmm2
	palignr	$1, %xmm4, %xmm3
	palignr	$1, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_1_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_1_bwd_end):
	lea	1(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_1_bwd_end_less_32)
	movaps	-0x11(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x21(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$1, %xmm2, %xmm1
	palignr	$1, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_1_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_2):
	prefetchnta 0x1c0(%eax)
L(shl_2_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$2, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$2, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$2, %xmm2, %xmm3
	palignr	$2, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_2_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_2_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_2_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_2_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$2, %xmm2, %xmm3
	palignr	$2, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_2_end_less_32):
	POP	(%edi)
	lea	2(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_2_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$2, %xmm2, %xmm1
	palignr	$2, %xmm3, %xmm2
	palignr	$2, %xmm4, %xmm3
	palignr	$2, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_2_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_2_bwd_end):
	lea	2(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_2_bwd_end_less_32)
	movaps	-0x12(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x22(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$2, %xmm2, %xmm1
	palignr	$2, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_2_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_3):
	prefetchnta 0x1c0(%eax)
L(shl_3_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$3, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$3, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$3, %xmm2, %xmm3
	palignr	$3, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_3_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_3_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_3_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_3_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$3, %xmm2, %xmm3
	palignr	$3, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_3_end_less_32):
	POP	(%edi)
	lea	3(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_3_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$3, %xmm2, %xmm1
	palignr	$3, %xmm3, %xmm2
	palignr	$3, %xmm4, %xmm3
	palignr	$3, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_3_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_3_bwd_end):
	lea	3(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_3_bwd_end_less_32)
	movaps	-0x13(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x23(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$3, %xmm2, %xmm1
	palignr	$3, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_3_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_4):
	prefetchnta 0x1c0(%eax)
L(shl_4_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$4, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$4, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$4, %xmm2, %xmm3
	palignr	$4, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_4_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_4_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_4_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_4_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$4, %xmm2, %xmm3
	palignr	$4, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_4_end_less_32):
	POP	(%edi)
	lea	4(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_4_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$4, %xmm2, %xmm1
	palignr	$4, %xmm3, %xmm2
	palignr	$4, %xmm4, %xmm3
	palignr	$4, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_4_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_4_bwd_end):
	lea	4(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_4_bwd_end_less_32)
	movaps	-0x14(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x24(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$4, %xmm2, %xmm1
	palignr	$4, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_4_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_5):
	prefetchnta 0x1c0(%eax)
L(shl_5_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$5, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$5, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$5, %xmm2, %xmm3
	palignr	$5, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_5_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_5_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_5_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_5_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$5, %xmm2, %xmm3
	palignr	$5, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_5_end_less_32):
	POP	(%edi)
	lea	5(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_5_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$5, %xmm2, %xmm1
	palignr	$5, %xmm3, %xmm2
	palignr	$5, %xmm4, %xmm3
	palignr	$5, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_5_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_5_bwd_end):
	lea	5(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_5_bwd_end_less_32)
	movaps	-0x15(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x25(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$5, %xmm2, %xmm1
	palignr	$5, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_5_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_6):
	prefetchnta 0x1c0(%eax)
L(shl_6_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$6, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$6, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$6, %xmm2, %xmm3
	palignr	$6, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_6_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_6_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_6_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_6_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$6, %xmm2, %xmm3
	palignr	$6, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_6_end_less_32):
	POP	(%edi)
	lea	6(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_6_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$6, %xmm2, %xmm1
	palignr	$6, %xmm3, %xmm2
	palignr	$6, %xmm4, %xmm3
	palignr	$6, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_6_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_6_bwd_end):
	lea	6(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_6_bwd_end_less_32)
	movaps	-0x16(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x26(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$6, %xmm2, %xmm1
	palignr	$6, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_6_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_7):
	prefetchnta 0x1c0(%eax)
L(shl_7_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$7, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$7, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$7, %xmm2, %xmm3
	palignr	$7, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_7_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_7_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_7_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_7_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$7, %xmm2, %xmm3
	palignr	$7, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_7_end_less_32):
	POP	(%edi)
	lea	7(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_7_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$7, %xmm2, %xmm1
	palignr	$7, %xmm3, %xmm2
	palignr	$7, %xmm4, %xmm3
	palignr	$7, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_7_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_7_bwd_end):
	lea	7(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_7_bwd_end_less_32)
	movaps	-0x17(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x27(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$7, %xmm2, %xmm1
	palignr	$7, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_7_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_8):
	prefetchnta 0x1c0(%eax)
L(shl_8_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$8, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$8, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$8, %xmm2, %xmm3
	palignr	$8, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_8_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_8_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_8_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_8_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$8, %xmm2, %xmm3
	palignr	$8, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_8_end_less_32):
	POP	(%edi)
	lea	8(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_8_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$8, %xmm2, %xmm1
	palignr	$8, %xmm3, %xmm2
	palignr	$8, %xmm4, %xmm3
	palignr	$8, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_8_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_8_bwd_end):
	lea	8(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_8_bwd_end_less_32)
	movaps	-0x18(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x28(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$8, %xmm2, %xmm1
	palignr	$8, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_8_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_9):
	prefetchnta 0x1c0(%eax)
L(shl_9_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$9, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$9, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$9, %xmm2, %xmm3
	palignr	$9, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_9_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_9_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_9_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_9_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$9, %xmm2, %xmm3
	palignr	$9, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_9_end_less_32):
	POP	(%edi)
	lea	9(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)
	ALIGN (4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_9_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$9, %xmm2, %xmm1
	palignr	$9, %xmm3, %xmm2
	palignr	$9, %xmm4, %xmm3
	palignr	$9, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_9_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_9_bwd_end):
	lea	9(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_9_bwd_end_less_32)
	movaps	-0x19(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x29(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$9, %xmm2, %xmm1
	palignr	$9, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_9_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_10):
	prefetchnta 0x1c0(%eax)
L(shl_10_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$10, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$10, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$10, %xmm2, %xmm3
	palignr	$10, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_10_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_10_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_10_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_10_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$10, %xmm2, %xmm3
	palignr	$10, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_10_end_less_32):
	POP	(%edi)
	lea	10(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_10_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$10, %xmm2, %xmm1
	palignr	$10, %xmm3, %xmm2
	palignr	$10, %xmm4, %xmm3
	palignr	$10, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_10_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_10_bwd_end):
	lea	10(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_10_bwd_end_less_32)
	movaps	-0x1a(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x2a(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$10, %xmm2, %xmm1
	palignr	$10, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_10_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_11):
	prefetchnta 0x1c0(%eax)
L(shl_11_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$11, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$11, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$11, %xmm2, %xmm3
	palignr	$11, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_11_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_11_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_11_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_11_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$11, %xmm2, %xmm3
	palignr	$11, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_11_end_less_32):
	POP	(%edi)
	lea	11(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_11_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$11, %xmm2, %xmm1
	palignr	$11, %xmm3, %xmm2
	palignr	$11, %xmm4, %xmm3
	palignr	$11, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_11_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_11_bwd_end):
	lea	11(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_11_bwd_end_less_32)
	movaps	-0x1b(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x2b(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$11, %xmm2, %xmm1
	palignr	$11, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_11_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_12):
	prefetchnta 0x1c0(%eax)
L(shl_12_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$12, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$12, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$12, %xmm2, %xmm3
	palignr	$12, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_12_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_12_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_12_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_12_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$12, %xmm2, %xmm3
	palignr	$12, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_12_end_less_32):
	POP	(%edi)
	lea	12(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_12_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$12, %xmm2, %xmm1
	palignr	$12, %xmm3, %xmm2
	palignr	$12, %xmm4, %xmm3
	palignr	$12, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_12_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_12_bwd_end):
	lea	12(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_12_bwd_end_less_32)
	movaps	-0x1c(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x2c(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$12, %xmm2, %xmm1
	palignr	$12, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_12_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_13):
	prefetchnta 0x1c0(%eax)
L(shl_13_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$13, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$13, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$13, %xmm2, %xmm3
	palignr	$13, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_13_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_13_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_13_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_13_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$13, %xmm2, %xmm3
	palignr	$13, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_13_end_less_32):
	POP	(%edi)
	lea	13(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_13_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$13, %xmm2, %xmm1
	palignr	$13, %xmm3, %xmm2
	palignr	$13, %xmm4, %xmm3
	palignr	$13, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_13_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_13_bwd_end):
	lea	13(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_13_bwd_end_less_32)
	movaps	-0x1d(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x2d(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$13, %xmm2, %xmm1
	palignr	$13, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_13_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_14):
	prefetchnta 0x1c0(%eax)
L(shl_14_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$14, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$14, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$14, %xmm2, %xmm3
	palignr	$14, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_14_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_14_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_14_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_14_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$14, %xmm2, %xmm3
	palignr	$14, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_14_end_less_32):
	POP	(%edi)
	lea	14(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_14_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$14, %xmm2, %xmm1
	palignr	$14, %xmm3, %xmm2
	palignr	$14, %xmm4, %xmm3
	palignr	$14, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_14_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_14_bwd_end):
	lea	14(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_14_bwd_end_less_32)
	movaps	-0x1e(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x2e(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$14, %xmm2, %xmm1
	palignr	$14, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_14_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_15):
	prefetchnta 0x1c0(%eax)
L(shl_15_loop):
	movaps	0x10(%eax), %xmm2
	sub	$64, %ecx
	movaps	0x20(%eax), %xmm3
	movaps	0x30(%eax), %xmm4
	movaps	0x40(%eax), %xmm5
	movaps	%xmm5, %xmm6
	palignr	$15, %xmm4, %xmm5
	lea	0x40(%eax), %eax
	palignr	$15, %xmm3, %xmm4
	lea	0x40(%edx), %edx
	palignr	$15, %xmm2, %xmm3
	palignr	$15, %xmm1, %xmm2
	movaps	%xmm6, %xmm1
	movaps	%xmm2, -0x40(%edx)
	movaps	%xmm3, -0x30(%edx)
#ifdef USE_AS_MEMMOVE
	jb	L(shl_15_end)
#endif
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#ifdef USE_AS_MEMMOVE
	jmp	*%edi
	ud2
L(shl_15_end):
	movaps	%xmm4, -0x20(%edx)
	movaps	%xmm5, -0x10(%edx)
#else
	jae	L(shl_15_loop)
#endif
	cmp	$-0x20, %ecx
	lea	0x40(%ecx), %ecx
	jl	L(shl_15_end_less_32)
	movaps	0x10(%eax), %xmm2
	lea	0x20(%edx), %edx
	movaps	0x20(%eax), %xmm3
	lea	0x20(%eax), %eax
	palignr	$15, %xmm2, %xmm3
	palignr	$15, %xmm1, %xmm2
	sub	$0x20, %ecx
	movaps	%xmm2, -0x20(%edx)
	movaps	%xmm3, -0x10(%edx)
L(shl_15_end_less_32):
	POP	(%edi)
	lea	15(%eax,%ecx), %eax
	add	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

	CFI_PUSH (%edi)
	ALIGN (4)
L(shl_15_bwd):
	prefetchnta -0x1c0(%eax)
	movaps	-0x10(%eax), %xmm2
	sub	$0x40, %ecx
	movaps	-0x20(%eax), %xmm3
	movaps	-0x30(%eax), %xmm4
	movaps	-0x40(%eax), %xmm5
	lea	-0x40(%eax), %eax
	palignr	$15, %xmm2, %xmm1
	palignr	$15, %xmm3, %xmm2
	palignr	$15, %xmm4, %xmm3
	palignr	$15, %xmm5, %xmm4
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm5, %xmm1
	movaps	%xmm2, -0x20(%edx)
	lea	-0x40(%edx), %edx
	movaps	%xmm3, 0x10(%edx)
	jb	L(shl_15_bwd_end)
	movaps	%xmm4, (%edx)
	jmp	*%edi
	ud2
	ALIGN (4)
L(shl_15_bwd_end):
	lea	15(%eax), %eax
	cmp	$-0x20, %ecx
	movaps	%xmm4, (%edx)
	lea	0x40(%ecx), %ecx
	jl	L(shl_15_bwd_end_less_32)
	movaps	-0x1f(%eax), %xmm2
	sub	$0x20, %ecx
	movaps	-0x2f(%eax), %xmm3
	lea	-0x20(%eax), %eax
	palignr	$15, %xmm2, %xmm1
	palignr	$15, %xmm3, %xmm2
	movaps	%xmm1, -0x10(%edx)
	movaps	%xmm2, -0x20(%edx)
	lea	-0x20(%edx), %edx
L(shl_15_bwd_end_less_32):
	POP	(%edi)
	sub	%ecx, %edx
	sub	%ecx, %eax
	BRANCH_TO_JMPTBL_ENTRY(L(table_48_bytes_bwd), %ecx, 4)

	ALIGN (4)
L(fwd_write_76bytes):
	mov	-76(%eax), %ecx
	mov	%ecx, -76(%edx)
L(fwd_write_72bytes):
	mov	-72(%eax), %ecx
	mov	%ecx, -72(%edx)
L(fwd_write_68bytes):
	mov	-68(%eax), %ecx
	mov	%ecx, -68(%edx)
L(fwd_write_64bytes):
	mov	-64(%eax), %ecx
	mov	%ecx, -64(%edx)
L(fwd_write_60bytes):
	mov	-60(%eax), %ecx
	mov	%ecx, -60(%edx)
L(fwd_write_56bytes):
	mov	-56(%eax), %ecx
	mov	%ecx, -56(%edx)
L(fwd_write_52bytes):
	mov	-52(%eax), %ecx
	mov	%ecx, -52(%edx)
L(fwd_write_48bytes):
	mov	-48(%eax), %ecx
	mov	%ecx, -48(%edx)
L(fwd_write_44bytes):
	mov	-44(%eax), %ecx
	mov	%ecx, -44(%edx)
L(fwd_write_40bytes):
	mov	-40(%eax), %ecx
	mov	%ecx, -40(%edx)
L(fwd_write_36bytes):
	mov	-36(%eax), %ecx
	mov	%ecx, -36(%edx)
L(fwd_write_32bytes):
	mov	-32(%eax), %ecx
	mov	%ecx, -32(%edx)
L(fwd_write_28bytes):
	mov	-28(%eax), %ecx
	mov	%ecx, -28(%edx)
L(fwd_write_24bytes):
	mov	-24(%eax), %ecx
	mov	%ecx, -24(%edx)
L(fwd_write_20bytes):
	mov	-20(%eax), %ecx
	mov	%ecx, -20(%edx)
L(fwd_write_16bytes):
	mov	-16(%eax), %ecx
	mov	%ecx, -16(%edx)
L(fwd_write_12bytes):
	mov	-12(%eax), %ecx
	mov	%ecx, -12(%edx)
L(fwd_write_8bytes):
	mov	-8(%eax), %ecx
	mov	%ecx, -8(%edx)
L(fwd_write_4bytes):
	mov	-4(%eax), %ecx
	mov	%ecx, -4(%edx)
L(fwd_write_0bytes):
#ifndef USE_AS_BCOPY
# ifdef USE_AS_MEMPCPY
	mov	%edx, %eax
# else
	mov	DEST(%esp), %eax
# endif
#endif
	RETURN

	ALIGN (4)
L(fwd_write_77bytes):
	mov	-77(%eax), %ecx
	mov	%ecx, -77(%edx)
L(fwd_write_73bytes):
	mov	-73(%eax), %ecx
	mov	%ecx, -73(%edx)
L(fwd_write_69bytes):
	mov	-69(%eax), %ecx
	mov	%ecx, -69(%edx)
L(fwd_write_65bytes):
	mov	-65(%eax), %ecx
	mov	%ecx, -65(%edx)
L(fwd_write_61bytes):
	mov	-61(%eax), %ecx
	mov	%ecx, -61(%edx)
L(fwd_write_57bytes):
	mov	-57(%eax), %ecx
	mov	%ecx, -57(%edx)
L(fwd_write_53bytes):
	mov	-53(%eax), %ecx
	mov	%ecx, -53(%edx)
L(fwd_write_49bytes):
	mov	-49(%eax), %ecx
	mov	%ecx, -49(%edx)
L(fwd_write_45bytes):
	mov	-45(%eax), %ecx
	mov	%ecx, -45(%edx)
L(fwd_write_41bytes):
	mov	-41(%eax), %ecx
	mov	%ecx, -41(%edx)
L(fwd_write_37bytes):
	mov	-37(%eax), %ecx
	mov	%ecx, -37(%edx)
L(fwd_write_33bytes):
	mov	-33(%eax), %ecx
	mov	%ecx, -33(%edx)
L(fwd_write_29bytes):
	mov	-29(%eax), %ecx
	mov	%ecx, -29(%edx)
L(fwd_write_25bytes):
	mov	-25(%eax), %ecx
	mov	%ecx, -25(%edx)
L(fwd_write_21bytes):
	mov	-21(%eax), %ecx
	mov	%ecx, -21(%edx)
L(fwd_write_17bytes):
	mov	-17(%eax), %ecx
	mov	%ecx, -17(%edx)
L(fwd_write_13bytes):
	mov	-13(%eax), %ecx
	mov	%ecx, -13(%edx)
L(fwd_write_9bytes):
	mov	-9(%eax), %ecx
	mov	%ecx, -9(%edx)
L(fwd_write_5bytes):
	mov	-5(%eax), %ecx
	mov	%ecx, -5(%edx)
L(fwd_write_1bytes):
	movzbl	-1(%eax), %ecx
	movb	%cl, -1(%edx)
#ifndef USE_AS_BCOPY
# ifdef USE_AS_MEMPCPY
	mov	%edx, %eax
# else
	mov	DEST(%esp), %eax
# endif
#endif
	RETURN

	ALIGN (4)
L(fwd_write_78bytes):
	mov	-78(%eax), %ecx
	mov	%ecx, -78(%edx)
L(fwd_write_74bytes):
	mov	-74(%eax), %ecx
	mov	%ecx, -74(%edx)
L(fwd_write_70bytes):
	mov	-70(%eax), %ecx
	mov	%ecx, -70(%edx)
L(fwd_write_66bytes):
	mov	-66(%eax), %ecx
	mov	%ecx, -66(%edx)
L(fwd_write_62bytes):
	mov	-62(%eax), %ecx
	mov	%ecx, -62(%edx)
L(fwd_write_58bytes):
	mov	-58(%eax), %ecx
	mov	%ecx, -58(%edx)
L(fwd_write_54bytes):
	mov	-54(%eax), %ecx
	mov	%ecx, -54(%edx)
L(fwd_write_50bytes):
	mov	-50(%eax), %ecx
	mov	%ecx, -50(%edx)
L(fwd_write_46bytes):
	mov	-46(%eax), %ecx
	mov	%ecx, -46(%edx)
L(fwd_write_42bytes):
	mov	-42(%eax), %ecx
	mov	%ecx, -42(%edx)
L(fwd_write_38bytes):
	mov	-38(%eax), %ecx
	mov	%ecx, -38(%edx)
L(fwd_write_34bytes):
	mov	-34(%eax), %ecx
	mov	%ecx, -34(%edx)
L(fwd_write_30bytes):
	mov	-30(%eax), %ecx
	mov	%ecx, -30(%edx)
L(fwd_write_26bytes):
	mov	-26(%eax), %ecx
	mov	%ecx, -26(%edx)
L(fwd_write_22bytes):
	mov	-22(%eax), %ecx
	mov	%ecx, -22(%edx)
L(fwd_write_18bytes):
	mov	-18(%eax), %ecx
	mov	%ecx, -18(%edx)
L(fwd_write_14bytes):
	mov	-14(%eax), %ecx
	mov	%ecx, -14(%edx)
L(fwd_write_10bytes):
	mov	-10(%eax), %ecx
	mov	%ecx, -10(%edx)
L(fwd_write_6bytes):
	mov	-6(%eax), %ecx
	mov	%ecx, -6(%edx)
L(fwd_write_2bytes):
	movzwl	-2(%eax), %ecx
	movw	%cx, -2(%edx)
#ifndef USE_AS_BCOPY
# ifdef USE_AS_MEMPCPY
	mov	%edx, %eax
# else
	mov	DEST(%esp), %eax
# endif
#endif
	RETURN

	ALIGN (4)
L(fwd_write_79bytes):
	mov	-79(%eax), %ecx
	mov	%ecx, -79(%edx)
L(fwd_write_75bytes):
	mov	-75(%eax), %ecx
	mov	%ecx, -75(%edx)
L(fwd_write_71bytes):
	mov	-71(%eax), %ecx
	mov	%ecx, -71(%edx)
L(fwd_write_67bytes):
	mov	-67(%eax), %ecx
	mov	%ecx, -67(%edx)
L(fwd_write_63bytes):
	mov	-63(%eax), %ecx
	mov	%ecx, -63(%edx)
L(fwd_write_59bytes):
	mov	-59(%eax), %ecx
	mov	%ecx, -59(%edx)
L(fwd_write_55bytes):
	mov	-55(%eax), %ecx
	mov	%ecx, -55(%edx)
L(fwd_write_51bytes):
	mov	-51(%eax), %ecx
	mov	%ecx, -51(%edx)
L(fwd_write_47bytes):
	mov	-47(%eax), %ecx
	mov	%ecx, -47(%edx)
L(fwd_write_43bytes):
	mov	-43(%eax), %ecx
	mov	%ecx, -43(%edx)
L(fwd_write_39bytes):
	mov	-39(%eax), %ecx
	mov	%ecx, -39(%edx)
L(fwd_write_35bytes):
	mov	-35(%eax), %ecx
	mov	%ecx, -35(%edx)
L(fwd_write_31bytes):
	mov	-31(%eax), %ecx
	mov	%ecx, -31(%edx)
L(fwd_write_27bytes):
	mov	-27(%eax), %ecx
	mov	%ecx, -27(%edx)
L(fwd_write_23bytes):
	mov	-23(%eax), %ecx
	mov	%ecx, -23(%edx)
L(fwd_write_19bytes):
	mov	-19(%eax), %ecx
	mov	%ecx, -19(%edx)
L(fwd_write_15bytes):
	mov	-15(%eax), %ecx
	mov	%ecx, -15(%edx)
L(fwd_write_11bytes):
	mov	-11(%eax), %ecx
	mov	%ecx, -11(%edx)
L(fwd_write_7bytes):
	mov	-7(%eax), %ecx
	mov	-4(%eax), %ebx
	mov	%ecx, -7(%edx)
	mov	%ebx, -4(%edx)
#ifndef USE_AS_BCOPY
# ifdef USE_AS_MEMPCPY
	mov	%edx, %eax
# else
	mov	DEST(%esp), %eax
# endif
#endif
	RETURN

	ALIGN (4)
L(fwd_write_3bytes):
	mov	-3(%eax), %cx
	mov	-2(%eax), %ax
	mov	%cx, -3(%edx)
	mov	%ax, -2(%edx)
#ifndef USE_AS_BCOPY
# ifdef USE_AS_MEMPCPY
	mov	%edx, %eax
# else
	mov	DEST(%esp), %eax
# endif
#endif
	RETURN

	cfi_restore_state
	cfi_remember_state
	CFI_PUSH (%edi)
	ALIGN (4)
L(large_page):
	movdqu	(%eax), %xmm0
	mov	%edx, %edi
	and	$-16, %edx
	PUSH (%esi)
	add	$16, %edx
	mov	%edi, %esi
	sub	%edx, %edi
	add	%edi, %ecx
	sub	%edi, %eax

	movdqu	(%eax), %xmm1
	lea	16(%eax), %eax
	movdqu	%xmm0, (%esi)
	movntdq	%xmm1, (%edx)
	lea	16(%edx), %edx
	lea	-0x90(%ecx), %ecx

#ifdef USE_AS_MEMMOVE

#ifdef SHARED_CACHE_SIZE_HALF
	mov	$SHARED_CACHE_SIZE_HALF, %edi
#else
# ifdef SHARED
	call	__i686.get_pc_thunk.bx
	add	$_GLOBAL_OFFSET_TABLE_, %ebx
	mov	__x86_shared_cache_size_half@GOTOFF(%ebx), %edi
# else
	mov	__x86_shared_cache_size_half, %edi
# endif
#endif
	mov	%eax, %esi
	sub	%edx, %esi
	cmp	%ecx, %esi
	jae	L(memmove_is_memcpy_fwd)
	shl	$2, %edi
	cmp	%edi, %ecx
	jb	L(ll_cache_copy_fwd_start)
L(memmove_is_memcpy_fwd):
#endif
	POP (%esi)
	POP (%edi)
L(large_page_loop):
	movdqu	(%eax), %xmm0
	movdqu	0x10(%eax), %xmm1
	movdqu	0x20(%eax), %xmm2
	movdqu	0x30(%eax), %xmm3
	movdqu	0x40(%eax), %xmm4
	movdqu	0x50(%eax), %xmm5
	movdqu	0x60(%eax), %xmm6
	movdqu	0x70(%eax), %xmm7
	lea	0x80(%eax), %eax

	sub	$0x80, %ecx
	movntdq	%xmm0, (%edx)
	movntdq	%xmm1, 0x10(%edx)
	movntdq	%xmm2, 0x20(%edx)
	movntdq	%xmm3, 0x30(%edx)
	movntdq	%xmm4, 0x40(%edx)
	movntdq	%xmm5, 0x50(%edx)
	movntdq	%xmm6, 0x60(%edx)
	movntdq	%xmm7, 0x70(%edx)
	lea	0x80(%edx), %edx
	jae	L(large_page_loop)
	cmp	$-0x40, %ecx
	lea	0x80(%ecx), %ecx
	jl	L(large_page_less_64bytes)

	movdqu	(%eax), %xmm0
	movdqu	0x10(%eax), %xmm1
	movdqu	0x20(%eax), %xmm2
	movdqu	0x30(%eax), %xmm3
	lea	0x40(%eax), %eax

	movntdq	%xmm0, (%edx)
	movntdq	%xmm1, 0x10(%edx)
	movntdq	%xmm2, 0x20(%edx)
	movntdq	%xmm3, 0x30(%edx)
	lea	0x40(%edx), %edx
	sub	$0x40, %ecx
L(large_page_less_64bytes):
	cmp	$32, %ecx
	jb	L(large_page_less_32bytes)
	movdqu	(%eax), %xmm0
	movdqu	0x10(%eax), %xmm1
	lea	0x20(%eax), %eax
	movntdq	%xmm0, (%edx)
	movntdq	%xmm1, 0x10(%edx)
	lea	0x20(%edx), %edx
	sub	$0x20, %ecx
L(large_page_less_32bytes):
	add	%ecx, %edx
	add	%ecx, %eax
	sfence
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)

#ifdef USE_AS_MEMMOVE
	CFI_PUSH (%edi)
	CFI_PUSH (%esi)
	ALIGN (4)
L(ll_cache_copy_fwd_start):
	prefetcht0 0x1c0(%eax)
	prefetcht0 0x280(%eax)
	movdqu	(%eax), %xmm0
	movdqu	0x10(%eax), %xmm1
	movdqu	0x20(%eax), %xmm2
	movdqu	0x30(%eax), %xmm3
	movdqu	0x40(%eax), %xmm4
	movdqu	0x50(%eax), %xmm5
	movdqu	0x60(%eax), %xmm6
	movdqu	0x70(%eax), %xmm7
	lea	0x80(%eax), %eax

	sub	$0x80, %ecx
	movaps	%xmm0, (%edx)
	movaps	%xmm1, 0x10(%edx)
	movaps	%xmm2, 0x20(%edx)
	movaps	%xmm3, 0x30(%edx)
	movaps	%xmm4, 0x40(%edx)
	movaps	%xmm5, 0x50(%edx)
	movaps	%xmm6, 0x60(%edx)
	movaps	%xmm7, 0x70(%edx)
	lea	0x80(%edx), %edx
	jae	L(ll_cache_copy_fwd_start)
	cmp	$-0x40, %ecx
	lea	0x80(%ecx), %ecx
	jl	L(large_page_ll_less_fwd_64bytes)

	movdqu	(%eax), %xmm0
	movdqu	0x10(%eax), %xmm1
	movdqu	0x20(%eax), %xmm2
	movdqu	0x30(%eax), %xmm3
	lea	0x40(%eax), %eax

	movaps	%xmm0, (%edx)
	movaps	%xmm1, 0x10(%edx)
	movaps	%xmm2, 0x20(%edx)
	movaps	%xmm3, 0x30(%edx)
	lea	0x40(%edx), %edx
	sub	$0x40, %ecx
L(large_page_ll_less_fwd_64bytes):
	add	%ecx, %eax
	add	%ecx, %edx
	POP (%esi)
	POP (%edi)
	BRANCH_TO_JMPTBL_ENTRY (L(table_48bytes_fwd), %ecx, 4)
#endif

	CFI_PUSH (%edi)
	CFI_PUSH (%esi)
	ALIGN (4)
L(large_page_bwd):
	movdqu	-0x10(%eax), %xmm1
	lea	-16(%eax), %eax
	movdqu	%xmm0, (%esi)
	movntdq	%xmm1, -0x10(%edx)
	lea	-16(%edx), %edx
	lea	-0x90(%ecx), %ecx
#ifdef USE_AS_MEMMOVE

#ifdef SHARED_CACHE_SIZE_HALF
	mov	$SHARED_CACHE_SIZE_HALF, %edi
#else
# ifdef SHARED
	call	__i686.get_pc_thunk.bx
	add	$_GLOBAL_OFFSET_TABLE_, %ebx
	mov	__x86_shared_cache_size_half@GOTOFF(%ebx), %edi
# else
	mov	__x86_shared_cache_size_half, %edi
# endif
#endif

	mov	%edx, %esi
	sub	%eax, %esi
	cmp	%ecx, %esi
	jae	L(memmove_is_memcpy_bwd)
	cmp	%edi, %esi
	jb	L(ll_cache_copy_bwd_start)
L(memmove_is_memcpy_bwd):
#endif
	POP (%esi)
	POP (%edi)
L(large_page_bwd_loop):
	movdqu	-0x10(%eax), %xmm0
	movdqu	-0x20(%eax), %xmm1
	movdqu	-0x30(%eax), %xmm2
	movdqu	-0x40(%eax), %xmm3
	movdqu	-0x50(%eax), %xmm4
	movdqu	-0x60(%eax), %xmm5
	movdqu	-0x70(%eax), %xmm6
	movdqu	-0x80(%eax), %xmm7
	lea	-0x80(%eax), %eax

	sub	$0x80, %ecx
	movntdq	%xmm0, -0x10(%edx)
	movntdq	%xmm1, -0x20(%edx)
	movntdq	%xmm2, -0x30(%edx)
	movntdq	%xmm3, -0x40(%edx)
	movntdq	%xmm4, -0x50(%edx)
	movntdq	%xmm5, -0x60(%edx)
	movntdq	%xmm6, -0x70(%edx)
	movntdq	%xmm7, -0x80(%edx)
	lea	-0x80(%edx), %edx
	jae	L(large_page_bwd_loop)
	cmp	$-0x40, %ecx
	lea	0x80(%ecx), %ecx
	jl	L(large_page_bwd_less_64bytes)

	movdqu	-0x10(%eax), %xmm0
	movdqu	-0x20(%eax), %xmm1
	lea	-0x40(%edx), %edx
	movdqu	-0x30(%eax), %xmm2
	movdqu	-0x40(%eax), %xmm3
	lea	-0x40(%eax), %eax

	movntdq	%xmm0, 0x30(%edx)
	movntdq	%xmm1, 0x20(%edx)
	sub	$0x40, %ecx
	movntdq	%xmm2, 0x10(%edx)
	movntdq	%xmm3, (%edx)
L(large_page_bwd_less_64bytes):
	cmp	$32, %ecx
	jb	L(large_page_bwd_less_32bytes)
	movdqu	-0x10(%eax), %xmm0
	movdqu	-0x20(%eax), %xmm1
	lea	-0x20(%eax), %eax
	movntdq	%xmm0, -0x10(%edx)
	movntdq	%xmm1, -0x20(%edx)
	lea	-0x20(%edx), %edx
	sub	$0x20, %ecx
L(large_page_bwd_less_32bytes):
	sub	%ecx, %edx
	sub	%ecx, %eax
	sfence
	BRANCH_TO_JMPTBL_ENTRY (L(table_48_bytes_bwd), %ecx, 4)

#ifdef USE_AS_MEMMOVE
	CFI_PUSH (%edi)
	CFI_PUSH (%esi)
	ALIGN (4)
L(ll_cache_copy_bwd_start):
	prefetcht0 -0x1c0(%eax)
	prefetcht0 -0x280(%eax)
	movdqu	-0x10(%eax), %xmm0
	movdqu	-0x20(%eax), %xmm1
	movdqu	-0x30(%eax), %xmm2
	movdqu	-0x40(%eax), %xmm3
	movdqu	-0x50(%eax), %xmm4
	movdqu	-0x60(%eax), %xmm5
	movdqu	-0x70(%eax), %xmm6
	movdqu	-0x80(%eax), %xmm7
	lea	-0x80(%eax), %eax

	sub	$0x80, %ecx
	movaps	%xmm0, -0x10(%edx)
	movaps	%xmm1, -0x20(%edx)
	movaps	%xmm2, -0x30(%edx)
	movaps	%xmm3, -0x40(%edx)
	movaps	%xmm4, -0x50(%edx)
	movaps	%xmm5, -0x60(%edx)
	movaps	%xmm6, -0x70(%edx)
	movaps	%xmm7, -0x80(%edx)
	lea	-0x80(%edx), %edx
	jae	L(ll_cache_copy_bwd_start)
	POP (%esi)
	POP (%edi)
	cmp	$-0x40, %ecx
	lea	0x80(%ecx), %ecx
	jl	L(large_page_ll_less_bwd_64bytes)

	movdqu	-0x10(%eax), %xmm0
	movdqu	-0x20(%eax), %xmm1
	movdqu	-0x30(%eax), %xmm2
	movdqu	-0x40(%eax), %xmm3
	lea	-0x40(%eax), %eax

	movaps	%xmm0, -0x10(%edx)
	movaps	%xmm1, -0x20(%edx)
	movaps	%xmm2, -0x30(%edx)
	movaps	%xmm3, -0x40(%edx)
	lea	-0x40(%edx), %edx
	sub	$0x40, %ecx
L(large_page_ll_less_bwd_64bytes):
	sub	%ecx, %eax
	sub	%ecx, %edx
	BRANCH_TO_JMPTBL_ENTRY (L(table_48_bytes_bwd), %ecx, 4)
#endif

	ALIGN (4)
L(bk_write_76bytes):
	mov	72(%eax), %ecx
	mov	%ecx, 72(%edx)
L(bk_write_72bytes):
	mov	68(%eax), %ecx
	mov	%ecx, 68(%edx)
L(bk_write_68bytes):
	mov	64(%eax), %ecx
	mov	%ecx, 64(%edx)
L(bk_write_64bytes):
	mov	60(%eax), %ecx
	mov	%ecx, 60(%edx)
L(bk_write_60bytes):
	mov	56(%eax), %ecx
	mov	%ecx, 56(%edx)
L(bk_write_56bytes):
	mov	52(%eax), %ecx
	mov	%ecx, 52(%edx)
L(bk_write_52bytes):
	mov	48(%eax), %ecx
	mov	%ecx, 48(%edx)
L(bk_write_48bytes):
	mov	44(%eax), %ecx
	mov	%ecx, 44(%edx)
L(bk_write_44bytes):
	mov	40(%eax), %ecx
	mov	%ecx, 40(%edx)
L(bk_write_40bytes):
	mov	36(%eax), %ecx
	mov	%ecx, 36(%edx)
L(bk_write_36bytes):
	mov	32(%eax), %ecx
	mov	%ecx, 32(%edx)
L(bk_write_32bytes):
	mov	28(%eax), %ecx
	mov	%ecx, 28(%edx)
L(bk_write_28bytes):
	mov	24(%eax), %ecx
	mov	%ecx, 24(%edx)
L(bk_write_24bytes):
	mov	20(%eax), %ecx
	mov	%ecx, 20(%edx)
L(bk_write_20bytes):
	mov	16(%eax), %ecx
	mov	%ecx, 16(%edx)
L(bk_write_16bytes):
	mov	12(%eax), %ecx
	mov	%ecx, 12(%edx)
L(bk_write_12bytes):
	mov	8(%eax), %ecx
	mov	%ecx, 8(%edx)
L(bk_write_8bytes):
	mov	4(%eax), %ecx
	mov	%ecx, 4(%edx)
L(bk_write_4bytes):
	mov	(%eax), %ecx
	mov	%ecx, (%edx)
L(bk_write_0bytes):
#ifndef USE_AS_BCOPY
	mov	DEST(%esp), %eax
# ifdef USE_AS_MEMPCPY
	mov	LEN(%esp), %ecx
	add	%ecx, %eax
# endif
#endif
	RETURN

	ALIGN (4)
L(bk_write_77bytes):
	mov	73(%eax), %ecx
	mov	%ecx, 73(%edx)
L(bk_write_73bytes):
	mov	69(%eax), %ecx
	mov	%ecx, 69(%edx)
L(bk_write_69bytes):
	mov	65(%eax), %ecx
	mov	%ecx, 65(%edx)
L(bk_write_65bytes):
	mov	61(%eax), %ecx
	mov	%ecx, 61(%edx)
L(bk_write_61bytes):
	mov	57(%eax), %ecx
	mov	%ecx, 57(%edx)
L(bk_write_57bytes):
	mov	53(%eax), %ecx
	mov	%ecx, 53(%edx)
L(bk_write_53bytes):
	mov	49(%eax), %ecx
	mov	%ecx, 49(%edx)
L(bk_write_49bytes):
	mov	45(%eax), %ecx
	mov	%ecx, 45(%edx)
L(bk_write_45bytes):
	mov	41(%eax), %ecx
	mov	%ecx, 41(%edx)
L(bk_write_41bytes):
	mov	37(%eax), %ecx
	mov	%ecx, 37(%edx)
L(bk_write_37bytes):
	mov	33(%eax), %ecx
	mov	%ecx, 33(%edx)
L(bk_write_33bytes):
	mov	29(%eax), %ecx
	mov	%ecx, 29(%edx)
L(bk_write_29bytes):
	mov	25(%eax), %ecx
	mov	%ecx, 25(%edx)
L(bk_write_25bytes):
	mov	21(%eax), %ecx
	mov	%ecx, 21(%edx)
L(bk_write_21bytes):
	mov	17(%eax), %ecx
	mov	%ecx, 17(%edx)
L(bk_write_17bytes):
	mov	13(%eax), %ecx
	mov	%ecx, 13(%edx)
L(bk_write_13bytes):
	mov	9(%eax), %ecx
	mov	%ecx, 9(%edx)
L(bk_write_9bytes):
	mov	5(%eax), %ecx
	mov	%ecx, 5(%edx)
L(bk_write_5bytes):
	mov	1(%eax), %ecx
	mov	%ecx, 1(%edx)
L(bk_write_1bytes):
	movzbl	(%eax), %ecx
	movb	%cl, (%edx)
#ifndef USE_AS_BCOPY
	mov	DEST(%esp), %eax
# ifdef USE_AS_MEMPCPY
	mov	LEN(%esp), %ecx
	add	%ecx, %eax
# endif
#endif
	RETURN

	ALIGN (4)
L(bk_write_78bytes):
	mov	74(%eax), %ecx
	mov	%ecx, 74(%edx)
L(bk_write_74bytes):
	mov	70(%eax), %ecx
	mov	%ecx, 70(%edx)
L(bk_write_70bytes):
	mov	66(%eax), %ecx
	mov	%ecx, 66(%edx)
L(bk_write_66bytes):
	mov	62(%eax), %ecx
	mov	%ecx, 62(%edx)
L(bk_write_62bytes):
	mov	58(%eax), %ecx
	mov	%ecx, 58(%edx)
L(bk_write_58bytes):
	mov	54(%eax), %ecx
	mov	%ecx, 54(%edx)
L(bk_write_54bytes):
	mov	50(%eax), %ecx
	mov	%ecx, 50(%edx)
L(bk_write_50bytes):
	mov	46(%eax), %ecx
	mov	%ecx, 46(%edx)
L(bk_write_46bytes):
	mov	42(%eax), %ecx
	mov	%ecx, 42(%edx)
L(bk_write_42bytes):
	mov	38(%eax), %ecx
	mov	%ecx, 38(%edx)
L(bk_write_38bytes):
	mov	34(%eax), %ecx
	mov	%ecx, 34(%edx)
L(bk_write_34bytes):
	mov	30(%eax), %ecx
	mov	%ecx, 30(%edx)
L(bk_write_30bytes):
	mov	26(%eax), %ecx
	mov	%ecx, 26(%edx)
L(bk_write_26bytes):
	mov	22(%eax), %ecx
	mov	%ecx, 22(%edx)
L(bk_write_22bytes):
	mov	18(%eax), %ecx
	mov	%ecx, 18(%edx)
L(bk_write_18bytes):
	mov	14(%eax), %ecx
	mov	%ecx, 14(%edx)
L(bk_write_14bytes):
	mov	10(%eax), %ecx
	mov	%ecx, 10(%edx)
L(bk_write_10bytes):
	mov	6(%eax), %ecx
	mov	%ecx, 6(%edx)
L(bk_write_6bytes):
	mov	2(%eax), %ecx
	mov	%ecx, 2(%edx)
L(bk_write_2bytes):
	movzwl	(%eax), %ecx
	movw	%cx, (%edx)
#ifndef USE_AS_BCOPY
	mov	DEST(%esp), %eax
# ifdef USE_AS_MEMPCPY
	mov	LEN(%esp), %ecx
	add	%ecx, %eax
# endif
#endif
	RETURN

	ALIGN (4)
L(bk_write_79bytes):
	mov	75(%eax), %ecx
	mov	%ecx, 75(%edx)
L(bk_write_75bytes):
	mov	71(%eax), %ecx
	mov	%ecx, 71(%edx)
L(bk_write_71bytes):
	mov	67(%eax), %ecx
	mov	%ecx, 67(%edx)
L(bk_write_67bytes):
	mov	63(%eax), %ecx
	mov	%ecx, 63(%edx)
L(bk_write_63bytes):
	mov	59(%eax), %ecx
	mov	%ecx, 59(%edx)
L(bk_write_59bytes):
	mov	55(%eax), %ecx
	mov	%ecx, 55(%edx)
L(bk_write_55bytes):
	mov	51(%eax), %ecx
	mov	%ecx, 51(%edx)
L(bk_write_51bytes):
	mov	47(%eax), %ecx
	mov	%ecx, 47(%edx)
L(bk_write_47bytes):
	mov	43(%eax), %ecx
	mov	%ecx, 43(%edx)
L(bk_write_43bytes):
	mov	39(%eax), %ecx
	mov	%ecx, 39(%edx)
L(bk_write_39bytes):
	mov	35(%eax), %ecx
	mov	%ecx, 35(%edx)
L(bk_write_35bytes):
	mov	31(%eax), %ecx
	mov	%ecx, 31(%edx)
L(bk_write_31bytes):
	mov	27(%eax), %ecx
	mov	%ecx, 27(%edx)
L(bk_write_27bytes):
	mov	23(%eax), %ecx
	mov	%ecx, 23(%edx)
L(bk_write_23bytes):
	mov	19(%eax), %ecx
	mov	%ecx, 19(%edx)
L(bk_write_19bytes):
	mov	15(%eax), %ecx
	mov	%ecx, 15(%edx)
L(bk_write_15bytes):
	mov	11(%eax), %ecx
	mov	%ecx, 11(%edx)
L(bk_write_11bytes):
	mov	7(%eax), %ecx
	mov	%ecx, 7(%edx)
L(bk_write_7bytes):
	mov	3(%eax), %ecx
	mov	(%eax), %ebx
	mov	%ecx, 3(%edx)
	mov	%ebx, (%edx)
#ifndef USE_AS_BCOPY
	mov	DEST(%esp), %eax
# ifdef USE_AS_MEMPCPY
	mov	LEN(%esp), %ecx
	add	%ecx, %eax
# endif
#endif
	RETURN

	ALIGN (4)
L(bk_write_3bytes):
	movzwl	1(%eax), %ecx
	movw	%cx, 1(%edx)
	movzbl	(%eax), %eax
	movb	%al, (%edx)
#ifndef USE_AS_BCOPY
	mov	DEST(%esp), %eax
# ifdef USE_AS_MEMPCPY
	mov	LEN(%esp), %ecx
	add	%ecx, %eax
# endif
#endif
	RETURN_END
END (MEMCPY)

	.section .rodata.ssse3,"a",@progbits
	ALIGN (2)
L(table_48bytes_fwd):
	.int	JMPTBL (L(fwd_write_0bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_1bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_2bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_3bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_4bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_5bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_6bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_7bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_8bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_9bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_10bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_11bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_12bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_13bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_14bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_15bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_16bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_17bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_18bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_19bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_20bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_21bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_22bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_23bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_24bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_25bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_26bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_27bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_28bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_29bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_30bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_31bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_32bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_33bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_34bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_35bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_36bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_37bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_38bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_39bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_40bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_41bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_42bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_43bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_44bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_45bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_46bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_47bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_48bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_49bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_50bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_51bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_52bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_53bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_54bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_55bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_56bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_57bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_58bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_59bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_60bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_61bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_62bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_63bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_64bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_65bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_66bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_67bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_68bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_69bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_70bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_71bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_72bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_73bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_74bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_75bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_76bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_77bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_78bytes), L(table_48bytes_fwd))
	.int	JMPTBL (L(fwd_write_79bytes), L(table_48bytes_fwd))

	ALIGN (2)
L(shl_table):
	.int	JMPTBL (L(shl_0), L(shl_table))
	.int	JMPTBL (L(shl_1), L(shl_table))
	.int	JMPTBL (L(shl_2), L(shl_table))
	.int	JMPTBL (L(shl_3), L(shl_table))
	.int	JMPTBL (L(shl_4), L(shl_table))
	.int	JMPTBL (L(shl_5), L(shl_table))
	.int	JMPTBL (L(shl_6), L(shl_table))
	.int	JMPTBL (L(shl_7), L(shl_table))
	.int	JMPTBL (L(shl_8), L(shl_table))
	.int	JMPTBL (L(shl_9), L(shl_table))
	.int	JMPTBL (L(shl_10), L(shl_table))
	.int	JMPTBL (L(shl_11), L(shl_table))
	.int	JMPTBL (L(shl_12), L(shl_table))
	.int	JMPTBL (L(shl_13), L(shl_table))
	.int	JMPTBL (L(shl_14), L(shl_table))
	.int	JMPTBL (L(shl_15), L(shl_table))

	ALIGN (2)
L(shl_bwd_table):
	.int	JMPTBL (L(shl_0_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_1_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_2_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_3_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_4_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_5_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_6_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_7_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_8_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_9_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_10_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_11_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_12_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_13_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_14_bwd), L(shl_bwd_table))
	.int	JMPTBL (L(shl_15_bwd), L(shl_bwd_table))

	ALIGN (2)
L(table_48_bytes_bwd):
	.int	JMPTBL (L(bk_write_0bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_1bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_2bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_3bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_4bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_5bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_6bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_7bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_8bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_9bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_10bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_11bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_12bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_13bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_14bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_15bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_16bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_17bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_18bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_19bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_20bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_21bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_22bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_23bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_24bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_25bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_26bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_27bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_28bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_29bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_30bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_31bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_32bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_33bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_34bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_35bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_36bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_37bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_38bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_39bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_40bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_41bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_42bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_43bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_44bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_45bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_46bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_47bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_48bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_49bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_50bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_51bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_52bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_53bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_54bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_55bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_56bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_57bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_58bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_59bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_60bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_61bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_62bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_63bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_64bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_65bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_66bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_67bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_68bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_69bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_70bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_71bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_72bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_73bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_74bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_75bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_76bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_77bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_78bytes), L(table_48_bytes_bwd))
	.int	JMPTBL (L(bk_write_79bytes), L(table_48_bytes_bwd))
