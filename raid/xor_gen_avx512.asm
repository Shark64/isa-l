;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;  Copyright(c) 2011-2017 Intel Corporation All rights reserved.
;
;  Redistribution and use in source and binary forms, with or without
;  modification, are permitted provided that the following conditions
;  are met:
;    * Redistributions of source code must retain the above copyright
;      notice, this list of conditions and the following disclaimer.
;    * Redistributions in binary form must reproduce the above copyright
;      notice, this list of conditions and the following disclaimer in
;      the documentation and/or other materials provided with the
;      distribution.
;    * Neither the name of Intel Corporation nor the names of its
;      contributors may be used to endorse or promote products derived
;      from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
;  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; Optimized xor of N source vectors using AVX512
;;; int xor_gen_avx512(int vects, int len, void **array)

;;; Generates xor parity vector from N (vects-1) sources in array of pointers
;;; (**array).  Last pointer is the dest.
;;; Vectors must be aligned to 64 bytes.  Length can be any value.

%include "reg_sizes.asm"

%ifdef HAVE_AS_KNOWS_AVX512

%ifidn __OUTPUT_FORMAT__, elf64
 %define arg0  rdi
 %define arg1  rsi
 %define arg2  rdx
 %define arg3  rcx
 %define arg4  r8
 %define arg5  r9
 %define tmp   r11
 %define tmp3  arg4
 %define func(x) x: endbranch
 %define return rax
 %define FUNC_SAVE
 %define FUNC_RESTORE

%elifidn __OUTPUT_FORMAT__, win64
 %define arg0  rcx
 %define arg1  rdx
 %define arg2  r8
 %define arg3  r9
 %define tmp   r11
 %define tmp3  r10
 %define func(x) proc_frame x
 %define return rax
 %define stack_size  2*16 + 8 	;must be an odd multiple of 8

 %macro FUNC_SAVE 0
	alloc_stack	stack_size
	vmovdqu	[rsp + 0*16], xmm6
	vmovdqu	[rsp + 1*16], xmm7
	end_prolog
 %endmacro
 %macro FUNC_RESTORE 0
	vmovdqu	xmm6, [rsp + 0*16]
	vmovdqu	xmm7, [rsp + 1*316]
	add	rsp, stack_size
 %endmacro

%endif	;output formats


%define vec arg0
%define len arg1
%define ptr tmp3
%define tmp2 rax
%define pos arg3
%define PS 8

%define NO_NT_LDST
;;; Use Non-temporal load/stor
%ifdef NO_NT_LDST
 %define XLDR vmovdqa64
 %define XSTR vmovdqa64
%else
 %define XLDR vmovntdqa
 %define XSTR vmovntdq
%endif


%use smartalign
ALIGNMODE P6

default rel
[bits 64]

section .text

align 32
mk_global  xor_gen_avx512, function
func(xor_gen_avx512)
	FUNC_SAVE
	xor	return, return
	xor	pos, pos
	test	len, len
	setz	BYTE(pos)
	sub	DWORD(vec), 2	;Keep as offset to last source
	setle	BYTE(return)
	add	DWORD(pos), DWORD(return)
	jnz	exit		;Must have at least 2 sources
	xor	DWORD(tmp2), DWORD(tmp2)
	mov	tmp, len
	shr	tmp, 7
	xor	pos, pos
	cmp	len, 127
	jle	len_not_aligned

align 16
loop128:
	mov	tmp2, pos
	add	tmp2, [arg2+vec*PS]	;Fetch last pointer in array
	XLDR	zmm0, [tmp2]	;Start with end of array in last vector
	XLDR	zmm1, [tmp2+64]	
	lea	DWORD(tmp2), [vec-1]

next_vect:
	mov	ptr, pos
	add 	ptr, [arg2+tmp2*PS]
%ifdef NO_NT_LDST
	vpxorq  zmm0, zmm0, [ptr]
	vpxorq  zmm1, zmm1, [ptr+64]
%else
	XLDR	zmm2, [ptr]		;Get next vector (source)
	XLDR	zmm3, [ptr+64]
	vpxorq	zmm0, zmm0, zmm2	;Add to xor parity
	vpxorq	zmm1, zmm1, zmm3
%endif
	sub	DWORD(tmp2), 1
	jge	next_vect		;Loop for each source

	mov	ptr, pos
	add	ptr, [arg2+vec*PS+PS]		;Address of parity vector
	XSTR	[ptr], zmm0		;Write parity xor vector
	XSTR	[ptr+64], zmm1
	sub	pos, -128		;shorter encoding for add pos, 128
	sub	tmp, 1
	jnz	loop128
	;; Unaligned length cases
align 16
len_not_aligned:
	xor	DWORD(tmp2), DWORD(tmp2)
	and	DWORD(len), 127
	jz	exit
	lea	ptr, [tmp2-1]		; tmp2 is 0 -> tmp=0xFFFFF...
	bzhi	len, ptr, len		; bzhi sets CF if pos > 63
	cmovnc	ptr, len		;If CF=0 put the bitmask in the lower
	cmovnc	len, ptr		; mask register and zero the upper.
	kmovq	k1, ptr		; else, mask in the upper reg and 0xFFFF..
	kmovq	k2, len			; in the lower.
	mov	tmp2, pos
	add	tmp2, [arg2+vec*PS]	;Fetch last pointer in array
	vmovdqu8	zmm0{k1}{z}, [tmp2]	;Start with end of array in last vector
	vmovdqu8	zmm1{k2}{z}, [tmp2+64]
	lea	DWORD(tmp2), [vec-1]
	

len_not_aligned_loop:
	mov	ptr, pos
	add 	ptr, [arg2+tmp2*PS]
	vmovdqu8	zmm2{k1}{z}, [ptr]	;Get next vector (source)
	vmovdqu8	zmm3{k2}{z}, [ptr+64]
	vpxorq	zmm0, zmm0, zmm2	;Add to xor parity
	vpxorq	zmm1, zmm1, zmm3
	sub	DWORD(tmp2), 1
	jge	len_not_aligned_loop	;Loop for each source

	mov	ptr, pos
	add	ptr, [arg2+vec*PS+PS]		;Address of parity vector
	
	vmovdqu8	[ptr]{k1}, zmm0		;Write parity xor vector
	vmovdqu8	[ptr+64]{k2}, zmm1
	xor	DWORD(return), DWORD(return)
	
exit:
	FUNC_RESTORE
	ret


endproc_frame

%endif  ; ifdef HAVE_AS_KNOWS_AVX512
