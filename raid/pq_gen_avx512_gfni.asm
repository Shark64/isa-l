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

;;; Optimized pq of N source vectors using AVX512
;;; int pq_gen_avx512(int vects, int len, void **array)

;;; Generates P+Q parity vector from N (vects-2) sources in array of pointers
;;; (**array).  Last two pointers are the P and Q destinations respectively.
;;; Vectors must be aligned to 64 bytes if NO_NT_LDST is not defined.
;;; Length must be 32 byte multiple.

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
 %define return rax
 %define func(x) x: endbranch
 %define FUNC_SAVE
 %define FUNC_RESTORE
%endif

%ifidn __OUTPUT_FORMAT__, win64
 %define arg0  rcx
 %define arg1  rdx
 %define arg2  r8
 %define arg3  r9
 %define tmp   r11
 %define tmp3  r10
 %define return rax
 %define stack_size  4*16 + 8 	; must be an odd multiple of 8
 %define func(x) proc_frame x
 %macro FUNC_SAVE 0
	alloc_stack	stack_size
	vmovdqu	[rsp + 0*16], xmm6
	vmovdqu	[rsp + 1*16], xmm7
	vmovdqu	[rsp + 2*16], xmm8
	vmovdqu	[rsp + 3*16], xmm9
	end_prolog
 %endmacro

 %macro FUNC_RESTORE 0
	vmovdqu	xmm6, [rsp + 0*16]
	vmovdqu	xmm7, [rsp + 1*16]
	vmovdqu	xmm8, [rsp + 2*16]
	vmovdqu	xmm9, [rsp + 3*16]
	add	rsp, stack_size
 %endmacro
%endif

%define vec    arg0
%define len    arg1
%define ptr    arg3
%define pos    rax

%define xp1    zmm0
%define xq1    zmm1
%define xtmp1  zmm2
%define xs1    zmm3

%define xp2    zmm4
%define xq2    zmm5
%define xtmp2  zmm6
%define xs2    zmm7

%define xzero  zmm8
%define xpoly  zmm9

%define xp1y   ymm0
%define xq1y   ymm1
%define xtmp1y ymm2
%define xs1y   ymm3
%define xzeroy ymm8
%define xpolyy ymm9

%define NO_NT_LDST
;;; Use Non-temporal load/stor
%ifdef NO_NT_LDST
 %define XLDR vmovdqu8		;u8
 %define XSTR vmovdqu8
%else
 %define XLDR vmovntdqa
 %define XSTR vmovntdq
%endif

default rel

[bits 64]
section .text

align 32
mk_global  pq_gen_avx512, function
func(pq_gen_avx512)
	FUNC_SAVE
	test	len, len
	je	return_pass
	sub	DWORD(vec), 3			;Keep as offset to last source
	jle	return_fail		;Must have at least 2 sources
	test	BYTE(len), (32-1)		;Check alignment of length
	jnz	return_fail
	shl	DWORD(vec), 3			;Keep as offset to last source
	xor	DWORD(pos), DWORD(pos)
	vpbroadcastq xpoly, [rel matrix]
	cmp	len, 127
	jle	loop32
	sub	len, 2*64		;Len points to last block

align 16
loop128:
	mov	ptr, pos
	add	ptr, [arg2+vec] 	;Fetch last source pointer
	lea	tmp, [vec-8]		;Set tmp to point back to last vector
	XLDR	xs1, [ptr]		;Preload last vector (source)
	XLDR	xs2, [ptr+64]	;Preload last vector (source)
	vpxorq	xp1y, xp1y, xp1y		;p1 = 0
	vpxorq	xp2, xp2, xp2		;p2 = 0
	vpxorq	xq1y, xq1y, xq1y	;q1 = 0
	vpxorq	xq2, xq2, xq2		;q2 = 0

align 16
next_vect:
	mov	ptr, pos
	add 	ptr, [arg2+tmp] 	; get pointer to next vect
	vpxorq	xq1, xq1, xs1		; q1 ^= s1
	vpxorq	xq2, xq2, xs2		; q2 ^= s2
	vpxorq	xp1, xp1, xs1		; p1 ^= s1
	vpxorq	xp2, xp2, xs2		; p2 ^= s2
	vgf2p8affineqb	xq1, xq1, xpoly, 0x0
	vgf2p8affineqb	xq2, xq2, xpoly, 0x0
	XLDR	xs1, [ptr]		; Get next vector (source data1)
	XLDR	xs2, [ptr+64]	; Get next vector (source data2)
	sub	tmp, 8		  	;Inner loop for each source vector
	jge	next_vect		; Loop for each vect except 0

	lea	tmp, [arg2+vec]
	mov	ptr, [tmp+8]	;Get address of P parity vector
	mov	tmp, [tmp+(2*8)]	;Get address of Q parity vector
	vpxorq	xp1, xp1, xs1		;p1 ^= s1[0] - last source is already loaded
	vpxorq	xq1, xq1, xs1		;q1 ^= 1 * s1[0]
	vpxorq	xp2, xp2, xs2		;p2 ^= s2[0]
	vpxorq	xq2, xq2, xs2		;q2 ^= 1 * s2[0]
	XSTR	[ptr+pos], xp1		;Write parity P1 vector
	XSTR	[ptr+pos+64], xp2	;Write parity P2 vector
	XSTR	[tmp+pos], xq1		;Write parity Q1 vector
	XSTR	[tmp+pos+64], xq2	;Write parity Q2 vector
	add	pos, 2*64
	cmp	pos, len
	jle	loop128

	;; ------------------------------
	;; Do last 32 or 64 Bytes remaining
	add	len, 2*64
	cmp	pos, len
	je	return_pass

align 16
loop32:
	mov 	ptr, [arg2+vec] 	;Fetch last source pointer
	lea	tmp, [vec-8]		;Set tmp to point back to last vector
	vmovdqu	xs1y, [ptr+pos]		;Preload last vector (source)
	vpxor	xp1y, xp1y, xp1y	;p = 0
	vpxor	xq1y, xq1y, xq1y	;q = 0

next_vect32:
	mov 	ptr, [arg2+tmp] 	; get pointer to next vect
	vpxor	xq1y, xq1y, xs1y	; q1 ^= s1
	vpxor	xp1y, xp1y, xs1y	; p ^= s
	vgf2p8affineqb	xq1y, xq1y, xpolyy, 0x0
	vmovdqu	xs1y, [ptr+pos]		; Get next vector (source data)
	sub	tmp, 8		  	;Inner loop for each source vector
	jge	next_vect32		; Loop for each vect except 0

	lea	tmp, [arg2+vec]
	mov	ptr, [tmp+8]	;Get address of P parity vector
	mov	tmp, [tmp+(2*8)]	;Get address of Q parity vector
	vpxor	xp1y, xp1y, xs1y	;p ^= s[0] - last source is already loaded
	vpxor	xq1y, xq1y, xs1y	;q ^= 1 * s[0]
	vmovdqu	[ptr+pos], xp1y		;Write parity P vector
	vmovdqu	[tmp+pos], xq1y		;Write parity Q vector
	add	pos, 32
	cmp	pos, len
	jl	loop32


return_pass:
	xor	DWORD(return), DWORD(return)
	FUNC_RESTORE
	ret

return_fail:
	mov	return, 1
	FUNC_RESTORE
	ret

endproc_frame

section .data
align 64
matrix:
db 0x40,0x20,0x10,0x88,0x84,0x82,0x01,0x80 ; 0x1D
%endif  ; ifdef HAVE_AS_KNOWS_AVX512
