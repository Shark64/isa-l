;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;  Copyright(c) 2011-2020 Intel Corporation All rights reserved.
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;       Function API:
;       UINT32 crc32_iscsi_by16_10(
;               UINT32 init_crc, //initial CRC value, 32 bits
;               const unsigned char *buf, //buffer pointer to calculate CRC on
;               UINT64 len //buffer length in bytes (64-bit data)
;       );
;
;       Authors:
;               Erdinc Ozturk
;               Vinodh Gopal
;               James Guilford
;
;       Reference paper titled "Fast CRC Computation for Generic Polynomials Using PCLMULQDQ Instruction"
;       URL: http://www.intel.com/content/dam/www/public/us/en/documents/white-papers/fast-crc-computation-generic-polynomials-pclmulqdq-paper.pdf
;
;

%include "reg_sizes.asm"

%ifndef FUNCTION_NAME
%define FUNCTION_NAME crc32_iscsi_by16_10
%endif

%if (AS_FEATURE_LEVEL) >= 10

[bits 64]
default rel

section .text


%ifidn __OUTPUT_FORMAT__, win64
	%xdefine	arg1 r8
	%xdefine	arg2 rcx
	%xdefine	arg3 rdx
	%xdefine	arg4 r9

	%xdefine	arg1_low32 r8d
%else
	%xdefine	arg1 rdx
	%xdefine	arg2 rdi
	%xdefine	arg3 rsi
	%xdefine	arg4 rcx

	%xdefine	arg1_low32 edx
%endif

%define TMP 16*0
%ifidn __OUTPUT_FORMAT__, win64
	%define XMM_SAVE 16*2
	%define VARIABLE_OFFSET 16*12+8
;%else
;	%define VARIABLE_OFFSET 16*2+8
%endif
%xdefine	base arg4

align 32
mk_global FUNCTION_NAME, function
FUNCTION_NAME:
	endbranch
	lea		base, [rk9]

%ifidn __OUTPUT_FORMAT__, win64
	; push the xmm registers into the stack to maintain
	sub		rsp, VARIABLE_OFFSET
	vmovdqa		[rsp + XMM_SAVE + 16*0], xmm6
	vmovdqa		[rsp + XMM_SAVE + 16*1], xmm7
	vmovdqa		[rsp + XMM_SAVE + 16*2], xmm8
	vmovdqa		[rsp + XMM_SAVE + 16*3], xmm9
	vmovdqa		[rsp + XMM_SAVE + 16*4], xmm10
	vmovdqa		[rsp + XMM_SAVE + 16*5], xmm11
	vmovdqa		[rsp + XMM_SAVE + 16*6], xmm12
	vmovdqa		[rsp + XMM_SAVE + 16*7], xmm13
	vmovdqa		[rsp + XMM_SAVE + 16*8], xmm14
	vmovdqa		[rsp + XMM_SAVE + 16*9], xmm15
%endif

	; check if smaller than 256B
	mov		eax, 256
	sub		arg3, 256
	jl		.less_than_256

	; load the initial crc value
	vmovd		xmm10, arg1_low32      ; initial crc

	; receive the initial 64B data, xor the initial crc value
	vmovdqa64	zmm4, [arg2+16*4]
	vpxorq		zmm0, zmm10, [arg2]
	vbroadcasti32x4	zmm10, [base+rk3-rk9]	;xmm10 has rk3 and rk4
					;imm value of pclmulqdq instruction will determine which constant to use

	cmp		arg3, 256
	jl		.fold_128_B_loop

	vmovdqa64	zmm7, [arg2+16*8]
	vmovdqa64	zmm8, [arg2+16*12]
	vbroadcasti32x4 zmm16, [base+rk_1-rk9]	;zmm16 has rk-1 and rk-2
	sub		arg3, 256

align 16
.fold_256_B_loop:
	add		arg2, 256
	vpclmulqdq	zmm1, zmm0, zmm16, 0x10
	vpclmulqdq	zmm0, zmm0, zmm16, 0x01
	vpternlogq	zmm0, zmm1, [arg2+64*0], 0x96

	vpclmulqdq	zmm1, zmm4, zmm16, 0x10
	vpclmulqdq	zmm4, zmm4, zmm16, 0x01
	vpternlogq	zmm4, zmm1, [arg2+64*1], 0x96

	vpclmulqdq	zmm1, zmm7, zmm16, 0x10
	vpclmulqdq	zmm7, zmm7, zmm16, 0x01
	vpternlogq	zmm7, zmm1, [arg2+64*2], 0x96

	vpclmulqdq	zmm1, zmm8, zmm16, 0x10
	vpclmulqdq	zmm8, zmm8, zmm16, 0x01
	vpternlogq	zmm8, zmm1, [arg2+64*3], 0x96

	sub		arg3, 256
	jge     	.fold_256_B_loop

	;; Fold 256 into 128
	add		arg2, 256
	vpclmulqdq	zmm1, zmm0, zmm10, 0x01
	vpclmulqdq	zmm0, zmm0, zmm10, 0x10
	vpternlogq	zmm0, zmm1, zmm7, 0x96	; xor ABC

	vpclmulqdq	zmm5, zmm4, zmm10, 0x01
	vpclmulqdq	zmm4, zmm4, zmm10, 0x10
	vpternlogq	zmm4, zmm5, zmm8, 0x96	; xor ABC

	sub		arg3, -128
	jmp		.fold_128_B_register



	; at this section of the code, there is 128*x+y (0<=y<128) bytes of buffer. The fold_128_B_loop
	; loop will fold 128B at a time until we have 128+y Bytes of buffer

	; fold 128B at a time. This section of the code folds 8 xmm registers in parallel
align 16
.fold_128_B_loop:
	sub		arg2, -128
	vpclmulqdq	zmm2, zmm0, zmm10, 0x10
	vpclmulqdq	zmm0, zmm0, zmm10, 0x01
	vpternlogq	zmm0, zmm2, [arg2], 0x96

	vmovdqu8	zmm9, [arg2+16*4]
	vpclmulqdq	zmm5, zmm4, zmm10, 0x10
	vpclmulqdq	zmm4, zmm4, zmm10, 0x01
	vpternlogq	zmm4, zmm5, [arg2+64], 0x96

	add		arg3, -128
	jge		.fold_128_B_loop
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	sub		arg2, -128
	; at this point, the buffer pointer is pointing at the last y Bytes of the buffer, where 0 <= y < 128
	; the 128B of folded data is in 8 of the xmm registers: xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7

align 16
.fold_128_B_register:
	; fold the 8 128b parts into 1 xmm register with different constants
	vmovdqu8	zmm16, [base+rk9-rk9]		; multiply by rk9-rk16
	vmovdqu8	zmm3, [base+rk17-rk9]		; multiply by rk17-rk20, rk1,rk2, 0,0
	vpclmulqdq	zmm1, zmm0, zmm16, 0x01
	vpclmulqdq	zmm2, zmm0, zmm16, 0x10
	vextracti64x2	xmm7, zmm4, 3		; save last that has no multiplicand

	vpclmulqdq	zmm5, zmm4, zmm3, 0x01
	vpclmulqdq	zmm6, zmm4, zmm3, 0x10
	vmovdqa		xmm10, [base+rk1-rk9]		; Needed later in reduction loop
	vpternlogq	zmm1, zmm2, zmm5, 0x96	; xor ABC
	vpternlogq	zmm1, zmm6, zmm7, 0x96	; xor ABC

	vextracti64x4   ymm8, zmm1, 0x1 ; 3,2
	vpxor           ymm8, ymm8, ymm1
	vextracti128    xmm5, ymm8, 1
	vpxor           xmm7, xmm8, xmm5

	; instead of 128, we add 128-16 to the loop counter to save 1 instruction from the loop
	; instead of a cmp instruction, we use the negative flag with the jl instruction
	add		arg3, 128-16
	jl		.final_reduction_for_128

	; now we have 16+y bytes left to reduce. 16 Bytes is in register xmm7 and the rest is in memory
	; we can fold 16 bytes at a time if y>=16
	; continue folding 16B at a time

align 16
.16B_reduction_loop:
	vpclmulqdq	xmm8, xmm7, xmm10, 0x1
	vpclmulqdq	xmm7, xmm7, xmm10, 0x10
	vpternlogq	xmm7, xmm8, [arg2], 0x96
	add		arg2, 16
	sub		arg3, 16
	; instead of a cmp instruction, we utilize the flags with the jge instruction
	; equivalent of: cmp arg3, 16-16
	; check if there is any more 16B in the buffer to be able to fold
	jge		.16B_reduction_loop

	;now we have 16+z bytes left to reduce, where 0<= z < 16.
	;first, we reduce the data in the xmm7 register


align 16
.final_reduction_for_128:
	add		arg3, 16
	je		.128_done

	; here we are getting data that is less than 16 bytes.
	; since we know that there was data before the pointer, we can offset
	; the input pointer before the actual point, to receive exactly 16 bytes.
	; after that the registers need to be adjusted.
.get_last_two_xmms:
	vmovdqu		xmm1, [arg2 - 16 + arg3]

	; get rid of the extra data that was loaded before
	; load the shift constant
	lea		rax, [base+pshufb_shf_table-rk9]
	vmovdqu		xmm0, [rax+arg3]
	mov		eax, 0x80808080	; mask3
	vpbroadcastd	xmm3, eax

	vpshufb		xmm2, xmm7, xmm0
	vpxor		xmm0, xmm0, xmm3
	vpshufb		xmm7, xmm7, xmm0

	vpblendvb	xmm7, xmm7, xmm1, xmm0
	;;;;;;;;;;
	vpclmulqdq	xmm1, xmm2, xmm10, 0x1
	vpclmulqdq	xmm2, xmm2, xmm10, 0x10
	vpternlogq	xmm7, xmm1, xmm2, 0x96

.128_done:
	; compute crc of a 128-bit value
	vmovdqa		xmm10, [base+rk5-rk9]

	;64b fold
	vpclmulqdq	xmm0, xmm7, xmm10, 0
	vpsrldq		xmm7, xmm7, 8
	vpxor		xmm7, xmm7, xmm0

	;32b fold
	vpslldq		xmm4, xmm7, 4
	vpclmulqdq	xmm4, xmm4, xmm10, 0x10
	vpxor		xmm7, xmm7, xmm4


	;barrett reduction
.barrett:
;;     mask:  dq     0xFFFFFFFFFFFFFFFF, 0x0000000000000000
;;     mask2: dq     0xFFFFFFFF00000000, 0xFFFFFFFFFFFFFFFF
	vmovdqa		xmm10, [base+rk7-rk9]
	vpxor		xmm0, xmm0, xmm0
	vpcmpeqw	xmm3, xmm3, xmm3
	vpblendd	xmm4, xmm3, xmm0, 0b1100 ; mask
	vpblendd	xmm7, xmm7, xmm0, 0b0001 ; mask2

	vpclmulqdq	xmm1, xmm7, xmm10, 0
	vpternlogq	xmm1, xmm7, xmm4, 0x28   ; xmm1 XOR xmm7 AND xmm4
	vpclmulqdq	xmm2, xmm1, xmm10, 0x10
	vpternlogq	xmm7, xmm2, xmm1, 0x96
	vpextrd		eax, xmm7, 2

.cleanup:

%ifidn __OUTPUT_FORMAT__, win64
	vmovdqa		xmm6, [rsp + XMM_SAVE + 16*0]
	vmovdqa		xmm7, [rsp + XMM_SAVE + 16*1]
	vmovdqa		xmm8, [rsp + XMM_SAVE + 16*2]
	vmovdqa		xmm9, [rsp + XMM_SAVE + 16*3]
	vmovdqa		xmm10, [rsp + XMM_SAVE + 16*4]
	vmovdqa		xmm11, [rsp + XMM_SAVE + 16*5]
	vmovdqa		xmm12, [rsp + XMM_SAVE + 16*6]
	vmovdqa		xmm13, [rsp + XMM_SAVE + 16*7]
	vmovdqa		xmm14, [rsp + XMM_SAVE + 16*8]
	vmovdqa		xmm15, [rsp + XMM_SAVE + 16*9]
	add		rsp, VARIABLE_OFFSET
%endif
	ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

align 16
.less_than_256:

	; check if there is enough buffer to be able to fold 16B at a time
	add	DWORD(arg3), eax
	cmp	DWORD(arg3), 32
	jl	.less_than_32

	; if there is, load the constants
	vmovdqa	xmm10, [base+rk1-rk9]    ; rk1 and rk2 in xmm10

	vmovd	xmm0, arg1_low32	; get the initial crc value
	vpxor	xmm7, xmm0, [arg2]	; load the plaintext and XOR

	; update the buffer pointer
	add	arg2, 16

	; update the counter. subtract 32 instead of 16 to save one instruction from the loop
	sub	DWORD(arg3), 32

	jmp	.16B_reduction_loop


.less_than_32:
	; mov initial crc to the return value. this is necessary for zero-length buffers.
	mov	eax, arg1_low32
	test	DWORD(arg3), DWORD(arg3)
	je	.cleanup

	vmovd	xmm0, arg1_low32	; get the initial crc value

	cmp	DWORD(arg3), 16
	jle	.less_than_16_left

	vpxor	xmm7, xmm0, [arg2]	; xor the initial crc value
	add	arg2, 16
	sub	DWORD(arg3), 16
	vmovdqa	xmm10, [base+rk1-rk9]		; rk1 and rk2 in xmm10
	jmp	.get_last_two_xmms

;align 16
.less_than_16_left:
	cmp	DWORD(arg3), 4
	jl	.only_less_than_4

	xor	eax, eax
	not	eax
	bzhi	eax, eax, DWORD(arg3)
	kmovw	k1, eax
	vmovdqu8 xmm7{k1}{z}, [arg2]
	vpxor	xmm7, xmm7, xmm0
	xor	eax, eax
	cmp	DWORD(arg3), 16
	setne	al
	neg	eax
	kmovw	k1, eax
	lea	rax,[base+pshufb_shf_table-rk9]
	vpshufb	xmm5, xmm7, [rax+arg3]
	vmovdqu8 xmm7{k1}, xmm5
	jmp	.128_done

;align 16
.only_less_than_4:
	xor	eax, eax
	not	eax
	bzhi	eax, eax, DWORD(arg3) ; mask
	kmovw	k1, eax
	vmovdqu8 xmm7{k1}{z}, [arg2]
	vpxor	xmm7, xmm7, xmm0
	vmovd	eax, xmm7
	shl	DWORD(arg3), 3 ; #bytes -> #bits
	neg	DWORD(arg3)
	add	DWORD(arg3), 24
	shlx	rax, rax, arg3
	vmovq	xmm7, rax
	vpslldq	xmm7, 5
	jmp	.barrett

section .data
align 64

pshufb_shf_table:
; use these values for shift constants for the pshufb instruction
; different alignments result in values as shown:
;       dq 0x8887868584838281, 0x008f8e8d8c8b8a89 ; shl 15 (16-1) / shr1
;       dq 0x8988878685848382, 0x01008f8e8d8c8b8a ; shl 14 (16-3) / shr2
;       dq 0x8a89888786858483, 0x0201008f8e8d8c8b ; shl 13 (16-4) / shr3
;       dq 0x8b8a898887868584, 0x030201008f8e8d8c ; shl 12 (16-4) / shr4
;       dq 0x8c8b8a8988878685, 0x04030201008f8e8d ; shl 11 (16-5) / shr5
;       dq 0x8d8c8b8a89888786, 0x0504030201008f8e ; shl 10 (16-6) / shr6
;       dq 0x8e8d8c8b8a898887, 0x060504030201008f ; shl 9  (16-7) / shr7
;       dq 0x8f8e8d8c8b8a8988, 0x0706050403020100 ; shl 8  (16-8) / shr8
;       dq 0x008f8e8d8c8b8a89, 0x0807060504030201 ; shl 7  (16-9) / shr9
;       dq 0x01008f8e8d8c8b8a, 0x0908070605040302 ; shl 6  (16-10) / shr10
;       dq 0x0201008f8e8d8c8b, 0x0a09080706050403 ; shl 5  (16-11) / shr11
;       dq 0x030201008f8e8d8c, 0x0b0a090807060504 ; shl 4  (16-12) / shr12
;       dq 0x04030201008f8e8d, 0x0c0b0a0908070605 ; shl 3  (16-13) / shr13
;       dq 0x0504030201008f8e, 0x0d0c0b0a09080706 ; shl 2  (16-14) / shr14
;       dq 0x060504030201008f, 0x0e0d0c0b0a090807 ; shl 1  (16-15) / shr15
dq 0x8786858483828100, 0x8f8e8d8c8b8a8988
dq 0x0706050403020100, 0x000e0d0c0b0a0908


%ifndef USE_CONSTS
; precomputed constants
rk_1: dq 0x00000000b9e02b86
rk_2: dq 0x00000000dcb17aa4
rk1: dq 0x00000000493c7d27
rk2: dq 0x0000000ec1068c50
rk3: dq 0x0000000206e38d70
rk4: dq 0x000000006992cea2
rk5: dq 0x00000000493c7d27
rk6: dq 0x00000000dd45aab8
rk7: dq 0x00000000dea713f0
rk8: dq 0x0000000105ec76f0
rk9: dq 0x0000000047db8317
rk10: dq 0x000000002ad91c30
rk11: dq 0x000000000715ce53
rk12: dq 0x00000000c49f4f67
rk13: dq 0x0000000039d3b296
rk14: dq 0x00000000083a6eec
rk15: dq 0x000000009e4addf8
rk16: dq 0x00000000740eef02
rk17: dq 0x00000000ddc0152b
rk18: dq 0x000000001c291d04
rk19: dq 0x00000000ba4fc28e
rk20: dq 0x000000003da6d0cb

rk_1b: dq 0x00000000493c7d27
rk_2b: dq 0x0000000ec1068c50
	dq 0x000000000
	dq 0x000000000

%else
INCLUDE_CONSTS
%endif
%else  ; Assembler doesn't understand these opcodes. Add empty symbol for windows.
%ifidn __OUTPUT_FORMAT__, win64
global no_ %+ FUNCTION_NAME
no_ %+ FUNCTION_NAME %+ :
%endif
%endif ; (AS_FEATURE_LEVEL) >= 10
