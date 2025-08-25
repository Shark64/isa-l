;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;  Copyright(c) 2011-2015 Intel Corporation All rights reserved.
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

;;; Optimized xor of N source vectors using SSE
;;; int xor_check_sse(int vects, int len, void **array)

;;; Checks that array has XOR parity sum of 0 across all vectors in **array.
;;; Vectors must be aligned to 16 bytes.  Length can be any value.

%include "reg_sizes.asm"

%ifidn __OUTPUT_FORMAT__, elf64
 %define arg0  rdi
 %define arg1  rsi
 %define arg2  rdx
 %define arg3  rcx
 %define arg4  r8
 %define arg5  r9
 %define tmp   r11
 %define tmp2  rax
 %define tmp2.b al
 %define tmp3  arg4
 %define return rax
 %define PS 8
 %define func(x) x: endbranch
 %define FUNC_SAVE
 %define FUNC_RESTORE

%elifidn __OUTPUT_FORMAT__, win64
 %define arg0  rcx
 %define arg1  rdx
 %define arg2  r8
 %define arg3  r9
 %define return rax
 %define tmp2  rax
 %define tmp2.b al
 %define PS 8
 %define tmp   r11
 %define tmp3  r10
 %define stack_size  2*16 + 8 	; must be an odd multiple of 8
 %define func(x) proc_frame x

 %macro FUNC_SAVE 0
	alloc_stack	stack_size
	save_xmm128	xmm6, 0*16
	save_xmm128	xmm7, 1*16
	end_prolog
 %endmacro
 %macro FUNC_RESTORE 0
	movdqa	xmm6, [rsp + 0*16]
	movdqa	xmm7, [rsp + 1*16]
	add	rsp, stack_size
 %endmacro

%endif	; output formats


%define vec arg0
%define	len arg1
%define ptr arg3
%define pos tmp3

default rel
[bits 64]

;;; Use Non-temporal load/stor
%ifdef NO_NT_LDST
 %define XLDR movdqa
 %define XSTR movdqa
%else
 %define XLDR movntdqa
 %define XSTR movntdq
%endif

section .text

align 32
mk_global  xor_check_sse, function
func(xor_check_sse)
	FUNC_SAVE
	sub	vec, 1			; Keep as offset to last source
	jng	return_fail		;Must have at least 2 sources
	test	len, len
	je	return_pass
	test	BYTE(len), (128-1)		;Check alignment of length
	jnz	len_not_aligned


len_aligned_128bytes:
	add	len, -128		; shorter encoding for sub len, 128
	xor	DWORD(pos), DWORD(pos)

loop128:
	mov	tmp, [arg2+vec*PS]	;Fetch last pointer in array
	XLDR	xmm0, [tmp+pos]	;Start with end of array in last vector
	XLDR	xmm1, [tmp+pos+16]	;Keep xor parity in xmm0-7
	XLDR	xmm2, [tmp+pos+(2*16)]
	XLDR	xmm3, [tmp+pos+(3*16)]
	XLDR	xmm4, [tmp+pos+(4*16)]
	XLDR	xmm5, [tmp+pos+(5*16)]
	XLDR	xmm6, [tmp+pos+(6*16)]
	XLDR	xmm7, [tmp+pos+(7*16)]
	lea	tmp, [vec-1]

next_vect:
	mov 	ptr, [arg2+tmp*PS]
	xorps	xmm0, [ptr+pos]		;Get next vector (source)
	xorps	xmm1, [ptr+pos+16]
	xorps	xmm2, [ptr+pos+(2*16)]
	xorps	xmm3, [ptr+pos+(3*16)]
	xorps	xmm4, [ptr+pos+(4*16)]
	xorps	xmm5, [ptr+pos+(5*16)]
	xorps	xmm6, [ptr+pos+(6*16)]
	xorps	xmm7, [ptr+pos+(7*16)]
;;;  	prefetch [ptr+pos+(8*16)]
	sub	tmp, 1
	jge	next_vect		;Loop for each vect

	;; End of vects, check that all parity regs = 0
	por	xmm0, xmm1
	por	xmm2, xmm3
	por	xmm4, xmm5
	por	xmm6, xmm7
	por	xmm0, xmm2
	por	xmm4, xmm6
	por	xmm0, xmm4
	ptest	xmm0, xmm0
	jnz	return_fail

	sub	pos, -128
	cmp	pos, len
	jle	loop128

return_pass:
	FUNC_RESTORE
	xor	DWORD(return), DWORD(return)
	ret



;;; Do one byte at a time for no alignment case

xor_gen_byte:
	mov	tmp, vec		;Preset to last vector

loop_1byte:
	mov 	ptr, [arg2+tmp*PS] 	;Fetch last pointer in array
	mov	tmp2.b, [ptr+len-1]	;Get array n
	sub	tmp, 1
nextvect_1byte:
	mov 	ptr, [arg2+tmp*PS]
	xor	tmp2.b, [ptr+len-1]
	sub	tmp, 1
	jge	nextvect_1byte

	mov	tmp, vec		;Back to last vector
	cmp	tmp2.b, 0
	jne	return_fail
	sub	len, 1
	test	len, (8-1)
	jnz	loop_1byte

	cmp	len, 0
	je	return_pass
	test	len, (128-1)		;If not 0 and 128bit aligned
	jz	len_aligned_128bytes	; then do aligned case. len = y * 128

	;; else we are 8-byte aligned so fall through to recheck


	;; Unaligned length cases
len_not_aligned:
	test	len, (PS-1)
	jne	xor_gen_byte
	mov	tmp3, len
	and	tmp3, (128-1)		;Do the unaligned bytes 4-8 at a time
	mov	tmp, vec		;Preset to last vector

	;; Run backwards 8 bytes (4B for 32bit) at a time for (tmp3) bytes
loopN_bytes:
	mov 	ptr, [arg2+tmp*PS] 	;Fetch last pointer in array
	mov	tmp2, [ptr+len-PS]	;Get array n
	sub	tmp, 1
nextvect_Nbytes:
	mov 	ptr, [arg2+tmp*PS] 	;Get pointer to next vector
	xor	tmp2, [ptr+len-PS]
	sub	tmp, 1
	jge	nextvect_Nbytes		;Loop for each source

	mov	tmp, vec		;Back to last vector
	cmp	tmp2, 0
	jne	return_fail
	sub	len, PS
	sub	tmp3, PS
	jg	loopN_bytes

	cmp	len, 128		;Now len is aligned to 128B
	jge	len_aligned_128bytes	;We can do the rest aligned

	cmp	len, 0
	je	return_pass

return_fail:
	mov	return, 1
	FUNC_RESTORE
	ret

endproc_frame
