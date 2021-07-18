/*
 * Copyright (c) 2018-2021 Maxime Villard, m00nbsd.net
 * All rights reserved.
 *
 * This code is part of the NVMM hypervisor.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _NVMM_X86_ASM_H_
#define _NVMM_X86_ASM_H_

/*
 * State indexes. We use X64 as naming convention, not to confuse with X86
 * which originally implied 32bit.
 */

/* Segments. */
#define NVMM_X64_SEG_ES			0
#define NVMM_X64_SEG_CS			1
#define NVMM_X64_SEG_SS			2
#define NVMM_X64_SEG_DS			3
#define NVMM_X64_SEG_FS			4
#define NVMM_X64_SEG_GS			5
#define NVMM_X64_SEG_GDT		6
#define NVMM_X64_SEG_IDT		7
#define NVMM_X64_SEG_LDT		8
#define NVMM_X64_SEG_TR			9
#define NVMM_X64_NSEG			10

/* General Purpose Registers. */
#define NVMM_X64_GPR_RAX		0
#define NVMM_X64_GPR_RCX		1
#define NVMM_X64_GPR_RDX		2
#define NVMM_X64_GPR_RBX		3
#define NVMM_X64_GPR_RSP		4
#define NVMM_X64_GPR_RBP		5
#define NVMM_X64_GPR_RSI		6
#define NVMM_X64_GPR_RDI		7
#define NVMM_X64_GPR_R8			8
#define NVMM_X64_GPR_R9			9
#define NVMM_X64_GPR_R10		10
#define NVMM_X64_GPR_R11		11
#define NVMM_X64_GPR_R12		12
#define NVMM_X64_GPR_R13		13
#define NVMM_X64_GPR_R14		14
#define NVMM_X64_GPR_R15		15
#define NVMM_X64_GPR_RIP		16
#define NVMM_X64_GPR_RFLAGS		17
#define NVMM_X64_NGPR			18

/* Control Registers. */
#define NVMM_X64_CR_CR0			0
#define NVMM_X64_CR_CR2			1
#define NVMM_X64_CR_CR3			2
#define NVMM_X64_CR_CR4			3
#define NVMM_X64_CR_CR8			4
#define NVMM_X64_CR_XCR0		5
#define NVMM_X64_NCR			6

/* Debug Registers. */
#define NVMM_X64_DR_DR0			0
#define NVMM_X64_DR_DR1			1
#define NVMM_X64_DR_DR2			2
#define NVMM_X64_DR_DR3			3
#define NVMM_X64_DR_DR6			4
#define NVMM_X64_DR_DR7			5
#define NVMM_X64_NDR			6

/* MSRs. */
#define NVMM_X64_MSR_EFER		0
#define NVMM_X64_MSR_STAR		1
#define NVMM_X64_MSR_LSTAR		2
#define NVMM_X64_MSR_CSTAR		3
#define NVMM_X64_MSR_SFMASK		4
#define NVMM_X64_MSR_KERNELGSBASE	5
#define NVMM_X64_MSR_SYSENTER_CS	6
#define NVMM_X64_MSR_SYSENTER_ESP	7
#define NVMM_X64_MSR_SYSENTER_EIP	8
#define NVMM_X64_MSR_PAT		9
#define NVMM_X64_MSR_TSC		10
#define NVMM_X64_NMSR			11

#endif /* _NVMM_X86_ASM_H_ */
