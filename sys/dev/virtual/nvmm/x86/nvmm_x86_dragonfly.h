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

#ifndef _NVMM_X86_DRAGONFLY_H_
#define _NVMM_X86_DRAGONFLY_H_

#ifndef _KERNEL
#error "This file should not be included by userland programs."
#endif

#include <sys/proc.h> /* struct lwp */
#include <machine/cpufunc.h>
#include <machine/npx.h>

/*
 * Register defines. We mainly rely on the already-existing OS definitions.
 */

#define XCR0_X87		CPU_XFEATURE_X87	/* 0x00000001 */
#define XCR0_SSE		CPU_XFEATURE_SSE	/* 0x00000002 */

#define MSR_MISC_ENABLE		MSR_IA32_MISC_ENABLE	/* 0x1a0 */
#define MSR_CR_PAT		MSR_PAT			/* 0x277 */
#define MSR_SFMASK		MSR_SF_MASK		/* 0xc0000084 */
#define MSR_KERNELGSBASE	MSR_KGSBASE		/* 0xc0000102 */
#define MSR_NB_CFG		MSR_AMD_NB_CFG		/* 0xc001001f */
#define MSR_IC_CFG		MSR_AMD_IC_CFG		/* 0xc0011021 */
#define MSR_DE_CFG		MSR_AMD_DE_CFG		/* 0xc0011029 */
#define MSR_UCODE_AMD_PATCHLEVEL MSR_AMD_PATCH_LEVEL	/* 0x0000008b */

/* MSR_IA32_ARCH_CAPABILITIES (0x10a) */
#define 	IA32_ARCH_RDCL_NO	IA32_ARCH_CAP_RDCL_NO
#define 	IA32_ARCH_IBRS_ALL	IA32_ARCH_CAP_IBRS_ALL
#define 	IA32_ARCH_RSBA		IA32_ARCH_CAP_RSBA
#define 	IA32_ARCH_SKIP_L1DFL_VMENTRY	IA32_ARCH_CAP_SKIP_L1DFL_VMENTRY
#define 	IA32_ARCH_SSB_NO	IA32_ARCH_CAP_SSB_NO
#define 	IA32_ARCH_MDS_NO	IA32_ARCH_CAP_MDS_NO
#define 	IA32_ARCH_IF_PSCHANGE_MC_NO	IA32_ARCH_CAP_IF_PSCHANGE_MC_NO
#define 	IA32_ARCH_TSX_CTRL	IA32_ARCH_CAP_TSX_CTRL
#define 	IA32_ARCH_TAA_NO	IA32_ARCH_CAP_TAA_NO

/* MSR_IA32_FLUSH_CMD (0x10b) */
#define 	IA32_FLUSH_CMD_L1D_FLUSH	IA32_FLUSH_CMD_L1D

/* -------------------------------------------------------------------------- */

/*
 * ASM defines. We mainly rely on the already-existing OS definitions.
 */

/* CPUID. */
#define x86_get_cpuid(l, d)	do_cpuid(l, (uint32_t *)d)
#define x86_get_cpuid2(l, c, d)	cpuid_count(l, c, (uint32_t *)d)

/* Control registers. */
#define x86_get_cr0()		rcr0()
#define x86_get_cr2()		rcr2()
#define x86_get_cr3()		rcr3()
#define x86_get_cr4()		rcr4()
#define x86_set_cr0(v)		load_cr0(v)
#define x86_set_cr2(v)		load_cr2(v)
#define x86_set_cr4(v)		load_cr4(v)

/* Debug registers. */
static inline void
x86_curthread_save_dbregs(uint64_t *drs)
{
	struct pcb *pcb = curthread->td_lwp->lwp_thread->td_pcb;

	if (__predict_true(!(pcb->pcb_flags & PCB_DBREGS)))
		return;

	drs[NVMM_X64_DR_DR0] = rdr0();
	drs[NVMM_X64_DR_DR1] = rdr1();
	drs[NVMM_X64_DR_DR2] = rdr2();
	drs[NVMM_X64_DR_DR3] = rdr3();
	drs[NVMM_X64_DR_DR6] = rdr6();
	drs[NVMM_X64_DR_DR7] = rdr7();
}

static inline void
x86_curthread_restore_dbregs(uint64_t *drs)
{
	struct pcb *pcb = curthread->td_lwp->lwp_thread->td_pcb;

	if (__predict_true(!(pcb->pcb_flags & PCB_DBREGS)))
		return;

	load_dr0(drs[NVMM_X64_DR_DR0]);
	load_dr1(drs[NVMM_X64_DR_DR1]);
	load_dr2(drs[NVMM_X64_DR_DR2]);
	load_dr3(drs[NVMM_X64_DR_DR3]);
	load_dr6(drs[NVMM_X64_DR_DR6]);
	load_dr7(drs[NVMM_X64_DR_DR7]);
}

#define x86_get_dr0()		rdr0()
#define x86_get_dr1()		rdr1()
#define x86_get_dr2()		rdr2()
#define x86_get_dr3()		rdr3()
#define x86_get_dr6()		rdr6()
#define x86_get_dr7()		rdr7()
#define x86_set_dr0(v)		load_dr0(v)
#define x86_set_dr1(v)		load_dr1(v)
#define x86_set_dr2(v)		load_dr2(v)
#define x86_set_dr3(v)		load_dr3(v)
#define x86_set_dr6(v)		load_dr6(v)
#define x86_set_dr7(v)		load_dr7(v)

/* FPU. */
#define x86_curthread_save_fpu()	/* TODO */
#define x86_curthread_restore_fpu()	/* TODO */
#define x86_save_fpu(a, m)				\
	({						\
		fpusave((union savefpu *)(a), m);	\
		load_cr0(rcr0() | CR0_TS);		\
	})
#define x86_restore_fpu(a, m)				\
	({						\
		__asm volatile("clts" ::: "memory");	\
		fpurstor((union savefpu *)(a), m);	\
	})

#define x86_xsave_features	npx_xcr0_mask
#define x86_fpu_mxcsr_mask	npx_mxcsr_mask

#endif /* _NVMM_X86_DRAGONFLY_H_ */
