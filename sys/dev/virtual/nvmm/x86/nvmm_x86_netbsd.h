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

#ifndef _NVMM_X86_NETBSD_H_
#define _NVMM_X86_NETBSD_H_

#ifndef _KERNEL
#error "This file should not be included by userland programs."
#endif

#include <x86/dbregs.h>
#include <x86/fpu.h>

/*
 * ASM defines. We mainly rely on the already-existing OS definitions.
 */

/* CPUID. */
#define x86_get_cpuid(l, d)	x86_cpuid(l, (uint32_t *)d)
#define x86_get_cpuid2(l, c, d)	x86_cpuid2(l, c, (uint32_t *)d)

/* Control registers. */
#define x86_get_cr0()		rcr0()
#define x86_get_cr2()		rcr2()
#define x86_get_cr3()		rcr3()
#define x86_get_cr4()		rcr4()
#define x86_set_cr0(v)		lcr0(v)
#define x86_set_cr2(v)		lcr2(v)
#define x86_set_cr4(v)		lcr4(v)

/* Debug registers. */
static inline void
x86_curthread_save_dbregs(uint64_t *drs __unused)
{
	x86_dbregs_save(curlwp);
}

static inline void
x86_curthread_restore_dbregs(uint64_t *drs __unused)
{
	x86_dbregs_restore(curlwp);
}

#define x86_get_dr0()		rdr0()
#define x86_get_dr1()		rdr1()
#define x86_get_dr2()		rdr2()
#define x86_get_dr3()		rdr3()
#define x86_get_dr6()		rdr6()
#define x86_get_dr7()		rdr7()
#define x86_set_dr0(v)		ldr0(v)
#define x86_set_dr1(v)		ldr1(v)
#define x86_set_dr2(v)		ldr2(v)
#define x86_set_dr3(v)		ldr3(v)
#define x86_set_dr6(v)		ldr6(v)
#define x86_set_dr7(v)		ldr7(v)

/* FPU. */
#define x86_curthread_save_fpu()	fpu_kern_enter()
#define x86_curthread_restore_fpu()	fpu_kern_leave()
#define x86_save_fpu(a, m)		fpu_area_save(a, m, true)
#define x86_restore_fpu(a, m)		fpu_area_restore(a, m, true)

#endif /* _NVMM_X86_NETBSD_H_ */
