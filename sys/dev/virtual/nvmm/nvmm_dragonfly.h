/*
 * Copyright (c) 2021 Maxime Villard, m00nbsd.net
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

#ifndef _NVMM_DRAGONFLY_H_
#define _NVMM_DRAGONFLY_H_

#ifndef _KERNEL
#error "This file should not be included by userland programs."
#endif

#include <sys/bitops.h>
#include <sys/cpumask.h>
#include <sys/globaldata.h>
#include <sys/lock.h>
#include <sys/malloc.h> /* contigmalloc, contigfree */
#include <sys/proc.h> /* LWP_MP_URETMASK */
#include <sys/thread2.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_map.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>
#include <vm/vm_param.h> /* KERN_SUCCESS, etc. */
#include <vm/pmap.h> /* pmap_ept_transform, pmap_npt_transform */

#include <machine/atomic.h>
#include <machine/cpu.h> /* hvm_break_wanted */
#include <machine/cpufunc.h> /* ffsl, ffs, etc. */
#include <machine/segments.h>
#include <machine/smp.h> /* smp_active_mask */

/* Types. */
typedef struct vmspace		os_vmspace_t;
typedef struct vm_object	os_vmobj_t;
typedef struct lock		os_rwl_t;
typedef struct lock		os_mtx_t;
/* A few standard types. */
typedef vm_offset_t		vaddr_t;
typedef vm_offset_t		voff_t;
typedef vm_size_t		vsize_t;
typedef vm_paddr_t		paddr_t;

/* Attributes. */
#define DIAGNOSTIC		INVARIANTS
#define __cacheline_aligned	__cachealign
#define __diagused		__debugvar

/* Macros. */
#define __arraycount(__x)	(sizeof(__x) / sizeof(__x[0]))
#define __insn_barrier()	__asm __volatile("":::"memory")

/* Bitops. */
#undef  __BIT
#define __BIT(__n)		__BIT64(__n)
#undef  __BITS
#define __BITS(__m, __n)	__BITS64(__m, __n)

/* Maps. */
#define os_kernel_map		kernel_map
#define os_curproc_map		&curproc->p_vmspace->vm_map

/* R/W locks. */
#define os_rwl_init(lock)	lockinit(lock, "nvmmrw", 0, 0)
#define os_rwl_destroy(lock)	lockuninit(lock)
#define os_rwl_rlock(lock)	lockmgr(lock, LK_SHARED);
#define os_rwl_wlock(lock)	lockmgr(lock, LK_EXCLUSIVE);
#define os_rwl_unlock(lock)	lockmgr(lock, LK_RELEASE)
#define os_rwl_wheld(lock)	(lockstatus(lock, curthread) == LK_EXCLUSIVE)

/* Mutexes. */
#define os_mtx_init(lock)	lockinit(lock, "nvmmmtx", 0, 0)
#define os_mtx_destroy(lock)	lockuninit(lock)
#define os_mtx_lock(lock)	lockmgr(lock, LK_EXCLUSIVE)
#define os_mtx_unlock(lock)	lockmgr(lock, LK_RELEASE)
#define os_mtx_owned(lock)	(lockstatus(lock, curthread) == LK_EXCLUSIVE)

/* Malloc. */
MALLOC_DECLARE(M_NVMM);
#define os_mem_alloc(size)	kmalloc(size, M_NVMM, M_WAITOK)
#define os_mem_zalloc(size)	kmalloc(size, M_NVMM, M_WAITOK | M_ZERO)
#define os_mem_free(ptr, size)	kfree(ptr, M_NVMM)

/* Printf. */
#define os_printf		kprintf

/* Atomics. */
#define os_atomic_inc_uint(x)	atomic_add_int(x, 1)
#define os_atomic_dec_uint(x)	atomic_subtract_int(x, 1)
#define os_atomic_load_uint(x)	atomic_load_acq_int(x)
#define os_atomic_inc_64(x)	atomic_add_64(x, 1)

/* Pmap. */
#define os_vmspace_pmap(vm)	vmspace_pmap(vm)
#define os_vmspace_pdirpa(vm)	(vtophys(vmspace_pmap(vm)->pm_pml4))
#define os_pmap_mach(pm)	/* unused */

/* CPU. */
typedef struct globaldata	os_cpu_t;
#define OS_MAXCPUS		SMP_MAXCPU
#define OS_CPU_FOREACH(cpu)	\
	for (int idx = 0; idx < ncpus && (cpu = globaldata_find(idx)); idx++)
#define os_cpu_number(cpu)	(cpu)->gd_cpuid
#define os_curcpu()		mycpu
#define os_curcpu_number()	mycpuid
#define os_curcpu_tss_sel()	GSEL(GPROC0_SEL, SEL_KPL)
#define os_curcpu_tss()		&mycpu->gd_prvspace->common_tss
#define os_curcpu_gdt()		mdcpu->gd_gdt
#define os_curcpu_idt()		r_idt_arr[mycpuid].rd_base

/* Cpusets. */
typedef cpumask_t		os_cpuset_t;
#define os_cpuset_init(s)	\
	({ *(s) = kmalloc(sizeof(cpumask_t), M_NVMM, M_WAITOK | M_ZERO); })
#define os_cpuset_destroy(s)	kfree((s), M_NVMM)
#define os_cpuset_isset(s, c)	CPUMASK_TESTBIT(*(s), c)
#define os_cpuset_clear(s, c)	ATOMIC_CPUMASK_NANDBIT(*(s), c)
#define os_cpuset_setrunning(s)	ATOMIC_CPUMASK_ORMASK(*(s), smp_active_mask)

/*
 * Preemption.
 *
 * In DragonFly, a thread in kernel mode will not be preemptively migrated
 * to another CPU or preemptively switched to another normal kernel thread,
 * but can be preemptively switched to an interrupt thread (which switches
 * back to the kernel thread it preempted the instant it is done or blocks).
 *
 * However, we still need to use a critical section to prevent this nominal
 * interrupt thread preemption to avoid exposing interrupt threads to
 * guest DB and FP register state.  We operate under the assumption that
 * the hard interrupt code won't mess with this state.
 */
#define os_preempt_disable()	crit_enter()
#define os_preempt_enable()	crit_exit()
#define os_preempt_disabled()	(curthread->td_critcount != 0)

/* Asserts. */
#define OS_ASSERT		KKASSERT

/* Misc. */
#define ilog2(n)		((sizeof(n) > 4 ? ffsl(n) : ffs(n)) - 1)
#define uimin(a, b)		((u_int)a < (u_int)b ? (u_int)a : (u_int)b)

/* -------------------------------------------------------------------------- */

static inline bool
os_return_needed(void)
{
	if (__predict_false(hvm_break_wanted())) {
		return true;
	}
	if (__predict_false(curthread->td_lwp->lwp_mpflags & LWP_MP_URETMASK)) {
		return true;
	}
	return false;
}

/* -------------------------------------------------------------------------- */

/* IPIs. */
#define OS_IPI_FUNC(func)	void func(void *arg)

static inline void
os_ipi_unicast(os_cpu_t *cpu, void (*func)(void *), void *arg)
{
	int seq;

	seq = lwkt_send_ipiq(cpu, func, arg);
	lwkt_wait_ipiq(cpu, seq);
}

static inline void
os_ipi_broadcast(void (*func)(void *), void *arg)
{
	cpumask_t mask;
	int i;

	for (i = 0; i < ncpus; i++) {
		CPUMASK_ASSBIT(mask, i);
		lwkt_cpusync_simple(mask, func, arg);
	}
}

/*
 * In DragonFly, no need to bind the thread, because any normal kernel
 * thread will not migrate to another CPU or be preempted (except by an
 * interrupt thread).
 */
#define curlwp_bind()		((int)0)
#define curlwp_bindx(bound)	/* nothing */

#endif /* _NVMM_DRAGONFLY_H_ */
