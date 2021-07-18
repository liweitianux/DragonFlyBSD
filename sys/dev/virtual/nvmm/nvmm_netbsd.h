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

#ifndef _NVMM_NETBSD_H_
#define _NVMM_NETBSD_H_

#ifndef _KERNEL
#error "This file should not be included by userland programs."
#endif

#include <sys/atomic.h>
#include <sys/bitops.h>
#include <sys/cpu.h>
#include <sys/kcpuset.h>
#include <sys/kmem.h>
#include <sys/xcall.h>

#include <uvm/uvm_object.h>
#include <uvm/uvm_extern.h>
#include <uvm/uvm_page.h>

/* Types. */
typedef struct vmspace		os_vmspace_t;
typedef struct uvm_object	os_vmobj_t;
typedef krwlock_t		os_rwl_t;
typedef kmutex_t		os_mtx_t;

/* Maps. */
#define os_kernel_map		kernel_map
#define os_curproc_map		&curproc->p_vmspace->vm_map

/* R/W locks. */
#define os_rwl_init(lock)	rw_init(lock)
#define os_rwl_destroy(lock)	rw_destroy(lock)
#define os_rwl_rlock(lock)	rw_enter(lock, RW_READER);
#define os_rwl_wlock(lock)	rw_enter(lock, RW_WRITER);
#define os_rwl_unlock(lock)	rw_exit(lock)
#define os_rwl_wheld(lock)	rw_write_held(lock)

/* Mutexes. */
#define os_mtx_init(lock)	mutex_init(lock, MUTEX_DEFAULT, IPL_NONE)
#define os_mtx_destroy(lock)	mutex_destroy(lock)
#define os_mtx_lock(lock)	mutex_enter(lock)
#define os_mtx_unlock(lock)	mutex_exit(lock)
#define os_mtx_owned(lock)	mutex_owned(lock)

/* Malloc. */
#define os_mem_alloc(size)	kmem_alloc(size, KM_SLEEP)
#define os_mem_zalloc(size)	kmem_zalloc(size, KM_SLEEP)
#define os_mem_free(ptr, size)	kmem_free(ptr, size)

/* Printf. */
#define os_printf		printf

/* Atomics. */
#define os_atomic_inc_uint(x)	atomic_inc_uint(x)
#define os_atomic_dec_uint(x)	atomic_dec_uint(x)
#define os_atomic_load_uint(x)	atomic_load_relaxed(x)
#define os_atomic_inc_64(x)	atomic_inc_64(x)

/* Pmap. */
extern bool pmap_ept_has_ad;
#define os_vmspace_pmap(vm)	((vm)->vm_map.pmap)
#define os_vmspace_pdirpa(vm)	((vm)->vm_map.pmap->pm_pdirpa[0])
#define os_pmap_mach(pm)	((pm)->pm_data)

/* CPU. */
typedef struct cpu_info		os_cpu_t;
#define OS_MAXCPUS		MAXCPUS
#define OS_CPU_FOREACH(cpu)	for (CPU_INFO_FOREACH(, cpu))
#define os_cpu_number(cpu)	cpu_index(cpu)
#define os_curcpu()		curcpu()
#define os_curcpu_number()	cpu_number()
#define os_curcpu_tss_sel()	curcpu()->ci_tss_sel
#define os_curcpu_tss()		curcpu()->ci_tss
#define os_curcpu_gdt()		curcpu()->ci_gdt
#define os_curcpu_idt()		curcpu()->ci_idtvec.iv_idt

/* Cpusets. */
typedef kcpuset_t		os_cpuset_t;
#define os_cpuset_init(s)	kcpuset_create(s, true)
#define os_cpuset_destroy(s)	kcpuset_destroy(s)
#define os_cpuset_isset(s, c)	kcpuset_isset(s, c)
#define os_cpuset_clear(s, c)	kcpuset_clear(s, c)
#define os_cpuset_setrunning(s)	kcpuset_copy(s, kcpuset_running)

/* Preemption. */
#define os_preempt_disable()	kpreempt_disable()
#define os_preempt_enable()	kpreempt_enable()
#define os_preempt_disabled()	kpreempt_disabled()

/* Asserts. */
#define OS_ASSERT		KASSERT

/* -------------------------------------------------------------------------- */

static inline bool
os_return_needed(void)
{
	if (preempt_needed()) {
		return true;
	}
	if (curlwp->l_flag & LW_USERRET) {
		return true;
	}
	return false;
}

/* -------------------------------------------------------------------------- */

/* IPIs. */
#define OS_IPI_FUNC(func)	void func(void *arg, void *unused)

static inline void
os_ipi_unicast(os_cpu_t *cpu, void (*func)(void *, void *), void *arg)
{
	xc_wait(xc_unicast(XC_HIGHPRI, func, arg, NULL, cpu));
}

static inline void
os_ipi_broadcast(void (*func)(void *, void *), void *arg)
{
	xc_wait(xc_broadcast(0, func, arg, NULL));
}

static inline void
os_ipi_kickall(void)
{
	/*
	 * XXX: this is probably too expensive. NetBSD should have a dummy
	 * interrupt handler that just IRETs without doing anything.
	 */
	pmap_tlb_shootdown(pmap_kernel(), -1, PTE_G, TLBSHOOT_NVMM);
}

#endif /* _NVMM_NETBSD_H_ */
