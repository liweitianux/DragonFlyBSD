/*
 * Copyright (c) 2003-2013 The DragonFly Project.  All rights reserved.
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Mihai Carabas <mihai.carabas@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/systm.h>
#include <sys/sfbuf.h>
#include <sys/proc.h>
#include <sys/thread.h>

#include <machine/pmap.h>
#include <machine/specialreg.h>
#include <machine/cpufunc.h>
#include <machine/vmm.h>

#include <vm/pmap.h>
#include <vm/vm_extern.h>
#include <vm/vm_map.h>

#include "vmx.h"
#include "ept.h"
#include "vmm_utils.h"
#include "vmm.h"

static int pmap_pm_flags_ept = 0;
static int eptp_bits = 0;

int
vmx_ept_init(void)
{
	vmx_ept_vpid_cap = rdmsr(IA32_VMX_EPT_VPID_CAP);

	if (!EPT_PWL4(vmx_ept_vpid_cap) ||
	    !EPT_MEMORY_TYPE_WB(vmx_ept_vpid_cap)) {
		return EINVAL;
	}

	eptp_bits |= EPTP_CACHE(PAT_WRITE_BACK) |
	    EPTP_PWLEN(EPT_PWLEVELS - 1);

	if (EPT_AD_BITS_SUPPORTED(vmx_ept_vpid_cap)) {
		eptp_bits |= EPTP_AD_ENABLE;
	} else {
		pmap_pm_flags_ept |= PMAP_EMULATE_AD_BITS;
	}

	return 0;
}

/* Build the VMCS_EPTP pointer
 * - the ept_address
 * - the EPTP bits indicating optional features
 */
uint64_t vmx_eptp(uint64_t ept_address)
{
	return (ept_address | eptp_bits);
}

/* Copyin from guest VMM */
static int
ept_copyin(const void *udaddr, void *kaddr, size_t len)
{
	struct lwbuf *lwb;
	struct lwbuf lwb_cache;
	vm_page_t m;
	register_t gpa;
	size_t n;
	int error;
	struct vmspace *vm = curproc->p_vmspace;
	struct vmx_thread_info *vti = curthread->td_vmm;
	register_t guest_cr3 = vti->guest_cr3;

	error = 0;

	while (len) {
		/* Get the GPA by manually walking the-GUEST page table*/
		error = guest_phys_addr(vm, &gpa, guest_cr3,
					(vm_offset_t)udaddr);
		if (error) {
			kprintf("%s: could not get guest_phys_addr\n",
				__func__);
			break;
		}

		m = vm_fault_page(&vm->vm_map, trunc_page(gpa),
				  VM_PROT_READ, VM_FAULT_NORMAL,
				  &error, NULL);
		if (error) {
			if (vmm_debug) {
				kprintf("%s: could not fault in "
					"vm map, gpa: %jx\n",
					__func__,
					(uintmax_t)gpa);
			}
			break;
		}

		n = PAGE_SIZE - ((vm_offset_t)udaddr & PAGE_MASK);
		if (n > len)
			n = len;

		lwb = lwbuf_alloc(m, &lwb_cache);
		bcopy((char *)lwbuf_kva(lwb) +
		       ((vm_offset_t)udaddr & PAGE_MASK),
		      kaddr, n);
		len -= n;
		udaddr = (const char *)udaddr + n;
		kaddr = (char *)kaddr + n;
		lwbuf_free(lwb);
		vm_page_unhold(m);
	}
	if (error)
		error = EFAULT;
	return (error);
}

/* Copyout from guest VMM */
static int
ept_copyout(const void *kaddr, void *udaddr, size_t len)
{
	struct lwbuf *lwb;
	struct lwbuf lwb_cache;
	vm_page_t m;
	register_t gpa;
	size_t n;
	int error;
	struct vmspace *vm = curproc->p_vmspace;
	struct vmx_thread_info *vti = curthread->td_vmm;
	register_t guest_cr3 = vti->guest_cr3;

	error = 0;

	while (len) {
		int busy;

		/* Get the GPA by manually walking the-GUEST page table*/
		error = guest_phys_addr(vm, &gpa, guest_cr3,
					(vm_offset_t)udaddr);
		if (error) {
			kprintf("%s: could not get guest_phys_addr\n",
				__func__);
			break;
		}

		m = vm_fault_page(&vm->vm_map, trunc_page(gpa),
				  VM_PROT_READ | VM_PROT_WRITE,
				  VM_FAULT_NORMAL,
				  &error, &busy);
		if (error) {
			if (vmm_debug) {
				kprintf("%s: could not fault in vm map, "
					"gpa: 0x%jx\n",
					__func__,
					(uintmax_t)gpa);
			}
			break;
		}

		n = PAGE_SIZE - ((vm_offset_t)udaddr & PAGE_MASK);
		if (n > len)
			n = len;

		lwb = lwbuf_alloc(m, &lwb_cache);
		bcopy(kaddr, (char *)lwbuf_kva(lwb) +
			     ((vm_offset_t)udaddr & PAGE_MASK), n);

		len -= n;
		udaddr = (char *)udaddr + n;
		kaddr = (const char *)kaddr + n;
		vm_page_dirty(m);
#if 0
		/* should not be needed */
		cpu_invlpg((char *)lwbuf_kva(lwb) +
			     ((vm_offset_t)udaddr & PAGE_MASK));
#endif
		lwbuf_free(lwb);
		if (busy)
			vm_page_wakeup(m);
		else
			vm_page_unhold(m);
	}
	if (error)
		error = EFAULT;
	return (error);
}

static int
ept_copyinstr(const void *udaddr, void *kaddr, size_t len, size_t *res)
{
	int error;
	size_t n;
	const char *uptr = udaddr;
	char *kptr = kaddr;

	if (res)
		*res = 0;
	while (len) {
		n = PAGE_SIZE - ((vm_offset_t)uptr & PAGE_MASK);
		if (n > 32)
			n = 32;
		if (n > len)
			n = len;
		if ((error = ept_copyin(uptr, kptr, n)) != 0)
			return(error);
		while (n) {
			if (res)
				++*res;
			if (*kptr == 0)
				return(0);
			++kptr;
			++uptr;
			--n;
			--len;
		}

	}
	return(ENAMETOOLONG);
}


static int
ept_fubyte(const uint8_t *base)
{
	uint8_t c = 0;

	if (ept_copyin(base, &c, 1) == 0)
		return((int)c);
	return(-1);
}

static int
ept_subyte(uint8_t *base, uint8_t byte)
{
	unsigned char c = byte;

	if (ept_copyout(&c, base, 1) == 0)
		return(0);
	return(-1);
}

static int32_t
ept_fuword32(const uint32_t *base)
{
	uint32_t v;

	if (ept_copyin(base, &v, sizeof(v)) == 0)
		return(v);
	return(-1);
}

static int64_t
ept_fuword64(const uint64_t *base)
{
	uint64_t v;

	if (ept_copyin(base, &v, sizeof(v)) == 0)
		return(v);
	return(-1);
}

static int
ept_suword64(uint64_t *base, uint64_t word)
{
	if (ept_copyout(&word, base, sizeof(word)) == 0)
		return(0);
	return(-1);
}

static int
ept_suword32(uint32_t *base, int word)
{
	if (ept_copyout(&word, base, sizeof(word)) == 0)
		return(0);
	return(-1);
}

static uint32_t
ept_swapu32(volatile uint32_t *uaddr, uint32_t v)
{
	struct lwbuf *lwb;
	struct lwbuf lwb_cache;
	vm_page_t m;
	register_t gpa;
	size_t n;
	int error;
	struct vmspace *vm = curproc->p_vmspace;
	struct vmx_thread_info *vti = curthread->td_vmm;
	register_t guest_cr3 = vti->guest_cr3;
	volatile void *ptr;
	int busy;

	/* Get the GPA by manually walking the-GUEST page table*/
	error = guest_phys_addr(vm, &gpa, guest_cr3, (vm_offset_t)uaddr);
	if (error) {
		kprintf("%s: could not get guest_phys_addr\n", __func__);
		return EFAULT;
	}
	m = vm_fault_page(&vm->vm_map, trunc_page(gpa),
			  VM_PROT_READ | VM_PROT_WRITE,
			  VM_FAULT_NORMAL,
			  &error, &busy);
	if (error) {
		if (vmm_debug) {
			kprintf("%s: could not fault in vm map, gpa: %llx\n",
				__func__, (unsigned long long) gpa);
		}
		return EFAULT;
	}

	n = PAGE_SIZE - ((vm_offset_t)uaddr & PAGE_MASK);
	if (n < sizeof(uint32_t)) {
		error = EFAULT;
		v = (uint32_t)-error;
		goto done;
	}

	lwb = lwbuf_alloc(m, &lwb_cache);
	ptr = (void *)(lwbuf_kva(lwb) + ((vm_offset_t)uaddr & PAGE_MASK));
	v = atomic_swap_int(ptr, v);

	vm_page_dirty(m);
	lwbuf_free(lwb);
	error = 0;
done:
	if (busy)
		vm_page_wakeup(m);
	else
		vm_page_unhold(m);
	return v;
}

static uint64_t
ept_swapu64(volatile uint64_t *uaddr, uint64_t v)
{
	struct lwbuf *lwb;
	struct lwbuf lwb_cache;
	vm_page_t m;
	register_t gpa;
	size_t n;
	int error;
	struct vmspace *vm = curproc->p_vmspace;
	struct vmx_thread_info *vti = curthread->td_vmm;
	register_t guest_cr3 = vti->guest_cr3;
	volatile void *ptr;
	int busy;

	/* Get the GPA by manually walking the-GUEST page table*/
	error = guest_phys_addr(vm, &gpa, guest_cr3, (vm_offset_t)uaddr);
	if (error) {
		kprintf("%s: could not get guest_phys_addr\n", __func__);
		return EFAULT;
	}
	m = vm_fault_page(&vm->vm_map, trunc_page(gpa),
			  VM_PROT_READ | VM_PROT_WRITE,
			  VM_FAULT_NORMAL,
			  &error, &busy);
	if (error) {
		if (vmm_debug) {
			kprintf("%s: could not fault in vm map, gpa: %llx\n",
				__func__, (unsigned long long) gpa);
		}
		return EFAULT;
	}

	n = PAGE_SIZE - ((vm_offset_t)uaddr & PAGE_MASK);
	if (n < sizeof(uint64_t)) {
		error = EFAULT;
		v = (uint64_t)-error;
		goto done;
	}

	lwb = lwbuf_alloc(m, &lwb_cache);
	ptr = (void *)(lwbuf_kva(lwb) + ((vm_offset_t)uaddr & PAGE_MASK));
	v = atomic_swap_long(ptr, v);

	vm_page_dirty(m);
	lwbuf_free(lwb);
	error = 0;
done:
	if (busy)
		vm_page_wakeup(m);
	else
		vm_page_unhold(m);
	return v;
}

static uint32_t
ept_fuwordadd32(volatile uint32_t *uaddr, uint32_t v)
{
	struct lwbuf *lwb;
	struct lwbuf lwb_cache;
	vm_page_t m;
	register_t gpa;
	size_t n;
	int error;
	struct vmspace *vm = curproc->p_vmspace;
	struct vmx_thread_info *vti = curthread->td_vmm;
	register_t guest_cr3 = vti->guest_cr3;
	volatile void *ptr;
	int busy;

	/* Get the GPA by manually walking the-GUEST page table*/
	error = guest_phys_addr(vm, &gpa, guest_cr3, (vm_offset_t)uaddr);
	if (error) {
		kprintf("%s: could not get guest_phys_addr\n", __func__);
		return EFAULT;
	}
	m = vm_fault_page(&vm->vm_map, trunc_page(gpa),
			  VM_PROT_READ | VM_PROT_WRITE,
			  VM_FAULT_NORMAL,
			  &error, &busy);
	if (error) {
		if (vmm_debug) {
			kprintf("%s: could not fault in vm map, gpa: %llx\n",
				__func__, (unsigned long long) gpa);
		}
		return EFAULT;
	}

	n = PAGE_SIZE - ((vm_offset_t)uaddr & PAGE_MASK);
	if (n < sizeof(uint32_t)) {
		error = EFAULT;
		v = (uint32_t)-error;
		goto done;
	}

	lwb = lwbuf_alloc(m, &lwb_cache);
	ptr = (void *)(lwbuf_kva(lwb) + ((vm_offset_t)uaddr & PAGE_MASK));
	v = atomic_fetchadd_int(ptr, v);

	vm_page_dirty(m);
	lwbuf_free(lwb);
	error = 0;
done:
	if (busy)
		vm_page_wakeup(m);
	else
		vm_page_unhold(m);
	return v;
}

static uint64_t
ept_fuwordadd64(volatile uint64_t *uaddr, uint64_t v)
{
	struct lwbuf *lwb;
	struct lwbuf lwb_cache;
	vm_page_t m;
	register_t gpa;
	size_t n;
	int error;
	struct vmspace *vm = curproc->p_vmspace;
	struct vmx_thread_info *vti = curthread->td_vmm;
	register_t guest_cr3 = vti->guest_cr3;
	volatile void *ptr;
	int busy;

	/* Get the GPA by manually walking the-GUEST page table*/
	error = guest_phys_addr(vm, &gpa, guest_cr3, (vm_offset_t)uaddr);
	if (error) {
		kprintf("%s: could not get guest_phys_addr\n", __func__);
		return EFAULT;
	}
	m = vm_fault_page(&vm->vm_map, trunc_page(gpa),
			  VM_PROT_READ | VM_PROT_WRITE,
			  VM_FAULT_NORMAL,
			  &error, &busy);
	if (error) {
		if (vmm_debug) {
			kprintf("%s: could not fault in vm map, gpa: %llx\n",
				__func__, (unsigned long long) gpa);
		}
		return EFAULT;
	}

	n = PAGE_SIZE - ((vm_offset_t)uaddr & PAGE_MASK);
	if (n < sizeof(uint64_t)) {
		error = EFAULT;
		v = (uint64_t)-error;
		goto done;
	}

	lwb = lwbuf_alloc(m, &lwb_cache);
	ptr = (void *)(lwbuf_kva(lwb) + ((vm_offset_t)uaddr & PAGE_MASK));
	v = atomic_fetchadd_long(ptr, v);

	vm_page_dirty(m);
	lwbuf_free(lwb);
	error = 0;
done:
	if (busy)
		vm_page_wakeup(m);
	else
		vm_page_unhold(m);
	return v;
}

void
vmx_ept_pmap_pinit(pmap_t pmap)
{
	pmap_ept_transform(pmap, pmap_pm_flags_ept);

	pmap->copyinstr = ept_copyinstr;
	pmap->copyin = ept_copyin;
	pmap->copyout = ept_copyout;
	pmap->fubyte = ept_fubyte;
	pmap->subyte = ept_subyte;
	pmap->fuword32 = ept_fuword32;
	pmap->fuword64 = ept_fuword64;
	pmap->suword32 = ept_suword32;
	pmap->suword64 = ept_suword64;
	pmap->swapu32 = ept_swapu32;
	pmap->swapu64 = ept_swapu64;
	pmap->fuwordadd32 = ept_fuwordadd32;
	pmap->fuwordadd64 = ept_fuwordadd64;
}
