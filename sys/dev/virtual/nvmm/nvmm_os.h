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

#ifndef _NVMM_OS_H_
#define _NVMM_OS_H_

#ifndef _KERNEL
#error "This file should not be included by userland programs."
#endif

#if defined(__NetBSD__)
#include "nvmm_netbsd.h"
#elif defined(__DragonFly__)
#include "nvmm_dragonfly.h"
#else
#error "Unsupported OS."
#endif

/* -------------------------------------------------------------------------- */

os_vmspace_t *	os_vmspace_create(vaddr_t, vaddr_t);
void		os_vmspace_destroy(os_vmspace_t *);
int		os_vmspace_fault(os_vmspace_t *, vaddr_t, vm_prot_t);

os_vmobj_t *	os_vmobj_create(voff_t);
void		os_vmobj_ref(os_vmobj_t *);
void		os_vmobj_rel(os_vmobj_t *);

int		os_vmobj_map(struct vm_map *, vaddr_t *, vsize_t, os_vmobj_t *,
		    voff_t, bool, bool, bool, int, int);
void		os_vmobj_unmap(struct vm_map *map, vaddr_t, vaddr_t, bool);

void *		os_pagemem_zalloc(size_t);
void		os_pagemem_free(void *, size_t);

paddr_t		os_pa_zalloc(void);
void		os_pa_free(paddr_t);

int		os_contigpa_zalloc(paddr_t *, vaddr_t *, size_t);
void		os_contigpa_free(paddr_t, vaddr_t, size_t);

#endif /* _NVMM_OS_H_ */
