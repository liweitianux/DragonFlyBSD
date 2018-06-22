/*
 * Copyright (c) 1987, 1988, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)disklabel.h	8.2 (Berkeley) 7/10/94
 */

#ifndef _SYS_DISKMBR_H_
#define	_SYS_DISKMBR_H_

#include <sys/types.h>

#define	DOSBBSECTOR	0	/* DOS boot block relative sector number */
#define	DOSPARTOFF	446
#define	DOSPARTSIZE	16
#define	NDOSPART	4
#define	NEXTDOSPART	32
#define	DOSMAGICOFFSET	510
#define	DOSMAGIC	0xAA55

/*
 * NOTE: DragonFlyBSD had been using 0xA5 forever but after many years
 *	 we're finally shifting to our own as A5 causes conflicts in grub.
 */
#define	DOSPTYP_DFLYBSD	0x6c	/* DragonFlyBSD partition type */
#define	DOSPTYP_386BSD	0xa5	/* 386BSD partition type */
#define	DOSPTYP_OPENBSD	0xa6	/* OpenBSD partition type */
#define	DOSPTYP_NETBSD	0xa9	/* NetBSD partition type */
#define	DOSPTYP_LINSWP	0x82	/* Linux swap partition */
#define	DOSPTYP_LINUX	0x83	/* Linux partition */
#define	DOSPTYP_PMBR	0xee	/* GPT Protective MBR */
#define	DOSPTYP_EFI	0xef	/* EFI system partition */
#define	DOSPTYP_EXT	5	/* DOS extended partition */
#define	DOSPTYP_EXTLBA	15	/* DOS extended partition */

/*
 * Note that sector numbers in a legacy MBR DOS partition start at 1 instead
 * of 0, so the first sector of the disk would be cyl 0, head 0, sector 1
 * and translate to block 0 in the actual disk I/O.
 */
struct dos_partition {
	unsigned char	dp_flag;	/* bootstrap flags */
	unsigned char	dp_shd;		/* starting head */
	unsigned char	dp_ssect;	/* starting sector */
	unsigned char	dp_scyl;	/* starting cylinder */
	unsigned char	dp_typ;		/* partition type */
	unsigned char	dp_ehd;		/* end head */
	unsigned char	dp_esect;	/* end sector */
	unsigned char	dp_ecyl;	/* end cylinder */
	uint32_t	dp_start;	/* absolute starting sector number */
	uint32_t	dp_size;	/* partition size in sectors */
};

#ifndef CTASSERT
#define CTASSERT(x)		_CTASSERT(x, __LINE__)
#define _CTASSERT(x, y)		__CTASSERT(x, y)
#define __CTASSERT(x, y)	typedef char __assert_ ## y [(x) ? 1 : -1]
#endif

CTASSERT(sizeof (struct dos_partition) == DOSPARTSIZE);

#define	DPSECT(s) ((s) & 0x3f)		/* isolate relevant bits of sector */
#define	DPCYL(c, s) ((c) + (((s) & 0xc0)<<2)) /* and those that are cylinder */

#endif /* !_SYS_DISKMBR_H_ */
