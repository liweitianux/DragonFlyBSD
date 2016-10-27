/*-
 * Copyright (c) 1997, 1998 Doug Rabson
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/sys/bus_private.h,v 1.11.2.2 2000/08/03 00:25:22 peter Exp $
 * $DragonFly: src/sys/sys/bus_private.h,v 1.10 2008/09/30 12:20:29 hasso Exp $
 */

#ifndef _SYS_BUS_PRIVATE_H_
#define _SYS_BUS_PRIVATE_H_

#include <sys/sysctl.h>

#if !defined(_KERNEL) && !defined(_KERNEL_STRUCTURES)

#error "This file should not be included by userland programs."

#else

#ifndef _SYS_BUS_H_
#include <sys/bus.h>
#endif

/*
 * Used to attach drivers to devclasses.
 */
typedef struct driverlink *driverlink_t;
struct driverlink {
	kobj_class_t		driver;
	TAILQ_ENTRY(driverlink) link;	/* list of drivers in devclass */
};

/*
 * Forward declarations
 */
typedef TAILQ_HEAD(devclass_list, devclass) devclass_list_t;
typedef TAILQ_HEAD(driver_list, driverlink) driver_list_t;
typedef TAILQ_HEAD(device_list, bsd_device) device_list_t;

struct devclass {
	TAILQ_ENTRY(devclass) link;
	devclass_t	parent;		/* parent in devclass hierarchy */
	driver_list_t	drivers;	/* bus devclasses store drivers for bus */
	char		*name;
	device_t	*devices;	/* array of devices indexed by unit */
	int		maxunit;	/* size of devices array */

	struct sysctl_ctx_list sysctl_ctx;
	struct sysctl_oid *sysctl_tree;
};

/*
 * Resources from config(8).
 */
typedef enum {
	RES_INT, RES_STRING, RES_LONG
} resource_type;

struct config_resource {
	char		*name;
	resource_type	type;
	union {
		long	longval;
		int	intval;
		char*	stringval;
	} u;
};

struct config_device {
	char			*name;		/* e.g. "lpt", "wdc" etc */
	int			unit;
	int			resource_count;
	struct config_resource	*resources;
};

/*
 * Implementation of device.
 */
struct bsd_device {
	/*
	 * A device is a kernel object. The first field must be the
	 * current ops table for the object.
	 */
	KOBJ_FIELDS;

	/*
	 * Device hierarchy.
	 */
	TAILQ_ENTRY(bsd_device)	link;		/* list of devices in parent */
	TAILQ_ENTRY(bsd_device)	devlink;	/* global device list membership */
	device_t		parent;
	device_list_t		children;	/* list of subordinate devices */

	/*
	 * Details of this device.
	 */
	driver_t	*driver;
	devclass_t	devclass;	/* device class which we are in */
	int		unit;
	char*		nameunit;	/* name+unit e.g. foodev0 */
	char*		desc;		/* driver specific description */
	int		busy;		/* count of calls to device_busy() */
	device_state_t	state;
	uint32_t	devflags;	/* api level flags for device_get_flags() */
	u_short		flags;
#define DF_ENABLED	0x0001		/* device should be probed/attached */
#define DF_FIXEDCLASS	0x0002		/* devclass specified at create time */
#define DF_WILDCARD	0x0004		/* unit was originally wildcard */
#define DF_DESCMALLOCED	0x0008		/* description was malloced */
#define DF_QUIET	0x0010		/* don't print verbose attach message */
#define DF_DONENOMATCH	0x0020		/* don't execute DEVICE_NOMATCH again */
#define DF_EXTERNALSOFTC 0x0040		/* softc not allocated by us */
#define DF_ASYNCPROBE	0x0080		/* can be probed with its own thread */
	u_char		order;		/* order from device_add_child_ordered() */
	u_char		pad;
	void		*ivars;
	void		*softc;

	struct sysctl_ctx_list sysctl_ctx; /**< state for sysctl variables  */
	struct sysctl_oid *sysctl_tree;	/**< state for sysctl variables */
};

struct device_op_desc {
	unsigned int	offset;		/* offset in driver ops */
	struct method*	method;		/* internal method implementation */
	devop_t		deflt;		/* default implementation */
	const char*	name;		/* unique name (for registration) */
};

#endif	/* _KERNEL || _KERNEL_STRUCTURES */
#endif	/* !_SYS_BUS_PRIVATE_H_ */
