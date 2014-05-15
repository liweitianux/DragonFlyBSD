/*	$FreeBSD: src/sys/dev/ral/if_ral_pci.c,v 1.10 2012/05/10 17:41:16 bschmidt Exp $	*/

/*-
 * Copyright (c) 2005, 2006
 *	Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */


/*
 * PCI/Cardbus front-end for the Ralink RT2560/RT2561/RT2561S/RT2661 driver.
 */

#include <sys/param.h>
#include <sys/sysctl.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/rman.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <netproto/802_11/ieee80211_var.h>
#include <netproto/802_11/ieee80211_radiotap.h>
#include <netproto/802_11/ieee80211_amrr.h>

#include "pcidevs.h"
#include <bus/pci/pcireg.h>
#include <bus/pci/pcivar.h>

#include <dev/netif/ral/rt2560var.h>
#include <dev/netif/ral/rt2661var.h>
#include <dev/netif/ral/rt2860var.h>

MODULE_DEPEND(ral, pci, 1, 1, 1);
MODULE_DEPEND(ral, firmware, 1, 1, 1);
MODULE_DEPEND(ral, wlan, 1, 1, 1);
MODULE_DEPEND(ral, wlan_amrr, 1, 1, 1);

struct ral_pci_ident {
	uint16_t	vendor;
	uint16_t	device;
	const char	*name;
};

static const struct ral_pci_ident ral_pci_ids[] = {
	{ PCI_VENDOR_AWT, PCI_PRODUCT_AWT_RT2890,
		"Ralink Technology RT2890" },
	{ PCI_VENDOR_EDIMAX, PCI_PRODUCT_EDIMAX_RT2860_1,
		"Ralink Technology RT2860" },
	{ PCI_VENDOR_EDIMAX, PCI_PRODUCT_EDIMAX_RT2860_2,
		"Ralink Technology RT2860" },
	{ PCI_VENDOR_EDIMAX, PCI_PRODUCT_EDIMAX_RT2860_3,
		"Ralink Technology RT2860" },
	{ PCI_VENDOR_EDIMAX, PCI_PRODUCT_EDIMAX_RT2860_4,
		"Ralink Technology RT2860" },
	{ PCI_VENDOR_EDIMAX, PCI_PRODUCT_EDIMAX_RT2860_5,
		"Ralink Technology RT2860" },
	{ PCI_VENDOR_EDIMAX, PCI_PRODUCT_EDIMAX_RT2860_6,
		"Ralink Technology RT2860" },
	{ PCI_VENDOR_EDIMAX, PCI_PRODUCT_EDIMAX_RT2860_7,
		"Ralink Technology RT2860" },
	{ PCI_VENDOR_EDIMAX, PCI_PRODUCT_EDIMAX_RT3591_1,
		"Ralink Technology RT3591" },
	{ PCI_VENDOR_EDIMAX, PCI_PRODUCT_EDIMAX_RT3591_2,
		"Ralink Technology RT3591" },
	{ PCI_VENDOR_MSI, PCI_PRODUCT_MSI_RT3090,
		"Ralink Technology RT3090" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT2560,
		"Ralink Technology RT2560" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT2561S,
		"Ralink Technology RT2561S" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT2561,
		"Ralink Technology RT2561" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT2661,
		"Ralink Technology RT2661" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT2760,
		"Ralink Technology RT2760" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT2790,
		"Ralink Technology RT2790" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT2860,
		"Ralink Technology RT2860" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT2890,
		"Ralink Technology RT2890" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT3060,
		"Ralink Technology RT3060" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT3062,
		"Ralink Technology RT3062" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT3090,
		"Ralink Technology RT3090" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT3091,
		"Ralink Technology RT3091" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT3092,
		"Ralink Technology RT3092" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT3390,
		"Ralink Technology RT3390" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT3562,
		"Ralink Technology RT3562" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT3592,
		"Ralink Technology RT3592" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT3593,
		"Ralink Technology RT3593" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT5390_1,
		"Ralink Technology RT5390" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT5390_2,
		"Ralink Technology RT5390" },
	{ PCI_VENDOR_RALINK, PCI_PRODUCT_RALINK_RT5390_3,
		"Ralink Technology RT5390" },

	{ 0, 0, NULL }
};

static struct ral_opns {
	int	(*attach)(device_t, int);
	int	(*detach)(void *);
	void	(*shutdown)(void *);
	void	(*suspend)(void *);
	void	(*resume)(void *);
	void	(*intr)(void *);

}  ral_rt2560_opns = {
	rt2560_attach,
	rt2560_detach,
	rt2560_stop,
	rt2560_stop,
	rt2560_resume,
	rt2560_intr

}, ral_rt2661_opns = {
	rt2661_attach,
	rt2661_detach,
	rt2661_shutdown,
	rt2661_suspend,
	rt2661_resume,
	rt2661_intr
}, ral_rt2860_opns = {
	rt2860_attach,
	rt2860_detach,
	rt2860_shutdown,
	rt2860_suspend,
	rt2860_resume,
	rt2860_intr
};

struct ral_pci_softc {
	union {
		struct rt2560_softc sc_rt2560;
		struct rt2661_softc sc_rt2661;
		struct rt2860_softc sc_rt2860;
	} u;

	struct ral_opns		*sc_opns;
	int			irq_rid;
	int			mem_rid;
	struct resource		*irq;
	struct resource		*mem;
	void			*sc_ih;
};

static int ral_pci_probe(device_t);
static int ral_pci_attach(device_t);
static int ral_pci_detach(device_t);
static int ral_pci_shutdown(device_t);
static int ral_pci_suspend(device_t);
static int ral_pci_resume(device_t);

static device_method_t ral_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ral_pci_probe),
	DEVMETHOD(device_attach,	ral_pci_attach),
	DEVMETHOD(device_detach,	ral_pci_detach),
	DEVMETHOD(device_shutdown,	ral_pci_shutdown),
	DEVMETHOD(device_suspend,	ral_pci_suspend),
	DEVMETHOD(device_resume,	ral_pci_resume),

	DEVMETHOD_END
};

static driver_t ral_pci_driver = {
	"ral",
	ral_pci_methods,
	sizeof (struct ral_pci_softc)
};

static devclass_t ral_devclass;

DRIVER_MODULE(ral, pci, ral_pci_driver, ral_devclass, NULL, NULL);

static int
ral_pci_probe(device_t dev)
{
	const struct ral_pci_ident *ident;

	wlan_serialize_enter();

	for (ident = ral_pci_ids; ident->name != NULL; ident++) {
		if (pci_get_vendor(dev) == ident->vendor &&
		    pci_get_device(dev) == ident->device) {
			device_set_desc(dev, ident->name);
			wlan_serialize_exit();
			return 0;
		}
	}
	wlan_serialize_exit();
	return ENXIO;
}

/* Base Address Register */
#define RAL_PCI_BAR0	0x10

static int
ral_pci_attach(device_t dev)
{
	struct ral_pci_softc *psc = device_get_softc(dev);
	struct rt2560_softc *sc = &psc->u.sc_rt2560;
	int error;

	wlan_serialize_enter();
	if (pci_get_powerstate(dev) != PCI_POWERSTATE_D0) {
		device_printf(dev, "chip is in D%d power mode "
		    "-- setting to D0\n", pci_get_powerstate(dev));
		pci_set_powerstate(dev, PCI_POWERSTATE_D0);
	}

	/* enable bus-mastering */
	pci_enable_busmaster(dev);

	switch (pci_get_device(dev)) {
	case PCI_PRODUCT_RALINK_RT2560:
		psc->sc_opns = &ral_rt2560_opns;
		break;
	case PCI_PRODUCT_RALINK_RT2561S:
	case PCI_PRODUCT_RALINK_RT2561:
	case PCI_PRODUCT_RALINK_RT2661:
		psc->sc_opns = &ral_rt2661_opns;
		break;
	default:
		psc->sc_opns = &ral_rt2860_opns;
		break;
	}

	psc->mem_rid = RAL_PCI_BAR0;
	psc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &psc->mem_rid,
	    RF_ACTIVE);
	if (psc->mem == NULL) {
		device_printf(dev, "could not allocate memory resource\n");
		wlan_serialize_exit();
		return ENXIO;
	}

	sc->sc_st = rman_get_bustag(psc->mem);
	sc->sc_sh = rman_get_bushandle(psc->mem);
	sc->sc_invalid = 1;
	
	psc->irq_rid = 0;
	psc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &psc->irq_rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (psc->irq == NULL) {
		device_printf(dev, "could not allocate interrupt resource\n");
		wlan_serialize_exit();
		return ENXIO;
	}

	error = (*psc->sc_opns->attach)(dev, pci_get_device(dev));
	if (error != 0) {
		wlan_serialize_exit();
		return error;
	}

	/*
	 * Hook our interrupt after all initialization is complete.
	 */
	error = bus_setup_intr(dev, psc->irq, INTR_MPSAFE,
	    psc->sc_opns->intr, psc, &psc->sc_ih, &wlan_global_serializer);
	if (error != 0) {
		device_printf(dev, "could not set up interrupt\n");
		wlan_serialize_exit();
		return error;
	}
	sc->sc_invalid = 0;
	
	wlan_serialize_exit();
	return 0;
}

static int
ral_pci_detach(device_t dev)
{
	struct ral_pci_softc *psc = device_get_softc(dev);
	struct rt2560_softc *sc = &psc->u.sc_rt2560;
	
	wlan_serialize_enter();
	/* check if device was removed */
	sc->sc_invalid = !bus_child_present(dev);
	
	(*psc->sc_opns->detach)(psc);

	bus_generic_detach(dev);
	bus_teardown_intr(dev, psc->irq, psc->sc_ih);
	bus_release_resource(dev, SYS_RES_IRQ, psc->irq_rid, psc->irq);

	bus_release_resource(dev, SYS_RES_MEMORY, psc->mem_rid, psc->mem);

	wlan_serialize_exit();
	return 0;
}

static int
ral_pci_shutdown(device_t dev)
{
	struct ral_pci_softc *psc = device_get_softc(dev);

	wlan_serialize_enter();
	(*psc->sc_opns->shutdown)(psc);
	wlan_serialize_exit();

	return 0;
}

static int
ral_pci_suspend(device_t dev)
{
	struct ral_pci_softc *psc = device_get_softc(dev);

	wlan_serialize_enter();
	(*psc->sc_opns->suspend)(psc);
	wlan_serialize_exit();

	return 0;
}

static int
ral_pci_resume(device_t dev)
{
	struct ral_pci_softc *psc = device_get_softc(dev);

	wlan_serialize_enter();
	(*psc->sc_opns->resume)(psc);
	wlan_serialize_exit();

	return 0;
}
