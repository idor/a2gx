/*
 * Copyright (C) 2013 Vlad Lazarenko <vlad@lazarenko.me>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include "a2gx.h"
#include "a2gx_pci.h"
#include "a2gx_net.h"
#include "a2gx_dma.h"
#include "a2gx_mac.h"
#include "a2gx_device.h"

#define A2GX_PCI_VENDOR_ID 0x1172
#define A2GX_PCI_DEVICE_ID 0x1986

static const struct pci_device_id device_ids[] =
{
    { PCI_DEVICE(A2GX_PCI_VENDOR_ID, A2GX_PCI_DEVICE_ID) },
    { 0 }
};

MODULE_DEVICE_TABLE(pci, device_ids);

static int map_io_bars(struct a2gx_dev *dev)
{
    struct pci_dev *pci_dev = dev->pci_dev;

    dev->bar0 = ioremap_nocache(pci_resource_start(pci_dev, 0),
                                pci_resource_len(pci_dev, 0));
    if (!dev->bar0)
        goto on_err;
    dev->bar2 = ioremap_nocache(pci_resource_start(pci_dev, 2),
                                pci_resource_len(pci_dev, 2));
    if (!dev->bar2)
        goto on_err;
    return 0;

  on_err:
    dev->bar0 = NULL;
    dev->bar2 = NULL;
    return -1;
}

static void unmap_io_bars(struct a2gx_dev *dev)
{
    if (dev->bar2)
        iounmap(dev->bar2);
    if (dev->bar0)
        iounmap(dev->bar0);
}

static int probe(struct pci_dev *pci_dev, const struct pci_device_id *pci_id)
{
    struct a2gx_dev *dev;
    int r;
    const char *err_msg = "";

    printk(A2GX_INFO "Device 0x%x has been found at bus %d dev %d func %d\n",
           pci_dev->device, pci_dev->bus->number, PCI_SLOT(pci_dev->devfn),
           PCI_FUNC(pci_dev->devfn));

    dev = a2gx_net_alloc(pci_dev);
    if (!dev) {
        r = -ENOMEM;
        err_msg = "Cannot allocate device";
        goto on_err;
    }
    pci_set_drvdata(pci_dev, dev);
    r = pci_enable_device(pci_dev);
    if (r) {
        err_msg = "Cannot enable PCI device";
        goto on_err;
    }
    pci_set_master(pci_dev);
    pci_try_set_mwi(pci_dev);

    r = map_io_bars(dev);
    if (r) {
        err_msg = "Cannot map I/O bars";
        goto on_err;
    }

    r = a2gx_mac_init(dev);
    /* TODO: Handle error! */

    r = a2gx_dma_init(dev);
    if (r) {
        err_msg = "Cannot initialize DMA";
        goto on_err;
    }

    printk(A2GX_INFO
           "Device 0x%x at bus %d dev %d func %d initialized.\n"
           "\t[bar0=%p, bar2=%p, dma_mask=0x%x]\n",
           pci_dev->device, pci_dev->bus->number, PCI_SLOT(pci_dev->devfn),
           PCI_FUNC(pci_dev->devfn),
           dev->bar0, dev->bar2, dev->dma_mask);

    return r;

  on_err:
    a2gx_net_free(dev);
    printk(A2GX_ERR "Device probe failed (%d): %s\n", r, err_msg);
    return r;
}

static void remove(struct pci_dev *pci_dev)
{
    struct a2gx_dev *dev;

    dev = pci_get_drvdata(pci_dev);
    pci_set_drvdata(pci_dev, NULL);
    if (!dev)
        goto pci_disable;
    unmap_io_bars(dev);
    a2gx_net_free(dev);
  pci_disable:
    pci_disable_device(pci_dev);
}

static struct pci_driver driver =
{
    .name = A2GX_DRIVER_NAME,
    .id_table = device_ids,
    .probe = probe,
    .remove = remove
};

int a2gx_pci_register(void)
{
    int r;

    r = pci_register_driver(&driver);
    if (r) {
        printk(A2GX_ERR "Failed to register PCI device (%d)\n", r);
        return -1;
    }
    return r;
}

void a2gx_pci_unregister(void)
{
    pci_unregister_driver(&driver);
}
