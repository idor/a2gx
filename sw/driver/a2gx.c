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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/pfn.h>

#define A2GX_PCI_VENDOR_ID 0x1172
#define A2GX_PCI_DEVICE_ID 0x1986

#define A2GX_DRIVER_NAME "a2gx"

#define A2GX_INFO KERN_INFO A2GX_DRIVER_NAME ": "
#define A2GX_ERR KERN_ERR A2GX_DRIVER_NAME ": "

#define A2GX_DMA_BUF_MAX 2

MODULE_AUTHOR("Vlad Lazarenko");
MODULE_VERSION("0.0.1");
MODULE_DESCRIPTION("Arria II GX Driver");
MODULE_LICENSE("Dual BSD/GPL");

static int a2gx_cdev_major = 86;

module_param(a2gx_cdev_major, int, S_IRUGO);

struct a2gx_dma_buf {
    void *cpu_addr;
    size_t size;
    dma_addr_t dma_addr;
    void *priv_data;
};

struct a2gx_dev {
    void __iomem *bar0;
    void __iomem *bar2;
    struct cdev cdev;
    struct pci_dev *pci_dev;
    struct a2gx_dma_buf dma_buf[A2GX_DMA_BUF_MAX];
};

static const struct pci_device_id a2gx_pci_ids[] =
{
    { PCI_DEVICE(A2GX_PCI_VENDOR_ID, A2GX_PCI_DEVICE_ID) },
    { 0 }
};

MODULE_DEVICE_TABLE(pci, a2gx_pci_ids);

struct vm_operations_struct a2gx_bar_vma_ops = {
};

static int a2gx_cdev_open(struct inode *inode, struct file *filp)
{
    struct a2gx_dev *dev;

    dev = container_of(inode->i_cdev, struct a2gx_dev, cdev);
    filp->private_data = dev;
    return 0;
}

static int a2gx_cdev_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static int a2gx_cdev_mmap_bar2(struct file *filp, struct vm_area_struct *vma)
{
    struct a2gx_dev *dev;
    size_t size;

    size = vma->vm_end - vma->vm_start;
    if (size != 134217728)
        return -EIO;

    dev = filp->private_data;
    vma->vm_ops = &a2gx_bar_vma_ops;
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    vma->vm_private_data = dev;

    if (remap_pfn_range(vma, vma->vm_start,
                        vmalloc_to_pfn(dev->bar2),
                        size, vma->vm_page_prot))
    {
        return -EAGAIN;
    }

    return 0;
}

static void a2gx_dma_vma_close(struct vm_area_struct *vma)
{
    struct a2gx_dma_buf *buf;
    struct a2gx_dev *dev;

    buf = vma->vm_private_data;
    dev = buf->priv_data;

    pci_free_consistent(dev->pci_dev, buf->size, buf->cpu_addr, buf->dma_addr);
    buf->cpu_addr = NULL;
}

struct vm_operations_struct a2gx_dma_vma_ops = {
    .close = a2gx_dma_vma_close
};

static int a2gx_cdev_mmap_dma(struct file *filp, struct vm_area_struct *vma)
{
    struct a2gx_dev *dev;
    struct a2gx_dma_buf *buf;
    size_t size;
    unsigned int i;

    dev = filp->private_data;
    size = vma->vm_end - vma->vm_start;

    if (size < sizeof(unsigned long))
        return -EINVAL;

    for (i = 0; i < A2GX_DMA_BUF_MAX; ++i) {
        buf = &dev->dma_buf[i];
        if (buf->cpu_addr == NULL)
            break;
    }

    if (buf->cpu_addr != NULL)
        return -ENOBUFS;

    buf->cpu_addr = pci_alloc_consistent(dev->pci_dev, size, &buf->dma_addr);
    if (buf->cpu_addr == NULL)
        return -ENOMEM;

    memcpy(buf->cpu_addr, &buf->dma_addr, sizeof(buf->dma_addr));

    buf->size = size;
    buf->priv_data = dev;
    vma->vm_ops = &a2gx_dma_vma_ops;
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    vma->vm_private_data = buf;

    if (remap_pfn_range(vma, vma->vm_start,
                        vmalloc_to_pfn(buf->cpu_addr),
                        size, vma->vm_page_prot))
    {
        pci_free_consistent(dev->pci_dev, buf->size, buf->cpu_addr,
                            buf->dma_addr);
        buf->cpu_addr = NULL;
        return -EAGAIN;
    }

    return 0;
}

/*
 * Page offset has special meaning:
 *
 * 1) '0' means 32-bit I/O registers bar (bar #2, uncached).
 * 2) '1' means 64-bit I/O bar (bar #0, cached).
 * 3) '2' means coherent DMA buffer.
 */
static int a2gx_cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
    if (vma->vm_pgoff == 0) {
        return a2gx_cdev_mmap_bar2(filp, vma);
    } else if (vma->vm_pgoff == 2) {
        return a2gx_cdev_mmap_dma(filp, vma);
    } else {
        return -EINVAL;
    }
}

static struct file_operations a2gx_fs_ops = {
    .owner = THIS_MODULE,
    .open = &a2gx_cdev_open,
    .release = &a2gx_cdev_release,
    .mmap = &a2gx_cdev_mmap
};

static int a2gx_add_cdev(struct a2gx_dev *dev)
{
    int r;
    dev_t dev_sn;

    dev_sn = MKDEV(a2gx_cdev_major, 0);
    r = register_chrdev_region(dev_sn, 1, A2GX_DRIVER_NAME);
    if (r < 0) {
        printk(A2GX_ERR "Cannot register device number %d (%d).",
               a2gx_cdev_major, r);
        goto on_err;
    }

    cdev_init(&dev->cdev, &a2gx_fs_ops);
    dev->cdev.owner = THIS_MODULE;
    r = cdev_add(&dev->cdev, dev_sn, 1);

    if (r) {
        printk(A2GX_ERR "Cannot register character device (%d)", r);
        goto on_add_err;
    }

    return 0;

  on_add_err:
    unregister_chrdev_region(dev_sn, 1);
  on_err:
    return r;
}

static void a2gx_remove_cdev(struct a2gx_dev *dev)
{
    dev_t dev_sn;

    dev_sn = MKDEV(a2gx_cdev_major, 0);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev_sn, 1);
}

static int a2gx_pci_probe(struct pci_dev *pci_dev,
                          const struct pci_device_id *pci_id)
{
    struct a2gx_dev *dev;
    int r;
    const char *err_msg = "";

    dev = kzalloc(sizeof(struct a2gx_dev), GFP_KERNEL);
    if (unlikely(!dev)) {
        r = -ENOMEM;
        err_msg = "Cannot allocate device";
        goto on_err;
    }

    pci_set_drvdata(pci_dev, dev);
    dev->pci_dev = pci_dev;

    r = pci_enable_device(pci_dev);
    if (unlikely(r)) {
        err_msg = "Cannot enable PCI device";
        goto on_enable_err;
    }

    pci_set_master(pci_dev);
    pci_try_set_mwi(pci_dev);

    dev->bar0 = ioremap_nocache(pci_resource_start(pci_dev, 0),
                                pci_resource_len(pci_dev, 0));
    if (unlikely(!dev->bar0)) {
        err_msg = "Cannot map I/O bar #0";
        goto on_map_err;
    }

    dev->bar2 = ioremap_nocache(pci_resource_start(pci_dev, 2),
                                pci_resource_len(pci_dev, 2));
    if (unlikely(!dev->bar2)) {
        err_msg = "Cannot map I/O bar #2";
        goto on_map_err;
    }

    r = a2gx_add_cdev(dev);
    if (r)
        goto on_map_err;

    printk(A2GX_INFO "Found device 0x%x at bus %d dev %d func %d.\n",
           pci_dev->device, pci_dev->bus->number,
           PCI_SLOT(pci_dev->devfn), PCI_FUNC(pci_dev->devfn));

    return r;

  on_map_err:
    if (dev->bar2)
        iounmap(dev->bar2);
    if (dev->bar0)
        iounmap(dev->bar0);
    pci_disable_device(pci_dev);

  on_enable_err:
    kfree(dev);

  on_err:
    printk(A2GX_ERR "Device probe failed (%d): %s\n", r, err_msg);
    return r;
}

static void a2gx_pci_remove(struct pci_dev *pci_dev)
{
    struct a2gx_dev *dev;

    dev = pci_get_drvdata(pci_dev);
    pci_set_drvdata(pci_dev, NULL);
    if (unlikely(!dev))
        goto pci_disable;
    a2gx_remove_cdev(dev);
    if (dev->bar2)
        iounmap(dev->bar2);
    if (dev->bar0)
        iounmap(dev->bar0);

    /* TODO: Just in case, be paranoid and free DMA buffers?
       In real life, none should be left allocated as VMA
       should call 'close()' for the user process. */

    kfree(dev);

  pci_disable:
    printk(A2GX_INFO "Removed device 0x%x at bus %d dev %d func %d.\n",
           pci_dev->device, pci_dev->bus->number,
           PCI_SLOT(pci_dev->devfn), PCI_FUNC(pci_dev->devfn));
    pci_disable_device(pci_dev);
}

static struct pci_driver a2gx_pci_drv =
{
    .name = A2GX_DRIVER_NAME,
    .id_table = a2gx_pci_ids,
    .probe = a2gx_pci_probe,
    .remove = a2gx_pci_remove
};

static int __init a2gx_init(void)
{
    int r;

    r = pci_register_driver(&a2gx_pci_drv);
    if (r) {
        printk(A2GX_ERR "Failed to register PCI driver (%d)\n", r);
        return -1;
    }
    return r;
}

static void __exit a2gx_exit(void)
{
    pci_unregister_driver(&a2gx_pci_drv);
}

module_init(a2gx_init);
module_exit(a2gx_exit);
