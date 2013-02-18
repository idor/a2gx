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

#include <linux/pci.h>
#include "a2gx.h"
#include "a2gx_device.h"
#include "a2gx_dma.h"

#define A2GX_DMA_BASE 0x0
#define A2GX_DMA_A2P_BASE0_LO 0x1000
#define A2GX_DMA_A2P_BASE0_HI 0x1004
#define A2GX_DMA_A2P_BASE1_LO 0x1008
#define A2GX_DMA_A2P_BASE1_HI 0x100C

#define RX_CSR_BASE 0x04000000
#define RX_DESC_BASE (RX_CSR_BASE + 0x20)

#define TX_CSR_BASE 0x03000000
#define TX_DESC_BASE (TX_CSR_BASE + 0x20)

#define CSR_STATUS_REG 0x00
#define CSR_CONTROL_REG 0x01

#define CSR_CONTROL_RESET_MASK (1<<1)

#define DESC_ADDR_SRC_REG 0x00
#define DESC_ADDR_DST_REG 0x01
#define DESC_LENGTH_REG 0x02
#define DESC_CONTROL_REG 0x03

#define DESC_CONTROL_SOP (1<<8)
#define DESC_CONTROL_EOP (1<<9)
#define DESC_CONTROL_END_ON_EOP (1<<12)
#define DESC_CONTROL_GO (1<<31)

/*
#define A2GX_DMA_CSR_STATUS_BUSY (1<<0)
#define A2GX_DMA_CSR_STATUS_EMPTY (1<<1)
#define A2GX_DMA_CSR_STATUS_FULL (1<<2)
#define A2GX_DMA_CSR_STATUS_RESP_EMPTY (1<<3)
#define A2GX_DMA_CSR_STATUS_RESP_FULL (1<<4)
#define A2GX_DMA_CSR_STATUS_STOP (1<<5)
#define A2GX_DMA_CSR_STATUS_RESET (1<<6)
#define A2GX_DMA_CSR_STATUS_STOP_ON_ERR (1<<7)
#define A2GX_DMA_CSR_STATUS_STOP_ON_TERM (1<<8)
#define A2GX_DMA_CSR_STATUS_IRQ (1<<9)
*/

#define A2GX_MAX_BUF_SIZE 1048576 /* 1 MB */

static void csr_issue_reset(u32 __iomem *csr)
{
    iowrite32(CSR_CONTROL_RESET_MASK, (csr + CSR_CONTROL_REG));
    wmb();
}

static void desc_queue_write(u32 __iomem *desc, u32 address, u32 length,
                             u32 control)
{
    printk(A2GX_INFO "WRITE %u to %u (%u)... \n", length, address, control);
    iowrite32(address, desc + DESC_ADDR_SRC_REG);
    iowrite32(0, desc + DESC_ADDR_DST_REG);
    iowrite32(length, desc + DESC_LENGTH_REG);
    wmb();
    iowrite32(control, desc + DESC_CONTROL_REG);
    wmb();
}

static void desc_queue_read(u32 __iomem *desc, u32 address, u32 length,
                            u32 control)
{
    iowrite32(0, desc + DESC_ADDR_SRC_REG);
    iowrite32(address, desc + DESC_ADDR_DST_REG);
    iowrite32(length, desc + DESC_LENGTH_REG);
    wmb();
    iowrite32(control, desc + DESC_CONTROL_REG);
    wmb();
}

int a2gx_dma_init(struct a2gx_dev *dev)
{
    void __iomem *base = (dev->bar + A2GX_DMA_BASE);

    /* TODO : handle errors... */
    if (pci_set_dma_mask(dev->pci_dev, DMA_BIT_MASK(24)) == 0) {
        if (pci_set_consistent_dma_mask(dev->pci_dev, DMA_BIT_MASK(24)) != 0) {
            printk(A2GX_ERR "Cannot use coherent DMA?\n");
        }
    } else {
        printk(A2GX_ERR "Cannot use DMA?\n");
    }

    a2gx_dma_reset(dev);

    dev->ring_buf = pci_alloc_consistent(
        dev->pci_dev, 1024, &dev->ring_buf_dma);
    if (!dev->ring_buf) {
        printk(A2GX_ERR "Cannot allocate ring buffers.\n");
        return -ENOMEM;
    }

    /*
     * Setup address translation map.
     */
    iowrite32(0xFFFFFFFC, base + A2GX_DMA_A2P_BASE0_LO);
    wmb();
    dev->a2p_mask = ioread32(base + A2GX_DMA_A2P_BASE0_LO);
    rmb();
    if (!dev->a2p_mask)
        goto on_err;
	iowrite32(dev->ring_buf_dma & dev->a2p_mask, base + A2GX_DMA_A2P_BASE0_LO);
    iowrite32(0x0, base + A2GX_DMA_A2P_BASE0_HI);

    printk("DMA MASK = 0x%x (24=0x%x), DMA= 0x%x BASE = 0x%x\n",
           dev->a2p_mask, DMA_BIT_MASK(24), dev->ring_buf_dma,
           dev->ring_buf_dma & (~dev->a2p_mask));

	//iowrite32(dev->ring_buf_dma, base + A2GX_DMA_A2P_BASE0_LO);
    //iowrite32(0x0, base + A2GX_DMA_A2P_BASE0_HI);
    //wmb();

    return 0;
  on_err:
    return -1;
}

void a2gx_dma_fini(struct a2gx_dev *dev)
{
    a2gx_dma_reset(dev);
    if (dev->ring_buf) {
        pci_free_consistent(dev->pci_dev, 1024,
                            dev->ring_buf, dev->ring_buf_dma);
        dev->ring_buf = NULL;
    }
}

void a2gx_dma_reset(struct a2gx_dev *dev)
{
    void __iomem *bar = dev->bar;
    csr_issue_reset(bar + TX_CSR_BASE);
    csr_issue_reset(bar + RX_CSR_BASE);
}

struct my_ether_header {
	unsigned char ether_dhost[6];
	unsigned char ether_shost[6];
	u16 ether_type;
};

static u32 prep_pkt(unsigned char *p)
{
    // 6 sender mac.
    p[0] = 0xf4;
    p[1] = 0x6d;
    p[2] = 0x04;
    p[3] = 0x05;
    p[4] = 0xe3;
    p[5] = 0x88;
    // 6 target mac
    p[6] = 0x00;
    p[7] = 0x07;
    p[8] = 0xed;
    p[9] = 0xff;
    p[10] = 0xdc;
    p[11] = 0xb9;
    // 2 eth type.
    p[12] = 0x01;
    p[13] = 0x02;
    // Payload...
    p[14] = 'a';
    p[15] = 'b';
    p[16] = 'c';
	return 17;
}

void a2gx_dma_test(struct a2gx_dev *dev)
{
    u32 v;
    u32 c;
    unsigned int i;
    prep_pkt(dev->ring_buf);

    v = ioread32(dev->bar + TX_CSR_BASE);

    desc_queue_write(dev->bar + TX_DESC_BASE,
                     dev->ring_buf_dma, // & (~dev->a2p_mask),
                     60,
                     DESC_CONTROL_END_ON_EOP | DESC_CONTROL_SOP |
                     DESC_CONTROL_EOP | DESC_CONTROL_GO);
    v = ioread32(dev->bar + TX_CSR_BASE);
    c = ioread32(dev->bar + TX_CSR_BASE + 4);
    rmb();
    printk(A2GX_INFO "DMA: WRITE 0x%x / 0x%x\n", v, c);

/*
    desc_queue_read(dev->bar + RX_DESC_BASE,
                    (dev->ring_buf_dma + 60) & (~dev->a2p_mask),
                    256, DESC_CONTROL_END_ON_EOP | DESC_CONTROL_GO);

    v = ioread32(dev->bar + RX_CSR_BASE);
    c = ioread32(dev->bar + RX_CSR_BASE + 4);
    rmb();
    printk(A2GX_INFO "DMA: READ 0x%x / 0x%x (0x%x:0x%x:0x%x)\n", v, c,
           *(dev->ring_buf + 60),
           *(dev->ring_buf + 61),
           *(dev->ring_buf + 62));
*/

/*
    void __iomem *csr = dev->bar + A2GX_DMA_W_BASE;
    void __iomem *desc = dev->bar + A2GX_DMA_W_DESC_BASE;
    u32 s;
    unsigned int n, i;

    printk(A2GX_INFO "Testing DMA writes (status=0x%x)...\n", read_status(csr));

    if (read_status(csr) & A2GX_DMA_CSR_STATUS_FULL) {
        printk(A2GX_INFO "DMA buffer is full! Exit.\n");
    } else {
        printk(A2GX_INFO "DMA buffer is not full... enqueuing...\n");
    }


    for (n = 0; n < 50000; ++n) {
        iowrite32(__pa(dev->ring_buf) & (~dev->a2p_mask), desc); // from
        wmb();
        iowrite32(0, desc + 4); // to
        wmb();
        iowrite32(512, desc + 8); // size
        wmb();
        iowrite32((1<<31), desc + 0xC); // GO!
        wmb();
        if (read_status(csr) != 0xa) {
            printk(A2GX_INFO "YES!!! 0x%x...\n", read_status(csr));
            break;
        }
    }

    printk(A2GX_INFO "DMA test done. (status=0x%x)\n", read_status(csr));
*/
}
