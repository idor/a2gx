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

#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h> /* TODO: Remove me. Don't print in this object... */
#include <a2gx/device.h>
#include "a2gx/eth_mac.h"

#define DMA_BUF_LEN 8196 /* TODO: Un-hard-code!!! */
#define A2GX_BAR2_SIZE 134217728 /* 128 MB */
#define A2GX_MMAP_REG_IO_BAR 0
#define A2GX_MMAP_REG_DMA (sysconf(_SC_PAGESIZE) * 2)

void a2gx_dev_init(struct a2gx_device *dev)
{
    dev->reg.mem = MAP_FAILED;
    dev->reg.len = 0;
    dev->reg_fd = -1;
    dev->rx_buf = MAP_FAILED;
}

int a2gx_dev_open(struct a2gx_device *dev, unsigned int number)
{
    int r;

    dev->reg_fd = open("/dev/a2gx_io", O_RDWR);
    if (dev->reg_fd == -1)
        goto on_err;

    dev->reg.mem = mmap(NULL, A2GX_BAR2_SIZE, PROT_READ | PROT_WRITE,
                        MAP_SHARED, dev->reg_fd, A2GX_MMAP_REG_IO_BAR);
    if (dev->reg.mem == MAP_FAILED)
        goto on_err;

    dev->rx_buf = mmap(NULL, DMA_BUF_LEN, PROT_READ | PROT_WRITE,
                       MAP_SHARED, dev->reg_fd, A2GX_MMAP_REG_DMA);
    if (dev->rx_buf == MAP_FAILED) {
        goto on_err;
    }
    dev->rx_buf_dma = *(unsigned long *)dev->rx_buf;
    a2gx_rmb();
    printf("RX DMA: 0x%lx\n", dev->rx_buf_dma);
    return 0;

  on_err:
    r = -errno;
    a2gx_dev_close(dev);
    return r;
}

void a2gx_dev_close(struct a2gx_device *dev)
{
    if (dev->rx_buf != MAP_FAILED) {
        munmap(dev->rx_buf, DMA_BUF_LEN);
        dev->rx_buf = MAP_FAILED;
    }
    if (dev->reg.mem != MAP_FAILED) {
        munmap(dev->reg.mem, A2GX_BAR2_SIZE);
        dev->reg.mem = MAP_FAILED;
    }
    if (dev->reg_fd != -1) {
        close(dev->reg_fd);
        dev->reg_fd = -1;
    }
}

int a2gx_dev_reset(struct a2gx_device *dev)
{
    int r;

    r = a2gx_eth_mac_reset(dev);
    if (r)
        goto on_err;

    return 0;

  on_err:
    return r;
}

/* TODO: This is just a test :-) */

#define A2GX_DMA_BASE 0x0
#define A2GX_A2P_BASE 0x1000

#define A2GX_DMA_A2P_BASE0_LO 0x00
#define A2GX_DMA_A2P_BASE0_HI 0x01
#define A2GX_DMA_A2P_BASE1_LO 0x02
#define A2GX_DMA_A2P_BASE1_HI 0x03

#define RX_CSR_BASE 0x04000000
#define RX_DESC_BASE 0x04000200
#define RX_STATUS_BASE 0x04000210

#define CSR_STATUS_REG 0x00
#define CSR_CONTROL_REG 0x01

#define CSR_CONTROL_RESET_MASK (1<<1)

#define CSR_STATUS_BUSY (1<<0)
#define CSR_STATUS_EMPTY (1<<3)

#define DESC_ADDR_SRC_REG 0x00
#define DESC_ADDR_DST_REG 0x01
#define DESC_LENGTH_REG 0x02
#define DESC_CONTROL_REG 0x03

#define DESC_CONTROL_SOP (1<<8)
#define DESC_CONTROL_EOP (1<<9)
#define DESC_CONTROL_END_ON_EOP (1<<12)
#define DESC_CONTROL_GO (1<<31)

size_t a2gx_dev_rx(struct a2gx_device *dev)
{
    uint32_t *a2p_base;
    uint32_t *rx_base;
    uint32_t *rx_desc;
    uint32_t *rx_status;
    unsigned int a2p_mask;
    uint32_t status;
    uint32_t resp_bytes;
    unsigned int i;
    struct a2gx_eth_mac_stats s;

    a2p_base = a2gx_io_base(dev->reg.mem, A2GX_A2P_BASE);
    rx_base = a2gx_io_base(dev->reg.mem, RX_CSR_BASE);
    rx_desc = a2gx_io_base(dev->reg.mem, RX_DESC_BASE);
    rx_status = a2gx_io_base(dev->reg.mem, RX_STATUS_BASE);

    /* Setup Altera's PCIe address translation table. */
    a2p_mask = a2p_base[A2GX_DMA_A2P_BASE0_LO];
    a2gx_rmb();
    printf("MASK BEFORE: 0x%x\n", a2p_mask);

    a2p_base[A2GX_DMA_A2P_BASE0_LO] = 0xFFFFFFFC;
    a2gx_wmb();

    a2p_mask = a2p_base[A2GX_DMA_A2P_BASE0_LO];
    a2gx_rmb();
    printf("MASK AFTER: 0x%x\n", a2p_mask);

    if (!a2p_mask) {
        fprintf(stderr, "Can't setup PCI ATT!\n");
        goto on_err;
    }

    a2p_base[A2GX_DMA_A2P_BASE0_LO] = (uint32_t)(dev->rx_buf_dma & a2p_mask);
    a2p_base[A2GX_DMA_A2P_BASE0_HI] = 0;
    a2gx_wmb();

    printf("DMA 0x%lx [mask=0x%x, low=0x%lx]\n",
           dev->rx_buf_dma, a2p_mask, dev->rx_buf_dma & a2p_mask);

    /* Reset DMA Controller */
    rx_base[CSR_CONTROL_REG] = CSR_CONTROL_RESET_MASK;
    a2gx_wmb();

    /* Wait until controller becomes free */
    do {
        status = rx_base[CSR_STATUS_REG];
        a2gx_rmb();
    } while (status & CSR_STATUS_BUSY /* Still busy */);

    rx_base[CSR_CONTROL_REG] = (1<<2) | (1<<3) | (1<<4);

    for (;;) {
        /* Prepare description of the transfer. */
        rx_desc[DESC_ADDR_SRC_REG] = 0;
        rx_desc[DESC_ADDR_DST_REG] = (uint32_t)(dev->rx_buf_dma & (~a2p_mask));
        rx_desc[DESC_LENGTH_REG] = DMA_BUF_LEN;
        a2gx_wmb();

        /* Ask device to read data and end as soon as MAC signals EOP. */
        rx_desc[DESC_CONTROL_REG] = DESC_CONTROL_END_ON_EOP | DESC_CONTROL_GO;
        a2gx_wmb();

        printf("READING...\n");

        /* Wait until transfer is complete... */
        do {
            status = rx_base[CSR_STATUS_REG];
            a2gx_rmb();
        } while (status & CSR_STATUS_BUSY /* Still busy */);

        a2gx_eth_mac_stats_read(&s, dev);
        printf("MAC RX=%u, CRC_ERR=%u, ALIGN_ERR=%u, IF_ERR=%u\n",
               s.rx_frames,
               s.rx_frames_crc_err,
               s.rx_frames_align_err,
               s.rx_if_errors);

        if (status & CSR_STATUS_EMPTY) {
            printf("RESPONSE EMPTY!\n");
            continue;
        }

        resp_bytes = rx_status[0];
        status = rx_status[1]; // X, X, early termination, error.
        a2gx_rmb();

        rx_base[CSR_STATUS_REG] = (1<<9); // CLEAR IRQ.

        // TODO: Clear IRQ? 9th bit of status should be set to 0.

        if (!resp_bytes) {
            continue;
        }

        printf("--- %u bytes ---", resp_bytes);
        for (i = 0; i < resp_bytes; ++i) {
            if (i % 16 == 0)
                printf("\n");
            printf("0x%.2X ", ((unsigned char *)dev->rx_buf)[i]);
        }
        printf("\n");
    }

    return 0;

  on_err:
    return 0;
}
