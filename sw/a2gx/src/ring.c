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
#include <stddef.h>
#include <unistd.h>
#include "a2gx/eth_mac.h"
#include "a2gx/ring.h"
#include "a2gx/device.h"

#define BUF_SIZE 1536
#define MMAP_DMA_PGOFF (sysconf(_SC_PAGESIZE) * 2)

#define RX_CSR_BASE  0x04000000
#define RX_DESC_OFF 0x200
#define RX_STATUS_OFF 0x210

#define TX_CSR_BASE 0x03000000
#define TX_DESC_OFF 0x020

#define CSR_STATUS_REG 0x00
#define CSR_CONTROL_REG 0x01
#define CSR_FILL_REG 0x02

#define CSR_CONTROL_RESET_MASK (1u<<1)

#define CSR_STATUS_BUSY       (1u<<0) /* DMA controller in progress */
#define CSR_STATUS_DESC_EMPTY (1u<<1) /* Descriptors FIFO empty */
#define CSR_STATUS_DESC_FULL  (1u<<2) /* Descriptors FIFO full */
#define CSR_STATUS_RESP_EMPTY (1u<<3) /* Response FIFO empty */
#define CSR_STATUS_RESP_FULL  (1u<<4) /* Response FIFO full */

#define DESC_ADDR_SRC_REG 0x00
#define DESC_ADDR_DST_REG 0x01
#define DESC_LENGTH_REG 0x02
#define DESC_CONTROL_REG 0x03

#define DESC_CONTROL_SOP (1u<<8)
#define DESC_CONTROL_EOP (1u<<9)
#define DESC_CONTROL_END_ON_EOP (1u<<12)
#define DESC_CONTROL_GO (1u<<31)

void a2gx_ring_init(struct a2gx_ring *ring)
{
    ring->base = MAP_FAILED;
    ring->size = 0;
    ring->dma = 0;
}

int a2gx_ring_alloc(struct a2gx_device *dev, struct a2gx_ring *ring,
                    unsigned int count)
{
    unsigned int size;

    size = BUF_SIZE * count;
    ring->base = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
                      dev->reg_fd, MMAP_DMA_PGOFF);
    if (ring->base == MAP_FAILED)
        return -ENOMEM;
    ring->size = size;
    ring->dma = *(unsigned long *)ring->base;
    a2gx_rmb();

    return 0;
}

void a2gx_ring_free(struct a2gx_device *dev, struct a2gx_ring *ring)
{
    (void)dev;
    if (ring->base != MAP_FAILED) {
        munmap(ring->base, ring->size);
        ring->base = MAP_FAILED;
    }
}

void a2gx_rx_init(struct a2gx_rx *rx)
{
    rx->base = NULL;
    rx->next_to_dev = 0;
    rx->next_from_dev = 0;
    a2gx_ring_init(&rx->ring);
}

void a2gx_tx_init(struct a2gx_tx *tx)
{
    tx->base = NULL;
    tx->next_to_dev = 0;
    a2gx_ring_init(&tx->ring);
}

void a2gx_rx_setup(struct a2gx_device *dev, struct a2gx_rx *rx)
{
    rx->base = a2gx_io_base(dev->reg.mem, RX_CSR_BASE);
}

void a2gx_tx_setup(struct a2gx_device *dev, struct a2gx_tx *tx)
{
    tx->base = a2gx_io_base(dev->reg.mem, TX_CSR_BASE);
}

static int dma_ring_reset(uint32_t *csr)
{
    uint32_t status;
    unsigned int i;

    csr[CSR_CONTROL_REG] = CSR_CONTROL_RESET_MASK;
    a2gx_wmb();

    for (i = 0; i < 50000; ++i) {
        status = csr[CSR_STATUS_REG];
        a2gx_rmb();
        if (!(status & CSR_STATUS_BUSY))
            break;
    }
    if (status & CSR_STATUS_BUSY)
        return -EAGAIN;
    return 0;
}

int a2gx_rx_reset(struct a2gx_rx *rx)
{
    return dma_ring_reset(rx->base);
}

int a2gx_tx_reset(struct a2gx_tx *tx)
{
    return dma_ring_reset(tx->base);
}

int a2gx_rx_request(struct a2gx_rx *rx)
{
    uint32_t *base;
    uint32_t status;

    base = rx->base;
    status = base[CSR_STATUS_REG];
    a2gx_rmb();

    if (status & CSR_STATUS_DESC_FULL) {
        return -EAGAIN;
    }

    if (rx->next_to_dev == rx->next_from_dev &&
        ((status & CSR_STATUS_BUSY) || !(status & CSR_STATUS_DESC_EMPTY)))
    {
        return -ENOBUFS;
    }

    base = a2gx_io_base(base, RX_DESC_OFF);
    base[DESC_ADDR_SRC_REG] = 0;
    base[DESC_ADDR_DST_REG] = (uint32_t)
        ((rx->ring.dma + rx->next_to_dev * BUF_SIZE) & (~rx->ring.a2p));
    base[DESC_LENGTH_REG] = BUF_SIZE;
    a2gx_wmb();

    base[DESC_CONTROL_REG] = DESC_CONTROL_END_ON_EOP | DESC_CONTROL_GO;
    a2gx_wmb();

    rx->next_to_dev = ((rx->next_to_dev + 1) %
                       ((unsigned int)(rx->ring.size / BUF_SIZE)));

    status = ((uint32_t *)rx->base)[CSR_STATUS_REG];
    a2gx_rmb();

    return 0;
}

int a2gx_rx_read(struct a2gx_rx *rx, unsigned char **buf, unsigned int *bytes)
{
    uint32_t *base;
    uint32_t status;
    uint32_t length;
    unsigned int next;

    base = rx->base;
    status = base[CSR_STATUS_REG];
    a2gx_rmb();

    if (status & CSR_STATUS_RESP_EMPTY)
        return -EAGAIN;

    base = a2gx_io_base(base, RX_STATUS_OFF);

    length = base[0];
    status = base[1]; // <reserved>|<reserved>|<early termination>|<error>
    a2gx_rmb();

    next = rx->next_from_dev;
    *buf = rx->ring.base + (next * BUF_SIZE);
    *bytes = length;

    rx->next_from_dev = ((next + 1) % ((unsigned int)
                                       (rx->ring.size / BUF_SIZE)));

    if (length > BUF_SIZE)
        return -EIO;

    return 0;
}

int a2gx_tx_request(struct a2gx_tx *tx, unsigned char **buf, uint32_t bytes)
{
    uint32_t *csr;
    union {
        uint32_t data;
        struct {
            uint16_t write;
            uint16_t read;
        } parts;
    } fill_level;

    if (!bytes)
        return -EINVAL;
    if (bytes > BUF_SIZE)
        return -ENOBUFS;

    csr = tx->base;
    fill_level.data = csr[CSR_FILL_REG];
    a2gx_rmb();

    if (fill_level.parts.write >= (uint8_t)(tx->ring.size / BUF_SIZE))
        return -EAGAIN;

    *buf = tx->ring.base + (tx->next_to_dev * BUF_SIZE);
    tx->next_to_dev = ((tx->next_to_dev + 1) %
                       ((unsigned int)tx->ring.size / BUF_SIZE));

    return 0;
}

int a2gx_tx_write(struct a2gx_tx *tx, unsigned char *buf, uint32_t bytes)
{
    uint32_t *base;
    uint32_t dma_addr;

    if (!bytes || bytes > BUF_SIZE)
        return -EINVAL;

    dma_addr = (uint32_t)
        ((tx->ring.dma + (uint32_t)(buf - (unsigned char *)tx->ring.base)) &
         (~tx->ring.a2p));

    base = a2gx_io_base(tx->base, TX_DESC_OFF);
    base[DESC_ADDR_SRC_REG] = dma_addr;
    base[DESC_ADDR_DST_REG] = 0;
    base[DESC_LENGTH_REG] = bytes;
    a2gx_wmb();

    base[DESC_CONTROL_REG] = (DESC_CONTROL_SOP | DESC_CONTROL_EOP |
                              DESC_CONTROL_GO);
    a2gx_wmb();
    return 0;
}
