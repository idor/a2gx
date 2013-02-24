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

#define MAC_BASE 0x02004000

#define A2GX_BAR2_SIZE 134217728 /* 128 MB */
#define A2GX_MMAP_REG_IO_BAR 0
#define A2GX_BUF_CNT 25 /* TODO: Un-hard code.
                           The max in HW is 256 (desciptor FIFO). */

#define A2GX_A2P_BASE 0x1000 /* See Table 6.17 in Altera PCIe User Guide */

static int setup_a2p(struct a2gx_device *dev, struct a2gx_ring *ring,
                     unsigned int offset)
{
    uint32_t *a2p;
    unsigned int mask;

    a2p = a2gx_io_base(dev->reg.mem, (A2GX_A2P_BASE + offset * 0x08));

    a2p[0] = 0xFFFFFFFC;
    a2gx_wmb();

    mask = a2p[0];
    a2gx_rmb();

    if (!mask)
        goto on_err;

    a2p[0] = (uint32_t)(ring->dma & mask);
    a2p[1] = 0;
    a2gx_wmb();

    ring->a2p = mask;
    return 0;

  on_err:
    return -EIO;
}

void a2gx_dev_init(struct a2gx_device *dev)
{
    dev->reg.mem = MAP_FAILED;
    dev->reg.len = 0;
    a2gx_rx_init(&dev->rx);
    a2gx_tx_init(&dev->tx);
    a2gx_eth_mac_init(&dev->mac);
    dev->reg_fd = -1;
}

int a2gx_dev_open(struct a2gx_device *dev, unsigned int number)
{
    int r;

    if (number != 0)
        return -EINVAL;

    /* Knock on the kernel's door. */
    dev->reg_fd = open("/dev/a2gx_io", O_RDWR);
    if (dev->reg_fd == -1)
        goto on_err;

    /* Setup registers I/O */
    dev->reg.mem = mmap(NULL, A2GX_BAR2_SIZE, PROT_READ | PROT_WRITE,
                        MAP_SHARED, dev->reg_fd, A2GX_MMAP_REG_IO_BAR);
    if (dev->reg.mem == MAP_FAILED)
        goto on_err;

    dev->mac.base = a2gx_io_base(dev->reg.mem, MAC_BASE);
    a2gx_rx_setup(dev, &dev->rx);
    a2gx_tx_setup(dev, &dev->tx);

    /* Setup DMA buffers */
    if (a2gx_ring_alloc(dev, &dev->rx.ring, A2GX_BUF_CNT))
        goto on_err;
    if (a2gx_ring_alloc(dev, &dev->tx.ring, A2GX_BUF_CNT))
        goto on_err;

    /* Setup address translation table for our DMA buffers */
    if (setup_a2p(dev, &dev->rx.ring, 0))
        goto on_err;
    if (setup_a2p(dev, &dev->tx.ring, 1))
        goto on_err;

    return 0;

  on_err:
    r = -errno;
    a2gx_dev_close(dev);
    return r;
}

void a2gx_dev_close(struct a2gx_device *dev)
{
    a2gx_ring_free(dev, &dev->rx.ring);
    a2gx_ring_free(dev, &dev->tx.ring);
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

    r = a2gx_eth_mac_reset(&dev->mac);
    if (r)
        goto on_err;

    r = a2gx_rx_reset(&dev->rx);
    if (r)
        goto on_err;

    r = a2gx_tx_reset(&dev->tx);
    if (r)
        goto on_err;

    return 0;

  on_err:
    return r;
}
