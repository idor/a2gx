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

#ifndef A2GX_RING_H
#define A2GX_RING_H

#include <stdint.h>

struct a2gx_device;

struct a2gx_ring {
    unsigned char *base; /* DMA buffer base virtual address. */
    size_t size;         /* Total size of DMA buffer in bytes. */
    unsigned long dma;   /* DMA buffer base BUS address. */
    unsigned int a2p;    /* Address translation mask. */
};

struct a2gx_rx {
    void *base;                   /* DMA controller base address. */
    unsigned int next_to_dev;
    unsigned int next_from_dev;
    struct a2gx_ring ring;        /* I/O buffer ring. */
};

struct a2gx_tx {
    void *base;
    unsigned int next_to_dev;
    struct a2gx_ring ring;        /* I/O buffer ring. */
};

void a2gx_ring_init(struct a2gx_ring *ring);

int a2gx_ring_alloc(struct a2gx_device *dev, struct a2gx_ring *ring,
                    unsigned int count);

void a2gx_ring_free(struct a2gx_device *dev, struct a2gx_ring *ring);

void a2gx_rx_init(struct a2gx_rx *rx);

void a2gx_rx_setup(struct a2gx_device *dev, struct a2gx_rx *rx);

int a2gx_rx_reset(struct a2gx_rx *rx);

int a2gx_rx_request(struct a2gx_rx *rx);

int a2gx_rx_read(struct a2gx_rx *rx, unsigned char **buf, unsigned int *bytes);

void a2gx_tx_init(struct a2gx_tx *tx);

void a2gx_tx_setup(struct a2gx_device *dev, struct a2gx_tx *tx);

int a2gx_tx_reset(struct a2gx_tx *tx);

int a2gx_tx_request(struct a2gx_tx *tx, unsigned char **buf, uint32_t bytes);

int a2gx_tx_write(struct a2gx_tx *tx, unsigned char *buf, uint32_t bytes);

#endif
