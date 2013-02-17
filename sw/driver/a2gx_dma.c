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

#include "a2gx_device.h"
#include "a2gx_dma.h"

int a2gx_dma_init(struct a2gx_dev *dev)
{
    void __iomem *mem = dev->bar2 + A2GX_DMA_BASE + A2GX_DMA_A2P_REG;

    iowrite32(0xFFFFFFFC, mem);
    wmb();
    dev->dma_mask = ioread32(mem);
    rmb();
    if (!dev->dma_mask)
        goto on_err;
    a2gx_dma_reset(dev);
    return 0;
  on_err:
    return -1;
}

void a2gx_dma_reset_reader(struct a2gx_dev *dev)
{
    u32 *base = dev->bar2 + A2GX_DMA_R_BASE;
    iowrite32(A2GX_DMA_CSR_RESET_MASK, base + A2GX_DMA_CSR_REG);
    wmb();
}

void a2gx_dma_reset_writer(struct a2gx_dev *dev)
{
    u32 *base = dev->bar2 + A2GX_DMA_W_BASE;
    iowrite32(A2GX_DMA_CSR_RESET_MASK, (base + A2GX_DMA_CSR_REG));
    wmb();
}

void a2gx_dma_reset(struct a2gx_dev *dev)
{
    a2gx_dma_reset_reader(dev);
    a2gx_dma_reset_writer(dev);
}