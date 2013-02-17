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

#include "a2gx.h"
#include "a2gx_device.h"
#include "a2gx_mac.h"

/*
 * MAC command configuration. See Table 6–2 (Register Map)
 * of Triple-Speed Ethernet User Guide for details.
 */
#define MAC_CMD_TX_ENA (1<<0)
#define MAC_CMD_RX_ENA (1<<1)
#define MAC_CMD_XON_GEN (1<<2)
#define MAC_CMD_ETH_SPEED (1<<3)
#define MAC_CMD_PROMIS_EN (1<<4)
#define MAC_CMD_PAD_EN (1<<5)
#define MAC_CMD_CRC_FWD (1<<6)
#define MAC_CMD_PAUSE_FWD (1<<7)
#define MAC_CMD_PAUSE_IGNORE (1<<8)
#define MAC_CMD_TX_ADDR_INS (1<<9)
#define MAC_CMD_HD_ENA (1<<10)
#define MAC_CMD_EXCESS_COL (1<<11)
#define MAC_CMD_LATE_COL (1<<12)
#define MAC_CMD_SW_RESET (1<<13)
#define MAC_CMD_MHASH_SEL (1<<14)
#define MAC_CMD_LOOP_ENA (1<<15)
#define MAC_CMD_MAGIC_ENA (1<<19)
#define MAC_CMD_SLEEP (1<<20)
#define MAC_CMD_WAKEUP (1<<21)

struct revision_info {
    u16 ip_core;
    u16 customer;
};

static void read_rev(u32 __iomem *base, struct revision_info *dst)
{
    u32 rev;
    rev = ioread32(base);
    rmb();
    memcpy(dst, &rev, sizeof(u32));
}

static int test_scratch(u32 __iomem *base)
{
    u32 *scratch = ((u32 *)base) + A2GX_MAC_SCRATCH_REG;
    u32 i;
    u32 v;

    for (i = 0; i < 5; ++i) {
        iowrite32(i, scratch);
        wmb();
        v = ioread32(scratch);
        rmb();
        if (v != i)
            goto on_err;
    }
    return 0;

  on_err:
    return -1;
}

static void reset(u32 __iomem *base)
{
    u32 __iomem *reg;
    u32 cmd;

    reg = base + A2GX_MAC_CMD_CFG_REG;
    cmd = ioread32(reg);
    rmb();

    cmd &= (~MAC_CMD_RX_ENA);
    cmd &= (~MAC_CMD_TX_ENA);
    cmd |= MAC_CMD_SW_RESET;

    iowrite32(cmd, reg);
    wmb();
}

static int is_resetting(u32 __iomem *base)
{
    u32 __iomem *reg;
    u32 cmd;
    reg = base + A2GX_MAC_CMD_CFG_REG;
    cmd = ioread32(reg);
    rmb();
    return (cmd & MAC_CMD_SW_RESET);
}

int a2gx_mac_init(struct a2gx_dev *dev)
{
    struct revision_info rev;
    u32 __iomem *base;
    u32 frm_len;
    unsigned int i;

    base = (dev->bar2 + A2GX_MAC_BASE);

    reset(base);
    for (i = 0; i < 50; ++i) {
        if (!is_resetting(base))
            break;
    }
    if (is_resetting(base)) {
        printk(A2GX_ERR "MAC stuck in reset.\n");
        goto on_err;
    }

    if (test_scratch(base)) {
        printk(A2GX_ERR "MAC scratch register test failed.\n");
        goto on_err;
    }
    read_rev(base, &rev);
    frm_len = ioread32(base + A2GX_MAC_FRM_LEN_REG);
    rmb();

    printk(A2GX_INFO
           "Triple-Speed Ethernet MAC rev. %u/%u [frm_len=%u].\n",
           rev.ip_core, rev.customer, frm_len);

    return 0;
  on_err:
    return -1;
}
