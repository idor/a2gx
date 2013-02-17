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

static void mac_reset(u32 __iomem *base)
{
    u32 __iomem *reg;
    u32 cmd;

    reg = base + A2GX_MAC_CMD_CFG_REG;
    cmd = ioread32(reg);
    rmb();
    iowrite32(cmd | MAC_CMD_SW_RESET, reg);
    wmb();
}

static int mac_in_reset(u32 __iomem *base)
{
    u32 __iomem *reg;
    u32 cmd;
    reg = base + A2GX_MAC_CMD_CFG_REG;
    cmd = ioread32(reg);
    rmb();
    return (cmd & MAC_CMD_SW_RESET);
}

static int mac_wait_for_reset_completion(u32 __iomem *base)
{
    unsigned int i;
    for (i = 0; i < 50; ++i) {
        if (!mac_in_reset(base))
            break;
    }
    return mac_in_reset(base) ? -1 : 0;
}

static void phy_sw_reset(u32 __iomem *base)
{
    u32 ctrl;
    ctrl = ioread32(base + A2GX_PHY_CTRL_REG);
    rmb();
    iowrite32(ctrl | 0x8000, base + A2GX_PHY_CTRL_REG);
    wmb();
}

/*
 * Initialize Marvell PHY. Since we don't have a spec for that PHY,
 * the logic is ported from Altera's "N647_TSE_Single_Port_RGMI_Dev_AIIGX_ACDS"
 * reference design for Arria II GX Board (tse_marvell_phy.tcl).
 */
static void init_phy(u32 __iomem *base)
{
    u32 v;
    unsigned int i;

    /* Initialize Control (REG 0) */
    v = ioread32(base + A2GX_PHY_CTRL_REG);
    rmb();

    v &= 0x8EBF; /* Reset */
    v |= 0x0040; /* Enable Speed 1000 */
    v |= 0x0100; /* Enable Full Duplex Mode */
    iowrite32(v, base + A2GX_PHY_CTRL_REG);
    wmb();

    phy_sw_reset(base);

    /* AN Advertisement Register (REG 4) */
    v = ioread32(base + A2GX_PHY_AUTO_NEG);
    rmb();

    iowrite32(v & 0xFE1F, base + A2GX_PHY_AUTO_NEG);
    wmb();
    phy_sw_reset(base);

    /* 1000BASE-T Control Register (REG 9) */
    v = ioread32(base + A2GX_PHY_1000BASE_T_CONTROL);
    rmb();
    iowrite32(v & 0xFCFF, base + A2GX_PHY_1000BASE_T_CONTROL);
    wmb();
    phy_sw_reset(base);

    /* PHYSpecific Control Register (REG 16).
       Set PHY Synchronizing FIFO to maximum */
    v = ioread32(base + A2GX_PHY_SPEC_CONTROL);
    rmb();

    iowrite32(v | 0xC000, base + A2GX_PHY_SPEC_CONTROL);
    wmb();
    phy_sw_reset(base);

    /* Extended PHYSpecific Status Register (REG 27).
       Set PHY HWCFG_MODE for RGMII to Copper. */
    v = ioread32(base + A2GX_PHY_SPEC_STATUS_EXT);
    rmb();
    iowrite32(v | 0x000B, base + A2GX_PHY_SPEC_STATUS_EXT);
    wmb();
    phy_sw_reset(base);

    /* Extended PHYSpecific Control Register (REG 20).
       Enable RGMII TX and RX Timing Control. */
    v = ioread32(base + A2GX_PHY_SPEC_CONTROL_EXT);
    rmb();

    v &= 0xFF7D;
    v |= 0x0082;
    iowrite32(v, base + A2GX_PHY_SPEC_CONTROL_EXT);
    wmb();
    phy_sw_reset(base);
}

static int phy_link_is_up(u32 __iomem *base)
{
    /* PHY Specific Status Register (REG 17). Wait for Link Up. */
    u32 status;
    status = ioread32(base + A2GX_PHY_SPEC_STATUS);
    rmb();
    return ((status & 0x0400) == 0x00000400);
}

int a2gx_mac_init(struct a2gx_dev *dev)
{
    struct revision_info rev;
    u32 __iomem *base;
    u32 frm_len;

    base = (dev->bar2 + A2GX_MAC_BASE);

    /*
     * Test scratch register (R/W).
     */
    if (test_scratch(base)) {
        printk(A2GX_ERR "MAC scratch register test failed.\n");
        goto on_err;
    }

    /*
     * Reset the MAC in case it was doing something else.
     */
    mac_reset(base);
    if (mac_wait_for_reset_completion(base)) {
        printk(A2GX_ERR "MAC stuck in reset.\n");
        goto on_err;
    }

    /*
     * Setup Marvell 88E1111 PHY.
     */
    iowrite32(0, base + A2GX_MAC_MDIO_ADDR0_REG);
    iowrite32(0, base + A2GX_MAC_MDIO_ADDR1_REG);
    wmb();
    init_phy(base + A2GX_MAC_MDIO0_REG);

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
