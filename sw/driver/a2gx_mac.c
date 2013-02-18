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

#include <linux/dmi.h>
#include <linux/netdevice.h>
#include "a2gx.h"
#include "a2gx_device.h"
#include "a2gx_mac.h"

#define MAC_BASE 0x02004000
#define MAC_BASE_END 0x020043ff
#define MAC_BASE_LEN (MAC_BASE_END-MAC_BASE)

#define MAC_REV_REG 0x00
#define MAC_SCRATCH_REG 0x01
#define MAC_CMD_CFG_REG 0x02
#define MAC_ADDR0_REG 0x03
#define MAC_ADDR1_REG 0x04
#define MAC_FRM_LEN_REG 0x05
#define MAC_PAUSE_QUANT_REG 0x06
#define MAC_TX_IPG_REG 0x17
#define MAC_MDIO_ADDR0_REG 0x0F
#define MAC_MDIO_ADDR1_REG 0x10
#define MAC_MDIO0_REG 0x80
#define MAC_MDIO1_REG 0xA0
#define PHY_CTRL_REG 0x00
#define PHY_AUTO_NEG 0x04
#define PHY_AUTO_NEG 0x04
#define PHY_1000BASE_T_CONTROL 0x09
#define PHY_SPEC_CONTROL 0x10
#define PHY_SPEC_STATUS 0x11 /* 0x2c4 */
#define PHY_SPEC_CONTROL_EXT 0x14
#define PHY_SPEC_STATUS_EXT 0x1B

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
#define MAC_CMD_CNT_RESET (1<<31)

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
    u32 *scratch = ((u32 *)base) + MAC_SCRATCH_REG;
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

static u32 mac_read_cfg(u32 __iomem *base)
{
    u32 ret;
    ret = ioread32(base + MAC_CMD_CFG_REG);
    rmb();
    return ret;
}

static void mac_write_cfg(u32 __iomem *base, u32 cfg)
{
    iowrite32(cfg, base + MAC_CMD_CFG_REG);
    wmb();
}

static void mac_reset(u32 __iomem *base)
{
    mac_write_cfg(base, mac_read_cfg(base) | MAC_CMD_SW_RESET);
}

static int mac_in_reset(u32 __iomem *base)
{
    u32 __iomem *reg;
    u32 cmd;
    reg = base + MAC_CMD_CFG_REG;
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
    ctrl = ioread32(base + PHY_CTRL_REG);
    rmb();
    iowrite32(ctrl | 0x8000, base + PHY_CTRL_REG);
    wmb();
}

/*
 * Initialize Marvell PHY. Since we don't have a spec for that PHY,
 * the logic is ported from Altera's "N647_TSE_Single_Port_RGMI_Dev_AIIGX_ACDS"
 * reference design for Arria II GX Board (tse_marvell_phy.tcl).
 * The below initialization will bring the PHY's link up.
 */
static void init_phy(u32 __iomem *base)
{
    u32 v;

    /* Initialize Control (REG 0) */
    v = ioread32(base + PHY_CTRL_REG);
    rmb();

    v &= 0x8EBF; /* Reset */
    v |= 0x0040; /* Enable Speed 1000 */
    v |= 0x0100; /* Enable Full Duplex Mode */
    iowrite32(v, base + PHY_CTRL_REG);
    wmb();

    phy_sw_reset(base);

    /* AN Advertisement Register (REG 4) */
    v = ioread32(base + PHY_AUTO_NEG);
    rmb();

    iowrite32(v & 0xFE1F, base + PHY_AUTO_NEG);
    wmb();
    phy_sw_reset(base);

    /* 1000BASE-T Control Register (REG 9) */
    v = ioread32(base + PHY_1000BASE_T_CONTROL);
    rmb();
    iowrite32(v & 0xFCFF, base + PHY_1000BASE_T_CONTROL);
    wmb();
    phy_sw_reset(base);

    /* PHYSpecific Control Register (REG 16).
       Set PHY Synchronizing FIFO to maximum */
    v = ioread32(base + PHY_SPEC_CONTROL);
    rmb();

    iowrite32(v | 0xC000, base + PHY_SPEC_CONTROL);
    wmb();
    phy_sw_reset(base);

    /* Extended PHYSpecific Status Register (REG 27).
       Set PHY HWCFG_MODE for RGMII to Copper. */
    v = ioread32(base + PHY_SPEC_STATUS_EXT);
    rmb();
    iowrite32(v | 0x000B, base + PHY_SPEC_STATUS_EXT);
    wmb();
    phy_sw_reset(base);

    /* Extended PHYSpecific Control Register (REG 20).
       Enable RGMII TX and RX Timing Control. */
    v = ioread32(base + PHY_SPEC_CONTROL_EXT);
    rmb();

    v &= 0xFF7D;
    v |= 0x0082;
    iowrite32(v, base + PHY_SPEC_CONTROL_EXT);
    wmb();
    phy_sw_reset(base);
}

#if 0
static int phy_link_is_up(u32 __iomem *base)
{
    /* PHY Specific Status Register (REG 17). Wait for Link Up. */
    u32 status;
    status = ioread32(base + PHY_SPEC_STATUS);
    rmb();
    return ((status & 0x0400) == 0x00000400);
}
#endif

static u32 get_board_serial(void)
{
    u32 res = 0;
    u64 id_n;
    const char *s;
    s = dmi_get_system_info(DMI_BOARD_SERIAL);
    if (!s)
        goto out;
    if (kstrtou64(s, 10, &id_n))
        goto out;
    res = (u32)id_n;
  out:
    return res;
}

static void generate_mac(unsigned char mac_addr[6])
{
    u32 ser_num;

    ser_num = get_board_serial();

    /* This is the Altera Vendor ID */
    mac_addr[0] = 0x0;
    mac_addr[1] = 0x7;
    mac_addr[2] = 0xed;

    /* Reserverd Board identifier */
    mac_addr[3] = 0xFF;
    mac_addr[4] = (ser_num & 0xff00) >> 8;
    mac_addr[5] = ser_num & 0xff;
}

/*
 * Sets 6-byte MAC primary address for TSE.
 *
 * The first four most significant bytes of the MAC address occupy mac_0 in
 * reverse order. The last two bytes of the MAC address occupy the two least
 * significant bytes of mac_1 in reverse order.
 *
 * For example, if the MAC address is 00-1C-23-17-4A-CB, the following
 * assignments are made:
 *
 *     mac_0 = 0x17231c00
 *     mac_1 = 0x0000CB4a
 *
 * Ensure that you configure these registers with a valid MAC address if you
 * disable the promiscuous mode (PROMIS_EN bit in command_config = 0).
 */
static void write_mac_addr(u32 __iomem *base, const unsigned char mac_addr[6])
{
    unsigned char str[4];
    u32 mac_0;
    u32 mac_1;

    memcpy(&mac_0, mac_addr, sizeof(u32));
    str[0] = mac_addr[4];
    str[1] = mac_addr[5];
    str[2] = 0;
    str[4] = 0;
    memcpy(&mac_1, str, sizeof(u32));

    iowrite32(mac_0, base + MAC_ADDR0_REG);
    iowrite32(mac_1, base + MAC_ADDR1_REG);
    wmb();
}

static void init_mac(u32 __iomem *base, struct net_device *net_dev)
{
    u32 cfg;
    unsigned char mac_addr[6];
    generate_mac(mac_addr);
    write_mac_addr(base, mac_addr);
    cfg = mac_read_cfg(base);

    /* Do not turn these on here! */
    cfg |= MAC_CMD_TX_ENA;
    cfg |= MAC_CMD_RX_ENA;
    cfg |= MAC_CMD_PROMIS_EN;

    cfg |= MAC_CMD_ETH_SPEED; /* Gigabit Mode */
    cfg |= MAC_CMD_PAUSE_IGNORE;
    cfg |= MAC_CMD_CNT_RESET;
    mac_write_cfg(base, cfg);
    memcpy(net_dev->dev_addr, mac_addr, sizeof(mac_addr));
}

int a2gx_mac_init(struct a2gx_dev *dev)
{
    struct revision_info rev;
    u32 __iomem *base;
    u32 frm_len;

    base = (dev->bar + MAC_BASE);

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
    iowrite32(0, base + MAC_MDIO_ADDR0_REG);
    iowrite32(0, base + MAC_MDIO_ADDR1_REG);
    wmb();
    init_phy(base + MAC_MDIO0_REG);

    /*
     * Finish MAC configuration and enable RX/TX.
     */
    init_mac(base, dev->net_dev);

    read_rev(base, &rev);
    frm_len = ioread32(base + MAC_FRM_LEN_REG);
    rmb();

    printk(A2GX_INFO
           "Triple-Speed Ethernet MAC rev. %u/%u [frm_len=%u].\n",
           rev.ip_core, rev.customer, frm_len);

    return 0;
  on_err:
    return -1;
}

void a2gx_mac_stats(struct a2gx_dev *dev, struct a2gx_mac_stats *stats)
{
    u32 __iomem *base = (dev->bar + MAC_BASE);

    stats->mac_0 = ioread32(base + 0x18);
    stats->mac_1 = ioread32(base + 0x19);
    stats->tx_frames = ioread32(base + 0x1A);
    stats->rx_frames = ioread32(base + 0x1B);
    stats->rx_frames_crc_err = ioread32(base + 0x1C);
    stats->rx_frames_align_err = ioread32(base + 0x1D);
    stats->tx_octets_lo = ioread32(base + 0x1E);
    stats->rx_octets_lo = ioread32(base + 0x1F);
    stats->tx_pause_frames = ioread32(base + 0x20);
    stats->rx_pause_frames = ioread32(base + 0x21);
    stats->rx_if_errors = ioread32(base + 0x22);
    stats->tx_if_errors = ioread32(base + 0x23);
    stats->rx_ucast_pkts = ioread32(base + 0x24);
    stats->rx_mcast_pkts = ioread32(base + 0x25);
    stats->rx_bcast_pkts = ioread32(base + 0x26);
    stats->tx_ucast_pkts = ioread32(base + 0x28);
    stats->tx_mcast_pkts = ioread32(base + 0x29);
    stats->tx_bcast_pkts = ioread32(base + 0x2A);
    stats->drop_events = ioread32(base + 0x2B);
    stats->rx_octets_total_lo = ioread32(base + 0x2C);
    stats->rx_frames_total_lo = ioread32(base + 0x2D);
    stats->rx_undersized_pkts = ioread32(base + 0x2E);
    stats->rx_oversized_pkts = ioread32(base + 0x2F);
    rmb();
}
