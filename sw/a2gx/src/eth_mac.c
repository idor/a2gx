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

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <a2gx/device.h>
#include <a2gx/eth_mac.h>

#define MAC_BASE 0x02004000

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
#define PHY_SPEC_STATUS 0x11
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
#define MAC_CMD_MAGIC_ENA (1<<18)
#define MAC_CMD_SLEEP (1<<19)
#define MAC_CMD_WAKEUP (1<<20)
#define MAC_CMD_CNT_RESET (1<<30)

/*
 * Tests TSE MAC's scratch register to make sure that I/O
 * address is correct.
 */
static int test_scratch_reg(uint32_t *base)
{
    uint32_t *reg;
    uint32_t i;
    uint32_t v;

    reg = base + MAC_SCRATCH_REG;

    for (i = 0; i < 5; ++i) {
        *reg = i;
        a2gx_wmb();
        v = *reg;
        a2gx_rmb();
        if (v != i)
            goto on_err;
    }
    return 0;

  on_err:
    return -EIO;
}

static void phy_sw_reset(uint32_t *base)
{
    uint32_t ctrl;

    ctrl = base[PHY_CTRL_REG];
    a2gx_rmb();
    base[PHY_CTRL_REG] = ctrl | 0x8000;
    a2gx_wmb();
}

/*
 * Resets Marvell 88E1111. Since we don't have a spec for the PHY,
 * the logic is ported from Altera's "N647_TSE_Single_Port_RGMI_Dev_AIIGX_ACDS"
 * reference design for Arria II GX Board (tse_marvell_phy.tcl). The below
 * initialization will bring the PHY's link up.
 */
static int phy_reset(uint32_t *base)
{
    uint32_t v;

    /* Initialize Control (REG 0) */
    v = base[PHY_CTRL_REG];
    a2gx_rmb();

    v &= 0x8EBF; /* Reset */
    v |= 0x0040; /* Enable Speed 1000 */
    v |= 0x0100; /* Enable Full Duplex Mode */
    base[PHY_CTRL_REG] = v;
    a2gx_wmb();

    phy_sw_reset(base);

    /* AN Advertisement Register (REG 4) */
    v = base[PHY_AUTO_NEG];
    a2gx_rmb();

    base[PHY_AUTO_NEG] = v & 0xFE1F;
    a2gx_wmb();
    phy_sw_reset(base);

    /* 1000BASE-T Control Register (REG 9) */
    v = base[PHY_1000BASE_T_CONTROL];
    a2gx_rmb();
    base[PHY_1000BASE_T_CONTROL] = v & 0xFCFF;
    a2gx_wmb();
    phy_sw_reset(base);

    /* PHYSpecific Control Register (REG 16).
       Set PHY Synchronizing FIFO to maximum */
    v = base[PHY_SPEC_CONTROL];
    a2gx_rmb();

    base[PHY_SPEC_CONTROL] = v | 0xC000;
    a2gx_wmb();
    phy_sw_reset(base);

    /* Extended PHYSpecific Status Register (REG 27).
       Set PHY HWCFG_MODE for RGMII to Copper. */
    v = base[PHY_SPEC_STATUS_EXT];
    a2gx_rmb();
    base[PHY_SPEC_STATUS_EXT] = v | 0x000B;
    a2gx_wmb();
    phy_sw_reset(base);

    /* Extended PHYSpecific Control Register (REG 20).
       Enable RGMII TX and RX Timing Control. */
    v = base[PHY_SPEC_CONTROL_EXT];
    a2gx_rmb();

    v &= 0xFF7D;
    v |= 0x0082;
    base[PHY_SPEC_CONTROL_EXT] = v;
    a2gx_wmb();
    phy_sw_reset(base);

    return 0;
}

static void generate_random_mac(unsigned char mac_addr[6])
{
    long int seed;

    seed = random();

    /* This is the Altera Vendor ID */
    mac_addr[0] = 0x0;
    mac_addr[1] = 0x7;
    mac_addr[2] = 0xed;

    /* Reserverd Board identifier */
    mac_addr[3] = (unsigned char)(0xFF ^ seed);
    mac_addr[4] = (unsigned char)((seed & 0xff00) >> 8);
    mac_addr[5] = (unsigned char)(seed & 0xff);
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
static void set_mac_address(uint32_t *mac, const unsigned char addr[6])
{
    unsigned char str[4];
    uint32_t mac_0;
    uint32_t mac_1;

    memcpy(&mac_0, addr, 4);
    str[0] = addr[4];
    str[1] = addr[5];
    str[2] = 0;
    str[4] = 0;
    memcpy(&mac_1, str, 4);

    mac[MAC_ADDR0_REG] = mac_0;
    mac[MAC_ADDR1_REG] = mac_1;
    a2gx_wmb();
}

static void set_random_mac_address(uint32_t *mac)
{
    unsigned char addr[6];
    generate_random_mac(addr);
    set_mac_address(mac, addr);
}

static uint32_t mac_cmd_read(uint32_t *mac)
{
    uint32_t ret;
    ret = mac[MAC_CMD_CFG_REG];
    a2gx_rmb();
    return ret;
}

static void mac_cmd_write(uint32_t *mac, uint32_t cmd)
{
    mac[MAC_CMD_CFG_REG] = cmd;
    a2gx_wmb();
}

static void mac_reset(uint32_t *mac)
{
    mac_cmd_write(mac, mac_cmd_read(mac) | MAC_CMD_SW_RESET);
}

static int mac_in_reset(uint32_t *mac)
{
    uint32_t cmd;
    cmd = mac[MAC_CMD_CFG_REG];
    a2gx_rmb();
    return (cmd & MAC_CMD_SW_RESET);
}

static int mac_wait_for_reset_completion(uint32_t *mac)
{
    unsigned int i;
    for (i = 0; i < 50000; ++i) {
        if (!mac_in_reset(mac))
            break;
        usleep(1);
    }
    return mac_in_reset(mac) ? -1 : 0;
}

int a2gx_eth_mac_reset(struct a2gx_device *dev)
{
    int r;
    uint32_t *base;
    uint32_t cmd;
    struct a2gx_eth_mac_stats stats; /* todo: remove me */

    base = a2gx_io_base(dev->reg.mem, MAC_BASE);

    r = test_scratch_reg(base);
    if (r) {
        printf("scratch test failed\n");
        goto on_err;
    }

    mac_reset(base);
    printf("BEFORE WE BEGIN = 0x%x\n", mac_cmd_read(base));
    mac_reset(base);

    /*
     * Set PHY MDIO addresses. Our Arria II GX board has default hard-wired
     * Marvell 88E1111 PHY. Both addresses are set to 0.
     */
    base[MAC_MDIO_ADDR0_REG] = 0;
    base[MAC_MDIO_ADDR1_REG] = 0;
    a2gx_wmb();

    r = phy_reset(&base[MAC_MDIO0_REG]);
    if (r)
        goto on_err;

    set_random_mac_address(base);
    mac_reset(base);
    if (mac_wait_for_reset_completion(base)) {
        printf("STUCK = 0x%x\n", mac_cmd_read(base));
        goto on_err;
    }

    cmd = mac_cmd_read(base);

    /* TODO: Do not turn these on here RX/TX ENA here! */
    cmd |= MAC_CMD_TX_ENA;
    cmd |= MAC_CMD_RX_ENA;
    cmd |= MAC_CMD_PROMIS_EN;

    cmd |= MAC_CMD_ETH_SPEED; /* Gigabit Mode */
    cmd |= MAC_CMD_PAUSE_IGNORE;
    cmd |= MAC_CMD_CNT_RESET;

    mac_cmd_write(base, cmd);

    a2gx_eth_mac_stats_read(&stats, dev);
    printf("stats: tx=%u, rx=%u, crc_err=%u, rx_if_err=%u, tx_if_err=%u\n",
           stats.tx_frames,
           stats.rx_frames,
           stats.rx_frames_crc_err,
           stats.rx_if_errors,
           stats.tx_if_errors);

    printf("AFTER ALL = 0x%x\n", mac_cmd_read(base));

    return 0;

  on_err:
    return -1;
}

void a2gx_eth_mac_stats_read(struct a2gx_eth_mac_stats *dst,
                             struct a2gx_device *dev)
{
    uint32_t *mac = a2gx_io_base(dev->reg.mem, MAC_BASE);

    dst->mac_0               = mac[0x18];
    dst->mac_1               = mac[0x19];
    dst->tx_frames           = mac[0x1A];
    dst->rx_frames           = mac[0x1B];
    dst->rx_frames_crc_err   = mac[0x1C];
    dst->rx_frames_align_err = mac[0x1D];
    dst->tx_octets_lo        = mac[0x1E];
    dst->rx_octets_lo        = mac[0x1F];
    dst->tx_pause_frames     = mac[0x20];
    dst->rx_pause_frames     = mac[0x21];
    dst->rx_if_errors        = mac[0x22];
    dst->tx_if_errors        = mac[0x23];
    dst->rx_ucast_pkts       = mac[0x24];
    dst->rx_mcast_pkts       = mac[0x25];
    dst->rx_bcast_pkts       = mac[0x26];
    dst->tx_ucast_pkts       = mac[0x28];
    dst->tx_mcast_pkts       = mac[0x29];
    dst->tx_bcast_pkts       = mac[0x2A];
    dst->drop_events         = mac[0x2B];
    dst->rx_octets_total_lo  = mac[0x2C];
    dst->rx_frames_total_lo  = mac[0x2D];
    dst->rx_undersized_pkts  = mac[0x2E];
    dst->rx_oversized_pkts   = mac[0x2F];
    a2gx_rmb();
}
