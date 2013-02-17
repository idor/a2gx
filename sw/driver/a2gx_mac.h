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

#ifndef A2GX_MAC_H
#define A2GX_MAC_H

#define A2GX_MAC_BASE 0x02004000
#define A2GX_MAC_BASE_END 0x020043ff
#define A2GX_MAC_BASE_LEN (A2GX_MAC_BASE_END-A2GX_MAC_BASE)

#define A2GX_MAC_REV_REG 0x00
#define A2GX_MAC_SCRATCH_REG 0x01
#define A2GX_MAC_CMD_CFG_REG 0x02
#define A2GX_MAC_ADDR0_REG 0x03
#define A2GX_MAC_ADDR1_REG 0x04
#define A2GX_MAC_FRM_LEN_REG 0x05
#define A2GX_MAC_PAUSE_QUANT_REG 0x06
#define A2GX_MAC_TX_IPG_REG 0x17
#define A2GX_MAC_MDIO_ADDR0_REG 0x0F
#define A2GX_MAC_MDIO_ADDR1_REG 0x10
#define A2GX_MAC_MDIO0_REG 0x80
#define A2GX_MAC_MDIO1_REG 0xA0
#define A2GX_PHY_CTRL_REG 0x00
#define A2GX_PHY_AUTO_NEG 0x04
#define A2GX_PHY_AUTO_NEG 0x04
#define A2GX_PHY_1000BASE_T_CONTROL 0x09
#define A2GX_PHY_SPEC_CONTROL 0x10
#define A2GX_PHY_SPEC_STATUS 0x11 /* 0x2c4 */
#define A2GX_PHY_SPEC_CONTROL_EXT 0x14
#define A2GX_PHY_SPEC_STATUS_EXT 0x1B

struct a2gx_dev;

int a2gx_mac_init(struct a2gx_dev *dev);

#endif
