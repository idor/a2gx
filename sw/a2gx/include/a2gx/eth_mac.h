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

#ifndef A2GX_ETH_PHY_H
#define A2GX_ETH_PHY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct a2gx_device;

struct a2gx_eth_mac_stats {
    uint32_t mac_0;
    uint32_t mac_1;
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t rx_frames_crc_err;
    uint32_t rx_frames_align_err;
    uint32_t tx_octets_lo;
    uint32_t rx_octets_lo;
    uint32_t tx_pause_frames;
    uint32_t rx_pause_frames;
    uint32_t rx_if_errors;
    uint32_t tx_if_errors;
    uint32_t rx_ucast_pkts;
    uint32_t rx_mcast_pkts;
    uint32_t rx_bcast_pkts;
    uint32_t tx_ucast_pkts;
    uint32_t tx_mcast_pkts;
    uint32_t tx_bcast_pkts;
    uint32_t drop_events;
    uint32_t rx_octets_total_lo;
    uint32_t rx_frames_total_lo;
    uint32_t rx_undersized_pkts;
    uint32_t rx_oversized_pkts;
};

int a2gx_eth_mac_reset(struct a2gx_device *dev);

void a2gx_eth_mac_stats_read(struct a2gx_eth_mac_stats *dst,
                             struct a2gx_device *dev);

#ifdef __cplusplus
}
#endif

#endif
