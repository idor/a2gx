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

struct a2gx_dev;

struct a2gx_mac_stats {
    u32 mac_0;
    u32 mac_1;
    u32 tx_frames;
    u32 rx_frames;
    u32 rx_frames_crc_err;
    u32 rx_frames_align_err;
    u32 tx_octets_lo;
    u32 rx_octets_lo;
    u32 tx_pause_frames;
    u32 rx_pause_frames;
    u32 rx_if_errors;
    u32 tx_if_errors;
    u32 rx_ucast_pkts;
    u32 rx_mcast_pkts;
    u32 rx_bcast_pkts;
    u32 tx_ucast_pkts;
    u32 tx_mcast_pkts;
    u32 tx_bcast_pkts;
    u32 drop_events;
    u32 rx_octets_total_lo;
    u32 rx_frames_total_lo;
    u32 rx_undersized_pkts;
    u32 rx_oversized_pkts;
};

int a2gx_mac_init(struct a2gx_dev *dev);
void a2gx_mac_stats(struct a2gx_dev *dev, struct a2gx_mac_stats *stats);

#endif
