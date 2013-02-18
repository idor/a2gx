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

#include <linux/etherdevice.h>
#include "a2gx.h"
#include "a2gx_device.h"
#include "a2gx_net.h"
#include "a2gx_dma.h"
#include "a2gx_mac.h"

struct a2gx_dev *a2gx_net_alloc(struct pci_dev *pci_dev)
{
    struct net_device *net_dev;
    struct a2gx_dev *dev = NULL;

    net_dev = alloc_etherdev(sizeof(struct a2gx_dev));
    if (!net_dev)
        goto out;
    dev = netdev_priv(net_dev);
    memset(dev, 0, sizeof(struct a2gx_dev));
    dev->pci_dev = pci_dev;
    dev->net_dev = net_dev;

  out:
    return dev;
}

void a2gx_net_free(struct a2gx_dev *dev)
{
    free_netdev(dev->net_dev);
}

static int net_open(struct net_device *dev)
{
    netif_start_queue(dev);
    netif_wake_queue(dev);
    return 0;
}

static int net_stop(struct net_device *dev)
{
    netif_stop_queue(dev);
    return 0;
}

static int net_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    printk(KERN_INFO "a2gx_net_start_xmit is called\n");
    dev_kfree_skb(skb);
    return 0;
}

static struct net_device_stats *net_get_stats(struct net_device *net_dev)
{
    struct a2gx_dev *dev;
    struct a2gx_mac_stats mac_stats;
    struct net_device_stats *net_stats;

    dev = netdev_priv(net_dev);
    a2gx_mac_stats(dev, &mac_stats);
    net_stats = &dev->net_stats;

    net_stats->rx_packets = mac_stats.rx_frames;
    net_stats->tx_packets = mac_stats.tx_frames;
    net_stats->rx_bytes = mac_stats.rx_octets_lo;
    net_stats->tx_bytes = mac_stats.tx_octets_lo;
    net_stats->rx_errors = (mac_stats.rx_if_errors +
                            mac_stats.rx_frames_crc_err +
                            mac_stats.rx_frames_align_err);
    net_stats->tx_errors = (mac_stats.tx_if_errors);
    net_stats->rx_length_errors = mac_stats.rx_undersized_pkts;
    net_stats->rx_over_errors = mac_stats.rx_oversized_pkts;
    net_stats->rx_crc_errors = mac_stats.rx_frames_crc_err;
    net_stats->rx_frame_errors = mac_stats.rx_frames_align_err;
    net_stats->rx_fifo_errors = mac_stats.drop_events / 2;
    net_stats->tx_fifo_errors = mac_stats.drop_events / 2;

    a2gx_dma_test(dev);

    return net_stats;
}

static const struct net_device_ops net_ops =
{
    .ndo_open = net_open,
    .ndo_stop = net_stop,
    .ndo_start_xmit = net_start_xmit,
    .ndo_get_stats = net_get_stats
};

int a2gx_net_init(struct a2gx_dev *dev)
{
    struct net_device *net_dev = dev->net_dev;
    unsigned int i;

    /* dev_addr is already set by MAC initialization */
    for (i = 0; i < 6; i++) {
        net_dev->broadcast[i] = 0xff;
    }
    net_dev->hard_header_len = 0;
    net_dev->netdev_ops = &net_ops;
    if (register_netdev(net_dev)) {
        printk(A2GX_ERR "Cannot register network device\n");
        return -1;
    }
    return 0;
}

void a2gx_net_fini(struct a2gx_dev *dev)
{
    unregister_netdev(dev->net_dev);
}
