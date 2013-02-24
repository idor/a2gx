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

#include <cstring>
#include <cstdio>
#include <a2gx/device.h>
#include <a2gx/eth_mac.h>
#include <a2gx/ring.h>
#include "test.hh"

static unsigned int write_read_requests(a2gx_device *dev)
{
    unsigned int count = 0;
    int ret;
    for (;;) {
        ret = a2gx_rx_request(&dev->rx);
        if (ret != 0)
            break;
        ++count;
    }
    return count;
}

TEST(device, echo)
{
    a2gx_device dev;
    unsigned int req_count;
    unsigned char *buf;
    unsigned int len;
    int ret;
    bool read_more;
    unsigned int seq;
    unsigned char *tx_buf;

    a2gx_dev_init(&dev);
    ASSERT_EQ(0, a2gx_dev_open(&dev, 0));
    ASSERT_EQ(0, a2gx_dev_reset(&dev));

    // Enable RX/TX.
    ASSERT_EQ(0, a2gx_eth_mac_flow_enable(&dev.mac, 1 /* PROMIS ON */));

    read_more = true;
    for (;;) {
        /* Poll the device for data */
        ret = a2gx_rx_read(&dev.rx, &buf, &len);
        if (ret != 0) {
            if (read_more) {
                req_count = write_read_requests(&dev);
                if (!req_count) {
                    fprintf(stderr, "Cannot request a single DMA read!\n");
                    goto out;
                }
                printf("Requested %u DMA reads.\n", req_count);
                read_more = false;
            }
            continue;
        }

        read_more = true;
        printf("Packet #%u\t%u bytes", ++seq, len);
        for (size_t i = 0; i < len; ++i) {
            if (i % 16 == 0)
                printf("\n");
            printf("0x%.2X ", buf[i]);
        }
        printf("\n");

        for (int j = 0; j < 3; ++j) {
            ret = a2gx_tx_request(&dev.tx, &tx_buf, len);
            if (ret != 0) {
                printf("Cannot allocate TX buffer (%d)\n", ret);
                continue;
            }
            memcpy(tx_buf, buf, len);
            a2gx_tx_write(&dev.tx, tx_buf, len);
        }
    }

  out:
    a2gx_dev_reset(&dev);
    a2gx_dev_close(&dev);
}
