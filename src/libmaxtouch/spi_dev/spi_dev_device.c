//------------------------------------------------------------------------------
// Copyright 2018 Solomon Systech. All rights reserved.
//
// Author: giuseppe.penone@solomon-systech.com
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
//    2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY ATMEL ''AS IS'' AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
// EVENT SHALL ATMEL OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
// OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

struct mxt_device;
struct mxt_conn_info;

#include "spi_dev_device.h"
#include "libmaxtouch/libmaxtouch.h"

// opcodes
#define SPI_WRITE_REQ    0x01
#define SPI_WRITE_OK     0x81
#define SPI_WRITE_FAIL   0x41
#define SPI_READ_REQ     0x02
#define SPI_READ_OK      0x82
#define SPI_READ_FAIL    0x42
#define SPI_INVALID_REQ  0x04
#define SPI_INVALID_CRC  0x08

#define SPI_HEADER_LEN      6
#define SPI_TX_RX_BUF_SIZE  (SPI_HEADER_LEN+SPI_DEV_MAX_BLOCK)

// header 6 bytes + Data[]
// 0 opcode
// 1 address LSB
// 2 address MSB
// 3 length LSB
// 4 length MSB
// 5 CRC
// 6+ Data[]

static uint32_t spi_mode32 = SPI_CPHA | SPI_CPOL;
static uint8_t spi_bits_per_word = 8;
static uint32_t spi_max_speed_hz = 8000000; // 8 MHz
static uint8_t spi_tx_buf[SPI_TX_RX_BUF_SIZE];
static uint8_t spi_rx_buf[SPI_TX_RX_BUF_SIZE];
static struct spi_ioc_transfer spi_ioc_tr;
static uint8_t spi_tx_dummy_buf[SPI_TX_RX_BUF_SIZE] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static void hex_dump(const void *src, size_t length, size_t line_size, char *prefix)
{
    int i = 0;
    const unsigned char *address = src;
    const unsigned char *line = address;
    unsigned char c;

    printf("%s | ", prefix);
    while (length-- > 0) {
        printf("%02X ", *address++);
        if (!(++i % line_size) || (length == 0 && i % line_size)) {
            if (length == 0) {
                while (i++ % line_size)
                    printf("__ ");
            }
            printf(" | ");  /* right close */
            while (line < address) {
                c = *line++;
                printf("%c", (c < 33 || c == 255) ? 0x2E : c);
            }
            printf("\n");
            if (length > 0)
                printf("%s | ", prefix);
        }
    }
}

static uint8_t get_crc8_iter(uint8_t crc, uint8_t data)
{
    static const uint8_t crcpoly = 0x8c;
    uint8_t index = 8;
    uint8_t fb;
    do
    {
        fb = (crc ^ data) & 0x01;
        data >>= 1;
        crc >>= 1;
        if (fb)
        {
            crc ^= crcpoly;
        }
    } while (--index);
    return crc;
}

static uint8_t get_header_crc(uint8_t *p_msg)
{
    uint8_t calc_crc = 0;
    int i = 0;
    for (; i < SPI_HEADER_LEN-1; i++)
    {
        calc_crc = get_crc8_iter(calc_crc, p_msg[i]);
    }
    return calc_crc;
}

static int spi_open_device(struct mxt_device *mxt, int *fd_out)
{
    int fd;
    int ret_val;
    char filename[20];

    snprintf(filename, 20, "/dev/spidev%d.%d", mxt->conn->spi_dev.bus, mxt->conn->spi_dev.chipselect);
    fd = open(filename, O_RDWR);
    if (fd < 0)
    {
        mxt_log_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
        return mxt_errno_to_rc(errno);
    }

    ret_val = ioctl(fd, SPI_IOC_WR_MODE32, &spi_mode32);
    if (ret_val == -1)
    {
        mxt_log_err(mxt->ctx, "can't set spi spi_mode32, error %s (%d)", strerror(errno), errno);
        close(fd);
        return mxt_errno_to_rc(errno);
    }
    ret_val = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (ret_val == -1)
    {
        mxt_log_err(mxt->ctx, "can't set bits per word, error %s (%d)", strerror(errno), errno);
        close(fd);
        return mxt_errno_to_rc(errno);
    }
    ret_val = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_max_speed_hz);
    if (ret_val == -1)
    {
        mxt_log_err(mxt->ctx, "can't set max speed hz, error %s (%d)", strerror(errno), errno);
        close(fd);
        return mxt_errno_to_rc(errno);
    }

    *fd_out = fd;
    return MXT_SUCCESS;
}

static void spi_prepare_header(uint8_t *header,
                               uint8_t opcode,
                               uint16_t start_register,
                               uint16_t count)
{
    header[0] = opcode;
    header[1] = start_register & 0xff;
    header[2] = start_register >> 8;
    header[3] = count & 0xff;
    header[4] = count >> 8;
    header[5] = get_header_crc(header);
}

int spi_dev_read_register(struct mxt_device *mxt,
                          uint8_t *buf,
                          uint16_t start_register,
                          uint16_t count,
                          size_t *bytes_read)
{
    int fd = -ENODEV;
    int ret_val, attempt=0;
    spi_ioc_tr.rx_buf = (unsigned long)spi_rx_buf;

    if (count > SPI_DEV_MAX_BLOCK)
    {
        count = SPI_DEV_MAX_BLOCK;
    }

    ret_val = spi_open_device(mxt, &fd);
    if (ret_val)
    {
        return ret_val;
    }

    do
    {
        attempt++;
        if (attempt > 1)
        {
            if (attempt > 5)
            {
                mxt_log_err(mxt->ctx, "Too many Retries\n");
                return MXT_ERROR_IO;
            }
            mxt_log_warn(mxt->ctx, "Retry %d after CRC fail\n", attempt-1);
        }
        /* WRITE SPI_READ_REQ */
        spi_prepare_header(spi_tx_buf, SPI_READ_REQ, start_register, count);
        spi_ioc_tr.tx_buf = (unsigned long)spi_tx_buf;
        spi_ioc_tr.len = SPI_HEADER_LEN;
        ret_val = ioctl(fd, SPI_IOC_MESSAGE(1), &spi_ioc_tr);
        if (ret_val < 1)
        {
            mxt_log_err(mxt->ctx, "Error %s (%d) writing to spi", strerror(errno), errno);
            ret_val = mxt_errno_to_rc(errno);
            goto close;
        }

        if (MXT_SUCCESS != mxt_wait_for_chg(mxt))
        {
            mxt_log_err(mxt->ctx, "Timeout on CHG");
        }

        //hex_dump(spi_tx_buf, spi_ioc_tr.len, 32, "TX");
        //hex_dump(spi_rx_buf, spi_ioc_tr.len, 32, "RX");
        /* READ SPI_READ_OK */
        spi_ioc_tr.tx_buf = (unsigned long)spi_tx_dummy_buf;
        spi_ioc_tr.len = SPI_HEADER_LEN + count;
        ret_val = ioctl(fd, SPI_IOC_MESSAGE(1), &spi_ioc_tr);
        if (ret_val < 1)
        {
            mxt_log_err(mxt->ctx, "Error %s (%d) reading from spi", strerror(errno), errno);
            ret_val = mxt_errno_to_rc(errno);
            goto close;
        }
        //hex_dump(spi_tx_dummy_buf, spi_ioc_tr.len, 32, "TX");
        //hex_dump(spi_rx_buf, spi_ioc_tr.len, 32, "RX");
        if (SPI_READ_OK != spi_rx_buf[0])
        {
            mxt_log_err(mxt->ctx, "SPI_READ_OK != %.2X reading from spi", spi_rx_buf[0]);
            ret_val = MXT_ERROR_PROTOCOL_FAULT;
            goto close;
        }
        if (spi_tx_buf[1] != spi_rx_buf[1] || spi_tx_buf[2] != spi_rx_buf[2])
        {
            mxt_log_err(mxt->ctx, "Unexpected address %d != %d reading from spi", spi_rx_buf[1] | (spi_rx_buf[2] << 8), start_register);
            ret_val = MXT_ERROR_PROTOCOL_FAULT;
            goto close;
        }
    }
    while (get_header_crc(spi_rx_buf) != spi_rx_buf[SPI_HEADER_LEN-1]);

    count = spi_rx_buf[3] | (spi_rx_buf[4] << 8);
    memcpy(buf, spi_rx_buf + SPI_HEADER_LEN, count);
    *bytes_read = count;

    ret_val = MXT_SUCCESS;

close:
    close(fd);
    return ret_val;
}

int spi_dev_write_register(struct mxt_device *mxt,
                           uint8_t const *buf,
                           uint16_t start_register,
                           uint16_t count)
{
    int fd = -ENODEV;
    int ret_val;
    uint16_t offset = 0;
    spi_ioc_tr.rx_buf = (unsigned long)spi_rx_buf;

    ret_val = spi_open_device(mxt, &fd);
    if (ret_val)
    {
        return ret_val;
    }

    while (offset < count)
    {
        int attempt = 0;
        uint16_t count_iter = count - offset;
        uint16_t address_iter = start_register + offset;
        if (count_iter > SPI_DEV_MAX_BLOCK)
        {
            count_iter = SPI_DEV_MAX_BLOCK;
        }

        do
        {
            attempt++;
            if (attempt > 1)
            {
                if (attempt > 5)
                {
                    mxt_log_err(mxt->ctx, "Too many Retries\n");
                    return MXT_ERROR_IO;
                }
                mxt_log_warn(mxt->ctx, "Retry %d after CRC fail\n", attempt-1);
            }
            /* WRITE SPI_WRITE_REQ */
            spi_prepare_header(spi_tx_buf, SPI_WRITE_REQ, address_iter, count_iter);
            memcpy(spi_tx_buf + SPI_HEADER_LEN, buf + offset, count_iter);
            spi_ioc_tr.tx_buf = (unsigned long)spi_tx_buf;
            spi_ioc_tr.len = SPI_HEADER_LEN + count_iter;
            ret_val = ioctl(fd, SPI_IOC_MESSAGE(1), &spi_ioc_tr);
            if (ret_val < 1)
            {
                mxt_log_err(mxt->ctx, "Error %s (%d) writing to spi", strerror(errno), errno);
                ret_val = mxt_errno_to_rc(errno);
                goto close;
            }

            if (MXT_SUCCESS != mxt_wait_for_chg(mxt))
            {
                mxt_log_err(mxt->ctx, "Timeout on CHG");
            }

            /* READ SPI_WRITE_OK */
            spi_ioc_tr.tx_buf = (unsigned long)spi_tx_dummy_buf;
            spi_ioc_tr.len = SPI_HEADER_LEN;
            ret_val = ioctl(fd, SPI_IOC_MESSAGE(1), &spi_ioc_tr);
            if (ret_val < 1)
            {
                mxt_log_err(mxt->ctx, "Error %s (%d) reading from spi", strerror(errno), errno);
                ret_val = mxt_errno_to_rc(errno);
                goto close;
            }
            if (SPI_WRITE_OK != spi_rx_buf[0])
            {
                mxt_log_err(mxt->ctx, "SPI_WRITE_OK != %.2X reading from spi", spi_rx_buf[0]);
                ret_val = MXT_ERROR_PROTOCOL_FAULT;
                goto close;
            }
            if (spi_tx_buf[1] != spi_rx_buf[1] || spi_tx_buf[2] != spi_rx_buf[2])
            {
                mxt_log_err(mxt->ctx, "Unexpected address %d != %d reading from spi", spi_rx_buf[1] | (spi_rx_buf[2] << 8), address_iter);
                ret_val = MXT_ERROR_PROTOCOL_FAULT;
                goto close;
            }
            if (spi_tx_buf[3] != spi_rx_buf[3] || spi_tx_buf[4] != spi_rx_buf[4])
            {
                mxt_log_err(mxt->ctx, "Unexpected count %d != %d reading from spi", spi_rx_buf[3] | (spi_rx_buf[4] << 8), count_iter);
                ret_val = MXT_ERROR_PROTOCOL_FAULT;
                goto close;
            }
        }
        while (get_header_crc(spi_rx_buf) != spi_rx_buf[SPI_HEADER_LEN-1]);

        offset += count_iter;
    }

    ret_val = MXT_SUCCESS;

close:
    close(fd);
    return ret_val;
}

int spi_dev_bootloader_read(struct mxt_device *mxt, uint8_t *buf, uint16_t count)
{
    return MXT_SUCCESS;
}

int spi_dev_bootloader_write(struct mxt_device *mxt, uint8_t const *buf, uint16_t count, size_t *bytes_transferred)
{
    return MXT_SUCCESS;
}

int spi_dev_bootloader_write_blks(struct mxt_device *mxt, unsigned char const *buf, int count)
{
    int ret;
    size_t received;
    int off = 0;

    while (off < count)
    {
        ret = spi_dev_bootloader_write(mxt, buf + off, count - off, &received);
        if (ret)
        {
            return ret;
        }

        off += received;
    }

    return MXT_SUCCESS;
}
