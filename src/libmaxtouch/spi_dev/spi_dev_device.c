//------------------------------------------------------------------------------
// Copyright 2018 Solomon Systech UL Ltd. All rights reserved.
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

#define SPI_HEADER_LEN  6

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

static int open_device(struct mxt_device *mxt, int *fd_out)
{
    int fd;
    int ret_val;
    char filename[20];

    snprintf(filename, 20, "/dev/spidev%d.%d", mxt->conn->spi_dev.bus, mxt->conn->spi_dev.chipselect);
    fd = open(filename, O_RDWR);
    if (fd < 0)
    {
        mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
        return mxt_errno_to_rc(errno);
    }

    ret_val = ioctl(fd, SPI_IOC_WR_MODE32, &spi_mode32);
    if (ret_val == -1)
    {
        mxt_err(mxt->ctx, "can't set spi spi_mode32, error %s (%d)", strerror(errno), errno);
        close(fd);
        return mxt_errno_to_rc(errno);
    }
    ret_val = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (ret_val == -1)
    {
        mxt_err(mxt->ctx, "can't set bits per word, error %s (%d)", strerror(errno), errno);
        close(fd);
        return mxt_errno_to_rc(errno);
    }
    ret_val = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_max_speed_hz);
    if (ret_val == -1)
    {
        mxt_err(mxt->ctx, "can't set max speed hz, error %s (%d)", strerror(errno), errno);
        close(fd);
        return mxt_errno_to_rc(errno);
    }

    *fd_out = fd;
    return MXT_SUCCESS;
}

int spi_dev_read_register(struct mxt_device *mxt,
                          unsigned char *buf,
                          int start_register,
                          int count,
                          size_t *bytes_read)
{
    return MXT_SUCCESS;
}

int spi_dev_write_register(struct mxt_device *mxt, unsigned char const *buf, int start_register, size_t count)
{
    return MXT_SUCCESS;
}

int spi_dev_bootloader_read(struct mxt_device *mxt, unsigned char *buf, int count)
{
    return MXT_SUCCESS;
}

int spi_dev_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, int count, size_t *bytes_transferred)
{
    return MXT_SUCCESS;
}
