//------------------------------------------------------------------------------
/// \file   i2c_dev_device.c
/// \brief  MXT device low level access via i2c-dev interface
/// \author Nick Dyer
//------------------------------------------------------------------------------
// Copyright 2011 Atmel Corporation. All rights reserved.
// Copyright 2018 Solomon Systech. All rights reserved.
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

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

struct mxt_device;
struct mxt_conn_info;

#include "i2c_dev_device.h"
#include "libmaxtouch/libmaxtouch.h"

#define I2C_SLAVE_FORCE 0x0706

/* Deep sleep retry delay 25 ms */
#define I2C_RETRY_DELAY 25000

//******************************************************************************
/// \brief  Open the i2c dev interface and set the slave address
/// \return #mxt_rc
static int open_and_set_slave_address(struct mxt_device *mxt, int *fd_out)
{
    int fd;
    int ret_val;
    char filename[20];

    snprintf(filename, 20, "/dev/i2c-%d", mxt->conn->i2c_dev.adapter);
    fd = open(filename, O_RDWR);
    if (fd < 0)
    {
        mxt_log_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
        return mxt_errno_to_rc(errno);
    }

    ret_val = ioctl(fd, I2C_SLAVE_FORCE, mxt->conn->i2c_dev.address);
    if (ret_val < 0)
    {
        mxt_log_err(mxt->ctx, "Error setting slave address, error %s (%d)", strerror(errno), errno);
        close(fd);
        return mxt_errno_to_rc(errno);
    }

    *fd_out = fd;
    return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return #mxt_rc
int i2c_dev_read_register(struct mxt_device *mxt,
                          unsigned char *buf,
                          int start_register,
                          int count,
                          size_t *bytes_read)
{
    int fd = -ENODEV;
    int ret_val;
    char register_buf[2];

    if (count > mxt->ctx->i2c_block_size)
    {
        count = mxt->ctx->i2c_block_size;
    }

    ret_val = open_and_set_slave_address(mxt, &fd);
    if (ret_val)
    {
        return ret_val;
    }

    register_buf[0] = start_register & 0xff;
    register_buf[1] = (start_register >> 8) & 0xff;

    if (write(fd, &register_buf, 2) != 2)
    {
        mxt_log_verb(mxt->ctx, "I2C retry");
        usleep(I2C_RETRY_DELAY);
        if (write(fd, &register_buf, 2) != 2)
        {
            mxt_log_err(mxt->ctx, "Error %s (%d) writing to i2c", strerror(errno), errno);
            ret_val = mxt_errno_to_rc(errno);
            goto close;
        }
    }

    ssize_t read_rc;
    read_rc = read(fd, buf, count);
    if (read_rc < 0)
    {
        mxt_log_err(mxt->ctx, "Error %s (%d) reading from i2c", strerror(errno), errno);
        ret_val = mxt_errno_to_rc(errno);
        goto close;
    }
    else if (read_rc == 0)
    {
        /* end of file */
        ret_val = MXT_ERROR_IO;
        goto close;
    }
    else
    {
        *bytes_read = (size_t)read_rc;
        ret_val = MXT_SUCCESS;
    }

close:
    close(fd);
    return ret_val;
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return #mxt_rc
int i2c_dev_write_register(struct mxt_device *mxt, unsigned char const *val,
                           int start_register, size_t datalength)
{
    int fd = -ENODEV;
    int count;
    int ret_val;
    unsigned char *buf;

    ret_val = open_and_set_slave_address(mxt, &fd);
    if (ret_val)
    {
        return ret_val;
    }

    count = datalength + 2;
    buf = (unsigned char *)calloc(count, sizeof(unsigned char));

    buf[0] = start_register & 0xff;
    buf[1] = (start_register >> 8) & 0xff;
    memcpy(buf + 2, val, datalength);

    if (write(fd, buf, count) != count)
    {
        mxt_log_verb(mxt->ctx, "I2C retry");
        usleep(I2C_RETRY_DELAY);
        if (write(fd, buf, count) != count)
        {
            mxt_log_err(mxt->ctx, "Error %s (%d) writing to i2c", strerror(errno), errno);
            ret_val = mxt_errno_to_rc(errno);
        }
        else
        {
            ret_val = MXT_SUCCESS;
        }
    }
    else
    {
        ret_val = MXT_SUCCESS;
    }

    free(buf);
    close(fd);
    return ret_val;
}

//******************************************************************************
/// \brief  Bootloader read
/// \return #mxt_rc
int i2c_dev_bootloader_read(struct mxt_device *mxt,
                            unsigned char *buf,
                            int count)
{
    int fd = -ENODEV;
    int ret_val;

    ret_val = open_and_set_slave_address(mxt, &fd);
    if (ret_val)
    {
        return ret_val;
    }

    mxt_log_dbg(mxt->ctx, "Reading %d bytes", count);

    if (read(fd, buf, count) != count)
    {
        mxt_log_err(mxt->ctx, "Error %s (%d) reading from i2c", strerror(errno), errno);
        ret_val = mxt_errno_to_rc(errno);
    }
    else
    {
        ret_val = MXT_SUCCESS;
    }

    close(fd);
    return ret_val;
}

//******************************************************************************
/// \brief  Bootloader write
/// \return #mxt_rc
static int i2c_dev_bootloader_write(struct mxt_device *mxt,
                                    unsigned char const *buf,
                                    int count,
                                    size_t *bytes_read,
                                    int fd)
{
    int ret_val;

    if (count > mxt->ctx->i2c_block_size)
    {
        count = mxt->ctx->i2c_block_size;
    }

    mxt_log_dbg(mxt->ctx, "I2C Writing %d bytes", count);

    if (write(fd, buf, count) != count)
    {
        mxt_log_err(mxt->ctx, "Error %s (%d) writing to i2c", strerror(errno), errno);
        ret_val = mxt_errno_to_rc(errno);
    }
    else
    {
        *bytes_read = count;
        ret_val = MXT_SUCCESS;
    }

    return ret_val;
}

//******************************************************************************
/// \brief Write to bootloader
/// \return #mxt_rc
int i2c_dev_bootloader_write_blks(struct mxt_device *mxt,
                                  unsigned char const *buf,
                                  int count)
{
    int ret_val;
    int offset = 0;
    int fd = -ENODEV;

    ret_val = open_and_set_slave_address(mxt, &fd);
    if (ret_val)
    {
        return ret_val;
    }

    while (offset < count)
    {
        size_t count_iter;
        ret_val = i2c_dev_bootloader_write(mxt,
                                           buf + offset,
                                           count - offset,
                                           &count_iter,
                                           fd);
        if (MXT_SUCCESS != ret_val)
        {
            break;
        }

        offset += count_iter;
    }

    close(fd);

    return ret_val;
}
