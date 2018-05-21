//------------------------------------------------------------------------------
/// \file   bootloader.c
/// \brief  Bootloader functions
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

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/sysfs/sysfs_device.h"

#ifdef HAVE_LIBUSB
#include "libmaxtouch/usb/usb_device.h"
#endif

#include "mxt_app.h"

#define MXT_UNLOCK_CMD_BYTE0     0xdc
#define MXT_UNLOCK_CMD_BYTE1     0xaa

/* Bootloader mode status bits 7 6 */
#define MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD 0xc0
#define MXT_BOOT_STATUS_WAITING_FRAME_DATA   0x80
#define MXT_BOOT_STATUS_WAITINGS_MASK        0x3f

/* Bootloader mode status bits 7..0 */
#define MXT_BOOT_STATUS_FRAME_CRC_CHECK      0x02
#define MXT_BOOT_STATUS_FRAME_CRC_FAIL       0x03
#define MXT_BOOT_STATUS_FRAME_CRC_PASS       0x04

#define FIRMWARE_BUFFER_SIZE     1024

#define MXT_RESET_TIME           2

//******************************************************************************
/// \brief Bootloader context object
struct flash_context
{
    struct mxt_device *mxt;
    struct mxt_conn_info *conn;
    struct libmaxtouch_ctx *ctx;
    FILE *fp;
    long file_size;
    char curr_version[MXT_FW_VER_LEN];
    int i2c_adapter;
    int appmode_address;
    int bootloader_address;
    bool check_version;
    const char *new_version;
    bool usb_bootloader;
};

//******************************************************************************
/// \brief Send a frame with length field set to 0x0000. This should force a
///        bootloader reset
/// \return #mxt_rc
static int mxt_send_bootloader_reset_cmd(struct flash_context *fw)
{
    unsigned char buf[2] = {0, 0};

    return mxt_bootloader_write(fw->mxt, buf, 2);
}

//******************************************************************************
/// \brief Send command to unlock bootloader
/// \return #mxt_rc
static int mxt_send_bootloader_unlock_cmd(struct flash_context *fw)
{
    unsigned char buf[2] = {MXT_UNLOCK_CMD_BYTE0, MXT_UNLOCK_CMD_BYTE1};

    return mxt_bootloader_write(fw->mxt, buf, 2);
}

//******************************************************************************
/// \brief Get printable bootloader status
static const char * get_bootl_state_str(uint8_t bootl_state)
{
    switch(bootl_state)
    {
        case MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD: return "WAITING_BOOTLOAD_CMD";
        case MXT_BOOT_STATUS_WAITING_FRAME_DATA: return "WAITING_FRAME_DATA";
        case MXT_BOOT_STATUS_FRAME_CRC_CHECK: return "FRAME_CRC_CHECK";
        case MXT_BOOT_STATUS_FRAME_CRC_FAIL: return "FRAME_CRC_FAIL";
        case MXT_BOOT_STATUS_FRAME_CRC_PASS: return "FRAME_CRC_PASS";
        default:
            break;
    }
    return "?";
}

//******************************************************************************
/// \brief Read bootloader state
/// \return #mxt_rc
static int mxt_check_bootloader(struct flash_context *fw, unsigned int expected_next_state)
{
    uint8_t status_byte;
    int ret_val;

recheck:
    if (MXT_SUCCESS != mxt_wait_for_chg(fw->mxt))
    {
        mxt_log_err(fw->ctx, "BOOTL Timeout on CHG expected_next_state 0x%.2x %s", expected_next_state, get_bootl_state_str(expected_next_state));
    }
    ret_val = mxt_bootloader_read(fw->mxt, &status_byte, 1);
    if (MXT_SUCCESS != ret_val)
    {
        mxt_log_err(fw->ctx, "Bootloader read FAIL");
        return ret_val;
    }
    mxt_log_verb(fw->ctx, "Bootloader status_byte 0x%02X", status_byte);

    if ( (MXT_BOOT_STATUS_WAITING_FRAME_DATA == expected_next_state) ||
         (MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD == expected_next_state) )
    {
        status_byte &= ~MXT_BOOT_STATUS_WAITINGS_MASK;
    }
    else if (MXT_BOOT_STATUS_FRAME_CRC_PASS == expected_next_state)
    {
        if (MXT_BOOT_STATUS_FRAME_CRC_FAIL == status_byte)
        {
            return MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL;
        }
        if (MXT_BOOT_STATUS_FRAME_CRC_CHECK == status_byte)
        {
            goto recheck;
        }
    }
    else
    {
        return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
    }

    if (status_byte != expected_next_state)
    {
        mxt_log_info(fw->ctx, "Invalid bootloader mode state 0x%.2X %s", status_byte, get_bootl_state_str(status_byte));
        if ( (MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD == expected_next_state) &&
             (MXT_BOOT_STATUS_WAITING_FRAME_DATA == status_byte) )
        {
            return MXT_ERROR_BOOTLOADER_UNLOCKED;
        }
        if (MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD == expected_next_state)
        {
            mxt_send_bootloader_reset_cmd(fw);
        }
        return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
    }

    return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read hexadecimal value from file
static int get_hex_value(struct flash_context *fw, unsigned char *ptr)
{
    char str[3];
    int val;

    if (2 != fread(str, 1/*size of one element in bytes*/, 2/*num of elements*/, fw->fp))
    {
        return EOF;
    }
    str[2] = 0;

    if (1 != sscanf(str, "%x", &val))
    {
        mxt_log_err(fw->ctx, "sscanf");
        return EOF;
    }

    *ptr = val;

    return 0;
}

//******************************************************************************
/// \brief Send firmware frames to bootloader
/// \return #mxt_rc
static int mxt_send_frames(struct flash_context *fw)
{
    unsigned char buffer[FIRMWARE_BUFFER_SIZE];
    uint8_t last_percent = 100;
    uint8_t curr_percent;
    int ret_val;
    int i;
    int frame_size = 0;
    int frame_count = 1;
    int frame_retry = 0;
    int bytes_sent = 0;

    ret_val = mxt_check_bootloader(fw, MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD);
    if (MXT_SUCCESS == ret_val)
    {
        mxt_log_info(fw->ctx, "Unlocking bootloader");
        ret_val = mxt_send_bootloader_unlock_cmd(fw);
        if (MXT_SUCCESS != ret_val)
        {
            mxt_log_err(fw->ctx, "Failure to unlock bootloader");
            return ret_val;
        }
        mxt_log_info(fw->ctx, "Bootloader unlocked");
    }
    else if (MXT_ERROR_BOOTLOADER_UNLOCKED == ret_val)
    {
        mxt_log_info(fw->ctx, "Bootloader already unlocked");
    }
    else
    {
        mxt_log_err(fw->ctx, "Bootloader not found");
        return MXT_ERROR_NO_DEVICE;
    }

    mxt_log_info(fw->ctx, "Flashing device...");

    while (!feof(fw->fp))
    {
        if (0 == frame_retry)
        {
            if (EOF == get_hex_value(fw, &buffer[0]))
            {
                mxt_log_info(fw->ctx, "End of file");
                break;
            }

            if (EOF == get_hex_value(fw, &buffer[1]))
            {
                mxt_log_err(fw->ctx, "Unexpected end of firmware file");
                return MXT_ERROR_FILE_FORMAT;
            }

            frame_size = (buffer[0] << 8) | buffer[1];

            mxt_log_dbg(fw->ctx, "Frame %d: size %d", frame_count, frame_size);

            /* Allow for CRC bytes at end of frame */
            frame_size += 2;

            if (frame_size > FIRMWARE_BUFFER_SIZE)
            {
                mxt_log_err(fw->ctx, "Frame too big");
                return MXT_ERROR_NO_MEM;
            }

            for (i = 2; i < frame_size; i++)
            {
                ret_val = get_hex_value(fw, &buffer[i]);

                if (EOF == ret_val)
                {
                    mxt_log_err(fw->ctx, "Unexpected end of firmware file");
                    return MXT_ERROR_FILE_FORMAT;
                }
            }
        }

        if (MXT_SUCCESS != mxt_check_bootloader(fw, MXT_BOOT_STATUS_WAITING_FRAME_DATA))
        {
            mxt_log_err(fw->ctx, "Unexpected bootloader state");
            return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
        }

        /* Write one frame to device */
        ret_val = mxt_bootloader_write(fw->mxt, buffer, frame_size);
        if (MXT_SUCCESS != ret_val)
        {
            return ret_val;
        }

        // Check CRC
        mxt_log_verb(fw->ctx, "Checking CRC");
        ret_val = mxt_check_bootloader(fw, MXT_BOOT_STATUS_FRAME_CRC_PASS);
        if (MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL == ret_val)
        {
            if (frame_retry > 0)
            {
                mxt_log_err(fw->ctx, "Failure sending frame %d - aborting", frame_count);
                return MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL;
            }
            frame_retry++;
            mxt_log_err(fw->ctx, "Frame %d: CRC fail, retry %d", frame_count, frame_retry);
        }
        else if (MXT_SUCCESS != ret_val)
        {
            mxt_log_err(fw->ctx, "Unexpected bootloader state");
            return ret_val;
        }
        else
        {
            mxt_log_verb(fw->ctx, "CRC pass");
            frame_retry = 0;
            frame_count++;
            bytes_sent += frame_size;
            curr_percent = (unsigned char)(0.5f + (100.0 * ftell(fw->fp)) / fw->file_size);

            if (curr_percent % 10 == 0)
            {
                /* No need to repeat for the same percentage */
                if (last_percent != curr_percent)
                {
                    /* clear previous line after first progress report */
                    if (curr_percent > 0)
                    {
                        mxt_log_info(fw->ctx, "%3d%%", curr_percent);
                    }
                    mxt_log_dbg(fw->ctx, "Sent %d frames, %d bytes. % 3d%%", frame_count, bytes_sent, curr_percent);
                    last_percent = curr_percent;
                }
            }
        }
    }
    mxt_log_info(fw->ctx, "Sent %d frames, %d bytes", frame_count, bytes_sent);

    fclose(fw->fp);

    return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Lookup bootloader I2C address
static int lookup_bootloader_addr(struct flash_context *fw, int addr)
{
    switch (addr)
    {
        case 0x4a:
        case 0x4b:
            if (fw->mxt->info.id->family >= 0xa2)
            {
                return (addr - 0x24);
            }
        /* Fall through for normal case */
        case 0x4c:
        case 0x4d:
        case 0x5a:
        case 0x5b:
            return (addr - 0x26);
            break;
        default:
            return -1;
    }
}

//******************************************************************************
/// \brief Initialise chip in bootloader mode
/// \return #mxt_rc
static int mxt_bootloader_init_chip(struct flash_context *fw)
{
    int ret;

    if (!fw->conn)
    {
        ret = mxt_scan(fw->ctx, &fw->conn, false);
        if (ret)
        {
            mxt_log_info(fw->ctx, "Could not find a device");
            return ret;
        }
    }

    switch (fw->conn->type)
    {
        case E_SYSFS:
            mxt_log_info(fw->ctx, "Switching to i2c-dev mode");

            struct mxt_conn_info *new_conn;
            ret = mxt_new_conn(&new_conn, E_I2C_DEV);
            if (ret)
            {
                return ret;
            }

            ret = sysfs_get_i2c_address(fw->ctx, fw->conn,
                                        &fw->i2c_adapter, &fw->appmode_address);
            if (ret)
            {
                return ret;
            }

            new_conn->i2c_dev.adapter = fw->i2c_adapter;
            new_conn->i2c_dev.address = fw->appmode_address;

            mxt_unref_conn(fw->conn);
            fw->conn = new_conn;
            break;

#ifdef HAVE_LIBUSB
        case E_USB:
            break;
#endif

        case E_I2C_DEV:
            if (fw->conn->i2c_dev.address < 0x4a)
            {
                mxt_log_info(fw->ctx, "Using bootloader address");
                fw->appmode_address = -1;
                return MXT_DEVICE_IN_BOOTLOADER;
            }
            break;

        case E_SPI_DEV:
            break;

        case E_HIDRAW:
        default:
            mxt_log_err(fw->ctx, "Device type not supported");
            return MXT_ERROR_NOT_SUPPORTED;
    }

    ret = mxt_new_device(fw->ctx, fw->conn, &fw->mxt);
    if (ret)
    {
        mxt_log_err(fw->ctx, "Could not open device");
        return ret;
    }

#ifdef HAVE_LIBUSB
    if (fw->conn->type == E_USB && usb_is_bootloader(fw->mxt))
    {
        mxt_log_info(fw->ctx, "USB device in bootloader mode");
        fw->usb_bootloader = true;
        mxt_free_device(fw->mxt);
        return MXT_DEVICE_IN_BOOTLOADER;
    }
    else
    {
        fw->usb_bootloader = false;
    }
#endif

    ret = mxt_get_info(fw->mxt);
    if (ret)
    {
        mxt_log_err(fw->ctx, "Could not get info block");
        return ret;
    }

    mxt_log_info(fw->ctx, "Chip detected");

    return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Test firmware to flash against current firmware
/// \return #mxt_rc
static int mxt_check_firmware_version(struct flash_context *fw)
{
    mxt_get_firmware_version(fw->mxt, fw->curr_version);
    mxt_log_info(fw->ctx, "Current firmware version: %s", fw->curr_version);

    if (!strcmp(fw->curr_version, fw->new_version))
    {

        mxt_log_info(fw->ctx, "Version already %s, exiting", fw->curr_version);
        return MXT_FIRMWARE_UPDATE_NOT_REQUIRED;
    }

    return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Reset into bootloader mode
/// \return #mxt_rc
static int mxt_enter_bootloader_mode(struct flash_context *fw)
{
    int ret;
    /* Change to the bootloader mode */
    ret = mxt_reset_chip(fw->mxt, true);
    if (ret)
    {
        mxt_log_err(fw->ctx, "Reset failure - aborting");
        return ret;
    }
    else
    {
        sleep(MXT_RESET_TIME);
    }

    if (fw->conn->type == E_I2C_DEV)
    {
        fw->appmode_address = fw->conn->i2c_dev.address;

        fw->conn->i2c_dev.address = lookup_bootloader_addr(fw, fw->appmode_address);
        if (fw->conn->i2c_dev.address == -1)
        {
            mxt_log_err(fw->ctx, "No bootloader address!");
            return MXT_ERROR_BOOTLOADER_NO_ADDRESS;
        }

        mxt_log_dbg(fw->ctx, "I2C Adapter:%d", fw->conn->i2c_dev.adapter);
        mxt_log_dbg(fw->ctx, "Bootloader addr:0x%02x", fw->conn->i2c_dev.address);
        mxt_log_dbg(fw->ctx, "App mode addr:0x%02x", fw->appmode_address);
    }

    mxt_free_device(fw->mxt);

    return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Flash firmware to chip
int mxt_flash_firmware(struct libmaxtouch_ctx *ctx,
                       struct mxt_device *maxtouch,
                       const char *filename, const char *new_version,
                       struct mxt_conn_info *conn)
{
    struct flash_context fw = { 0 };
    int ret;

    fw.ctx = ctx;
    fw.mxt = maxtouch;
    fw.conn = conn;

    mxt_log_info(fw.ctx, "Opening firmware file %s", filename);

    fw.fp = fopen(filename, "r");
    if (!fw.fp)
    {
        mxt_log_err(fw.ctx, "Cannot open firmware file %s!", filename);
        return mxt_errno_to_rc(errno);
    }
    fseek(fw.fp, 0L, SEEK_END);
    fw.file_size = ftell(fw.fp);
    rewind(fw.fp);

    ret = mxt_bootloader_init_chip(&fw);
    if (ret && (ret != MXT_DEVICE_IN_BOOTLOADER))
    {
        return ret;
    }

    if (ret != MXT_DEVICE_IN_BOOTLOADER)
    {
        if (strlen(new_version) > 0)
        {
            fw.check_version = true;
            fw.new_version = new_version;
            mxt_log_dbg(fw.ctx, "New firmware version is:%s", fw.new_version);
            ret = mxt_check_firmware_version(&fw);
            if (ret)
            {
                goto release;
            }
        }
        else
        {
            fw.check_version = false;
            mxt_log_dbg(fw.ctx, "check_version:%d", fw.check_version);
        }

        ret = mxt_enter_bootloader_mode(&fw);
        if (ret)
        {
            mxt_log_err(fw.ctx, "Could not enter bootloader mode");
            goto release;
        }
    }

    ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
    if (ret)
    {
        mxt_log_info(fw.ctx, "Could not initialise chip");
        return ret;
    }

    ret = mxt_send_frames(&fw);
    if (ret)
    {
        return ret;
    }

    /* Handle transition back to appmode address */
    if (fw.mxt->conn->type == E_I2C_DEV)
    {
        sleep(MXT_RESET_TIME);

        if (fw.appmode_address < 0)
        {
            mxt_log_info(fw.ctx, "Sent all firmware frames");
            ret = 0;
            goto release;
        }
        else
        {
            mxt_log_info(fw.ctx, "Switching back to app mode");
            struct mxt_conn_info *new_conn;
            ret = mxt_new_conn(&new_conn, E_I2C_DEV);
            if (ret)
            {
                return ret;
            }

            new_conn->i2c_dev.adapter = fw.i2c_adapter;
            new_conn->i2c_dev.address = fw.appmode_address;

            mxt_unref_conn(fw.conn);
            fw.conn = new_conn;
        }
    }
#ifdef HAVE_LIBUSB
    else if (fw.mxt->conn->type == E_USB)
    {
        bool bus_devices[USB_MAX_BUS_DEVICES] = { 0 };
        int tries = 10;

        ret = usb_find_bus_devices(fw.mxt, bus_devices);
        if (ret)
        {
            return ret;
        }

        while (tries--)
        {
            sleep(MXT_RESET_TIME);

            ret = usb_rediscover_device(fw.mxt, bus_devices);
            if (ret == MXT_SUCCESS)
            {
                break;
            }
        }

        if (ret)
        {
            mxt_log_err(fw.ctx, "Did not find device after reset");
            return ret;
        }
    }
#endif

    mxt_free_device(fw.mxt);

    ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
    if (ret)
    {
        mxt_log_err(fw.ctx, "FAILURE - chip did not reset");
        return MXT_ERROR_RESET_FAILURE;
    }

#ifdef HAVE_LIBUSB
    if (fw.mxt->conn->type == E_USB && usb_is_bootloader(fw.mxt))
    {
        mxt_log_err(fw.ctx, "USB device still in bootloader mode");
        ret = MXT_ERROR_RESET_FAILURE;
        goto release;
    }
#endif

    ret = mxt_get_info(fw.mxt);
    if (ret)
    {
        mxt_log_err(fw.ctx, "Failed to get info block");
        goto release;
    }

    mxt_get_firmware_version(fw.mxt, (char *)&fw.curr_version);

    if (!fw.check_version)
    {
        mxt_log_info(fw.ctx, "SUCCESS - version is %s", fw.curr_version);
        ret = MXT_SUCCESS;
        goto release;
    }


    if (!strcmp(fw.curr_version, fw.new_version))
    {
        mxt_log_info(fw.ctx, "SUCCESS - version %s verified", fw.curr_version);
        ret = MXT_SUCCESS;
    }
    else
    {
        mxt_log_err(fw.ctx, "FAILURE - detected version is %s", fw.curr_version);
        ret = MXT_ERROR_FIRMWARE_UPDATE_FAILED;
    }

release:
    mxt_free_device(fw.mxt);
    mxt_unref_conn(fw.conn);
    return ret;
}

//******************************************************************************
/// \brief  Bootloader version query
int mxt_bootloader_version(struct libmaxtouch_ctx *ctx, struct mxt_device *mxt, struct mxt_conn_info *conn)
{
    struct flash_context fw = {0};
    int ret;
    unsigned char buf[3];

    fw.ctx = ctx;
    fw.mxt = mxt;
    fw.conn = conn;

    ret = mxt_bootloader_init_chip(&fw);
    if (ret && ret != MXT_DEVICE_IN_BOOTLOADER)
    {
        mxt_log_err(fw.ctx, "Could not init device");
        return ret;
    }

    if (ret != MXT_DEVICE_IN_BOOTLOADER)
    {
        ret = mxt_enter_bootloader_mode(&fw);
        if (ret)
        {
            mxt_log_err(fw.ctx, "Could not enter bootloader mode");
            return ret;
        }
    }

    ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
    if (ret)
    {
        mxt_log_err(fw.ctx, "Could not open device");
        return ret;
    }

    ret = mxt_check_bootloader(&fw, MXT_BOOT_STATUS_WAITING_BOOTLOAD_CMD);
    if (ret)
    {
        goto release;
    }

    ret = mxt_bootloader_read(fw.mxt, buf, sizeof(buf));
    if (ret)
    {
        goto release;
    }

    printf("Bootloader ID:%d Version:%d\n", (buf[1] & 0x1f), buf[2]);

release:
    mxt_log_info(fw.ctx, "Reset into app mode");
    mxt_send_bootloader_reset_cmd(&fw);

    mxt_free_device(fw.mxt);
    mxt_unref_conn(fw.conn);
    return ret;
}
