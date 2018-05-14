/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * gcc spidev_test.c -o spidev_test
 * 
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <getopt.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MAX_BUF 64
#define USE_GPIOS

#ifdef USE_GPIOS
#define GPIO_SS  60
#define GPIO_CHG 48
#endif //USE_GPIOS

// opcodes
#define SPI_WRITE_REQ    0x01
#define SPI_WRITE_OK     0x81
#define SPI_WRITE_FAIL   0x41
#define SPI_READ_REQ     0x02
#define SPI_READ_OK      0x82
#define SPI_READ_FAIL    0x42
#define SPI_INVALID_REQ  0x04
#define SPI_INVALID_CRC  0x08

// header 6 bytes + Data[]
// 0 opcode
// 1 address LSB
// 2 address MSB
// 3 length LSB
// 4 length MSB
// 5 CRC
// 6+ Data[]

#define SPI_HEADER_LEN 6
#define MXT_INFOBLOCK_LEN 7

static void pabort(const char *s)
{
    perror(s);
    abort();
}

static const char *device = "/dev/spidev1.0";
static uint32_t spi_mode32 = SPI_CPHA | SPI_CPOL;
static uint8_t spi_bits_per_word = 8;
static uint32_t spi_max_speed_hz = 8000000;
static uint16_t delay;

uint8_t default_tx[SPI_HEADER_LEN] = {
// read 7 bytes from address 0
    SPI_READ_REQ, 0x00, 0x00, 0x07, 0x00, 0xFF,
};

uint8_t default_rx[SPI_HEADER_LEN + MXT_INFOBLOCK_LEN];
uint8_t default_dummy_tx[SPI_HEADER_LEN + MXT_INFOBLOCK_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
char *input_tx;

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

#ifdef USE_GPIOS
int set_gpio_direction(unsigned int gpio, const char *dir_in_or_out)
{
    int fd, ret_val, tx_len;
    char buf[MAX_BUF];
    snprintf(buf, MAX_BUF, "/sys/class/gpio/gpio%d/direction", gpio);
    fd = open(buf, O_WRONLY);
    if (fd < 0)
    {
        perror("gpio/direction");
        return fd;
    }

    tx_len = 1 + strlen(dir_in_or_out);
    ret_val = write(fd, dir_in_or_out, tx_len);
    ret_val = tx_len == ret_val ? 0 : -1;

    close(fd);
    return ret_val;
}
static int get_gpio_value(unsigned int gpio)
{
    int fd, ret_val;
    char buf[MAX_BUF];
    char ch;

    snprintf(buf, MAX_BUF, "/sys/class/gpio/gpio%d/value", gpio);

    fd = open(buf, O_RDONLY);
    if (fd < 0)
    {
        perror("gpio/get-value");
        return fd;
    }

    ret_val = read(fd, &ch, 1);
    if (1 == ret_val)
    {
        ret_val = ch != '0' ? 1 : 0;
    }
    else
    {
        ret_val = -1;
    }

    close(fd);
    return ret_val;
}
static int set_gpio_value(unsigned int gpio, int value)
{
    int fd, ret_val;
    char buf[MAX_BUF];

    snprintf(buf, MAX_BUF, "/sys/class/gpio/gpio%d/value", gpio);

    fd = open(buf, O_WRONLY);
    if (fd < 0)
    {
        perror("gpio/set-value");
        return fd;
    }

    ret_val = write(fd, value ? "1" : "0", 2);
    ret_val = 2 == ret_val ? 0 : -1;

    close(fd);
    return ret_val;
}
#endif //USE_GPIOS

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

#ifdef USE_GPIOS
static int mxt_wait_for_chg()
{
    int timeout_counter = 500; // 10 msec

    while (0 != get_gpio_value(GPIO_CHG) && timeout_counter > 0)
    {
        timeout_counter--;
        usleep(20);
    }

    return timeout_counter > 0 ? 0 : -1;
}
#endif //USE_GPIOS

static void check_pending_messages(int fd, uint8_t *rx, size_t len_rx)
{
    int ret_val, attempt;
    struct spi_ioc_transfer tr = {
        .rx_buf = (unsigned long)rx,
        .tx_buf = (unsigned long)default_dummy_tx,
        .len = len_rx,
        .delay_usecs = delay,
    };

#ifdef USE_GPIOS
    while (0 == get_gpio_value(GPIO_CHG))
    {
        attempt = 0;
        do
        {
            attempt++;
            if (attempt > 1)
            {
                if (attempt > 5)
                {
                    printf("Too many Retries\n");
                    break;
                }
                printf("Retry %d after CRC fail\n", attempt-1);
            }
            if (0 != set_gpio_value(GPIO_SS, 0))
            {
                pabort("can't set gpio val");
            }
            /* READ */
            memset(rx, 0, sizeof(rx));
            ret_val = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
            if (ret_val < 1)
            {
                pabort("can't recv spi message");
            }
            if (0 != set_gpio_value(GPIO_SS, 1))
            {
                pabort("can't set gpio val");
            }
            hex_dump(default_dummy_tx, len_rx, 32, "TX");
            hex_dump(rx, len_rx, 32, "RX");
            if (0xff == rx[0])
            {
                // no msg
                break;
            }
        }
        while (get_header_crc(rx) != rx[SPI_HEADER_LEN-1]);
        if (0xff == rx[0])
        {
            // no msg
            break;
        }
    }
#endif //USE_GPIOS
}

static void transfer(int fd, uint8_t const *tx, uint8_t *rx, size_t len_tx, size_t len_rx)
{
    int ret_val, attempt=0;
    struct spi_ioc_transfer tr = {
        .rx_buf = (unsigned long)rx,
        .delay_usecs = delay,
    };

    do
    {
        attempt++;
        if (attempt > 1)
        {
            if (attempt > 5)
            {
                printf("Too many Retries\n");
                break;
            }
            printf("Retry %d after CRC fail\n", attempt-1);
        }
#ifdef USE_GPIOS
        if (0 != set_gpio_value(GPIO_SS, 0))
        {
            pabort("can't set gpio val");
        }
#endif //USE_GPIOS
        /* WRITE */
        tr.len = len_tx;
        tr.tx_buf = (unsigned long)tx;
        memset(rx, 0, sizeof(rx));
        ret_val = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret_val < 1)
        {
            pabort("can't send spi message");
        }
#ifdef USE_GPIOS
        if (0 != set_gpio_value(GPIO_SS, 1))
        {
            pabort("can't set gpio val");
        }
        if (0 != mxt_wait_for_chg())
        {
            pabort("wait for CHG timeout\n");
        }
        if (0 != set_gpio_value(GPIO_SS, 0))
        {
            pabort("can't set gpio val");
        }
#endif //USE_GPIOS
        /* READ */
        tr.len = len_rx;
        tr.tx_buf = (unsigned long)default_dummy_tx;
        memset(rx, 0, sizeof(rx));
        ret_val = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret_val < 1)
        {
            pabort("can't recv spi message");
        }
#ifdef USE_GPIOS
        if (0 != set_gpio_value(GPIO_SS, 1))
        {
            pabort("can't set gpio val");
        }
#endif //USE_GPIOS

        hex_dump(tx, len_tx, 32, "TX");
        hex_dump(rx, len_rx, 32, "RX");
        //printf("calc_crc = %02X\n", get_header_crc(rx));
    }
    while (get_header_crc(rx) != rx[SPI_HEADER_LEN-1]);
}

static void print_usage(const char *prog)
{
    printf("Usage: %s [-DsbdlHOLC3]\n", prog);
    puts("  -D --device   device to use (default /dev/spidev1.1)\n"
         "  -s --speed    max speed (Hz)\n"
         "  -d --delay    delay (usec)\n"
         "  -b --bpw      bits per word\n"
         "  -l --loop     loopback\n"
         "  -H --cpha     clock phase\n"
         "  -O --cpol     clock polarity\n"
         "  -L --lsb      least significant bit first\n"
         "  -C --cs-high  chip select active high\n"
         "  -3 --3wire    SI/SO signals shared\n"
         "  -p            Send data (e.g. \"1234\\xde\\xad\")\n"
         "  -N --no-cs    no chip select\n"
         "  -R --ready    slave pulls low to pause\n");
    exit(1);
}

static void parse_opts(int argc, char *argv[])
{
    while (1) {
        static const struct option lopts[] = {
            { "device",  1, 0, 'D' },
            { "speed",   1, 0, 's' },
            { "delay",   1, 0, 'd' },
            { "bpw",     1, 0, 'b' },
            { "input",   1, 0, 'i' },
            { "output",  1, 0, 'o' },
            { "loop",    0, 0, 'l' },
            { "cpha",    0, 0, 'H' },
            { "cpol",    0, 0, 'O' },
            { "lsb",     0, 0, 'L' },
            { "cs-high", 0, 0, 'C' },
            { "3wire",   0, 0, '3' },
            { "no-cs",   0, 0, 'N' },
            { "ready",   0, 0, 'R' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "D:s:d:b:i:o:lHOLC3NR24p:v",
                lopts, NULL);

        if (c == -1)
            break;

        switch (c) {
        case 'D':
            device = optarg;
            break;
        case 's':
            spi_max_speed_hz = atoi(optarg);
            break;
        case 'd':
            delay = atoi(optarg);
            break;
        case 'b':
            spi_bits_per_word = atoi(optarg);
            break;
        case 'l':
            spi_mode32 |= SPI_LOOP;
            break;
        case 'H':
            spi_mode32 |= SPI_CPHA;
            break;
        case 'O':
            spi_mode32 |= SPI_CPOL;
            break;
        case 'L':
            spi_mode32 |= SPI_LSB_FIRST;
            break;
        case 'C':
            spi_mode32 |= SPI_CS_HIGH;
            break;
        case '3':
            spi_mode32 |= SPI_3WIRE;
            break;
        case 'N':
            spi_mode32 |= SPI_NO_CS;
            break;
        case 'R':
            spi_mode32 |= SPI_READY;
            break;
        case 'p':
            input_tx = optarg;
            break;
        default:
            print_usage(argv[0]);
            break;
        }
    }
}

int main(int argc, char *argv[])
{
    int ret_val = 0;
    int fd;

#ifdef USE_GPIOS
    if (0 != set_gpio_direction(GPIO_SS, "out") || 0 != set_gpio_direction(GPIO_CHG, "in"))
    {
        pabort("can't set gpio dir");
    }
    if (0 != set_gpio_value(GPIO_SS, 1))
    {
        pabort("can't set gpio val");
    }
#endif //USE_GPIOS

    parse_opts(argc, argv);

    fd = open(device, O_RDWR);
    if (fd < 0)
        pabort("can't open device");

    /*
     * spi_mode32
     */
    ret_val = ioctl(fd, SPI_IOC_WR_MODE32, &spi_mode32);
    if (ret_val == -1)
        pabort("can't set spi spi_mode32");

    ret_val = ioctl(fd, SPI_IOC_RD_MODE32, &spi_mode32);
    if (ret_val == -1)
        pabort("can't get spi spi_mode32");

    /*
     * spi_bits_per_word
     */
    ret_val = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (ret_val == -1)
        pabort("can't set bits per word");

    ret_val = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (ret_val == -1)
        pabort("can't get bits per word");

    /*
     * spi_max_speed_hz
     */
    ret_val = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_max_speed_hz);
    if (ret_val == -1)
        pabort("can't set max speed hz");

    ret_val = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_max_speed_hz);
    if (ret_val == -1)
        pabort("can't get max speed hz");

    printf("device: %s\n", device);
    printf("spi_mode32: 0x%x\n", spi_mode32);
    printf("spi_bits_per_word: %d\n", spi_bits_per_word);
    printf("spi_max_speed_hz: %d (%d KHz)\n", spi_max_speed_hz, spi_max_speed_hz/1000);

#ifdef USE_GPIOS
    check_pending_messages(fd, default_rx, sizeof(default_rx));
#endif //USE_GPIOS

    default_tx[SPI_HEADER_LEN-1] = get_header_crc(default_tx);
    transfer(fd, default_tx, default_rx, sizeof(default_tx), sizeof(default_rx));

#ifdef USE_GPIOS
    check_pending_messages(fd, default_rx, sizeof(default_rx));
#endif //USE_GPIOS

    close(fd);

    return ret_val;
}
