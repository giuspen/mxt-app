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

#define HEADER_LEN 6
#define INFOBLOCK_LEN 7

static void pabort(const char *s)
{
    perror(s);
    abort();
}

static const char *device = "/dev/spidev1.0";
static uint32_t mode = SPI_CPHA | SPI_CPOL;
static uint8_t bits = 8;
static uint32_t speed = 8000000;
static uint16_t delay;
static int verbose;

uint8_t default_tx[HEADER_LEN] = {
// read 7 bytes from address 0
    SPI_READ_REQ, 0x00, 0x00, 0x07, 0x00, 0xFF,
};

uint8_t default_rx[HEADER_LEN + INFOBLOCK_LEN];
uint8_t default_dummy_tx[HEADER_LEN + INFOBLOCK_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
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
    for (; i < HEADER_LEN-1; i++)
    {
        calc_crc = get_crc8_iter(calc_crc, p_msg[i]);
    }
    return calc_crc;
}

#ifdef USE_GPIOS
static int get_gpio_value(unsigned int gpio)
{
    int fd, ret;
    char buf[MAX_BUF];
    char ch;

    snprintf(buf, MAX_BUF, "/sys/class/gpio/gpio%d/value", gpio);

    fd = open(buf, O_RDONLY);
    if (fd < 0)
    {
        perror("gpio/get-value");
        return fd;
    }

    ret = read(fd, &ch, 1);
    if (1 == ret)
    {
        ret = ch != '0' ? 1 : 0;
    }
    else
    {
        ret = -1;
    }

    close(fd);
    return ret;
}
static int set_gpio_value(unsigned int gpio, int value)
{
    int fd, ret;
    char buf[MAX_BUF];

    snprintf(buf, MAX_BUF, "/sys/class/gpio/gpio%d/value", gpio);

    fd = open(buf, O_WRONLY);
    if (fd < 0)
    {
        perror("gpio/set-value");
        return fd;
    }

    ret = write(fd, value ? "1" : "0", 2);
    ret = 2 == ret ? 0 : -1;

    close(fd);
    return ret;
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

static void transfer(int fd, uint8_t const *tx, uint8_t *rx, size_t len_tx, size_t len_rx)
{
    int ret, attempt=0;
    struct spi_ioc_transfer tr = {
        .rx_buf = (unsigned long)rx,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    if (mode & SPI_TX_QUAD)
        tr.tx_nbits = 4;
    else if (mode & SPI_TX_DUAL)
        tr.tx_nbits = 2;
    if (mode & SPI_RX_QUAD)
        tr.rx_nbits = 4;
    else if (mode & SPI_RX_DUAL)
        tr.rx_nbits = 2;
    if (!(mode & SPI_LOOP)) {
        if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
            tr.rx_buf = 0;
        else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
            tr.tx_buf = 0;
    }

    do
    {
        attempt++;
        if (attempt > 1)
        {
            printf("Retry %d after CRC fail\n", attempt-1);
        }
#ifdef USE_GPIOS
        if (0 != set_gpio_value(GPIO_SS, 0))
        {
            pabort("can't set gpio");
        }
#endif //USE_GPIOS
        /* WRITE */
        tr.len = len_tx;
        tr.tx_buf = (unsigned long)tx;
        ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1)
        {
            pabort("can't send spi message");
        }
#ifdef USE_GPIOS
        if (0 != set_gpio_value(GPIO_SS, 1))
        {
            pabort("can't set gpio");
        }
        if (0 != mxt_wait_for_chg())
        {
            pabort("wait for CHG timeout\n");
        }
        if (0 != set_gpio_value(GPIO_SS, 0))
        {
            pabort("can't set gpio");
        }
#endif //USE_GPIOS
        /* READ */
        tr.len = len_rx;
        tr.tx_buf = (unsigned long)default_dummy_tx;
        ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1)
        {
            pabort("can't recv spi message");
        }
#ifdef USE_GPIOS
        if (0 != set_gpio_value(GPIO_SS, 1))
        {
            pabort("can't set gpio");
        }
#endif //USE_GPIOS

        if (verbose)
        {
            hex_dump(tx, len_tx, 32, "TX");
        }

        hex_dump(rx, len_rx, 32, "RX");
        //printf("calc_crc = %02X\n", get_header_crc(rx));
    }
    while (get_header_crc(rx) != rx[HEADER_LEN-1]);
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
         "  -v --verbose  Verbose (show tx buffer)\n"
         "  -p            Send data (e.g. \"1234\\xde\\xad\")\n"
         "  -N --no-cs    no chip select\n"
         "  -R --ready    slave pulls low to pause\n"
         "  -2 --dual     dual transfer\n"
         "  -4 --quad     quad transfer\n");
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
            { "dual",    0, 0, '2' },
            { "verbose", 0, 0, 'v' },
            { "quad",    0, 0, '4' },
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
            speed = atoi(optarg);
            break;
        case 'd':
            delay = atoi(optarg);
            break;
        case 'b':
            bits = atoi(optarg);
            break;
        case 'l':
            mode |= SPI_LOOP;
            break;
        case 'H':
            mode |= SPI_CPHA;
            break;
        case 'O':
            mode |= SPI_CPOL;
            break;
        case 'L':
            mode |= SPI_LSB_FIRST;
            break;
        case 'C':
            mode |= SPI_CS_HIGH;
            break;
        case '3':
            mode |= SPI_3WIRE;
            break;
        case 'N':
            mode |= SPI_NO_CS;
            break;
        case 'v':
            verbose = 1;
            break;
        case 'R':
            mode |= SPI_READY;
            break;
        case 'p':
            input_tx = optarg;
            break;
        case '2':
            mode |= SPI_TX_DUAL;
            break;
        case '4':
            mode |= SPI_TX_QUAD;
            break;
        default:
            print_usage(argv[0]);
            break;
        }
    }
    if (mode & SPI_LOOP) {
        if (mode & SPI_TX_DUAL)
            mode |= SPI_RX_DUAL;
        if (mode & SPI_TX_QUAD)
            mode |= SPI_RX_QUAD;
    }
}

int main(int argc, char *argv[])
{
    int ret = 0;
    int fd;

#ifdef USE_GPIOS
    if (0 != set_gpio_value(GPIO_SS, 1))
    {
        pabort("can't set gpio");
    }
#endif //USE_GPIOS

    parse_opts(argc, argv);

    fd = open(device, O_RDWR);
    if (fd < 0)
        pabort("can't open device");

    /*
     * spi mode
     */
    ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
    if (ret == -1)
        pabort("can't set spi mode");

    ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
    if (ret == -1)
        pabort("can't get spi mode");

    /*
     * bits per word
     */
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't set bits per word");

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't get bits per word");

    /*
     * max speed hz
     */
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't set max speed hz");

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't get max speed hz");

    printf("device: %s\n", device);
    printf("spi mode: 0x%x\n", mode);
    printf("bits per word: %d\n", bits);
    printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

    default_tx[HEADER_LEN-1] = get_header_crc(default_tx);
    transfer(fd, default_tx, default_rx, sizeof(default_tx), sizeof(default_rx));

    close(fd);

    return ret;
}
