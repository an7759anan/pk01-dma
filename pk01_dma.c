#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <bits/time.h>

#include "pk01_dma_utils.h"

#define SAMPLE_RATE 32000     // Default sample rate (samples/sec)
#define RX_SAMPLE_SIZE 4      // Number of raw Rx bytes per sample
#define SPI_CSVAL 0           // Additional CS register settings
#define MIN_SPI_FREQ 1000     // Minimum SPI frequency
#define MAX_SPI_FREQ 42000000 // Maximum SPI frequency
#define VC_MEM_SIZE(ns) (PAGE_SIZE + ((ns) + 4) * RX_SAMPLE_SIZE)
#define ADC_BASE_VOLTAGE 2.5
#define SAMPLE_BITS 24
#define DISCRETES 8388608.0 // 2**(SAMPLE_BITS-1)
#define ADC_VOLTAGE(n) (((n) * ADC_BASE_VOLTAGE) / DISCRETES)

#define SPI0_CE_NUM 0
#define SPI0_CE0_PIN 8
#define SPI0_MISO_PIN 9
#define SPI0_MOSI_PIN 10
#define SPI0_SCLK_PIN 11

#define SPI0_BASE (PHYS_REG_BASE + 0x204000)
#define SPI_CS 0x00
#define SPI_FIFO 0x04
#define SPI_CLK 0x08
#define SPI_DLEN 0x0c
#define SPI_DC 0x14
#define SPI_FIFO_CLR (3 << 4)
#define SPI_TX_FIFO_CLR (1 << 4)
#define SPI_TFR_ACT (1 << 7)
#define SPI_DMA_EN (1 << 8)
#define SPI_AUTO_CS (1 << 11)

#define IN_DATA_NO 0b00
#define IN_DATA_YES 0b01
#define IN_OVERFLOW 0b11

// Buffer for processed ADC samples
double *sample_buff;
char textBuffer[100];

// Virtual memory pointers to acceess GPIO, DMA & SPI from user space
extern MEM_MAP gpio_regs, dma_regs;
MEM_MAP vc_mem, spi_regs;
FILE *file;

void terminate(int sig);
void map_devices(void);
void dma_wait(int chan);
int init_spi(int hz);
void spi_disable(void);
uint32_t *read_samples_dma(MEM_MAP *mp, int nsamp, double *sample_buff);
char *formatValue(char *buf, uint32_t inValue);
double volts(uint32_t inValue);
struct timespec ts;

int print_all = 0;

// Main program
int main(int argc, char *argv[])
{
    int args = 0, sample_count = 0, sample_rate = SAMPLE_RATE;
    int f, spi_freq, val, i, n;
    int out_file = 0;
    char out_file_name[1000];
    uint32_t *rxdata;

    while (argc > ++args) // Process command-line args
    {
        if (argv[args][0] == '-')
        {
            switch (toupper(argv[args][1]))
            {
            case 'N': // -N: number of samples
                if (args >= argc - 1 || !isdigit(argv[args + 1][0]))
                    fprintf(stderr, "Error: no sample count\n");
                else
                    sample_count = atoi(argv[++args]);
                break;
            case 'R': // -R: sample rate (samples/sec)
                if (args >= argc - 1 || !isdigit(argv[args + 1][0]))
                    fprintf(stderr, "Error: no sample rate\n");
                else
                    sample_rate = atoi(argv[++args]);
                break;
            case 'P': // -P: print all
                print_all = 1;
                break;
            case 'F': // -F: out data to file
                if (args >= argc - 1)
                    fprintf(stderr, "Error: no sample output file name\n");
                else
                    strcpy(out_file_name, argv[++args]);
                    out_file = 1;
                break;
            }
        }
    }
    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE(sample_count));
    print_all &&printf("%u samples at %u S/s\n", sample_count, f / (RX_SAMPLE_SIZE * 8));
    signal(SIGINT, terminate);
    spi_freq = sample_rate * RX_SAMPLE_SIZE * 8;
    if (spi_freq < MIN_SPI_FREQ)
        fail("Invalid sample rate\n");
    f = init_spi(spi_freq);
    if (print_all == 1)
    {
        printf("SPI frequency %u Hz, %u samples at %u S/s\n", f, sample_count, f / (RX_SAMPLE_SIZE * 8));
        printf("%u samples at %u S/s\n", sample_count, f / (RX_SAMPLE_SIZE * 8));
        printf("%u samples at %u S/s\n", sample_count, f / (RX_SAMPLE_SIZE * 8));
    }
    sample_buff = malloc(sample_count * sizeof(double));
    rxdata = read_samples_dma(&vc_mem, sample_count, sample_buff);

    for (int i = 0; i < sample_count; i++)
    {
        printf("%s", formatValue(textBuffer, rxdata[i]));
    }

    if (out_file == 1)
    {
        file = fopen(out_file_name, "w");
        if (file != NULL)
        {
            for (int i = 0; i < sample_count; i++)
            {
                fputs(formatValue(textBuffer, rxdata[i]), file);
            }
        }
        else
        {
            fail("Can't open output file\n");
        }
    }

    terminate(0);
}

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
    terminate(0);
}

// Free memory segments and exit
void terminate(int sig)
{
    print_all == 1 && printf("Closing\n");
    spi_disable();
    stop_dma(DMA_CHAN_A);
    stop_dma(DMA_CHAN_B);
    unmap_periph_mem(&vc_mem);
    unmap_periph_mem(&spi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
    if (sample_buff)
        free(sample_buff);
    if (file != NULL)
        fclose(file);
    exit(0);
}

// Map GPIO, DMA and SPI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&spi_regs, (void *)SPI0_BASE, PAGE_SIZE);
}

uint32_t *read_samples_dma(MEM_MAP *mp, int nsamp, double *sample_buff)
{
    DMA_CB *cbs = mp->virt;
    uint32_t dlen, *txd = (uint32_t *)(cbs + 3);
    uint32_t *rxdata = txd + 0x10;

    enable_dma(DMA_CHAN_A);
    enable_dma(DMA_CHAN_B);
    dlen = nsamp * 4;
    txd[0] = (dlen << 16) | SPI_TFR_ACT;
    txd[1] = 0;
    txd[2] = 0x00000004;
    cbs[0].ti = DMA_SRCE_DREQ | (DMA_SPI_RX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_DEST_INC;
    cbs[0].tfr_len = dlen;
    cbs[0].srce_ad = REG_BUS_ADDR(spi_regs, SPI_FIFO);
    cbs[0].dest_ad = MEM_BUS_ADDR(mp, rxdata);
    cbs[1].ti = DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_SRCE_INC;
    cbs[1].tfr_len = 8;
    cbs[1].srce_ad = MEM_BUS_ADDR(mp, txd);
    cbs[1].dest_ad = REG_BUS_ADDR(spi_regs, SPI_FIFO);
    cbs[1].next_cb = MEM_BUS_ADDR(mp, &cbs[2]);
    cbs[2].ti = DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_SRCE_INC;
    cbs[2].tfr_len = 4;
    cbs[2].srce_ad = MEM_BUS_ADDR(mp, &txd[2]);
    cbs[2].dest_ad = REG_BUS_ADDR(spi_regs, SPI_FIFO);
    cbs[2].next_cb = MEM_BUS_ADDR(mp, &cbs[2]);

    *REG64(spi_regs, SPI_DC) = (8 << 24) | (4 << 16) | (8 << 8) | 4;
    *REG64(spi_regs, SPI_CS) = SPI_TFR_ACT | SPI_DMA_EN | SPI_AUTO_CS | SPI_FIFO_CLR | SPI_CSVAL;
    *REG64(spi_regs, SPI_DLEN) = 0;

    start_dma(mp, DMA_CHAN_A, &cbs[0], 0);
    start_dma(mp, DMA_CHAN_B, &cbs[1], 0);
    dma_wait(DMA_CHAN_A);

    return (rxdata);
}

// Wait until DMA is complete
void dma_wait(int chan)
{
    int n = 10000;

    do
    {
        usleep(100);
    } while (dma_transfer_len(chan) && --n);
    if (n == 0)
        printf("DMA transfer timeout\n");
}

char *formatValue(char *buf, uint32_t inValue)
{
    uint8_t *byt = (uint8_t *)&inValue;
    uint8_t cb = byt[0];
    uint8_t status = (cb & 0b00001100) >> 2;
    uint8_t cycle = cb & 0b00000011;
    uint32_t sample = (((byt[1]) << 8) + byt[2] << 8) + byt[3];

    char statName = ' ';
    if (status == IN_DATA_NO)
        statName = '-';
    else if (status == IN_DATA_YES)
        statName = '*';
    else if (status == IN_OVERFLOW)
        statName = 'o';
    sprintf(buf, "%.8x %.1x %u %c %.6x %+f\n", inValue, status, cycle, statName, sample, volts(inValue));
    return buf;
}

double volts(uint32_t inValue)
{
    uint8_t *byt = (uint8_t *)&inValue;
    uint32_t sample = (((byt[1]) << 8) + byt[2] << 8) + byt[3];
    double volts = ADC_VOLTAGE(sample);
    if (volts > ADC_BASE_VOLTAGE)
    {
        volts -= 2 * ADC_BASE_VOLTAGE;
    }
    return volts;
}

// Initialise SPI0, given desired clock freq; return actual value
int init_spi(int hz)
{
    int f, div = (CLOCK_HZ / hz + 1) & ~1;
    gpio_set(SPI0_CE0_PIN, GPIO_ALT0, GPIO_PULLUP);
    gpio_set(SPI0_MISO_PIN, GPIO_ALT0, GPIO_PULLDN);
    gpio_set(SPI0_MOSI_PIN, GPIO_ALT0, GPIO_PULLDN);
    gpio_set(SPI0_SCLK_PIN, GPIO_ALT0, GPIO_PULLDN);
    while (div == 0 || (f = CLOCK_HZ / div) > MAX_SPI_FREQ)
        div += 2;
    *REG64(spi_regs, SPI_CS) = 0x30;
    *REG64(spi_regs, SPI_CLK) = div;
    // printf("div: %i\n", div);
    return (f);
}

// Disable SPI
void spi_disable(void)
{
    *REG64(spi_regs, SPI_CS) = SPI_FIFO_CLR;
    *REG64(spi_regs, SPI_CS) = 0;
}

// EOF
