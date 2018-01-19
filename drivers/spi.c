/**
 * @file   spi.c
 *
 * @brief  SPI implementation on rpi 2
 *
 * @date   July 20 2015
 * @author Aaron Reyes <areyes@andrew.cmu.edu>
 */

#include <linux/kernel.h> 				// Contains types, macros, functions for the kernel
#include <linux/io.h>
#include "rpi_gpio_lib.h"


/* SPI0 MMIO register addresses */
/** c */
#define SPI0_BASE (MMIO_BASE_PHYSICAL + 0x204000)
#define SPI0_CS_REG   0  // SPI Master Control and Status
/** c */
#define SPI0_FIFO_REG 1 // SPI Master TX and RX FIFOs
/** c */
#define SPI0_CLK_REG  2 // SPI Master Clock Divider
#define NUM_SPI_REGS 3
/** c */

static volatile unsigned *spi = NULL;

/* Register masks for SPI0_CS_REG (section 10.5) */
/** c */
#define SPI_LEN_LONG  25 // Enable Long data word in Lossi mode if DMA_LEN is set
/** c */
#define SPI_DMA_LEN   24 // Enable DMA mode in Lossi mode
/** c */
#define SPI_CSPOL2    23 // Chip Select 2 Polarity
/** c */
#define SPI_CSPOL1    22 // Chip Select 1 Polarity
/** c */
#define SPI_CSPOL0    21 // Chip Select 0 Polarity
/** c */
#define SPI_RX        20 // RX FIFO Full
/** c */
#define SPI_RXR       19 // RX FIFO needs Reading
/** c */
#define SPI_TXD       18 // TX FIFO can accept Data
/** c */
#define SPI_RXD       17 // RX FIFO contains Data
/** c */
#define SPI_DONE      16 // transfer Done
/** c */
#define SPI_LEN       13 // LoSSI enable
/** c */
#define SPI_REN       12 // Read Enable
/** c */
#define SPI_ADCS      11 // Automatically Deassert Chip Select
/** c */
#define SPI_INTR      10 // Interrupt on RXR
/** c */
#define SPI_INTD      9  // Interrupt on Done
/** c */
#define SPI_DMAEN     8  // DMA Enable
/** c */
#define SPI_TA        7  // Transfer Active
/** c */
#define SPI_CSPOL     6  // Chip Select Polarity
/** c */
#define SPI_CLEAR_RX  5  // RX FIFO Clear
/** c */
#define SPI_CLEAR_TX  4  // TX FIFO Clear
/** c */
#define SPI_CPOL      3  // Clock Polarity
/** c */
#define SPI_CPHA      2  // Clock Phase
/** c */
#define SPI_CS1       1  // Chip Select 1
/** c */
#define SPI_CS0       0  // Chip Select 0

#define PIN_MOSI 10
#define PIN_CLK 11
#define PIN_MOSI_OFFSET 0
#define PIN_CLK_OFFSET 3
#define FUN_ALT0 4
//static volatile uint32_t *GPFSEL1 = (volatile uint32_t*)0x7E200004;

#define CDIV_488K 512

/*
void gpio_config(void) {
    uint32_t config = *GPFSEL1;
    config &= ~((0x7 << PIN_MOSI_OFFSET) | (0x7 << PIN_CLK_OFFSET));
    config |= (FUN_ALT0 << PIN_MOSI_OFFSET) | (FUN_ALT0 << PIN_CLK_OFFSET);
    *GPFSEL1 = config;
}
*/

/** NOTE: SPI is always MSB first on rpi */
void wait(unsigned int delay) {
  while(delay--) {
    asm("mov r0, r0");
  }
}

/** NOTE: Aaron Reyes doesn't know doxygen */
void spi_master_init(uint32_t clk) {
  unsigned int var;

  gpio_config(PIN_MOSI, FUN_ALT0);
  gpio_config(PIN_CLK, FUN_ALT0);

  spi = ioremap(SPI0_BASE, NUM_SPI_REGS*4);

 // clear the SPI_CS register
  spi[SPI0_CS_REG] = 0;

  // clear TX/RX FIFOs
  spi[SPI0_CS_REG] |= (1 << SPI_CLEAR_RX) | (1 << SPI_CLEAR_TX);

  var = spi[SPI0_CS_REG];

  // Set mode = 0 (clock rest state == low, first sclk in middle of data bit)
  var &= ~((1 << SPI_CPOL) | (1 << SPI_CPHA));

  // Set chip select = 0
  var &= ~((1 << SPI_CS0) | (1 << SPI_CS1));

  // Set polarity = HIGH
  var |= (1 << SPI_CSPOL);

  spi[SPI0_CS_REG] = var;

  // set the clock rate
  spi[SPI0_CLK_REG] = clk;
  wait(10000);
}

void spi_master_deinit(void) {
    iounmap(spi);
    spi = NULL;
}

/** ccccccc */
/*
void spi_begin(uint8_t cmdMode, uint32_t clk) {
  unsigned int var;

  // set SPI pins according to pg 102
//XXX  gpio_config(CE1_N, GPIO_FUN_ALT0);
// XXX  gpio_config(CE0_N, GPIO_FUN_ALT0);
// XXX  gpio_config(MISO, GPIO_FUN_OUTPUT);
  //gpio_config(MISO, GPIO_FUN_ALT0);
// XXX  gpio_config(MOSI, GPIO_FUN_ALT0);
// XXX  gpio_config(SCLK, GPIO_FUN_ALT0);


  // clear the SPI_CS register
  *SPI0_CS_REG = 0;

  // clear TX/RX FIFOs
  *SPI0_CS_REG |= (1 << SPI_CLEAR_RX) | (1 << SPI_CLEAR_TX);

  var = *SPI0_CS_REG;

  // Set mode = 0
  var &= ~((1 << SPI_CPOL) | (1 << SPI_CPHA));

  // Set chip select = 0
  var &= ~((1 << SPI_CS0) | (1 << SPI_CS1));

  // Set polarity = HIGH
  var |= (1 << SPI_CSPOL);

  *SPI0_CS_REG = var;

  // set the clock rate
  *SPI0_CLK_REG = clk;
}
*/

/** TA as in TA? */
/*
void spi_end(void) {
  // clear TA
  *SPI0_CS_REG &= ~(1 << SPI_TA);
}
*/

inline void spi_start(void) {
  // clear TX FIFO
  spi[SPI0_CS_REG] |= 1 << SPI_CLEAR_TX;
  // Set transfer active
  spi[SPI0_CS_REG] |= 1 << SPI_TA;
}

inline void spi_send_byte(uint8_t data) {
    // Wait for TXD
    while (!((spi[SPI0_CS_REG]) & (1 << SPI_TXD))) {
        //wait(1);
    }

    // Write to TX fifo
    spi[SPI0_FIFO_REG] = data;
    spi[SPI0_CS_REG] |= (1 << SPI_CLEAR_RX);
}

inline void spi_finish(void) {
  // Wait for done to be set
  spi[SPI0_CS_REG] |= (1 << SPI_CLEAR_RX);
  while (!((spi[SPI0_CS_REG]) & (1 << SPI_DONE))) {
    //wait(1);
    spi[SPI0_CS_REG] |= (1 << SPI_CLEAR_RX);
  }

  // Set TA = 0
  spi[SPI0_CS_REG] &= ~(1<< SPI_TA);
}

void set_strip(uint32_t len, uint8_t brightness, uint32_t *leds) {
    //printk(KERN_INFO "SPI set strip called\n");
    spi_start();

    // Start frame 0 * 32
    //printk(KERN_INFO "SPI sending header\n");
    spi_send_byte(0);
    spi_send_byte(0);
    spi_send_byte(0);
    spi_send_byte(0);

    printk(KERN_INFO "SPI sending LEDs\n");
    for (; len > 0; len--, leds++) {
	int i;
        uint32_t data = *leds | ((uint32_t)0x7 << 29) | ((uint32_t)brightness << 24);
	for (i = 3; i >= 0; i--) {
      //printk(KERN_INFO "SPI sending byte %d led %u\n", i, len);
	    spi_send_byte((uint8_t)(data >> (i*8)));
	}
    }

    // End frame 1 * 32
    //printk(KERN_INFO "SPI sending footer\n");
    spi_send_byte(0xFF);
    spi_send_byte(0xFF);
    spi_send_byte(0xFF);
    spi_send_byte(0xFF);

    //printk(KERN_INFO "SPI finishing\n");
    spi_finish();
    //printk(KERN_INFO "SPI done\n");
}
