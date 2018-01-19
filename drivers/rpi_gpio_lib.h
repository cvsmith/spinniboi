/**
 * @file rpi_gpio_lib.h
 * @date Thursday, October 20, 2016 at 11:07:50 PM EDT
 * @author Karthic Palaniappan (kapalani)
 *
 * Raspberry pi linux gpio header file
 *
 * @bug No known bugs.
 **/

#ifndef RPI_GPIO_LIB_H
#define RPI_GPIO_LIB_H

/** @brief base of MMIO (physical address) on the pi */
#define MMIO_BASE_PHYSICAL 0x3F000000
/** @brief base of GPIO in memory mapped IO on pi */
#define GPIO_BASE (MMIO_BASE_PHYSICAL + 0x200000)
/** @brief GPIO Pin Output Set 0 */
#define GPIO_REG_GPSET0 7
/** @brief GPIO Pin Output Set 1 */
#define GPIO_REG_GPSET1 8
/** @brief GPIO Pin Output Clear 0 */
#define GPIO_REG_GPCLR0 10
/** @brief GPIO Pin Output Clear 1 */
#define GPIO_REG_GPCLR1 11
/** @brief GPIO Pin Level 0 **/
#define GPIO_REG_GPLEV0 13
/** @brief GPIO Pin Level 1 **/
#define GPIO_REG_GPLEV1 14
/** @brief GPIO Pin Pull-up/down Enable **/
#define GPIO_REG_GPPUD 37
/** @brief GPIO Pin Pull-up/Down Clock 0 **/
#define GPIO_REG_GPPUDCLK0 38
/** @brief GPIO Pin Pull-up/Down Clock 1 **/
#define GPIO_REG_GPPUDCLK1 39

#define GPIO_NUM_REGS 41

/** @brief GPIO Disable pull-up/down **/
#define GPIO_PUD_DISABLE 0
/** @brief GPIO Pull down control **/
#define GPIO_PUD_DOWN 1
/** @brief GPIO Pull up control **/
#define GPIO_PUD_UP 2

int gpio_lib_init(void);
void gpio_lib_deinit(void);
void gpio_config(unsigned char pin, unsigned char alt);
void gpio_set(unsigned char pin);
void gpio_clr(unsigned char pin);
unsigned char gpio_read(unsigned char pin);
void gpio_set_pullup(unsigned char pin, unsigned char pud);

#endif /* RPI_GPIO_LIB_H */