/**
 * @file rpi_gpio_lib.c
 * @date Thursday, October 20, 2016 at 09:33:18 PM EDT
 * @author Karthic Palaniappan (kapalani)
 *
 * GPIO setup for the Raspberry Pi
 *
 * @author Karthic Palaniappan <kapalani@andrew.cmu.edu>
 *
 * @bug No known bugs.
 **/

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include "rpi_gpio_lib.h"

/** @brief GPIO register set */
static volatile unsigned *gpio = NULL;

/** @brief configures a pin for a given functionality.
 *
 *  See BCM2835 peripherals pg 102 - 103 for various alternate
 *  functions of GPIO pins.
 *
 *  @param pin the pin number to configure (0 to 53 on pi)
 *  @param alt the alternate
 */
void gpio_config(unsigned char pin, unsigned char alt)
{
    //5 alternatives and 5 function select regs with each reg
    //holding the function alternatives for 10 gpio pins

    // get the offset into MM GPIO
    unsigned int reg = pin / 10;
    // get contents of correct GPIO_REG_GPFSEL register
    unsigned int config = gpio[reg];

    // get the bit offset into the GPIO_REG_GPFSEL register
    // multiply by 3 because each gpio pin gets 3 bits
    unsigned int offset = (pin % 10) * 3;

    //clear any existing alternate and set the new alternate
    config &= ~(0x7 << offset);
    config |= (alt << offset);
    gpio[reg] = config;
}

/** @brief sets a given GPIO pin high.
 *
 *  @param pin the pin number to set (0 to 53 on pi)
 */
void gpio_set(unsigned char pin)
{
    if (pin > 31)
        gpio[GPIO_REG_GPSET1] = (1 << (pin - 32));
    else
        gpio[GPIO_REG_GPSET0] = (1 << pin);
}

/** @brief sets a given GPIO pin low.
 *
 *  @param pin the pin number to clear (0 to 53 on pi)
 */
void gpio_clr(unsigned char pin)
{
    if (pin > 31)
        gpio[GPIO_REG_GPCLR1] = (1 << (pin - 32));
    else
        gpio[GPIO_REG_GPCLR0] = (1 << pin);
}

/**
 * @brief Read the value of a GPIO pin
 *
 * @param pin the pin number to read (0 to 53 on pi)
 *
 * @return 1 if the pin is high 0 if low
 *
 **/
unsigned char gpio_read(unsigned char pin)
{
    if(pin > 31)
        return !!(gpio[GPIO_REG_GPLEV1] & (1 << (pin - 32)));
    else
        return !!(gpio[GPIO_REG_GPLEV0] & (1 << pin));
}

/**
 * @brief Set the pullup of a gpio pin
 *
 * @param  pin pin to set the pull up
 * @param  pud pull up/down/disable for the gpio pin
 *
 **/
void gpio_set_pullup(unsigned char pin, unsigned char pud)
{
    //1. set the required pull up
    gpio[GPIO_REG_GPPUD] = pud;
    udelay(10);

    //2. assert clock line on the appropriate pin
    if(pin > 31)
        gpio[GPIO_REG_GPPUDCLK1] = (1 << (pin - 32));
    else
        gpio[GPIO_REG_GPPUDCLK0] = (1 << pin);
    udelay(10);

    //3. remove the control signal
    gpio[GPIO_REG_GPPUD] = GPIO_PUD_DISABLE;

    //4. remove the clock
    if(pin > 31)
        gpio[GPIO_REG_GPPUDCLK1] = 0;
    else
        gpio[GPIO_REG_GPPUDCLK0] = 0;
}

/**
 * @brief Initialize the gpio library by setting up the mmio
 *
 * @return 1 on success, else 0
 **/
int gpio_lib_init(void)
{
    printk(KERN_INFO "Initializing gpio library \n");

    if(gpio == NULL)
    {
        printk(KERN_ALERT "Remapping GPIO base \n");
        gpio = ioremap(GPIO_BASE, GPIO_NUM_REGS*4);
    }

    printk(KERN_ALERT "gpio = %p \n", gpio);
    return (gpio != NULL);
}

void gpio_lib_deinit(void)
{
    printk(KERN_INFO "Deinitializing gpio library \n");
    iounmap(gpio);
    gpio = NULL;
}