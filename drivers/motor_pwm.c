/**
 * @file   motor_pwm.c
 *
 * @brief  LKM driver for pwm using software irq
 *
 * @author Vikram Shanker <vshanker@andrew.cmu.edu>
 * @author Ram Verma <ramv@andrew.cmu.edu>
 */
#include <linux/init.h>   				// Macros used to mark up functions e.g. __init __exit
#include <linux/module.h> 				// Core header for loading LKMs into the kernel
#include <linux/device.h> 				// Header to support the kernel Driver Model
#include <linux/kernel.h> 				// Contains types, macros, functions for the kernel
#include <linux/fs.h>     				// Header for the Linux file system support
#include <asm/uaccess.h>  				// Required for the copy to user function
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/hrtimer.h>				// for high-resolution timer
#include <linux/ktime.h>

/** @brief Device name to appear in /dev */
#define DEVICE_NAME "pwm"
/** @brief Class which the device belongs to */
#define CLASS_NAME "PWMCLASS"

/** @brief Module info: license */
MODULE_LICENSE("GPL");
/** @brief Module info: author(s) */
MODULE_AUTHOR("Vikram Shanker");
MODULE_AUTHOR("Ram Verma");
/** @brief Module info: description */
MODULE_DESCRIPTION("A simple Linux char driver for pwm using software irq");
/** @brief Module info: version */
MODULE_VERSION("0.1");

/** @brief major number to identify device */
static int major_number;
/** @brief class pointer for PWM driver */
static struct class *PWM_driver_class = NULL;
/** @brief device pointer for PWM driver */
static struct device *PWM_driver_device = NULL;
/* Message buffer */
static char message_buf[32] = {0};
/* len to be read */
int size_of_message;
/* GPIO stuff */
static unsigned int gpioPWM = 18;

volatile int period = 100; /* in ms for period */
volatile int dutyCycle = 0; // initialize to 0
volatile int count = 0;
static struct hrtimer hr_timer;
ktime_t ktime;

int clockSpeed = 1;

enum hrtimer_restart my_hrtimer_callback( struct hrtimer *timer ) {

	if (count >= dutyCycle) {
		gpio_set_value(gpioPWM, false);
	}
	else {
		gpio_set_value(gpioPWM, true);
	}
	count++;
	if (count >= period) {
		count = 0;
	}
	//printk(KERN_ALERT "In timer\n");
	hrtimer_forward_now(&hr_timer, ktime);
	return HRTIMER_RESTART;
}

/* Declaration of helper functions */
static int PWM_driver_open(struct inode *inodep, struct file *filep);
static int PWM_driver_release(struct inode *inodep, struct file *filep);
static ssize_t PWM_driver_read(struct file *filep, char *buffer, size_t len,loff_t *offset);
static ssize_t PWM_driver_write(struct file *filep, const char *buffer,size_t len, loff_t *offset);


/** @brief global file_operations struct to create character device driver */
static struct file_operations fops =
{
	.open = PWM_driver_open,
	.read = PWM_driver_read,
	.write = PWM_driver_write,
	.release = PWM_driver_release,
};

// ****************************************************************************
// Module interface functions
// ****************************************************************************

static int __init PWM_driver_init(void) {
	printk(KERN_INFO "PWM driver: hello world!\n");

	// Register
	major_number = register_chrdev(0,DEVICE_NAME,&fops);
	if (major_number < 0)
	{
	  printk(KERN_ALERT "PWM driver: failed to register a major number \n");
	  return major_number;
	}
	printk(KERN_INFO "PWM driver: registered correctly with major number %d\n",major_number);

	// Create class
	PWM_driver_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(PWM_driver_class))
	{
		unregister_chrdev(major_number,DEVICE_NAME);
		printk(KERN_ALERT "Did not register device class \n");
		return PTR_ERR(PWM_driver_class);
	}
	printk(KERN_INFO "PWM driver: device class registered correctly \n");

	// Create device
	PWM_driver_device = device_create(PWM_driver_class,NULL,MKDEV(major_number,0),NULL,DEVICE_NAME);
	if(IS_ERR(PWM_driver_device))
	{
		class_destroy(PWM_driver_class);
		unregister_chrdev(major_number,DEVICE_NAME);
		printk(KERN_ALERT "Did not register device (not class), error:%ld \n",PTR_ERR(PWM_driver_device));
		return PTR_ERR(PWM_driver_device);
	}
	printk(KERN_INFO "PWM driver: device (not class) created correctly \n");

	// if char driver is up, need set the gpio
	gpio_request(gpioPWM, "gpioPWM");		// gpio for PWM requested
	gpio_direction_output(gpioPWM, false);	// Set the gpio to be in output mode and off

	// init timer
	ktime = ktime_set( 0, clockSpeed*100000 );
	hrtimer_init( &hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	hr_timer.function = &my_hrtimer_callback;
	hrtimer_start( &hr_timer, ktime, HRTIMER_MODE_REL );

	// Success
	return 0;
}



static void __exit PWM_driver_exit(void) {
	int ret;
	device_destroy(PWM_driver_class,MKDEV(major_number,0));
	class_destroy(PWM_driver_class);
	unregister_chrdev(major_number,DEVICE_NAME);

	gpio_set_value(gpioPWM, 0);
	gpio_unexport(gpioPWM);
	gpio_free(gpioPWM);

	ret = hrtimer_cancel( &hr_timer );
	if (ret)
	    printk(KERN_INFO "The timer was still in use...\n");

	printk(KERN_INFO "PWM driver: Goodbye from the LKM!\n");
}

static int PWM_driver_open(struct inode *inodep,struct file *filep)
{
	printk(KERN_INFO "PWM driver: Device opened \n");
	return 0;
}

static ssize_t PWM_driver_read(struct file *filep, char *buffer, size_t len,
							   loff_t *offset)
{
	int error_count = 0;
	// copy_to_user has the format ( * to, *from, size) and returns 0 on success
	error_count = copy_to_user(buffer, message_buf, size_of_message);

	if (error_count==0){		// if true then have success
		printk(KERN_INFO "PWM driver: Sent %d characters to the user\n", size_of_message);
		return (size_of_message=0);	// clear the position to the start and return 0
	}
	else {
		printk(KERN_INFO "PWM driver: Failed to send %d characters to the user\n", error_count);
		return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
	}
}

static ssize_t PWM_driver_write(struct file *filep, const char *buffer,
								size_t len,loff_t *offset)
{
	int PWMSwitch;

	//copy the message the user typed
	unsigned copy_len = (len > 31) ? 31 : len;
	int ret = copy_from_user(message_buf,buffer,copy_len);
	if(ret != 0)
	{
		printk(KERN_ALERT "PWM driver: Could not copy %d characters \n", ret);
		return (copy_len - ret);
	}
	//null terminate string, since we are going to use sscanf
	size_of_message = len;
	message_buf[len] = '\0';

	sscanf(message_buf, "%d\n", &PWMSwitch);

	if (PWMSwitch < 0 || PWMSwitch > 100) {
		printk(KERN_ALERT "Invalid duty cycle");
		return copy_len;
	}
	else {
		dutyCycle = PWMSwitch;
	}

	return copy_len;
}

static int PWM_driver_release(struct inode *inodep, struct file *filep)
{
	printk(KERN_INFO "PWM driver: Device successfully closed \n");
	return 0;
}

module_init(PWM_driver_init);
module_exit(PWM_driver_exit);
