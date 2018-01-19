#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/fs.h>                   // Header for the Linux file system support
#include <asm/uaccess.h>                // Required for the copy to user function
#include <linux/device.h>
#include "rpi_gpio_lib.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Karthic Palaniappan");
MODULE_DESCRIPTION("Encoder Driver with Interrupts");
MODULE_VERSION("0.1");

#define MOTOR_ENCODER_A_GPIO 6
#define MOTOR_ENCODER_B_GPIO 13

#define DEVICE_NAME "encoder"
#define CLASS_NAME "ENCODER"

typedef struct
{
    unsigned int encoder_A_gpio;
    unsigned int encoder_B_gpio;
    long encoder_pos;
    bool A_set;
    bool B_set;
} encoder_t;

bool read_already = false;

/** @brief major number to identify device */
static int major_number;
/** @brief class pointer for encoder driver */
static struct class *encoder_driver_class = NULL;
/** @brief device pointer for encoder driver */
static struct device *encoder_driver_device = NULL;
/* Message buffer */
static char message_buf[32] = {0};
/* len to be read */
int size_of_message;

//encoders
static encoder_t motor_encoder = {.encoder_A_gpio = MOTOR_ENCODER_A_GPIO,
                                  .encoder_B_gpio = MOTOR_ENCODER_B_GPIO,
                                  .encoder_pos = 0,
                                  .A_set = false,
                                  .B_set = false
                                  };

static unsigned int irq_motor_enc_A;
static unsigned int irq_motor_enc_B;

/* Declaration of helper functions */
static int encoder_driver_open(struct inode *inodep, struct file *filep);
static int encoder_driver_release(struct inode *inodep, struct file *filep);
static ssize_t encoder_driver_read(struct file *filep, char *buffer, size_t len,loff_t *offset);
static ssize_t encoder_driver_write(struct file *filep, const char *buffer,size_t len, loff_t *offset);


/** @brief global file_operations struct to create character device driver */
static struct file_operations fops =
{
	.open = encoder_driver_open,
	.read = encoder_driver_read,
	.write = encoder_driver_write,
	.release = encoder_driver_release,
};

/// Function prototype for the custom IRQ handler function -- see below for
/// the implementation
static irq_handler_t  encoder_irq_handler(unsigned int irq, void *dev_id,
                                          struct pt_regs *regs);

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within
 *  this C file. The __init macro means that for a built-in driver
 *  (not a LKM) the function is only used at initialization time and that it
 *  can be discarded and its memory freed up after that point. In this
 *  example this function sets up the GPIOs and the IRQ
 *
 *  @return returns 0 if successful
 */
static int __init encoder_gpio_init(void){
  int result;
  result = 0;
	printk(KERN_INFO "encoder driver: hello world!\n");

	// Register
	major_number = register_chrdev(0,DEVICE_NAME,&fops);
	if (major_number < 0)
	{
	  printk(KERN_ALERT "encoder driver: failed to register a major number \n");
	  return major_number;
	}
	printk(KERN_INFO "encoder driver: registered correctly with major number %d\n",major_number);

	// Create class
	encoder_driver_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(encoder_driver_class))
	{
		unregister_chrdev(major_number,DEVICE_NAME);
		printk(KERN_ALERT "Did not register device class \n");
		return PTR_ERR(encoder_driver_class);
	}
	printk(KERN_INFO "encoder driver: device class registered correctly \n");

	// Create device
	encoder_driver_device = device_create(encoder_driver_class,NULL,MKDEV(major_number,0),NULL,DEVICE_NAME);
	if(IS_ERR(encoder_driver_device))
	{
		class_destroy(encoder_driver_class);
		unregister_chrdev(major_number,DEVICE_NAME);
		printk(KERN_ALERT "Did not register device (not class), error:%ld \n",PTR_ERR(encoder_driver_device));
		return PTR_ERR(encoder_driver_device);
	}
	printk(KERN_INFO "encoder driver: device (not class) created correctly \n");

    //Gpio setup
    gpio_lib_init();
    printk(KERN_INFO "ENCODER_DRIVER: Initializing the encoder driver LKM\n");
    if ((!gpio_is_valid(MOTOR_ENCODER_A_GPIO)) ||
        (!gpio_is_valid(MOTOR_ENCODER_B_GPIO)))
    {
       return -ENODEV;
    }

    gpio_request(MOTOR_ENCODER_A_GPIO, "motor_encoder_A_gpio");
    gpio_request(MOTOR_ENCODER_B_GPIO, "motor_encoder_B_gpio");

    //encoder gpio pins are all inputs
    gpio_direction_input(MOTOR_ENCODER_A_GPIO);
    gpio_direction_input(MOTOR_ENCODER_B_GPIO);

    //bool stops direction from being changed
    gpio_export(MOTOR_ENCODER_A_GPIO, false);
    gpio_export(MOTOR_ENCODER_B_GPIO, false);

    irq_motor_enc_A = gpio_to_irq(MOTOR_ENCODER_A_GPIO);
    irq_motor_enc_B = gpio_to_irq(MOTOR_ENCODER_B_GPIO);

    // This next call requests an interrupt line
    result = request_irq(irq_motor_enc_A,
                         (irq_handler_t) encoder_irq_handler,
                         IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                         "motor_encoder_A_gpio_handler",
                         NULL);

    result |= request_irq(irq_motor_enc_B,
                         (irq_handler_t) encoder_irq_handler,
                         IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                         "motor_encoder_B_gpio_handler",
                         NULL);

    return result;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro
 *  notifies that if this code is used for a built-in driver (not a LKM) that
 *  this function is not required. Used to release the GPIOs and display
 *  cleanup messages.
 */
static void __exit encoder_gpio_exit(void){

    device_destroy(encoder_driver_class,MKDEV(major_number,0));
    class_destroy(encoder_driver_class);
	unregister_chrdev(major_number,DEVICE_NAME);

    //unexport all the GPIOs
    gpio_unexport(MOTOR_ENCODER_A_GPIO);
    gpio_unexport(MOTOR_ENCODER_B_GPIO);

    //free all the installed irq handlers
    free_irq(irq_motor_enc_A,NULL);
    free_irq(irq_motor_enc_B,NULL);

    gpio_free(MOTOR_ENCODER_A_GPIO);
    gpio_free(MOTOR_ENCODER_B_GPIO);

    gpio_lib_deinit();
    printk(KERN_INFO "ENCODER_DRIVER: Goodbye from the LKM!\n");
}

static int encoder_driver_open(struct inode *inodep,struct file *filep)
{
	printk(KERN_INFO "encoder driver: Device opened \n");
	return 0;
}

static int encoder_driver_release(struct inode *inodep, struct file *filep)
{
	printk(KERN_INFO "encoder driver: Device successfully closed \n");
	return 0;
}

static ssize_t encoder_driver_read(struct file *filep, char *buffer, size_t len,
							   loff_t *offset)
{
    int error_count = 0;
    ssize_t size_of_message;

    if(read_already)
    {
	read_already = false;
	return 0;
    }

    size_of_message = snprintf(message_buf, 32, "%ld\n", motor_encoder.encoder_pos);

	// copy_to_user has the format ( * to, *from, size) and returns 0 on success
    //plus one because snprintf doesn't account for the NULL pointer
	error_count = copy_to_user(buffer, message_buf, size_of_message);

	if (error_count==0)
    {		// if true then have success
		printk(KERN_INFO "encoder driver: Sent %d characters to the user\n", size_of_message+1);
                read_already = true;
		return (size_of_message);
	}
	else
    {
		printk(KERN_INFO "encoder driver: Failed to send %d characters to the user\n", error_count);
		return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
	}

}

static ssize_t encoder_driver_write(struct file *filep, const char *buffer,
								size_t len,loff_t *offset)
{
	//copy the message the user typed
    int encoder_choice;
	unsigned copy_len = (len > 31) ? 31 : len;

	int ret = copy_from_user(message_buf,buffer,copy_len);
	if(ret != 0)
	{
		printk(KERN_ALERT "encoder driver: Could not copy %d characters \n", ret);
		return (copy_len - ret);
	}

	printk(KERN_INFO "encoder driver: Received %d characters from the user \n",len);
	//null terminate string, since we are going to use sscanf
	size_of_message = len;
	message_buf[len] = '\0';

	sscanf(message_buf, "%d\n", &encoder_choice);

	return copy_len;
}

/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO
 *  above. The same interrupthandler cannot be invoked concurrently as the
 *  interrupt line is masked out until the function is complete. This
 *  function is static as it should not be invoked directly from outside of
 *  this file.
 *
 *  @param irq  the IRQ number that is associated with the GPIO -- useful
 *              for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify
 *                which device caused the interrupt
 *  @param regs   h/w specific register values -- only really ever used for
 *                debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE
 *                  otherwise.
 */
static irq_handler_t encoder_irq_handler(unsigned int irq, void *dev_id,
                                         struct pt_regs *regs)
{
    encoder_t *enc = NULL;
    enc = &motor_encoder;

    if(irq == irq_motor_enc_A)
    {
        enc->A_set = (gpio_get_value(enc->encoder_A_gpio) == 1);
        //Increment if A Leads B
        enc->encoder_pos += (enc->A_set != enc->B_set) ? 1 : -1;
    }
    else if(irq == irq_motor_enc_B)
    {
        enc->B_set = (gpio_get_value(enc->encoder_B_gpio) == 1);
        //Incrementent if B follows A
        enc->encoder_pos += (enc->A_set == enc->B_set) ? 1 : -1;
    }

    return (irq_handler_t) IRQ_HANDLED;
}

/// This next calls are  mandatory -- they identify the initialization
/// function and the cleanup function (as above).
module_init(encoder_gpio_init);
module_exit(encoder_gpio_exit);
