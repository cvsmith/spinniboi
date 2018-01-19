obj-m+=encoder.o pwm.o ioctl_test.o
encoder-objs := spi.o rpi_gpio_lib.o encoder_driver.o
pwm-objs := motor_pwm.o
ioctl_test-objs := ioctl_chardev.o

all:
		make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
		make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
