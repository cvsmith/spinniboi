obj-m+=encoder.o pwm.o
encoder-objs := rpi_gpio_lib.o encoder_driver.o
pwm-objs := motor_pwm.o

all:
		make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
		make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
