obj-m+=encoder.o
encoder-objs := rpi_gpio_lib.o encoder_driver.o

all:
		make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
		make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
