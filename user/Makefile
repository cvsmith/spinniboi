HEADERS = ioctl_chardev.h

default: ioctl_user

ioctl_user.o: ioctl_user.c $(HEADERS)
	    gcc -c ioctl_user.c -o ioctl_user.o

ioctl_user: ioctl_user.o
	    gcc ioctl_user.o -o ioctl_user

clean:
	    -rm -f ioctl_user.o
			-rm -f ioctl_user
