import array, fcntl

with open('char_dev', 'r+') as f:
    buf = array.array('c', "hello world")
    print fcntl.ioctl(f.fileno(), 0x80046400, buf, False)
