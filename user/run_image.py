import cPickle as pickle
import sys

ticks_per_rev = 265
strip_len = 144

def load_image(name):
    with open(name, 'rb') as f:
        buf = pickle.load(f)
    return buf

def pass_to_driver(f, samp_img):
    data = ""
    for val in samp_img:
        data += chr(val & 0xFF)
        data += chr((val >> 8) & 0xFF)
        data += chr((val >> 16) & 0xFF)
        data += chr((val >> 24) & 0xFF)

    f.write(data)



def main():
    print "Importing image"
    samp_img = load_image(sys.argv[1])
    with open('/dev/encoder', 'wb') as f:
        print "passing to driver"
        pass_to_driver(f, samp_img)

    print "done"

main()
