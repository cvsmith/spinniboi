import array, fcntl
from math import sin, cos, pi
import numpy as np
import PIL import Image

ticks_per_rev = 100
strip_len = 144

def import_image(name):
    img = Image.open(name)
    img = np.array(img)
    img = img[0:144, 0:144, :]
    return img

def sample_image(img):
    samp_img = []
    for tick_num in xrange(ticks_per_rev):
        for radius_idx in xrange(-strip_len/2, strip_len/2):
            angle1 =
