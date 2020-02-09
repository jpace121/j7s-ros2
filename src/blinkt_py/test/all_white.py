#!/usr/bin/env python3

from blinkt_interface import Blinkt, Pixel
import blinkt_interface.color as color

blinkt = Blinkt()

for pixel in blinkt.getPixelArray():
    pixel.set(color.white(0.5))

blinkt.display()
