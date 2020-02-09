#!/usr/bin/env python3

from blinkt_interface import Blinkt, Pixel

blinkt = Blinkt()

white = Pixel(255, 255, 255, 0.5)

for pixel in blinkt.getPixelArray():
    pixel.set(white)

blinkt.display()
