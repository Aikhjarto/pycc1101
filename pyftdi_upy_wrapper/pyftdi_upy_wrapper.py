# -*- coding: utf-8 -*-
"""
@author: ratschenberger
"""
from pyftdi.spi import SpiController

class mSPI():

    def __init__(self, cs, mode, freq):
        self._cs = cs
        self._mode = mode
        self._freq = freq
        self.spi = SpiController()
        self.spi.configure('ftdi://ftdi:2232/2')
        #self.spi.configure('ftdi://ftdi:4232/2')
        self._slave = self.spi.get_port(cs=cs, mode=mode, freq=freq)

    def write_readinto(self, writeBuf, readBuf):
        assert len(writeBuf) == len(readBuf), "Buffers must have the self length"
        readBuf2 = bytearray(self._slave.exchange(writeBuf, duplex = True, start = False, stop = False))
        readBuf[:] = readBuf2

class ftdiPin():

    def __init__(self, spi, pin, direction = 1):
        self.gpio = spi.get_gpio()
        used = self.gpio.all_pins
        used_dir = self.gpio.direction
        self.gpio.set_direction(used | 1<<pin, used_dir | direction<<pin)
        self.pin = pin
        self.direction = direction

    def value(self):
        val = self.gpio.read() & 1<<self.pin
        if val != 0:
            return 1
        return 0

    def on(self):
        self.gpio.write(1<<self.pin)

    def off(self):
        self.gpio.write(0<<self.pin)
