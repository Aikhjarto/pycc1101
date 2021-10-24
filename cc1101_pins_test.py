from machine import Pin, SoftSPI
from pycc1101.pycc1101 import TICC1101

# ---- eLITe-connect
pCS433 = Pin(Pin.cpu.I10, Pin.OUT) #433
pCS433.on()
pGDO0433 = Pin(Pin.cpu.I12, Pin.IN) #433
pGDO2433 = Pin(Pin.cpu.I13, Pin.IN) #433

pCS868 = Pin(Pin.cpu.I4, Pin.OUT) #868
pCS868.on()
pGDO0868 = Pin(Pin.cpu.I7, Pin.IN) #868
pGDO2868 = Pin(Pin.cpu.I8, Pin.IN) #868

pSCK = Pin(Pin.cpu.K0, Pin.OUT)
pMOSI = Pin(Pin.cpu.J10, Pin.OUT)
pMISO = Pin(Pin.cpu.F8, Pin.IN)
# ---- eLITe-connect

spi = SoftSPI(baudrate=int(.1e6), polarity=0, phase=0, bits=8, firstbit = SoftSPI.MSB, sck = pSCK, miso = pMISO, mosi = pMOSI)

CC1101_433 = TICC1101(spi, pCS=pCS433, pGDO0=pGDO0433, pGDO2=pGDO2433, debug = False)
CC1101_433.reset()
CC1101_433.selfTest()

CC1101_868 = TICC1101(spi, pCS=pCS868, pGDO0=pGDO0868, pGDO2=pGDO2868, debug = False)
CC1101_868.reset()
CC1101_868.selfTest()