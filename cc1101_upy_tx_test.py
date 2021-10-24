import time
from machine import Pin, SoftSPI
from pycc1101.pycc1101 import TICC1101

# ---- eLITe-connect
pCS433 = Pin(Pin.cpu.I10, Pin.OUT)  # 433
pCS433.on()
pGDO0433 = Pin(Pin.cpu.I12, Pin.IN)  # 433
pGDO2433 = Pin(Pin.cpu.I13, Pin.IN)  # 433

pCS868 = Pin(Pin.cpu.I4, Pin.OUT)  # 868
pCS868.on()
pGDO0868 = Pin(Pin.cpu.I7, Pin.IN)  # 868
pGDO2868 = Pin(Pin.cpu.I8, Pin.IN)  # 868

pSCK = Pin(Pin.cpu.K0, Pin.OUT)
pMOSI = Pin(Pin.cpu.J10, Pin.OUT)
pMISO = Pin(Pin.cpu.F8, Pin.IN)
# ---- eLITe-connect

# ---- ESP32
# pCS433 = Pin(22, Pin.OUT) #433
# pCS433.on()
# pGDO0433 = Pin(5, Pin.IN) #433
# pGDO2433 = Pin(18, Pin.IN) #433
# pSCK = Pin(23, Pin.OUT)
# pMOSI = Pin(19, Pin.OUT)
# pMISO = Pin(21, Pin.IN)
# ---- ESP32

spi = SoftSPI(
    baudrate=int(1e6),
    polarity=0,
    phase=0,
    bits=8,
    firstbit=SoftSPI.MSB,
    sck=pSCK,
    miso=pMISO,
    mosi=pMOSI,
)

CC1101_433 = TICC1101(spi, pCS=pCS433, pGDO0=pGDO0433,
                      pGDO2=pGDO2433, debug=False)
CC1101_433.reset()
CC1101_433.selfTest()

CC1101_868 = TICC1101(spi, pCS=pCS868, pGDO0=pGDO0868, pGDO2=pGDO2868, debug=False)
CC1101_868.reset()
CC1101_868.selfTest()

CC1101_433.setDefaultValues()
# CC1101_433.setFilteringAddress(0x9D)
CC1101_433.configureAddressFiltering("DISABLED")
# CC1101_433.configureAddressFiltering("ENABLED_NO_BROADCAST")
CC1101_433.setChannel(0)
CC1101_433.setPacketMode("PKT_LEN_FIXED")
CC1101_433.setBaud(1000)
CC1101_433.enWhiteData(False)
CC1101_433.enFEC(False)
CC1101_433.enCRC(False)
CC1101_433.setModulation("2-FSK")
# CC1101_433.setModulation('4-FSK')
# CC1101_433.setModulation('GFSK')
# CC1101_433.setModulation('ASK')
# CC1101_433.setModulation('MSK')
CC1101_433.setDevtnDefault()
# CC1101_433.setDevtnLarge()
CC1101_433.setPktLen(1)
CC1101_433.setSyncWord("0F0F")  # 0xD391

data = [0xA5]

# TX - part
j = 0
# for j in range(1000):
while True:
    j += 1
    if not(CC1101_433.sendData(data)):
        print("send: {:04d}, {}".format(j, False))
    else:
        print("send: {:04d}, {}".format(j, True))
    time.sleep(0.08)
