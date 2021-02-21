import time
from pyftdi_upy_wrapper.pyftdi_upy_wrapper import mSPI as SPI, ftdiPin
from pycc1101.pycc1101 import TICC1101

#define the SPI-Interface
spi = SPI(0, 1, 1e6)

#define the ftdi-PINS
pCS = ftdiPin(spi.spi, 4)
pCS.on()
pGDO0 = ftdiPin(spi.spi, 5, direction = 0)
pGDO2 = ftdiPin(spi.spi, 6, direction = 0)

CC1101_433 = TICC1101(spi, pCS=pCS, pGDO0=pGDO0,
                      pGDO2=pGDO2, debug=False)
CC1101_433.reset()
CC1101_433.selfTest()

CC1101_433.setDefaultValues()
CC1101_433.setFreqOffset(10)
# CC1101_433.setFilteringAddress(0x9D)
CC1101_433.configureAddressFiltering("DISABLED")
# CC1101_433.configureAddressFiltering("ENABLED_NO_BROADCAST")
CC1101_433.setChannel(0)
CC1101_433.setPacketMode("PKT_LEN_FIXED")
CC1101_433.setBaud(100000)
CC1101_433.enWhiteData(True)
CC1101_433.enFEC(False)
CC1101_433.enCRC(False)
#CC1101_433.setPreamble(4)
CC1101_433.setModulation("2-FSK")
# CC1101_433.setModulation('4-FSK')
# CC1101_433.setModulation('GFSK')
# CC1101_433.setModulation('ASK')
# CC1101_433.setModulation('MSK')
CC1101_433.setDevtnDefault()
# CC1101_433.setDevtnLarge()
CC1101_433.setPktLen(128)
CC1101_433.setSyncWord("D391")  # 0xD391, 0F0F
CC1101_433.setTXPower(7)

data = [0x00, 0x00, 0xFF, 0xAA]*32

# TX - part
for j in range(1000):
    data[0] = j%256
    success = CC1101_433.sendData(data)
    if not(success):
        print("send: {:04d}, success: {}".format(j, success))
    #time.sleep(0.04)
