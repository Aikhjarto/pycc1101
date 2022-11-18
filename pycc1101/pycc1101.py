import time
import math
try:
    from micropython import const
except ImportError:  # probably not running Micropython
    const = lambda x: x

def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

class TICC1101(object):
    WRITE_SINGLE_BYTE = const(0x00)
    WRITE_BURST = const(0x40)
    READ_SINGLE_BYTE = const(0x80)
    READ_BURST = const(0xC0)

    # Configuration Register Details - Registers with preserved values in SLEEP state
    # TI-CC1101 Datasheet

    IOCFG2 = const(0x00)  # GDO2 Output Pin Configuration
    IOCFG1 = const(0x01)  # GDO1 Output Pin Configuration
    IOCFG0 = const(0x02)  # GDO0 Output Pin Configuration
    FIFOTHR = const(0x03)  # RX FIFO and TX FIFO Thresholds
    SYNC1 = const(0x04)  # Sync Word, High Byte
    SYNC0 = const(0x05)  # Sync Word, Low Byte
    PKTLEN = const(0x06)  # Packet Length
    PKTCTRL1 = const(0x07)  # Packet Automation Control
    PKTCTRL0 = const(0x08)  # Packet Automation Control
    ADDR = const(0x09)  # Device Address
    CHANNR = const(0x0A)  # Channel Number
    FSCTRL1 = const(0x0B)  # Frequency Synthesizer Control
    FSCTRL0 = const(0x0C)  # Frequency Synthesizer Control
    FREQ2 = const(0x0D)  # Frequency Control Word, High Byte
    FREQ1 = const(0x0E)  # Frequency Control Word, Middle Byte
    FREQ0 = const(0x0F)  # Frequency Control Word, Low Byte
    MDMCFG4 = const(0x10)  # Modem Configuration
    MDMCFG3 = const(0x11)  # Modem Configuration
    MDMCFG2 = const(0x12)  # Modem Configuration
    MDMCFG1 = const(0x13)  # Modem Configuration
    MDMCFG0 = const(0x14)  # Modem Configuration
    DEVIATN = const(0x15)  # Modem Deviation Setting
    MCSM2 = const(0x16)  # Main Radio Control State Machine Configuration
    MCSM1 = const(0x17)  # Main Radio Control State Machine Configuration
    MCSM0 = const(0x18)  # Main Radio Control State Machine Configuration
    FOCCFG = const(0x19)  # Frequency Offset Compensation Configuration
    BSCFG = const(0x1A)  # Bit Synchronization Configuration
    AGCCTRL2 = const(0x1B)  # AGC Control
    AGCCTRL1 = const(0x1C)  # AGC Control
    AGCCTRL0 = const(0x1D)  # AGC Control
    WOREVT1 = const(0x1E)  # High Byte Event0 Timeout
    WOREVT0 = const(0x1F)  # Low Byte Event0 Timeout
    WORCTRL = const(0x20)  # Wake On Radio Control
    FREND1 = const(0x21)  # Front End RX Configuration
    FREND0 = const(0x22)  # Front End TX Configuration
    FSCAL3 = const(0x23)  # Frequency Synthesizer Calibration
    FSCAL2 = const(0x24)  # Frequency Synthesizer Calibration
    FSCAL1 = const(0x25)  # Frequency Synthesizer Calibration
    FSCAL0 = const(0x26)  # Frequency Synthesizer Calibration
    RCCTRL1 = const(0x27)  # RC Oscillator Configuration
    RCCTRL0 = const(0x28)  # RC Oscillator Configuration

    # Configuration Register Details - Registers that Loose Programming in SLEEP State

    FSTEST = const(0x29)  # Frequency Synthesizer Calibration Control
    PTEST = const(0x2A)  # Production Test
    AGCTEST = const(0x2B)  # AGC Test
    TEST2 = const(0x2C)  # Various Test Settings
    TEST1 = const(0x2D)  # Various Test Settings
    TEST0 = const(0x2E)  # Various Test Settings

    MODULATION_DICT = {"2-FSK": 0b000,
                       "GFSK": 0b001,
                       "ASK": 0b011,
                       "OOK": 0b011,
                       "4-FSK": 0b100,
                       "MSK": 0b111}

    # Command Strobe Registers

    SRES = const(0x30)  # Reset chip
    SFSTXON = const(0x31)  # Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
    # If in RX (with CCA): Go to a wait state where only the synthesizer
    # is running (for quick RX / TX turnaround).

    SXOFF = const(0x32)  # Turn off crystal oscillator.
    SCAL = const(0x33)  # Calibrate frequency synthesizer and turn it off.
    # SCAL can be strobed from IDLE mode without setting manual calibration mode.

    SRX = const(0x34)  # Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
    STX = const(0x35)  # In IDLE state: Enable TX. Perform calibration first
    # if MCSM0.FS_AUTOCAL=1.
    # If in RX state and CCA is enabled: Only go to TX if channel is clear.

    SIDLE = const(0x36)  # Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
    SWOR = const(0x38)  # Start automatic RX polling sequence (Wake-on-Radio)
    # as described in Section 19.5 if WORCTRL.RC_PD=0.

    SPWD = const(0x39)  # Enter power down mode when CSn goes high.
    SFRX = const(0x3A)  # Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
    SFTX = const(0x3B)  # Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
    SWORRST = const(0x3C)  # Reset real time clock to Event1 value.
    SNOP = const(0x3D)  # No operation. May be used to get access to the chip status byte.

    PATABLE = const(0x3E)  # PATABLE
    TXFIFO = const(0x3F)  # TXFIFO
    RXFIFO = const(0x3F)  # RXFIFO

    # Status Register Details

    PARTNUM = const(0xF0)  # Chip ID
    VERSION = const(0xF1)  # Chip ID
    FREQEST = const(0xF2)  # Frequency Offset Estimate from Demodulator
    LQI = const(0xF3)  # Demodulator Estimate for Link Quality
    RSSI = const(0xF4)  # Received Signal Strength Indication
    MARCSTATE = const(0xF5)  # Main Radio Control State Machine State
    WORTIME1 = const(0xF6)  # High Byte of WOR Time
    WORTIME0 = const(0xF7)  # Low Byte of WOR Time
    PKTSTATUS = const(0xF8)  # Current GDOx Status and Packet Status
    VCO_VC_DAC = const(0xF9)  # Current Setting from PLL Calibration Module
    TXBYTES = const(0xFA)  # Underflow and Number of Bytes
    RXBYTES = const(0xFB)  # Overflow and Number of Bytes
    RCCTRL1_STATUS = const(0xFC)  # Last RC Oscillator Calibration Result
    RCCTRL0_STATUS = const(0xFD)  # Last RC Oscillator Calibration Result

    # FSM States
    FSM_SLEEP = const(0x00)
    FSM_IDLE = const(0x01)
    FSM_XOFF = const(0x02)
    FSM_VCOON_MC = const(0x03)
    FSM_REGON_MC = const(0x04)
    FSM_MANCAL = const(0x05)
    FSM_VCOON = const(0x06)
    FSM_REGON = const(0x07)
    FSM_STARTCAL = const(0x08)
    FSM_BWBOOST = const(0x09)
    FSM_FS_LOCK = const(0x0A)
    FSM_IFADCON = const(0x0B)
    FSM_ENDCAL = const(0x0C)
    FSM_RX = const(0x0D)
    FSM_RX_END = const(0x0E)
    FSM_RX_RST = const(0x0F)
    FSM_TXRX_SWITCH = const(0x10)
    FSM_RXFIFO_OVERFLOW = const(0x11)
    FSM_FSTXON = const(0x12)
    FSM_TX = const(0x13)
    FSM_TX_END = const(0x14)
    FSM_RXTX_SWITCH = const(0x15)
    FSM_TXFIFO_UNDERFLOW = const(0x16)
    REFCLK = const(26_000_000)

    # state names according to register value of MARC_STATE
    STATES = ('SLEEP', 'IDLE', 'XOFF', 'VCOON_MC', 'REGON_MC', 'MANCAL', 
              'VCOON', 'REGON', 'STARTCAL', 'BWBOOST', 'FS_LOCK', 'IFADCON', 
              'ENDCAL', 'RX', 'RX_END', 'RX_RST', 'TXRX_SWITCH', 
              'RXFIFO_OVERFLOW', 'FSTXON', 'TX', 'TX_END', 'RXTS_SWITCH')

    def __init__(self, spi, pCS, pGDO0, pGDO2, debug=True):
        self._spi = spi
        self._pCS = pCS
        self._pGDO0 = pGDO0
        self._pGDO2 = pGDO2
        self.debug = debug
        self.freq = 433

    @staticmethod
    def _usDelay(useconds):
        time.sleep(useconds / 1000000.0)

    @staticmethod
    def toBits(byte):
        byte = bin(byte)[2:]
        return "0" * (8 - len(byte)) + byte

    def _write_readinto(self, data_to_write, buf):
        self._pCS.off()
        self._spi.write_readinto(data_to_write, buf)
        self._pCS.on()

    def _writeSingleByte(self, address, byte_data):
        buffer = bytearray(2)
        self._write_readinto(bytearray([self.WRITE_SINGLE_BYTE | address, byte_data]), buffer)
        return buffer

    def _readSingleByte(self, address):
        buffer = bytearray(2)
        self._write_readinto(bytearray([self.READ_SINGLE_BYTE | address, 0x00]), buffer)
        return buffer[1]

    # reads bytes in burst mode
    # returns the read bytes
    def _readBurst(self, addr, length):
        buf = writeBuf = bytearray(length + 1)
        for i in writeBuf:
            writeBuf[i] = (addr + i * 8) | self.READ_BURST

        self._write_readinto(writeBuf, buf)
        return buf

    def _writeBurst(self, address, data):
        data.insert(0, (self.WRITE_BURST | address))
        buf = bytearray(len(data))
        self._write_readinto(bytearray(data), buf)

        if self.debug:
            for dat in buf:
                byte = self.toBits(dat)
                print(
                    "CHIP_RDY: ",
                    str(byte[0]),
                    " STATE: ",
                    str(byte[1:4]),
                    " FIFO- ",
                    int(str(byte[4:]), 2),
                )
        return buf

    # writes burst signal, to send data, and checks if the FIFO is ready for new data
    def _writeBurstTX(self, addr, bytelist):
        bytelist.insert(0, (self.WRITE_BURST | addr))

        # set up the the GDO0
        # Asserts when the TX-FIFO is full
        # De-asserts when the TX-FIFO is drained below TX-FIFO threshold
        self._writeSingleByte(self.IOCFG0, 0x03)

        # set up GDO2
        # Asserts when sync word has been received
        # de-asserts at the end of the packet
        self._writeSingleByte(self.IOCFG2, 0x06)

        # set the TX-STATE
        self._setTXState()

        if self.debug:
            marcstate = self._getMRStateMachineState()
            print("Marcstate: {:d} ".format(marcstate))

        # while self._pGDO2.value() == 0:  # wait until the sync word has been sent
        #     if self.debug:
        #         print("sendBurstRX | Waiting until the sync word has been sent")

        self._pCS.off()

        d_chunks = chunks(bytelist, 64)  # 64...FIFO len in CC1101
        # if self._pGDO2.value() == 1:
        if True:
            for d_chunk in d_chunks:
#             for b in bytelist:
                while self._pGDO0.value() == 1:
                    if self.debug:
                        print("Waiting to send new data to the FIFO!")
                        self._usDelay(100)

                buf = bytearray(len(d_chunk))
                self._spi.write_readinto(bytearray(d_chunk), buf)  # write the data to the TX-FIFO

                if buf[0] & 0x70 == 0x70:  # check if there is an underflow
                    if self.debug:
                        print("TX-FIFO-UNDERFLOW")
                    self._flushTXFifo()
                    return False

                if self.debug:
                    byte = self.toBits(buf[0])
                    print("Send data: ", d_chunk)
                    print(
                        "CHIP_RDY: ",
                        str(byte[0]),
                        " STATE: ",
                        str(byte[1:4]),
                        " FIFO- ",
                        int(str(byte[4:]), 2),
                    )
        self._pCS.on()
        return True

    # writes burst signal, to send data, and checks if the FIFO is ready for new data
    def _readBurstRX(self, addr, length):
        buff = []
        ret = []

        buff.append(0xFF)

        for _ in range(length):
            buff.append(0)

        # set up GDO0
        # Asserts when the RX-FIFO is filled above the threshold or the end of packet is reached
        # De-asserts when the RX-FIFO is empty
        self._writeSingleByte(self.IOCFG0, 0x00)

        # set up GDO2
        # Asserts when sync word has been received
        # de-asserts at the end of the packet
        self._writeSingleByte(self.IOCFG2, 0x06)

        if self.debug:
            print('Waiting for sync word')
            
        while self._pGDO2.value() == 0:  # wait until the sync word has been received        
            marcstate = self._getMRStateMachineState()
            if marcstate != self.FSM_RX:            
                if self.debug:
                    print("Marcstate in _readBurstRX: {:d} ".format(marcstate))
            if marcstate == self.FSM_IDLE:
                self._setRXState()  # go back to RX state

        if self.debug:
            print('Sync word found!')

        if length < 64:  # reveive the complete packet
            numBytes = self._readSingleByte(self.RXBYTES)
            while self._pGDO2.value() == 1 and numBytes != length:
                if self.debug:
                    print(
                        "Waiting until the complete packet has been reveived-", numBytes
                    )
                    self._usDelay(100)
                numBytes = self._readSingleByte(self.RXBYTES)

            self._pCS.off()
            for d in buff:
                data = bytearray(1)

                self._spi.write_readinto(bytearray([d]), data)
                ret.append(int(data[0]))
                if self.debug:
                    print(
                        "Data: ",
                        data[0],
                        "GPO0 = ",
                        self._pGDO0.value(),
                        "GPO2 = ",
                        self._pGDO2.value(),
                    )
        else:
            numBytes = self._readSingleByte(self.RXBYTES)
            remDat = length
            if numBytes > 0 and (
                self._getMRStateMachineState() != self.FSM_RXFIFO_OVERFLOW
            ):
                self._pCS.off()
                for d in buff:
                    data = bytearray(1)

                    while (
                        self._pGDO0.value() == 0 and remDat > 30
                    ):  # wait until the fifo were filled
                        pass

                    self._spi.write_readinto(bytearray([d]), data)
                    if self.debug:
                        print(
                            "Data: ",
                            data[0],
                            "GPO0 = ",
                            self._pGDO0.value(),
                            "GPO2 = ",
                            self._pGDO2.value(),
                        )
                    ret.append(data[0])
                    remDat -= 1

        self._pCS.on()
        return ret

    def reset(self):
        return self._strobe(self.SRES)

    def _strobe(self, address):
        buf = bytearray(2)
        self._write_readinto(bytearray([address, 0x00]), buf)
        return buf[1]

    def selfTest(self):
        part_number = self._readSingleByte(self.PARTNUM)
        component_version = self._readSingleByte(self.VERSION)

        if self.debug:
            print("Part Number: {:x}".format(part_number))
            print("Component Version: {:x}".format(component_version))

        # These asserts are based on the documentation
        # Section 29.3 "Status Register Details"
        # On reset PARTNUM == 0x00
        # On reset VERSION == 0x14

        assert part_number == 0x00, "Self test: Wrong part number"
        assert component_version == 0x14, "Self test: Wrong component version"

        self._writeSingleByte(self.IOCFG0, 0x2f)  # set to low
        assert self._pGDO0.value() == 0, "Self test: GDO0 should be low"
        self._writeSingleByte(self.IOCFG0, 0x2f | 1<<6)  # set to high
        assert self._pGDO0.value() == 1, "Self test: GDO0 should be high"
        self._writeSingleByte(self.IOCFG0, 0x2b)  # osc stable
        assert self._pGDO0.value() == 1, "Self test: GDO0 returns 'Osc unstable'"

        self._writeSingleByte(self.IOCFG2, 0x2f)  # set to low
        assert self._pGDO2.value() == 0, "GDO2 should be low"
        self._writeSingleByte(self.IOCFG2, 0x2f | 1<<6)  # set to high
        assert self._pGDO2.value() == 1, "GDO2 should be high"
        self._writeSingleByte(self.IOCFG2, 0x2b)  # osc stable
        assert self._pGDO2.value() == 1, "Self test: GDO2 returns 'Osc unstable'"

        if self.debug:
            print("Self test OK")


    def _sidle(self):
        self._strobe(self.SIDLE)
        while self._readSingleByte(self.MARCSTATE) != self.FSM_IDLE:
            self._usDelay(100)

    def powerDown(self):
        self._sidle()
        self._strobe(self.SPWD)

    def calibrate(self):
        """Manual calibration (CC1101 will be idle afterwards)"""
        self._sidle()
        self._strobe(self.SCAL)
    
    def setFSAutoCal(self, mode):
        """
        0 Never (manually calibrate using SCAL strobe)
        1 When going from IDLE to RX or TX (or FSTXON)
        2 When going from RX or TX back to IDLE automatically
        3 Every 4th time when going from RX or TX to IDLE automatically
        """
        assert(0<=mode<=0x03)
        tmp = self._readSingleByte(self.MCSM0)
        tmp = (tmp & 0b11001111) | mode << 4
        self._writeSingleByte(self.MCSM0, tmp)

    def getFSAutoCal(self):
        return (self._readSingleByte(self.MCSM0) & 0b00110000) >> 4

    def setCarrierFrequency(self, freq=433):
        # Register values extracted from SmartRF Studio 7
        if freq == 433:
            self._writeSingleByte(self.FREQ2, 0x10)
            self._writeSingleByte(self.FREQ1, 0xA7)
            self._writeSingleByte(self.FREQ0, 0x62)
            self.freq = 433
        elif freq == 868:
            self._writeSingleByte(self.FREQ2, 0x21)
            self._writeSingleByte(self.FREQ1, 0x62)
            self._writeSingleByte(self.FREQ0, 0x76)
            self.freq = 868
        else:
            raise Exception("Only 433MHz and 868MHz are currently supported")

    def setCarrierFrequencyHz(self, freq):
        reg_val = int(freq*2**16/self.REFCLK)
        assert(0<=reg_val<=0xFFFFFF)
        self._writeSingleByte(self.FREQ2, reg_val >> 16)
        self._writeSingleByte(self.FREQ1, (reg_val & 0xFFFF) >> 8)
        self._writeSingleByte(self.FREQ0, (reg_val & 0xFF))

    def getCarrierFrequency(self):
        return self.REFCLK * (
            (self._readSingleByte(self.FREQ2) << 16) +
            (self._readSingleByte(self.FREQ1) << 8) + 
            (self._readSingleByte(self.FREQ0))
            ) / 2**16

    def setChannel(self, channel=0x00):
        self._writeSingleByte(self.CHANNR, channel)

    def getChannel(self):
        return self._readSingleByte(self.CHANNR)
    
    def getCenterFrequency(self):
        return self.getCarrierFrequency()+self.getChannelSpacing()*self.getChannel()
        
    def setRXAttenuation(self, value):
        """Attenuation is value*6dB"""
        assert(0<=value<=3)
        tmp = self._readSingleByte(self.FIFOTHR)
        tmp = (tmp & 0b11001111) | (value << 4)
        self._writeSingleByte(self.FIFOTHR, tmp)

    def getRXAttenuationdB(self):
        return ((self._readSingleByte(self.FIFOTHR) & 0b00110000) >> 4)*6

    def setFIFOThreshold(self, percentage):
        assert 0x00<=percentage<=0x0F
        tmp = self._readSingleByte(self.FIFOTHR)
        tmp = (tmp & 0b00001111) | percentage
        self._writeSingleByte(self.FIFOTHR)

    def getRXFIFOThreshold(self):
        tmp = self._readSingleByte(self.FIFOTHR) & 0b00001111
        return 4 + tmp * 4
    
    def getTXFIFOThreshold(self):
        tmp = self._readSingleByte(self.FIFOTHR) & 0b00001111
        return 61 - tmp * 4

    # Sets the sync-word - automatically added at the start
    # of the packet by the CC1101 transceiver
    def setSyncWord(self, sync_word="D391"):
        assert len(sync_word) == 4

        self._writeSingleByte(self.SYNC1, int(sync_word[:2], 16))  # High Byte
        self._writeSingleByte(self.SYNC0, int(sync_word[2:], 16))  # Low Byte

    # Returns the configuration of a specific register as a bit-represented string
    def getRegisterConfiguration(self, register, showConfig=True):
        def toBits(byte):
            binary = bin(byte)[2:]
            binary = "0" * (8 - len(binary)) + binary
            return binary

        if register == "PKTCTRL1":
            bits = toBits(self._readSingleByte(self.PKTCTRL1))

            if showConfig:
                print("\n**PKTCTRL1**")
                print("PQT[7:5] = {}".format(bits[:3]))
                print("CRC_AUTOFLUSH = {}".format(bits[4]))
                print("APPEND_STATUS = {}".format(bits[5]))
                print("ADR_CHK[1:0] = {}".format(bits[6:]))

        elif register == "PKTCTRL0":
            bits = toBits(self._readSingleByte(self.PKTCTRL0))

            if showConfig:
                print("\n**PKTCTRL0**")
                print("WHITE_DATA = {}".format(bits[1]))
                print("PKT_FORMAT[1:0] = {}".format(bits[2:4]))
                print("CRC_EN = {}".format(bits[5]))
                print("LENGTH_CONFIG[1:0] = {}".format(bits[6:]))

        elif register == "ADDR":
            bits = toBits(self._readSingleByte(self.ADDR))

            if showConfig:
                print("\n**ADDR**")
                print("DEVICE_ADDR = {}".format(bits))

        elif register == "CHANNR":
            bits = toBits(self._readSingleByte(self.CHANNR))

            if showConfig:
                print("\n**CHANNR**")
                print("CHAN = {}".format(bits))

        elif register == "PKTSTATUS":
            bits = toBits(self._readSingleByte(self.CHANNR))

            if showConfig:
                print("\n**PKTSTATUS**")
                print("CRC_OK = {}".format(bits[0]))
                print("CS = {}".format(bits[1]))
                print("PQT_REACHED = {}".format(bits[2]))
                print("CCA = {}".format(bits[3]))
                print("SFD = {}".format(bits[4]))
                print("GDO2 = {}".format(bits[5]))
                print("GDO0 = {}".format(bits[7]))

        elif register == "MDMCFG2":
            bits = toBits(self._readSingleByte(self.MDMCFG2))

            if showConfig:
                print("\n**MDMCFG2**")
                print("DEM_DCFILT_OFF = {}".format(bits[0]))
                print("MOD_FORMAT = {}".format(bits[1:4]))
                print("MANCHESTER_EN = {}".format(bits[4]))
                print("SYNC_MODE = {}".format(bits[5:]))

        elif register == "MDMCFG1":
            bits = toBits(self._readSingleByte(self.MDMCFG1))

            if showConfig:
                print("\n**MDMCFG1**")
                print("FEC_EN = {}".format(bits[0]))
                print("NUM_PREAMBLE = {}".format(bits[1:4]))
                print("CHANSPC_E = {}".format(bits[6:]))

        elif register == "MCSM1":
            bits = toBits(self._readSingleByte(self.MCSM1))

            if showConfig:
                print("\n**MCSM1**")
                print("MCSM1 = {}".format(bits))
                print("CCA_MODE = {}".format(bits[2:4]))
                print("RXOFF_MODE = {}".format(bits[4:6]))
                print("TXOFF_MODE = {}".format(bits[6:]))

        return bits

    def setDefaultValues(self):

        # Default values extracted from Smart RF Studio 7

        self._writeSingleByte(self.IOCFG2, 0x2E)    # Panstamp
        self._writeSingleByte(self.IOCFG1, 0x2E)    # Panstamp
        self._writeSingleByte(self.IOCFG0, 0x06)    # Panstamp
        self._writeSingleByte(self.FIFOTHR, 0x07)   # Panstamp
        self._writeSingleByte(self.PKTLEN, 20)
        self._writeSingleByte(self.PKTCTRL1, 0x06)  # Panstamp
        self._writeSingleByte(self.PKTCTRL0, 0x04)  # Panstamp

        self.setSyncWord()
        self.setChannel()
        self.configureAddressFiltering()

        self._writeSingleByte(self.FSCTRL1, 0x08)   # Panstamp
        self._writeSingleByte(self.FSCTRL0, 0x00)   # Panstamp

        self.setCarrierFrequency()

        self._writeSingleByte(self.MDMCFG4, 0xCA)   # Panstamp
        self._writeSingleByte(self.MDMCFG3, 0x83)   # Panstamp
        self._writeSingleByte(self.MDMCFG2, 0x93)   # Panstamp
        self._writeSingleByte(self.MDMCFG1, 0x22)
        self._writeSingleByte(self.MDMCFG0, 0xF8)

        self._writeSingleByte(self.DEVIATN, 0x35)   # Panstamp
        self._writeSingleByte(self.MCSM2, 0x07)
        self._writeSingleByte(self.MCSM1, 0x20)     # Panstamp
        self._writeSingleByte(self.MCSM0, 0x18)
        self._writeSingleByte(self.FOCCFG, 0x16)
        self._writeSingleByte(self.BSCFG, 0x6C)
        self._writeSingleByte(self.AGCCTRL2, 0x43)  # Panstamp
        self._writeSingleByte(self.AGCCTRL1, 0x40)
        self._writeSingleByte(self.AGCCTRL0, 0x91)
        self._writeSingleByte(self.WOREVT1, 0x87)
        self._writeSingleByte(self.WOREVT0, 0x6B)
        self._writeSingleByte(self.WORCTRL, 0xFB)
        self._writeSingleByte(self.FREND1, 0x56)
        self._writeSingleByte(self.FREND0, 0x10)
        self._writeSingleByte(self.FSCAL3, 0xE9)
        self._writeSingleByte(self.FSCAL2, 0x2A)
        self._writeSingleByte(self.FSCAL1, 0x00)
        self._writeSingleByte(self.FSCAL0, 0x1F)
        self._writeSingleByte(self.RCCTRL1, 0x41)
        self._writeSingleByte(self.RCCTRL0, 0x00)
        self._writeSingleByte(self.FSTEST, 0x59)
        self._writeSingleByte(self.PTEST, 0x7F)
        self._writeSingleByte(self.AGCTEST, 0x3F)
        self._writeSingleByte(self.TEST2, 0x81)
        self._writeSingleByte(self.TEST1, 0x35)
        self._writeSingleByte(self.TEST0, 0x09)
        self._writeSingleByte(0x3E, 0xC0)           # Power 10dBm

    def setSyncMode(self, syncmode):
        regVal = self._readSingleByte(self.MDMCFG2)
        
        if syncmode > 7:
            raise Exception("Invalid SYNC mode")

        regVal = (regVal & 0xFC) | syncmode
        self._writeSingleByte(self.MDMCFG2, regVal)
        
    def getSyncMode(self):
        regVal = self._readSingleByte(self.MDMCFG2)
        return regVal & 0x03

    def setGDO0Cfg(self, reg_val):
        assert(0<=reg_val<=0b00111111)
        tmp = self._readSingleByte(self.IOCFG0)
        self._writeSingleByte(self.IOCFG0, (tmp & 0b11000000) | reg_val)

    def setGDO1Cfg(self, reg_val):
        assert(0<=reg_val<=0b00111111)
        tmp = self._readSingleByte(self.IOCFG1)
        self._writeSingleByte(self.IOCFG1, (tmp & 0b11000000) | reg_val)  

    def setGDO2Cfg(self, reg_val):
        assert(0<=reg_val<=0b00111111)
        tmp = self._readSingleByte(self.IOCFG2)
        self._writeSingleByte(self.IOCFG2, (tmp & 0b11000000) | reg_val)  
    
    def setAsynchronousTransparentRXMode(self):
        """From SmartRF Studio: RX data without sanity checks is sent do GPO2"""
        self._writeSingleByte(self.MDMCFG2, 0x00)
        self._writeSingleByte(self.PKTCTRL0, 0x32)
        self.setGDO0Cfg(0x0D)
        self.setGDO2Cfg(0x0D)

    def setSynchronousTransparentRXMode(self):
        """From SmartRF Studio: RX data without sanity checks is sent do GPO2 with a clock sent to GPO0"""
        self._writeSingleByte(self.MDMCFG2, 0x00)
        self._writeSingleByte(self.PKTCTRL0, 0x12)
        self.setGDO0Cfg(0x0C)
        self.setGDO2Cfg(0x0B)

    def setModulation(self, modulation):
        modVal = self.MODULATION_DICT[modulation]
        tmp = self._readSingleByte(self.MDMCFG2) 
        tmp = (tmp & 0b10001111) | (modVal << 4)
        self._writeSingleByte(self.MDMCFG2, tmp)

    def getModulation(self):
        tmp = self._readSingleByte(self.MDMCFG2)
        tmp = (tmp & 0b01110000) >> 4
        for key, val in self.MODULATION_DICT.items():
            if val == tmp:
                return key

    def setOptimumASKGainControl(self, MAGN_TARGET=0x05, DECISION_BOUNARY=0x01):
        """
        Automatic gain control settings from SmartRF are not considered 
        optimal by Texas Instruments for ASK mode any more. 
        See: http://www.ti.com/lit/swra215

        Must be called after channel bandwidth is changed when ASK signals
        should be received
        """
        assert(0x03 <= MAGN_TARGET <=0x07)
        assert(0x01 <= DECISION_BOUNARY <= 0x02)
        self._writeSingleByte(self.AGCCTRL2, MAGN_TARGET) # between 0x03 and 0x07
        self._writeSingleByte(self.AGCCTRL1, 0x00)
        self._writeSingleByte(self.AGCCTRL0, 0x90+DECISION_BOUNARY) #0x91 or 0x92

        RX_FILTER_BANDWIDTH_HZ=self.getChannelBandwidth()

        # FSCTRL1 should be first possible value greater than RX_FILTER_BANDWIDTH_HZ
        val = int((RX_FILTER_BANDWIDTH_HZ*2**10)/self.REFCLK)+1
        val = max(1,min(val,15))
        self._writeSingleByte(self.FSCTRL1, val)

        if RX_FILTER_BANDWIDTH_HZ > 101_000:
            self._writeSingleByte(self.FREND1, 0xB6)
        else:
            self._writeSingleByte(self.FREND1, 0x56)
        
        if RX_FILTER_BANDWIDTH_HZ > 325_000:
            self._writeSingleByte(self.TEST2, 0x88)
            self._writeSingleByte(self.TEST1, 0x31)
            self._writeSingleByte(self.FIFOTHR, 0x07)
        else:
            self._writeSingleByte(self.TEST2, 0x81)
            self._writeSingleByte(self.TEST2, 0x35)
            self._writeSingleByte(self.FIFOTHR, 0x47)

    def _flushRXFifo(self):
        self._strobe(self.SFRX)
        self._usDelay(2)

    def _flushTXFifo(self):
        self._strobe(self.SFTX)
        self._usDelay(2)

    def _setTXState(self):
        self._strobe(self.STX)
        self._usDelay(2)

    def _setRXState(self):
        self._strobe(self.SRX)
        self._usDelay(2)

    def getRSSI(self):
        return self._readSingleByte(self.RSSI)

    def getRSSIdBm(self):
        rssi = self.getRSSI()
        if rssi>=128:
            return (rssi-256)/2-74
        return rssi/2 - 74

    def _getMRStateMachineState(self):
        # The &0x1F works as a mask due to the fact
        # that the MARCSTATE register only uses the
        # first 5 bits
        return self._readSingleByte(self.MARCSTATE) & 0x1F

    def getMRStateMachineState(self):
        state = self._getMRStateMachineState()
        if self.debug:
            print(self.STATES[state])
        return state

    # Returns the packet configuration
    def getPacketConfigurationMode(self):
        pktCtrlVal = self.getRegisterConfiguration("PKTCTRL0", False)

        if pktCtrlVal[6:] == "00": # Packet len is fixed
            return "PKT_LEN_FIXED"

        if pktCtrlVal[6:] == "01": # Packet len is variable
            return "PKT_LEN_VARIABLE"

        if pktCtrlVal[6:] == "10":  # Infinite packet len mode
            return "PKT_LEN_INFINITE"

        return "ERROR_PKT_LEN"

    # Sets the packet configuration
    # PKT_LEN_FIXED: Fixed packet length mode.
    #               Length configured in PKTLEN register
    # PKT_LEN_VARIABLE: Variable packet length mode.
    #                   Packet length configured by the
    #                   first byte after sync word
    # PKT_LEN_INFINITE: Infinite packet length mode
    def setPacketMode(self, mode="PKT_LEN_VARIABLE"):
        regVal = list(self.getRegisterConfiguration("PKTCTRL0", False))

        if mode == "PKT_LEN_FIXED":
            val = "00"

        elif mode == "PKT_LEN_VARIABLE":
            val = "01"

        elif mode == "PKT_LEN_INFINITE":
            val = "10"

        else:
            raise Exception("Packet mode NOT SUPPORTED!")

        regVal[6:] = list(val)
        regVal = int("".join(regVal), 2)
        self._writeSingleByte(self.PKTCTRL0, regVal)

    # Sets the filtering address used for packet filtration
    def setFilteringAddress(self, address=0x0E):
        self._writeSingleByte(self.ADDR, address)

    # Sets the address check configuration
    # DISABLED: No address checking
    # ENABLE_NO_BROADCAST: Address check, no broadcast
    # ENABLE_00_BROADCAST: Address check and 0 broadcast
    # ENABLE_00_255_BROADCAST: Address check and 0 and 255 broadcast
    def configureAddressFiltering(self, value="DISABLED"):
        regVal = list(self.getRegisterConfiguration("PKTCTRL1", False))

        if value == "DISABLED":
            val = "00"

        elif value == "ENABLED_NO_BROADCAST":
            val = "01"

        elif value == "ENABLED_00_BROADCAST":
            val = "10"

        elif value == "ENABLED_00_255_BROADCAST":
            val = "11"

        else:
            raise Exception("Address filtering configuration NOT SUPPORTED!")

        # Set ADR_CHK (1:0)
        regVal[6:] = list(val)

        regVal = int("".join(regVal), 2)
        self._writeSingleByte(self.PKTCTRL1, regVal)

    def sendData(self, dataBytes):
        self._flushTXFifo()
        self._setRXState()
        marcstate = self._getMRStateMachineState()
        dataToSend = []

        if len(dataBytes) == 0:
            raise ValueError("Must include payload")

        while (marcstate & 0x1F) != 0x0D:
            if self.debug:
                print("marcstate = {:x}".format(marcstate))
                print("waiting for marcstate == 0x0D")

            if marcstate == 0x11:
                self._flushRXFifo()

            marcstate = self._getMRStateMachineState()

        sending_mode = self.getPacketConfigurationMode()
        data_len = len(dataBytes)

        if sending_mode == "PKT_LEN_FIXED":
            if data_len > self._readSingleByte(self.PKTLEN):
                raise ValueError("Payload exceeds PKTLEN.")

            if self.getRegisterConfiguration("PKTCTRL1", False)[6:] != "00":
                dataToSend.append(self._readSingleByte(self.ADDR))

            dataToSend.extend(dataBytes)
            dataToSend.extend([0] * (self._readSingleByte(self.PKTLEN) - len(dataToSend)))

            if self.debug:
                print("Sending a fixed len packet")
                print("data len = {:d}".format((data_len)))

        elif sending_mode == "PKT_LEN_VARIABLE":
            dataToSend.append(data_len)

            if self.getRegisterConfiguration("PKTCTRL1", False)[6:] != "00":
                dataToSend.append(self._readSingleByte(self.ADDR))
                dataToSend[0] += 1

            dataToSend.extend(dataBytes)

            if self.debug:
                print("Sending a variable len packet")
                print("Length of the packet is: {:d}".format(data_len))

        elif sending_mode == "PKT_LEN_INFINITE":
            if self.getRegisterConfiguration("PKTCTRL1", False)[6:] != "00":
                dataToSend.append(self._readSingleByte(self.ADDR))

            data_len = data_len + len(dataToSend) + 2
            data_len_fixed = data_len % 256
            data_len_inf = data_len - data_len_fixed

            dataToSend.append(data_len_fixed)
            data_len_inf_app = int(data_len_inf / 256)

            dataToSend.append(data_len_inf_app)
            # extend the list with the data to send
            dataToSend.extend(dataBytes)

            data_len = len(dataToSend)

            if data_len > 255:
                state = self._writeBurstTX(self.TXFIFO, dataToSend[0 : (data_len - data_len_fixed)])
                if not state:
                    self._flushTXFifo()
                    self._sidle()
                    return False

                remaining_bytes = self._readSingleByte(self.TXBYTES) & 0x7F
                while remaining_bytes != 0:
                    self._usDelay(1000)
                    remaining_bytes = self._readSingleByte(self.TXBYTES) & 0x7F
                    if self.debug:
                        print("Waiting inf until all bytes are transmitted, remaining bytes: {:d}".format(remaining_bytes))

                dataToSend = dataToSend[data_len - data_len_fixed :]

                self._writeSingleByte(self.PKTLEN, len(dataToSend))
                self.setPacketMode("PKT_LEN_FIXED")

            else:
                self._writeSingleByte(self.PKTLEN, len(dataToSend))
                self.setPacketMode("PKT_LEN_FIXED")

        if self.debug:
            print(dataToSend)
        state = self._writeBurstTX(self.TXFIFO, dataToSend)

        if not state:
            self._flushTXFifo()
            self._sidle()
            return False

        remaining_bytes = self._readSingleByte(self.TXBYTES) & 0x7F
        while remaining_bytes != 0:
            self._usDelay(1000)
            remaining_bytes = self._readSingleByte(self.TXBYTES) & 0x7F
            if self.debug:
                print("Waiting until all bytes are transmited, remaining bytes: {:d}".format(remaining_bytes))


        if (self._readSingleByte(self.TXBYTES) & 0x7F) == 0:
            if self.debug:
                print("Packet sent!")
            return True

        if self.debug:
            print("{}".format(self._readSingleByte(self.TXBYTES) & 0x7F))
            print("sendData | MARCSTATE: {:x}".format(self._getMRStateMachineState()))
            self._sidle()
            self._flushTXFifo()
            time.sleep(5)
            self._setRXState()
        return False

    def recvData(self):
        self._setRXState()
        self._usDelay(100)
        rx_bytes_val = self._readSingleByte(self.RXBYTES)  # get the number of bytes in the fifo
        data = []

        #if rx_bytes_val has something and overflow bit is not 1
        if rx_bytes_val & 0x7F and not (rx_bytes_val & 0x80):
            sending_mode = self.getPacketConfigurationMode()
            valPktCtrl1 = self.getRegisterConfiguration("PKTCTRL1", False)
            if sending_mode == "PKT_LEN_FIXED":
                data_len = self._readSingleByte(self.PKTLEN)
                data = self._readBurstRX(self.RXFIFO, data_len)

                # check if there is an address-check
                if self.getRegisterConfiguration("PKTCTRL1", False)[6:] != "00":
                    pass
                else:
                    data = data[1:]
            elif sending_mode == "PKT_LEN_VARIABLE":
                max_len = self._readSingleByte(self.PKTLEN)
                data_len = self._readSingleByte(self.RXFIFO)

                if data_len > max_len:
                    if self.debug:
                        print("Len of data exceeds the configured maximum packet len")
                    self._flushRXFifo()
                    self._sidle()
                    return False

                if self.debug:
                    print("Receiving a variable len packet")
                    print("max len: {:d}".format(max_len))
                    print("Packet length: {:d}".format(data_len))

                data = self._readBurstRX(self.RXFIFO, data_len)

                # check if there is an address-check
                if self.getRegisterConfiguration("PKTCTRL1", False)[6:] != "00":
                    data = data[2:]
                else:
                    data = data[1:]
            elif sending_mode == "PKT_LEN_INFINITE":
                print("Mode: PKT_LEN_INFINITE work in progress")
                if rx_bytes_val > 0:

                    # self._readSingleByte(self.RXFIFO)
                    if self.getRegisterConfiguration("PKTCTRL1", False)[6:] != "00":
                        self._readSingleByte(self.RXFIFO)
                        # dataAdd = self._readSingleByte(self.RXFIFO)
                    dataLen = self._readSingleByte(self.RXFIFO)
                    dataLen2 = self._readSingleByte(self.RXFIFO)

                    data_len = dataLen + 0 * dataLen2
                    if self.debug:
                        print("Receiving a infinite len packet-infinite")
                        print("dataLen2: {:d} dataLen: {:d}".format(dataLen2, dataLen))

                    if data_len > 255:
                        print("Data length: {:d}".format(data_len))
                        data_len2 = 256 * dataLen2
                        data = self._readBurstRX(self.RXFIFO, data_len2)[1:]

                    self._writeSingleByte(self.PKTLEN, dataLen)
                    self.setPacketMode("PKT_LEN_FIXED")
                    data2 = self._readBurstRX(self.RXFIFO, dataLen - 3)[1:]
                    if not data2:
                        self._flushRXFifo()
                        self._sidle()
                        if self.debug:
                            print("RX-FIFO-ERROR")
                        return False
                    data.extend(data2)
                else:
                    self._flushRXFifo()
                    self._sidle()
                    if self.debug:
                        print("RX-FIFO-ERROR")
                    return False

            # if self.getRegisterConfiguration("PKTCTRL1", False)[6:] != "00":
            #   data = data
            if valPktCtrl1[5] == "1":  # PKTCTRL1[5] == APPEND_STATUS
            # When enabled, two status bytes will be appended to the payload of the
            # packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

                rssi = self._readSingleByte(self.RXFIFO)
                val = self._readSingleByte(self.RXFIFO)
                lqi = val & 0x7F

            if self.debug and valPktCtrl1[5] == "1":
                print("Marcstate after recv: ", self._getMRStateMachineState())
                print("Packet information is enabled")
                print("RSSI: {:d}".format((rssi)))
                print("VAL: {:d}".format((val)))
                print("LQI: {:d}".format((lqi)))
                print("Data: " + str(data))
            self._flushRXFifo()
            return data
        # if self.debug:
        # print("RX-Bytes-Error: ", rx_bytes_val)
        return False

    #Sets the baudrate (datasheet p. 35)
    def setBaud(self, baudrate):
        DRATE_E = int(math.log2((baudrate*(2**20))/self.REFCLK))
        DRATE_M = int((baudrate*(2**28))/(self.REFCLK*(2**DRATE_E)))-256
        if DRATE_M == 256:
            DRATE_M = 0
            DRATE_E += 1
        mdmcfg4_tmp = self._readSingleByte(self.MDMCFG4)
        self._writeSingleByte(self.MDMCFG4, (mdmcfg4_tmp & 0xF0) | DRATE_E)  # lower 4 bits for DRATE
        self._writeSingleByte(self.MDMCFG3, DRATE_M)  # all bits for DRATE
    
    def getBaud(self):
        DRATE_E = self._readSingleByte(self.MDMCFG4) & 0b00000111
        DRATE_M = self._readSingleByte(self.MDMCFG3)
        return ((256+DRATE_M) * 2**DRATE_E)/2**28*self.REFCLK

    def setChannelBandwidth(self, CHANBW_M, CHANBW_E):
        assert(0<=CHANBW_M<=3)
        assert(0<=CHANBW_E<=3)
        mdmcfg4_tmp = self._readSingleByte(self.MDMCFG4)
        mdmcfg4_tmp = (mdmcfg4_tmp & 0b11001111) | (CHANBW_M << 4)
        mdmcfg4_tmp = (mdmcfg4_tmp & 0b00111111) | (CHANBW_E << 6)
        self._writeSingleByte(self.MDMCFG4, mdmcfg4_tmp)
    
    def getChannelBandwidth(self):
        mdmcfg4_tmp = self._readSingleByte(self.MDMCFG4)
        CHANBW_E = mdmcfg4_tmp >> 6
        CHANBW_M = (mdmcfg4_tmp & 0b00110000) >> 4        
        return self.REFCLK/(8*(4+CHANBW_M)*2**CHANBW_E)
    
    def setChannelSpacing(self, CHANSPC_M, CHANSPC_E):
        assert(0<=CHANSPC_E<=3)
        assert(0<=CHANSPC_M<=255)
        mdmcfg1_tmp = self._readSingleByte(self.MDMCFG1)
        mdmcfg1_tmp = (mdmcfg1_tmp & 0xFC) | CHANSPC_E
        self._writeSingleByte(self.MDMCFG0, CHANSPC_M)
        self._writeSingleByte(self.MDMCFG1, mdmcfg1_tmp)
                
    def getChannelSpacing(self):
        CHANSPC_M = self._readSingleByte(self.MDMCFG0)
        CHANSPC_E = self._readSingleByte(self.MDMCFG1) & 0x03
        return self.REFCLK/(2**18)*(256+CHANSPC_M)*2**(CHANSPC_E)  
        
    #Turn data whitening on / off (datasheet p. 74)
    def enWhiteData(self, enable):
        pktctrl0_tmp = self._readSingleByte(self.PKTCTRL0)
        if enable:
            self._writeSingleByte(self.PKTCTRL0, (pktctrl0_tmp) | 1<<6)
        else:
            self._writeSingleByte(self.PKTCTRL0, (pktctrl0_tmp & (~(1<<6))))

    #CRC calculation in TX and CRC check in RX (datasheet p. 74)
    def enCRC(self, enable):
        pktctrl0_tmp = self._readSingleByte(self.PKTCTRL0)
        if enable:
            self._writeSingleByte(self.PKTCTRL0, (pktctrl0_tmp) | 1<<2)
        else:
            self._writeSingleByte(self.PKTCTRL0, (pktctrl0_tmp & (~(1<<2))))

    #Enable Forward Error Correction (FEC) with interleaving for packet payload (datasheet p. 78)
    def enFEC(self, enable):
        mdmcfg1_tmp = self._readSingleByte(self.MDMCFG1)
        if enable:
            self._writeSingleByte(self.MDMCFG1, (mdmcfg1_tmp) | 1<<7)
        else:
            self._writeSingleByte(self.MDMCFG1, (mdmcfg1_tmp & (~(1<<7))))

    #Indicates the packet length when fixed packet length mode is enabled.
    #If variable packet length mode is used, this value indicates the
    #maximum packet length allowed
    def setPktLen(self, length):
        if 0 < length < 0xFF:
            self._writeSingleByte(self.PKTLEN, (length & 0xFF))
        else:
            raise Exception("Invalid 0<PKTLEN<256")

    #Modem Deviation Setting
    def _setDevtn(self, expon, mant):
        devtn_tmp = self._readSingleByte(self.DEVIATN)
        devtn_tmp = (devtn_tmp & 0b10001111) | expon<<4
        devtn_tmp = (devtn_tmp & 0b11111000) | mant
        self._writeSingleByte(self.DEVIATN, devtn_tmp)

    def setDevtnLarge(self):
        e = 0b101
        m = 0b111
        #deviation = (self.REFCLK/2**17)*(8+m)*2**e = 95214.84375
        self._setDevtn(e,m)

    def setDevtnDefault(self):
        e = 0b100
        m = 0b111
        #deviation = (self.REFCLK/2**17)*(8+m)*2**e = 47607.421875
        self._setDevtn(e,m)

    def setFreqOffset(self, offset):
        if not -129 < offset < 128:
            raise ValueError("Offset needs to be between including -128 and +127")
        self._writeSingleByte(self.FSCTRL0, offset)

    def setPreamble(self, preamble):
        mdmcfg1_tmp = self._readSingleByte(self.MDMCFG1)
        self._writeSingleByte(self.MDMCFG1, (mdmcfg1_tmp & 0b10001111) | preamble<<4)

    def setTXPower(self, paTableIdx):
        #Patable index: -30   -20   -15   -10     0     5     7    10 dBm
        if self.freq == 433:
            patable = (0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0)
        elif self.freq == 868:
            patable = (0x03, 0x0F, 0x1E, 0x27, 0x50, 0x81, 0xCB, 0xC2)
        else:
            raise ValueError("No PA-table for this frequency. Only 433MHz and 868MHz are currently supported")
        self._writeSingleByte(self.PATABLE, patable[paTableIdx])
        frend0_tmp = self._readSingleByte(self.FREND0)
        frend0_tmp &= 0b11111000
        self._writeSingleByte(self.FREND0, frend0_tmp)
