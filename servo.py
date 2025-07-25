import time
import serial

BROADCAST_ID = 0xFE  # 254
MAX_ID = 0xFC  # 252
SCS_END = 0

# Instruction for DXL Protocol
INST_PING = 1
INST_READ = 2
INST_WRITE = 3
INST_REG_WRITE = 4
INST_ACTION = 5
INST_SYNC_WRITE = 131  # 0x83
INST_SYNC_READ = 130  # 0x82

# Communication Result
COMM_SUCCESS = 0  # tx or rx packet communication success
COMM_PORT_BUSY = -1  # Port is busy (in use)
COMM_TX_FAIL = -2  # Failed transmit instruction packet
COMM_RX_FAIL = -3  # Failed get status packet
COMM_TX_ERROR = -4  # Incorrect instruction packet
COMM_RX_WAITING = -5  # Now recieving status packet
COMM_RX_TIMEOUT = -6  # There is no status packet
COMM_RX_CORRUPT = -7  # Incorrect status packet
COMM_NOT_AVAILABLE = -9  #

TXPACKET_MAX_LEN = 250
RXPACKET_MAX_LEN = 250

# for Protocol Packet
PKT_HEADER0 = 0
PKT_HEADER1 = 1
PKT_ID = 2
PKT_LENGTH = 3
PKT_INSTRUCTION = 4
PKT_ERROR = 4
PKT_PARAMETER0 = 5

# Protocol Error bit
ERRBIT_VOLTAGE = 1
ERRBIT_ANGLE = 2
ERRBIT_OVERHEAT = 4
ERRBIT_OVERELE = 8
ERRBIT_OVERLOAD = 32


class Connection(object):
    def __init__(self, protocol_end, port):
        self.port = port
        SCS_SETEND(protocol_end)

    def getProtocolVersion(self):
        return 1.0

    def getTxRxResult(self, result):
        if result == COMM_SUCCESS:
            return "[TxRxResult] Communication success!"
        elif result == COMM_PORT_BUSY:
            return "[TxRxResult] Port is in use!"
        elif result == COMM_TX_FAIL:
            return "[TxRxResult] Failed transmit instruction packet!"
        elif result == COMM_RX_FAIL:
            return "[TxRxResult] Failed get status packet from device!"
        elif result == COMM_TX_ERROR:
            return "[TxRxResult] Incorrect instruction packet!"
        elif result == COMM_RX_WAITING:
            return "[TxRxResult] Now receiving status packet!"
        elif result == COMM_RX_TIMEOUT:
            return "[TxRxResult] There is no status packet!"
        elif result == COMM_RX_CORRUPT:
            return "[TxRxResult] Incorrect status packet!"
        elif result == COMM_NOT_AVAILABLE:
            return "[TxRxResult] Protocol does not support this function!"
        else:
            return ""

    def getRxPacketError(self, error):
        if error & ERRBIT_VOLTAGE:
            return "[RxPacketError] Input voltage error!"

        if error & ERRBIT_ANGLE:
            return "[RxPacketError] Angle sen error!"

        if error & ERRBIT_OVERHEAT:
            return "[RxPacketError] Overheat error!"

        if error & ERRBIT_OVERELE:
            return "[RxPacketError] OverEle error!"

        if error & ERRBIT_OVERLOAD:
            return "[RxPacketError] Overload error!"

        return ""

    def txPacket(self, txpacket):
        checksum = 0
        total_packet_length = txpacket[PKT_LENGTH] + 4  # 4: HEADER0 HEADER1 ID LENGTH

        if self.port.is_using:
            return COMM_PORT_BUSY
        self.port.is_using = True

        # check max packet length
        if total_packet_length > TXPACKET_MAX_LEN:
            self.port.is_using = False
            return COMM_TX_ERROR

        # make packet header
        txpacket[PKT_HEADER0] = 0xFF
        txpacket[PKT_HEADER1] = 0xFF

        # add a checksum to the packet
        for idx in range(2, total_packet_length - 1):  # except header, checksum
            checksum += txpacket[idx]

        txpacket[total_packet_length - 1] = ~checksum & 0xFF

        # print "[TxPacket] %r" % txpacket

        # tx packet
        self.port.clearPort()
        written_packet_length = self.port.writePort(txpacket)
        if total_packet_length != written_packet_length:
            self.port.is_using = False
            return COMM_TX_FAIL

        return COMM_SUCCESS

    def rxPacket(self):
        rxpacket = []

        result = COMM_TX_FAIL
        checksum = 0
        rx_length = 0
        wait_length = 6  # minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

        while True:
            rxpacket.extend(self.port.readPort(wait_length - rx_length))
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                # find packet header
                for idx in range(0, (rx_length - 1)):
                    if (rxpacket[idx] == 0xFF) and (rxpacket[idx + 1] == 0xFF):
                        break

                if idx == 0:  # found at the beginning of the packet
                    if (
                        (rxpacket[PKT_ID] > 0xFD)
                        or (rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN)
                        or (rxpacket[PKT_ERROR] > 0x7F)
                    ):
                        # unavailable ID or unavailable Length or unavailable Error
                        # remove the first byte in the packet
                        del rxpacket[0]
                        rx_length -= 1
                        continue

                    # re-calculate the exact length of the rx packet
                    if wait_length != (rxpacket[PKT_LENGTH] + PKT_LENGTH + 1):
                        wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1
                        continue

                    if rx_length < wait_length:
                        # check timeout
                        if port.isPacketTimeout():
                            if rx_length == 0:
                                result = COMM_RX_TIMEOUT
                            else:
                                result = COMM_RX_CORRUPT
                            break
                        else:
                            continue

                    # calculate checksum
                    for i in range(2, wait_length - 1):  # except header, checksum
                        checksum += rxpacket[i]
                    checksum = ~checksum & 0xFF

                    # verify checksum
                    if rxpacket[wait_length - 1] == checksum:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                    break

                else:
                    # remove unnecessary packets
                    del rxpacket[0:idx]
                    rx_length -= idx

            else:
                # check timeout
                if self.port.isPacketTimeout():
                    if rx_length == 0:
                        result = COMM_RX_TIMEOUT
                    else:
                        result = COMM_RX_CORRUPT
                    break

        self.port.is_using = False

        # print "[RxPacket] %r" % rxpacket

        return rxpacket, result

    def txRxPacket(self, txpacket):
        rxpacket = None
        error = 0

        # tx packet
        result = self.txPacket(txpacket)
        if result != COMM_SUCCESS:
            return rxpacket, result, error

        # (ID == Broadcast ID) == no need to wait for status packet or not available
        if txpacket[PKT_ID] == BROADCAST_ID:
            self.port.is_using = False
            return rxpacket, result, error

        # set packet timeout
        if txpacket[PKT_INSTRUCTION] == INST_READ:
            self.port.setPacketTimeout(txpacket[PKT_PARAMETER0 + 1] + 6)
        else:
            self.port.setPacketTimeout(6)  # HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

        # rx packet
        while True:
            rxpacket, result = self.rxPacket()
            if result != COMM_SUCCESS or txpacket[PKT_ID] == rxpacket[PKT_ID]:
                break

        if result == COMM_SUCCESS and txpacket[PKT_ID] == rxpacket[PKT_ID]:
            error = rxpacket[PKT_ERROR]

        return rxpacket, result, error

    def ping(self, scs_id):
        model_number = 0
        error = 0

        txpacket = [0] * 6

        if scs_id >= BROADCAST_ID:
            return model_number, COMM_NOT_AVAILABLE, error

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 2
        txpacket[PKT_INSTRUCTION] = INST_PING

        rxpacket, result, error = self.txRxPacket(txpacket)

        if result == COMM_SUCCESS:
            data_read, result, error = self.readTxRx(
                scs_id, 3, 2
            )  # Address 3 : Model Number
            if result == COMM_SUCCESS:
                model_number = SCS_MAKEWORD(data_read[0], data_read[1])

        return model_number, result, error

    def readTxRx(self, scs_id, address, length):
        txpacket = [0] * 8
        data = []

        if scs_id >= BROADCAST_ID:
            return data, COMM_NOT_AVAILABLE, 0

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 4
        txpacket[PKT_INSTRUCTION] = INST_READ
        txpacket[PKT_PARAMETER0 + 0] = address
        txpacket[PKT_PARAMETER0 + 1] = length

        rxpacket, result, error = self.txRxPacket(txpacket)
        if result == COMM_SUCCESS:
            error = rxpacket[PKT_ERROR]

            data.extend(rxpacket[PKT_PARAMETER0 : PKT_PARAMETER0 + length])

        return data, result, error

    def read(self, scs_id, address):
        data, result, error = self.readTxRx(scs_id, address[0], address[1])

        if result != COMM_SUCCESS:
            return 0, result, error

        if address[1] == 1:
            return data[0], result, error

        return SCS_MAKEWORD(data[0], data[1]), result, error

    def writeTxOnly(self, scs_id, address, length, data):
        txpacket = [0] * (length + 7)

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_WRITE
        txpacket[PKT_PARAMETER0] = address

        txpacket[PKT_PARAMETER0 + 1 : PKT_PARAMETER0 + 1 + length] = data[0:length]

        result = self.txPacket(txpacket)
        self.port.is_using = False

        return result

    def writeTxRx(self, scs_id, address, length, data):
        txpacket = [0] * (length + 7)

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_WRITE
        txpacket[PKT_PARAMETER0] = address

        txpacket[PKT_PARAMETER0 + 1 : PKT_PARAMETER0 + 1 + length] = data[0:length]
        rxpacket, result, error = self.txRxPacket(txpacket)

        return result, error

    def write(self, scs_id, address, data):
        data_write = [data] if address[1] == 1 else [SCS_LOBYTE(data), SCS_HIBYTE(data)]
        return self.writeTxRx(scs_id, address[0], address[1], data_write)


# Macro for Control Table Value
def SCS_SETEND(e):
    global SCS_END
    SCS_END = e


def SCS_TOHOST(a, b):
    if a & (1 << b):
        return -(a & ~(1 << b))
    else:
        return a


def SCS_TOSCS(a, b):
    if a < 0:
        return -a | (1 << b)
    else:
        return a


def SCS_MAKEWORD(a, b):
    global SCS_END
    if SCS_END == 0:
        return (a & 0xFF) | ((b & 0xFF) << 8)
    else:
        return (b & 0xFF) | ((a & 0xFF) << 8)


def SCS_MAKEDWORD(a, b):
    return (a & 0xFFFF) | (b & 0xFFFF) << 16


def SCS_LOWORD(l):
    return l & 0xFFFF


def SCS_HIWORD(l):
    return (l >> 16) & 0xFFFF


def SCS_LOBYTE(w):
    global SCS_END
    if SCS_END == 0:
        return w & 0xFF
    else:
        return (w >> 8) & 0xFF


def SCS_HIBYTE(w):
    global SCS_END
    if SCS_END == 0:
        return (w >> 8) & 0xFF
    else:
        return w & 0xFF


SCS_END = 0


def SCS_SETEND(e):
    global SCS_END
    SCS_END = e


class PortHandler(object):
    def __init__(self, port_name):
        self.is_open = False
        self.baudrate = 1000000
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0

        self.is_using = False
        self.port_name = port_name
        self.ser = None

    def openPort(self):
        return self.setBaudRate(self.baudrate)

    def closePort(self):
        self.ser.close()
        self.is_open = False

    def clearPort(self):
        self.ser.flush()

    def setBaudRate(self, baudrate):
        baud = self.getCFlagBaud(baudrate)
        self.baudrate = baudrate
        return self.setupPort(baud)

    def getBaudRate(self):
        return self.baudrate

    def readPort(self, length):
        return self.ser.read(length)

    def writePort(self, packet):
        return self.ser.write(packet)

    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * packet_length) + (16 * 2.0) + 2.0

    def setPacketTimeoutMillis(self, msec):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = msec

    def isPacketTimeout(self):
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True

        return False

    def getCurrentTime(self):
        return round(time.time() * 1000000000) / 1000000.0

    def getTimeSinceStart(self):
        time_since = self.getCurrentTime() - self.packet_start_time
        if time_since < 0.0:
            self.packet_start_time = self.getCurrentTime()

        return time_since

    def setupPort(self, cflag_baud):
        if self.is_open:
            self.closePort()

        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            timeout=0,
        )

        self.is_open = True

        self.ser.reset_input_buffer()

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

        return True

    def getCFlagBaud(self, baudrate):
        if baudrate in [
            4800,
            9600,
            14400,
            19200,
            38400,
            57600,
            115200,
            128000,
            250000,
            500000,
            1000000,
        ]:
            return baudrate
        else:
            raise Exception(f"Invalid baud rate {baudrate}")

