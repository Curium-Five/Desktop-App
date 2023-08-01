import time

# HLDLC library: https://python-hdlc-controller.readthedocs.io/en/latest/usage.html
# ECSS, possibly use: https://gitlab.com/acubesat/obc/ecss-services

import logging
import asyncio
import serial
import serial_asyncio
import struct

logging.basicConfig(level=logging.DEBUG)

class HLDLC:
    FLAG_SEQUENCE = 0x7E
    ESCAPE_CHARACTER = 0x7D
    BIT_STUFFING_XOR = 0x20

    logging.debug(f"FLAG_SEQUENCE: {FLAG_SEQUENCE}")
    logging.debug(f"ESCAPE_CHARACTER: {ESCAPE_CHARACTER}")
    logging.debug(f"BIT_STUFFING_XOR: {BIT_STUFFING_XOR}")

    def __init__(self):
        self.reader = None
        self.writer = None

    async def open(self):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url='/dev/tty.usbserial-0001',
                       baudrate=9600,
                       bytesize=serial.EIGHTBITS,
                       parity=serial.PARITY_NONE,
                       stopbits=serial.STOPBITS_ONE)

    async def close(self):
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()

    async def send_frame(self, address, control, data):
        frame = self._build_frame(address, control, data)
        logging.debug(f"sending frame: {frame}")
        self.writer.write(frame)
        await self.writer.drain()

    def _build_frame(self, address, control, data):
        start_flag = struct.pack('B', self.FLAG_SEQUENCE)
        end_flag = struct.pack('B', self.FLAG_SEQUENCE)

        address = struct.pack('B', address)
        control = struct.pack('B', control)

        data = self._bit_stuffing(data)
        frame_check_sequence = self._calculate_fcs(address + control + data)

        frame = start_flag + address + control + data + frame_check_sequence + end_flag

        return frame

    def _bit_stuffing(self, data):
        logging.debug(f"Data: {data}")
        stuffed_data = b''
        for byte in data:
            if byte == self.FLAG_SEQUENCE or byte == self.ESCAPE_CHARACTER:
                stuffed_data += struct.pack('B', self.ESCAPE_CHARACTER)
                stuffed_data += struct.pack('B', byte ^ self.BIT_STUFFING_XOR)
            else:
                stuffed_data += struct.pack('B', byte)


        logging.debug(f"Bit stuffed: {stuffed_data}")

        return stuffed_data

    async def receive_frame(self):
        logging.debug("receiving frame")
        frame = await self._read_frame()
        logging.debug("received frame")
        address, control, data, received_fcs = self._decode_frame(frame)
        calculated_fcs = self._calculate_fcs(address + control + data)

        if received_fcs != calculated_fcs:
            raise ValueError('Frame Check Sequence does not match')

        return address, control, data

    async def _read_frame(self):
        buffer = b''
        start_flag_detected = False
        while True:
            byte = await self.reader.read(1)
            logging.debug(f"read byte: {byte}")
            if byte == struct.pack('B', self.FLAG_SEQUENCE):
                # End of frame detected
                if len(buffer) > 0:
                    if not start_flag_detected:
                        logging.debug(f"Error. : {buffer}")
                        buffer = b''
                        continue
                    logging.debug(f"End detected. Buffer: {buffer}")
                    return buffer
                else:
                    start_flag_detected = True
                logging.debug("Start flag detected.")
            else:
                buffer += byte

            logging.debug(f"Buffer: {buffer}")

    def _decode_frame(self, frame):
        destuffed_frame = self._bit_destuffing(frame)
        address, control, data_with_fcs = (destuffed_frame[0:1], destuffed_frame[1:2], destuffed_frame[2:])
        data, received_fcs = data_with_fcs[:-2], data_with_fcs[-2:]
        logging.debug(f"decoded frame:\n addr: {address}, control: {control}, data: {data}")
        return address, control, data, received_fcs

    def _bit_destuffing(self, data):
        destuffed_data = b''
        escape_character = False
        for byte in data:
            if escape_character:
                destuffed_data += struct.pack('B', byte ^ self.BIT_STUFFING_XOR)
                escape_character = False
            elif byte == self.ESCAPE_CHARACTER:
                escape_character = True
            else:
                destuffed_data += struct.pack('B', byte)

        logging.debug(f"destuffed_data: {destuffed_data}")
        return destuffed_data

    def _calculate_fcs(self, data):
        # Implement Frame Check Sequence (FCS) here.
        # FCS is commonly a CRC-16, but this is just an example.
        return struct.pack('B'*2, 0x00, 0x00)

"""
def read_serial():
    with serial.Serial(port='/dev/tty.usbserial-0001',
                       baudrate=9600,
                       bytesize=serial.EIGHTBITS,
                       parity=serial.PARITY_NONE,
                       stopbits=serial.STOPBITS_ONE) as ser:
        while True:
            message = "Hello, World!\n"
            message = message.encode('utf-8')
            ser.write(message)
            x = ser.readline()  # read one byte+
            logging.debug(x)
            #time.sleep(2)
"""

async def main():
    hldlc = HLDLC()
    await hldlc.open()

    address = 0xA2
    control = 0x03
    data = struct.pack('B'*3, 0x78, 0x7E, 0x7D)

    await hldlc.send_frame(address, control, data)

    received_address, received_control, received_data = await hldlc.receive_frame()

    logging.debug('Received address:' + str(received_address))
    logging.debug('Received control:' + str(received_control))
    logging.debug('Received data:' + str(received_data))

    await hldlc.close()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    #read_serial()
    asyncio.run(main())


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
