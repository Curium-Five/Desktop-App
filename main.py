import serial
import time

def read_serial():
    with serial.Serial(port='/dev/tty.usbserial-0001',
                       baudrate=9600,
                       bytesize=serial.EIGHTBITS,
                       parity=serial.PARITY_NONE,
                       stopbits=serial.STOPBITS_ONE) as ser:
        while True:
            x = ser.readline()  # read one byte+
            #print(len(x))
            print(x)
            #print(x.decode("utf-8"))




# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    read_serial()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
