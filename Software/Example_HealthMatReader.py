# Standard libraries
import sys
import time

# Third-party libraries
import numpy as np
import matplotlib as plt
import serial


# This is how serial devices appear on Mac.
# For other operating systems, you may need to change this.
DEFAULT_DEVICE = '/dev/cu.usbmodem104742601'
ROWS = 48  # Rows of the sensor
COLS = 48  # Columns of the sensor


class Mat:
    def __init__(self, port):
        self.ser = serial.Serial(
            port,
            baudrate=115200,
            timeout=0.1)

    def request_pressure_data(self):
        data = 'R'
        self.ser.write(data.encode())

    def read_pressure_data(self):
        matrix = np.zeros((ROWS, COLS), dtype=int)
        _ = self.ser.read().decode('utf-8')
        high_byte = self.ser.read()
        low_byte = self.ser.read()
        high = int.from_bytes(high_byte, 'big')
        low = int.from_bytes(low_byte, 'big')
        num_points = ((high << 8) | low)
        _ = self.ser.read().decode('utf-8')
        _ = self.ser.read().decode('utf-8')
        x = y = n = 0
        while n < num_points:
            x = self.ser.read()
            y = self.ser.read()
            x = int.from_bytes(x, 'big')
            y = int.from_bytes(y, 'big')
            high_byte = self.ser.read()
            low_byte = self.ser.read()
            high = int.from_bytes(high_byte, 'big')
            low = int.from_bytes(low_byte, 'big')
            val = ((high << 8) | low)
            matrix[y][x] = val
            n += 1
        return matrix

    def get_pressure_data(self):
        xbyte = ''
        if self.ser.in_waiting > 0:
            xbyte = self.ser.read().decode('utf-8')
            if(xbyte == 'N'):
                return self.read_pressure_map()
            else:
                self.ser.flush()

    def get_pressure_map(self):
        self.request_pressure_data()
        return self.get_pressure_data()

    def print_matrix(self, data):
        for i in range(COLS):
            tmp = ''
            for j in range(ROWS):
                tmp = tmp + hex(int(data[i][j]))[-1]
            print(tmp)
        print('\n')


def generatePlot(Z):
    fig, ax = plt.subplots()

    ax.contourf(Z, levels=np.linspace(Z.min(), Z.max(), 7))

    plt.show()


def main():
    mat = Mat(DEFAULT_DEVICE)
    while True:
        data = mat.get_pressure_map()
        mat.print_matrix(data)


if __name__ == '__main__':
    main()
