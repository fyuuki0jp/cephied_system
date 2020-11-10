import serial
import sys
import struct
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import os
import math as m

args = sys.argv

dataclass = args[2]
raw_count = 0
fft_count = 0
loop = int(args[3])+1
print("data set count ="+str(loop-1))

os.makedirs("./data/"+dataclass,exist_ok=True)
with serial.Serial(args[1],115200, timeout=None) as ser:
    while True:
        line = ser.readline().decode()
        if 'serial signal' in line:
            if loop != int(args[3])+1:
                print('sample start')
            else:
                print('dumy data sample')
            if loop >= 0:
                loop -= 1
                if loop == -1:
                    break
                ser.write('run'.encode())
        elif 'raw' in line:
            print('raw data import')
            path = './data/'+dataclass+"/raw_"+str(raw_count)
            print(path)
            raw_count += 1
            with open(path+".bin",mode='wb') as file:
                print('read line')
                raw_data = []
                for j in range(256):
                    row_data = []
                    for i in range(512):
                        row_data.append(struct.unpack('<H',ser.read(size=2))[0])
                    raw_data.append(row_data)
                print('read complete')
                data = np.array(raw_data,dtype=np.uint16)
                print(data.shape)
                file.write(data)
        elif 'fft' in line:
            print('fft data import')
            path = './data/'+dataclass+"/fft_"+str(fft_count)
            print(path)
            fft_count += 1
            with open(path+".bin",mode='wb') as file:
                print('read line')
                fft_data = []
                for i in range(256):
                    row_data = []
                    for i in range(256):
                        imag = struct.unpack("<h",ser.read(size=2))[0]
                        real = struct.unpack("<h",ser.read(size=2))[0]
                        row_data.append([imag,real])
                    fft_data.append(row_data)
                data = np.array(fft_data)
                print(data.shape)
                file.write(data)
        else:
            print(line,end='')

print('data set class : '+dataclass+' complete.')