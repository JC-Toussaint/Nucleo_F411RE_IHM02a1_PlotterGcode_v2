import numpy as np
import serial
import serial.tools.list_ports as port_list
from skimage import io
import matplotlib.pyplot as plt
        
ports = list(port_list.comports())
STM32_port=[]
for p in ports:
    print(p)
    port_str=str(p)
    if 'STM32' in port_str:
        STM32_port=port_str.split()
    
#print(STM32_port[0])    
#s = serial.Serial(STM32_port[0], baudrate=115200)
s = serial.Serial('COM39', baudrate=115200)

print('*** RESET STM32 BOARD ***')

for line in s:
    print(line)

cmd='goto 10000, 10000'
s.write(cmd.encode('ascii'))
s.write(b'\r\n')
s.flush()

for line in s:
    print(line)
        

cmd='goto 1000, 1000'
s.write(cmd.encode('ascii'))
s.write(b'\r\n')
s.flush()

for line in s:
    print(line)

s.close()




