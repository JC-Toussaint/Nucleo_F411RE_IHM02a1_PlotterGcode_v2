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

def plotter_init():
    
    print('*** RESET STM32 BOARD ***')
    for line in s:
        print(line)
        if 'CPLT' in line.decode('ascii'):
            break
    
def uart_send(cmd):
    s.write(cmd.encode('ascii'))
    s.write(b'\r\n')
    s.flush()
    for line in s:
        print(line)
        if 'CPLT' in line.decode('ascii'):
            break
        
def gohome():
    cmd='gohome'
    uart_send(cmd)
        
def goto(x, y):
    cmd='goto '+str(x)+' '+str(y)
    uart_send(cmd)
    
def circle(x, y, R):
    cmd='circle '+str(x)+' '+str(y)+' '+str(R)
    uart_send(cmd)
  
def draw(dx, dy):
    cmd='draw '+str(dx)+' '+str(dy)
    uart_send(cmd)

def drawto(x, y):
    cmd='drawto '+str(x)+' '+str(y)
    uart_send(cmd)

def ellipse(x, y, a, b):
    cmd='circle '+str(x)+' '+str(y)+' '+str(a)+' '+str(b)
    uart_send(cmd)
 
def move(dx, dy):
    cmd='move '+str(dx)+' '+str(dy)
    uart_send(cmd)
    
def pendown():
    cmd='pendown'
    uart_send(cmd)

def penup():
    cmd='penup'
    uart_send(cmd)

def wait(t):
    cmd='wait '+ str(t)
    uart_send(cmd)
    
def play(filename):
    with open(filename, 'r') as f:
         for cmd in f:
             if not cmd.strip(): # empty or whitespace
                 continue
             if cmd[0]=='%' or cmd=='#':
                 continue
             print(cmd)
             s.write(cmd.encode('ascii'))
     #        s.write(b'\r\n')
             s.flush()
             for line in s:
                 print(line)
                 if 'CPLT' in line.decode('ascii'):
                     break


def grid(x0, y0, Lx, Ly, Nx, Ny):
    goto(x0, y0)
    deltax=Lx/Nx
    deltay=Ly/Ny
    for ix in range(Nx+1):
        x=x0+ix*deltax
        goto(x, y0)
        y=y0+Ly
        drawto(x, y)
        
    for iy in range(Ny+1):
        y=y0+iy*deltay
        goto(x0, y)
        x=x0+Lx
        drawto(x, y)
        
            
plotter_init()
grid(10000, 10000, 50000, 50000, 5, 5)
circle(30000, 30000, 25000)
circle(30000, 30000, 20000)
circle(30000, 30000, 15000)
circle(30000, 30000, 10000)

s.close()




