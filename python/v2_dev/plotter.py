import serial
import serial.tools.list_ports as port_list
import matplotlib
matplotlib.use('qt5agg')
import matplotlib.pyplot as plt
import sys
import time
import gc
from math import *

# ports = list(port_list.comports())
# STM32_port=[]
# for p in ports:
#     print(p)
#     port_str=str(p)
#     if 'STM32' in port_str:
#         STM32_port=port_str.split()


# matplotlib.use("TkAgg")
# print(matplotlib.get_backend() ) 

#print(STM32_port[0])    
#s = serial.Serial(STM32_port[0], baudrate=115200)
#s = serial.Serial('COM39', baudrate=115200)
s = serial.Serial()

# plt.ion() # activate matplotlib’s interactive mode
# fig, ax = plt.subplots()

#plt.show()
#ax = plt.gca()
xpos=0
ypos=0
DOWN=0
UP=1
PLOTTING=0
PENSTATUS=0

def plotter_start(plotting):  
    global ax
    global xpos, ypos
    global PLOTTING
    PLOTTING=plotting

    if PLOTTING:
        print('trying to open serial port ...', end='')
        s.port='COM39'
        s.baudrate=115200
        s.timeout=1000
        try:
            s.open()
            time.sleep(0.1)     # wait fro 100 ms for pyserial port to actually be ready
            # s.reset_input_buffer()
            # s.reset_output_buffer()
            print('OK')
        except (OSError, serial.SerialException):
            print('ERROR')
            raise SystemExit

        print('*** RESET STM32 BOARD ***')
        for line in s:
            print(line)
            if 'CPLT' in line.decode('ascii'):
                break
    else:
        plt.ioff() # Turn the interactive mode off
        fig, ax = plt.subplots(figsize=(10, 10))
        
        ax.cla() # clear for fresh plot
        ax.set_xlim((0, 100000))
        ax.set_ylim((0, 100000))
        plt.draw() 


def plotter_stop():
    global PLOTTING
    if PLOTTING:
        s.close() 
    else:
        print('Close figure window to continue')
        plt.show()


def uart_send(cmd):
    s.reset_input_buffer()
    s.reset_output_buffer()
    s.write(cmd.encode('ascii'))
    s.write(b'\r\n')
    s.flush()
    for line in s:
        print(line)
        if 'CPLT' in line.decode('ascii'):
            break
        
def gohome():
    global xpos, ypos
    global PLOTTING
    xpos=0
    ypos=0
    if PLOTTING:
        cmd='gohome'
        uart_send(cmd)
        
def goto(x, y):
    global xpos, ypos
    global PLOTTING
    xpos=x
    ypos=y
    if PLOTTING:
        cmd='goto '+str(x)+' '+str(y)
        uart_send(cmd)
    
def circle(x, y, R):
    global ax
    global xpos, ypos
    global PLOTTING
    if PLOTTING: 
        cmd='circle '+str(x)+' '+str(y)+' '+str(R)
        uart_send(cmd)
    else:
        circle1 = plt.Circle((x, y), R, color='b', fill=False)
        ax.add_artist(circle1)
        plt.draw() 
    xpos=x+R
    ypos=y

def draw(dx, dy):
    global xpos, ypos
    global PLOTTING
    if PLOTTING:
        cmd='draw '+str(dx)+' '+str(dy)
        uart_send(cmd)
    else:
        line1, = ax.plot([xpos, xpos+dx], [ypos, ypos+dy], 'b')
        plt.draw() 
    xpos += dx
    ypos += dy


def drawto(x, y):
    global xpos, ypos
    global PLOTTING
    if PLOTTING:
        cmd='drawto '+str(x)+' '+str(y)
        uart_send(cmd)
    else:
        line1, = ax.plot([xpos, x], [ypos, y], 'b')
        plt.draw() 
    xpos=x
    ypos=y

def arcG2(Xend, Yend, DX, DY):
    if PLOTTING:
        cmd='arcG2 '+ str(Xend)+' '+str(Yend)+' '+str(DX)+' '+str(DY)
        uart_send(cmd)
    else:
        Xstart=xpos
        Ystart=ypos

        epsilon=1e-6
        N=200
        dthetaMax=2*pi/N
        Xc=Xstart+DX
        Yc=Ystart+DY
        Radius=sqrt(DX*DX+DY*DY)
        DXend=Xend-Xc
        DYend=Yend-Yc
        RadiusEnd=sqrt(DXend*DXend+DYend*DYend)
        if abs(Radius-RadiusEnd)>epsilon*(Radius+RadiusEnd):
            print('ERROR in arc function')
            raise SystemExit

        pendown()

        thetaStart=atan2(-DY, -DX)
        thetaEnd  =atan2((Yend-Yc), (Xend-Xc))
        if (thetaEnd>=thetaStart):
            thetaEnd-=2*pi

        sign=-1
        N=round(sign*(thetaEnd-thetaStart)/dthetaMax)

        dthetaMax = sign*dthetaMax
        thetap=thetaStart
        for n in range(N+1):
            theta=thetaStart+ n *dthetaMax
            dct=cos(theta)-cos(thetap)
            dst=sin(theta)-sin(thetap)
            DX = (+Radius*dct)
            DY = (+Radius*dst)
            move(DX, DY)
            thetap=theta

        goto(Xend, Yend)
        pendown()

def arcG3(Xend, Yend, DX, DY):
    if PLOTTING:
        cmd='arcG3 '+ str(Xend)+' '+str(Yend)+' '+str(DX)+' '+str(DY)
        uart_send(cmd)
    else:
        Xstart=xpos
        Ystart=ypos

        epsilon=1e-6
        N=200
        dthetaMax=2*pi/N
        Xc=Xstart+DX
        Yc=Ystart+DY
        Radius=sqrt(DX*DX+DY*DY)
        DXend=Xend-Xc
        DYend=Yend-Yc
        RadiusEnd=sqrt(DXend*DXend+DYend*DYend)
        if abs(Radius-RadiusEnd)>epsilon*(Radius+RadiusEnd):
            print('ERROR in arc function')
            raise SystemExit

        pendown()

        thetaStart=atan2(-DY, -DX)
        thetaEnd  =atan2((Yend-Yc), (Xend-Xc))
        if (thetaEnd<=thetaStart):
            thetaStart-=2*pi

        sign=+1
        N=round(sign*(thetaEnd-thetaStart)/dthetaMax)

        dthetaMax = sign*dthetaMax
        thetap=thetaStart
        for n in range(N+1):
            theta=thetaStart+ n *dthetaMax
            dct=cos(theta)-cos(thetap)
            dst=sin(theta)-sin(thetap)
            DX = (+Radius*dct)
            DY = (+Radius*dst)
            move(DX, DY)
            thetap=theta

        goto(Xend, Yend)
        pendown()

def arc(Xstart, Ystart, Xend, Yend, DX, DY):
    if PLOTTING:
        cmd='arc '+str(Xstart)+' '+str(Ystart)+' '+str(Xend)+' '+str(Yend)+' '+str(DX)+' '+str(DY)
        uart_send(cmd)
    else:
        epsilon=1e-6
        N=200
        dthetaMax=2*pi/N
        Xc=Xstart+DX
        Yc=Ystart+DY
        Radius=sqrt(DX*DX+DY*DY)
        DXend=Xend-Xc
        DYend=Yend-Yc
        RadiusEnd=sqrt(DXend*DXend+DYend*DYend)
        if abs(Radius-RadiusEnd)>epsilon*(Radius+RadiusEnd):
            print('ERROR in arc function')
            raise SystemExit

        penup()
        goto(Xstart, Ystart)
        pendown()

        thetaStart=atan2(-DY, -DX)
        thetaEnd  =atan2((Yend-Yc), (Xend-Xc))
        if (thetaEnd-thetaStart>+pi):
            thetaEnd-=2*pi
        if (thetaEnd-thetaStart<-pi): 
            thetaEnd+=2*pi

        if thetaEnd<thetaStart:
            sign=-1.0
        else:
            sign=+1.0

        N=round(sign*(thetaEnd-thetaStart)/dthetaMax)

        dthetaMax = sign*dthetaMax
        thetap=thetaStart
        for n in range(N+1):
            theta=thetaStart+ n *dthetaMax
            dct=cos(theta)-cos(thetap)
            dst=sin(theta)-sin(thetap)
            DX = (+Radius*dct)
            DY = (+Radius*dst)
            move(DX, DY)
            thetap=theta

        goto(Xend, Yend)
        pendown()


def ellipse(x, y, a, b):
    global ax
    global xpos, ypos
    global PLOTTING
    if PLOTTING:
        cmd='circle '+str(x)+' '+str(y)+' '+str(a)+' '+str(b)
        uart_send(cmd)
    else:
        ellipse1 =plt.Ellipse(xy=[x, y], width=a, height=b, angle=0,edgecolor='b')
        ax.add_artist(ellipse1)
        plt.draw() 
    xpos=x+a
    ypos=y+b

 
def move(dx, dy):
    global xpos, ypos
    global PLOTTING
    global PENSTATUS
    x = xpos+dx
    y = ypos+dy
    if PLOTTING:
        cmd='move '+str(dx)+' '+str(dy)
        uart_send(cmd)
    else :
        if PENSTATUS == DOWN:
            line1, = ax.plot([xpos, x], [ypos, y], 'b')
            plt.draw() 
    xpos =x
    ypos =y

    
def pendown():
    global PLOTTING
    if PLOTTING:
        cmd='pendown'
        uart_send(cmd)
    PENSTATUS=DOWN

def penup():
    global PLOTTING
    if PLOTTING:
        cmd='penup'
        uart_send(cmd)
    PENSTATUS=UP

def wait(t):
    global PLOTTING
    if PLOTTING:
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
        

if __name__ == "__main__":        
    plotter_start(1)
    grid(10000, 10000, 50000, 50000, 5, 5)
    circle(30000, 30000, 25000)
    circle(30000, 30000, 20000)
    circle(30000, 30000, 15000)
    circle(30000, 30000, 10000)

    plotter_stop()




