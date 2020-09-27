from math import *
from plotter import *
 
if __name__ == "__main__":  
    plotter_start()

    #grid(10000, 10000, 50000, 50000, 10, 10)

    Xc=50000
    Yc=50000
    R =40000
    circle(Xc, Yc, 3*R/4)
    #circle(Xc, Yc, R/2)

    N=32
    for n in range(N):
        theta=2*n*pi/N
        goto(Xc, Yc)
        Xdest=Xc+R*cos(theta)
        Ydest=Yc+R*sin(theta)
        drawto(Xdest, Ydest)
    #    circle(Xdest, Ydest, 2*pi*R/N)
        
    gohome()
    plotter_stop()

