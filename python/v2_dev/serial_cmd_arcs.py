from math import *
from plotter import *
from random import *

def run(plotting):
    plotter_start(plotting)
    L=60000
    Ax=0 
    Ay=0
    Bx=L
    By=L

    # goto(L/4, L/4)
    # arcG2(L/4, L/4, L/4, L/4)

    for n in range(10):
        R=L/2*random()
        r=random()
        Xs=R*cos(2*pi*r)+L/2
        Ys=R*sin(2*pi*r)+L/2

        r=random()
        Xe=R*cos(2*pi*r)+L/2
        Ye=R*sin(2*pi*r)+L/2

        DX=L/2-Xs
        DY=L/2-Ys

        goto(Xs, Ys)
        arcG2(Xe, Ye, DX, DY)
        goto(Xs, Ys)
        arcG3(Xe, Ye, DX, DY)
        
    gohome()
    plotter_stop()

if __name__ == "__main__":  
    run(0)
    run(1)

