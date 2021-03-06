from math import *
from plotter import *
import multiprocessing
from functools import partial
import gc

def run(plotting):
    plotter_start(plotting)

    #grid(10000, 10000, 50000, 50000, 10, 10)

    Xc=30000
    Yc=30000
    R =10000
    circle(Xc, Yc, R)
    #circle(Xc, Yc, R/2)

    N=4
    fact=sqrt((1-cos(2*pi/N))**2+sin(2*pi/N)**2)
    print(fact)
    for n in range(N):
        theta=2*n*pi/N 
        goto(Xc, Yc)
        Xdest=Xc+R*cos(theta)
        Ydest=Yc+R*sin(theta)
        drawto(Xdest, Ydest)
        circle(Xdest, Ydest, R*fact)
        
    gohome()
    plotter_stop()

if __name__ == "__main__":  
    run(0)
    run(1)

    # iterable = [0]
    # pool = multiprocessing.Pool()
    # pool.map(run, iterable)
    # pool.close()
    # pool.join()

    # iterable = [1]
    # pool = multiprocessing.Pool()
    # pool.map(run, iterable)
    # pool.close()
    # pool.join()
