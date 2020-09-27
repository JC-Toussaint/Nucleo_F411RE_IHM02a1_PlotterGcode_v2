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
    #circle(Xc, Yc, R)
    #circle(Xc, Yc, R/2)

    N=5
    fact=sqrt((1-cos(2*pi/N))**2+sin(2*pi/N)**2)
    print(fact)
    for n in range(N):
        theta =2*n*pi/N 
        X0=Xc+R*cos(theta)
        Y0=Yc+R*sin(theta)

        theta -= 2*pi/N       
        Xstart=Xc+R*cos(theta)
        Ystart=Yc+R*sin(theta)

        goto(Xstart, Ystart)
        theta += 4*pi/N 
        Xend=Xc+R*cos(theta)
        Yend=Yc+R*sin(theta)
        arcG2(Xend, Yend,X0-Xstart, Y0-Ystart )
        
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
