from math import *
from plotter import *
 
def carre(N, Ax, Ay, Bx, By):
    if (N==0):
        goto(Ax, Ay)
        pendown()
        move(Bx-Ax, 0)
        move(0, By-Ay)
        move(Ax-Bx, 0)  
        move(0, Ay-By)
        penup()
        return
    goto(Ax, Ay)
    pendown()
    move(Bx-Ax, 0)
    move(0, By-Ay)
    move(Ax-Bx, 0)  
    move(0, Ay-By)
    penup()
    N -= 1
    carre(N, Ax,        Ay,        (Ax+Bx)/2, (Ay+By)/2)
    carre(N, (Ax+Bx)/2, (Ay+By)/2, Bx, By)

def run(plotting):
    plotter_start()
    L=80000
    Ax=0 
    Ay=0
    Bx=L
    By=L

    carre(6, Ax, Ay, Bx, By)
        
    gohome()
    plotter_stop()

if __name__ == "__main__":  
    run(0)
    run(1)

