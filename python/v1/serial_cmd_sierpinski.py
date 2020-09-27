from math import *
from plotter import *
 
def sierpinski(N, Ax, Ay, Bx, By, Cx, Cy):
    if (N==0):
        goto(Ax, Ay)
        MOVE=True
        if MOVE:
            pendown()
            move(Bx-Ax, By-Ay)
            move(Cx-Bx, Cy-By)
            move(Ax-Cx, Ay-Cy)
            penup()
        else:
            drawto(Bx, By)
            drawto(Cx, Cy)
            drawto(Ax, Ay)
        return
    N -= 1
    sierpinski(N, Ax,        Ay,        (Ax+Bx)/2, (Ay+By)/2, (Ax+Cx)/2, (Ay+Cy)/2)
    sierpinski(N, (Ax+Bx)/2, (Ay+By)/2, Bx,        By,        (Bx+Cx)/2, (By+Cy)/2)    
    sierpinski(N, (Ax+Cx)/2, (Ay+Cy)/2, (Bx+Cx)/2, (By+Cy)/2, Cx,        Cy)

def run(plotting):
    plotter_start(plotting)
    L=80000
    Ax=0 
    Ay=0
    Bx=L
    By=0
    Cx=(Ax+Bx)/2
    Cy=L*sqrt(3)/2

    sierpinski(6, Ax, Ay, Bx, By, Cx, Cy)
        
    gohome()
    plotter_stop()

if __name__ == "__main__":  
    run(0)
    run(1)

