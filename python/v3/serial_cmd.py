from math import *
from plotter import *
import multiprocessing
from functools import partial
import gc

def run(filename, plotting):
    plotter_start(plotting)

    play(filename, plotting)
        
    gohome()
    plotter_stop()

if __name__ == "__main__":  
 #   run('minnie_mouse_0001.gcode', 0)
    run('minnie_mouse_0001.gcode', 1)

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
