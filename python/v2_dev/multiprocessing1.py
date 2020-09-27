import multiprocessing
from functools import partial

def f(a):
    print("{} ".format(a))

def main():
    iterable = [0, 1]
    pool = multiprocessing.Pool()
    pool.map(f, iterable)
    pool.close()
    pool.join()

if __name__ == "__main__":
    main()