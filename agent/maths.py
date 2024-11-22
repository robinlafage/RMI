from math import cos, sin, pi

def theta(theta, rot):
    res = (theta+rot)%(2*pi)
    if res > pi and res <= 2*pi :
        res = -pi + (theta+rot)%(pi)
    print(res)
    return res

# In our env, D is always 1.
def rot(outl, outr, D):
    return (outr-outl)/D

def lin(outl, outr):
    return (outl+outr)/2

def x(x, lin, theta):
    return x + lin*cos(theta)

#At the start, y is 0 and lin is 0
def y(y, lin, theta):
    return y + lin*sin(theta)

def out(IN, OUT):
    minmax = min(max(IN, -0.15), 0.15)
    return ((minmax+OUT)/2)