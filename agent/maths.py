from math import cos, sin

def theta(theta, rot):
    return theta+rot

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

def out(IN, out, gaussian):
    minmax = min(max(IN, -0.15), 0.15)
    return ((minmax+out)/2)*gaussian

