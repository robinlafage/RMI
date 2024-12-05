from math import cos, sin, pi, radians, degrees

# This function is not rounded
def theta(theta, rot):
    res = (theta+rot)%(2*pi)
    if res > pi and res <= 2*pi :
        res = -pi + (theta+rot)%(pi)
    return res

# def theta(theta, rot):
#     resArrondi = round((theta+rot)*(180/pi))*(pi/180)
#     res = (resArrondi)%(2*pi)
#     if res > pi and res <= 2*pi :
#         res = -pi + (resArrondi)%(pi)
#     return res

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

def degToRad(angle):
    angle = radians(angle)
    res = (angle)%(2*pi)
    if res > pi and res <= 2*pi :
        res = -pi + (angle)%(pi)
    return res

def frameAngle(angle):
    res = (angle)%(2*pi)
    if res > pi and res <= 2*pi :
        res = -pi + (angle)%(pi)
    return res