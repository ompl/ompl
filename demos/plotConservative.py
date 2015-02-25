#!/usr/bin/python

# plotConservative.py
# COMP 450 Project 5
# 25 November 2014
# Caleb Voss (cav2) & Wilson Beebe (wsb1)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pylab import *
from matplotlib import cm
import numpy

def readFile(fileName):
    file = open(fileName, "r")
    points = map(lambda line: map(float, line.split()), file.readlines())
    file.close()
    return points

def potential(x,y):
    return 1 + numpy.sin(x)*numpy.sin(y)

def addPotential(points):
    x = map(lambda x: x[0], points)
    y = map(lambda x: x[1], points)
    z = map(lambda p: potential(p[0],p[1]), points)
    return x,y,z

def potentialSurface():
    X = numpy.arange(-8, 8, 0.25)
    Y = numpy.arange(-8, 8, 0.25)
    X, Y = numpy.meshgrid(X, Y)
    Z = potential(X,Y)
    return X,Y,Z

fig = plt.figure()
ax = fig.gca(projection='3d', aspect='equal')
X,Y,Z = potentialSurface()
ax.plot_surface(X,Y,Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0)
x,y,z = addPotential(readFile("vfrrt-conservative.path"))
ax.plot(x,y,z,color='b')
x,y,z = addPotential(readFile("trrt-conservative.path"))
ax.plot(x,y,z,color='r')
x,y,z = addPotential(readFile("rrtstar-conservative.path"))
ax.plot(x,y,z,color='g')
plt.show()
