#!/usr/bin/python

# plotNonconservative.py
# COMP 450 Project 5
# 25 November 2014
# Caleb Voss (cav2) & Wilson Beebe (wsb1)

import matplotlib.pyplot as plt
from pylab import *
from matplotlib import cm
import numpy

def readFile(fileName):
    file = open(fileName, "r")
    points = map(lambda line: map(float, line.split()), file.readlines())
    file.close()
    return points

def makeVectorField(f, xmin, xmax, ymin, ymax, step):
    X,Y = meshgrid(arange(xmin,xmax,step),arange(ymin,ymax,step))
    U,V = zip(*map(lambda xx: f(*xx), zip(X,Y)))
    Q = quiver( X, Y, U, V, units='width')
    quiverkey(Q, 0, 0, 4, '', coordinates='figure', labelpos='W')

fig = plt.figure()
ax = fig.gca(aspect='equal')
x,y = zip(*readFile("vfrrt-nonconservative.path"))
makeVectorField(lambda x,y: (y/sqrt(x*x+y*y),-x/sqrt(x*x+y*y)), -6, 6, -6, 6, 0.5)
ax.plot(x,y)
plt.show()
