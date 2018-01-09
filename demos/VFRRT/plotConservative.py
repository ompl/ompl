#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Caleb Voss and Wilson Beebe
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Authors: Caleb Voss, Wilson Beebe


from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def potential(x, y):
    return 1 + np.sin(x) * np.sin(y)

def potentialSurface():
    X = np.arange(-8, 8, 0.25)
    Y = np.arange(-8, 8, 0.25)
    X, Y = np.meshgrid(X, Y)
    Z = potential(X, Y)
    return X, Y, Z

fig = plt.figure()
ax = fig.gca(projection='3d', aspect='equal')
X, Y, Z = potentialSurface()
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0)

x = np.loadtxt("vfrrt-conservative.path")
ax.plot(x[:, 0], x[:, 1], potential(x[:, 0], x[:, 1]), color='b')

x = np.loadtxt("trrt-conservative.path")
ax.plot(x[:, 0], x[:, 1], potential(x[:, 0], x[:, 1]), color='r')

x = np.loadtxt("rrtstar-conservative.path")
ax.plot(x[:, 0], x[:, 1], potential(x[:, 0], x[:, 1]), color='g')

plt.show()
