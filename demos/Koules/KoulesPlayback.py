#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
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

# Author: Beck Chen, Mark Moll

from sys import argv, stdout
from os.path import basename, splitext
from math import cos, sin, atan2, pi, ceil
import matplotlib.pyplot as plt
import matplotlib.animation as animation

targetFrameRate = 30 # desired number of frames per second
speedUp = 1.
# the parameters will be read from file
sideLength = 0
shipRadius = 0
kouleRadius = 0
propagationStepSize = 0
shipAcceleration = 0
shipRotVel = 0
shipDelta = 0
shipEps = 0

fig = plt.figure(figsize=(6, 6))
ax = plt.axes(xlim=(0, 1), ylim=(0, 1))
fig.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=None, hspace=None)
handle, = ax.plot([], [])
path = None

def normalizeAngle(theta):
    if theta < -pi:
        return theta + 2. * pi
    if theta > pi:
        return theta - 2. * pi
    return theta

def plotShip(x, u):
    pos = (x[0], x[1])
    theta = x[4]
    (cs, ss) = (shipRadius*cos(theta), shipRadius * sin(theta))
    v = [u[0] - x[2], u[1] - x[3]]
    deltaTheta = normalizeAngle(atan2(v[1], v[0]) - theta)
    if v[0]*v[0] + v[1]*v[1] >= shipDelta * shipDelta:
        if abs(deltaTheta) < shipEps:
            # accelerate forward, draw thruster on the back
            ax.add_patch(plt.Circle((pos[0] - cs, pos[1] - ss), .3 * shipRadius, color="red"))
        elif deltaTheta > 0:
            # rotate counterclockwise, draw thruster on right side
            ax.add_patch(plt.Circle((pos[0] + ss, pos[1] - cs), .3 * shipRadius, color="red"))
        else:
            # rotate clockwise, draw thruster on left side
            ax.add_patch(plt.Circle((pos[0] - ss, pos[1] + cs), .3 * shipRadius, color="red"))
    # draw ship
    ax.add_patch(plt.Circle(x[:2], shipRadius, color="yellow"))
    # draw two blue "eyes"
    ax.add_patch(plt.Circle((pos[0] + .7*shipRadius*cos(theta + .75), \
        pos[1] + .7*shipRadius*sin(theta + .75)), .2 * shipRadius, color="blue"))
    ax.add_patch(plt.Circle((pos[0] + .7*shipRadius*cos(theta - .75), \
        pos[1] + .7*shipRadius*sin(theta - .75)), .2 * shipRadius, color="blue"))

def plotKoules(state):
    numKoules = int(len(state)/4)
    for i in range(numKoules):
        ax.add_patch(plt.Circle((state[4 * i], state[4 * i + 1]), kouleRadius, color="red"))

def plotSystem(index):
    ax.clear()
    ax.add_patch(plt.Rectangle((0, 0), 1, 1, color='black'))
    plotKoules(path[index][5:-3])
    plotShip(path[index][0:5], path[index][-3:])
    if index % 10 == 0:
        stdout.write('.')
        stdout.flush()
    return handle,

def makeMovie(fname):
    with open(fname, 'r') as f:
        global sideLength, shipRadius, kouleRadius, propagationStepSize, shipAcceleration, \
            shipRotVel, shipDelta, shipEps, path
        sideLength, shipRadius, kouleRadius, propagationStepSize, shipAcceleration, \
            shipRotVel, shipDelta, shipEps = [float(x) for x in next(f).split()]
        path = [[float(x) for x in line.split(' ')] for line in f]
        if not path:
            print('Error: %s contains no solution path' % fname)
            return
        step = int(ceil(speedUp / (propagationStepSize * targetFrameRate)))
        path = path[0:len(path):step]
        print('Creating a movie with %d frames...' % len(path))
        print('Printing a \'.\' for every 10th frame:')
        ani = animation.FuncAnimation(fig, plotSystem, frames=len(path), \
            interval=1000. / step, blit=True)
        (base, _) = splitext(basename(fname))
        outfname = base + '.mp4'
        ani.save(outfname, bitrate=300, fps=targetFrameRate)
        print('')

if __name__ == '__main__':
    if len(argv) == 1:
        print('Usage: KoulesPlayback.py <filename> [<filename2> ...]')
    else:
        for trajectory_file in argv[1:]:
            makeMovie(trajectory_file)
