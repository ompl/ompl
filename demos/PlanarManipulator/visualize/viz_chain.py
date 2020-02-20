#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, Rice University
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

# Author: Ryan Luna

import sys
from math import sin, cos
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def readPathFile(pathFile):

    # Read in file line by line
    lines = [line.rstrip() for line in open(pathFile) if len(line.rstrip()) > 0]

    # first line is meta data
    metadata = lines[0]
    metadata = metadata.split(' ')
    if len(metadata) != 4:
        raise RuntimeError('Malformed path file. '
                           'Expected first line with # links, link length, originX, originY')

    numLinks = int(metadata[0])
    linkLength = float(metadata[1])
    origin = (float(metadata[2]), float(metadata[3]))

    path = []

    for l in lines[1:]:
        entries = l.split(' ')
        if len(entries) != numLinks:
            raise RuntimeError('Malformed path file. Path entries must have length = # links')
        config = [float(e) for e in entries]
        path.append(config)

    return numLinks, linkLength, origin, path

def readEnvFile(envFile):
    # Read in file line by line
    lines = [line.rstrip() for line in open(envFile) if len(line.rstrip()) > 0]

    segments = []
    # each line is a segment
    for l in lines:
        entries = [float(e) for e in l.split(' ')]
        if len(entries) != 4:
            raise RuntimeError('Malformed env file. '
                               'Entries must have length 4 (two coordinates). '
                               'Line has length %d' %(len(entries)))
        segments.append([[entries[0], entries[2]], [entries[1], entries[3]]])

    return segments

def plotEnvironment(ax, segments):
    for s in segments:
        ax.plot(s[0], s[1], '-', color='black')

def plotChain(ax, numLinks, linkLength, origin, angles, color='blue'):
    x = origin[0]
    y = origin[1]
    angle = 0.0

    if len(angles) != numLinks:
        raise RuntimeError('Logic error. '
                           'numLinks(%d) != len(angles) (%d)' %(numLinks, len(angles)))

    for theta in angles:
        angle += theta
        xN = x + (cos(angle) * linkLength)
        yN = y + (sin(angle) * linkLength)
        ax.plot([x, xN], [y, yN], '-')
        x = xN
        y = yN

def plotChainFrame(frame_num, ax, numLinks, linkLength, origin, path, env):
    ax.clear()
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)

    if env:
        plotEnvironment(ax, env)

    if frame_num >= len(path):
        plotChain(ax, numLinks, linkLength, origin, path[-1])
    else:
        plotChain(ax, numLinks, linkLength, origin, path[frame_num])

def animateChain(numLinks, linkLength, origin, path, env, filename='animation.mp4'):
    fig = plt.figure()
    axes = plt.axes()
    framerate = 30
    anim = animation.FuncAnimation(fig, plotChainFrame,
                                   fargs=[axes, numLinks, linkLength, origin, path, env],
                                   frames=len(path) + framerate, interval=(1/framerate)*1000,
                                   blit=False)
    anim.save(filename, fps=framerate, writer='mencoder')

def main(pathFile, envFile, outName):
    numLinks, linkLength, origin, path = readPathFile(pathFile)

    if envFile:
        env = readEnvFile(envFile)
    else:
        env = None

    if len(path) == 1:
        ax = plt.gca()
        plotEnvironment(ax, env)
        plotChain(ax, numLinks, linkLength, origin, path[0], env)
        plt.show()
    else:
        animateChain(numLinks, linkLength, origin, path, env, outName)


if __name__ == '__main__':
    pathFile = 'solution.txt'
    envFile = None
    outName = 'animation.mp4'

    if len(sys.argv) >= 2:
        pathFile = sys.argv[1]
    if len(sys.argv) >= 3:
        outName = sys.argv[2]

    main(pathFile, envFile, outName)
