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

# Make the fonts PDF friendly
import matplotlib
matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

import yaml
import numpy as np
import sys
from math import sin, cos

def SignedPolygonArea(coordinates):
    area = 0.

    for i in range(len(coordinates)-1):
        area += (coordinates[i][0] * coordinates[i+1][1] - coordinates[i+1][0] * coordinates[i][1])

    return 0.5*area

def PolygonCentroid(coordinates):

    coordinates.append(coordinates[0]) # Duplicate first coordinate to the end

    area = SignedPolygonArea(coordinates)
    centroidX = 0.
    centroidY = 0.
    for i in range(len(coordinates)-1):
        centroidX += (coordinates[i][0] + coordinates[i+1][0])*(coordinates[i][0] * coordinates[i+1][1] - coordinates[i+1][0] * coordinates[i][1])
        centroidY += (coordinates[i][1] + coordinates[i+1][1])*(coordinates[i][0] * coordinates[i+1][1] - coordinates[i+1][0] * coordinates[i][1])

    centroidX = 1/(6.*area) * centroidX
    centroidY = 1/(6.*area) * centroidY

    return (centroidX, centroidY)

def plotRectangle(ax, data, color, alpha=1.0, edgecolor=None, label=None, labelcolor='black'):

    w = float(data['width'])
    h = float(data['height'])

    # Matplotlib expects lower left coord
    coords = data['upperleft'].strip()
    if coords.startswith('('):
        coords = coords[1:-1]
    coords = coords.split(',')

    c = (float(coords[0]), float(coords[1])-h)

    ax.add_patch(patches.Rectangle(c, w, h, facecolor=color, alpha=alpha, fill=True, edgecolor=edgecolor))
    if label:
        centroid = (float(coords[0]) + w/2.0, float(coords[1]) - h/2.0)
        plt.text(centroid[0]+0.05, centroid[1], label, size=14, weight='heavy', color=labelcolor, ha='center', va='center')

def plotPolygon(ax, data, color, alpha=1.0, edgecolor=None, label=None, labelcolor='black'):

    points = []
    for d in data:
        for k in d.keys():
            if k == 'vertex':
                coords = d[k].strip()
                if coords.startswith('('):
                    coords = coords[1:-1]
                coords = coords.strip().split(',')
                pt = [float(coords[0]), float(coords[1])]
                points.append(pt)

            else:
                print('Expected "vertex", but got ' + k)

    if len(points) > 0:
        arr = np.array(points)
        ax.add_patch(patches.Polygon(arr, facecolor=color, alpha=alpha, fill=True, edgecolor=edgecolor))

        # Plot the label
        if label:
            centroid = PolygonCentroid(points)
            plt.text(centroid[0], centroid[1], label, size=14, weight='heavy', color=labelcolor, ha='center', va='center')

def readPathFile(pathFile):

    # Read in file line by line
    lines = [line.rstrip() for line in open(pathFile) if len(line.rstrip()) > 0]

    # first line is meta data
    metadata = lines[0]
    metadata = metadata.split(' ')
    if len(metadata) != 6:
        raise RuntimeError('Malformed path file.  Expected first line with # links, link length, originX, originY, originTheta, world file')

    numLinks = int(metadata[0])
    linkLength = float(metadata[1])
    origin = (float(metadata[2]), float(metadata[3]), float(metadata[4]))

    path = []

    for l in lines[1:]:
        entries = l.split(' ')
        if len(entries) != numLinks:
            raise RuntimeError('Malformed path file.  Path entries must have length = # links')
        config = [float(e) for e in entries]
        path.append(config)

    return numLinks, linkLength, origin, path

def plotChain(ax, numLinks, linkLength, origin, angles, color='blue'):
    x = origin[0]
    y = origin[1]
    angle = origin[2]

    if len(angles) != numLinks:
        raise RuntimeError('Logic error.  numLinks(%d) != len(angles) (%d)' % (numLinks, len(angles)))

    for theta in angles:
        angle += theta

        xN = x + (cos(angle) * linkLength)
        yN = y + (sin(angle) * linkLength)

        ax.plot([x, xN], [y, yN], '-')

        x = xN
        y = yN

# Plots a YAML world file
def plotWorld(ax, bounds, obs):
    for k, v in obs.iteritems():
        for o in v:
            if k == 'rectangle':
                plotRectangle(ax, o, '0.4', edgecolor='0.4')
            elif k == 'polygon':
                plotPolygon(ax, o, '0.4', edgecolor='0.4')

    plt.axis([bounds[0][0], bounds[0][1], bounds[1][0], bounds[1][1]])

def plotChainFrame(frame_num, ax, obs, bounds, links, length, origin, path):
    ax.clear()
    plotWorld(ax, bounds, obs)

    if frame_num >= len(path):
        plotChain(ax, links, length, origin, path[-1])
    else:
        plotChain(ax, links, length, origin, path[frame_num])

def animateChain(obs, bounds, links, length, origin, path, filename='animation.html'):
    fig = plt.figure()
    axes = plt.axes()
    framerate = 30

    anim = animation.FuncAnimation(fig, plotChainFrame, fargs=[axes, obs, bounds, links, length, origin, path],
                                   frames=len(path)+framerate, interval=(1/framerate)*1000, blit=False)
    anim.save(filename, fps=framerate, writer='mencoder')


def readWorldFile(worldFile):
    handle = open(worldFile, 'r')
    # use safe_load instead load
    dataMap = yaml.safe_load(handle)
    handle.close()

    # Read in bounds
    xlower = float(dataMap['bounds']['x']['lower'])
    xupper = float(dataMap['bounds']['x']['upper'])
    ylower = float(dataMap['bounds']['y']['lower'])
    yupper = float(dataMap['bounds']['y']['upper'])

    bounds = [[xlower, xupper], [ylower, yupper]]

    obs = {}

    if 'obstacles' in dataMap:
        for r in dataMap['obstacles']:
            for k in r.keys():
                if k not in obs:
                    obs[k] = []
                obs[k].append(r[k])
    return obs, bounds

def plot(worldFile, pathFile):

    obs, bounds = readWorldFile(worldFile)
    links, length, origin, path = readPathFile(pathFile)

    animateChain(obs, bounds, links, length, origin, path)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Please enter a yaml world file and path file to visualize')
    else:
        world = sys.argv[1]
        path = sys.argv[2]

        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        import matplotlib.animation as animation

        plot(world, path)
