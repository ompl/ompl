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

# PDF Plotting
import matplotlib
matplotlib.use('pdf')

from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt

def readPathFile(pathFile):

    # Read in file line by line
    lines = [line.rstrip() for line in open(pathFile) if len(line.rstrip()) > 0]

    # first line is meta data
    metadata = lines[0]
    metadata = metadata.split(' ')
    if len(metadata) != 4:
        raise RuntimeError('Malformed path file.  Expected first line with # links, link length, originX, originY')

    numLinks = int(metadata[0])
    linkLength = float(metadata[1])
    origin = (float(metadata[2]), float(metadata[3]))

    path = []

    for l in lines[1:]:
        entries = l.split(' ')
        if len(entries) != numLinks:
            raise RuntimeError('Malformed path file.  Path entries must have length = # links')
        config = [float(e) for e in entries]
        path.append(config)

    return numLinks, linkLength, origin, path

def getLocs(numLinks, linkLength, origin, angles):
    if len(angles) != numLinks:
        raise RuntimeError('Logic error.  numLinks(%d) != len(angles) (%d)' % (numLinks, len(angles)))

    x = origin[0]
    y = origin[1]
    angle = 0.0

    locX = []
    locY = []

    for theta in angles:
        locX.append(x)
        locY.append(y)

        angle += theta

        x = x + (cos(angle) * linkLength)
        y = y + (sin(angle) * linkLength)

    locX.append(x)
    locY.append(y)
    return locX, locY

def main(pathFile, outFile):
    numLinks, linkLength, origin, path = readPathFile(pathFile)
    dcolor = 1.0 / (len(path) - 1)

    stride = 4

    if outFile:
        pp = PdfPages(outFile)

    ax = plt.gca()
    ax.set_xlim(-0.1, 0.8)
    ax.set_ylim(-0.65, 0.85)

    for i in range(0, len(path), stride):
        color = (1.0 - i * dcolor, 0.0, i * dcolor)
        locX, locY = getLocs(numLinks, linkLength, origin, path[i])
        ax.plot(locX, locY, color=color)

    for i in range(0, len(path), stride):
        color = (1.0 - i * dcolor, 0.0, i * dcolor)
        locX, locY = getLocs(numLinks, linkLength, origin, path[i])
        if i == 0:
            ax.scatter(locX[0], locY[0], color='black', s=30.0)
        ax.scatter(locX[1:], locY[1:], color='black', s=30.0)

    plt.show()

    if outFile:
        pp.savefig(plt.gcf(), bbox_inches='tight')
        pp.close()

if __name__ == '__main__':
    pathFile = './interpolate.txt'
    outFile = 'out.pdf'

    if len(sys.argv) > 1:
        pathFile = sys.argv[1]
    main(pathFile, outFile)
