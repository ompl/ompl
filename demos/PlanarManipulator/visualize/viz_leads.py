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
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def readEnvFile(envFile):
    # Read in file line by line
    lines = [line.rstrip() for line in open(envFile) if len(line.rstrip()) > 0]

    segments = []
    # each line is a segment
    for l in lines:
        entries = [float(e) for e in l.split(' ')]
        if len(entries) != 4:
            raise RuntimeError('Malformed env file.  Entries must have length 4 (two coordinates).  Line has length %d' %(len(entries)))
        segments.append([[entries[0], entries[2]], [entries[1], entries[3]]])

    return segments

def plotEnvironment(ax, segments):
    for s in segments:
        ax.plot(s[0], s[1], '-', color='black')

def readLeadFile(leadFile):
    # Read in file line by line
    lines = [line.rstrip() for line in open(leadFile) if len(line.rstrip()) > 0]

    # first line is meta data
    metadata = lines[0]
    metadata = metadata.split(' ')
    if len(metadata) != 6:
        raise RuntimeError('Malformed lead file.  Expected first line with xSlices, ySlices, xLow, xHigh, yLow, yHigh')

    xSlices = int(metadata[0])
    ySlices = int(metadata[1])
    xLow = float(metadata[2])
    xHigh = float(metadata[3])
    yLow = float(metadata[4])
    yHigh = float(metadata[5])

    leads = []

    for l in lines[1:]:
        entries = l.split(' ')
        lead = [int(e) for e in entries]
        leads.append(lead)

    return xSlices, ySlices, xLow, xHigh, yLow, yHigh, leads

def readSelectFile(selectFile):
    lines = [line.rstrip() for line in open(selectFile) if len(line.rstrip()) > 0]
    selections = {}

    for l in lines:
        entries = [int(e) for e in l.split(' ')]
        for e in entries:
            if e in selections:
                selections[e] += 1
            else:
                selections[e] = 1
    return selections

def readStatesFile(statesFile):
    lines = [line.rstrip() for line in open(statesFile) if len(line.rstrip()) > 0]
    for l in lines:
        states = [int(e) for e in l.split(' ')]
    return states

def readWeightsFile(weightsFile):
    lines = [line.rstrip() for line in open(weightsFile) if len(line.rstrip()) > 0]
    for l in lines:
        weights = [float(e) for e in l.split(' ')]
    return weights

def readExteriorFile(extFile):
    lines = [line.rstrip() for line in open(extFile) if len(line.rstrip()) > 0]
    for l in lines:
        ext = [bool(int(e) == 1) for e in l.split(' ')]
    return ext

def createGrid(nX, nY, xLow, xHigh, yLow, yHigh):
    x = xLow
    y = yLow

    dx = (xHigh - xLow) / nX
    dy = (yHigh - yLow) / nY

    grid = []
    y = yLow
    yc = 0
    while yc < nY:
        x = xLow
        xc = 0
        while xc < nX:
        # lower left corner
            c = (x, y)
            w = dx
            h = dy
            cell = (c, w, h)
            grid.append(cell)

            x += dx
            xc += 1
        y += dy
        yc += 1

    return grid

def plotLeadIntensity(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, leads):
    grid = createGrid(xSlices, ySlices, xLow, xHigh, yLow, yHigh)
    xyCells = xSlices * ySlices

    intensity = [0 for g in grid]
    increment = 1.0 / len(leads)
    for lead in leads:
        for l in lead:
            cell = l % xyCells  # Collapse all orientation cells into one dimension

            intensity[cell] += increment
            # Numerical instability near 1.0
            if intensity[cell] > 1.0:
                intensity[cell] = 1.0

            #print l, ridToGridCell(l, xSlices, ySlices)

    # Plotting grid
    index = 0
    for g in grid:
        if intensity[index] > 1e-3:
            ax.add_patch(patches.Rectangle(g[0], g[1], g[2], facecolor=(1.0, 0.0, 0.0), alpha=intensity[index], fill=True))
        ax.add_patch(patches.Rectangle(g[0], g[1], g[2], facecolor=(1.0, 1.0, 1.0), fill=False, edgecolor='gray'))
        index += 1

def plotSelections(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, selections):
    grid = createGrid(xSlices, ySlices, xLow, xHigh, yLow, yHigh)

    xyCells = xSlices * ySlices

    # Collapse orientation into one dimension
    select = [0 for i in range(xyCells)]
    intensity = 0
    for k, v in selections.iteritems():
        cell = k % xyCells
        select[cell] += v

        if select[cell] > intensity:
            intensity = select[cell]
    intensity = 1.0 / intensity

    # Plotting grid
    index = 0
    for g in grid:
        if select[index] > 0:
            ax.add_patch(patches.Rectangle(g[0], g[1], g[2], facecolor=(1.0, 0.0, 0.0),
                                           alpha=intensity*select[index], fill=True))
        ax.add_patch(patches.Rectangle(g[0], g[1], g[2], facecolor=(1.0, 1.0, 1.0), fill=False, edgecolor='gray'))
        index += 1

def plotStates(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, states):

    grid = createGrid(xSlices, ySlices, xLow, xHigh, yLow, yHigh)

    xyCells = xSlices * ySlices

    # Collapse orientation into one dimension
    st = [0 for i in range(xyCells)]

    maxStates = 0
    index = 0
    for s in states:
        cell = index % xyCells
        st[cell] += s

        if st[cell] > maxStates:
            maxStates = st[cell]
        index += 1

    # Plotting grid
    index = 0
    for g in grid:
        if st[index] > 0:
            ax.add_patch(patches.Rectangle(g[0], g[1], g[2], facecolor=(1.0, 0.0, 0.0),
                                           alpha=st[index]/float(maxStates), fill=True))
        ax.add_patch(patches.Rectangle(g[0], g[1], g[2], facecolor=(1.0, 1.0, 1.0), fill=False, edgecolor='gray'))
        index += 1

def plotWeights(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, weights):
    grid = createGrid(xSlices, ySlices, xLow, xHigh, yLow, yHigh)

    xyCells = xSlices * ySlices

    # Collapse orientation into one dimension
    wts = [0 for i in range(xyCells)]
    index = 0
    maxWeight = 0.0
    for w in weights:
        cell = index % xyCells
        wts[cell] += w

        if wts[cell] > maxWeight:
            maxWeight = wts[cell]
        index += 1

    # Plotting grid
    index = 0
    for g in grid:
        if wts[index] > 0:
            ax.add_patch(patches.Rectangle(g[0], g[1], g[2], facecolor=(1.0, 0.0, 0.0),
                                           alpha=wts[index]/maxWeight, fill=True))
        ax.add_patch(patches.Rectangle(g[0], g[1], g[2], facecolor=(1.0, 1.0, 1.0), fill=False,
                                       edgecolor='gray'))
        index += 1

def plotExterior(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, ext):
    grid = createGrid(xSlices, ySlices, xLow, xHigh, yLow, yHigh)

    # Plotting grid
    index = 0
    for g in grid:
        if ext[index]:
            ax.add_patch(patches.Rectangle(g[0], g[1], g[2], facecolor=(1.0, 0.0, 0.0), alpha=1.0, fill=True))
        ax.add_patch(patches.Rectangle(g[0], g[1], g[2], fill=False, edgecolor='gray'))
        index += 1


def main(leadFile, selectFile, statesFile, weightsFile, envFile):
    xSlices, ySlices, xLow, xHigh, yLow, yHigh, leads = readLeadFile(leadFile)
    env = readEnvFile(envFile)
    selections = readSelectFile(selectFile)
    states = readStatesFile(statesFile)
    weights = readWeightsFile(weightsFile)

    ax = plt.gca()
    ax.set_xlim(xLow, xHigh)
    ax.set_ylim(yLow, yHigh)

    print('Number of times a cell appears in the lead')
    plotEnvironment(ax, env)
    plotLeadIntensity(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, leads)
    plt.show()

    print('Number of times a cell was selected for expansion')
    ax = plt.gca()
    ax.set_xlim(xLow, xHigh)
    ax.set_ylim(yLow, yHigh)

    plotEnvironment(ax, env)
    plotSelections(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, selections)
    plt.show()

    print('Number of states in each cell')
    ax = plt.gca()
    ax.set_xlim(xLow, xHigh)
    ax.set_ylim(yLow, yHigh)

    plotEnvironment(ax, env)
    plotStates(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, states)
    plt.show()

    print('Weights in each cell')
    ax = plt.gca()
    ax.set_xlim(xLow, xHigh)
    ax.set_ylim(yLow, yHigh)

    plotEnvironment(ax, env)
    plotWeights(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, weights)
    plt.show()

    # print 'Exterior cells (shaded)'
    # ax = plt.gca()
    # ax.set_xlim(xLow, xHigh)
    # ax.set_ylim(yLow, yHigh)

    # plotEnvironment(ax, env)
    # plotExterior(ax, xSlices, ySlices, xLow, xHigh, yLow, yHigh, exterior)
    # plt.show()

if __name__ == '__main__':
    envFile = '../build/bin/environment.txt'
    leadFile = '../build/bin/leads.txt'
    selectFile = '../build/bin/selection.txt'
    statesFile = '../build/bin/states.txt'
    weightsFile = '../build/bin/weights.txt'

    if len(sys.argv) >= 2:
        leadFile = sys.argv[1]
    if len(sys.argv) >= 3:
        envFile = sys.argv[2]

    main(leadFile, selectFile, statesFile, weightsFile, envFile)
