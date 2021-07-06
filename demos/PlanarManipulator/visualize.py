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
import yaml
from math import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

# Reads a YAML world specification from the worldFile.
# Returns the set of obstacles and the bounds of the world.
def readWorldFile(worldFile):
    handle = open(worldFile, "r")
    # use safe_load instead load
    dataMap = yaml.safe_load(handle)
    handle.close()

    # Read in bounds
    xlower = float(dataMap["bounds"]["x"]["lower"])
    xupper = float(dataMap["bounds"]["x"]["upper"])
    ylower = float(dataMap["bounds"]["y"]["lower"])
    yupper = float(dataMap["bounds"]["y"]["upper"])

    bounds = [[xlower, xupper], [ylower, yupper]]

    obs = {}

    if "obstacles" in dataMap:
        for r in dataMap["obstacles"]:
            for k in r.keys():
                if k not in obs:
                    obs[k] = []
                obs[k].append(r[k])
    return obs, bounds


def readPathFile(pathFile):
    # Read in file line by line
    lines = [line.rstrip() for line in open(pathFile) if len(line.rstrip()) > 0]

    # first line is meta data
    metadata = lines[0]
    metadata = metadata.split(" ")
    if len(metadata) != 5:
        raise RuntimeError(
            "Malformed path file.  Expected first line with # links, link length, originX, originY, xySlices"
        )

    numLinks = int(metadata[0])
    linkLength = float(metadata[1])
    origin = (float(metadata[2]), float(metadata[3]))
    slices = int(metadata[4])

    path = []

    for l in lines[1:]:
        entries = l.split(" ")
        if len(entries) != numLinks:
            raise RuntimeError(
                "Malformed path file.  Path entries must have length = # links"
            )
        config = [float(e) for e in entries]
        path.append(config)

    return numLinks, linkLength, origin, path, slices


def plotPolygon(axes, data, color, alpha=1.0, edgecolor=None):
    points = []
    for d in data:
        for k in d.keys():
            if k == "vertex":
                coords = d[k].strip()
                if coords.startswith("("):
                    coords = coords[1:-1]
                coords = coords.strip().split(",")
                pt = [float(coords[0]), float(coords[1])]
                points.append(pt)

            else:
                raise RuntimeError('Expected "vertex", but got ', k)

    if len(points) > 0:
        arr = np.array(points)
        axes.add_patch(
            patches.Polygon(
                arr, facecolor=color, alpha=alpha, fill=True, edgecolor=edgecolor
            )
        )


def plotObstacles(axes, obstacles, bounds):
    for k, v in obstacles.items():
        for o in v:
            if k == "polygon":
                plotPolygon(axes, o, "0.4", edgecolor="0.4")
            else:
                raise RuntimeError("Unknown geometry type: ", k)

    plt.axis([bounds[0][0], bounds[0][1], bounds[1][0], bounds[1][1]])


def plotChain(axes, angles, origin, color="blue"):
    x = origin[0]
    y = origin[1]
    angle = 0.0

    linkLength = 1.0 / len(angles)
    for theta in angles:
        angle += theta

        xN = x + (cos(angle) * linkLength)
        yN = y + (sin(angle) * linkLength)

        axes.plot([x, xN], [y, yN], "-", color=color)

        x = xN
        y = yN


def plotChainFrame(frame_num, ax, path, obstacles, bounds, origin, rate, slices):
    ax.clear()
    plotObstacles(ax, obstacles, bounds)

    # Hold for 1 second at the beginning and end
    if frame_num < rate:
        configuration = path[0]
    elif frame_num >= len(path) + rate:
        configuration = path[-1]
    else:
        configuration = path[frame_num - rate]

    plotChain(ax, configuration, origin)


def AnimatePath(obstacles, bounds, numLinks, linkLength, origin, path, slices):
    fig = plt.figure()
    axes = plt.axes()
    framerate = 30

    numFrames = len(path)
    anim = animation.FuncAnimation(
        fig,
        plotChainFrame,
        fargs=[axes, path, obstacles, bounds, origin, slices, framerate],
        frames=numFrames + (framerate * 2),
        interval=(1.0 / framerate) * 1000,
        blit=False,
    )

    filename = "animation.mp4"
    anim.save(filename, fps=framerate)


if __name__ == "__main__":
    worldFile = "./world.yaml"
    pathFile = "./manipulator_path.txt"

    obs, bounds = readWorldFile(worldFile)
    numLinks, linkLength, origin, path, slices = readPathFile(pathFile)

    print("Creating animation for planar kinematic chain...  ")
    AnimatePath(obs, bounds, numLinks, linkLength, origin, path, slices)
    print("Done")
