#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2017, Rice University
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

# Author: Mark Moll

# This is a helper program to visualize the output of the KinematicChainBenchmark.cpp.
# Sample usage (when running from the OMPL build directory):
#
#       ./bin/demo_KinematicChainBenchmark 10 1
#       /path/to/KinematicChainPathPlot.py 10 # produces kinematic_10.pdf
#       /path/to/KinematicChainPathPlot.py 10 movie # produces kinematic_10.mp4

from sys import argv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches  import PathPatch
from matplotlib.path import Path
from matplotlib import animation, cm, colors

fig = plt.figure()
plt.axis('equal')
ax = plt.axes(xlim=(-1, 1), ylim=(-1, 1))
ln, = plt.plot([], [], animated=True)

def getPositions(pose, linkLength, color='red'):
    angles = np.cumsum(pose)
    return PathPatch(Path(
        np.insert(linkLength * np.cumsum([np.cos(angles), np.sin(angles)], \
            axis=1), 0, 0., axis=1).T), facecolor='none', edgecolor=color)

def drawEnvironment(env):
    ax.clear()
    if env:
        plt.gca().add_patch(env)

def drawPose(index, env, poses, linkLength):
    drawEnvironment(env)
    if index == -1:
        cMap = cm.ScalarMappable(
            norm=colors.Normalize(vmin=0, vmax=poses.shape[0] - 1),
            cmap=plt.get_cmap('viridis'))
        for (i, pose) in enumerate(poses):
            plt.gca().add_patch(getPositions(pose, linkLength, \
                cMap.to_rgba(i)))
    else:
        plt.gca().add_patch(getPositions(poses[index, :], linkLength))
        return ln,

def makeMovie(fname, env, poses, linkLength):
    ani = animation.FuncAnimation(plt.gcf(), drawPose, \
        fargs=(env, poses, linkLength), frames=poses.shape[0], \
        blit=True)
    ani.save(fname, bitrate=300, fps=20)

if __name__ == '__main__':
    if len(argv) < 2:
        print('Usage: %s num_links [movie]' % argv[0])
        exit(1)

    dims = int(argv[1])

    env = None
    try:
        coords = np.loadtxt('environment_%d.dat' % dims)
        env = PathPatch(Path(coords), facecolor='none', edgecolor='black')
    except ValueError:
        pass

    poses = np.loadtxt('kinematic_path_%d.dat' % dims)
    linkLength = 1. / dims

    if len(argv) > 2:
        makeMovie('kinematic_%d.mp4' % dims, env, poses, linkLength)
    else:
        drawPose(-1, env, poses, linkLength)
        fig = plt.gcf()
        fig.savefig('kinematic_%d.pdf' % dims)
