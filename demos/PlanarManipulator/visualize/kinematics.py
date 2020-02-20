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
import copy
from math import sin, cos, pi, sqrt
import yaml
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def readJointPath(pathFile):
    lines = [line.rstrip() for line in open(pathFile) if len(line.rstrip()) > 0]
    if len(lines) == 0:
        return None
    return [[float(x) for x in line.split(' ')] for line in lines]


def readChainInfo(systemFile):
    handle = open(systemFile, 'r')
    # use safe_load instead load
    dataMap = yaml.safe_load(handle)
    handle.close()

    # Parsing origin string into list
    origin = []
    originData = dataMap['origin'][1:-1].split(' ')
    for d in originData:
        origin.append(float(d))

    dataMap['origin'] = origin
    dataMap['num_links'] = int(dataMap['num_links'])
    dataMap['link_length'] = float(dataMap['link_length'])
    return dataMap

# Return the pose of all end-effectors
def FK(chain_info, joints):
    poses = []

    current_point = copy.copy(chain_info['origin'])
    current_angle = 0.0

    # origin, for convenience
    poses.append((copy.copy(current_point), copy.copy(current_angle)))

    for j in joints:
        current_angle += j
        while current_angle > pi:
            current_angle -= 2.0*pi
        while current_angle < -pi:
            current_angle += 2.0*pi

        current_point[0] += (chain_info['link_length'] * cos(current_angle))
        current_point[1] += (chain_info['link_length'] * sin(current_angle))
        poses.append((copy.copy(current_point), copy.copy(current_angle)))
    return poses

def plotLink(ax, pose1, pose2, length, color='black'):
    x1 = pose1[0][0]
    y1 = pose1[0][1]

    x2 = pose2[0][0]
    y2 = pose2[0][1]

    ax.plot([x1, x2], [y1, y2], color=color)

    # Sanity check
    dx = x2-x1
    dy = y2-y1
    norm = sqrt(dx*dx + dy*dy)
    if abs(length - norm) > 1e-2:
        print('KINEMATIC CONSTRAINTS VIOLATED.  ERROR = %f' % (abs(length - norm)))

def plotJoint(ax, pose, color='black', s=5.0):
    ax.scatter(pose[0][0], pose[0][1], color=color, s=s)

def plotChain(ax, chain_info, joints, color='black'):
    poses = FK(chain_info, joints)

    if len(poses) == 0:
        return

    for i in range(len(poses)-1):
        plotLink(ax, poses[i], poses[i+1], chain_info['link_length'], color=color)
    for i in range(len(poses)):
        plotJoint(ax, poses[i], color='black')

def plotChainStatic(chain_info, path):
    # Colors
    r = 0.0
    g = 0.0
    b = 1.0
    color_diff = 1.0 / (len(path)-1)

    axes = plt.axes()

    for p in path:
        plotChain(axes, chain_info, p, (r, g, b))
        r += color_diff
        if r > 1.0:
            r = 1.0
        b -= color_diff
        if b < 0.0:
            b = 0.0

    plt.show()

def plotChainFrame(frame_num, ax, chain_info, path):
    ax.clear()
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    plotChain(ax, chain_info, path[frame_num])
    plotChain(ax, chain_info, path[0], color='blue')
    plotChain(ax, chain_info, path[-1], color='red')
    return ax

def plotChainAnimation(chain_info, path):
    fig = plt.figure()
    axes = plt.axes()
    axes.set_xlim(-1, 1)
    axes.set_ylim(-1, 1)
    framerate = 30

    anim = animation.FuncAnimation(fig, plotChainFrame, fargs=[axes, chain_info, path], frames=len(path),
                                   interval=(1/framerate)*1000, blit=False)
    anim.save('animation.mp4', fps=framerate, writer='mencoder')

def plotChainDebug(chain_info, path):
    ax = plt.axes()

    plotChain(ax, chain_info, path[0], color='blue')
    plotChain(ax, chain_info, path[-1], color='red')
    plotChain(ax, chain_info, path[len(path)/2], color='purple')

    # ground truth
    pstart = FK(chain_info, path[0])[1:]
    pend = FK(chain_info, path[-1])[1:]
    for i in range(len(pstart)):
        xmid = (pstart[i][0][0] + pend[i][0][0])/2.0
        ymid = (pstart[i][0][1] + pend[i][0][1])/2.0

        ax.scatter(xmid, ymid, color='green', s=10.0)

    plt.show()

def main(systemFile, pathFile):
    path = readJointPath(pathFile)
    chain_info = readChainInfo(systemFile)
    plotChainAnimation(chain_info, path)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Specify a yaml file for chain definition and a path to plot')
    else:
        main(sys.argv[1], sys.argv[2])
