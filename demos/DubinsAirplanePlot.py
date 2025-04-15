#!/usr/bin/env python3

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2023, Metron, Inc.
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
#   * Neither the name of the Metron, Inc. nor the names of its
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

import shutil
from pathlib import Path
from tempfile import TemporaryDirectory
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from skspatial.objects import Sphere
from subprocess import run

cmap = plt.cm.tab10

# Change the projection type of an existing Matplotlib axis object
def updateProjection(ax, axi, projection='3d', fig=None):
    if fig is None:
        fig = plt.gcf()
    rows, cols, start, stop = axi.get_subplotspec().get_geometry()
    ax.flat[start].remove()
    ax.flat[start] = fig.add_subplot(rows, cols, start+1, projection=projection)

# read the contents of a file containing a path, each line representing one
# waypoint. Each waypoint consists of 5 numbers, separated by whitespace.
def readPath(path_file):
    return np.array([[float(x) for x in line.split()] \
                     for line in open(path_file).readlines()
                     if len(line)>1])

# Run demo_DubinsAirplane, print the output on stdout and return the path that was computed
# as an nx5 numpy array, where n is the number of waypoints and the columns
# represent X, Y, Z, pitch, and yaw.
def getPath(exec_path, **kwargs):
    with TemporaryDirectory() as tmpdir:
        path_file = Path(tmpdir) / "path.dat"
        demo_args = ' '.join([f'--{key} {value}' for key, value in kwargs.items()])
        result = run(f'{exec_path} {demo_args} --savepath {path_file}', shell=True, capture_output=True)
        print(result.stdout.decode("utf-8"))
        return readPath(path_file)

# Create 3 plots of a path:
# 1. A 3D plot of the path as well as the spherical obstacles that the path avoids
# 2. A plot of the X-, Y-, and Z-coordinate as a function of path waypoint index
# 3. A plot of pitch and yaw as a function of path waypoint index
def plotPath(path, radius, spheres=True):
    fig, axs = plt.subplots(3, 1, gridspec_kw={'height_ratios': [6, 1, 1]})
    updateProjection(axs, axs[0])
    axs[0].plot(path[:,0], path[:,1], path[:,2], color='k')
    diameter = 2. * radius
    n = int(10 // diameter)
    if spheres:
        for xi in range(-n, n):
            for yi in range(-n, n):
                for zi in range(-n, n):
                    sphere = Sphere([xi*diameter + radius, yi*diameter + radius, zi*diameter + radius], .75*radius)
                    sphere.plot_3d(axs[0], alpha=.2)
    axs[0].set_xlabel('X')
    axs[0].set_ylabel('Y')
    axs[0].set_zlabel('Z')
    axs[0].set_aspect('equal')
    px, = axs[1].plot(path[:,0], color=cmap(0), label='X')
    py, = axs[1].plot(path[:,1], color=cmap(1), label='Y')
    pz, = axs[1].plot(path[:,2], color=cmap(2), label='Z')
    axs[1].legend(handles=[px, py, pz])
    if path.shape[1]>4:
        ppitch, = axs[2].plot(path[:,3], color=cmap(0), label='pitch')
        pyaw, = axs[2].plot(path[:,4], color=cmap(1), label='yaw')
        axs[2].legend(handles=[ppitch, pyaw])
    else:
        axs[2].plot(path[:,3], color=cmap(0), label='yaw')
        axs[2].set_ylabel('yaw')
    plt.grid()
    plt.show()
    return axs

# return the full path to the demo_DubinsAirplane executable or exit
def findExecutable(exec_name='demo_DubinsAirplane'):
    # check if demo_DubinsAirplane is in the $PATH already
    executable = shutil.which(exec_name)
    # If not check in the current directory, parent directory, and "grandparent" directory (recursively)
    # (This should discover demo_Veno for in-source builds.)
    if executable == None:
        for match in Path(__file__).parent.glob(f'**/{exec_name}'):
            executable = str(match.resolve())
            break
        else:
            for match in Path(__file__).parent.parent.glob(f'**/{exec_name}'):
                executable = str(match.resolve())
                break
            else:
                for match in Path(__file__).parent.parent.parent.glob(f'**/{exec_name}'):
                    executable = str(match.resolve())
                    break
                else:
                    print(f"Could not find the {exec_name} executable; please update the path in this script")
                    exit(1)
    return executable

if __name__ == "__main__":
    # hard code path to demo_DubinsAirplane executable here if findExecutable() fails to find it
    exec_path = findExecutable()
    radius = 2
    # change command line arguments for demo_DubinsAirplane as needed here
    path = getPath(exec_path, 
                   plan='',
                   radius=radius,
                   planner="aps",
                   time=10,
                   start="0 0 -8 0 0",
                   goal="2 2 8 0 -3")
    # or call readPath() on a precomputed path
    #path = readPath('/my/path.dat')

    plotPath(path, radius)
