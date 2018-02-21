#!/bin/env python

from graphics import Point
from graphics import Circle
from graphics import Line
from graphics import GraphWin 
from graphics import color_rgb
import argparse
import logging
import math
import random

# Circle objects are just gonna be circles from the graphics lib.

class CircleEnv(object):
    """ Python equivalent to code from resources/circled2D.h
    """

    def __init__(self):
        self.minX_ = 0.0
        self.minY_ = 0.0
        self.maxX_ = 0.0
        self.maxY_ = 0.0
        self.scale_ = 16.0
        self.circles = []
        self.paths = [] # A list of lists of lines to be drawn
        self.points = []

    def loadCircles(self, filename):
        """
        File needs to be in this format:
        
        units: meter // these two lines are for human readability
            x  y  z  // will be ignored
        1   0  0  5
        2   -10 8 1
        ...
        """

        file = open(filename)

        # Ignore first 2 lines
        file.readline()
        file.readline()

        for line in file:
            c = line.split()
            self.circles.append(Circle(
                    Point(self.scale_ * float(c[1]), self.scale_ * float(c[2])),
                self.scale_ * float(c[3])))

        file.close()
        if self.circles:
            self.minX_ = float("infinity")
            self.minY_ = float("infinity")
            self.maxX_ = -1 * float("infinity")
            self.maxY_ = -1 * float("infinity")

            for cir in self.circles:
                if cir.p1.x < self.minX_:
                    self.minX_ = cir.p1.x
                if cir.p1.y < self.minY_:
                    self.minY_ = cir.p1.y
                if cir.p2.x > self.maxX_:
                    self.maxX_ = cir.p2.x
                if cir.p2.y > self.maxY_:
                    self.maxY_ = cir.p2.y
            self.minX_ -= 50
            self.minY_ -= 50
            self.maxX_ += 50
            self.maxY_ += 50

    def loadPaths(self, filename):
        """
        File needs to be in this format:

        units: meter
        x y
        1.3 4.2
        3.2 -4.2

        7.2 4.3
        ...
        """

        paths = open(filename)
        paths.readline()
        paths.readline()

        last_point = None
        current_path = []
        for line in paths:
            p = line.split()
            if p and not last_point:
                last_point = Point(self.scale_ * float(p[0]), self.scale_ * float(p[1]))
                p = Circle(last_point, 0.05 * self.scale_)
                p.setFill('black')
                self.points.append(p)
            elif p and last_point:
                current_point = Point(self.scale_ * float(p[0]), self.scale_ * float(p[1]))
                line = Line(last_point, current_point)
                current_path.append(line)
                last_point = current_point
                p = Circle(last_point, 0.05 * self.scale_)
                p.setFill('black')
                self.points.append(p)
            else:
                # Starting a new trajectory
                self.paths.append(current_path)
                current_path = []
                last_point = None

        if current_path != []:
            self.paths.append(current_path)

        # Progressively shade the paths from red to green.
        current_red = 255
        inc = 255 / len(self.paths)
        for path in self.paths:
            for line in path:
                line.setFill(color_rgb(int(current_red), int(255 - current_red), 0))
            current_red = current_red - inc

    def makeWindow(self):
        total_x = self.maxX_ - self.minX_
        total_y = self.maxY_ - self.minY_
        win = GraphWin('circles', total_x, total_y)
        win.setCoords(self.minX_, self.minY_, self.maxX_, self.maxY_)
        return win

    def draw(self, win):
        for cir in self.circles:
            cir.draw(win)
        for path in self.paths:
            for line in path:
                line.draw(win)
        for p in self.points:
            p.draw(win)

class ArticulatedEnv(object):
    """
    Python equivalent to the code in articulated2D.h
    """

    def __init__(self):
        self.circleEnv = CircleEnv()
        self.joint_paths = []
        self.root = (0, 0)
        self.root_point = Point
        self.lengths = (0, 0)
    
    def loadCircles(self, filename):
        """
        File needs to be in this format:

        root: xpos ypos
        lengths: len_1, len_2
        1 x y z
        2 x2 y2 z2
        ...
        """
        file = open(filename)

        self.root = tuple([float(val) for val in file.readline().split()[1:]])
        self.root_point = Point(self.circleEnv.scale_ * self.root[0], 
                      self.circleEnv.scale_ * self.root[1])
        #self.root_point = Circle(point, 0.05 * self.circleEnv.scale_)
        #self.root_point.setFill('black')
        self.lengths = tuple([float(val) for val in file.readline().split()[1:]])
        file.close()

        self.circleEnv.loadCircles(filename)

    def forwardKinematics(self, angle1, angle2):
        point1 = (self.root[0] + math.cos(angle1) * self.lengths[0],
                  self.root[1] + math.sin(angle1) * self.lengths[1])
        point2 = (point1[0] + math.cos(angle1 + angle2) * self.lengths[0],
                  point1[1] + math.sin(angle1 + angle2) * self.lengths[1])
        return [point1, point2]

    def loadPath(self, filename):
        """
        File needs to be in this format

        1.3 4.2
        3.2 -4.2

        7.2 3.1
        ...
        """
        path = open(filename)

        joint_path = [] # 2-ples of the joint path from the articulated robot
        for line in path:
            q = [float(val) for val in line.split()]
            if not q:
                self.joint_paths.append(joint_path)
                joint_path = []
                continue
            endPoints = self.forwardKinematics(q[0], q[1])
            end0 = Point(self.circleEnv.scale_ * endPoints[0][0], self.circleEnv.scale_ * endPoints[0][1])
            end1 = Point(self.circleEnv.scale_ * endPoints[1][0], self.circleEnv.scale_ * endPoints[1][1])
            line1 = Line(self.root_point, end0)
            line2 = Line(end0, end1)
            joint_path.append([line1, line2])
        if joint_path != []:
            self.joint_paths.append(joint_path)
            joint_path = []
        
        # Progressively shade start to finished from red to green.
        for joint_path in self.joint_paths:
            current_red = 255
            inc = 255 / len(joint_path)
            for path in joint_path:
                for line in path:
                    line.setFill(color_rgb(int(current_red), int(255 - current_red), 0))
                current_red = current_red - inc

    def makeWindow(self):
        """ TODO: eventually take into account the root position of the arm.
        """
        total_x = self.circleEnv.maxX_ - self.circleEnv.minX_
        total_y = self.circleEnv.maxY_ - self.circleEnv.minY_
        win = GraphWin('circles', total_x, total_y)
        win.setCoords(self.circleEnv.minX_, self.circleEnv.minY_, self.circleEnv.maxX_, self.circleEnv.maxY_)
        return win

    def draw(self, win):
        for joint_path in self.joint_paths:
            self.circleEnv.draw(win)
            for path in joint_path:
                for line in path:
                    line.draw(win)
            win.getMouse()
            for item in win.items[:]:
                item.undraw()
            

def run_circles(obstacle_path, paths_path):
    """ A point robot with circles as collision obstacles.
    """
    # Read in the circles env
    cirEnv = CircleEnv()
    cirEnv.loadCircles(obstacle_path)
    cirEnv.loadPaths(paths_path)
    win = cirEnv.makeWindow()
    cirEnv.draw(win)
    win.getMouse()


def run_artic(obstacle_path, path_path):
    """ A 2 link RR robot with circles as collision obstacles.
    """
    env = ArticulatedEnv()
    env.loadCircles(obstacle_path)
    env.loadPath(path_path)
    win = env.makeWindow()
    env.draw(win)
    win.getMouse()

def main():
    parser = argparse.ArgumentParser(description='Display circle environments')
    parser.add_argument('--obs', help='circle obstacle file', default='circle_obstacles.txt')
    parser.add_argument('--paths', help='the file that contains the resulting paths', default='/tmp/tmpfile2.txt')

    subparser = parser.add_subparsers(help='environment types')
    circle_parser = subparser.add_parser('circle', help=run_circles.__doc__)
    circle_parser.set_defaults(func=run_circles)

    artic_parser = subparser.add_parser('articulated', help=run_artic.__doc__)
    artic_parser.set_defaults(func=run_artic)

    args = parser.parse_args()

    if not args.func:
        logging.error("Give a real command")
        quit()

    args.func("../../resources/" + args.obs, args.paths)


if __name__ == '__main__':
    main()
