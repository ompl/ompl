#!/bin/env python

from graphics import *
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
        self.paths = []
        self.points = []

    def loadCircles(self, filename):
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

    def loadPaths(self, filename):
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

def main():
    # Read in the circles env
    cirEnv = CircleEnv()
    cirEnv.loadCircles("../../resources/simple_circle_obstacles.txt")
    cirEnv.loadPaths("/tmp/tmpfile.txt")#"circle_paths.txt")
    win = cirEnv.makeWindow()
    #win.yUp()
    cirEnv.draw(win)
    raw_input('Press Enter to continue')


if __name__ == '__main__':
    main()
