/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ryan Luna */

#ifndef POLYWORLD_H_
#define POLYWORLD_H_

#include <cmath>
#include <string>
#include <vector>

// Definition of a point (x,y).
typedef std::pair<double, double> Point;

// Returns true if the difference between a and b is < eps.
bool cmpDouble(double a, double b, const double eps = 1e-12);

// Returns true if the two points are the same.
bool equalPoints(Point p0, Point p1);

// Definition of a planar convex polygon.
class ConvexPolygon
{
public:
    // Initializes a new convex polygon from the set of points.  The
    // convex hull of the points are taken to guarantee convexity.
    ConvexPolygon(const std::vector<Point> &coords);

    // Returns the number of vertices on the convex polygon.
    size_t numPoints() const
    {
        return coordinates_.size();
    }
    // Returns the ith point of this polygon.
    Point operator[](size_t i) const
    {
        return coordinates_[i];
    }

    // Returns true if this polygon contains the given point.
    bool inside(Point point) const;

private:
    std::vector<Point> coordinates_;
};

// A representation of a bounded planar world composed of polygonal obstacles.
class PolyWorld
{
public:
    PolyWorld(const std::string &worldName, const std::pair<double, double> &xBounds,
              const std::pair<double, double> &yBounds);

    const std::string &worldName() const;
    std::pair<double, double> xBounds() const;
    std::pair<double, double> yBounds() const;

    size_t numObstacles() const;
    const std::vector<ConvexPolygon> &obstacles() const;
    const ConvexPolygon &obstacle(size_t i) const;

    // Adds the obstacle to the world.
    void addObstacle(const ConvexPolygon &polygon);

    // Returns true if the given point is outside of the bounds
    // defined for this world, false otherwise.
    bool outOfBounds(Point p) const;

    // Returns true if the given point does not collide with any obstacle.
    bool pointCollisionFree(Point p) const;

    // Write the world to the given filename in YAML format
    void writeWorld(const char *filename) const;

protected:
    std::string worldName_;
    std::vector<std::pair<double, double>> bounds_;

    std::vector<ConvexPolygon> obstacles_;
};

#endif
