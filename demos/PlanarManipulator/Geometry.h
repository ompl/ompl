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

#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <utility>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#define GEOM_F_EQ(a, b) (fabs(a - b) < 1e-12)

namespace Geometry
{
    typedef std::pair<double, double> Point;

    // HELPER FUNCTIONS //

    // Return true if two points are equal
    static inline bool equalPoints(const Point &p0, const Point &p1)
    {
        return GEOM_F_EQ(p0.first, p1.first) && GEOM_F_EQ(p0.second, p1.second);
    }

    // Return true if p2 is between p0 and p1
    bool inBetween(const Point &p0, const Point &p1, const Point &p2);

    // Return true if the three coordinates are collinear AND p2 lies between p0 and p1
    bool collinearPts(const Point &p0, const Point &p1, const Point &p2);

    // Intersect two line segments defined by p1-p2 and p3-p4 and get the intersection point
    bool lineLineIntersection(const Point &p1, const Point &p2, const Point &p3, const Point &p4, Point &out);

    // Compute the distance between a point and a line segment
    double linePointDistance(const Point &point, const Point &lineEnd1, const Point &lineEnd2);

    // Abstract definition of a discrete, 2D geometric shape
    class Geometry
    {
    public:
        Geometry()
        {
        }
        virtual ~Geometry()
        {
        }

        virtual unsigned int numVertices() const = 0;
        virtual void print(std::ostream &o = std::cout) const = 0;
        virtual bool inside(const Point &point) const = 0;

        // Return any point that lies inside the geometry
        virtual Point pointInside() const = 0;

        // Return a copy of the idxth vertex
        virtual Point operator[](unsigned int idx) const = 0;

        // Return the area of the geometry
        virtual double getArea() const = 0;
    };

    // Definition of a planar convex polygon.
    class ConvexPolygon : public Geometry
    {
    public:
        ConvexPolygon();
        virtual ~ConvexPolygon();

        // Computes the convex hull over a set of points to guarantee convexity
        virtual void initialize(const std::vector<Point> &coords);
        virtual unsigned int numVertices() const;

        virtual void print(std::ostream &o = std::cout) const;
        virtual bool inside(const Point &point) const;

        virtual Point pointInside() const;
        virtual Point operator[](unsigned int idx) const;

        virtual Point getCentroid() const;
        virtual double getArea() const;
        virtual double getSignedArea() const;
        virtual bool pointOnBoundary(const Point &p) const;

        // Return true if p1 is a subset of p2
        static bool polygonIsSubset(const ConvexPolygon &p1, const ConvexPolygon &p2);

        // Intersect polygons p1 and p2.  The intersection will be in "out"
        // If p1 and p2 do not intersect at all, false is returned
        static bool intersectPolygons(const ConvexPolygon &p1, const ConvexPolygon &p2, ConvexPolygon &out);

        // Compute the distance of the polygon from the point
        // If the point is inside the polygon, the method will return zero.
        static double distanceFromPoint(const ConvexPolygon &poly, const Point &pt);

    protected:
        // Intersect the line defined by points p1 and p2 with the polygon.  Retrieve the intersection point.
        // It is assumed that the line intersects at only one point
        static bool linePolygonIntersection(const Point &p1, const Point &p2, const ConvexPolygon &poly, Point &out);

        std::vector<Point> coordinates_;
    };

    class Triangle : public ConvexPolygon
    {
    public:
        Triangle();
        virtual ~Triangle();

        virtual void initialize(const std::vector<Point> &coords);
        virtual unsigned int numVertices() const;

        virtual void print(std::ostream &o = std::cout) const;

        virtual Point pointInside() const;
        virtual Point getCentroid() const;
    };
}  // namespace Geometry

#endif
