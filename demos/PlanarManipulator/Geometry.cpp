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

#include <algorithm>
#include <limits>
#include "Geometry.h"

// Return true if p2 is between p0 and p1
bool Geometry::inBetween(const Point &p0, const Point &p1, const Point &p2)
{
    // p2.first should be between p0.first and p1.first
    bool goodx;
    // Check if the x coords are equal first
    if (GEOM_F_EQ(p1.first, p0.first))
        goodx = GEOM_F_EQ(p2.first, p0.first);
    else
    {
        if (p0.first > p1.first)
            goodx = (p2.first <= p0.first && p2.first >= p1.first);
        else
            goodx = (p2.first >= p0.first && p2.first <= p1.first);
    }

    bool goody;
    // Check if the y coords are equal first
    if (GEOM_F_EQ(p1.second, p0.second))
        goody = GEOM_F_EQ(p2.second, p0.second);
    else
    {
        if (p0.second > p1.second)
            goody = (p2.second <= p0.second && p2.second >= p1.second);
        else
            goody = (p2.second >= p0.second && p2.second <= p1.second);
    }

    return goodx && goody;
}

// Return true if the three coordinates are collinear AND p2 lies between p0 and p1
bool Geometry::collinearPts(const Point &p0, const Point &p1, const Point &p2)
{
    // Check degenerate case
    if (fabs(p1.first - p0.first) < 1e-6)  // Infinite slope
    {
        bool samex = fabs(p2.first - p1.first) < 1e-6;
        bool goody;
        if (p0.second > p1.second)
            goody = (p2.second <= p0.second && p2.second >= p1.second);
        else
            goody = (p2.second >= p0.second && p2.second <= p1.second);

        return samex && goody;
    }

    double m = (p1.second - p0.second) / (p1.first - p0.first);
    double b = p0.second - p0.first * m;

    // y = mx+b
    double y = m * p2.first + b;
    bool collinear = fabs(y - p2.second) < 1e-6;

    if (collinear)
        return inBetween(p0, p1, p2);
    return false;
}

// http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
double Geometry::linePointDistance(const Point &point, const Point &lineEnd1, const Point &lineEnd2)
{
    double dx = lineEnd2.first - lineEnd1.first;
    double dy = lineEnd2.second - lineEnd1.second;
    double norm = dx * dx + dy * dy;

    double t = ((point.first - lineEnd1.first) * dx + (point.second - lineEnd1.second) * dy) / norm;

    if (t > 1.0)
        t = 1.0;
    else if (t < 0.0)
        t = 0.0;

    double x = lineEnd1.first + t * dx;
    double y = lineEnd1.second + t * dy;

    dx = x - point.first;
    dy = y - point.second;

    return sqrt(dx * dx + dy * dy);
}

// Intersect two line segments defined by p1-p2 and p3-p4 and get the intersection point
bool Geometry::lineLineIntersection(const Point &p1, const Point &p2, const Point &p3, const Point &p4, Point &out)
{
    // There is a degenerate case where one of the end-points lies exactly on the line
    // defined by the other two points check this case (all combinations)
    if (collinearPts(p3, p4, p1))  // p1 lies on the line between p3 and p4
    {
        out = p1;
        return true;
    }

    if (collinearPts(p3, p4, p2))  // p2 lies on the line between p3 and p4
    {
        out = p2;
        return true;
    }

    if (collinearPts(p1, p2, p3))  // p3 lies on the line between p1 and p2
    {
        out = p3;
        return true;
    }

    if (collinearPts(p1, p2, p4))  // p4 lies on the line between p1 and p2
    {
        out = p4;
        return true;
    }

    double x1 = p1.first;
    double x2 = p2.first;
    double x3 = p3.first;
    double x4 = p4.first;

    double y1 = p1.second;
    double y2 = p2.second;
    double y3 = p3.second;
    double y4 = p4.second;

    // Check if the lines are parallel
    double check = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (fabs(check) < 1e-6)  // lines are parallel if check == 0
        return false;

    // Computing intersection point using determinant method
    double x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) /
               ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    double y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) /
               ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

    Point candidate(x, y);
    // If candidate is between p1-p2 and p3-p4, then we win
    if (inBetween(p1, p2, candidate) && inBetween(p3, p4, candidate))
    {
        out = candidate;
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////

Geometry::ConvexPolygon::ConvexPolygon()
{
}

Geometry::ConvexPolygon::~ConvexPolygon()
{
}

// 1 indicates a "left" turn, -1 indicates a "right" turn, 0 indicates no turn (collinear)
static int turn(const Geometry::Point &p, const Geometry::Point &q, const Geometry::Point &r)
{
    double orn = (q.first - p.first) * (r.second - p.second) - (r.first - p.first) * (q.second - p.second);

    if (orn < 0)
        return -1;
    if (orn > 0)
        return 1;
    return 0;
}

static inline double sqdistance(const Geometry::Point &p1, const Geometry::Point &p2)
{
    double dx = p2.first - p1.first;
    double dy = p2.second - p1.second;
    return dx * dx + dy * dy;
}

void Geometry::ConvexPolygon::initialize(const std::vector<std::pair<double, double>> &coords)
{
    if (coords.size() < 3)
    {
        std::cerr << "[ERROR] Polygon initialization must have at least three coordinates." << std::endl;
        return;
    }

    // Computing convex hull of coordinates
    coordinates_.clear();

    // Compute convex hull of points
    // Find the left-most point in the convex hull
    unsigned int left = 0;
    for (unsigned int i = 1; i < coords.size(); ++i)
    {
        if (coords[i].first < coords[left].first)
            left = i;
        else if (coords[i].first == coords[left].first)
        {
            if (coords[i].second < coords[left].second)
                left = i;
        }
    }

    // Giftwrap
    coordinates_.push_back(coords[left]);
    for (unsigned int i = 0; i < coordinates_.size(); ++i)
    {
        Point p = coordinates_[i];
        Point q(p);
        for (unsigned int j = 0; j < coords.size(); ++j)
        {
            int t = turn(p, q, coords[j]);
            if (t == -1 ||
                (t == 0 && sqdistance(p, coords[j]) > sqdistance(p, q)))  // find the furthest, 'right-most' vertex
                q = coords[j];
        }

        if (!equalPoints(q, coordinates_[0]))
            coordinates_.push_back(q);
    }
}

unsigned int Geometry::ConvexPolygon::numVertices() const
{
    return coordinates_.size();
}

void Geometry::ConvexPolygon::print(std::ostream &o) const
{
    o << "Convex polygon with " << coordinates_.size() << " vertices:" << std::endl;
    for (size_t i = 0; i < coordinates_.size(); ++i)
        o << "  (" << coordinates_[i].first << " " << coordinates_[i].second << ")" << std::endl;
}

// Borrowed from PNPOLY
// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
// Count the number of times a ray extended horizontally from the y coordinate of the point intersects the polygon.
// If it intersects an odd number of times, the point is inside the polygon.
// Probably does not handle the border case properly.
bool Geometry::ConvexPolygon::inside(const Point &point) const
{
    size_t i, j;
    bool c = false;
    for (i = 0, j = coordinates_.size() - 1; i < coordinates_.size(); j = i++)
    {
        if (((coordinates_[i].second > point.second) != (coordinates_[j].second > point.second)) &&
            (point.first < (coordinates_[j].first - coordinates_[i].first) * (point.second - coordinates_[i].second) /
                                   (coordinates_[j].second - coordinates_[i].second) +
                               coordinates_[i].first))
            c = !c;
    }
    return c;
}

Geometry::Point Geometry::ConvexPolygon::pointInside() const
{
    return getCentroid();
}

Geometry::Point Geometry::ConvexPolygon::getCentroid() const
{
    double area = getSignedArea();
    Point centroid;
    centroid.first = 0.0;
    centroid.second = 0.0;

    for (size_t i = 0; i < coordinates_.size(); ++i)
    {
        Point p1 = coordinates_[i];
        Point p2 = coordinates_[(i + 1) % coordinates_.size()];

        centroid.first += (p1.first + p2.first) * (p1.first * p2.second - p2.first * p1.second);
        centroid.second += (p1.second + p2.second) * (p1.first * p2.second - p2.first * p1.second);
    }

    double coeff = 1.0 / (6.0 * area);
    centroid.first *= coeff;
    centroid.second *= coeff;
    return centroid;
}

double Geometry::ConvexPolygon::getSignedArea() const
{
    double area = 0.0;
    for (size_t i = 0; i < coordinates_.size(); ++i)
    {
        Point p1 = coordinates_[i];
        Point p2 = coordinates_[(i + 1) % coordinates_.size()];
        area += (p1.first * p2.second - p2.first * p1.second);
    }

    return 0.5 * area;
}

double Geometry::ConvexPolygon::getArea() const
{
    return fabs(getSignedArea());
}

bool Geometry::ConvexPolygon::pointOnBoundary(const Point &p) const
{
    bool onBoundary = false;
    for (unsigned int i = 0; i < coordinates_.size() && !onBoundary; ++i)
    {
        const Point &p1 = coordinates_[i];
        const Point &p2 = coordinates_[(i + 1) % coordinates_.size()];

        onBoundary = collinearPts(p1, p2, p);
    }

    return onBoundary;
}

Geometry::Point Geometry::ConvexPolygon::operator[](unsigned int idx) const
{
    return coordinates_[idx];
}

// Return true if p1 is a subset of p2
bool Geometry::ConvexPolygon::polygonIsSubset(const ConvexPolygon &p1, const ConvexPolygon &p2)
{
    bool subset = true;
    for (unsigned int i = 0; i < p1.numVertices() && subset; ++i)
        subset = (p2.inside(p1[i]) || p2.pointOnBoundary(p1[i]));

    return subset;
}

bool Geometry::ConvexPolygon::intersectPolygons(const ConvexPolygon &p1, const ConvexPolygon &p2, ConvexPolygon &out)
{
    // P1 is wholly contained in p2
    if (polygonIsSubset(p1, p2))
    {
        out = p1;
        return true;
    }

    // P2 is wholly contained in p1
    if (polygonIsSubset(p2, p1))
    {
        out = p2;
        return true;
    }

    // Find all vertices that overlap
    std::vector<Point> newPoints;
    for (size_t i = 0; i < p1.numVertices(); ++i)
    {
        if (p2.inside(p1[i]))
        {
            newPoints.push_back(p1[i]);
        }
    }
    for (size_t i = 0; i < p2.numVertices(); ++i)
    {
        if (p1.inside(p2[i]))
        {
            newPoints.push_back(p2[i]);
        }
    }

    // This is a horrible implementation.  Intersect all pairs of lines.
    // There is clearly a better method to do this. Some sort of line sweep + sort algorithm
    for (unsigned int i = 0; i < p1.numVertices(); ++i)
    {
        Point pt1 = p1[i];
        Point pt2 = p1[(i + 1) % p1.numVertices()];

        for (unsigned int j = 0; j < p2.numVertices(); ++j)
        {
            Point pt3 = p2[j];
            Point pt4 = p2[(j + 1) % p2.numVertices()];

            Point intersection;
            if (lineLineIntersection(pt1, pt2, pt3, pt4, intersection))
            {
                // See if this point already exists
                bool exists = false;
                for (size_t k = 0; k < newPoints.size() && !exists; ++k)
                    exists = equalPoints(intersection, newPoints[k]);

                if (!exists)
                    newPoints.push_back(intersection);
            }
        }
    }

    // The two polygons do not intersect
    if (newPoints.size() == 0)
        return false;

    // This case happens when GEOM_F_EQ says two points are the same, but in reality
    // they are just really really close to each other.  Willfully neglect this case.
    if (newPoints.size() < 3)
    {
        // std::cout << "[WARNING] Polygon intersection area is so small that it breaks maths." << std::endl;
        return false;
    }

    out.initialize(newPoints);
    return true;
}

// Intersect the line defined by points p1 and p2 with the polygon.  Retrieve the intersection point.
// It is assumed that the line intersects the polygon at only one point
bool Geometry::ConvexPolygon::linePolygonIntersection(const Point &p1, const Point &p2, const ConvexPolygon &poly,
                                                      Point &out)
{
    for (size_t i = 0; i < poly.numVertices(); ++i)
    {
        const Point &poly1 = poly[i];
        const Point &poly2 = poly[(i + 1) % poly.numVertices()];

        if (lineLineIntersection(p1, p2, poly1, poly2, out))
            return true;
    }

    return false;
}

double Geometry::ConvexPolygon::distanceFromPoint(const ConvexPolygon &poly, const Point &pt)
{
    if (poly.inside(pt))
        return 0;

    double minDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < poly.numVertices(); ++i)
    {
        const Point &p1 = poly[i];
        const Point &p2 = poly[(i + 1) % poly.numVertices()];

        double d = linePointDistance(pt, p1, p2);
        if (d < minDist)
            minDist = d;
    }

    return minDist;
}

/////////////////////////////////////////////////////////////////////////////////

Geometry::Triangle::Triangle()
{
}

Geometry::Triangle::~Triangle()
{
}

void Geometry::Triangle::initialize(const std::vector<Point> &coords)
{
    if (coords.size() != 3)
    {
        std::cout << "[ERROR] Triangles need exactly three coordinates" << std::endl;
        return;
    }

    coordinates_ = coords;
}

unsigned int Geometry::Triangle::numVertices() const
{
    return coordinates_.size();
}

void Geometry::Triangle::print(std::ostream &o) const
{
    o << "Triangle with " << coordinates_.size() << " vertices:" << std::endl;
    for (auto &coord : coordinates_)
        o << "  (" << coord.first << " " << coord.second << ")" << std::endl;
}

Geometry::Point Geometry::Triangle::pointInside() const
{
    return getCentroid();
}

Geometry::Point Geometry::Triangle::getCentroid() const
{
    Point p;
    p.first = 0.0;
    p.second = 0.0;

    // Average the vertices
    for (auto &coord : coordinates_)
    {
        p.first += coord.first;
        p.second += coord.second;
    }

    p.first /= coordinates_.size();
    p.second /= coordinates_.size();
    return p;
}
