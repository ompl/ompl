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

#ifndef PLANAR_MANIPULATOR_DEMO_H_
#define PLANAR_MANIPULATOR_DEMO_H_

#include <iomanip>
#include <fstream>
#include <ompl/geometric/planners/xxl/XXLDecomposition.h>
#include <ompl/base/StateValidityChecker.h>

#include "PolyWorld.h"
#include "PlanarManipulator.h"
#include "PlanarManipulatorStateSpace.h"
#include "PlanarManipulatorXXLDecomposition.h"
#include "BoundedPlanarManipulatorStateSpace.h"

// a point is just two numbers
// a segment is two points
typedef std::pair<double, double> Point2D;
typedef std::pair<Point2D, Point2D> Segment;

// an environment is a set of segments
typedef std::vector<Segment> Environment;
#define MAKE_SEGMENT(x1, y1, x2, y2) std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2))

// Meant for chains of unit length
// NOTE: The inputs actually do nothing.
Environment createZigZagEnvironment(int /*numLinks*/, double len = 0.1)
{
    std::vector<Segment> env;

    // "Floor"
    env.push_back(MAKE_SEGMENT(0, 0, 1.25, 0));
    // Left and right 'walls'
    env.push_back(MAKE_SEGMENT(0, 0, 0, 1.25));
    env.push_back(MAKE_SEGMENT(1.25, 0, 1.25, 1.25));
    // "Ceiling"
    env.push_back(MAKE_SEGMENT(0, 1.25, 1.25, 1.25));

    // Zig zaggey obstacles
    // Gap size defines the width of the corridor for the manipulator
    double gapSize = std::min(len * 2.5, 0.50);

    double toothSize = len / 1.5;

    double corridorX = 0.75;
    double corridorLeft = corridorX - (gapSize / 2.0);
    double corridorRight = corridorX + (gapSize / 2.0);

    // left bottom zig zag
    double x = corridorLeft;
    double y = gapSize + toothSize;
    double toothWidth = x / 5.0;  // this means we will have five teeth
    double dx = toothWidth / 2.0;

    while (x > 1e-3)
    {
        env.push_back(MAKE_SEGMENT(x, y, x - dx, y - toothSize));
        env.push_back(MAKE_SEGMENT(x - dx, y - toothSize, x - toothWidth, y));
        x -= toothWidth;
    }

    // left corridor zig zag
    x = corridorLeft;
    y = gapSize + toothSize;
    double toothHeight = (1.25 - gapSize) / 5.0;
    double dy = toothHeight / 2.0;

    while (y < (1.25 - gapSize) - 1e-3)
    {
        env.push_back(MAKE_SEGMENT(x, y, x - toothSize, y + dy));
        env.push_back(MAKE_SEGMENT(x - toothSize, y + dy, x, y + toothHeight));
        y += toothHeight;
    }

    // dog-ear the most recent segment
    env.pop_back();  // we don't need this segment anymore
    Point2D p1 = env.back().first;
    Point2D p2 = env.back().second;

    // Figure out the x intercept of the line
    y = 1.25 - gapSize;
    double m = (p2.second - p1.second) / (p2.first - p1.first);
    // x = (y - y1) / m + x1
    x = ((y - p1.second) / m) + p1.first;
    Segment &seg = env.back();
    Point2D &pt = seg.second;
    pt.first = x;
    pt.second = y;

    // top zig zag
    toothWidth = x / 5.0;
    dx = toothWidth / 2.0;
    while (x > 1e-3)
    {
        env.push_back(MAKE_SEGMENT(x, y, x - dx, y - toothSize));
        env.push_back(MAKE_SEGMENT(x - dx, y - toothSize, x - toothWidth, y));
        x -= toothWidth;
    }

    // right side obstacle
    x = corridorRight;
    y = gapSize + toothSize;

    // bottom
    toothWidth = (1.25 - x) / 3.0;
    dx = toothWidth / 2.0;
    while (x < 1.25)
    {
        env.push_back(MAKE_SEGMENT(x, y, x + dx, y - toothSize));
        env.push_back(MAKE_SEGMENT(x + dx, y - toothSize, x + toothWidth, y));
        x += toothWidth;
    }

    // right corridor.  Try to keep gapSize width at all points
    x = corridorRight;
    y = gapSize + toothSize;
    toothHeight = (1.25 - gapSize) / 5.0;
    dy = toothHeight / 2.0;

    // add segment to line up with the left side of the corridor
    // env.push_back(MAKE_SEGMENT(x, y, corridorLeft + gapSize, gapSize + toothSize));
    // x = corridorLeft + gapSize;
    // y = gapSize + toothSize;
    while (y < ((1.25 - gapSize) - 1e-3))
    {
        env.push_back(MAKE_SEGMENT(x, y, x - toothSize, y + dy));
        env.push_back(MAKE_SEGMENT(x - toothSize, y + dy, x, y + toothHeight));
        y += toothHeight;
    }

    // dog-ear the most recent segment
    env.pop_back();  // we don't need this segment anymore
    p1 = env.back().first;
    p2 = env.back().second;

    // Figure out the x intercept of the line
    y = 1.25 - gapSize;
    m = (p2.second - p1.second) / (p2.first - p1.first);
    x = ((y - p1.second) / m) + p1.first;
    Point2D &pt2 = env.back().second;
    pt2.first = x;
    pt2.second = y;

    // top zig zag
    toothWidth = (1.25 - x) / 3.0;
    dx = toothWidth / 2.0;
    while (x < 1.25)
    {
        env.push_back(MAKE_SEGMENT(x, y, x + dx, y - toothSize));
        env.push_back(MAKE_SEGMENT(x + dx, y - toothSize, x + toothWidth, y));
        x += toothWidth;
    }

    return env;
}

void createRectangleEnvironment(int numLinks, double len, PolyWorld &world)
{
    double gap = len * (1.1 * M_PI + log(numLinks) / (double)numLinks);

    double minX = 0;
    double minY = 0;
    double maxX = 1.25;
    double maxY = 1.25;

    std::vector<Geometry::Point> pts(4);

    // two obstacles with a gap in the middle
    double w = ((maxX - minX) * 0.50) - gap;
    double h1 = minY + gap;
    double h2 = maxY - gap;

    pts[0] = std::make_pair(minX, h1);
    pts[1] = std::make_pair(minX + w, h1);
    pts[2] = std::make_pair(minX + w, h2);
    pts[3] = std::make_pair(minX, h2);
    Geometry::ConvexPolygon *leftObs = new Geometry::ConvexPolygon();
    leftObs->initialize(pts);

    pts[0] = std::make_pair(minX + w + gap, h1);
    pts[1] = std::make_pair(maxX, h1);
    pts[2] = std::make_pair(maxX, h2);
    pts[3] = std::make_pair(minX + w + gap, h2);
    Geometry::ConvexPolygon *rightObs = new Geometry::ConvexPolygon();
    rightObs->initialize(pts);

    world.addObstacle(leftObs);
    world.addObstacle(rightObs);
}

void createClutterEnvironment(int numLinks, double len, PolyWorld &world)
{
    double gap = len * (1.1 * M_PI + log(numLinks) / (double)numLinks);
    double y1 = 0.25;                      // minY + gap;
    double height = 0.05;                  // gap / 2.0;
    double y2 = y1 + height + 0.75 * gap;  //(gap / 2.0);

    // lower level of obstacles.  One largeish gap, and a second smaller gap
    std::vector<Geometry::Point> pts(4);
    pts[0] = std::make_pair(0, y1);
    pts[1] = std::make_pair(0.25 - (gap / 2.0), y1);
    pts[2] = std::make_pair(0.25 - (gap / 2.0), y1 + height);
    pts[3] = std::make_pair(0, y1 + height);

    Geometry::ConvexPolygon *leftLower = new Geometry::ConvexPolygon();
    leftLower->initialize(pts);
    world.addObstacle(leftLower);

    pts[0] = std::make_pair(0.25 + (gap / 2.0), y1);
    pts[1] = std::make_pair(pts[0].first + (gap / 2.0), y1);
    pts[2] = std::make_pair(pts[0].first + (gap / 2.0), y1 + height);
    pts[3] = std::make_pair(0.25 + (gap / 2.0), y1 + height);

    Geometry::ConvexPolygon *leftMiddle = new Geometry::ConvexPolygon();
    leftMiddle->initialize(pts);
    world.addObstacle(leftMiddle);

    pts[0] = std::make_pair(pts[0].first + gap, y1);
    pts[1] = std::make_pair(world.getXBounds().second, y1);
    pts[2] = std::make_pair(world.getXBounds().second, y1 + height);
    pts[3] = std::make_pair(pts[0].first, y1 + height);

    Geometry::ConvexPolygon *leftRight = new Geometry::ConvexPolygon();
    leftRight->initialize(pts);
    world.addObstacle(leftRight);

    // Second level of obstacles.  Two obstacles blocking the gaps below
    pts[0] = std::make_pair(0.25 - (gap / 2.0), y2);
    pts[1] = std::make_pair(0.25 - (gap / 2.0) + gap, y2);
    pts[2] = std::make_pair(0.25 - (gap / 2.0) + gap, y2 + height);
    pts[3] = std::make_pair(0.25 - (gap / 2.0), y2 + height);

    Geometry::ConvexPolygon *upLeft = new Geometry::ConvexPolygon();
    upLeft->initialize(pts);
    world.addObstacle(upLeft);

    pts[0] = std::make_pair(0.25 + gap, y2);
    pts[1] = std::make_pair(0.25 + 1.5 * gap, y2);
    pts[2] = std::make_pair(0.25 + 1.5 * gap, y2 + height);
    pts[3] = std::make_pair(0.25 + gap, y2 + height);

    Geometry::ConvexPolygon *upRight = new Geometry::ConvexPolygon();
    upRight->initialize(pts);
    world.addObstacle(upRight);
}

void createConstrictedEnvironment(double len, PolyWorld &world)
{
    double gap = 1.5 * len;
    double w = 0.5;
    double ymax = 1.25;

    // robot base at y = 0.5

    std::vector<Geometry::Point> pts(4);

    // lower corridor obstacle
    pts[0] = std::make_pair(0, 0);
    pts[1] = std::make_pair(w, 0);
    pts[2] = std::make_pair(w, 0.5 - (gap / 2.0));
    pts[3] = std::make_pair(0, 0.5 - (gap / 2.0));
    Geometry::ConvexPolygon *lower = new Geometry::ConvexPolygon();
    lower->initialize(pts);
    world.addObstacle(lower);

    // upper corridor obstacle
    pts[0] = std::make_pair(0, 0.5 + (gap / 2.0));
    pts[1] = std::make_pair(w, 0.5 + (gap / 2.0));
    pts[2] = std::make_pair(w, ymax);
    pts[3] = std::make_pair(0, ymax);
    Geometry::ConvexPolygon *upper = new Geometry::ConvexPolygon();
    upper->initialize(pts);
    world.addObstacle(upper);

    // right side, nuisance obstacle.  Should be smallish
    // pts[0] = std::make_pair(0.5 + gap, 0.6);
    // pts[1] = std::make_pair(0.5 + gap + (gap/2.0), 0.6);
    // pts[2] = std::make_pair(0.5 + gap + (gap/2.0), 0.6 + (gap/2.0));
    // pts[3] = std::make_pair(0.5 + gap, 0.6 + (gap/2.0));
    pts[0] = std::make_pair(0.5 + 2.0 * gap, 0.6);
    pts[1] = std::make_pair(0.5 + 3.0 * gap, 0.6);
    pts[2] = std::make_pair(0.5 + 3.0 * gap, 0.6 + (gap / 2.0));
    pts[3] = std::make_pair(0.5 + 2.0 * gap, 0.6 + (gap / 2.0));
    Geometry::ConvexPolygon *right = new Geometry::ConvexPolygon();
    right->initialize(pts);
    world.addObstacle(right);
}

// now with less constriction
void createConstrictedEnvironment2(double gap, PolyWorld &world)
{
    // double gap = 2.0 * len;
    double w = 0.5;
    double ymax = 1.25;

    // robot base at y = 0.5
    std::vector<Geometry::Point> pts(4);

    // lower corridor obstacle
    pts[0] = std::make_pair(0, 0);
    pts[1] = std::make_pair(w, 0);
    pts[2] = std::make_pair(w, 0.5 - (gap / 2.0));
    pts[3] = std::make_pair(0, 0.5 - (gap / 2.0));
    Geometry::ConvexPolygon *lower = new Geometry::ConvexPolygon();
    lower->initialize(pts);
    world.addObstacle(lower);

    // upper corridor obstacle
    pts[0] = std::make_pair(0, 0.5 + (gap / 2.0));
    pts[1] = std::make_pair(w, 0.5 + (gap / 2.0));
    pts[2] = std::make_pair(w, ymax);
    pts[3] = std::make_pair(0, ymax);
    Geometry::ConvexPolygon *upper = new Geometry::ConvexPolygon();
    upper->initialize(pts);
    world.addObstacle(upper);

    // right side, nuisance obstacle.  Should be smallish
    pts[0] = std::make_pair(0.5 + gap, 0.5 + (gap / 2.0));
    pts[1] = std::make_pair(1.0, 0.5 + (gap / 2.0));
    pts[2] = std::make_pair(1.0, 0.5 + gap);
    pts[3] = std::make_pair(0.5 + gap, 0.5 + gap);
    Geometry::ConvexPolygon *right = new Geometry::ConvexPolygon();
    right->initialize(pts);
    world.addObstacle(right);
}

// Similar to above, but the tunnel the first part of the chain is
// in is tighter.
void createConstrictedEnvironment3(double gap, int links, PolyWorld &world)
{
    // Make the interior gap small enough so that it is difficult for the
    // chain to double back on itself on the first half.
    const double inner_gap = 1.0 / (links);  // * 2.0);

    // robot base at (0, 0.5), chain has unit length.
    const double robot_y = 0.5;
    const double ymax = 1.25;

    std::vector<Geometry::Point> pts(4);

    // lower corridor obstacle
    const double lower_interior_y = robot_y - inner_gap;
    const double upper_interior_y = robot_y + inner_gap;

    pts[0] = std::make_pair(0, 0);
    pts[1] = std::make_pair(0.5, 0);
    pts[2] = std::make_pair(0.5, lower_interior_y);
    pts[3] = std::make_pair(0, lower_interior_y);
    Geometry::ConvexPolygon *lower = new Geometry::ConvexPolygon();
    lower->initialize(pts);
    world.addObstacle(lower);

    // upper corridor obstacle
    pts[0] = std::make_pair(0, upper_interior_y);
    pts[1] = std::make_pair(0.5, upper_interior_y);
    pts[2] = std::make_pair(0.5, ymax);
    pts[3] = std::make_pair(0, ymax);
    Geometry::ConvexPolygon *upper = new Geometry::ConvexPolygon();
    upper->initialize(pts);
    world.addObstacle(upper);

    // The obstacle separating the start and goal configurations.
    pts[0] = std::make_pair(0.5 + gap, 0.5 + (gap / 2.0));
    pts[1] = std::make_pair(1.0, 0.5 + (gap / 2.0));
    pts[2] = std::make_pair(1.0, 0.5 + gap);
    pts[3] = std::make_pair(0.5 + gap, 0.5 + gap);
    Geometry::ConvexPolygon *right = new Geometry::ConvexPolygon();
    right->initialize(pts);
    world.addObstacle(right);
}

// Returns the y coordinate of the tunnel midpoint.
double createTunnelEnvironment(int links, PolyWorld &world)
{
    const double len = 1.0 / links;
    double gap = len * (1.1 * M_PI + log(links) / (double)links);
    OMPL_INFORM("Tunnel world.  Links: %d  Gap: %.3f", links, gap);
    std::vector<Geometry::Point> pts(4);

    // Two obstacles  that form a tunnel containing the goal.
    const double min_obs_y = 0.5;
    const double obs_height = 0.1;
    pts[0] = std::make_pair(0.5, min_obs_y);
    pts[1] = std::make_pair(1.0, min_obs_y);
    pts[2] = std::make_pair(1.0, min_obs_y + obs_height);
    pts[3] = std::make_pair(0.5, min_obs_y + obs_height);
    Geometry::ConvexPolygon *bottom = new Geometry::ConvexPolygon();
    bottom->initialize(pts);
    world.addObstacle(bottom);

    const double min_top_obs_y = min_obs_y + obs_height + gap;
    pts[0] = std::make_pair(0.5, min_top_obs_y);
    pts[1] = std::make_pair(1.0, min_top_obs_y);
    pts[2] = std::make_pair(1.0, min_top_obs_y + obs_height);
    pts[3] = std::make_pair(0.5, min_top_obs_y + obs_height);
    Geometry::ConvexPolygon *top = new Geometry::ConvexPolygon();
    top->initialize(pts);
    world.addObstacle(top);

    return min_obs_y + obs_height + (gap / 2.0);
}

// Use this and nothing else
class IsValidPolyWorld : public ompl::base::StateValidityChecker
{
public:
    IsValidPolyWorld(const ompl::base::SpaceInformationPtr si, const PlanarManipulator *manip, const PolyWorld *world)
      : ompl::base::StateValidityChecker(si), manip_(manip), world_(world)
    {
    }

    ~IsValidPolyWorld() = default;

    virtual bool isValid(const ompl::base::State *state) const
    {
        const double *angles = state->as<PlanarManipulatorStateSpace::StateType>()->values;
        std::vector<Eigen::Affine2d> frames;
        manip_->FK(angles, frames);

        // Convert to points
        std::vector<Point2D> coordinates;
        const Eigen::Affine2d &baseFrame = manip_->getBaseFrame();
        coordinates.push_back(std::make_pair(baseFrame.translation()[0], baseFrame.translation()[1]));
        for (size_t i = 0; i < frames.size(); ++i)
            coordinates.push_back(std::make_pair(frames[i].translation()(0), frames[i].translation()(1)));

        // in bounds?
        for (size_t i = 1; i < coordinates.size(); ++i)  // don't check base coord
            if (world_->outOfBounds(coordinates[i]))
            {
                return false;
            }

        // Check for the case where the robot is inside the obstacle
        for (size_t i = 0; i < coordinates.size(); ++i)
            for (unsigned int j = 0; j < world_->getNumObstacles(); ++j)
                if (world_->getObstacle(j)->inside(coordinates[i]))
                {
                    return false;
                }

        if (inSelfCollision(coordinates))
        {
            return false;
        }

        // Intersect all robosegments with all envirosegments
        for (size_t i = 0; i < coordinates.size() - 1; ++i)
        {
            Point2D p1 = coordinates[i];
            Point2D p2 = coordinates[i + 1];

            for (unsigned int j = 0; j < world_->getNumObstacles(); ++j)
            {
                const Geometry::Geometry *obstacle = world_->getObstacle(j);
                Geometry::Point prev = obstacle->operator[](obstacle->numVertices() - 1);
                for (unsigned int k = 0; k < obstacle->numVertices(); ++k)
                {
                    if (lineLineIntersection(p1, p2, prev, obstacle->operator[](k)))
                        return false;
                    prev = obstacle->operator[](k);
                }
            }
        }

        return true;
    }

    static bool inSelfCollision(const std::vector<Point2D> &points)
    {
        // a single line cannot intersect with itself
        if (points.size() < 3)
            return false;

        // intersect all pairs of lines
        for (size_t i = 0; i < points.size() - 1; ++i)
            for (size_t j = i + 1; j < points.size() - 1; ++j)
                if (lineLineIntersection(points[i], points[i + 1], points[j], points[j + 1]))
                    return true;
        return false;
    }

    static bool lineLineIntersection(const Point2D &p1, const Point2D &p2, const Point2D &q1, const Point2D &q2)
    {
        // re-encode the lines as rays
        // direction vector for l1
        std::vector<double> d1(2);
        d1[0] = p2.first - p1.first;
        d1[1] = p2.second - p1.second;

        // direction vector for l2
        std::vector<double> d2(2);
        d2[0] = q2.first - q1.first;
        d2[1] = q2.second - q1.second;

        // Compute determinant of direction vectors
        double det = d1[1] * d2[0] - d1[0] * d2[1];
        if (fabs(det) < 1e-6)  // rays are parallel.
            return false;

        double dx, dy;
        // Do not divide by zero when l2 is horizontal - just swap l1 and l2
        if (fabs(d2[1]) < 1e-4)
        {
            std::swap(d1[0], d2[0]);
            std::swap(d1[1], d2[1]);

            dx = q1.first - p1.first;
            dy = q1.second - p1.second;
            det = -det;
        }
        else
        {
            dx = p1.first - q1.first;
            dy = p1.second - q1.second;
        }

        // time parameter for the intersection of ray 1
        double t1 = (dx * d2[1] - dy * d2[0]) / det;
        if (fabs(t1) < 1e-6)
            t1 = 0.0;
        // time parameter for the intersection of ray 2
        double t2 = (dy + t1 * d1[1]) / d2[1];
        if (fabs(t2) < 1e-6)
            t2 = 0.0;
        // eps defines an 'allowable' intersection when lines join at the end
        double eps = 1e-3;
        if (t1 < eps || t2 < eps)  // Both scalars must be non-negative for an intersection
            return false;
        if (t1 > (1 - eps) || t2 > (1 - eps))  // Both scalars must be <= 1 for an intersection
            return false;

        return true;  // Yep, lines intersect
    }

protected:
    const PlanarManipulator *manip_;
    const PolyWorld *world_;

    mutable int numChecks, validStates, selfCollision, envCollision, outOfBounds;
};

void writeEnvironment(const Environment &env, const char *filename)
{
    std::ofstream fout;
    fout.open(filename);
    if (!fout)
    {
        std::cerr << "FAILED TO OPEN " << filename << " FOR WRITING" << std::endl;
        return;
    }

    fout << std::setprecision(10);  // lots of sigfigs

    for (size_t i = 0; i < env.size(); ++i)
    {
        // first point
        fout << env[i].first.first << " " << env[i].first.second
             << " "
             // second point
             << env[i].second.first << " " << env[i].second.second << std::endl;
    }

    fout.close();
}

void writeEnvironment(const PolyWorld &world, const char *filename)
{
    const std::vector<Geometry::Geometry *> &obstacles = world.getObstacles();
    Environment segments;
    for (const auto &obstacle : obstacles)
    {
        const Geometry::Geometry &geom = *(obstacle);
        for (unsigned int vx = 0; vx < geom.numVertices(); ++vx)
        {
            // close the loop
            if (vx == geom.numVertices() - 1)
            {
                segments.push_back(std::make_pair(geom[vx], geom[0]));
            }
            else
            {
                segments.push_back(std::make_pair(geom[vx], geom[vx + 1]));
            }
        }
    }
    writeEnvironment(segments, filename);
}

ompl::geometric::XXLDecompositionPtr getXXLDecomp(const ompl::base::SpaceInformationPtr &si,
                                                  const PlanarManipulator *manip, int numXYSlices, int numThetaSlices,
                                                  const std::pair<double, double> &xBounds,
                                                  const std::pair<double, double> &yBounds)
{
    unsigned int numLinks = manip->getNumLinks();
    double linkLength = 1.0 / numLinks;  // TODO: pass this in
    double chainLength = numLinks * linkLength;

    // Creating decomposition for XXL
    // Point2D origin = manip->getOrigin();
    const Eigen::Affine2d &baseFrame = manip->getBaseFrame();
    Point2D origin = std::make_pair(baseFrame.translation()(0), baseFrame.translation()(1));
    ompl::base::RealVectorBounds xyBounds(2);

    // Clip the bounds based on actual workspace bounds
    double xMin = std::max(origin.first - chainLength, xBounds.first);
    double xMax = std::min(origin.first + chainLength, xBounds.second);
    double yMin = std::max(origin.second - chainLength, yBounds.first);
    double yMax = std::min(origin.second + chainLength, yBounds.second);
    xyBounds.setLow(0, xMin);
    xyBounds.setHigh(0, xMax);
    xyBounds.setLow(1, yMin);
    xyBounds.setHigh(1, yMax);

    std::cout << "Decomposition bounds:  X[" << xyBounds.low[0] << "," << xyBounds.high[0] << "]  Y[" << xyBounds.low[1]
              << "," << xyBounds.high[1] << "]" << std::endl;

    std::vector<int> xySlices(2, numXYSlices);
    int thetaSlices = numThetaSlices;

    // Define a set of links whose end effector poses will constitute layers of projection
    // For chains with relatively few links, the reference point is the end of the chain
    // Longer chains put the reference point in the center of the chain
    // Can also add links for additional layers
    std::vector<int> projLinks;
    if (numLinks > 6)
        projLinks.push_back((numLinks / 2) - 1);
    projLinks.push_back(numLinks - 1);

    ompl::geometric::XXLDecompositionPtr decomp(new PMXXLDecomposition(si, manip, xyBounds, xySlices, thetaSlices,
                                                                       projLinks,
                                                                       true));  // true means diagonal edges
    return decomp;
}

#endif
