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

#ifndef PLANAR_MANIPULATOR_STATE_VALIDITY_CHECKER_H_
#define PLANAR_MANIPULATOR_STATE_VALIDITY_CHECKER_H_

#include <ompl/base/StateValidityChecker.h>
#include <eigen3/Eigen/Dense>
#include "PlanarManipulator.h"
#include "PlanarManipulatorStateSpace.h"
#include "PolyWorld.h"

typedef std::pair<Point, Point> Segment;

// Returns true if the x and y coordinates of p2 lie between the
// x and y coordinates of p0 and p1.
bool inBetween(Point p0, Point p1, Point p2)
{
    // p2.first should be between p0.first and p1.first
    bool goodx;
    // Check if the x coords are equal first
    if (cmpDouble(p1.first, p0.first))
        goodx = cmpDouble(p2.first, p0.first);
    else
    {
        if (p0.first > p1.first)
            goodx = (p2.first <= p0.first && p2.first >= p1.first);
        else
            goodx = (p2.first >= p0.first && p2.first <= p1.first);
    }

    bool goody;
    // Check if the y coords are equal first
    if (cmpDouble(p1.second, p0.second))
        goody = cmpDouble(p2.second, p0.second);
    else
    {
        if (p0.second > p1.second)
            goody = (p2.second <= p0.second && p2.second >= p1.second);
        else
            goody = (p2.second >= p0.second && p2.second <= p1.second);
    }

    return goodx && goody && !equalPoints(p0, p2) && !equalPoints(p1, p2);
}

// Return true if the three coordinates are collinear.
bool collinearPts(Point p0, Point p1, Point p2)
{
    // Check degenerate infinite slope case
    if (fabs(p1.first - p0.first) < 1e-6)
    {
        bool samex = fabs(p2.first - p1.first) < 1e-6;
        bool goody;
        if (p0.second > p1.second)
            goody = (p2.second <= p0.second && p2.second >= p1.second);
        else
            goody = (p2.second >= p0.second && p2.second <= p1.second);

        return samex && goody;
    }

    const double m = (p1.second - p0.second) / (p1.first - p0.first);
    const double b = p0.second - p0.first * m;

    // y = mx+b
    const double y = m * p2.first + b;
    const bool collinear = fabs(y - p2.second) < 1e-6;
    return collinear;
}

// Returns true if the line segments defined by p1-p2 and p3-p4 intersect.
bool lineLineIntersection(Point p1, Point p2, Point p3, Point p4)
{
    // There is a degenerate case where one of the endpoints lies on the line
    // defined by the other two points check this case (all combinations)
    if (collinearPts(p3, p4, p1) && inBetween(p3, p4, p1))
    {
        // p1 lies on the line between p3 and p4
        return true;
    }

    if (collinearPts(p3, p4, p2) && inBetween(p3, p4, p2))
    {
        // p2 lies on the line between p3 and p4
        return true;
    }

    if (collinearPts(p1, p2, p3) && inBetween(p1, p2, p3))
    {
        // p3 lies on the line between p1 and p2
        return true;
    }

    if (collinearPts(p1, p2, p4) && inBetween(p1, p2, p4))
    {
        // p4 lies on the line between p1 and p2
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
        // out = candidate;
        return true;
    }
    return false;
}

class PlanarManipulatorCollisionChecker : public ompl::base::StateValidityChecker
{
public:
    PlanarManipulatorCollisionChecker(const ompl::base::SpaceInformationPtr si, const PlanarManipulator &manip,
                                      const PolyWorld &world)
      : ompl::base::StateValidityChecker(si), manip_(manip), world_(world)
    {
    }

    ~PlanarManipulatorCollisionChecker() = default;

    virtual bool isValid(const ompl::base::State *state) const
    {
        const double *angles = state->as<PlanarManipulatorStateSpace::StateType>()->values;
        std::vector<Eigen::Affine2d> frames;
        manip_.FK(angles, frames);

        // Get the coordinates of the endpoint of each segment and the base
        // of the manipulator.
        std::vector<Point> coordinates;
        const Eigen::Affine2d &baseFrame = manip_.getBaseFrame();
        coordinates.push_back({baseFrame.translation()[0], baseFrame.translation()[1]});
        for (size_t i = 0; i < frames.size(); ++i)
            coordinates.push_back({frames[i].translation()(0), frames[i].translation()(1)});

        // Check each coordinate to make sure they are in bounds.
        for (size_t i = 1; i < coordinates.size(); ++i)
            if (world_.outOfBounds(coordinates[i]))
                return false;

        // Check each coordinate for obstacle intersection.
        for (size_t i = 1; i < coordinates.size(); ++i)
            for (size_t j = 0; j < world_.numObstacles(); ++j)
                if (world_.obstacle(j).inside(coordinates[i]))
                    return false;

        // Self-collision with the manipulator.
        if (inSelfCollision(coordinates))
            return false;

        // Above, we only checked the endpoints of the links of the manipulator.
        // We can still cut a corner of an obstacle.  Check for this.
        for (size_t i = 0; i < coordinates.size() - 1; ++i)
        {
            Point p1 = coordinates[i];
            Point p2 = coordinates[i + 1];

            for (size_t j = 0; j < world_.numObstacles(); ++j)
            {
                const ConvexPolygon &obstacle = world_.obstacle(j);
                Point prev = obstacle[obstacle.numPoints() - 1];
                for (size_t k = 0; k < obstacle.numPoints(); ++k)
                {
                    if (lineLineIntersection(p1, p2, prev, obstacle[k]))
                        return false;
                    prev = obstacle[k];
                }
            }
        }

        return true;
    }

private:
    bool inSelfCollision(const std::vector<Point> &points) const
    {
        // a single line cannot intersect with itself
        if (points.size() < 3)
            return false;

        // intersect all pairs of lines
        for (size_t i = 0; i < points.size() - 1; ++i)
            for (size_t j = i + 1; j < points.size() - 1; ++j)
                if (lineLineIntersection(points[i], points[i + 1], points[j], points[j + 1]))
                {
                    return true;
                }
        return false;
    }

    const PlanarManipulator &manip_;
    const PolyWorld &world_;
};

#endif
