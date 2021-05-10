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

#ifndef PLANAR_MANIPULATOR_POLY_WORLD_H_
#define PLANAR_MANIPULATOR_POLY_WORLD_H_

#include "PolyWorld.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

// Create the 'corridor' environment for a unit-length planar manipulator with
// n links.  The chain must navigate a narrow gap.
PolyWorld createCorridorProblem(int n, Eigen::Affine2d &basePose, Eigen::Affine2d &goalPose)
{
    // The dimensions of the world.
    const double minX = 0;
    const double minY = 0;
    const double maxX = 1.25;
    const double maxY = 1.25;
    PolyWorld world("corridor", {minX, maxX}, {minY, maxY});

    const double len = 1.0 / n;
    const double gap = len * (1.1 * M_PI + log(n) / (double)n);

    std::vector<Point> pts(4);

    // two obstacles with a gap in the middle
    const double w = ((maxX - minX) * 0.50) - gap;
    const double h1 = minY + gap;
    const double h2 = maxY - gap;

    pts[0] = std::make_pair(minX, h1);
    pts[1] = std::make_pair(minX + w, h1);
    pts[2] = std::make_pair(minX + w, h2);
    pts[3] = std::make_pair(minX, h2);
    const ConvexPolygon left_obj(pts);

    pts[0] = std::make_pair(minX + w + gap, h1);
    pts[1] = std::make_pair(maxX, h1);
    pts[2] = std::make_pair(maxX, h2);
    pts[3] = std::make_pair(minX + w + gap, h2);
    const ConvexPolygon right_obj(pts);

    world.addObstacle(left_obj);
    world.addObstacle(right_obj);

    // Set the base frame for the manipulator.
    basePose = Eigen::Affine2d::Identity();
    basePose.translation()(1) = gap / 2.0;  // set the y coordinate of the base.

    // Set the goal frame for the end effector.
    goalPose = Eigen::Affine2d::Identity();
    goalPose.translation()(0) = (w + gap) * 0.95;
    goalPose.translation()(1) = std::min(4.0 * gap, 0.50);
    goalPose.rotate(0.0);

    return world;
}

// Create the 'constricted' environment for a planar manipulator with n links
// and unit-length.
// The first half of the chain lies in a narrow gap between two obstacles,
// whereas the second half of the chain is in a fairly open environment.
// Returns the position of the base of the chain.
PolyWorld createConstrictedProblem(int n, Eigen::Affine2d &basePose, Eigen::Affine2d &goalPose)
{
    const double minX = 0;
    const double minY = 0;
    const double maxX = 1.25;
    const double maxY = 1.25;
    PolyWorld world("constricted", {minX, maxX}, {minY, maxY});

    const double gap = (log10(n) / (double)n) * 2.0;

    // Make the interior gap small enough so that it is difficult for the
    // chain to double back on itself on the first half.
    const double inner_gap = 1.0 / n;

    // robot base at (0, 0.5), chain has unit length.
    const double robot_y = 0.5;
    const double ymax = 1.25;

    std::vector<Point> pts(4);

    // lower corridor obstacle
    const double lower_interior_y = robot_y - inner_gap;
    const double upper_interior_y = robot_y + inner_gap;

    {
        pts[0] = std::make_pair(0, 0);
        pts[1] = std::make_pair(0.5, 0);
        pts[2] = std::make_pair(0.5, lower_interior_y);
        pts[3] = std::make_pair(0, lower_interior_y);
        const ConvexPolygon lower(pts);
        world.addObstacle(lower);
    }

    // upper corridor obstacle
    {
        pts[0] = std::make_pair(0, upper_interior_y);
        pts[1] = std::make_pair(0.5, upper_interior_y);
        pts[2] = std::make_pair(0.5, ymax);
        pts[3] = std::make_pair(0, ymax);
        const ConvexPolygon upper(pts);
        world.addObstacle(upper);
    }

    // The obstacle separating the start and goal configurations.
    {
        pts[0] = std::make_pair(0.5 + gap, 0.5 + (gap / 2.0));
        pts[1] = std::make_pair(1.0, 0.5 + (gap / 2.0));
        pts[2] = std::make_pair(1.0, 0.5 + gap);
        pts[3] = std::make_pair(0.5 + gap, 0.5 + gap);
        const ConvexPolygon right(pts);
        world.addObstacle(right);
    }

    basePose = Eigen::Affine2d::Identity();
    basePose.translation()(1) = robot_y;

    goalPose = Eigen::Affine2d::Identity();
    goalPose.translation()(0) = 0.75;
    goalPose.translation()(1) = 0.5 + 1.5 * gap;
    goalPose.rotate(0.0);

    return world;
}

#endif
