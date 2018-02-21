/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University.
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Bryce Willey */

#ifndef OMPL_TEST_ARTICULATED_CIRCLES_2D_
#define OMPL_TEST_ARTICULATED_CIRCLES_2D_

#include "circles2D.h"

struct Articulated2D
{

    struct Query
    {
        Query(Circles2D::Query cirQ) :
            start_one_(cirQ.startX_),
            start_two_(cirQ.startY_),
            goal_one_(cirQ.goalX_),
            goal_two_(cirQ.goalY_)
        {}

        double start_one_, start_two_, goal_one_, goal_two_;
    };

    Articulated2D(General2D *environ) :
        environment(environ),
        rootX_(0.0),
        rootY_(0.0),
        link_1_length_(0.0),
        link_2_length_(0.0)
    {}

    /**
     * Given the joint angles of the robot, return the end point of the 1st link and the end point
     * of the second link (the begining of the first link is root, and the beginning of the second
     * link is the end of the first link).
     * 
     * Generally meant to be used internally, for collision checking.
     */
    void forwardKinematics(double angle1, double angle2, Eigen::Vector2d& end1, Eigen::Vector2d& end2) const
    {
        end1[0] = rootX_ + cos(angle1) * link_1_length_;
        end1[1] = rootY_ + sin(angle1) * link_1_length_;

        end2[0] = end1[0] + cos(angle1 + angle2) * link_2_length_;
        end2[1] = end1[1] + sin(angle1 + angle2) * link_2_length_;
    }

    bool isInCollision(double angle1, double angle2) const
    {
        Eigen::Vector2d end1, end2, point1, point2;
        forwardKinematics(angle1, angle2, end1, end2);
        double dist1 = environment->lineSignedDistance(rootX_, rootY_, end1[0], end1[1], point1);
        if (dist1 <= 0.0)
            return true;
        double dist2 = environment->lineSignedDistance(end1[0], end1[1], end2[0], end2[1], point2);
        if (dist2 <= 0.0)
            return true;
        return false;
    }

    double signedDistance(double angle1, double angle2, int &link, Eigen::Vector2d& point, Eigen::Vector2d& global_point) const
    {
        Eigen::Vector2d end1, end2, point1, point2;
        forwardKinematics(angle1, angle2, end1, end2);
        double dist1 = environment->lineSignedDistance(rootX_, rootY_, end1[0], end1[1], point1);
        double dist2 = environment->lineSignedDistance(end1[0], end1[1], end2[0], end2[1], point2);
        if (dist1 <= dist2)
        {
            point[0] = point1[0] - rootX_;
            point[1] = point1[1] - rootY_;
            // We only want the rotated distance.
            point[0] = sqrt(point[0] * point[0] + point[1] * point[1]);
            point[1] = 0;
            global_point[0] = point1[0];
            global_point[1] = point1[1];
            link = 0;
            return dist1;
        }
        else
        {
            point[0] = point2[0] - end1[0];
            point[1] = point2[1] - end1[1];
            // We only want the rotated distance.
            point[0] = sqrt(point[0] * point[0] + point[1] * point[1]);
            point[1] = 0;
            global_point[0] = point2[0];
            global_point[1] = point2[1];
            link = 1;
            return dist2;
        }
    }

    double signedDistanceGradient(double angle1, double angle2, Eigen::MatrixXd &grad) const
    {
        // Get the signed distance and the closest point.
        int link;
        Eigen::Vector2d point, global_point;
        double min_dist = signedDistance(angle1, angle2, link, point, global_point);

        // Get the workspace gradient.
        environment->obstacleDistanceGradient(global_point[0], global_point[1], grad);
        
        // Get the jacobian: (hardcoded for RR manip from Lynch and Park book)
        Eigen::MatrixXd jaco(2, 2);
        if (link == 1)
        {
            double new_length = point[0];
            jaco << -link_1_length_ * sin(angle1) - new_length * sin(angle1 + angle2), 
                        -new_length * sin(angle1 + angle2),
                    link_1_length_ * cos(angle1) + link_1_length_ * cos(angle1 + angle2),
                        new_length * cos(angle1+ angle2);
        }
        else if (link == 0)
        {
            double new_length = point[0];
            jaco << -new_length * sin(angle1), 0,
                    new_length * cos(angle1), 0;
        }

        // Combine the workspace gradient with the jacobian.
        grad = grad * jaco; 

        return min_dist;
    }

    // When reading queries from here, they shouldn't be X and Y, they are angle1 and angle2.
    General2D *environment;

    double rootX_;
    double rootY_;

    double link_1_length_;
    double link_2_length_;
};

#endif