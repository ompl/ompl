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

#ifndef OMPL_TEST_GENERAL_2D_
#define OMPL_TEST_GENERAL_2D_

#include <fstream>
#include <iostream>
#include <vector>
#include <limits>
#include <functional>

/**
 * \brief A general 2d environment interface. Includes methods for getting
 * the validity, clearance, and gradients for points and lines.
 */
struct General2D
{
    /** \brief Returns true if the point (x, y) doesn't overlap with obstacles. */
    virtual bool noOverlap(double x, double y) const = 0;

    /** \brief Gets the positive or negative distance of a point towards obstacles. */
    virtual double signedDistance(double x, double y) const = 0;

    /** \brief Gets the signed distance and the gradient of the signed distance field. */
    virtual double obstacleDistanceGradient(double x, double y, Eigen::MatrixXd &grad) const = 0;

    /** \brief Gets the gradient, or direction that moves the quickes towards the medial axis. */
    virtual double medialAxisGradient(double x, double y, Eigen::MatrixXd &grad) const = 0;

    /** \brief Returns true if a line between the two given points is valid. */
    virtual bool lineNoOverlap(double x1, double y1, double x2, double y2) const = 0;

    /** \brief Gets the signed distance of a line, as well as the closest point. */
    virtual double lineSignedDistance(double x1, double y1, double x2, double y2, Eigen::Vector2d& point) const = 0;
};

#endif