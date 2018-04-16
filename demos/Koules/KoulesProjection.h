/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
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

/* Author: Beck Chen, Mark Moll */

#ifndef DEMOS_KOULES_PROJECTION_
#define DEMOS_KOULES_PROJECTION_

#include <ompl/base/spaces/RealVectorStateSpace.h>

// A projection for the KoulesStateSpace
class KoulesProjection : public ompl::base::ProjectionEvaluator
{
public:
    KoulesProjection(const ompl::base::StateSpace *space, unsigned int numDimensions = 3)
      : ompl::base::ProjectionEvaluator(space), numDimensions_(numDimensions)
    {
        unsigned int n = (space_->getDimension() - 1) / 2 + 1;
        if (numDimensions_ > n)
            numDimensions_ = n;
        else if (numDimensions_ < 3)
            numDimensions_ = 3;
    }

    virtual unsigned int getDimension() const
    {
        return numDimensions_;
    }
    virtual void defaultCellSizes()
    {
        cellSizes_.resize(numDimensions_, .05);
    }
    // projection with coordinates in the same order as described in Andrew
    // Ladd's thesis: (x,y,theta) of the ship, followed by the positions of
    // the koules (up to the dimensionality of the projection)
    virtual void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
    {
        const double *x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        unsigned int numKoules = (numDimensions_ - 3) / 2;
        projection[0] = x[0];
        projection[1] = x[1];
        projection[2] = numDimensions_ == 3 ? distanceGoal(state) : x[2];
        for (unsigned int i = 0; i < numKoules; ++i)
        {
            projection[2 * i + 3] = x[4 * i + 5];
            projection[2 * i + 4] = x[4 * i + 6];
        }
    }

    // distance to goal is used as a projection coordinate in 3D projections.
    //
    // This distance definition is adapted from the KoulesGoal class
    double distanceGoal(const ompl::base::State *st) const
    {
        double minX, minY;
        const double *v = st->as<KoulesStateSpace::StateType>()->values;
        std::size_t numKoules = (space_->getDimension() - 5) / 4, liveKoules = numKoules;
        double minDist = sideLength;

        for (std::size_t i = 1, j = 5; i <= numKoules; ++i, j += 4)
        {
            if (space_->as<KoulesStateSpace>()->isDead(st, i))
                liveKoules--;
            else
            {
                minX = std::min(v[j], sideLength - v[j]);
                minY = std::min(v[j + 1], sideLength - v[j + 1]);
                minDist = std::min(minDist, std::min(minX, minY) - kouleRadius);
            }
        }
        if (minDist < 0 || liveKoules == 0)
            minDist = 0;
        return .5 * sideLength * (double)liveKoules + minDist;
    }

protected:
    unsigned int numDimensions_;
};

#endif
