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

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// A projection for the KoulesStateSpace
class KoulesProjection : public ompl::base::ProjectionEvaluator
{
public:
    KoulesProjection(const ompl::base::StateSpace* space, unsigned int numDimensions = 3)
        : ompl::base::ProjectionEvaluator(space), numDimensions_(numDimensions)
    {
        unsigned int n = (space_->getDimension() - 1) / 2 + 1;
        if (numDimensions_ > n)
            numDimensions_ = n;
        else if (numDimensions_ < 3)
            numDimensions_ = 3;
    }

    virtual unsigned int getDimension(void) const
    {
        return numDimensions_;
    }
    virtual void defaultCellSizes(void)
    {
        cellSizes_.resize(numDimensions_, .05);
    }
    virtual void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
    {
        const ompl::base::CompoundStateSpace::StateType* cs = state->as<ompl::base::CompoundStateSpace::StateType>();
        const double* xv = cs->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
        const double theta = cs->as<ompl::base::SO2StateSpace::StateType>(1)->value;
        unsigned int numKoules = (numDimensions_ - 3) / 2;
        // projection with coordinates in the same order as described in Andrew Ladd's thesis
        projection[0] = xv[4 * numKoules];
        projection[1] = xv[4 * numKoules + 1];
        projection[2] = theta;
        for (unsigned int i = 0; i < numKoules; ++i)
        {
            projection[2 * i + 3] = xv[4 * i];
            projection[2 * i + 4] = xv[4 * i + 1];
        }
    }
protected:
    unsigned int numDimensions_;
};

#endif
