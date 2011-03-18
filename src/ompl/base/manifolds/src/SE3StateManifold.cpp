/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#include "ompl/base/manifolds/SE3StateManifold.h"
#include "ompl/util/MagicConstants.h"
#include <cstring>

ompl::base::State* ompl::base::SE3StateManifold::allocState(void) const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::base::SE3StateManifold::freeState(State *state) const
{
    CompoundStateManifold::freeState(state);
}

void ompl::base::SE3StateManifold::registerProjections(void)
{
    class SE3DefaultProjection : public ProjectionEvaluator
    {
    public:

        SE3DefaultProjection(const StateManifold *manifold) : ProjectionEvaluator(manifold)
        {
        }

        virtual unsigned int getDimension(void) const
        {
            return 3;
        }

        virtual void defaultCellSizes(void)
        {
            cellSizes_.resize(3);
            const RealVectorBounds &b = manifold_->as<SE3StateManifold>()->getBounds();
            cellSizes_[0] = (b.high[0] - b.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (b.high[1] - b.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[2] = (b.high[2] - b.low[2]) / magic::PROJECTION_DIMENSION_SPLITS;
        }

        virtual void project(const State *state, EuclideanProjection &projection) const
        {
            memcpy(projection.values, state->as<SE3StateManifold::StateType>()->as<RealVectorStateManifold::StateType>(0)->values, 3 * sizeof(double));
        }
    };

    registerDefaultProjection(ProjectionEvaluatorPtr(dynamic_cast<ProjectionEvaluator*>(new SE3DefaultProjection(this))));
}
