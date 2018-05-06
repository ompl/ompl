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

/* Author: Caleb Voss */

#include "ompl/extensions/morse/MorseProjection.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/Exception.h"

ompl::base::MorseProjection::MorseProjection(const StateSpacePtr &space)
  : ProjectionEvaluator(space), space_(dynamic_cast<MorseStateSpace *>(space.get()))
{
    if (!space_)
        throw Exception("MORSE State Space needed for Morse Projection");
}

void ompl::base::MorseProjection::setup()
{
    ProjectionEvaluator::setup();
}

unsigned int ompl::base::MorseProjection::getDimension() const
{
    // default projection uses 2 coordinates for each rigid body
    return 2 * space_->getEnvironment()->rigidBodies_;
}

void ompl::base::MorseProjection::defaultCellSizes()
{
    cellSizes_ = std::vector<double>(getDimension(), 1.0);
}

void ompl::base::MorseProjection::project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const
{
    // this projection uses the x and y coordinates of every rigid body
    const MorseStateSpace::StateType *mstate = state->as<MorseStateSpace::StateType>();
    projection.resize(getDimension());
    for (unsigned int i = 0; i < space_->getEnvironment()->rigidBodies_; i++)
    {
        // for each rigid body, grab the x and y coords
        const double *values = mstate->as<RealVectorStateSpace::StateType>(4 * i)->values;
        projection[2 * i] = values[0];
        projection[2 * i + 1] = values[1];
    }
}
