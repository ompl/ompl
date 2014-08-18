/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

#include "ompl/base/spaces/AtlasConstraint.h"

#include <eigen3/Eigen/Dense>

ompl::base::AtlasConstraint::AtlasConstraint (AtlasStateSpacePtr &atlas)
: Constraint(boost::dynamic_pointer_cast<StateSpace>(atlas)), atlas_(*atlas), sampler_(&atlas_)
{
}

bool ompl::base::AtlasConstraint::isSatisfied (const State *state) const
{
    return distance(state) <= atlas_.getProjectionTolerance();
}

double ompl::base::AtlasConstraint::distance (const State *state) const
{
    return atlas_.bigF(toVector(state)).norm();
}

bool ompl::base::AtlasConstraint::sample (State *state)
{
    sampler_.sampleUniform(state->as<ompl::base::RealVectorStateSpace::StateType>());
    return project(state);
}

bool ompl::base::AtlasConstraint::project (State *state)
{
    Eigen::VectorXd x = toVector(state);
    if (atlas_.project(x, x))
        fromVector(state, x);
    
    return isSatisfied(state);
}

Eigen::VectorXd ompl::base::AtlasConstraint::toVector(const State *state) const
{
    Eigen::VectorXd x(atlas_.getAmbientDimension());
    for (unsigned int i = 0; i < x.size(); i++)
        x[i] = state->as<RealVectorStateSpace::StateType>()->values[i];
    return x;
}

void ompl::base::AtlasConstraint::fromVector(State *state, const Eigen::VectorXd &x) const
{
    for (unsigned int i = 0; i < x.size(); i++)
        state->as<RealVectorStateSpace::StateType>()->values[i] = x[i];
}
