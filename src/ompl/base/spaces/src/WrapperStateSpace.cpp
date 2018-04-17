/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

/* Author: Zachary Kingston */

#include "ompl/base/spaces/WrapperStateSpace.h"

void ompl::base::WrapperStateSampler::sampleUniform(State *state)
{
    sampler_->sampleUniform(state->as<ompl::base::WrapperStateSpace::StateType>()->getState());
}

void ompl::base::WrapperStateSampler::sampleUniformNear(State *state, const State *near, double distance)
{
    sampler_->sampleUniformNear(state->as<ompl::base::WrapperStateSpace::StateType>()->getState(),
                                near->as<ompl::base::WrapperStateSpace::StateType>()->getState(), distance);
}

void ompl::base::WrapperStateSampler::sampleGaussian(State *state, const State *mean, double stdDev)
{
    sampler_->sampleGaussian(state->as<ompl::base::WrapperStateSpace::StateType>()->getState(),
                             mean->as<ompl::base::WrapperStateSpace::StateType>()->getState(), stdDev);
}

ompl::base::WrapperProjectionEvaluator::WrapperProjectionEvaluator(const ompl::base::WrapperStateSpace *space)
  : ompl::base::ProjectionEvaluator(space), projection_(space->getSpace()->getDefaultProjection())
{
}

void ompl::base::WrapperProjectionEvaluator::setup()
{
    cellSizes_ = projection_->getCellSizes();
    ProjectionEvaluator::setup();
}

unsigned int ompl::base::WrapperProjectionEvaluator::getDimension() const
{
    return projection_->getDimension();
}

void ompl::base::WrapperProjectionEvaluator::project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const
{
    projection_->project(state->as<ompl::base::WrapperStateSpace::StateType>()->getState(), projection);
}

void ompl::base::WrapperStateSpace::setup()
{
    space_->setup();
    maxExtent_ = space_->getMaximumExtent();
    longestValidSegmentFraction_ = space_->getLongestValidSegmentFraction();
    longestValidSegmentCountFactor_ = space_->getValidSegmentCountFactor();
    longestValidSegment_ = space_->getLongestValidSegmentLength();
    projections_ = space_->getRegisteredProjections();
    params_ = space_->params();

    valueLocationsInOrder_ = space_->getValueLocations();
    valueLocationsByName_ = space_->getValueLocationsByName();
    substateLocationsByName_ = space_->getSubstateLocationsByName();

    registerDefaultProjection(std::make_shared<WrapperProjectionEvaluator>(this));
}
