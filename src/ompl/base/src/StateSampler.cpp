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

#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"

void ompl::base::CompoundStateSampler::addSampler(const StateSamplerPtr &sampler, double weightImportance)
{
    samplers_.push_back(sampler);
    weightImportance_.push_back(weightImportance);
    samplerCount_ = samplers_.size();
}

void ompl::base::CompoundStateSampler::sampleUniform(State *state)
{
    State **comps = state->as<CompoundState>()->components;
    for (unsigned int i = 0; i < samplerCount_; ++i)
        samplers_[i]->sampleUniform(comps[i]);
}

void ompl::base::CompoundStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    State **comps = state->as<CompoundState>()->components;
    State **nearComps = near->as<CompoundState>()->components;
    for (unsigned int i = 0; i < samplerCount_; ++i)
        if (weightImportance_[i] > std::numeric_limits<double>::epsilon())
            samplers_[i]->sampleUniformNear(comps[i], nearComps[i], distance * weightImportance_[i]);
        else
            samplers_[i]->sampleUniform(comps[i]);
}

void ompl::base::CompoundStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    State **comps = state->as<CompoundState>()->components;
    State **meanComps = mean->as<CompoundState>()->components;
    for (unsigned int i = 0; i < samplerCount_; ++i)
        samplers_[i]->sampleGaussian(comps[i], meanComps[i], stdDev * weightImportance_[i]);
}

ompl::base::SubspaceStateSampler::SubspaceStateSampler(const StateSpace *space, const StateSpace *subspace,
                                                       double weight)
  : StateSampler(space), subspace_(subspace), weight_(weight)
{
    work_ = subspace_->allocState();
    work2_ = subspace_->allocState();
    subspaceSampler_ = subspace_->allocStateSampler();
    space_->getCommonSubspaces(subspace_, subspaces_);
    if (subspaces_.empty())
        OMPL_WARN("Subspace state sampler did not find any common subspaces. Sampling will have no effect.");
}

ompl::base::SubspaceStateSampler::~SubspaceStateSampler()
{
    subspace_->freeState(work_);
    subspace_->freeState(work2_);
}

void ompl::base::SubspaceStateSampler::sampleUniform(State *state)
{
    subspaceSampler_->sampleUniform(work_);
    copyStateData(space_, state, subspace_, work_, subspaces_);
}

void ompl::base::SubspaceStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    copyStateData(subspace_, work2_, space_, near);
    subspaceSampler_->sampleUniformNear(work_, work2_, distance * weight_);
    copyStateData(space_, state, subspace_, work_, subspaces_);
}

void ompl::base::SubspaceStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    copyStateData(subspace_, work2_, space_, mean);
    subspaceSampler_->sampleGaussian(work_, work2_, stdDev * weight_);
    copyStateData(space_, state, subspace_, work_, subspaces_);
}
