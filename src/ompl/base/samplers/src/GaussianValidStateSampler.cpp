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

#include "ompl/base/samplers/GaussianValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/tools/config/MagicConstants.h"

ompl::base::GaussianValidStateSampler::GaussianValidStateSampler(const SpaceInformation *si)
  : ValidStateSampler(si)
  , sampler_(si->allocStateSampler())
  , stddev_(si->getMaximumExtent() * magic::STD_DEV_AS_SPACE_EXTENT_FRACTION)
{
    name_ = "gaussian";
    params_.declareParam<double>("standard_deviation",
                                 [this](double stddev)
                                 {
                                     setStdDev(stddev);
                                 },
                                 [this]
                                 {
                                     return getStdDev();
                                 });
}

bool ompl::base::GaussianValidStateSampler::sample(State *state)
{
    bool result = false;
    unsigned int attempts = 0;
    State *temp = si_->allocState();
    do
    {
        sampler_->sampleUniform(state);
        bool v1 = si_->isValid(state);
        sampler_->sampleGaussian(temp, state, stddev_);
        bool v2 = si_->isValid(temp);
        if (v1 != v2)
        {
            if (v2)
                si_->copyState(state, temp);
            result = true;
        }
        ++attempts;
    } while (!result && attempts < attempts_);
    si_->freeState(temp);
    return result;
}

bool ompl::base::GaussianValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    bool result = false;
    unsigned int attempts = 0;
    State *temp = si_->allocState();
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        bool v1 = si_->isValid(state);
        sampler_->sampleGaussian(temp, state, distance);
        bool v2 = si_->isValid(temp);
        if (v1 != v2)
        {
            if (v2)
                si_->copyState(state, temp);
            result = true;
        }
        ++attempts;
    } while (!result && attempts < attempts_);
    si_->freeState(temp);
    return result;
}
