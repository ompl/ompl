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

#include "ompl/base/samplers/MaximizeClearanceValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"

ompl::base::MaximizeClearanceValidStateSampler::MaximizeClearanceValidStateSampler(const SpaceInformation *si)
  : ValidStateSampler(si), sampler_(si->allocStateSampler()), improveAttempts_(3), work_(si->allocState())
{
    name_ = "max_clearance";
    params_.declareParam<unsigned int>("nr_improve_attempts",
                                       [this](unsigned int n)
                                       {
                                           setNrImproveAttempts(n);
                                       },
                                       [this]
                                       {
                                           return getNrImproveAttempts();
                                       });
}

ompl::base::MaximizeClearanceValidStateSampler::~MaximizeClearanceValidStateSampler()
{
    si_->freeState(work_);
}

bool ompl::base::MaximizeClearanceValidStateSampler::sample(State *state)
{
    unsigned int attempts = 0;
    bool valid = false;
    double dist = 0.0;
    do
    {
        sampler_->sampleUniform(state);
        valid = si_->getStateValidityChecker()->isValid(state, dist);
        ++attempts;
    } while (!valid && attempts < attempts_);

    if (valid)
    {
        bool validW = false;
        double distW = 0.0;
        attempts = 0;
        while (attempts < improveAttempts_)
        {
            sampler_->sampleUniform(work_);
            validW = si_->getStateValidityChecker()->isValid(work_, distW);
            ++attempts;
            if (validW && distW > dist)
            {
                dist = distW;
                si_->copyState(state, work_);
            }
        }
        return true;
    }
    return false;
}

bool ompl::base::MaximizeClearanceValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    unsigned int attempts = 0;
    bool valid = false;
    double dist = 0.0;
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        valid = si_->getStateValidityChecker()->isValid(state, dist);
        ++attempts;
    } while (!valid && attempts < attempts_);

    if (valid)
    {
        bool validW = false;
        double distW = 0.0;
        attempts = 0;
        while (attempts < improveAttempts_)
        {
            sampler_->sampleUniformNear(work_, near, distance);
            validW = si_->getStateValidityChecker()->isValid(work_, distW);
            ++attempts;
            if (validW && distW > dist)
            {
                dist = distW;
                si_->copyState(state, work_);
            }
        }
        return true;
    }
    return false;
}
