/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "ompl/base/goals/GoalState.h"
#include "ompl/base/SpaceInformation.h"

ompl::base::GoalState::~GoalState()
{
    if (state_ != nullptr)
        si_->freeState(state_);
}

double ompl::base::GoalState::distanceGoal(const State *st) const
{
    return si_->distance(st, state_);
}

void ompl::base::GoalState::print(std::ostream &out) const
{
    out << "Goal state, threshold = " << threshold_ << ", memory address = " << this << ", state = " << std::endl;
    si_->printState(state_, out);
}

void ompl::base::GoalState::sampleGoal(base::State *st) const
{
    si_->copyState(st, state_);
}

unsigned int ompl::base::GoalState::maxSampleCount() const
{
    return 1;
}

void ompl::base::GoalState::setState(const State *st)
{
    if (state_ != nullptr)
        si_->freeState(state_);
    state_ = si_->cloneState(st);
}

void ompl::base::GoalState::setState(const ScopedState<> &st)
{
    setState(st.get());
}

const ompl::base::State *ompl::base::GoalState::getState() const
{
    return state_;
}

ompl::base::State *ompl::base::GoalState::getState()
{
    return state_;
}
