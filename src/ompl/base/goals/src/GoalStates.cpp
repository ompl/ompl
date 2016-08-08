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

#include "ompl/base/goals/GoalStates.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <limits>

ompl::base::GoalStates::~GoalStates()
{
    freeMemory();
}

void ompl::base::GoalStates::clear()
{
    freeMemory();
    states_.clear();
}

void ompl::base::GoalStates::freeMemory()
{
    for (auto &state : states_)
        si_->freeState(state);
}

double ompl::base::GoalStates::distanceGoal(const State *st) const
{
    double dist = std::numeric_limits<double>::infinity();
    for (auto state : states_)
    {
        double d = si_->distance(st, state);
        if (d < dist)
            dist = d;
    }
    return dist;
}

void ompl::base::GoalStates::print(std::ostream &out) const
{
    out << states_.size() << " goal states, threshold = " << threshold_ << ", memory address = " << this << std::endl;
    for (auto state : states_)
    {
        si_->printState(state, out);
        out << std::endl;
    }
}

void ompl::base::GoalStates::sampleGoal(base::State *st) const
{
    if (states_.empty())
        throw Exception("There are no goals to sample");

    // Roll over the samplePosition_ if it points past the number of states.
    samplePosition_ = samplePosition_ % states_.size();
    // Get the next state.
    si_->copyState(st, states_[samplePosition_]);
    // Increment the counter. Do NOT roll over incase a new state is added before sampleGoal is called again.
    samplePosition_++;
}

unsigned int ompl::base::GoalStates::maxSampleCount() const
{
    return states_.size();
}

void ompl::base::GoalStates::addState(const State *st)
{
    states_.push_back(si_->cloneState(st));
}

void ompl::base::GoalStates::addState(const ScopedState<> &st)
{
    addState(st.get());
}

const ompl::base::State *ompl::base::GoalStates::getState(unsigned int index) const
{
    if (index >= states_.size())
        throw Exception("Index " + std::to_string(index) + " out of range. Only " + std::to_string(states_.size()) +
                        " states are available");
    return states_[index];
}

std::size_t ompl::base::GoalStates::getStateCount() const
{
    return states_.size();
}

bool ompl::base::GoalStates::hasStates() const
{
    return !states_.empty();
}
