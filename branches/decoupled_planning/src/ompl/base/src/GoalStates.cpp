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

#include "ompl/base/GoalStates.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <limits>

ompl::base::GoalStates::~GoalStates(void)
{
    freeMemory();
}

void ompl::base::GoalStates::clear(void)
{
    freeMemory();
    states.clear();
}

void ompl::base::GoalStates::freeMemory(void)
{
    for (unsigned int i = 0 ; i < states.size() ; ++i)
	si_->freeState(states[i]);
}

double ompl::base::GoalStates::distanceGoal(const State *st) const
{
    double dist = std::numeric_limits<double>::infinity();
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
	double d = si_->distance(st, states[i]);
	if (d < dist)
	    dist = d;
    }
    return dist;
}

void ompl::base::GoalStates::print(std::ostream &out) const
{
    out << states.size() << " goal states, threshold = " << threshold_ << ", memory address = " << this << std::endl;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
	si_->printState(states[i], out);
	out << std::endl;
    }
}

void ompl::base::GoalStates::sampleGoal(base::State *st) const
{
    if (states.empty())
	throw Exception("There are no goals to sample");
    si_->copyState(st, states[samplePosition]);
    samplePosition = (samplePosition + 1) % states.size();
}

unsigned int ompl::base::GoalStates::maxSampleCount(void) const
{
    return states.size();
}

void ompl::base::GoalStates::addState(const State* st)
{
    states.push_back(si_->cloneState(st));
}

void ompl::base::GoalStates::addState(const ScopedState<> &st)
{
    addState(st.get());
}

	    
