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

#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/GoalState.h"
#include "ompl/base/GoalStates.h"

#include <sstream>

void ompl::base::ProblemDefinition::setStartAndGoalStates(const State *start, const State *goal, const double threshold)
{
    clearStartStates();
    clearGoal();
    addStartState(start);
    GoalState *gs = new GoalState(si_);
    gs->setState(goal);
    gs->threshold = threshold;
    setGoal(base::GoalPtr(gs));
}

bool ompl::base::ProblemDefinition::hasStartState(const State *state, unsigned int *startIndex)
{
    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
	if (si_->equalStates(state, startStates_[i]))
	{
	    if (startIndex)
		*startIndex = i;
	    return true;
	}
    return false;
}

bool ompl::base::ProblemDefinition::fixInvalidInputState(State *state, double dist, bool start, unsigned int attempts)
{ 
    bool result = false;

    bool b = si_->satisfiesBounds(state);
    bool v = false;
    if (b)
    {
	v = si_->isValid(state);
	if (!v)
	    msg_.debug("%s state is not valid", start ? "Start" : "Goal");
    }
    else
	msg_.debug("%s state is not within space bounds", start ? "Start" : "Goal");
    
    if (!b || !v)
    {
	std::stringstream ss;
	si_->printState(state, ss);
	ss << " within distance " << dist;
	msg_.debug("Attempting to fix %s state %s", start ? "start" : "goal", ss.str().c_str());
	
	State *temp = si_->allocState();
	if (si_->searchValidNearby(temp, state, dist, attempts))
	{
	    si_->copyState(state, temp);
	    result = true;
	}
	else
	    msg_.warn("Unable to fix %s state", start ? "start" : "goal");
	si_->freeState(temp);
    }
    
    return result;    
}


bool ompl::base::ProblemDefinition::fixInvalidInputStates(double distStart, double distGoal, unsigned int attempts)
{
    bool result = true;
    
    // fix start states
    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
	if (!fixInvalidInputState(startStates_[i], distStart, true, attempts))
	    result = false;
    
    // fix goal state
    GoalState *goal = dynamic_cast<GoalState*>(goal_.get());
    if (goal)
    {
	if (!fixInvalidInputState(goal->state, distGoal, false, attempts))
	    result = false;
    }

    // fix goal state
    GoalStates *goals = dynamic_cast<GoalStates*>(goal_.get());
    if (goals)
    {
	for (unsigned int i = 0 ; i < goals->states.size() ; ++i)
	    if (!fixInvalidInputState(goals->states[i], distGoal, false, attempts))
		result = false;
    }
    
    return result;    
}

void ompl::base::ProblemDefinition::getInputStates(std::vector<const State*> &states) const
{
    states.clear();
    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
	states.push_back(startStates_[i]);

    GoalState *goal = dynamic_cast<GoalState*>(goal_.get());
    if (goal)
	states.push_back(goal->state);

    GoalStates *goals = dynamic_cast<GoalStates*>(goal_.get());
    if (goals)
	for (unsigned int i = 0 ; i < goals->states.size() ; ++i)
	    states.push_back(goals->states[i]);
}

bool ompl::base::ProblemDefinition::isTrivial(unsigned int *startIndex, double *distance) const
{
    if (!goal_)
    {
	msg_.error("Goal undefined");
	return false;
    }
    
    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
    {
	const State *start = startStates_[i];
	if (start && si_->isValid(start) && si_->satisfiesBounds(start))
	{
	    double dist;
	    if (goal_->isSatisfied(start, &dist))
	    {
		if (startIndex)
		    *startIndex = i;
		if (distance)
		    *distance = dist;
		return true;
	    }	    
	}
	else
	{
	    msg_.error("Initial state is in collision!");
	}
    }
    
    return false;    
}

void ompl::base::ProblemDefinition::print(std::ostream &out) const
{
    out << "Start states:" << std::endl;
    for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
	si_->printState(startStates_[i], out);
    if (goal_)
	goal_->print(out);
    else
	out << "Goal = NULL" << std::endl;
}
