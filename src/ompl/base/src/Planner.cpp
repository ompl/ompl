/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Rice University
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

#include "ompl/base/Planner.h"
#include "ompl/util/Exception.h"
#include "ompl/base/GoalSampleableRegion.h"

ompl::base::Planner::Planner(const SpaceInformationPtr &si, const std::string &name) : si_(si), pis_(this), name_(name), type_(PLAN_UNKNOWN), setup_(false), msg_(name)
{
    if (!si_)
	throw Exception(name_, "Invalid space information instance for planner");
}

ompl::base::PlannerType ompl::base::Planner::getType(void) const
{
    return type_;
}

const std::string& ompl::base::Planner::getName(void) const
{
    return name_;
}

const ompl::base::SpaceInformationPtr&  ompl::base::Planner::getSpaceInformation(void) const
{
    return si_;
}

const ompl::base::ProblemDefinitionPtr& ompl::base::Planner::getProblemDefinition(void) const
{
    return pdef_;
}

void ompl::base::Planner::setProblemDefinition(const ProblemDefinitionPtr &pdef)
{
    pdef_ = pdef;
    pis_.update();
}

void ompl::base::Planner::setup(void)
{
    if (!si_->isSetup())
    {
	msg_.inform("Space information setup was not yet called. Calling now.");
	si_->setup();
    }
    
    if (setup_)
	msg_.warn("Planner setup called multiple times");
    else
	setup_ = true;
}

bool ompl::base::Planner::isSetup(void) const
{
    return setup_;
}

void ompl::base::Planner::clear(void)
{
    pis_.clear();
    pis_.update();
}

void ompl::base::PlannerData::print(std::ostream &out) const
{
    out << states.size() << std::endl;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
	out << i << ": ";
	if (si)
	    si->printState(states[i], out);
	else
	    out << states[i] << std::endl;
    }
    
    for (unsigned int i = 0 ; i < edges.size() ; ++i)
    {
	if (edges[i].empty())
	    continue;
	out << i << ": ";
	for (unsigned int j = 0 ; j < edges[i].size() ; ++j)
	    out << edges[i][j] << ' ';
	out << std::endl;
    }    
}

void ompl::base::PlannerInputStates::clear(void)
{
    if (tempState_)
    {
	si_->freeState(tempState_);
	tempState_ = NULL;
    }
    addedStartStates_ = 0;
    sampledGoalsCount_ = 0;
    pdef_ = NULL;
    si_ = NULL;
}

bool ompl::base::PlannerInputStates::update(void)
{
    if (!planner_)
	throw Exception("No planner set for PlannerInputStates");
    return use(planner_->getSpaceInformation(), planner_->getProblemDefinition());
}

void ompl::base::PlannerInputStates::checkValidity(void) const
{
    std::string error;
    
    if (!pdef_)
	error = "Problem definition not specified";
    else
    {
	if (pdef_->getStartStateCount() <= 0)
	    error = "No start states specified";
	else
	    if (!pdef_->getGoal())
		error = "No goal specified";
    }
    
    if (!error.empty())
    {
	if (planner_)
	    throw Exception(planner_->getName(), error);
	else
	    throw Exception(error);
    }
}

bool ompl::base::PlannerInputStates::use(const SpaceInformationPtr &si, const ProblemDefinitionPtr &pdef)
{
    if (si && pdef)
	return use(si.get(), pdef.get());
    else
    {
	clear();
	return true;
    }
}

bool ompl::base::PlannerInputStates::use(const SpaceInformation *si, const ProblemDefinition *pdef)
{
    if (pdef_ != pdef || si_ != si)
    {
	clear();
	pdef_ = pdef;
	si_ = si;
	return true;
    }
    return false;
}

const ompl::base::State* ompl::base::PlannerInputStates::nextStart(void)
{
    if (pdef_ == NULL || si_ == NULL)
    {
	std::string error = "Missing space information or problem definition";
	if (planner_)
	    throw Exception(planner_->getName(), error);
	else
	    throw Exception(error);
    }
    
    while (addedStartStates_ < pdef_->getStartStateCount())
    {
	const base::State *st = pdef_->getStartState(addedStartStates_);
	addedStartStates_++;
	if (si_->satisfiesBounds(st) && si_->isValid(st))
	    return st;
	else
	{
	    msg::Interface msg(planner_ ? planner_->getName() : "");
	    msg.warn("Skipping invalid start state");
	}
    }
    return NULL;
}

const ompl::base::State* ompl::base::PlannerInputStates::nextGoal(time::point maxEndTime)
{
    if (pdef_ == NULL || si_ == NULL)
    {
	std::string error = "Missing space information or problem definition";
	if (planner_)
	    throw Exception(planner_->getName(), error);
	else
	    throw Exception(error);
    }
        
    const GoalSampleableRegion *goal = dynamic_cast<const GoalSampleableRegion*>(pdef_->getGoal().get());
    
    if (goal)
	if (sampledGoalsCount_ < goal->maxSampleCount())
	{
	    if (tempState_ == NULL)
		tempState_ = si_->allocState();
	    
	    do 
	    {
		goal->sampleGoal(tempState_);
		sampledGoalsCount_++;
		
		if (si_->satisfiesBounds(tempState_) && si_->isValid(tempState_))
		    return tempState_;
		else
		{
		    msg::Interface msg(planner_ ? planner_->getName() : "");
		    msg.warn("Skipping invalid goal state");
		}
	    }
	    while (sampledGoalsCount_ < goal->maxSampleCount() && time::now() < maxEndTime);
	}
    return NULL;
}

bool ompl::base::PlannerInputStates::haveMoreStartStates(void) const
{
    if (pdef_)
	return addedStartStates_ < pdef_->getStartStateCount();
    return false;
}

bool ompl::base::PlannerInputStates::haveMoreGoalStates(void) const
{
    if (pdef_ && pdef_->getGoal())
    {
	const GoalSampleableRegion *goal = dynamic_cast<const GoalSampleableRegion*>(pdef_->getGoal().get());
	if (goal)
	    return sampledGoalsCount_ < goal->maxSampleCount();
    }
    return false;
}
