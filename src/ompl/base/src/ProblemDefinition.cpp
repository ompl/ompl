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
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/PathControl.h"
#include <sstream>

void ompl::base::ProblemDefinition::setStartAndGoalStates(const State *start, const State *goal, const double threshold)
{
    clearStartStates();
    addStartState(start);
    setGoalState(goal, threshold);
}

void ompl::base::ProblemDefinition::setGoalState(const State *goal, const double threshold)
{
    clearGoal();
    GoalState *gs = new GoalState(si_);
    gs->setState(goal);
    gs->setThreshold(threshold);
    setGoal(GoalPtr(gs));
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

ompl::base::PathPtr ompl::base::ProblemDefinition::isStraightLinePathValid(void) const
{
    PathPtr path;
    if (control::SpaceInformationPtr sic = boost::dynamic_pointer_cast<control::SpaceInformation, SpaceInformation>(si_))
    {
        unsigned int startIndex;
        if (isTrivial(&startIndex, NULL))
        {
            control::PathControl *pc = new control::PathControl(sic);
            pc->states.push_back(sic->cloneState(startStates_[startIndex]));
            pc->states.push_back(sic->cloneState(startStates_[startIndex]));
            pc->controls.push_back(sic->allocControl());
            sic->nullControl(pc->controls.back());
            pc->controlDurations.push_back(0);
            path.reset(pc);
        }
        else
        {
            control::Control *nc = sic->allocControl();
            State *result1 = sic->allocState();
            State *result2 = sic->allocState();
            sic->nullControl(nc);

            for (unsigned int k = 0 ; k < startStates_.size() && !path ; ++k)
            {
                const State *start = startStates_[k];
                if (start && si_->isValid(start) && si_->satisfiesBounds(start))
                {
                    sic->copyState(result1, start);
                    for (unsigned int i = 0 ; i < sic->getMaxControlDuration() && !path ; ++i)
                        if (sic->propagateWhileValid(result1, nc, 1, result2))
                        {
                            if (goal_->isSatisfied(result2))
                            {
                                control::PathControl *pc = new control::PathControl(sic);
                                pc->states.push_back(sic->cloneState(start));
                                pc->states.push_back(sic->cloneState(result2));
                                pc->controls.push_back(sic->cloneControl(nc));
                                pc->controlDurations.push_back(i + 1);
                                path.reset(pc);
                                break;
                            }
                            std::swap(result1, result2);
                        }
                }
            }
            sic->freeState(result1);
            sic->freeState(result2);
            sic->freeControl(nc);
        }
    }
    else
    {
        std::vector<const State*> states;
        GoalState *goal = dynamic_cast<GoalState*>(goal_.get());
        if (goal)
            if (si_->isValid(goal->state) && si_->satisfiesBounds(goal->state))
                states.push_back(goal->state);
        GoalStates *goals = dynamic_cast<GoalStates*>(goal_.get());
        if (goals)
            for (unsigned int i = 0 ; i < goals->states.size() ; ++i)
                if (si_->isValid(goals->states[i]) && si_->satisfiesBounds(goals->states[i]))
                    states.push_back(goals->states[i]);

        if (states.empty())
        {
            unsigned int startIndex;
            if (isTrivial(&startIndex))
            {
                geometric::PathGeometric *pg = new geometric::PathGeometric(si_);
                pg->states.push_back(si_->cloneState(startStates_[startIndex]));
                pg->states.push_back(si_->cloneState(startStates_[startIndex]));
                path.reset(pg);
            }
        }
        else
        {
            for (unsigned int i = 0 ; i < startStates_.size() && !path ; ++i)
            {
                const State *start = startStates_[i];
                if (start && si_->isValid(start) && si_->satisfiesBounds(start))
                {
                    for (unsigned int j = 0 ; j < states.size() && !path ; ++j)
                        if (si_->checkMotion(start, states[j]))
                        {
                            geometric::PathGeometric *pg = new geometric::PathGeometric(si_);
                            pg->states.push_back(si_->cloneState(start));
                            pg->states.push_back(si_->cloneState(states[j]));
                            path.reset(pg);
                            break;
                        }
                }
            }
        }
    }

    return path;
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
