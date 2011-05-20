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

#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"

ompl::base::PlannerPtr ompl::geometric::getDefaultPlanner(const base::GoalPtr &goal)
{
    base::PlannerPtr planner;
    if (!goal)
        throw Exception("Unable to allocate default planner for unspecified goal definition");

    // if we can sample the goal region, use a bi-directional planner
    if (dynamic_cast<const base::GoalSampleableRegion*>(goal.get()))
    {
        // if we have a default projection
        if (goal->getSpaceInformation()->getStateSpace()->hasDefaultProjection())
            planner = base::PlannerPtr(new LBKPIECE1(goal->getSpaceInformation()));
        else
            planner = base::PlannerPtr(new RRTConnect(goal->getSpaceInformation()));
    }
    // other use a single-tree planner
    else
    {
        // if we have a default projection
        if (goal->getSpaceInformation()->getStateSpace()->hasDefaultProjection())
            planner = base::PlannerPtr(new KPIECE1(goal->getSpaceInformation()));
        else
            planner = base::PlannerPtr(new RRT(goal->getSpaceInformation()));
    }

    if (!planner)
        throw Exception("Unable to allocate default planner");

    return planner;
}

void ompl::geometric::SimpleSetup::setup(void)
{
    if (!configured_)
    {
        if (!si_)
            throw Exception("No space information defined");

        if (!si_->isSetup())
            si_->setup();
        if (!planner_)
        {
            if (pa_)
                planner_ = pa_(si_);
            else
            {
                msg_.inform("No planner specified. Using default.");
                planner_ = getDefaultPlanner(getGoal());
            }
        }
        planner_->setProblemDefinition(pdef_);
        if (!planner_->isSetup())
            planner_->setup();
        configured_ = true;
    }
}

void ompl::geometric::SimpleSetup::clear(void)
{
    if (planner_)
        planner_->clear();
    if (pdef_ && pdef_->getGoal())
        pdef_->getGoal()->clearSolutionPath();
}

bool ompl::geometric::SimpleSetup::solve(double time)
{
    setup();
    time::point start = time::now();
    bool result = planner_->solve(time);
    planTime_ = time::seconds(time::now() - start);
    if (result)
        msg_.inform("Solution found in %f seconds", planTime_);
    else
        msg_.inform("No solution found after %f seconds", planTime_);
    return result;
}

void ompl::geometric::SimpleSetup::simplifySolution(void)
{
    if (pdef_ && pdef_->getGoal())
    {
        const base::PathPtr &p = pdef_->getGoal()->getSolutionPath();
        if (p)
        {
            time::point start = time::now();
            psk_->simplifyMax(static_cast<PathGeometric&>(*p));
            simplifyTime_ = time::seconds(time::now() - start);
            msg_.inform("Path simplification took %f seconds", simplifyTime_);
        }
        else
            msg_.warn("No solution to simplify");
    }
    else
        msg_.warn("No solution to simplify");
}

ompl::geometric::PathGeometric& ompl::geometric::SimpleSetup::getSolutionPath(void) const
{
    if (pdef_ && pdef_->getGoal())
    {
        const base::PathPtr &p = pdef_->getGoal()->getSolutionPath();
        if (p)
            return static_cast<PathGeometric&>(*p);
    }
    throw Exception("No solution path");
}

ompl::base::PlannerData ompl::geometric::SimpleSetup::getPlannerData(void) const
{
    base::PlannerData pd;
    if (planner_)
        planner_->getPlannerData(pd);
    return pd;
}

void ompl::geometric::SimpleSetup::updateProjectionCellSizes(void)
{
    const std::vector<const base::State*> &states = getPlannerData().states;
    if (states.empty())
        msg_.warn("There are no states in the exploration data structure of the planner");
    else
    {
        const std::map<std::string, base::ProjectionEvaluatorPtr> &prj = getStateSpace()->getRegisteredProjections();
        for (std::map<std::string, base::ProjectionEvaluatorPtr>::const_iterator it = prj.begin() ; it != prj.end() ; ++it)
            it->second->computeCellSizes(states);
    }
}
