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
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/TRRT.h"
#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"

ompl::base::PlannerPtr ompl::geometric::getDefaultPlanner(const base::GoalPtr &goal)
{
    base::PlannerPtr planner;
    if (!goal)
        throw Exception("Unable to allocate default planner for unspecified goal definition");

    // if we can sample the goal region, use a bi-directional planner
    if (goal->hasType(base::GOAL_SAMPLEABLE_REGION) && false ) // TODO: remove this false
    {
        // if we have a default projection
        if (goal->getSpaceInformation()->getStateSpace()->hasDefaultProjection())
        {
            logInform("Using LBKPIECE1 planner 1");
            planner = base::PlannerPtr(new LBKPIECE1(goal->getSpaceInformation()));
        }
        else
        {
            logInform("Using RRTConnect planner");
            planner = base::PlannerPtr(new RRTConnect(goal->getSpaceInformation()));
        }
    }
    // otherwise use a single-tree planner
    else
    {
        // if we have a default projection
        if (goal->getSpaceInformation()->getStateSpace()->hasDefaultProjection() && false) //TODO: remove this false
        {
            logInform("Using KPIECE1 planner 2");
            planner = base::PlannerPtr(new KPIECE1(goal->getSpaceInformation()));
        }
        else
        {
            logInform("Using TRRT planner");
            planner = base::PlannerPtr(new TRRT(goal->getSpaceInformation()));
        }
    }

    if (!planner)
        throw Exception("Unable to allocate default planner");

    return planner;
}

ompl::geometric::SimpleSetup::SimpleSetup(const base::StateSpacePtr &space) :
    configured_(false), planTime_(0.0), simplifyTime_(0.0), invalid_request_(false)
{
    si_.reset(new base::SpaceInformation(space));
    pdef_.reset(new base::ProblemDefinition(si_));
    psk_.reset(new PathSimplifier(si_));
    params_.include(si_->params());
}

void ompl::geometric::SimpleSetup::setup(void)
{
    if (!configured_ || !si_->isSetup() || !planner_->isSetup())
    {
        if (!si_->isSetup())
            si_->setup();
        if (!planner_)
        {
            if (pa_)
                planner_ = pa_(si_);
            if (!planner_)
            {
                logInform("No planner specified. Using default. (geometric)");
                planner_ = getDefaultPlanner(getGoal());
            }
        }
        planner_->setProblemDefinition(pdef_);
        if (!planner_->isSetup())
            planner_->setup();

        params_.clear();
        params_.include(si_->params());
        params_.include(planner_->params());
        configured_ = true;
    }
}

void ompl::geometric::SimpleSetup::clear(void)
{
    if (planner_)
        planner_->clear();
    if (pdef_)
        pdef_->clearSolutionPaths();
}

ompl::base::PlannerStatus ompl::geometric::SimpleSetup::solve(double time)
{
    setup();
    invalid_request_ = false;
    time::point start = time::now();
    base::PlannerStatus result = planner_->solve(time);
    planTime_ = time::seconds(time::now() - start);
    if (result)
        logInform("Solution found in %f seconds", planTime_);
    else
    {
        if (planTime_ < time)
            invalid_request_ = true;
        logInform("No solution found after %f seconds", planTime_);
    }
    return result;
}

ompl::base::PlannerStatus ompl::geometric::SimpleSetup::solve(const base::PlannerTerminationCondition &ptc)
{
    setup();
    invalid_request_ = false;
    time::point start = time::now();
    base::PlannerStatus result = planner_->solve(ptc);
    planTime_ = time::seconds(time::now() - start);
    if (result)
        logInform("Solution found in %f seconds", planTime_);
    else
    {
        if (!ptc())
            invalid_request_ = true;
        logInform("No solution found after %f seconds", planTime_);
    }
    return result;
}

void ompl::geometric::SimpleSetup::simplifySolution(const base::PlannerTerminationCondition &ptc)
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
        {
            time::point start = time::now();
            psk_->simplify(static_cast<PathGeometric&>(*p), ptc);
            simplifyTime_ = time::seconds(time::now() - start);
            logInform("Path simplification took %f seconds", simplifyTime_);
            return;
        }
    }
    logWarn("No solution to simplify");
}

void ompl::geometric::SimpleSetup::simplifySolution(double duration)
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
        {
            time::point start = time::now();
            if (duration < std::numeric_limits<double>::epsilon())
                psk_->simplifyMax(static_cast<PathGeometric&>(*p));
            else
                psk_->simplify(static_cast<PathGeometric&>(*p), duration);
            simplifyTime_ = time::seconds(time::now() - start);
            logInform("Path simplification took %f seconds", simplifyTime_);
            return;
        }
    }
    logWarn("No solution to simplify");
}

ompl::geometric::PathGeometric& ompl::geometric::SimpleSetup::getSolutionPath(void) const
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
            return static_cast<PathGeometric&>(*p);
    }
    throw Exception("No solution path");
}

bool ompl::geometric::SimpleSetup::haveExactSolutionPath(void) const
{
    return haveSolutionPath() && (!pdef_->hasApproximateSolution() || pdef_->getSolutionDifference() < std::numeric_limits<double>::epsilon());
}

void ompl::geometric::SimpleSetup::getPlannerData(base::PlannerData &pd) const
{
    pd.clear();
    if (planner_)
        planner_->getPlannerData(pd);
}

void ompl::geometric::SimpleSetup::print(std::ostream &out) const
{
    if (si_)
    {
        si_->printProperties(out);
        si_->printSettings(out);
    }
    if (planner_)
    {
        planner_->printProperties(out);
        planner_->printSettings(out);
    }
    if (pdef_)
        pdef_->print(out);
}
