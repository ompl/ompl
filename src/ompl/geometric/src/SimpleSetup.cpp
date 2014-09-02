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
#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"

ompl::base::PlannerPtr ompl::geometric::getDefaultPlanner(const base::GoalPtr &goal)
{
    base::PlannerPtr planner;
    if (!goal)
        throw Exception("Unable to allocate default planner for unspecified goal definition");

    // if we can sample the goal region, use a bi-directional planner
    if (goal->hasType(base::GOAL_SAMPLEABLE_REGION))
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

ompl::geometric::SimpleSetup::SimpleSetup(const base::SpaceInformationPtr &si) :
    configured_(false), planTime_(0.0), simplifyTime_(0.0), lastStatus_(base::PlannerStatus::UNKNOWN)
{
    si_ = si;
    pdef_.reset(new base::ProblemDefinition(si_));
    psk_.reset(new PathSimplifier(si_));
    params_.include(si_->params());
}

ompl::geometric::SimpleSetup::SimpleSetup(const base::StateSpacePtr &space) :
    configured_(false), planTime_(0.0), simplifyTime_(0.0), lastStatus_(base::PlannerStatus::UNKNOWN)
{
    si_.reset(new base::SpaceInformation(space));
    pdef_.reset(new base::ProblemDefinition(si_));
    psk_.reset(new PathSimplifier(si_));
    params_.include(si_->params());
}

void ompl::geometric::SimpleSetup::setup()
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
                OMPL_INFORM("No planner specified. Using default.");
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

void ompl::geometric::SimpleSetup::clear()
{
    if (planner_)
        planner_->clear();
    if (pdef_)
        pdef_->clearSolutionPaths();
}

// we provide a duplicate implementation here to allow the planner to choose how the time is turned into a planner termination condition
ompl::base::PlannerStatus ompl::geometric::SimpleSetup::solve(double time)
{
    setup();
    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();
    lastStatus_ = planner_->solve(time);
    planTime_ = time::seconds(time::now() - start);
    if (lastStatus_)
        OMPL_INFORM("Solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("No solution found after %f seconds", planTime_);
    return lastStatus_;
}

ompl::base::PlannerStatus ompl::geometric::SimpleSetup::solve(const base::PlannerTerminationCondition &ptc)
{
    setup();
    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();
    lastStatus_ = planner_->solve(ptc);
    planTime_ = time::seconds(time::now() - start);
    if (lastStatus_)
        OMPL_INFORM("Solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("No solution found after %f seconds", planTime_);
    return lastStatus_;
}

void ompl::geometric::SimpleSetup::simplifySolution(const base::PlannerTerminationCondition &ptc)
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
        {
            time::point start = time::now();
            PathGeometric &path = static_cast<PathGeometric&>(*p);
            std::size_t numStates = path.getStateCount();
            psk_->simplify(path, ptc);
            simplifyTime_ = time::seconds(time::now() - start);
            OMPL_INFORM("Path simplification took %f seconds and changed from %d to %d states",
                        simplifyTime_, numStates, path.getStateCount());
            return;
        }
    }
    OMPL_WARN("No solution to simplify");
}

void ompl::geometric::SimpleSetup::simplifySolution(double duration)
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
        {
            time::point start = time::now();
            PathGeometric &path = static_cast<PathGeometric&>(*p);
            std::size_t numStates = path.getStateCount();
            if (duration < std::numeric_limits<double>::epsilon())
                psk_->simplifyMax(static_cast<PathGeometric&>(*p));
            else
                psk_->simplify(static_cast<PathGeometric&>(*p), duration);
            simplifyTime_ = time::seconds(time::now() - start);
            OMPL_INFORM("Path simplification took %f seconds and changed from %d to %d states",
                        simplifyTime_, numStates, path.getStateCount());
            return;
        }
    }
    OMPL_WARN("No solution to simplify");
}

const std::string ompl::geometric::SimpleSetup::getSolutionPlannerName(void) const
{
    if (pdef_)
    {
        const ompl::base::PathPtr path; // convert to a generic path ptr
        ompl::base::PlannerSolution solution(path); // a dummy solution

        // Get our desired solution
        pdef_->getSolution(solution);
        return solution.plannerName_;
    }
    throw Exception("No problem definition found");
}

ompl::geometric::PathGeometric& ompl::geometric::SimpleSetup::getSolutionPath() const
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
            return static_cast<PathGeometric&>(*p);
    }
    throw Exception("No solution path");
}

bool ompl::geometric::SimpleSetup::haveExactSolutionPath() const
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
