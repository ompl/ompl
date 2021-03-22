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
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::SimpleSetup::SimpleSetup(const base::SpaceInformationPtr &si)
  : configured_(false), planTime_(0.0), simplifyTime_(0.0), lastStatus_(base::PlannerStatus::UNKNOWN)
{
    si_ = si;
    pdef_ = std::make_shared<base::ProblemDefinition>(si_);
}

ompl::geometric::SimpleSetup::SimpleSetup(const base::StateSpacePtr &space)
  : configured_(false), planTime_(0.0), simplifyTime_(0.0), lastStatus_(base::PlannerStatus::UNKNOWN)
{
    si_ = std::make_shared<base::SpaceInformation>(space);
    pdef_ = std::make_shared<base::ProblemDefinition>(si_);
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
                planner_ = tools::SelfConfig::getDefaultPlanner(getGoal());
            }
        }
        planner_->setProblemDefinition(pdef_);
        if (!planner_->isSetup())
            planner_->setup();
        psk_ = std::make_shared<PathSimplifier>(si_, pdef_->getGoal(), pdef_->getOptimizationObjective());
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

void ompl::geometric::SimpleSetup::setStartAndGoalStates(const base::ScopedState<> &start,
                                                         const base::ScopedState<> &goal, const double threshold)
{
    pdef_->setStartAndGoalStates(start, goal, threshold);

    // Clear any past solutions since they no longer correspond to our start and goal states
    pdef_->clearSolutionPaths();

    // force setup to rerun
    configured_ = false;
}

void ompl::geometric::SimpleSetup::setGoalState(const base::ScopedState<> &goal, const double threshold)
{
    pdef_->setGoalState(goal, threshold);

    // force setup to rerun 
    configured_ = false;
}

/** \brief Set the goal for planning. This call is not
    needed if setStartAndGoalStates() has been called. */
void ompl::geometric::SimpleSetup::setGoal(const base::GoalPtr &goal)
{
    pdef_->setGoal(goal);

    if (goal && goal->hasType(base::GOAL_SAMPLEABLE_REGION))
        psk_ = std::make_shared<PathSimplifier>(si_, pdef_->getGoal());
    else
        psk_ = std::make_shared<PathSimplifier>(si_);

    // force setup to rerun
    configured_ = false;
}

// we provide a duplicate implementation here to allow the planner to choose how the time is turned into a planner
// termination condition
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
            auto &path = static_cast<PathGeometric &>(*p);
            std::size_t numStates = path.getStateCount();
            psk_->simplify(path, ptc);
            simplifyTime_ = time::seconds(time::now() - start);
            OMPL_INFORM("SimpleSetup: Path simplification took %f seconds and changed from %d to %d states",
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
            auto &path = static_cast<PathGeometric &>(*p);
            std::size_t numStates = path.getStateCount();
            if (duration < std::numeric_limits<double>::epsilon())
                psk_->simplifyMax(static_cast<PathGeometric &>(*p));
            else
                psk_->simplify(static_cast<PathGeometric &>(*p), duration);
            simplifyTime_ = time::seconds(time::now() - start);
            OMPL_INFORM("SimpleSetup: Path simplification took %f seconds and changed from %d to %d states",
                        simplifyTime_, numStates, path.getStateCount());
            return;
        }
    }
    OMPL_WARN("No solution to simplify");
}

const std::string ompl::geometric::SimpleSetup::getSolutionPlannerName() const
{
    if (pdef_)
    {
        const ompl::base::PathPtr path;              // convert to a generic path ptr
        ompl::base::PlannerSolution solution(path);  // a dummy solution

        // Get our desired solution
        pdef_->getSolution(solution);
        return solution.plannerName_;
    }
    throw Exception("No problem definition found");
}

ompl::geometric::PathGeometric &ompl::geometric::SimpleSetup::getSolutionPath() const
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
            return static_cast<PathGeometric &>(*p);
    }
    throw Exception("No solution path");
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
