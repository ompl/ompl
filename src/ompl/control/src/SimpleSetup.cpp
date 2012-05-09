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

#include "ompl/control/SimpleSetup.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/kpiece/KPIECE1.h"

ompl::base::PlannerPtr ompl::control::getDefaultPlanner(const base::GoalPtr &goal)
{
    base::PlannerPtr planner;
    if (!goal)
        throw Exception("Unable to allocate default planner for unspecified goal definition");

    SpaceInformationPtr si = boost::static_pointer_cast<SpaceInformation, base::SpaceInformation>(goal->getSpaceInformation());
    if (si->getStateSpace()->hasDefaultProjection())
        planner = base::PlannerPtr(new KPIECE1(si));
    else
        planner = base::PlannerPtr(new RRT(si));

    return planner;
}

ompl::control::SimpleSetup::SimpleSetup(const ControlSpacePtr &space) :
    configured_(false), planTime_(0.0), invalid_request_(false), msg_("SimpleSetup")
{
    si_.reset(new SpaceInformation(space->getStateSpace(), space));
    pdef_.reset(new base::ProblemDefinition(si_));
    params_.include(si_->params());
}

void ompl::control::SimpleSetup::setup(void)
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
                msg_.inform("No planner specified. Using default.");
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

void ompl::control::SimpleSetup::clear(void)
{
    if (planner_)
        planner_->clear();
    if (pdef_)
        pdef_->clearSolutionPaths();
}

ompl::base::PlannerStatus ompl::control::SimpleSetup::solve(double time)
{
    setup();
    invalid_request_ = false;
    time::point start = time::now();
    base::PlannerStatus result = planner_->solve(time);
    planTime_ = time::seconds(time::now() - start);
    if (result)
        msg_.inform("Solution found in %f seconds", planTime_);
    else
    {
        if (planTime_ < time)
            invalid_request_ = true;
        msg_.inform("No solution found after %f seconds", planTime_);
    }
    return result;
}

ompl::base::PlannerStatus ompl::control::SimpleSetup::solve(const base::PlannerTerminationCondition &ptc)
{
    setup();
    invalid_request_ = false;
    time::point start = time::now();
    base::PlannerStatus result = planner_->solve(ptc);
    planTime_ = time::seconds(time::now() - start);
    if (result)
        msg_.inform("Solution found in %f seconds", planTime_);
    else
    {
        if (!ptc())
            invalid_request_ = true;
        msg_.inform("No solution found after %f seconds", planTime_);
    }
    return result;
}

ompl::control::PathControl& ompl::control::SimpleSetup::getSolutionPath(void) const
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
            return static_cast<PathControl&>(*p);
    }
    throw Exception("No solution path");
}

bool ompl::control::SimpleSetup::haveExactSolutionPath(void) const
{
    return haveSolutionPath() && (!pdef_->isApproximate() || pdef_->getDifference() < std::numeric_limits<double>::epsilon());
}

void ompl::control::SimpleSetup::getPlannerData(base::PlannerData &pd) const
{
    pd.clear();
    if (planner_)
        planner_->getPlannerData(pd);
}

void ompl::control::SimpleSetup::print(std::ostream &out) const
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
