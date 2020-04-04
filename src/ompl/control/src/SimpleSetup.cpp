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
#include "ompl/tools/config/SelfConfig.h"

ompl::control::SimpleSetup::SimpleSetup(const SpaceInformationPtr &si)
  : configured_(false), planTime_(0.0), last_status_(base::PlannerStatus::UNKNOWN)
{
    si_ = si;
    pdef_ = std::make_shared<base::ProblemDefinition>(si_);
}

ompl::control::SimpleSetup::SimpleSetup(const ControlSpacePtr &space)
  : configured_(false), planTime_(0.0), last_status_(base::PlannerStatus::UNKNOWN)
{
    si_ = std::make_shared<SpaceInformation>(space->getStateSpace(), space);
    pdef_ = std::make_shared<base::ProblemDefinition>(si_);
}

void ompl::control::SimpleSetup::setup()
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

        configured_ = true;
    }
}

void ompl::control::SimpleSetup::clear()
{
    if (planner_)
        planner_->clear();
    if (pdef_)
        pdef_->clearSolutionPaths();
}

// we provide a duplicate implementation here to allow the planner to choose how the time is turned into a planner
// termination condition
ompl::base::PlannerStatus ompl::control::SimpleSetup::solve(double time)
{
    setup();
    last_status_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();
    last_status_ = planner_->solve(time);
    planTime_ = time::seconds(time::now() - start);
    if (last_status_)
        OMPL_INFORM("Solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("No solution found after %f seconds", planTime_);
    return last_status_;
}

ompl::base::PlannerStatus ompl::control::SimpleSetup::solve(const base::PlannerTerminationCondition &ptc)
{
    setup();
    last_status_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();
    last_status_ = planner_->solve(ptc);
    planTime_ = time::seconds(time::now() - start);
    if (last_status_)
        OMPL_INFORM("Solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("No solution found after %f seconds", planTime_);
    return last_status_;
}

ompl::control::PathControl &ompl::control::SimpleSetup::getSolutionPath() const
{
    if (pdef_)
    {
        const base::PathPtr &p = pdef_->getSolutionPath();
        if (p)
            return static_cast<PathControl &>(*p);
    }
    throw Exception("No solution path");
}

bool ompl::control::SimpleSetup::haveExactSolutionPath() const
{
    return haveSolutionPath() && (!pdef_->hasApproximateSolution() ||
                                  pdef_->getSolutionDifference() < std::numeric_limits<double>::epsilon());
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
