/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman */

#include "ompl/geometric/ParallelSetup.h"

ompl::geometric::ParallelSetup::ParallelSetup(const base::SpaceInformationPtr &si)
  : SimpleSetup(si)
{
    // Create the parallel component for splitting into two threads
    pp_ = std::make_shared<ompl::tools::ParallelPlan>(pdef_);
}

ompl::geometric::ParallelSetup::ParallelSetup(const base::StateSpacePtr &space)
  : SimpleSetup(space)
{
    // Create the parallel component for splitting into two threads
    pp_ = std::make_shared<ompl::tools::ParallelPlan>(pdef_);
}

// we provide a duplicate implementation here to allow the planner to choose how the time is turned into a planner
// termination condition
ompl::base::PlannerStatus ompl::geometric::ParallelSetup::solve(double time)
{
    setup();
    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();
    bool hybridize = false;
    lastStatus_ = pp_->solve(time, hybridize);
    planTime_ = time::seconds(time::now() - start);
    if (lastStatus_)
        OMPL_INFORM("Solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("No solution found after %f seconds", planTime_);
    return lastStatus_;
}

ompl::base::PlannerStatus ompl::geometric::ParallelSetup::solve(const base::PlannerTerminationCondition &ptc)
{
    setup();
    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();
    bool hybridize = false;
    lastStatus_ = pp_->solve(ptc, hybridize);
    planTime_ = time::seconds(time::now() - start);
    if (lastStatus_)
        OMPL_INFORM("Solution found in %f seconds", planTime_);
    else
        OMPL_INFORM("No solution found after %f seconds", planTime_);
    return lastStatus_;
}

void ompl::geometric::ParallelSetup::clear()
{
    pp_->clear();
    pp_->clearHybridizationPaths();
    if (pdef_)
        pdef_->clearSolutionPaths();
}

void ompl::geometric::ParallelSetup::setup()
{
    if (!configured_ || !si_->isSetup())
    {
        if (!si_->isSetup())
            si_->setup();
        configured_ = true;
    }
}

void ompl::geometric::ParallelSetup::print(std::ostream &out) const
{
    if (si_)
    {
        si_->printProperties(out);
        si_->printSettings(out);
    }
    if (pdef_)
        pdef_->print(out);
}
