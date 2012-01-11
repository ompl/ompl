/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "ompl/tools/multiplan/OptimizePlan.h"

void ompl::OptimizePlan::addPlanner(const base::PlannerPtr &planner)
{
    if (planner && planner->getSpaceInformation().get() != getProblemDefinition()->getSpaceInformation().get())
        throw Exception("Planner instance does not match space information");
    planners_.push_back(planner);
}

void ompl::OptimizePlan::addPlannerAllocator(const base::PlannerAllocator &pa)
{
    planners_.push_back(pa(getProblemDefinition()->getSpaceInformation()));
}

void ompl::OptimizePlan::clearPlanners(void)
{
    planners_.clear();
}

bool ompl::OptimizePlan::solve(double solveTime, unsigned int nthreads)
{
    return solve(base::timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)), nthreads);
}

bool ompl::OptimizePlan::solve(const base::PlannerTerminationCondition &ptc, unsigned int nthreads)
{
    bool result = false;
    unsigned int np = 0;
    const base::GoalPtr &goal = getProblemDefinition()->getGoal();

    while (ptc() == false)
    {
        pp_.clearPlanners();
        for (unsigned int i = 0 ; i < nthreads ; ++i)
        {
            pp_.addPlanner(planners_[np]);
            np = (np + 1) % planners_.size();
        }
        if (pp_.solve(ptc, true))
        {
            result = true;
            if (goal->getSolutionPath()->length() <= goal->getMaximumPathLength())
                break;
        }
    }

    return result;
}
