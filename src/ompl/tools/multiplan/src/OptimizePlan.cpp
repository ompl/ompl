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
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/base/OptimizationObjective.h"

void ompl::tools::OptimizePlan::addPlanner(const base::PlannerPtr &planner)
{
    if (planner && planner->getSpaceInformation().get() != getProblemDefinition()->getSpaceInformation().get())
        throw Exception("Planner instance does not match space information");
    planners_.push_back(planner);
}

void ompl::tools::OptimizePlan::addPlannerAllocator(const base::PlannerAllocator &pa)
{
    planners_.push_back(pa(getProblemDefinition()->getSpaceInformation()));
}

void ompl::tools::OptimizePlan::clearPlanners()
{
    planners_.clear();
}

ompl::base::PlannerStatus ompl::tools::OptimizePlan::solve(double solveTime, unsigned int maxSol, unsigned int nthreads)
{
    time::point end = time::now() + time::seconds(solveTime);
    unsigned int nt = std::min(nthreads, (unsigned int)planners_.size());
    OMPL_DEBUG("Using %u threads", nt);

    base::PlannerStatus result;
    unsigned int np = 0;
    const base::ProblemDefinitionPtr &pdef = getProblemDefinition();
    pp_.clearHybridizationPaths();

    while (time::now() < end)
    {
        pp_.clearPlanners();
        for (unsigned int i = 0; i < nt; ++i)
        {
            planners_[np]->clear();
            pp_.addPlanner(planners_[np]);
            np = (np + 1) % planners_.size();
        }
        base::PlannerStatus localResult = pp_.solve(std::max(time::seconds(end - time::now()), 0.0), true);
        if (localResult)
        {
            if (result != base::PlannerStatus::EXACT_SOLUTION)
                result = localResult;

            if (!pdef->hasOptimizationObjective())
            {
                OMPL_DEBUG("Terminating early since there is no optimization objective specified");
                break;
            }

            base::Cost obj_cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective());

            if (pdef->getOptimizationObjective()->isSatisfied(obj_cost))
            {
                OMPL_DEBUG("Terminating early since solution path satisfies the optimization objective");
                break;
            }
            if (pdef->getSolutionCount() >= maxSol)
            {
                OMPL_DEBUG("Terminating early since %u solutions were generated", maxSol);
                break;
            }
        }
    }

    // if we have more time, and we have a geometric path, we try to simplify it
    if (time::now() < end && result)
    {
        geometric::PathGeometric *p = dynamic_cast<geometric::PathGeometric *>(pdef->getSolutionPath().get());
        if (p)
        {
            geometric::PathSimplifier ps(getProblemDefinition()->getSpaceInformation());
            ps.simplify(*p, std::max(time::seconds(end - time::now()), 0.0));
        }
    }

    return result;
}
