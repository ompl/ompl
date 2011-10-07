/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#include "ompl/tools/parallelplan/ParallelPlan.h"
#include <boost/thread.hpp>

bool ompl::geometric::ParallelPlan::solve(double solveTime)
{
    return solve(base::timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)));
}

bool ompl::geometric::ParallelPlan::solve(const base::PlannerTerminationCondition &ptc)
{
    if (!pdef_->getSpaceInformation()->isSetup())
        pdef_->getSpaceInformation()->setup();
    pdef_->getGoal()->clearSolutionPaths();

    time::point start = time::now();
    std::vector<boost::thread*> threads(planners_.size());
    if (hybridize_)
        for (std::size_t i = 0 ; i < threads.size() ; ++i)
            threads[i] = new boost::thread(boost::bind(&ParallelPlan::solveMore, this, planners_[i].get(), &ptc));
    else
        for (std::size_t i = 0 ; i < threads.size() ; ++i)
            threads[i] = new boost::thread(boost::bind(&ParallelPlan::solveOne, this, planners_[i].get(), &ptc));

    for (std::size_t i = 0 ; i < threads.size() ; ++i)
    {
        threads[i]->join();
        delete threads[i];
    }

    if (hybridize_)
    {
        // add the best solution to the goal
        // pdef_->getGoal()->addSolutionPath(...);
    }

    msg_.inform("Solution found in %f seconds", time::seconds(time::now() - start));

    return pdef_->getGoal()->isAchieved();
}

void ompl::geometric::ParallelPlan::solveOne(base::Planner *planner, const base::PlannerTerminationCondition *ptc)
{
    if (planner->solve(*ptc))
    {
        ptc->terminate();
        msg_.debug("Solution found by " + planner->getName());
    }
}

void ompl::geometric::ParallelPlan::solveMore(base::Planner *planner, const base::PlannerTerminationCondition *ptc)
{
    if (planner->solve(*ptc))
    {
        std::size_t solCount = pdef_->getGoal()->getSolutionCount();
        if (solCount >= minSolCount_)
            hybridizeSolutions(*ptc);
        if (solCount >= maxSolCount_)
            ptc->terminate();
    }
}

void ompl::geometric::ParallelPlan::hybridizeSolutions(const base::PlannerTerminationCondition &ptc)
{
    // lock for hybridization

    // add to H-graph

}
