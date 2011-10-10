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

#include "ompl/tools/multiplan/ParallelPlan.h"
#include "ompl/geometric/PathHybridization.h"

ompl::ParallelPlan::ParallelPlan(const base::ProblemDefinitionPtr &pdef) :
    pdef_(pdef), hybridize_(false), minSolCount_(2), maxSolCount_(5),
    phybrid_(new geometric::PathHybridization(pdef->getSpaceInformation())), msg_("ParallelPlan")
{
}

ompl::ParallelPlan::~ParallelPlan(void)
{
}

void ompl::ParallelPlan::addPlanner(const base::PlannerPtr &planner)
{
    if (planner && planner->getSpaceInformation().get() != pdef_->getSpaceInformation().get())
        throw Exception("Planner instance does not match space information");
    if (planner->getProblemDefinition().get() != pdef_.get())
        planner->setProblemDefinition(pdef_);
    planners_.push_back(planner);
}

void ompl::ParallelPlan::addPlannerAllocator(const base::PlannerAllocator &pa)
{
    base::PlannerPtr planner = pa(pdef_->getSpaceInformation());
    planner->setProblemDefinition(pdef_);
    planners_.push_back(planner);
}

void ompl::ParallelPlan::clearPlanners(void)
{
    planners_.clear();
}

void ompl::ParallelPlan::setHybridization(bool flag)
{
    hybridize_ = flag;
}

bool ompl::ParallelPlan::solve(double solveTime)
{
    return solve(base::timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)));
}

bool ompl::ParallelPlan::solve(const base::PlannerTerminationCondition &ptc)
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
        if (phybrid_->pathCount() > 1)
            if (const base::PathPtr &hsol = phybrid_->getHybridPath())
            {
                geometric::PathGeometric *pg = static_cast<geometric::PathGeometric*>(hsol.get());
                double difference = 0.0;
                bool approximate = !pdef_->getGoal()->isSatisfied(pg->states.back(), &difference);
                pdef_->getGoal()->addSolutionPath(hsol, approximate, difference);
            }
    }

    msg_.inform("Solution found in %f seconds", time::seconds(time::now() - start));

    return pdef_->getGoal()->isAchieved();
}

void ompl::ParallelPlan::solveOne(base::Planner *planner, const base::PlannerTerminationCondition *ptc)
{
    msg_.debug("Starting " + planner->getName());
    if (planner->solve(*ptc))
    {
        ptc->terminate();
        msg_.debug("Solution found by " + planner->getName());
    }
}

void ompl::ParallelPlan::solveMore(base::Planner *planner, const base::PlannerTerminationCondition *ptc)
{
    if (planner->solve(*ptc))
    {
        const std::vector<base::PlannerSolution> &paths = pdef_->getGoal()->getSolutions();

        if (phybrid_->pathCount() >= maxSolCount_)
            ptc->terminate();

        msg_.debug("Solution found by " + planner->getName());

        boost::mutex::scoped_lock slock(phlock_);

        for (std::size_t i = 0 ; i < paths.size() ; ++i)
            phybrid_->recordPath(paths[i].path_);

        if (phybrid_->pathCount() >= minSolCount_)
            phybrid_->computeHybridPath();
    }
}
