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
#include <thread>

ompl::tools::ParallelPlan::ParallelPlan(const base::ProblemDefinitionPtr &pdef)
  : pdef_(pdef), phybrid_(std::make_shared<geometric::PathHybridization>(pdef->getSpaceInformation()))
{
}

ompl::tools::ParallelPlan::~ParallelPlan() = default;

void ompl::tools::ParallelPlan::addPlanner(const base::PlannerPtr &planner)
{
    if (planner && planner->getSpaceInformation().get() != pdef_->getSpaceInformation().get())
        throw Exception("Planner instance does not match space information");
    if (planner->getProblemDefinition().get() != pdef_.get())
        planner->setProblemDefinition(pdef_);
    planners_.push_back(planner);
}

void ompl::tools::ParallelPlan::addPlannerAllocator(const base::PlannerAllocator &pa)
{
    base::PlannerPtr planner = pa(pdef_->getSpaceInformation());
    planner->setProblemDefinition(pdef_);
    planners_.push_back(planner);
}

void ompl::tools::ParallelPlan::clearPlanners()
{
    planners_.clear();
}

void ompl::tools::ParallelPlan::clearHybridizationPaths()
{
    phybrid_->clear();
}

ompl::base::PlannerStatus ompl::tools::ParallelPlan::solve(double solveTime, bool hybridize)
{
    return solve(solveTime, 1, planners_.size(), hybridize);
}

ompl::base::PlannerStatus ompl::tools::ParallelPlan::solve(double solveTime, std::size_t minSolCount,
                                                           std::size_t maxSolCount, bool hybridize)
{
    return solve(base::timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1)), minSolCount,
                 maxSolCount, hybridize);
}

ompl::base::PlannerStatus ompl::tools::ParallelPlan::solve(const base::PlannerTerminationCondition &ptc, bool hybridize)
{
    return solve(ptc, 1, planners_.size(), hybridize);
}

ompl::base::PlannerStatus ompl::tools::ParallelPlan::solve(const base::PlannerTerminationCondition &ptc,
                                                           std::size_t minSolCount, std::size_t maxSolCount,
                                                           bool hybridize)
{
    if (!pdef_->getSpaceInformation()->isSetup())
        pdef_->getSpaceInformation()->setup();
    foundSolCount_ = 0;

    time::point start = time::now();
    std::vector<std::thread *> threads(planners_.size());

    // Decide if we are combining solutions or just taking the first one
    if (hybridize)
        for (std::size_t i = 0; i < threads.size(); ++i)
            threads[i] = new std::thread([this, i, minSolCount, maxSolCount, &ptc]
                                         {
                                             solveMore(planners_[i].get(), minSolCount, maxSolCount, &ptc);
                                         });
    else
        for (std::size_t i = 0; i < threads.size(); ++i)
            threads[i] = new std::thread([this, i, minSolCount, &ptc]
                                         {
                                             solveOne(planners_[i].get(), minSolCount, &ptc);
                                         });

    for (auto &thread : threads)
    {
        thread->join();
        delete thread;
    }

    if (hybridize)
    {
        if (phybrid_->pathCount() > 1)
            if (const geometric::PathGeometricPtr &hsol = phybrid_->getHybridPath())
            {
                double difference = 0.0;
                bool approximate = !pdef_->getGoal()->isSatisfied(hsol->getStates().back(), &difference);
                pdef_->addSolutionPath(hsol, approximate, difference,
                                       phybrid_->getName());  // name this solution after the hybridization algorithm
            }
    }

    if (pdef_->hasSolution())
        OMPL_INFORM("ParallelPlan::solve(): Solution found by one or more threads in %f seconds",
                    time::seconds(time::now() - start));
    else
        OMPL_WARN("ParallelPlan::solve(): Unable to find solution by any of the threads in %f seconds",
                  time::seconds(time::now() - start));

    return {pdef_->hasSolution(), pdef_->hasApproximateSolution()};
}

void ompl::tools::ParallelPlan::solveOne(base::Planner *planner, std::size_t minSolCount,
                                         const base::PlannerTerminationCondition *ptc)
{
    OMPL_DEBUG("ParallelPlan.solveOne starting planner %s", planner->getName().c_str());

    time::point start = time::now();
    if (planner->solve(*ptc))
    {
        double duration = time::seconds(time::now() - start);
        foundSolCountLock_.lock();
        unsigned int nrSol = ++foundSolCount_;
        foundSolCountLock_.unlock();
        if (nrSol >= minSolCount)
            ptc->terminate();
        OMPL_DEBUG("ParallelPlan.solveOne: Solution found by %s in %lf seconds", planner->getName().c_str(), duration);
    }
}

void ompl::tools::ParallelPlan::solveMore(base::Planner *planner, std::size_t minSolCount, std::size_t maxSolCount,
                                          const base::PlannerTerminationCondition *ptc)
{
    OMPL_DEBUG("ParallelPlan.solveMore: starting planner %s", planner->getName().c_str());

    time::point start = time::now();
    if (planner->solve(*ptc))
    {
        double duration = time::seconds(time::now() - start);
        foundSolCountLock_.lock();
        unsigned int nrSol = ++foundSolCount_;
        foundSolCountLock_.unlock();

        if (nrSol >= maxSolCount)
            ptc->terminate();

        OMPL_DEBUG("ParallelPlan.solveMore: Solution found by %s in %lf seconds", planner->getName().c_str(), duration);

        const std::vector<base::PlannerSolution> &paths = pdef_->getSolutions();

        std::lock_guard<std::mutex> slock(phlock_);
        start = time::now();
        unsigned int attempts = 0;
        for (const auto &path : paths)
            attempts += phybrid_->recordPath(std::static_pointer_cast<geometric::PathGeometric>(path.path_), false);

        if (phybrid_->pathCount() >= minSolCount)
            phybrid_->computeHybridPath();

        duration = time::seconds(time::now() - start);
        OMPL_DEBUG("ParallelPlan.solveMore: Spent %f seconds hybridizing %u solution paths (attempted %u connections "
                   "between paths)",
                   duration, (unsigned int)phybrid_->pathCount(), attempts);
    }
}
