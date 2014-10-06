/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Ryan Luna */

#include "ompl/geometric/planners/AnytimePathShortening.h"
#include "ompl/geometric/PathHybridization.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

#include <boost/thread.hpp>

ompl::geometric::AnytimePathShortening::AnytimePathShortening (const ompl::base::SpaceInformationPtr &si) :
    ompl::base::Planner(si, "APS"),
    shortcut_(true),
    hybridize_(true),
    maxHybridPaths_(24),
    defaultNumPlanners_(std::max(1u, boost::thread::hardware_concurrency()))
{
    specs_.approximateSolutions = true;
    specs_.multithreaded = true;
    specs_.optimizingPaths = true;

    Planner::declareParam<bool>("shortcut", this, &AnytimePathShortening::setShortcut, &AnytimePathShortening::isShortcutting, "0,1");
    Planner::declareParam<bool>("hybridize", this, &AnytimePathShortening::setHybridize, &AnytimePathShortening::isHybridizing, "0,1");
    Planner::declareParam<unsigned int>("max_hybrid_paths", this, &AnytimePathShortening::setMaxHybridizationPath, &AnytimePathShortening::maxHybridizationPaths, "0:1:50");
    Planner::declareParam<unsigned int>("num_planners", this, &AnytimePathShortening::setDefaultNumPlanners, &AnytimePathShortening::getDefaultNumPlanners, "0:64");

    addPlannerProgressProperty("best cost REAL",
                               boost::bind(&AnytimePathShortening::getBestCost, this));
}

ompl::geometric::AnytimePathShortening::~AnytimePathShortening()
{
}

void ompl::geometric::AnytimePathShortening::addPlanner(base::PlannerPtr &planner)
{
    if (planner && planner->getSpaceInformation().get() != si_.get())
    {
        OMPL_ERROR("NOT adding planner %s: SpaceInformation instances are different", planner->getName().c_str());
        return;
    }

    // Ensure all planners are unique instances
    for(size_t i = 0; i < planners_.size(); ++i)
    {
        if (planner.get() == planners_[i].get())
        {
            OMPL_ERROR("NOT adding planner %s: Planner instances MUST be unique", planner->getName().c_str());
            return;
        }
    }

    planners_.push_back(planner);
}

void ompl::geometric::AnytimePathShortening::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
    ompl::base::Planner::setProblemDefinition(pdef);
    for (size_t i = 0; i < planners_.size(); ++i)
        planners_[i]->setProblemDefinition(pdef);
}

ompl::base::PlannerStatus ompl::geometric::AnytimePathShortening::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    base::Goal *goal = pdef_->getGoal().get();
    std::vector<boost::thread*> threads(planners_.size());
    geometric::PathHybridization phybrid(si_);
    base::Path *bestSln = NULL;

    base::OptimizationObjectivePtr opt = pdef_->getOptimizationObjective();
    if (!opt)
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
        opt.reset(new base::PathLengthOptimizationObjective(si_));
        pdef_->setOptimizationObjective(opt);
    }
    else
    {
        if (!dynamic_cast<base::PathLengthOptimizationObjective*>(opt.get()))
            OMPL_WARN("The optimization objective is not set for path length.  The specified optimization criteria may not be optimized over.");
    }

    // Disable output from the motion planners, except for errors
    msg::LogLevel currentLogLevel = msg::getLogLevel();
    msg::setLogLevel(std::max(msg::LOG_ERROR, currentLogLevel));
    while (!ptc())
    {
        // We have found a solution that is good enough
        if (bestSln && opt->isSatisfied(base::Cost(bestSln->length())))
            break;

        // Clear any previous planning data for the set of planners
        clear();

        // Spawn a thread for each planner.  This will shortcut the best path after solving.
        for (size_t i = 0; i < threads.size(); ++i)
            threads[i] = new boost::thread(boost::bind(&AnytimePathShortening::threadSolve, this, planners_[i].get(), ptc));

        // Join each thread, and then delete it
        for (std::size_t i = 0 ; i < threads.size() ; ++i)
        {
            threads[i]->join();
            delete threads[i];
        }

        // Hybridize the set of paths computed.  Add the new hybrid path to the mix.
        if (hybridize_ && !ptc())
        {
            const std::vector<base::PlannerSolution> &paths = pdef_->getSolutions();
            for (size_t j = 0; j < paths.size() && j < maxHybridPaths_ && !ptc(); ++j)
                phybrid.recordPath(paths[j].path_, false);

            phybrid.computeHybridPath();
            const base::PathPtr &hsol = phybrid.getHybridPath();
            if (hsol)
            {
                geometric::PathGeometric *pg = static_cast<geometric::PathGeometric*>(hsol.get());
                double difference = 0.0;
                bool approximate = !goal->isSatisfied(pg->getStates().back(), &difference);
                pdef_->addSolutionPath(hsol, approximate, difference);
            }
        }

        // keep track of the best solution found so far
        if (pdef_->getSolutionCount() > 0)
            bestSln = pdef_->getSolutionPath().get();
    }
    msg::setLogLevel(currentLogLevel);

    if (bestSln)
    {
        if (goal->isSatisfied (static_cast<geometric::PathGeometric*>(bestSln)->getStates().back()))
            return base::PlannerStatus::EXACT_SOLUTION;
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }
    return base::PlannerStatus::UNKNOWN;
}

void ompl::geometric::AnytimePathShortening::threadSolve(base::Planner* planner, const base::PlannerTerminationCondition &ptc)
{
    // compute a motion plan
    base::PlannerStatus status = planner->solve(ptc);

    // Shortcut the best solution found so far
    if (shortcut_ && status == base::PlannerStatus::EXACT_SOLUTION)
    {
        geometric::PathGeometric* sln = static_cast<geometric::PathGeometric*>(pdef_->getSolutionPath().get());
        geometric::PathGeometric* pathCopy = new geometric::PathGeometric(*sln);
        geometric::PathSimplifier ps(pdef_->getSpaceInformation());
        if (ps.shortcutPath(*pathCopy))
        {
            double difference = 0.0;
            bool approximate = !pdef_->getGoal()->isSatisfied(pathCopy->getStates().back(), &difference);
            pdef_->addSolutionPath(base::PathPtr(pathCopy), approximate, difference);
        }
        else delete pathCopy;
    }
}

void ompl::geometric::AnytimePathShortening::clear(void)
{
    Planner::clear();
    for (size_t i = 0; i < planners_.size(); ++i)
        planners_[i]->clear();
}

void ompl::geometric::AnytimePathShortening::getPlannerData(ompl::base::PlannerData &data) const
{
    if (planners_.size() == 0)
        return;

    OMPL_WARN("Returning planner data for planner #0");
    getPlannerData(data, 0);
}

void ompl::geometric::AnytimePathShortening::getPlannerData(ompl::base::PlannerData &data, unsigned int idx) const
{
    if(planners_.size() < idx)
        return;
    planners_[idx]->getPlannerData(data);
}

void ompl::geometric::AnytimePathShortening::setup(void)
{
    Planner::setup();

    if (planners_.size() == 0)
    {
        planners_.reserve(defaultNumPlanners_);
        for (unsigned int i = 0; i < defaultNumPlanners_; ++i)
        {
            planners_.push_back(tools::SelfConfig::getDefaultPlanner(pdef_->getGoal()));
            planners_.back()->setProblemDefinition(pdef_);
        }
        OMPL_INFORM("%s: No planners specified; using %u instances of %s",
            getName().c_str(), planners_.size(), planners_[0]->getName().c_str());
    }

    for (size_t i = 0; i < planners_.size(); ++i)
        planners_[i]->setup();
}

void ompl::geometric::AnytimePathShortening::checkValidity(void)
{
    for (size_t i = 0; i < planners_.size(); ++i)
        planners_[i]->checkValidity();
}

unsigned int ompl::geometric::AnytimePathShortening::getNumPlanners(void) const
{
    return planners_.size();
}

ompl::base::PlannerPtr ompl::geometric::AnytimePathShortening::getPlanner(unsigned int idx) const
{
    assert(idx < planners_.size());
    return planners_[idx];
}

bool ompl::geometric::AnytimePathShortening::isShortcutting(void) const
{
    return shortcut_;
}

void ompl::geometric::AnytimePathShortening::setShortcut(bool shortcut)
{
    shortcut_ = shortcut;
}

bool ompl::geometric::AnytimePathShortening::isHybridizing(void) const
{
    return hybridize_;
}

void ompl::geometric::AnytimePathShortening::setHybridize(bool hybridize)
{
    hybridize_ = hybridize;
}

unsigned int ompl::geometric::AnytimePathShortening::maxHybridizationPaths(void) const
{
    return maxHybridPaths_;
}

void ompl::geometric::AnytimePathShortening::setMaxHybridizationPath(unsigned int maxPathCount)
{
    maxHybridPaths_ = maxPathCount;
}

void ompl::geometric::AnytimePathShortening::setDefaultNumPlanners(unsigned int numPlanners)
{
    defaultNumPlanners_ = numPlanners;
}

unsigned int ompl::geometric::AnytimePathShortening::getDefaultNumPlanners() const
{
    return defaultNumPlanners_;
}

std::string ompl::geometric::AnytimePathShortening::getBestCost() const
{
    base::Cost bestCost(std::numeric_limits<double>::quiet_NaN());
    if (pdef_ && pdef_->getSolutionCount() > 0)
        bestCost = base::Cost(pdef_->getSolutionPath()->length());
    return boost::lexical_cast<std::string>(bestCost);
}
