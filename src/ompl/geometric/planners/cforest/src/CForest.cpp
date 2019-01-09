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

/* Authors: Javier V. Gómez, Ioan Sucan, Mark Moll */

#include "ompl/geometric/planners/cforest/CForest.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/util/String.h"
#include <thread>

ompl::geometric::CForest::CForest(const base::SpaceInformationPtr &si) : base::Planner(si, "CForest")
{
    specs_.optimizingPaths = true;
    specs_.multithreaded = true;

    numThreads_ = std::max(std::thread::hardware_concurrency(), 2u);
    Planner::declareParam<bool>("focus_search", this, &CForest::setFocusSearch, &CForest::getFocusSearch, "0,1");
    Planner::declareParam<unsigned int>("num_threads", this, &CForest::setNumThreads, &CForest::getNumThreads, "0:64");

    addPlannerProgressProperty("best cost REAL", [this]
                               {
                                   return getBestCost();
                               });
    addPlannerProgressProperty("shared paths INTEGER", [this]
                               {
                                   return getNumPathsShared();
                               });
    addPlannerProgressProperty("shared states INTEGER", [this]
                               {
                                   return getNumStatesShared();
                               });
}

ompl::geometric::CForest::~CForest() = default;

void ompl::geometric::CForest::setNumThreads(unsigned int numThreads)
{
    numThreads_ = numThreads != 0u ? numThreads : std::max(std::thread::hardware_concurrency(), 2u);
}

void ompl::geometric::CForest::addPlannerInstanceInternal(const base::PlannerPtr &planner)
{
    if (!planner->getSpecs().canReportIntermediateSolutions)
        OMPL_WARN("%s cannot report intermediate solutions, not added as CForest planner.", planner->getName().c_str());
    else
    {
        planner->setProblemDefinition(pdef_);
        if (planner->params().hasParam("focus_search"))
            planner->params()["focus_search"] = focusSearch_;
        else
            OMPL_WARN("%s does not appear to support search focusing.", planner->getName().c_str());

        planners_.push_back(planner);
    }
}

void ompl::geometric::CForest::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    for (std::size_t i = 0; i < planners_.size(); ++i)
    {
        base::PlannerData pd(si_);
        planners_[i]->getPlannerData(pd);

        for (unsigned int j = 0; j < pd.numVertices(); ++j)
        {
            base::PlannerDataVertex &v = pd.getVertex(j);

            v.setTag(i);
            std::vector<unsigned int> edgeList;
            unsigned int numEdges = pd.getIncomingEdges(j, edgeList);
            for (unsigned int k = 0; k < numEdges; ++k)
            {
                base::Cost edgeWeight;
                base::PlannerDataVertex &w = pd.getVertex(edgeList[k]);

                w.setTag(i);
                pd.getEdgeWeight(j, k, &edgeWeight);
                data.addEdge(v, w, pd.getEdge(j, k), edgeWeight);
            }
        }

        for (unsigned int j = 0; j < pd.numGoalVertices(); ++j)
            data.markGoalState(pd.getGoalVertex(j).getState());

        for (unsigned int j = 0; j < pd.numStartVertices(); ++j)
            data.markStartState(pd.getStartVertex(j).getState());
    }
}

void ompl::geometric::CForest::clear()
{
    Planner::clear();
    for (auto &planner : planners_)
        planner->clear();

    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    numPathsShared_ = 0;
    numStatesShared_ = 0;

    std::vector<base::StateSamplerPtr> samplers;
    samplers.reserve(samplers_.size());
    for (auto &sampler : samplers_)
        if (sampler.use_count() > 1)
            samplers.push_back(sampler);
    samplers_.swap(samplers);
}

void ompl::geometric::CForest::setup()
{
    Planner::setup();
    if (pdef_->hasOptimizationObjective())
        opt_ = pdef_->getOptimizationObjective();
    else
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                    "planning time.",
                    getName().c_str());
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
    }

    bestCost_ = opt_->infiniteCost();

    if (planners_.empty())
    {
        OMPL_INFORM("%s: Number and type of instances not specified. Defaulting to %d instances of RRTstar.",
                    getName().c_str(), numThreads_);
        addPlannerInstances<RRTstar>(numThreads_);
    }

    for (auto &planner : planners_)
        if (!planner->isSetup())
            planner->setup();

    // This call is needed to make sure the ParamSet is up to date after changes induced by the planner setup calls
    // above, via the state space wrappers for CForest.
    si_->setup();
}

ompl::base::PlannerStatus ompl::geometric::CForest::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    time::point start = time::now();
    std::vector<std::thread *> threads(planners_.size());
    const base::ReportIntermediateSolutionFn prevSolutionCallback =
        getProblemDefinition()->getIntermediateSolutionCallback();

    if (prevSolutionCallback)
        OMPL_WARN("Cannot use previously set intermediate solution callback with %s", getName().c_str());

    pdef_->setIntermediateSolutionCallback(
        [this](const base::Planner *planner, const std::vector<const base::State *> &states, const base::Cost cost) {
            return newSolutionFound(planner, states, cost);
        });
    bestCost_ = opt_->infiniteCost();

    // run each planner in its own thread, with the same ptc.
    for (std::size_t i = 0; i < threads.size(); ++i)
    {
        base::Planner *planner = planners_[i].get();
        threads[i] = new std::thread([this, planner, &ptc]
                                     {
                                         return solve(planner, ptc);
                                     });
    }

    for (auto &thread : threads)
    {
        thread->join();
        delete thread;
    }

    // restore callback
    getProblemDefinition()->setIntermediateSolutionCallback(prevSolutionCallback);
    OMPL_INFORM("Solution found in %f seconds", time::seconds(time::now() - start));
    return {pdef_->hasSolution(), pdef_->hasApproximateSolution()};
}

std::string ompl::geometric::CForest::getBestCost() const
{
    return ompl::toString(bestCost_.value());
}

std::string ompl::geometric::CForest::getNumPathsShared() const
{
    return std::to_string(numPathsShared_);
}

std::string ompl::geometric::CForest::getNumStatesShared() const
{
    return std::to_string(numStatesShared_);
}

void ompl::geometric::CForest::newSolutionFound(const base::Planner *planner,
                                                const std::vector<const base::State *> &states, const base::Cost cost)
{
    bool change = false;
    std::vector<const base::State *> statesToShare;
    newSolutionFoundMutex_.lock();
    if (opt_->isCostBetterThan(cost, bestCost_))
    {
        ++numPathsShared_;
        bestCost_ = cost;
        change = true;

        // Filtering the states to add only those not already added.
        statesToShare.reserve(states.size());
        for (auto state : states)
        {
            if (statesShared_.find(state) == statesShared_.end())
            {
                statesShared_.insert(state);
                statesToShare.push_back(state);
                ++numStatesShared_;
            }
        }
    }
    newSolutionFoundMutex_.unlock();

    if (!change || statesToShare.empty())
        return;

    for (auto &i : samplers_)
    {
        auto *sampler = static_cast<base::CForestStateSampler *>(i.get());
        const auto *space =
            static_cast<const base::CForestStateSpaceWrapper *>(sampler->getStateSpace());
        const base::Planner *cfplanner = space->getPlanner();
        if (cfplanner != planner)
            sampler->setStatesToSample(statesToShare);
    }
}

void ompl::geometric::CForest::solve(base::Planner *planner, const base::PlannerTerminationCondition &ptc)
{
    OMPL_DEBUG("Starting %s", planner->getName().c_str());
    time::point start = time::now();
    if (planner->solve(ptc))
    {
        double duration = time::seconds(time::now() - start);
        OMPL_DEBUG("Solution found by %s in %lf seconds", planner->getName().c_str(), duration);
    }
}
