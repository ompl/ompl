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

ompl::geometric::CForest::CForest(const base::SpaceInformationPtr &si) : base::Planner(si, "CForest")
{
    specs_.optimizingPaths = true;
    specs_.multithreaded = true;

    pathsShared_ = 0;
    prune_ = true;

    Planner::declareParam<bool>("prune", this, &CForest::setPrune, &CForest::getPrune, "0,1");

    addPlannerProgressProperty("best cost REAL",
                               boost::bind(&CForest::getBestCost, this));
    addPlannerProgressProperty("shared paths INTEGER",
                               boost::bind(&CForest::getPathsShared, this));
}

ompl::geometric::CForest::~CForest()
{
}

void ompl::geometric::CForest::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    for (std::size_t i = 0 ; i < planners_.size() ; ++i)
    {
        base::PlannerData pd(si_);
        planners_[i]->getPlannerData(pd);

        for (unsigned int j = 0; j < pd.numVertices(); ++j)
        {
            base::PlannerDataVertex &v = pd.getVertex(j);

            v.setTag(i);
            std::vector<unsigned int> edgeList;
            unsigned int numEdges = pd.getIncomingEdges(j, edgeList);
            for (unsigned int k = 0; k <numEdges; ++k)
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

    data.properties["shared paths INTEGER"] = getPathsShared();
}

void ompl::geometric::CForest::clear()
{
    Planner::clear();
    for (std::size_t i = 0; i < planners_.size(); ++i)
        planners_[i]->clear();

    bestCost_ = opt_->infiniteCost();
    pathsShared_ = 0;

    std::vector<base::StateSamplerPtr> samplers;
    samplers.reserve(samplers_.size());
    for (std::size_t i = 0; i < samplers_.size(); ++i)
        if (samplers_[i].use_count() > 1)
            samplers.push_back(samplers_[i]);
    samplers_.swap(samplers);
}

void ompl::geometric::CForest::setup()
{
    Planner::setup();
    if (pdef_->hasOptimizationObjective())
        opt_ = pdef_->getOptimizationObjective();
    else
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
        opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }

    bestCost_ = opt_->infiniteCost();

    if (planners_.empty())
    {
        OMPL_INFORM("%s: Number and type of instances not specified. Defaulting to 2 instances of RRTstar", getName().c_str());
        addPlannerInstances<RRTstar>(2);
    }

    for (std::size_t i = 0; i < planners_.size() ; ++i)
        planners_[i]->setup();
}

ompl::base::PlannerStatus ompl::geometric::CForest::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    time::point start = time::now();
    std::vector<boost::thread*> threads(planners_.size());
    const base::ReportIntermediateSolutionFn prevSolutionCallback = getProblemDefinition()->getIntermediateSolutionCallback();

    if (prevSolutionCallback)
        OMPL_WARN("Cannot use previously set intermediate solution callback with %s", getName().c_str());

    pdef_->setIntermediateSolutionCallback(boost::bind(&CForest::newSolutionFound, this, _1, _2, _3));

    // run each planner in its own thread, with the same ptc.
    for (std::size_t i = 0 ; i < threads.size() ; ++i)
        threads[i] = new boost::thread(boost::bind(&CForest::solve, this, planners_[i].get(), ptc));

    for (std::size_t i = 0 ; i < threads.size() ; ++i)
    {
        threads[i]->join();
        delete threads[i];
    }

    // restore callback
    getProblemDefinition()->setIntermediateSolutionCallback(prevSolutionCallback);
    OMPL_INFORM("Solution found in %f seconds", time::seconds(time::now() - start));
    return base::PlannerStatus(pdef_->hasSolution(), pdef_->hasApproximateSolution());
}

std::string ompl::geometric::CForest::getBestCost() const
{
    return boost::lexical_cast<std::string>(bestCost_.v);
}

std::string ompl::geometric::CForest::getPathsShared() const
{
    return boost::lexical_cast<std::string>(pathsShared_);
}

void ompl::geometric::CForest::newSolutionFound(const base::Planner *planner, const std::vector<const base::State *> &states, const base::Cost cost)
{
    bool change = false;
    newSolutionFoundMutex_.lock();
    if (opt_->isCostBetterThan(cost, bestCost_))
    {
        pathsShared_++;
        bestCost_ = cost;
        change = true;
    }
    newSolutionFoundMutex_.unlock();

    if (!change) return;

    for (std::size_t i = 0; i < samplers_.size(); ++i)
    {
        base::CForestStateSampler* sampler = dynamic_cast<base::CForestStateSampler*>(samplers_[i].get());
        const base::CForestStateSpace *space = dynamic_cast<const base::CForestStateSpace*>(sampler->getStateSpace());
        const base::Planner *cfplanner = space->getPlanner();
        if (cfplanner != planner)
            sampler->setStatesToSample(states);
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