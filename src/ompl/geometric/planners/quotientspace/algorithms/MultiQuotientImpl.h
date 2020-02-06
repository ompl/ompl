/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */
#include <ompl/geometric/planners/quotientspace/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/util/Exception.h>
#include <ompl/util/Time.h>
#include <queue>

template <class T>
ompl::geometric::MultiQuotient<T>::MultiQuotient(std::vector<ompl::base::SpaceInformationPtr> &siVec, std::string type)
  : ompl::base::Planner(siVec.back(), type), siVec_(siVec)
{
    T::resetCounter();
    for (unsigned int k = 0; k < siVec_.size(); k++)
    {
        BundleSpace *parent = nullptr;
        if (k > 0)
            parent = bundleSpaces_.back();

        T *ss = new T(siVec_.at(k), parent);
        bundleSpaces_.push_back(ss);
        bundleSpaces_.back()->setLevel(k);
    }
    stopAtLevel_ = bundleSpaces_.size();
    OMPL_DEVMSG2("Created %d BundleSpace levels.", siVec_.size());
}

template <class T>
int ompl::geometric::MultiQuotient<T>::getLevels() const
{
    return stopAtLevel_;
}

template <class T>
std::vector<int> ompl::geometric::MultiQuotient<T>::getNodes() const
{
    std::vector<int> nodesPerLevel;
    for (unsigned int k = 0; k < stopAtLevel_; k++)
    {
        unsigned int Nk = bundleSpaces_.at(k)->getTotalNumberOfSamples();
        nodesPerLevel.push_back(Nk);
    }
    return nodesPerLevel;
}

template <class T>
std::vector<int> ompl::geometric::MultiQuotient<T>::getFeasibleNodes() const
{
    std::vector<int> feasibleNodesPerLevel;
    for (unsigned int k = 0; k < bundleSpaces_.size(); k++)
    {
        unsigned int Nk = bundleSpaces_.at(k)->getTotalNumberOfFeasibleSamples();
        feasibleNodesPerLevel.push_back(Nk);
    }
    return feasibleNodesPerLevel;
}

template <class T>
std::vector<int> ompl::geometric::MultiQuotient<T>::getDimensionsPerLevel() const
{
    std::vector<int> dimensionsPerLevel;
    for (unsigned int k = 0; k < bundleSpaces_.size(); k++)
    {
        unsigned int Nk = bundleSpaces_.at(k)->getDimension();
        dimensionsPerLevel.push_back(Nk);
    }
    return dimensionsPerLevel;
}

template <class T>
ompl::geometric::MultiQuotient<T>::~MultiQuotient()
{
}

template <class T>
void ompl::geometric::MultiQuotient<T>::setup()
{
    BaseT::setup();
    for (unsigned int k = 0; k < stopAtLevel_; k++)
    {
        bundleSpaces_.at(k)->setup();
    }
    currentQuotientLevel_ = 0;
}

template <class T>
void ompl::geometric::MultiQuotient<T>::setStopLevel(unsigned int level_)
{
    if (level_ > bundleSpaces_.size())
    {
        stopAtLevel_ = bundleSpaces_.size();
    }
    else
    {
        stopAtLevel_ = level_;
    }
}

template <class T>
void ompl::geometric::MultiQuotient<T>::clear()
{
    Planner::clear();

    for (unsigned int k = 0; k < bundleSpaces_.size(); k++)
    {
        bundleSpaces_.at(k)->clear();
    }
    currentQuotientLevel_ = 0;

    while (!priorityQueue_.empty())
        priorityQueue_.pop();
    foundKLevelSolution_ = false;

    solutions_.clear();
    pdef_->clearSolutionPaths();
}

template <class T>
ompl::base::PlannerStatus ompl::geometric::MultiQuotient<T>::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    ompl::time::point t_start = ompl::time::now();

    for (unsigned int k = currentQuotientLevel_; k < stopAtLevel_; k++)
    {
        foundKLevelSolution_ = false;

        if (priorityQueue_.size() <= currentQuotientLevel_)
            priorityQueue_.push(bundleSpaces_.at(k));

        ompl::base::PlannerTerminationCondition ptcOrSolutionFound(
            [this, &ptc] { return ptc || foundKLevelSolution_; });

        while (!ptcOrSolutionFound())
        {
            BundleSpace *jQuotient = priorityQueue_.top();
            priorityQueue_.pop();
            jQuotient->grow();

            bool hasSolution = bundleSpaces_.at(k)->hasSolution();
            if (hasSolution)
            {
                ompl::base::PathPtr sol_k;
                bundleSpaces_.at(k)->getSolution(sol_k);
                solutions_.push_back(sol_k);
                double t_k_end = ompl::time::seconds(ompl::time::now() - t_start);
                OMPL_DEBUG("Found Solution on Level %d after %f seconds.", k, t_k_end);
                foundKLevelSolution_ = true;
                currentQuotientLevel_ = k + 1;//std::min(k + 1, bundleSpaces_.size()-1);
                if(currentQuotientLevel_ > (bundleSpaces_.size()-1)) 
                  currentQuotientLevel_ = bundleSpaces_.size()-1;

                // add solution to pdef
                ompl::base::PlannerSolution psol(sol_k);
                std::string lvl_name = getName() + " LvL" + std::to_string(k);
                psol.setPlannerName(lvl_name);
                bundleSpaces_.at(k)->getProblemDefinition()->addSolutionPath(psol);
            }
            priorityQueue_.push(jQuotient);
        }

        if (!foundKLevelSolution_)
        {
            OMPL_DEBUG("Planner failed finding solution on BundleSpace level %d", k);
            return ompl::base::PlannerStatus::TIMEOUT;
        }
    }
    double t_end = ompl::time::seconds(ompl::time::now() - t_start);
    OMPL_DEBUG("Found exact solution after %f seconds.", t_end);

    ompl::base::PathPtr sol;
    int lastSolvedBundleSpaceLevel = currentQuotientLevel_ - 1;
    if(lastSolvedBundleSpaceLevel < 0) lastSolvedBundleSpaceLevel = 0;

    if (bundleSpaces_.at(lastSolvedBundleSpaceLevel)->getSolution(sol))
    {
        ompl::base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        pdef_->addSolutionPath(psol);
    }

    return ompl::base::PlannerStatus::EXACT_SOLUTION;
}

template <class T>
const ompl::base::ProblemDefinitionPtr &
ompl::geometric::MultiQuotient<T>::getProblemDefinition(unsigned int kBundleSpace) const
{
    assert(kBundleSpace >= 0);
    assert(kBundleSpace <= siVec_.size() - 1);
    return bundleSpaces_.at(kBundleSpace)->getProblemDefinition();
}

template <class T>
void ompl::geometric::MultiQuotient<T>::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
    this->Planner::setProblemDefinition(pdef);

    // Compute projection of qInit and qGoal onto BundleSpaces
    ompl::base::Goal *goal = pdef_->getGoal().get();
    ompl::base::GoalState *goalRegion = dynamic_cast<ompl::base::GoalState *>(goal);
    double epsilon = goalRegion->getThreshold();
    assert(bundleSpaces_.size() == siVec_.size());

    ompl::base::State *sInit = pdef->getStartState(0);
    ompl::base::State *sGoal = goalRegion->getState();

    OMPL_DEVMSG1("Projecting start and goal onto BundleSpaces.");

    bundleSpaces_.back()->setProblemDefinition(pdef);

    base::OptimizationObjectivePtr obj = pdef->getOptimizationObjective();

    for (unsigned int k = siVec_.size() - 1; k > 0; k--)
    {
        BundleSpace *quotientParent = bundleSpaces_.at(k);
        BundleSpace *quotientChild = bundleSpaces_.at(k - 1);
        ompl::base::SpaceInformationPtr sik = quotientChild->getSpaceInformation();
        ompl::base::ProblemDefinitionPtr pdefk = std::make_shared<base::ProblemDefinition>(sik);

        ompl::base::State *sInitK = sik->allocState();
        ompl::base::State *sGoalK = sik->allocState();

        quotientParent->projectBase(sInit, sInitK);
        quotientParent->projectBase(sGoal, sGoalK);

        pdefk->setStartAndGoalStates(sInitK, sGoalK, epsilon);

        quotientChild->setProblemDefinition(pdefk);

        sInit = sInitK;
        sGoal = sGoalK;
    }
}

template <class T>
void ompl::geometric::MultiQuotient<T>::getPlannerData(ompl::base::PlannerData &data) const
{
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        OMPL_ERROR("PlannerData has %d vertices.", Nvertices);
        throw ompl::Exception("cannot get planner data if plannerdata is already populated");
    }

    unsigned int K = std::min(solutions_.size() + 1, bundleSpaces_.size());
    K = std::min(K, stopAtLevel_);

    for (unsigned int k = 0; k < K; k++)
    {
        BundleSpace *Qk = bundleSpaces_.at(k);
        Qk->getPlannerData(data);

        // label all new vertices
        unsigned int ctr = 0;
        for (unsigned int vidx = Nvertices; vidx < data.numVertices(); vidx++)
        {
            ompl::base::PlannerDataVertexAnnotated &v =
                static_cast<ompl::base::PlannerDataVertexAnnotated &>(data.getVertex(vidx));
            v.setLevel(k);
            v.setMaxLevel(K);

            ompl::base::State *s_lift = Qk->getBundle()->cloneState(v.getState());
            v.setQuotientState(s_lift);

            for (unsigned int m = k + 1; m < bundleSpaces_.size(); m++)
            {
                BundleSpace *Qm = bundleSpaces_.at(m);

                if (Qm->getFiberDimension() > 0)
                {
                    ob::State *s_Bundle = Qm->getBundle()->allocState();
                    ob::State *s_Fiber = Qm->allocIdentityStateFiber();

                    // Qm->mergeStates(s_lift, s_Fiber, s_Bundle); //TODO: segfault?
                    s_lift = Qm->getBundle()->cloneState(s_Bundle);

                    Qm->getBundle()->freeState(s_Bundle);
                    Qm->getFiber()->freeState(s_Fiber);
                }
            }
            v.setState(s_lift);
            ctr++;
        }
        Nvertices = data.numVertices();
    }
}
