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

#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/util/Exception.h>
#include <ompl/util/Time.h>
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/Projection.h>

template <class T>
ompl::multilevel::BundleSpaceSequence<T>::BundleSpaceSequence(ompl::base::SpaceInformationPtr si, std::string type)
  : BaseT(si, type)
{
    declareBundleSpaces(true);
}

template <class T>
ompl::multilevel::BundleSpaceSequence<T>::BundleSpaceSequence(std::vector<ompl::base::SpaceInformationPtr> &siVec,
                                                              std::string type)
  : BaseT(siVec, type)
{
    declareBundleSpaces(true);
}

template <class T>
ompl::multilevel::BundleSpaceSequence<T>::BundleSpaceSequence(std::vector<ompl::base::SpaceInformationPtr> &siVec,
                                                              std::vector<ompl::multilevel::ProjectionPtr> &projVec,
                                                              std::string type)
  : BaseT(siVec, type)
{
    assert(siVec.size() > 0);
    assert((siVec.size() - 1) == projVec.size());
    declareBundleSpaces(false);

    // None projection (from last state to empty)
    bundleSpaces_.front()->makeProjection();
    for (unsigned int k = 1; k < bundleSpaces_.size(); k++)
    {
        BundleSpace *bk = bundleSpaces_.at(k);
        bk->setProjection(projVec.at(k - 1));
        // need to precompute the location helper functions to utilize
        //"copyToReals" inside the projection function
        bk->getBundle()->setup();
    }
}

template <class T>
void ompl::multilevel::BundleSpaceSequence<T>::declareBundleSpaces(bool guessProjection)
{
    T::resetCounter();
    for (unsigned int k = 0; k < siVec_.size(); k++)
    {
        T *parent = nullptr;
        if (k > 0)
            parent = bundleSpaces_.back();

        T *ss = new T(siVec_.at(k), parent);
        bundleSpaces_.push_back(ss);
        static_cast<BundleSpace *>(bundleSpaces_.back())->setLevel(k);
    }
    stopAtLevel_ = bundleSpaces_.size();

    if (guessProjection)
    {
        for (unsigned int k = 0; k < bundleSpaces_.size(); k++)
        {
            bundleSpaces_.at(k)->makeProjection();
        }
    }

    OMPL_DEBUG("Created %d BundleSpace levels (%s).", siVec_.size(), getName().c_str());
}

template <class T>
ompl::multilevel::BundleSpaceSequence<T>::~BundleSpaceSequence()
{
    for (unsigned int k = 0; k < bundleSpaces_.size(); k++)
    {
        if (bundleSpaces_.at(k))
        {
            delete bundleSpaces_.at(k);
        }
    }
    bundleSpaces_.clear();
}

template <class T>
void ompl::multilevel::BundleSpaceSequence<T>::setStopLevel(unsigned int level_)
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
void ompl::multilevel::BundleSpaceSequence<T>::setFindSectionStrategy(FindSectionType type)
{
    for (unsigned int k = 0; k < bundleSpaces_.size(); k++)
    {
        BundleSpaceGraph *bsg = dynamic_cast<BundleSpaceGraph *>(bundleSpaces_.at(k));
        if (bsg != nullptr)
        {
            bsg->setFindSectionStrategy(type);
        }
    }
}

template <class T>
void ompl::multilevel::BundleSpaceSequence<T>::setup()
{
    BaseT::setup();
    for (unsigned int k = 0; k < stopAtLevel_; k++)
    {
        static_cast<BundleSpace *>(bundleSpaces_.at(k))->setup();
    }
    currentBundleSpaceLevel_ = 0;
}

template <class T>
void ompl::multilevel::BundleSpaceSequence<T>::clear()
{
    BaseT::clear();

    for (unsigned int k = 0; k < bundleSpaces_.size(); k++)
    {
        static_cast<BundleSpace *>(bundleSpaces_.at(k))->clear();
    }
    currentBundleSpaceLevel_ = 0;

    while (!priorityQueue_.empty())
        priorityQueue_.pop();

    foundKLevelSolution_ = false;
}

template <class T>
ompl::base::PlannerStatus
ompl::multilevel::BundleSpaceSequence<T>::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    ompl::time::point t_start = ompl::time::now();

    for (unsigned int k = currentBundleSpaceLevel_; k < stopAtLevel_; k++)
    {
        BundleSpace *kBundle = static_cast<BundleSpace *>(bundleSpaces_.at(k));

        foundKLevelSolution_ = false;

        if (priorityQueue_.size() <= currentBundleSpaceLevel_)
            priorityQueue_.push(kBundle);

        ompl::base::PlannerTerminationCondition ptcOrSolutionFound(
            [this, &ptc] { return ptc || foundKLevelSolution_; });

        while (!ptcOrSolutionFound())
        {
            BundleSpace *jBundle = priorityQueue_.top();
            priorityQueue_.pop();
            jBundle->grow();

            bool hasSolution = kBundle->hasSolution();
            if (hasSolution)
            {
                ompl::base::PathPtr sol_k;
                kBundle->getSolution(sol_k);
                if (solutions_.size() < k + 1)
                {
                    solutions_.push_back(sol_k);
                    double t_k_end = ompl::time::seconds(ompl::time::now() - t_start);
                    OMPL_DEBUG("Found Solution on Level %d/%d after %f seconds.", k + 1, stopAtLevel_, t_k_end);
                    currentBundleSpaceLevel_ = k + 1;
                    if (currentBundleSpaceLevel_ > (bundleSpaces_.size() - 1))
                        currentBundleSpaceLevel_ = bundleSpaces_.size() - 1;
                }
                else
                {
                    solutions_.at(k) = sol_k;
                }
                foundKLevelSolution_ = true;

                // add solution to pdef
                ompl::base::PlannerSolution psol(sol_k);
                std::string lvl_name = getName() + " LvL" + std::to_string(k);
                psol.setPlannerName(lvl_name);

                kBundle->getProblemDefinition()->clearSolutionPaths();
                kBundle->getProblemDefinition()->addSolutionPath(psol);
            }

            bool isInfeasible = kBundle->isInfeasible();
            if (isInfeasible)
            {
                double t_end = ompl::time::seconds(ompl::time::now() - t_start);
                OMPL_DEBUG("Infeasibility detected after %f seconds (level %d).", t_end, k);
                return ompl::base::PlannerStatus::INFEASIBLE;
            }
            priorityQueue_.push(jBundle);
        }

        if (!foundKLevelSolution_)
        {
            OMPL_DEBUG("-- Planner failed finding solution on BundleSpace level %d", k);
            return ompl::base::PlannerStatus::TIMEOUT;
        }
    }

    ompl::base::PathPtr sol;
    ompl::base::PlannerSolution psol(sol);
    static_cast<BundleSpace *>(bundleSpaces_.back())->getProblemDefinition()->getSolution(psol);
    pdef_->addSolutionPath(psol);

    return ompl::base::PlannerStatus::EXACT_SOLUTION;
}

template <class T>
void ompl::multilevel::BundleSpaceSequence<T>::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
    BaseT::setProblemDefinition(pdef);

    if (siVec_.size() < 1)
        return;

    pdefVec_.clear();
    pdefVec_.push_back(pdef);
    bundleSpaces_.back()->setProblemDefinition(pdef);

    if (siVec_.size() <= 1)
        return;

    assert(bundleSpaces_.size() == siVec_.size());

    //#########################################################################
    // Check if goal type is projectable
    //#########################################################################
    ompl::base::GoalType type = pdef_->getGoal()->getType();
    if (!(type == ompl::base::GoalType::GOAL_STATE || type == ompl::base::GoalType::GOAL_STATES))
    {
        OMPL_ERROR("If you want to use other goal classes than \"GoalSampleableRegion\", you need to specify them "
                   "manually for each SpaceInformationPtr in the hierarchy.");
        throw ompl::Exception("Multilevel framework does not support provided goal specs.");
    }

    //#########################################################################
    // Iterate through all levels and project from the last level down
    //#########################################################################
    ompl::base::GoalSampleableRegion *goalRegion =
        static_cast<ompl::base::GoalSampleableRegion *>(pdef_->getGoal().get());
    double epsilon = goalRegion->getThreshold();

    base::OptimizationObjectivePtr obj = pdef->getOptimizationObjective();

    for (unsigned int k = siVec_.size() - 1; k > 0; k--)
    {
        BundleSpace *parent = static_cast<BundleSpace *>(bundleSpaces_.at(k));
        BundleSpace *child = static_cast<BundleSpace *>(bundleSpaces_.at(k - 1));
        ompl::base::SpaceInformationPtr siChild = child->getBundle();

        ompl::base::ProblemDefinitionPtr pdefParent = pdefVec_.back();

        ompl::base::ProblemDefinitionPtr pdefChild = std::make_shared<base::ProblemDefinition>(siChild);

        // Project Start State onto lower dimensional quotient space
        const ompl::base::State *sInitParent = pdefParent->getStartState(0);
        ompl::base::State *sInitChild = siChild->allocState();

        parent->getProjection()->project(sInitParent, sInitChild);
        parent->getProjection()->project(sInitParent, sInitChild);
        pdefChild->addStartState(sInitChild);

        // Now project goal state(s) down
        if (type == ompl::base::GoalType::GOAL_STATE)
        {
            ompl::base::GoalState *goal = static_cast<ompl::base::GoalState *>(pdefParent->getGoal().get());

            const ompl::base::State *sGoalParent = goal->getState();
            ompl::base::State *sGoalChild = siChild->allocState();
            parent->getProjection()->project(sGoalParent, sGoalChild);
            pdefChild->setGoalState(sGoalChild, epsilon);
        }
        else if (type == ompl::base::GoalType::GOAL_STATES)
        {
            ompl::base::GoalStates *goal = static_cast<ompl::base::GoalStates *>(pdefParent->getGoal().get());
            unsigned int N = goal->getStateCount();

            ompl::base::GoalStatesPtr goalStates = std::make_shared<ompl::base::GoalStates>(siChild);
            goalStates->setThreshold(epsilon);

            for (unsigned int j = 0; j < N; j++)
            {
                const ompl::base::State *sGoalParent = goal->getState(j);
                ompl::base::State *sGoalChild = siChild->allocState();
                parent->getProjection()->project(sGoalParent, sGoalChild);
                goalStates->addState(sGoalChild);
            }
            pdefChild->setGoal(goalStates);
        }

        child->setProblemDefinition(pdefChild);
        pdefVec_.push_back(pdefChild);
    }

    std::reverse(pdefVec_.begin(), pdefVec_.end());
}

template <class T>
ompl::base::State *ompl::multilevel::BundleSpaceSequence<T>::getTotalState(int baseLevel,
                                                                           const base::State *baseState) const
{
    BundleSpace *Qprev = bundleSpaces_.at(baseLevel);
    ompl::base::State *s_lift = Qprev->getBundle()->cloneState(baseState);

    for (unsigned int m = baseLevel + 1; m < bundleSpaces_.size(); m++)
    {
        BundleSpace *Qm = bundleSpaces_.at(m);

        if (Qm->getProjection()->getCoDimension() > 0)
        {
            base::State *s_Bundle = Qm->allocIdentityStateBundle();

            Qm->getProjection()->lift(s_lift, s_Bundle);

            Qprev->getBundle()->freeState(s_lift);

            s_lift = Qm->getBundle()->cloneState(s_Bundle);

            Qm->getBundle()->freeState(s_Bundle);

            Qprev = Qm;
        }
    }
    return s_lift;
}

template <class T>
void ompl::multilevel::BundleSpaceSequence<T>::getPlannerData(ompl::base::PlannerData &data) const
{
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        OMPL_ERROR("PlannerData has %d vertices.", Nvertices);
        throw ompl::Exception("cannot get planner data if plannerdata is already populated");
    }

    unsigned int K = std::min(solutions_.size() + 1, bundleSpaces_.size());
    K = std::min(K, stopAtLevel_);

    BundleSpace *Qlast = this->bundleSpaces_.back();
    for (unsigned int k = 0; k < K; k++)
    {
        BundleSpace *Qk = static_cast<BundleSpace *>(bundleSpaces_.at(k));
        Qk->getPlannerData(data);

        // lift all states into the last bundle space (original state space)
        // Required for decouplePlannerData() function in PlannerData
        for (unsigned int vidx = Nvertices; vidx < data.numVertices(); vidx++)
        {
            ompl::multilevel::PlannerDataVertexAnnotated &v =
                static_cast<ompl::multilevel::PlannerDataVertexAnnotated &>(data.getVertex(vidx));
            v.setLevel(k);
            v.setMaxLevel(K);

            base::State *s_lift = getTotalState(k, v.getBaseState());
            v.setTotalState(s_lift, Qlast->getBundle());
        }
        Nvertices = data.numVertices();
    }
    OMPL_DEBUG("Multilevel Graph has %d/%d vertices/edges", data.numVertices(), data.numEdges());
}
