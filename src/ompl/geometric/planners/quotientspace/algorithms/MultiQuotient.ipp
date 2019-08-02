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
#include <ompl/geometric/planners/quotientspace/datastructure/PlannerDataVertexAnnotated.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/util/Time.h>
#include <queue>

using namespace og;
using namespace ob;

template <class T>
MultiQuotient<T>::MultiQuotient(std::vector<ob::SpaceInformationPtr> &siVec, std::string type)
  : ob::Planner(siVec.back(), type), siVec_(siVec)
{
    T::resetCounter();
    for (unsigned int k = 0; k < siVec_.size(); k++)
    {
        og::QuotientSpace *parent = nullptr;
        if (k > 0)
            parent = quotientSpaces_.back();

        T *ss = new T(siVec_.at(k), parent);
        quotientSpaces_.push_back(ss);
        quotientSpaces_.back()->setLevel(k);
    }
    stopAtLevel_ = quotientSpaces_.size();
    OMPL_DEVMSG2("Created %d QuotientSpace levels.",siVec_.size());
}

template <class T>
int MultiQuotient<T>::getLevels() const
{
    return stopAtLevel_;
}
template <class T>
std::vector<int> MultiQuotient<T>::getNodes() const
{
    std::vector<int> nodesPerLevel;
    for (unsigned int k = 0; k < stopAtLevel_; k++)
    {
        unsigned int Nk = quotientSpaces_.at(k)->getTotalNumberOfSamples();
        nodesPerLevel.push_back(Nk);
    }
    return nodesPerLevel;
}
template <class T>
std::vector<int> MultiQuotient<T>::getFeasibleNodes() const
{
    std::vector<int> feasibleNodesPerLevel;
    for (unsigned int k = 0; k < quotientSpaces_.size(); k++)
    {
        unsigned int Nk = quotientSpaces_.at(k)->getTotalNumberOfFeasibleSamples();
        feasibleNodesPerLevel.push_back(Nk);
    }
    return feasibleNodesPerLevel;
}

template <class T>
std::vector<int> MultiQuotient<T>::getDimensionsPerLevel() const
{
    std::vector<int> dimensionsPerLevel;
    for (unsigned int k = 0; k < quotientSpaces_.size(); k++)
    {
        unsigned int Nk = quotientSpaces_.at(k)->getDimension();
        dimensionsPerLevel.push_back(Nk);
    }
    return dimensionsPerLevel;
}

template <class T>
MultiQuotient<T>::~MultiQuotient()
{
}

template <class T>
void MultiQuotient<T>::setup()
{
    BaseT::setup();
    for (unsigned int k = 0; k < stopAtLevel_; k++)
    {
        quotientSpaces_.at(k)->setup();
    }
    currentQuotientLevel_ = 0;
}

template <class T>
void MultiQuotient<T>::setStopLevel(unsigned int level_)
{
    if (level_ > quotientSpaces_.size())
    {
        stopAtLevel_ = quotientSpaces_.size();
    }
    else
    {
        stopAtLevel_ = level_;
    }
}

template <class T>
void MultiQuotient<T>::clear()
{
    Planner::clear();

    for (unsigned int k = 0; k < quotientSpaces_.size(); k++)
    {
        quotientSpaces_.at(k)->clear();
    }
    currentQuotientLevel_ = 0;

    while (!priorityQueue_.empty())
        priorityQueue_.pop();
    foundKLevelSolution_ = false;

    solutions_.clear();
    pdef_->clearSolutionPaths();
}

template <class T>
ob::PlannerStatus MultiQuotient<T>::solve(const ob::PlannerTerminationCondition &ptc)
{
    ompl::time::point t_start = ompl::time::now();

    for (unsigned int k = currentQuotientLevel_; k < stopAtLevel_; k++)
    {
        foundKLevelSolution_ = false;

        if (priorityQueue_.size() <= currentQuotientLevel_)
            priorityQueue_.push(quotientSpaces_.at(k));

        ob::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc] { return ptc || foundKLevelSolution_; });

        while (!ptcOrSolutionFound())
        {
            og::QuotientSpace *jQuotient = priorityQueue_.top();
            priorityQueue_.pop();
            jQuotient->grow();

            bool hasSolution = quotientSpaces_.at(k)->hasSolution();
            if (hasSolution)
            {
                ob::PathPtr sol_k;
                quotientSpaces_.at(k)->getSolution(sol_k);
                solutions_.push_back(sol_k);
                double t_k_end = ompl::time::seconds(ompl::time::now() - t_start);
                OMPL_DEBUG("Found Solution on Level %d after %f seconds.", k, t_k_end);
                foundKLevelSolution_ = true;
                currentQuotientLevel_ = k + 1;

                // add solution to pdef
                ob::PlannerSolution psol(sol_k);
                std::string lvl_name = getName() + " LvL" + std::to_string(k);
                psol.setPlannerName(lvl_name);
                quotientSpaces_.at(k)->getProblemDefinition()->addSolutionPath(psol);
            }
            priorityQueue_.push(jQuotient);
        }

        if (!foundKLevelSolution_)
        {
            OMPL_DEBUG("Planner failed finding solution on QuotientSpace level %d", k);
            return ob::PlannerStatus::TIMEOUT;
        }
    }
    double t_end = ompl::time::seconds(ompl::time::now() - t_start);
    OMPL_DEBUG("Found exact solution after %f seconds.", t_end);

    ob::PathPtr sol;
    if (quotientSpaces_.at(currentQuotientLevel_ - 1)->getSolution(sol))
    {
        ob::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        pdef_->addSolutionPath(psol);
    }

    return ob::PlannerStatus::EXACT_SOLUTION;
}

template <class T>
const ob::ProblemDefinitionPtr &MultiQuotient<T>::getProblemDefinition(unsigned kQuotientSpace) const
{
    assert(kQuotientSpace >= 0);
    assert(kQuotientSpace <= siVec_.size()-1);
    return quotientSpaces_.at(kQuotientSpace)->getProblemDefinition();
}

template <class T>
void MultiQuotient<T>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
    this->Planner::setProblemDefinition(pdef);

    //Compute projection of qInit and qGoal onto QuotientSpaces
    ob::Goal *goal = pdef_->getGoal().get();
    ob::GoalState *goalRegion = dynamic_cast<ob::GoalState *>(goal);
    double epsilon = goalRegion->getThreshold();
    assert(quotientSpaces_.size() == siVec_.size());

    ob::State *sInit = pdef->getStartState(0);
    ob::State *sGoal = goalRegion->getState();

    OMPL_DEVMSG1("Projecting start and goal onto QuotientSpaces.");

    quotientSpaces_.back()->setProblemDefinition(pdef);

    for (unsigned int k = siVec_.size()-1; k > 0 ; k--)
    {
        og::QuotientSpace *quotientParent = quotientSpaces_.at(k);
        og::QuotientSpace *quotientChild = quotientSpaces_.at(k-1);
        ob::SpaceInformationPtr sik = quotientChild->getSpaceInformation();
        ob::ProblemDefinitionPtr pdefk = std::make_shared<base::ProblemDefinition>(sik);

        ob::State *sInitK = sik->allocState();
        ob::State *sGoalK = sik->allocState();

        quotientParent->projectQ0Subspace(sInit, sInitK);
        quotientParent->projectQ0Subspace(sGoal, sGoalK);

        pdefk->setStartAndGoalStates(sInitK, sGoalK, epsilon);

        quotientChild->setProblemDefinition(pdefk);

        sInit = sInitK;
        sGoal = sGoalK;
    }
}


template <class T>
void MultiQuotient<T>::getPlannerData(ob::PlannerData &data) const
{
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        OMPL_ERROR("cannot get planner data if plannerdata is already populated");
        OMPL_ERROR("PlannerData has %d vertices.", Nvertices);
        exit(0);
    }

    unsigned int K = std::min(solutions_.size() + 1, quotientSpaces_.size());
    K = std::min(K, stopAtLevel_);

    std::vector<int> fn = getFeasibleNodes();
    std::vector<int> n = getNodes();
    int fn_sum = 0;
    int n_sum = 0;
    for (unsigned int k = 0; k < fn.size(); k++)
    {
        fn_sum += fn.at(k);
        n_sum += n.at(k);
    }

    for (unsigned int k = 0; k < K; k++)
    {
        og::QuotientSpace *Qk = quotientSpaces_.at(k);
        Qk->getPlannerData(data);

        // label all new vertices
        unsigned int ctr = 0;
        for (unsigned int vidx = Nvertices; vidx < data.numVertices(); vidx++)
        {
            PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated *>(&data.getVertex(vidx));
            v.setLevel(k);
            v.setMaxLevel(K);

            ob::State *s_lift = Qk->getSpaceInformation()->cloneState(v.getState());
            v.setQuotientState(s_lift);

            for (unsigned int m = k + 1; m < quotientSpaces_.size(); m++)
            {
                og::QuotientSpace *Qm = quotientSpaces_.at(m);

                if (Qm->getX1() != nullptr)
                {
                    ob::State *s_X1 = Qm->getX1()->allocState();
                    ob::State *s_Q1 = Qm->getSpaceInformation()->allocState();
                    if (Qm->getX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO3)
                    {
                        static_cast<ob::SO3StateSpace::StateType *>(s_X1)->setIdentity();
                    }
                    if (Qm->getX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO2)
                    {
                        static_cast<ob::SO2StateSpace::StateType *>(s_X1)->setIdentity();
                    }
                    Qm->mergeStates(s_lift, s_X1, s_Q1);
                    s_lift = Qm->getSpaceInformation()->cloneState(s_Q1);

                    Qm->getX1()->freeState(s_X1);
                    Qm->getQ1()->freeState(s_Q1);
                }
            }
            v.setState(s_lift);
            ctr++;
        }
        Nvertices = data.numVertices();
    }
}
