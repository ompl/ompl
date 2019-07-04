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
#include "PlannerDataVertexAnnotated.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <queue>

using namespace og;
using namespace ob;

template <class T>
MultiQuotient<T>::MultiQuotient(std::vector<ob::SpaceInformationPtr> &siVec, std::string type)
  : ob::Planner(siVec.back(), type), siVec_(siVec)
{
    T::resetCounter();
    for (uint k = 0; k < siVec_.size(); k++)
    {
        og::QuotientSpace *parent = nullptr;
        if (k > 0)
            parent = quotientSpaces_.back();

        T *ss = new T(siVec_.at(k), parent);
        quotientSpaces_.push_back(ss);
        quotientSpaces_.back()->setLevel(k);
    }
    stopAtLevel_ = quotientSpaces_.size();
    if (DEBUG)
        std::cout << "Created hierarchy with " << siVec_.size() << " levels." << std::endl;
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
    for (uint k = 0; k < stopAtLevel_; k++)
    {
        uint Nk = quotientSpaces_.at(k)->getTotalNumberOfSamples();
        nodesPerLevel.push_back(Nk);
    }
    return nodesPerLevel;
}
template <class T>
std::vector<int> MultiQuotient<T>::getFeasibleNodes() const
{
    std::vector<int> feasibleNodesPerLevel;
    for (uint k = 0; k < quotientSpaces_.size(); k++)
    {
        uint Nk = quotientSpaces_.at(k)->getTotalNumberOfFeasibleSamples();
        feasibleNodesPerLevel.push_back(Nk);
    }
    return feasibleNodesPerLevel;
}

template <class T>
std::vector<int> MultiQuotient<T>::getDimensionsPerLevel() const
{
    std::vector<int> dimensionsPerLevel;
    for (uint k = 0; k < quotientSpaces_.size(); k++)
    {
        uint Nk = quotientSpaces_.at(k)->getDimension();
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
    Planner::setup();
    for (uint k = 0; k < stopAtLevel_; k++)
    {
        quotientSpaces_.at(k)->setup();
    }
    currentQuotientLevel_ = 0;
}

template <class T>
void MultiQuotient<T>::setStopLevel(uint level_)
{
    if (level_ > quotientSpaces_.size())
    {
        stopAtLevel_ = quotientSpaces_.size();
    }
    else
    {
        stopAtLevel_ = level_;
    }
    std::cout << "new stop level: " << stopAtLevel_ << " from " << quotientSpaces_.size() << std::endl;
}

template <class T>
void MultiQuotient<T>::clear()
{
    Planner::clear();

    for (uint k = 0; k < quotientSpaces_.size(); k++)
    {
        quotientSpaces_.at(k)->clear();
    }
    currentQuotientLevel_ = 0;

    while (!priorityQueue_.empty())
        priorityQueue_.pop();
    foundKLevelSolution_ = false;

    solutions_.clear();
    pdef_->clearSolutionPaths();
    for (uint k = 0; k < pdefVec_.size(); k++)
    {
        pdefVec_.at(k)->clearSolutionPaths();
    }
}

template <class T>
ob::PlannerStatus MultiQuotient<T>::solve(const base::PlannerTerminationCondition &ptc)
{
    ompl::time::point t_start = ompl::time::now();

    for (uint k = currentQuotientLevel_; k < stopAtLevel_; k++)
    {
        foundKLevelSolution_ = false;

        if (priorityQueue_.size() <= currentQuotientLevel_)
            priorityQueue_.push(quotientSpaces_.at(k));

        base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc] { return ptc || foundKLevelSolution_; });

        while (!ptcOrSolutionFound())
        {
            og::QuotientSpace *jQuotient = priorityQueue_.top();
            priorityQueue_.pop();
            jQuotient->grow();

            bool hasSolution = quotientSpaces_.at(k)->hasSolution();
            if (hasSolution)
            {
                base::PathPtr sol_k;
                quotientSpaces_.at(k)->getSolution(sol_k);
                solutions_.push_back(sol_k);
                if (DEBUG)
                {
                    double t_k_end = ompl::time::seconds(ompl::time::now() - t_start);
                    std::cout << std::string(80, '#') << std::endl;
                    std::cout << "Found Solution on Level " << k << " after " << t_k_end << " seconds." << std::endl;
                    std::cout << *quotientSpaces_.at(k) << std::endl;
                }
                foundKLevelSolution_ = true;
                currentQuotientLevel_ = k + 1;

                // add solution to pdef
                base::PlannerSolution psol(sol_k);
                std::string lvl_name = getName() + " LvL" + std::to_string(k);
                psol.setPlannerName(lvl_name);
                pdefVec_.at(k)->addSolutionPath(psol);
            }
            priorityQueue_.push(jQuotient);
        }

        if (!foundKLevelSolution_)
        {
            if (DEBUG)
            {
                std::cout << std::string(80, '#') << std::endl;
                for (uint i = 0; i < k + 1; i++)
                {
                    std::cout << *quotientSpaces_.at(i) << std::endl;
                }
            }
            return ob::PlannerStatus::TIMEOUT;
        }
    }
    if (DEBUG)
    {
        double t_end = ompl::time::seconds(ompl::time::now() - t_start);
        std::cout << std::string(80, '#') << std::endl;
        std::cout << "Found exact solution after " << t_end << " seconds." << std::endl;
        std::cout << std::string(80, '#') << std::endl;
    }

    base::PathPtr sol;
    if (quotientSpaces_.at(currentQuotientLevel_ - 1)->getSolution(sol))
    {
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        pdef_->addSolutionPath(psol);
    }

    return ob::PlannerStatus::EXACT_SOLUTION;
}

template <class T>
void MultiQuotient<T>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_)
{
    if (siVec_.size() != pdef_.size())
    {
        OMPL_ERROR("Number of ProblemDefinitionPtr is %d but we have %d SpaceInformationPtr.", pdef_.size(),
                   siVec_.size());
        exit(0);
    }
    pdefVec_ = pdef_;
    ob::Planner::setProblemDefinition(pdefVec_.back());
    for (uint k = 0; k < pdefVec_.size(); k++)
    {
        quotientSpaces_.at(k)->setProblemDefinition(pdefVec_.at(k));
    }
}

template <class T>
void MultiQuotient<T>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
    if (siVec_.size() == 1)
    {
        this->Planner::setProblemDefinition(pdef);
    }
    else
    {
        OMPL_ERROR("You need to provide a ProblemDefinitionPtr for each SpaceInformationPtr.");
        exit(0);
    }
}

template <class T>
void MultiQuotient<T>::getPlannerData(ob::PlannerData &data) const
{
    uint Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        std::cout << "cannot get planner data if plannerdata is already populated" << std::endl;
        std::cout << "PlannerData has " << Nvertices << " vertices." << std::endl;
        exit(0);
    }

    uint K = std::min(solutions_.size() + 1, quotientSpaces_.size());
    K = std::min(K, stopAtLevel_);

    std::vector<int> fn = getFeasibleNodes();
    std::vector<int> n = getNodes();
    int fn_sum = 0;
    int n_sum = 0;
    for (uint k = 0; k < fn.size(); k++)
    {
        std::cout << fn.at(k) << "/" << n.at(k) << std::endl;
        fn_sum += fn.at(k);
        n_sum += n.at(k);
    }
    std::cout << std::string(80, '-') << std::endl;
    std::cout << fn_sum << "/" << n_sum << std::endl;

    for (uint k = 0; k < K; k++)
    {
        og::QuotientSpace *Qk = quotientSpaces_.at(k);
        Qk->getPlannerData(data);

        // label all new vertices
        uint ctr = 0;
        for (uint vidx = Nvertices; vidx < data.numVertices(); vidx++)
        {
            PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated *>(&data.getVertex(vidx));
            v.setLevel(k);
            v.setMaxLevel(K);

            ob::State *s_lift = Qk->getSpaceInformation()->cloneState(v.getState());
            v.setQuotientState(s_lift);

            for (uint m = k + 1; m < quotientSpaces_.size(); m++)
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
