/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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

#include "ompl/geometric/planners/est/BiEST.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::BiEST::BiEST(const base::SpaceInformationPtr &si) : base::Planner(si, "BiEST")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &BiEST::setRange, &BiEST::getRange, "0.:1.:10000.");
}

ompl::geometric::BiEST::~BiEST()
{
    freeMemory();
}

void ompl::geometric::BiEST::setup()
{
    Planner::setup();

    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    // Make the neighborhood radius smaller than sampling range to
    // keep probabilities relatively high for rejection sampling
    nbrhoodRadius_ = maxDistance_ / 3.0;

    if (!nnStart_)
        nnStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!nnGoal_)
        nnGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nnStart_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                  {
                                      return distanceFunction(a, b);
                                  });
    nnGoal_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                 {
                                     return distanceFunction(a, b);
                                 });
}

void ompl::geometric::BiEST::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nnStart_)
        nnStart_->clear();
    if (nnGoal_)
        nnGoal_->clear();

    startMotions_.clear();
    startPdf_.clear();

    goalMotions_.clear();
    goalPdf_.clear();

    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
}

void ompl::geometric::BiEST::freeMemory()
{
    for (auto &startMotion : startMotions_)
    {
        if (startMotion->state != nullptr)
            si_->freeState(startMotion->state);
        delete startMotion;
    }

    for (auto &goalMotion : goalMotions_)
    {
        if (goalMotion->state != nullptr)
            si_->freeState(goalMotion->state);
        delete goalMotion;
    }
}

ompl::base::PlannerStatus ompl::geometric::BiEST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    std::vector<Motion *> neighbors;

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;

        nnStart_->nearestR(motion, nbrhoodRadius_, neighbors);
        addMotion(motion, startMotions_, startPdf_, nnStart_, neighbors);
    }

    if (startMotions_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                startMotions_.size() + goalMotions_.size());

    base::State *xstate = si_->allocState();
    auto *xmotion = new Motion();

    bool startTree = true;
    bool solved = false;

    while (!ptc && !solved)
    {
        // Make sure goal tree has at least one state.
        if (goalMotions_.empty() || pis_.getSampledGoalsCount() < goalMotions_.size() / 2)
        {
            const base::State *st = goalMotions_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;

                nnGoal_->nearestR(motion, nbrhoodRadius_, neighbors);
                addMotion(motion, goalMotions_, goalPdf_, nnGoal_, neighbors);
            }

            if (goalMotions_.empty())
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        // Pointers to the tree structure we are expanding
        std::vector<Motion *> &motions = startTree ? startMotions_ : goalMotions_;
        PDF<Motion *> &pdf = startTree ? startPdf_ : goalPdf_;
        std::shared_ptr<NearestNeighbors<Motion *>> nn = startTree ? nnStart_ : nnGoal_;

        // Select a state to expand from
        Motion *existing = pdf.sample(rng_.uniform01());
        assert(existing);

        // Sample a state in the neighborhood
        if (!sampler_->sampleNear(xstate, existing->state, maxDistance_))
            continue;

        // Compute neighborhood of candidate state
        xmotion->state = xstate;
        nn->nearestR(xmotion, nbrhoodRadius_, neighbors);

        // reject state with probability proportional to neighborhood density
        if (!neighbors.empty() )
        {
            double p = 1.0 - (1.0 / neighbors.size());
            if (rng_.uniform01() < p)
                continue;
        }

        // Is motion good?
        if (si_->checkMotion(existing->state, xstate))
        {
            // create a motion
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, xstate);
            motion->parent = existing;
            motion->root = existing->root;

            // add it to everything
            addMotion(motion, motions, pdf, nn, neighbors);

            // try to connect this state to the other tree
            // Get all states in the other tree within a maxDistance_ ball (bigger than "neighborhood" ball)
            startTree ? nnGoal_->nearestR(motion, maxDistance_, neighbors) :
                        nnStart_->nearestR(motion, maxDistance_, neighbors);
            for (size_t i = 0; i < neighbors.size() && !solved; ++i)
            {
                if (goal->isStartGoalPairValid(motion->root, neighbors[i]->root) &&
                    si_->checkMotion(motion->state, neighbors[i]->state))  // win!  solution found.
                {
                    connectionPoint_ = std::make_pair(motion->state, neighbors[i]->state);

                    Motion *startMotion = startTree ? motion : neighbors[i];
                    Motion *goalMotion = startTree ? neighbors[i] : motion;

                    Motion *solution = startMotion;
                    std::vector<Motion *> mpath1;
                    while (solution != nullptr)
                    {
                        mpath1.push_back(solution);
                        solution = solution->parent;
                    }

                    solution = goalMotion;
                    std::vector<Motion *> mpath2;
                    while (solution != nullptr)
                    {
                        mpath2.push_back(solution);
                        solution = solution->parent;
                    }

                    auto path(std::make_shared<PathGeometric>(si_));
                    path->getStates().reserve(mpath1.size() + mpath2.size());
                    for (int i = mpath1.size() - 1; i >= 0; --i)
                        path->append(mpath1[i]->state);
                    for (auto &i : mpath2)
                        path->append(i->state);

                    pdef_->addSolutionPath(path, false, 0.0, getName());
                    solved = true;
                }
            }
        }

        // swap trees for next iteration
        startTree = !startTree;
    }

    si_->freeState(xstate);
    delete xmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(),
                startMotions_.size() + goalMotions_.size(), startMotions_.size(), goalMotions_.size());
    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::BiEST::addMotion(Motion *motion, std::vector<Motion *> &motions, PDF<Motion *> &pdf,
                                       const std::shared_ptr<NearestNeighbors<Motion *>> &nn,
                                       const std::vector<Motion *> &neighbors)
{
    // Updating neighborhood size counts
    for (auto neighbor : neighbors)
    {
        PDF<Motion *>::Element *elem = neighbor->element;
        double w = pdf.getWeight(elem);
        pdf.update(elem, w / (w + 1.));
    }

    motion->element = pdf.add(motion, 1. / (neighbors.size() + 1.));  // +1 for self
    motions.push_back(motion);
    nn->add(motion);
}

void ompl::geometric::BiEST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    for (auto startMotion : startMotions_)
    {
        if (startMotion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(startMotion->state, 1));
        else
            data.addEdge(base::PlannerDataVertex(startMotion->parent->state, 1),
                         base::PlannerDataVertex(startMotion->state, 1));
    }

    for (auto goalMotion : goalMotions_)
    {
        if (goalMotion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(goalMotion->state, 2));
        else
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(goalMotion->state, 2),
                         base::PlannerDataVertex(goalMotion->parent->state, 2));
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}
