/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2025
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
*   * Neither the name of the copyright holder nor the names of its
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

#include "ompl/geometric/planners/dirt/DIRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::DIRT::DIRT(const base::SpaceInformationPtr &si) : base::Planner(si, "DIRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
    prevSolution_.clear();

    Planner::declareParam<double>("range", this, &DIRT::setRange, &DIRT::getRange, ".1:.1:100");
    Planner::declareParam<unsigned int>("blossom_number", this, &DIRT::setBlossomNumber, &DIRT::getBlossomNumber, "1:1:20");
    Planner::declareParam<bool>("use_pruning", this, &DIRT::setUsePruning, &DIRT::getUsePruning, "0,1");

    addPlannerProgressProperty("best cost REAL", [this] { return std::to_string(this->prevSolutionCost_.value()); });
}

ompl::geometric::DIRT::~DIRT()
{
    freeMemory();
}

void ompl::geometric::DIRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_WARN("%s: No optimization objective set. Using path length", getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }
    else
    {
        OMPL_WARN("%s: No optimization objective set. Using path length", getName().c_str());
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
    }
    prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::geometric::DIRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (opt_)
        prevSolutionCost_ = opt_->infiniteCost();
    maxRadius_ = 0.;
    childExtension_ = true;
    previousChild_ = nullptr;
}

void ompl::geometric::DIRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state_)
                si_->freeState(motion->state_);
            for (auto &gen : motion->edgeGenerators_)
                if (gen)
                    si_->freeState(gen);
            delete motion;
        }
    }

    for (auto &i : prevSolution_)
        if (i)
            si_->freeState(i);
    prevSolution_.clear();
}

ompl::geometric::DIRT::Motion *ompl::geometric::DIRT::selectNode(ompl::geometric::DIRT::Motion *sample)
{
    std::vector<Motion *> ret;
    nn_->nearestR(sample, maxRadius_, ret);
    
    std::vector<Motion *> candidates;
    for (auto &m : ret)
    {
        if (!m->inactive_ && si_->distance(m->state_, sample->state_) <= m->dirRadius_)
            candidates.push_back(m);
    }
    
    if (candidates.empty())
    {
        Motion *closest = nn_->nearest(sample);
        return closest->inactive_ ? nullptr : closest;
    }
    
    return candidates[rng_.uniformInt(0, candidates.size() - 1)];
}

double ompl::geometric::DIRT::computeDirectionalRadius(Motion *newMotion, Motion *parent,
                                                       const std::vector<Motion *> &nearby)
{
    double dirRadius = si_->distance(newMotion->state_, parent->state_);
    base::Cost newFValue = opt_->combineCosts(newMotion->accCost_, newMotion->estimatedCostToGoal_);
    
    for (auto &node : nearby)
    {
        if (node->inactive_)
            continue;
        base::Cost nodeFValue = opt_->combineCosts(node->accCost_, node->estimatedCostToGoal_);
        if (opt_->isCostBetterThan(nodeFValue, newFValue))
            dirRadius = std::min(dirRadius, si_->distance(newMotion->state_, node->state_));
    }
    
    return dirRadius;
}

ompl::base::State *ompl::geometric::DIRT::monteCarloProp(Motion *m)
{
    base::State *xstate = si_->allocState();
    sampler_->sampleUniform(xstate);
    
    double step = rng_.uniformReal(0, maxDistance_);
    double d = si_->distance(m->state_, xstate);
    si_->getStateSpace()->interpolate(m->state_, xstate, step / d, xstate);
    si_->enforceBounds(xstate);
    
    return xstate;
}

ompl::base::Cost ompl::geometric::DIRT::estimateCostToGoal(const base::State *state) const
{
    const base::Goal *goal = pdef_->getGoal().get();
    if (auto *goal_s = dynamic_cast<const base::GoalSampleableRegion *>(goal))
    {
        if (goal_s->canSample())
        {
            base::State *goalState = si_->allocState();
            goal_s->sampleGoal(goalState);
            base::Cost cost = opt_->motionCostHeuristic(state, goalState);
            si_->freeState(goalState);
            return cost;
        }
    }
    return opt_->identityCost();
}

void ompl::geometric::DIRT::pruneNode(Motion *node)
{
    if (node->inactive_)
        return;
    
    node->inactive_ = true;
    nn_->remove(node);
    
    for (auto &gen : node->edgeGenerators_)
        if (gen)
            si_->freeState(gen);
    node->edgeGenerators_.clear();
    
    if (node->numChildren_ == 0 && node->parent_)
    {
        node->parent_->numChildren_--;
        Motion *parent = node->parent_;
        while (parent && parent->inactive_ && parent->numChildren_ == 0 && parent->parent_)
        {
            parent->parent_->numChildren_--;
            parent = parent->parent_;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::DIRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state_, st);
        motion->accCost_ = opt_->identityCost();
        motion->estimatedCostToGoal_ = estimateCostToGoal(st);
        motion->dirRadius_ = 0.;
        motion->blossomNumber_ = blossomNumber_;
        nn_->add(motion);
        previousChild_ = motion;
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state_;

    while (ptc == false)
    {
        Motion *nmotion = nullptr;
        
        if (!childExtension_)
        {
            sampler_->sampleUniform(rstate);
            nmotion = selectNode(rmotion);
            if (!nmotion)
            {
                nn_->nearest(rmotion);
                nmotion = nn_->nearest(rmotion);
            }
            previousChild_ = nmotion;
        }
        else
        {
            nmotion = previousChild_;
        }
        
        childExtension_ = false;
        
        if (nmotion->edgeGenerators_.empty())
        {
            for (unsigned i = 0; i < nmotion->blossomNumber_; ++i)
                nmotion->edgeGenerators_.push_back(monteCarloProp(nmotion));
            
            std::sort(nmotion->edgeGenerators_.begin(), nmotion->edgeGenerators_.end(),
                     [this](base::State *a, base::State *b) {
                         return opt_->isCostBetterThan(estimateCostToGoal(b), estimateCostToGoal(a));
                     });
        }
        
        while (!nmotion->edgeGenerators_.empty())
        {
            base::State *dstate = nmotion->edgeGenerators_.back();
            nmotion->edgeGenerators_.pop_back();
            
            base::Cost incCost = opt_->motionCost(nmotion->state_, dstate);
            base::Cost newCost = opt_->combineCosts(nmotion->accCost_, incCost);
            base::Cost newHeuristic = estimateCostToGoal(dstate);
            base::Cost newFValue = opt_->combineCosts(newCost, newHeuristic);
            
            if (solution && opt_->isCostBetterThan(prevSolutionCost_, newFValue))
            {
                si_->freeState(dstate);
                continue;
            }
            
            std::vector<Motion *> nearby;
            double parentDist = si_->distance(nmotion->state_, dstate);
            nn_->nearestR(rmotion, std::max(parentDist, maxRadius_), nearby);
            
            double dirRadius = parentDist;
            bool dominated = false;
            
            for (auto &node : nearby)
            {
                if (node->inactive_)
                    continue;
                base::Cost nodeFValue = opt_->combineCosts(node->accCost_, node->estimatedCostToGoal_);
                if (opt_->isCostBetterThan(nodeFValue, newFValue))
                {
                    dirRadius = std::min(dirRadius, si_->distance(dstate, node->state_));
                    if (usePruning_ && dirRadius + si_->distance(dstate, node->state_) < node->dirRadius_)
                    {
                        dominated = true;
                        break;
                    }
                }
            }
            
            if (dominated)
            {
                si_->freeState(dstate);
                continue;
            }
            
            si_->copyState(rstate, dstate);
            if (!si_->checkMotion(nmotion->state_, rstate))
            {
                si_->freeState(dstate);
                continue;
            }
            
            auto *motion = new Motion(si_);
            si_->copyState(motion->state_, rstate);
            motion->parent_ = nmotion;
            motion->accCost_ = newCost;
            motion->estimatedCostToGoal_ = newHeuristic;
            motion->dirRadius_ = dirRadius;
            motion->blossomNumber_ = blossomNumber_;
            
            nmotion->numChildren_++;
            nn_->add(motion);
            maxRadius_ = std::max(maxRadius_, dirRadius);
            
            for (auto &node : nearby)
            {
                if (node->inactive_)
                    continue;
                base::Cost nodeFValue = opt_->combineCosts(node->accCost_, node->estimatedCostToGoal_);
                if (opt_->isCostBetterThan(newFValue, nodeFValue))
                {
                    double siblingDist = si_->distance(node->state_, motion->state_);
                    node->dirRadius_ = std::min(node->dirRadius_, siblingDist);
                    if (usePruning_ && node->dirRadius_ + siblingDist < dirRadius)
                        pruneNode(node);
                }
            }
            
            if (opt_->isCostBetterThan(newHeuristic, nmotion->estimatedCostToGoal_))
            {
                childExtension_ = true;
                previousChild_ = motion;
            }
            
            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state_, &dist);
            if (solv && opt_->isCostBetterThan(motion->accCost_, prevSolutionCost_))
            {
                solution = motion;
                prevSolutionCost_ = motion->accCost_;
                
                for (auto &i : prevSolution_)
                    if (i)
                        si_->freeState(i);
                prevSolution_.clear();
                
                Motion *solTrav = solution;
                while (solTrav != nullptr)
                {
                    prevSolution_.push_back(si_->cloneState(solTrav->state_));
                    solTrav = solTrav->parent_;
                }
                
                OMPL_INFORM("Found solution with cost %.2f", solution->accCost_.value());
            }
            
            si_->freeState(dstate);
            break;
        }
    }

    bool solved = solution != nullptr;
    if (solved)
    {
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = prevSolution_.size() - 1; i >= 0; --i)
            path->append(prevSolution_[i]);
        pdef_->addSolutionPath(path, false, 0.0, getName());
    }

    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, false};
}

void ompl::geometric::DIRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (prevSolution_.size() != 0)
        data.addGoalVertex(base::PlannerDataVertex(prevSolution_[0]));

    for (auto &motion : motions)
    {
        if (motion->parent_ == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state_));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent_->state_),
                         base::PlannerDataVertex(motion->state_));
    }
}
