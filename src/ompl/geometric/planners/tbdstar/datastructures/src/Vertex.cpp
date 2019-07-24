/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019-present University of Oxford
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
 *   * Neither the names of the copyright holders nor the names of its
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

// Authors: Marlin Strub

#include "ompl/geometric/planners/tbdstar/datastructures/Vertex.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <string>

#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"

using namespace std::string_literals;

namespace ompl
{
    namespace geometric
    {
        namespace tbdstar
        {
            namespace
            {
                std::size_t generateId()
                {
                    static std::atomic<std::size_t> id{0u};
                    return id++;
                }
            }  // namespace

            Vertex::Vertex(const ompl::base::SpaceInformationPtr &spaceInformation,
                           const ompl::base::ProblemDefinitionPtr &problemDefinition,
                           const std::shared_ptr<std::size_t> &batchId, const std::shared_ptr<std::size_t> &searchId)
              : spaceInformation_(spaceInformation)
              , problemDefinition_(problemDefinition)
              , optimizationObjective_(problemDefinition->getOptimizationObjective())
              , children_()
              , parent_()
              , state_(spaceInformation->allocState())  // This allocates memory for a state.
              , costToCome_(std::numeric_limits<double>::infinity())
              , costToComeFromGoal_(std::numeric_limits<double>::infinity())
              , costToGo_(std::numeric_limits<double>::infinity())
              , vertexId_(generateId())
              , batchId_(batchId)
              , searchId_(searchId)
            {
            }

            Vertex::~Vertex()
            {
                // The state has associated memory that needs to be freed manually.
                spaceInformation_->freeState(state_);
            };

            std::size_t Vertex::getId() const
            {
                return vertexId_;
            }

            ompl::base::State *Vertex::getState()
            {
                return state_;
            }

            ompl::base::State const *Vertex::getState() const
            {
                return state_;
            }

            ompl::base::ScopedState<> Vertex::getScopedState() const
            {
                return ompl::base::ScopedState<>(spaceInformation_->getStateSpace(), state_);
            }

            ompl::base::Cost Vertex::getCostToCome() const
            {
                return costToCome_;
            }

            ompl::base::Cost Vertex::getCostToComeFromGoal() const
            {
                if (backwardSearchBatchId_ != *batchId_.lock())
                {
                    costToComeFromGoal_ = optimizationObjective_->infiniteCost();
                    backwardSearchBatchId_ = *batchId_.lock();
                }
                return costToComeFromGoal_;
            }

            ompl::base::Cost Vertex::getCostToGo() const
            {
                // If the cost to go hasn't been set, compute it.
                if (!std::isfinite(costToGo_.value()))
                {
                    auto goalPtr = problemDefinition_->getGoal();
                    if (goalPtr->hasType(ompl::base::GOAL_STATE))
                    {
                        costToGo_ = optimizationObjective_->motionCostHeuristic(
                            state_, goalPtr->as<ompl::base::GoalState>()->getState());
                    }
                    else if (goalPtr->hasType(ompl::base::GOAL_STATES))
                    {
                        for (std::size_t i = 0u; i < goalPtr->as<ompl::base::GoalStates>()->getStateCount(); ++i)
                        {
                            ompl::base::Cost costToGoToThis = optimizationObjective_->motionCostHeuristic(
                                state_, goalPtr->as<ompl::base::GoalStates>()->getState(i));
                            if (optimizationObjective_->isCostBetterThan(costToGoToThis, costToGo_))
                            {
                                costToGo_ = costToGoToThis;
                            }
                        }
                    }
                    else
                    {
                        auto msg =
                            "TBDstar's OMPL implementation is limited to the goal types GOAL_STATE and GOAL_STATES."s;
                        throw ompl::Exception(msg);
                    }
                }

                if (!std::isfinite(costToComeFromGoal_.value()))
                {
                    return costToGo_;
                }
                else
                {
                    return costToComeFromGoal_;
                }
            }

            bool Vertex::hasParent() const
            {
                // See https://stackoverflow.com/questions/45507041/how-to-check-if-weak-ptr-is-empty-non-assigned.
                // return parent_.owner_before(std::weak_ptr<Vertex>{}) &&
                // std::weak_ptr<Vertex>{}.owner_before(parent_);
                return static_cast<bool>(parent_.lock());
            }

            std::shared_ptr<Vertex> Vertex::getParent() const
            {
                return parent_.lock();
            }

            void Vertex::setCostToCome(const ompl::base::Cost &cost)
            {
                costToCome_ = cost;
            }

            void Vertex::setCostToComeFromGoal(const ompl::base::Cost &cost)
            {
                backwardSearchBatchId_ = *batchId_.lock();
                costToComeFromGoal_ = cost;
            }

            void Vertex::setCostToGo(const ompl::base::Cost &cost)
            {
                costToGo_ = cost;
            }

            void Vertex::setParent(const std::shared_ptr<Vertex> &vertex)
            {
                parent_ = std::weak_ptr<Vertex>(vertex);
            }

            void Vertex::addToChildren(const std::shared_ptr<Vertex> &vertex)
            {
                children_.emplace_back(vertex);
            }

            void Vertex::addToChildren(const std::vector<std::shared_ptr<Vertex>> &vertices)
            {
                children_.insert(children_.end(), vertices.begin(), vertices.end());
            }

            void Vertex::removeFromChildren(const std::shared_ptr<Vertex> &vertex)
            {
                // Find the child.
                auto it =
                    std::find_if(children_.begin(), children_.end(), [vertex](const std::weak_ptr<Vertex> &child) {
                        return vertex->getId() == child.lock()->getId();
                    });

                // Throw if it is not found.
                if (it == children_.end())
                {
                    auto msg = "Asked to remove vertex from children that is currently not a child."s;
                    throw ompl::Exception(msg);
                }

                // Swap and pop.
                std::iter_swap(it, children_.rbegin());
                children_.pop_back();
            }

            void Vertex::blacklistAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                blacklistedChildren_.emplace_back(vertex);
            }

            bool Vertex::isChildBlacklisted(const std::shared_ptr<Vertex> &vertex) const
            {
                for (const auto &blacklistedChild : blacklistedChildren_)
                {
                    if (blacklistedChild.lock()->getId() == vertex->getId())
                    {
                        return true;
                    }
                }
                return false;
            }

            bool Vertex::hasCachedNeighbors() const
            {
                return neighborBatchId_ == *batchId_.lock();
            }

            void Vertex::cacheNeighbors(const std::vector<std::shared_ptr<Vertex>> &neighbors) const
            {
                neighbors_.clear();
                neighbors_.insert(neighbors_.begin(), neighbors.begin(), neighbors.end());
                neighborBatchId_ = *batchId_.lock();
            }

            std::vector<std::shared_ptr<Vertex>> Vertex::getNeighbors() const
            {
                if (neighborBatchId_ != *batchId_.lock())
                {
                    throw ompl::Exception("Requested neighbors from vertex of outdated approximation.");
                }

                std::vector<std::shared_ptr<Vertex>> neighbors;
                for (const auto &neighbor : neighbors_)
                {
                    neighbors.emplace_back(neighbor.lock());
                }
                return neighbors;
            }

            std::vector<std::shared_ptr<Vertex>> Vertex::getChildren() const
            {
                std::vector<std::shared_ptr<Vertex>> children;
                for (const auto &child : children_)
                {
                    children.emplace_back(child.lock());
                }
                return children;
            }

            bool Vertex::hasBeenExpandedDuringCurrentSearch() const
            {
                return expandedSearchId_ == *searchId_.lock();
            }

            void Vertex::registerExpansionDuringCurrentSearch()
            {
                expandedSearchId_ = *searchId_.lock();
            }

        }  // namespace tbdstar

    }  // namespace geometric

}  // namespace ompl
