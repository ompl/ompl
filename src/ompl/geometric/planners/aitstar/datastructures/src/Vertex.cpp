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

#include "ompl/geometric/planners/aitstar/datastructures/Vertex.h"

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
        namespace aitstar
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
                           const std::shared_ptr<std::size_t> &batchId,
                           const std::shared_ptr<std::size_t> &forwardSearchId,
                           const std::shared_ptr<std::size_t> &backwardSearchId)
              : spaceInformation_(spaceInformation)
              , problemDefinition_(problemDefinition)
              , optimizationObjective_(problemDefinition->getOptimizationObjective())
              , forwardChildren_()
              , forwardParent_()
              , state_(spaceInformation->allocState())  // The memory allocated here is freed in the destructor.
              , costToComeFromStart_(optimizationObjective_->infiniteCost())
              , edgeCostFromForwardParent_(optimizationObjective_->infiniteCost())
              , costToComeFromGoal_(optimizationObjective_->infiniteCost())
              , expandedCostToComeFromGoal_(optimizationObjective_->infiniteCost())
              , costToGoToGoal_(optimizationObjective_->infiniteCost())
              , vertexId_(generateId())
              , batchId_(batchId)
              , forwardSearchId_(forwardSearchId)
              , backwardSearchId_(backwardSearchId)
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

            ompl::base::Cost Vertex::getCostToComeFromStart() const
            {
                return costToComeFromStart_;
            }

            ompl::base::Cost Vertex::getCostToComeFromGoal() const
            {
                if (backwardSearchBatchId_ != *batchId_.lock())
                {
                    costToComeFromGoal_ = optimizationObjective_->infiniteCost();
                }
                return costToComeFromGoal_;
            }

            ompl::base::Cost Vertex::getExpandedCostToComeFromGoal() const
            {
                if (expandedBackwardSearchId_ != *backwardSearchId_.lock())
                {
                    expandedCostToComeFromGoal_ = optimizationObjective_->infiniteCost();
                }
                return expandedCostToComeFromGoal_;
            }

            ompl::base::Cost Vertex::getCostToGoToGoal() const
            {
                // // If the cost to go hasn't been set, compute it.
                // if (!std::isfinite(costToGoToGoal_.value()))
                // {
                //     auto goalPtr = problemDefinition_->getGoal();
                //     if (goalPtr->hasType(ompl::base::GOAL_STATE))
                //     {
                //         costToGoToGoal_ = optimizationObjective_->motionCostHeuristic(
                //             state_, goalPtr->as<ompl::base::GoalState>()->getState());
                //     }
                //     else if (goalPtr->hasType(ompl::base::GOAL_STATES))
                //     {
                //         for (std::size_t i = 0u; i < goalPtr->as<ompl::base::GoalStates>()->getStateCount(); ++i)
                //         {
                //             ompl::base::Cost costToGoToThis = optimizationObjective_->motionCostHeuristic(
                //                 state_, goalPtr->as<ompl::base::GoalStates>()->getState(i));
                //             if (optimizationObjective_->isCostBetterThan(costToGoToThis, costToGoToGoal_))
                //             {
                //                 costToGoToGoal_ = costToGoToThis;
                //             }
                //         }
                //     }
                //     else
                //     {
                //         auto msg =
                //             "TBDstar's OMPL implementation is limited to the goal types GOAL_STATE and
                //             GOAL_STATES."s;
                //         throw ompl::Exception(msg);
                //     }
                // }

                // if (!std::isfinite(costToComeFromGoal_.value()))
                // {
                //     return costToGoToGoal_;
                // }
                // else
                // {
                //     return costToComeFromGoal_;
                // }
                return getCostToComeFromGoal();
            }

            ompl::base::Cost Vertex::getEdgeCostFromForwardParent() const
            {
                return edgeCostFromForwardParent_;
            }

            bool Vertex::hasForwardParent() const
            {
                // See https://stackoverflow.com/questions/45507041/how-to-check-if-weak-ptr-is-empty-non-assigned.
                // return parent_.owner_before(std::weak_ptr<Vertex>{}) &&
                // std::weak_ptr<Vertex>{}.owner_before(parent_);
                return static_cast<bool>(forwardParent_.lock());
            }

            std::shared_ptr<Vertex> Vertex::getForwardParent() const
            {
                return forwardParent_.lock();
            }

            bool Vertex::hasBackwardParent() const
            {
                return static_cast<bool>(backwardParent_.lock());
            }

            std::shared_ptr<Vertex> Vertex::getBackwardParent() const
            {
                return backwardParent_.lock();
            }

            void Vertex::setForwardEdgeCost(const ompl::base::Cost &cost)
            {
                edgeCostFromForwardParent_ = cost;
            }

            void Vertex::setCostToComeFromStart(const ompl::base::Cost &cost)
            {
                costToComeFromStart_ = cost;
            }

            void Vertex::setCostToComeFromGoal(const ompl::base::Cost &cost)
            {
                backwardSearchBatchId_ = *batchId_.lock();
                costToComeFromGoal_ = cost;
            }

            void Vertex::setExpandedCostToComeFromGoal(const ompl::base::Cost &cost)
            {
                expandedCostToComeFromGoal_ = cost;
            }

            void Vertex::setCostToGoToGoal(const ompl::base::Cost &cost)
            {
                costToGoToGoal_ = cost;
            }

            void Vertex::updateCostOfForwardBranch() const
            {
                // Update the cost of all forward children.
                for (const auto &child : getForwardChildren())
                {
                    child->setCostToComeFromStart(optimizationObjective_->combineCosts(
                        costToComeFromStart_, child->getEdgeCostFromForwardParent()));
                    child->updateCostOfForwardBranch();
                }
            }

            std::vector<std::weak_ptr<aitstar::Vertex>> Vertex::invalidateBackwardBranch()
            {
                std::vector<std::weak_ptr<aitstar::Vertex>> accumulatedChildren = backwardChildren_;

                // Remove all children.
                for (const auto &child : backwardChildren_)
                {
                    child.lock()->setCostToComeFromGoal(optimizationObjective_->infiniteCost());
                    child.lock()->resetBackwardParent();
                    auto childsAccumulatedChildren = child.lock()->invalidateBackwardBranch();
                    accumulatedChildren.insert(accumulatedChildren.end(), childsAccumulatedChildren.begin(),
                                               childsAccumulatedChildren.end());
                }
                backwardChildren_.clear();

                return accumulatedChildren;
            }

            void Vertex::setForwardParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost)
            {
                // If this is a rewiring, remove from my parent's children.
                if (static_cast<bool>(forwardParent_.lock()))
                {
                    forwardParent_.lock()->removeFromForwardChildren(vertexId_);
                }

                // Remember the edge cost.
                edgeCostFromForwardParent_ = edgeCost;

                // Remember the corresponding parent.
                forwardParent_ = std::weak_ptr<Vertex>(vertex);

                // Update the cost to come.
                costToComeFromStart_ = optimizationObjective_->combineCosts(vertex->getCostToComeFromStart(), edgeCost);
            }

            void Vertex::setBackwardParent(const std::shared_ptr<Vertex> &vertex)
            {
                // If this is a rewiring, remove from my parent's children.
                if (static_cast<bool>(backwardParent_.lock()))
                {
                    backwardParent_.lock()->removeFromBackwardChildren(vertexId_);
                }

                // Remember the parent.
                backwardParent_ = std::weak_ptr<Vertex>(vertex);
            }

            void Vertex::resetBackwardParent()
            {
                backwardParent_.reset();
            }

            void Vertex::addToForwardChildren(const std::shared_ptr<Vertex> &vertex)
            {
                forwardChildren_.emplace_back(vertex);
            }

            void Vertex::removeFromForwardChildren(std::size_t vertexId)
            {
                // Find the child.
                auto it = std::find_if(
                    forwardChildren_.begin(), forwardChildren_.end(),
                    [vertexId](const std::weak_ptr<Vertex> &child) { return vertexId == child.lock()->getId(); });

                // Throw if it is not found.
                if (it == forwardChildren_.end())
                {
                    auto msg = "Asked to remove vertex from forward children that is currently not a child."s;
                    throw ompl::Exception(msg);
                }

                // Swap and pop.
                std::iter_swap(it, forwardChildren_.rbegin());
                forwardChildren_.pop_back();
            }

            void Vertex::addToBackwardChildren(const std::shared_ptr<Vertex> &vertex)
            {
                backwardChildren_.push_back(vertex);
            }

            void Vertex::removeFromBackwardChildren(std::size_t vertexId)
            {
                // Find the child.
                auto it = std::find_if(
                    backwardChildren_.begin(), backwardChildren_.end(),
                    [vertexId](const std::weak_ptr<Vertex> &child) { return vertexId == child.lock()->getId(); });

                // Throw if it is not found.
                if (it == backwardChildren_.end())
                {
                    auto msg = "Asked to remove vertex from backward children that is currently not a child."s;
                    throw ompl::Exception(msg);
                }

                // Swap and pop.
                std::iter_swap(it, backwardChildren_.rbegin());
                backwardChildren_.pop_back();
            }

            void Vertex::whitelistAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                whitelistedChildren_.emplace_back(vertex);
            }

            bool Vertex::isWhitelistedAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                for (const auto &whitelistedChild : whitelistedChildren_)
                {
                    if (whitelistedChild.lock()->getId() == vertex->getId())
                    {
                        return true;
                    }
                }
                return false;
            }

            void Vertex::blacklistAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                blacklistedChildren_.emplace_back(vertex);
            }

            bool Vertex::isBlacklistedAsChild(const std::shared_ptr<Vertex> &vertex) const
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
                neighbors_ = neighbors;
                neighborBatchId_ = *batchId_.lock();
            }

            const std::vector<std::shared_ptr<Vertex>> &Vertex::getNeighbors() const
            {
                if (neighborBatchId_ != *batchId_.lock())
                {
                    throw ompl::Exception("Requested neighbors from vertex of outdated approximation.");
                }

                return neighbors_;
            }

            std::vector<std::shared_ptr<Vertex>> Vertex::getForwardChildren() const
            {
                std::vector<std::shared_ptr<Vertex>> children;
                for (const auto &child : forwardChildren_)
                {
                    assert(!child.expired());
                    children.emplace_back(child.lock());
                }
                return children;
            }

            std::vector<std::shared_ptr<Vertex>> Vertex::getBackwardChildren() const
            {
                std::vector<std::shared_ptr<Vertex>> children;
                children.reserve(backwardChildren_.size());
                for (const auto &child : backwardChildren_)
                {
                    assert(!child.expired());
                    children.emplace_back(child.lock());
                }
                return children;
            }

            void Vertex::registerExpansionDuringForwardSearch()
            {
                expandedForwardSearchId_ = *forwardSearchId_.lock();
            }

            void Vertex::registerExpansionDuringBackwardSearch()
            {
                assert(!backwardSearchId_.expired());
                expandedCostToComeFromGoal_ = costToComeFromGoal_;
                expandedBackwardSearchId_ = *backwardSearchId_.lock();
            }

            void Vertex::registerInsertionIntoQueueDuringBackwardSearch()
            {
                assert(!backwardSearchId_.expired());
                insertedIntoQueueBackwardSearchId_ = *backwardSearchId_.lock();
            }

            bool Vertex::hasBeenExpandedDuringCurrentForwardSearch() const
            {
                assert(!forwardSearchId_.expired());
                return expandedForwardSearchId_ == *forwardSearchId_.lock();
            }

            bool Vertex::hasBeenExpandedDuringCurrentBackwardSearch() const
            {
                assert(!backwardSearchId_.expired());
                return expandedBackwardSearchId_ == *backwardSearchId_.lock();
            }

            bool Vertex::hasBeenInsertedIntoQueueDuringCurrentBackwardSearch() const
            {
                return insertedIntoQueueBackwardSearchId_ == *backwardSearchId_.lock();
            }

            void Vertex::setBackwardQueuePointer(
                typename ompl::BinaryHeap<
                    std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                    std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                       const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                    Element *pointer)
            {
                backwardQueuePointerBackwardSearchId_ = *backwardSearchId_.lock();
                backwardQueuePointer_ = pointer;
            }

            typename ompl::BinaryHeap<
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                   const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                Element *
                Vertex::getBackwardQueuePointer() const
            {
                if (*backwardSearchId_.lock() != backwardQueuePointerBackwardSearchId_)
                {
                    backwardQueuePointer_ = nullptr;
                }
                return backwardQueuePointer_;
            }

            void Vertex::resetBackwardQueuePointer()
            {
                backwardQueuePointer_ = nullptr;
            }

            void Vertex::addToForwardQueueLookup(
                typename ompl::BinaryHeap<
                    aitstar::Edge, std::function<bool(const aitstar::Edge &, const aitstar::Edge &)>>::Element *pointer)
            {
                forwardQueueLookup_.emplace_back(pointer);
            }

            /** \brief Returns the backward queue pointer of this vertex. */
            typename std::vector<ompl::BinaryHeap<
                aitstar::Edge, std::function<bool(const aitstar::Edge &, const aitstar::Edge &)>>::Element *>
            Vertex::getForwardQueueLookup() const
            {
                return forwardQueueLookup_;
            }

            void Vertex::removeFromForwardQueueLookup(
                ompl::BinaryHeap<aitstar::Edge,
                                 std::function<bool(const aitstar::Edge &, const aitstar::Edge &)>>::Element *element)
            {
                forwardQueueLookup_.erase(std::remove(forwardQueueLookup_.begin(), forwardQueueLookup_.end(), element));
            }

            void Vertex::resetForwardQueueLookup()
            {
                forwardQueueLookup_.clear();
            }

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl
