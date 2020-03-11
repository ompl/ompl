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

#pragma once

#include <memory>
#include <vector>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/datastructures/BinaryHeap.h"

#include "ompl/geometric/planners/aitstar/datastructures/Edge.h"

namespace ompl
{
    namespace geometric
    {
        namespace aitstar
        {
            class Vertex
            {
            public:
                /** \brief Constructs a vertex by sampling a state. */
                Vertex(const ompl::base::SpaceInformationPtr &spaceInformation,
                       const ompl::base::ProblemDefinitionPtr &problemDefinition,
                       const std::shared_ptr<std::size_t> &batchId, const std::shared_ptr<std::size_t> &forwardSearchId,
                       const std::shared_ptr<std::size_t> &backwardSearchId);

                /** \brief Destructs the vertex. */
                virtual ~Vertex();

                /** \brief Get the unique id of this vertex. */
                std::size_t getId() const;

                /** \brief Provides write access to the underlying state. */
                ompl::base::State *getState();

                /** \brief Provides read access to the underlying state. */
                ompl::base::State const *getState() const;

                /** \brief Returns a scoped copy of the underlying state. */
                ompl::base::ScopedState<> getScopedState() const;

                /** \brief Returns the cost to come to this vertex. */
                ompl::base::Cost getCostToComeFromStart() const;

                /** \brief Returns the cost to come to this vertex from the goal. */
                ompl::base::Cost getCostToComeFromGoal() const;

                /** \brief Returns the cost to come to this vertex from the goal when it was expanded. */
                ompl::base::Cost getExpandedCostToComeFromGoal() const;

                /** \brief Returns the cost to go heuristic from this vertex. */
                ompl::base::Cost getCostToGoToGoal() const;

                /** \brief Returns the edge cost from the forward parent. */
                ompl::base::Cost getEdgeCostFromForwardParent() const;

                /** \brief Returns whether this vertex has a parent in the forward search. */
                bool hasForwardParent() const;

                /** \brief Sets the parent vertex (in the forward-search tree). */
                void setForwardParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost);

                /** \brief Returns whether this vertex has a parent in the backward search. */
                bool hasBackwardParent() const;

                /** \brief Sets the parent vertex (in the backward-search tree). */
                void setBackwardParent(const std::shared_ptr<Vertex> &vertex);

                /** \brief Resets the backward parent of the vertex. */
                void resetBackwardParent();

                /** \brief Returns the parent of the vertex (in the forward-search tree). */
                std::shared_ptr<Vertex> getForwardParent() const;

                /** \brief Returns the parent of the vertex (in the backward-search tree). */
                std::shared_ptr<Vertex> getBackwardParent() const;

                /** \brief Sets the cost to come to this vertex. */
                void setForwardEdgeCost(const ompl::base::Cost &cost);

                /** \brief Sets the cost to come to this vertex. */
                void setCostToComeFromStart(const ompl::base::Cost &cost);

                /** \brief Sets the cost to come to this vertex from the goal. */
                void setCostToComeFromGoal(const ompl::base::Cost &cost);

                /** \brief Sets the cost to come to this vertex from the goal when it was expanded. */
                void setExpandedCostToComeFromGoal(const ompl::base::Cost &cost);

                /** \brief Sets the cost to go heuristic of this vertex. */
                void setCostToGoToGoal(const ompl::base::Cost &cost);

                /** \brief Updates the cost to the whole branch rooted at this vertex. */
                void updateCostOfForwardBranch() const;

                /** \brief Recursively invalidates the branch of the reverse tree rooted in this vertex. */
                std::vector<std::weak_ptr<aitstar::Vertex>> invalidateBackwardBranch();

                /** \brief Adds a vertex this vertex's children. */
                void addToForwardChildren(const std::shared_ptr<Vertex> &vertex);

                /** \brief Removes a vertex from this vertex's forward children. */
                void removeFromForwardChildren(std::size_t vertexId);

                /** \brief Returns this vertex's children in the forward search tree. */
                std::vector<std::shared_ptr<Vertex>> getForwardChildren() const;

                /** \brief Adds a vertex this vertex's children. */
                void addToBackwardChildren(const std::shared_ptr<Vertex> &vertex);

                /** \brief Removes a vertex from this vertex's forward children. */
                void removeFromBackwardChildren(std::size_t vertexId);

                /** \brief Returns this vertex's children in the backward search tree. */
                std::vector<std::shared_ptr<Vertex>> getBackwardChildren() const;

                /** \brief Blacklists a child. */
                void whitelistAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns whether a child is blacklisted. */
                bool isWhitelistedAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Blacklists a child. */
                void blacklistAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns whether a child is blacklisted. */
                bool isBlacklistedAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns whether the vertex knows its nearest neighbors on the current approximation. */
                bool hasCachedNeighbors() const;

                /** \brief Caches the neighbors for the current approximation. */
                void cacheNeighbors(const std::vector<std::shared_ptr<Vertex>> &neighbors) const;

                /** \brief Returns the nearest neighbors, throws if not up to date. */
                const std::vector<std::shared_ptr<Vertex>> &getNeighbors() const;

                /** \brief Registers the expansion of this vertex during the current forward search. */
                void registerExpansionDuringForwardSearch();

                /** \brief Registers the expansion of this vertex during the current backward search. */
                void registerExpansionDuringBackwardSearch();

                /** \brief Registers the insertion of this vertex into the open queue during the current backward
                 * search. */
                void registerInsertionIntoQueueDuringBackwardSearch();

                /** \brief Returns whether the vertex has been expanded during the current forward search. */
                bool hasBeenExpandedDuringCurrentForwardSearch() const;

                /** \brief Returns whether the vertex has been expanded during the current backward search. */
                bool hasBeenExpandedDuringCurrentBackwardSearch() const;

                /** \brief Returns whether the vertex has been inserted into the queue during the current backward
                 * search. */
                bool hasBeenInsertedIntoQueueDuringCurrentBackwardSearch() const;

                /** \brief Sets the backward queue pointer of this vertex. */
                void setBackwardQueuePointer(
                    typename ompl::BinaryHeap<
                        std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                        std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                           const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>
                                               &)>>::Element *pointer);

                /** \brief Returns the backward queue pointer of this vertex. */
                typename ompl::BinaryHeap<
                    std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                    std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                       const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                    Element *
                    getBackwardQueuePointer() const;

                /** \brief Resets the backward queue pointer. */
                void resetBackwardQueuePointer();

                /** \brief Adds an element to the forward queue lookup. */
                void addToForwardQueueLookup(
                    typename ompl::BinaryHeap<
                        aitstar::Edge, std::function<bool(const aitstar::Edge &, const aitstar::Edge &)>>::Element
                        *pointer);

                /** \brief Returns the backward queue pointer of this vertex. */
                std::vector<ompl::BinaryHeap<
                    aitstar::Edge, std::function<bool(const aitstar::Edge &, const aitstar::Edge &)>>::Element *>
                getForwardQueueLookup() const;

                /** \brief Remove an element from the forward queue lookup. */
                void removeFromForwardQueueLookup(
                    ompl::BinaryHeap<aitstar::Edge,
                                     std::function<bool(const aitstar::Edge &, const aitstar::Edge &)>>::Element
                        *element);

                /** \brief Resets the forward queue lookup. */
                void resetForwardQueueLookup();

            private:
                /** \brief The space information of the planning problem. */
                const ompl::base::SpaceInformationPtr spaceInformation_;

                /** \brief The definition of the planning problem. */
                const ompl::base::ProblemDefinitionPtr problemDefinition_;

                /** \brief The optimization objective of the planning problem. */
                const ompl::base::OptimizationObjectivePtr optimizationObjective_;

                /** \brief The children of this vertex in the forward search tree. */
                std::vector<std::weak_ptr<Vertex>> forwardChildren_{};

                /** \brief The children of this vertex in the backward search tree. */
                std::vector<std::weak_ptr<Vertex>> backwardChildren_{};

                /** \brief The cached neighbors of this vertex. */
                mutable std::vector<std::shared_ptr<Vertex>> neighbors_{};

                /** \brief The list of chind. */
                mutable std::vector<std::weak_ptr<Vertex>> whitelistedChildren_{};

                /** \brief The list of chegel. */
                mutable std::vector<std::weak_ptr<Vertex>> blacklistedChildren_{};

                /** \brief The parent of this vertex. */
                std::weak_ptr<Vertex> forwardParent_;

                /** \brief The parent of this vertex. */
                std::weak_ptr<Vertex> backwardParent_;

                /** \brief The state associated with this vertex. */
                ompl::base::State *state_;

                /** \brief The cost to come to this vertex. */
                ompl::base::Cost costToComeFromStart_;

                /** \brief The edge cost from the parent. */
                ompl::base::Cost edgeCostFromForwardParent_;

                /** \brief The backward cost to come. */
                mutable ompl::base::Cost costToComeFromGoal_;

                /** \brief The cost to come from the goal when this vertex was expanded. */
                mutable ompl::base::Cost expandedCostToComeFromGoal_;

                /** \brief The cost to go estimate for this vertex. */
                mutable ompl::base::Cost costToGoToGoal_;

                /** \brief The unique id of this vertex. */
                const std::size_t vertexId_;

                /** \brief The id of the most recent batch. */
                const std::weak_ptr<const std::size_t> batchId_;

                /** \brief The id of the current forward search. */
                const std::weak_ptr<const std::size_t> forwardSearchId_;

                /** \brief The id of the current backward search. */
                const std::weak_ptr<const std::size_t> backwardSearchId_;

                /** \brief The batch id for which the cached neighbor list is valid. */
                mutable std::size_t neighborBatchId_{0u};

                /** \brief The batch id for which the backward search cost to come is valid. */
                mutable std::size_t backwardSearchBatchId_{0u};

                /** \brief The forward search id this vertex has last been expanded on. */
                mutable std::size_t expandedForwardSearchId_{0u};

                /** \brief The backward search id this vertex has last been expanded on. */
                mutable std::size_t expandedBackwardSearchId_{0u};

                /** \brief The backward search id this vertex has last been inserted into open on. */
                mutable std::size_t insertedIntoQueueBackwardSearchId_{0u};

                /** \brief The backward search id for which the backward queue pointer is valid. */
                mutable std::size_t backwardQueuePointerBackwardSearchId_{0u};

                /** \brief The pointer to the backward queue element. */
                mutable typename ompl::BinaryHeap<
                    std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                    std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                       const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                    Element *backwardQueuePointer_{nullptr};

                /** \brief The lookup to outgoing edges in the forward queue. */
                mutable std::vector<ompl::BinaryHeap<
                    aitstar::Edge, std::function<bool(const aitstar::Edge &, const aitstar::Edge &)>>::Element *>
                    forwardQueueLookup_;
            };

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl
