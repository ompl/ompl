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

namespace ompl
{
    namespace geometric
    {
        namespace tbdstar
        {
            class Vertex
            {
            public:
                /** \brief Constructs a vertex by sampling a state. */
                Vertex(const ompl::base::SpaceInformationPtr &spaceInformation,
                       const ompl::base::ProblemDefinitionPtr &problemDefinition,
                       const std::shared_ptr<std::size_t> &batchId, const std::shared_ptr<std::size_t> &searchId);

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
                ompl::base::Cost getCostToCome() const;

                /** \brief Returns the cost to come to this vertex from the goal. */
                ompl::base::Cost getCostToComeFromGoal() const;

                /** \brief Returns the cost to go heuristic from this vertex. */
                ompl::base::Cost getCostToGo() const;

                /** \brief Returns whether this vertex has a parent. */
                bool hasParent() const;

                /** \brief Sets the parent vertex (in the forward-search tree). */
                void setParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost);

                /** \brief Returns the parent of the vertex (in the forward-search tree). */
                std::shared_ptr<Vertex> getParent() const;

                /** \brief Sets the cost to come to this vertex. */
                void setEdgeCost(const ompl::base::Cost &cost);

                /** \brief Sets the cost to come to this vertex. */
                void setCostToCome(const ompl::base::Cost &cost);

                /** \brief Sets the cost to come to this vertex from the goal. */
                void setCostToComeFromGoal(const ompl::base::Cost &cost);

                /** \brief Sets the cost to go heuristic of this vertex. */
                void setCostToGo(const ompl::base::Cost &cost);

                /** \brief Updates the cost to the whole branch rooted at this vertex. */
                void updateCostOfBranch() const;

                /** \brief Adds a vertex this vertex's children. */
                void addToChildren(const std::shared_ptr<Vertex> &vertex);

                /** \brief Adds multiple vertices to this vertex's children. */
                void addToChildren(const std::vector<std::shared_ptr<Vertex>> &vertices);

                /** \brief Removes a vertex from this vertex's children. */
                void removeFromChildren(const std::shared_ptr<Vertex> &vertex);

                /** \brief Blacklists a child. */
                void whitelistAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns whether a child is blacklisted. */
                bool isChildWhitelisted(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Blacklists a child. */
                void blacklistAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns whether a child is blacklisted. */
                bool isChildBlacklisted(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns whether the vertex knows its nearest neighbors on the current approximation. */
                bool hasCachedNeighbors() const;

                /** \brief Caches the neighbors for the current approximation. */
                void cacheNeighbors(const std::vector<std::shared_ptr<Vertex>> &neighbors) const;

                /** \brief Returns the nearest neighbors, throws if not up to date. */
                std::vector<std::shared_ptr<Vertex>> getNeighbors() const;

                /** \brief Returns this vertex's children. */
                std::vector<std::shared_ptr<Vertex>> getChildren() const;

                /** \brief Returns whether the vertex has been expanded during the current search. */
                bool hasBeenExpandedDuringCurrentSearch() const;

                /** \brief Registers the expansion of this vertex. */
                void registerExpansionDuringCurrentSearch();

            private:
                /** \brief The space information of the planning problem. */
                const ompl::base::SpaceInformationPtr spaceInformation_;

                /** \brief The definition of the planning problem. */
                const ompl::base::ProblemDefinitionPtr problemDefinition_;

                /** \brief The optimization objective of the planning problem. */
                const ompl::base::OptimizationObjectivePtr optimizationObjective_;

                /** \brief The children of this vertex. */
                std::vector<std::weak_ptr<Vertex>> children_{};

                /** \brief The cached neighbors of this vertex. */
                mutable std::vector<std::weak_ptr<Vertex>> neighbors_{};

                /** \brief The list of chind. */
                mutable std::vector<std::weak_ptr<Vertex>> whitelistedChildren_{};

                /** \brief The list of chegel. */
                mutable std::vector<std::weak_ptr<Vertex>> blacklistedChildren_{};

                /** \brief The parent of this vertex. */
                std::weak_ptr<Vertex> parent_;

                /** \brief The state associated with this vertex. */
                ompl::base::State *state_;

                /** \brief The cost to come to this vertex. */
                ompl::base::Cost costToCome_;

                /** \brief The edge cost from the parent. */
                ompl::base::Cost edgeCost_;

                /** \brief The backward cost to come. */
                mutable ompl::base::Cost costToComeFromGoal_;

                /** \brief The cost to go estimate for this vertex. */
                mutable ompl::base::Cost costToGo_;

                /** \brief The unique id of this vertex. */
                const std::size_t vertexId_;

                /** \brief The id of the most recent batch. */
                const std::weak_ptr<const std::size_t> batchId_;

                /** \brief The id of the current search. */
                const std::weak_ptr<const std::size_t> searchId_;

                /** \brief The batch id for which the cached neighbor list is valid. */
                mutable std::size_t neighborBatchId_{0u};

                /** \brief The batch id for which the backward search cost to go is valid. */
                mutable std::size_t backwardSearchBatchId_{0u};

                /** \brief The search id this vertex has last been expanded on. */
                mutable std::size_t expandedSearchId_{0u};
            };

        }  // namespace tbdstar

    }  // namespace geometric

}  // namespace ompl
