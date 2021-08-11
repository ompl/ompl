/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR_VERTEX_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR_VERTEX_

#include <memory>
#include <vector>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/datastructures/BinaryHeap.h"

#include "ompl/geometric/planners/informedtrees/aitstar/Edge.h"
#include "ompl/geometric/planners/informedtrees/aitstar/Queuetypes.h"

namespace ompl
{
    namespace geometric
    {
        namespace aitstar
        {
            class Vertex : public std::enable_shared_from_this<Vertex>
            {
            public:
                /** \brief Constructs a vertex by sampling a state. */
                Vertex(const ompl::base::SpaceInformationPtr &spaceInformation,
                       const ompl::base::ProblemDefinitionPtr &problemDefinition, const std::size_t &batchId);

                /** \brief Constructs a copy of another vertex. */
                explicit Vertex(const std::shared_ptr<Vertex> &other);

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

                /** \brief Returns the cost to come to this vertex from the start. */
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

                /** \brief Resets the forward parent of the vertex. */
                void resetForwardParent();

                /** \brief Returns whether this vertex has a parent in the reverse search. */
                bool hasReverseParent() const;

                /** \brief Sets the parent vertex (in the reverse-search tree). */
                void setReverseParent(const std::shared_ptr<Vertex> &vertex);

                /** \brief Resets the reverse parent of the vertex. */
                void resetReverseParent();

                /** \brief Returns the parent of the vertex (in the forward-search tree). */
                std::shared_ptr<Vertex> getForwardParent() const;

                /** \brief Returns the parent of the vertex (in the reverse-search tree). */
                std::shared_ptr<Vertex> getReverseParent() const;

                /** \brief Sets the cost to come to this vertex. */
                void setForwardEdgeCost(const ompl::base::Cost &cost);

                /** \brief Sets the cost to come to this vertex. */
                void setCostToComeFromStart(const ompl::base::Cost &cost);

                /** \brief Sets the cost to come to this vertex from the goal. */
                void setCostToComeFromGoal(const ompl::base::Cost &cost);

                /** \brief Resets the cost to come to this vertex from the goal to infinity. */
                void resetCostToComeFromGoal();

                /** \brief Resets the expanded cost to come to this vertex from the goal to infinity. */
                void resetExpandedCostToComeFromGoal();

                /** \brief Sets the cost to come to this vertex from the goal when it was expanded. */
                void setExpandedCostToComeFromGoal(const ompl::base::Cost &cost);

                /** \brief Sets the cost to go heuristic of this vertex. */
                void setCostToGoToGoal(const ompl::base::Cost &cost);

                /** \brief Updates the cost to the whole branch rooted at this vertex. */
                void updateCostOfForwardBranch() const;

                /** \brief Recursively invalidates the branch of the reverse tree rooted in this vertex. */
                std::vector<std::weak_ptr<Vertex>> invalidateReverseBranch();

                /** \brief Recursively invalidates the branch of the forward tree rooted in this vertex. */
                std::vector<std::weak_ptr<Vertex>> invalidateForwardBranch();

                /** \brief Adds a vertex to this vertex's forward children. */
                void addToForwardChildren(const std::shared_ptr<Vertex> &vertex);

                /** \brief Removes a vertex from this vertex's forward children. */
                void removeFromForwardChildren(std::size_t vertexId);

                /** \brief Returns this vertex's children in the forward search tree. */
                std::vector<std::shared_ptr<Vertex>> getForwardChildren() const;

                /** \brief Adds a vertex this vertex's children. */
                void addToReverseChildren(const std::shared_ptr<Vertex> &vertex);

                /** \brief Removes a vertex from this vertex's forward children. */
                void removeFromReverseChildren(std::size_t vertexId);

                /** \brief Returns this vertex's children in the reverse search tree. */
                std::vector<std::shared_ptr<Vertex>> getReverseChildren() const;

                /** \brief Whitelists a child. */
                void whitelistAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns whether a child is whitelisted. */
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
                const std::vector<std::shared_ptr<Vertex>> getNeighbors() const;

                /** \brief Returns whether the vertex is consistent, i.e., whether its cost-to-come is equal to the
                 * cost-to-come when it was last expanded. */
                bool isConsistent() const;

                /** \brief Sets the reverse queue pointer of this vertex. */
                void setReverseQueuePointer(typename VertexQueue::Element *pointer);

                /** \brief Returns the reverse queue pointer of this vertex. */
                typename VertexQueue::Element *getReverseQueuePointer() const;

                /** \brief Resets the reverse queue pointer. */
                void resetReverseQueuePointer();

                /** \brief Adds an element to the forward queue incoming lookup. */
                void addToForwardQueueIncomingLookup(typename EdgeQueue::Element *pointer);

                /** \brief Adds an element to the forward queue outgoing lookup. */
                void addToForwardQueueOutgoingLookup(typename EdgeQueue::Element *pointer);

                /** \brief Returns the forward queue incoming lookup of this vertex. */
                std::vector<EdgeQueue::Element *> getForwardQueueIncomingLookup() const;

                /** \brief Returns the forward queue outgoing lookup of this vertex. */
                std::vector<EdgeQueue::Element *> getForwardQueueOutgoingLookup() const;

                /** \brief Remove an element from the incoming queue lookup. */
                void removeFromForwardQueueIncomingLookup(typename EdgeQueue::Element *element);

                /** \brief Remove an element from the outgoing queue lookup. */
                void removeFromForwardQueueOutgoingLookup(typename EdgeQueue::Element *element);

                /** \brief Resets the forward queue incoming lookup. */
                void resetForwardQueueIncomingLookup();

                /** \brief Resets the forward queue outgoing lookup. */
                void resetForwardQueueOutgoingLookup();

                /** \brief Calls the given function on this vertex and all of its children. */
                void callOnForwardBranch(const std::function<void(const std::shared_ptr<Vertex> &)> &function);

                /** \brief Calls the given function on this vertex and all of its children. */
                void callOnReverseBranch(const std::function<void(const std::shared_ptr<Vertex> &)> &function);

            private:
                /** \brief The space information of the planning problem. */
                const ompl::base::SpaceInformationPtr spaceInformation_;

                /** \brief The definition of the planning problem. */
                const ompl::base::ProblemDefinitionPtr problemDefinition_;

                /** \brief The optimization objective of the planning problem. */
                const ompl::base::OptimizationObjectivePtr objective_;

                /** \brief The children of this vertex in the forward search tree. */
                std::vector<std::weak_ptr<Vertex>> forwardChildren_{};

                /** \brief The children of this vertex in the reverse search tree. */
                std::vector<std::weak_ptr<Vertex>> reverseChildren_{};

                /** \brief The cached neighbors of this vertex. */
                mutable std::vector<std::weak_ptr<Vertex>> neighbors_{};

                /** \brief The list of whitelisted children. */
                mutable std::vector<std::weak_ptr<Vertex>> whitelistedChildren_{};

                /** \brief The list of blacklisted children. */
                mutable std::vector<std::weak_ptr<Vertex>> blacklistedChildren_{};

                /** \brief The parent of this vertex in the forward search tree. */
                std::weak_ptr<Vertex> forwardParent_;

                /** \brief The parent of this vertex in the reverse search tree. */
                std::weak_ptr<Vertex> reverseParent_;

                /** \brief The state associated with this vertex. */
                ompl::base::State *state_;

                /** \brief The cost to come to this vertex. */
                ompl::base::Cost costToComeFromStart_;

                /** \brief The edge cost from the parent. */
                ompl::base::Cost edgeCostFromForwardParent_;

                /** \brief The reverse cost to come. */
                mutable ompl::base::Cost costToComeFromGoal_;

                /** \brief The cost to come from the goal when this vertex was expanded. */
                mutable ompl::base::Cost expandedCostToComeFromGoal_;

                /** \brief The cost to go estimate for this vertex. */
                mutable ompl::base::Cost costToGoToGoal_;

                /** \brief The unique id of this vertex. */
                const std::size_t vertexId_;

                /** \brief The id of the most recent batch. */
                const std::size_t &batchId_;

                /** \brief The batch id for which the cached neighbor list is valid. */
                mutable std::size_t neighborBatchId_{0u};

                /** \brief The reverse search id for which the reverse queue pointer is valid. */
                mutable std::size_t reverseQueuePointerId_{0u};

                /** \brief The pointer to the reverse queue element. */
                mutable typename VertexQueue::Element *reverseQueuePointer_{nullptr};

                /** \brief The lookup to incoming edges in the forward queue. */
                mutable std::vector<EdgeQueue::Element *> forwardQueueIncomingLookup_;

                /** \brief The lookup to outgoing edges in the forward queue. */
                mutable std::vector<EdgeQueue::Element *> forwardQueueOutgoingLookup_;
            };

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR_VERTEX_
