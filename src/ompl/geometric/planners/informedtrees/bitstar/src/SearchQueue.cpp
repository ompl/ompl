/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Toronto
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
 *   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell, Marlin Strub */

// My definition:
#include "ompl/geometric/planners/informedtrees/bitstar/SearchQueue.h"

// For std::lexicographical_compare and the std::*_heap functions.
#include <algorithm>
// For std::advance.
#include <iterator>
// For std::move.
#include <utility>

// OMPL:
// For OMPL_INFORM et al.
#include "ompl/util/Console.h"
// For exceptions:
#include "ompl/util/Exception.h"

// BIT*:
// A collection of common helper functions
#include "ompl/geometric/planners/informedtrees/bitstar/HelperFunctions.h"
// The vertex class:
#include "ompl/geometric/planners/informedtrees/bitstar/Vertex.h"
// The cost-helper class:
#include "ompl/geometric/planners/informedtrees/bitstar/CostHelper.h"
// The implicit graph:
#include "ompl/geometric/planners/informedtrees/bitstar/ImplicitGraph.h"

// Debug macros
#ifdef BITSTAR_DEBUG
/** \brief A debug-only call to assert that the object is setup. */
#define ASSERT_SETUP this->assertSetup();
#else
#define ASSERT_SETUP
#endif  // BITSTAR_DEBUG

using namespace std::string_literals;

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::SearchQueue::SearchQueue(NameFunc nameFunc)
          : nameFunc_(std::move(nameFunc))
          , edgeQueue_([this](const SortKeyAndVertexPtrPair &lhs, const SortKeyAndVertexPtrPair &rhs) {
              return lexicographicalBetterThan(lhs.first, rhs.first);
          })  // This tells the edgeQueue_ to use lexicographical comparison for sorting.
          , searchId_(std::make_shared<unsigned int>(1u))
        {
        }

        void BITstar::SearchQueue::setup(CostHelper *costHelpPtr, ImplicitGraph *graphPtr)
        {
            // Store that I am setup.
            isSetup_ = true;

            // Get access to the cost helper and the graph.
            costHelpPtr_ = costHelpPtr;
            graphPtr_ = graphPtr;

            // Set the the cost threshold to infinity to start.
            solutionCost_ = costHelpPtr_->infiniteCost();
        }

        void BITstar::SearchQueue::reset()
        {
            // Reset everything to the state of construction except for the planner name.
            // Keep this in the order of the constructors for easy verification.

            // The queue is not ready for handling data after resetting.
            isSetup_ = false;

            // Reset the pointers to external information.
            costHelpPtr_ = nullptr;
            graphPtr_ = nullptr;

            // Clear the queue.
            edgeQueue_.clear();

            // Clear the set of inconsistent vertices.
            inconsistentVertices_.clear();

            // Reset the inflation factor.
            inflationFactor_ = 1.0;

            // Reset the number of queues that have been searched.
            *searchId_ = 1u;

            // Reset the cost threshold to infinite cost.
            solutionCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());

            // Make sure the queue doesn't think it has a solution.
            hasExactSolution_ = false;

            // Finally, reset the progress info.
            numEdgesPopped_ = 0u;
        }

        void BITstar::SearchQueue::enableCascadingRewirings(bool enable)
        {
            isCascadingOfRewiringsEnabled_ = enable;
        }

        void BITstar::SearchQueue::enqueueEdge(const VertexPtrPair &edge)
        {
            ASSERT_SETUP

            // Create convenience aliases.
            const VertexPtr &parent = edge.first;
            const VertexPtr &child = edge.second;

            // If we already have the edge in the queue, we need to update its value.
            EdgeQueueElemPtr updateEdge = nullptr;
            for (auto it = child->edgeQueueInLookupConstBegin(); it != child->edgeQueueInLookupConstEnd(); ++it)
            {
                if ((*it)->data.second.first->getId() == parent->getId())
                {
                    updateEdge = (*it);
                    break;
                }
            }

            if (updateEdge)  // The edge is already in the queue, we need to update it's sort key (presumably because
                             // the cost-to-come to the source changed).
            {
#ifdef BITSTAR_DEBUG
                if (updateEdge->data.second.first->getId() != edge.first->getId() ||
                    updateEdge->data.second.second->getId() != edge.second->getId())
                {
                    throw ompl::Exception("Updating the wrong edge.");
                }
#endif  // BITSTAR_DEBUG
                updateEdge->data.first = this->createSortKey(edge);
                edgeQueue_.update(updateEdge);
            }
            else  // This edge is not yet in the queue.
            {
                // The iterator to the new edge in the queue:
                EdgeQueueElemPtr edgeElemPtr;

                // Insert into the edge queue, getting the element pointer
                edgeElemPtr = edgeQueue_.insert(std::make_pair(this->createSortKey(edge), edge));

                // Push the newly created edge back on the vector of edges from the parent.
                parent->insertInEdgeQueueOutLookup(edgeElemPtr);

                // Push the newly created edge back on the vector of edges to the child.
                child->insertInEdgeQueueInLookup(edgeElemPtr);
            }
        }

        BITstar::VertexPtrPair BITstar::SearchQueue::getFrontEdge()
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (edgeQueue_.empty())
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Return the a copy of the front edge.
            return edgeQueue_.top()->data.second;
        }

        BITstar::SearchQueue::SortKey BITstar::SearchQueue::getFrontEdgeValue()
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (edgeQueue_.empty())
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Return a copy of the front value.
            return edgeQueue_.top()->data.first;
        }

        BITstar::VertexPtrPair BITstar::SearchQueue::popFrontEdge()
        {
            ASSERT_SETUP
#ifdef BITSTAR_DEBUG
            if (edgeQueue_.empty())
            {
                throw ompl::Exception("Attempted to pop an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Increment the counter of popped edges.
            ++numEdgesPopped_;

            // Get the front element in the edge queue.
            EdgeQueueElemPtr frontEdgeQueueElement = edgeQueue_.top();
            VertexPtrPair frontEdge = frontEdgeQueueElement->data.second;

#ifdef BITSTAR_DEBUG
            if (frontEdge.first->isPruned() || frontEdge.second->isPruned())
            {
                throw ompl::Exception("The edge queue contains an edge with a pruned vertex.");
            }
#endif  // BITSTAR_DEBUG

            // Remove the edge from the respective vertex lookups.
            frontEdge.first->removeFromEdgeQueueOutLookup(frontEdgeQueueElement);
            frontEdge.second->removeFromEdgeQueueInLookup(frontEdgeQueueElement);

            // Remove it from the queue.
            edgeQueue_.pop();

            // Return the edge.
            return frontEdge;
        }

        void BITstar::SearchQueue::registerSolutionCost(const ompl::base::Cost &solutionCost)
        {
            ASSERT_SETUP

            // Flag
            hasExactSolution_ = true;

            // Store
            solutionCost_ = solutionCost;
        }

        void BITstar::SearchQueue::removeInEdgesConnectedToVertexFromQueue(const VertexPtr &vertex)
        {
            ASSERT_SETUP

            if (!edgeQueue_.empty())
            {
                // Iterate over the vector of incoming edges to this vertex and remove them from the queue (and clean up
                // their other lookup).
                for (auto it = vertex->edgeQueueInLookupConstBegin(); it != vertex->edgeQueueInLookupConstEnd(); ++it)
                {
                    // Remove the edge from the *other* lookup (by value since this is NOT an iter to THAT container).
                    // No need to remove from this lookup, as that's being cleared.
                    (*it)->data.second.first->removeFromEdgeQueueOutLookup(*it);

                    // Finally remove it from the queue
                    edgeQueue_.remove(*it);
                }

                // Clear the list.
                vertex->clearEdgeQueueInLookup();
            }
            // No else, nothing to remove from.
        }

        void BITstar::SearchQueue::removeOutEdgesConnectedToVertexFromQueue(const VertexPtr &vertex)
        {
            ASSERT_SETUP

            if (!edgeQueue_.empty())
            {
                // Iterate over the vector of outgoing edges to this vertex and remove them from the queue (and clean up
                // their other lookup).
                for (auto it = vertex->edgeQueueOutLookupConstBegin(); it != vertex->edgeQueueOutLookupConstEnd(); ++it)
                {
                    // Remove the edge from the *other* lookup (by value since this is NOT an iter to THAT container).
                    // No need to remove from this lookup, as that's being cleared.
                    (*it)->data.second.second->removeFromEdgeQueueInLookup(*it);

                    // Finally, remove it from the queue.
                    edgeQueue_.remove(*it);
                }

                // Clear the list.
                vertex->clearEdgeQueueOutLookup();
            }
            // No else, nothing to remove from.
        }

        void BITstar::SearchQueue::removeAllEdgesConnectedToVertexFromQueue(const VertexPtr &vertex)
        {
            this->removeOutEdgesConnectedToVertexFromQueue(vertex);
            this->removeInEdgesConnectedToVertexFromQueue(vertex);
        }

        void BITstar::SearchQueue::removeFromInconsistentSet(const VertexPtr &vertex)
        {
#ifdef BITSTAR_DEBUG
            if (vertex->isConsistent())
            {
                throw ompl::Exception("Attempting to remove a consistent vertex from the set of inconsistent "
                                      "vertices.");
            }
#endif  // BITSTAR_DEBUG
            inconsistentVertices_.erase(
                std::remove_if(inconsistentVertices_.begin(), inconsistentVertices_.end(),
                               [vertex](const VertexPtr &element) { return vertex->getId() == element->getId(); }),
                inconsistentVertices_.end());
        }

        void BITstar::SearchQueue::clear()
        {
            ASSERT_SETUP

            // Clear the edge queue.
            edgeQueue_.clear();

            // Increment the queue processing number.
            ++(*searchId_);
        }

        void BITstar::SearchQueue::insertOutgoingEdgesOfStartVertices()
        {
            ASSERT_SETUP

            // Insert the outgoing edges of the start vertices.
            for (auto it = graphPtr_->startVerticesBeginConst(); it != graphPtr_->startVerticesEndConst(); ++it)
            {
#ifdef BITSTAR_DEBUG
                if ((*it)->isPruned())
                {
                    throw ompl::Exception("Inserting outgoing edges of a pruned start vertex.");
                }
#endif  // BITSTAR_DEBUG
                this->insertOutgoingEdges(*it);
            }
        }

        void BITstar::SearchQueue::setInflationFactor(double factor)
        {
            inflationFactor_ = factor;
        }

        double BITstar::SearchQueue::getInflationFactor() const
        {
            return inflationFactor_;
        }

        std::shared_ptr<const unsigned int> BITstar::SearchQueue::getSearchId() const
        {
            return searchId_;
        }

        bool BITstar::SearchQueue::canPossiblyImproveCurrentSolution(const VertexPtr &state) const
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (state->isPruned())
            {
                throw ompl::Exception("Asking whether pruned state can possibly improve current solution.");
            }
#endif  // BITSTAR_DEBUG

            // Threshold should always be g_t(x_g)

            // Can it ever be a better solution?
            // Just in case we're using a vertex that is exactly optimally connected
            // g^(v) + h^(v) <= g_t(x_g)?
            return costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(state),
                                                                solutionCost_);
        }

        bool BITstar::SearchQueue::canPossiblyImproveCurrentSolution(const VertexPtrPair &edge) const
        {
            ASSERT_SETUP

            // Can it ever be a better solution? Less-than-equal to just in case we're using an edge that is exactly
            // optimally connected
            // g^(v) + c^(v,x) + h^(x) <= g_t(x_g)?
            bool canImprove = costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicEdge(edge),
                                                                           solutionCost_);

            // If the child is connected already, we need to check if we could do better than it's current connection.
            // But only if we're inserting the edge
            if (edge.second->hasParent() && canImprove)
            {
                // Can it ever be a better path to the vertex? Less-than-equal to just in case we're using an edge that
                // is exactly optimally connected
                // g^(v) + c^(v,x) <= g_t(x)?
                canImprove = costHelpPtr_->isCostBetterThanOrEquivalentTo(
                    costHelpPtr_->lowerBoundHeuristicToTarget(edge), edge.second->getCost());  // Ever rewire?
            }

            return canImprove;
        }

        unsigned int BITstar::SearchQueue::numEdges()
        {
            ASSERT_SETUP

            return edgeQueue_.size();
        }

        bool BITstar::SearchQueue::isEmpty()
        {
            ASSERT_SETUP

            return edgeQueue_.empty();
        }

        void BITstar::SearchQueue::getEdges(VertexConstPtrPairVector *edgeQueue)
        {
            ASSERT_SETUP

            // Clear the inout argument.
            edgeQueue->clear();

            // Get the contents on the binary heap (key and edge).
            std::vector<SortKeyAndVertexPtrPair> queueContents;
            edgeQueue_.getContent(queueContents);

            // Fill the inout argument.
            for (const auto &queueElement : queueContents)
            {
                edgeQueue->push_back(queueElement.second);
            }
        }

        void BITstar::SearchQueue::insertOutgoingEdges(const VertexPtr &vertex)
        {
#ifdef BITSTAR_DEBUG
            if (vertex->isPruned())
            {
                throw ompl::Exception("Inserting outgoing edges of pruned vertex.");
            }
#endif  // BITSTAR_DEBUG
        // Should we expand this vertex?
            if (this->canPossiblyImproveCurrentSolution(vertex))
            {
                // Get the neighbouring samples.
                VertexPtrVector neighbourSamples;
                graphPtr_->nearestSamples(vertex, &neighbourSamples);

                // Add all outgoing edges to neighbouring vertices and samples.
                this->enqueueEdges(vertex, neighbourSamples);
            }
            // No else
        }

        void BITstar::SearchQueue::insertOutgoingEdgesOfInconsistentVertices()
        {
            // Insert all outgoing edges of the inconsistent vertices.
            for (const auto &vertex : inconsistentVertices_)
            {
#ifdef BITSTAR_DEBUG
                if (vertex->isPruned())
                {
                    throw ompl::Exception("Attempting to insert outgoing edges of an inconsistent "
                                          "vertex that has been pruned.");
                }
#endif  // BITSTAR_DEBUG
                this->insertOutgoingEdges(vertex);
            }
        }

        void BITstar::SearchQueue::clearInconsistentSet()
        {
            inconsistentVertices_.clear();
        }

        void BITstar::SearchQueue::rebuildEdgeQueue()
        {
            // Ok this is going to be kinda dirty. We would like to have access to the actual underlying
            // std::vector of the binary heap, holding pointers to the elements in the heap.
            // Unfortunately, the binary heap interface only provides access to a copy. Now, we can still
            // access get the pointers to the elements because we stored them upon insertion. But it's a mess
            // (and suggests a flawed encapsulation or incomplete interface of the bin heap class?).

            // Get a copy of the contents.
            std::vector<SortKeyAndVertexPtrPair> contentCopy;
            edgeQueue_.getContent(contentCopy);

            // Now, get the parent vertices of all the edges still in the queue.
            std::set<VertexPtr> parents;
            for (const auto &element : contentCopy)
            {
                parents.insert(element.second.first);
            }

            // Update the sort keys of all outgoing edges of all vertices that have outgoing edges in the queue.
            for (const auto &parent : parents)
            {
                for (auto it = parent->edgeQueueOutLookupConstBegin(); it != parent->edgeQueueOutLookupConstEnd(); ++it)
                {
                    (*it)->data.first = this->createSortKey((*it)->data.second);
                }
            }

            // All edges have an updated key, let's rebuild the queue. Presumably this is more efficient than calling
            // EdgeQueue::update() on each edge individually while looping?
            edgeQueue_.rebuild();
        }

        void BITstar::SearchQueue::update(const EdgeQueueElemPtr elementPtr)
        {
            assert(elementPtr);

            // Create the up-to-date sort key for this edge.
            elementPtr->data.first = createSortKey(elementPtr->data.second);

            // Update its position in the queue.
            edgeQueue_.update(elementPtr);
        }

        void BITstar::SearchQueue::addToInconsistentSet(const VertexPtr &vertex)
        {
#ifdef BITSTAR_DEBUG
            if (vertex->isConsistent())
            {
                ompl::Exception("Attempted to add a consistent vertex to the inconsistent set.");
            }
            if (!vertex->isExpandedOnCurrentSearch())
            {
                ompl::Exception("Attempted to add an unexpanded vertex to the inconsistent set.");
            }
#endif  // BITSTAR_DEBUG

            inconsistentVertices_.push_back(vertex);
        }

        void BITstar::SearchQueue::enqueueEdges(const VertexPtr &parent, const VertexPtrVector &possibleChildren)
        {
#ifdef BITSTAR_DEBUG
            if (!parent->isInTree())
            {
                auto msg = "Trying to enqueue edges from a parent (" + std::to_string(parent->getId()) +
                           ") that's not in the tree."s;
                throw std::runtime_error(msg);
            }
#endif
            // Start with this vertex' current kiddos.
            VertexPtrVector currentChildren;
            parent->getChildren(&currentChildren);
            for (const auto &child : currentChildren)
            {
                this->enqueueEdgeConditionally(parent, child);
            }

            // We need to store whether an outgoing edge is a rewiring.
            bool isExpandedAsRewiring = false;

            // Now consider all neighbouring vertices that are not already my kids.
            for (auto &child : possibleChildren)
            {
                // If this sample is not connected to the search tree, just enqueue the edge if it's useful.
                if (!child->isInTree())
                {
                    this->enqueueEdgeConditionally(parent, child);
                }
                else  // If this sample is part of the tree, we need to be a little more careful.
                {
                    if (isCascadingOfRewiringsEnabled_ || !parent->hasEverBeenExpandedAsRewiring())
                    {
                        // Remember that this parent is expanded as a rewiring.
                        isExpandedAsRewiring = true;

                        // Make sure the child is not the root and distinct from this vertex (which is the parent).
                        if (!child->isRoot() && child->getId() != parent->getId())
                        {
                            // Make sure edges to kiddos aren't added twice.
                            if (child->getParent()->getId() != parent->getId())
                            {
                                // Make sure the neighbour vertex is not already my parent.
                                if (parent->isRoot() || child->getId() != parent->getParent()->getId())
                                {
                                    // The neighbour is not my parent, attempt to queue the edge.
                                    this->enqueueEdgeConditionally(parent, child);
                                }
                                // No else, this vertex is my parent.
                            }
                            // No else
                        }
                        // No else
                    }
                }
            }

            // If the parent is expanded to a vertex in the tree, it is a rewiring. This needs to be registered.
            if (isExpandedAsRewiring)
            {
                parent->registerRewiringExpansion();
            }
        }

        void BITstar::SearchQueue::enqueueEdgeConditionally(const VertexPtr &parent, const VertexPtr &child)
        {
            // Don't enqueue the edge if it's blacklisted.
            if (parent->isBlacklistedAsChild(child))
            {
                return;
            }
            else
            {
                // Create the edge object.
                VertexPtrPair newEdge = std::make_pair(parent, child);

                // Enqueue the edge only if it can possibly improve the current solution.
                if (this->canPossiblyImproveCurrentSolution(newEdge))
                {
                    this->enqueueEdge(newEdge);
                }
            }
        }

        BITstar::SearchQueue::SortKey BITstar::SearchQueue::createSortKey(const VertexPtrPair &edge) const
        {
            // The sort key of an edge (u, v) is [ g_t(u) + c^hat(u, v) + epsilon_s * h^hat(v); g_t(u) + c^hat(u, v);
            // g_t(u) ].
            return {{costHelpPtr_->combineCosts(
                         costHelpPtr_->currentHeuristicToTarget(edge),
                         costHelpPtr_->inflateCost(costHelpPtr_->costToGoHeuristic(edge.second), inflationFactor_)),
                     costHelpPtr_->currentHeuristicToTarget(edge), edge.first->getCost()}};
        }

        bool BITstar::SearchQueue::lexicographicalBetterThan(const std::array<ompl::base::Cost, 3> &lhs,
                                                             const std::array<ompl::base::Cost, 3> &rhs) const
        {
            return std::lexicographical_compare(lhs.cbegin(), lhs.cend(), rhs.cbegin(), rhs.cend(),
                                                [this](const ompl::base::Cost &a, const ompl::base::Cost &b) {
                                                    return costHelpPtr_->isCostBetterThan(a, b);
                                                });
        }

        void BITstar::SearchQueue::assertSetup() const
        {
            if (isSetup_ == false)
            {
                throw ompl::Exception("BITstar::SearchQueue was used before it was setup.");
            }
        }

        unsigned int BITstar::SearchQueue::numEdgesPopped() const
        {
            return numEdgesPopped_;
        }

    }  // namespace geometric
}  // namespace ompl
