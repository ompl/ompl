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

/* Authors: Jonathan Gammell */

// My definition:
#include "ompl/geometric/planners/bitstar/datastructures/SearchQueue.h"

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
#include "ompl/geometric/planners/bitstar/datastructures/HelperFunctions.h"
// The vertex class:
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
// The cost-helper class:
#include "ompl/geometric/planners/bitstar/datastructures/CostHelper.h"
// The implicit graph:
#include "ompl/geometric/planners/bitstar/datastructures/ImplicitGraph.h"

// Debug macros
#ifdef BITSTAR_DEBUG
    /** \brief A debug-only call to assert that the object is setup. */
    #define ASSERT_SETUP this->assertSetup();
#else
    #define ASSERT_SETUP
#endif  // BITSTAR_DEBUG

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::SearchQueue::SearchQueue(NameFunc nameFunc)
          : nameFunc_(std::move(nameFunc))
          , edgeQueue_([this](const SortKeyAndVertexPtrPair &lhs, const SortKeyAndVertexPtrPair &rhs)
                       {
                           return lexicographicalBetterThan(lhs.first, rhs.first);
                       })  // This tells the edgeQueue_ to use lexicographical comparison for sorting.
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

            // Reset the inflation factor.
            inflationFactor_ = 1.0;

            // Reset the number of queues that have been searched.
            numQueueResets_ = 0u;

            // Reset the cost threshold to infinite cost.
            solutionCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());

            // Make sure the queue doesn't think it has a solution.
            hasExactSolution_ = false;

            // Finally, reset the progress info.
            numEdgesPopped_ = 0u;
        }

        void BITstar::SearchQueue::enqueueEdge(const VertexPtrPair &edge)
        {
            ASSERT_SETUP

            // Create convenience aliases.
            const VertexPtr &parent = edge.first;
            const VertexPtr &child = edge.second;

            // If we already have the edge in the queue, we need to update its value.
            EdgeQueueElemPtr updateEdge = nullptr;
            for (auto it = child->edgeQueueInLookupConstBegin(searchId_); it != child->edgeQueueInLookupConstEnd(searchId_); ++it)
            {
                if ((*it)->data.second.first->getId() == parent->getId())
                {
                    updateEdge = (*it);
                    break;
                }
            }

            if (updateEdge) // The edge is already in the queue, we need to update it's sort key (presumably because the cost-to-come to the source changed).
            {
#ifdef BITSTAR_DEBUG
                if (updateEdge->data.second.first->getId() != edge.first->getId() || updateEdge->data.second.second->getId() != edge.second->getId())
                {
                    throw ompl::Exception("Updating the wrong edge.");
                }
#endif // BITSTAR_DEBUG
                updateEdge->data.first = this->createSortKey(edge);
                edgeQueue_.update(updateEdge);
            }
            else // This edge is not yet in the queue.
            {
                // The iterator to the new edge in the queue:
                EdgeQueueElemPtr edgeElemPtr;

                // Insert into the edge queue, getting the element pointer
                edgeElemPtr = edgeQueue_.insert(std::make_pair(this->createSortKey(edge), edge));

                // Push the newly created edge back on the vector of edges from the parent.
                parent->insertInEdgeQueueOutLookup(edgeElemPtr, searchId_);

                // Push the newly created edge back on the vector of edges to the child.
                child->insertInEdgeQueueInLookup(edgeElemPtr, searchId_);
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

            // Remove the edge from the respective vertex lookups.
            frontEdgeQueueElement->data.second.first->removeFromEdgeQueueOutLookup(frontEdgeQueueElement, numQueueResets_);
            frontEdgeQueueElement->data.second.second->removeFromEdgeQueueInLookup(frontEdgeQueueElement, numQueueResets_);

            // Remove it from the queue.
            edgeQueue_.pop();

            // Return the edge.
            return frontEdgeQueueElement->data.second;
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
                // Iterate over the vector of incoming edges to this vertex and remove them from the queue (and clean up their other lookup).
                for (auto it = vertex->edgeQueueInLookupConstBegin(numQueueResets_); it != vertex->edgeQueueInLookupConstEnd(numQueueResets_); ++it)
                {
                    // Remove the edge from the *other* lookup (by value since this is NOT an iter to THAT container).
                    // No need to remove from this lookup, as that's being cleared.
                    (*it)->data.second.first->removeFromEdgeQueueOutLookup(*it, numQueueResets_);

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
                // Iterate over the vector of outgoing edges to this vertex and remove them from the queue (and clean up their other lookup).
                for (auto it = vertex->edgeQueueOutLookupConstBegin(numQueueResets_); it != vertex->edgeQueueOutLookupConstEnd(numQueueResets_); ++it)
                {
                    // Remove the edge from the *other* lookup (by value since this is NOT an iter to THAT container).
                    // No need to remove from this lookup, as that's being cleared.
                    (*it)->data.second.second->removeFromEdgeQueueInLookup(*it, numQueueResets_);

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

        void BITstar::SearchQueue::clear()
        {
            ASSERT_SETUP

            // Clear the edge queue.
            edgeQueue_.clear();

            // Increment the queue processing number.
            ++numQueueResets_;
        }

        void BITstar::SearchQueue::restart()
        {
            ASSERT_SETUP

            // Insert the outgoing edges of the start vertices.
            for (auto it = graphPtr_->startVerticesBeginConst(); it != graphPtr_->startVerticesEndConst(); ++it)
            {
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

        bool BITstar::SearchQueue::canPossiblyImproveCurrentSolution(const VertexPtr &state) const
        {
            ASSERT_SETUP

            // Threshold should always be g_t(x_g)

            // Can it ever be a better solution?
            // Just in case we're using a vertex that is exactly optimally connected
            // g^(v) + h^(v) <= g_t(x_g)?
            return costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(state), solutionCost_);
        }

        bool BITstar::SearchQueue::canPossiblyImproveCurrentSolution(const VertexPtrPair &edge) const
        {
            ASSERT_SETUP

            // Can it ever be a better solution? Less-than-equal to just in case we're using an edge that is exactly
            // optimally connected
            // g^(v) + c^(v,x) + h^(x) <= g_t(x_g)?
            bool canImprove = costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicEdge(edge), solutionCost_);

            // If the child is connected already, we need to check if we could do better than it's current connection.
            // But only if we're inserting the edge
            if (edge.second->hasParent() && canImprove)
            {
                // Can it ever be a better path to the vertex? Less-than-equal to just in case we're using an edge that
                // is exactly optimally connected
                // g^(v) + c^(v,x) <= g_t(x)?
                canImprove = costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicToTarget(edge), edge.second->getCost());  // Ever rewire?
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
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Private functions:
        void BITstar::SearchQueue::insertOutgoingEdges(const VertexPtr &vertex)
        {
#ifdef BITSTAR_DEBUG
            // Assert that this vertex has no outgoing edge queue entries.
            if (vertex->edgeQueueOutLookupSize(numQueueResets_) != 0u)
            {
                std::cout << std::endl << "vId: " << vertex->getId() << std::endl;
                throw ompl::Exception("Unexpanded vertex already has outgoing entries in the edge queue.");
            }
#endif  // BITSTAR_DEBUG

            // Should we expand this vertex?
            if (this->canPossiblyImproveCurrentSolution(vertex))
            {
                // Get the neighbouring samples.
                VertexPtrVector neighbourSamples;
                graphPtr_->nearestSamples(vertex, &neighbourSamples);

                // Get the neighbouring vertices.
                VertexPtrVector neighbourVertices;
                graphPtr_->nearestVertices(vertex, &neighbourVertices);

                // If we're using k-nearest, we technically need to be doing to combined k-nearest.
                if (graphPtr_->getUseKNearest())
                {
                    this->processKNearest(vertex, &neighbourSamples, &neighbourVertices);
                }
                // No else, if we're using r-disc, we keep both sets.

                // Add all outgoing edges to neighbouring vertices and samples.
                this->enqueueEdgesToSamples(vertex, neighbourSamples);
                this->enqueueEdgesToVertices(vertex, neighbourVertices);
            }
            // No else
        }

        void BITstar::SearchQueue::enqueueEdgesToSamples(const VertexPtr &vertex, const VertexPtrVector& neighbourSamples)
        {
            // Iterate through the samples and add each one
            for (auto &sample : neighbourSamples)
            {
                // It is new or we don't care, attempt to queue the edge.
                this->enqueueEdgeConditionally(vertex, sample);
            }
        }

        void BITstar::SearchQueue::enqueueEdgesToVertices(const VertexPtr &vertex, const VertexPtrVector& neighbourVertices)
        {
            // Start with this vertex' current kids.
            VertexPtrVector children;
            vertex->getChildren(&children);
            for (const auto &child : children)
            {
                this->enqueueEdgeConditionally(vertex, child);
            }

            // Now consider all neighbouring vertices that are not already my kids.
            for (auto &neighbourVertex : neighbourVertices)
            {
                // Make sure the child is not the root and distinct from this vertex (which is the parent).
                if (!neighbourVertex->isRoot() && neighbourVertex->getId() != vertex->getId())
                {
                    // Make sure I am not already the parent
                    if (neighbourVertex->getParent()->getId() != vertex->getId())
                    {
                        // Make sure the neighbour vertex is not already my parent:
                        if (vertex->isRoot())
                        {
                            // I am root, I have no parent, so attempt to queue the edge:
                            this->enqueueEdgeConditionally(vertex, neighbourVertex);
                        }
                        else if (neighbourVertex->getId() != vertex->getParent()->getId())
                        {
                            // The neighbour is not my parent, attempt to queue the edge.
                            this->enqueueEdgeConditionally(vertex, neighbourVertex);
                        }
                        // No else, this vertex is my parent.
                    }
                    // No else
                }
                // No else
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

        void BITstar::SearchQueue::processKNearest(const VertexConstPtr &vertex, VertexPtrVector *kNearSamples,
                                                   VertexPtrVector *kNearVertices)
        {
            // Variables
            // The position in the sample vector
            unsigned int samplePos = 0u;
            // The position in the vertex vector
            unsigned int vertexPos = 0u;

            // Iterate through the first k in the combined vectors
            while (samplePos + vertexPos < graphPtr_->getConnectivityK() &&
                   (samplePos < kNearSamples->size() || vertexPos < kNearVertices->size()))
            {
                // Where along are we in the relative vectors?
                if (samplePos < kNearSamples->size() && vertexPos >= kNearVertices->size())
                {
                    // There are just samples left. Easy, move the sample token:
                    ++samplePos;
                }
                else if (samplePos >= kNearSamples->size() && vertexPos < kNearVertices->size())
                {
                    // There are just vertices left. Easy, move the vertex token:
                    ++vertexPos;
                }
                else
                {
                    // Both are left, which is closest?
                    if (graphPtr_->distance(kNearVertices->at(vertexPos), vertex) <
                        graphPtr_->distance(kNearSamples->at(samplePos), vertex))
                    {
                        // The vertex is closer than the sample, move that token:
                        ++vertexPos;
                    }
                    else
                    {
                        // The vertex is not closer than the sample, move the sample token:
                        ++samplePos;
                    }
                }
            }

            // Now erase the extra. Resize will truncate the extras
            kNearSamples->resize(samplePos);
            kNearVertices->resize(vertexPos);
        }

        BITstar::SearchQueue::SortKey BITstar::SearchQueue::createSortKey(const VertexPtrPair &edge) const
        {
            // The sort key of an edge (u, v) is [ g_t(u) + c^hat(u, v) + epsilon_s * h^hat(v); g_t(u) + c^hat(u, v); g_t(u) ].
            return {{costHelpPtr_->combineCosts(costHelpPtr_->currentHeuristicToTarget(edge), costHelpPtr_->inflateCost(costHelpPtr_->costToGoHeuristic(edge.second), inflationFactor_)),
                     costHelpPtr_->currentHeuristicToTarget(edge),
                     edge.first->getCost()}};
        }

        bool BITstar::SearchQueue::lexicographicalBetterThan(const std::array<ompl::base::Cost, 3> &lhs,
                                                             const std::array<ompl::base::Cost, 3> &rhs) const
        {
            return std::lexicographical_compare(lhs.cbegin(), lhs.cend(), rhs.cbegin(), rhs.cend(),
                                                [this](const ompl::base::Cost &a, const ompl::base::Cost &b)
                                                {
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
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Boring sets/gets (Public):
        unsigned int BITstar::SearchQueue::numEdgesPopped() const
        {
            return numEdgesPopped_;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    }  // geometric
}  // ompl
