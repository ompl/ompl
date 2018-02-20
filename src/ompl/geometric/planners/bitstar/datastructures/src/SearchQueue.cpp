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

// For std::move
#include <utility>
// For std::lexicographical_compare and the std::*_heap functions.
#include <algorithm>

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
          , vertexQueue_([this](const CostDouble &lhs, const CostDouble &rhs)
                         {
                             return queueComparison(lhs, rhs);
                         })  // This tells the vertexQueue_ to use the queueComparison for sorting
          , vertexToExpand_(vertexQueue_.begin())
          , edgeQueue_([this](const CostTripleAndVertexPtrPair &lhs, const CostTripleAndVertexPtrPair &rhs)
                       {
                           return queueComparison(lhs.first, rhs.first);
                       })  // This tells the edgeQueue_ to use the queueComparison for sorting
        {
        }

        void BITstar::SearchQueue::setup(CostHelper *costHelpPtr, ImplicitGraph *graphPtr)
        {
            // Store that I am setup
            isSetup_ = true;

            // Get my copies
            costHelpPtr_ = costHelpPtr;
            graphPtr_ = graphPtr;

            // Set the the cost threshold to infinity to start:
            solnCost_ = costHelpPtr_->infiniteCost();
        }

        void BITstar::SearchQueue::clear()
        {
            // Reset everything to the state of construction OTHER than planner name and settings/parameters
            // Keep this in the order of the constructors for easy verification:

            // Mark as cleared
            isSetup_ = false;

            // The pointers
            costHelpPtr_ = nullptr;
            graphPtr_ = nullptr;

            // The vertex queue:
            vertexQueue_.clear();
            vertexToExpand_ = vertexQueue_.begin();

            // The edge queue:
            edgeQueue_.clear();

            // The number of times we're gone through the vertex queue:
            numQueueResets_ = 0u;

            // The resort vector:
            resortVertices_.clear();

            // The cost threshold:
            solnCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());

            // The existence of a solution:
            hasExactSolution_ = false;

            // Progress properties
            numEdgesPopped_ = 0u;

            // DO NOT reset the settings:
            // useStrictQueueOrdering_
            // delayRewiring_
            // pruneDuringResort_
        }

        void BITstar::SearchQueue::enqueueVertex(const VertexPtr &newVertex, bool removeFromFree)
        {
            ASSERT_SETUP

            // Insert the vertex:
            this->vertexInsertHelper(newVertex, true, removeFromFree, true);
        }

        void BITstar::SearchQueue::enqueueEdge(const VertexPtrPair &newEdge)
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            // Assert that the parent vertex is in the vertex queue
            if (!newEdge.first->hasVertexQueueEntry())
            {
                throw ompl::Exception("Attempted to enqueue an edge from a vertex not in the vertex queue.");
            }
#endif  // BITSTAR_DEBUG

            // Variable:
            // The iterator to the new edge in the queue:
            EdgeQueueElemPtr edgeElemPtr;

            // Insert into the edge queue, getting the element pointer
            edgeElemPtr = edgeQueue_.insert(std::make_pair(this->edgeQueueValue(newEdge), newEdge));

            // Push the newly created edge back on the vector of edges from the parent.
            newEdge.first->addOutgoingEdgeQueuePtr(edgeElemPtr, numQueueResets_);

            // Push the newly created edge back on the vector of edges to the child.
            newEdge.second->addIncomingEdgeQueuePtr(edgeElemPtr, numQueueResets_);
        }

        void BITstar::SearchQueue::enqueueEdge(const VertexPtr &sourceVertex, const VertexPtr &targetVertex)
        {
            ASSERT_SETUP

            // Call my helper function:
            this->enqueueEdge(std::make_pair(sourceVertex, targetVertex));
        }

        void BITstar::SearchQueue::unqueueVertex(const VertexPtr &oldVertex)
        {
            ASSERT_SETUP

            // Disconnect from parent if necessary, cascading cost updates:
            if (oldVertex->hasParent())
            {
                this->disconnectParent(oldVertex, true);
            }

            // Remove it from vertex queue and lookup, and edge queues:
            this->vertexRemoveHelper(oldVertex, true);
        }

        BITstar::VertexPtr BITstar::SearchQueue::frontVertex()
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Update the queue:
            this->updateQueue();

            // Return the front edge
            return vertexQueue_.begin()->second;
        }

        BITstar::VertexPtrPair BITstar::SearchQueue::frontEdge()
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Update the queue:
            this->updateQueue();

            // Return the front edge
            return edgeQueue_.top()->data.second;
        }

        BITstar::SearchQueue::CostDouble BITstar::SearchQueue::frontVertexValue()
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Update the queue:
            this->updateQueue();

            // Return the front value
            return vertexQueue_.begin()->first;
        }

        BITstar::SearchQueue::CostTriple BITstar::SearchQueue::frontEdgeValue()
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Update the queue:
            this->updateQueue();

            // Return the front value
            return edgeQueue_.top()->data.first;
        }

        void BITstar::SearchQueue::popFrontEdge(VertexPtrPair *bestEdge)
        {
            ASSERT_SETUP

            // Variable
            // The top of the binary heap
            EdgeQueueElemPtr bestElemPtr;

#ifdef BITSTAR_DEBUG
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to pop an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Update the queue:
            this->updateQueue();

#ifdef BITSTAR_DEBUG
            if (edgeQueue_.empty())
            {
                throw ompl::Exception("Edge queue is still empty after an update.");
            }
#endif  // BITSTAR_DEBUG

            // Get the front:
            bestElemPtr = edgeQueue_.top();

            // Store it in the return value;
            *bestEdge = bestElemPtr->data.second;

            // Remove the lookups to the element
            bestElemPtr->data.second.first->rmOutgoingEdgeQueuePtr(bestElemPtr, numQueueResets_);
            bestElemPtr->data.second.second->rmIncomingEdgeQueuePtr(bestElemPtr, numQueueResets_);

            // Finally, remove from the queue itself
            edgeQueue_.remove(bestElemPtr);

            // Increment my counter
            ++numEdgesPopped_;
        }

        BITstar::VertexPtrPair BITstar::SearchQueue::popFrontEdge()
        {
            ASSERT_SETUP

            VertexPtrPair rval;

            this->popFrontEdge(&rval);

            return rval;
        }

        void BITstar::SearchQueue::hasSolution(const ompl::base::Cost &solnCost)
        {
            ASSERT_SETUP

            // Flag
            hasExactSolution_ = true;

            // Store
            solnCost_ = solnCost;
        }

        void BITstar::SearchQueue::removeEdgesTo(const VertexPtr &cVertex)
        {
            ASSERT_SETUP

            if (!edgeQueue_.empty())
            {
                // Iterate over the vector of incoming edges to this vertex and remove them from the queue (and clean up their other lookup)
                for (auto inQueueElemIter = cVertex->incomingEdgeQueuePtrsBeginConst(numQueueResets_);
                      inQueueElemIter != cVertex->incomingEdgeQueuePtrsEndConst(numQueueResets_);
                      ++inQueueElemIter)
                {
                    // Remove the edge from the *other* lookup (by value since this is NOT an iter to THAT container).
                    // No need to remove from this lookup, as that's being cleared:
                    (*inQueueElemIter)->data.second.first->rmOutgoingEdgeQueuePtr(*inQueueElemIter, numQueueResets_);

                    // Finally remove it from the queue
                    edgeQueue_.remove(*inQueueElemIter);
                }

                // Clear the list:
                cVertex->clearIncomingEdgeQueuePtrs();
            }
            // No else, nothing to remove_to
        }

        void BITstar::SearchQueue::removeEdgesFrom(const VertexPtr &pVertex)
        {
            ASSERT_SETUP

            if (!edgeQueue_.empty())
            {
                // Iterate over the vector of outgoing edges to this vertex and remove them from the queue (and clean up their other lookup)
                for (auto outQueueElemIter = pVertex->outgoingEdgeQueuePtrsBeginConst(numQueueResets_);
                  outQueueElemIter != pVertex->outgoingEdgeQueuePtrsEndConst(numQueueResets_);
                  ++outQueueElemIter)
                {
                    // Remove the edge from the *other* lookup (by value since this is NOT an iter to THAT container).
                    // No need to remove from this lookup, as that's being cleared:
                    (*outQueueElemIter)->data.second.second->rmIncomingEdgeQueuePtr(*outQueueElemIter, numQueueResets_);

                    // Finally, remove it from the queue
                    edgeQueue_.remove(*outQueueElemIter);
                }

                // Clear the list:
                pVertex->clearOutgoingEdgeQueuePtrs();
            }
            // No else, nothing to remove_from
        }

        void BITstar::SearchQueue::removeExtraEdgesTo(const VertexPtr &cVertex)
        {
            ASSERT_SETUP

            if (!edgeQueue_.empty())
            {
                // Variable
                // The vector of edges to delete in the vector:
                std::vector<EdgeQueueElemPtrVector::const_iterator> inItersToDelete;

                // Iterate over the incoming edges and record those that are to be deleted
                for (auto inQueueElemIter = cVertex->incomingEdgeQueuePtrsBeginConst(numQueueResets_);
                     inQueueElemIter != cVertex->incomingEdgeQueuePtrsEndConst(numQueueResets_);
                     ++inQueueElemIter)
                {
                    // Check if it would have been inserted
                    if (!this->edgeInsertCondition((*inQueueElemIter)->data.second))
                    {
                        // It would not, delete
                        inItersToDelete.push_back(inQueueElemIter);
                    }
                    // No else, we're not deleting this iterator
                }

                // Now, iterate over the vector of iterators to delete and remove them from the queue and their lookups
                for (const auto &inVectorIter : inItersToDelete)
                {
                    // Remove the entry in the other lookup (by value since this is NOT an iter to THAT container)
                    (*inVectorIter)->data.second.first->rmOutgoingEdgeQueuePtr(*inVectorIter, numQueueResets_);

                    // Remove from the queue itself
                    edgeQueue_.remove(*inVectorIter);

                    // And finally erase the lookup iterator from my lookup. If this was done first, the
                    // iterator would have been invalidated for the above.
                    cVertex->rmIncomingEdgeQueuePtrByIter(inVectorIter, numQueueResets_);
                }
            }
            // No else, nothing to prune_to
        }

        void BITstar::SearchQueue::removeExtraEdgesFrom(const VertexPtr &pVertex)
        {
            ASSERT_SETUP

            if (!edgeQueue_.empty())
            {
                // Variable
                // The vector of edges to delete in the vector:
                std::vector<EdgeQueueElemPtrVector::const_iterator> outItersToDelete;

                // Iterate over the incoming edges and record those that are to be deleted
                for (auto outQueueElemIter = pVertex->outgoingEdgeQueuePtrsBeginConst(numQueueResets_);
                  outQueueElemIter != pVertex->outgoingEdgeQueuePtrsEndConst(numQueueResets_);
                  ++outQueueElemIter)
                {
                    // Check if it would have been inserted
                    if (!this->edgeInsertCondition((*outQueueElemIter)->data.second))
                    {
                        // It would not, delete
                        outItersToDelete.push_back(outQueueElemIter);
                    }
                    // No else, we're not deleting this iterator
                }

                // Now, iterate over the vector of iterators to delete and remove them from the queue and their lookups
                for (const auto &outVectorIter : outItersToDelete)
                {
                    // Remove the entry in the other lookup (by value since this is NOT an iter to THAT container)
                    (*outVectorIter)->data.second.second->rmIncomingEdgeQueuePtr(*outVectorIter, numQueueResets_);

                    // Remove from the queue itself
                    edgeQueue_.remove(*outVectorIter);

                    // And finally erase the lookup iterator from my lookup. If this was done first, the
                    // iterator would have been invalidated for the above.
                    pVertex->rmOutgoingEdgeQueuePtrByIter(outVectorIter, numQueueResets_);
                }
            }
            // No else, nothing to prune_from
        }

        void BITstar::SearchQueue::markVertexUnsorted(const VertexPtr &vertex)
        {
            ASSERT_SETUP

            resortVertices_.push_back(vertex);
        }

        std::pair<unsigned int, unsigned int> BITstar::SearchQueue::prune(const VertexConstPtr &vertex)
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (hasExactSolution_ == false)
            {
                throw ompl::Exception("Prune cannot be called until a solution is found");
            }
            if (this->isSorted() == false)
            {
                throw ompl::Exception("Prune cannot be called on an unsorted queue.");
            }
#endif  // BITSTAR_DEBUG

            // The vertex expansion queue is sorted on an estimated solution cost considering the *current* cost-to-come
            // of the vertices, while we prune by considering the best-case cost-to-come.
            // This means that the value of the vertices in the queue are an upper-bounding estimate of the value we
            // will use to prune them.
            // Therefore, we can start our pruning at the goal vertex and iterate forward through the queue from there.

            // Variables:
            // The number of vertices and samples pruned:
            std::pair<unsigned int, unsigned int> numPruned(0u, 0u);
            // The iterator into the queue:
            VertexQueueIter queueIter;

            // Get the vertex queue iterator of the given starting point.:
            queueIter = vertex->getVertexQueueIter();

            // Iterate through to the end of the queue
            while (queueIter != vertexQueue_.end())
            {
                // Check if it should be pruned (value) or has lost its parent.
                if (this->vertexPruneCondition(queueIter->second))
                {
                    // The vertex should be pruned.
                    // Variables
                    // An iter to the vertex to prune:
                    VertexQueueIter pruneIter;

                    // Copy the iterator to prune:
                    pruneIter = queueIter;

                    // Move the queue iterator back one so we can step to the next *valid* vertex after pruning:
                    --queueIter;

                    // Prune the branch:
                    numPruned = numPruned + this->pruneBranch(pruneIter->second);
                }
                // No else, skip this vertex.

                // Iterate forward to the next value in the queue
                ++queueIter;
            }

            // Return the number of vertices and samples pruned.
            return numPruned;
        }

        void BITstar::SearchQueue::resort()
        {
            ASSERT_SETUP

            // Iterate through every vertex marked for resorting:
            if (!resortVertices_.empty())
            {
                // Variable:
                // The container ordered on vertex depth:
                typedef std::unordered_map<BITstar::VertexId, VertexPtr> VertexIdToVertexPtrUMap;
                typedef std::map<unsigned int, VertexIdToVertexPtrUMap> DepthToUMapMap;
                DepthToUMapMap uniqueResorts;
                // The number of vertices and samples pruned, respectively:
                std::pair<unsigned int, unsigned int> numPruned(0u, 0u);

                OMPL_INFORM("%s: Resorting %d vertices in the queue.", nameFunc_().c_str(), resortVertices_.size());

                // Variable:

                // Iterate over the vector and place into the unique queue indexed on *depth*. This guarantees that we
                // won't process a branch multiple times by being given different vertices down its chain
                for (auto &vertex : resortVertices_)
                {
                    // Add the vertex to the unordered map stored at the given depth.
                    // The [] return a reference to the existing entry, or creates a new entry:
                    uniqueResorts[vertex->getDepth()].emplace(vertex->getId(), vertex);
                }

                // Clear the vector of vertices to resort from:
                resortVertices_.clear();

                // Now process the vertices in order of depth.
                for (auto &depthAndVertexMapPair : uniqueResorts)
                {
                    for (auto &vIdAndPtrPair : depthAndVertexMapPair.second)
                    {
                        // Make sure it has not already been pruned:
                        if (!vIdAndPtrPair.second->isPruned())
                        {
                            // Make sure it has not already been returned to the set of samples:
                            if (vIdAndPtrPair.second->isInTree())
                            {
                                // If this was a new vertex, would we *not* insert it in the queue (and do we have
                                // "permission" not to do so)?
                                if (pruneDuringResort_ &&
                                    !this->vertexInsertCondition(vIdAndPtrPair.second))
                                {
                                    // The vertex should just be pruned and forgotten about.
                                    // Prune the branch:
                                    numPruned = numPruned + this->pruneBranch(vIdAndPtrPair.second);
                                }
                                else
                                {
                                    // The vertex is going to be kept.
                                    // Does it have any children?
                                    if (vIdAndPtrPair.second->hasChildren())
                                    {
                                        // Variables:
                                        // The vector of children:
                                        VertexPtrVector resortChildren;

                                        // Put its children in the vector to be resorted:
                                        // Get the vector of children:
                                        vIdAndPtrPair.second->getChildren(&resortChildren);

                                        // Get a reference to the container for the children, all children are 1 level
                                        // deeper than their parent.:
                                        // The [] return a reference to the existing entry, or creates a new entry:
                                        VertexIdToVertexPtrUMap &depthContainer =
                                            uniqueResorts[vIdAndPtrPair.second->getDepth() + 1u];

                                        // Place the children into the container, as the container is a map, it will not
                                        // allow the children to be entered twice.
                                        for (auto &childPtr : resortChildren)
                                        {
                                            depthContainer.emplace(childPtr->getId(), childPtr);
                                        }
                                    }

                                    // Resort the vertex:
                                    this->resortVertex(vIdAndPtrPair.second);
                                }
                            }
                            // No else, this vertex was a child of a vertex pruned during the resort. It has already
                            // been returned to the set of free samples by the call to pruneBranch.
                        }
                        // No else, this vertex was a child of a vertex pruned during the resort. It has already been
                        // deleted by the call to pruneBranch.
                    }
                }

                if (numPruned.first > 0u || numPruned.second > 0u)
                {
                    OMPL_INFORM("%s: Resorting the queue disconnected %d vertices from the tree and completely removed "
                                "%d samples.",
                                nameFunc_().c_str(), numPruned.first, numPruned.second);
                }
                // No else, sshhh
            }
            // No else, nothing to resort
        }

        void BITstar::SearchQueue::finish()
        {
            ASSERT_SETUP

            // Is there anything to resort before we're marked as finished?
            if (!resortVertices_.empty())
            {
                // There are unsorted vertices, sort them:
                OMPL_DEBUG("%s (SearchQueue): Resorting an unsorted queue instead of marking it as finished.",
                           nameFunc_().c_str());
                this->resort();
            }
            else
            {
                // We have exhausted this queue.
                // Clear the edge container:
                edgeQueue_.clear();

                // Increment the queue processing number
                ++numQueueResets_;

                // Move the token to the end:
                vertexToExpand_ = vertexQueue_.end();
            }
        }

        void BITstar::SearchQueue::reset()
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (resortVertices_.empty() == false)
            {
                throw ompl::Exception("SearchQueue::reset() called with an unsorted queue.");
            }
#endif  // BITSTAR_DEBUG

            // Clear the edge container:
            edgeQueue_.clear();

            // Increment the queue processing number
            ++numQueueResets_;

            // The resort vector:
            resortVertices_.clear();

            // Restart the expansion queue:
            vertexToExpand_ = vertexQueue_.begin();
        }

        bool BITstar::SearchQueue::vertexInsertCondition(const VertexPtr &state) const
        {
            ASSERT_SETUP

            // Threshold should always be g_t(x_g)

            // Can it ever be a better solution?
            // Just in case we're using a vertex that is exactly optimally connected
            // g^(v) + h^(v) <= g_t(x_g)?
            return costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(state),
                                                                solnCost_);
        }

        bool BITstar::SearchQueue::edgeInsertCondition(const VertexPtrPair &edge) const
        {
            ASSERT_SETUP

            bool rval;

            // Can it ever be a better solution? Less-than-equal to just in case we're using an edge that is exactly
            // optimally connected
            // g^(v) + c^(v,x) + h^(x) <= g_t(x_g)?
            rval = costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicEdge(edge),
                                                                solnCost_);

            // If the child is connected already, we need to check if we could do better than it's current connection.
            // But only if we're inserting the edge
            if (edge.second->hasParent() && rval)
            {
                // Can it ever be a better path to the vertex? Less-than-equal to just in case we're using an edge that
                // is exactly optimally connected
                // g^(v) + c^(v,x) <= g_t(x)?
                rval = costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicToTarget(edge),
                                                                    edge.second->getCost());  // Ever rewire?
            }

            return rval;
        }

        bool BITstar::SearchQueue::vertexPruneCondition(const VertexPtr &state) const
        {
            ASSERT_SETUP

            // Threshold should always be g_t(x_g)

            // Prune the vertex if it could cannot part of a better solution in the current graph.  Greater-than just in
            // case we're using an edge that is exactly optimally connected.
            // g_t(v) + h^(v) > g_t(x_g)?
            return costHelpPtr_->isCostWorseThan(costHelpPtr_->currentHeuristicVertex(state), solnCost_);
        }

        bool BITstar::SearchQueue::samplePruneCondition(const VertexPtr &state) const
        {
            ASSERT_SETUP

            // Threshold should always be g_t(x_g)
            // Prune the unconnected sample if it could never be better of a better solution.
            // g^(v) + h^(v) >= g_t(x_g)?
            return costHelpPtr_->isCostWorseThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(state),
                                                               solnCost_);
        }

        bool BITstar::SearchQueue::edgePruneCondition(const VertexPtrPair &edge) const
        {
            ASSERT_SETUP

            bool rval;
            // Threshold should always be g_t(x_g)

            // Prune the edge if it could cannot part of a better solution in the current graph.  Greater-than just in
            // case we're using an edge that is exactly optimally connected.
            // g_t(v) + c^(v,x) + h^(x) > g_t(x_g)?
            rval = costHelpPtr_->isCostWorseThan(costHelpPtr_->currentHeuristicEdge(edge), solnCost_);

            // If the child is connected already, we need to check if we could do better than it's current connection.
            // But only if we're not pruning based on the first check
            if (edge.second->hasParent() && !rval)
            {
                // Can it ever be a better path to the vertex in the current graph? Greater-than to just in case we're
                // using an edge that is exactly optimally connected
                // g_t(v) + c^(v,x) > g_t(x)?
                rval = costHelpPtr_->isCostWorseThan(costHelpPtr_->currentHeuristicToTarget(edge),
                                                     edge.second->getCost());  // Currently rewire?
            }

            return rval;
        }

        unsigned int BITstar::SearchQueue::numEdges()
        {
            ASSERT_SETUP

            // Update the queue:
            this->updateQueue();

            return edgeQueue_.size();
        }

        unsigned int BITstar::SearchQueue::numVertices()
        {
            ASSERT_SETUP

            // Update the queue:
            this->updateQueue();

            // Variables:
            // The number of vertices left to expand:
            unsigned int numToExpand;

            // Start at 0:
            numToExpand = 0u;

            // Iterate until the end:
            for (VertexQueueAsMMap::const_iterator vIter = vertexToExpand_; vIter != vertexQueue_.end(); ++vIter)
            {
                // Increment counter:
                ++numToExpand;
            }

            // Return
            return numToExpand;
        }

        unsigned int BITstar::SearchQueue::numEdgesTo(const VertexPtr &cVertex)
        {
            ASSERT_SETUP

            // Update the queue:
            this->updateQueue();

            // Return:
            return cVertex->getNumIncomingEdgeQueuePtrs(numQueueResets_);
        }

        unsigned int BITstar::SearchQueue::numEdgesFrom(const VertexPtr &pVertex)
        {
            ASSERT_SETUP

            // Update the queue:
            this->updateQueue();

            // Return:
            return pVertex->getNumOutgoingEdgeQueuePtrs(numQueueResets_);
        }

        unsigned int BITstar::SearchQueue::numUnsorted() const
        {
            ASSERT_SETUP

            return resortVertices_.size();
        }

        bool BITstar::SearchQueue::isSorted() const
        {
            ASSERT_SETUP

            return resortVertices_.empty();
        }

        bool BITstar::SearchQueue::isReset() const
        {
            ASSERT_SETUP

            return (vertexToExpand_ == vertexQueue_.begin() && edgeQueue_.empty());
        }

        bool BITstar::SearchQueue::isEmpty()
        {
            ASSERT_SETUP

            // Update the queue:
            this->updateQueue();

            // Expand if the edge queue is empty but the vertex queue is not:
            while (edgeQueue_.empty() && vertexToExpand_ != vertexQueue_.end())
            {
                // Expand the next vertex, this pushes the token:
                this->expandNextVertex();
            }

            // If the edge queue is actually empty, than use this opportunity to resort any unsorted vertices
            if (edgeQueue_.empty())
            {
                this->resort();
            }
            // No else

            // Return whether the edge queue is empty:
            return edgeQueue_.empty();
        }

        bool BITstar::SearchQueue::isVertexExpanded(const VertexConstPtr &vertex) const
        {
            ASSERT_SETUP

            // Compare the value used to currently sort the vertex in the queue to the value of the token.
            if (vertexToExpand_ == vertexQueue_.end())
            {
                // If the token is at the end of the queue, obviously the vertex is expanded:
                return true;
            }
            // else:

            // By virtue of the vertex expansion rules, the token will always sit at the front of a group of
            // equivalent cost vertices (that is to say, all vertices with the same cost get expanded at the same
            // time). Therefore, the vertex is expanded if it's cost is strictly better than the token.
            return this->queueComparison(vertex->getVertexQueueIter()->first, vertexToExpand_->first);
        }

        void BITstar::SearchQueue::getVertices(VertexConstPtrVector *vertexQueue)
        {
            ASSERT_SETUP

            // Update the queue:
            this->updateQueue();

            // Clear the given vector:
            vertexQueue->clear();

            // Iterate until the end, pushing back:
            for (VertexQueueAsMMap::const_iterator vIter = vertexToExpand_; vIter != vertexQueue_.end(); ++vIter)
            {
                // Push back:
                vertexQueue->push_back(vIter->second);
            }
        }

        void BITstar::SearchQueue::getEdges(VertexConstPtrPairVector *edgeQueue)
        {
            ASSERT_SETUP

            // Variable
            // The contents on the binary heap (key and edge)
            std::vector<CostTripleAndVertexPtrPair> queueContents;

            // Update the queue:
            this->updateQueue();

            // Get the contents
            edgeQueue_.getContent(queueContents);

            // Clear the vector
            edgeQueue->clear();

            // I don't think there's a std::copy way to do this, so just iterate
            for (const auto &queueElement : queueContents)
            {
                edgeQueue->push_back(queueElement.second);
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Private functions:
        void BITstar::SearchQueue::updateQueue()
        {
            // If we are using strict queue ordering, we must resort the queue before we update it.
            if (useStrictQueueOrdering_)
            {
                // Resort and vertices that have been rewired.
                this->resort();
            }

            // Variables:
            // Whether to expand:
            bool expand;

            expand = true;
            while (expand)
            {
                // Check if there are any vertices to expand:
                if (vertexToExpand_ != vertexQueue_.end())
                {
                    // Expand a vertex if the edge queue is empty, or the vertex could place a better edge into it:
                    if (edgeQueue_.empty())
                    {
                        // The edge queue is empty, any edge is better than this!
                        this->expandNextVertex();
                    }
                    // This is isCostBetterThanOrEquivalentTo because of the second ordering criteria. The vertex
                    // expanded could match the edge in queue on total cost, but have less cost-to-come.
                    else if (costHelpPtr_->isCostBetterThanOrEquivalentTo(vertexToExpand_->first.at(0u),
                                                                          edgeQueue_.top()->data.first.at(0u)))
                    {
                        // The vertex *could* give a better edge than our current best edge:
                        this->expandNextVertex();
                    }
                    else
                    {
                        // We are done expanding for now:
                        expand = false;
                    }
                }
                else
                {
                    // There are no vertices left to expand
                    expand = false;
                }
            }
        }

        void BITstar::SearchQueue::expandNextVertex()
        {
            // Should we expand the next vertex?
            if (this->vertexInsertCondition(vertexToExpand_->second))
            {
                // Expand the vertex in the front:
                this->expandVertex(vertexToExpand_->second);

                // Increment the vertex token:
                ++vertexToExpand_;
            }
            else
            {
                // The next vertex would get pruned, so just jump to the end:
                vertexToExpand_ = vertexQueue_.end();
            }
        }

        void BITstar::SearchQueue::expandVertex(const VertexPtr &vertex)
        {
#ifdef BITSTAR_DEBUG
            // Assert that this vertex has no outgoing edge queue entries.
            if (vertex->hasOutgoingEdgeQueueEntries(numQueueResets_))
            {
                std::cout << std::endl << "vId: " << vertex->getId() << std::endl;
                throw ompl::Exception("Unexpanded vertex already has outgoing entries in the edge queue.");
            }
#endif  // BITSTAR_DEBUG

            // Should we expand this vertex?
            if (this->vertexInsertCondition(vertex))
            {
                // Variables:
                // The vector of nearby samples (either within r or the k-nearest)
                VertexPtrVector neighbourSamples;
                // The vector of nearby vertices
                VertexPtrVector neighbourVertices;

                // Get the set of nearby free states
                graphPtr_->nearestSamples(vertex, &neighbourSamples);

                // If we're usjng k-nearest, we technically need to be doing to combined k-nearest.
                // So get the nearestVertices and do some post-processing
                if (graphPtr_->getUseKNearest())
                {
                    // Get the set of nearby vertices
                    graphPtr_->nearestVertices(vertex, &neighbourVertices);

                    // Post process them:
                    this->processKNearest(vertex, &neighbourSamples, &neighbourVertices);
                }
                // No else

                // Add potential edges from the vertex to nearby states.
                // Do so intelligently to avoid repeatedly considering the same failed edges (likely due to collision).

                // Add edges to unconnected targets who could ever provide a better solution:
                // Has the vertex been expanded into edges towards unconnected samples before?
                if (!vertex->hasBeenExpandedToSamples())
                {
                    // It has not, that means none of its outgoing edges have been considered. Add them all
                    this->enqueueSamples(vertex, neighbourSamples, true);
                }
                else
                {
                    // It has, which means that outgoing edges to old unconnected vertices have already been considered.
                    // Only add those that lead to new vertices
                    this->enqueueSamples(vertex, neighbourSamples, false);
                }

                // If the vertex has never been expanded into possible rewiring edges *and* either we're not delaying
                // rewiring or we have a solution, we add those rewiring candidates:
                if (!vertex->hasBeenExpandedToVertices() && (!delayRewiring_ || hasExactSolution_))
                {
                    // If we're using an r-disc RGG, we will not have gotten the neighbour vertices yet, get them now
                    if (!graphPtr_->getUseKNearest())
                    {
                        // Get the set of nearby vertices
                        graphPtr_->nearestVertices(vertex, &neighbourVertices);
                    }
                    // No else

                    // Iterate over the vector of connected targets and add only those who could ever provide a better
                    // solution:
                    this->enqueueVertices(vertex, neighbourVertices);
                }
                // No else
            }
            // No else
        }

        void BITstar::SearchQueue::enqueueSamples(const VertexPtr &vertex, const VertexPtrVector& neighbourSamples, bool addAll)
        {
            // Iterate through the samples and add each one
            for (auto &targetSample : neighbourSamples)
            {
                // Is the target new? Do we care?
                if (addAll || targetSample->isNew())
                {
                    // It is new or we don't care, attempt to queue the edge.
                    this->enqueueEdgeConditionally(vertex, targetSample);
                }
                // No else, we've considered this edge before and we're being selective.
            }

            // Mark it as expanded
            vertex->markExpandedToSamples();
        }

        void BITstar::SearchQueue::enqueueVertices(const VertexPtr &vertex, const VertexPtrVector& neighbourVertices)
        {
            // Iterate over the vector of connected targets and add only those who could ever provide a better
            // solution:
            for (auto &targetVertex : neighbourVertices)
            {
                // Make sure it is not the root or myself.
                if (!targetVertex->isRoot() && targetVertex->getId() != vertex->getId())
                {
                    // Make sure I am not already the parent
                    if (targetVertex->getParent()->getId() != vertex->getId())
                    {
                        // Make sure the neighbour vertex is not already my parent:
                        if (vertex->isRoot())
                        {
                            // I am root, I have no parent, so attempt to queue the edge:
                            this->enqueueEdgeConditionally(vertex, targetVertex);
                        }
                        else if (targetVertex->getId() != vertex->getParent()->getId())
                        {
                            // The neighbour is not my parent, attempt to queue the edge:
                            this->enqueueEdgeConditionally(vertex, targetVertex);
                        }
                        // No else, this vertex is my parent.
                    }
                    // No else
                }
                // No else
            }

            // Mark the vertex as expanded into rewirings
            vertex->markExpandedToVertices();
        }

        void BITstar::SearchQueue::enqueueEdgeConditionally(const VertexPtr &parent, const VertexPtr &child)
        {
            // Variable:
            // The edge:
            VertexPtrPair newEdge;

            // Make the edge
            newEdge = std::make_pair(parent, child);

            // Should this edge be in the queue?
            if (this->edgeInsertCondition(newEdge))
            {
                this->enqueueEdge(newEdge);
            }
            // No else, it can never provide a better solution
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
                    if (graphPtr_->distanceFunction(kNearVertices->at(vertexPos), vertex) <
                        graphPtr_->distanceFunction(kNearSamples->at(samplePos), vertex))
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

        void BITstar::SearchQueue::resortVertex(const VertexPtr &unorderedVertex)
        {
            // Variables:
            // Whether the vertex is expanded.
            bool alreadyExpanded;

            // Test if it I am currently expanded.
            if (vertexToExpand_ == vertexQueue_.end())
            {
                // The token is at the end, therefore this vertex is in front of it:
                alreadyExpanded = true;
            }
            else if (this->queueComparison(unorderedVertex->getVertexQueueIter()->first, vertexToExpand_->first))
            {
                // This vertex is currently in the queue with a cost that is in front of the current token. It has been expanded:
                alreadyExpanded = true;
            }
            else
            {
                // Otherwise I have not been expanded yet.
                alreadyExpanded = false;
            }

#ifdef BITSTAR_DEBUG
            // Assert that unexpanded vertices have no outgoing edges in the queue
            if (!alreadyExpanded && unorderedVertex->hasOutgoingEdgeQueueEntries(numQueueResets_))
            {
                throw ompl::Exception("Unexpanded vertex has outgoing queue edges during a resort.");
            }
#endif  // BITSTAR_DEBUG

            // Update my place in the vertex queue by removing and adding myself:
            // Remove myself, not touching my edge-queue entries
            this->vertexRemoveHelper(unorderedVertex, false);

            // Reinsert myself, expanding if I cross the token if I am not already expanded but not removing/adding
            // either NN struct
            this->vertexInsertHelper(unorderedVertex, !alreadyExpanded, false, false);

            // If I was already expanded my edge-queue entries are out of date
            if (alreadyExpanded == true)
            {
                // I have been previously expanded.
                // Iterate over my outgoing edges and update them in the edge queue:
                for (auto edgePtr = unorderedVertex->outgoingEdgeQueuePtrsBeginConst(numQueueResets_);
                      edgePtr != unorderedVertex->outgoingEdgeQueuePtrsEndConst(numQueueResets_);
                      ++edgePtr)
                {
                    // Update the queue value
                    (*edgePtr)->data.first = this->edgeQueueValue((*edgePtr)->data.second);

                    // Update the entry in the queue
                    edgeQueue_.update(*edgePtr);
                }
            }
            // No else, I was not previously expanded so my edges (if there are now any) were created up to date
        }

        std::pair<unsigned int, unsigned int> BITstar::SearchQueue::pruneBranch(const VertexPtr &branchBase)
        {
#ifdef BITSTAR_DEBUG
            // Assert the vertex is in the tree
            if (branchBase->isInTree() == false)
            {
                throw ompl::Exception("Trying to prune a disconnected vertex. Something went wrong.");
            }
#endif  // BITSTAR_DEBUG

            // We must iterate over the children of this vertex and prune each one.
            // Then we must decide if this vertex (a) gets deleted or (b) placed back on the sample set.
            //(a) occurs if it has a lower-bound heuristic greater than the current solution
            //(b) occurs if it doesn't.

            // Variables:
            // The counter of vertices and samples pruned:
            std::pair<unsigned int, unsigned int> numPruned(1u, 0u);
            // The vector of my children:
            VertexPtrVector children;

            // Disconnect myself from my parent, not cascading costs as I know my children are also being disconnected:
            this->disconnectParent(branchBase, false);

            // Get the vector of children
            branchBase->getChildren(&children);

            // Remove myself from everything:
            numPruned.second = this->vertexRemoveHelper(branchBase, true);

            // Prune my children:
            for (auto &child : children)
            {
                // Prune my children:
                numPruned = numPruned + this->pruneBranch(child);
            }

            // Return the number pruned
            return numPruned;
        }

        void BITstar::SearchQueue::disconnectParent(const VertexPtr &oldVertex, bool cascadeCostUpdates)
        {
#ifdef BITSTAR_DEBUG
            if (oldVertex->hasParent() == false)
            {
                throw ompl::Exception("An orphaned vertex has been passed for disconnection. Something went wrong.");
            }
#endif  // BITSTAR_DEBUG

            // Check if my parent has already been pruned. This can occur if we're cascading vertex disconnections.
            if (!oldVertex->getParent()->isPruned())
            {
                // If not, remove myself from my parent's vector of children, not updating down-stream costs
                oldVertex->getParent()->removeChild(oldVertex);
            }

            // Remove my parent link, cascading cost updates if requested:
            oldVertex->removeParent(cascadeCostUpdates);
        }

        void BITstar::SearchQueue::vertexInsertHelper(const VertexPtr &newVertex, bool expandIfBeforeToken,
                                                      bool removeFromFree, bool addToNNStruct)
        {
            // Variable:
            // The iterator to the new edge in the queue:
            VertexQueueIter vertexIter;

            // Add the vertex to the graph
            if (addToNNStruct)
            {
                graphPtr_->addVertex(newVertex, removeFromFree);
            }

            // Insert into the order map, getting the iterator
            vertexIter = vertexQueue_.insert(std::make_pair(this->vertexQueueValue(newVertex), newVertex));

            // Store the iterator.
            newVertex->setVertexQueueIter(vertexIter);

            // Check if we are in front of the token and expand if so:
            if (vertexQueue_.size() == 1u)
            {
                // If the vertex queue is now of size 1, that means that this was the first vertex. Set the token to it
                // and don't even think of expanding anything:
                vertexToExpand_ = vertexQueue_.begin();
            }
            else if (expandIfBeforeToken)
            {
                /*
                There are 3ish cases:
                    1 The new vertex is immediately before the token.
                        a The token is not at the end: Don't expand and shift the token to the new vertex.
                        b The token is at the end: Don't expand and shift the token to the new vertex.
                    2 The new vertex is before the token, but *not* immediately (i.e., there are vertices between it):
                        a The token is at the end: Expand the vertex
                        b The token is not at the end: Expand the vertex
                    3 The new vertex is after the token: Don't expand. It cleanly goes into the vector of vertices to
                expand
                Note: By shifting the token, we assure that if the new vertex is better than the best edge, it will get
                expanded on the next pop.

                The cases look like this (-: expanded vertex, x: unexpanded vertex, X: token (next to expand), *: new
                vertex):
                We represent the token at the end with no X in the line:

                    1a: ---*Xxx   ->   ---Xxxx
                    1b: ------*   ->   ------X
                    2a: ---*---   ->   -------
                    2b: --*-Xxx   ->   ----Xxx
                    3: ---Xx*x   ->   ---Xxxx
                */

                // Variable:
                // The vertex before the token. Remember that since we have already added the new vertex, this could be
                // ourselves:
                VertexQueueIter preToken;

                // Get the vertex before the current token:
                preToken = vertexToExpand_;
                --preToken;

                // Check if we are immediately before: (1a & 1b)
                if (preToken == vertexIter)
                {
                    // The vertex before the token is the newly added vertex. Therefore we can just move the token up to
                    // the newly added vertex:
                    vertexToExpand_ = vertexIter;
                }
                else
                {
                    // We are not immediately before the token.

                    // Check if the token is at the end (2a)
                    if (vertexToExpand_ == vertexQueue_.end())
                    {
                        // It is. We've expanded the whole queue, and the new vertex isn't at the end of the queue.
                        // Expand!
                        this->expandVertex(newVertex);
                    }
                    else
                    {
                        // The token is not at the end. That means we can safely dereference it:
                        // Are we in front of it (2b)?
                        if (this->queueComparison(vertexIter->first, vertexToExpand_->first))
                        {
                            // We're before it, so expand it:
                            this->expandVertex(newVertex);
                        }
                        // No else, the vertex is behind the current token (3) and will get expanded as necessary.
                    }
                }
            }
            // No else, this vertex must have already been expanded
        }

        unsigned int BITstar::SearchQueue::vertexRemoveHelper(const VertexPtr &oldVertex, bool fullyRemove)
        {
            // Variables
            // The number of samples deleted (i.e., if this vertex is NOT recycled as a sample, this is a 1)
            unsigned int deleted = 0u;
#ifdef BITSTAR_DEBUG
            // The use count of the passed shared pointer. Used in debug mode to assert that we took ownership of our own copy.
            unsigned int initCount = oldVertex.use_count();
#endif  // BITSTAR_DEBUG
            // A copy of the vertex pointer to be removed so we can't delete it out from under ourselves (occurs when
            // this function is given an element of the maintained set as the argument)
            VertexPtr vertexToDelete(oldVertex);

#ifdef BITSTAR_DEBUG
            // Assert that the vertexToDelete took it's own copy
            if (vertexToDelete.use_count() <= initCount)
            {
                throw ompl::Exception("A code change has prevented SearchQueue::vertexRemoveHelper() "
                                      "from taking it's own copy of the given shared pointer. See "
                                      "https://bitbucket.org/ompl/ompl/issues/364/code-cleanup-breaking-bit");
            }
            // Check that the vertex is not connected to a parent:
            if (vertexToDelete->hasParent() == true && fullyRemove == true)
            {
                throw ompl::Exception("Cannot delete a vertex connected to a parent unless the vertex is being "
                                      "immediately reinserted, in which case fullyRemove should be false.");
            }
            // Assert there is something to delete:
            if (vertexQueue_.empty() == true)
            {
                std::cout << std::endl << "vId: " << vertexToDelete->getId() << std::endl;
                throw ompl::Exception("Removing a nonexistent vertex. Something went wrong.");
            }
#endif  // BITSTAR_DEBUG

            // Check if we need to move the expansion token:
            if (vertexToDelete->getVertexQueueIter() == vertexToExpand_)
            {
                // The token is this vertex, move it to the next:
                ++vertexToExpand_;
            }
            // No else, not the token.

            // Remove myself from the vertex queue:
            vertexQueue_.erase(vertexToDelete->getVertexQueueIter());
            vertexToDelete->clearVertexQueueIter();

            // Remove from lookups map as requested
            if (fullyRemove)
            {
                this->removeEdgesFrom(vertexToDelete);
                this->removeEdgesTo(vertexToDelete);

                // Remove myself from the set of connected vertices, this will recycle if necessary.
                deleted = graphPtr_->removeVertex(vertexToDelete, true);
            }

            // Return if the sample was deleted:
            return deleted;
        }

        BITstar::SearchQueue::CostDouble BITstar::SearchQueue::vertexQueueValue(const VertexPtr &vertex) const
        {
            // Construct and return an array
            return {{costHelpPtr_->currentHeuristicVertex(vertex), vertex->getCost()}};
        }

        BITstar::SearchQueue::CostTriple BITstar::SearchQueue::edgeQueueValue(const VertexPtrPair &edge) const
        {
            // Construct and return an array
            return {{costHelpPtr_->currentHeuristicEdge(edge), costHelpPtr_->currentHeuristicToTarget(edge),
                              edge.first->getCost()}};
        }

        template <std::size_t SIZE>
        bool BITstar::SearchQueue::queueComparison(const std::array<ompl::base::Cost, SIZE> &lhs,
                                                   const std::array<ompl::base::Cost, SIZE> &rhs) const
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
        void BITstar::SearchQueue::setStrictQueueOrdering(bool beStrict)
        {
            useStrictQueueOrdering_ = beStrict;
        }

        bool BITstar::SearchQueue::getStrictQueueOrdering() const
        {
            return useStrictQueueOrdering_;
        }

        void BITstar::SearchQueue::setDelayedRewiring(bool delayRewiring)
        {
            delayRewiring_ = delayRewiring;
        }

        bool BITstar::SearchQueue::getDelayedRewiring() const
        {
            return delayRewiring_;
        }

        void BITstar::SearchQueue::setPruneDuringResort(bool prune)
        {
            pruneDuringResort_ = prune;
        }

        bool BITstar::SearchQueue::getPruneDuringResort() const
        {
            return pruneDuringResort_;
        }

        unsigned int BITstar::SearchQueue::numEdgesPopped() const
        {
            return numEdgesPopped_;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    }  // geometric
}  // ompl
