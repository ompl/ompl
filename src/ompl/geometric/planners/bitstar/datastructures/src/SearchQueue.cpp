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
// The vertex class:
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
// The cost-helper class:
#include "ompl/geometric/planners/bitstar/datastructures/CostHelper.h"
// The implicit graph:
#include "ompl/geometric/planners/bitstar/datastructures/ImplicitGraph.h"

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::SearchQueue::SearchQueue(std::function<std::string()> nameFunc)
          : nameFunc_(std::move(nameFunc))
          , isSetup_(false)
          , costHelpPtr_()
          , graphPtr_()
          , vertexQueue_([this](const CostDouble &lhs, const CostDouble &rhs)
                         {
                             return queueComparison(lhs, rhs);
                         })  // This tells the vertexQueue_ to use the queueComparison for sorting
          , vertexToExpand_(vertexQueue_.begin())
          , edgeQueue_([this](const CostTriple &lhs, const CostTriple &rhs)
                       {
                           return queueComparison(lhs, rhs);
                       })  // This tells the edgeQueue_ to use the queueComparison for sorting
          , vertexIterLookup_()
          , outgoingEdges_()
          , incomingEdges_()
          , resortVertices_()
          , costThreshold_(std::numeric_limits<double>::infinity())  // Purposeful gibberish
          , hasExactSolution_(false)
          , numEdgesPopped_(0u)
          , useStrictQueueOrdering_(false)
          , delayRewiring_(true)
          , pruneDuringResort_(true)
        {
        }

        void BITstar::SearchQueue::setup(const CostHelperPtr &costHelpPtr, const ImplicitGraphPtr &graphPtr)
        {
            // Store that I am setup
            isSetup_ = true;

            // Get my copies
            costHelpPtr_ = costHelpPtr;
            graphPtr_ = graphPtr;

            // Set the the cost threshold to infinity to start:
            costThreshold_ = costHelpPtr_->infiniteCost();
        }

        void BITstar::SearchQueue::clear()
        {
            // Reset everything to the state of construction OTHER than planner name and settings/parameters
            // Keep this in the order of the constructors for easy verification:

            // Mark as cleared
            isSetup_ = false;

            // The pointers
            costHelpPtr_.reset();
            graphPtr_.reset();

            // The vertex queue:
            vertexQueue_.clear();
            vertexToExpand_ = vertexQueue_.begin();

            // The edge queue:
            edgeQueue_.clear();

            // The lookups:
            vertexIterLookup_.clear();
            outgoingEdges_.clear();
            incomingEdges_.clear();

            // The resort vector:
            resortVertices_.clear();

            // The cost threshold:
            costThreshold_ = ompl::base::Cost(std::numeric_limits<double>::infinity());

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
            this->confirmSetup();

            // Insert the vertex:
            this->vertexInsertHelper(newVertex, true, removeFromFree, true);
        }

        void BITstar::SearchQueue::enqueueEdge(const VertexPtrPair &newEdge)
        {
            this->confirmSetup();

            // Call my helper function:
            this->edgeInsertHelper(newEdge, edgeQueue_.end());
        }

        void BITstar::SearchQueue::enqueueEdge(const VertexPtr &sourceVertex, const VertexPtr &targetVertex)
        {
            this->confirmSetup();

            // Call my helper function:
            this->enqueueEdge(std::make_pair(sourceVertex, targetVertex));
        }

        void BITstar::SearchQueue::unqueueVertex(const VertexPtr &oldVertex)
        {
            this->confirmSetup();

            // Disconnect from parent if necessary, cascading cost updates:
            if (oldVertex->hasParent() == true)
            {
                this->disconnectParent(oldVertex, true);
            }

            // Remove it from vertex queue and lookup, and edge queues:
            this->vertexRemoveHelper(oldVertex, true);
        }

        BITstar::VertexPtr BITstar::SearchQueue::frontVertex()
        {
            this->confirmSetup();

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
            this->confirmSetup();

#ifdef BITSTAR_DEBUG
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Update the queue:
            this->updateQueue();

            // Return the front edge
            return edgeQueue_.begin()->second;
        }

        BITstar::SearchQueue::CostDouble BITstar::SearchQueue::frontVertexValue()
        {
            this->confirmSetup();

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
            this->confirmSetup();

#ifdef BITSTAR_DEBUG
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Update the queue:
            this->updateQueue();

            // Return the front value
            return edgeQueue_.begin()->first;
        }

        void BITstar::SearchQueue::popFrontEdge(VertexPtrPair *bestEdge)
        {
            this->confirmSetup();

#ifdef BITSTAR_DEBUG
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to pop an empty SearchQueue.");
            }
#endif  // BITSTAR_DEBUG

            // Update the queue:
            this->updateQueue();

            // Return the front:
            *bestEdge = edgeQueue_.begin()->second;

            // Erase the edge:
            this->edgeRemoveHelper(edgeQueue_.begin(), true, true);

            // Increment my counter
            ++numEdgesPopped_;
        }

        BITstar::VertexPtrPair BITstar::SearchQueue::popFrontEdge()
        {
            this->confirmSetup();

            VertexPtrPair rval;

            this->popFrontEdge(&rval);

            return rval;
        }

        void BITstar::SearchQueue::hasSolution(const ompl::base::Cost &solnCost)
        {
            this->confirmSetup();

            // Flag
            hasExactSolution_ = true;

            // Store
            costThreshold_ = solnCost;
        }

        void BITstar::SearchQueue::removeEdgesTo(const VertexPtr &cVertex)
        {
            this->confirmSetup();

            if (edgeQueue_.empty() == false)
            {
                // Variable:
                // The iterator to the vector of edges to the child:
                VertexIdToEdgeQueueIterVectorUMap::iterator vidAndEdgeQueueIterVectorPairAsUMapIterToDelete;

                // Get the vector of iterators
                vidAndEdgeQueueIterVectorPairAsUMapIterToDelete = incomingEdges_.find(cVertex->getId());

                // Make sure it was found before we start dereferencing it:
                if (vidAndEdgeQueueIterVectorPairAsUMapIterToDelete != incomingEdges_.end())
                {
                    // Iterate over the vector removing them from queue
                    for (auto &edgeQueueIter : vidAndEdgeQueueIterVectorPairAsUMapIterToDelete->second)
                    {
                        // Erase the edge, removing it from the *other* lookup. No need to remove from this lookup, as
                        // that's being cleared:
                        this->edgeRemoveHelper(edgeQueueIter, false, true);
                    }

                    // Clear the vector:
                    vidAndEdgeQueueIterVectorPairAsUMapIterToDelete->second.clear();
                }
                // No else, why was this called?
            }
            // No else, nothing to remove_to
        }

        void BITstar::SearchQueue::removeEdgesFrom(const VertexPtr &pVertex)
        {
            this->confirmSetup();

            if (edgeQueue_.empty() == false)
            {
                // Variable:
                // The iterator to the vector of edges from the parent:
                VertexIdToEdgeQueueIterVectorUMap::iterator vidAndEdgeQueueIterVectorPairAsUMapIterToDelete;

                // Get the vector of iterators
                vidAndEdgeQueueIterVectorPairAsUMapIterToDelete = outgoingEdges_.find(pVertex->getId());

                // Make sure it was found before we start dereferencing it:
                if (vidAndEdgeQueueIterVectorPairAsUMapIterToDelete != outgoingEdges_.end())
                {
                    // Iterate over the vector removing them from queue
                    for (auto &edgeQueueIter : vidAndEdgeQueueIterVectorPairAsUMapIterToDelete->second)
                    {
                        // Erase the edge, removing it from the *other* lookup. No need to remove from this lookup, as
                        // that's being cleared:
                        this->edgeRemoveHelper(edgeQueueIter, true, false);
                    }

                    // Clear the vector:
                    vidAndEdgeQueueIterVectorPairAsUMapIterToDelete->second.clear();
                }
                // No else, why was this called?
            }
            // No else, nothing to remove_from
        }

        void BITstar::SearchQueue::removeExtraEdgesTo(const VertexPtr &cVertex)
        {
            this->confirmSetup();

            if (edgeQueue_.empty() == false)
            {
                // Variable:
                // The iterator to the key,value of the child-lookup map, i.e., an iterator to a pair whose second is a
                // vector of edges to the child (which are actually iterators to the queue):
                VertexIdToEdgeQueueIterVectorUMap::iterator vectorOfEdgeQueueItersToVertexAsUMapIter;

                // Get my incoming edges as a vector of iterators
                vectorOfEdgeQueueItersToVertexAsUMapIter = incomingEdges_.find(cVertex->getId());

                // Make sure it was found before we start dereferencing it:
                if (vectorOfEdgeQueueItersToVertexAsUMapIter != incomingEdges_.end())
                {
                    // Variable
                    // The vector of edges to delete in the vector:
                    std::vector<EdgeQueueIterVector::iterator> edgeQueueItersToDelete;

                    // Iterate over the incoming edges and record those that are to be deleted
                    for (auto costAndEdgePairAsQueueIter = vectorOfEdgeQueueItersToVertexAsUMapIter->second.begin();
                         costAndEdgePairAsQueueIter != vectorOfEdgeQueueItersToVertexAsUMapIter->second.end();
                         ++costAndEdgePairAsQueueIter)
                    {
                        // Check if it would have been inserted
                        if (this->edgeInsertCondition((*costAndEdgePairAsQueueIter)->second) == false)
                        {
                            // It would not, delete
                            edgeQueueItersToDelete.push_back(costAndEdgePairAsQueueIter);
                        }
                        // No else, we're not deleting this iterator
                    }

                    // Now, iterate over the vector of iterators to delete
                    for (auto &edgeQueueIter : edgeQueueItersToDelete)
                    {
                        // Remove the edge and the edge iterator from the other lookup table:
                        this->edgeRemoveHelper(*edgeQueueIter, false, true);

                        // And finally erase the lookup iterator from the from lookup. If this was done first, the
                        // iterator would be invalidated for the above.
                        // Swap to the back
                        if (edgeQueueIter != (vectorOfEdgeQueueItersToVertexAsUMapIter->second.end() - 1))
                        {
                            std::swap(*edgeQueueIter, vectorOfEdgeQueueItersToVertexAsUMapIter->second.back());
                        }

                        // Delete off the back
                        vectorOfEdgeQueueItersToVertexAsUMapIter->second.pop_back();
                    }
                }
                // No else, nothing to delete
            }
            // No else, nothing to prune_to
        }

        void BITstar::SearchQueue::removeExtraEdgesFrom(const VertexPtr &pVertex)
        {
            this->confirmSetup();

            if (edgeQueue_.empty() == false)
            {
                // Variable:
                // The iterator to the key, value of the parent-lookup map, i.e., an iterator to a pair whose second is
                // a vector of edges from the child (which are actually iterators to the queue):
                VertexIdToEdgeQueueIterVectorUMap::iterator vectorOfEdgeQueueItersFromVertexAsUMapIter;

                // Get my outgoing edges as a vector of iterators
                vectorOfEdgeQueueItersFromVertexAsUMapIter = outgoingEdges_.find(pVertex->getId());

                // Make sure it was found before we start dereferencing it:
                if (vectorOfEdgeQueueItersFromVertexAsUMapIter != outgoingEdges_.end())
                {
                    // Variable
                    // The vector of edges to delete in the vector:
                    std::vector<EdgeQueueIterVector::iterator> edgeQueueItersToDelete;

                    // Iterate over the incoming edges and record those that are to be deleted
                    for (auto costAndEdgePairAsQueueIter = vectorOfEdgeQueueItersFromVertexAsUMapIter->second.begin();
                         costAndEdgePairAsQueueIter != vectorOfEdgeQueueItersFromVertexAsUMapIter->second.end();
                         ++costAndEdgePairAsQueueIter)
                    {
                        // Check if it would have been inserted
                        if (this->edgeInsertCondition((*costAndEdgePairAsQueueIter)->second) == false)
                        {
                            // It would not, delete
                            edgeQueueItersToDelete.push_back(costAndEdgePairAsQueueIter);
                        }
                        // No else, we're not deleting this iterator
                    }

                    // Now, iterate over the vector of iterators to delete
                    for (auto &edgeQueueIter : edgeQueueItersToDelete)
                    {
                        // Remove the edge and the edge iterator from the other lookup table:
                        this->edgeRemoveHelper(*edgeQueueIter, true, false);

                        // And finally erase the lookup iterator from the from lookup. If this was done first, the
                        // iterator would be invalidated for the above.
                        // Swap to the back
                        if (edgeQueueIter != (vectorOfEdgeQueueItersFromVertexAsUMapIter->second.end() - 1))
                        {
                            std::swap(*edgeQueueIter, vectorOfEdgeQueueItersFromVertexAsUMapIter->second.back());
                        }

                        // Delete off the back
                        vectorOfEdgeQueueItersFromVertexAsUMapIter->second.pop_back();
                    }
                }
                // No else, nothing to delete
            }
            // No else, nothing to prune_from
        }

        void BITstar::SearchQueue::markVertexUnsorted(const VertexPtr &vertex)
        {
            this->confirmSetup();

            resortVertices_.push_back(vertex);
        }

        std::pair<unsigned int, unsigned int> BITstar::SearchQueue::prune(const VertexConstPtr &goalVertexPtr)
        {
            this->confirmSetup();

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
            // The iterator into the lookup helper:
            VertexIdToVertexQueueIterUMap::iterator lookupIter;
            // The iterator into the queue:
            VertexQueueIter queueIter;

            // Get the iterator to the queue of the given starting point.
            lookupIter = vertexIterLookup_.find(goalVertexPtr->getId());

#ifdef BITSTAR_DEBUG
            // Check that it was found
            if (lookupIter == vertexIterLookup_.end())
            {
                // Complain
                throw ompl::Exception("The provided starting point is not in the queue?");
            }
#endif  // BITSTAR_DEBUG

            // Get the vertex queue iterator:
            queueIter = lookupIter->second;

            // Iterate through to the end of the queue
            while (queueIter != vertexQueue_.end())
            {
                // Check if it should be pruned (value) or has lost its parent.
                if (this->vertexPruneCondition(queueIter->second) == true)
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
            this->confirmSetup();

            // Iterate through every vertex marked for resorting:
            if (resortVertices_.empty() == false)
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
                        if (vIdAndPtrPair.second->isPruned() == false)
                        {
                            // Make sure it has not already been returned to the set of samples:
                            if (vIdAndPtrPair.second->isInTree() == true)
                            {
                                // If this was a new vertex, would we *not* insert it in the queue (and do we have
                                // "permission" not to do so)?
                                if (pruneDuringResort_ == true &&
                                    this->vertexInsertCondition(vIdAndPtrPair.second) == false)
                                {
                                    // The vertex should just be pruned and forgotten about.
                                    // Prune the branch:
                                    numPruned = numPruned + this->pruneBranch(vIdAndPtrPair.second);
                                }
                                else
                                {
                                    // The vertex is going to be kept.
                                    // Does it have any children?
                                    if (vIdAndPtrPair.second->hasChildren() == true)
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

                                    // Reinsert the vertex:
                                    this->reinsertVertex(vIdAndPtrPair.second);
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
                // NO else, sshhh
            }
            // No else, nothing to resort
        }

        void BITstar::SearchQueue::finish()
        {
            this->confirmSetup();

            // Is there anything to resort before we're marked as finished?
            if (resortVertices_.empty() == false)
            {
                OMPL_DEBUG("%s (SearchQueue): Resorting an unsorted queue instead of marking it as finished.",
                           nameFunc_().c_str());
                this->resort();
            }
            else
            {
                // Clear the edge containers:
                edgeQueue_.clear();
                outgoingEdges_.clear();
                incomingEdges_.clear();

                // Move the token to the end:
                vertexToExpand_ = vertexQueue_.end();

                // Do NOT clear:
                //  - vertexIterLookup_ (it's still valid)
            }
        }

        void BITstar::SearchQueue::reset()
        {
            this->confirmSetup();

#ifdef BITSTAR_DEBUG
            if (resortVertices_.empty() == false)
            {
                throw ompl::Exception("SearchQueue::reset() called with an unsorted queue.");
            }
#endif  // BITSTAR_DEBUG

            // Clear the edge containers:
            edgeQueue_.clear();
            outgoingEdges_.clear();
            incomingEdges_.clear();

            // The resort vector:
            resortVertices_.clear();

            // Restart the expansion queue:
            vertexToExpand_ = vertexQueue_.begin();
        }

        bool BITstar::SearchQueue::vertexInsertCondition(const VertexPtr &state) const
        {
            this->confirmSetup();

            // Threshold should always be g_t(x_g)

            // Can it ever be a better solution?
            // Just in case we're using a vertex that is exactly optimally connected
            // g^(v) + h^(v) <= g_t(x_g)?
            return costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(state),
                                                                costThreshold_);
        }

        bool BITstar::SearchQueue::edgeInsertCondition(const VertexPtrPair &edge) const
        {
            this->confirmSetup();

            bool rval;

            // Can it ever be a better solution? Less-than-equal to just in case we're using an edge that is exactly
            // optimally connected
            // g^(v) + c^(v,x) + h^(x) <= g_t(x_g)?
            rval = costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicEdge(edge),
                                                                costThreshold_);

            // If the child is connected already, we need to check if we could do better than it's current connection.
            // But only if we're inserting the edge
            if (edge.second->hasParent() == true && rval == true)
            {
                // Can it ever be a better path to the vertex? Less-than-equal to just in case we're using an edge that
                // is exactly optimally connected
                // g^(v) + c^(v,x) <= g_t(x)?
                rval = costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicTarget(edge),
                                                                    edge.second->getCost());  // Ever rewire?
            }

            return rval;
        }

        bool BITstar::SearchQueue::vertexPruneCondition(const VertexPtr &state) const
        {
            this->confirmSetup();

            // Threshold should always be g_t(x_g)

            // Prune the vertex if it could cannot part of a better solution in the current graph.  Greater-than just in
            // case we're using an edge that is exactly optimally connected.
            // g_t(v) + h^(v) > g_t(x_g)?
            return costHelpPtr_->isCostWorseThan(costHelpPtr_->currentHeuristicVertex(state), costThreshold_);
        }

        bool BITstar::SearchQueue::samplePruneCondition(const VertexPtr &state) const
        {
            this->confirmSetup();

            // Threshold should always be g_t(x_g)
            // Prune the unconnected sample if it could never be better of a better solution.
            // g^(v) + h^(v) >= g_t(x_g)?
            return costHelpPtr_->isCostWorseThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(state),
                                                               costThreshold_);
        }

        bool BITstar::SearchQueue::edgePruneCondition(const VertexPtrPair &edge) const
        {
            this->confirmSetup();

            bool rval;
            // Threshold should always be g_t(x_g)

            // Prune the edge if it could cannot part of a better solution in the current graph.  Greater-than just in
            // case we're using an edge that is exactly optimally connected.
            // g_t(v) + c^(v,x) + h^(x) > g_t(x_g)?
            rval = costHelpPtr_->isCostWorseThan(costHelpPtr_->currentHeuristicEdge(edge), costThreshold_);

            // If the child is connected already, we need to check if we could do better than it's current connection.
            // But only if we're not pruning based on the first check
            if (edge.second->hasParent() == true && rval == false)
            {
                // Can it ever be a better path to the vertex in the current graph? Greater-than to just in case we're
                // using an edge that is exactly optimally connected
                // g_t(v) + c^(v,x) > g_t(x)?
                rval = costHelpPtr_->isCostWorseThan(costHelpPtr_->currentHeuristicTarget(edge),
                                                     edge.second->getCost());  // Currently rewire?
            }

            return rval;
        }

        unsigned int BITstar::SearchQueue::numEdges()
        {
            this->confirmSetup();

            // Update the queue:
            this->updateQueue();

            return edgeQueue_.size();
        }

        unsigned int BITstar::SearchQueue::numVertices()
        {
            this->confirmSetup();

            // Update the queue:
            this->updateQueue();

            // Variables:
            // The number of vertices left to expand:
            unsigned int numToExpand;

            // Start at 0:
            numToExpand = 0u;

            // Iterate until the end:
            for (QValueToVertexMMap::const_iterator vIter = vertexToExpand_; vIter != vertexQueue_.end(); ++vIter)
            {
                // Increment counter:
                ++numToExpand;
            }

            // Return
            return numToExpand;
        }

        unsigned int BITstar::SearchQueue::numEdgesTo(const VertexPtr &cVertex)
        {
            this->confirmSetup();

            // Update the queue:
            this->updateQueue();

            // Variables:
            // The number of edges to, starting at 0:
            unsigned int rval = 0u;

            // Is there anything to count?
            if (edgeQueue_.empty() == false)
            {
                // Variable:
                // The iterator to the vector of edges to the child:
                VertexIdToEdgeQueueIterVectorUMap::const_iterator toIter;

                // Get the vector of iterators
                toIter = incomingEdges_.find(cVertex->getId());

                // Make sure it was found before we dereferencing it:
                if (toIter != incomingEdges_.end())
                {
                    rval = toIter->second.size();
                }
                // No else, there are none.
            }
            // No else, there is nothing.

            // Return:
            return rval;
        }

        unsigned int BITstar::SearchQueue::numEdgesFrom(const VertexPtr &pVertex)
        {
            this->confirmSetup();

            // Update the queue:
            this->updateQueue();

            // Variables:
            // The number of edges to, starting at 0:
            unsigned int rval = 0u;

            // Is there anything to count?
            if (edgeQueue_.empty() == false)
            {
                // Variable:
                // The iterator to the vector of edges from the parent:
                VertexIdToEdgeQueueIterVectorUMap::const_iterator toIter;

                // Get the vector of iterators
                toIter = outgoingEdges_.find(pVertex->getId());

                // Make sure it was found before we dereferencing it:
                if (toIter != outgoingEdges_.end())
                {
                    rval = toIter->second.size();
                }
                // No else, 0u.
            }
            // No else, there is nothing.

            // Return
            return rval;
        }

        unsigned int BITstar::SearchQueue::numUnsorted() const
        {
            this->confirmSetup();

            return resortVertices_.size();
        }

        bool BITstar::SearchQueue::isSorted() const
        {
            this->confirmSetup();

            return resortVertices_.empty();
        }

        bool BITstar::SearchQueue::isReset() const
        {
            this->confirmSetup();

            return (vertexToExpand_ == vertexQueue_.begin() && edgeQueue_.empty());
        }

        bool BITstar::SearchQueue::isEmpty()
        {
            this->confirmSetup();

            // Update the queue:
            this->updateQueue();

            // Expand if the edge queue is empty but the vertex queue is not:
            while (edgeQueue_.empty() && vertexToExpand_ != vertexQueue_.end())
            {
                // Expand the next vertex, this pushes the token:
                this->expandNextVertex();
            }

            // Return whether the edge queue is empty:
            return edgeQueue_.empty();
        }

        bool BITstar::SearchQueue::isVertexExpanded(const VertexConstPtr &vertex) const
        {
            this->confirmSetup();

            // Variable
            // The vertex iterator
            VertexIdToVertexQueueIterUMap::const_iterator lkupIter;

            // Get the lookup iterator for the provided vertex
            lkupIter = vertexIterLookup_.find(vertex->getId());

#ifdef BITSTAR_DEBUG
            if (lkupIter == vertexIterLookup_.end())
            {
                throw ompl::Exception("Attempting to check the expansion status of a vertex not in the queue");
            }
#endif  // BITSTAR_DEBUG

            // Compare the value used to currently sort the vertex in the queue to the value of the token.
            if (vertexToExpand_ == vertexQueue_.end())
            {
                // If the token is at the end of the queue, obviously the vertex is expanded:
                return true;
            }
            else
            {
                // By virtue of the vertex expansion rules, the token will always sit at the front of a group of
                // equivalent cost vertices (that is to say, all vertices with the same cost get expanded at the same
                // time). Therefore, the vertex is expanded if it's cost is strictly better than the token.
                return this->queueComparison(lkupIter->second->first, vertexToExpand_->first);
            }
        }

        void BITstar::SearchQueue::getVertices(VertexConstPtrVector *vertexQueue)
        {
            this->confirmSetup();

            // Update the queue:
            this->updateQueue();

            // Clear the given vector:
            vertexQueue->clear();

            // Iterate until the end, pushing back:
            for (QValueToVertexMMap::const_iterator vIter = vertexToExpand_; vIter != vertexQueue_.end(); ++vIter)
            {
                // Push back:
                vertexQueue->push_back(vIter->second);
            }
        }

        void BITstar::SearchQueue::getEdges(VertexConstPtrPairVector *edgeQueue)
        {
            this->confirmSetup();

            // Update the queue:
            this->updateQueue();

            // Clear the vector
            edgeQueue->clear();

            // I don't think there's a std::copy way to do this, so just iterate
            for (const auto &queueElement : edgeQueue_)
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
            if (useStrictQueueOrdering_ == true)
            {
                // Resort and vertices that have been rewired.
                this->resort();
            }

            // Variables:
            // Whether to expand:
            bool expand;

            expand = true;
            while (expand == true)
            {
                // Check if there are any vertices to expand:
                if (vertexToExpand_ != vertexQueue_.end())
                {
                    // Expand a vertex if the edge queue is empty, or the vertex could place a better edge into it:
                    if (edgeQueue_.empty() == true)
                    {
                        // The edge queue is empty, any edge is better than this!
                        this->expandNextVertex();
                    }
                    // This is isCostBetterThanOrEquivalentTo because of the second ordering criteria. The vertex
                    // expanded could match the edge in queue on total cost, but have less cost-to-come.
                    else if (costHelpPtr_->isCostBetterThanOrEquivalentTo(vertexToExpand_->first.at(0u),
                                                                          edgeQueue_.cbegin()->first.at(0u)) == true)
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
            if (this->vertexInsertCondition(vertexToExpand_->second) == true)
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
            // Should we expand this vertex?
            if (this->vertexInsertCondition(vertex) == true)
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
                if (graphPtr_->getUseKNearest() == true)
                {
                    // Get the set of nearby vertices
                    graphPtr_->nearestVertices(vertex, &neighbourVertices);

                    // Post process them:
                    this->processKNearest(vertex, &neighbourSamples, &neighbourVertices);
                }
                // No else

                // Add edges to unconnected targets who could ever provide a better solution:
                // Has the vertex been expanded into edges towards unconnected samples before?
                if (vertex->hasBeenExpandedToSamples() == false)
                {
                    // It has not, that means none of its outgoing edges have been considered. Add them all
                    for (auto &targetSample : neighbourSamples)
                    {
                        // Attempt to queue the edge.
                        this->enqueueEdgeConditionally(vertex, targetSample);
                    }

                    // Mark it as expanded
                    vertex->markExpandedToSamples();
                }
                else
                {
                    // It has, which means that outgoing edges to old unconnected vertices have already been considered.
                    // Only add those that lead to new vertices
                    for (auto &targetSample : neighbourSamples)
                    {
                        // Is the target new?
                        if (targetSample->isNew() == true)
                        {
                            // It is, attempt to queue the edge.
                            this->enqueueEdgeConditionally(vertex, targetSample);
                        }
                        // No else, we've considered this edge before.
                    }
                }

                // If the vertex has never been expanded into possible rewiring edges *and* either we're not delaying
                // rewiring or we have a solution, we add those rewiring candidates:
                if (vertex->hasBeenExpandedToVertices() == false &&
                    (delayRewiring_ == false || hasExactSolution_ == true))
                {
                    // If we're using an r-disc RGG, we will not have gotten the neighbour vertices yet, get them now
                    if (graphPtr_->getUseKNearest() == false)
                    {
                        // Get the set of nearby vertices
                        graphPtr_->nearestVertices(vertex, &neighbourVertices);
                    }
                    // No else

                    // Iterate over the vector of connected targets and add only those who could ever provide a better
                    // solution:
                    for (auto &targetVertex : neighbourVertices)
                    {
                        // Make sure it is not the root or myself.
                        if (targetVertex->isRoot() == false && targetVertex->getId() != vertex->getId())
                        {
                            // Make sure I am not already the parent
                            if (targetVertex->getParent()->getId() != vertex->getId())
                            {
                                // Make sure the neighbour vertex is not already my parent:
                                if (vertex->isRoot() == true)
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
                // No else
            }
            // No else
        }

        void BITstar::SearchQueue::enqueueEdgeConditionally(const VertexPtr &parent, const VertexPtr &child)
        {
            // Variable:
            // The edge:
            VertexPtrPair newEdge;

            // Make the edge
            newEdge = std::make_pair(parent, child);

            // Should this edge be in the queue?
            if (this->edgeInsertCondition(newEdge) == true)
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

        void BITstar::SearchQueue::reinsertVertex(const VertexPtr &unorderedVertex)
        {
            // Variables:
            // Whether the vertex is expanded.
            bool alreadyExpanded;
            // My entry in the vertex lookup:
            VertexIdToVertexQueueIterUMap::iterator myLookup;
            // The vector of edges from the vertex:
            VertexIdToEdgeQueueIterVectorUMap::iterator vertexAndEdgeQueueVectorVectorPairAsIter;

            // Get my iterator:
            myLookup = vertexIterLookup_.find(unorderedVertex->getId());

#ifdef BITSTAR_DEBUG
            // Assert it was found
            if (myLookup == vertexIterLookup_.end())
            {
                throw ompl::Exception("Vertex to reinsert is not in the lookup. Something went wrong.");
            }
#endif  // BITSTAR_DEBUG

            // Test if it I am currently expanded.
            if (vertexToExpand_ == vertexQueue_.end())
            {
                // The token is at the end, therefore this vertex is in front of it:
                alreadyExpanded = true;
            }
            else if (this->queueComparison(myLookup->second->first, vertexToExpand_->first) == true)
            {
                // This vertex was entered into the queue with a cost that is in front of the current token:
                alreadyExpanded = true;
            }
            else
            {
                // Otherwise I have not been expanded yet.
                alreadyExpanded = false;
            }

            // Remove myself, not touching my lookup entries
            this->vertexRemoveHelper(unorderedVertex, false);

            // Reinsert myself, expanding if I cross the token if I am not already expanded but not removing/adding to
            // either NN struct
            this->vertexInsertHelper(unorderedVertex, alreadyExpanded == false, false, false);

            // Iterate over my outgoing edges and reinsert them in the queue:
            // Get my vector of outgoing edges
            vertexAndEdgeQueueVectorVectorPairAsIter = outgoingEdges_.find(unorderedVertex->getId());

            // Reinsert the edges:
            if (vertexAndEdgeQueueVectorVectorPairAsIter != outgoingEdges_.end())
            {
                // Variables
                // The iterators to the edge queue from this vertex
                EdgeQueueIterVector edgeQueueItersToResort;

                // Copy the iters to resort
                edgeQueueItersToResort = vertexAndEdgeQueueVectorVectorPairAsIter->second;

                // Clear the outgoing lookup
                vertexAndEdgeQueueVectorVectorPairAsIter->second.clear();

                // Iterate over the vector of iters to resort, inserting each one as a new edge, and then removing it as
                // an iterator from the edge queue and the incoming lookup
                for (auto &costAndEdgePairAsQueueIter : edgeQueueItersToResort)
                {
                    // Check if the edge should be reinserted
                    if (this->edgeInsertCondition(costAndEdgePairAsQueueIter->second) == true)
                    {
                        // Call helper to reinsert. Looks after lookups, hint at the location it's coming out of
                        this->edgeInsertHelper(costAndEdgePairAsQueueIter->second, costAndEdgePairAsQueueIter);
                    }
                    // No else, prune.

                    // Remove the old edge and its entry in the incoming lookup. No need to remove from this lookup, as
                    // that's been cleared:
                    this->edgeRemoveHelper(costAndEdgePairAsQueueIter, true, false);
                }
            }
            // No else, no edges from this vertex to requeue
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
            if (oldVertex->getParent()->isPruned() == false)
            {
                // If not, remove myself from my parent's vector of children, not updating down-stream costs
                oldVertex->getParent()->removeChild(oldVertex, false);
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
            if (addToNNStruct == true)
            {
                graphPtr_->addVertex(newVertex, removeFromFree);
            }

            // Insert into the order map, getting the iterator
            vertexIter = vertexQueue_.insert(std::make_pair(this->vertexQueueValue(newVertex), newVertex));

            // Store the iterator in the lookup. This will create if necessary and otherwise lookup
            vertexIterLookup_[newVertex->getId()] = vertexIter;

            // Check if we are in front of the token and expand if so:
            if (vertexQueue_.size() == 1u)
            {
                // If the vertex queue is now of size 1, that means that this was the first vertex. Set the token to it
                // and don't even think of expanding anything:
                vertexToExpand_ = vertexQueue_.begin();
            }
            else if (expandIfBeforeToken == true)
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
                        if (this->queueComparison(this->vertexQueueValue(newVertex), vertexToExpand_->first) == true)
                        {
                            // We're before it, so expand it:
                            this->expandVertex(newVertex);
                        }
                        // No else, the vertex is behind the current token (3) and will get expanded as necessary.
                    }
                }
            }
        }

        unsigned int BITstar::SearchQueue::vertexRemoveHelper(const VertexPtr &oldVertex, bool fullyRemove)
        {
            // Variable
            // The number of samples deleted (i.e., if this vertex is NOT recycled as a sample, this is a 1)
            unsigned int deleted = 0u;
            // A copy of the vertex pointer to be removed so we can't delete it out from under ourselves (occurs when
            // this function is given an element of the maintained set as the argument)
            VertexPtr vertexToDelete(oldVertex);
            // The iterator into the lookup:
            VertexIdToVertexQueueIterUMap::iterator lookupIter = vertexIterLookup_.find(vertexToDelete->getId());

#ifdef BITSTAR_DEBUG
            // Check that the vertex is not connected to a parent:
            if (vertexToDelete->hasParent() == true && fullyRemove == true)
            {
                throw ompl::Exception("Cannot delete a vertex connected to a parent unless the vertex is being "
                                      "immediately reinserted, in which case fullyRemove should be false.");
            }
            // Assert there is something to delete:
            if (vertexQueue_.empty() == true)
            {
                std::cout << std::endl
                          << "vId: " << vertexToDelete->getId() << std::endl;
                throw ompl::Exception("Removing a nonexistent vertex. Something went wrong.");
            }
            // Assert that it was found
            if (lookupIter == vertexIterLookup_.end())
            {
                std::cout << std::endl
                          << "vId: " << vertexToDelete->getId() << std::endl;
                throw ompl::Exception("Deleted vertex is not found in lookup. Something went wrong.");
            }
#endif  // BITSTAR_DEBUG

            // Check if we need to move the expansion token:
            if (lookupIter->second == vertexToExpand_)
            {
                // It is the token, move it to the next:
                ++vertexToExpand_;
            }
            // No else, not the token.

            // Remove myself from the vertex queue:
            vertexQueue_.erase(lookupIter->second);

            // Remove from lookups map as requested
            if (fullyRemove == true)
            {
                vertexIterLookup_.erase(lookupIter);
                this->removeEdgesFrom(vertexToDelete);
                this->removeEdgesTo(vertexToDelete);

                // Remove myself from the set of connected vertices, this will recycle if necessary.
                deleted = graphPtr_->removeVertex(vertexToDelete, true);
            }

            // Return if the sample was deleted:
            return deleted;
        }

        void BITstar::SearchQueue::edgeInsertHelper(const VertexPtrPair &newEdge, EdgeQueueIter positionHint)
        {
            // Variable:
            // The iterator to the new edge in the queue:
            EdgeQueueIter edgeIter;

            // Insert into the edge queue, getting the iter
            if (positionHint == edgeQueue_.end())
            {
                // No hint, insert:
                edgeIter = edgeQueue_.insert(std::make_pair(this->edgeQueueValue(newEdge), newEdge));
            }
            else
            {
                // Insert with hint:
                edgeIter = edgeQueue_.insert(positionHint, std::make_pair(this->edgeQueueValue(newEdge), newEdge));
            }

            // Push the newly created edge back on the vector of edges from the parent.
            // The [] return an reference to the existing entry, or create a new entry:
            outgoingEdges_[newEdge.first->getId()].push_back(edgeIter);

            // Push the newly created edge back on the vector of edges from the child.
            // The [] return an reference to the existing entry, or create a new entry:
            incomingEdges_[newEdge.second->getId()].push_back(edgeIter);
        }

        void BITstar::SearchQueue::edgeRemoveHelper(const EdgeQueueIter &oldEdgeIter, bool rmIncomingLookup,
                                                    bool rmOutgoingLookup)
        {
            // Erase the lookup tables:
            if (rmIncomingLookup == true)
            {
                // Erase the entry in the outgoing lookup table:
                this->rmEdgeLookupHelper(incomingEdges_, oldEdgeIter->second.second->getId(), oldEdgeIter);
            }
            // No else

            if (rmOutgoingLookup == true)
            {
                // Erase  the entry in the ingoing lookup table:
                this->rmEdgeLookupHelper(outgoingEdges_, oldEdgeIter->second.first->getId(), oldEdgeIter);
            }
            // No else

            // Finally erase from the queue:
            edgeQueue_.erase(oldEdgeIter);
        }

        void BITstar::SearchQueue::rmEdgeLookupHelper(VertexIdToEdgeQueueIterVectorUMap &lookup,
                                                      const BITstar::VertexId &idx, const EdgeQueueIter &mmapIterToRm)
        {
            // Variable:
            // An iterator to the vertex,vector pair in the lookup
            VertexIdToEdgeQueueIterVectorUMap::iterator iterToVertexVectorPair;
            // Whether I've found the mmapIterToRm in my vector:
            bool found = false;
            // The iterator to the mmapIterToRm in my vector:
            EdgeQueueIterVector::iterator iterToVector;

            // Get the vector in the lookup for the given index:
            iterToVertexVectorPair = lookup.find(idx);

#ifdef BITSTAR_DEBUG
            // Make sure it was actually found before derefencing it:
            if (iterToVertexVectorPair == lookup.end())
            {
                throw ompl::Exception("Indexing vertex not found in lookup hash.");
            }
#endif  // BITSTAR_DEBUG

            // Start at the front:
            iterToVector = iterToVertexVectorPair->second.begin();

            // Iterate through the vector and find mmapIterToRm
            while (found == false && iterToVector != iterToVertexVectorPair->second.end())
            {
                // Compare the value in the vector to the target:
                if (*iterToVector == mmapIterToRm)
                {
                    // Mark as found:
                    found = true;

                    // Swap it to the back
                    if (iterToVector != (iterToVertexVectorPair->second.end() - 1))
                    {
                        std::swap(*iterToVector, iterToVertexVectorPair->second.back());
                    }

                    // Delete it off the back
                    iterToVertexVectorPair->second.pop_back();
                }
                else
                {
                    // Increment the iterator:
                    ++iterToVector;
                }
            }

#ifdef BITSTAR_DEBUG
            // Make sure it was actually found:
            if (found == false)
            {
                throw ompl::Exception("Edge iterator not found under given index in lookup hash.");
            }
#endif  // BITSTAR_DEBUG
        }

        BITstar::SearchQueue::CostDouble BITstar::SearchQueue::vertexQueueValue(const VertexPtr &vertex) const
        {
            // Construct and return an array
            return CostDouble{costHelpPtr_->currentHeuristicVertex(vertex), vertex->getCost()};
        }

        BITstar::SearchQueue::CostTriple BITstar::SearchQueue::edgeQueueValue(const VertexPtrPair &edge) const
        {
            // Construct and return an array
            return CostTriple{costHelpPtr_->currentHeuristicEdge(edge), costHelpPtr_->currentHeuristicTarget(edge),
                              edge.first->getCost()};
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

        void BITstar::SearchQueue::confirmSetup() const
        {
#ifdef BITSTAR_DEBUG
            if (isSetup_ == false)
            {
                throw ompl::Exception("BITstar::SearchQueue was used before it was setup.");
            }
#endif  // BITSTAR_DEBUG
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
