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
#include "ompl/geometric/planners/bitstar/datastructures/IntegratedQueue.h"

// For std::move
#include <utility>
// For std::lexicographical_compare
#include <algorithm>

// OMPL:
// For exceptions:
#include "ompl/util/Exception.h"

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::IntegratedQueue::IntegratedQueue(std::shared_ptr<CostHelper> costHelpPtr, DistanceFunc distanceFunc,
                                                  NeighbourhoodFunc nearSamplesFunc, NeighbourhoodFunc nearVerticesFunc)
          : costHelpPtr_(std::move(costHelpPtr))
          , distanceFunc_(std::move(distanceFunc))
          , nearSamplesFunc_(std::move(nearSamplesFunc))
          , nearVerticesFunc_(std::move(nearVerticesFunc))
          , delayRewiring_(true)
          , outgoingLookupTables_(true)
          , incomingLookupTables_(true)
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
        {
            // Set the the cost threshold to infinity to start:
            costThreshold_ = costHelpPtr_->infiniteCost();
        }

        void BITstar::IntegratedQueue::insertVertex(const VertexPtr &newVertex)
        {
            // Insert the vertex:
            this->vertexInsertHelper(newVertex, true);
        }

        void BITstar::IntegratedQueue::insertEdge(const VertexPtrPair &newEdge)
        {
            // Call my helper function:
            this->edgeInsertHelper(newEdge, edgeQueue_.end());
        }

        void BITstar::IntegratedQueue::eraseVertex(const VertexPtr &oldVertex, bool disconnectParent,
                                                   const VertexPtrNNPtr &vertexNN, const VertexPtrNNPtr &freeStateNN,
                                                   VertexPtrVector *recycledVertices)
        {
            // If requested, disconnect from parent, cascading cost updates:
            if (disconnectParent == true)
            {
                this->disconnectParent(oldVertex, true);
            }

            // Remove it from vertex queue and lookup, and edge queues (as requested):
            this->vertexRemoveHelper(oldVertex, vertexNN, freeStateNN, recycledVertices, true);
        }

        BITstar::VertexPtr BITstar::IntegratedQueue::frontVertex()
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty IntegratedQueue.");
            }

            // Update the queue:
            this->updateQueue();

            // Return the front edge
            return vertexQueue_.begin()->second;
        }

        BITstar::VertexPtrPair BITstar::IntegratedQueue::frontEdge()
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty IntegratedQueue.");
            }

            // Update the queue:
            this->updateQueue();

            // Return the front edge
            return edgeQueue_.begin()->second;
        }

        BITstar::IntegratedQueue::CostDouble BITstar::IntegratedQueue::frontVertexValue()
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty IntegratedQueue.");
            }

            // Update the queue:
            this->updateQueue();

            // Return the front value
            return vertexQueue_.begin()->first;
        }

        BITstar::IntegratedQueue::CostTriple BITstar::IntegratedQueue::frontEdgeValue()
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to access the first element in an empty IntegratedQueue.");
            }

            // Update the queue:
            this->updateQueue();

            // Return the front value
            return edgeQueue_.begin()->first;
        }

        void BITstar::IntegratedQueue::popFrontEdge(VertexPtrPair *bestEdge)
        {
            if (this->isEmpty() == true)
            {
                throw ompl::Exception("Attempted to pop an empty IntegratedQueue.");
            }

            // Update the queue:
            this->updateQueue();

            // Return the front:
            *bestEdge = edgeQueue_.begin()->second;

            // Erase the edge:
            this->edgeRemoveHelper(edgeQueue_.begin(), true, true);
        }

        BITstar::VertexPtrPair BITstar::IntegratedQueue::popFrontEdge()
        {
            VertexPtrPair rval;

            this->popFrontEdge(&rval);

            return rval;
        }

        void BITstar::IntegratedQueue::hasSolution()
        {
            hasExactSolution_ = true;
        }

        void BITstar::IntegratedQueue::setThreshold(const ompl::base::Cost &costThreshold)
        {
            costThreshold_ = costThreshold;
        }

        ompl::base::Cost BITstar::IntegratedQueue::getThreshold() const
        {
            return costThreshold_;
        }

        void BITstar::IntegratedQueue::removeEdgesTo(const VertexPtr &cVertex)
        {
            if (edgeQueue_.empty() == false)
            {
                if (incomingLookupTables_ == true)
                {
                    // Variable:
                    // The iterator to the vector of edges to the child:
                    VertexIdToEdgeQueueIterListUMap::iterator vidAndEdgeQueueIterListPairAsUMapIterToDelete;

                    // Get the vector of iterators
                    vidAndEdgeQueueIterListPairAsUMapIterToDelete = incomingEdges_.find(cVertex->getId());

                    // Make sure it was found before we start dereferencing it:
                    if (vidAndEdgeQueueIterListPairAsUMapIterToDelete != incomingEdges_.end())
                    {
                        // Iterate over the vector removing them from queue
                        for (auto &edgeQueueIter : vidAndEdgeQueueIterListPairAsUMapIterToDelete->second)
                        {
                            // Erase the edge, removing it from the *other* lookup. No need to remove from this lookup,
                            // as that's being cleared:
                            this->edgeRemoveHelper(edgeQueueIter, false, true);
                        }

                        // Clear the list:
                        vidAndEdgeQueueIterListPairAsUMapIterToDelete->second = EdgeQueueIterList();
                    }
                    // No else, why was this called?
                }
                else
                {
                    throw ompl::Exception("Child lookup is not enabled for this instance of the container.");
                }
            }
            // No else, nothing to remove_to
        }

        void BITstar::IntegratedQueue::removeEdgesFrom(const VertexPtr &pVertex)
        {
            if (edgeQueue_.empty() == false)
            {
                if (outgoingLookupTables_ == true)
                {
                    // Variable:
                    // The iterator to the vector of edges from the parent:
                    VertexIdToEdgeQueueIterListUMap::iterator vidAndEdgeQueueIterListPairAsUMapIterToDelete;

                    // Get the vector of iterators
                    vidAndEdgeQueueIterListPairAsUMapIterToDelete = outgoingEdges_.find(pVertex->getId());

                    // Make sure it was found before we start dereferencing it:
                    if (vidAndEdgeQueueIterListPairAsUMapIterToDelete != outgoingEdges_.end())
                    {
                        // Iterate over the vector removing them from queue
                        for (auto &edgeQueueIter : vidAndEdgeQueueIterListPairAsUMapIterToDelete->second)
                        {
                            // Erase the edge, removing it from the *other* lookup. No need to remove from this lookup,
                            // as that's being cleared:
                            this->edgeRemoveHelper(edgeQueueIter, true, false);
                        }

                        // Clear the list:
                        vidAndEdgeQueueIterListPairAsUMapIterToDelete->second = EdgeQueueIterList();
                    }
                    // No else, why was this called?
                }
                else
                {
                    throw ompl::Exception("Removing edges in the queue coming from a vertex requires parent vertex "
                                          "lookup, which is not enabled for this instance of the container.");
                }
            }
            // No else, nothing to remove_from
        }

        void BITstar::IntegratedQueue::updateEdgesTo(const VertexPtr &cVertex)
        {
            if (edgeQueue_.empty() == false)
            {
                if (incomingLookupTables_ == true)
                {
                    // Variable:
                    // The iterator to the key,value of the child-lookup map, i.e., an iterator to a pair whose second
                    // is a list of edges to the child (which are actually iterators to the queue):
                    VertexIdToEdgeQueueIterListUMap::iterator listOfEdgeQueueItersToVertexAsUMapIter;

                    // Get my incoming edges as a vector of iterators
                    listOfEdgeQueueItersToVertexAsUMapIter = incomingEdges_.find(cVertex->getId());

                    // Make sure it was found before we start dereferencing it:
                    if (listOfEdgeQueueItersToVertexAsUMapIter != incomingEdges_.end())
                    {
                        // Variable
                        // The vector of edges to delete in the list:
                        std::vector<EdgeQueueIterList::iterator> edgeQueueItersToDelete;

                        // Iterate over the incoming edges and record those that are to be deleted
                        for (auto costAndEdgePairAsQueueIter = listOfEdgeQueueItersToVertexAsUMapIter->second.begin();
                             costAndEdgePairAsQueueIter != listOfEdgeQueueItersToVertexAsUMapIter->second.end();
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

                        // Now, iterate over the list of iterators to delete
                        for (auto &edgeQueueIter : edgeQueueItersToDelete)
                        {
                            // Remove the edge and the edge iterator from the other lookup table:
                            this->edgeRemoveHelper(*edgeQueueIter, false, true);

                            // And finally erase the lookup iterator from the from lookup. If this was done first, the
                            // iterator would be invalidated for the above.
                            listOfEdgeQueueItersToVertexAsUMapIter->second.erase(edgeQueueIter);
                        }
                    }
                    // No else, nothing to delete
                }
                else
                {
                    throw ompl::Exception("Removing edges in the queue going to a vertex requires child vertex lookup, "
                                          "which is not enabled for this instance of the container.");
                }
            }
            // No else, nothing to prune_to
        }

        void BITstar::IntegratedQueue::updateEdgesFrom(const VertexPtr &pVertex)
        {
            if (edgeQueue_.empty() == false)
            {
                if (outgoingLookupTables_ == true)
                {
                    // Variable:
                    // The iterator to the key, value of the parent-lookup map, i.e., an iterator to a pair whose second
                    // is a list of edges from the child (which are actually iterators to the queue):
                    VertexIdToEdgeQueueIterListUMap::iterator listOfEdgeQueueItersFromVertexAsUMapIter;

                    // Get my outgoing edges as a vector of iterators
                    listOfEdgeQueueItersFromVertexAsUMapIter = outgoingEdges_.find(pVertex->getId());

                    // Make sure it was found before we start dereferencing it:
                    if (listOfEdgeQueueItersFromVertexAsUMapIter != outgoingEdges_.end())
                    {
                        // Variable
                        // The vector of edges to delete in the list:
                        std::vector<EdgeQueueIterList::iterator> edgeQueueItersToDelete;

                        // Iterate over the incoming edges and record those that are to be deleted
                        for (auto costAndEdgePairAsQueueIter = listOfEdgeQueueItersFromVertexAsUMapIter->second.begin();
                             costAndEdgePairAsQueueIter != listOfEdgeQueueItersFromVertexAsUMapIter->second.end();
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

                        // Now, iterate over the list of iterators to delete
                        for (auto &edgeQueueIter : edgeQueueItersToDelete)
                        {
                            // Remove the edge and the edge iterator from the other lookup table:
                            this->edgeRemoveHelper(*edgeQueueIter, true, false);

                            // And finally erase the lookup iterator from the from lookup. If this was done first, the
                            // iterator would be invalidated for the above.
                            listOfEdgeQueueItersFromVertexAsUMapIter->second.erase(edgeQueueIter);
                        }
                    }
                    // No else, nothing to delete
                }
                else
                {
                    throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
                }
            }
            // No else, nothing to prune_from
        }

        void BITstar::IntegratedQueue::markVertexUnsorted(const VertexPtr &vertex)
        {
            resortVertices_.push_back(vertex);
        }

        std::pair<unsigned int, unsigned int> BITstar::IntegratedQueue::prune(const VertexConstPtr &pruneStartPtr,
                                                                              const VertexPtrNNPtr &vertexNN,
                                                                              const VertexPtrNNPtr &freeStateNN,
                                                                              VertexPtrVector *recycledVertices)
        {
            if (this->isSorted() == false)
            {
                throw ompl::Exception("Prune cannot be called on an unsorted queue.");
            }
            // The vertex expansion queue is sorted on an estimated solution cost considering the *current* cost-to-come
            // of the vertices, while we prune by considering the best-case cost-to-come.
            // This means that the value of the vertices in the queue are an upper-bounding estimate of the value we
            // will use to prune them.
            // Therefore, we can start our pruning at the goal vertex and iterate forward through the queue from there.

            // Variables:
            // The number of vertices and samples pruned:
            std::pair<unsigned int, unsigned int> numPruned;
            // The iterator into the lookup helper:
            VertexIdToVertexQueueIterUMap::iterator lookupIter;
            // The iterator into the queue:
            VertexQueueIter queueIter;

            // Initialize the counters:
            numPruned = std::make_pair(0u, 0u);

            // Get the iterator to the queue of the given starting point.
            lookupIter = vertexIterLookup_.find(pruneStartPtr->getId());

            // Check that it was found
            if (lookupIter == vertexIterLookup_.end())
            {
                // Complain
                throw ompl::Exception("The provided starting point is not in the queue?");
            }

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
                    numPruned =
                        numPruned + this->pruneBranch(pruneIter->second, vertexNN, freeStateNN, recycledVertices);
                }
                // No else, skip this vertex.

                // Iterate forward to the next value in the queue
                ++queueIter;
            }

            // Return the number of vertices and samples pruned.
            return numPruned;
        }

        std::pair<unsigned int, unsigned int> BITstar::IntegratedQueue::resort(const VertexPtrNNPtr &vertexNN,
                                                                               const VertexPtrNNPtr &freeStateNN,
                                                                               VertexPtrVector *recycledVertices)
        {
            // Variable:
            typedef std::unordered_map<BITstar::VertexId, VertexPtr> VertexIdToVertexPtrUMap;
            typedef std::map<unsigned int, VertexIdToVertexPtrUMap> DepthToUMapMap;
            // The number of vertices and samples pruned, respectively:
            std::pair<unsigned int, unsigned int> numPruned;

            // Initialize the counters:
            numPruned = std::make_pair(0u, 0u);

            // Iterate through every vertex listed for resorting:
            if (resortVertices_.empty() == false)
            {
                // Variable:
                // The container ordered on vertex depth:
                DepthToUMapMap uniqueResorts;

                // Iterate over the vector and place into the unique queue indexed on *depth*. This guarantees that we
                // won't process a branch multiple times by being given different vertices down its chain
                for (auto &vertex : resortVertices_)
                {
                    // Add the vertex to the unordered map stored at the given depth.
                    // The [] return a reference to the existing entry, or creates a new entry:
                    uniqueResorts[vertex->getDepth()].emplace(vertex->getId(), vertex);
                }

                // Clear the list of vertices to resort from:
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
                                if (this->vertexInsertCondition(vIdAndPtrPair.second) == false &&
                                    static_cast<bool>(vertexNN) == true && static_cast<bool>(freeStateNN) == true)
                                {
                                    // The vertex should just be pruned and forgotten about.
                                    // Prune the branch:
                                    numPruned = numPruned +
                                                this->pruneBranch(vIdAndPtrPair.second, vertexNN, freeStateNN,
                                                                  recycledVertices);
                                }
                                else
                                {
                                    // The vertex is going to be kept.

                                    // Does it have any children?
                                    if (vIdAndPtrPair.second->hasChildren() == true)
                                    {
                                        // Variables:
                                        // The list of children:
                                        VertexPtrVector resortChildren;

                                        // Put its children in the list to be resorted:
                                        // Get the list of children:
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
                            // No else, this vertex was a child of a vertex pruned during the resort. It has been
                            // returned to the set of free samples.
                        }
                        // No else, this vertex was a child of a vertex pruned during the resort. It has been deleted.
                    }
                }
            }

            // Return the number of vertices pruned.
            return numPruned;
        }

        void BITstar::IntegratedQueue::finish()
        {
            // Clear the edge containers:
            edgeQueue_.clear();
            outgoingEdges_.clear();
            incomingEdges_.clear();

            // Move the token to the end:
            vertexToExpand_ = vertexQueue_.end();

            // Do NOT clear:
            //  -  resortVertices_ (they may still need to be resorted)
            //  - vertexIterLookup_ (it's still valid)
        }

        void BITstar::IntegratedQueue::reset()
        {
            // Make sure the queue is "finished":
            this->finish();

            // Restart the expansion queue:
            vertexToExpand_ = vertexQueue_.begin();
        }

        void BITstar::IntegratedQueue::clear()
        {
            // Clear:
            // The vertex queue:
            vertexQueue_.clear();
            vertexToExpand_ = vertexQueue_.begin();

            // The edge queue:
            edgeQueue_.clear();

            // The lookups:
            vertexIterLookup_.clear();
            outgoingEdges_.clear();
            incomingEdges_.clear();

            // The resort list:
            resortVertices_.clear();

            // The cost threshold:
            costThreshold_ = costHelpPtr_->infiniteCost();

            // The existence of a solution:
            hasExactSolution_ = false;
        }

        bool BITstar::IntegratedQueue::vertexInsertCondition(const VertexPtr &state) const
        {
            // Threshold should always be g_t(x_g)

            // Can it ever be a better solution?
            // Just in case we're using a vertex that is exactly optimally connected
            // g^(v) + h^(v) <= g_t(x_g)?
            return costHelpPtr_->isCostBetterThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(state),
                                                                costThreshold_);
        }

        bool BITstar::IntegratedQueue::edgeInsertCondition(const VertexPtrPair &edge) const
        {
            bool rval;
            // Threshold should always be g_t(x_g)

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

        bool BITstar::IntegratedQueue::vertexPruneCondition(const VertexPtr &state) const
        {
            // Threshold should always be g_t(x_g)

            // Prune the vertex if it could cannot part of a better solution in the current graph.  Greater-than just in
            // case we're using an edge that is exactly optimally connected.
            // g_t(v) + h^(v) > g_t(x_g)?
            return costHelpPtr_->isCostWorseThan(costHelpPtr_->currentHeuristicVertex(state), costThreshold_);
        }

        bool BITstar::IntegratedQueue::samplePruneCondition(const VertexPtr &state) const
        {
            // Threshold should always be g_t(x_g)
            // Prune the unconnected sample if it could never be better of a better solution.
            // g^(v) + h^(v) >= g_t(x_g)?
            return costHelpPtr_->isCostWorseThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(state),
                                                               costThreshold_);
        }

        bool BITstar::IntegratedQueue::edgePruneCondition(const VertexPtrPair &edge) const
        {
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

        unsigned int BITstar::IntegratedQueue::numEdges() const
        {
            return edgeQueue_.size();
        }

        unsigned int BITstar::IntegratedQueue::numVertices() const
        {
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

        unsigned int BITstar::IntegratedQueue::numEdgesTo(const VertexPtr &cVertex) const
        {
            // Variables:
            // The number of edges to:
            unsigned int rval;

            // Start at 0:
            rval = 0u;

            // Is there anything to count?
            if (edgeQueue_.empty() == false)
            {
                if (incomingLookupTables_ == true)
                {
                    // Variable:
                    // The iterator to the vector of edges to the child:
                    VertexIdToEdgeQueueIterListUMap::const_iterator toIter;

                    // Get the vector of iterators
                    toIter = incomingEdges_.find(cVertex->getId());

                    // Make sure it was found before we dereferencing it:
                    if (toIter != incomingEdges_.end())
                    {
                        rval = toIter->second.size();
                    }
                    // No else, there are none.
                }
                else
                {
                    throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
                }
            }
            // No else, there is nothing.

            // Return:
            return rval;
        }

        unsigned int BITstar::IntegratedQueue::numEdgesFrom(const VertexPtr &pVertex) const
        {
            // Variables:
            // The number of edges to:
            unsigned int rval;

            // Start at 0:
            rval = 0u;

            // Is there anything to count?
            if (edgeQueue_.empty() == false)
            {
                if (outgoingLookupTables_ == true)
                {
                    // Variable:
                    // The iterator to the vector of edges from the parent:
                    VertexIdToEdgeQueueIterListUMap::const_iterator toIter;

                    // Get the vector of iterators
                    toIter = outgoingEdges_.find(pVertex->getId());

                    // Make sure it was found before we dereferencing it:
                    if (toIter != outgoingEdges_.end())
                    {
                        rval = toIter->second.size();
                    }
                    // No else, 0u.
                }
                else
                {
                    throw ompl::Exception("Parent lookup is not enabled for this instance of the container.");
                }
            }
            // No else, there is nothing.

            // Return
            return rval;
        }

        bool BITstar::IntegratedQueue::isSorted() const
        {
            return resortVertices_.empty();
        }

        bool BITstar::IntegratedQueue::isReset() const
        {
            return (vertexToExpand_ == vertexQueue_.begin() && edgeQueue_.empty());
        }

        bool BITstar::IntegratedQueue::isEmpty()
        {
            // Expand if the edge queue is empty but the vertex queue is not:
            while (edgeQueue_.empty() && vertexToExpand_ != vertexQueue_.end())
            {
                // Expand the next vertex, this pushes the token:
                this->expandNextVertex();
            }

            // Return whether the edge queue is empty:
            return edgeQueue_.empty();
        }

        bool BITstar::IntegratedQueue::isVertexExpanded(const VertexConstPtr &vertex) const
        {
            // Variable
            // The vertex iterator
            VertexIdToVertexQueueIterUMap::const_iterator lkupIter;

            // Get the lookup iterator for the provided vertex
            lkupIter = vertexIterLookup_.find(vertex->getId());

            if (lkupIter == vertexIterLookup_.end())
            {
                throw ompl::Exception("Attempting to check the expansion status of a vertex not in the queue");
            }

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
                // time)
                // Therefore, the vertex is expanded if it's cost is strictly better than the token.
                return this->queueComparison(lkupIter->second->first, vertexToExpand_->first);
            }
        }

        void BITstar::IntegratedQueue::getVertices(VertexConstPtrVector *vertexQueue)
        {
            // Clear the given list:
            vertexQueue->clear();

            // Iterate until the end, pushing back:
            for (QValueToVertexMMap::const_iterator vIter = vertexToExpand_; vIter != vertexQueue_.end(); ++vIter)
            {
                // Push back:
                vertexQueue->push_back(vIter->second);
            }
        }

        void BITstar::IntegratedQueue::getEdges(VertexConstPtrPairVector *edgeQueue)
        {
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
        void BITstar::IntegratedQueue::updateQueue()
        {
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
                                                                          edgeQueue_.begin()->first.at(0u)) == true)
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

        void BITstar::IntegratedQueue::expandNextVertex()
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

        void BITstar::IntegratedQueue::expandVertex(const VertexPtr &vertex)
        {
            // Should we expand this vertex?
            if (this->vertexInsertCondition(vertex) == true)
            {
                // Variables:
                // The vector of nearby samples (either within r or the k-nearest)
                VertexPtrVector neighbourSamples;
                // The vector of nearby vertices
                VertexPtrVector neighbourVertices;
                // Are we using k-nearest?
                bool usingKNearest;
                // If we're using k-nearest, what number that is
                unsigned int k;

                // Get the set of nearby free states, returns the number k if it's k nearest, 0u otherwise
                k = nearSamplesFunc_(vertex, &neighbourSamples);

                // Decode if we're using k-nearest for readability
                usingKNearest = (k > 0u);

                // If we're usjng k-nearest, we always have to also get the neighbourVertices and the do some
                // post-processing
                if (usingKNearest == true)
                {
                    // Get the set of nearby vertices
                    nearVerticesFunc_(vertex, &neighbourVertices);

                    // Post process them:
                    this->processKNearest(k, vertex, &neighbourSamples, &neighbourVertices);
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
                        this->queueupEdge(vertex, targetSample);
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
                            this->queueupEdge(vertex, targetSample);
                        }
                        // No else, we've considered this edge before.
                    }
                }

                // If the vertex has never been expanded into possible rewiring edges *and* either we're not delaying
                // rewiring or we have a solution, we add those rewiring candidates:
                if (vertex->hasBeenExpandedToVertices() == false &&
                    (delayRewiring_ == false || hasExactSolution_ == true))
                {
                    // If we're not using k-nearest, we will not have gotten the neighbour vertices yet, get them now
                    if (usingKNearest == false)
                    {
                        // Get the set of nearby vertices
                        nearVerticesFunc_(vertex, &neighbourVertices);
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
                                    this->queueupEdge(vertex, targetVertex);
                                }
                                else if (targetVertex->getId() != vertex->getParent()->getId())
                                {
                                    // The neighbour is not my parent, attempt to queue the edge:
                                    this->queueupEdge(vertex, targetVertex);
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

        void BITstar::IntegratedQueue::queueupEdge(const VertexPtr &parent, const VertexPtr &child)
        {
            // Variable:
            // The edge:
            VertexPtrPair newEdge;

            // Make the edge
            newEdge = std::make_pair(parent, child);

            // Should this edge be in the queue?
            if (this->edgeInsertCondition(newEdge) == true)
            {
                this->edgeInsertHelper(newEdge, edgeQueue_.end());
            }
            // No else, it can never provide a better solution
        }

        void BITstar::IntegratedQueue::processKNearest(unsigned int k, const VertexConstPtr &vertex,
                                                       VertexPtrVector *kNearSamples, VertexPtrVector *kNearVertices)
        {
            // Variables
            // The position in the sample vector
            unsigned int samplePos;
            // The position in the vertex vector
            unsigned int vertexPos;

            // Iterate through the first k in the combined vectors
            samplePos = 0u;
            vertexPos = 0u;
            while (samplePos + vertexPos < k && (samplePos < kNearSamples->size() || vertexPos < kNearVertices->size()))
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
                    if (distanceFunc_(kNearVertices->at(vertexPos), vertex) <
                        distanceFunc_(kNearSamples->at(samplePos), vertex))
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

        void BITstar::IntegratedQueue::reinsertVertex(const VertexPtr &unorderedVertex)
        {
            // Variables:
            // Whether the vertex is expanded.
            bool alreadyExpanded;
            // My entry in the vertex lookup:
            VertexIdToVertexQueueIterUMap::iterator myLookup;
            // The list of edges from the vertex:
            VertexIdToEdgeQueueIterListUMap::iterator vertexAndEdgeQueueIterListPairAsIter;

            // Get my iterator:
            myLookup = vertexIterLookup_.find(unorderedVertex->getId());

            // Assert
            if (myLookup == vertexIterLookup_.end())
            {
                throw ompl::Exception("Vertex to reinsert is not in the lookup. Something went wrong.");
            }

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
            this->vertexRemoveHelper(unorderedVertex, VertexPtrNNPtr(), VertexPtrNNPtr(), nullptr, false);

            // Reinsert myself, expanding if I cross the token if I am not already expanded
            this->vertexInsertHelper(unorderedVertex, alreadyExpanded == false);

            // Iterate over my outgoing edges and reinsert them in the queue:
            // Get my list of outgoing edges
            vertexAndEdgeQueueIterListPairAsIter = outgoingEdges_.find(unorderedVertex->getId());

            // Reinsert the edges:
            if (vertexAndEdgeQueueIterListPairAsIter != outgoingEdges_.end())
            {
                // Variables
                // The iterators to the edge queue from this vertex
                EdgeQueueIterList edgeQueueItersToResort;

                // Copy the iters to resort
                edgeQueueItersToResort = vertexAndEdgeQueueIterListPairAsIter->second;

                // Clear the outgoing lookup
                vertexAndEdgeQueueIterListPairAsIter->second = EdgeQueueIterList();

                // Iterate over the list of iters to resort, inserting each one as a new edge, and then removing it as
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

        std::pair<unsigned int, unsigned int> BITstar::IntegratedQueue::pruneBranch(const VertexPtr &branchBase,
                                                                                    const VertexPtrNNPtr &vertexNN,
                                                                                    const VertexPtrNNPtr &freeStateNN,
                                                                                    VertexPtrVector *recycledVertices)
        {
            // We must iterate over the children of this vertex and prune each one.
            // Then we must decide if this vertex (a) gets deleted or (b) placed back on the sample set.
            //(a) occurs if it has a lower-bound heuristic greater than the current solution
            //(b) occurs if it doesn't.

            // Some asserts:
            if (branchBase->isInTree() == false)
            {
                throw ompl::Exception("Trying to prune a disconnected vertex. Something went wrong.");
            }

            // Variables:
            // The counter of vertices and samples pruned:
            std::pair<unsigned int, unsigned int> numPruned;
            // The vector of my children:
            VertexPtrVector children;

            // Initialize the counter:
            numPruned = std::make_pair(1u, 0u);

            // Disconnect myself from my parent, not cascading costs as I know my children are also being disconnected:
            this->disconnectParent(branchBase, false);

            // Get the vector of children
            branchBase->getChildren(&children);

            // Remove myself from everything:
            numPruned.second = this->vertexRemoveHelper(branchBase, vertexNN, freeStateNN, recycledVertices, true);

            // Prune my children:
            for (auto &child : children)
            {
                // Prune my children:
                numPruned = numPruned + this->pruneBranch(child, vertexNN, freeStateNN, recycledVertices);
            }

            // Return the number pruned
            return numPruned;
        }

        void BITstar::IntegratedQueue::disconnectParent(const VertexPtr &oldVertex, bool cascadeCostUpdates)
        {
            if (oldVertex->hasParent() == false)
            {
                throw ompl::Exception("An orphaned vertex has been passed for disconnection. Something went wrong.");
            }

            // Check if my parent has already been pruned. This can occur if we're cascading vertex disconnections.
            if (oldVertex->getParent()->isPruned() == false)
            {
                // If not, remove myself from my parent's list of children, not updating down-stream costs
                oldVertex->getParent()->removeChild(oldVertex, false);
            }

            // Remove my parent link, cascading cost updates if requested:
            oldVertex->removeParent(cascadeCostUpdates);
        }

        void BITstar::IntegratedQueue::vertexInsertHelper(const VertexPtr &newVertex, bool expandIfBeforeToken)
        {
            // Variable:
            // The iterator to the new edge in the queue:
            VertexQueueIter vertexIter;

            // Insert into the order map, getting the interator
            vertexIter = vertexQueue_.insert(std::make_pair(this->vertexQueueValue(newVertex), newVertex));

            // Store the iterator in the lookup. This will create insert if necessary and otherwise lookup
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
                    3 The new vertex is after the token: Don't expand. It cleanly goes into the list of vertices to
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

        unsigned int BITstar::IntegratedQueue::vertexRemoveHelper(const VertexPtr &oldVertex,
                                                                  const VertexPtrNNPtr &vertexNN,
                                                                  const VertexPtrNNPtr &freeStateNN,
                                                                  VertexPtrVector *recycledVertices, bool removeLookups)
        {
            // Variable
            // The number of samples deleted (i.e., if this vertex is NOT moved to a sample, this is a 1)
            unsigned int deleted;
            // A copy of the vertex pointer to be removed so we can't delete it out from under ourselves (occurs when
            // this function is given an element of the maintained set as the argument)
            VertexPtr vertexToDelete(oldVertex);

            // Check that the vertex is not connected to a parent:
            if (vertexToDelete->hasParent() == true && removeLookups == true)
            {
                throw ompl::Exception("Cannot delete a vertex connected to a parent unless the vertex is being "
                                      "immediately reinserted, in which case removeLookups should be false.");
            }

            // Start undeleted:
            deleted = 0u;

            // Check if there's anything to delete:
            if (vertexQueue_.empty() == false)
            {
                // Variable
                // The iterator into the lookup:
                VertexIdToVertexQueueIterUMap::iterator lookupIter;

                // Get my lookup iter:
                lookupIter = vertexIterLookup_.find(vertexToDelete->getId());

                // Assert
                if (lookupIter == vertexIterLookup_.end())
                {
                    std::cout << std::endl
                              << "vId: " << vertexToDelete->getId() << std::endl;
                    throw ompl::Exception("Deleted vertex is not found in lookup. Something went wrong.");
                }

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
                if (removeLookups == true)
                {
                    vertexIterLookup_.erase(lookupIter);
                    this->removeEdgesFrom(vertexToDelete);
                }

                // Check if I have been given permission to change sets:
                if (static_cast<bool>(vertexNN) == true && static_cast<bool>(freeStateNN) == true &&
                    static_cast<bool>(recycledVertices) == true)
                {
                    // Check if I should be discarded completely:
                    if (this->samplePruneCondition(vertexToDelete) == true)
                    {
                        // Yes, the vertex isn't even useful as a sample
                        // Update the counter:
                        deleted = 1u;

                        // Remove from the incoming edge container if requested:
                        if (removeLookups == true)
                        {
                            this->removeEdgesTo(vertexToDelete);
                        }

                        // Remove myself from the nearest neighbour structure:
                        vertexNN->remove(vertexToDelete);

                        // Finally, mark as pruned. This is a lock that prevents accessing anything about the vertex.
                        vertexToDelete->markPruned();
                    }
                    else
                    {
                        // No, the vertex is still useful as a sample:
                        // Remove myself from the nearest neighbour structure:
                        vertexNN->remove(vertexToDelete);

                        // Mark myself as a "new" sample. This assures that all possible incoming edges will be
                        // considered
                        vertexToDelete->markNew();

                        // Add myself to the list of recycled vertices:
                        recycledVertices->push_back(vertexToDelete);

                        // And add the vertex to the set of samples, keeping the incoming edges:
                        freeStateNN->add(vertexToDelete);
                    }
                }
                // Else, if I was given null pointers, that's because this sample is not allowed to change sets.
            }
            else
            {
                std::cout << std::endl
                          << "vId: " << vertexToDelete->getId() << std::endl;
                throw ompl::Exception("Removing a nonexistent vertex. Something went wrong.");
            }

            // Return if the sample was deleted:
            return deleted;
        }

        void BITstar::IntegratedQueue::edgeInsertHelper(const VertexPtrPair &newEdge, EdgeQueueIter positionHint)
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

            if (outgoingLookupTables_ == true)
            {
                // Push the newly created edge back on the list of edges from the parent.
                // The [] return an reference to the existing entry, or create a new entry:
                outgoingEdges_[newEdge.first->getId()].push_back(edgeIter);
            }

            if (incomingLookupTables_ == true)
            {
                // Push the newly created edge back on the list of edges from the child.
                // The [] return an reference to the existing entry, or create a new entry:
                incomingEdges_[newEdge.second->getId()].push_back(edgeIter);
            }
        }

        void BITstar::IntegratedQueue::edgeRemoveHelper(const EdgeQueueIter &oldEdgeIter, bool rmIncomingLookup,
                                                        bool rmOutgoingLookup)
        {
            // Erase the lookup tables:
            if (rmIncomingLookup == true)
            {
                // Erase the entry in the outgoing lookup table:
                this->rmIncomingLookup(oldEdgeIter);
            }
            // No else

            if (rmOutgoingLookup == true)
            {
                // Erase  the entry in the ingoing lookup table:
                this->rmOutgoingLookup(oldEdgeIter);
            }
            // No else

            // Finally erase from the queue:
            edgeQueue_.erase(oldEdgeIter);
        }

        void BITstar::IntegratedQueue::rmIncomingLookup(const EdgeQueueIter &mmapIterToRm)
        {
            if (incomingLookupTables_ == true)
            {
                this->rmEdgeLookupHelper(incomingEdges_, mmapIterToRm->second.second->getId(), mmapIterToRm);
            }
            // No else
        }

        void BITstar::IntegratedQueue::rmOutgoingLookup(const EdgeQueueIter &mmapIterToRm)
        {
            if (outgoingLookupTables_ == true)
            {
                this->rmEdgeLookupHelper(outgoingEdges_, mmapIterToRm->second.first->getId(), mmapIterToRm);
            }
            // No else
        }

        void BITstar::IntegratedQueue::rmEdgeLookupHelper(VertexIdToEdgeQueueIterListUMap &lookup,
                                                          const BITstar::VertexId &idx,
                                                          const EdgeQueueIter &mmapIterToRm)
        {
            // Variable:
            // An iterator to the vertex,list pair in the lookup
            VertexIdToEdgeQueueIterListUMap::iterator iterToVertexListPair;

            // Get the list in the lookup for the given index:
            iterToVertexListPair = lookup.find(idx);

            // Make sure it was actually found before derefencing it:
            if (iterToVertexListPair != lookup.end())
            {
                // Variable:
                // Whether I've found the mmapIterToRm in my list:
                bool found;
                // The iterator to the mmapIterToRm in my list:
                EdgeQueueIterList::iterator iterToList;

                // Start at the front:
                iterToList = iterToVertexListPair->second.begin();

                // Iterate through the list and find mmapIterToRm
                found = false;
                while (found == false && iterToList != iterToVertexListPair->second.end())
                {
                    // Compare the value in the list to the target:
                    if (*iterToList == mmapIterToRm)
                    {
                        // Mark as found:
                        found = true;
                    }
                    else
                    {
                        // Increment the iterator:
                        ++iterToList;
                    }
                }

                if (found == true)
                {
                    iterToVertexListPair->second.erase(iterToList);
                }
                else
                {
                    throw ompl::Exception("Edge iterator not found under given index in lookup hash.");
                }
            }
            else
            {
                throw ompl::Exception("Indexing vertex not found in lookup hash.");
            }
        }

        BITstar::IntegratedQueue::CostDouble BITstar::IntegratedQueue::vertexQueueValue(const VertexPtr &vertex) const
        {
            // Construct and return an array
            return CostDouble{costHelpPtr_->currentHeuristicVertex(vertex), vertex->getCost()};
        }

        BITstar::IntegratedQueue::CostTriple BITstar::IntegratedQueue::edgeQueueValue(const VertexPtrPair &edge) const
        {
            // Construct and return an array
            return CostTriple{costHelpPtr_->currentHeuristicEdge(edge), costHelpPtr_->currentHeuristicTarget(edge),
                              edge.first->getCost()};
        }

        template <std::size_t SIZE>
        bool BITstar::IntegratedQueue::queueComparison(const std::array<ompl::base::Cost, SIZE> &lhs,
                                                       const std::array<ompl::base::Cost, SIZE> &rhs) const
        {
            return std::lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(),
                                                [this](const ompl::base::Cost &a, const ompl::base::Cost &b)
                                                {
                                                    return costHelpPtr_->isCostBetterThan(a, b);
                                                });
        }

        template <typename T, typename U>
        std::pair<T, U> operator+(const std::pair<T, U> &lhs, const std::pair<T, U> &rhs)
        {
            return std::make_pair(lhs.first + rhs.first, lhs.second + rhs.second);
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Boring sets/gets (Public):
        void BITstar::IntegratedQueue::setDelayedRewiring(bool delayRewiring)
        {
            delayRewiring_ = delayRewiring;
        }

        bool BITstar::IntegratedQueue::getDelayedRewiring() const
        {
            return delayRewiring_;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    }  // geometric
}  // ompl
