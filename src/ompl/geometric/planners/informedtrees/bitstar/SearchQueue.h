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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_SEARCHQUEUE_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_SEARCHQUEUE_

#include <array>
#include <functional>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ompl/base/Cost.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/informedtrees/BITstar.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor SearchQueue
        \par Short Description
        A search queue holding edges ordered on a sort key, i.e., a cost triple with a lexicographical comparison.
        The queue is implemented as a binary heap.
        */

        /** \brief A queue of edges, sorted according to a sort key. */
        class BITstar::SearchQueue
        {
        public:
            // ---
            // Aliases.
            // ---

            /** \brief A triplet of costs, i.e., the edge queue sorting key. */
            using SortKey = std::array<ompl::base::Cost, 3u>;

            /** \brief The data stored in the edge-queue binary heap. */
            using SortKeyAndVertexPtrPair = std::pair<SortKey, VertexPtrPair>;

            /** \brief The function signature of the sorting function for the Edge Queue*/
            using EdgeComparisonFunction = std::function<bool(const SortKeyAndVertexPtrPair &, const SortKeyAndVertexPtrPair &)>;

            /** \brief The underlying edge queue. Using static keys for the same reason as the Vertex Queue */
            using EdgeQueue = ompl::BinaryHeap<SortKeyAndVertexPtrPair, EdgeComparisonFunction>;

            /** \brief An element pointer into the edge queue binary heap */
            using EdgeQueueElemPtr = EdgeQueue::Element*;

            /** \brief A vector of edge queue pointers */
            using EdgeQueueElemPtrVector = std::vector<EdgeQueueElemPtr>;

            // ---
            // Construction, initialization, and destruction.
            // ---

            /** \brief Construct a search queue. It must be setup before use. */
            SearchQueue(NameFunc nameFunc);

            /** \brief Destruct the search queue using the default deconstructor. */
            virtual ~SearchQueue() = default;

            /** \brief Setup the SearchQueue, must be called before use. */
            void setup(CostHelper *costHelpPtr, ImplicitGraph *graphPtr);

            /** \brief Reset the queue to the state of construction. */
            void reset();

            /** \brief Set whether cascading of rewirings is enabled. */
            void enableCascadingRewirings(bool enable);

            // ---
            // Insertion.
            // ---

            /** \brief Update the edge queue by adding all the potential edges from the vertex to nearby states. */
            void insertOutgoingEdges(const VertexPtr &vertex);

            /** \brief Insert the outgoing edges of all start vertices.*/
            void insertOutgoingEdgesOfStartVertices();

            /** \brief Insert the outgoing edges of all inconsistent vertices. */
            void insertOutgoingEdgesOfInconsistentVertices();

            /** \brief Add a vertex to the set of inconsistent vertices. */
            void addToInconsistentSet(const VertexPtr &vertex);

            // ---
            // Access.
            // ---

            /** \brief Get the best edge on the queue, leaving it at the front of the edge queue. */
            VertexPtrPair getFrontEdge();

            /** \brief Get the value of the best edge on the queue, leaving it at the front of the edge queue. */
            SortKey getFrontEdgeValue();

            /** \brief Pop the best edge off the queue, removing it from the front of the edge queue in the process. */
            VertexPtrPair popFrontEdge();

            // ---
            // Modification.
            // ---

            /** \brief Finish the queue if it is sorted, if not resort the queue. Finishing the queue clears all the
             * edge containers and moves the vertex expansion token to the end. After calling finish() ON A SORTED QUEUE,
             * isEmpty() will return true. Keeps threshold, etc.*/
            void clear();

            /** \brief Clear the set of inconsistent vertices. */
            void clearInconsistentSet();

            /** \brief Update all the sort keys of the edges in the queue and resort. */
            void rebuildEdgeQueue();

            /** \brief Update the sort key of a particular edge and its position in the queue. */
            void update(const EdgeQueueElemPtr elementPtr);

            /** \brief Set the cost-to-go inflation factor. */
            void setInflationFactor(double factor);

            /** \brief Mark that a solution has been found */
            void registerSolutionCost(const ompl::base::Cost &solutionCost);

            /** \brief Erase all edges in the edge queue that lead to the given vertex */
            void removeInEdgesConnectedToVertexFromQueue(const VertexPtr &vertex);

            /** \brief Erase all edges in the edge queue that leave from the given vertex */
            void removeOutEdgesConnectedToVertexFromQueue(const VertexPtr &vertex);

            /** \brief Erase all edges in the edge queue that are connected to the given vertex. */
            void removeAllEdgesConnectedToVertexFromQueue(const VertexPtr &vertex);

            /** \brief Remove a vertex from the set of inconsistent vertices. */
            void removeFromInconsistentSet(const VertexPtr &vertex);

            // ---
            // Information access.
            // ---

            /** \brief Get the cost-to-go inflation factor. */
            double getInflationFactor() const;

            /** \brief Allow access to the current search id. */
            std::shared_ptr<const unsigned int> getSearchId() const;

            /** \brief The condition used to insert vertices into the queue. Compares lowerBoundHeuristicVertex to the
             * given threshold. Returns true if the vertex's best cost is lower than the internally set threshold.*/
            bool canPossiblyImproveCurrentSolution(const VertexPtr &state) const;

            /** \brief The condition used to insert edges into the queue. Compares lowerBoundHeuristicEdge to the given
             * threshold. Returns true if the edge's best cost is lower than the internally set threshold.*/
            bool canPossiblyImproveCurrentSolution(const VertexPtrPair &edge) const;

            /** \brief Returns the number of edges in the queue. Will resort/expand the queue if necessary. */
            unsigned int numEdges();

            /** \brief Returns true if the queue is empty. In the case where the edge queue is empty but the vertex
             * queue is not, this function will expand vertices *until* the edge queue is not empty or there are no
             * vertices to expand. */
            bool isEmpty();

            /** \brief Get a copy of the edge queue. This is expensive and is only meant for animations/debugging. */
            void getEdges(VertexConstPtrPairVector *edgeQueue);

            /** \brief The number of edges popped off the queue for processing (numEdgesPopped_). */
            unsigned int numEdgesPopped() const;

        private:
            // ---
            // High level primitives.
            // ---

            /** \brief Iterate through the list of neighbouring samples and add potential edges to the queue. */
            void enqueueEdges(const VertexPtr &parent, const VertexPtrVector &possibleChildren);

            /** \brief Attempt to add an edge to the queue. Checks that the edge meets the queueing condition. */
            void enqueueEdgeConditionally(const VertexPtr &parent, const VertexPtr &child);

            /** \brief Insert an edge into the edge processing queue. The source vertex of this edge must be in the
             * expansion queue (although it may already be expanded). */
            void enqueueEdge(const VertexPtrPair &edge);

            // ---
            // Sorting.
            // ---

            /** \brief Constructs a sort key for the given edge. */
            SortKey createSortKey(const VertexPtrPair &edge) const;

            /** A lexicographical comparison function for the std::arrays of costs. This is the sorting function for
             * both the vertex and edge queues and is just a wrapper to std::lexicographical_compare. */
            bool lexicographicalBetterThan(const std::array<ompl::base::Cost, 3> &lhs,
                                 const std::array<ompl::base::Cost, 3> &rhs) const;

            // ---
            // Debug helpers.
            // ---

            /** \brief Test if the class is setup and throw if not. */
            void assertSetup() const;

            // ---
            // Member variables (Make all are configured in setup() and reset in reset()).
            // ---

            /** \brief A function pointer to the planner name, for better OMPL_INFO, etc. output. */
            NameFunc nameFunc_;

            /** \brief Whether the class is setup */
            bool isSetup_{false};

            /** \brief Whether cascading of rewirings is enabled. */
            bool isCascadingOfRewiringsEnabled_{false};

            /** \brief A cost/heuristic helper class. As I am a copy of the version owned by BITstar.cpp, I can be reset
             * in a clear().*/
            CostHelper *costHelpPtr_{nullptr};

            /** \brief The samples represented as an edge-implicit graph. */
            ImplicitGraph *graphPtr_{nullptr};

            /** \brief The underlying queue of edges. Sorted by edgeQueueComparison. */
            EdgeQueue edgeQueue_;

            /** \brief A copy of the vertices found inconsistent by the most recent search. */
            VertexPtrVector inconsistentVertices_;

            /** \brief The factor by which the cost-to-go heuristic is inflated. */
            double inflationFactor_{1.0};

            /** \brief The cost of the best solution, which is the maximum heuristic value allowed for vertices/edges in the queue. */
            ompl::base::Cost solutionCost_{std::numeric_limits<double>::infinity()};

            /** \brief Whether the problem has a solution. */
            bool hasExactSolution_{false};

            /** \brief A counter for the number of times we've reset the vertex queue. Used to efficiently reset the edge queue lookups stored in vertices. */
            const std::shared_ptr<unsigned int> searchId_;

            /** \brief The number of edges processed, in one way or other, from the queue. Accessible via numEdgesPopped
             */
            unsigned int numEdgesPopped_{0u};

        }; // class SearchQueue
    } // namespace geometric
}  // namespace ompl
#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_SEARCHQUEUE_
