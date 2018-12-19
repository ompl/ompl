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

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_SEARCHQUEUE_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_SEARCHQUEUE_

// STL:
// std::pair
#include <utility>
// std::vector
#include <vector>
// std::array
#include <array>
// std::multimap
#include <map>
// std::unordered_map
#include <unordered_map>
// For std::function
#include <functional>

// OMPL:
// The cost class:
#include "ompl/base/Cost.h"
// The optimization objective class:
#include "ompl/base/OptimizationObjective.h"
// The nearest neighbours structure
#include "ompl/datastructures/NearestNeighbors.h"
// The binary heap data structure
#include "ompl/datastructures/BinaryHeap.h"

// BIT*:
// I am member class of the BITstar class (i.e., I am in it's namespace), so I need to include it's definition to be
// aware of the class BITstar. It has a forward declaration to me and the other helper classes but I will need to
// include any I use in my cpp (to avoid dependency loops).
#include "ompl/geometric/planners/bitstar/BITstar.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor SearchQueue
        \par Short Description
        A two-stage queue that consists of vertices expanded into edges to be processed. The queue consists of a vertex
        expansion queue and an edge processing queue. Vertices are expanded as needed from the vertex queue into edges
        places in the edge queue. Edges are removed from the edge queue for processing by \ref gBITstar "BIT*". The
        vertex queue is implemented as a static ordered list of the vertices in the graph with a token (i.e., an
        iterator) pointing to the next vertex that needs to be expanded. This is specifically a multimap ordered on
        a search key. The edge queue is implemented as an ordered list of potential edges. It is filled by the
        vertex queue and emptied by popping the best value off the front. It is specifically a multimap ordered on
        std::pair<ompl::base::Cost, ompl::base::Cost>

        @par Notes:
            - An eraseEdge() function could be made by mimicking the vertex -> vertexQueue_::iterator lookup
        datastructure for the edgeQueue_
        */

        /** \brief A queue of edges to be processed that integrates both the expansion of \ref gVertex "Vertices" and
         * the ordering of the resulting edges. */
        class BITstar::SearchQueue
        {
        public:
            ////////////////////////////////
            // Aliases to underlying data types used by the SearchQueue
            // Types for the vertex queue:
            /** \brief A pair of costs, i.e., the vertex queue sorting key. Done as an array instead
             * of a pair for consistency with the EdgeQueue */
            using CostDouble = std::array<ompl::base::Cost, 2u>;
            /** \brief The function signature of the sorting function for the Vertex Queue*/
            using VertexQueueSortFunc = std::function<bool(const CostDouble &, const CostDouble &)>;
            /** \brief The underlying vertex queue.  The advantage of a multimap over a multiset is that a copy
             * of the key is stored with the value, which guarantees that the ordering remains sane
             * even if the data inherently behind the queue value changes. In such a case the queue
             * will remain sorted by the old key until manually updated */
            using VertexQueueAsMMap = std::multimap<CostDouble, VertexPtr, VertexQueueSortFunc>;
            /** \brief An iterator into the vertex queue multimap */
            using VertexQueueIter = VertexQueueAsMMap::iterator;
            // Types for the edge queue
            /** \brief A triplet of costs, i.e., the edge queue sorting key. */
            using CostTriple = std::array<ompl::base::Cost, 3u>;
            /** \brief The data stored in the edge-queue binary heap. */
            using CostTripleAndVertexPtrPair = std::pair<CostTriple, VertexPtrPair>;
            /** \brief The function signature of the sorting function for the Edge Queue*/
            using EdgeQueueSortFunc = std::function<bool(const CostTripleAndVertexPtrPair &, const CostTripleAndVertexPtrPair &)>;
            /** \brief The underlying edge queue. Using static keys for the same reason as the Vertex Queue */
            using EdgeQueueAsPairBinHeap = ompl::BinaryHeap<CostTripleAndVertexPtrPair, EdgeQueueSortFunc>;
            /** \brief An element pointer into the edge queue binary heap */
            using EdgeQueueElemPtr = EdgeQueueAsPairBinHeap::Element*;
            /** \brief A vector of edge queue pointers */
            using EdgeQueueElemPtrVector = std::vector<EdgeQueueElemPtr>;
            ////////////////////////////////

            ////////////////////////////////
            //////////////////
            // Public functions:
            /** \brief Construct a search queue. It must be setup before use. */
            SearchQueue(NameFunc nameFunc);

            virtual ~SearchQueue() = default;

            /** \brief Setup the SearchQueue, must be called before use */
            void setup(CostHelper *costHelpPtr, ImplicitGraph *graphPtr);

            /** \brief Clear the queue to the state of construction. */
            void reset();
            //////////////////

            //////////////////
            // Insert and erase
            /** \brief Update the edge queue by adding all the potential edges from the vertex to nearby states. */
            void expand(const VertexPtr &vertex);

            //////////////////

            //////////////////
            // Access the queue
            /** \brief Get the best edge on the queue, leaving it at the front of the edge queue. */
            VertexPtrPair getFrontEdge();

            /** \brief Get the value of the best edge on the queue, leaving it at the front of the edge queue. */
            CostTriple getFrontEdgeValue();

            /** \brief Pop the best edge off the queue, removing it from the front of the edge queue in the process. */
            VertexPtrPair popFrontEdge();
            //////////////////

            //////////////////
            // Queue maintenance
            /** \brief Mark that a solution has been found */
            void registerSolutionCost(const ompl::base::Cost &solutionCost);

            /** \brief Erase all edges in the edge queue that lead to the given vertex */
            void removeInEdgesConnectedToVertexFromQueue(const VertexPtr &vertex);

            /** \brief Erase all edges in the edge queue that leave from the given vertex */
            void removeOutEdgesConnectedToVertexFromQueue(const VertexPtr &vertex);

            /** 'brief Erase all edges in the edge queue that are connected to the given vertex. */
            void removeAllEdgesConnectedToVertexFromQueue(const VertexPtr &vertex);

            /** \brief Finish the queue if it is sorted, if not resort the queue. Finishing the queue clears all the
             * edge containers and moves the vertex expansion token to the end. After calling finish() ON A SORTED QUEUE,
             * isEmpty() will return true. Keeps threshold, etc.*/
            void clear();

            /** \brief Reset the queue, clearing all the edge containers and moving the vertex expansion token to the
             * start. After a call to reset, isEmpty() will return false (unless there is no data in the queue of
             * course). Keeps threshold, list of unsorted vertices, etc.*/
            void restart();
            //////////////////

            //////////////////
            // Queue info:
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
            //////////////////

            //////////////////
            // Get the progress property counters
            /** \brief The number of edges popped off the queue for processing (numEdgesPopped_). */
            unsigned int numEdgesPopped() const;
            //////////////////
        private:
            ////////////////////////////////
            // High level primitives:
            /** \brief Iterate through the list of neighbouring unconnected vertices and add potential edges to
            * the queue if the vertex is marked as new *or* we're adding all of them. */
            void enqueueEdgesToSamples(const VertexPtr &vertex, const VertexPtrVector& neighbourSamples);

            /** \brief Iterate through the list of neighbouring vertices and add potential edges to the queue. */
            void enqueueEdgesToVertices(const VertexPtr &vertex, const VertexPtrVector& neighbourVertices);

            /** \brief Attempt to add an edge to the queue. Checks that the edge meets the queueing condition. */
            void enqueueEdgeConditionally(const VertexPtr &parent, const VertexPtr &child);

            /** \brief Insert an edge into the edge processing queue. The source vertex of this edge must be in the
             * expansion queue (although it may already be expanded). */
            void enqueueEdge(const VertexPtrPair &edge);

            /** \brief Given two subsets containing (up to) the k-nearest members of each, finds the k-nearest of the
             * union */
            void processKNearest(const VertexConstPtr &vertex, VertexPtrVector *kNearSamples,
                                 VertexPtrVector *kNearVertices);
            ////////////////////////////////

            ////////////////////////////////
            // Base-queue basic helper functions:
            /** \brief A convenience function for the value of an edge in the queue */
            CostTriple sortKey(const VertexPtrPair &edge) const;

            /** A lexicographical comparison function for the std::arrays of costs. This is the sorting function for
             * both the vertex and edge queues and is just a wrapper to std::lexicographical_compare.*/
            bool lexicographicalBetterThan(const std::array<ompl::base::Cost, 3> &lhs,
                                 const std::array<ompl::base::Cost, 3> &rhs) const;
            ////////////////////////////////

            ////////////////////////////////
            // Debug
            /** \brief Test if the class is setup and throw if not. */
            void assertSetup() const;
            ////////////////////////////////

            ////////////////////////////////
            // Variables -- Make sure every one is configured in setup() and reset in clear():
            /** \brief A function pointer to the planner name, for better OMPL_INFO, etc. output */
            NameFunc nameFunc_;

            /** \brief Whether the class is setup */
            bool isSetup_{false};

            /** \brief A cost/heuristic helper class. As I am a copy of the version owned by BITstar.cpp, I can be reset
             * in a clear().*/
            CostHelper *costHelpPtr_{nullptr};

            /** \brief The samples represented as an edge-implicit graph */
            ImplicitGraph *graphPtr_{nullptr};

            /** \brief The underlying queue of edges. Sorted by edgeQueueComparison. */
            EdgeQueueAsPairBinHeap edgeQueue_;

            /** \brief The cost of the best solution, which is the maximum heuristic value allowed for vertices/edges in the queue.*/
            ompl::base::Cost solutionCost_{std::numeric_limits<double>::infinity()};

            /** \brief Whether the problem has a solution */
            bool hasExactSolution_{false};

            /** \brief A counter for the number of times we've reset the vertex queue. Used to efficiently reset the edge queue lookups stored in vertices */
            unsigned int numQueueResets_{0u};
            ////////////////////////////////

            ////////////////////////////////
            // Informational variables - Make sure initialized in setup and reset in clear
            /** \brief The number of edges processed, in one way or other, from the queue. Accessible via numEdgesPopped
             */
            unsigned int numEdgesPopped_{0u};
            ////////////////////////////////

            ////////////////////////////////
            // Parameters - Set defaults in construction/setup and DO NOT reset in clear.
            /** \brief Whether to use a strict-queue ordering (param) */
            bool useStrictQueueOrdering_{false};

            /** \brief Whether to delay rewiring until an initial solution is found or not */
            bool delayRewiring_{true};

            /** \brief Whether we are allowed to prune during resorts. Generally speaking this is whether or not we're
             * using pruning in general.*/
            bool pruneDuringResort_{true};
            ////////////////////////////////
        };  // class: SearchQueue
    }       // geometric
}  // ompl
#endif  // OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_SEARCHQUEUE_
