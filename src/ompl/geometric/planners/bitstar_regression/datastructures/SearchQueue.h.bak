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
        ompl::base::Cost. The edge queue is implemented as an ordered list of potential edges. It is filled by the
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
            typedef std::array<ompl::base::Cost, 2u> CostDouble;
            /** \brief The function signature of the sorting function for the Vertex Queue*/
            typedef std::function<bool(const CostDouble &, const CostDouble &)> VertexQueueSortFunc;
            /** \brief The underlying vertex queue.  The advantage of a multimap over a multiset is that a copy
             * of the key is stored with the value, which guarantees that the ordering remains sane
             * even if the data inherently behind the queue value changes. In such a case the queue
             * will remain sorted by the old key until manually updated */
            typedef std::multimap<CostDouble, VertexPtr, VertexQueueSortFunc> VertexQueueAsMMap;
            /** \brief An iterator into the vertex queue multimap */
            typedef VertexQueueAsMMap::iterator VertexQueueIter;
            // Types for the edge queue
            /** \brief A triplet of costs, i.e., the edge queue sorting key. */
            typedef std::array<ompl::base::Cost, 3u> CostTriple;
            /** \brief The data stored in the edge-queue binary heap. */
            typedef std::pair<CostTriple, VertexPtrPair> CostTripleAndVertexPtrPair;
            /** \brief The function signature of the sorting function for the Edge Queue*/
            typedef std::function<bool(const CostTripleAndVertexPtrPair &, const CostTripleAndVertexPtrPair &)> EdgeQueueSortFunc;
            /** \brief The underlying edge queue. Using static keys for the same reason as the Vertex Queue */
            typedef ompl::BinaryHeap<CostTripleAndVertexPtrPair, EdgeQueueSortFunc> EdgeQueueAsPairBinHeap;
            /** \brief An element pointer into the edge queue binary heap */
            typedef EdgeQueueAsPairBinHeap::Element* EdgeQueueElemPtr;
            /** \brief A vector of edge queue pointers */
            typedef std::vector<EdgeQueueElemPtr> EdgeQueueElemPtrVector;
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
            void clear();
            //////////////////

            //////////////////
            // Insert and erase
            /** \brief Insert a vertex into the vertex expansion queue. Vertices remain in the vertex queue until pruned
             * or manually removed. A moving token marks the line between expanded and not expanded vertices. Will
             * instruct the ImplicitGraph to move the vertex between sets, as necessary.*/
            void enqueueVertex(const VertexPtr &newVertex, bool removeFromFree);

            /** \brief Insert an edge into the edge processing queue. The source vertex of this edge must be in the
             * expansion queue (although it may already be expanded). */
            void enqueueEdge(const VertexPtrPair &newEdge);

            /** \brief Insert an edge into the edge processing queue. The source vertex of this edge must be in the
             * expansion queue (although it may already be expanded). */
            void enqueueEdge(const VertexPtr &sourceVertex, const VertexPtr &targetVertex);

            /** \brief Erase a vertex from the vertex expansion queue. Will disconnect the vertex from its parent and
             * remove the associated incoming and outgoing edges from the edge queue. Will instruct the ImplicitGraph to
             * move the vertex between sets, as necessary.*/
            void unqueueVertex(const VertexPtr &oldVertex);
            //////////////////

            //////////////////
            // Access the queue
            /** \brief Get the best vertex on the queue, leaving it at the front of the vertex queue. */
            VertexPtr frontVertex();

            /** \brief Get the best edge on the queue, leaving it at the front of the edge queue. */
            VertexPtrPair frontEdge();

            /** \brief Get the value of the best vertex on the queue, leaving it at the front of the vertex queue. */
            CostDouble frontVertexValue();

            /** \brief Get the value of the best edge on the queue, leaving it at the front of the edge queue. */
            CostTriple frontEdgeValue();

            /** \brief Pop the best edge off the queue, removing it from the front of the edge queue in the process. */
            void popFrontEdge(VertexPtrPair *bestEdge);

            /** \brief Pop the best edge off the queue, removing it from the front of the edge queue in the process. */
            VertexPtrPair popFrontEdge();
            //////////////////

            //////////////////
            // Queue maintenance
            /** \brief Mark that a solution has been found */
            void hasSolution(const ompl::base::Cost &solnCost);

            /** \brief Erase all edges in the edge queue that lead to the given vertex */
            void removeEdgesTo(const VertexPtr &cVertex);

            /** \brief Erase all edges in the edge queue that leave from the given vertex */
            void removeEdgesFrom(const VertexPtr &pVertex);

            /** \brief Removes edges in the edge queue that lead to the given vertex that would not be added to the
             * queue now */
            void removeExtraEdgesTo(const VertexPtr &cVertex);

            /** \brief Removes edges in the edge queue that leave from the given vertex that would not be added to the
             * queue now */
            void removeExtraEdgesFrom(const VertexPtr &pVertex);

            /** \brief Mark the queue as requiring resorting downstream of the specified vertex */
            void markVertexUnsorted(const VertexPtr &vertex);

            /** \brief Prune the vertex queue of vertices whose their lower-bound heuristic is greater then the
             * threshold. Descendents of pruned vertices that are not pruned themselves are returned to the set of free
             * states. Returns the number of vertices removed, and the number of said vertices that are completely
             * thrown away (i.e., are not even useful as a sample) */
            std::pair<unsigned int, unsigned int> prune(const VertexConstPtr &vertex);

            /** \brief Resort the queue around the marked unsorted vertices. If allowed, will remove any vertices
             * that need to be resorted but would later be pruned. */
            void resort();

            /** \brief Finish the queue if it is sorted, if not resort the queue. Finishing the queue clears all the
             * edge containers and moves the vertex expansion token to the end. After calling finish() ON A SORTED QUEUE,
             * isEmpty() will return true. Keeps threshold, etc.*/
            void finish();

            /** \brief Reset the queue, clearing all the edge containers and moving the vertex expansion token to the
             * start. After a call to reset, isEmpty() will return false (unless there is no data in the queue of
             * course). Keeps threshold, list of unsorted vertices, etc.*/
            void reset();
            //////////////////

            //////////////////
            // Queue info:
            /** \brief The condition used to insert vertices into the queue. Compares lowerBoundHeuristicVertex to the
             * given threshold. Returns true if the vertex's best cost is lower than the internally set threshold.*/
            bool vertexInsertCondition(const VertexPtr &state) const;

            /** \brief The condition used to insert edges into the queue. Compares lowerBoundHeuristicEdge to the given
             * threshold. Returns true if the edge's best cost is lower than the internally set threshold.*/
            bool edgeInsertCondition(const VertexPtrPair &edge) const;

            /** \brief The condition used to prune vertices out of the queue. Compares currentHeuristicVertex to the
             * given threshold. Returns true if the vertex's best cost is greater than the internally set threshold.*/
            bool vertexPruneCondition(const VertexPtr &state) const;

            /** \brief The condition used to prune disconnected samples from the free set. Compares
             * lowerBoundHeuristicVertex to the given threshold. Returns true if the vertex's best cost is greater than
             * or equal to the internally set threshold.*/
            bool samplePruneCondition(const VertexPtr &state) const;

            /** \brief The condition used to prune edge (i.e., vertex-pair) out of the queue. Compares
             * currentHeuristicEdge to the given threshold. Returns true if the edge's best cost is greater than the
             * internally set threshold.*/
            bool edgePruneCondition(const VertexPtrPair &edge) const;

            /** \brief Returns the number of edges in the queue. Will resort/expand the queue if necessary. */
            unsigned int numEdges();

            /** \brief Returns the number of vertices left to expand. This has nontrivial cost, as the token must be
             * moved through the vector to count.  Will resort/expand the queue if necessary. */
            unsigned int numVertices();

            /** \brief Get the number of edges in the queue pointing to a specific vertex. Will resort/expand the queue
             * if necessary. */
            unsigned int numEdgesTo(const VertexPtr &cVertex);

            /** \brief Get the number of edges in the queue coming from a specific vertex. Will resort/expand the queue
             * if necessary. */
            unsigned int numEdgesFrom(const VertexPtr &pVertex);

            /** \brief Return the number of vertices marked as unsorted */
            unsigned int numUnsorted() const;

            /** \brief Return whether the queue is still sorted */
            bool isSorted() const;

            /** \brief Returns true if the queue is reset. This means that no edges have been expanded and the vertex
             * expansion token is pointing at the start. */
            bool isReset() const;

            /** \brief Returns true if the queue is empty. In the case where the edge queue is empty but the vertex
             * queue is not, this function will expand vertices *until* the edge queue is not empty or there are no
             * vertices to expand. */
            bool isEmpty();

            /** \brief Returns whether a given vertex has been expanded or not */
            bool isVertexExpanded(const VertexConstPtr &vertex) const;

            /** \brief Get a copy of the vertices in the vertex queue that are left to be expanded. This is expensive
             * and is only meant for animations/debugging. */
            void getVertices(VertexConstPtrVector *vertexQueue);

            /** \brief Get a copy of the edge queue. This is expensive and is only meant for animations/debugging. */
            void getEdges(VertexConstPtrPairVector *edgeQueue);
            //////////////////

            //////////////////
            // Queue settings:
            /** \brief Set the queue to stay strictly sorted with each rewiring. */
            void setStrictQueueOrdering(bool beStrict);

            /** \brief Get whether the queue stays strictly sorted with each rewiring. */
            bool getStrictQueueOrdering() const;

            /** \brief Delay considering rewiring edges until an initial solution is found. */
            void setDelayedRewiring(bool delayRewiring);

            /** \brief Get whether BIT* is delaying rewiring until a solution is found. */
            bool getDelayedRewiring() const;

            /** \brief Prune during resorts */
            void setPruneDuringResort(bool prune);

            /** \brief Prune during resorts */
            bool getPruneDuringResort() const;
            //////////////////

            //////////////////
            // Get the progress property counters
            /** \brief The number of edges popped off the queue for processing (numEdgesPopped_). */
            unsigned int numEdgesPopped() const;
            //////////////////
        private:
            ////////////////////////////////
            // High level primitives:
            /** \brief Make sure that all vertices in our tree with a cost-to-come less than the minimum cost in our
             * edge queue has been expanded. Calls expandVertex() for each such vertex. */
            void updateQueue();

            /** \brief Update the edge queue by expanding the next vertex. */
            void expandNextVertex();

            /** \brief Update the edge queue by adding all the potential edges from the vertex to nearby states. */
            void expandVertex(const VertexPtr &vertex);

            /** \brief Iterate through the list of neighbouring unconnected vertices and add potential edges to
            * the queue if the vertex is marked as new *or* we're adding all of them. */
            void enqueueSamples(const VertexPtr &vertex, const VertexPtrVector& neighbourSamples, bool addAll);

            /** \brief Iterate through the list of neighbouring vertices and add potential edges to the queue. */
            void enqueueVertices(const VertexPtr &vertex, const VertexPtrVector& neighbourVertices);

            /** \brief Attempt to add an edge to the queue. Checks that the edge meets the queueing condition. */
            void enqueueEdgeConditionally(const VertexPtr &parent, const VertexPtr &child);

            /** \brief Given two subsets containing (up to) the k-nearest members of each, finds the k-nearest of the
             * union */
            void processKNearest(const VertexConstPtr &vertex, VertexPtrVector *kNearSamples,
                                 VertexPtrVector *kNearVertices);
            ////////////////////////////////

            ////////////////////////////////
            // Vertex helper functions:
            /** \brief Resort a vertex and its associated edges in the various queues. This is the main workhorse of resorting. */
            void resortVertex(const VertexPtr &unorderedVertex);

            /** \brief Prune a branch of the graph. Returns the number of vertices removed, and the number of said
             * vertices that are completely thrown away (i.e., are not even useful as a sample) */
            std::pair<unsigned int, unsigned int> pruneBranch(const VertexPtr &branchBase);

            /** \brief Disconnect a vertex from its parent by removing the edges stored in itself, and its parents.
             * Cascades cost updates if requested.*/
            void disconnectParent(const VertexPtr &oldVertex, bool cascadeCostUpdates);

            /** \brief Insert a vertex into the queue and lookups. Expands vertex into edges if it comes before the
             * expansion token and expandIfBeforeToken is true. */
            void vertexInsertHelper(const VertexPtr &newVertex, bool expandIfBeforeToken, bool removeFromFree,
                                    bool addToNNStruct);

            /** \brief Remove a vertex from the vertex queue and optionally also its edge queue and NN entries.
             * Returns the number of vertices that are completely deleted. */
            unsigned int vertexRemoveHelper(const VertexPtr &oldVertex, bool fullyRemove);
            ////////////////////////////////

            ////////////////////////////////
            // Base-queue basic helper functions:
            /** \brief A convenience function for the value of a vertex in the queue */
            CostDouble vertexQueueValue(const VertexPtr &vertex) const;

            /** \brief A convenience function for the value of an edge in the queue */
            CostTriple edgeQueueValue(const VertexPtrPair &edge) const;

            /** A lexicographical comparison function for the std::arrays of costs. This is the sorting function for
             * both the vertex and edge queues and is just a wrapper to std::lexicographical_compare.*/
            template <std::size_t SIZE>
            bool queueComparison(const std::array<ompl::base::Cost, SIZE> &lhs,
                                 const std::array<ompl::base::Cost, SIZE> &rhs) const;
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

            /** \brief The underlying queue of vertices. Sorted by vertexQueueComparison. */
            VertexQueueAsMMap vertexQueue_;

            /** \brief The next vertex in the expansion queue to expand*/
            VertexQueueIter vertexToExpand_;

            /** \brief The underlying queue of edges. Sorted by edgeQueueComparison. */
            EdgeQueueAsPairBinHeap edgeQueue_;

            /** \brief A vector of vertices that we will need to process when resorting the queue: */
            VertexPtrVector resortVertices_;

            /** \brief The cost of the best solution, which is the maximum heuristic value allowed for vertices/edges in the queue.*/
            ompl::base::Cost solnCost_{std::numeric_limits<double>::infinity()};

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
