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

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_INTEGRATEDQUEUE_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_INTEGRATEDQUEUE_

//STL/Boost/etc.:
//std::pair
#include <utility>
//std::vector
#include <vector>
//std::list
#include <list>
//std::multimap
#include <map>
//std::unordered_map
#include <unordered_map>
//For std::function
#include <functional>

//OMPL:
//The cost class:
#include "ompl/base/Cost.h"
//The optimization objective class:
#include "ompl/base/OptimizationObjective.h"
//The nearest neighbours structure
#include "ompl/datastructures/NearestNeighbors.h"

//BIT*:
//I am member class of the BITstar class, so I need to include it's definition to be aware of the class BITstar. It has a forward declaration to me.
#include "ompl/geometric/planners/bitstar/BITstar.h"
//The vertex class:
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"

namespace ompl
{
    namespace geometric
    {


        /** @anchor IntegratedQueue
        \par Short Description
        An integrated two-stage queue that consists of vertices expanded into edges to be processed.
        The integrated queue consists of a vertex expansion queue and an edge processing queue.
        Vertices are expanded as needed from the vertex queue into edges places in the edge queue.
        Edges are removed from the edge queue for processing by BIT*.
        The vertex queue is implemented as a static ordered list of the vertices in the graph with a token (i.e., an iterator)
        pointing to the next vertex that needs to be expanded. This is specifically a multimap ordered on ompl::base::Cost
        The edge queue is implemented as an ordered list of potential edges. It is filled by the vertex queue and emptied by
        popping the best value off the front. It is specifically a multimap ordered on std::pair<ompl::base::Cost, ompl::base::Cost>

        @par Notes:
            - An eraseEdge() function could be made by mimicking the vertex -> vertexQueue_::iterator lookup datastructure for the edgeQueue_
        */

        /** \brief A queue of edges to be processed that integrates both the expansion of \ref gVertex "Vertices" and the ordering of the resulting edges. */
        class BITstar::IntegratedQueue
        {
        public:
            ////////////////////////////////
            //Data typedefs:
            /** \brief A typedef for a pair of costs, i.e., the edge sorting key */
            typedef std::pair<ompl::base::Cost, ompl::base::Cost> CostPair;
            ////////////////////////////////

            ////////////////////////////////
            //Function typedefs:
            /** \brief A std::function definition of a heuristic function for a vertex. */
            typedef std::function<ompl::base::Cost (const VertexConstPtr&)> VertexHeuristicFunc;

            /** \brief A std::function definition of a heuristic function for an edge. */
            typedef std::function<ompl::base::Cost (const VertexConstPtrPair&)> EdgeHeuristicFunc;

            /** \brief A std::function definition for the distance between two vertices. */
            typedef std::function<double (const VertexConstPtr&, const VertexConstPtr&)> DistanceFunc;

            /** \brief A std::function definition for the neighbourhood of a vertex . */
            typedef std::function<unsigned int (const VertexPtr&, std::vector<VertexPtr>*)> NeighbourhoodFunc;
            ////////////////////////////////



            ////////////////////////////////
            //Public functions:
            /** \brief Construct an integrated queue. */
            //std::make_shared can only take 9 arguments, so be careful:
            IntegratedQueue(const ompl::base::OptimizationObjectivePtr& opt, const DistanceFunc& distanceFunc, const NeighbourhoodFunc& nearSamplesFunc, const NeighbourhoodFunc& nearVerticesFunc, const VertexHeuristicFunc& lowerBoundHeuristicVertex, const VertexHeuristicFunc& currentHeuristicVertex, const EdgeHeuristicFunc& lowerBoundHeuristicEdge, const EdgeHeuristicFunc& currentHeuristicEdge, const EdgeHeuristicFunc& currentHeuristicEdgeTarget);

            virtual ~IntegratedQueue();

            /** \brief Delay considering rewiring edges until an initial solution is found. This improves
            the time required to find an initial solution when doing so requires multiple batches and has
            no effects on theoretical asymptotic optimality (as the rewiring edges are eventually considered). */
            void setDelayedRewiring(bool delayRewiring);

            /** \brief Get whether BIT* is delaying rewiring until a solution is found. */
            bool getDelayedRewiring() const;

            //////////////////
            //Insert and erase
            /** \brief Insert a vertex into the vertex expansion queue. Vertices remain in the vertex queue until pruned or manually removed. A moving token marks the line between expanded and not expanded vertices.*/
            void insertVertex(const VertexPtr& newVertex);

            /** \brief Insert an edge into the edge processing queue. Edges are removed from the processing queue. This is only valid if the source vertex is already in the expansion queue (though it may already be expanded). */
            void insertEdge(const VertexPtrPair& newEdge);

            /** \brief Erase a vertex from the vertex expansion queue. Will disconnect the vertex from its parent and remove the associated incoming and outgoing edges from the edge queue as requested.*/
            void eraseVertex(const VertexPtr& oldVertex, bool disconnectParent, const VertexPtrNNPtr& vertexNN, const VertexPtrNNPtr& freeStateNN, std::vector<VertexPtr>* recycledVertices);
            //////////////////

            //////////////////
            //Access the queue
            /** \brief Get the best vertex on the queue without incrementing the vertex queue. */
            VertexPtr frontVertex();

            /** \brief Get the best edge on the queue, leaving it on the edge queue. */
            VertexPtrPair frontEdge();

            /** \brief Get the value of the best vertex on the queue without incrementing the vertex queue */
            ompl::base::Cost frontVertexValue();

            /** \brief Get the value of the best edge on the queue, leaving it on the edge queue */
            CostPair frontEdgeValue();

            /** \brief Pop the best edge off the queue, removing it from the edge queue in the process. */
            void popFrontEdge(VertexPtrPair* bestEdge);

            /** \brief Pop the best edge off the queue, removing it from the edge queue in the process. */
            VertexPtrPair popFrontEdge();
            //////////////////

            //////////////////
            //Queue maintenance
            /** \brief Mark that a solution has been found */
            void hasSolution();

            /** \brief Set the threshold of the queue */
            void setThreshold(const ompl::base::Cost& costThreshold);

            /** \brief Get the threshold of the queue */
            ompl::base::Cost getThreshold() const;

            /** \brief Erase all edges in the edge queue that lead to the given vertex */
            void removeEdgesTo(const VertexPtr& cVertex);

            /** \brief Erase all edges in the edge queue that leave from the given vertex */
            void removeEdgesFrom(const VertexPtr& pVertex);

            /** \brief Prune edges in the edge queue that lead to the given vertex using the prune function  */
            void pruneEdgesTo(const VertexPtr& cVertex);

            /** \brief Prune edges in the edge queue that leave from the given vertex using the prune function */
            void pruneEdgesFrom(const VertexPtr& pVertex);

            /** \brief Mark the queue as requiring resorting downstream of the specified vertex */
            void markVertexUnsorted(const VertexPtr& vertex);

            /** \brief Prune the vertex queue of vertices whose their lower-bound heuristic is greater then the threshold. Descendents of pruned vertices that are not pruned themselves are returned to the set of free states. Returns the number of vertices pruned (either removed completely or moved to the set of free states). */
            std::pair<unsigned int, unsigned int> prune(const VertexPtr& pruneStartPtr, const VertexPtrNNPtr& vertexNN, const VertexPtrNNPtr& freeStateNN, std::vector<VertexPtr>* recycledVertices);

            /** \brief Resort the queue, only reinserting edges/vertices if their lower-bound heuristic is less then the threshold. Descendents of pruned vertices that are not pruned themselves are returned to the set of free states. Requires first marking the queue as unsorted. Returns the number of vertices pruned (either removed completely or moved to the set of free states). */
            std::pair<unsigned int, unsigned int> resort(const VertexPtrNNPtr& vertexNN, const VertexPtrNNPtr& freeStateNN, std::vector<VertexPtr>* recycledVertices);

            /** \brief Finish the queue, clearing all the edge containers and moving the vertex expansion token to the end. After a call to finish, isEmpty() will return true. Keeps threshold, list of unsorted vertices, etc.*/
            void finish();

            /** \brief Reset the queue, clearing all the edge containers and moving the vertex expansion token to the start. After a call to reset, isEmpty() will return false (unless there is no data in the queue of course). Keeps threshold, list of unsorted vertices, etc.*/
            void reset();

            /** \brief Clear the queue to the state of construction. */
            void clear();
            //////////////////

            //////////////////
            //Queue info:
            /** \brief The condition used to prune vertices out of the queue. Compares lowerBoundHeuristicVertex to the given threshold. Returns true if the vertex's best cost is greater than the internally set threshold. Used internally during resort()*/
            bool vertexPruneCondition(const VertexPtr& vertex) const;

            /** \brief The condition used to prune disconnected samples from the free set. Compares lowerBoundHeuristicVertex to the given threshold. Returns true if the vertex's best cost is greater than or equal to the internally set threshold. Used internally during resort()*/
            bool samplePruneCondition(const VertexPtr& vertex) const;

            /** \brief The condition used to prune edge (i.e., vertex-pair) out of the queue. Compares lowerBoundHeuristicEdge to the given threshold. Returns true if the edge's best cost is greater than the internally set threshold. Used internally during resort()*/
            bool edgePruneCondition(const VertexPtrPair& edge) const;

            /** \brief Returns the number of edges in the queue */
            unsigned int numEdges() const;

            /** \brief Returns the number of vertices left to expand. This has nontrivial cost, as the token must be moved through the list to count */
            unsigned int numVertices() const;

            /** \brief Get the number of edges in the queue pointing to a specific vertex */
            unsigned int numEdgesTo(const VertexPtr& cVertex) const;

            /** \brief Get the number of edges in the queue coming from a specific vertex */
            unsigned int numEdgesFrom(const VertexPtr& pVertex) const;

            /** \brief Return whether the queue is still sorted */
            bool isSorted() const;

            /** \brief Returns true if the queue is reset. This means that no edges have been expanded and the vertex expansion token is pointing at the start. */
            bool isReset() const;

            /** \brief Returns true if the queue is empty. In the case where the edge queue is empty but the vertex queue is not, this function will expand vertices *until* the edge queue is not empty or there are no vertices to expand. */
            bool isEmpty();

            /** \brief Returns whether a given vertex has been expanded or not */
            bool isVertexExpanded(const VertexConstPtr& vertex) const;

            /** \brief Get a copy of the vertices in the vertex queue that are left to be expanded. This is expensive and is only meant for animations/debugging. */
            void listVertices(std::vector<VertexConstPtr>* vertexQueue);

            /** \brief Get a copy of the edge queue. This is expensive and is only meant for animations/debugging. */
            void listEdges(std::vector<std::pair<VertexConstPtr, VertexConstPtr> >* edgeQueue);
            //////////////////
            ////////////////////////////////




        private:
            ////////////////////////////////
            //Helpful typedefs:
            /** \brief A typedef to the underlying vertex queue as a multiset.  The advantage to a multimap over a multiset is that a copy of the key is stored with the value, which guarantees that the ordering remains sane. Even if the inherent key for a value has changed, it will still be sorted under the old key until manually updated and the map will be sorted */
            typedef std::multimap<ompl::base::Cost, VertexPtr, std::function<bool (const ompl::base::Cost&, const ompl::base::Cost&)> > CostToVertexMMap;

            /** \brief A typedef to the underlying edge queue as a multimap. Multimapped for the same reason as CostToVertexMMap */
            typedef std::multimap<CostPair, VertexPtrPair, std::function<bool (const CostPair&, const CostPair&)> > CostToVertexPtrPairMMap;

            /** \brief A typedef for an iterator into the vertex queue multimap */
            typedef CostToVertexMMap::iterator VertexQueueIter;

            /** \brief A typedef for an unordered_map of vertex queue iterators indexed on vertex*/
            typedef std::unordered_map<BITstar::VertexId, VertexQueueIter> VertexIdToVertexQueueIterUMap;

            /** \brief A typedef for an iterator into the edge queue multimap */
            typedef CostToVertexPtrPairMMap::iterator EdgeQueueIter;

            /** \brief A typedef for a list of edge queue iterators*/
            typedef std::list<EdgeQueueIter> EdgeQueueIterList;

            /** \brief A typedef for an unordered_map of edge queue iterators indexed by vertex*/
            typedef std::unordered_map<BITstar::VertexId, EdgeQueueIterList> VertexIdToEdgeQueueIterListUMap;
            ////////////////////////////////


            ////////////////////////////////
            //High level primitives:
            /** \brief Make sure that all vertices in our tree with a cost-to-come less than the minimum cost in our edge queue has been expanded. Calls expandVertex() for each such vertex. */
            void updateQueue();

            /** \brief Update the edge queue by expanding the next vertex. */
            void expandNextVertex();

            /** \brief Update the edge queue by adding all the potential edges from the vertex to nearby states. */
            void expandVertex(const VertexPtr& vertex);

            /** \brief Attempt to add an edge to the queue. Checks that the edge meets the queueing condition and that it is not in the failed set (if appropriate). */
            void queueupEdge(const VertexPtr& parent, const VertexPtr& child);

            /** \brief Given two subsets containing (up to) the k-nearest members of each, finds the k-nearest of the union */
            void processKNearest(unsigned int k, const VertexConstPtr& vertex, std::vector<VertexPtr>* kNearSamples, std::vector<VertexPtr>* kNearVertices);
            ////////////////////////////////


            ////////////////////////////////
            //Vertex helper functions:
            /** \brief Reinsert a vertex and its associated queue edges. This is the main workhorse of resorting. */
            void reinsertVertex(const VertexPtr& unorderedVertex);

            /** \brief Prune a branch of the graph. Returns the number of vertices removed, and the number of said vertices that are completely thrown away (i.e., are not even useful as a sample) */
            std::pair<unsigned int, unsigned int> pruneBranch(const VertexPtr& branchBase, const VertexPtrNNPtr& vertexNN, const VertexPtrNNPtr& freeStateNN, std::vector<VertexPtr>* recycledVertices);

            /** \brief Disconnect a vertex from its parent by removing the edges stored in itself, and its parents. Cascades cost updates if requested.*/
            void disconnectParent(const VertexPtr& oldVertex, bool cascadeCostUpdates);

            /** \brief Insert a vertex into the queue and lookups. Expands vertex into edges if it comes before the expansion token and expandIfBeforeToken is true. */
            void vertexInsertHelper(const VertexPtr& newVertex, bool expandIfBeforeToken);

            /** \brief Remove a vertex from the queue and optionally its entries in the various lookups. Returns the number of vertices that are completely deleted. */
            //This is *NOT* by const-reference so that the oldVertex pointer doesn't go out of scope on me... which was happening if it was being called with an iter->second where the iter gets deleted in this function...
            unsigned int vertexRemoveHelper(VertexPtr oldVertex, const VertexPtrNNPtr& vertexNN, const VertexPtrNNPtr& freeStateNN, std::vector<VertexPtr>* recycledVertices, bool removeLookups);
            ////////////////////////////////

            ////////////////////////////////
            //Edge helper functions:
            /** \brief Insert an edge into the queue and lookups with an optional hint. Pass edgeQueue_.end() as the iterator if the hint is not needed. */
            void edgeInsertHelper(const VertexPtrPair& newEdge, EdgeQueueIter positionHint);

            /** \brief Erase an edge by iterator. The two boolean flags should be true by default. */
            void edgeRemoveHelper(const EdgeQueueIter& oldEdgeIter, bool rmIncomingLookup, bool rmOutgoingLookup);

            /** \brief Helper wrapper to remove an incoming lookup*/
            void rmIncomingLookup(const EdgeQueueIter& mmapIterToRm);

            /** \brief Helper wrapper to remove an outgoing lookup*/
            void rmOutgoingLookup(const EdgeQueueIter& mmapIterToRm);

            /** \brief Erase an edge from the given lookup container at the specified index */
            void rmEdgeLookupHelper(VertexIdToEdgeQueueIterListUMap& lookup, const BITstar::VertexId& idx, const EdgeQueueIter& mmapIterToRm);
            ////////////////////////////////




            ////////////////////////////////
            //Base-queue basic helper functions:
            /** \brief A convenience function for the value of a vertex in the queue, uses currentHeuristicVertex */
            ompl::base::Cost vertexQueueValue(const VertexPtr& vertex) const;

            /** \brief A convenience function for the value of an edge in the queue, uses currentHeuristicEdge */
            CostPair edgeQueueValue(const VertexPtrPair& edge) const;

            /** \brief The comparison function for the cost associated with two vertices in the vertex queue. */
            bool vertexQueueComparison(const ompl::base::Cost& lhs, const ompl::base::Cost& rhs) const;

            /** A comparison function for the cost-pairs associated with two edges (i.e., vertex-pairs) in the edge queue. Sorts lexicographically */
            bool edgeQueueComparison(const CostPair& lhs, const CostPair& rhs) const;
            ////////////////////////////////

            ////////////////////////////////
            //Cost helpers
            /** \brief Compare whether cost a is worse than cost b by checking whether b is better than a. */
            bool isCostWorseThan(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Compare whether cost a and cost b are not equivalent by checking if either a or b is better than the other. */
            bool isCostNotEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Compare whether cost a is better or equivalent to cost b by checking that b is not better than a. */
            bool isCostBetterThanOrEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Compare whether cost a is worse or equivalent to cost b by checking that a is not better than b. */
            bool isCostWorseThanOrEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const;
            ////////////////////////////////



            ////////////////////////////////
            //Member variables:
            /** \brief My optimization objective. */
            ompl::base::OptimizationObjectivePtr                     opt_;

            /** \brief The distance function */
            DistanceFunc                                             distanceFunc_;

            /** \brief The function to find nearby samples. */
            NeighbourhoodFunc                                        nearSamplesFunc_;

            /** \brief The function to find nearby samples. */
            NeighbourhoodFunc                                        nearVerticesFunc_;

            /** \brief The lower-bounding heuristic for a vertex. */
            VertexHeuristicFunc                                      lowerBoundHeuristicVertexFunc_;

            /** \brief The current heuristic for a vertex. */
            VertexHeuristicFunc                                      currentHeuristicVertexFunc_;

            /** \brief The lower-bounding heuristic for an edge. */
            EdgeHeuristicFunc                                        lowerBoundHeuristicEdgeFunc_;

            /** \brief The current heuristic for an edge. */
            EdgeHeuristicFunc                                        currentHeuristicEdgeFunc_;

            /** \brief The current heuristic to the end of an edge. */
            EdgeHeuristicFunc                                        currentHeuristicEdgeTargetFunc_;

            /** \brief Whether to delay rewiring until an initial solution is found or not */
            bool                                                     delayRewiring_;

            /** \brief Whether to use parent lookup tables or not */
            bool                                                     outgoingLookupTables_;

            /** \brief Whether to use child lookup tables or not */
            bool                                                     incomingLookupTables_;

            /** \brief The underlying queue of vertices. Sorted by vertexQueueComparison. */
            CostToVertexMMap                                         vertexQueue_;

            /** \brief The next vertex in the expansion queue to expand*/
            VertexQueueIter                                          vertexToExpand_;

            /** \brief The underlying queue of edges. Sorted by edgeQueueComparison. */
            CostToVertexPtrPairMMap                                  edgeQueue_;

            /** \brief A lookup from vertex to iterator in the vertex queue */
            VertexIdToVertexQueueIterUMap                            vertexIterLookup_;

            /** \brief A unordered map from a vertex to all the edges in the queue emanating from the vertex: */
            VertexIdToEdgeQueueIterListUMap                          outgoingEdges_;

            /** \brief A unordered map from a vertex to all the edges in the queue leading into the vertex: */
            VertexIdToEdgeQueueIterListUMap                          incomingEdges_;

            /** \brief A list of vertices that we will need to process when resorting the queue: */
            std::list<VertexPtr>                                     resortVertices_;

            /** \brief The maximum heuristic value allowed for vertices/edges in the queue.*/
            ompl::base::Cost                                         costThreshold_;

            /** \brief Whether the problem has a solution */
            bool                                                     hasSolution_;
            ////////////////////////////////
        }; //class: IntegratedQueue
    } //geometric
} //ompl
#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_INTEGRATEDQUEUE_

