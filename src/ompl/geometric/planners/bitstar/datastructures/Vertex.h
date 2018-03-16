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

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_VERTEX_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_VERTEX_

// vector
#include <vector>

// shared and weak pointers
#include <memory>
// For unordered sets of failed children:
#include <unordered_set>

// OMPL:
// The space information
#include "ompl/base/SpaceInformation.h"
// The optimization objective
#include "ompl/base/OptimizationObjective.h"

// BIT*:
// I am member class of the BITstar class (i.e., I am in it's namespace), so I need to include it's definition to be
// aware of the class BITstar. It has a forward declaration to me and the other helper classes.
#include "ompl/geometric/planners/bitstar/BITstar.h"
// I store data for the SearchQueue, get their definitions.
#include "ompl/geometric/planners/bitstar/datastructures/SearchQueue.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor gVertex
        @par Short description
        A class to store a state as a vertex in a (tree) graph. Allocates and frees it's own memory on
        construction/destruction. Parent vertices are owned by their children as shared pointers, assuring that a parent
        vertex will not be deleted while the child exists. Child vertices are owned by their parents as weak pointers,
        assuring that the shared-pointer ownership loop is broken.

        @par Note
        Add/Remove functions should almost always update their children's cost. The only known exception is when a
        series of operations are being performed and it would be beneficial to delay the update until the last
        operation. In this case, make sure that the last call updates the children and is on the highest ancestor that
        has been changed. Updates only flow downstream.
        */

        /** \brief The vertex of the underlying graphs in \ref gBITstar "BIT*"*/
        class BITstar::Vertex
        {
        public:
            ////////////////////////////////////////////////////
            // Public functions:
            /** \brief Constructor */
            Vertex(ompl::base::SpaceInformationPtr si, const CostHelper *const costHelpPtr, bool root = false);

            /** \brief Destructor */
            virtual ~Vertex();

            /** \brief The (unique) vertex ID */
            BITstar::VertexId getId() const;

            /** \brief The state of a vertex as a constant pointer */
            ompl::base::State const *stateConst() const;

            /** \brief The state of a vertex as a mutable pointer*/
            ompl::base::State *state();

            ////////////////////////////
            // The vertex's graph properties:
            /** \brief Whether the vertex is root */
            bool isRoot() const;

            /** \brief Get whether this vertex has a parent */
            bool hasParent() const;

            /** \brief Get whether a vertex is "in the graph" or not. This returns true if the vertex is the graph root
             * or is connected to a parent. */
            bool isInTree() const;

            /** \brief Get the "depth" of the vertex from the root. A root vertex is at depth 0, a direct descendent of
             * the root 1, etc. */
            unsigned int getDepth() const;

            /** \brief Get the parent of a vertex as a constant pointer */
            VertexConstPtr getParentConst() const;

            /** \brief Get the parent of a vertex as a mutable pointer*/
            VertexPtr getParent();

            /** \brief Set the parent of this vertex, cannot be used to replace a previous parent. Will always update
             * this vertex's cost, and can update descendent costs */
            void addParent(const VertexPtr &newParent, const ompl::base::Cost &edgeInCost, bool updateChildCosts);

            /** \brief Remove the parent of this vertex. Will always update this vertex's cost, and can update the descendent costs */
            void removeParent(bool updateChildCosts);

            /** \brief Get whether this vertex has any children */
            bool hasChildren() const;

            /** \brief Get the children of a vertex as constant pointers */
            void getChildrenConst(VertexConstPtrVector *children) const;

            /** \brief Get the children of a vertex as mutable pointers */
            void getChildren(VertexPtrVector *children);

            /** \brief Add a child to this vertex. Does not change this vertex's cost or those of its descendants. Child must already have this vertex listed as it's parent. */
            void addChild(const VertexPtr &newChild);

            /** \brief Remove a child from this vertex. Does not change this vertex's cost or those of its descendants.
            * Child must still have this vertex listed as its parent and it will also throw an exception if the given
            * vertex pointer is not found. */
            void removeChild(const VertexPtr &oldChild);

            /** \brief Get the cost-to-come of a vertex. Return infinity if the edge is disconnected */
            ompl::base::Cost getCost() const;

            /** \brief Get the incremental cost-to-come of a vertex */
            ompl::base::Cost getEdgeInCost() const;

            /** \brief Returns true if the vertex is marked as new. Vertices are new until marked old. */
            bool isNew() const;

            /** \brief Mark the vertex as new. */
            void markNew();

            /** \brief Mark the vertex as old. */
            void markOld();

            /** \brief Returns true if the vertex has been expanded towards samples. */
            bool hasBeenExpandedToSamples() const;

            /** \brief Mark the vertex as expanded towards samples. */
            void markExpandedToSamples();

            /** \brief Mark the vertex as not expanded towards samples. */
            void markUnexpandedToSamples();

            /** \brief Returns true if the vertex has been expanded towards vertices. */
            bool hasBeenExpandedToVertices() const;

            /** \brief Mark the vertex as expanded towards vertices. */
            void markExpandedToVertices();

            /** \brief Mark the vertex as not expanded towards vertices. */
            void markUnexpandedToVertices();

            /** \brief Whether the vertex has been pruned */
            bool isPruned() const;

            /** \brief Mark the vertex as pruned. */
            void markPruned();

            /** \brief Mark the vertex as unpruned. */
            void markUnpruned();
            ////////////////////////////

            ////////////////////////////
            // Functions for the vertex's SearchQueue data
            //////////////
            // Vertex queue info:
            /** \brief Get an iterator to this vertex in the vertex queue */
            SearchQueue::VertexQueueIter getVertexQueueIter() const;

            /** \brief Set the iterator to this vertex in the vertex queue */
            void setVertexQueueIter(const SearchQueue::VertexQueueIter &newPtr);

            /** \brief Clear the iterator to this vertex in the vertex queue */
            void clearVertexQueueIter();

            /** \brief Return true if the vertex has a link to it's entry in the Vertex Queue */
            bool hasVertexQueueEntry() const;
            //////////////

            //////////////
            // Edge queue info (incoming edges):
            /** \brief Add to the list of the edge queue entries that point in to this vertex. Will clear existing in/out lookups if they were added under a different id.*/
            void addIncomingEdgeQueuePtr(const SearchQueue::EdgeQueueElemPtr &newInPtr, unsigned int vertexQueueResetNum);

            /** \brief Remove an incoming edge queue entry by value to the member vector.*/
            void rmIncomingEdgeQueuePtr(const SearchQueue::EdgeQueueElemPtr &elemToDelete, unsigned int vertexQueueResetNum);

            /** \brief Remove an incoming edge queue entry by iterator to the member vector.*/
            void rmIncomingEdgeQueuePtrByIter(const SearchQueue::EdgeQueueElemPtrVector::const_iterator &constIterToDelete, unsigned int vertexQueueResetNum);

            /** \brief Clear the pointers to all of the incoming edge queue entries */
            void clearIncomingEdgeQueuePtrs();

            /** \brief Get an iterator to the front of the incoming edge queue entry vector. Will clear existing in/out lookups if they were added under a different id. */
            BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator incomingEdgeQueuePtrsBeginConst(unsigned int vertexQueueResetNum);

            /** \brief Get an iterator to the end of the incoming edge queue entry vector. Will clear existing in/out lookups if they were added under a different id. */
            BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator incomingEdgeQueuePtrsEndConst(unsigned int vertexQueueResetNum);

            /** \brief Get the number of edge queue entries incoming to this vertex. Will clear existing in/out lookups if they were added under a different id. */
            unsigned int getNumIncomingEdgeQueuePtrs(unsigned int vertexQueueResetNum);

            /** \brief Return true if the vertex has links to incoming entries in the Edge Queue. Will clear existing in/out lookups if they were added under a different id. */
            bool hasIncomingEdgeQueueEntries(unsigned int vertexQueueResetNum);
            //////////////

            //////////////
            // Edge queue info (outgoing edges):
            /** \brief Add to the list of the edge queue entries that point out of this vertex. Will clear existing in/out lookups if they were added under a different id.*/
            void addOutgoingEdgeQueuePtr(const SearchQueue::EdgeQueueElemPtr &newOutPtr, unsigned int vertexQueueResetNum);

            /** \brief Remove an outgoing edge queue entry by value.*/
            void rmOutgoingEdgeQueuePtr(const SearchQueue::EdgeQueueElemPtr &elemToDelete, unsigned int vertexQueueResetNum);

            /** \brief Remove an outgoing edge queue entry by iterator to the member vector.*/
            void rmOutgoingEdgeQueuePtrByIter(const SearchQueue::EdgeQueueElemPtrVector::const_iterator &constIterToDelete, unsigned int vertexQueueResetNum);

            /** \brief Clear the pointers to all of the outgoing edge queue entries */
            void clearOutgoingEdgeQueuePtrs();

            /** \brief Get an iterator to the front of the outgoing edge queue entry vector. Will clear existing in/out lookups if they were added under a different id. */
            BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator outgoingEdgeQueuePtrsBeginConst(unsigned int vertexQueueResetNum);

            /** \brief Get an iterator to the end of the outgoing edge queue entry vector. Will clear existing in/out lookups if they were added under a different id. */
            BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator outgoingEdgeQueuePtrsEndConst(unsigned int vertexQueueResetNum);

            /** \brief Get the number of edge queue entries outgoing from this vertex. Will clear existing in/out lookups if they were added under a different id. */
            unsigned int getNumOutgoingEdgeQueuePtrs(unsigned int vertexQueueResetNum);

            /** \brief Return true if the vertex has links to outgoing entries in the Edge Queue. Will clear existing in/out lookups if they were added under a different id. */
            bool hasOutgoingEdgeQueueEntries(unsigned int vertexQueueResetNum);
            //////////////
            ////////////////////////////
            ////////////////////////////////////////////////////
        protected:
            /** \brief Calculates the updated cost and depth of the current state, as well as calling all children's
             * updateCostAndDepth() functions and thus updating everything down-stream (if desired).*/
            void updateCostAndDepth(bool cascadeUpdates = true);

        private:
            /** \brief The vertex ID */
            BITstar::VertexId vId_;

            /** \brief The state space used by the planner */
            ompl::base::SpaceInformationPtr si_;

            /** \brief The optimization objective used by the planner */
            const CostHelper *const costHelpPtr_;

            /** \brief The state itself */
            ompl::base::State *state_;

            /** \brief Whether the vertex is a root */
            bool isRoot_;

            /** \brief Whether the vertex is new. */
            bool isNew_{true};

            /** \brief Whether the vertex had been expanded to samples. */
            bool hasBeenExpandedToSamples_{false};

            /** \brief Whether the vertex has been expanded to vertices. */
            bool hasBeenExpandedToVertices_{false};

            /** \brief Whether the vertex is pruned. Vertices throw if any member function other than isPruned() is
             * access after they are pruned. */
            bool isPruned_{false};

            /** \brief The depth of the state  */
            unsigned int depth_{0u};

            /** \brief The parent state as a shared pointer such that the parent will not be deleted until all the
             * children are. */
            VertexPtr parentSPtr_;

            /** \brief The incremental cost to get to the state. I.e., the cost of the parent -> state edge */
            ompl::base::Cost edgeCost_;

            /** \brief The cost of the state  */
            ompl::base::Cost cost_;

            /** \brief The child states as weak pointers, such that the ownership loop is broken and a state can be
             * deleted once it's children are.*/
            std::vector<VertexWeakPtr> childWPtrs_;

            /** \brief A pointer to this vertex in the vertex queue */
            SearchQueue::VertexQueueIter vertexQueueIter_;

            /** \brief Whether a valid iterator the vertex queue exists (as iterators have no equivalent to NULL) */
            bool isVertexQueueSet_{false};

            /** \brief A list of pointers to elements in the edge queue that point in to this vertex */
            SearchQueue::EdgeQueueElemPtrVector edgeQueueInPtrs_;

            /** \brief A list of pointers to elements in the edge queue that point out from this vertex */
            SearchQueue::EdgeQueueElemPtrVector edgeQueueOutPtrs_;

            /** \brief The id-number associated with the currently stored edge lookups. This is used to reset existing
            * lookups when on the next pass through the vertex queue. */
            unsigned int edgeLookupPass_{0u};

            /** \brief A helper function to clear the given incoming lookup (and in debug mode assert it existed) */
            void rmIncomingHelper(const SearchQueue::EdgeQueueElemPtrVector::iterator &iterToDelete);

            /** \brief A helper function to clear the given outgoing lookup (and in debug mode assert it existed) */
            void rmOutgoingHelper(const SearchQueue::EdgeQueueElemPtrVector::iterator &iterToDelete);

            /** \brief A helper function to clear existing lookups if they are out of date (i.e., created at a different id than the one given) */
            void clearOldLookups(unsigned int vertexQueueResetNum);

            /** \brief A helper function to check that the vertex is not pruned and throw if so */
            void assertNotPruned() const;
        };  // class: Vertex
    }       // geometric
}  // ompl
#endif  // OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_VERTEX_
