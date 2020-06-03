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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_VERTEX_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_VERTEX_

#include <memory>
#include <vector>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/planners/informedtrees/BITstar.h"
#include "ompl/geometric/planners/informedtrees/bitstar/SearchQueue.h"

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

        /** \brief The vertex of the underlying graphs in \ref gBITstar BIT*. */
        class BITstar::Vertex
        {
        public:
            // ---
            // Construction and destruction.
            // ---

            /** \brief Construct a vertex using space information, and helpers to compute various costs. */
          Vertex(ompl::base::SpaceInformationPtr spaceInformation, const CostHelper *const costHelpPtr, SearchQueue *const queuePtr,
                   const std::shared_ptr<const unsigned int> &approximationId, bool root = false);

            /** \brief Destruct a vertex. */
            virtual ~Vertex();

            // ---
            // State access.
            // ---

            /** \brief The (unique) vertex ID. */
            BITstar::VertexId getId() const;

            /** \brief The state of a vertex as a pointer. */
            ompl::base::State *state();

            /** \brief The state of a vertex as a pointer to const. */
            ompl::base::State const *state() const;

            // ---
            // Graph information access.
            // ---

            /** \brief Returns whether the vertex is the root of the search tree. */
            bool isRoot() const;

            /** \brief Returns whether this vertex has a parent. */
            bool hasParent() const;

            /** \brief Get whether a vertex is in the search tree or a sample (i.e., a vertex of the RRG). */
            bool isInTree() const;

            /** \brief Get the depth of the vertex from the root. */
            unsigned int getDepth() const;

            /** \brief Get a const pointer to the parent of this vertex. */
            VertexConstPtr getParent() const;

            /** \brief Get a pointer to the parent of this vertex. */
            VertexPtr getParent();

            /** \brief Whether the vertex is consistent. */
            bool isConsistent() const;

            /** \brief Whether the vertex has been pruned. */
            bool isPruned() const;

            /** \brief Returns whether the vertex is expanded on current approximation. */
            bool isExpandedOnCurrentApproximation() const;

            /** \brief Returns whether the vertex is expaned on current search. */
            bool isExpandedOnCurrentSearch() const;

            /** \brief Returns whether the vertex has ever been expanded as a rewiring. */
            bool hasEverBeenExpandedAsRewiring() const;

            // ---
            // Graph modification.
            // ---

            /** \brief Set the parent of this vertex, cannot be used to replace a previous parent. Will always update
             * this vertex's cost, and can update descendent costs. */
            void addParent(const VertexPtr &newParent, const ompl::base::Cost &edgeInCost);

            /** \brief Remove the parent of this vertex. Will always update this vertex's cost, and can update the descendent costs. */
            void removeParent(bool updateChildCosts);

            /** \brief Get whether this vertex has any children. */
            bool hasChildren() const;

            /** \brief Get the children of a vertex as constant pointers. */
            void getChildren(VertexConstPtrVector *children) const;

            /** \brief Get the children of a vertex as mutable pointers. */
            void getChildren(VertexPtrVector *children);

            /** \brief Add a child to this vertex. Does not change this vertex's cost or those of its descendants.
             * Child must already have this vertex listed as it's parent. */
            void addChild(const VertexPtr &newChild);

            /** \brief Remove a child from this vertex. Does not change this vertex's cost or those of its descendants.
            * Child must still have this vertex listed as its parent and it will also throw an exception if the given
            * vertex pointer is not found. */
            void removeChild(const VertexPtr &oldChild);

            /** \brief Put the vertex on the blacklist of children. */
            void blacklistChild(const VertexConstPtr &vertex);

            /** \brief Put the vertex on the whitelist of children. */
            void whitelistChild(const VertexConstPtr &vertex);

            /** \brief Returns true if the vertex is blacklisted as a child of this vertex. */
            bool isBlacklistedAsChild(const VertexConstPtr &vertex) const;

            /** \brief Returns true if the vertex is blacklisted as a child of this vertex. */
            bool isWhitelistedAsChild(const VertexConstPtr &vertex) const;

            /** \brief Clears the blacklist. */
            void clearBlacklist();

            /** \brief Clears the whitelist. */
            void clearWhitelist();

            /** \brief Get the cost-to-come of a vertex. Return infinity if the edge is disconnected. */
            ompl::base::Cost getCost() const;

            /** \brief Get the incremental cost-to-come of a vertex. */
            ompl::base::Cost getEdgeInCost() const;

            /** \brief Mark the vertex as expanded. */
            void registerExpansion();

            /** \brief Mark expansion to vertices. */
            void registerRewiringExpansion();

            /** \brief Mark the vertex as pruned. */
            void markPruned();

            /** \brief Mark the vertex as unpruned. */
            void markUnpruned();

            // ---
            // Edge queue lookups.
            // ---

            /** \brief Add to the list of the edge queue entries that point in to this vertex. Will clear existing in/out lookups if they were added under a different id. */
            void insertInEdgeQueueInLookup(const SearchQueue::EdgeQueueElemPtr &inEdge);

            /** \brief Add to the list of the edge queue entries that point out of this vertex. Will clear existing in/out lookups if they were added under a different id. */
            void insertInEdgeQueueOutLookup(const SearchQueue::EdgeQueueElemPtr &outEdge);

            /** \brief Remove an incoming edge queue entry by value to the member vector. */
            void removeFromEdgeQueueInLookup(const SearchQueue::EdgeQueueElemPtr &inEdge);

            /** \brief Remove an outgoing edge queue entry by value. */
            void removeFromEdgeQueueOutLookup(const SearchQueue::EdgeQueueElemPtr &outEdge);

            /** \brief Remove an incoming edge queue entry by iterator to the member vector. */
            void removeFromEdgeQueueInLookup(const SearchQueue::EdgeQueueElemPtrVector::const_iterator &inEdge);

            /** \brief Remove an outgoing edge queue entry by iterator to the member vector. */
            void removeFromEdgeQueueOutLookup(const SearchQueue::EdgeQueueElemPtrVector::const_iterator &outEdge);

            /** \brief Get an iterator to the front of the incoming edge queue entry vector. Will clear existing in/out lookups if they were added under a different id. */
            BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator edgeQueueInLookupConstBegin();

            /** \brief Get an iterator to the front of the outgoing edge queue entry vector. Will clear existing in/out lookups if they were added under a different id. */
            BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator edgeQueueOutLookupConstBegin();

            /** \brief Get an iterator to the end of the incoming edge queue entry vector. Will clear existing in/out lookups if they were added under a different id. */
            BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator edgeQueueInLookupConstEnd();

            /** \brief Get an iterator to the end of the outgoing edge queue entry vector. Will clear existing in/out lookups if they were added under a different id. */
            BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator edgeQueueOutLookupConstEnd();

            /** \brief Get the number of edge queue entries incoming to this vertex. Will clear existing in/out lookups if they were added under a different id. */
            unsigned int edgeQueueInLookupSize();

            /** \brief Get the number of edge queue entries outgoing from this vertex. Will clear existing in/out lookups if they were added under a different id. */
            unsigned int edgeQueueOutLookupSize();

            /** \brief Clear the pointers to all of the incoming edge queue entries. */
            void clearEdgeQueueInLookup();

            /** \brief Clear the pointers to all of the outgoing edge queue entries. */
            void clearEdgeQueueOutLookup();

        private:
            // ---
            // Internal bookkeeping.
            // ---

            /** \brief Calculates the updated cost and depth of the current state, optionally calls itself on all children. */
            void updateCostAndDepth(bool cascadeUpdates = true);

            // ---
            // Member variables.
            // ---

            /** \brief The vertex id. */
            BITstar::VertexId id_;

            /** \brief The state space used by the planner. */
            ompl::base::SpaceInformationPtr si_;

            /** \brief The helper class to compute different costs. */
            const CostHelper *const costHelpPtr_;

            /** \brief The search queue used by the algorithms. */
            SearchQueue *const queuePtr_;

            /** \brief The state itself. */
            ompl::base::State *state_;

            /** \brief Whether the vertex is a root. */
            bool isRoot_;

            /** \brief Whether the vertex is pruned. Vertices throw if any member function other than isPruned() is
             * access after they are pruned. */
            bool isPruned_{false};

            /** \brief The depth of the state.  */
            unsigned int depth_{0u};

            /** \brief The parent state as a shared pointer such that the parent will not be deleted until all the
             * children are. */
            VertexPtr parentPtr_;

            /** \brief The incremental cost to get to the state. I.e., the cost of the parent -> state edge. */
            ompl::base::Cost edgeCost_;

            /** \brief The cost-to-come to this vertex. */
            ompl::base::Cost cost_;

            /** \brief The cost-to-come to this vertex at the time of its last expansion. */
            ompl::base::Cost costAtExpansion_;

            /** \brief The child states as weak pointers, such that the ownership loop is broken and a state can be
             * deleted once it's children are. */
            std::vector<VertexWeakPtr> children_;

            /** \brief A list of pointers to elements in the edge queue that point in to this vertex. */
            SearchQueue::EdgeQueueElemPtrVector edgeQueueInLookup_;

            /** \brief A list of pointers to elements in the edge queue that point out from this vertex. */
            SearchQueue::EdgeQueueElemPtrVector edgeQueueOutLookup_;

            /** \brief A collection of potential child vertex ids that are blacklisted for edges (due to a collision). */
            std::set<BITstar::VertexId> childIdBlacklist_;

            /** \brief A collection of potential child vertex ids that are whitelisted for edges. */
            std::set<BITstar::VertexId> childIdWhitelist_;

            /** \brief The id number associated with the search in which the lookups are up to date. */
            unsigned int lookupApproximationId_{0u};

            /** \brief The id number associated with the approximation on which this vertex was last expanded. */
            unsigned int expansionApproximationId_{0u};

            /** \brief The id number associated with the search in which this vertex was last expanded. */
            unsigned int expansionSearchId_{0u};

            /** \brief Whether this sample has ever been expanded to vertices. */
            bool hasEverBeenExpandedAsRewiring_{false};

            /** \brief A pointer to the shared memory that holds the current search id. */
            const std::shared_ptr<const unsigned int> currentSearchId_;

            /** \brief A pointer to the shared memory that holds the current approximation id. */
            const std::shared_ptr<const unsigned int> currentApproximationId_;

            /** \brief A helper function to clear the given incoming lookup (and in debug mode assert it existed). */
            void removeFromEdgeQueueInLookup(const SearchQueue::EdgeQueueElemPtrVector::iterator &iterToDelete);

            /** \brief A helper function to clear the given outgoing lookup (and in debug mode assert it existed). */
            void removeFromEdgeQueueOutLookup(const SearchQueue::EdgeQueueElemPtrVector::iterator &iterToDelete);

            /** \brief A helper function to clear existing lookups if they are out of date (i.e., created at a different id than the one given). */
            void clearLookupsIfOutdated();
        };  // class Vertex
    } // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_VERTEX_
