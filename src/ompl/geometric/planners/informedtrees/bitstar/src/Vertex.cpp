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

// My definition:
#include "ompl/geometric/planners/informedtrees/bitstar/Vertex.h"

// For std::move
#include <utility>

// For exceptions:
#include "ompl/util/Exception.h"

// BIT*:
// A collection of common helper functions
#include "ompl/geometric/planners/informedtrees/bitstar/HelperFunctions.h"
// The ID generator class
#include "ompl/geometric/planners/informedtrees/bitstar/IdGenerator.h"
// The cost-helper class:
#include "ompl/geometric/planners/informedtrees/bitstar/CostHelper.h"

// Debug macros.
#ifdef BITSTAR_DEBUG
    // Debug setting. The id number of a vertex to track. Requires BITSTAR_DEBUG to be defined in BITstar.h
    #define TRACK_VERTEX_ID 0

    /** \brief A helper function to print out every function called on vertex "TRACK_VERTEX_ID" that changes it */
    #define PRINT_VERTEX_CHANGE \
        if (id_ == TRACK_VERTEX_ID) \
        { \
            std::cout << "Vertex " << id_ << ": " << __func__ << "" << std::endl; \
        }

    /** \brief Assert that the vertex is not pruned. */
    #define ASSERT_NOT_PRUNED \
        if (this->isPruned_) \
        { \
            std::cout << "Vertex " << id_ << ": " << __func__ << std::endl; \
            throw ompl::Exception("Attempting to access a pruned vertex."); \
        }
#else
    #define PRINT_VERTEX_CHANGE
    #define ASSERT_NOT_PRUNED
#endif  // BITSTAR_DEBUG

// An anonymous namespace to hide the instance:
namespace
{
    // Global variables:
    // The initialization flag stating that the ID generator has been created:
    std::once_flag g_IdInited;
    // A pointer to the actual ID generator
    boost::scoped_ptr<ompl::geometric::BITstar::IdGenerator> g_IdGenerator;

    // A function to initialize the ID generator pointer:
    void initIdGenerator()
    {
        g_IdGenerator.reset(new ompl::geometric::BITstar::IdGenerator());
    }

    // A function to get the current ID generator:
    ompl::geometric::BITstar::IdGenerator &getIdGenerator()
    {
        std::call_once(g_IdInited, &initIdGenerator);
        return *g_IdGenerator;
    }
}

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::Vertex::Vertex(ompl::base::SpaceInformationPtr spaceInformation, const CostHelper *const costHelpPtr, SearchQueue *const queuePtr,
                   const std::shared_ptr<const unsigned int> &approximationId, bool root)
          : id_(getIdGenerator().getNewId())
          , si_(std::move(spaceInformation))
          , costHelpPtr_(std::move(costHelpPtr))
          , queuePtr_(queuePtr)
          , state_(si_->allocState())
          , isRoot_(root)
          , edgeCost_(costHelpPtr_->infiniteCost())
          , cost_(costHelpPtr_->infiniteCost())
          , costAtExpansion_(costHelpPtr_->infiniteCost())
          , currentSearchId_(queuePtr->getSearchId())
          , currentApproximationId_(approximationId)
        {
            PRINT_VERTEX_CHANGE

            if (this->isRoot())
            {
                cost_ = costHelpPtr_->identityCost();
            }
            // No else, infinite by default
        }

        BITstar::Vertex::~Vertex()
        {
            PRINT_VERTEX_CHANGE

            // Free the state on destruction
            si_->freeState(state_);
        }

        BITstar::VertexId BITstar::Vertex::getId() const
        {
            ASSERT_NOT_PRUNED

            return id_;
        }

        ompl::base::State const *BITstar::Vertex::state() const
        {
            ASSERT_NOT_PRUNED

            return state_;
        }

        ompl::base::State *BITstar::Vertex::state()
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

            return state_;
        }

        /////////////////////////////////////////////
        // The vertex's graph properties:
        bool BITstar::Vertex::isRoot() const
        {
            ASSERT_NOT_PRUNED

            return isRoot_;
        }

        bool BITstar::Vertex::hasParent() const
        {
            ASSERT_NOT_PRUNED

            return static_cast<bool>(parentPtr_);
        }

        bool BITstar::Vertex::isInTree() const
        {
            ASSERT_NOT_PRUNED

            return this->isRoot() || this->hasParent();
        }

        unsigned int BITstar::Vertex::getDepth() const
        {
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            if (!this->isRoot() && !this->hasParent())
            {
                throw ompl::Exception("Attempting to get the depth of a vertex that does not have a parent yet is not "
                                      "root.");
            }
#endif  // BITSTAR_DEBUG

            return depth_;
        }

        BITstar::VertexConstPtr BITstar::Vertex::getParent() const
        {
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            if (!this->hasParent())
            {
                if (this->isRoot())
                {
                    throw ompl::Exception("Attempting to access the parent of the root vertex.");
                }
                else
                {
                    throw ompl::Exception("Attempting to access the parent of a vertex that does not have one.");
                }
            }
#endif  // BITSTAR_DEBUG

            return parentPtr_;
        }

        BITstar::VertexPtr BITstar::Vertex::getParent()
        {
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            if (!this->hasParent())
            {
                if (this->isRoot())
                {
                    throw ompl::Exception("Attempting to access the parent of the root vertex.");
                }
                else
                {
                    throw ompl::Exception("Attempting to access the parent of a vertex that does not have one.");
                }
            }
#endif  // BITSTAR_DEBUG

            return parentPtr_;
        }

        void BITstar::Vertex::addParent(const VertexPtr &newParent, const ompl::base::Cost &edgeInCost)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            // Assert I can take a parent
            if (this->isRoot())
            {
                throw ompl::Exception("Attempting to add a parent to the root vertex, which cannot have a parent.");
            }
            if (this->hasParent())
            {
                throw ompl::Exception("Attempting to add a parent to a vertex that already has one.");
            }
#endif  // BITSTAR_DEBUG

            // Store the parent.
            parentPtr_ = newParent;

            // Store the edge cost.
            edgeCost_ = edgeInCost;

            // Update my cost and that of my children.
            this->updateCostAndDepth(true);
        }

        void BITstar::Vertex::removeParent(bool updateChildCosts)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            // Assert I have a parent
            if (this->isRoot())
            {
                throw ompl::Exception("Attempting to remove the parent of the root vertex, which cannot have a "
                                      "parent.");
            }
            if (!this->hasParent())
            {
                throw ompl::Exception("Attempting to remove the parent of a vertex that does not have a parent.");
            }
#endif  // BITSTAR_DEBUG

            // Clear my parent
            parentPtr_.reset();

            // Update my cost and possibly the cost of my descendants:
            this->updateCostAndDepth(updateChildCosts);
        }

        bool BITstar::Vertex::hasChildren() const
        {
            ASSERT_NOT_PRUNED

            return !children_.empty();
        }

        void BITstar::Vertex::getChildren(VertexConstPtrVector *children) const
        {
            ASSERT_NOT_PRUNED

            children->clear();

            for (const auto &child : children_)
            {
#ifdef BITSTAR_DEBUG
                // Check that the weak pointer hasn't expired
                if (child.expired())
                {
                    throw ompl::Exception("A (weak) pointer to a child was found to have expired while collecting the "
                                          "children of a vertex.");
                }
#endif  // BITSTAR_DEBUG

                // Lock and push back
                children->push_back(child.lock());
            }
        }

        void BITstar::Vertex::getChildren(VertexPtrVector *children)
        {
            ASSERT_NOT_PRUNED

            children->clear();

            for (const auto &child : children_)
            {
#ifdef BITSTAR_DEBUG
                // Check that the weak pointer hasn't expired
                if (child.expired())
                {
                    throw ompl::Exception("A (weak) pointer to a child was found to have expired while collecting the "
                                          "children of a vertex.");
                }
#endif  // BITSTAR_DEBUG

                // Lock and push back
                children->push_back(child.lock());
            }
        }

        void BITstar::Vertex::addChild(const VertexPtr &child)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            // Assert that I am this child's parent
            if (child->isRoot())
            {
                throw ompl::Exception("Attempted to add a root vertex as a child.");
            }
            if (!child->hasParent())
            {
                throw ompl::Exception("Attempted to add child that does not have a listed parent.");
            }
            if (child->getParent()->getId() != id_)
            {
                throw ompl::Exception("Attempted to add someone else's child as mine.");
            }
#endif  // BITSTAR_DEBUG

            // Push back the shared_ptr into the vector of weak_ptrs, this makes a weak_ptr copy
            children_.push_back(child);

            // Leave the costs of the child out of date.
        }

        void BITstar::Vertex::removeChild(const VertexPtr &child)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            // Assert that I am this child's parent
            if (child->isRoot())
            {
                throw ompl::Exception("Attempted to remove a root vertex as a child.");
            }
            if (!child->hasParent())
            {
                throw ompl::Exception("Attempted to remove a child that does not have a listed parent.");
            }
            if (child->getParent()->getId() != id_)
            {
                throw ompl::Exception("Attempted to remove a child vertex from the wrong parent.");
            }
#endif  // BITSTAR_DEBUG

            // Variables
            // Whether the child has been found (and then deleted);
            bool foundChild;

            // Iterate over the vector of children pointers until the child is found. Iterators make erase easier
            foundChild = false;
            for (auto it = children_.begin(); it != children_.end() && !foundChild; ++it)
            {
#ifdef BITSTAR_DEBUG
                // Check that the weak pointer hasn't expired
                if (it->expired())
                {
                    throw ompl::Exception("A (weak) pointer to a child was found to have expired while removing a "
                                          "child from a vertex.");
                }
#endif  // BITSTAR_DEBUG

                // Check if this is the child we're looking for
                if (it->lock()->getId() == child->getId())
                {
                    // It is, mark as found
                    foundChild = true;

                    // First, clear the entry in the vector
                    it->reset();

                    // Then remove that entry from the vector efficiently
                    swapPopBack(it, &children_);
                }
                // No else.
            }

            // Leave the costs of the child out of date.
#ifdef BITSTAR_DEBUG
            if (!foundChild)
            {
                throw ompl::Exception("Attempting to remove a child vertex not present in the vector of children "
                                      "stored in the (supposed) parent vertex.");
            }
#endif  // BITSTAR_DEBUG
        }

        void BITstar::Vertex::blacklistChild(const VertexConstPtr &vertex)
        {
            childIdBlacklist_.emplace(vertex->getId());
        }

        void BITstar::Vertex::whitelistChild(const VertexConstPtr &vertex)
        {
            childIdWhitelist_.emplace(vertex->getId());
        }

        bool BITstar::Vertex::isBlacklistedAsChild(const VertexConstPtr &vertex) const
        {
            return childIdBlacklist_.find(vertex->getId()) != childIdBlacklist_.end();
        }

        bool BITstar::Vertex::isWhitelistedAsChild(const VertexConstPtr &vertex) const
        {
            return childIdWhitelist_.find(vertex->getId()) != childIdWhitelist_.end();
        }

        void BITstar::Vertex::clearBlacklist()
        {
            childIdBlacklist_.clear();
        }

        void BITstar::Vertex::clearWhitelist()
        {
            childIdWhitelist_.clear();
        }

        ompl::base::Cost BITstar::Vertex::getCost() const
        {
            ASSERT_NOT_PRUNED

            return cost_;
        }

        ompl::base::Cost BITstar::Vertex::getEdgeInCost() const
        {
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            if (!this->hasParent())
            {
                throw ompl::Exception("Attempting to access the incoming-edge cost of a vertex without a parent.");
            }
#endif  // BITSTAR_DEBUG

            return edgeCost_;
        }

        bool BITstar::Vertex::isConsistent() const
        {
            return cost_.value() == costAtExpansion_.value();
        }

        bool BITstar::Vertex::isPruned() const
        {
            return isPruned_;
        }

        bool BITstar::Vertex::isExpandedOnCurrentApproximation() const
        {
            return expansionApproximationId_ == *currentApproximationId_;
        }

        bool BITstar::Vertex::isExpandedOnCurrentSearch() const
        {
            return expansionSearchId_ == *currentSearchId_;
        }

        bool BITstar::Vertex::hasEverBeenExpandedAsRewiring() const
        {
            return hasEverBeenExpandedAsRewiring_;
        }

        void BITstar::Vertex::registerExpansion()
        {
            // Store the cost-to-come at expansion.
            costAtExpansion_ = cost_;

            // Remember the search and approximation ids.
            expansionApproximationId_ = *currentApproximationId_;
            expansionSearchId_ = *currentSearchId_;
        }

        void BITstar::Vertex::registerRewiringExpansion()
        {
            hasEverBeenExpandedAsRewiring_ = true;
        }

        void BITstar::Vertex::markPruned()
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

            isPruned_ = true;
        }

        void BITstar::Vertex::markUnpruned()
        {
            PRINT_VERTEX_CHANGE

            isPruned_ = false;
        }

        void BITstar::Vertex::insertInEdgeQueueInLookup(const SearchQueue::EdgeQueueElemPtr &element)
        {
            ASSERT_NOT_PRUNED

            // Conditionally clear any existing lookups.
            this->clearLookupsIfOutdated();

#ifdef BITSTAR_DEBUG
            // Assert that this edge is NOT _from_ this vertex.
            if (element->data.second.first->getId() == id_)
            {
                throw ompl::Exception("Attempted to add a cyclic incoming queue edge.");
            }
            // Assert that this edge is _to_ this vertex.
            if (element->data.second.second->getId() != id_)
            {
                throw ompl::Exception("Attempted to add an incoming queue edge to the wrong vertex.");
            }
            // Assert that an edge from this source does not already exist.
            for (const auto &inEdge : edgeQueueInLookup_)
            {
                if (element->data.second.first->getId() == inEdge->data.second.first->getId())
                {
                    throw ompl::Exception("Attempted to add a second edge to the queue from a single source vertex.");
                }
            }
#endif  // BITSTAR_DEBUG

            // Insert it into the lookup.
            edgeQueueInLookup_.push_back(element);
        }

        void BITstar::Vertex::removeFromEdgeQueueInLookup(const SearchQueue::EdgeQueueElemPtr &element)
        {
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            // Assert that the edge queue entries we have are of the same set as the one we're seeking to delete.
            // If so, there's no point clearing them, as then we'd be trying to remove an edge that doesn't exist which would be an error.
            if (*currentApproximationId_ != lookupApproximationId_)
            {
                throw ompl::Exception("Attempted to remove an incoming queue edge added under a different expansion id.");
            }
#endif  // BITSTAR_DEBUG

            // Variable
            // Element found
            bool found = false;

            // Iterate through the list and find the address of the element to delete
            for (auto it = edgeQueueInLookup_.begin(); it != edgeQueueInLookup_.end() && !found; ++it)
            {
                // Is it the element we're looking for? Source id
                if ((*it)->data.second.first->getId() == element->data.second.first->getId())
                {
                    // Remove by iterator
                    this->removeFromEdgeQueueInLookup(it);

                    // Mark as found
                    found = true;
                }
                // No else, try the next
            }

#ifdef BITSTAR_DEBUG
            if (!found)
            {
                throw ompl::Exception("Attempted to remove an edge not in the incoming lookup.");
            }
#endif  // BITSTAR_DEBUG
        }

        void BITstar::Vertex::removeFromEdgeQueueInLookup(const SearchQueue::EdgeQueueElemPtrVector::const_iterator& element)
        {
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            // Assert that the edge queue entries we have are of the same set as the one we're seeking to delete.
            // If so, there's no point clearing them, as then we'd be trying to remove an edge that doesn't exist which would be an error.
            if (*currentApproximationId_ != lookupApproximationId_)
            {
                throw ompl::Exception("Attempted to remove an incoming queue edge added under a different expansion id.");
            }
#endif  // BITSTAR_DEBUG

            // Remove a non-const version of the given iterator.
            // (trick from https://stackoverflow.com/a/10669041/1442500)
            this->removeFromEdgeQueueInLookup(edgeQueueInLookup_.erase(element, element));
        }

        void BITstar::Vertex::clearEdgeQueueInLookup()
        {
            ASSERT_NOT_PRUNED

            edgeQueueInLookup_.clear();
        }

        BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator BITstar::Vertex::edgeQueueInLookupConstBegin()
        {
            ASSERT_NOT_PRUNED

            // Conditionally clear any existing lookups
            this->clearLookupsIfOutdated();

            return edgeQueueInLookup_.cbegin();
        }

        BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator BITstar::Vertex::edgeQueueInLookupConstEnd()
        {
            ASSERT_NOT_PRUNED

            // Conditionally clear any existing lookups
            this->clearLookupsIfOutdated();

            return edgeQueueInLookup_.cend();
        }

        unsigned int BITstar::Vertex::edgeQueueInLookupSize()
        {
            ASSERT_NOT_PRUNED

            // Conditionally clear any existing lookups
            this->clearLookupsIfOutdated();

            return edgeQueueInLookup_.size();
        }

        void BITstar::Vertex::insertInEdgeQueueOutLookup(const SearchQueue::EdgeQueueElemPtr& element)
        {
            ASSERT_NOT_PRUNED

            // Conditionally clear any existing lookups
            this->clearLookupsIfOutdated();

#ifdef BITSTAR_DEBUG
            // Assert that this edge is _from_ this vertex
            if (element->data.second.first->getId() != id_)
            {
                throw ompl::Exception("Attempted to add an outgoing queue edge to the wrong vertex.");
            }
            // Assert that this edge is NOT _to_ this vertex
            if (element->data.second.second->getId() == id_)
            {
                throw ompl::Exception("Attempted to add a cyclic outgoing queue edge.");
            }
            // Assert that an edge to this target does not already exist
            for (const auto &outEdge : edgeQueueOutLookup_)
            {
                if (element->data.second.second->getId() == outEdge->data.second.second->getId())
                {
                    throw ompl::Exception("Attempted to add a second edge to the queue to a single target vertex.");
                }
            }
#endif  // BITSTAR_DEBUG

            // Push back
            edgeQueueOutLookup_.push_back(element);
        }

        void BITstar::Vertex::removeFromEdgeQueueOutLookup(const SearchQueue::EdgeQueueElemPtr &element)
        {
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            // Assert that the edge queue entries we have are of the same set as the one we're seeking to delete.
            // If so, there's no point clearing them, as then we'd be trying to remove an edge that doesn't exist which would be an error.
            if (*currentApproximationId_ != lookupApproximationId_)
            {
                throw ompl::Exception("Attempted to remove an incoming queue edge added under a different expansion id.");
            }
#endif  // BITSTAR_DEBUG

            // Variable
            // Element found
            bool found = false;

            // Iterate through the list and find the address of the element to delete
            for (auto it = edgeQueueOutLookup_.begin(); it != edgeQueueOutLookup_.end() && !found; ++it)
            {
                // Is it the element we're looking for? Source id
                if ((*it)->data.second.second->getId() == element->data.second.second->getId())
                {
                    // Remove by iterator
                    this->removeFromEdgeQueueOutLookup(it);

                    // Mark as found
                    found = true;
                }
                // No else, try the next
            }

#ifdef BITSTAR_DEBUG
            if (!found)
            {
                throw ompl::Exception("Attempted to remove an edge not in the outgoing lookup.");
            }
#endif  // BITSTAR_DEBUG
        }

        void BITstar::Vertex::removeFromEdgeQueueOutLookup(const SearchQueue::EdgeQueueElemPtrVector::const_iterator& element)
        {
            ASSERT_NOT_PRUNED

#ifdef BITSTAR_DEBUG
            // Assert that the edge queue entries we have are of the same set as the one we're seeking to delete.
            // If so, there's no point clearing them, as then we'd be trying to remove an edge that doesn't exist which would be an error.
            if (*currentApproximationId_ != lookupApproximationId_)
            {
                throw ompl::Exception("Attempted to remove an outgoing queue edge added under a different expansion id.");
            }
#endif  // BITSTAR_DEBUG

            // Remove a non-const version of the given iterator
            // (trick from https://stackoverflow.com/a/10669041/1442500)
            this->removeFromEdgeQueueOutLookup(edgeQueueOutLookup_.erase(element, element));
        }

        void BITstar::Vertex::clearEdgeQueueOutLookup()
        {
            ASSERT_NOT_PRUNED

            edgeQueueOutLookup_.clear();
        }

        BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator BITstar::Vertex::edgeQueueOutLookupConstBegin()
        {
            ASSERT_NOT_PRUNED

            // Make sure the lookups aren't out of date.
            this->clearLookupsIfOutdated();

            return edgeQueueOutLookup_.cbegin();
        }

        BITstar::SearchQueue::EdgeQueueElemPtrVector::const_iterator BITstar::Vertex::edgeQueueOutLookupConstEnd()
        {
            ASSERT_NOT_PRUNED

            // Make sure the lookups aren't out of date.
            this->clearLookupsIfOutdated();

            return edgeQueueOutLookup_.cend();
        }

        unsigned int BITstar::Vertex::edgeQueueOutLookupSize()
        {
            ASSERT_NOT_PRUNED

            // Make sure the lookups aren't out of date.
            this->clearLookupsIfOutdated();

            return edgeQueueOutLookup_.size();
        }

        void BITstar::Vertex::updateCostAndDepth(bool cascadeUpdates /*= true*/)
        {
            PRINT_VERTEX_CHANGE
            ASSERT_NOT_PRUNED

            if (this->isRoot())
            {
                // Am I root? -- I don't really know how this would ever be called, but ok.
                cost_ = costHelpPtr_->identityCost();
                depth_ = 0u;
            }
            else if (!this->hasParent())
            {
                // Am I disconnected?
                cost_ = costHelpPtr_->infiniteCost();

                // Set the depth to 0u, getDepth will throw in this condition
                depth_ = 0u;

#ifdef BITSTAR_DEBUG
                // Assert that I have not been asked to cascade this bad data to my children:
                if (this->hasChildren() && cascadeUpdates)
                {
                    throw ompl::Exception("Attempting to update descendants' costs and depths of a vertex that does "
                                          "not have a parent and is not root. This information would therefore be "
                                          "gibberish.");
                }
#endif  // BITSTAR_DEBUG
            }
            else
            {
                // I have a parent, so my cost is my parent cost + my edge cost to the parent
                cost_ = costHelpPtr_->combineCosts(parentPtr_->getCost(), edgeCost_);

                // If I have outgoing edges in the search queue, they need to be updated.
                for (const auto& edge : edgeQueueOutLookup_) {
                    if (lookupApproximationId_ == *currentApproximationId_) {
                        queuePtr_->update(edge);
                    }
                }

                // I am one more than my parent's depth:
                depth_ = (parentPtr_->getDepth() + 1u);
            }

            // Am I updating my children?
            if (cascadeUpdates)
            {
                // Now, iterate over my vector of children and tell each one to update its own damn cost:
                for (auto &child : children_)
                {
#ifdef BITSTAR_DEBUG
                    // Check that it hasn't expired
                    if (child.expired())
                    {
                        throw ompl::Exception("A (weak) pointer to a child has was found to have expired while "
                                              "updating the costs and depths of descendant vertices.");
                    }
#endif  // BITSTAR_DEBUG

                    // Get a lock and tell the child to update:
                    child.lock()->updateCostAndDepth(true);
                }
            }
            // No else, do not update the children. Let's hope the caller knows what they're doing.
        }

        void BITstar::Vertex::removeFromEdgeQueueInLookup(const SearchQueue::EdgeQueueElemPtrVector::iterator &element)
        {
#ifdef BITSTAR_DEBUG
            // Store the parent id of the edge we're removing.
            VertexId parentId = (*element)->data.second.first->getId();

            // Assert that this edge is not from this vertex.
            if (parentId == id_)
            {
                throw ompl::Exception("Attempted to remove a cyclic incoming queue edge.");
            }

            // Assert that this edge is to this vertex.
            if ((*element)->data.second.second->getId() != id_)
            {
                throw ompl::Exception("Attempted to remove an incoming queue edge from the wrong vertex.");
            }

            // Assert that it could exist.
            if (edgeQueueInLookup_.empty())
            {
                throw ompl::Exception("Attempted to remove an incoming queue edge from a vertex with an empty list.");
            }

            // Assert that this edge actually exists.
            bool found = false;
            for (auto it = edgeQueueInLookup_.begin(); it != edgeQueueInLookup_.end() && !found; ++it)
            {
                found = ((*it)->data.second.first->getId() == parentId);
            }
            if (!found)
            {
                throw ompl::Exception("Attempted to remove an edge not in the incoming lookup.");
            }
#endif  // BITSTAR_DEBUG

            // Clear our entry in the list
            *element = nullptr;

            // Remove it efficiently
            swapPopBack(element, &edgeQueueInLookup_);

#ifdef BITSTAR_DEBUG
            // Assert that it's now gone.
            for (const auto &edgePtr : edgeQueueInLookup_)
            {
                if (edgePtr->data.second.first->getId() == parentId)
                {
                    throw ompl::Exception("Failed to remove the designated edge in the incoming lookup.");
                }
            }
#endif  // BITSTAR_DEBUG
        }

        void BITstar::Vertex::removeFromEdgeQueueOutLookup(const SearchQueue::EdgeQueueElemPtrVector::iterator &element)
        {
#ifdef BITSTAR_DEBUG
            // Store the child id of the edge we're removing.
            VertexId childId = (*element)->data.second.second->getId();

            // Assert that this edge is this vertex.
            if ((*element)->data.second.first->getId() != id_)
            {
                throw ompl::Exception("Attempted to remove an outgoing queue edge from the wrong vertex.");
            }
            // Assert that this edge is not to this vertex.
            if (childId == id_)
            {
                throw ompl::Exception("Attempted to remove a cyclic outgoing queue edge.");
            }
            // Assert that it could exist
            if (edgeQueueOutLookup_.empty())
            {
                throw ompl::Exception("Attempted to remove an outgoing queue edge from a vertex with an empty list.");
            }
            // Assert that this edge actually exists
            bool found = false;
            for (auto it = edgeQueueOutLookup_.begin(); it != edgeQueueOutLookup_.end() && !found; ++it)
            {
                found = ((*it)->data.second.second->getId() == childId);
            }
            if (!found)
            {
                throw ompl::Exception("Attempted to remove an edge not in the outgoing lookup.");
            }
#endif  // BITSTAR_DEBUG

            // Clear our entry in the list.
            *element = nullptr;

            // Remove it efficiently.
            swapPopBack(element, &edgeQueueOutLookup_);

#ifdef BITSTAR_DEBUG
            // Assert that it's now gone.
            for (const auto &edgePtr : edgeQueueOutLookup_)
            {
                if (edgePtr->data.second.second->getId() == childId)
                {
                    throw ompl::Exception("Failed to remove the designated edge in the outgoing lookup.");
                }
            }
#endif  // BITSTAR_DEBUG
        }

        void BITstar::Vertex::clearLookupsIfOutdated()
        {
            // Clean up any old lookups.
            if (lookupApproximationId_ != *currentApproximationId_)
            {
                // Clear the existing entries.
                this->clearEdgeQueueInLookup();
                this->clearEdgeQueueOutLookup();

                // Update the counter.
                lookupApproximationId_ = *currentApproximationId_;
            }
            // No else, this is the same pass through the vertex queue.
        }
    }  // geometric
}  // ompl
