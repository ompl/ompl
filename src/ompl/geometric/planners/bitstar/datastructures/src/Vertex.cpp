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
#include <utility>

#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
// The ID generator class, this is actually included via Vertex.h->BITstar.h, but to be clear.
#include "ompl/geometric/planners/bitstar/datastructures/IdGenerator.h"

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::Vertex::Vertex(ompl::base::SpaceInformationPtr si, ompl::base::OptimizationObjectivePtr opt,
                                bool root /*= false*/)
          : vId_(getIdGenerator().getNewId())
          , si_(std::move(si))
          , opt_(std::move(opt))
          , state_(si_->allocState())
          , isRoot_(root)
          , isNew_(true)
          , hasBeenExpandedToSamples_(false)
          , hasBeenExpandedToVertices_(false)
          , isPruned_(false)
          , depth_(0u)
          , parentSPtr_(VertexPtr())
          , edgeCost_(opt_->infiniteCost())
          , cost_(opt_->infiniteCost())
          , childWPtrs_()
        {
            if (this->isRoot() == true)
            {
                cost_ = opt_->identityCost();
            }
            // No else, infinite by default
        }

        BITstar::Vertex::~Vertex()
        {
            // Free the state on destruction
            si_->freeState(state_);
        }

        BITstar::VertexId BITstar::Vertex::getId() const
        {
            this->assertNotPruned();
            return vId_;
        }

        ompl::base::OptimizationObjectivePtr BITstar::Vertex::getOpt() const
        {
            this->assertNotPruned();

            return opt_;
        }

        ompl::base::State const *BITstar::Vertex::stateConst() const
        {
            this->assertNotPruned();

            return state_;
        }

        ompl::base::State *BITstar::Vertex::state()
        {
            this->assertNotPruned();

            return state_;
        }

        bool BITstar::Vertex::isRoot() const
        {
            this->assertNotPruned();

            return isRoot_;
        }

        bool BITstar::Vertex::hasParent() const
        {
            this->assertNotPruned();

            return static_cast<bool>(parentSPtr_);
        }

        bool BITstar::Vertex::isInTree() const
        {
            // No need to assert, as the two other functions both do

            return this->isRoot() || this->hasParent();
        }

        unsigned int BITstar::Vertex::getDepth() const
        {
            this->assertNotPruned();

            if (this->isRoot() == false && this->hasParent() == false)
            {
                throw ompl::Exception("Attempting to get the depth of a vertex that does not have a parent yet is not "
                                      "root.");
            }

            return depth_;
        }

        BITstar::VertexConstPtr BITstar::Vertex::getParentConst() const
        {
            this->assertNotPruned();

            if (this->hasParent() == false)
            {
                if (this->isRoot() == true)
                {
                    throw ompl::Exception("Attempting to access the parent of the root vertex.");
                }
                else
                {
                    throw ompl::Exception("Attempting to access the parent of a vertex that does not have one.");
                }
            }

            return parentSPtr_;
        }

        BITstar::VertexPtr BITstar::Vertex::getParent()
        {
            this->assertNotPruned();

            if (this->hasParent() == false)
            {
                if (this->isRoot() == true)
                {
                    throw ompl::Exception("Attempting to access the parent of the root vertex.");
                }
                else
                {
                    throw ompl::Exception("Attempting to access the parent of a vertex that does not have one.");
                }
            }

            return parentSPtr_;
        }

        void BITstar::Vertex::addParent(const VertexPtr &newParent, const ompl::base::Cost &edgeInCost,
                                        bool updateChildCosts /*= true*/)
        {
            this->assertNotPruned();

            if (this->hasParent() == true)
            {
                throw ompl::Exception("Attempting to add a parent to a vertex that already has one.");
            }
            else if (this->isRoot() == true)
            {
                throw ompl::Exception("Attempting to add a parent to the root vertex, which cannot have a parent.");
            }
            // No else.

            // Store the parent
            parentSPtr_ = newParent;

            // Store the edge cost
            edgeCost_ = edgeInCost;

            // Update my cost
            this->updateCostAndDepth(updateChildCosts);
        }

        void BITstar::Vertex::removeParent(bool updateChildCosts /*= true*/)
        {
            this->assertNotPruned();

            if (this->hasParent() == false)
            {
                throw ompl::Exception("Attempting to remove the parent of a vertex that does not have a parent.");
            }
            else if (this->isRoot() == true)
            {
                throw ompl::Exception("Attempting to remove the parent of the root vertex, which cannot have a "
                                      "parent.");
            }

            // Clear my parent
            parentSPtr_.reset();

            // Update costs:
            this->updateCostAndDepth(updateChildCosts);
        }

        bool BITstar::Vertex::hasChildren() const
        {
            this->assertNotPruned();

            return !childWPtrs_.empty();
        }

        void BITstar::Vertex::getChildrenConst(VertexConstPtrVector *children) const
        {
            this->assertNotPruned();

            children->clear();

            for (const auto &childWPtr : childWPtrs_)
            {
                // Check that the weak pointer hasn't expired
                if (childWPtr.expired() == true)
                {
                    throw ompl::Exception("A (weak) pointer to a child was found to have expired while calculating the "
                                          "children of a vertex.");
                }
                else
                {
                    children->push_back(childWPtr.lock());
                }
            }
        }

        void BITstar::Vertex::getChildren(VertexPtrVector *children)
        {
            this->assertNotPruned();

            children->clear();

            for (const auto &childWPtr : childWPtrs_)
            {
                // Check that the weak pointer hasn't expired
                if (childWPtr.expired() == true)
                {
                    throw ompl::Exception("A (weak) pointer to a child was found to have expired while calculating the "
                                          "children of a vertex.");
                }
                else
                {
                    children->push_back(childWPtr.lock());
                }
            }
        }

        void BITstar::Vertex::addChild(const VertexPtr &newChild, bool updateChildCosts /*= true*/)
        {
            this->assertNotPruned();

            // Push back the shared_ptr into the vector of weak_ptrs, this makes a weak_ptr copy
            childWPtrs_.push_back(newChild);

            if (updateChildCosts == true)
            {
                newChild->updateCostAndDepth(true);
            }
            // No else, leave the costs out of date.
        }

        void BITstar::Vertex::removeChild(const VertexPtr &oldChild, bool updateChildCosts /*= true*/)
        {
            this->assertNotPruned();

            // Variables
            // Whether the child has been found (and then deleted);
            bool foundChild;

            // Iterate over the list of children pointers until the child is found. Iterators make erase easier
            foundChild = false;
            for (auto childIter = childWPtrs_.begin(); childIter != childWPtrs_.end() && foundChild == false;
                 ++childIter)
            {
                // Check that the weak pointer hasn't expired
                if (childIter->expired() == true)
                {
                    throw ompl::Exception("A (weak) pointer to a child was found to have expired while removing a "
                                          "child from a vertex.");
                }
                // No else, weak pointer is valid

                // Check if this is the child we're looking for
                if (childIter->lock()->getId() == oldChild->getId())
                {
                    // Remove the child from the vector
                    childWPtrs_.erase(childIter);

                    // Mark as found
                    foundChild = true;

                    // Update the child cost if appropriate
                    if (updateChildCosts == true)
                    {
                        oldChild->updateCostAndDepth(true);
                    }
                    // No else, leave the costs out of date.
                }
                // No else, move on
            }

            // Throw if we did not find the child
            if (foundChild == false)
            {
                throw ompl::Exception("Attempting to remove a child vertex not present in the list of children stored "
                                      "in the (supposed) parent vertex.");
            }
            // No else, we were successful
        }

        ompl::base::Cost BITstar::Vertex::getCost() const
        {
            this->assertNotPruned();

            return cost_;
        }

        ompl::base::Cost BITstar::Vertex::getEdgeInCost() const
        {
            this->assertNotPruned();

            if (this->hasParent() == false)
            {
                throw ompl::Exception("Attempting to access the incoming-edge cost of a vertex without a parent.");
            }

            return edgeCost_;
        }

        bool BITstar::Vertex::isNew() const
        {
            this->assertNotPruned();

            return isNew_;
        }

        void BITstar::Vertex::markNew()
        {
            this->assertNotPruned();

            isNew_ = true;
        }

        void BITstar::Vertex::markOld()
        {
            this->assertNotPruned();

            isNew_ = false;
        }

        bool BITstar::Vertex::hasBeenExpandedToSamples() const
        {
            this->assertNotPruned();

            return hasBeenExpandedToSamples_;
        }

        void BITstar::Vertex::markExpandedToSamples()
        {
            this->assertNotPruned();

            hasBeenExpandedToSamples_ = true;
        }

        void BITstar::Vertex::markUnexpandedToSamples()
        {
            this->assertNotPruned();

            hasBeenExpandedToSamples_ = false;
        }

        bool BITstar::Vertex::hasBeenExpandedToVertices() const
        {
            this->assertNotPruned();

            return hasBeenExpandedToVertices_;
        }

        void BITstar::Vertex::markExpandedToVertices()
        {
            this->assertNotPruned();

            hasBeenExpandedToVertices_ = true;
        }

        void BITstar::Vertex::markUnexpandedToVertices()
        {
            this->assertNotPruned();

            hasBeenExpandedToVertices_ = false;
        }

        bool BITstar::Vertex::isPruned() const
        {
            return isPruned_;
        }

        void BITstar::Vertex::markPruned()
        {
            this->assertNotPruned();

            isPruned_ = true;
        }

        void BITstar::Vertex::markUnpruned()
        {
            isPruned_ = false;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Protected functions:
        void BITstar::Vertex::updateCostAndDepth(bool cascadeUpdates /*= true*/)
        {
            this->assertNotPruned();

            if (this->isRoot() == true)
            {
                // Am I root? -- I don't really know how this would ever be called, but ok.
                cost_ = opt_->identityCost();
                depth_ = 0u;
            }
            else if (this->hasParent() == false)
            {
                // Am I disconnected?
                cost_ = opt_->infiniteCost();

                // Set the depth to 0u, getDepth will throw in this condition
                depth_ = 0u;

                // Assert that I have not been asked to cascade this bad data to my children:
                if (this->hasChildren() == true && cascadeUpdates == true)
                {
                    throw ompl::Exception("Attempting to update descendants' costs and depths of a vertex that does "
                                          "not have a parent and is not root. This information would therefore be "
                                          "gibberish.");
                }
            }
            else
            {
                // I have a parent, so my cost is my parent cost + my edge cost to the parent
                cost_ = opt_->combineCosts(parentSPtr_->getCost(), edgeCost_);

                // I am one more than my parent's depth:
                depth_ = (parentSPtr_->getDepth() + 1u);
            }

            // Am I updating my children?
            if (cascadeUpdates == true)
            {
                // Now, iterate over my list of children and tell each one to update its own damn cost:
                for (auto &childWPtr : childWPtrs_)
                {
                    // Check that it hasn't expired
                    if (childWPtr.expired() == true)
                    {
                        throw ompl::Exception("A (weak) pointer to a child has was found to have expired while "
                                              "updating the costs and depths of descendant vertices.");
                    }
                    // No else, weak pointer is valid

                    // Get a lock and tell the child to update:
                    childWPtr.lock()->updateCostAndDepth(true);
                }
            }
            // No else, do not update the children. I hope the caller knows what they're doing.
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Private functions:
        void BITstar::Vertex::assertNotPruned() const
        {
            if (isPruned_ == true)
            {
                std::cout << std::endl
                          << "vId: " << vId_ << std::endl;
                throw ompl::Exception("Attempting to access a pruned vertex.");
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    }  // geometric
}  // ompl
