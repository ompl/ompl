/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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

// Authors: Marlin Strub

#include "ompl/geometric/planners/aeitstar/Vertex.h"

#include <algorithm>
#include <limits>

namespace ompl
{
    namespace geometric
    {
        namespace aeitstar
        {
            namespace
            {
                std::size_t generateId()
                {
                    static std::size_t id{0u};
                    return id++;
                }
            }  // namespace

            Vertex::Vertex(const std::shared_ptr<State> &state,
                           const std::shared_ptr<ompl::base::OptimizationObjective> &objective)
              : id_(generateId())
              , cost_(objective->infiniteCost())
              , edgeCost_(objective->infiniteCost())
              , extendCost_(objective->infiniteCost())
              , state_(state)
            {
            }

            std::size_t Vertex::getId() const
            {
                return id_;
            }

            ompl::base::Cost Vertex::getCost() const
            {
                return cost_;
            }

            void Vertex::setCost(const ompl::base::Cost &cost)
            {
                cost_ = cost;
            }

            ompl::base::Cost Vertex::getExtendedCost() const
            {
                return extendCost_;
            }

            void Vertex::setExtendedCost(const ompl::base::Cost &cost)
            {
                extendCost_ = cost;
            }

            std::shared_ptr<State> Vertex::getState() const
            {
                return state_;
            }

            const std::vector<std::shared_ptr<Vertex>> &Vertex::getChildren() const
            {
                return children_;
            }

            bool Vertex::hasChildren() const
            {
                return !children_.empty();
            }

            std::vector<std::shared_ptr<Vertex>>
            Vertex::updateChildren(const std::shared_ptr<ompl::base::OptimizationObjective> &objective)
            {
                std::vector<std::shared_ptr<Vertex>> accumulatedChildren = children_;
                for (auto &child : children_)
                {
                    child->updateCost(objective);
                    auto childsAccumulatedChildren = child->updateChildren(objective);
                    accumulatedChildren.insert(accumulatedChildren.end(), childsAccumulatedChildren.begin(),
                                               childsAccumulatedChildren.end());
                }

                return accumulatedChildren;
            }

            void Vertex::addChild(const std::shared_ptr<Vertex> &vertex)
            {
                assert(this);
                assert(vertex);
                assert((!parent_.lock()) || (parent_.lock() && parent_.lock()->getId() != vertex->getId()));
                children_.emplace_back(vertex);
            }

            void Vertex::removeChild(const std::shared_ptr<Vertex> &vertex)
            {
                assert(this);
                assert(vertex);

                // Find the child that is to be removed.
                auto it = std::find_if(children_.begin(), children_.end(),
                                       [&vertex](const auto &child) { return child->getId() == vertex->getId(); });

                // If the provided vertex is not a child, this is a bug.
                assert(it != children_.end());

                // Do the good ol' swap and pop.
                std::iter_swap(it, children_.rbegin());
                children_.pop_back();
            }

            void Vertex::removeIfChild(const std::shared_ptr<Vertex> &vertex)
            {
                assert(this);
                assert(vertex);

                // Find the child that is to be removed.
                auto it = std::find_if(children_.begin(), children_.end(),
                                       [&vertex](const auto &child) { return child->getId() == vertex->getId(); });

                // If the provided vertex is not a child, this is a bug.
                if (it != children_.end())
                {
                    // Do the good ol' swap and pop.
                    std::iter_swap(it, children_.rbegin());
                    children_.pop_back();
                }
            }

            std::weak_ptr<Vertex> Vertex::getParent() const
            {
                return parent_;
            }

            std::weak_ptr<Vertex> Vertex::getTwin() const
            {
                return twin_;
            }

            void Vertex::setEdgeCost(const ompl::base::Cost &edgeCost)
            {
                edgeCost_ = edgeCost;
            }

            void Vertex::updateParent(const std::shared_ptr<Vertex> &vertex)
            {
                assert(std::find_if(children_.begin(), children_.end(), [&vertex](const auto &child) {
                           return vertex->getId() == child->getId();
                       }) == children_.end());
                if (auto parent = parent_.lock())
                {
                    parent->removeChild(shared_from_this());
                }
                parent_ = vertex;
            }

            void Vertex::updateCost(const std::shared_ptr<ompl::base::OptimizationObjective> &objective)
            {
                assert(parent_.lock());
                cost_ = objective->combineCosts(parent_.lock()->getCost(), edgeCost_);
            }

            void Vertex::resetParent()
            {
                parent_.reset();
            }

            void Vertex::setTwin(const std::shared_ptr<Vertex> &vertex)
            {
                twin_ = vertex;
            }

            void Vertex::clearChildren()
            {
                children_.clear();
            }

            std::size_t Vertex::getExpandTag() const
            {
                return expandTag_;
            }

            void Vertex::setExpandTag(std::size_t expandTag)
            {
                expandTag_ = expandTag;
            }

        }  // namespace aeitstar

    }  // namespace geometric

}  // namespace ompl
