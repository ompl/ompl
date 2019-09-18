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

#include "ompl/geometric/planners/aibitstar/Vertex.h"

#include <limits>

namespace ompl
{
    namespace geometric
    {
        namespace aibitstar
        {
            namespace
            {
                std::size_t generateId()
                {
                    static std::size_t id{0u};
                    return id++;
                }
            }  // namespace

            Vertex::Vertex(const std::shared_ptr<State> &state) : id_(generateId()), state_(state)
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

            std::shared_ptr<State> Vertex::getState() const
            {
                return state_;
            }

            const std::vector<std::shared_ptr<Vertex>> &Vertex::getChildren() const
            {
                return children_;
            }

            void Vertex::addChild(const std::shared_ptr<Vertex> &vertex)
            {
                children_.emplace_back(vertex);
            }

            void Vertex::removeChild(const std::shared_ptr<Vertex> &vertex)
            {
                // Find the child that is to be removed.
                auto it = std::find_if(children_.begin(), children_.end(),
                                       [&vertex](const auto &child) { return child->getId() == vertex->getId(); });

                // If the provided vertex is not a child, this is a bug.
                assert(it != children_.end());

                // Do the good ol' swap and pop.
                std::iter_swap(it, children_.rbegin());
                children_.pop_back();
            }

            std::weak_ptr<Vertex> Vertex::getParent() const
            {
                return parent_;
            }

            void Vertex::setParent(const std::shared_ptr<Vertex> &vertex)
            {
                parent_ = vertex;
            }

        }  // namespace aibitstar

    }  // namespace geometric

}  // namespace ompl
