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

#include "ompl/geometric/planners/aibitstar/State.h"

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

            State::State(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo)
              : id_(generateId()), state_(spaceInfo->allocState()), neighbors_({0u, {}}), spaceInfo_(spaceInfo)
            {
                if (state_ == nullptr)
                {
                    OMPL_ERROR("Could not allocate state.");
                    throw std::bad_alloc();
                }
            }

            State::~State()
            {
                spaceInfo_->freeState(state_);
            }

            std::size_t State::getId() const
            {
                return id_;
            }

            ompl::base::State *State::raw() const
            {
                return state_;
            }

            bool State::hasForwardVertex() const
            {
                return static_cast<bool>(forwardVertex_.lock());
            }

            bool State::hasReverseVertex() const
            {
                return static_cast<bool>(reverseVertex_.lock());
            }

            std::shared_ptr<Vertex> State::asForwardVertex()
            {
                // If this state exists as a forward vertex, return this object.
                if (auto forwardVertex = forwardVertex_.lock())
                {
                    return forwardVertex;
                }
                else  // If this state does not yet exist as a forward vertex, create the object.
                {
                    forwardVertex = std::make_shared<Vertex>(shared_from_this());
                    if (auto reverseVertex = reverseVertex_.lock())
                    {
                        reverseVertex->setTwin(forwardVertex);
                        forwardVertex->setTwin(reverseVertex);
                    }
                    forwardVertex_ = forwardVertex;
                    return forwardVertex;
                }
            }

            std::shared_ptr<Vertex> State::asReverseVertex()
            {
                // If this state exists as a reverse vertex, return this object.
                if (auto reverseVertex = reverseVertex_.lock())
                {
                    return reverseVertex;
                }
                else  // If this state does not yet exist as a reverse vertex, create the object.
                {
                    reverseVertex = std::make_shared<Vertex>(shared_from_this());
                    if (auto forwardVertex = forwardVertex_.lock())
                    {
                        forwardVertex->setTwin(reverseVertex);
                        reverseVertex->setTwin(forwardVertex);
                    }
                    reverseVertex_ = reverseVertex;
                    return reverseVertex;
                }
            }

            void State::blacklist(const std::shared_ptr<State> &state)
            {
                blacklist_.insert(state->getId());
            }

            void State::whitelist(const std::shared_ptr<State> &state)
            {
                whitelist_.insert(state->getId());
            }

            bool State::isBlacklisted(const std::shared_ptr<State> &state) const
            {
                return blacklist_.find(state->getId()) != blacklist_.end();
            }

            bool State::isWhitelisted(const std::shared_ptr<State> &state) const
            {
                return whitelist_.find(state->getId()) != whitelist_.end();
            }

        }  // namespace aibitstar

    }  // namespace geometric

}  // namespace ompl
