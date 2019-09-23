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

            ompl::base::State *State::getState() const
            {
                return state_;
            }

            std::weak_ptr<Vertex> State::getForwardVertex() const
            {
                return forwardVertex_;
            }

            std::weak_ptr<Vertex> State::getReverseVertex() const
            {
                return reverseVertex_;
            }

            void State::setForwardVertex(const std::weak_ptr<Vertex> &vertex)
            {
                forwardVertex_ = vertex;
            }

            void State::setReverseVertex(const std::weak_ptr<Vertex> &vertex)
            {
                reverseVertex_ = vertex;
            }

            void State::resetForwardVertex()
            {
                forwardVertex_.reset();
            }

            void State::resetReverseVertex()
            {
                reverseVertex_.reset();
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
