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

#include "ompl/geometric/planners/informedtrees/eitstar/State.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            namespace
            {
                std::size_t generateId()
                {
                    static std::size_t id{0u};
                    return id++;
                }
            }  // namespace

            State::State(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo,
                         const std::shared_ptr<ompl::base::OptimizationObjective> &objective)
              : id_(generateId())
              , estimatedCostToGo_(objective->infiniteCost())
              , lowerBoundCostToGo_(objective->infiniteCost())
              , lowerBoundCostToCome_(objective->infiniteCost())
              , state_(spaceInfo->allocState())
              , neighbors_({0u, {}})
              , spaceInfo_(spaceInfo)
              , objective_(objective)
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
                else  // If this state does not yet exist as a forward vertex, create it.
                {
                    // Create a forward vertex from this state.
                    forwardVertex = std::make_shared<Vertex>(shared_from_this(), objective_, Direction::FORWARD);

                    // If this state has a reverse vertex, the newly created vertex is its twin.
                    if (auto reverseVertex = reverseVertex_.lock())
                    {
                        reverseVertex->setTwin(forwardVertex);
                        forwardVertex->setTwin(reverseVertex);
                    }

                    // Remember this state's forward vertex and return.
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
                else  // If this state does not yet exist as a reverse vertex, create it.
                {
                    // Create a reverse vertex from this state.
                    reverseVertex = std::make_shared<Vertex>(shared_from_this(), objective_, Direction::REVERSE);

                    // If this state has a forward vertex, the newly created vertex is its twin.
                    if (auto forwardVertex = forwardVertex_.lock())
                    {
                        forwardVertex->setTwin(reverseVertex);
                        reverseVertex->setTwin(forwardVertex);
                    }

                    // Remember this state's reverse vertex and return.
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

            void State::setEstimatedEffortToGo(std::size_t effort)
            {
                estimatedEffortToGo_ = effort;
            }

            void State::setEstimatedCostToGo(ompl::base::Cost cost)
            {
                estimatedCostToGo_ = cost;
            }

            void State::setAdmissibleCostToGo(ompl::base::Cost cost)
            {
                admissibleCostToGo_ = cost;
            }

            void State::setLowerBoundCostToGo(ompl::base::Cost cost)
            {
                lowerBoundCostToGo_ = cost;
            }

            void State::setLowerBoundCostToCome(ompl::base::Cost cost)
            {
                lowerBoundCostToCome_ = cost;
            }

            void State::setCurrentCostToCome(ompl::base::Cost cost)
            {
                currentCostToCome_ = cost;
            }

            void State::setLowerBoundEffortToCome(unsigned int effort)
            {
                lowerBoundEffortToCome_ = effort;
            }

            void State::setInadmissibleEffortToCome(unsigned int effort)
            {
                inadmissibleEffortToCome_ = effort;
            }

            unsigned int State::getEstimatedEffortToGo() const
            {
                return estimatedEffortToGo_;
            }

            ompl::base::Cost State::getEstimatedCostToGo() const
            {
                return estimatedCostToGo_;
            }

            ompl::base::Cost State::getAdmissibleCostToGo() const
            {
                return admissibleCostToGo_;
            }

            ompl::base::Cost State::getLowerBoundCostToGo() const
            {
                return lowerBoundCostToGo_;
            }

            ompl::base::Cost State::getLowerBoundCostToCome() const
            {
                return lowerBoundCostToCome_;
            }

            ompl::base::Cost State::getCurrentCostToCome() const
            {
                return currentCostToCome_;
            }

            unsigned int State::getLowerBoundEffortToCome() const
            {
                return lowerBoundEffortToCome_;
            }

            unsigned int State::getInadmissibleEffortToCome() const
            {
                return inadmissibleEffortToCome_;
            }

            const std::vector<std::weak_ptr<State>> State::getSourcesOfIncomingEdgesInForwardQueue() const
            {
                return sourcesOfIncomingEdgesInForwardQueue_;
            }

            void State::addToSourcesOfIncomingEdgesInForwardQueue(const std::shared_ptr<State> &state) const
            {
                sourcesOfIncomingEdgesInForwardQueue_.emplace_back(state);
            }

            void State::removeFromSourcesOfIncomingEdgesInForwardQueue(const std::shared_ptr<State> &state) const
            {
                const auto iter = std::find_if(
                    sourcesOfIncomingEdgesInForwardQueue_.begin(), sourcesOfIncomingEdgesInForwardQueue_.end(),
                    [&state](const auto &source) { return state->getId() == source.lock()->getId(); });

                if (iter != sourcesOfIncomingEdgesInForwardQueue_.end())
                {
                    sourcesOfIncomingEdgesInForwardQueue_.erase(iter);
                }
                else
                {
                    throw std::out_of_range("Unable to remove source from incoming edges in forward queue.");
                }
            }

            void State::resetSourcesOfIncomingEdgesInForwardQueue()
            {
                sourcesOfIncomingEdgesInForwardQueue_.clear();
            }

            void State::setIncomingCollisionCheckResolution(const std::shared_ptr<State> &source,
                                                            std::size_t numChecks) const
            {
                incomingCollisionCheckResolution_[source->getId()] = numChecks;
            }

            std::size_t State::getIncomingCollisionCheckResolution(const std::shared_ptr<State> &source) const
            {
                if (incomingCollisionCheckResolution_.find(source->getId()) == incomingCollisionCheckResolution_.end())
                {
                    return 0u;
                }
                else
                {
                    return incomingCollisionCheckResolution_[source->getId()];
                }
            }

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl
