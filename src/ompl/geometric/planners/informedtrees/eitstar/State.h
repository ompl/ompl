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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_STATE_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_STATE_

#include <memory>
#include <set>
#include <utility>

#include "ompl/base/Cost.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"

#include "ompl/geometric/planners/informedtrees/eitstar/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            /** \brief A wrapper class for OMPL's state. */
            class State : public std::enable_shared_from_this<State>  // Inheritance must be public here.
            {
            public:
                /** \brief Constructs the state, allocating the associated memory using information about the underlying
                 * space. */
                State(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo,
                      const std::shared_ptr<ompl::base::OptimizationObjective> &objective);

                /** \brief Destructs this state, freeing the associated memory. */
                ~State();

                /** \brief Returns the state's unique id. */
                std::size_t getId() const;

                /** \brief Returns the raw OMPL version of this state. */
                ompl::base::State *raw() const;

                /** \brief Returns whether the state has an associated forward vertex. */
                bool hasForwardVertex() const;

                /** \brief Returns whether the state has an associated reverse vertex. */
                bool hasReverseVertex() const;

                /** \brief Returns the state as a reverse vertex. */
                std::shared_ptr<Vertex> asForwardVertex();

                /** \brief Returns the state as a reverse vertex. */
                std::shared_ptr<Vertex> asReverseVertex();

                /** \brief Blacklists the given state as neighbor. */
                void blacklist(const std::shared_ptr<State> &state);

                /** \brief Whitelists the given state as neighbor. */
                void whitelist(const std::shared_ptr<State> &state);

                /** \brief Returns whether the given state has been blacklisted. */
                bool isBlacklisted(const std::shared_ptr<State> &state) const;

                /** \brief Returns whether the given has been whitelisted. */
                bool isWhitelisted(const std::shared_ptr<State> &state) const;

                /** \brief Set the estimated effort (number of collision detections) to go from this state to the goal
                 * through the current RGG. */
                void setEstimatedEffortToGo(std::size_t effort);

                /** \brief Set the best estimate of the cost to go from this state to the goal through the current RGG.
                 */
                void setEstimatedCostToGo(ompl::base::Cost cost);

                /** \brief Set the admissible estimate of the cost to go from this state to the goal through the current
                 * RGG. */
                void setAdmissibleCostToGo(ompl::base::Cost cost);

                /** \brief Set the lower bound cost to go from this state to the goal through the continuous state
                 * space. */
                void setLowerBoundCostToGo(ompl::base::Cost cost);

                /** \brief Set the lower bound cost to come from the start to this state through the continuous state
                 * space. */
                void setLowerBoundCostToCome(ompl::base::Cost cost);

                /** \brief Set the current cost to come from the start to this state. */
                void setCurrentCostToCome(ompl::base::Cost cost);

                /** \brief Sets the lower bound effort to come to this state through the continuous state space. */
                void setLowerBoundEffortToCome(unsigned int effort);

                /** \brief Sets the inadmissible effort to come to this state through the continuous state space. */
                void setInadmissibleEffortToCome(unsigned int effort);

                /** \brief Get the estimated effort (number of collision detections) to go from this state to the goal
                 * through the current RGG. */
                unsigned int getEstimatedEffortToGo() const;

                /** \brief Returns the best estimate of the cost to go from this state to the goal through the current
                 * RGG. */
                ompl::base::Cost getEstimatedCostToGo() const;

                /** \brief Returns the admissible estimate of the cost to go from this state to the goal through the
                 * current RGG. */
                ompl::base::Cost getAdmissibleCostToGo() const;

                /** \brief Returns the lower bound cost to go from this state to the goal through the continuous state
                 * space. */
                ompl::base::Cost getLowerBoundCostToGo() const;

                /** \brief Returns the lower bound cost to come from the start to this state through the continuous
                 * state space. */
                ompl::base::Cost getLowerBoundCostToCome() const;

                /** \brief Returns the current cost to come from the start to this state. */
                ompl::base::Cost getCurrentCostToCome() const;

                /** \brief Returns the lower bound effort to come from the start to this state through the continuous
                 * state space. */
                unsigned int getLowerBoundEffortToCome() const;

                /** \brief Returns the inadmissible effort to come from the start to this state through the continuous
                 * state space. */
                unsigned int getInadmissibleEffortToCome() const;

                /** \brief Returns the sources of incoming edges in forward queue. */
                const std::vector<std::weak_ptr<State>> getSourcesOfIncomingEdgesInForwardQueue() const;

                /** \brief Adds a source to sources of incoming edges in forward queue. */
                void addToSourcesOfIncomingEdgesInForwardQueue(const std::shared_ptr<State> &state) const;

                /** \brief Removes a source from sources of incoming edges in forward queue. */
                void removeFromSourcesOfIncomingEdgesInForwardQueue(const std::shared_ptr<State> &state) const;

                /** \brief Resets the sources of incoming edges in the forward queue. */
                void resetSourcesOfIncomingEdgesInForwardQueue();

                /** \brief Sets the number of collision checks already performed on the edge incoming from source. */
                void setIncomingCollisionCheckResolution(const std::shared_ptr<State> &source,
                                                         std::size_t numChecks) const;

                /** \brief Returns the number of collision checks already performed on the edge incoming from source. */
                std::size_t getIncomingCollisionCheckResolution(const std::shared_ptr<State> &source) const;

            private:
                /** \brief Grant access to the state internals to the random geometric graph. */
                friend class RandomGeometricGraph;

                /** \brief The unique id of this state. */
                const std::size_t id_;

                /** \brief The estimated effort (number of collision detections) to go from this state to the goal
                 * through the current graph. */
                unsigned int estimatedEffortToGo_{std::numeric_limits<unsigned int>::max()};

                /** \brief A potentially inadmissible estimate of the cost to go from this state to the goal through the
                 * current RGG. */
                ompl::base::Cost estimatedCostToGo_;

                /** \brief A lower bound on the effort to come. */
                unsigned int lowerBoundEffortToCome_{std::numeric_limits<unsigned int>::max()};

                /** \brief An inadmissible estimate of the effort to come. */
                unsigned int inadmissibleEffortToCome_{std::numeric_limits<unsigned int>::max()};

                /** \brief An admissible estimate of the cost to go from this state to the goal through the current RGG.
                 */
                ompl::base::Cost admissibleCostToGo_;

                /** \brief A lower bound on the cost to go from this state to the goal through the continuous state
                 * space. */
                ompl::base::Cost lowerBoundCostToGo_;

                /** \brief A lower bound on the cost to come from the start to this state through the continuous state
                 * space. */
                ompl::base::Cost lowerBoundCostToCome_;

                /** \brief The current cost to come from the start to this state. */
                ompl::base::Cost currentCostToCome_;

                /** \brief The underlying OMPL state. */
                ompl::base::State *state_;

                /** \brief The associated vertex in the forward search tree. */
                std::weak_ptr<Vertex> forwardVertex_{};

                /** \brief The associated vertex in the forward search tree. */
                std::weak_ptr<Vertex> reverseVertex_{};

                /** \brief The neighbors in the graph, associated with a graph tag. */
                mutable std::pair<std::size_t, std::vector<std::weak_ptr<State>>> neighbors_{};

                /** \brief The blacklist of states that can not be connected to this state. */
                std::set<std::size_t> blacklist_{};  // Maybe this would be faster as vector?

                /** \brief The whitelist of states that can be connected to this state. */
                std::set<std::size_t> whitelist_{};  // Maybe this would be faster as vector?

                /** \brief The source states that for which this state is a target in the forward queue. */
                mutable std::vector<std::weak_ptr<State>> sourcesOfIncomingEdgesInForwardQueue_{};

                /** \brief A map that holds the checked collision checking resolution of the incoming edges.
                 The key is the id of the source state and the value is the number of checks already performed on that
                 edge. */
                mutable std::map<std::size_t, std::size_t> incomingCollisionCheckResolution_{};

                /** \brief The info on the state space this state lives in. */
                std::shared_ptr<ompl::base::SpaceInformation> spaceInfo_;

                /** \brief The optimization objective. */
                std::shared_ptr<ompl::base::OptimizationObjective> objective_;
            };

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_STATE_
