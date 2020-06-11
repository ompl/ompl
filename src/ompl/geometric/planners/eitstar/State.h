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

#ifndef OMPL_GEOMETRIC_PLANNERS_EITSTAR_STATE_
#define OMPL_GEOMETRIC_PLANNERS_EITSTAR_STATE_

#include <memory>
#include <set>
#include <utility>

#include "ompl/base/Cost.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"

#include "ompl/geometric/planners/eitstar/Vertex.h"

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

                /** \brief Destructs the state, freeing the associated memory. */
                ~State();

                /** \brief Get the state's unique id. */
                std::size_t getId() const;

                /** \brief Get the raw OMPL version of this state. */
                ompl::base::State *raw() const;

                /** \brief Returns whether the state has an associated forward vertex. */
                bool hasForwardVertex() const;

                /** \brief Returns whether the state has an associated reverse vertex. */
                bool hasReverseVertex() const;

                /** \brief Get the state as a reverse vertex. */
                std::shared_ptr<Vertex> asForwardVertex();

                /** \brief Get the state as a reverse vertex(); */
                std::shared_ptr<Vertex> asReverseVertex();

                /** \brief Blacklist a state as neighbor. */
                void blacklist(const std::shared_ptr<State> &state);

                /** \brief Whitelist a state as neighbor. */
                void whitelist(const std::shared_ptr<State> &state);

                /** \brief Returns whether a state has been blacklisted. */
                bool isBlacklisted(const std::shared_ptr<State> &state) const;

                /** \brief Returns whether a state has been whitelisted. */
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

                /** \brief Get the estimated effort (number of collision detections) to go from this state to the goal
                 * through the current RGG. */
                std::size_t getEstimatedEffortToGo() const;

                /** \brief Get the best estimate of the cost to go from this state to the goal through the current RGG.
                 */
                ompl::base::Cost getEstimatedCostToGo() const;

                /** \brief Get the admissible estimate of the cost to go from this state to the goal through the current
                 * RGG. */
                ompl::base::Cost getAdmissibleCostToGo() const;

                /** \brief Get the lower bound cost to go from this state to the goal through the continuous state
                 * space. */
                ompl::base::Cost getLowerBoundCostToGo() const;

                /** \brief Get the lower bound cost to come from the start to this state through the continuous state
                 * space. */
                ompl::base::Cost getLowerBoundCostToCome() const;

                /** \brief Get the current cost to come from the start to this state. */
                ompl::base::Cost getCurrentCostToCome() const;

            private:
                /** \brief Grant access to the state internals to the random geometric graph. */
                friend class RandomGeometricGraph;

                /** \brief The unique id of this state. */
                const std::size_t id_;

                /** \brief The estimated effort (number of collision detections) to go from this state to the goal
                 * through the current graph. */
                std::size_t estimatedEffortToGo_{std::numeric_limits<std::size_t>::max()};

                /** \brief A best estimate of the cost to go from this state to the goal through the current RGG. */
                ompl::base::Cost estimatedCostToGo_;

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

                /** \brief The neighbors in the graph, associated with a graph tag. **/
                mutable std::pair<std::size_t, std::vector<std::shared_ptr<State>>> neighbors_{};

                /** \brief The blacklist of states that can not be connected to this state. */
                std::set<std::size_t> blacklist_{};  // Maybe this would be faster as vector?

                /** \brief The whitelist of states that can be connected to this state. */
                std::set<std::size_t> whitelist_{};  // Maybe this would be faster as vector?

                /** \brief The info on the state space this state lives in. */
                std::shared_ptr<ompl::base::SpaceInformation> spaceInfo_;

                /** \brief The optimization objective. */
                std::shared_ptr<ompl::base::OptimizationObjective> objective_;
            };

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_EITSTAR_STATE_
