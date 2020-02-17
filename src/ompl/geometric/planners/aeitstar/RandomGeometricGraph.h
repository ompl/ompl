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

#ifndef OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_RANDOM_GEOMETRIC_GRAPH_
#define OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_RANDOM_GEOMETRIC_GRAPH_

#include <limits>
#include <memory>
#include <iostream>  // This is needed for ompl's nearest neighbors struct...

#include "ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/OptimizationObjective.h"

#include "ompl/geometric/planners/aeitstar/State.h"
#include "ompl/geometric/planners/aeitstar/Edge.h"

namespace ompl
{
    namespace geometric
    {
        namespace aeitstar
        {
            class RandomGeometricGraph
            {
            public:
                /** \brief Constructs a random geometric graph. */
                RandomGeometricGraph(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo);

                /** \brief Destricts a random geometric graph. */
                ~RandomGeometricGraph() = default;

                /** \brief Sets the optimization objective. */
                void setProblemDefinition(const std::shared_ptr<ompl::base::ProblemDefinition> &problem);

                /** \brief Sets the radius factor. */
                void setRadiusFactor(double factor);

                /** \brief Samples random states and adds them to the graph. */
                void addStates(std::size_t numStates);

                /** \brief Enable pruning of the graph. */
                void enablePruning(bool prune);

                /** \brief Prunes the graph of states that can not improve the current solution. */
                void prune();

                /** \brief Gets the neighbors of a state. */
                const std::vector<std::shared_ptr<State>> &getNeighbors(const std::shared_ptr<State> &state) const;

                /** \brief Get start state. */
                std::shared_ptr<State> getStartState() const;

                /** \brief Get goal state. */
                std::shared_ptr<State> getGoalState() const;

                /** \brief Sets the start state. */
                std::shared_ptr<State> setStartState(const ompl::base::State *startState);

                /** \brief Sets the goal state. */
                std::shared_ptr<State> setGoalState(const ompl::base::State *goalState);

                /** \brief Returns whether a start state is set. */
                bool hasStartState() const;

                /** \brief Returns whether a goal state is set. */
                bool hasGoalState() const;

                /** \brief Returns all sampled states. */
                std::vector<std::shared_ptr<State>> getSamples() const;

                /** \brief Registers an invalid edge. */
                void registerInvalidEdge(const Edge &edge) const;

            private:
                /** \brief Returns the number of states in the informed set. */
                std::size_t countSamplesInInformedSet() const;

                /** \brief Returns whether a state can possibly improve the current solution. */
                bool canPossiblyImproveSolution(const std::shared_ptr<State> &state) const;

                /** \brief Computes the radius for the RGG. */
                double computeRadius(std::size_t numInformedSamples) const;

                /** \brief The tag of the current RGG. */
                std::size_t tag_{1u};

                /** \brief The sampled states in a nearest neighbor structure. */
                NearestNeighborsGNATNoThreadSafety<std::shared_ptr<State>> samples_;

                /** \brief The state sampler. */
                std::shared_ptr<ompl::base::InformedSampler> sampler_{nullptr};

                /** \brief The info about the underlying state space. */
                std::shared_ptr<ompl::base::SpaceInformation> spaceInfo_;

                /** \brief The problem this graph is supposed to help solve. */
                std::shared_ptr<ompl::base::ProblemDefinition> problem_;

                /** \brief The optimization objective this graph is supposed to help optimize. */
                std::shared_ptr<ompl::base::OptimizationObjective> objective_;

                /** \brief The start state of the problem. */
                std::shared_ptr<State> startState_;

                /** \brief The goal state of the problem. */
                std::shared_ptr<State> goalState_;

                /** \brief Whether pruning is enabled. */
                bool isPruningEnabled_{true};

                /** \brief The connection radius of the RGG. */
                double radius_{std::numeric_limits<double>::infinity()};

                /** \brief The factor by which to scale the connection radius. */
                double radiusFactor_{1.001};

                /** \brief The dimension of the state space this graph is embedded in. */
                const double dimension_;

                /** \brief The measure of a unit ball in n dimensions. */
                const double unitNBallMeasure_;
            };

        }  // namespace aeitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_RANDOM_GEOMETRIC_GRAPH_
