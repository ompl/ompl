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

#ifndef OMPL_GEOMETRIC_PLANNERS_EITSTAR_RANDOM_GEOMETRIC_GRAPH_
#define OMPL_GEOMETRIC_PLANNERS_EITSTAR_RANDOM_GEOMETRIC_GRAPH_

#include <limits>
#include <memory>
#include <iostream>  // This is needed for ompl's nearest neighbors struct...

#include "ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/Planner.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/OptimizationObjective.h"

#include "ompl/geometric/planners/eitstar/State.h"
#include "ompl/geometric/planners/eitstar/Edge.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            class RandomGeometricGraph
            {
            public:
                /** \brief Constructs a random geometric graph. */
                RandomGeometricGraph(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo,
                                     const ompl::base::Cost &solutionCost);

                /** \brief Destricts a random geometric graph. */
                ~RandomGeometricGraph() = default;

                /** \brief Sets the optimization objective. */
                void setup(const std::shared_ptr<ompl::base::ProblemDefinition> &problem,
                           ompl::base::PlannerInputStates *inputStates);

                /** \brief Adds new start and goals to the graph if available and creates a new informed sampler if
                 * necessary. */
                void updateStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition,
                                              ompl::base::PlannerInputStates *inputStates);

                /** \brief Sets the radius factor. */
                void setRadiusFactor(double factor);

                /** \brief Samples random states and adds them to the graph. */
                void addStates(std::size_t numStates);

                /** \brief Enable pruning of the graph. */
                void enablePruning(bool prune);

                /** \brief Whether pruning is enabled. */
                bool isPruningEnabled() const;

                /** \brief Prunes the graph of states that can not improve the current solution. */
                void prune();

                /** \brief Gets the neighbors of a state. */
                const std::vector<std::shared_ptr<State>> &getNeighbors(const std::shared_ptr<State> &state) const;

                /** \brief Get the start states. */
                const std::vector<std::shared_ptr<State>> &getStartStates() const;

                /** \brief Get the goal states. */
                const std::vector<std::shared_ptr<State>> &getGoalStates() const;

                /** \brief Sets the start state. */
                std::shared_ptr<State> registerStartState(const ompl::base::State *start);

                /** \brief Sets the goal state. */
                std::shared_ptr<State> registerGoalState(const ompl::base::State *goal);

                /** \brief Returns whether a start state is set. */
                bool hasStartState() const;

                /** \brief Returns whether a goal state is set. */
                bool hasGoalState() const;

                /** \brief Returns whether a state is a start state. */
                bool isStart(const std::shared_ptr<State> &state) const;

                /** \brief Returns whether a state is a goal state. */
                bool isGoal(const std::shared_ptr<State> &state) const;

                /** \brief Returns all sampled states. */
                std::vector<std::shared_ptr<State>> getSamples() const;

                /** \brief Registers an invalid edge. */
                void registerInvalidEdge(const Edge &edge) const;

                /** \brief Get the tag of the current RGG. */
                std::size_t getTag() const;

            private:
                /** \brief Returns the number of states in the informed set. */
                std::size_t countSamplesInInformedSet() const;

                /** \brief Returns whether a state can possibly improve the current solution. */
                bool canPossiblyImproveSolution(const std::shared_ptr<State> &state) const;

                /** \brief Get the heuristic cost from the preferred start of a state. */
                ompl::base::Cost heuristicCostFromPreferredStart(const std::shared_ptr<State> &state) const;

                /** \brief Get the heuristic cost to the preferred goal of a state. */
                ompl::base::Cost heuristicCostToPreferredGoal(const std::shared_ptr<State> &state) const;

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

                /** \brief The start states of the problem. */
                std::vector<std::shared_ptr<State>> startStates_;

                /** \brief The goal states of the problem. */
                std::vector<std::shared_ptr<State>> goalStates_;

                /** \brief The pruned start states of the problem. We keep these around because if a new start is added
                 * after pruning a goal, we might want to consider the new goal again. */
                std::vector<std::shared_ptr<State>> prunedStartStates_;

                /** \brief The pruned goal states of the problem. We keep these around because if a new goal is added
                 * after pruning a start, we might want to consider the new start again. */
                std::vector<std::shared_ptr<State>> prunedGoalStates_;

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

                /** \brief The cost of the incumbent solution. */
                const ompl::base::Cost &solutionCost_;
            };

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_EITSTAR_RANDOM_GEOMETRIC_GRAPH_
