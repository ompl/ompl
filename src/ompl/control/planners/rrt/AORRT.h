/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Worcester Polytechnic Institute ELPIS Lab
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
 *   * Neither the name of the Worcester Polytechnic Institute nor the
 *     names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
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

/* Author: Zhuoyun Zhong */

#ifndef OMPL_CONTROL_PLANNERS_RRT_AORRT_
#define OMPL_CONTROL_PLANNERS_RRT_AORRT_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/control/PathControl.h"
#include <cmath>
#include <limits>

namespace ompl
{
    namespace control
    {
        /**
           @anchor AORRT
           @par Short description
           AORRT, or AO-RRT, is an asymptotically optimal kinodynamic planner based on the
           AO-x state-cost formulation. This implementation grows a single search tree by
           sampling states, controls, and propagation durations, then forward-propagating
           the system dynamics from a selected existing motion. Before an initial solution
           is found, it behaves like vanilla control RRT. After a solution is found, it
           augments states with an accumulated cost coordinate and samples within the
           current best cost bound, biasing the search toward trajectories that can improve
           the solution. This implementation also supports terminal-cost objectives, where
           the total cost of a solution is evaluated using the accumulated running cost
           plus the terminal cost.
           @par External documentation
           K. Hauser and Y. Zhou, Asymptotically Optimal Planning by
           Feasible Kinodynamic Planning in State-Cost Space, arXiv:1505.04098,
           2015.<br>
           [[PDF]](https://arxiv.org/pdf/1505.04098)<br>
           Z. Zhong, S. Golestaneh, and C. Chamzas, Terminal Matters:
           Kinodynamic Planning with a Terminal Cost and Learned Uncertainty
           in Belief State-Cost Space, arXiv:2605.09046, 2026.<br>
           [[PDF]](https://arxiv.org/pdf/2605.09046)
        */

        /** \brief Asymptotically Optimal Rapidly-exploring Random Tree in State-Cost Space */
        class AORRT : public base::Planner
        {
        public:
            AORRT(const SpaceInformationPtr &si);

            ~AORRT() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Solve the problem once given current best cost */
            base::PlannerStatus solveOnce(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

            /** \brief Set goal bias for sampling

                In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the weight for the cost component in the agumented-space metric */
            void setCostWeight(double weight)
            {
                costWeight_ = weight;
            }

            /** \brief Get the weight for the cost component in the agumented-space metric */
            double getCostWeight() const
            {
                return costWeight_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            /** \brief Get the planner structure */
            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            /** \brief Setup the planner */
            void setup() override;

            /** \brief Get the cost of the previous solution */
            double getBestCost() const
            {
                return bestCost_.value();
            }

        protected:
            /** \brief Representation of a motion
            This only contains pointers to parent motions as we
            only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                /** \brief The cost of the motion */
                base::Cost cost{0.0};

                /** \brief The cumulative motion cost + terminal cost*/
                base::Cost totalCost{0.0};
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief The weight for the cost component in the agumented-space metric */
            double costWeight_{1.0};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_{false};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion. Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            /** \brief Compute distance between motions (regular state distance and that in cost-state space) */
            double distanceFunction(const Motion *a, const Motion *b) const;
            double costDistanceFunction(const Motion *a, const Motion *b) const;

            /** \brief Results of single solving */
            base::PathPtr prevSolution_;
            base::PathPtr prevExactSolution_;
            bool prevApproximate_{true};
            double prevApproximateDif_{std::numeric_limits<double>::infinity()};

            /** \brief Set the current best cost */
            void setBestCost(base::Cost cost);

            /** \brief Prune all the nodes that have a cost greater than the best cost */
            void pruneNN(base::Cost costBound);

            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::infinity()};

            /** \brief The optimization objective. */
            base::OptimizationObjectivePtr opt_;
        };
    }  // namespace control
}  // namespace ompl

#endif
