/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Francesco Grothe */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_STRRT_STAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_STRRT_STAR_

#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/AnimationStateSpace.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/GeometricEquations.h>
#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace geometric
    {
        class STRRTstar : public base::Planner
        {
        public:
            /** \brief Constructor */
            explicit STRRTstar(const ompl::base::SpaceInformationPtr &si);

            ~STRRTstar() override;

            void clear() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Set the range the planner is supposed to use.

                        This parameter greatly influences the runtime of the
                        algorithm. It represents the maximum length of a
                        motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set whether the planner should optimize the solution with respect to time. */
            void setOptimize(bool optimize)
            {
                optimize_ = optimize;
            }

            /** \brief Get whether the planner optimizes the solution. */
            bool getOptimize() const
            {
                return optimize_;
            }

            double getOptimumApproxFactor() const
            {
                return optimumApproxFactor_;
            }

            void setOptimumApproxFactor(double optimumApproxFactor)
            {
                if (optimumApproxFactor <= 0 || optimumApproxFactor > 1)
                {
                    OMPL_ERROR("%s: The optimum approximation factor needs to be between 0 and 1.", getName().c_str());
                }
                optimumApproxFactor_ = optimumApproxFactor;
            }

            std::string getRewiringState() const
            {
                std::vector<std::string> s{"Radius", "KNearest", "Off"};
                return s[rewireState_];
            }

            void setRewiringToOff()
            {
                rewireState_ = OFF;
            }

            void setRewiringToRadius()
            {
                rewireState_ = RADIUS;
            }

            void setRewiringToKNearest()
            {
                rewireState_ = KNEAREST;
            }

            double getRewireFactor() const
            {
                return rewireFactor_;
            }

            void setRewireFactor(double v)
            {
                if (v <= 1)
                {
                    OMPL_ERROR("%s: Rewire Factor needs to be greater than 1.", getName().c_str());
                }
                rewireFactor_ = v;
            }

            unsigned int getBatchSize() const
            {
                return initialBatchSize_;
            }

            void setBatchSize(int v)
            {
                if (v < 1)
                {
                    OMPL_ERROR("%s: Batch Size needs to be at least 1.", getName().c_str());
                }
                initialBatchSize_ = v;
            }

            void setTimeBoundFactorIncrease(double f)
            {
                if (f <= 1.0)
                {
                    OMPL_ERROR("%s: Time Bound Factor Increase needs to be higher than 1.", getName().c_str());
                    OMPL_ERROR("%s: Time Bound Factor Increase needs to be higher than 1.", getName().c_str());
                }
                timeBoundFactorIncrease_ = f;
            }

            void setInitialTimeBoundFactor(double f)
            {
                if (f <= 1.0)
                {
                    OMPL_ERROR("%s: Initial Time Bound Factor Increase needs to be higher than 1.", getName().c_str());
                }
                initialTimeBoundFactor_ = f;
            }

            void setSampleUniformForUnboundedTime(bool uniform)
            {
                sampleUniformForUnboundedTime_ = uniform;
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion */
            class Motion
            {
            public:
                Motion() = default;

                explicit Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                const base::State *root{nullptr};
                base::State *state{nullptr};
                Motion *parent{nullptr};
                /** \brief The set of motions descending from the current motion */
                std::vector<Motion *> children{};
                // only used by goal tree
                Motion *connectionPoint{nullptr};  // the start tree motion, if there is a direct connection
                int numConnections{0};  // number of connections to the start tree of self and all descendants
            };

            /** \brief Optimization Objective to minimize time at which any goal can be achieved.
             * It is not possible to set a different optimization objective. */
            class MinimizeGoalTime : public base::OptimizationObjective
            {
            public:
                explicit MinimizeGoalTime(const base::SpaceInformationPtr &si) : OptimizationObjective(si)
                {
                }

            private:
                base::Cost stateCost(const ompl::base::State *s) const override
                {
                    return base::Cost(s->as<base::CompoundState>()->as<base::TimeStateSpace::StateType>(1)->position);
                }

                base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
                {
                    return combineCosts(stateCost(s1), stateCost(s2));
                }

                base::Cost combineCosts(ompl::base::Cost c1, ompl::base::Cost c2) const override
                {
                    return c1.value() > c2.value() ? c1 : c2;
                }

                base::Cost identityCost() const override
                {
                    return base::Cost(-std::numeric_limits<double>::infinity());
                }
            };

            class ConditionalSampler : public base::ValidStateSampler
            {
            public:
                ConditionalSampler(const ompl::base::SpaceInformation *si, Motion *&startMotion,
                                   std::vector<Motion *> &goalMotions, std::vector<Motion *> &newlyAddedGoalMotions,
                                   bool &sampleOldBatch)
                  : ValidStateSampler(si)
                  , startMotion_(startMotion)
                  , goalMotions_(goalMotions)
                  , newBatchGoalMotions_(newlyAddedGoalMotions)
                  , sampleOldBatch_(sampleOldBatch)
                {
                    name_ = "ConditionalSampler";
                }

                bool sample(base::State *state) override
                {
                    for (int i = 0; i < maxTries_; ++i)
                    {
                        internalSampler_->sampleUniform(state);
                        double leftBound, rightBound;
                        // get minimum time, when the state can be reached from the start
                        double startBound = startMotion_->state->as<base::CompoundState>()
                                                ->as<base::TimeStateSpace::StateType>(1)
                                                ->position +
                                            si_->getStateSpace()->as<base::AnimationStateSpace>()->timeToCoverDistance(
                                                state, startMotion_->state);
                        // sample old batch
                        if (sampleOldBatch_)
                        {
                            leftBound = startBound;
                            // get maximum time, at which any goal can be reached from the state
                            rightBound = std::numeric_limits<double>::min();
                            for (auto goal : goalMotions_)
                            {
                                double t = goal->state->as<base::CompoundState>()
                                               ->as<base::TimeStateSpace::StateType>(1)
                                               ->position -
                                           si_->getStateSpace()->as<base::AnimationStateSpace>()->timeToCoverDistance(
                                               goal->state, state);
                                if (t > rightBound)
                                {
                                    rightBound = t;
                                }
                            }
                        }
                        // sample new batch
                        else
                        {
                            // get maximum time, at which any goal from the new batch can be reached from the state
                            rightBound = std::numeric_limits<double>::min();
                            for (auto goal : newBatchGoalMotions_)
                            {
                                double t = goal->state->as<base::CompoundState>()
                                               ->as<base::TimeStateSpace::StateType>(1)
                                               ->position -
                                           si_->getStateSpace()->as<base::AnimationStateSpace>()->timeToCoverDistance(
                                               goal->state, state);
                                if (t > rightBound)
                                {
                                    rightBound = t;
                                }
                            }
                            // get maximum time, at which any goal from the old batch can be reached from the state
                            // only allow the left bound to be smaller than the right bound
                            leftBound = std::numeric_limits<double>::min();
                            for (auto goal : goalMotions_)
                            {
                                double t = goal->state->as<base::CompoundState>()
                                               ->as<base::TimeStateSpace::StateType>(1)
                                               ->position -
                                           si_->getStateSpace()->as<base::AnimationStateSpace>()->timeToCoverDistance(
                                               goal->state, state);
                                if (t > leftBound && t < rightBound)
                                {
                                    leftBound = t;
                                }
                            }
                            leftBound = std::max(leftBound, startBound);
                        }

                        if (leftBound <= rightBound)
                        {
                            double time = rng_.uniformReal(leftBound, rightBound);
                            state->as<base::CompoundState>()->as<base::TimeStateSpace::StateType>(1)->position = time;
                            return true;
                        }
                    }
                    return false;
                }

                bool sampleNear(base::State *state, const base::State *near, const double distance) override
                {
                    throw ompl::Exception("ConditionalSampler::sampleNear", "not implemented");
                }

            private:
                base::StateSamplerPtr internalSampler_ = si_->allocStateSampler();

                /** References to the start state and goal states */
                Motion *&startMotion_;

                std::vector<Motion *> &goalMotions_;
                std::vector<Motion *> &newBatchGoalMotions_;

                /** References to whether the old or new batch region is sampled */
                bool &sampleOldBatch_;

                /** \brief Maximum tries to sample a new state, if no new state could be sampled, force a new goal
                 * sample. */
                int maxTries_ = 10;

                /** \brief The random number generator */
                ompl::RNG rng_;
            };

            /** \brief Whether the solution is optimized for time or the first solution terminates the algorithm. */
            bool optimize_ = true;

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            using TreeData = std::shared_ptr<ompl::NearestNeighbors<Motion *>>;

            /** \brief Information attached to growing a tree of motions (used internally) */
            struct TreeGrowingInfo
            {
                base::State *xstate;
                Motion *xmotion;
                bool start;
            };

            /** \brief The state of the tree after an attempt to extend it */
            enum GrowState
            {
                /// no progress has been made
                TRAPPED,
                /// progress has been made towards the randomly sampled state
                ADVANCED,
                /// the randomly sampled state was reached
                REACHED
            };

            /** \brief Grow a tree towards a random state for a single nearest state */
            GrowState growTreeSingle(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion, Motion *nmotion);

            /** \brief Attempt to grow a tree towards a random state for the neighborhood of the random state. The
             * neighborhood is determined by the used rewire state. For the start tree closest state with respect to
             * distance are tried first. For the goal tree states with the minimum time root node are tried first. If
             * connect is true, multiple vertices can be added to the tree until the random state is reached or an
             * basestacle is met. If connect is false, the tree is only extended by a single new state. */
            GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion, std::vector<Motion *> &nbh,
                               bool connect);

            /** \brief Gets the neighbours of a given motion, using either k-nearest or radius_ as appropriate. */
            void getNeighbors(TreeData &tree, Motion *motion, std::vector<Motion *> &nbh) const;

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Prune the start tree after a solution was found. */
            void pruneStartTree();

            /** \brief Prune the goal tree after a solution was found.
             * Return the goal motion, that is connected to the start tree, if a new solution was found.
             * If no new solution was found, return nullpointer. */
            Motion *pruneGoalTree();

            /** \brief State sampler */
            ConditionalSampler sampler_;

            /** \brief The start tree */
            TreeData tStart_;

            /** \brief The goal tree */
            TreeData tGoal_;

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Distance between the nearest pair of start tree and goal tree nodes. */
            double distanceBetweenTrees_;

            /** \brief The current best solution path with respect to shortest time. */
            base::PathPtr bestSolution_{nullptr};

            /** \brief The current best time i.e. cost of all found solutions */
            double bestTime_ = std::numeric_limits<double>::infinity();

            /** \brief The number of while loop iterations */
            unsigned int numIterations_ = 0;

            /** \brief The number of found solutions */
            int numSolutions_ = 0;

            /** \brief Minimum Time at which any goal can be reached, if moving on a straight line. */
            double minimumTime_ = std::numeric_limits<double>::infinity();

            /** \brief Upper bound for the time up to which solutions are searched for. */
            double upperTimeBound_;

            /** \brief The factor to which found solution times need to be reduced compared to minimum time, (0, 1]. */
            double optimumApproxFactor_ = 1.0;

            /** \brief The start Motion, used for conditional sampling and start tree pruning. */
            Motion *startMotion_{nullptr};

            /** \brief The goal Motions, used for conditional sampling and pruning. */
            std::vector<Motion *> goalMotions_{};

            /** \brief The goal Motions, that were added in the current expansion step, used for uniform sampling over a
             * growing region. */
            std::vector<Motion *> newBatchGoalMotions_{};

            /**
             * Goal Sampling is not handled by PlannerInputStates, but directly by the SpaceTimeRRT,
             * because the time component of every goal sample is sampled dependent on the sampled space component.
             *
             */

            base::State *tempState_{nullptr};  // temporary sampled goal states are stored here.

            /** \brief N tries to sample a goal. */
            base::State *nextGoal(int n, double oldBatchTimeBoundFactor, double newBatchTimeBoundFactor);

            /** \brief Samples a goal until successful or the termination condition is fulfilled. */
            base::State *nextGoal(const base::PlannerTerminationCondition &ptc, double oldBatchTimeBoundFactor,
                                  double newBatchTimeBoundFactor);

            /** \brief Samples a goal until successful or the termination condition is fulfilled. */
            base::State *nextGoal(const base::PlannerTerminationCondition &ptc, int n, double oldBatchTimeBoundFactor,
                                  double newBatchTimeBoundFactor);

            /** \brief Samples the time component of a goal state dependant on its space component. Returns false, if
             * goal can't be reached in time. */
            bool sampleGoalTime(base::State *goal, double oldBatchTimeBoundFactor, double newBatchTimeBoundFactor);

            /** \brief Removes the given motion from the parent's child list. */
            static void removeFromParent(Motion *m);

            /** \brief Adds given all descendants of the given motion to given tree and checks whether one of the added
             * motions is the goal motion. */
            static void addDescendants(Motion *m, const TreeData &tree);

            void constructSolution(Motion *startMotion, Motion *goalMotion,
                                   const base::ReportIntermediateSolutionFn &intermediateSolutionCallback);

            enum RewireState
            {
                // use r-disc search for rewiring
                RADIUS,
                // use k-nearest for rewiring
                KNEAREST,
                // don't use any rewiring
                OFF
            };

            RewireState rewireState_ = KNEAREST;

            /** \brief The rewiring factor, s, so that r_rrt = s \times r_rrt* > r_rrt* (or k_rrt = s \times k_rrt* >
             * k_rrt*) */
            double rewireFactor_{1.1};

            /** \brief A constant for k-nearest rewiring calculations */
            double k_rrt_{0u};

            /** \brief A constant for r-disc rewiring calculations */
            double r_rrt_{0.};

            /** \brief Calculate the k_RRG* and r_RRG* terms */
            void calculateRewiringLowerBounds();

            bool rewireGoalTree(Motion *addedMotion);

            /** \brief Whether the time is bounded or not. The first solution automatically bounds the time. */
            bool isTimeBounded_;

            /** \brief The time bound the planner is initialized with. Used to reset for repeated planning */
            double initialTimeBound_;

            /** \brief Number of samples of the first batch */
            unsigned int initialBatchSize_ = 512;

            /** \brief Initial factor, the minimum time of each goal is multiplied with to calculate the upper time
             * bound. */
            double initialTimeBoundFactor_ = 2.0;

            /** \brief The factor, the time bound is increased with after the batch is full. */
            double timeBoundFactorIncrease_ = 2.0;

            bool sampleOldBatch_ = true;

            /** \brief Whether the samples are uniformly distributed over the whole space or are centered at lower
             * times. */
            bool sampleUniformForUnboundedTime_ = true;

            /** \brief The ratio, a goal state is sampled compared to the size of the goal tree. */
            int goalStateSampleRatio_ = 4;

            /** \brief The random number generator */
            ompl::RNG rng_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
