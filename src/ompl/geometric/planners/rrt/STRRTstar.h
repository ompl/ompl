/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
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
*   * Neither the name of the TU Berlin nor the names of its
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
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/objectives/MinimizeArrivalTime.h>
#include <ompl/base/samplers/ConditionalStateSampler.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/GeometricEquations.h>
#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gSTRRTstar
           @par Short description
           ST-RRT* is a bidirectional, time-optimal planner for planning in space-time.
           It operates similar to a bidirectional version of RRT*, but allows planning in unbounded time spaces by
           gradual time-bound extensions and is highly optimized for planning in space-time by employing
           Conditional Sampling and Simplified Rewiring.
        */

        /** \brief Space-Time RRT* (STRRTstar) */
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
            void setRange(double distance);

            /** \brief Get the range the planner is using */
            double getRange() const;

            /** \brief The Optimum Approximation factor (0 - 1). */
            double getOptimumApproxFactor() const;

            /** \brief Set the Optimum Approximation factor. This allows the planner to converge more quickly, but only
             * yields approximately optimal solutions. */
            void setOptimumApproxFactor(double optimumApproxFactor);

            std::string getRewiringState() const;

            /** \brief Do not rewire at all. */
            void setRewiringToOff();

            /** \brief Rewire by radius. */
            void setRewiringToRadius();

            /** \brief Rewire by k-nearest. */
            void setRewiringToKNearest();

            double getRewireFactor() const;

            void setRewireFactor(double v);

            /** \brief The number of samples before the time bound is increased. */
            unsigned int getBatchSize() const;

            void setBatchSize(int v);

            /** \brief The value by which the time bound factor is multiplied in each increase step. */
            void setTimeBoundFactorIncrease(double f);

            /** \brief The initial time bound factor. */
            void setInitialTimeBoundFactor(double f);

            /** \brief Whether the state space is sampled uniformly or centered at lower time values. */
            void setSampleUniformForUnboundedTime(bool uniform);

            void setup() override;

        protected:

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            using TreeData = std::shared_ptr<ompl::NearestNeighbors<base::Motion *>>;

            /** \brief Information attached to growing a tree of motions (used internally) */
            struct TreeGrowingInfo
            {
                base::State *xstate;
                base::Motion *xmotion;
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
            GrowState growTreeSingle(TreeData &tree, TreeGrowingInfo &tgi, base::Motion *rmotion, base::Motion *nmotion);

            /** \brief Attempt to grow a tree towards a random state for the neighborhood of the random state. The
             * neighborhood is determined by the used rewire state. For the start tree closest state with respect to
             * distance are tried first. For the goal tree states with the minimum time root node are tried first. If
             * connect is true, multiple vertices can be added to the tree until the random state is reached or an
             * basestacle is met. If connect is false, the tree is only extended by a single new state. */
            GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, base::Motion *rmotion, std::vector<base::Motion *> &nbh,
                               bool connect);

            /** \brief  */
            void increaseTimeBound(bool hasEqualBounds, double &oldBatchTimeBoundFactor, double &newBatchTimeBoundFactor,
                                   bool &startTree, unsigned int &batchSize, int &numBatchSamples);

            /** \brief Gets the neighbours of a given motion, using either k-nearest or radius_ as appropriate. */
            void getNeighbors(TreeData &tree, base::Motion *motion, std::vector<base::Motion *> &nbh) const;

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const base::Motion *a, const base::Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Prune the start tree after a solution was found. */
            void pruneStartTree();

            /** \brief Prune the goal tree after a solution was found.
             * Return the goal motion, that is connected to the start tree, if a new solution was found.
             * If no new solution was found, return nullpointer. */
            base::Motion *pruneGoalTree();

            /** \brief Find the solution (connecting) motion for a motion that is indirectly connected. */
            static base::Motion *computeSolutionMotion(base::Motion *motion);

            /** \brief Remove invalid goal states from the goal set. */
            void removeInvalidGoals(const std::vector<base::Motion *> &invalidGoals);

            /** \brief State sampler */
            base::ConditionalStateSampler sampler_;

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
            base::Motion *startMotion_{nullptr};

            /** \brief The goal Motions, used for conditional sampling and pruning. */
            std::vector<base::Motion *> goalMotions_{};

            /** \brief The goal Motions, that were added in the current expansion step, used for uniform sampling over a
             * growing region. */
            std::vector<base::Motion *> newBatchGoalMotions_{};

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
            static void removeFromParent(base::Motion *m);

            /** \brief Adds given all descendants of the given motion to given tree and checks whether one of the added
             * motions is the goal motion. */
            static void addDescendants(base::Motion *m, const TreeData &tree);

            void constructSolution(base::Motion *startMotion, base::Motion *goalMotion,
                                   const base::ReportIntermediateSolutionFn &intermediateSolutionCallback,
                                   const ompl::base::PlannerTerminationCondition& ptc);

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

            bool rewireGoalTree(base::Motion *addedMotion);

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
