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

/* Author: Dave Coleman, Ryan Luna */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_TRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_TRRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/OptimizationObjective.h"

/*
  NOTES:
  **Variable Names that have been converted to longer versions from standards:
  nearest_neighbors_ -> nn_
  planner_termination_condition -> ptc

  **Inherited Member Variables Key:
  si_ -> SpaceInformation
  pdef_ -> ProblemDefinition
  pis_ -> PlannerInputStates - Utility class to extract valid input states
*/

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gTRRT
           @par Short description
           T-RRT is an RRT variant and tree-based motion planner that takes into consideration state costs
           to compute low-cost paths that follow valleys and saddle points of the configuration-space
           costmap. It uses transition tests from stochastic optimization methods to accept or reject new
           potential states.
           @par Example usage
           Please see [Dave Coleman's example](https://github.com/davetcoleman/ompl_rviz_viewer/) to see how TRRT can be
           used.
           @par External documentation
           L. Jaillet, J. Cortés, T. Siméon, Sampling-Based Path Planning on Configuration-Space Costmaps, in <em>IEEE
           TRANSACTIONS ON ROBOTICS, VOL. 26, NO. 4, AUGUST 2010</em>. DOI:
           [10.1109/TRO.2010.2049527](http://dx.doi.org/10.1109/TRO.2010.2049527)<br />
           [[PDF]](http://homepages.laas.fr/nic/Papers/10TRO.pdf)

           D. Devaurs, T. Siméon, J. Cortés, Enhancing the Transition-based RRT to Deal with Complex Cost Spaces, in
           <em>IEEE International Conference on Robotics and Automation, 2013, pp. 4120-4125. DOI:
           [10.1109/ICRA.2013.6631158](http://dx.doi.org/10.1109/ICRA.2013.6631158)<br/>
           [[PDF]](https://hal.archives-ouvertes.fr/hal-00872224/document)
        */

        /** \brief Transition-based Rapidly-exploring Random Trees */
        class TRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            TRRT(const base::SpaceInformationPtr &si);

            ~TRRT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &plannerTerminationCondition) override;

            void clear() override;

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

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

            /** \brief Set the factor by which the temperature is increased
                after a failed transition test.  This value should be in the
                range (0, 1], typically close to zero (default is 0.1).
                This value is an exponential (e^factor) that is multiplied with
                the current temperature. */
            void setTempChangeFactor(double factor)
            {
                tempChangeFactor_ = exp(factor);
            }

            /** \brief Get the factor by which the temperature rises based on current acceptance/rejection rate */
            double getTempChangeFactor() const
            {
                return log(tempChangeFactor_);
            }

            /** \brief Set the cost threshold (default is infinity).
                Any motion cost that is not better than this cost (according to
                the optimization objective) will not be expanded by the planner. */
            void setCostThreshold(double maxCost)
            {
                costThreshold_ = base::Cost(maxCost);
            }

            /** \brief Get the cost threshold (default is infinity).
                 Any motion cost that is not better than this cost (according to
                 the optimization objective) will not be expanded by the planner. */
            double getCostThreshold() const
            {
                return costThreshold_.value();
            }

            /** \brief Set the initial temperature at the beginning of the algorithm. Should be high
                       to allow for initial exploration. */
            void setInitTemperature(double initTemperature)
            {
                initTemperature_ = initTemperature;
                temp_ = initTemperature_;
            }

            /** \brief Get the temperature at the start of planning. */
            double getInitTemperature() const
            {
                return initTemperature_;
            }

            /** \brief Set the distance between a new state and the nearest neighbor
                that qualifies that state as being a frontier */
            void setFrontierThreshold(double frontier_threshold)
            {
                frontierThreshold_ = frontier_threshold;
            }

            /** \brief Get the distance between a new state and the nearest neighbor
                that qualifies that state as being a frontier */
            double getFrontierThreshold() const
            {
                return frontierThreshold_;
            }

            /** \brief Set the ratio between adding nonfrontier nodes to frontier nodes,
                for example .1 is 1/10 or one nonfrontier node for every 10 frontier nodes added */
            void setFrontierNodeRatio(double frontierNodeRatio)
            {
                frontierNodeRatio_ = frontierNodeRatio;
            }

            /** \brief Get the ratio between adding nonfrontier nodes to frontier nodes,
                for example .1 is 1/10 or one nonfrontier node for every 10 frontier nodes added */
            double getFrontierNodeRatio() const
            {
                return frontierNodeRatio_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nearestNeighbors_->size() == 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nearestNeighbors_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                /** \brief Cost of the state */
                base::Cost cost;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Filter irrelevant configuration regarding the search of low-cost paths before inserting into tree
                \param motionCost - cost of the motion to be evaluated
            */
            bool transitionTest(const base::Cost &motionCost);

            /** \brief Use ratio to prefer frontier nodes to nonfrontier ones */
            bool minExpansionControl(double randMotionDistance);

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nearestNeighbors_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            // *********************************************************************************************************
            // TRRT-Specific Variables
            // *********************************************************************************************************

            // Transtion Test -----------------------------------------------------------------------

            /** \brief Temperature parameter used to control the difficulty level of transition tests. Low temperatures
                limit the expansion to a slightly positive slopes, high temps enable to climb the steeper slopes.
                Dynamically tuned according to the information acquired during exploration */
            double temp_;

            /** \brief The most desirable (e.g., minimum) cost value in the search tree */
            base::Cost bestCost_;

            /** \brief The least desirable (e.g., maximum) cost value in the search tree */
            base::Cost worstCost_;

            /** \brief All motion costs must be better than this cost (default is infinity) */
            base::Cost costThreshold_;

            /** \brief The value of the expression exp^T_rate.  The temperature
                 is increased by this factor whenever the transition test fails. */
            double tempChangeFactor_;

            /** \brief The initial value of \e temp_ */
            double initTemperature_;

            // Minimum Expansion Control --------------------------------------------------------------

            /** \brief The number of non-frontier nodes in the search tree */
            double nonfrontierCount_;
            /** \brief The number of frontier nodes in the search tree */
            double frontierCount_;

            /** \brief The distance between an old state and a new state that
                qualifies it as a frontier state */
            double frontierThreshold_;

            /** \brief Target ratio of non-frontier nodes to frontier nodes. rho */
            double frontierNodeRatio_;

            /** \brief The optimization objective being optimized by TRRT */
            ompl::base::OptimizationObjectivePtr opt_;
        };
    }
}

#endif
