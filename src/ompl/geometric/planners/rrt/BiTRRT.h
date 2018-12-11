/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_BITRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_BITRRT_

#include <cmath>
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace geometric
    {
        /// @anchor gBiTRRT
        /// @par Short description
        /// This planner grows two T-RRTs, one from the start and one from the
        /// goal, and attempts to connect the trees somewhere in the middle.
        /// T-RRT is an RRT variant and tree-based motion planner that takes into consideration state costs
        /// to compute low-cost paths that follow valleys and saddle points of the configuration-space
        /// costmap. It uses transition tests from stochastic optimization methods to accept or reject new
        /// potential states.
        /// @par External documentation
        /// L. Jaillet, J. Cortés, T. Siméon, Sampling-Based Path Planning on Configuration-Space Costmaps, in <em>IEEE
        /// TRANSACTIONS ON ROBOTICS, VOL. 26, NO. 4, AUGUST 2010</em>. DOI:
        /// [10.1109/TRO.2010.2049527](http://dx.doi.org/10.1109/TRO.2010.2049527)<br />
        /// [[PDF]](http://homepages.laas.fr/nic/Papers/10TRO.pdf)
        ///
        /// D. Devaurs, T. Siméon, J. Cortés, Enhancing the Transition-based RRT to Deal with Complex Cost Spaces, in
        /// <em>IEEE International Conference on Robotics and Automation (ICRA), 2013, pp. 4120-4125. DOI:
        /// [10.1109/ICRA.2013.6631158](http://dx.doi.org/10.1109/ICRA.2013.6631158)<br/>
        ///[[PDF]](https://hal.archives-ouvertes.fr/hal-00872224/document)

        /// \brief Bi-directional Transition-based Rapidly-exploring Random Trees
        class BiTRRT : public base::Planner
        {
        public:
            /// Constructor
            BiTRRT(const base::SpaceInformationPtr &si);
            ~BiTRRT() override;
            void clear() override;
            void setup() override;
            void getPlannerData(base::PlannerData &data) const override;
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /// \brief Set the maximum possible length of any one motion in
            ///  the search tree.  Very short/long motions may inhibit
            ///  the exploratory capabilities of the planner.
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /// \brief Get the range the planner is using
            double getRange() const
            {
                return maxDistance_;
            }

            /// \brief Set the factor by which the temperature is increased
            /// after a failed transition test.  This value should be in the
            /// range (0, 1], typically close to zero (default is 0.1).
            /// This value is an exponential (e^factor) that is multiplied with
            /// the current temperature.
            void setTempChangeFactor(double factor)
            {
                tempChangeFactor_ = exp(factor);
            }

            /// \brief Get the factor by which the temperature is
            /// increased after a failed transition.
            double getTempChangeFactor() const
            {
                return log(tempChangeFactor_);
            }

            /// \brief Set the cost threshold (default is infinity).
            /// Any motion cost that is not better than this cost (according to
            /// the optimization objective) will not be expanded by the planner.
            void setCostThreshold(double maxCost)
            {
                costThreshold_ = base::Cost(maxCost);
            }

            /// \brief Get the cost threshold (default is infinity).
            /// Any motion cost that is not better than this cost (according to
            /// the optimization objective) will not be expanded by the planner. */
            double getCostThreshold() const
            {
                return costThreshold_.value();
            }

            /// \brief Set the initial temperature at the start of planning.
            /// Should be high to allow for initial exploration.
            void setInitTemperature(double initTemperature)
            {
                initTemperature_ = initTemperature;
            }

            /// \brief Get the initial temperature at the start of planning.
            double getInitTemperature() const
            {
                return initTemperature_;
            }

            /// \brief Set the distance between a new state and the nearest
            /// neighbor that qualifies a state as being a frontier node.
            void setFrontierThreshold(double frontierThreshold)
            {
                frontierThreshold_ = frontierThreshold;
            }

            /// \brief Get the distance between a new state and the nearest
            /// neighbor that qualifies a state as being a frontier node.
            double getFrontierThreshold() const
            {
                return frontierThreshold_;
            }

            /// \brief Set the ratio between adding non-frontier nodes to
            /// frontier nodes.  For example: .1 is one non-frontier node for
            /// every 10 frontier nodes added.
            void setFrontierNodeRatio(double frontierNodeRatio)
            {
                frontierNodeRatio_ = frontierNodeRatio;
            }

            /// \brief Get the ratio between adding non-frontier nodes to
            /// frontier nodes.
            double getFrontierNodeRatio() const
            {
                return frontierNodeRatio_;
            }

            /// \brief Set a different nearest neighbors datastructure
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0))
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                tStart_ = std::make_shared<NN<Motion *>>();
                tGoal_ = std::make_shared<NN<Motion *>>();
                setup();
            }

        protected:
            /// \brief Representation of a motion in the search tree
            class Motion
            {
            public:
                /// \brief Default constructor
                Motion() = default;

                /// \brief Constructor that allocates memory for the state
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /// \brief The state contained by the motion
                base::State *state{nullptr};

                /// \brief The parent motion in the exploration tree
                Motion *parent{nullptr};

                /// \brief Cost of the state
                base::Cost cost;

                /// \brief Pointer to the root of the tree this motion is
                /// contained in.
                const base::State *root{nullptr};
            };

            /// \brief Free all memory allocated during planning
            void freeMemory();

            /// \brief The nearest-neighbors data structure that contains the
            /// entire the tree of motions generated during planning.
            using TreeData = std::shared_ptr<NearestNeighbors<Motion *>>;

            /// \brief Add a state to the given tree.  The motion created
            /// is returned.
            Motion *addMotion(const base::State *state, TreeData &tree, Motion *parent = nullptr);

            /// \brief Transition test that filters transitions based on the
            /// motion cost.  If the motion cost is near or below zero, the motion
            /// is always accepted, otherwise a probabilistic criterion based on
            /// the temperature and motionCost is used.
            bool transitionTest(const base::Cost &motionCost);

            /// \brief Use frontier node ratio to filter nodes that do not add
            /// new information to the search tree.
            bool minExpansionControl(double dist);

            /// \brief The result of a call to extendTree
            enum GrowResult
            {
                /// No extension was possible
                FAILED,
                /// Progress was made toward extension
                ADVANCED,
                /// The desired state was reached during extension
                SUCCESS
            };

            /// \brief Extend \e tree toward the state in \e rmotion.
            /// Store the result of the extension, if any, in result
            GrowResult extendTree(Motion *toMotion, TreeData &tree, Motion *&result);

            /// \brief Extend \e tree from \e nearest toward \e toMotion.
            /// Store the result of the extension, if any, in result
            GrowResult extendTree(Motion *nearest, TreeData &tree, Motion *toMotion, Motion *&result);

            /// \brief Attempt to connect \e tree to \e nmotion, which is in
            /// the other tree.  \e xmotion is scratch space and will be overwritten
            bool connectTrees(Motion *nmotion, TreeData &tree, Motion *xmotion);

            /// \brief Compute distance between motions (actually distance between contained states)
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /// \brief The maximum length of a motion to be added to a tree
            double maxDistance_{0.};

            /// \brief The factor by which the temperature is increased after
            /// a failed transition test.
            double tempChangeFactor_;

            /// \brief The most desirable (e.g., minimum) cost value in the search tree
            base::Cost bestCost_;

            /// \brief The least desirable (e.g., maximum) cost value in the search tree
            base::Cost worstCost_;

            /// \brief All motion costs must be better than this cost (default is infinity)
            base::Cost costThreshold_{std::numeric_limits<double>::infinity()};

            /// \brief The temperature that planning begins at.
            double initTemperature_{100.};

            /// \brief The distance between an existing state and a new state
            /// that qualifies it as a frontier state.
            double frontierThreshold_{0.};

            /// \brief The target ratio of non-frontier nodes to frontier nodes.
            double frontierNodeRatio_{.1};

            /// \brief The current temperature
            double temp_;

            /// \brief A count of the number of non-frontier nodes in the trees
            double nonfrontierCount_;

            /// \brief A count of the number of frontier nodes in the trees
            double frontierCount_;

            /// \brief The range at which the algorithm will attempt to connect
            /// the two trees.
            double connectionRange_;

            /// \brief The most recent connection point for the two trees.
            /// Used for PlannerData computation.
            std::pair<Motion *, Motion *> connectionPoint_{nullptr, nullptr};

            /// \brief The start tree
            TreeData tStart_;

            /// \brief The goal tree
            TreeData tGoal_;

            /// \brief The objective (cost function) being optimized
            ompl::base::OptimizationObjectivePtr opt_;
        };
    }
}

#endif