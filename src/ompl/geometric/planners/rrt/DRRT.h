/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Darie Roman
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

/* Authors: Darie Roman */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_DRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_DRRT_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include <queue>
#include <set>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gDRRT
           @par Short description
           \ref gDRRT "DRRT" (Dynamic RRT) is a tree-based motion planning algorithm for dynamic
           environments. In addition to traditional RRT, it implements a replanning procedure which
           repairs the RRT when changes have been made to the configuration space. In static
           environments, \ref gDRRT "DRRT" behaves identically to RRT, and thus inherits properties
           such as probabilistic completeness.
           @par External documentation
           D. Ferguson, N. Kalra, and A. Stentz, “Replanning with RRTs,” in Proceedings 2006 IEEE
           International Conference on Robotics and Automation, 2006. ICRA 2006., Orlando, FL, USA,
           2006, pp. 1243–1248. doi: 10.1109/ROBOT.2006.1641879.<br>
           [[PDF]](https://www.clear.rice.edu/comp450/papers/drrt.pdf)
        */

        /** \brief Dynamic Rapidly-exploring Random Trees */
        class DRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            DRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);

            ~DRRT() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void getPlannerData(base::PlannerData &data) const override;

            void clear() override;

            void setup() override;

            /** \brief Informs the algorithm that the environment has changed. Triggers tree pruning
             * and updates waypoints. Call this either regularly, or only when the path becomes
             * invalidated by an obstacle */
            void pruneTree();

            /** \brief Set the start bias

                Note: DRRT is rooted at goal, builds toward start.
                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual robot start state
                with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setStartBias(double startBias)
            {
                startBias_ = startBias;
            }

            /** \brief Get the start bias the planner is using */
            double getStartBias() const
            {
                return startBias_;
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

        protected:
            /** \brief Representation of a motion */
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

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion *> children;

                /** \brief Flag to indicate if this motion is on the path to the goal, for faster waypoint updating */
                bool onPath{false};

                /** \brief Set to true if this vertex is in the start state*/
                bool inStart;

            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Removes the given motion from the parent's child list */
            static void removeFromParent(Motion *m);

            /** \brief Recursively get all children stemming from Motion m */
            void getAllChildren(const Motion *m, std::vector<Motion *> &cs,
                                const std::set<Motion *> &directlyInvalidatedMotions);

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief List of motions along the path */
            std::vector<Motion *> path_;

            /** \brief List of waypoint motions, which are motions that used to be along now-deleted path segments
             * to goal */
             std::vector<Motion *> waypoints_;

            /** \brief The fraction of time the start state is picked as the state to expand towards (if such a state is
             * available) */
            double startBias_{.05};

            /** \brief The fraction of time a random waypoint state is picked as the state to expand towards (if such a
             * state is available, i.e., tree is regrowing) */
            double waypointBias_{0.5};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            /** \brief Flag indicating whether there is a path to the goal */
            bool pathToGoal_;

            /** \brief Flag indicating whether tree is currently regrowing/recovering */
            bool regrowing_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion. Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
        };
    }
}

#endif

