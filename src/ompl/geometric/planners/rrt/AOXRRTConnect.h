/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Queen's University
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
 *   * Neither the name of the Queen's University nor the names of its
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

/* Author: Tyler Wilson */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_AOX_RRT_CONNECT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_AOX_RRT_CONNECT_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "ompl/base/Path.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gAORRTC RRT-Connect
           @par Short description
           AORRTC leverages RRT-Connect to repeatedly search
           in a cost-augmented space, which is slowly constrained
           to force the planner to iteratively find better
           solutions. This modified RRT-Connect performs
           its search in the configuration space of the robot
           with an added cost dimension, and finds higher quality
           solutions than its current best solution by only
           allowing connections that could result in an improved
           solution. This is done by sampling a cost for each new
           sample uniformly from a range that is both reachable
           and can improve upon the current solution, and uses
           this sampled cost as an upper bound for the cost of
           connections to this sample/
           @par External documentation
           J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc.
           2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI:
           [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
           T. S. Wilson, W. Thomason, Z. Kingston, and J. D. Gammell, AORRTC: Finding optimal paths with AO-x and
           RRT-Connect, in <em>Proc. Workshop on RoboARCH: Robotics Acceleration with Computing Hardware and Systems,
           IEEE Intl. Conf. on Robotics and Automation</em>, May 2025.
        */

        /** \brief Modified RRT-Connect for AORRTC (AOXRRTConnect) */
        class AOXRRTConnect : public ompl::geometric::RRTConnect
        {
        public:
            /** \brief Constructor */
            AOXRRTConnect(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);

            ~AOXRRTConnect() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void setFoundPath(const base::PathPtr &p)
            {
                foundPath = p;
            }

            base::PathPtr getFoundPath() const
            {
                return foundPath;
            }

            /** \brief Set a different nearest neighbors datastructure */
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

            void setup() override;

            void setPathCost(double pc);

        protected:
            /** \brief Representation of a motion */
            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                const base::State *root{nullptr};
                base::State *state{nullptr};
                Motion *parent{nullptr};
                double cost{0};
                bool connecting{false};
            };

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            using TreeData = std::shared_ptr<NearestNeighbors<Motion *>>;

            /** \brief Information attached to growing a tree of motions (used internally) */
            struct TreeGrowingInfo
            {
                base::State *xstate;
                Motion *xmotion;
                bool start;
            };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                auto space_diff = si_->distance(a->state, b->state);
                auto cost_diff = a->cost - b->cost;

                auto dist = sqrt(pow(space_diff, 2) + pow(cost_diff, 2));

                return dist;
            }

            /** \brief Find a valid neighbour with asymmetric distance funtion via iteration */
            Motion *findNeighbour(Motion *sampled_motion, float rootDist, TreeData &tree);

            /** \brief Grow a tree towards a random state */
            GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

            /** \brief State sampler */
            base::InformedSamplerPtr sampler_;

            /** \brief The start tree */
            TreeData tStart_;

            /** \brief The goal tree */
            TreeData tGoal_;

            /** \brief Maximum allowed cost resampling iterations */
            int maxResampleAttempts_{100};

            base::State *startState{nullptr};
            base::State *goalState{nullptr};

            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            base::PathPtr foundPath{nullptr};

            base::PlannerTerminationCondition _ptc{nullptr};

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
