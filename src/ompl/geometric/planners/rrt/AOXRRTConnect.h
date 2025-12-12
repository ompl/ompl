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
           T. S. Wilson, W. Thomason, Z. Kingston, and J. D. Gammell, AORRTC: Almost-surely asymptotically optimal
           planning with RRT-Connect, in <em>IEEE Robotics and Automation Letters</em>, Vol. 10, no. 12, pp. 13375-13382, Dec. 2025. DOI:
           [10.1109/LRA.2025.3615522](http://dx.doi.org/10.1109/LRA.2025.3615522)<br>
           [[PDF]](https://arxiv.org/abs/2505.10542)
        */

        /** \brief Modified RRT-Connect for AORRTC (AOXRRTConnect) */
        class AOXRRTConnect : public base::Planner
        {
        public:
            /** \brief Constructor */
            AOXRRTConnect(const base::SpaceInformationPtr &si);

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
                if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0))
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                tStart_ = std::make_shared<NN<Motion *>>();
                tGoal_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            /** \brief Check if the inner loop planner met its  condition to terminate */
            bool internalResetCondition()
            {
                bool shouldReset = false;
                if (tStart_ && tGoal_) {
                    /* Reset if we have met our maximum internal vertices */
                    shouldReset = shouldReset || (tStart_->size() + tGoal_->size() >= maxInternalVertices);
                } else {
                    /* If our trees don't exist, we're not in the middle of a search anyways */
                    shouldReset = true;
                }
                /* Reset if we have attempted our maximum internal samples */
                shouldReset = shouldReset || (sampleAttempts >= maxInternalSamples);
                return shouldReset;
            }

            void setup() override;

            void reset(bool solvedProblem);

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

            /** \brief The state of the tree after an attempt to extend it */
            enum GrowState
            {
                /* no progress has been made */
                TRAPPED,
                /* progress has been made towards the randomly sampled state */
                ADVANCED,
                /* the randomly sampled state was reached */
                REACHED
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute euclidian distance between motions */
            double euclideanDistanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Compute AOX distance between motions */
            double aoxDistanceFunction(const Motion *a, const Motion *b) const
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

            std::size_t sampleAttempts{0};

            /* Pad rootDist to account for floating point error
               Needed to make sure the root is included in nearest list
               TODO: Should use some relative epsilon for padding 
               (FLT_EPSILON is good but does not scale with the magnitude of rootDist and may be too small) */
            const float rootDistPadding = 0.00001;

            /** \brief The start tree */
            TreeData tStart_;

            /** \brief The goal tree */
            TreeData tGoal_;

            /** \brief A flag that toggles between expanding the start tree (true) or goal tree (false). */
            bool startTree_{true};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Maximum allowed cost resampling iterations before moving on */
            long int maxResampleAttempts_{100};

            /** \brief Maximum allowed total vertices in trees before the search is restarted */
            std::size_t maxInternalVertices{10000};

            /** \brief Increment by which maxVertices is increased */
            std::size_t maxInternalVerticesIncrement{10000};

            /** \brief Maximum samples tried before the search is restarted */
            std::size_t maxInternalSamples{10000000};
            
            /** \brief Increment by which maxSamples is increased */
            std::size_t maxInternalSamplesIncrement{10000000};

            base::State *startState{nullptr};
            base::State *goalState{nullptr};

            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::infinity()};

            /** \brief Path found by the algorithm */
            base::PathPtr foundPath{nullptr};

            /** \brief Outer loop termination condition for AORRTC */
            base::PlannerTerminationCondition _ptc{nullptr};

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State *, base::State *> connectionPoint_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
