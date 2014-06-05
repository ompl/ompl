/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Oren Salzman
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

/* Author: Oren Salzman, Sertac Karaman, Ioan Sucan */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_LBT_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_LBT_RRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <fstream>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gLBTRRT
           @par Short description
           \ref gLBTRRT "LBTRRT" (Lower Bound Tree RRT) is a near asymptotically-optimal
           incremental sampling-based motion planning algorithm. \ref gLBTRRT "LBTRRT"
           algorithm is guaranteed to converge to a solution that is within a constant
           factor of the optimal solution. The notion of optimality is with respect to
           the distance function defined on the state space we are operating on.

           @par External documentation
           O. Salzman and D. Halperin, Sampling-based
           Asymptotically near-optimal RRT for fast, high-quality, motion planning, 2013.
           [[PDF]](http://arxiv.org/abs/1308.0189)
        */

        /** \brief Lower Bound Tree Rapidly-exploring Random Trees */
        class LBTRRT : public base::Planner
        {
        public:

            /** \brief Constructor */
            LBTRRT (const base::SpaceInformationPtr &si);

            virtual ~LBTRRT (void);

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

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
            double getGoalBias(void) const
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
            double getRange(void) const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Motion*>());
            }

            virtual void setup(void);

            /** \brief Set the apprimation factor */
            void setApproximationFactor (double epsilon)
            {
                epsilon_ = epsilon;
            }

            /** \brief Get the apprimation factor */
            double getApproximationFactor (void) const
            {
                return epsilon_;
            }
        protected:

            /** \brief kRRG = 2e~5.5 is a valid choice for all problem instances */
            static const double kRRG; // = 5.5

            /** \brief Representation of a motion

                a motion is a simultunaeous represntation of the two trees used by LBT-RRT
                a lower bound tree named Tlb and an approximaion tree named Tapx. */
            class Motion
            {
            public:

                Motion(void) : state(NULL), parentLb_(NULL), parentApx_(NULL), costLb_(0.0), costApx_(0.0)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parentLb_(NULL), parentApx_(NULL), costLb_(0.0), costApx_(0.0)
                {
                }

                ~Motion(void)
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parentLb_;

                /** \brief The parent motion in the exploration tree */
                Motion            *parentApx_;

                double             costLb_, costApx_;

                std::vector<Motion*> childrenLb_;
                std::vector<Motion*> childrenApx_;
            };

            struct IsLessThan
            {
                IsLessThan (LBTRRT *plannerPtr, Motion * motion_): plannerPtr_(plannerPtr), motion(motion_)
                {
                }

                bool operator() (const Motion * motionA, const Motion * motionB)
                {
                    double sqDistA = plannerPtr_->distanceFunction(motionA, motion);
                    double distA = std::sqrt(sqDistA);

                    double sqDistB = plannerPtr_->distanceFunction(motionB, motion);
                    double distB = std::sqrt(sqDistB);

                    return (motionA->costLb_ + distA < motionB->costLb_ + distB);
                }
                LBTRRT *plannerPtr_;
                Motion *motion;
            }; //IsLessThan

            /** \brief attempt to rewire the trees */
            void attemptNodeUpdate(Motion *potentialParent, Motion *child);

            /** \brief update the child cost of the lower bound tree */
            void updateChildCostsLb(Motion *m, double delta);

            /** \brief update the child cost of the approximation tree */
            void updateChildCostsApx(Motion *m, double delta);

            /** \brief remove motion from its parent in the lower bound tree*/
            void removeFromParentLb(Motion *m);

            /** \brief remove motion from its parent in the approximation tree*/
            void removeFromParentApx(Motion *m);

            /** \brief remove motion from a vector*/
            void removeFromParent(const Motion *m, std::vector<Motion*>& vec);

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief approximation factor*/
            double                                          epsilon_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;
        };

    }
}

#endif //OMPL_GEOMETRIC_PLANNERS_RRT_LBT_RRT_
