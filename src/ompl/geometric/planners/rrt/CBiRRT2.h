/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_CBIRRT2_
#define OMPL_GEOMETRIC_PLANNERS_RRT_CBIRRT2_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include "ompl/base/ConstrainedSpaceInformation.h"

namespace ompl
{

    namespace geometric
    {

        /// \brief An Bi-directional RRT that respects constraints in the state space
        class CBiRRT2 : public base::Planner
        {
        public:

            /** \brief Constructor */
            CBiRRT2(const base::SpaceInformationPtr &si);

            virtual ~CBiRRT2(void);

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

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
                tStart_.reset(new NN<Motion*>());
                tGoal_.reset(new NN<Motion*>());
            }

            virtual void setup(void);

        protected:

            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:

                Motion(void) : root(nullptr), state(nullptr), parent(nullptr)
                {
                }

                Motion(const base::SpaceInformationPtr &si) : root(nullptr), state(si->allocState()), parent(nullptr)
                {
                }

                Motion(base::State* st) : root(nullptr), state(st), parent(nullptr)
                {
                }

                ~Motion(void)
                {
                }

                const base::State *root;
                base::State       *state;
                const Motion      *parent;

            };

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            typedef std::shared_ptr< NearestNeighbors<Motion*> > TreeData;

            /** \brief Information attached to growing a tree of motions (used internally) */
            struct TreeGrowingInfo
            {
                base::State         *xstate;
                Motion              *xmotion;
                bool                 start;
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

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion* a, const Motion* b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Grow a tree from nearMotion towards randMotion */
            GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, const Motion *randMotion, const Motion *nearMotion);

            /// \brief Starting at a, interpolate toward b along the constraint manifold.
            /// Store the resulting states in result.
            bool constrainedExtend(const base::State* a, const base::State* b,
                                   std::vector<base::State*>& result) const;

            bool shortcutPath(PathGeometric *path, const base::PlannerTerminationCondition &ptc);

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief The start tree */
            TreeData                                       tStart_;

            /** \brief The goal tree */
            TreeData                                       tGoal_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State*, base::State*>          connectionPoint_;

            /// \brief A pointer to the constraint information object that this planner uses
            base::ConstraintInformationPtr                 ci_;

            /** \brief Distance between the nearest pair of start tree and goal tree nodes. */
            double                        distanceBetweenTrees_;
        };

    }
}

#endif
