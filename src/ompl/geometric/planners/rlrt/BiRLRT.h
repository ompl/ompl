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
 *   * Neither the name of Rice University nor the names of its
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

#ifndef OMPL_GEOMETRIC_PLANNERS_RLRT_BIRLRT_H_
#define OMPL_GEOMETRIC_PLANNERS_RLRT_BIRLRT_H_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <vector>

namespace ompl
{
    namespace geometric
    {
        /**
        \anchor gBiRLRT

        \ref gBiRLRT "BiRLRT" is a basic bidirectional tree-based planner without any sophistic heuristics to guide the
        exploration. It should be used as a baseline for comparison against other bidirectional tree-based planners. In
        high-dimensional search spaces it can sometimes perform surprisingly well.

        \par Associated publication:
        R. Luna, M. Moll, J. Badger, and L. E. Kavraki,
        A Scalable Motion Planner for High-Dimensional Kinematic Systems,
        <em>Intl. J. of Robotics Research</em>, vol. 39, issue 4, pp. 361-388, Mar. 2020.
        DOI: [10.1177/0278364919890408](http://dx.doi.org/10.1177/0278364919890408)<br>
        [[PDF]](http://www.kavrakilab.org/publications/luna2020a-scalable-motion-planner-for-high-dimensional.pdf)
        **/

        /// \brief Bi-directional Range-Limited Random Tree (Ryan Luna's Random Tree)
        class BiRLRT : public base::Planner
        {
        public:
            BiRLRT(const base::SpaceInformationPtr &si);

            virtual ~BiRLRT();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            /// \brief Set the maximum distance between states in the tree.
            void setRange(double distance)
            {
                range_ = distance;
            }

            /// \brief Get the maximum distance between states in the tree.
            double getRange() const
            {
                return range_;
            }

            /// \brief Set the maximum distance (per dimension) when sampling
            /// near an existing state
            void setMaxDistanceNear(double dNear)
            {
                maxDistNear_ = dNear;
            }

            /// \brief Get the maximum distance (per dimension) when sampling
            /// near an existing state
            double getMaxDistanceNear() const
            {
                return maxDistNear_;
            }

            /// \brief If true, the planner will not have the range limitation.
            /// Instead, if a collision is detected, the last valid state is
            /// retained.
            bool getKeepLast() const
            {
                return keepLast_;
            }

            /// \brief Set whether the planner will use the range or keep last
            /// heuristic.  If keepLast = false, motions are limited in
            /// distance to range_, otherwise the last valid state is retained
            /// when a collision is detected.
            void setKeepLast(bool keepLast)
            {
                keepLast_ = keepLast;
            }

            virtual void setup();

        protected:
            /// A motion (tree node) with parent pointer
            class Motion
            {
            public:
                /// \brief Constructor that allocates memory for the state
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /// \brief The state contained by the motion
                base::State *state{nullptr};

                /// \brief The parent motion in the exploration tree
                Motion *parent{nullptr};

                /// \brief Pointer to the root of the tree this motion is connected to
                const base::State *root{nullptr};
            };

            /// \brief Free the memory allocated by this planner
            void freeMemory();

            /// Try to grow the tree randomly.  Return true if a new state was added
            /// xmotion is scratch space for sampling, etc.
            bool growTreeRangeLimited(std::vector<Motion *> &tree, Motion *xmotion);

            /// Try to grow the tree randomly.  Return true if a new state was added
            /// xmotion is scratch space for sampling, etc.
            bool growTreeKeepLast(std::vector<Motion *> &tree, Motion *xmotion,
                                  std::pair<base::State *, double> &lastValid);

            /// Attempt to connect the given motion (presumed to be in a tree)
            /// to a state in another tree (presumed to be different from the
            /// tree motion is in). If connection is successful, the index of
            /// the motion in the other tree that the motion connects to is
            /// returned.  -1 for failed connection.
            int connectToTree(const Motion *motion, std::vector<Motion *> &tree);

            /// Start tree
            std::vector<Motion *> tStart_;
            /// Goal tree
            std::vector<Motion *> tGoal_;

            /// \brief State sampler.
            base::StateSamplerPtr sampler_;

            /// \brief The maximum total length of a motion to be added to a tree
            double range_{0.0};

            /// \brief The maximum distance (per dimension) when sampling near
            /// an existing configuration
            double maxDistNear_{0.0};

            /// \brief The random number generator
            RNG rng_;

            /// \brief The pair of states in each tree connected during
            /// planning.  Used for PlannerData computation
            std::pair<base::State *, base::State *> connectionPoint_{nullptr, nullptr};

            bool keepLast_{false};
        };

    }  // namespace geometric
}  // namespace ompl

#endif