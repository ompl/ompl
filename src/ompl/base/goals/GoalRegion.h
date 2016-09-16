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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_GOALS_GOAL_REGION_
#define OMPL_BASE_GOALS_GOAL_REGION_

#include "ompl/base/Goal.h"

namespace ompl
{
    namespace base
    {
        /** \brief Definition of a goal region */
        class GoalRegion : public Goal
        {
        public:
            /** \brief Create a goal region */
            GoalRegion(const SpaceInformationPtr &si);

            ~GoalRegion() override = default;

            /** \brief Equivalent to calling isSatisfied(const State *, double *) with a nullptr second argument. */
            bool isSatisfied(const State *st) const override;

            /** \brief Decide whether a given state is part of the
                goal region. Returns true if the distance to goal is
                less than the threshold (using distanceGoal()) */
            bool isSatisfied(const State *st, double *distance) const override;

            /** \brief Compute the distance to the goal
                (heuristic). This function is the one used in
                computing the distance to the goal in a call to
                isSatisfied() */
            virtual double distanceGoal(const State *st) const = 0;

            /** \brief Print information about the goal data structure
                to a stream */
            void print(std::ostream &out = std::cout) const override;

            /** \brief Set the distance to the goal that is allowed
                for a state to be considered in the goal region */
            void setThreshold(double threshold)
            {
                threshold_ = threshold;
            }

            /** \brief Get the distance to the goal that is allowed
                for a state to be considered in the goal region */
            double getThreshold() const
            {
                return threshold_;
            }

        protected:
            /** \brief The maximum distance that is allowed to the
                goal. By default, this is initialized to the minimum
                epsilon value a double can represent */
            double threshold_;
        };
    }
}

#endif
