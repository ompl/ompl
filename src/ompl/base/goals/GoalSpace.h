/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, University of Oxford.
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
*   * Neither the name of the University of Oxford nor the names of its
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

/* Author: Jonathan Gammell */

#ifndef OMPL_BASE_GOALS_GOAL_SPACE_
#define OMPL_BASE_GOALS_GOAL_SPACE_

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {
        /** \brief Definition of a goal space, i.e., a subspace of the problem state space that defines the goal. */
        class GoalSpace : public GoalSampleableRegion
        {
        public:
            /** \brief Create a goal representation that is a space */
            GoalSpace(const SpaceInformationPtr &si) : GoalSampleableRegion(si)
            {
                type_ = GOAL_SAMPLEABLE_REGION;
            }

            ~GoalSpace() override;

            /** \brief Sample a state in the goal region */
            void sampleGoal(State *st) const override;

            /** \brief Return the maximum number of samples that can be asked for before repeating */
            unsigned int maxSampleCount() const override;

            /** \brief Compute the distance to the goal (heuristic) */
            double distanceGoal(const State *st) const override;

            /** \brief Print information about the goal data structure
                to a stream */
            void print(std::ostream &out = std::cout) const override;

            /** \brief Set the goal space */
            void setSpace(const StateSpacePtr space);

            /** \brief Get the goal space */
            StateSpacePtr getSpace() const;

        protected:
            /** \brief The goal volume as defined by a state space. */
            StateSpacePtr goalSpace_;

            /** \brief The goal space sampler. */
            StateSamplerPtr goalSampler_;
        };
    }
}

#endif
