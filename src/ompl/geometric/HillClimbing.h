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

#ifndef OMPL_GEOMETRIC_HILL_CLIMBING_
#define OMPL_GEOMETRIC_HILL_CLIMBING_

#include <utility>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/goals/GoalRegion.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor HillClimbing

           @par Short description

           HillClimbing searches for a state using hill climbing, starting from a given seed state.

           @par External documentation
        */

        /** \brief Hill Climbing search */
        class HillClimbing
        {
        public:
            /** \brief Constructor */
            HillClimbing(base::SpaceInformationPtr si) : si_(std::move(si)), maxImproveSteps_(2), checkValidity_(true)
            {
            }

            ~HillClimbing() = default;

            /** \brief Try to improve a state (reduce distance to goal). The updates are performed by sampling near the
                state, within the specified distance. If improvements were found, the function returns true and the
               better
                goal distance is optionally returned */
            bool tryToImprove(const base::GoalRegion &goal, base::State *state, double nearDistance,
                              double *betterGoalDistance = nullptr) const;

            /** \brief Set the number of steps to perform */
            void setMaxImproveSteps(unsigned int steps)
            {
                maxImproveSteps_ = steps;
            }

            /** \brief Get the number of steps to perform */
            unsigned int getMaxImproveSteps() const
            {
                return maxImproveSteps_;
            }

            /** \brief Set the state validity flag; if this is false, states are not checked for validity */
            void setValidityCheck(bool valid)
            {
                checkValidity_ = valid;
            }

            /** \brief Get the state validity flag; if this is false, states are not checked for validity */
            bool getValidityCheck() const
            {
                return checkValidity_;
            }

        private:
            bool valid(const base::State *state) const
            {
                return checkValidity_ ? si_->isValid(state) : true;
            }

            base::SpaceInformationPtr si_;
            unsigned int maxImproveSteps_;
            bool checkValidity_;
        };
    }
}

#endif
