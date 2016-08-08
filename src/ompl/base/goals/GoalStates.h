/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_GOALS_GOAL_STATES_
#define OMPL_BASE_GOALS_GOAL_STATES_

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/ScopedState.h"
#include <vector>

namespace ompl
{
    namespace base
    {
        /** \brief Definition of a set of goal states */
        class GoalStates : public GoalSampleableRegion
        {
        public:
            /** \brief Create a goal representation that is in fact a set of states  */
            GoalStates(const SpaceInformationPtr &si) : GoalSampleableRegion(si), samplePosition_(0)
            {
                type_ = GOAL_STATES;
            }

            ~GoalStates() override;

            void sampleGoal(State *st) const override;

            unsigned int maxSampleCount() const override;

            double distanceGoal(const State *st) const override;

            void print(std::ostream &out = std::cout) const override;

            /** \brief Add a goal state */
            virtual void addState(const State *st);

            /** \brief Add a goal state (calls the previous definition of addState())*/
            void addState(const ScopedState<> &st);

            /** \brief Clear all goal states */
            virtual void clear();

            /** \brief Check if there are any states in this goal region */
            virtual bool hasStates() const;

            /** \brief Return a pointer to the indexth state in the state list */
            virtual const State *getState(unsigned int index) const;

            /** \brief Return the number of valid goal states */
            virtual std::size_t getStateCount() const;

        protected:
            /** \brief The goal states. Only ones that are valid are considered by the motion planner. */
            std::vector<State *> states_;

        private:
            /** \brief The index of the next sample to be returned  */
            mutable unsigned int samplePosition_;

            /** \brief Free allocated memory */
            void freeMemory();
        };
    }
}

#endif
