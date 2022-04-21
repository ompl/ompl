/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

#ifndef OMPL_BASE_GOAL_TYPES_
#define OMPL_BASE_GOAL_TYPES_

namespace ompl
{
    namespace base
    {
        /** \brief The type of goal */
        enum GoalType
        {
            /** \brief This bit is set if casting to generic goal regions (ompl::base::Goal) is possible. This bit shold
               always be set */
            GOAL_ANY = 1,

            /** \brief This bit is set if casting to goal regions (ompl::base::GoalRegion) is possible */
            GOAL_REGION = GOAL_ANY + 2,

            /** \brief This bit is set if casting to sampleable goal regions (ompl::base::GoalSampleableRegion) is
               possible */
            GOAL_SAMPLEABLE_REGION = GOAL_REGION + 4,

            /** \brief This bit is set if casting to goal state (ompl::base::GoalState) is possible */
            GOAL_STATE = GOAL_SAMPLEABLE_REGION + 8,

            /** \brief This bit is set if casting to goal states (ompl::base::GoalStates) is possible */
            GOAL_STATES = GOAL_SAMPLEABLE_REGION + 16,

            /** \brief This bit is set if casting to goal states (ompl::base::GoalLazySamples) is possible */
            GOAL_LAZY_SAMPLES = GOAL_STATES + 32,

            /** \brief This bit is set if casting to goal space (ompl::base::GoalSpace) is possible */
            GOAL_SPACE = GOAL_LAZY_SAMPLES + 64
        };
    }
}

#endif
