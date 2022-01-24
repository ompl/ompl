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

/* Author: Wolfgang Hönig */

#ifndef OMPL_BASE_OBJECTIVES_CONTROL_DURATION_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_CONTROL_DURATION_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/control/SpaceInformation.h"

namespace ompl
{
    namespace base
    {
        /** \brief Defines optimization objectives where the total
            time of a control action is summed. This cost function is
            specified by implementing the controlCost() method. */
        class ControlDurationObjective : public OptimizationObjective
        {
        public:
            /** \brief Requires a control::SpaceInformationPtr to access \p dt. */
            ControlDurationObjective(const control::SpaceInformationPtr &si);

            /** \brief Returns a cost with a value of 0. */
            Cost stateCost(const State *s) const override;

            /** \brief Returns a cost with a value of 0. */
            Cost motionCost(const State *s1, const State *s2) const override;

            /** \brief Returns the cost with the value of steps*dt. */
            Cost controlCost(const control::Control *c, unsigned int steps) const override;

        protected:
            /** \brief Duration of each control step. */
            double dt_;
        };
    }
}

#endif
