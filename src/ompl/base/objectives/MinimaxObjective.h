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

/* Author: Luis G. Torres */

#ifndef OMPL_BASE_OBJECTIVES_MINIMAX_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_MINIMAX_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief The cost of a path is defined as the worst state
            cost over the entire path. This objective attempts to find
            the path with the "best worst cost" over all paths. */
        class MinimaxObjective : public OptimizationObjective
        {
        public:
            MinimaxObjective(const SpaceInformationPtr &si);

            /** \brief Returns a cost with a value of 1. */
            Cost stateCost(const State *s) const override;

            /** \brief Interpolates between \e s1 and \e s2 to check for
                state costs along the motion between the two
                states. Assumes all costs are worse than identity */
            Cost motionCost(const State *s1, const State *s2) const override;

            /** \brief Since we're only concerned about the "worst"
                state cost in the path, combining two costs just
                returns the worse of the two. */
            Cost combineCosts(Cost c1, Cost c2) const override;
        };
    }
}

#endif
