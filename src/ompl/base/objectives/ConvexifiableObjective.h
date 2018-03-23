/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

/* Author: Bryce Willey */

#ifndef OMPL_BASE_OBJECTIVES_CONVEXIFIABLE_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_CONVEXIFIABLE_OBJECTIVE_

#include "ompl/base/objectives/ConvexifiableOptimization.h"

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective. When added to the SQP Model (addToModel), it will add
         *  itself as a cost.
         *  TODO: add the references in this description.
         */
        class ConvexifiableObjective : public ConvexifiableOptimization
        {
        public:
            ConvexifiableObjective(const SpaceInformationPtr &si) : ConvexifiableOptimization(si) {}

            /** \brief Adds itself to the current optimization problem definition as a
                cost to be optimized (as opposed to a hard constraint). */
            void addToProblem(sco::OptProbPtr problem)
            {
                problem->addCost(toCost(problem));
            }

        //protected:
            /** \brief Turns this optimization objective into a convexifiable cost.
                Must be implemented by each subclass individually. */
            virtual sco::CostPtr toCost(sco::OptProbPtr problem) = 0;
        };
    }
}

#endif
