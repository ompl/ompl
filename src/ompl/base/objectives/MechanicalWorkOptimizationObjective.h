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

#ifndef OMPL_BASE_OBJECTIVES_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective which defines path cost using the idea of mechanical work. To be used in
         * conjunction with TRRT. */
        class MechanicalWorkOptimizationObjective : public OptimizationObjective
        {
        public:
            /** \brief The mechanical work formulation requires a weighing factor to use for the length of a path in
             * order to disambiguate optimal paths. This weighing factor should be small. The default value for this
             * weight is 0.00001. */
            MechanicalWorkOptimizationObjective(const SpaceInformationPtr &si, double pathLengthWeight = 0.00001);

            /** \brief Set the factor to use for weighing path length in the mechanical work objective formulation. */
            virtual double getPathLengthWeight() const;

            /** \brief Returns a cost with a value of 1. */
            Cost stateCost(const State *s) const override;

            /** \brief Defines motion cost in terms of the mechanical work formulation used for TRRT. */
            Cost motionCost(const State *s1, const State *s2) const override;

        protected:
            /** \brief The weighing factor for the path length in the mechanical work objective formulation. */
            double pathLengthWeight_;
        };
    }
}

#endif
