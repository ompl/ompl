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

/* Author: Luis G. Torres, Jonathan Gammell (allocInformedStateSampler) */

#ifndef OMPL_BASE_OBJECTIVES_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective which corresponds to optimizing path length. */
        class PathLengthOptimizationObjective : public OptimizationObjective
        {
        public:
            PathLengthOptimizationObjective(const SpaceInformationPtr &si);

            /** \brief Returns identity cost. */
            Cost stateCost(const State *s) const override;

            /** \brief Motion cost for this objective is defined as
                the configuration space distance between \e s1 and \e
                s2, using the method SpaceInformation::distance(). */
            Cost motionCost(const State *s1, const State *s2) const override;

            /** \brief the motion cost heuristic for this objective is
                simply the configuration space distance between \e s1
                and \e s2, since this is the optimal cost between any
                two states assuming no obstacles. */
            Cost motionCostHeuristic(const State *s1, const State *s2) const override;

            /** \brief the best motion cost estimate for this objective is
                simply the configuration space distance between \e s1
                and \e s2, since this is the optimal cost between any
                two states assuming no obstacles. */
            Cost motionCostBestEstimate(const State *s1, const State *s2) const override;

            /** \brief Allocate a state sampler for the path-length objective (i.e., direct ellipsoidal sampling). */
            InformedSamplerPtr allocInformedStateSampler(const ProblemDefinitionPtr &probDefn,
                                                         unsigned int maxNumberCalls) const override;
        };
    }
}

#endif
