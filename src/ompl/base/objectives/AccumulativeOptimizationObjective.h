/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Authors: Ioan Sucan, Luis G. Torres */

#ifndef OMPL_BASE_OBJECTIVES_ACCUMULATIVE_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_ACCUMULATIVE_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief Representation of optimization objectives that are accumulative, i.e. if a path is a sequence of states (q_1, ..., q_n), then the cost of a path c((q_1,...,q_n)) can be defined as c((q_1,...,q_{n-1})) combined with c((q_{n-1}, q_n)). */
        class AccumulativeOptimizationObjective : public OptimizationObjective
        {
        public:
            /** \brief Constructor. The objective must always know the space information it is part of */
            AccumulativeOptimizationObjective(const SpaceInformationPtr &si) :
                OptimizationObjective(si)
            {
            }

	    /** \brief Computes cost of an entire path by accumulating cost from the the start state to the end state. This default implementation assumes \e path is a PathGeometric. */
            virtual void getCost(const PathPtr &path, Cost* cost) const;

            /** \brief Get the cost that corresponds to the motion segment between \e s1 and \e s2 */
            virtual void getIncrementalCost(const State *s1, const State *s2, Cost* cost) const = 0;

            /** \brief Get the cost that corresponds to combining the costs \e c1 and \e c2. Implementations of this method should allow for \e c1 and \e cost to point to the same memory location. */
            virtual void combineObjectiveCosts(const Cost* c1, const Cost* c2, Cost* cost) const = 0;

	    /** \brief Get the cost corresponding to the beginning of a path that starts at \e s. */
	    virtual void getInitialCost(const State* s, Cost* cost) const = 0;

            // virtual double getTerminalCost(const State *s) const;

	    /** \brief Check if this objective has a symmetric cost metric, i.e. getIncrementalCost(s1, s2) = getIncrementalCost(s2, s1). Default implementation returns whether the underlying state space has symmetric interpolation. */
	    virtual bool isSymmetric(void) const;
        };
    }
}

#endif
