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

#ifndef OMPL_BASE_OBJECTIVES_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/objectives/AccumulativeOptimizationObjective.h"

namespace ompl
{
    namespace base
    {

	/* \brief Definition of an optimization objective corresponding to minimizing path length, where length is defined by the sum of the distances between consecutive states of the path. */
        class PathLengthOptimizationObjective : public AccumulativeOptimizationObjective
        {
        public:
	    class CostType : public OptimizationObjective::CostType
	    {
	    public:
		CostType(double value = 0.0) : value_(value) {}
		virtual double getValue() const { return value_; }
		friend class PathLengthOptimizationObjective;
	    private:
		double value_;
	    };

            /** \brief Constructor. The objective must always know the space information it is part of */
            PathLengthOptimizationObjective(const SpaceInformationPtr &si, double maximumPathLength = std::numeric_limits<double>::infinity());

	    virtual bool isSatisfied(const Cost* cost) const;
	    virtual bool compareCost(const Cost* c1, const Cost* c2) const;
            virtual void getIncrementalCost(const State *s1, const State *s2, Cost* cost) const;
	    virtual void combineObjectiveCosts(const Cost* c1, const Cost* c2, Cost* cost) const;
	    virtual void getInitialCost(const State* s, Cost* cost) const;
	    virtual Cost* allocCost(void) const;
	    virtual void copyCost(Cost* dest, const Cost* src) const;
	    virtual void freeCost(Cost* cost) const;

	protected:
	    double maxPathLength_;
        };
    }
}

#endif
