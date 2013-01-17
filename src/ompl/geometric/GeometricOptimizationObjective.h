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

#ifndef OMPL_GEOMETRIC_GEOMETRIC_OPTIMIZATION_OBJECTIVE_
#define OMPL_GEOMETRIC_GEOMETRIC_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace geometric
    {
        /** \brief Representation of optimization objectives that are accumulative, i.e. if a path is a sequence of states (q_1, ..., q_n), then the cost of a path c((q_1,...,q_n)) can be defined as c((q_1,...,q_{n-1})) combined with c((q_{n-1}, q_n)). The optimization criteria is met if the cost of the path is below a given upper bound. */
        class BoundedAccumulativeOptimizationObjective : public base::OptimizationObjective
        {
        public:
	    class CostType : public OptimizationObjective::CostType
	    {
	    public:
		virtual double getValue(void) const = 0;
	    };

            /** \brief Constructor. The objective must always know the space information it is part of */
            BoundedAccumulativeOptimizationObjective(const base::SpaceInformationPtr &si, double maximumUpperBound) :
                OptimizationObjective(si),
                maximumUpperBound_(maximumUpperBound)
            {
            }

	    /** \brief Computes cost of an entire path by accumulating cost from the the start state to the end state. */
            virtual void getCost(const base::PathPtr &path, base::Cost* cost) const;

            /** \brief Get the cost that corresponds to the motion segment between \e s1 and \e s2 */
            virtual void getIncrementalCost(const base::State *s1, const base::State *s2, base::Cost* cost) const = 0;

            /** \brief Get the cost that corresponds to combining the costs \e c1 and \e c2 */
            virtual void combineObjectiveCosts(const base::Cost* c1, const base::Cost* c2, base::Cost* cost) const = 0;

	    /** \brief Get the cost corresponding to the beginning of a path that starts at \e s. */
	    virtual void getInitialCost(const base::State* s, base::Cost* cost) const = 0;

	    /** \brief Check if this objective has a symmetric cost metric, i.e. getIncrementalCost(s1, s2) = getIncrementalCost(s2, s1). Default implementation returns whether the underlying state space has symmetric interpolation. */
	    virtual bool isSymmetric(void) const;

            /** \brief Get the maximum upper bound for the objective cost that is to be accepted as satisfactory */
            double getMaximumUpperBound(void) const
            {
                return maximumUpperBound_;
            }

            /** \brief Set the maximum upper bound for the objective cost that is to be accepted as satisfactory.
                The default value is usually infinity. */
            void setMaximumUpperBound(double maximumUpperBound)
            {
                maximumUpperBound_ = maximumUpperBound;
            }

            virtual bool isSatisfied(const base::Cost* cost) const;

            // virtual double getTerminalCost(const State *s) const;

	    virtual bool compareCost(const base::Cost* c1, const base::Cost* c2) const;

        protected:

            /** \brief The maximum upper bound for the objective cost that is to be accepted as satisfactory */
            double  maximumUpperBound_;
        };

	/* \brief Definition of an optimization objective corresponding to minimizing path length, where length is defined by the sum of the distances between consecutive states of the path. */
        class PathLengthOptimizationObjective : public BoundedAccumulativeOptimizationObjective
        {
        public:
	    class CostType : public BoundedAccumulativeOptimizationObjective::CostType
	    {
	    public:
		CostType(double value = 0.0) : value_(value) {}
		virtual double getValue() const { return value_; }
		friend class PathLengthOptimizationObjective;
	    private:
		double value_;
	    };

            /** \brief Constructor. The objective must always know the space information it is part of */
            PathLengthOptimizationObjective(const base::SpaceInformationPtr &si, double maximumPathLength = std::numeric_limits<double>::infinity());

            virtual void getIncrementalCost(const base::State *s1, const base::State *s2, base::Cost* cost) const;
	    virtual void combineObjectiveCosts(const base::Cost* c1, const base::Cost* c2, base::Cost* cost) const;
	    virtual void getInitialCost(const base::State* s, base::Cost* cost) const;
	    virtual base::Cost* allocCost(void) const;
	    virtual void copyCost(base::Cost* dest, const base::Cost* src) const;
	    virtual void freeCost(base::Cost* cost) const;
        };

        class StateCostOptimizationObjective : public BoundedAccumulativeOptimizationObjective
        {
        public:
	    class CostType : public BoundedAccumulativeOptimizationObjective::CostType
	    {
	    public:
		CostType(double value = 0.0) : value_(value) {}
		virtual double getValue() const { return value_; }
		friend class StateCostOptimizationObjective;
	    private:
		double value_;
	    };

            /** \brief Constructor. The objective must always know the space information it is part of */
            StateCostOptimizationObjective(const base::SpaceInformationPtr &si, double maximumCostSum = std::numeric_limits<double>::infinity());

            virtual void getIncrementalCost(const base::State *s1, const base::State *s2, base::Cost* cost) const;
	    virtual void combineObjectiveCosts(const base::Cost* c1, const base::Cost* c2, base::Cost* cost) const;
	    virtual void getInitialCost(const base::State* s, base::Cost* cost) const;
	    virtual base::Cost* allocCost(void) const;
	    virtual void copyCost(base::Cost* dest, const base::Cost* src) const;
	    virtual void freeCost(base::Cost* cost) const;
        };
    }
}

#endif
