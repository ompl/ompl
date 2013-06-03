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

/* Author: Ioan Sucan, Luis G. Torres */

#ifndef OMPL_BASE_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>
#include <boost/concept_check.hpp>
#include <limits>

namespace ompl
{
    namespace base
    {   
	/** \brief Definition of an abstract cost. */
	class Cost 
	{
	public:
	    /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as(void) const
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Cost*>));

                return static_cast<const T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as(void)
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Cost*>));

                return static_cast<T*>(this);
            }
	protected:
	    Cost(void) {}
	    virtual ~Cost(void) {}
	private:
	    /** \brief Disable copy-constructors */
	    Cost(const Cost&);
	    const Cost& operator=(const Cost&);
	};


        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::OptimizationObjective */
        OMPL_CLASS_FORWARD(OptimizationObjective);
        /// @endcond

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Path);
        /// @endcond	

        /** \class ompl::base::OptimizationObjectivePtr
            \brief A boost shared pointer wrapper for ompl::base::OptimizationObjective */

        /** \brief Abstract definition of optimization objectives.

            \note This implementation has greatly benefited from discussions with <a href="http://www.cs.indiana.edu/~hauserk/">Kris Hauser</a> */
        class OptimizationObjective : private boost::noncopyable
        {
        public:
	    typedef Cost CostType;

            /** \brief Constructor. The objective must always know the space information it is part of */
            OptimizationObjective(const SpaceInformationPtr &si) : si_(si)
	    {
	    }

            virtual ~OptimizationObjective(void)
            {
            }

            /** \brief Get the description of this optimization objective */
            const std::string& getDescription(void) const
            {
                return description_;
            }

            /** \brief Verify that our objective is satisfied already and we can stop planning */
            virtual bool isSatisfied(const Cost *cost) const = 0;

	    /** \brief Get \e cost as a double value. This is what is used for comparison by isCostLessThan(). */
	    virtual double getCostValue(const Cost *cost) const = 0;

            /** \brief Get the cost that corresponds to an entire path. This implementation assumes \e Path is of type \e PathGeometric.*/
            virtual void getCost(const Path &path, Cost *cost) const;

	    /** \brief Check whether the the cost \e c1 is considered less than the cost \e c2. */
	    bool isCostLessThan(const Cost *c1, const Cost *c2) const;
	    
	    /** \brief Allocate storage for a cost calculation in this OptimizationObjective. */
	    virtual Cost* allocCost(void) const = 0;

	    /** \brief Copy one cost into another. Memory for source and destination should NOT overlap. */
	    virtual void copyCost(Cost *dest, const Cost *src) const = 0;

	    /** \brief Free the memory of the allocated cost. */
	    virtual void freeCost(Cost *cost) const = 0;

	    /** \brief Evaluate a cost map defined on the state space at a state \e s. Default implementation maps all states to 1.0. */
	    virtual double getStateCost(const State *s) const;

	    /** \brief Get the cost that corresponds to the motion segment between \e s1 and \e s2 */
            virtual void getIncrementalCost(const State *s1, const State *s2, Cost *cost) const = 0;

            /** \brief Get the cost that corresponds to combining the costs \e c1 and \e c2. Implementations of this method should allow for \e c1 and \e cost to point to the same memory location. */
            virtual void combineObjectiveCosts(const Cost *c1, const Cost *c2, Cost *cost) const = 0;

	    /** \brief Get the cost corresponding to the beginning of a path that starts at \e s. */
	    virtual void getInitialCost(const State *s, Cost *cost) const = 0;

	    /** \brief Get a cost which is greater than all other costs in this OptimizationObjective; required for use in Dijkstra/Astar. */
	    virtual void getInfiniteCost(Cost *cost) const = 0;

	    /** \brief Check if this objective has a symmetric cost metric, i.e. getIncrementalCost(s1, s2) = getIncrementalCost(s2, s1). Default implementation returns whether the underlying state space has symmetric interpolation. */
	    virtual bool isSymmetric(void) const;

	    /** \brief Compute the average state cost of this objective by taking a sample of \e numStates states */
	    virtual double averageStateCost(unsigned int numStates) const;

        protected:
            /** \brief The space information for this objective */
            SpaceInformationPtr si_;

            /** \brief The description of this optimization objective */
            std::string description_;
        };

	class PathIntegralOptimizationObjective : public OptimizationObjective
	{
	public:
	    class CostType : public OptimizationObjective::CostType
	    {
	    public:
		CostType(double valueIn = 0.0) : value(valueIn) {}
		double value;
	    };
	
	    PathIntegralOptimizationObjective(const SpaceInformationPtr &si, double maxPathCost = std::numeric_limits<double>::epsilon());

	    double getMaxPathCost(void) const;

	    void setMaxPathCost(double maxPathCost);

	    double getCostValue(const Cost *cost) const;

	    virtual void getCost(const Path &path, Cost *cost) const;

	    /** \brief Get the value of the cost that corresponds to an entire path. */
	    virtual double getCost(const Path &path) const;

	    /** \brief Returns true if the value of \e cost is less than or equal to \e maxPathCost_ */
	    virtual bool isSatisfied(const Cost *cost) const;

	    /** \brief Compute the cost of a path segment from \e s1 to \e s2 (including endpoints)
		\param s1 start state of the motion to be evaluated
		\param s2 final state of the motion to be evaluated
		\param cost the cost of the motion segment

		By default, this function computes
		\f{eqnarray*}{
		\mbox{cost} &=& \frac{cost(s_1) + cost(s_2)}{2}\vert s_1 - s_2 \vert
		\f}
	    */
	    virtual void getIncrementalCost(const State *s1, const State *s2, Cost *cost) const;

	    /** \brief Default implementation places in \e cost the sum of the values of \e c1 and \e c2. **/
	    virtual void combineObjectiveCosts(const Cost *c1, const Cost *c2, Cost *cost) const;

	    /** \brief Default implementation sets value of \e cost to 0.0. */
	    virtual void getInitialCost(const State *s, Cost *cost) const;

	    /** \brief Default implementation sets value of \e cost to inf. */
	    virtual void getInfiniteCost(Cost *cost) const;

	    virtual Cost* allocCost(void) const;
	    virtual void copyCost(Cost *dest, const Cost *src) const;
	    virtual void freeCost(Cost *cost) const;

	protected:
	    double maxPathCost_;
	};

	class MechanicalWorkOptimizationObjective : public PathIntegralOptimizationObjective
	{
	public:
	    MechanicalWorkOptimizationObjective(const SpaceInformationPtr &si, double maxPathCost, double pathLengthWeight = 0.00001) :
		PathIntegralOptimizationObjective(si, maxPathCost), 
		pathLengthWeight_(pathLengthWeight) {}

	    virtual double getPathLengthWeight(void) const;
	    virtual void setPathLengthWeight(double weight);

	    virtual void getIncrementalCost(const State *s1, const State *s2, Cost *cost) const;

	protected:
	    double pathLengthWeight_;
	};
    }
}

#endif
