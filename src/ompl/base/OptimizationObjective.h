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

#include "ompl/base/State.h"
#include "ompl/base/Path.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>
#include <boost/concept_check.hpp>
#include <limits>

namespace ompl
{
    namespace base
    {   
	// /// @cond IGNORE
	// /** \brief Forward declaration of ompl::base::Cost */
	OMPL_CLASS_FORWARD(Cost);
	// /// @endcond

        /** \class ompl::base::StateSpacePtr
            \brief A boost shared pointer wrapper for ompl::base::StateSpace */

	/** \brief Definition of an abstract cost. */
	class Cost 
	{
	public:
	    /** \brief Cast this instance to a desired type. */
	    template<typename T>
	    static boost::shared_ptr<T> as(const CostPtr& c)
	    {
	    	/** \brief Make sure the type we are casting to is indeed a cost */
                BOOST_CONCEPT_ASSERT((boost::Convertible<boost::shared_ptr<T>, 
	    						 CostPtr>));
	    	return boost::static_pointer_cast<T>(c);
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

        /** \class ompl::base::OptimizationObjectivePtr
            \brief A boost shared pointer wrapper for ompl::base::OptimizationObjective */

        /** \brief Abstract definition of optimization objectives.

            \note This implementation has greatly benefited from discussions with <a href="http://www.cs.indiana.edu/~hauserk/">Kris Hauser</a> */
        class OptimizationObjective : private boost::noncopyable
        {
        public:
	    typedef Cost CostType;

            /** \brief Constructor. The objective must always know the space information it is part of */
            OptimizationObjective(const SpaceInformationPtr &si);

            virtual ~OptimizationObjective(void)
            {
            }

            /** \brief Get the description of this optimization objective */
            const std::string& getDescription(void) const
            {
                return description_;
            }

            /** \brief Verify that our objective is satisfied already and we can stop planning (it the cost is \e totalObjectiveCost) */
            virtual bool isSatisfied(const CostPtr& cost) const = 0;

            /** \brief Get the cost that corresponds to the final state on the path (that satisfies the goal) */
            // virtual double getTerminalCost(const State *s) const = 0;

            /** \brief Get the cost that corresponds to an entire path. */
            virtual CostPtr getCost(const PathPtr &path) const = 0;

	    /** \brief Check whether the the cost \e c1 is considered less than the cost \e c2. */
	    virtual bool compareCost(const CostPtr& c1, const CostPtr& c2) const = 0;

        protected:
            /** \brief The space information for this objective */
            SpaceInformationPtr si_;

            /** \brief The description of this optimization objective */
            std::string description_;
        };
    }
}

#endif
