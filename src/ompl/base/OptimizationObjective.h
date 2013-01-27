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
            OptimizationObjective(const SpaceInformationPtr &si);

            virtual ~OptimizationObjective(void)
            {
            }

            /** \brief Get the description of this optimization objective */
            const std::string& getDescription(void) const
            {
                return description_;
            }

            /** \brief Verify that our objective is satisfied already and we can stop planning */
            virtual bool isSatisfied(const Cost* cost) const = 0;

            /** \brief Get the cost that corresponds to an entire path. */
	    virtual void getCost(const Path& path, Cost* cost) const = 0;

            /** \brief Get the cost that corresponds to an entire path. */
            void getCost(const PathPtr &path, Cost* cost) const;

	    /** \brief Check whether the the cost \e c1 is considered less than the cost \e c2. */
	    virtual bool isCostLessThan(const Cost* c1, const Cost* c2) const = 0;
	    
	    /** \brief Allocate storage for a cost calculation in this OptimizationObjective. */
	    virtual Cost* allocCost(void) const = 0;

	    /** \brief Copy one cost into another. Memory for source and destination should NOT overlap. */
	    virtual void copyCost(Cost* dest, const Cost* src) const = 0;

	    /** \brief Free the memory of the allocated cost. */
	    virtual void freeCost(Cost* cost) const = 0;

        protected:
            /** \brief The space information for this objective */
            SpaceInformationPtr si_;

            /** \brief The description of this optimization objective */
            std::string description_;
        };
    }
}

#endif
