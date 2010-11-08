/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_GOAL_
#define OMPL_BASE_GOAL_

#include "ompl/base/State.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/Path.h"
#include "ompl/util/ClassForward.h"
#include <iostream>
#include <boost/noncopyable.hpp>
#include <boost/concept_check.hpp>

namespace ompl
{
    namespace base
    {
	
	/** \brief Forward declaration of ompl::base::Goal */
	ClassForward(Goal);

	/** \class ompl::base::GoalPtr
	    \brief A boost shared pointer wrapper for ompl::base::Goal */

	/** \brief Abstract definition of goals. Will contain solutions, if found */
	class Goal : private boost::noncopyable
	{
	public:
	    
	    /** \brief Constructor. The goal must always know the space information it is part of */
	    Goal(const SpaceInformationPtr &si);
	    
	    /** \brief Destructor. Clears the solution as well */
	    virtual ~Goal(void)
	    {
	    }	

	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    T* as(void)
	    {
		/** \brief Make sure the type we are casting to is indeed a goal */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Goal*>));
		
		return static_cast<T*>(this);
	    }

	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    const T* as(void) const
	    {	
		/** \brief Make sure the type we are casting to is indeed a goal */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Goal*>));
		
		return static_cast<const T*>(this);
	    }
	    
	    /** \brief Return true if the state statisfies the goal
	     *  constraints. */
	    virtual bool isSatisfied(const State *st) const = 0;

	    /** \brief Return true if the state statisfies the goal
	     *  constraints and compute the distance between the state
	     *  given as argument and the goal (even if the goal is
	     *  not satisfied). This distance can be an
	     *  approximation. It can even be set to a constant, if
	     *  such a computation is not possible.
	     *  \param st the state to check for validity
	     *  \param distance location at which distance to goal will be stored
	     *  \note The default implementation sets the distance to a constant.
	     *  \note If this function returns true,
	     *  isStartGoalPairValid() need not be called. */
	    virtual bool isSatisfied(const State *st, double *distance) const;
	    
	    /** \brief Return true if the state statisfies the goal
	     *  constraints and the path length is less than the
	     *  desired maximum length.  This call aslo computes the
	     *  distance between the state given as argument and the
	     *  goal. 	     
	     *  \param st the state to check for validity
	     *  \param pathLength the length of the path that leads to \e st
	     *  \param distance location at which distance to goal will be stored
	     */
	    bool isSatisfied(const State *st, double pathLength, double *distance) const;
	    
	    /** \brief Since there can be multiple starting states
		(and multiple goal states) it is possible certain
		pairs are not to be allowed. By default we however
		assume all such pairs are allowed. Note: if this
		function returns true, isSatisfied() need not be
		called. */
	    virtual bool isStartGoalPairValid(const State * /* start */, const State * /* goal */) const 
	    {
		return true;
	    }
	    
	    /** \brief Returns true if a solution path has been found (could be approximate) */
	    bool isAchieved(void) const
	    {
		return path_;
	    }

	    /** \brief Get the maximum length allowed for a solution path */
	    double getMaximumPathLength(void) const
	    {
		return maximumPathLength_;
	    }

	    /** \brief Set the maximum length allowed for a solution
		path. This value is checked only in the version of
		isSatisfied() that takes the path length as
		argument */
	    void setMaximumPathLength(double maximumPathLength)
	    {
		maximumPathLength_ = maximumPathLength;
	    }
	    
	    /** \brief Return the found solution path. 

		This will need to be casted into the specialization
		computed by the planner */
	    const PathPtr& getSolutionPath(void) const
	    {
		return path_;
	    }	    
	    
	    /** \brief Update the solution path. If a previous solution path exists, it is deleted. */
	    void setSolutionPath(const PathPtr &path, bool approximate = false)
	    {
		path_ = path;
		approximate_ = approximate;
	    }

	    /** \brief Forget the solution path. Memory is freed. */
	    void clearSolutionPath(void)
	    {
		path_.reset();
	    }
	    
	    /** \brief If a difference between the desired solution and the
		solution found is computed by the planner, this functions
		returns it */
	    double getDifference(void) const
	    {
		return difference_;
	    }
	    
	    /** \brief Set the difference between the found solution
		path and the desired solution path */
	    void setDifference(double difference)
	    {
		difference_ = difference;
	    }
	    
	    /** \brief Return true if the found solution is
		approximate (does not actually reach the desired goal,
		but hopefully is closer to it) */
	    bool isApproximate(void) const
	    {
		return approximate_;
	    }
	    
	    /** \brief Print information about the goal */
	    virtual void print(std::ostream &out = std::cout) const
	    {
		out << "Goal memory address " << this << std::endl;
	    }
	    
	protected:
	    
	    /** \brief The space information for this goal */
	    SpaceInformationPtr      si_;
	    
	    /** \brief The maximum length allowed for the solution path */
	    double                   maximumPathLength_;
	    
	    /** \brief Solution path, if found */
	    PathPtr                  path_;
	    	    
	    /** \brief The achieved difference between the found solution and the desired goal */
	    double                   difference_;
	    
	    /** \brief True if goal was not achieved, but an approximate solution was found */
	    bool                     approximate_;

	};
		
    }
}

#endif
