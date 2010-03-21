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

/* \author Ioan Sucan */

#ifndef OMPL_KINEMATIC_SPACE_INFORMATION_KINEMATIC_
#define OMPL_KINEMATIC_SPACE_INFORMATION_KINEMATIC_

#include "ompl/base/SpaceInformation.h"
#include "ompl/kinematic/PathKinematic.h"
#include "ompl/kinematic/GoalKinematic.h"

#include "ompl/base/L2SquareStateDistanceEvaluator.h"
#include "ompl/kinematic/LinearStateInterpolatorKinematic.h"

#include <vector>

namespace ompl
{
    
    /** \brief The namespace that contains code specific to planning
	under geometric constraints */
    namespace kinematic
    {
	
	/** \brief Space information useful for kinematic planning */
	class SpaceInformationKinematic : public base::SpaceInformation
	{
	public:
	    
	    /** \brief Constructor; setup() needs to be called as well, before use */
	    SpaceInformationKinematic(void) : base::SpaceInformation(),
					      m_defaultDistanceEvaluator(static_cast<base::SpaceInformation*>(this)),
					      m_defaultStateInterpolator(static_cast<base::SpaceInformation*>(this))
	    {
		m_stateDistanceEvaluator = &m_defaultDistanceEvaluator;
		m_stateInterpolator = &m_defaultStateInterpolator;
	    }
	    
	    /** \brief Destructor */
	    virtual ~SpaceInformationKinematic(void)
	    {
	    }
	    
	    /** \brief Set the instance of the state interpolator
		(local planner) to use. No memory freeing is
		performed. Parallel implementations of algorithms
		assume the state interpolator is thread safe. */
	    void setStateInterpolator(StateInterpolatorKinematic *sik)
	    {
		m_stateInterpolator = sik;
	    }

	    /** \brief Return the instance of the used state interpolator */
	    StateInterpolatorKinematic* getStateInterpolator(void) const
	    {
		return m_stateInterpolator;
	    }	
	    
	    /** \brief Incrementally check if the path between two motions is valid */
	    bool checkMotion(const base::State *s1, const base::State *s2,
			     base::State *lastValidState = NULL, double *lastValidTime = NULL) const
	    {
		return m_stateInterpolator->checkMotion(s1, s2, lastValidState, lastValidTime);
	    }
	    
	    /** \brief Get the states that make up a motion. Returns the number of states that were added */
	    unsigned int getMotionStates(const base::State *s1, const base::State *s2, std::vector<base::State*> &states, bool alloc) const
	    {
		return m_stateInterpolator->getStates(s1, s2, states, 1.0, true, alloc);
	    }	    
	    
	    /** \brief Check if the path is valid */
	    bool checkPath(const PathKinematic *path) const;
	
	    /** \brief Insert states in a path, at the collision checking resolution */
	    void interpolatePath(PathKinematic *path, double factor = 1.0) const;
	
	    /** \brief Perform additional tasks to finish the initialization of
		the space information */
	    virtual void setup(void);

	private:
	    
	    base::L2SquareStateDistanceEvaluator m_defaultDistanceEvaluator;
	    LinearStateInterpolatorKinematic     m_defaultStateInterpolator;
	    
	protected:
	    
	    StateInterpolatorKinematic *m_stateInterpolator;
	};
    }
    
}

#endif
