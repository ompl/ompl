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

#ifndef OMPL_KINEMATIC_PLANNERS_IK_HCIK_
#define OMPL_KINEMATIC_PLANNERS_IK_HCIK_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/GoalRegion.h"

namespace ompl
{

    namespace kinematic
    {
	    
	/**
	   @anchor HCIK
	   
	   @par Short description
	   
	   HCIK does inverse kinematics with hill climbing, starting from a given state.       
	   
	   @par External documentation
	*/

	/** \brief Inverse Kinematics with Hill Climbing */
	class HCIK
	{
	public:
	    
	    HCIK(base::SpaceInformation *si)
	    {
		m_si = si;
		m_maxImproveSteps = 2;
		m_checkValidity = true;
	    }
	    
	    virtual ~HCIK(void)
	    {
	    }
	    
	    /** \brief Try to improve a state (reduce distance to goal). The step size to add is also specified */
	    bool tryToImprove(const base::GoalRegion *goal, base::State *state, double add, double *distance = NULL) const;
	    
	    /** \brief Set the number of steps to perform */
	    void setMaxImproveSteps(unsigned int steps)
	    {
		m_maxImproveSteps = steps;
	    }

	    /** \brief Get the number of steps to perform */
	    unsigned int getMaxImproveSteps(void) const
	    {
		return m_maxImproveSteps;
	    }
	    
	    /** \brief Set the state validity flag; if this is false, states are not checked for validity */
	    void setValidityCheck(bool valid)
	    {
		m_checkValidity = valid;
	    }

	    /** \brief Get the state validity flag; if this is false, states are not checked for validity */
	    bool getValidityCheck(void) const
	    {
		return m_checkValidity;
	    }
	    
	protected:	

	    bool valid(const base::State *state) const
	    {
		return m_checkValidity ? m_si->isValid(state) : true;
	    }
	    
	    base::SpaceInformation *m_si;
	    unsigned int            m_maxImproveSteps;
	    bool                    m_checkValidity;	
	};
	
    }

}
    
#endif
