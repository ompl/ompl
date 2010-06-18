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

#ifndef OMPL_KINEMATIC_PLANNERS_IK_GAIK_
#define OMPL_KINEMATIC_PLANNERS_IK_GAIK_

#include "ompl/kinematic/SpaceInformationKinematic.h"
#include "ompl/kinematic/planners/ik/HCIK.h"
#include "ompl/util/Console.h"

namespace ompl
{
    
    namespace kinematic
    {
	
	/**
	   @anchor GAIK
	   
	   @par Short description
	   
	   GAIK does inverse kinematics, but makes sure the produced
	   goal states are in fact valid.       
	   
	   @par External documentation
	*/

	/** \brief Inverse Kinematics with Genetic Algorithms */
	class GAIK
	{
	public:
	    
	    GAIK(SpaceInformationKinematic *si) : m_hcik(si), m_msg("GAIK")
	    {
		m_si = si;
		m_rho = 0.04;
		m_poolSize = 80;
		m_poolExpansion = 100;
		m_hcik.setMaxImproveSteps(3);
		setValidityCheck(true);
		m_sCore = m_si->allocNewStateSampler();
	    }
	    
	    virtual ~GAIK(void)
	    {
		delete m_sCore;
	    }

	    /** \brief Find a state that fits the request */
	    virtual bool solve(double solveTime, const base::GoalRegion *goal, base::State *result, const std::vector<base::State*> &hint = std::vector<base::State*>());
	    
	    /** \brief Set the number of steps to perform when using hill climbing to improve an individual in the population */
	    void setMaxImproveSteps(unsigned int maxSteps)
	    {
		m_hcik.setMaxImproveSteps(maxSteps);
	    }

	    /** \brief Get the number of steps to perform when using hill climbing to improve an individual in the population */
	    unsigned int getMaxImproveSteps(void) const
	    {
		return m_hcik.getMaxImproveSteps();
	    }
	    
	    /** \brief Set the state validity flag; if this is false, states are not checked for validity */
	    void setValidityCheck(bool valid)
	    {
		m_checkValidity = valid;
		m_hcik.setValidityCheck(valid);
	    }

	    /** \brief Get the state validity flag; if this is false, states are not checked for validity */
	    bool getValidityCheck(void) const
	    {
		return m_checkValidity;
	    }
	    
	    /** \brief Set the number of individuals in the population */
	    void setPoolSize(unsigned int size)
	    {
		m_poolSize = size;
	    }

	    /** \brief Get the number number of individuals in the population */
	    unsigned int getPoolSize(void) const
	    {
		return m_poolSize;
	    }
	    
	    /** \brief Set the number of individuals to add to the population in each generation */
	    void setPoolExpansionSize(unsigned int size)
	    {
		m_poolExpansion = size;
	    }
	    
	    /** \brief Get the number of individuals to add to the population in each generation */
	    unsigned int getPoolExpansionSize(void) const
	    {
		return m_poolExpansion;
	    }
	    
	    /** \brief Set the range (percentage of each dimension's extent) to be used when sampling around a state */
	    void setRange(double rho)
	    {
		m_rho = rho;
	    }
	    
	    /** \brief Get the range the planner is using */
	    double getRange(void) const
	    {
		return m_rho;
	    }
	    
	protected:
	    
	    bool tryToImprove(const base::GoalRegion* goal, base::State *state, double distance);
	    bool valid(const base::State *state) const
	    {
		return m_checkValidity ? m_si->isValid(state) : true;
	    }
	    
	    
	    struct Individual
	    {
		base::State *state;
		double       distance;
		bool         valid;
	    };
	    
	    struct IndividualSort
	    {
		bool operator()(const Individual& a, const Individual& b)
		{
		    if (a.valid == b.valid)
			return a.distance < b.distance;
		    return a.valid;
		}
	    };
	    
	    HCIK                                         m_hcik;
	    base::StateSampler                          *m_sCore;
	    SpaceInformationKinematic                   *m_si;	
	    unsigned int                                 m_poolSize;
	    unsigned int                                 m_poolExpansion;
	    bool                                         m_checkValidity;	
	    
	    double                                       m_rho;	

	    msg::Interface                               m_msg;
	};
	
    }
}

#endif
