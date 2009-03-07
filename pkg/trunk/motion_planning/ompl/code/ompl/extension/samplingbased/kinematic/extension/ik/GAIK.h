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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_IK_GAIK_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_IK_GAIK_

#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"
#include "ompl/extension/samplingbased/kinematic/extension/ik/HCIK.h"
#include "ompl/base/util/time.h"

namespace ompl
{
    
    namespace sb
    {
	
	/**
	   @subsubsection GAIK Inverse Kinematics with Genetic Algorithms
	   
	   @par Short description
	   
	   GAIK does inverse kinematics, but makes sure the produced
	   goal states are in fact valid.       
	   
	   @par External documentation
	*/
	class GAIK
	{
	public:
	    
	    GAIK(SpaceInformationKinematic *si) : m_hcik(dynamic_cast<SpaceInformation*>(si)),
						  m_sCore(si)
	    {					
		m_si = si;
		m_rho = 0.04;
		m_poolSize = 80;
		m_poolExpansion = 100;
		m_hcik.setMaxImproveSteps(3);
		m_checkValidity = true;
	    }
	    
	    virtual ~GAIK(void)
	    {
	    }
	    
	    virtual bool solve(double solveTime, State *result, const State* hint = NULL);
	    
	    void setMaxImproveSteps(unsigned int maxSteps)
	    {
		m_hcik.setMaxImproveSteps(maxSteps);
	    }
	    
	    unsigned int getMaxImproveSteps(void) const
	    {
		return m_hcik.getMaxImproveSteps();
	    }
	    
	    void setValidityCheck(bool valid)
	    {
		m_checkValidity = valid;
	    }
	    
	    bool getValidityCheck(void) const
	    {
		return m_checkValidity;
	    }
	    
	    /** The number of individuals in the population */
	    void setPoolSize(unsigned int size)
	    {
		m_poolSize = size;
	    }
	    
	    unsigned int getPoolSize(void) const
	    {
		return m_poolSize;
	    }
	    
	    /** The number of individuals to add to the population in each generation */
	    void setPoolExpansionSize(unsigned int size)
	    {
		m_poolExpansion = size;
	    }
	    
	    unsigned int getPoolExtensionSize(void) const
	    {
		return m_poolExpansion;
	    }
	    
	    void setRange(double rho)
	    {
		m_rho = rho;
	    }
	    
	    /** Get the range the planner is using */
	    double getRange(void) const
	    {
		return m_rho;
	    }
	    
	protected:
	    
	    bool tryToImprove(State *state, double distance);
	    bool valid(const State *state) const;
	    
	    struct Individual
	    {
		State *state;
		double distance;
	    };
	    
	    struct IndividualSort
	    {
		bool operator()(const Individual& a, const Individual& b)
		{
		    return a.distance < b.distance;
		}	    
	    };
	    
	    HCIK                                    m_hcik;
	    SpaceInformationKinematic::SamplingCore m_sCore;
	    SpaceInformationKinematic              *m_si;	
	    unsigned int                            m_poolSize;
	    unsigned int                            m_poolExpansion;
	    unsigned int                            m_maxImproveSteps;	
	    bool                                    m_checkValidity;	
	    
	    double                                  m_rho;	
	    
	    msg::Interface                          m_msg;
	};
	
    }
}

#endif
