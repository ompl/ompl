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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_GAIK_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_GAIK_

#include "ompl/base/Planner.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"

namespace ompl
{
    
    /** Forward class declaration */
    ForwardClassDeclaration(GAIK);
    
    /**
       @subsubsection Inverse Kinematics with Genetic Algorithms
       
       @par Short description

       GAIK does inverse kinematics, but makes sure the produced
       goal states are in fact valid.       
       
       @par External documentation
    */
    class GAIK : public Planner
    {
    public:

        GAIK(SpaceInformation_t si) : Planner(si)
	{
	    m_type = PLAN_TO_GOAL_REGION;
	    random_utils::init(&m_rngState);
	    m_rho = 0.05;
	    m_poolSize = 60;
	    m_poolExpansion = 80;
	}
	
	virtual ~GAIK(void)
	{
	}
	
	virtual bool solve(double solveTime);
	
	virtual void clear(void)
	{
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
	
	bool tryToSolve(SpaceInformationKinematic::StateKinematic_t state, double *distance);
	bool tryToSolveFact(double factorP, SpaceInformationKinematic::StateKinematic_t state, double *distance);
	
	struct Individual
	{
	    SpaceInformationKinematic::StateKinematic_t state;
	    double                                      distance;
	};
	
	struct IndividualSort
	{
	    bool operator()(const Individual& a, const Individual& b)
	    {
		return a.distance < b.distance;
	    }	    
	};
	
	unsigned int           m_poolSize;
	unsigned int           m_poolExpansion;
	
	double                 m_rho;	
	random_utils::rngState m_rngState;	
    };

}

#endif
