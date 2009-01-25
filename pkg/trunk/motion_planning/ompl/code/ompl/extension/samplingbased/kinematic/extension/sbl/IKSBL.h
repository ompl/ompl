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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_IKSBL_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_IKSBL_

#include "ompl/extension/samplingbased/kinematic/extension/sbl/SBL.h"
#include "ompl/extension/samplingbased/kinematic/extension/ik/GAIK.h"

/** Main namespace */
namespace ompl
{

    /** Forward class declaration */
    ForwardClassDeclaration(IKSBL);
    
    /**
       @subsubsection IKSBL Inberse Kinematics Single-query Bi-directional Lazy collision checking planner (IKSBL)
       
       @par Short description     

       IKSBL is actually SBL that uses GAIK internally to compute
       possible goal states (only if the goal is not specified as a
       state). This avoids the need for inverse kinematics before
       SBL. GAIK does inverse kinematics, but makes sure the produced
       goal states are in fact valid.

       @par External documentation

    */
    class IKSBL : public SBL
    {
    public:

        IKSBL(SpaceInformation_t si) : SBL(si),
 	                               m_gaik(si)
	{
	    m_type = PLAN_TO_GOAL_STATE | PLAN_TO_GOAL_REGION;
	}

	virtual ~IKSBL(void)
	{
	}
	
	virtual void setup(void)
	{
	    m_gaik.setRange(m_rho);
	    m_gaik.setup();
	    SBL::setup();
	}

	virtual bool solve(double solveTime);

	virtual void clear(void)
	{
	    m_gaik.clear();
	    SBL::clear();	    
	}
	
    protected:
	
	GAIK m_gaik;
	
    };

}

#endif
