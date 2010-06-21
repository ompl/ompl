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

#ifndef OMPL_KINEMATIC_SIMPLE_SETUP_
#define OMPL_KINEMATIC_SIMPLE_SETUP_

#include "ompl/kinematic/SpaceInformationKinematic.h"
#include "ompl/kinematic/PathSimplifierKinematic.h"
#include "ompl/base/Planner.h"
#include "ompl/util/Console.h"

namespace ompl
{

    namespace kinematic
    {
	
	/** \brief Create the set of classes typically needed to solve a
	    kinematic problem */
	class SimpleSetup
	{
	public:
	    SimpleSetup(void) : m_si(NULL), m_sik(NULL), m_psk(NULL), m_pdef(NULL), m_planner(NULL),
				m_svc(NULL), m_sde(NULL), m_goal(NULL), m_configured(false)
	    {
	    }
	    
	    virtual ~SimpleSetup(void)
	    {
		delete m_planner;
		delete m_goal;
		delete m_pdef;
		delete m_psk;
		delete m_si;
		delete m_sik;
		delete m_sde;
		delete m_svc;
	    }
	    
	    virtual SpaceInformationKinematic* allocSpaceInformation(void);
	    virtual StateInterpolatorKinematic* allocStateInterpolator(base::SpaceInformation *si);
	    virtual base::ProblemDefinition* allocProblemDefinition(base::SpaceInformation *si);
	    virtual base::StateDistanceEvaluator* allocStateDistanceEvaluator(base::SpaceInformation *si);
	    virtual base::StateSamplerAllocator stateSamplerAllocator(void);

	    virtual void configureSpaceInformation(SpaceInformationKinematic *si) = 0;
	    virtual base::StateValidityChecker* allocStateValidityChecker(base::SpaceInformation *si) = 0;
	    virtual base::Goal* allocGoal(base::SpaceInformation *si) = 0;
	    virtual base::Planner* allocPlanner(base::SpaceInformation *si) = 0;
	    
	    virtual void configure(void);
	    
	protected:
	    
	    SpaceInformationKinematic    *m_si;
	    StateInterpolatorKinematic   *m_sik;
	    PathSimplifierKinematic      *m_psk;
	    
	    base::ProblemDefinition      *m_pdef;
	    base::Planner                *m_planner;
	    base::StateValidityChecker   *m_svc;
	    base::StateDistanceEvaluator *m_sde;
	    
	    base::Goal                   *m_goal;
	    base::StateSamplerAllocator   m_ssa;

	    bool                          m_configured;
	    msg::Interface                m_msg;
	};
    }
    
}
#endif
