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
	    SimpleSetup(void) : m_si(NULL), m_sik(NULL), m_pdef(NULL), m_planner(NULL),
				m_svc(NULL), m_sde(NULL), m_goal(NULL), m_psk(NULL), m_configured(false)
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
	    
	    
	    SpaceInformationKinematic* getSpaceInformation(void)
	    {
		return m_si;
	    }
	    
	    StateInterpolatorKinematic* getStateInterpolator(void)
	    {
		return m_sik;
	    }
	    
	    base::ProblemDefinition* getProblemDefinition(void)
	    {
		return m_pdef;
	    }
	    
	    base::StateDistanceEvaluator* getStateDistanceEvaluator(void)
	    {
		return m_sde;
	    }
	    
	    base::StateValidityChecker* getStateValidityChecker(void)
	    {
		return m_svc;
	    }

	    base::Goal* getGoal(void)
	    {
		return m_goal;
	    }

	    base::Planner* getPlanner(void)
	    {
		return m_planner;
	    }

	    PathSimplifierKinematic* getPathSimplifier(void)
	    {
		return m_psk;
	    }
	    
	    base::StateSamplerAllocator getStateSamplerAllocator(void)
	    {
		return m_alloc_ssa;
	    }	    
	    
	    void setStateSamplerAllocator(const base::StateSamplerAllocator &alloc)
	    {
		m_alloc_ssa = alloc;
	    }
	    
	    void setSpaceInformationAllocator(const boost::function<SpaceInformationKinematic*()> &alloc)
	    {
		m_alloc_si = alloc;
	    }

	    void setStateInterpolatorAllocator(boost::function<StateInterpolatorKinematic*(const base::SpaceInformation*)> &alloc)
	    {
		m_alloc_sik = alloc;
	    }

	    void setProblemDefinitionAllocator(boost::function<base::ProblemDefinition*(const base::SpaceInformation*)> &alloc)
	    {
		m_alloc_pdef = alloc;
	    }	    
	    
	    void setStateDistanceEvaluatorAllocator(boost::function<base::StateDistanceEvaluator*(const base::SpaceInformation*)> &alloc)
	    {
		m_alloc_sde = alloc;
	    }

	    void setStateValidityCheckerAllocator(boost::function<base::StateValidityChecker*(const base::SpaceInformation*)> &alloc)
	    {
		m_alloc_svc = alloc;
	    }

	    void setGoalAllocator(boost::function<base::Goal*(const base::SpaceInformation*)> &alloc)
	    {
		m_alloc_goal = alloc;
	    }

	    void setPlannerAllocator(boost::function<base::Planner*(const base::SpaceInformation*)> &alloc)
	    {
		m_alloc_planner = alloc;
	    }
	    
	    virtual SpaceInformationKinematic* allocSpaceInformation(void);
	    virtual StateInterpolatorKinematic* allocStateInterpolator(base::SpaceInformation *si);
	    virtual base::ProblemDefinition* allocProblemDefinition(base::SpaceInformation *si);
	    virtual base::StateDistanceEvaluator* allocStateDistanceEvaluator(base::SpaceInformation *si);
	    virtual base::StateValidityChecker* allocStateValidityChecker(base::SpaceInformation *si);
	    virtual base::Goal* allocGoal(base::SpaceInformation *si);
	    virtual base::Planner* allocPlanner(base::SpaceInformation *si);
	    
	    virtual void configureSpaceInformation(SpaceInformationKinematic *si) = 0;
	    
	    virtual void configure(void);
	    
	    virtual void clear(void)
	    {
		if (m_configured)
		{
		    m_planner->clear();
		    m_pdef->clearStartStates();
		    m_goal->setSolutionPath(NULL);
		}
	    }
	    
	protected:
	    
	    SpaceInformationKinematic    *m_si;
	    boost::function<SpaceInformationKinematic*()> 
	                                  m_alloc_si;
	    
	    StateInterpolatorKinematic   *m_sik;
	    boost::function<StateInterpolatorKinematic*(const base::SpaceInformation*)>
	                                  m_alloc_sik;
	    
	    base::ProblemDefinition      *m_pdef;
	    boost::function<base::ProblemDefinition*(const base::SpaceInformation*)> 
	                                  m_alloc_pdef;
	    
	    base::Planner                *m_planner;
	    boost::function<base::Planner*(const base::SpaceInformation*)> 
	                                  m_alloc_planner;
	    
	    base::StateValidityChecker   *m_svc;
	    boost::function<base::StateValidityChecker*(const base::SpaceInformation*)> 
	                                  m_alloc_svc;

	    base::StateDistanceEvaluator *m_sde;
	    boost::function<base::StateDistanceEvaluator*(const base::SpaceInformation*)> 
	                                  m_alloc_sde;
	    
	    base::Goal                   *m_goal;
	    boost::function<base::Goal*(const base::SpaceInformation*)> 
	                                  m_alloc_goal;
	    
	    PathSimplifierKinematic      *m_psk;

	    base::StateSamplerAllocator   m_alloc_ssa;

	    bool                          m_configured;
	    msg::Interface                m_msg;
	};
    }
    
}
#endif
