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

#include "ompl/kinematic/SimpleSetup.h"
#include "ompl/kinematic/LinearStateInterpolatorKinematic.h"
#include "ompl/base/L2SquareStateDistanceEvaluator.h"
#include "ompl/base/UniformStateSampler.h"

ompl::kinematic::SpaceInformationKinematic* ompl::kinematic::SimpleSetup::allocSpaceInformation(void)
{
    if (m_alloc_si)
	return m_alloc_si();
    else
	return new SpaceInformationKinematic();
}

ompl::kinematic::StateInterpolatorKinematic* ompl::kinematic::SimpleSetup::allocStateInterpolator(const base::SpaceInformation *si)
{
    if (m_alloc_sik)
	return m_alloc_sik(si);
    else
	return new LinearStateInterpolatorKinematic(si);
}

ompl::base::ProblemDefinition* ompl::kinematic::SimpleSetup::allocProblemDefinition(base::SpaceInformation *si)
{
    if (m_alloc_pdef)
	return m_alloc_pdef(si);
    else
	return new base::ProblemDefinition(si);
}

ompl::base::StateDistanceEvaluator* ompl::kinematic::SimpleSetup::allocStateDistanceEvaluator(const base::SpaceInformation *si)
{
    if (m_alloc_sde)
	return m_alloc_sde(si);
    else
	return new base::L2SquareStateDistanceEvaluator(si);
}

ompl::base::StateValidityChecker* ompl::kinematic::SimpleSetup::allocStateValidityChecker(const base::SpaceInformation *si)
{
    if (m_alloc_svc)
	return m_alloc_svc(si);
    else
	m_msg.error("Need to specify state validity checker");
    return NULL;
}

ompl::base::Goal* ompl::kinematic::SimpleSetup::allocGoal(const base::SpaceInformation *si)
{   
    if (m_alloc_goal)
	return m_alloc_goal(si);
    else
	m_msg.error("Need to specify goal");
    return NULL;
}

ompl::base::Planner* ompl::kinematic::SimpleSetup::allocPlanner(base::SpaceInformation *si)
{
    if (m_alloc_planner)
	return m_alloc_planner(si);
    else
	m_msg.error("Need to specify planner");
    return NULL;
}

namespace ompl
{
    namespace kinematic
    {
	static ompl::base::StateSampler* allocUniformStateSampler(const base::SpaceInformation *si)
	{
	    return new base::UniformStateSampler(si);
	}
    }
}

void ompl::kinematic::SimpleSetup::configure(void)
{
    if (m_configured)
    {
	m_msg.warn("Simple kinematic setup already configured");
	return;
    }
    
    m_configured = true;
    
    m_si = allocSpaceInformation();
    configureSpaceInformation(m_si);
    
    m_pdef = allocProblemDefinition(m_si);
    m_goal = allocGoal(m_si);
    m_pdef->setGoal(m_goal);
    
    m_svc = allocStateValidityChecker(m_si);
    m_si->setStateValidityChecker(m_svc);
    
    m_sik = allocStateInterpolator(m_si);
    m_si->setStateInterpolator(m_sik);
    
    m_sde = allocStateDistanceEvaluator(m_si);
    m_si->setStateDistanceEvaluator(m_sde);
    
    if (!m_alloc_ssa)
	m_alloc_ssa = boost::bind(allocUniformStateSampler, _1);
    
    m_si->setStateSamplerAllocator(m_alloc_ssa);
    
    m_si->setup();
    
    m_psk = new PathSimplifierKinematic(m_si);
    
    m_planner = allocPlanner(m_si);
    m_planner->setProblemDefinition(m_pdef);
    m_planner->setup();
}

void ompl::kinematic::SimpleSetup::clear(void)
{
    if (m_configured)
    {
	m_planner->clear();
	m_pdef->clearStartStates();
	m_goal->setSolutionPath(NULL);
    }
}
