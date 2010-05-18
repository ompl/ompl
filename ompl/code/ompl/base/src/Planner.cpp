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

#include "ompl/base/Planner.h"

ompl::base::Planner::Planner(SpaceInformation *si) : m_si(si), m_pdef(NULL), m_type(PLAN_UNKNOWN), m_setup(false)
{
    if (!m_si)
	m_msg.error("Invalid space information instance");
    if (!m_si->isSetup())
	m_msg.warn("It is best if space information setup has been called before a planner is instantiated");
}

ompl::base::PlannerType ompl::base::Planner::getType(void) const
{
    return m_type;
}

const ompl::base::ProblemDefinition* ompl::base::Planner::getProblemDefinition(void) const
{
    return m_pdef;
}

void ompl::base::Planner::setProblemDefinition(ProblemDefinition *pdef)
{
    clear();
    m_pdef = pdef;
}

void ompl::base::Planner::setup(void)
{
    if (!m_si->isSetup())
	m_msg.error("Space information setup should have been called before planner setup was called");
    if (m_setup)
	m_msg.error("Planner setup called multiple times");		
    m_setup = true;
}

