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

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/UniformStateSampler.h"

#include <cstring>
#include <cassert>

namespace ompl
{
    namespace base
    {
	static StateSampler* allocUniformStateSampler(const SpaceInformation *si)
	{
	    return new UniformStateSampler(si);
	}
    }
}

void ompl::base::SpaceInformation::copyState(State *destination, const State *source) const
{
    memcpy(destination->values, source->values, sizeof(double) * m_stateDimension);
}

bool ompl::base::SpaceInformation::equalStates(const State *a, const State *b) const
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	if (a->values[i] != b->values[i])
	    return false;
    return true;
}

void ompl::base::SpaceInformation::setup(void)
{
    if (m_setup)
	m_msg.error("Space information setup called multiple times");
    
    if (m_stateDimension <= 0)
	m_msg.error("State space dimension must be > 0");
    
    if (m_stateComponent.size() != m_stateDimension)
	m_msg.error("State component specification does not agree with state dimension");
    
    if (!m_stateSamplerAllocator)
	m_stateSamplerAllocator = boost::bind(allocUniformStateSampler, _1);
    
    m_setup = true;
}

bool ompl::base::SpaceInformation::isSetup(void) const
{
    return m_setup;
}

void ompl::base::SpaceInformation::printState(const State *state, std::ostream &out) const
{
    if (state)
    {
	for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	    out << state->values[i] << " ";
	out << std::endl;
    }
    else
	out << "NULL" << std::endl;
}

bool ompl::base::SpaceInformation::satisfiesBounds(const State *s) const
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	if (s->values[i] - STATE_EPSILON > m_stateComponent[i].maxValue ||
	    s->values[i] + STATE_EPSILON < m_stateComponent[i].minValue)
	    return false;
    return true;
}

void ompl::base::SpaceInformation::enforceBounds(base::State *state) const
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
    {
	if (state->values[i] > m_stateComponent[i].maxValue)
	    state->values[i] = m_stateComponent[i].maxValue;
	else
	    if (state->values[i] < m_stateComponent[i].minValue)
		state->values[i] = m_stateComponent[i].minValue;
    }
}

bool ompl::base::SpaceInformation::searchValidNearby(base::State *state, const base::State *near, const std::vector<double> &rho, unsigned int attempts) const
{
    assert(near != state);
    
    copyState(state, near);
    
    // fix bounds, if needed
    if (!satisfiesBounds(state))
	enforceBounds(state);
    
    bool result = isValid(state);
    
    if (!result)
    {
	// try to find a valid state nearby
	StateSamplerInstance ss(this);
	base::State          temp(m_stateDimension);
	copyState(&temp, state);	
	for (unsigned int i = 0 ; i < attempts && !result ; ++i)
	{
	    ss().sampleNear(state, &temp, rho);
	    result = isValid(state);
	}
    }
    
    return result;
}

void ompl::base::SpaceInformation::printSettings(std::ostream &out) const
{
    out << "State space settings:" << std::endl;
    out << "  - dimension = " << m_stateDimension << std::endl;
    out << "  - bounding box:" << std::endl;
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	out << "[" << m_stateComponent[i].minValue << ", " <<  m_stateComponent[i].maxValue << "](" << m_stateComponent[i].resolution << ") ";
    out << std::endl;
}
