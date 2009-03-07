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

#include "ompl/extension/samplingbased/SpaceInformation.h"
#include <cstring>
#include <cassert>

void ompl::sb::SpaceInformation::copyState(State *destination, const State *source) const
{
    memcpy(destination->values, source->values, sizeof(double) * m_stateDimension);
}

void ompl::sb::SpaceInformation::setup(void)
{
    assert(m_stateDimension > 0);
    assert(m_stateComponent.size() == m_stateDimension);
    base::SpaceInformation::setup();
}

void ompl::sb::SpaceInformation::printState(const State *state, std::ostream &out) const
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

bool ompl::sb::SpaceInformation::satisfiesBounds(const State *s) const
{
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	if (s->values[i] > m_stateComponent[i].maxValue ||
	    s->values[i] < m_stateComponent[i].minValue)
	    return false;
    return true;
}

void ompl::sb::SpaceInformation::printSettings(std::ostream &out) const
{
    out << "Kinematic state space settings:" << std::endl;
    out << "  - dimension = " << m_stateDimension << std::endl;
    out << "  - start states:" << std::endl;
    for (unsigned int i = 0 ; i < getStartStateCount() ; ++i)
	printState(dynamic_cast<const State*>(getStartState(i)), out);
    if (m_goal)
	m_goal->print(out);
    else
	out << "  - goal = NULL" << std::endl;
    out << "  - bounding box:" << std::endl;
    for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
	out << "[" << m_stateComponent[i].minValue << ", " <<  m_stateComponent[i].maxValue << "](" << m_stateComponent[i].resolution << ") ";
    out << std::endl;
}
