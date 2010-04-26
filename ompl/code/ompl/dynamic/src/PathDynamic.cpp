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

#include "ompl/dynamic/PathDynamic.h"
#include "ompl/dynamic/SpaceInformationControls.h"
#include <cassert>

ompl::dynamic::PathDynamic::PathDynamic(const PathDynamic &path) : base::Path(path.getSpaceInformation())
{
    states.resize(path.states.size());
    controls.resize(path.controls.size());
    unsigned int sdim = m_si->getStateDimension();
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
	states[i] = new base::State(sdim);
	m_si->copyState(states[i], path.states[i]);
    }
    const SpaceInformationControls *si = dynamic_cast<const SpaceInformationControls*>(m_si);
    assert(si);
    unsigned int cdim = si->getControlDimension();
    for (unsigned int i = 0 ; i < controls.size() ; ++i)
    {
	controls[i] = new Control(cdim);
	si->copyControl(controls[i], path.controls[i]);
    }
    controlDurations = path.controlDurations;
}

double ompl::dynamic::PathDynamic::length(void) const
{
    double sum = 0.0;
    for (unsigned int i = 0 ; i < controlDurations.size() ; ++i)
	sum += controlDurations[i];
    return sum;
}

void ompl::dynamic::PathDynamic::freeMemory(void)
{
    for (unsigned int i = 0 ; i < states.size() ; ++i)
	delete states[i];
    for (unsigned int i = 0 ; i < controls.size() ; ++i)
	delete controls[i];
}
