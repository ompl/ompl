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

#include "ompl/dynamic/SpaceInformationControls.h"
#include "ompl/dynamic/UniformControlSampler.h"
#include <algorithm>
#include <cstring>
#include <ros/console.h>

namespace ompl
{
    namespace dynamic
    {
	static ControlSampler* allocUniformControlSampler(const SpaceInformationControls *si)
	{
	    return new UniformControlSampler(si);
	}
    }
}

void ompl::dynamic::SpaceInformationControls::setup(void)
{
    if (m_resolution <= 0.0)
	ROS_ERROR("Resolution of must be >0 when planning with controls");
    
    if (m_controlDimension <= 0)
	ROS_FATAL("The dimension of the control space needs to be larger than 0");
    
    if (m_controlComponent.size() != m_controlDimension)
	ROS_FATAL("Control component specification does not agree with control dimension");
    
    if (!m_stateDistanceEvaluator)
	ROS_FATAL("No state distance evaluator defined");
    
    if (m_minControlDuration > m_maxControlDuration)
	ROS_FATAL("The maximum duration of a control is less than the minimum one");
    
    if (m_minControlDuration <= 0)
	ROS_ERROR("The minimum duration of a control must be positive");    
    
    if (!m_controlSamplerAllocator)
	m_controlSamplerAllocator = boost::bind(allocUniformControlSampler, _1);
    
    SpaceInformation::setup();
}

void ompl::dynamic::SpaceInformationControls::setKinematicPath(const kinematic::PathKinematic *hint)
{
    if (m_hint)
	delete m_hint;
    if (hint == NULL)
	m_hint = NULL;
    else
	m_hint = new kinematic::PathKinematic(*hint);
}

void ompl::dynamic::SpaceInformationControls::copyControl(Control *destination, const Control *source) const
{
    memcpy(destination->values, source->values, sizeof(double) * m_controlDimension);
}

void ompl::dynamic::SpaceInformationControls::printControl(const Control *control, std::ostream &out) const
{
    if (control)
    {
	for (unsigned int i = 0 ; i < m_controlDimension ; ++i)
	    out << control->values[i] << " ";
	out << std::endl;
    }
    else
	out << "NULL" << std::endl;
}

void ompl::dynamic::SpaceInformationControls::nullControl(Control *ctrl) const
{
    for (unsigned int i = 0 ; i < m_controlDimension ; ++i)
	ctrl->values[i] = 0.0;
}
