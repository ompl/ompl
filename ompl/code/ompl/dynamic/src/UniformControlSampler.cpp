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

#include "ompl/dynamic/UniformControlSampler.h"
#include "ompl/dynamic/SpaceInformationControls.h"

unsigned int ompl::dynamic::UniformControlSampler::sampleStepCount(void)
{
    return m_rng.uniformInt(m_minControlDuration, m_maxControlDuration);
}

void ompl::dynamic::UniformControlSampler::sample(Control *ctrl)
{
    const unsigned int dim = m_si->getControlDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const ControlComponent &comp = m_si->getControlComponent(i);
	ctrl->values[i] = m_rng.uniformReal(comp.minValue, comp.maxValue);
    }
}

void ompl::dynamic::UniformControlSampler::sampleNear(Control *ctrl, const Control *near, const double rho)
{
    const unsigned int dim = m_si->getControlDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const ControlComponent &comp = m_si->getControlComponent(i);
	ctrl->values[i] = m_rng.uniformReal(std::max(comp.minValue, near->values[i] - rho), 
					    std::min(comp.maxValue, near->values[i] + rho));
    }
}

void ompl::dynamic::UniformControlSampler::sampleNear(Control *ctrl, const Control *near, const std::vector<double> &rho)
{
    const unsigned int dim = m_si->getControlDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const ControlComponent &comp = m_si->getControlComponent(i);
	ctrl->values[i] = m_rng.uniformReal(std::max(comp.minValue, near->values[i] - rho[i]), 
					    std::min(comp.maxValue, near->values[i] + rho[i]));
    }
}

