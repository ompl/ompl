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

#include "ompl/extension/dynamic/SpaceInformationControls.h"
#include <algorithm>
#include <cassert>

void ompl::dynamic::SpaceInformationControls::setup(void)
{
    assert(m_resolution > 0.0);
    assert(m_controlDimension > 0);
    assert(m_controlComponent.size() == m_controlDimension);
    assert(m_stateDistanceEvaluator);
    assert(m_minControlDuration < m_maxControlDuration);
    assert(m_minControlDuration > 0);
    SpaceInformation::setup();
}

void ompl::dynamic::SpaceInformationControls::copyControl(base::Control *destination, const base::Control *source) const
{
    memcpy(destination->values, source->values, sizeof(double) * m_controlDimension);
}

void ompl::dynamic::SpaceInformationControls::printControl(const base::Control *control, std::ostream &out) const
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

void ompl::dynamic::SpaceInformationControls::nullControl(base::Control *ctrl) const
{
    for (unsigned int i = 0 ; i < m_controlDimension ; ++i)
	ctrl->values[i] = 0.0;
}

unsigned int ompl::dynamic::SpaceInformationControls::SamplingCore::sampleStepCount(void)
{
    return m_rng.uniformInt(m_minControlDuration, m_maxControlDuration);
}

void ompl::dynamic::SpaceInformationControls::SamplingCore::sample(base::Control *ctrl)
{
    const unsigned int dim = m_si->getControlDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const base::ControlComponent &comp = m_si->getControlComponent(i);
	ctrl->values[i] = m_rng.uniform(comp.minValue, comp.maxValue);
    }
}

void ompl::dynamic::SpaceInformationControls::SamplingCore::sampleNear(base::Control *ctrl, const base::Control *near, const double rho)
{
    const unsigned int dim = m_si->getControlDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const base::ControlComponent &comp = m_si->getControlComponent(i);
	ctrl->values[i] = m_rng.uniform(std::max(comp.minValue, near->values[i] - rho), 
					std::min(comp.maxValue, near->values[i] + rho));
    }
}

void ompl::dynamic::SpaceInformationControls::SamplingCore::sampleNear(base::Control *ctrl, const base::Control *near, const std::vector<double> &rho)
{
    const unsigned int dim = m_si->getControlDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const base::ControlComponent &comp = m_si->getControlComponent(i);
	ctrl->values[i] = m_rng.uniform(std::max(comp.minValue, near->values[i] - rho[i]), 
					std::min(comp.maxValue, near->values[i] + rho[i]));
    }
}

void ompl::dynamic::SpaceInformationControls::SamplingCore::sample(base::State *state)
{
    const unsigned int dim = m_si->getStateDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const base::StateComponent &comp = m_si->getStateComponent(i);	
	if (comp.type == base::StateComponent::QUATERNION)
	{
	    m_rng.quaternion(state->values + i);
	    i += 3;
	}
	else
	    state->values[i] = m_rng.uniform(comp.minValue, comp.maxValue);	    
    }
}

void ompl::dynamic::SpaceInformationControls::SamplingCore::sampleNear(base::State *state, const base::State *near, const double rho)
{
    const unsigned int dim = m_si->getStateDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const base::StateComponent &comp = m_si->getStateComponent(i);	
	if (comp.type == base::StateComponent::QUATERNION)
	{
	    /* no notion of 'near' is employed for quaternions */
	    m_rng.quaternion(state->values + i);
	    i += 3;
	}
	else
	    state->values[i] =
		m_rng.uniform(std::max(comp.minValue, near->values[i] - rho), 
			      std::min(comp.maxValue, near->values[i] + rho));
    }
}

void ompl::dynamic::SpaceInformationControls::SamplingCore::sampleNear(base::State *state, const base::State *near, const std::vector<double> &rho)
{
    const unsigned int dim = m_si->getStateDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {	
	const base::StateComponent &comp = m_si->getStateComponent(i);	
	if (comp.type == base::StateComponent::QUATERNION)
	{
	    /* no notion of 'near' is employed for quaternions */
	    m_rng.quaternion(state->values + i);
	    i += 3;
	}
	else
	    state->values[i] = 
		m_rng.uniform(std::max(comp.minValue, near->values[i] - rho[i]), 
			      std::min(comp.maxValue, near->values[i] + rho[i]));
    }
}
