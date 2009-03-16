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

#include "ompl/extension/samplingbased/kinodynamic/SpaceInformationControls.h"
#include <algorithm>
#include <cassert>

void ompl::sb::SpaceInformationControls::setup(void)
{
    assert(m_resolution > 0.0);
    assert(m_controlDimension > 0);
    assert(m_controlComponent.size() == m_controlDimension);
    assert(m_stateDistanceEvaluator);
    assert(m_minControlDuration < m_maxControlDuration);
    assert(m_minControlDuration > 0);
    SpaceInformation::setup();
}

void ompl::sb::SpaceInformationControls::copyControl(Control *destination, const Control *source) const
{
    memcpy(destination->values, source->values, sizeof(double) * m_controlDimension);
}

void ompl::sb::SpaceInformationControls::printControl(const Control *control, std::ostream &out) const
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

void ompl::sb::SpaceInformationControls::nullControl(Control *ctrl) const
{
    for (unsigned int i = 0 ; i < m_controlDimension ; ++i)
	ctrl->values[i] = 0.0;
}

bool ompl::sb::SpaceInformationControls::satisfiesBounds(const Control *control) const
{
    for (unsigned int i = 0 ; i < m_controlDimension ; ++i)
	if (control->values[i] > m_controlComponent[i].maxValue ||
	    control->values[i] < m_controlComponent[i].minValue)
	    return false;
    return true;
}

unsigned int ompl::sb::SpaceInformationControls::SamplingCore::sampleStepCount(void)
{
    return m_rng.uniformInt(m_minControlDuration, m_maxControlDuration);
}

void ompl::sb::SpaceInformationControls::SamplingCore::sample(Control *ctrl)
{
    const unsigned int dim = m_si->getControlDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const ControlComponent &comp = m_si->getControlComponent(i);
	ctrl->values[i] = m_rng.uniform(comp.minValue, comp.maxValue);
    }
}

void ompl::sb::SpaceInformationControls::SamplingCore::sampleNear(Control *ctrl, const Control *near, const double rho)
{
    const unsigned int dim = m_si->getControlDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const ControlComponent &comp = m_si->getControlComponent(i);
	ctrl->values[i] = m_rng.uniform(std::max(comp.minValue, near->values[i] - rho), 
					std::min(comp.maxValue, near->values[i] + rho));
    }
}

void ompl::sb::SpaceInformationControls::SamplingCore::sampleNear(Control *ctrl, const Control *near, const std::vector<double> &rho)
{
    const unsigned int dim = m_si->getControlDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	const ControlComponent &comp = m_si->getControlComponent(i);
	ctrl->values[i] = m_rng.uniform(std::max(comp.minValue, near->values[i] - rho[i]), 
					std::min(comp.maxValue, near->values[i] + rho[i]));
    }
}
