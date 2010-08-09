/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan */

#include "ompl/control/PathControl.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/util/Exception.h"
#include <numeric>
#include <cmath>

ompl::control::PathControl::PathControl(const base::SpaceInformationPtr &si) : base::Path(si)
{
    if (!dynamic_cast<const SpaceInformation*>(si_.get()))
	throw Exception("Cannot create a path with controls from a space that does not support controls");
}

ompl::control::PathControl::PathControl(const PathControl &path) : base::Path(path.si_)
{
    copyFrom(path);
}

ompl::base::PathPtr ompl::control::PathControl::asGeometric(void) const
{
    PathControl pc(*this);
    pc.interpolate();
    geometric::PathGeometric *pg = new geometric::PathGeometric(si_);
    pg->states.swap(pc.states);
    return base::PathPtr(pg);
}

ompl::control::PathControl& ompl::control::PathControl::operator=(const PathControl& other)
{
    freeMemory();
    si_ = other.si_;
    copyFrom(other);
    return *this;
}

void ompl::control::PathControl::copyFrom(const PathControl& other) 
{
    states.resize(other.states.size()); 
    controls.resize(other.controls.size());

    for (unsigned int i = 0 ; i < states.size() ; ++i)
	states[i] = si_->cloneState(other.states[i]);
    
    const SpaceInformation *si = static_cast<const SpaceInformation*>(si_.get());
    for (unsigned int i = 0 ; i < controls.size() ; ++i)
	controls[i] = si->cloneControl(other.controls[i]);
    
    controlDurations = other.controlDurations; 
}

double ompl::control::PathControl::length(void) const
{
    return std::accumulate(controlDurations.begin(), controlDurations.end(), 0.0);
}

void ompl::control::PathControl::print(std::ostream &out) const
{
    const SpaceInformation *si = static_cast<const SpaceInformation*>(si_.get());
    double res = si->getPropagationStepSize();
    out << "Control path with " << states.size() << " states" << std::endl;
    for (unsigned int i = 0 ; i < controls.size() ; ++i)
    {
	out << "At state ";
	si_->printState(states[i], out);
	out << "  apply control ";
	si->printControl(controls[i], out);
	out << "  for " << (int)floor(0.5 + controlDurations[i]/res) << " steps" << std::endl;
    }
    out << "Arrive at state ";
    si_->printState(states[controls.size()], out);
    out << std::endl;
}

void ompl::control::PathControl::interpolate(void) 
{
    const SpaceInformation *si = static_cast<const SpaceInformation*>(si_.get());
    std::vector<base::State*> newStates;
    std::vector<Control*> newControls;
    std::vector<double> newControlDurations;
    
    double res = si->getPropagationStepSize();
    for (unsigned int  i = 0 ; i < controls.size() ; ++i)
    {
	int steps = (int)floor(0.5 + controlDurations[i] / res);
	assert(steps >= 0);
	if (steps <= 1)
	{
	    newStates.push_back(states[i]);
	    newControls.push_back(controls[i]);
	    newControlDurations.push_back(controlDurations[i]);
	    continue;
	}
	std::vector<base::State*> istates;
	si->propagate(states[i], controls[i], steps, istates, true);
	// last state is already in the non-interpolated path
	if (!istates.empty())
	{
	    si_->freeState(istates.back());
	    istates.pop_back();
	}
	newStates.push_back(states[i]);
	newStates.insert(newStates.end(), istates.begin(), istates.end());
	newControls.push_back(controls[i]);
	newControlDurations.push_back(res);
	for (int j = 1 ; j < steps; ++j)
	{
	    newControls.push_back(si->cloneControl(controls[i]));
	    newControlDurations.push_back(res);
	}
    }
    newStates.push_back(states[controls.size()]);
    states.swap(newStates);
    controls.swap(newControls);
    controlDurations.swap(newControlDurations);
}

bool ompl::control::PathControl::check(void) const
{
    bool valid = true;
    const SpaceInformation *si = static_cast<const SpaceInformation*>(si_.get());
    double res = si->getPropagationStepSize();
    base::State *dummy = si_->allocState();
    for (unsigned int  i = 0 ; i < controls.size() ; ++i)
    {
	unsigned int steps = (unsigned int)floor(0.5 + controlDurations[i] / res);
	if (si->propagateWhileValid(states[i], controls[i], steps, dummy) != steps)
	{
	    valid = false;
	    break;
	}
    }
    si_->freeState(dummy);
    return valid;
}

void ompl::control::PathControl::freeMemory(void)
{
    for (unsigned int i = 0 ; i < states.size() ; ++i)
	si_->freeState(states[i]);
    const SpaceInformation *si = static_cast<const SpaceInformation*>(si_.get());
    for (unsigned int i = 0 ; i < controls.size() ; ++i)
	si->freeControl(controls[i]);
}
