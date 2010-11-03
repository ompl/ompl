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

#include "ompl/control/ControlManifold.h"
#include "ompl/util/Exception.h"

void ompl::control::ControlManifold::setup(void)
{
}

bool ompl::control::ControlManifold::canPropagateBackward(void) const
{
    return true;
}

void ompl::control::ControlManifold::printControl(const Control *control, std::ostream &out) const
{
    out << "Control instance: " << control << std::endl;
}

void ompl::control::ControlManifold::printSettings(std::ostream &out) const
{
    out << "ControlManifold '" << name_ << "' instance: " << this << std::endl;
}

void ompl::control::ControlManifold::propagate(const base::State *state, const Control* control, const double duration, const unsigned int step, base::State *result) const
{
    if (statePropagation_)
	statePropagation_(state, control, duration, step, result);
    else
	throw Exception("State propagation routine is not set for control manifold. Either set this routine or provide a different implementation in an inherited class.");
}

void ompl::control::ControlManifold::setPropagationFunction(const StatePropagationFn &fn)
{
    statePropagation_ = fn;
}

void ompl::control::CompoundControlManifold::addSubManifold(const ControlManifoldPtr &component)
{
    if (locked_)
	throw Exception("This manifold is locked. No further components can be added");
    
    components_.push_back(component);
    componentCount_ = components_.size();
}

unsigned int ompl::control::CompoundControlManifold::getSubManifoldCount(void) const
{
    return componentCount_;
}

const ompl::control::ControlManifoldPtr& ompl::control::CompoundControlManifold::getSubManifold(const unsigned int index) const
{
    if (componentCount_ > index)
	return components_[index];
    else
	throw Exception("Submanifold index does not exist");
}

const ompl::control::ControlManifoldPtr& ompl::control::CompoundControlManifold::getSubManifold(const std::string &name) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	if (components_[i]->getName() == name)
	    return components_[i];
    throw Exception("Submanifold " + name + " does not exist");
}

unsigned int ompl::control::CompoundControlManifold::getDimension(void) const
{
    unsigned int dim = 0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	dim += components_[i]->getDimension();
    return dim;
}

ompl::control::Control* ompl::control::CompoundControlManifold::allocControl(void) const
{
    CompoundControl *control = new CompoundControl();
    control->components = new Control*[componentCount_];
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	control->components[i] = components_[i]->allocControl();
    return static_cast<Control*>(control);
}

void ompl::control::CompoundControlManifold::freeControl(Control *control) const
{  
    CompoundControl *ccontrol = static_cast<CompoundControl*>(control);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->freeControl(ccontrol->components[i]);
    delete[] ccontrol->components;
    delete ccontrol;
}

void ompl::control::CompoundControlManifold::copyControl(Control *destination, const Control *source) const
{  
    CompoundControl      *cdest = static_cast<CompoundControl*>(destination);
    const CompoundControl *csrc = static_cast<const CompoundControl*>(source);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->copyControl(cdest->components[i], csrc->components[i]);
}

bool ompl::control::CompoundControlManifold::equalControls(const Control *control1, const Control *control2) const
{ 
    const CompoundControl *ccontrol1 = static_cast<const CompoundControl*>(control1);
    const CompoundControl *ccontrol2 = static_cast<const CompoundControl*>(control2);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	if (!components_[i]->equalControls(ccontrol1->components[i], ccontrol2->components[i]))
	    return false;
    return true;
}

void ompl::control::CompoundControlManifold::nullControl(Control *control) const
{   
    CompoundControl *ccontrol = static_cast<CompoundControl*>(control);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->nullControl(ccontrol->components[i]);
}

ompl::control::ControlSamplerPtr ompl::control::CompoundControlManifold::allocControlSampler(void) const
{
    CompoundControlSampler *ss = new CompoundControlSampler(this);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	ss->addSampler(components_[i]->allocControlSampler());
    return ControlSamplerPtr(ss);
}

void ompl::control::CompoundControlManifold::propagate(const base::State *state, const Control* control, const double duration, const unsigned int step, base::State *result) const
{
    if (statePropagation_)
	statePropagation_(state, control, duration, step, result);
    else
    {
	const base::CompoundState *cstate = static_cast<const base::CompoundState*>(state);
	const CompoundControl *ccontrol = static_cast<const CompoundControl*>(control);
	base::CompoundState *cresult = static_cast<base::CompoundState*>(result);
	for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	    components_[i]->propagate(cstate->components[i], ccontrol->components[i], duration, step, cresult->components[i]);
    }
}

void ompl::control::CompoundControlManifold::lock(void)
{
    locked_ = true;
}

bool ompl::control::CompoundControlManifold::canPropagateBackward(void) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	if (!components_[i]->canPropagateBackward())
	    return false;
    return true;
}

void ompl::control::CompoundControlManifold::printControl(const Control *control, std::ostream &out) const
{
    out << "Compound control [" << std::endl;
    const CompoundControl *ccontrol = static_cast<const CompoundControl*>(control);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->printControl(ccontrol->components[i], out);
    out << "]" << std::endl;
}

void ompl::control::CompoundControlManifold::printSettings(std::ostream &out) const
{
    out << "Compound control manifold '" << name_ << "' [" << std::endl;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->printSettings(out);
    out << "]" << std::endl;
}

void ompl::control::CompoundControlManifold::setup(void)
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->setup();
    ControlManifold::setup();
}
