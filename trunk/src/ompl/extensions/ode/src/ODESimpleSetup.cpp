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

#include "ompl/extensions/ode/ODESimpleSetup.h"
#include "ompl/util/Exception.h"

ompl::control::ODESimpleSetup::ODESimpleSetup(const ControlManifoldPtr &manifold) : SimpleSetup(manifold)
{
    if (!dynamic_cast<ODEControlManifold*>(manifold.get()))
	throw Exception("ODE Control Manifold needed for ODE Simple Setup");
    useEnvParams();
}

ompl::control::ODESimpleSetup::ODESimpleSetup(const base::StateManifoldPtr &manifold) :
    SimpleSetup(ControlManifoldPtr(new ODEControlManifold(manifold)))
{
    useEnvParams();
}

ompl::control::ODESimpleSetup::ODESimpleSetup(const ODEEnvironmentPtr &env) : 
    SimpleSetup(ControlManifoldPtr(new ODEControlManifold(base::StateManifoldPtr(new ODEStateManifold(env)))))
{
    useEnvParams();
}

void ompl::control::ODESimpleSetup::useEnvParams(void)
{
    si_->setPropagationStepSize(getStateManifold()->as<ODEStateManifold>()->getEnvironment()->stepSize_);
    si_->setMinMaxControlDuration(getStateManifold()->as<ODEStateManifold>()->getEnvironment()->minControlSteps_,
                                  getStateManifold()->as<ODEStateManifold>()->getEnvironment()->maxControlSteps_);
}

ompl::base::ScopedState<ompl::control::ODEStateManifold> ompl::control::ODESimpleSetup::getCurrentState(void) const
{
    base::ScopedState<ODEStateManifold> current(getStateManifold());
    getStateManifold()->as<ODEStateManifold>()->readState(current.get());
    return current;
}

void ompl::control::ODESimpleSetup::setCurrentState(const base::ScopedState<> &state)
{
    getStateManifold()->as<ODEStateManifold>()->writeState(state.get());
}

void ompl::control::ODESimpleSetup::setup(void)
{
    if (!si_->getStateValidityChecker())
    {
	msg_.inform("Using default state validity checker for ODE");
	si_->setStateValidityChecker(base::StateValidityCheckerPtr(new ODEStateValidityChecker(si_)));
    }
    if (pdef_->getStartStateCount() == 0)
    {
	msg_.inform("Using the initial state of ODE as the starting state for the planner");
	pdef_->addStartState(getCurrentState());
    }
    SimpleSetup::setup();
}

void ompl::control::ODESimpleSetup::playSolutionPath(double timeFactor) const
{
    if (haveSolutionPath())
	playPath(getSolutionPath(), timeFactor);
}

void ompl::control::ODESimpleSetup::playPath(const PathControl &path, double timeFactor) const
{
    const geometric::PathGeometric &pg = path.asGeometric();
    if (!pg.states.empty())
    {
	msg_.debug("Playing through %u states (%0.3f seconds)", (unsigned int)pg.states.size(),
		   timeFactor * si_->getPropagationStepSize() * (double)(pg.states.size() - 1));
	time::duration d = time::seconds(timeFactor * si_->getPropagationStepSize());
	getStateManifold()->as<ODEStateManifold>()->writeState(pg.states[0]);
	for (unsigned int i = 1 ; i < pg.states.size() ; ++i)
	{
	    boost::this_thread::sleep(d);
	    getStateManifold()->as<ODEStateManifold>()->writeState(pg.states[i]);
	}
    }
}

void ompl::control::ODESimpleSetup::playControl(const double* control, double duration, double timeFactor) const
{
    Control *c = si_->allocControl();
    memcpy(c->as<ODEControlManifold::ControlType>()->values, control, sizeof(double) * getControlManifold()->getDimension());
    playControl(c, duration, timeFactor);
    si_->freeControl(c);
}

void ompl::control::ODESimpleSetup::playControl(const Control* control, double duration, double timeFactor) const
{
    unsigned int steps = floor(0.5 + duration / si_->getPropagationStepSize());
    PathControl p(si_);

    base::State *s0 = si_->allocState();
    getStateManifold()->as<ODEStateManifold>()->readState(s0);
    p.states.push_back(s0);
    
    base::State *s1 = si_->allocState();
    si_->propagate(s0, control, steps, s1);
    p.states.push_back(s1);
    
    p.controls.push_back(si_->cloneControl(control));
    p.controlDurations.push_back(steps);
    playPath(p, timeFactor);
}

void ompl::control::ODESimpleSetup::play(double duration, double timeFactor) const
{
    Control *c = si_->allocControl();
    si_->nullControl(c);
    playControl(c, duration, timeFactor);
}
