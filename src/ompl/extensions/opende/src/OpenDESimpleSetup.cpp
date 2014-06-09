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

#include "ompl/extensions/opende/OpenDESimpleSetup.h"
#include "ompl/util/Exception.h"
#include <boost/thread.hpp>

ompl::control::OpenDESimpleSetup::OpenDESimpleSetup(const ControlSpacePtr &space) : SimpleSetup(space)
{
    if (!dynamic_cast<OpenDEControlSpace*>(space.get()))
        throw Exception("OpenDE Control Space needed for OpenDE Simple Setup");
    useEnvParams();
}

ompl::control::OpenDESimpleSetup::OpenDESimpleSetup(const base::StateSpacePtr &space) :
    SimpleSetup(ControlSpacePtr(new OpenDEControlSpace(space)))
{
    useEnvParams();
}

ompl::control::OpenDESimpleSetup::OpenDESimpleSetup(const OpenDEEnvironmentPtr &env) :
    SimpleSetup(ControlSpacePtr(new OpenDEControlSpace(base::StateSpacePtr(new OpenDEStateSpace(env)))))
{
    useEnvParams();
}

void ompl::control::OpenDESimpleSetup::useEnvParams()
{
    si_->setPropagationStepSize(getStateSpace()->as<OpenDEStateSpace>()->getEnvironment()->stepSize_);
    si_->setMinMaxControlDuration(getStateSpace()->as<OpenDEStateSpace>()->getEnvironment()->minControlSteps_,
                                  getStateSpace()->as<OpenDEStateSpace>()->getEnvironment()->maxControlSteps_);
    si_->setStatePropagator(StatePropagatorPtr(new OpenDEStatePropagator(si_)));
}

ompl::base::ScopedState<ompl::control::OpenDEStateSpace> ompl::control::OpenDESimpleSetup::getCurrentState() const
{
    base::ScopedState<OpenDEStateSpace> current(getStateSpace());
    getStateSpace()->as<OpenDEStateSpace>()->readState(current.get());
    return current;
}

void ompl::control::OpenDESimpleSetup::setCurrentState(const base::State *state)
{
    getStateSpace()->as<OpenDEStateSpace>()->writeState(state);
}

void ompl::control::OpenDESimpleSetup::setCurrentState(const base::ScopedState<> &state)
{
    getStateSpace()->as<OpenDEStateSpace>()->writeState(state.get());
}

void ompl::control::OpenDESimpleSetup::setup()
{
    if (!si_->getStateValidityChecker())
    {
        OMPL_INFORM("Using default state validity checker for OpenDE");
        si_->setStateValidityChecker(base::StateValidityCheckerPtr(new OpenDEStateValidityChecker(si_)));
    }
    if (pdef_->getStartStateCount() == 0)
    {
        OMPL_INFORM("Using the initial state of OpenDE as the starting state for the planner");
        pdef_->addStartState(getCurrentState());
    }
    SimpleSetup::setup();
}

void ompl::control::OpenDESimpleSetup::playSolutionPath(double timeFactor) const
{
    if (haveSolutionPath())
        playPath(pdef_->getSolutionPath(), timeFactor);
}

void ompl::control::OpenDESimpleSetup::playPath(const base::PathPtr &path, double timeFactor) const
{
    bool ctl = false;
    if (dynamic_cast<PathControl*>(path.get()))
        ctl = true;
    else
        if (!dynamic_cast<geometric::PathGeometric*>(path.get()))
            throw Exception("Unknown type of path");

    const geometric::PathGeometric &pg = ctl ?
        static_cast<PathControl*>(path.get())->asGeometric() : *static_cast<geometric::PathGeometric*>(path.get());

    if (pg.getStateCount() > 0)
    {
        OMPL_DEBUG("Playing through %u states (%0.3f seconds)", (unsigned int)pg.getStateCount(),
                   timeFactor * si_->getPropagationStepSize() * (double)(pg.getStateCount() - 1));
        time::duration d = time::seconds(timeFactor * si_->getPropagationStepSize());
        getStateSpace()->as<OpenDEStateSpace>()->writeState(pg.getState(0));
        for (unsigned int i = 1 ; i < pg.getStateCount() ; ++i)
        {
            boost::this_thread::sleep(d);
            getStateSpace()->as<OpenDEStateSpace>()->writeState(pg.getState(i));
        }
    }
}

ompl::base::PathPtr ompl::control::OpenDESimpleSetup::simulateControl(const double *control, unsigned int steps) const
{
    Control *c = si_->allocControl();
    memcpy(c->as<OpenDEControlSpace::ControlType>()->values, control, sizeof(double) * getControlSpace()->getDimension());
    base::PathPtr path = simulateControl(c, steps);
    si_->freeControl(c);
    return path;
}

ompl::base::PathPtr ompl::control::OpenDESimpleSetup::simulateControl(const Control *control, unsigned int steps) const
{
    PathControl *p = new PathControl(si_);

    base::State *s0 = si_->allocState();
    getStateSpace()->as<OpenDEStateSpace>()->readState(s0);
    p->getStates().push_back(s0);

    base::State *s1 = si_->allocState();
    si_->propagate(s0, control, steps, s1);
    p->getStates().push_back(s1);

    p->getControls().push_back(si_->cloneControl(control));
    p->getControlDurations().push_back(steps);
    return base::PathPtr(p);
}

ompl::base::PathPtr ompl::control::OpenDESimpleSetup::simulate(unsigned int steps) const
{
    Control *c = si_->allocControl();
    si_->nullControl(c);
    base::PathPtr path = simulateControl(c, steps);
    si_->freeControl(c);
    return path;
}
